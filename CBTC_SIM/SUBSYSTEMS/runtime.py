from __future__ import annotations

import random
import time
from copy import deepcopy
from datetime import datetime, timezone, timedelta
from typing import Any, Dict, List, Tuple

from CONFIG.config import DT
from OPERATION.headway_manager import HeadwayManager
from SUBSYSTEMS.atp_ato import *
from SUBSYSTEMS.core_engine import SafeMovementPacket, STOP_SVL_OFFSET_M
from SUBSYSTEMS.physics import kmh_to_ms, ms_to_kmh
from SUBSYSTEMS.train import Train
from SUBSYSTEMS.zc import ZoneController


TSR_COLOR = "#c94a36"
SOURCE_TRAIN_SPACING_M = 120.0
SOURCE_TRAIN_LENGTH_M = 200.0
SOURCE_TRAIN_START_M = -SOURCE_TRAIN_LENGTH_M
SOURCE_TRAIN_EXIT_M = 0.0
SOURCE_TRAIN_STAGING_CLEARANCE_M = 35.0
SOURCE_VISIBLE_ACTIVE_TRAINS = 2
MIN_PASSENGER_DWELL_S = 25.0
MIN_TIMETABLE_RECOVERY_DWELL_S = 20.0
PARALLEL_ROMAN_LABELS = ["I", "II", "III", "IV", "V", "VI", "VII", "VIII", "IX", "X"]
PARALLEL_RELEASE_MARGIN_M = 5.0
DEPARTURE_RELEASE_MIN_AUTHORITY_M = 30.0
STATION_ROUTE_APPROACH_M = 800.0
TURNOUT_LOCK_S = 5.0
SOURCE_RELEASE_LOCK_S = 5.0
LINE_CENTER_SPACING_M = 4.0


class Simulation:
    def __init__(self, scenario: Dict[str, object]):
        self.load_scenario(scenario)
        self.running = False
        self.last_step = time.time()
        self.tsr_zones = []

    def load_scenario(self, scenario: Dict[str, object]):
        self.scenario = scenario
        self.track_profile = list(scenario["track_profile"])
        self.track_end_m = float(scenario["track_end_m"])
        self.track_min_m = float(scenario["track_min_m"])
        self.track_max_m = float(scenario["track_max_m"])
        self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self.track_labels = list(scenario["track_labels"])
        raw_headway_cfg = scenario.get("headway")
        headway_cfg = raw_headway_cfg if isinstance(raw_headway_cfg, dict) else {}
        requested_block_mode = str(scenario.get("block_mode", "")).lower()
        if requested_block_mode in {"fixed", "fixed_block"}:
            self.block_mode = "fixed_block"
        elif requested_block_mode in {"moving", "moving_block"}:
            self.block_mode = "moving_block"
        else:
            explicit_off_mode = (
                bool(scenario.get("headway_config_present", isinstance(raw_headway_cfg, dict) and bool(raw_headway_cfg)))
                and str(headway_cfg.get("mode", "off")).lower() == "off"
            )
            self.block_mode = "fixed_block" if explicit_off_mode else "moving_block"
        self.headway_manager = HeadwayManager.from_scenario(scenario)
        self.scheduled_stops = [dict(stop) for stop in scenario.get("scheduled_stops", [])]
        self.timetable_services = self._build_timetable_services(scenario)
        headway_runtime = scenario.get("headway", {}) if isinstance(scenario.get("headway", {}), dict) else {}
        self.timetable_wall_clock = bool(headway_runtime.get("timetable_wall_clock", False))
        self.timetable_loaded_clock_s = (
            float(headway_runtime.get("timetable_loaded_clock_s"))
            if headway_runtime.get("timetable_loaded_clock_s") is not None
            else None
        )
        self.timetable_clock_scale = max(1.0, float(headway_runtime.get("timetable_clock_scale", 1.0) or 1.0))
        self._timetable_clock_anchor_real_s = time.monotonic()
        self._timetable_clock_anchor_operational_s = 0.0
        self.fixed_blocks = self._build_fixed_blocks(scenario)
        self.station_route_states: List[Dict[str, Any]] = []
        self.parallel_release_locks: Dict[Tuple[str, int], float] = {}
        self.source_trains = []
        for idx, source in enumerate(scenario.get("source_trains", [])):
            capacity = max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS)))
            total_trains = max(0, int(source.get("total_trains", capacity)))
            self.source_trains.append(
                {
                    "name": str(source.get("name", f"DEPOT_{idx + 1}")),
                    "start_m": SOURCE_TRAIN_START_M,
                    "length_m": SOURCE_TRAIN_LENGTH_M,
                    "capacity": capacity,
                    "total_trains": total_trains,
                    "generated": min(total_trains, max(0, int(source.get("generated", 0)))),
                }
            )
            self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self.line_conditions = []
        for condition in scenario.get("line_conditions", []):
            self.line_conditions.append(
                {
                    "start": float(condition.get("start", condition.get("start_m", self.track_min_m))),
                    "end": float(condition.get("end", condition.get("end_m", self.track_max_m))),
                    "condition": str(condition.get("condition", "dry")),
                }
            )
        palette = list(scenario["color_palette"])
        self.color_palette = palette
        self.generated_train_counter = 0
        self.train_generation_changed = False
        self.station_last_arrival_s: Dict[int, float] = {}
        self.station_arrival_headway_actual_s: Dict[int, List[float]] = {}
        self.station_last_departure_s: Dict[int, float] = {}
        self.station_next_departure_release_s: Dict[int, float] = {}
        self.station_headway_actual_s: Dict[int, List[float]] = {}
        self.station_headway_deviation_s: Dict[int, List[float]] = {}
        self.trains = []
        for idx, train_cfg in enumerate(scenario["trains"]):
            cfg = dict(train_cfg)
            cfg["color"] = train_color(cfg, idx, palette)
            cfg["track_profile"] = self.track_profile
            cfg["scheduled_stops"] = self.scheduled_stops
            train = Train(cfg)
            train.source_lane = cfg.get("source_lane")
            self.trains.append(train)
        self._stage_initial_source_trains()
        self._sync_station_route_states()
        self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
        self.tsr_zones = []
        self.sim_time_s = 0.0
        self._reset_timetable_clock_anchor()
        self.analytics = {
            "min_headway_s": None,
            "target_headway_s": self.headway_manager.nominal_target_headway_s(),
            "actual_headways_s": [],
            "actual_headway_pairs": [],
            "current_open_headway_s": None,
            "min_actual_headway_s": None,
            "avg_actual_headway_s": None,
            "max_actual_headway_s": None,
            "dispatch_delays_s": [],
            "avg_dispatch_delay_s": None,
            "trains_per_hour": 0.0,
            "station_passenger_metrics": [],
            "station_arrivals": {},
            "traction_work_kwh": 0.0,
            "brake_work_kwh": 0.0,
            "collision_count": 0,
            "active_collision_count": 0,
            "collision_events": [],
            "ebi_count": 0,
            "sbi_count": 0,
            "journey_times": {},
            "last_actions": {},
        }
        self._dispatch_safe_packets(with_delay=False)

    def _build_fixed_blocks(self, scenario: Dict[str, object]) -> List[Dict[str, float]]:
        cfg = scenario.get("capacity_baseline", {}) if isinstance(scenario.get("capacity_baseline", {}), dict) else {}
        blocks_per_section = max(1, int(cfg.get("blocks_per_section", cfg.get("fixed_blocks_per_section", 4))))
        track_start_m = float(self.track_profile[0][0]) if self.track_profile else 0.0
        sections: List[Tuple[float, float]] = []
        section_start = track_start_m
        for stop in self.scheduled_stops:
            stop_pos = float(stop.get("pos_m", track_start_m))
            stop_len = max(0.0, float(stop.get("length_m", 160.0)))
            station_start = max(track_start_m, stop_pos - stop_len / 2.0)
            station_end = min(self.track_end_m, stop_pos + stop_len / 2.0)
            if station_start > section_start + STOP_ACCURACY_TOL_M:
                sections.append((section_start, station_start))
            section_start = max(section_start, station_end)
        if self.track_end_m > section_start + STOP_ACCURACY_TOL_M:
            sections.append((section_start, self.track_end_m))
        if not sections:
            sections.append((track_start_m, self.track_end_m))
        blocks: List[Dict[str, float]] = []
        block_idx = 1
        for section_idx, (start, end) in enumerate(sections, start=1):
            length = max(0.0, end - start)
            if length <= 0.0:
                continue
            block_len = length / blocks_per_section
            for local_idx in range(blocks_per_section):
                block_start = start + block_len * local_idx
                block_end = end if local_idx == blocks_per_section - 1 else start + block_len * (local_idx + 1)
                blocks.append(
                    {
                        "id": f"FB{section_idx}.{local_idx + 1}",
                        "start_m": block_start,
                        "end_m": block_end,
                        "index": block_idx,
                    }
                )
                block_idx += 1
        return blocks

    def _sync_station_route_states(self):
        while len(self.station_route_states) < len(self.scheduled_stops):
            self.station_route_states.append(
                {
                    "route_lane": None,
                    "assigned_train_id": None,
                    "route_state": "FREE",
                    "lines": [],
                    "invariant_block": False,
                    "lock_remaining_s": 0.0,
                    "locking_train_id": None,
                    "switch_started": False,
                }
            )
        if len(self.station_route_states) > len(self.scheduled_stops):
            self.station_route_states = self.station_route_states[:len(self.scheduled_stops)]
        for station_idx, stop in enumerate(self.scheduled_stops):
            self.detect_station_lines(station_idx)

    def _make_source_train_config(self, source: Dict[str, Any], train_id: str, start_pos: float, lane: int = 0) -> Dict[str, Any]:
        sequence = int(source.get("_pending_sequence", 0) or 0)
        service = self._timetable_service_for_sequence(sequence)
        if service:
            train_id = str(service.get("train_id", train_id))
        cfg = {
            "id": train_id,
            "start_pos": start_pos,
            "length_m": float(self.scenario["train_defaults"]["length_m"]),
            "mass_kg": float(self.scenario["train_defaults"]["mass_kg"]),
            "drive_mode": str(self.scenario["train_defaults"].get("drive_mode", "ATO")),
            "requested_drive_mode": str(self.scenario["train_defaults"].get("drive_mode", "ATO")),
            "max_manual_speed_kmh": float(self.scenario["train_defaults"].get("max_manual_speed_kmh", 45.0)),
            "dcs_mute_windows": [],
            "color": self.color_palette[len(self.trains) % len(self.color_palette)],
            "track_profile": self.track_profile,
            "scheduled_stops": self.scheduled_stops,
            "source_name": str(source.get("name", "DEPOT")),
            "source_lane": lane,
        }
        if service:
            cfg["schedule_service_id"] = service.get("train_id")
            cfg["schedule_profile"] = service.get("profile", "")
            cfg["schedule_records"] = service.get("records", [])
            cfg["schedule_planned_dispatch_s"] = service.get("planned_dispatch_time_s")
        return cfg

    def _build_timetable_services(self, scenario: Dict[str, Any]) -> List[Dict[str, Any]]:
        headway = scenario.get("headway", {}) if isinstance(scenario.get("headway", {}), dict) else {}
        records = [dict(record) for record in headway.get("timetable_records", []) or []]
        services: Dict[str, Dict[str, Any]] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            if not train_id:
                continue
            service = services.setdefault(train_id, {"train_id": train_id, "records": []})
            service["records"].append(record)
            if service.get("planned_dispatch_time_s") is None and record.get("departure_time_s") is not None:
                service["planned_dispatch_time_s"] = float(record.get("departure_time_s"))
            if not service.get("profile") and record.get("profile") not in {None, "", "--"}:
                service["profile"] = str(record.get("profile"))
        result = list(services.values())
        result.sort(
            key=lambda service: min(
                (
                    float(record.get("departure_time_s"))
                    for record in service.get("records", [])
                    if record.get("departure_time_s") is not None
                ),
                default=float("inf"),
            )
        )
        return result

    def _timetable_service_for_sequence(self, sequence: int) -> Dict[str, Any] | None:
        if self.headway_manager.mode != "timetable" or sequence <= 0:
            return None
        idx = sequence - 1
        if 0 <= idx < len(self.timetable_services):
            return self.timetable_services[idx]
        return None

    def _vietnam_clock_seconds(self) -> float:
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        return float(now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 1_000_000.0)

    def _clock_delay_from_reference_s(self, clock_s: float, reference_clock_s: float) -> float:
        day_s = 24.0 * 3600.0
        return (float(clock_s) - float(reference_clock_s)) % day_s

    def _reset_timetable_clock_anchor(self) -> None:
        self._timetable_clock_anchor_real_s = time.monotonic()
        if self.timetable_wall_clock and self.timetable_loaded_clock_s is not None:
            self._timetable_clock_anchor_operational_s = self._clock_delay_from_reference_s(
                self._vietnam_clock_seconds(),
                self.timetable_loaded_clock_s,
            )
        else:
            self._timetable_clock_anchor_operational_s = self.sim_time_s

    def set_timetable_clock_scale(self, scale: float) -> None:
        current_operational_s = self._timetable_operational_time_s()
        self.timetable_clock_scale = max(1.0, float(scale))
        self._timetable_clock_anchor_operational_s = current_operational_s
        self._timetable_clock_anchor_real_s = time.monotonic()

    def _timetable_operational_time_s(self) -> float:
        if (
            self.headway_manager.mode == "timetable"
            and self.timetable_wall_clock
            and self.timetable_loaded_clock_s is not None
        ):
            elapsed_real_s = max(0.0, time.monotonic() - getattr(self, "_timetable_clock_anchor_real_s", time.monotonic()))
            return float(getattr(self, "_timetable_clock_anchor_operational_s", 0.0)) + elapsed_real_s * float(
                getattr(self, "timetable_clock_scale", 1.0)
            )
        return self.sim_time_s

    def timetable_display_clock_s(self) -> float | None:
        if not (self.headway_manager.mode == "timetable" and self.timetable_wall_clock and self.timetable_loaded_clock_s is not None):
            return None
        return (self.timetable_loaded_clock_s + self._timetable_operational_time_s()) % (24.0 * 3600.0)

    def _source_staging_head_pos(self) -> float:
        train_length = float(self.scenario["train_defaults"]["length_m"])
        min_head = SOURCE_TRAIN_START_M + train_length + SOURCE_TRAIN_STAGING_CLEARANCE_M
        max_head = SOURCE_TRAIN_EXIT_M - SOURCE_TRAIN_STAGING_CLEARANCE_M
        return min(max_head, max(min_head, SOURCE_TRAIN_START_M + train_length))

    def _next_source_train_id(self, source: Dict[str, Any], sequence: int) -> str:
        train_id = f"{source.get('name', 'DEPOT')}_{sequence}"
        existing_ids = {train.id for train in self.trains}
        while train_id in existing_ids:
            self.generated_train_counter += 1
            train_id = f"DEPOT_{self.generated_train_counter}"
        return train_id

    def _source_train_matches(self, train: Train, source_name: str) -> bool:
        if getattr(train, "source_name", None) == source_name:
            return True
        return train.id.startswith(f"{source_name}_")

    def _source_train_sequence(self, train: Train, source_name: str) -> int:
        prefix = f"{source_name}_"
        if not train.id.startswith(prefix):
            return 0
        try:
            return int(train.id[len(prefix):])
        except ValueError:
            return 0

    def _source_owned_trains(self, source_name: str) -> List[Train]:
        owned = [train for train in self.trains if self._source_train_matches(train, source_name)]
        return sorted(owned, key=lambda item: self._source_train_sequence(item, source_name))

    def _rebuild_after_train_set_change(self):
        self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
        self._dispatch_safe_packets(with_delay=False)
        self.train_generation_changed = True

    def _sync_source_train_count(self, index: int, previous_name: str | None = None):
        if index < 0 or index >= len(self.source_trains):
            return
        source = self.source_trains[index]
        source_name = str(source.get("name", "DEPOT"))
        match_name = previous_name or source_name
        total = max(0, int(source.get("total_trains", source.get("capacity", 0))))
        owned = self._source_owned_trains(match_name)
        changed = False

        for train in owned:
            train.source_name = source_name

        if len(owned) > total:
            remove_set = set(owned[total:])
            self.trains = [train for train in self.trains if train not in remove_set]
            changed = True
        elif len(owned) < total:
            before = len(self.trains)
            source["generated"] = len(owned)
            self._stage_initial_source_trains()
            changed = len(self.trains) != before

        source["generated"] = min(total, len(self._source_owned_trains(source_name)))
        if changed:
            self._rebuild_after_train_set_change()

    def _stage_initial_source_trains(self):
        for source in self.source_trains:
            total = int(source.get("total_trains", 0))
            generated = int(source.get("generated", 0))
            capacity = max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS)))
            occupied_lanes = {
                int(train.source_lane)
                for train in self.trains
                if train.source_lane is not None and self._source_zone_contains(train)
            }
            free_lanes = [lane for lane in range(capacity) if lane not in occupied_lanes]
            to_stage = min(len(free_lanes), max(0, total - generated))
            for idx, lane in enumerate(free_lanes[:to_stage]):
                sequence = generated + idx + 1
                start_pos = self._source_staging_head_pos()
                train_id = self._next_source_train_id(source, sequence)
                source["_pending_sequence"] = sequence
                train = Train(self._make_source_train_config(source, train_id, start_pos, lane))
                source.pop("_pending_sequence", None)
                train.source_lane = lane
                self.trains.append(train)
            source["generated"] = generated + to_stage

    def add_station(self, name: str, pos_m: float, length_m: float, capacity: int = 3, dwell_s: float = 30.0):
        self.scheduled_stops.append(
            {
                "name": name,
                "pos_m": pos_m,
                "dwell_s": dwell_s,
                "length_m": max(1.0, length_m),
                "capacity": max(1, capacity),
            }
        )
        self.scheduled_stops.sort(key=lambda item: item["pos_m"])
        self._sync_station_route_states()
        for train in self.trains:
            train.scheduled_stops = self.scheduled_stops

    def update_station(self, index: int, name: str, pos_m: float, length_m: float, capacity: int, dwell_s: float):
        if index < 0 or index >= len(self.scheduled_stops):
            return
        self.scheduled_stops[index].update(
            {
                "name": name,
                "pos_m": pos_m,
                "dwell_s": dwell_s,
                "length_m": max(1.0, length_m),
                "capacity": max(1, capacity),
            }
        )
        self.scheduled_stops.sort(key=lambda item: item["pos_m"])
        self._sync_station_route_states()
        for train in self.trains:
            train.scheduled_stops = self.scheduled_stops

    def add_psr_segment(self, start_m: float, end_m: float, psr_kmh: float):
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        updated: List[Tuple[float, float, float, float]] = []
        covered = False
        for seg_start, seg_end, gradient, old_psr in self.track_profile:
            if seg_end <= start_m or seg_start >= end_m:
                updated.append((seg_start, seg_end, gradient, old_psr))
                continue
            covered = True
            if seg_start < start_m:
                updated.append((seg_start, start_m, gradient, old_psr))
            updated.append((max(seg_start, start_m), min(seg_end, end_m), gradient, psr_kmh))
            if seg_end > end_m:
                updated.append((end_m, seg_end, gradient, old_psr))
        if not covered:
            gradient, _old_psr = get_track_info(self.track_profile, start_m)
            updated.append((start_m, end_m, gradient, psr_kmh))
        self.track_profile = sorted(
            [segment for segment in updated if segment[1] > segment[0]],
            key=lambda item: item[0],
        )
        self.track_end_m = max(self.track_end_m, end_m)
        self.track_max_m = max(self.track_max_m, end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def update_track_segment(self, index: int, start_m: float, end_m: float, gradient: float, psr_kmh: float):
        if index < 0 or index >= len(self.track_profile):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.track_profile[index] = (start_m, end_m, gradient, psr_kmh)
        self.track_profile.sort(key=lambda item: item[0])
        self.track_end_m = max(end for _start, end, _gradient, _psr in self.track_profile)
        self.track_max_m = max(self.track_max_m, self.track_end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def add_gradient_segment(self, start_m: float, end_m: float, gradient: float):
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        updated: List[Tuple[float, float, float, float]] = []
        covered = False
        for seg_start, seg_end, old_gradient, old_psr in self.track_profile:
            if seg_end <= start_m or seg_start >= end_m:
                updated.append((seg_start, seg_end, old_gradient, old_psr))
                continue
            covered = True
            if seg_start < start_m:
                updated.append((seg_start, start_m, old_gradient, old_psr))
            updated.append((max(seg_start, start_m), min(seg_end, end_m), gradient, old_psr))
            if seg_end > end_m:
                updated.append((end_m, seg_end, old_gradient, old_psr))
        if not covered:
            _old_gradient, psr = get_track_info(self.track_profile, start_m)
            updated.append((start_m, end_m, gradient, psr))
        self.track_profile = sorted(
            [segment for segment in updated if segment[1] > segment[0]],
            key=lambda item: item[0],
        )
        self.track_end_m = max(self.track_end_m, end_m)
        self.track_max_m = max(self.track_max_m, end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def add_line_element(
        self,
        length_m: float,
        gradient_start_m: float,
        gradient_end_m: float,
        gradient: float,
        condition_start_m: float,
        condition_end_m: float,
        condition: str,
        psr_kmh: float,
    ):
        length_m = max(1.0, length_m)
        self.track_end_m = max(self.track_end_m, length_m)
        self.track_max_m = max(self.track_max_m, length_m)
        if gradient_end_m < gradient_start_m:
            gradient_start_m, gradient_end_m = gradient_end_m, gradient_start_m
        if gradient_end_m > gradient_start_m:
            self.track_profile.append((gradient_start_m, gradient_end_m, gradient, psr_kmh))
            self.track_profile.sort(key=lambda item: item[0])
        if condition_end_m < condition_start_m:
            condition_start_m, condition_end_m = condition_end_m, condition_start_m
        if condition_end_m > condition_start_m:
            self.line_conditions.append(
                {
                    "start": condition_start_m,
                    "end": condition_end_m,
                    "condition": condition,
                }
            )
        for train in self.trains:
            train.track_profile = self.track_profile

    def update_line_condition(self, index: int, start_m: float, end_m: float, condition: str):
        if index < 0 or index >= len(self.line_conditions):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.line_conditions[index].update({"start": start_m, "end": end_m, "condition": condition})

    def update_tsr(self, index: int, start_m: float, end_m: float, speed_kmh: float):
        if index < 0 or index >= len(self.tsr_zones):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.tsr_zones[index].update({"start": start_m, "end": end_m, "speed": speed_kmh})

    def update_source_train(self, index: int, name: str, capacity: int, total_trains: int):
        if index < 0 or index >= len(self.source_trains):
            return
        source = self.source_trains[index]
        previous_name = str(source.get("name", name))
        capacity = max(1, capacity)
        total_trains = max(0, total_trains)
        source.update(
            {
                "name": name,
                "start_m": SOURCE_TRAIN_START_M,
                "length_m": SOURCE_TRAIN_LENGTH_M,
                "capacity": capacity,
                "total_trains": total_trains,
            }
        )
        self._sync_source_train_count(index, previous_name)

    def add_source_train(self, name: str, start_m: float, length_m: float, capacity: int, total_trains: int):
        capacity = max(1, capacity)
        total_trains = max(0, total_trains)
        source = {
            "name": name,
            "start_m": SOURCE_TRAIN_START_M,
            "length_m": SOURCE_TRAIN_LENGTH_M,
            "capacity": capacity,
            "total_trains": total_trains,
            "generated": 0,
        }
        self.source_trains.append(source)
        self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self._sync_source_train_count(len(self.source_trains) - 1)

    def _source_exit_clear(self, source: Dict[str, Any], exit_pos: float, train_length: float) -> bool:
        source_start = SOURCE_TRAIN_START_M
        source_end = SOURCE_TRAIN_EXIT_M
        protected_start = source_start - train_length
        protected_end = source_end + SOURCE_TRAIN_SPACING_M
        return not any((train.pos - train.length) < protected_end and train.pos > protected_start for train in self.trains)

    def _spawn_source_trains(self):
        changed = False
        for source in self.source_trains:
            total = int(source.get("total_trains", 0))
            generated = int(source.get("generated", 0))
            if generated >= total:
                continue
            train_length = float(self.scenario["train_defaults"]["length_m"])
            start_pos = self._source_staging_head_pos()
            if not self._source_exit_clear(source, start_pos, train_length):
                continue
            self.generated_train_counter += 1
            train_id = self._next_source_train_id(source, generated + 1)
            source["_pending_sequence"] = generated + 1
            train = Train(self._make_source_train_config(source, train_id, start_pos, 0))
            source.pop("_pending_sequence", None)
            dispatched_front_gap_m = max(
                (
                    existing.pos - SOURCE_TRAIN_EXIT_M
                    for existing in self.trains
                    if existing.headway_dispatch_released and existing.pos > SOURCE_TRAIN_EXIT_M
                ),
                default=None,
            )
            tsr_active = bool(self.tsr_zones)
            decision = self.headway_manager.decide(
                train,
                self._timetable_operational_time_s(),
                dispatched_front_pos_m=dispatched_front_gap_m,
                tsr_active=tsr_active,
            )
            if decision.decision == "HOLD":
                source["last_hold_reason"] = decision.reason
                source["planned_dispatch_time_s"] = decision.planned_dispatch_time_s
                continue
            train.headway_target_s = decision.target_headway_s
            train.headway_planned_dispatch_s = decision.planned_dispatch_time_s
            train.headway_dispatch_delay_s = decision.dispatch_delay_s
            train.headway_dispatch_released = True
            train.headway_hold_reason = decision.reason
            train.source_lane = 0
            self.trains.append(train)
            source["generated"] = generated + 1
            source["last_hold_reason"] = ""
            changed = True
        if changed:
            self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
            self._dispatch_safe_packets(with_delay=False)
        return changed

    def _scheduled_stop_eoa(self, stop_pos_m: float) -> float:
        return stop_pos_m - (STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)

    def _source_zone_contains(self, train: Train) -> bool:
        if not self.source_trains:
            return False
        if train.source_lane is not None:
            return train.safe_rear_end_pos() <= SOURCE_TRAIN_EXIT_M
        return train.pos >= SOURCE_TRAIN_START_M and train.pos <= SOURCE_TRAIN_EXIT_M + STOP_ACCURACY_TOL_M

    def _update_source_headway_releases(self):
        source_trains = [
            train for train in self.trains
            if train.source_lane is not None and self._source_zone_contains(train)
        ]
        source_trains.sort(key=lambda item: (item.source_lane if item.source_lane is not None else 0, item.id))
        for train in source_trains:
            if train.headway_dispatch_released:
                train.departure_hold = False
                continue
            released_front_gaps = [
                existing.pos - SOURCE_TRAIN_EXIT_M
                for existing in self.trains
                if (
                    existing is not train
                    and existing.headway_dispatch_released
                    and existing.pos > SOURCE_TRAIN_EXIT_M
                )
            ]
            dispatched_front_gap_m = max(released_front_gaps, default=None)
            decision = self.headway_manager.decide(
                train,
                self._timetable_operational_time_s(),
                dispatched_front_pos_m=dispatched_front_gap_m,
                tsr_active=bool(self.tsr_zones),
            )
            train.headway_target_s = decision.target_headway_s
            train.headway_planned_dispatch_s = decision.planned_dispatch_time_s
            train.headway_dispatch_delay_s = decision.dispatch_delay_s
            train.headway_hold_reason = decision.reason
            if decision.decision == "HOLD":
                train.departure_hold = True
                continue
            train.headway_dispatch_released = True
            train.departure_hold = False

    def _station_for_train(self, train: Train) -> Tuple[int, Dict[str, Any]] | None:
        for idx, stop in enumerate(self.scheduled_stops):
            pos_m = float(stop["pos_m"])
            length_m = float(stop.get("length_m", 160.0))
            station_start = pos_m - length_m / 2.0
            station_end = pos_m + length_m / 2.0
            if station_start <= train.pos and train.safe_rear_end_pos() <= station_end:
                return idx, stop
        return None

    def _station_overlapped_by_train(self, train: Train) -> Tuple[int, Dict[str, Any]] | None:
        for idx, stop in enumerate(self.scheduled_stops):
            if self._train_overlaps_station(train, stop):
                return idx, stop
        return None

    def _station_bounds(self, stop: Dict[str, Any]) -> Tuple[float, float]:
        pos_m = float(stop["pos_m"])
        length_m = float(stop.get("length_m", 160.0))
        return pos_m - length_m / 2.0, pos_m + length_m / 2.0

    def _same_station_stop(self, left: Dict[str, Any] | None, right: Dict[str, Any] | None) -> bool:
        if left is None or right is None:
            return False
        return (
            str(left.get("name", "")) == str(right.get("name", ""))
            and abs(float(left.get("pos_m", 0.0)) - float(right.get("pos_m", 0.0))) <= 1e-9
        )

    def _station_index_for_stop(self, stop: Dict[str, Any] | None) -> int | None:
        for idx, candidate in enumerate(self.scheduled_stops):
            if self._same_station_stop(stop, candidate):
                return idx
        return None

    def _train_overlaps_station(self, train: Train, stop: Dict[str, Any]) -> bool:
        station_start, station_end = self._station_bounds(stop)
        return train.safe_rear_end_pos() < station_end and train.pos > station_start

    def _train_head_in_station(self, train: Train, stop: Dict[str, Any]) -> bool:
        station_start, station_end = self._station_bounds(stop)
        return station_start <= train.pos <= station_end

    def _station_line_id(self, station_idx: int, lane: int) -> str:
        stop = self.scheduled_stops[station_idx]
        return f"{stop.get('name', f'STATION_{station_idx}')}:{int(lane)}"

    def detect_station_lines(self, station_idx: int) -> List[Dict[str, Any]]:
        self._sync_station_route_states() if station_idx >= len(self.station_route_states) else None
        stop = self.scheduled_stops[station_idx]
        state = self.station_route_states[station_idx]
        capacity = max(1, int(stop.get("capacity", 3)))
        station_start, station_end = self._station_bounds(stop)
        stop_point_m = float(stop["pos_m"])
        existing = {int(line.get("lane", idx)): line for idx, line in enumerate(state.get("lines", []))}
        lines: List[Dict[str, Any]] = []
        for lane in range(capacity):
            line = dict(existing.get(lane, {}))
            line.update(
                {
                    "lane": lane,
                    "line_id": self._station_line_id(station_idx, lane),
                    "station_id": str(stop.get("name", station_idx)),
                    "start_m": station_start,
                    "end_m": station_end,
                    "stop_point_m": stop_point_m,
                    "type": "MAIN" if lane == 0 else "PLATFORM",
                    "capacity": int(line.get("capacity", 1)),
                    "reserved_by_train_id": line.get("reserved_by_train_id"),
                    "occupied_by_train_id": line.get("occupied_by_train_id"),
                    "route_state": line.get("route_state", "FREE"),
                }
            )
            lines.append(line)
        state["lines"] = lines
        return lines

    def _station_line_for_lane(self, station_idx: int, lane: int | None) -> Dict[str, Any] | None:
        if lane is None or not (0 <= station_idx < len(self.station_route_states)):
            return None
        for line in self.detect_station_lines(station_idx):
            if int(line["lane"]) == int(lane):
                return line
        return None

    def _train_overlaps_station_line(self, train: Train, line: Dict[str, Any]) -> bool:
        head_pos = train.pos
        tail_pos = train.pos - train.length
        return tail_pos < float(line["end_m"]) and head_pos > float(line["start_m"])

    def _station_line_tail_clear(self, train: Train, line: Dict[str, Any]) -> bool:
        return train.safe_rear_end_pos() > float(line["end_m"]) + PARALLEL_RELEASE_MARGIN_M

    def _train_stably_stopped_in_station_line(self, train: Train, station_idx: int) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        return (
            train.zero_speed_detected
            and train.speed <= STANDSTILL_SPEED_EPS
            and abs(train.pos - float(line["stop_point_m"])) <= STOP_ACCURACY_TOL_M
            and line.get("occupied_by_train_id") == train.id
        )

    def _station_line_reservation_still_valid(
        self,
        station_idx: int,
        stop: Dict[str, Any],
        line: Dict[str, Any],
        train: Train | None,
    ) -> bool:
        if train is None:
            return False
        if self._train_overlaps_station_line(train, line):
            return True
        assigned_to_line = train.station_lane is not None and int(train.station_lane) == int(line["lane"])
        if not assigned_to_line:
            return False
        active_for_station = self._same_station_stop(train.active_scheduled_stop, stop)
        lifecycle_holds_station = (
            train.last_station_idx == station_idx
            and train.station_state in {
                "ROUTE_ASSIGNED",
                "DOCKING",
                "STOPPED_AT_PLATFORM",
                "DWELLING",
                "READY_TO_DEPART",
                "DEPARTING",
            }
        )
        if not active_for_station and not lifecycle_holds_station:
            return False
        return (
            not self._station_line_tail_clear(train, line)
            or train.pos <= float(line["end_m"]) + PARALLEL_RELEASE_MARGIN_M
        )

    def update_station_occupancy(self, station_idx: int) -> None:
        state = self.station_route_states[station_idx]
        stop = self.scheduled_stops[station_idx]
        lines = self.detect_station_lines(station_idx)
        route_lane = state.get("route_lane")
        locking_train_id = state.get("locking_train_id")
        assigned_train_id = state.get("assigned_train_id")
        for line in lines:
            lane = int(line["lane"])
            previous_occupied = line.get("occupied_by_train_id")
            overlapping = [
                train
                for train in self.trains
                if self._train_overlaps_station_line(train, line)
                and (
                    train.station_lane is None
                    or int(train.station_lane) == lane
                    or self._station_physical_lane(train, station_idx) == lane
                )
            ]
            occupying_train = min(overlapping, key=lambda item: abs(float(stop["pos_m"]) - item.pos), default=None)
            line["occupied_by_train_id"] = occupying_train.id if occupying_train is not None else None
            if route_lane == lane and assigned_train_id is not None:
                line["reserved_by_train_id"] = assigned_train_id
            elif line.get("reserved_by_train_id") is not None:
                reserved_train = next((train for train in self.trains if train.id == line.get("reserved_by_train_id")), None)
                if not self._station_line_reservation_still_valid(station_idx, stop, line, reserved_train):
                    line["reserved_by_train_id"] = None
            if occupying_train is not None:
                if occupying_train.station_lane is None:
                    occupying_train.station_lane = lane
                occupying_train.assigned_station_id = str(stop.get("name", station_idx))
                occupying_train.assigned_station_line_id = line["line_id"]
                occupying_train.assigned_platform = lane
                if previous_occupied != occupying_train.id:
                    self.log_station_event(station_idx, occupying_train, line, "STATION_LINE_OCCUPIED")
                line["route_state"] = "OCCUPIED"
            elif locking_train_id is not None and route_lane == lane:
                line["route_state"] = "LOCKED"
            elif route_lane == lane or line.get("reserved_by_train_id") is not None:
                line["route_state"] = "RESERVED"
            elif previous_occupied is not None:
                line["route_state"] = "RELEASE_PENDING"
            else:
                line["route_state"] = "FREE"

    def _station_slot_count(self, station_idx: int) -> int:
        count = 0
        for line in self.detect_station_lines(station_idx):
            if (
                line.get("reserved_by_train_id") is not None
                or line.get("occupied_by_train_id") is not None
                or line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
            ):
                count += 1
        return count

    def _station_reserved_count(self, station_idx: int) -> int:
        return sum(1 for line in self.detect_station_lines(station_idx) if line.get("reserved_by_train_id") is not None)

    def _station_occupied_count(self, station_idx: int) -> int:
        return sum(1 for line in self.detect_station_lines(station_idx) if line.get("occupied_by_train_id") is not None)

    def can_accept_train(self, station_idx: int, train: Train) -> Tuple[bool, str]:
        state = self.station_route_states[station_idx]
        capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
        if state.get("invariant_block"):
            return False, "ROUTE_CONFLICT"
        if self._station_receiving_route_active_for_other_train(station_idx, train):
            return False, "ROUTE_CONFLICT"
        if train.station_lane is not None:
            existing_line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if existing_line is not None and existing_line.get("reserved_by_train_id") == train.id:
                return True, "OK"
        if self._station_slot_count(station_idx) >= capacity:
            return False, "STATION_FULL"
        if train.station_lane is not None:
            line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if line is None:
                return False, "INVALID_LINE_ASSIGNMENT"
            if self._station_line_available_for_receive_route(station_idx, line, train):
                return True, "OK"
            return False, "STATION_LINE_OCCUPIED"
        if any(self._station_line_available_for_receive_route(station_idx, line, train) for line in self.detect_station_lines(station_idx)):
            return True, "OK"
        return False, "NO_FREE_PLATFORM"

    def _station_line_available_for_train(self, line: Dict[str, Any], train: Train) -> bool:
        reserved_by = line.get("reserved_by_train_id")
        occupied_by = line.get("occupied_by_train_id")
        if reserved_by is not None and reserved_by != train.id:
            return False
        if occupied_by is not None and occupied_by != train.id:
            return False
        return line.get("route_state") not in {"LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"} or reserved_by == train.id

    def _station_receiving_route_active_for_other_train(self, station_idx: int, train: Train) -> bool:
        state = self.station_route_states[station_idx]
        assigned_train_id = state.get("assigned_train_id")
        return (
            state.get("route_lane") is not None
            and assigned_train_id is not None
            and assigned_train_id != train.id
        )

    def _station_line_available_for_receive_route(self, station_idx: int, line: Dict[str, Any], train: Train) -> bool:
        if self._station_receiving_route_active_for_other_train(station_idx, train):
            return False
        reserved_by = line.get("reserved_by_train_id")
        occupied_by = line.get("occupied_by_train_id")
        if reserved_by is not None and reserved_by != train.id:
            return False
        if occupied_by is not None and occupied_by != train.id:
            return False
        if line.get("route_state") in {"LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}:
            return False
        if reserved_by == train.id:
            return True
        for other in self.trains:
            if other.id == train.id:
                continue
            if self._train_overlaps_station_line(other, line):
                other_lane = self._station_physical_lane(other, station_idx)
                if other_lane is None and other.station_lane is not None and other.last_station_idx == station_idx:
                    other_lane = int(other.station_lane)
                if other_lane is None or int(other_lane) == int(line["lane"]):
                    return False
                if not self._train_stably_stopped_in_station_line(other, station_idx):
                    return False
            if (
                other.station_lane is not None
                and int(other.station_lane) == int(line["lane"])
                and other.last_station_idx == station_idx
                and not self._station_line_tail_clear(other, line)
            ):
                return False
        return True

    def _station_holding_eoa(self, station_idx: int) -> float:
        station_start, _station_end = self._station_bounds(self.scheduled_stops[station_idx])
        return station_start - STOP_TARGET_BUFFER_M

    def assign_receive_route(self, station_idx: int, train: Train) -> bool:
        can_accept, reason = self.can_accept_train(station_idx, train)
        if not can_accept:
            train.station_reject_reason = reason
            line = self._station_line_for_lane(station_idx, train.station_lane)
            self.log_station_event(station_idx, train, line, reason, reject_reason=reason)
            return False
        lines = self.detect_station_lines(station_idx)
        candidate_line = None
        if train.station_lane is not None:
            line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if line is not None and self._station_line_available_for_receive_route(station_idx, line, train):
                candidate_line = line
        else:
            for line in lines:
                if self._station_line_available_for_receive_route(station_idx, line, train):
                    candidate_line = line
                    break
        if candidate_line is None:
            train.station_reject_reason = "NO_FREE_PLATFORM"
            self.log_station_event(station_idx, train, None, "NO_FREE_PLATFORM", reject_reason="NO_FREE_PLATFORM")
            return False
        lane = int(candidate_line["lane"])
        train.station_reject_reason = ""
        train.station_lane = lane
        train.assigned_platform = lane
        train.assigned_station_id = str(candidate_line["station_id"])
        train.assigned_station_line_id = str(candidate_line["line_id"])
        candidate_line["reserved_by_train_id"] = train.id
        candidate_line["route_state"] = "RESERVED"
        self.lock_station_line(station_idx, lane, train)
        self.assign_stop_target(station_idx, train)
        self.log_station_event(station_idx, train, candidate_line, "RECEIVE_ROUTE_ASSIGNED")
        return True

    def lock_station_line(self, station_idx: int, lane: int, train: Train) -> None:
        state = self.station_route_states[station_idx]
        line = self._station_line_for_lane(station_idx, lane)
        state["route_lane"] = int(lane)
        state["assigned_train_id"] = train.id
        state["route_state"] = "RECEIVE_OPEN"
        if line is not None:
            line["reserved_by_train_id"] = train.id
            line["route_state"] = "RESERVED"

    def assign_stop_target(self, station_idx: int, train: Train) -> float:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        stop_target = float(line["stop_point_m"]) if line is not None else float(self.scheduled_stops[station_idx]["pos_m"])
        train.stop_target_pos = stop_target
        self.log_station_event(station_idx, train, line, "STOP_TARGET_ASSIGNED")
        return stop_target

    def start_dwell_if_stopped_correctly(self, station_idx: int, train: Train, stop: Dict[str, Any]) -> bool:
        if not (train.commanded_stop and train.zero_speed_detected and train.door_authorized):
            return False
        if train.station_lane is None or abs(train.pos - float(stop["pos_m"])) > STOP_ACCURACY_TOL_M:
            return False
        self.assign_stop_target(station_idx, train)
        train.pos = train.stop_target_pos
        train.speed = 0.0
        train.prev_accel = 0.0
        train.reset_non_emergency_stop_latches()
        train.standstill_required = True
        train.standstill_anchor_pos = train.pos
        self._set_train_station_state(train, station_idx, "STOPPED_AT_PLATFORM", "aligned_stop")
        train.dwell_remaining_s = self._station_dwell_time_s(station_idx, stop, train)
        self._record_station_arrival(station_idx, train, train.dwell_remaining_s)
        train.next_scheduled_stop_idx += 1
        self._set_train_station_state(train, station_idx, "DWELLING", "dwell_started")
        self.log_station_event(station_idx, train, self._station_line_for_lane(station_idx, train.station_lane), "DWELL_STARTED")
        return True

    def prepare_departure_route(self, station_idx: int, train: Train) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            self.log_station_event(station_idx, train, None, "INVALID_LINE_ASSIGNMENT", reject_reason="INVALID_LINE_ASSIGNMENT")
            return False
        train.station_reject_reason = ""
        line["route_state"] = "DEPARTING"
        self.log_station_event(station_idx, train, line, "DEPARTURE_ROUTE_ASSIGNED")
        return True

    def release_station_line_only_after_tail_clear(self, station_idx: int, train: Train) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        if not self._station_line_tail_clear(train, line):
            line["route_state"] = "RELEASE_PENDING"
            self.log_station_event(station_idx, train, line, "TAIL_NOT_CLEAR")
            return False
        line["occupied_by_train_id"] = None
        line["reserved_by_train_id"] = None
        line["route_state"] = "FREE"
        state = self.station_route_states[station_idx]
        if state.get("assigned_train_id") == train.id:
            state["assigned_train_id"] = None
        if state.get("locking_train_id") == train.id:
            state["locking_train_id"] = None
        self.log_station_event(station_idx, train, line, "TAIL_CLEAR_RELEASED")
        self.log_station_event(station_idx, train, line, "LINE_RELEASED")
        self.clear_completed_stop_for_train(station_idx, train)
        return True

    def clear_completed_stop_for_train(self, station_idx: int | None, train: Train) -> None:
        train.station_lane = None
        train.assigned_platform = None
        train.assigned_station_id = None
        train.assigned_station_line_id = None
        self._set_train_station_state(train, station_idx, "COMPLETED_STOP", "station_line_released")

    def log_station_event(self, station_idx: int, train: Train | None, line: Dict[str, Any] | None, reason: str, **extra):
        stop = self.scheduled_stops[station_idx]
        station_id = str(stop.get("name", station_idx))
        capacity = max(1, int(stop.get("capacity", 3)))
        if train is None:
            return
        station_start, station_end = self._station_bounds(stop)
        train.log_event(
            "STATION_EVENT",
            reason,
            station_id=station_id,
            station_capacity=capacity,
            station_reserved_count=self._station_reserved_count(station_idx),
            station_occupied_count=self._station_occupied_count(station_idx),
            train_state_at_station=train.station_state,
            assigned_station_line_id=train.assigned_station_line_id,
            line_id=None if line is None else line.get("line_id"),
            line_type=None if line is None else line.get("type"),
            line_state=None if line is None else line.get("route_state"),
            reserved_by_train_id=None if line is None else line.get("reserved_by_train_id"),
            occupied_by_train_id=None if line is None else line.get("occupied_by_train_id"),
            train_head_pos=train.pos,
            train_tail_pos=train.pos - train.length,
            line_start_m=station_start if line is None else line.get("start_m"),
            line_end_m=station_end if line is None else line.get("end_m"),
            stop_target_m=train.stop_target_pos,
            route_lock_state=self._route_state_label(self.station_route_states[station_idx]),
            departure_route_state=None if line is None else line.get("route_state"),
            tail_clear=False if line is None else self._station_line_tail_clear(train, line),
            dwell_remaining=train.dwell_remaining_s,
            **extra,
        )

    def _check_station_invariants(self) -> bool:
        ok = True
        for station_idx, stop in enumerate(self.scheduled_stops):
            self.update_station_occupancy(station_idx)
            state = self.station_route_states[station_idx]
            state["invariant_block"] = False
            capacity = max(1, int(stop.get("capacity", 3)))
            active_slots = self._station_slot_count(station_idx)
            if active_slots > capacity:
                ok = False
                state["invariant_block"] = True
                train = next((item for item in self.trains if self._station_overlapped_by_train(item) and self._station_overlapped_by_train(item)[0] == station_idx), None)
                if train is not None:
                    self.log_station_event(station_idx, train, None, "STATION_FULL", reject_reason="STATION_CAPACITY_INVARIANT")
            for line in self.detect_station_lines(station_idx):
                line_capacity = max(1, int(line.get("capacity", 1)))
                reserved_train_ids = {
                    str(line.get("reserved_by_train_id"))
                    for line_candidate in [line]
                    if line_candidate.get("reserved_by_train_id") is not None
                }
                overlapping = [
                    train
                    for train in self.trains
                    if self._train_overlaps_station_line(train, line)
                    and (train.station_lane is None or int(train.station_lane) == int(line["lane"]))
                ]
                unique_users = set(reserved_train_ids)
                unique_users.update(train.id for train in overlapping)
                if len(unique_users) > line_capacity:
                    ok = False
                    state["invariant_block"] = True
                    train = overlapping[0] if overlapping else next((item for item in self.trains if item.id in unique_users), None)
                    if train is not None:
                        self.log_station_event(station_idx, train, line, "ROUTE_CONFLICT", reject_reason="LINE_CAPACITY_INVARIANT")
                if line.get("route_state") == "FREE" and overlapping:
                    ok = False
                    state["invariant_block"] = True
                    self.log_station_event(station_idx, overlapping[0], line, "TAIL_NOT_CLEAR", reject_reason="FREE_WITH_OVERLAP")
                for train in overlapping:
                    if train.station_state in {"DOCKING", "DWELLING", "READY_TO_DEPART"} and train.assigned_station_line_id not in {None, line.get("line_id")}:
                        ok = False
                        state["invariant_block"] = True
                        self.log_station_event(station_idx, train, line, "INVALID_LINE_ASSIGNMENT", reject_reason="STOP_TARGET_LINE_MISMATCH")
        return ok

    def _station_physical_lane(self, train: Train, station_idx: int) -> int | None:
        zone_id = f"STATION:{station_idx}"
        if train.protection_zone_id == zone_id:
            return int(train.protection_lane)
        station = self._station_overlapped_by_train(train)
        if station is None or station[0] != station_idx:
            return None
        if train.station_lane is not None and self._same_station_stop(train.active_scheduled_stop, station[1]):
            return int(train.station_lane)
        return None

    def _station_lane_occupied(self, station_idx: int, lane: int) -> bool:
        line = self._station_line_for_lane(station_idx, lane)
        if line is not None and (
            line.get("reserved_by_train_id") is not None
            or line.get("occupied_by_train_id") is not None
            or line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
        ):
            return True
        stop = self.scheduled_stops[station_idx]
        return any(
            self._station_physical_lane(train, station_idx) == lane
            and self._train_overlaps_station(train, stop)
            for train in self.trains
        )

    def _station_route_lane_order(self, capacity: int) -> List[int]:
        return list(range(max(1, capacity)))

    def _is_terminal_station(self, station_idx: int | None) -> bool:
        if station_idx is None or not (0 <= station_idx < len(self.scheduled_stops)):
            return False
        stop = self.scheduled_stops[station_idx]
        return float(stop.get("pos_m", 0.0)) >= self.track_end_m - STOP_ACCURACY_TOL_M

    def _is_final_scheduled_station(self, station_idx: int | None) -> bool:
        return station_idx is not None and station_idx == len(self.scheduled_stops) - 1

    def _schedule_record_for_train_station(self, train: Train | None, stop: Dict[str, Any], station_idx: int | None = None) -> Dict[str, Any] | None:
        if train is None or self.headway_manager.mode != "timetable":
            return None
        station_name = str(stop.get("name", "")).strip().lower()
        station_aliases = {station_name}
        if station_idx is not None:
            station_aliases.add(f"s{station_idx + 2}".lower())
        for record in getattr(train, "schedule_records", []) or []:
            if str(record.get("station", "")).strip().lower() in station_aliases:
                return dict(record)
        return None

    def _station_dwell_time_s(self, station_idx: int | None, stop: Dict[str, Any], train: Train | None = None) -> float:
        if self._is_terminal_station(station_idx) or self._is_final_scheduled_station(station_idx):
            return float("inf")
        schedule_record = self._schedule_record_for_train_station(train, stop, station_idx)
        if schedule_record is not None:
            planned_departure_s = schedule_record.get("departure_time_s")
            planned_arrival_s = schedule_record.get("arrival_time_s")
            operational_time_s = self._timetable_operational_time_s()
            late_for_schedule = planned_arrival_s is not None and operational_time_s > float(planned_arrival_s)
            if late_for_schedule:
                late_s = operational_time_s - float(planned_arrival_s)
                scheduled_dwell_s = schedule_record.get("dwell_s")
                if scheduled_dwell_s is None and planned_departure_s is not None:
                    scheduled_dwell_s = max(0.0, float(planned_departure_s) - float(planned_arrival_s))
                if scheduled_dwell_s is not None:
                    return max(MIN_TIMETABLE_RECOVERY_DWELL_S, float(scheduled_dwell_s) - late_s)
                return max(MIN_TIMETABLE_RECOVERY_DWELL_S, MIN_PASSENGER_DWELL_S - late_s)
            if planned_departure_s is not None:
                return max(MIN_PASSENGER_DWELL_S, float(planned_departure_s) - operational_time_s)
            scheduled_dwell_s = schedule_record.get("dwell_s")
            if scheduled_dwell_s is not None:
                return max(MIN_PASSENGER_DWELL_S, float(scheduled_dwell_s))
        minimum_departure_s = self.sim_time_s + MIN_PASSENGER_DWELL_S
        if station_idx is None:
            return MIN_PASSENGER_DWELL_S
        target_headway_s = max(0.0, float(self.headway_manager.nominal_target_headway_s()))
        if target_headway_s <= 0.0:
            return MIN_PASSENGER_DWELL_S
        previous_candidates = [
            value
            for value in (
                self.station_next_departure_release_s.get(station_idx),
                self.station_last_departure_s.get(station_idx),
            )
            if value is not None
        ]
        previous_slot_s = max(previous_candidates) if previous_candidates else None
        if previous_slot_s is None:
            departure_slot_s = minimum_departure_s
        else:
            departure_slot_s = max(minimum_departure_s, previous_slot_s + target_headway_s)
        self.station_next_departure_release_s[station_idx] = departure_slot_s
        return max(MIN_PASSENGER_DWELL_S, departure_slot_s - self.sim_time_s)

    def _record_station_departure_headway(self, station_idx: int, train: Train):
        previous = self.station_last_departure_s.get(station_idx)
        self.station_last_departure_s[station_idx] = self.sim_time_s
        self.station_next_departure_release_s[station_idx] = max(
            self.station_next_departure_release_s.get(station_idx, self.sim_time_s),
            self.sim_time_s,
        )
        for record in reversed(self.analytics.get("station_arrivals", {}).get(station_idx, [])):
            if record.get("train_id") == train.id and record.get("departure_time_s") is None:
                record["departure_time_s"] = self.sim_time_s
                arrival_time_s = record.get("arrival_time_s")
                if arrival_time_s is not None:
                    record["actual_station_wait_s"] = max(0.0, self.sim_time_s - float(arrival_time_s))
                break
        if previous is None:
            return
        actual = max(0.0, self.sim_time_s - previous)
        target = self.headway_manager.nominal_target_headway_s()
        self.station_headway_actual_s.setdefault(station_idx, []).append(actual)
        if target > 0.0:
            self.station_headway_deviation_s.setdefault(station_idx, []).append(actual - target)

    def _station_departure_headway_hold_s(self, station_idx: int | None) -> float:
        if station_idx is None:
            return 0.0
        previous = self.station_last_departure_s.get(station_idx)
        if previous is None:
            return 0.0
        target_headway_s = max(0.0, float(self.headway_manager.nominal_target_headway_s()))
        if target_headway_s <= 0.0:
            return 0.0
        return max(0.0, previous + target_headway_s - self.sim_time_s)

    def _station_schedule_departure_hold_s(self, station_idx: int | None, train: Train) -> float:
        if station_idx is None or self.headway_manager.mode != "timetable":
            return 0.0
        if not (0 <= station_idx < len(self.scheduled_stops)):
            return 0.0
        record = self._schedule_record_for_train_station(train, self.scheduled_stops[station_idx], station_idx)
        if record is None or record.get("departure_time_s") is None:
            return 0.0
        operational_time_s = self._timetable_operational_time_s()
        planned_arrival_s = record.get("arrival_time_s")
        if planned_arrival_s is not None and operational_time_s > float(planned_arrival_s):
            return 0.0
        return max(0.0, float(record["departure_time_s"]) - operational_time_s)

    def _record_station_arrival(self, station_idx: int, train: Train, dwell_s: float) -> None:
        previous = self.station_last_arrival_s.get(station_idx)
        self.station_last_arrival_s[station_idx] = self.sim_time_s
        arrival_headway_s = None
        if previous is not None:
            arrival_headway_s = max(0.0, self.sim_time_s - previous)
            self.station_arrival_headway_actual_s.setdefault(station_idx, []).append(arrival_headway_s)
        schedule_record = (
            self._schedule_record_for_train_station(train, self.scheduled_stops[station_idx], station_idx)
            if 0 <= station_idx < len(self.scheduled_stops)
            else None
        )
        schedule_variance_s = None
        if schedule_record is not None and schedule_record.get("arrival_time_s") is not None:
            schedule_variance_s = self._timetable_operational_time_s() - float(schedule_record["arrival_time_s"])
        self.analytics.setdefault("station_arrivals", {}).setdefault(station_idx, []).append(
            {
                "train_id": train.id,
                "arrival_time_s": self.sim_time_s,
                "arrival_headway_s": arrival_headway_s,
                "scheduled_arrival_time_s": None if schedule_record is None else schedule_record.get("arrival_time_s"),
                "schedule_station": None if schedule_record is None else schedule_record.get("station"),
                "schedule_variance_s": schedule_variance_s,
                "planned_dwell_s": dwell_s,
                "passenger_dwell_s": dwell_s,
                "station_wait_s": dwell_s,
            }
        )

    def _set_train_station_state(self, train: Train, station_idx: int | None, state: str, reason: str = ""):
        stop_key = train.stop_identity() if train.active_scheduled_stop is not None else train.station_state_stop_key
        if state == train.station_state and stop_key == train.station_state_stop_key and station_idx == train.last_station_idx:
            return
        train.station_state = state
        train.station_state_stop_key = stop_key
        train.last_station_idx = station_idx
        train.assigned_platform = train.station_lane
        train.last_station_state_reason = reason
        station_id = "--"
        if station_idx is not None and 0 <= station_idx < len(self.scheduled_stops):
            station_id = str(self.scheduled_stops[station_idx].get("name", station_idx))
        train.log_event(
            "STATION_STATE",
            reason or state,
            station_id=station_id,
            station_state=state,
            assigned_line=train.station_lane,
        )

    def _route_state_label(self, state: Dict[str, Any]) -> str:
        if state.get("route_lane") is not None:
            return "RECEIVE_OPEN"
        if state.get("locking_train_id") is not None:
            return "LOCKED"
        return "FREE"

    def _log_eoa_update(self, train: Train, old_eoa: float | None, new_eoa: float, reason: str, station_idx: int | None):
        if old_eoa is not None and abs(old_eoa - new_eoa) <= 1e-6 and reason == train.last_dispatched_eoa_reason:
            return
        station_id = "--"
        route_state = "--"
        if station_idx is not None and 0 <= station_idx < len(self.station_route_states):
            station_id = str(self.scheduled_stops[station_idx].get("name", station_idx))
            route_state = self._route_state_label(self.station_route_states[station_idx])
        train.log_event(
            "EOA_UPDATE",
            reason,
            station_id=station_id,
            station_state=train.station_state,
            assigned_line=train.station_lane,
            route_state=route_state,
            old_eoa=old_eoa,
            new_eoa=new_eoa,
            assigned_station_id=train.assigned_station_id,
            assigned_station_line_id=train.assigned_station_line_id,
            station_receive_state=train.station_state,
            station_reject_reason=train.station_reject_reason,
            stop_target=train.stop_target_pos,
            distance_to_stop=train.distance_to_stop_target(),
            atp_curve_limit_kmh=ms_to_kmh(train.curves.get("P", 0.0)),
            sbi_state=train.service_brake_latch,
            ebi_state=train.emg_latch,
            dwell_remaining=train.dwell_remaining_s,
            tail_clear_state=(
                station_idx is None
                or not self._train_overlaps_station(train, self.scheduled_stops[station_idx])
            ),
        )
        train.last_dispatched_eoa = new_eoa
        train.last_dispatched_eoa_reason = reason

    def _update_station_routes(self) -> bool:
        changed = False
        self._sync_station_route_states()
        for station_idx, stop in enumerate(self.scheduled_stops):
            capacity = max(1, int(stop.get("capacity", 3)))
            state = self.station_route_states[station_idx]
            self.update_station_occupancy(station_idx)
            station_start, station_end = self._station_bounds(stop)
            route_lane = state.get("route_lane")

            if route_lane is not None:
                for train in self.trains:
                    if (
                        self._train_overlaps_station(train, stop)
                        and (
                            train.station_lane == route_lane
                            or self._station_physical_lane(train, station_idx) == route_lane
                        )
                    ):
                        # As soon as the train touches the routed station line, revoke the green aspect.
                        state["route_lane"] = None
                        state["assigned_train_id"] = train.id
                        state["locking_train_id"] = train.id
                        state["route_state"] = "LOCKED"
                        line = self._station_line_for_lane(station_idx, int(route_lane))
                        if line is not None:
                            line["reserved_by_train_id"] = train.id
                            line["occupied_by_train_id"] = train.id
                            line["route_state"] = "OCCUPIED"
                        state["switch_started"] = False
                        state["lock_remaining_s"] = 0.0
                        self._set_train_station_state(train, station_idx, "DOCKING", "route_consumed")
                        route_lane = None
                        changed = True
                        break

            locking_train = next((train for train in self.trains if train.id == state.get("locking_train_id")), None)
            if locking_train is not None:
                if locking_train.speed <= STANDSTILL_SPEED_EPS and self._train_overlaps_station(locking_train, stop):
                    if not state.get("switch_started", False):
                        state["lock_remaining_s"] = TURNOUT_LOCK_S
                        state["switch_started"] = True
                        if locking_train.dwell_remaining_s > 0.0:
                            self._set_train_station_state(locking_train, station_idx, "DWELLING", "switch_lock_started")
                        else:
                            self._set_train_station_state(locking_train, station_idx, "STOPPED_AT_PLATFORM", "switch_lock_started")
                        changed = True
                    else:
                        state["lock_remaining_s"] = max(0.0, float(state.get("lock_remaining_s", 0.0)) - DT)
                if state.get("switch_started") and float(state.get("lock_remaining_s", 0.0)) <= 0.0:
                    state["locking_train_id"] = None
                    state["switch_started"] = False
                    state["route_state"] = "FREE"
                    changed = True
            elif float(state.get("lock_remaining_s", 0.0)) > 0.0:
                state["lock_remaining_s"] = max(0.0, float(state.get("lock_remaining_s", 0.0)) - DT)

            self.update_station_occupancy(station_idx)

            target_zone_id = f"STATION:{station_idx}"
            for train in self.trains:
                if (
                    self._same_station_stop(train.active_scheduled_stop, stop)
                    and train.station_lane is not None
                    and not self._train_overlaps_station(train, stop)
                ):
                    current_station = self._station_for_train(train)
                    if (
                        current_station is not None
                        and not self._same_station_stop(current_station[1], stop)
                        and train.protection_zone_id != target_zone_id
                        and self._station_overlapped_by_train(train) is None
                    ):
                        self.clear_completed_stop_for_train(station_idx, train)

            preassigned = [
                train for train in self.trains
                if self._same_station_stop(train.active_scheduled_stop, stop)
                and train.pos < station_end
                and train.pos >= station_start - STATION_ROUTE_APPROACH_M
                and train.station_lane is not None
                and not self._train_overlaps_station(train, stop)
            ]
            if preassigned:
                preassigned.sort(key=lambda item: abs(float(stop["pos_m"]) - item.pos))
                for candidate in preassigned:
                    lane = int(candidate.station_lane)
                    if 0 <= lane < capacity and self.assign_receive_route(station_idx, candidate):
                        self._set_train_station_state(candidate, station_idx, "ROUTE_ASSIGNED", "preassigned_route_open")
                        changed = True
                        break

            approaching = [
                train for train in self.trains
                if self._same_station_stop(train.active_scheduled_stop, stop)
                and train.pos < station_end
                and train.pos >= station_start - STATION_ROUTE_APPROACH_M
                and train.station_lane is None
            ]
            if not approaching:
                continue
            approaching.sort(key=lambda item: (int(item.protection_lane), abs(float(stop["pos_m"]) - item.pos)))
            for candidate in approaching:
                if self.assign_receive_route(station_idx, candidate):
                    self._set_train_station_state(candidate, station_idx, "ROUTE_ASSIGNED", "route_open")
                    changed = True
                    break
        return changed

    def _train_has_station_route_authority(self, train: Train) -> bool:
        if train.active_scheduled_stop is None or train.station_lane is None:
            return False
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return False
        capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
        if not (0 <= int(train.station_lane) < capacity):
            return False
        state = self.station_route_states[station_idx]
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        if state.get("route_lane") == train.station_lane and state.get("assigned_train_id") == train.id:
            return True
        if line.get("reserved_by_train_id") == train.id:
            return True
        if line.get("occupied_by_train_id") == train.id:
            return True
        return self._train_overlaps_station_line(train, line)

    def _train_has_station_stop_eoa_authority(self, train: Train) -> bool:
        if not self._train_has_station_route_authority(train):
            return False
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return False
        stop = self.scheduled_stops[station_idx]
        _station_start, station_end = self._station_bounds(stop)
        if train.pos > station_end:
            return False
        if train.commanded_stop:
            return True
        state = self.station_route_states[station_idx] if station_idx < len(self.station_route_states) else {}
        line = self._station_line_for_lane(station_idx, train.station_lane)
        # A reserved receive route is route authority, not yet stop authority.
        # Keep the station stop EOA out until the train enters the commanded-stop
        # approach; otherwise ATP/ATO starts braking hundreds of metres too early.
        if line is not None and line.get("reserved_by_train_id") == train.id:
            return self._train_overlaps_station(train, stop)
        return (
            (
                state.get("route_lane") == train.station_lane
                and state.get("assigned_train_id") == train.id
            )
            and self._train_overlaps_station(train, stop)
        )

    def _zone_cleared_by_train(self, train: Train, zone_end_m: float, follower: Train) -> bool:
        return train.safe_rear_end_pos() > zone_end_m + PARALLEL_RELEASE_MARGIN_M

    def _train_waiting_for_station_departure(self, train: Train, station_idx: int) -> bool:
        if not (0 <= station_idx < len(self.scheduled_stops)):
            return False
        if (
            train.active_scheduled_stop is None
            and train.dwell_remaining_s <= 0.0
            and not (
                train.station_lane is not None
                and train.last_station_idx == station_idx
                and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
            )
        ):
            return False
        stop = self.scheduled_stops[station_idx]
        stop_pos = float(stop["pos_m"])
        station_start, station_end = self._station_bounds(stop)
        if not (station_start <= train.pos <= station_end):
            return False
        if train.dwell_remaining_s > 0.0 or train.departure_hold:
            return True
        if train.zero_speed_detected and train.pos >= stop_pos - STOP_ACCURACY_TOL_M:
            return True
        if train.pos > stop_pos + STOP_ACCURACY_TOL_M and train.speed <= kmh_to_ms(5.0):
            return True
        return False

    def _train_still_holding_previous_station_line(self, train: Train, next_station_idx: int | None) -> bool:
        if train.station_lane is None or train.last_station_idx is None:
            return False
        previous_station_idx = train.last_station_idx
        if previous_station_idx == next_station_idx or not (0 <= previous_station_idx < len(self.scheduled_stops)):
            return False
        line = self._station_line_for_lane(previous_station_idx, train.station_lane)
        if line is None:
            return False
        previous_stop = self.scheduled_stops[previous_station_idx]
        return self._train_overlaps_station(train, previous_stop) or not self._station_line_tail_clear(train, line)

    def _arm_parallel_release_lock(self, zone_id: str, lane: int):
        key = (zone_id, int(lane))
        lock_s = SOURCE_RELEASE_LOCK_S if zone_id == "SOURCE" else TURNOUT_LOCK_S
        self.parallel_release_locks[key] = max(float(self.parallel_release_locks.get(key, 0.0)), lock_s)

    def _tick_parallel_release_locks(self):
        expired = []
        for key, remaining_s in self.parallel_release_locks.items():
            next_remaining = max(0.0, remaining_s - DT)
            self.parallel_release_locks[key] = next_remaining
            if next_remaining <= 0.0:
                expired.append(key)
        for key in expired:
            self.parallel_release_locks.pop(key, None)

    def _parallel_departure_holds(self) -> Dict[str, float]:
        holds: Dict[str, float] = {}

        def apply_lane_gate(
            zone_id: str,
            zone_end_m: float,
            trains: List[Train],
            gated_train_ids: set[str] | None = None,
            zone_start_m: float | None = None,
        ):
            lane_map: Dict[int, Train] = {}
            for train in trains:
                lane_map[int(train.protection_lane)] = train
            for lane in sorted(lane_map):
                train = lane_map[lane]
                if gated_train_ids is not None and train.id not in gated_train_ids:
                    continue
                for prior_lane in range(lane):
                    prior_train = lane_map.get(prior_lane)
                    release_lock_active = self.parallel_release_locks.get((zone_id, prior_lane), 0.0) > 0.0
                    prior_train_in_departure_zone = (
                        prior_train is not None
                        and (zone_start_m is None or prior_train.pos >= zone_start_m - STOP_ACCURACY_TOL_M)
                    )
                    prior_train_not_clear = (
                        prior_train_in_departure_zone
                        and prior_train is not None
                        and not self._zone_cleared_by_train(prior_train, zone_end_m, train)
                    )
                    if release_lock_active or prior_train_not_clear:
                        hold_margin = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
                        hold_eoa = train.reported_pos + hold_margin + STOP_SVL_OFFSET_M + PARALLEL_RELEASE_MARGIN_M
                        if isinstance(zone_id, str) and zone_id.startswith("STATION:"):
                            tail_clear_pos = zone_end_m + PARALLEL_RELEASE_MARGIN_M + train.length
                            hold_eoa = max(hold_eoa, tail_clear_pos + STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                        holds[train.id] = hold_eoa
                        break

        source_trains = [train for train in self.trains if train.protection_zone_id == "SOURCE"]
        if source_trains:
            apply_lane_gate("SOURCE", SOURCE_TRAIN_EXIT_M, source_trains)

        station_zone_ids = sorted(
            {
                train.protection_zone_id
                for train in self.trains
                if isinstance(train.protection_zone_id, str) and train.protection_zone_id.startswith("STATION:")
            }
        )
        for zone_id in station_zone_ids:
            try:
                station_idx = int(zone_id.split(":", 1)[1])
            except (IndexError, ValueError):
                continue
            if not (0 <= station_idx < len(self.scheduled_stops)):
                continue
            station_start, station_end = self._station_bounds(self.scheduled_stops[station_idx])
            station_trains = [train for train in self.trains if train.protection_zone_id == zone_id]
            if station_trains:
                gated_train_ids = {
                    train.id
                    for train in station_trains
                    if self._train_waiting_for_station_departure(train, station_idx)
                }
                apply_lane_gate(zone_id, station_end, station_trains, gated_train_ids, station_start)

        return holds

    def _needs_station_departure_authority_hold(self, train: Train, packet: SafeMovementPacket) -> bool:
        if (
            train.station_lane is not None
            and train.last_station_idx is not None
            and 0 <= train.last_station_idx < len(self.scheduled_stops)
            and train.next_scheduled_stop_idx >= len(train.scheduled_stops)
        ):
            _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
            if station_end >= self.track_end_m - STOP_ACCURACY_TOL_M:
                return True
        current_station = self._station_for_train(train)
        if (
            train.station_lane is not None
            and train.last_station_idx is not None
            and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
            and train.speed <= kmh_to_ms(5.0)
        ):
            _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
            if train.pos > station_end + STOP_ACCURACY_TOL_M:
                return False
            authority_ahead_m = packet.eoa_m - train.reported_pos
            return authority_ahead_m < DEPARTURE_RELEASE_MIN_AUTHORITY_M
        if current_station is None or train.active_scheduled_stop is None:
            return False
        _current_idx, current_stop = current_station
        if self._same_station_stop(train.active_scheduled_stop, current_stop):
            return False
        if train.dwell_remaining_s > 0.0:
            return False
        if not (train.zero_speed_detected or train.speed <= kmh_to_ms(5.0)):
            return False
        authority_ahead_m = packet.eoa_m - train.reported_pos
        return authority_ahead_m < DEPARTURE_RELEASE_MIN_AUTHORITY_M

    def _enforce_station_cd_routes(self):
        for station_idx, state in enumerate(self.station_route_states):
            route_lane = state.get("route_lane")
            if route_lane is None:
                continue
            occupied_train = next(
                (
                    train for train in self.trains
                    if self._station_physical_lane(train, station_idx) == int(route_lane)
                    and self._train_overlaps_station(train, self.scheduled_stops[station_idx])
                ),
                None,
            )
            if occupied_train is not None:
                state["route_lane"] = None
                state["assigned_train_id"] = occupied_train.id
                state["locking_train_id"] = occupied_train.id
                state["route_state"] = "LOCKED"
                state["switch_started"] = False
                state["lock_remaining_s"] = 0.0

    def _update_parallel_protection_zones(self):
        previous_station_lanes: Dict[str, Tuple[str, int]] = {}
        for train in self.trains:
            if isinstance(train.protection_zone_id, str) and train.protection_zone_id.startswith("STATION:"):
                previous_station_lanes[train.id] = (train.protection_zone_id, int(train.protection_lane))
        for train in self.trains:
            previous_zone_id = train.protection_zone_id
            previous_lane = int(train.protection_lane)
            train.protection_zone_id = None
            train.protection_lane = 0
            train.departure_hold = False
            if not self._source_zone_contains(train):
                if train.source_lane is not None:
                    self._arm_parallel_release_lock("SOURCE", int(train.source_lane))
                train.source_lane = None
            if train.station_lane is not None:
                routed_station = self._station_for_train(train)
                active_station_start = None
                active_station_end = None
                if train.active_scheduled_stop is not None:
                    active_station_start, active_station_end = self._station_bounds(train.active_scheduled_stop)
                active_station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                active_line_id = (
                    None
                    if active_station_idx is None
                    else self._station_line_id(active_station_idx, int(train.station_lane))
                )
                station_assignment_matches_active_stop = (
                    active_station_idx is not None
                    and train.assigned_station_line_id == active_line_id
                )
                approaching_routed_stop = (
                    train.active_scheduled_stop is not None
                    and train.station_lane is not None
                    and active_station_start is not None
                    and station_assignment_matches_active_stop
                    and active_station_start - STATION_ROUTE_APPROACH_M <= train.pos <= active_station_end
                )
                if routed_station is None and not approaching_routed_stop:
                    if isinstance(previous_zone_id, str) and previous_zone_id.startswith("STATION:"):
                        try:
                            previous_station_idx = int(previous_zone_id.split(":", 1)[1])
                        except (IndexError, ValueError):
                            previous_station_idx = None
                        if previous_station_idx is not None and 0 <= previous_station_idx < len(self.scheduled_stops):
                            line = self._station_line_for_lane(previous_station_idx, train.station_lane)
                            if line is not None and not self._station_line_tail_clear(train, line):
                                train.protection_zone_id = previous_zone_id
                                train.protection_lane = previous_lane
                                line["route_state"] = "RELEASE_PENDING"
                                self._set_train_station_state(train, previous_station_idx, "DEPARTING", "tail_not_clear")
                                self.log_station_event(previous_station_idx, train, line, "TAIL_NOT_CLEAR")
                                continue
                            self._arm_parallel_release_lock(previous_zone_id, previous_lane)
                            if self.release_station_line_only_after_tail_clear(previous_station_idx, train):
                                continue
                    train.station_lane = None
                    train.assigned_platform = None
                    train.assigned_station_id = None
                    train.assigned_station_line_id = None
                    if train.active_scheduled_stop is None:
                        self._set_train_station_state(train, None, "COMPLETED_STOP", "station_lane_released")

        source_trains = [train for train in self.trains if self._source_zone_contains(train)]
        source_trains.sort(key=lambda item: item.pos, reverse=True)
        source_capacity = max(
            (max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS))) for source in self.source_trains),
            default=SOURCE_VISIBLE_ACTIVE_TRAINS,
        )
        used_source_lanes: set[int] = set()
        for train in source_trains:
            lane = int(train.source_lane) if train.source_lane is not None else None
            if lane is None or lane < 0 or lane >= source_capacity or lane in used_source_lanes:
                lane = next((candidate for candidate in range(source_capacity) if candidate not in used_source_lanes), None)
            if lane is None:
                continue
            train.source_lane = lane
            used_source_lanes.add(lane)
            train.protection_zone_id = "SOURCE"
            train.protection_lane = int(train.source_lane)

        station_groups: Dict[int, List[Train]] = {}
        for train in self.trains:
            if self._train_has_station_route_authority(train):
                station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                _station_start, station_end = self._station_bounds(train.active_scheduled_stop)
                if (
                    station_idx is not None
                    and train.pos <= station_end
                    and train.protection_zone_id != "SOURCE"
                ):
                    train.protection_zone_id = f"STATION:{station_idx}"
                    train.protection_lane = int(train.station_lane)
            station = self._station_overlapped_by_train(train)
            if station is None:
                continue
            station_idx, _stop = station
            station_groups.setdefault(station_idx, []).append(train)
        for station_idx, trains in station_groups.items():
            capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
            if capacity <= 1:
                continue
            zone_id = f"STATION:{station_idx}"
            current_stop = self.scheduled_stops[station_idx]
            used_lanes: set[int] = set()
            trains.sort(key=lambda item: item.id)
            for train in trains:
                previous_zone_lane = previous_station_lanes.get(train.id)
                previous_lane = previous_zone_lane[1] if previous_zone_lane is not None and previous_zone_lane[0] == zone_id else None
                if previous_lane is not None and 0 <= previous_lane < capacity and previous_lane not in used_lanes:
                    lane = previous_lane
                else:
                    lane = next((candidate for candidate in range(capacity) if candidate not in used_lanes), None)
                if lane is None:
                    continue
                used_lanes.add(lane)
                active_is_current_station = self._same_station_stop(train.active_scheduled_stop, current_stop)
                if train.station_lane is None and active_is_current_station:
                    train.station_lane = lane
                train.protection_zone_id = zone_id
                train.protection_lane = int(train.station_lane if active_is_current_station and train.station_lane is not None else lane)

        self._enforce_station_cd_routes()

    def _update_train_stop_schedule(self, train: Train) -> bool:
        immediate_packet_required = False
        while train.next_scheduled_stop_idx < len(train.scheduled_stops):
            stop = train.scheduled_stops[train.next_scheduled_stop_idx]
            if stop["pos_m"] < train.pos - STOP_ACCURACY_TOL_M and train.dwell_remaining_s <= 0.0:
                train.next_scheduled_stop_idx += 1
                continue
            break

        if train.dwell_remaining_s > 0.0:
            train.active_scheduled_stop = train.scheduled_stops[max(0, train.next_scheduled_stop_idx - 1)]
            train.commanded_stop = True
            station_idx = self._station_index_for_stop(train.active_scheduled_stop)
            if station_idx is None and train.station_lane is not None:
                station_idx = train.last_station_idx
            self._set_train_station_state(train, station_idx, "DWELLING", "dwell_countdown")
            train.pos = train.standstill_anchor_pos
            train.speed = 0.0
            train.prev_accel = 0.0
            train.reset_non_emergency_stop_latches()
            train.standstill_required = True
            train.standstill_anchor_pos = train.pos
            if train.dwell_remaining_s != float("inf"):
                train.dwell_remaining_s = max(0.0, train.dwell_remaining_s - DT)
            if train.dwell_remaining_s <= 0.0:
                schedule_hold_s = self._station_schedule_departure_hold_s(station_idx, train)
                if schedule_hold_s > 0.0:
                    train.dwell_remaining_s = schedule_hold_s
                    return immediate_packet_required
                if self.headway_manager.mode != "timetable":
                    headway_hold_s = self._station_departure_headway_hold_s(station_idx)
                    if headway_hold_s > 0.0:
                        train.dwell_remaining_s = headway_hold_s
                        return immediate_packet_required
                train.commanded_stop = False
                self._set_train_station_state(train, station_idx, "READY_TO_DEPART", "dwell_complete")
                if station_idx is not None:
                    self._record_station_departure_headway(station_idx, train)
                    self.prepare_departure_route(station_idx, train)
                    self.log_station_event(
                        station_idx,
                        train,
                        self._station_line_for_lane(station_idx, train.station_lane),
                        "DWELL_COMPLETED",
                    )
                train.active_scheduled_stop = None
                train.reset_non_emergency_stop_latches()
                train.standstill_required = False
                immediate_packet_required = True
            return immediate_packet_required

        if train.next_scheduled_stop_idx >= len(train.scheduled_stops):
            train.active_scheduled_stop = None
            train.commanded_stop = False
            return False

        stop = train.scheduled_stops[train.next_scheduled_stop_idx]
        station_idx = self._station_index_for_stop(stop)
        distance_to_stop = stop["pos_m"] - train.pos
        service_decel = equivalent_mass_adjusted_accel(BRAKE_FORCE_N / max(train.mass, 1.0))
        dynamic_stop_activation_m = max(
            STOP_TARGET_MIN_ACTIVATION_M,
            ATO_TARGET_PREP_MAX_M,
            stopping_distance_with_buildup(train.speed, service_decel, BRAKE_BUILDUP_S)
            + train.speed * P_TIME_S
            + STOP_TARGET_BUFFER_M,
        )
        if self._train_still_holding_previous_station_line(train, station_idx):
            train.active_scheduled_stop = None
            train.commanded_stop = False
            if train.station_state not in {"READY_TO_DEPART", "DEPARTING"}:
                self._set_train_station_state(train, train.last_station_idx, "DEPARTING", "between_stations")
            return False
        train.active_scheduled_stop = stop
        train.commanded_stop = 0.0 <= distance_to_stop <= dynamic_stop_activation_m
        if train.station_lane is None and distance_to_stop <= STATION_ROUTE_APPROACH_M:
            self._set_train_station_state(train, station_idx, "APPROACHING_STATION", "scheduled_stop_approach")
        elif train.station_lane is not None and distance_to_stop > 25.0:
            self._set_train_station_state(train, station_idx, "ROUTE_ASSIGNED", "route_assigned")
        elif train.station_lane is not None and distance_to_stop <= 25.0:
            self._set_train_station_state(train, station_idx, "DOCKING", "final_approach")

        aligned_to_scheduled_stop = abs(train.pos - float(stop["pos_m"])) <= STOP_ACCURACY_TOL_M
        if (
            train.commanded_stop
            and train.zero_speed_detected
            and train.door_authorized
            and train.station_lane is not None
            and aligned_to_scheduled_stop
        ):
            # Dwell should start from the same aligned-stop condition that enables
            # doors, otherwise the train can remain held short of the platform or
            # creep past the stop marker before the schedule advances.
            train.pos = train.stop_target_pos
            train.speed = 0.0
            train.prev_accel = 0.0
            train.reset_non_emergency_stop_latches()
            train.standstill_required = True
            train.standstill_anchor_pos = train.pos
            self._set_train_station_state(train, station_idx, "STOPPED_AT_PLATFORM", "aligned_stop")
            train.dwell_remaining_s = self._station_dwell_time_s(station_idx, stop, train)
            self._record_station_arrival(station_idx, train, train.dwell_remaining_s)
            train.next_scheduled_stop_idx += 1
            self._set_train_station_state(train, station_idx, "DWELLING", "dwell_started")
            immediate_packet_required = True
        return False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def _timetable_regulated_speed_cap_kmh(self, train: Train, base_cap_kmh: float) -> Tuple[float, str]:
        if self.headway_manager.mode != "timetable" or train.active_scheduled_stop is None:
            return base_cap_kmh, ""
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return base_cap_kmh, ""
        record = self._schedule_record_for_train_station(train, train.active_scheduled_stop, station_idx)
        if record is None or record.get("arrival_time_s") is None:
            return base_cap_kmh, ""
        distance_m = max(0.0, float(train.active_scheduled_stop["pos_m"]) - train.pos)
        if distance_m <= STOP_ACCURACY_TOL_M:
            return base_cap_kmh, ""
        remaining_s = float(record["arrival_time_s"]) - self._timetable_operational_time_s()
        if remaining_s <= 0.0:
            return base_cap_kmh, "TIMETABLE_LATE_FAST"
        return base_cap_kmh, "TIMETABLE_RUN_MAX_DWELL_BALANCE"

    def _dispatch_safe_packets(self, with_delay: bool):
        self._update_parallel_protection_zones()
        self._update_source_headway_releases()
        stop_eoa_map: Dict[str, float] = {}
        for train in self.trains:
            if train.active_scheduled_stop is not None and self._train_has_station_stop_eoa_authority(train):
                stop_eoa_map[train.id] = self._scheduled_stop_eoa(float(train.active_scheduled_stop["pos_m"]))
            elif train.active_scheduled_stop is not None and train.station_lane is None:
                station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                if station_idx is not None:
                    self.update_station_occupancy(station_idx)
                    can_accept, reject_reason = self.can_accept_train(station_idx, train)
                    station_start, _station_end = self._station_bounds(self.scheduled_stops[station_idx])
                    if not can_accept and train.pos < station_start:
                        hold_eoa = self._station_holding_eoa(station_idx)
                        stop_eoa_map[train.id] = hold_eoa
                        train.station_reject_reason = reject_reason
                        self.log_station_event(
                            station_idx,
                            train,
                            None,
                            reject_reason,
                            old_EOA=train.last_dispatched_eoa,
                            new_EOA=hold_eoa,
                            EOA_update_reason="STATION_FULL",
                            reject_reason=reject_reason,
                        )
                    elif can_accept:
                        train.station_reject_reason = ""
        safe_packets = self.zc.build_safe_packets(
            self.track_profile,
            self.tsr_zones,
            self.track_end_m,
            stop_eoa_map,
        )
        departure_holds = self._parallel_departure_holds()
        for train in self.trains:
            train.dcs_muted = any(
                window["start_s"] <= self.sim_time_s <= window["end_s"]
                for window in train.dcs_mute_windows
            )
            if train.dcs_muted:
                continue
            delay_s = random.uniform(DCS_DELAY_MIN_S, DCS_DELAY_MAX_S) if with_delay else 0.0
            packet = safe_packets[train.id]
            regulated_cap_kmh, regulation_reason = self._timetable_regulated_speed_cap_kmh(train, packet.tsr_kmh)
            if regulation_reason and regulated_cap_kmh < packet.tsr_kmh - 1e-6:
                variants = dict(packet.variants)
                variants["schedule_regulation"] = regulation_reason
                variants["schedule_speed_cap_kmh"] = regulated_cap_kmh
                packet = SafeMovementPacket(
                    eoa_m=packet.eoa_m,
                    tsr_kmh=regulated_cap_kmh,
                    variants=variants,
                    issued_time_s=packet.issued_time_s,
                )
            if train.departure_hold or train.id in departure_holds or self._needs_station_departure_authority_hold(train, packet):
                hold_eoa = departure_holds.get(train.id)
                terminal_station_hold = (
                    train.station_lane is not None
                    and train.last_station_idx is not None
                    and 0 <= train.last_station_idx < len(self.scheduled_stops)
                    and train.next_scheduled_stop_idx >= len(train.scheduled_stops)
                    and self._station_bounds(self.scheduled_stops[train.last_station_idx])[1] >= self.track_end_m - STOP_ACCURACY_TOL_M
                )
                if hold_eoa is None:
                    fixed_block_station_hold = (
                        self.block_mode == "fixed_block"
                        and train.station_lane is not None
                        and train.last_station_idx is not None
                        and 0 <= train.last_station_idx < len(self.scheduled_stops)
                        and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
                    )
                    if fixed_block_station_hold:
                        _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
                        hold_eoa = station_end - STOP_SVL_OFFSET_M
                    else:
                        hold_margin = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
                        hold_eoa = train.reported_pos + hold_margin + STOP_SVL_OFFSET_M + PARALLEL_RELEASE_MARGIN_M
                    if (
                        not fixed_block_station_hold
                        and
                        train.station_lane is not None
                        and train.last_station_idx is not None
                        and 0 <= train.last_station_idx < len(self.scheduled_stops)
                        and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
                    ):
                        _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
                        tail_clear_pos = station_end + PARALLEL_RELEASE_MARGIN_M + train.length
                        hold_eoa = max(hold_eoa, tail_clear_pos + STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                packet = SafeMovementPacket(
                    eoa_m=hold_eoa,
                    tsr_kmh=0.0 if train.protection_zone_id == "SOURCE" or terminal_station_hold else packet.tsr_kmh,
                    variants=dict(packet.variants),
                    issued_time_s=packet.issued_time_s,
                )
                train.departure_hold = True
            station_idx = self._station_index_for_stop(train.active_scheduled_stop)
            if station_idx is None and train.station_lane is not None:
                station_idx = train.last_station_idx
            reason = "LEADER_PROTECTION"
            station_stop_eoa = stop_eoa_map.get(train.id)
            if train.trip_mode or train.emergency_stop or train.emg_latch:
                reason = "EMERGENCY"
            elif train.dcs_muted:
                reason = "DCS_TIMEOUT"
            elif train.departure_hold:
                reason = "SAFETY_RESTRICTION"
            elif (
                train.station_reject_reason
                in {"STATION_FULL", "NO_FREE_PLATFORM", "ALL_LINES_OCCUPIED", "ROUTE_CONFLICT", "STATION_LINE_OCCUPIED"}
                and (station_stop_eoa is None or packet.eoa_m >= station_stop_eoa - 1e-6)
            ):
                reason = "SAFETY_RESTRICTION"
            elif self.block_mode == "fixed_block" and station_stop_eoa is None:
                reason = "FIXED_BLOCK"
            elif station_stop_eoa is not None:
                if packet.eoa_m < station_stop_eoa - 1e-6:
                    reason = "FIXED_BLOCK" if self.block_mode == "fixed_block" else "LEADER_PROTECTION"
                elif train.station_state in {"ROUTE_ASSIGNED", "APPROACHING_STATION"}:
                    reason = "ROUTE_ASSIGNED"
                else:
                    reason = "STATION_STOP_TARGET"
            elif train.last_dispatched_eoa_reason in {"ROUTE_ASSIGNED", "STATION_STOP_TARGET"} and train.active_scheduled_stop is not None:
                reason = "ROUTE_INVALIDATED"
            elif train.last_station_state_reason == "dwell_complete" and (
                train.last_dispatched_eoa is None or packet.eoa_m > train.last_dispatched_eoa + 1e-6
            ):
                reason = "DEPARTURE_RELEASE"
            self._log_eoa_update(train, train.last_dispatched_eoa, packet.eoa_m, reason, station_idx)
            packet.issued_time_s = self.sim_time_s
            train.receive_safe_packet(packet, self.sim_time_s + delay_s)

    def _update_analytics(self, include_station_metrics: bool = False):
        headway_snapshot = self.headway_manager.snapshot()
        self.analytics.update(headway_snapshot)
        if headway_snapshot.get("actual_headways_s"):
            self.analytics["min_headway_s"] = headway_snapshot.get("min_actual_headway_s")
            avg_headway = headway_snapshot.get("avg_actual_headway_s")
            self.analytics["trains_per_hour"] = 3600.0 / avg_headway if avg_headway else 0.0
        release_times = headway_snapshot.get("release_times_s", {})
        if release_times:
            self.analytics["current_open_headway_s"] = max(0.0, self.sim_time_s - max(release_times.values()))
        self.analytics["traction_work_kwh"] = sum(t.analytics_traction_work_j for t in self.trains) / 3_600_000.0
        self.analytics["brake_work_kwh"] = sum(t.analytics_brake_work_j for t in self.trains) / 3_600_000.0
        station_metric_indices = sorted(
            set(self.station_arrival_headway_actual_s)
            | set(self.station_headway_actual_s)
            | set(self.analytics.get("station_arrivals", {}))
        )
        station_metrics = []
        for idx in station_metric_indices:
            arrival_headways = list(self.station_arrival_headway_actual_s.get(idx, []))
            departure_headways = list(self.station_headway_actual_s.get(idx, []))
            arrivals = [dict(record) for record in self.analytics.get("station_arrivals", {}).get(idx, [])]
            avg_arrival_headway = (
                sum(arrival_headways) / len(arrival_headways)
                if arrival_headways
                else None
            )
            avg_departure_headway = (
                sum(departure_headways) / len(departure_headways)
                if departure_headways
                else None
            )
            for record in arrivals:
                record["avg_station_arrival_headway_s"] = avg_arrival_headway
            station_metrics.append(
                {
                    "station_index": idx,
                    "station_name": self.scheduled_stops[idx].get("name", f"STATION_{idx}")
                    if idx < len(self.scheduled_stops)
                    else f"STATION_{idx}",
                    "arrival_headways_s": arrival_headways,
                    "departure_headways_s": departure_headways,
                    "avg_arrival_headway_s": avg_arrival_headway,
                    "avg_departure_headway_s": avg_departure_headway,
                    "headway_deviation_s": list(self.station_headway_deviation_s.get(idx, [])),
                    "arrivals": arrivals,
                }
            )
        self.analytics["station_passenger_metrics"] = station_metrics
        ordered_for_collision = sorted(self.trains, key=lambda item: item.pos)
        active_collisions = 0
        for left, right in zip(ordered_for_collision, ordered_for_collision[1:]):
            if left.source_lane is not None and right.source_lane is not None and left.source_lane != right.source_lane:
                continue
            if (
                left.protection_zone_id == right.protection_zone_id
                and left.protection_lane != right.protection_lane
            ):
                continue
            overlap = left.pos - right.safe_rear_end_pos()
            if overlap > 0.0:
                active_collisions += 1
                event = {
                    "time_s": self.sim_time_s,
                    "front_train_id": right.id,
                    "following_train_id": left.id,
                    "overlap_m": overlap,
                }
                if not left.collision_latched or not right.collision_latched:
                    self.analytics["collision_count"] = self.analytics.get("collision_count", 0) + 1
                    self.analytics.setdefault("collision_events", []).append(event)
                for train, partner in ((left, right), (right, left)):
                    train.collision_latched = True
                    train.collision_partner_id = partner.id
                    train.collision_overlap_m = overlap
                    train.enter_trip_mode("COLLISION", train.reported_pos)
                    train.atp_state = "ATP_TRIP"
                    train.atp_alert = "COLLISION"
                    train.atp_action = "EBI"
                    train.atp_brake = "EMERGENCY"
                    train.emg_latch = True
        self.analytics["active_collision_count"] = active_collisions
        for train in self.trains:
            if train.headway_time_s is not None and train.headway_time_s > 0.0:
                current_min = self.analytics["min_headway_s"]
                if current_min is None or train.headway_time_s < current_min:
                    self.analytics["min_headway_s"] = train.headway_time_s

            journey_times = self.analytics["journey_times"]
            if train.id not in journey_times and train.pos >= self.track_end_m:
                journey_times[train.id] = self.sim_time_s

            previous_action = self.analytics["last_actions"].get(train.id, "")
            if train.atp_action == "EBI" and previous_action != "EBI":
                self.analytics["ebi_count"] += 1
            if train.atp_action == "SBI" and previous_action != "SBI":
                self.analytics["sbi_count"] += 1
            self.analytics["last_actions"][train.id] = train.atp_action

    def step(self):
        self._tick_parallel_release_locks()
        self.train_generation_changed = self._spawn_source_trains()
        immediate_packet_required = False
        for t in self.trains:
            t.update_reported_position()
            immediate_packet_required = self._update_train_stop_schedule(t) or immediate_packet_required
        immediate_packet_required = self._update_station_routes() or immediate_packet_required
        self._check_station_invariants()
        self._dispatch_safe_packets(with_delay=not immediate_packet_required)
        ordered = sorted(self.trains, key=lambda t: t.pos, reverse=True)
        for i, t in enumerate(ordered):
            front = min(
                (
                    candidate for candidate in ordered
                    if candidate.pos > t.pos
                    and (
                        t.protection_zone_id is None
                        or candidate.protection_zone_id is None
                        or t.protection_zone_id != candidate.protection_zone_id
                        or t.protection_lane == candidate.protection_lane
                    )
                ),
                key=lambda candidate: candidate.pos,
                default=None,
            )
            if front is None:
                t.headway_time_s = None
                continue
            gap = (front.pos - front.length) - t.pos
            t.headway_time_s = gap / max(t.speed, 0.1)
        for t in self.trains:
            t.step(self.sim_time_s)
            if (
                getattr(t, "headway_dispatch_released", False)
                and not getattr(t, "headway_actual_dispatched", False)
                and not self._source_zone_contains(t)
            ):
                self.headway_manager.mark_actual_dispatch(t.id, self.sim_time_s)
                t.headway_actual_dispatched = True
            if t.active_scheduled_stop is None and t.station_lane is not None and t.speed > STANDSTILL_SPEED_EPS:
                self._set_train_station_state(t, self._station_index_for_stop(self._station_overlapped_by_train(t)[1]) if self._station_overlapped_by_train(t) is not None else t.last_station_idx, "DEPARTING", "departing_from_station")
        self._enforce_station_cd_routes()
        self._update_analytics()
        self.sim_time_s += DT



__all__ = ["Simulation"]
