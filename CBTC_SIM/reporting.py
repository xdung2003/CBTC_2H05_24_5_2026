from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List


PROJECT_ROOT = Path(__file__).resolve().parent.parent
REPORTS_DIR = PROJECT_ROOT / "reports"
REPORT_SCHEMA_VERSION = 1
REPORT_TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"


def generated_at_timestamp() -> str:
    return datetime.now().isoformat(timespec="seconds")


def report_file_timestamp() -> str:
    return datetime.now().strftime(REPORT_TIMESTAMP_FORMAT)


def safe_report_slug(name: Any) -> str:
    scenario_name = str(name or "scenario").strip().lower()
    return "".join(ch if ch.isalnum() else "_" for ch in scenario_name).strip("_") or "scenario"


def runtime_dynamics_label(sim) -> str:
    if not sim.trains:
        return "LEGACY_FALLBACK"
    first_train = sim.trains[0]
    if getattr(first_train, "runtime_used_legacy_fallback", False):
        return "LEGACY_FALLBACK"
    return "FORCE_BALANCE"


def build_simulation_report(sim) -> Dict[str, Any]:
    """Build a local JSON-serializable report from the current simulation state."""
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "generated_at": generated_at_timestamp(),
        "scenario": {
            "name": sim.scenario.get("name", "Scenario"),
            "source_path": sim.scenario.get("source_path"),
            "track_min_m": sim.track_min_m,
            "track_max_m": sim.track_max_m,
            "track_end_m": sim.track_end_m,
        },
        "simulation": {
            "time_s": sim.sim_time_s,
            "train_count": len(sim.trains),
            "tsr_count": len(sim.tsr_zones),
            "block_mode": getattr(sim, "block_mode", "moving_block"),
            "debug_trace_rows": len(getattr(sim, "debug_trace_records", [])),
            "runtime_dynamics": runtime_dynamics_label(sim),
        },
        "analytics": {
            "min_headway_s": sim.analytics.get("min_headway_s"),
            "target_headway_s": sim.analytics.get("target_headway_s"),
            "actual_headways_s": list(sim.analytics.get("actual_headways_s", [])),
            "actual_headway_pairs": [dict(pair) for pair in sim.analytics.get("actual_headway_pairs", [])],
            "current_open_headway_s": sim.analytics.get("current_open_headway_s"),
            "min_actual_headway_s": sim.analytics.get("min_actual_headway_s"),
            "avg_actual_headway_s": sim.analytics.get("avg_actual_headway_s"),
            "max_actual_headway_s": sim.analytics.get("max_actual_headway_s"),
            "headway_deviation_s": list(sim.analytics.get("headway_deviation_s", [])),
            "avg_abs_headway_deviation_s": sim.analytics.get("avg_abs_headway_deviation_s"),
            "dispatch_delay_s": list(sim.analytics.get("dispatch_delay_s", [])),
            "avg_dispatch_delay_s": sim.analytics.get("avg_dispatch_delay_s"),
            "station_passenger_metrics": [
                {
                    **dict(station),
                    "arrivals": [dict(record) for record in station.get("arrivals", [])],
                }
                for station in sim.analytics.get("station_passenger_metrics", [])
            ],
            "trains_per_hour": sim.analytics.get("trains_per_hour", 0.0),
            "traction_work_kwh": sim.analytics.get("traction_work_kwh", 0.0),
            "brake_work_kwh": sim.analytics.get("brake_work_kwh", 0.0),
            "coast_distance_m": sim.analytics.get("coast_distance_m", 0.0),
            "total_distance_m": sim.analytics.get("total_distance_m", 0.0),
            "ebi_count": sim.analytics.get("ebi_count", 0),
            "sbi_count": sim.analytics.get("sbi_count", 0),
            "collision_count": sim.analytics.get("collision_count", 0),
            "active_collision_count": sim.analytics.get("active_collision_count", 0),
            "collision_events": [dict(event) for event in sim.analytics.get("collision_events", [])],
            "journey_times": dict(sim.analytics.get("journey_times", {})),
        },
        "trains": [
            {
                "id": train.id,
                "drive_mode": train.drive_mode,
                "pos_m": train.pos,
                "reported_pos_m": train.reported_pos,
                "speed_kmh": train.speed * 3.6,
                "eoa_m": train.eoa,
                "svl_m": getattr(train, "stop_target_pos", train.eoa),
                "target_distance_m": max(0.0, getattr(train, "stop_target_pos", train.eoa) - train.reported_pos),
                "distance_to_eoa_m": getattr(train, "distance_to_eoa", None),
                "distance_to_train_ahead_m": getattr(train, "distance_to_train_ahead_m", None),
                "collision_latched": bool(getattr(train, "collision_latched", False)),
                "collision_partner_id": getattr(train, "collision_partner_id", ""),
                "collision_overlap_m": getattr(train, "collision_overlap_m", 0.0),
                "psr_kmh": train.psr_kmh,
                "headway_s": train.headway_time_s,
                "headway_dispatch_released": bool(getattr(train, "headway_dispatch_released", False)),
                "headway_actual_dispatched": bool(getattr(train, "headway_actual_dispatched", False)),
                "headway_hold_reason": getattr(train, "headway_hold_reason", ""),
                "headway_target_s": getattr(train, "headway_target_s", 0.0),
                "headway_planned_dispatch_s": getattr(train, "headway_planned_dispatch_s", None),
                "headway_dispatch_delay_s": getattr(train, "headway_dispatch_delay_s", 0.0),
                "atp_state": train.atp_state,
                "atp_action": train.atp_action,
                "atp_alert": train.atp_alert,
                "ato_state": getattr(train, "ato_state", ""),
                "ato_profile_phase": getattr(train, "ato_profile_phase", ""),
                "ato_traction_command": getattr(train, "ato_traction_command", 0.0),
                "ato_brake_command": getattr(train, "ato_brake_command", 0.0),
                "ato_brake_mode": getattr(train, "ato_brake_mode", "none"),
                "ato_door_mode": getattr(train, "ato_door_mode", "LOCKED"),
                "door_authorized": bool(getattr(train, "door_authorized", False)),
                "door_open_allowed": bool(getattr(train, "door_open_allowed", False)),
                "precise_stop_state": getattr(train, "precise_stop_state", ""),
                "commanded_stop": bool(getattr(train, "commanded_stop", False)),
                "zero_speed_detected": bool(getattr(train, "zero_speed_detected", False)),
                "train_integrity_confirmed": bool(getattr(train, "train_integrity_confirmed", True)),
                "odometry_error_m": getattr(train, "pos_error_m", 0.0),
                "safe_packet_valid": train.safe_packet_valid,
                "safe_packet_age_s": train.safe_packet_age_s,
                "dcs_muted": train.dcs_muted,
                "analytics": {
                    "distance_m": getattr(train, "analytics_distance_m", 0.0),
                    "traction_work_kwh": getattr(train, "analytics_traction_work_j", 0.0) / 3_600_000.0,
                    "brake_work_kwh": getattr(train, "analytics_brake_work_j", 0.0) / 3_600_000.0,
                    "coast_distance_m": getattr(train, "analytics_coast_distance_m", 0.0),
                    "runtime_accel_ms2": getattr(train, "runtime_accel_ms2", 0.0),
                    "runtime_traction_force_n": getattr(train, "runtime_traction_force_n", 0.0),
                    "runtime_brake_force_n": getattr(train, "runtime_brake_force_n", 0.0),
                    "runtime_resistance_force_n": getattr(train, "runtime_resistance_force_n", 0.0),
                    "runtime_grade_force_n": getattr(train, "runtime_grade_force_n", 0.0),
                    "runtime_used_legacy_fallback": bool(getattr(train, "runtime_used_legacy_fallback", False)),
                },
                "curves_kmh": {
                    "P": train.curves["P"] * 3.6,
                    "W": train.curves["W"] * 3.6,
                    "SBD": train.curves["SBD"] * 3.6,
                    "EBD": train.curves["EBD"] * 3.6,
                    "SBI": train.hidden_curves["SBI"] * 3.6,
                    "EBI": train.hidden_curves["EBI"] * 3.6,
                },
            }
            for train in sim.trains
        ],
        "track_segments": [
            {
                "start_m": start,
                "end_m": end,
                "gradient": gradient,
                "psr_kmh": psr,
            }
            for start, end, gradient, psr in sim.track_profile
        ],
        "tsr_zones": [dict(zone) for zone in sim.tsr_zones],
    }


def save_simulation_report(sim, reports_dir: Path = REPORTS_DIR) -> Path:
    reports_dir.mkdir(parents=True, exist_ok=True)
    safe_name = safe_report_slug(sim.scenario.get("name", "scenario"))
    timestamp = report_file_timestamp()
    path = reports_dir / f"{timestamp}_{safe_name}.json"
    report = build_simulation_report(sim)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    save_simulation_csv_report(sim, path.with_suffix(".trains.csv"))
    save_simulation_event_log_csv(sim, path.with_suffix(".events.csv"))
    save_atp_debug_trace_csv(sim, path.with_suffix(".atp_trace.csv"))
    return path


def save_simulation_csv_report(sim, path: str | Path) -> Path:
    csv_path = Path(path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "train_id",
        "drive_mode",
        "pos_m",
        "speed_kmh",
        "distance_to_eoa_m",
        "distance_to_train_ahead_m",
        "headway_s",
        "atp_action",
        "ato_profile_phase",
        "traction_work_kwh",
        "brake_work_kwh",
        "coast_distance_m",
        "distance_m",
        "runtime_accel_ms2",
        "runtime_traction_force_n",
        "runtime_brake_force_n",
        "sbi_count",
        "ebi_count",
        "collision_count",
        "collision_latched",
        "collision_partner_id",
        "collision_overlap_m",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for train in sim.trains:
            writer.writerow(
                {
                    "train_id": train.id,
                    "drive_mode": train.drive_mode,
                    "pos_m": train.pos,
                    "speed_kmh": train.speed * 3.6,
                    "distance_to_eoa_m": getattr(train, "distance_to_eoa", None),
                    "distance_to_train_ahead_m": getattr(train, "distance_to_train_ahead_m", None),
                    "headway_s": train.headway_time_s,
                    "atp_action": train.atp_action,
                    "ato_profile_phase": getattr(train, "ato_profile_phase", ""),
                    "traction_work_kwh": getattr(train, "analytics_traction_work_j", 0.0) / 3_600_000.0,
                    "brake_work_kwh": getattr(train, "analytics_brake_work_j", 0.0) / 3_600_000.0,
                    "coast_distance_m": getattr(train, "analytics_coast_distance_m", 0.0),
                    "distance_m": getattr(train, "analytics_distance_m", 0.0),
                    "runtime_accel_ms2": getattr(train, "runtime_accel_ms2", 0.0),
                    "runtime_traction_force_n": getattr(train, "runtime_traction_force_n", 0.0),
                    "runtime_brake_force_n": getattr(train, "runtime_brake_force_n", 0.0),
                    "sbi_count": sim.analytics.get("sbi_count", 0),
                    "ebi_count": sim.analytics.get("ebi_count", 0),
                    "collision_count": sim.analytics.get("collision_count", 0),
                    "collision_latched": bool(getattr(train, "collision_latched", False)),
                    "collision_partner_id": getattr(train, "collision_partner_id", ""),
                    "collision_overlap_m": getattr(train, "collision_overlap_m", 0.0),
                }
            )
    return csv_path


def save_simulation_event_log_csv(sim, path: str | Path) -> Path:
    csv_path = Path(path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "sim_time",
        "train_id",
        "event",
        "reason",
        "pos_m",
        "speed_kmh",
        "distance_to_eoa_m",
        "collision_partner_id",
        "collision_overlap_m",
        "collision_gap_m",
        "atp_state",
        "ato_state",
        "ato_profile_phase",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for train in sim.trains:
            for record in getattr(train, "event_records", []):
                writer.writerow(
                    {
                        "sim_time": record.get("sim_time"),
                        "train_id": record.get("train_id"),
                        "event": record.get("event"),
                        "reason": record.get("reason"),
                        "pos_m": record.get("current_pos"),
                        "speed_kmh": record.get("speed_kmh"),
                        "distance_to_eoa_m": record.get("distance_to_eoa"),
                        "collision_partner_id": record.get("collision_partner_id"),
                        "collision_overlap_m": record.get("collision_overlap_m"),
                        "collision_gap_m": record.get("collision_gap_m"),
                        "atp_state": record.get("atp_state"),
                        "ato_state": record.get("ato_state"),
                        "ato_profile_phase": record.get("ato_profile_phase"),
                    }
                )
    return csv_path


def save_atp_debug_trace_csv(sim, path: str | Path) -> Path:
    csv_path = Path(path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "sim_time_s",
        "train_id",
        "block_mode",
        "fixed_block_id",
        "fixed_block_occupants",
        "pos_m",
        "reported_pos_m",
        "safe_front_m",
        "safe_rear_m",
        "position_error_m",
        "position_uncertainty_m",
        "speed_kmh",
        "vital_speed_kmh",
        "eoa_m",
        "svl_m",
        "stop_target_m",
        "distance_to_eoa_m",
        "distance_to_train_ahead_m",
        "constraint_type",
        "constraint_target_speed_kmh",
        "distance_to_constraint_m",
        "psr_kmh",
        "next_limit_kmh",
        "next_limit_dist_m",
        "gradient",
        "curve_mode",
        "p_kmh",
        "w_kmh",
        "sbi_kmh",
        "sbd_kmh",
        "ebi_kmh",
        "ebd_kmh",
        "raw_p_kmh",
        "raw_w_kmh",
        "raw_sbi_kmh",
        "raw_ebi_kmh",
        "over_p_kmh",
        "over_w_kmh",
        "over_sbi_kmh",
        "over_ebi_kmh",
        "over_ebd_kmh",
        "atp_state",
        "atp_action",
        "atp_alert",
        "atp_brake",
        "service_brake_latch",
        "emergency_brake_latch",
        "trip_mode",
        "last_eoa_reason",
        "safe_packet_valid",
        "safe_packet_age_s",
        "dcs_muted",
        "drive_mode",
        "ato_state",
        "ato_profile_phase",
        "ato_target_speed_kmh",
        "ato_traction_command",
        "ato_brake_command",
        "ato_brake_mode",
        "runtime_accel_ms2",
        "runtime_traction_force_n",
        "runtime_brake_force_n",
        "station_state",
        "station_lane",
        "station_reject_reason",
        "active_stop",
        "commanded_stop",
        "door_authorized",
        "dwell_remaining_s",
        "departure_hold",
        "faults",
        "collision_latched",
        "collision_partner_id",
        "collision_overlap_m",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for record in getattr(sim, "debug_trace_records", []):
            writer.writerow(record)
    return csv_path


def list_reports(reports_dir: Path = REPORTS_DIR) -> List[Dict[str, Any]]:
    reports_dir.mkdir(parents=True, exist_ok=True)
    items: List[Dict[str, Any]] = []
    for path in sorted(reports_dir.glob("*.json"), reverse=True):
        items.append(
            {
                "name": path.name,
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "modified_at": datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds"),
            }
        )
    return items


def load_report(path: str | Path) -> Dict[str, Any]:
    report_path = Path(path)
    if not report_path.is_absolute():
        report_path = REPORTS_DIR / report_path
    return json.loads(report_path.read_text(encoding="utf-8"))
