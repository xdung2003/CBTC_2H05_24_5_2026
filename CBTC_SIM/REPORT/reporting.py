from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List


REPORTS_DIR = Path(__file__).resolve().parent
REPORT_SCHEMA_VERSION = 2
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


def round_value(value: Any, digits: int = 2) -> Any:
    if value is None:
        return None
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return value


def train_status_label(train) -> str:
    if getattr(train, "collision_latched", False):
        return "COLLISION"
    if getattr(train, "trip_mode", False) or getattr(train, "emergency_stop", False) or getattr(train, "emg_latch", False):
        return "EMERGENCY"
    if getattr(train, "door_authorized", False):
        return "DOOR_READY"
    if getattr(train, "dwell_remaining_s", 0.0) > 0.0:
        return "DWELL"
    if getattr(train, "zero_speed_detected", False):
        return "STOPPED"
    return "RUNNING"


def station_call_wait_s(record: Dict[str, Any]) -> Any:
    arrival_time_s = record.get("arrival_time_s")
    departure_time_s = record.get("departure_time_s")
    if arrival_time_s is not None and departure_time_s is not None:
        return max(0.0, float(departure_time_s) - float(arrival_time_s))
    return record.get("actual_station_wait_s", record.get("station_wait_s", record.get("planned_dwell_s")))


def aggregate_numeric(values: List[Any]) -> float:
    total = 0.0
    for value in values:
        try:
            total += float(value)
        except (TypeError, ValueError):
            continue
    return total


def build_simulation_report(sim) -> Dict[str, Any]:
    """Build a compact, human-readable report from the current simulation state."""
    min_headway_s = sim.analytics.get("min_actual_headway_s")
    if min_headway_s is None:
        min_headway_s = sim.analytics.get("min_headway_s")
    total_distance_m = sim.analytics.get("total_distance_m")
    if not total_distance_m:
        total_distance_m = aggregate_numeric([getattr(train, "analytics_distance_m", 0.0) for train in sim.trains])
    coast_values = [
        getattr(train, "analytics_coast_distance_m")
        for train in sim.trains
        if hasattr(train, "analytics_coast_distance_m")
    ]
    coast_distance_m = sim.analytics.get("coast_distance_m")
    if coast_values:
        coast_distance_m = aggregate_numeric(coast_values)
    station_metrics = []
    station_calls = []
    for station in sim.analytics.get("station_passenger_metrics", []):
        station_name = station.get("station_name") or station.get("name")
        arrivals = station.get("arrivals", [])
        wait_values = [
            float(wait_s)
            for wait_s in (station_call_wait_s(record) for record in arrivals)
            if wait_s is not None
        ]
        station_metrics.append(
            {
                "name": station_name,
                "arrivals": len(arrivals),
                "avg_wait_s": round_value(
                    station.get("avg_wait_s")
                    if station.get("avg_wait_s") is not None
                    else (sum(wait_values) / len(wait_values) if wait_values else None)
                ),
                "max_wait_s": round_value(
                    station.get("max_wait_s")
                    if station.get("max_wait_s") is not None
                    else (max(wait_values) if wait_values else None)
                ),
                "passengers_boarded": round_value(station.get("passengers_boarded"), 0),
            }
        )
        for record in arrivals:
            station_calls.append(
                {
                    "station": station_name,
                    "train_id": record.get("train_id"),
                    "arrival_time_s": round_value(record.get("arrival_time_s")),
                    "station_wait_s": round_value(station_call_wait_s(record)),
                    "departure_time_s": round_value(record.get("departure_time_s")),
                    "scheduled_arrival_time_s": round_value(record.get("scheduled_arrival_time_s")),
                    "schedule_variance_s": round_value(record.get("schedule_variance_s")),
                }
            )

    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "generated_at": generated_at_timestamp(),
        "scenario": {
            "name": sim.scenario.get("name", "Scenario"),
            "duration_s": round_value(sim.sim_time_s),
            "track_length_m": round_value(sim.track_end_m, 1),
            "block_mode": getattr(sim, "block_mode", "moving_block"),
        },
        "summary": {
            "train_count": len(sim.trains),
            "running_trains": sum(1 for train in sim.trains if train_status_label(train) == "RUNNING"),
            "stopped_trains": sum(1 for train in sim.trains if train_status_label(train) in ("STOPPED", "DWELL", "DOOR_READY")),
            "emergency_trains": sum(1 for train in sim.trains if train_status_label(train) == "EMERGENCY"),
            "tsr_count": len(sim.tsr_zones),
        },
        "analytics": {
            "target_headway_s": round_value(sim.analytics.get("target_headway_s")),
            "min_headway_s": round_value(min_headway_s),
            "avg_headway_s": round_value(sim.analytics.get("avg_actual_headway_s")),
            "max_headway_s": round_value(sim.analytics.get("max_actual_headway_s")),
            "trains_per_hour": round_value(sim.analytics.get("trains_per_hour", 0.0)),
            "avg_dispatch_delay_s": round_value(sim.analytics.get("avg_dispatch_delay_s")),
            "total_distance_m": round_value(total_distance_m, 1),
            "traction_work_kwh": round_value(sim.analytics.get("traction_work_kwh", 0.0), 3),
            "brake_work_kwh": round_value(sim.analytics.get("brake_work_kwh", 0.0), 3),
            "coast_distance_m": round_value(coast_distance_m, 1),
        },
        "safety": {
            "sbi_count": sim.analytics.get("sbi_count", 0),
            "ebi_count": sim.analytics.get("ebi_count", 0),
            "collision_count": sim.analytics.get("collision_count", 0),
            "active_collision_count": sim.analytics.get("active_collision_count", 0),
        },
        "trains": [
            {
                "id": train.id,
                "status": train_status_label(train),
                "drive_mode": train.drive_mode,
                "position_m": round_value(train.pos, 1),
                "speed_kmh": round_value(train.speed * 3.6, 1),
                "target_distance_m": round_value(max(0.0, getattr(train, "stop_target_pos", train.eoa) - train.reported_pos), 1),
                "headway_s": round_value(train.headway_time_s),
                "psr_kmh": round_value(train.psr_kmh, 1),
                "atp_state": train.atp_state,
                "atp_action": train.atp_action,
                "atp_alert": train.atp_alert,
                "ato_state": getattr(train, "ato_state", ""),
                "door_authorized": bool(getattr(train, "door_authorized", False)),
                "dwell_remaining_s": round_value(getattr(train, "dwell_remaining_s", 0.0)),
                "distance_m": round_value(getattr(train, "analytics_distance_m", 0.0), 1),
                "traction_work_kwh": round_value(getattr(train, "analytics_traction_work_j", 0.0) / 3_600_000.0, 3),
                "brake_work_kwh": round_value(getattr(train, "analytics_brake_work_j", 0.0) / 3_600_000.0, 3),
            }
            for train in sim.trains
        ],
        "stations": station_metrics,
        "station_calls": sorted(
            station_calls,
            key=lambda item: (
                float(item["arrival_time_s"]) if item.get("arrival_time_s") is not None else float("inf"),
                str(item.get("train_id") or ""),
                str(item.get("station") or ""),
            ),
        ),
        "speed_limits": [
            {
                "start_m": round_value(start, 1),
                "end_m": round_value(end, 1),
                "psr_kmh": round_value(psr, 1),
                "gradient": round_value(gradient, 4),
            }
            for start, end, gradient, psr in sim.track_profile
        ],
        "temporary_speed_restrictions": [
            {
                "start_m": round_value(zone.get("start"), 1),
                "end_m": round_value(zone.get("end"), 1),
                "speed_kmh": round_value(zone.get("speed"), 1),
            }
            for zone in sim.tsr_zones
        ],
    }


def md_value(value: Any) -> str:
    if value is None:
        return "--"
    if isinstance(value, bool):
        return "YES" if value else "NO"
    return str(value)


def md_time_s(value: Any) -> str:
    if value is None:
        return "--"
    try:
        total_s = int(round(float(value)))
    except (TypeError, ValueError):
        return md_value(value)
    hours = total_s // 3600
    minutes = (total_s % 3600) // 60
    seconds = total_s % 60
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def dispatch_delay_label(value: Any) -> str:
    if value is None:
        return "--"
    try:
        delay_s = float(value)
    except (TypeError, ValueError):
        return md_value(value)
    if abs(delay_s) < 1e-9:
        return "0.0 (khong tre)"
    return f"{delay_s:.2f}"


def md_table(headers: List[str], rows: List[List[Any]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(md_value(value) for value in row) + " |")
    return "\n".join(lines)


def build_simulation_markdown_report(sim) -> str:
    report = build_simulation_report(sim)
    scenario = report["scenario"]
    summary = report["summary"]
    analytics = report["analytics"]
    safety = report["safety"]

    sections = [
        f"# Bao cao mo phong CBTC - {scenario['name']}",
        "",
        f"- Thoi gian xuat: {report['generated_at']}",
        f"- Thoi luong mo phong: {scenario['duration_s']} s",
        f"- Chieu dai tuyen: {scenario['track_length_m']} m",
        f"- Che do block: {scenario['block_mode']}",
        "",
        "## Tong quan",
        "",
        md_table(
            ["Chi so", "Gia tri"],
            [
                ["So tau", summary["train_count"]],
                ["Dang chay", summary["running_trains"]],
                ["Dang dung / dwell / door ready", summary["stopped_trains"]],
                ["Khan cap", summary["emergency_trains"]],
                ["So TSR", summary["tsr_count"]],
            ],
        ),
        "",
        "## KPI van hanh",
        "",
        md_table(
            ["KPI", "Gia tri"],
            [
                ["Headway muc tieu (s)", analytics["target_headway_s"]],
                ["Headway nho nhat (s)", analytics["min_headway_s"]],
                ["Headway trung binh (s)", analytics["avg_headway_s"]],
                ["Headway lon nhat (s)", analytics["max_headway_s"]],
                ["Tau/gio", analytics["trains_per_hour"]],
                ["Tre dispatch TB (s)", dispatch_delay_label(analytics["avg_dispatch_delay_s"])],
                ["Tong quang duong (m)", analytics["total_distance_m"]],
                ["Dien keo (kWh)", analytics["traction_work_kwh"]],
                ["Dien phanh (kWh)", analytics["brake_work_kwh"]],
                ["Quang duong coast (m)", analytics["coast_distance_m"]],
            ],
        ),
        "",
        "## An toan",
        "",
        md_table(
            ["Chi so", "Gia tri"],
            [
                ["SBI", safety["sbi_count"]],
                ["EBI", safety["ebi_count"]],
                ["Va cham", safety["collision_count"]],
                ["Va cham dang active", safety["active_collision_count"]],
            ],
        ),
        "",
        "## Trang thai tung tau",
        "",
        md_table(
            [
                "Tau",
                "Trang thai",
                "Mode",
                "Vi tri (m)",
                "Toc do (km/h)",
                "Target (m)",
                "Headway (s)",
                "ATP",
                "ATO",
                "Cua",
                "Dwell (s)",
                "Quang duong (m)",
                "Keo (kWh)",
                "Phanh (kWh)",
            ],
            [
                [
                    train["id"],
                    train["status"],
                    train["drive_mode"],
                    train["position_m"],
                    train["speed_kmh"],
                    train["target_distance_m"],
                    train["headway_s"],
                    f"{train['atp_action']} / {train['atp_alert']}",
                    train["ato_state"],
                    train["door_authorized"],
                    train["dwell_remaining_s"],
                    train["distance_m"],
                    train["traction_work_kwh"],
                    train["brake_work_kwh"],
                ]
                for train in report["trains"]
            ],
        ),
        "",
        "## Thong ke ga",
        "",
        md_table(
            ["Ga", "Luot den", "Wait TB (s)", "Wait max (s)", "Khach len"],
            [
                [
                    station["name"],
                    station["arrivals"],
                    station["avg_wait_s"],
                    station["max_wait_s"],
                    station["passengers_boarded"],
                ]
                for station in report["stations"]
            ],
        ) if report["stations"] else "_Chua co du lieu ga._",
        "",
        "## Lich tau tai ga",
        "",
        md_table(
            ["Tau", "Ga", "Den", "Cho tai ga (s)", "Di", "Lech lich (s)"],
            [
                [
                    call["train_id"],
                    call["station"],
                    md_time_s(call["arrival_time_s"]),
                    call["station_wait_s"],
                    md_time_s(call["departure_time_s"]),
                    call["schedule_variance_s"],
                ]
                for call in report["station_calls"]
            ],
        ) if report["station_calls"] else "_Chua co luot tau den ga._",
        "",
        "## Gioi han toc do PSR",
        "",
        md_table(
            ["Start (m)", "End (m)", "PSR (km/h)", "Gradient"],
            [
                [limit["start_m"], limit["end_m"], limit["psr_kmh"], limit["gradient"]]
                for limit in report["speed_limits"]
            ],
        ),
        "",
        "## TSR dang ap dung",
        "",
        md_table(
            ["Start (m)", "End (m)", "Speed (km/h)"],
            [
                [zone["start_m"], zone["end_m"], zone["speed_kmh"]]
                for zone in report["temporary_speed_restrictions"]
            ],
        ) if report["temporary_speed_restrictions"] else "_Khong co TSR._",
        "",
    ]
    return "\n".join(sections)


def save_simulation_report(sim, reports_dir: Path = REPORTS_DIR) -> Path:
    reports_dir.mkdir(parents=True, exist_ok=True)
    safe_name = safe_report_slug(sim.scenario.get("name", "scenario"))
    timestamp = report_file_timestamp()
    path = reports_dir / f"{timestamp}_{safe_name}.md"
    path.write_text(build_simulation_markdown_report(sim), encoding="utf-8")
    return path


def save_simulation_csv_report(sim, path: str | Path) -> Path:
    csv_path = Path(path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "train_id",
        "status",
        "drive_mode",
        "position_m",
        "speed_kmh",
        "target_distance_m",
        "headway_s",
        "psr_kmh",
        "atp_state",
        "atp_action",
        "atp_alert",
        "ato_state",
        "door_authorized",
        "dwell_remaining_s",
        "distance_m",
        "traction_work_kwh",
        "brake_work_kwh",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for train in sim.trains:
            writer.writerow(
                {
                    "train_id": train.id,
                    "status": train_status_label(train),
                    "drive_mode": train.drive_mode,
                    "position_m": round_value(train.pos, 1),
                    "speed_kmh": round_value(train.speed * 3.6, 1),
                    "target_distance_m": round_value(max(0.0, getattr(train, "stop_target_pos", train.eoa) - train.reported_pos), 1),
                    "headway_s": round_value(train.headway_time_s),
                    "psr_kmh": round_value(train.psr_kmh, 1),
                    "atp_state": train.atp_state,
                    "atp_action": train.atp_action,
                    "atp_alert": train.atp_alert,
                    "ato_state": getattr(train, "ato_state", ""),
                    "door_authorized": bool(getattr(train, "door_authorized", False)),
                    "dwell_remaining_s": round_value(getattr(train, "dwell_remaining_s", 0.0)),
                    "distance_m": round_value(getattr(train, "analytics_distance_m", 0.0), 1),
                    "traction_work_kwh": round_value(getattr(train, "analytics_traction_work_j", 0.0) / 3_600_000.0, 3),
                    "brake_work_kwh": round_value(getattr(train, "analytics_brake_work_j", 0.0) / 3_600_000.0, 3),
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
    for path in sorted(reports_dir.glob("*.md"), reverse=True):
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

