from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import main_gui
import reporting
from scenario_loader import normalize_scenario


def make_scenario():
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 70}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500.0, "length_m": 160.0, "capacity": 1, "dwell_s": 5.0}],
            "trains": [{"id": "A1", "start_pos": 0.0, "drive_mode": "ATO"}],
            "source_trains": [],
        }
    )


def test_analytics_accumulates_train_metrics():
    sim = main_gui.Simulation(make_scenario())
    saw_runtime_traction = False
    for _ in range(1200):
        sim.step()
        if sim.trains[0].runtime_traction_force_n > 0.0:
            saw_runtime_traction = True
        if sim.trains[0].dwell_remaining_s > 0.0:
            break
    train = sim.trains[0]
    if train.analytics_distance_m <= 0.0:
        raise AssertionError("train distance analytics did not accumulate")
    if train.analytics_traction_work_j <= 0.0:
        raise AssertionError("traction work analytics did not accumulate")
    if sim.analytics.get("traction_work_kwh", 0.0) <= 0.0:
        raise AssertionError("aggregate traction work was not published")
    if "trains_per_hour" not in sim.analytics:
        raise AssertionError("capacity metric trains_per_hour missing")
    if train.distance_to_eoa is None:
        raise AssertionError("distance to EOA should be available")
    if train.runtime_used_legacy_fallback:
        raise AssertionError("simulation runtime should use force-balance dynamics by default")
    if not saw_runtime_traction:
        raise AssertionError("runtime traction force telemetry did not accumulate from Train.step")
    station_metrics = sim.analytics.get("station_passenger_metrics", [])
    if not station_metrics or not station_metrics[0].get("arrivals"):
        raise AssertionError("station passenger metrics should record train arrivals at platforms")
    arrival = station_metrics[0]["arrivals"][0]
    if arrival.get("train_id") != train.id or arrival.get("arrival_time_s") is None:
        raise AssertionError("station passenger arrival record missing train id or arrival time")
    planned_dwell = arrival.get("planned_dwell_s")
    if planned_dwell is not None and planned_dwell < main_gui.MIN_PASSENGER_DWELL_S:
        raise AssertionError("station dwell should be clamped to the minimum passenger dwell time")


def test_report_contains_tables_and_csv(tmp_dir: Path | None = None):
    sim = main_gui.Simulation(make_scenario())
    for _ in range(200):
        sim.step()
    report = reporting.build_simulation_report(sim)
    analytics = report["analytics"]
    if "traction_work_kwh" not in analytics or "brake_work_kwh" not in analytics:
        raise AssertionError("report analytics missing energy fields")
    if "distance_to_eoa_m" not in report["trains"][0]:
        raise AssertionError("train report missing distance_to_eoa_m")
    if "analytics" not in report["trains"][0]:
        raise AssertionError("train report missing per-train analytics block")
    if report["simulation"].get("runtime_dynamics") != "FORCE_BALANCE":
        raise AssertionError("report should identify force-balance runtime dynamics")
    if "capacity_comparison" in report or "capacity_comparison" in analytics:
        raise AssertionError("report should not include moving-block versus fixed-block capacity comparison")
    if "capacity_baseline_note" in report:
        raise AssertionError("report should not include fixed-block baseline note")
    if "station_passenger_metrics" not in analytics:
        raise AssertionError("report analytics missing station passenger metrics")
    train_analytics = report["trains"][0]["analytics"]
    if "runtime_traction_force_n" not in train_analytics or "runtime_brake_force_n" not in train_analytics:
        raise AssertionError("report analytics missing runtime force telemetry")

    reports_dir = PROJECT_DIR.parent / "reports" / "_test_analytics"
    path = reporting.save_simulation_report(sim, reports_dir)
    trains_csv = path.with_suffix(".trains.csv")
    events_csv = path.with_suffix(".events.csv")
    capacity_csv = path.with_suffix(".capacity.csv")
    if not trains_csv.exists() or not events_csv.exists():
        raise AssertionError("CSV sidecar reports were not written")
    if capacity_csv.exists():
        raise AssertionError("capacity comparison CSV should not be written")
    header = trains_csv.read_text(encoding="utf-8").splitlines()[0]
    if "traction_work_kwh" not in header:
        raise AssertionError("train CSV missing energy columns")
    if "runtime_traction_force_n" not in header or "runtime_brake_force_n" not in header:
        raise AssertionError("train CSV missing runtime force columns")


def main() -> int:
    test_analytics_accumulates_train_metrics()
    test_report_contains_tables_and_csv()
    print("analytics report regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
