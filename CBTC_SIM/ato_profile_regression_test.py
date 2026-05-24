from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import main_gui
from scenario_loader import normalize_scenario


def make_station_scenario():
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 70}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500.0, "length_m": 160.0, "capacity": 1, "dwell_s": 5.0}],
            "trains": [{"id": "ATO1", "start_pos": 0.0, "drive_mode": "ATO"}],
            "source_trains": [],
        }
    )


def assert_not_above_atp_envelope(train: main_gui.Train):
    permitted = train.curves.get("P", 0.0)
    if permitted <= 0.0:
        return
    if train.ato_target_speed > permitted + main_gui.kmh_to_ms(0.1):
        raise AssertionError("ATO target speed exceeded ATP permitted curve")


def test_ato_profile_stops_at_station_without_exceeding_atp():
    sim = main_gui.Simulation(make_station_scenario())
    train = sim.trains[0]
    phases: set[str] = set()
    reached_dwell = False

    for _ in range(5000):
        sim.step()
        phases.add(train.ato_profile_phase)
        assert_not_above_atp_envelope(train)
        if train.atp_action == "EBI" or train.emg_latch:
            raise AssertionError("ATO profile should not cause emergency intervention during normal station stop")
        if train.dwell_remaining_s > 0.0:
            reached_dwell = True
            if abs(train.pos - 500.0) > main_gui.STOP_ACCURACY_TOL_M:
                raise AssertionError("ATO profile did not stop at the scheduled station target")
            break

    if not reached_dwell:
        raise AssertionError("ATO profile train did not reach station dwell")
    if "LK" not in phases:
        raise AssertionError("ATO profile should use LK traction phase before the station stop")
    if not ({"QT", "HD", "HC"} & phases):
        raise AssertionError("ATO profile should transition to QT/HD/HC near the target")


def test_ato_profile_command_bounds():
    sim = main_gui.Simulation(make_station_scenario())
    train = sim.trains[0]
    saw_traction_force = False
    saw_brake_force = False
    for _ in range(300):
        sim.step()
        if not (0.0 <= train.ato_traction_command <= 1.0):
            raise AssertionError("ATO traction command must stay in 0..1")
        if not (0.0 <= train.ato_brake_command <= 1.0):
            raise AssertionError("ATO brake command must stay in 0..1")
        if train.ato_profile_phase not in {"LK", "OK", "QT", "HD", "HC"}:
            raise AssertionError("ATO profile phase must be one of LK/OK/QT/HD/HC")
        if train.ato_traction_command > 0.05 and train.runtime_traction_force_n > 0.0:
            saw_traction_force = True
        if train.ato_brake_command > 0.05 and train.runtime_brake_force_n > 0.0:
            saw_brake_force = True
        if train.runtime_used_legacy_fallback:
            raise AssertionError("ATO runtime should use force-balance adapter, not legacy fallback")
    if not saw_traction_force:
        raise AssertionError("ATO traction command was not reflected in runtime traction force")
    if not saw_brake_force:
        raise AssertionError("ATO brake command was not reflected in runtime brake force")


def main() -> int:
    test_ato_profile_stops_at_station_without_exceeding_atp()
    test_ato_profile_command_bounds()
    print("ato profile regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
