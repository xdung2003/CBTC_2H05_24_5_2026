from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import GUI.main_gui as main_gui
from CONFIG.scenario_loader import normalize_scenario


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
        if train.ato_state == "ATO_TRACTION" or train.runtime_traction_force_n > 0.0:
            phases.add("TRACTION")
        if train.ato_state in {"ATO_BRAKE", "ATO_STOP", "ATO_CREEP"} or train.runtime_brake_force_n > 0.0:
            phases.add("BRAKE")
        if train.ato_target_speed <= main_gui.kmh_to_ms(3.0):
            phases.add("LOW_SPEED_APPROACH")
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
    if "TRACTION" not in phases:
        raise AssertionError("ATO profile should use traction before the station stop")
    if not ({"BRAKE", "LOW_SPEED_APPROACH"} & phases):
        raise AssertionError("ATO profile should transition to braking/low-speed approach near the target")


def test_ato_profile_command_bounds():
    sim = main_gui.Simulation(make_station_scenario())
    train = sim.trains[0]
    saw_traction_force = False
    saw_deceleration = False
    reached_dwell = False
    prev_speed = train.speed
    for _ in range(800):
        sim.step()
        if train.ato_target_speed < -1e-6:
            raise AssertionError("ATO target speed must stay non-negative")
        assert_not_above_atp_envelope(train)
        if train.runtime_traction_force_n > 0.0:
            saw_traction_force = True
        if train.speed < prev_speed - main_gui.kmh_to_ms(0.05):
            saw_deceleration = True
        prev_speed = train.speed
        if train.dwell_remaining_s > 0.0:
            reached_dwell = True
            break
        if train.runtime_used_legacy_fallback:
            raise AssertionError("ATO runtime should use force-balance adapter, not legacy fallback")
    if not saw_traction_force:
        raise AssertionError("ATO profile did not produce runtime traction force")
    if not saw_deceleration:
        raise AssertionError("ATO profile did not decelerate for the station target")
    if not reached_dwell:
        raise AssertionError("ATO profile did not reach station dwell during command-bound test")


def main() -> int:
    test_ato_profile_stops_at_station_without_exceeding_atp()
    test_ato_profile_command_bounds()
    print("ato profile regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


