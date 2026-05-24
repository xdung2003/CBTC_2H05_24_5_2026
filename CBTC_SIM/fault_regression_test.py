from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import main_gui
from scenario_loader import normalize_scenario


def make_single_train_scenario(drive_mode: str = "ATO"):
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 80}]},
            "trains": [{"id": "F1", "start_pos": 100, "drive_mode": drive_mode}],
            "source_trains": [],
            "scheduled_stops": [],
        }
    )


def run_steps(sim: main_gui.Simulation, steps: int):
    for _ in range(steps):
        sim.step()


def has_event(train: main_gui.Train, event_name: str) -> bool:
    return any(record["event"] == event_name for record in train.event_records)


def test_atp_fault_forces_fail_safe_trip():
    sim = main_gui.Simulation(make_single_train_scenario())
    run_steps(sim, 3)
    train = sim.trains[0]
    train.set_fault("ATP", True, sim.sim_time_s)
    sim.step()
    if not train.atp_fault_active:
        raise AssertionError("ATP fault flag was not set")
    if not train.emg_latch or train.atp_action != "EBI" or train.atp_state != "ATP_TRIP":
        raise AssertionError("ATP fault must force fail-safe EBI/trip")
    if not has_event(train, "ATP_FAULT_FAIL_SAFE_TRIP"):
        raise AssertionError("ATP fault reaction was not logged")


def test_ato_fault_disables_ato_but_keeps_atp_available():
    sim = main_gui.Simulation(make_single_train_scenario("ATO"))
    run_steps(sim, 3)
    train = sim.trains[0]
    train.set_fault("ATO", True, sim.sim_time_s)
    sim.step()
    if not train.ato_fault_active:
        raise AssertionError("ATO fault flag was not set")
    if train.drive_mode == "ATO" or train.ato_state != "ATO_FAULT":
        raise AssertionError("ATO fault should degrade out of ATO and mark ATO_FAULT")
    if train.ato_target_speed != 0.0:
        raise AssertionError("ATO target must be inhibited while ATO fault is active")
    if train.atp_state not in {"ATP_OK", "ATP_STANDBY", "ATP_CUTOFF", "ATP_WARNING", "ATP_SERVICE"}:
        raise AssertionError("ATO fault must not disable ATP supervision")


def test_dcs_loss_invalidates_safe_packet_and_trips():
    sim = main_gui.Simulation(make_single_train_scenario())
    run_steps(sim, 3)
    train = sim.trains[0]
    train.set_fault("DCS", True, sim.sim_time_s)
    run_steps(sim, int(main_gui.DCS_TIMEOUT_S / main_gui.DT) + 5)
    if train.safe_packet_valid:
        raise AssertionError("DCS loss should invalidate the onboard safe packet")
    if not train.emg_latch or train.atp_action != "EBI":
        raise AssertionError("DCS timeout must trip the train fail-safe")
    if not has_event(train, "DCS_LOSS_ACTIVE"):
        raise AssertionError("DCS loss injection was not logged")


def main() -> int:
    test_atp_fault_forces_fail_safe_trip()
    test_ato_fault_disables_ato_but_keeps_atp_available()
    test_dcs_loss_invalidates_safe_packet_and_trips()
    print("fault regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
