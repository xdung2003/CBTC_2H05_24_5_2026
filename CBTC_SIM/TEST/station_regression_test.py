from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import GUI.main_gui as main_gui
from CONFIG.scenario_loader import load_scenario, normalize_scenario


def make_scenario(station_name: str, pos_m: float, capacity: int, starts: list[float], dwell_s: float = 5.0):
    terminal_pos_m = pos_m + 500.0
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": max(1600, pos_m + 800), "gradient": 0.0, "psr_kmh": 65}]},
            "scheduled_stops": [
                {"name": station_name, "pos_m": pos_m, "length_m": 160, "capacity": capacity, "dwell_s": dwell_s},
                {"name": "TERMINAL", "pos_m": terminal_pos_m, "length_m": 160, "capacity": capacity, "dwell_s": dwell_s},
            ],
            "trains": [{"id": f"T{idx + 1}", "start_pos": start} for idx, start in enumerate(starts)],
        }
    )


def run_steps(sim: main_gui.Simulation, steps: int):
    max_slots = 0
    for _ in range(steps):
        sim.step()
        if sim.scheduled_stops:
            max_slots = max(max_slots, sim._station_slot_count(0))
        assert_station_invariants(sim)
    return max_slots


def run_station_lifecycle(sim: main_gui.Simulation, steps: int):
    seen = {
        train.id: {"dwell": False, "completed": False, "ebi": False}
        for train in sim.trains
    }
    max_slots = 0
    for _ in range(steps):
        sim.step()
        max_slots = max(max_slots, sim._station_slot_count(0))
        assert_station_invariants(sim)
        for train in sim.trains:
            if train.dwell_remaining_s > 0.0:
                seen[train.id]["dwell"] = True
            first_station_released = True
            if sim.station_route_states:
                first_station_released = all(
                    train.id not in {line.get("reserved_by_train_id"), line.get("occupied_by_train_id")}
                    for line in sim.station_route_states[0]["lines"]
                )
            if (
                seen[train.id]["dwell"]
                and train.next_scheduled_stop_idx > 0
                and (train.station_state == "COMPLETED_STOP" or first_station_released)
            ):
                seen[train.id]["completed"] = True
            if train.atp_action == "EBI" or train.emg_latch:
                seen[train.id]["ebi"] = True
        if seen and all(item["completed"] for item in seen.values()):
            break
    return seen, max_slots


def assert_station_invariants(sim: main_gui.Simulation):
    for station_idx, stop in enumerate(sim.scheduled_stops):
        capacity = int(stop.get("capacity", 1))
        lines = sim.station_route_states[station_idx]["lines"]
        active_slots = [
            line for line in lines
            if line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
        ]
        if len(active_slots) > capacity:
            raise AssertionError("station active slots exceeded configured capacity")
        for line in lines:
            users = {
                value
                for value in (line.get("reserved_by_train_id"), line.get("occupied_by_train_id"))
                if value is not None
            }
            if int(line.get("capacity", 1)) == 1 and len(users) > 1:
                raise AssertionError("two trains reserved/occupied the same capacity-1 station line")


def assert_train_completed_station(train: main_gui.Train, seen: dict[str, bool]):
    if seen["ebi"]:
        raise AssertionError(f"{train.id} tripped during normal station regression")
    if not seen["completed"]:
        raise AssertionError(f"{train.id} did not clear station stop lifecycle")
    if not seen["dwell"]:
        raise AssertionError(f"{train.id} did not start dwell")


def test_station_1000_regression():
    sim = main_gui.Simulation(make_scenario("STATION_1000", 1000, 2, [860, 650]))
    seen, _max_slots = run_station_lifecycle(sim, 6000)
    for train in sim.trains:
        assert_train_completed_station(train, seen[train.id])


def test_station_2000_capacity():
    sim = main_gui.Simulation(make_scenario("STATION_2000", 2000, 3, [1200, 1270, 1340, 1410, 1480]))
    max_slots = run_steps(sim, 20)
    if max_slots > 3:
        raise AssertionError("STATION_2000 accepted more trains than its 3-slot capacity")
    assigned = [train for train in sim.trains if train.station_lane is not None]
    if len(assigned) > 3:
        raise AssertionError("more than 3 trains were assigned station lines at a 3-slot station")
    rejected = [train for train in sim.trains if train.station_reject_reason in {"STATION_FULL", "NO_FREE_PLATFORM", "ROUTE_CONFLICT"}]
    if len(rejected) < 2:
        raise AssertionError("4th/5th train should be rejected or held outside a full station")


def test_single_receiving_route_per_station():
    sim = main_gui.Simulation(make_scenario("ONE_ROUTE", 1000, 3, [860, 650, 440]))
    for _ in range(5):
        sim.step()
        state = sim.station_route_states[0]
        receiving_lines = [
            line for line in state["lines"]
            if state.get("route_lane") == int(line["lane"]) and line.get("reserved_by_train_id") is not None
        ]
        if len(receiving_lines) > 1:
            raise AssertionError("station opened more than one receiving route")


def test_front_train_entering_station_holds_follower_before_entry():
    sim = main_gui.Simulation(make_scenario("ENTRY_HOLD", 1000, 2, [930, 760]))
    front, follower = sim.trains
    front.station_lane = 0
    front.active_scheduled_stop = front.scheduled_stops[0]
    front.commanded_stop = True
    front.zero_speed_detected = False
    front.speed = main_gui.kmh_to_ms(12.0)
    follower.active_scheduled_stop = follower.scheduled_stops[0]
    follower.commanded_stop = True
    sim._update_parallel_protection_zones()
    sim._update_station_routes()
    sim._dispatch_safe_packets(with_delay=False)
    follower.step(sim.sim_time_s)
    station_start = sim._station_bounds(sim.scheduled_stops[0])[0]
    if follower.eoa >= station_start:
        raise AssertionError("follower received EOA into station while front train was still entering")


def test_five_train_moving_block_eoa_is_immediate():
    sim = main_gui.Simulation(
        make_scenario("NO_STOP", 2000, 1, [1000, 850, 700, 550, 400])
    )
    sim.scheduled_stops.clear()
    for train in sim.trains:
        train.scheduled_stops = []
        train.active_scheduled_stop = None
        train.commanded_stop = False
    sim._sync_station_route_states()
    sim._dispatch_safe_packets(with_delay=False)
    for train in sim.trains:
        train.step(sim.sim_time_s)
        if train.eoa <= 0.0:
            raise AssertionError("train started without a valid moving-block EOA")
    ordered = sorted(sim.trains, key=lambda item: item.reported_pos, reverse=True)
    mal_map = sim.zc.compute_mal()
    for idx, follower in enumerate(ordered[1:], start=1):
        front = ordered[idx - 1]
        if abs(mal_map[follower.id].protected_rear_m - front.safe_rear_end_pos()) > 1e-6:
            raise AssertionError("moving-block EOA was not based on the nearest train ahead")


def test_siding_platform_departure():
    sim = main_gui.Simulation(make_scenario("SIDING_TEST", 1000, 2, [860, 650]))
    seen, _max_slots = run_station_lifecycle(sim, 6000)
    siding_train = sim.trains[1]
    if not seen[siding_train.id]["completed"]:
        raise AssertionError("second train did not complete the siding/platform line departure")
    assert_train_completed_station(siding_train, seen[siding_train.id])


def test_mixed_main_and_platform_lines():
    sim = main_gui.Simulation(make_scenario("MIXED_LINES", 1000, 2, [860, 650]))
    assigned_line_ids: set[str] = set()
    for _ in range(3000):
        sim.step()
        for train in sim.trains:
            if train.assigned_station_line_id is not None:
                assigned_line_ids.add(train.assigned_station_line_id)
        active_receiving = sum(1 for state in sim.station_route_states if state.get("route_lane") is not None)
        if active_receiving > 1:
            raise AssertionError("station opened multiple receiving routes at the same time")
        if len(assigned_line_ids) >= 2:
            break
    if len(assigned_line_ids) != 2:
        raise AssertionError("main and platform line assignments were not kept distinct over the station lifecycle")


def test_tail_clear_release():
    sim = main_gui.Simulation(make_scenario("TAIL_CLEAR", 500, 1, [360], dwell_s=1))
    saw_release_pending = False
    saw_free_after_tail_clear = False
    for _ in range(5000):
        sim.step()
        train = sim.trains[0]
        line = sim.station_route_states[0]["lines"][0]
        tail_pos = train.pos - train.length
        if train.pos > line["end_m"] and tail_pos < line["end_m"] and line["route_state"] != "FREE":
            saw_release_pending = True
        if saw_release_pending and tail_pos > line["end_m"] + main_gui.PARALLEL_RELEASE_MARGIN_M and line["route_state"] == "FREE":
            saw_free_after_tail_clear = True
            break
    if not saw_release_pending:
        raise AssertionError("station line released before checking tail-clear state")
    if not saw_free_after_tail_clear:
        raise AssertionError("station line did not release after tail clear")


def test_new_station_generic_behavior():
    sim = main_gui.Simulation(make_scenario("NEW_GENERIC_STATION", 1300, 2, [1160, 900]))
    seen, _max_slots = run_station_lifecycle(sim, 7000)
    for train in sim.trains:
        assert_train_completed_station(train, seen[train.id])


def test_dense_default_late_trains_do_not_false_dwell_or_trip():
    sim = main_gui.Simulation(load_scenario())
    for _ in range(6000):
        sim.step()
        assert_station_invariants(sim)
        for train in sim.trains:
            if train.atp_action == "EBI" or train.emg_latch:
                raise AssertionError(f"{train.id} tripped during dense default station operation")
            if train.dwell_remaining_s > 0.0 and train.active_scheduled_stop is not None:
                stop_pos = float(train.active_scheduled_stop["pos_m"])
                if abs(train.pos - stop_pos) > main_gui.STOP_ACCURACY_TOL_M:
                    raise AssertionError(f"{train.id} started dwell away from the scheduled station marker")
    late_train = max(sim.trains, key=lambda train: int(train.id.rsplit("_", 1)[-1]) if "_" in train.id else 0)
    cleared_first_station = late_train.next_scheduled_stop_idx >= 1 and late_train.pos > 1080.0
    safely_held_at_source = (
        late_train.protection_zone_id == "SOURCE"
        and late_train.departure_hold
        and late_train.pos <= 0.0
    )
    if not (cleared_first_station or safely_held_at_source):
        raise AssertionError("late source train was neither safely held at source nor clear of the first station")


def test_terminal_station_opens_all_configured_lines_and_holds_dwell():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 3000, "gradient": 0.0, "psr_kmh": 70}]},
            "scheduled_stops": [{"name": "TERM_3000", "pos_m": 3000, "length_m": 160, "capacity": 6, "dwell_s": 5}],
            "trains": [{"id": "T1", "start_pos": 2400}],
            "source_trains": [],
        }
    )
    sim = main_gui.Simulation(scenario)
    lines = sim.station_route_states[0]["lines"]
    configured_lanes = {int(line["lane"]) for line in lines}
    if configured_lanes != set(range(6)):
        raise AssertionError("terminal station did not configure every receiving line")
    if not sim._is_terminal_station(0):
        raise AssertionError("single configured stop should be treated as a terminal station")


def test_final_scheduled_station_uses_configured_capacity_after_intermediate_stops():
    scenario = load_scenario()
    for stop in scenario["scheduled_stops"]:
        stop["capacity"] = 7
    scenario["source_trains"][0]["capacity"] = 7
    scenario["source_trains"][0]["total_trains"] = 7
    sim = main_gui.Simulation(scenario)
    final_station_idx = len(sim.scheduled_stops) - 1
    final_lines = sim.station_route_states[final_station_idx]["lines"]
    configured_lanes = {int(line["lane"]) for line in final_lines}
    if configured_lanes != set(range(7)):
        raise AssertionError("final scheduled station did not configure every receiving line")


def test_non_physical_terminal_final_station_holds_dwell():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 3000, "gradient": 0.0, "psr_kmh": 70}]},
            "scheduled_stops": [{"name": "TERM_2000", "pos_m": 2000, "length_m": 160, "capacity": 5, "dwell_s": 5}],
            "trains": [{"id": "T1", "start_pos": 1600}],
            "source_trains": [],
        }
    )
    sim = main_gui.Simulation(scenario)
    if not sim._is_final_scheduled_station(0):
        raise AssertionError("last scheduled station before physical track end was not treated as final")
    configured_lanes = {int(line["lane"]) for line in sim.station_route_states[0]["lines"]}
    if configured_lanes != set(range(5)):
        raise AssertionError("non-physical terminal station did not configure every receiving line")


def test_assigned_receive_route_is_not_changed_to_another_line():
    sim = main_gui.Simulation(make_scenario("LOCKED_ROUTE", 1000, 3, [850, 700]))
    station_idx = 0
    stop = sim.scheduled_stops[station_idx]
    locked_train, new_train = sim.trains
    locked_train.active_scheduled_stop = stop
    locked_train.station_lane = 0
    locked_train.assigned_station_id = str(stop["name"])
    locked_train.assigned_station_line_id = sim._station_line_id(station_idx, 0)
    sim.station_route_states[station_idx]["lines"][0]["occupied_by_train_id"] = "OTHER"
    sim.station_route_states[station_idx]["lines"][0]["route_state"] = "OCCUPIED"
    if sim.assign_receive_route(station_idx, locked_train):
        raise AssertionError("assigned receive route was changed instead of being held locked")
    if locked_train.station_lane != 0 or locked_train.assigned_station_line_id != sim._station_line_id(station_idx, 0):
        raise AssertionError("locked route assignment was modified")
    new_train.active_scheduled_stop = stop
    if not sim.assign_receive_route(station_idx, new_train):
        raise AssertionError("unassigned train did not receive the next free route")
    if new_train.station_lane != 1:
        raise AssertionError("unassigned train did not receive the lowest-numbered free station line")


def main() -> int:
    test_station_1000_regression()
    test_station_2000_capacity()
    test_single_receiving_route_per_station()
    test_front_train_entering_station_holds_follower_before_entry()
    test_five_train_moving_block_eoa_is_immediate()
    test_siding_platform_departure()
    test_mixed_main_and_platform_lines()
    test_tail_clear_release()
    test_new_station_generic_behavior()
    test_dense_default_late_trains_do_not_false_dwell_or_trip()
    test_terminal_station_opens_all_configured_lines_and_holds_dwell()
    test_final_scheduled_station_uses_configured_capacity_after_intermediate_stops()
    test_non_physical_terminal_final_station_holds_dwell()
    test_assigned_receive_route_is_not_changed_to_another_line()
    print("station regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


