from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import main_gui
from capacity_baseline import build_capacity_comparison
from scenario_loader import normalize_scenario


def make_source_scenario(headway: dict, capacity: int = 3, psr_kmh: float = 80.0):
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1800, "gradient": 0.0, "psr_kmh": psr_kmh}]},
            "scheduled_stops": [],
            "trains": [],
            "source_trains": [{"name": "SRC", "capacity": capacity, "total_trains": capacity}],
            "headway": headway,
        }
    )


def make_two_train_scenario(front_pos: float = 500.0, follower_pos: float = 260.0, headway: dict | None = None):
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1800, "gradient": 0.0, "psr_kmh": 80.0}]},
            "scheduled_stops": [],
            "source_trains": [],
            "headway": headway if headway is not None else {"mode": "fixed", "target_headway_s": 10.0},
            "capacity_baseline": {"blocks_per_section": 6},
            "trains": [
                {"id": "FRONT", "start_pos": front_pos, "drive_mode": "ATO"},
                {"id": "FOLLOW", "start_pos": follower_pos, "drive_mode": "ATO"},
            ],
        }
    )


def make_station_departure_scenario(blocker_pos: float):
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1800, "gradient": 0.0, "psr_kmh": 80.0}]},
            "scheduled_stops": [
                {"name": "S1", "pos_m": 1000.0, "length_m": 160.0, "capacity": 2, "dwell_s": 1.0},
                {"name": "S2", "pos_m": 1600.0, "length_m": 160.0, "capacity": 2, "dwell_s": 1.0},
            ],
            "source_trains": [],
            "headway": {"mode": "off"},
            "capacity_baseline": {"blocks_per_section": 4},
            "trains": [
                {"id": "BLOCKER", "start_pos": blocker_pos, "drive_mode": "ATO"},
                {"id": "STATION", "start_pos": 1000.0, "drive_mode": "ATO"},
            ],
        }
    )


def run_steps(sim: main_gui.Simulation, steps: int):
    for _ in range(steps):
        sim.step()


def run_until_dispatches(sim: main_gui.Simulation, count: int, max_steps: int = 6000):
    for _ in range(max_steps):
        sim.step()
        if len(sim.headway_manager.stats.dispatch_times_s) >= count:
            return
    raise AssertionError(f"only saw {len(sim.headway_manager.stats.dispatch_times_s)} actual dispatches")


def test_fixed_headway_dispatch_spacing():
    sim = main_gui.Simulation(make_source_scenario({"mode": "fixed", "target_headway_s": 12.0}, 3))
    run_until_dispatches(sim, 2)
    actual = sim.headway_manager.stats.actual_headways_s
    if not actual:
        raise AssertionError("fixed headway did not record actual dispatch headway")
    if actual[0] < 12.0 - main_gui.DT:
        raise AssertionError(f"fixed headway spacing {actual[0]:.2f}s was shorter than target")
    if sim.analytics["target_headway_s"] != 12.0:
        raise AssertionError("target headway analytics did not track fixed headway")


def test_actual_headway_is_measured_between_consecutive_dispatch_pairs():
    sim = main_gui.Simulation(make_source_scenario({"mode": "fixed", "target_headway_s": 20.0}, 3))
    run_until_dispatches(sim, 1)
    if sim.analytics.get("avg_actual_headway_s") is not None:
        raise AssertionError("average actual headway should wait until a consecutive dispatch pair is measured")
    if sim.analytics.get("current_open_headway_s") is None:
        raise AssertionError("open headway interval should be tracked separately from actual pair averages")

    run_until_dispatches(sim, 2)
    pairs = sim.analytics.get("actual_headway_pairs", [])
    actual = sim.analytics.get("actual_headways_s", [])
    if len(pairs) != 1 or len(actual) != 1:
        raise AssertionError("actual headway should be recorded once per consecutive dispatch pair")
    pair = pairs[0]
    if pair["front_train_id"] == pair["following_train_id"]:
        raise AssertionError("actual headway pair should reference two different trains")
    expected = pair["following_dispatch_time_s"] - pair["front_dispatch_time_s"]
    if abs(pair["actual_headway_s"] - expected) > 1e-9:
        raise AssertionError("actual pair headway should equal the difference between dispatch times")
    if abs(sim.analytics["avg_actual_headway_s"] - actual[0]) > 1e-9:
        raise AssertionError("average actual headway should be the mean of measured pair headways")


def test_timetable_releases_on_exact_planned_times():
    sim = main_gui.Simulation(make_source_scenario({"mode": "timetable", "timetable_s": [0.0, 130.0, 245.0, 380.0]}, 4))
    run_until_dispatches(sim, 4, max_steps=9000)
    dispatch_times = sorted(sim.headway_manager.stats.release_times_s.values())
    expected = [0.0, 130.0, 245.0, 380.0]
    if len(dispatch_times) < len(expected):
        raise AssertionError("timetable did not dispatch every planned train")
    for actual, planned in zip(dispatch_times, expected):
        if abs(actual - planned) > main_gui.DT + 1e-9:
            raise AssertionError(f"timetable dispatch {actual:.2f}s did not match planned {planned:.2f}s")


def test_timetable_keeps_moving_block_speed_cap_until_schedule_hold():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 2400, "gradient": 0.0, "psr_kmh": 80.0}]},
            "scheduled_stops": [{"name": "S2", "pos_m": 2000.0, "length_m": 160.0, "capacity": 2, "dwell_s": 25.0}],
            "source_trains": [],
            "headway": {
                "mode": "timetable",
                "timetable_s": [0.0],
                "timetable_records": [
                    {
                        "train_id": "T01",
                        "station": "S2",
                        "arrival_time_s": 1000.0,
                        "departure_time_s": 1040.0,
                        "profile": "Eco",
                    }
                ],
            },
            "trains": [
                {
                    "id": "T01",
                    "start_pos": 1200.0,
                    "drive_mode": "ATO",
                    "schedule_records": [
                        {
                            "train_id": "T01",
                            "station": "S2",
                            "arrival_time_s": 1000.0,
                            "departure_time_s": 1040.0,
                            "profile": "Eco",
                        }
                    ],
                }
            ],
        }
    )
    sim = main_gui.Simulation(scenario)
    train = sim.trains[0]
    train.schedule_records = [
        {
            "train_id": "T01",
            "station": "S2",
            "arrival_time_s": 1000.0,
            "departure_time_s": 1040.0,
            "profile": "Eco",
        }
    ]
    train.active_scheduled_stop = sim.scheduled_stops[0]
    regulated, reason = sim._timetable_regulated_speed_cap_kmh(train, 80.0)
    if regulated != 80.0:
        raise AssertionError("timetable should keep the moving-block speed cap and balance early running at dwell")
    if reason != "TIMETABLE_RUN_MAX_DWELL_BALANCE":
        raise AssertionError("timetable should report max-running schedule balance instead of early coasting")


def test_timetable_station_departure_uses_schedule_not_headway_hold():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1800, "gradient": 0.0, "psr_kmh": 80.0}]},
            "scheduled_stops": [{"name": "S1", "pos_m": 1000.0, "length_m": 160.0, "capacity": 2, "dwell_s": 1.0}],
            "source_trains": [],
            "headway": {"mode": "timetable", "timetable_s": [0.0, 100.0]},
            "trains": [{"id": "T01", "start_pos": 1000.0, "drive_mode": "ATO"}],
        }
    )
    sim = main_gui.Simulation(scenario)
    train = sim.trains[0]
    train.active_scheduled_stop = sim.scheduled_stops[0]
    train.next_scheduled_stop_idx = 1
    train.commanded_stop = True
    train.station_lane = 0
    train.last_station_idx = 0
    train.dwell_remaining_s = main_gui.DT
    train.standstill_required = True
    train.standstill_anchor_pos = train.pos
    train.zero_speed_detected = True
    sim.station_last_departure_s[0] = sim.sim_time_s

    sim._update_train_stop_schedule(train)

    if train.dwell_remaining_s > 0.0:
        raise AssertionError("timetable station departure should not be extended by nominal headway hold")
    if train.commanded_stop:
        raise AssertionError("timetable train should be released once scheduled dwell/hold is complete")


def test_safety_restriction_eoa_is_supervised_before_min_activation_distance():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 2000, "gradient": 0.0, "psr_kmh": 80.0}]},
            "scheduled_stops": [],
            "source_trains": [],
            "trains": [{"id": "T04", "start_pos": 1000.0, "drive_mode": "ATO"}],
        }
    )
    sim = main_gui.Simulation(scenario)
    train = sim.trains[0]
    train.speed = main_gui.kmh_to_ms(48.0)
    train.vital_speed = train.speed
    train.filtered_speed = train.speed
    train.psr_kmh = 80.0
    train.limit_ahead_dist = float("inf")
    train.limit_ahead_speed_kmh = 80.0
    train.reported_pos = train.pos
    train.safe_front_end_pos = train.reported_pos + train.effective_position_uncertainty_m()
    train.distance_to_eoa = main_gui.STOP_TARGET_MIN_ACTIVATION_M + 40.0
    train.commanded_stop = False

    train.last_dispatched_eoa_reason = "LEADER_PROTECTION"
    leader_atp = train.atp_engine.compute(train)
    if leader_atp.stop_target_active:
        raise AssertionError("non-stop leader authority should not force early stop-target supervision")

    train.last_dispatched_eoa_reason = "SAFETY_RESTRICTION"
    safety_atp = train.atp_engine.compute(train)
    if not safety_atp.stop_target_active:
        raise AssertionError("safety-restriction EOA should be supervised before the late activation threshold")
    if safety_atp.curves["P"] >= leader_atp.curves["P"]:
        raise AssertionError("safety-restriction stop curve should reduce the permitted curve continuously from distance")


def test_adaptive_hold_when_front_train_is_slow():
    sim = main_gui.Simulation(
        make_source_scenario(
            {"mode": "adaptive", "target_headway_s": 1.0, "adaptive": {"min_gap_m": 500.0}},
            2,
            psr_kmh=20.0,
        )
    )
    saw_adaptive_hold = False
    for _ in range(250):
        sim.step()
        for train in sim.trains:
            if train.headway_hold_reason == "ADAPTIVE_GAP_ACTIVE":
                saw_adaptive_hold = True
        if len(sim.headway_manager.stats.dispatch_times_s) >= 2:
            break
    if not saw_adaptive_hold:
        raise AssertionError("adaptive headway did not hold the following train behind a slow front train")
    if len(sim.headway_manager.stats.dispatch_times_s) >= 2:
        first, second = sorted(sim.headway_manager.stats.dispatch_times_s.values())[:2]
        if second - first <= 1.0:
            raise AssertionError("adaptive gap hold released the second train too early")


def test_eoa_tracks_nearest_train_ahead_without_overgrant():
    sim = main_gui.Simulation(make_two_train_scenario())
    sim._dispatch_safe_packets(with_delay=False)
    ordered = sorted(sim.trains, key=lambda item: item.reported_pos, reverse=True)
    front, follower = ordered[0], ordered[1]
    mal = sim.zc.compute_mal()[follower.id]
    expected_rear = front.safe_rear_end_pos()
    if abs(mal.protected_rear_m - expected_rear) > 1e-6:
        raise AssertionError("EOA/MAL did not protect the nearest train ahead safe rear")
    if mal.mal_m > expected_rear - (main_gui.SAFETY_MARGIN_M + main_gui.OVERLAP_M) + 1e-6:
        raise AssertionError("EOA/MAL was granted too close to the train ahead")
    follower.step(sim.sim_time_s)
    if follower.eoa > mal.mal_m + 1e-6:
        raise AssertionError("onboard EOA exceeded the ZC movement authority limit")


def test_off_mode_uses_fixed_block_runtime_authority():
    moving = main_gui.Simulation(make_two_train_scenario(front_pos=760.0, follower_pos=120.0, headway={"mode": "fixed", "target_headway_s": 10.0}))
    fixed = main_gui.Simulation(make_two_train_scenario(front_pos=760.0, follower_pos=120.0, headway={"mode": "off"}))
    moving._dispatch_safe_packets(with_delay=False)
    fixed._dispatch_safe_packets(with_delay=False)
    moving_follow = next(train for train in moving.trains if train.id == "FOLLOW")
    fixed_follow = next(train for train in fixed.trains if train.id == "FOLLOW")
    moving_follow.step(moving.sim_time_s)
    fixed_follow.step(fixed.sim_time_s)
    if moving.block_mode != "moving_block":
        raise AssertionError("fixed headway mode should keep moving-block runtime authority")
    if fixed.block_mode != "fixed_block":
        raise AssertionError("off mode should switch runtime authority to fixed-block")
    if abs(fixed_follow.eoa - 599.0) > 1e-6:
        raise AssertionError("fixed-block runtime should stop at the boundary before the occupied block")
    if abs(fixed_follow.eoa - moving_follow.eoa) <= 1e-6:
        raise AssertionError("fixed-block runtime should use block boundaries, not moving-block rear tracking")
    occupancy = fixed.zc.fixed_block_occupancy()
    if occupancy.get("FB1.3") != ["FRONT"]:
        raise AssertionError("fixed-block occupancy did not mark the front train block")


def test_fixed_block_occupancy_tracks_train_body_overlap():
    sim = main_gui.Simulation(make_two_train_scenario(front_pos=610.0, follower_pos=120.0, headway={"mode": "off"}))
    occupancy = sim.zc.fixed_block_occupancy()
    front_blocks = [block_id for block_id, train_ids in occupancy.items() if "FRONT" in train_ids]
    if front_blocks != ["FB1.2", "FB1.3"]:
        raise AssertionError(f"fixed-block occupancy should follow train body overlap, saw {front_blocks}")
    sim._dispatch_safe_packets(with_delay=False)
    follower = next(train for train in sim.trains if train.id == "FOLLOW")
    follower.step(sim.sim_time_s)
    if abs(follower.eoa - 299.0) > 1e-6:
        raise AssertionError("fixed-block authority should stop before the first occupied block boundary")


def test_fixed_block_authority_grants_one_clear_block_ahead():
    sim = main_gui.Simulation(make_two_train_scenario(front_pos=1300.0, follower_pos=120.0, headway={"mode": "off"}))
    sim._dispatch_safe_packets(with_delay=False)
    follower = next(train for train in sim.trains if train.id == "FOLLOW")
    follower.step(sim.sim_time_s)
    if abs(follower.eoa - 599.0) > 1e-6:
        raise AssertionError("fixed-block authority should grant only the current block plus one clear block ahead")


def test_fixed_block_boundary_crossing_does_not_self_hold():
    sim = main_gui.Simulation(make_two_train_scenario(front_pos=700.0, follower_pos=301.0, headway={"mode": "off"}))
    sim._dispatch_safe_packets(with_delay=False)
    follower = next(train for train in sim.trains if train.id == "FOLLOW")
    follower.step(sim.sim_time_s)
    if abs(follower.eoa - 599.0) > 1e-6:
        raise AssertionError("fixed-block follower crossing a contiguous boundary should stop at the next block boundary, not at its own nose")


def test_fixed_block_station_departure_waits_for_outbound_block():
    sim = main_gui.Simulation(make_station_departure_scenario(blocker_pos=1161.0))
    station_train = next(train for train in sim.trains if train.id == "STATION")
    station_train.station_lane = 0
    station_train.last_station_idx = 0
    station_train.next_scheduled_stop_idx = 1
    station_train.active_scheduled_stop = station_train.scheduled_stops[1]
    station_train.commanded_stop = False
    station_train.station_state = "READY_TO_DEPART"
    station_train.dwell_remaining_s = 0.0
    sim._dispatch_safe_packets(with_delay=False)
    station_train.step(sim.sim_time_s)
    expected_eoa = 1080.0 - main_gui.STOP_SVL_OFFSET_M
    if abs(station_train.eoa - expected_eoa) > 1e-6:
        raise AssertionError("fixed-block station departure should hold at the station exit until the outbound block is free")


def test_dcs_fault_restricts_following_train_fail_safe():
    sim = main_gui.Simulation(make_two_train_scenario())
    follower = next(train for train in sim.trains if train.id == "FOLLOW")
    run_steps(sim, 3)
    follower.set_fault("DCS", True, sim.sim_time_s)
    run_steps(sim, int(main_gui.DCS_TIMEOUT_S / main_gui.DT) + 10)
    if follower.safe_packet_valid:
        raise AssertionError("DCS fault should invalidate the following train safe packet")
    if follower.atp_action != "EBI" or not follower.emg_latch:
        raise AssertionError("DCS fault should hold/restrict the following train fail-safe with EBI")


def test_ato_fault_does_not_break_atp_supervision():
    sim = main_gui.Simulation(make_two_train_scenario())
    train = next(item for item in sim.trains if item.id == "FOLLOW")
    run_steps(sim, 5)
    train.set_fault("ATO", True, sim.sim_time_s)
    sim.step()
    if train.drive_mode == "ATO" or train.ato_state != "ATO_FAULT":
        raise AssertionError("ATO fault should disable ATO/degrade mode")
    if train.atp_state not in {"ATP_OK", "ATP_STANDBY", "ATP_CUTOFF", "ATP_WARNING", "ATP_SERVICE"}:
        raise AssertionError("ATO fault should not disable ATP supervision")
    supervision = [
        train.curves["P"],
        train.curves["W"],
        train.hidden_curves["OFF"],
        train.hidden_curves["SBI"],
        train.curves["SBD"],
        train.hidden_curves["EBI"],
        train.curves["EBD"],
    ]
    if any(left > right + 1e-9 for left, right in zip(supervision, supervision[1:])):
        raise AssertionError("ATP supervision curve hierarchy broke after ATO fault")


def test_atp_fault_forces_safe_state():
    sim = main_gui.Simulation(make_two_train_scenario())
    train = next(item for item in sim.trains if item.id == "FOLLOW")
    run_steps(sim, 3)
    train.set_fault("ATP", True, sim.sim_time_s)
    sim.step()
    if not train.atp_fault_active:
        raise AssertionError("ATP fault flag was not set")
    if train.atp_action != "EBI" or not train.emg_latch or train.atp_state != "ATP_TRIP":
        raise AssertionError("ATP fault must force a fail-safe ATP_TRIP/EBI state")


def test_tsr_reduces_speed_and_capacity():
    speed_base = main_gui.Simulation(make_two_train_scenario(front_pos=700.0, follower_pos=0.0))
    speed_tsr = main_gui.Simulation(make_two_train_scenario(front_pos=700.0, follower_pos=0.0))
    speed_tsr.tsr_zones.append({"start": 0.0, "end": 650.0, "speed": 25.0})
    run_steps(speed_base, 500)
    run_steps(speed_tsr, 500)
    base_follow = next(train for train in speed_base.trains if train.id == "FOLLOW")
    tsr_follow = next(train for train in speed_tsr.trains if train.id == "FOLLOW")
    if tsr_follow.speed >= base_follow.speed:
        raise AssertionError("TSR did not reduce observed train speed")

    base = main_gui.Simulation(
        make_source_scenario(
            {"mode": "adaptive", "target_headway_s": 8.0, "adaptive": {"tsr_extra_s": 20.0}},
            3,
            psr_kmh=80.0,
        )
    )
    with_tsr = main_gui.Simulation(
        make_source_scenario(
            {"mode": "adaptive", "target_headway_s": 8.0, "adaptive": {"tsr_extra_s": 20.0}},
            3,
            psr_kmh=80.0,
        )
    )
    with_tsr.tsr_zones.append({"start": 0.0, "end": 1200.0, "speed": 25.0})
    run_steps(base, 900)
    run_steps(with_tsr, 900)
    if len(with_tsr.headway_manager.stats.dispatch_times_s) >= len(base.headway_manager.stats.dispatch_times_s):
        raise AssertionError("TSR did not reduce dispatched train count within the test window")
    base_hw = base.analytics.get("avg_actual_headway_s") or 0.0
    tsr_hw = with_tsr.analytics.get("avg_actual_headway_s") or 0.0
    if tsr_hw and base_hw and tsr_hw <= base_hw:
        raise AssertionError("TSR did not increase dispatch headway / reduce capacity")


def test_headway_mode_regulates_station_dwell_without_station_dwell_config():
    sim = main_gui.Simulation(
        normalize_scenario(
            {
                "track": {"segments": [{"start_m": 0, "end_m": 1600, "gradient": 0.0, "psr_kmh": 80.0}]},
                "scheduled_stops": [
                    {"name": "S500", "pos_m": 500.0, "length_m": 160.0, "capacity": 3},
                    {"name": "TERM", "pos_m": 1500.0, "length_m": 160.0, "capacity": 3},
                ],
                "trains": [],
                "source_trains": [{"name": "SRC", "capacity": 3, "total_trains": 3}],
                "headway": {"mode": "fixed", "target_headway_s": 20.0, "station_min_dwell_s": 5.0},
            }
        )
    )
    for _ in range(16000):
        sim.step()
        station_headways = sim.station_headway_actual_s.get(0, [])
        if len(station_headways) >= 2:
            break
    else:
        raise AssertionError("headway mode did not record regulated station departures")
    deviations = sim.station_headway_deviation_s.get(0, [])
    if not deviations or max(abs(value) for value in deviations) > 30.0 + main_gui.DT:
        raise AssertionError(f"station dwell regulation missed target tolerance: {deviations}")
    if any("dwell_s" in stop for stop in sim.scheduled_stops):
        raise AssertionError("station dwell regulation should not require per-station dwell_s")


def test_moving_block_headway_beats_fixed_block_baseline():
    sim = main_gui.Simulation(make_source_scenario({"mode": "fixed", "target_headway_s": 10.0}, 3))
    run_until_dispatches(sim, 2)
    comparison = build_capacity_comparison(sim)
    moving = comparison["MOVING_BLOCK"]
    fixed = comparison["FIXED_BLOCK_BASELINE"]
    if moving["average_headway_s"] >= fixed["average_headway_s"]:
        raise AssertionError("MOVING_BLOCK should have lower average headway than FIXED_BLOCK_BASELINE")
    if moving["trains_per_hour"] <= fixed["trains_per_hour"]:
        raise AssertionError("MOVING_BLOCK should have higher capacity than FIXED_BLOCK_BASELINE")


def test_collision_detection_latches_trip_and_records_event():
    sim = main_gui.Simulation(make_two_train_scenario(front_pos=500.0, follower_pos=450.0))
    sim.step()
    if sim.analytics.get("collision_count", 0) < 1:
        raise AssertionError("overlapping train bodies did not produce a collision detection")
    collided = [train for train in sim.trains if train.collision_latched]
    if len(collided) != 2:
        raise AssertionError("collision should latch both involved trains")
    if not all(train.trip_mode and train.atp_action == "EBI" for train in collided):
        raise AssertionError("collision should force trip mode and emergency intervention")
    if not sim.analytics.get("collision_events"):
        raise AssertionError("collision analytics missing collision event details")


def main() -> int:
    test_fixed_headway_dispatch_spacing()
    test_actual_headway_is_measured_between_consecutive_dispatch_pairs()
    test_timetable_releases_on_exact_planned_times()
    test_timetable_keeps_moving_block_speed_cap_until_schedule_hold()
    test_timetable_station_departure_uses_schedule_not_headway_hold()
    test_safety_restriction_eoa_is_supervised_before_min_activation_distance()
    test_adaptive_hold_when_front_train_is_slow()
    test_eoa_tracks_nearest_train_ahead_without_overgrant()
    test_off_mode_uses_fixed_block_runtime_authority()
    test_fixed_block_occupancy_tracks_train_body_overlap()
    test_fixed_block_authority_grants_one_clear_block_ahead()
    test_fixed_block_boundary_crossing_does_not_self_hold()
    test_fixed_block_station_departure_waits_for_outbound_block()
    test_dcs_fault_restricts_following_train_fail_safe()
    test_ato_fault_does_not_break_atp_supervision()
    test_atp_fault_forces_safe_state()
    test_tsr_reduces_speed_and_capacity()
    test_headway_mode_regulates_station_dwell_without_station_dwell_config()
    test_moving_block_headway_beats_fixed_block_baseline()
    test_collision_detection_latches_trip_and_records_event()
    print("headway regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
