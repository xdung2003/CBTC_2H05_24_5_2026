from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def main() -> int:
    import GUI.main_gui as main_gui
    import REPORT.reporting as reporting
    from CONFIG.scenario_loader import load_scenario, normalize_scenario

    docs_dir = PROJECT_DIR / "DOCS"
    default_scenario = load_scenario(docs_dir / "default_scenario.yaml")
    dense_path = docs_dir / "scenario_dense_4_trains.yaml"
    degraded_path = docs_dir / "scenario_moving_block_degraded.yaml"
    dense_scenario = load_scenario(dense_path) if dense_path.exists() else normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1800, "gradient": 0.0, "psr_kmh": 80}]},
            "trains": [
                {"id": "D1", "start_pos": 900},
                {"id": "D2", "start_pos": 620},
                {"id": "D3", "start_pos": 340},
                {"id": "D4", "start_pos": 60},
            ],
        }
    )
    degraded_scenario = load_scenario(degraded_path) if degraded_path.exists() else normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 60}]},
            "trains": [
                {"id": "G1", "start_pos": 300, "drive_mode": "ATO"},
                {
                    "id": "G2",
                    "start_pos": 0,
                    "drive_mode": "CMD25",
                    "dcs_mute_windows": [{"start_s": 0.2, "end_s": 2.5}],
                },
            ],
        }
    )

    if default_scenario["trains"]:
        raise AssertionError("default_scenario.yaml should stage trains through source_trains")
    if sum(source.get("total_trains", 0) for source in default_scenario["source_trains"]) != 7:
        raise AssertionError("default_scenario.yaml should define 7 source trains")
    default_source = default_scenario["source_trains"][0]
    if default_source["start_m"] != -200 or default_source["length_m"] != 200:
        raise AssertionError("default source train should be fixed at -200..0")
    if len(dense_scenario["trains"]) != 4:
        raise AssertionError("scenario_dense_4_trains.yaml should define 4 trains")
    if degraded_scenario["trains"][1]["drive_mode"] != "CMD25":
        raise AssertionError("degraded scenario should preserve train drive modes")
    if not degraded_scenario["trains"][1]["dcs_mute_windows"]:
        raise AssertionError("degraded scenario should define DCS mute windows")
    alias_scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 500, "gradient": 0.0, "psr_kmh": 60}]},
            "trains": [
                {"id": "M1", "start_pos": 0, "drive_mode": "MCS"},
                {"id": "R1", "start_pos": -120, "drive_mode": "RM"},
            ],
        }
    )
    if alias_scenario["trains"][0]["drive_mode"] != "LMD":
        raise AssertionError("MCS/MTC aliases should normalize to LMD")
    if alias_scenario["trains"][1]["drive_mode"] != "CMD25":
        raise AssertionError("RM aliases should normalize to CMD25")
    default_mode_scenario = normalize_scenario(
        {
            "train_defaults": {"length_m": 60, "mass_kg": 291600, "drive_mode": "MTC"},
            "track": {"segments": [{"start_m": 0, "end_m": 500, "gradient": 0.0, "psr_kmh": 60}]},
            "trains": [{"id": "DM1", "start_pos": 0}],
        }
    )
    if default_mode_scenario["trains"][0]["drive_mode"] != "LMD":
        raise AssertionError("train_defaults.drive_mode should apply to trains without an override")

    watchdog = main_gui.DCSWatchdog(timeout_s=1.0, startup_grace_s=0.0)
    watchdog.mark_received(0.0)
    if not watchdog.packet_is_valid(0.5, True):
        raise AssertionError("DCS watchdog should accept fresh packets")
    if watchdog.packet_is_valid(1.2, True):
        raise AssertionError("DCS watchdog should fail safe after timeout")
    cc = main_gui.OnboardControlCenter("TSEQ", timeout_s=1.0, startup_grace_s=0.0)

    class PacketTrain:
        drive_mode = "ATO"
        eoa = 0.0
        psr_kmh = 0.0
        gradient = 0.0
        limit_ahead_speed_kmh = 0.0
        limit_ahead_dist = float("inf")
        safe_packet_age_s = 0.0
        safe_packet_valid = True

    packet_train = PacketTrain()
    newer_packet = main_gui.SafeMovementPacket(
        eoa_m=200.0,
        tsr_kmh=70.0,
        variants={"gradient": 0.0, "next_speed_limit_kmh": 70.0, "next_speed_limit_dist_m": float("inf")},
        issued_time_s=2.0,
    )
    older_packet = main_gui.SafeMovementPacket(
        eoa_m=100.0,
        tsr_kmh=70.0,
        variants={"gradient": 0.0, "next_speed_limit_kmh": 70.0, "next_speed_limit_dist_m": float("inf")},
        issued_time_s=1.0,
    )
    cc.receive_safe_packet(newer_packet, arrival_time_s=0.1)
    cc.receive_safe_packet(older_packet, arrival_time_s=0.2)
    cc.apply_to_train(packet_train, now_s=0.3)
    if packet_train.eoa != 200.0:
        raise AssertionError("Onboard CC must discard stale safe packets that arrive after newer packets")

    flat_brake_model = main_gui.VitalBrakeModel(
        [(0.0, 1000.0, 0.0, 80.0)],
        start_pos_m=0.0,
        position_uncertainty_m=4.0,
        vital_speed_ms=main_gui.kmh_to_ms(80.0),
        brake_build_s=1.5,
    )
    downhill_brake_model = main_gui.VitalBrakeModel(
        [(0.0, 1000.0, -0.04, 80.0)],
        start_pos_m=0.0,
        position_uncertainty_m=4.0,
        vital_speed_ms=main_gui.kmh_to_ms(80.0),
        brake_build_s=1.5,
    )
    no_delay_speed = flat_brake_model.speed_for_stop(300.0, 1.25, 0.0)
    delayed_speed = flat_brake_model.speed_for_stop(300.0, 1.25, 2.0)
    downhill_speed = downhill_brake_model.speed_for_stop(300.0, 1.25, 0.0)
    if not delayed_speed < no_delay_speed:
        raise AssertionError("EBD speed must be lower when reaction delay is included")
    if not downhill_speed < no_delay_speed:
        raise AssertionError("EBD speed must be lower on downhill gradient")
    service_stop_distance = main_gui.stopping_distance_with_buildup(main_gui.kmh_to_ms(60.0), 1.0, 1.5)
    emergency_stop_distance = main_gui.stopping_distance_with_buildup(main_gui.kmh_to_ms(60.0), 1.25, 1.5)
    if not service_stop_distance > emergency_stop_distance:
        raise AssertionError("SBD distance must be longer than EBD distance for the same speed")

    open_line_curve_scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1000, "gradient": 0.0, "psr_kmh": 80}]},
            "trains": [{"id": "CURVE", "start_pos": 100}],
        }
    )
    curve_train = main_gui.Simulation(open_line_curve_scenario).trains[0]
    curve_train._display_curves_initialized = True
    curve_train.commanded_stop = False
    curve_train.vital_speed = main_gui.kmh_to_ms(30.0)
    curve_train.curves = {
        "P": main_gui.kmh_to_ms(80.0),
        "W": main_gui.kmh_to_ms(82.0),
        "SBD": main_gui.kmh_to_ms(86.0),
        "EBD": main_gui.kmh_to_ms(90.0),
    }
    curve_train.hidden_curves = {
        "I": main_gui.kmh_to_ms(78.0),
        "OFF": main_gui.kmh_to_ms(84.0),
        "SBI": main_gui.kmh_to_ms(85.0),
        "EBI": main_gui.kmh_to_ms(88.0),
    }
    raw_curves = {
        "P": main_gui.kmh_to_ms(50.0),
        "W": main_gui.kmh_to_ms(52.0),
        "SBD": main_gui.kmh_to_ms(56.0),
        "EBD": main_gui.kmh_to_ms(60.0),
    }
    raw_hidden = {
        "I": main_gui.kmh_to_ms(48.0),
        "OFF": main_gui.kmh_to_ms(54.0),
        "SBI": main_gui.kmh_to_ms(55.0),
        "EBI": main_gui.kmh_to_ms(58.0),
    }
    curve_train.apply_display_curve_smoothing(raw_curves, raw_hidden)
    if main_gui.ms_to_kmh(curve_train.curves["P"]) < 78.7:
        raise AssertionError("open-line ATP permitted curve should rate-limit a non-dangerous target drop")
    smoothed_atp = main_gui.ATPEnvelopeResult(
        control_speed=main_gui.kmh_to_ms(40.0),
        actual_distance_to_stop=500.0,
        distance_to_svl=501.0,
        svl_m=600.0,
        target_active=True,
        stop_target_active=False,
        release_active=False,
        release_blend=0.0,
        p_t=curve_train.curves["P"],
        p_r=0.0,
        a_service=1.0,
        a_emergency=1.25,
        a_traction=1.0,
        curves=dict(curve_train.curves),
        hidden_curves=dict(curve_train.hidden_curves),
        curve_mode="TARGET",
        cutoff_threshold=curve_train.hidden_curves["OFF"],
        margin_dyn_m=0.0,
        atp_service_brake_decel=1.0,
        atp_emergency_brake_decel=1.25,
        release_speed_kmh=0.0,
    )
    ato_curve = curve_train.ato_engine.compute(curve_train, smoothed_atp).ato_curve_speed
    if main_gui.ms_to_kmh(ato_curve) < 78.7:
        raise AssertionError("ATO should see the rate-limited open-line ATP curve, not the one-tick raw drop")

    default_curve_sim = main_gui.Simulation(default_scenario)
    previous_p_curve: dict[str, float] = {}
    for _ in range(2200):
        default_curve_sim.step()
        in_debug_window = 195.0 <= default_curve_sim.sim_time_s <= 210.0
        for train in default_curve_sim.trains:
            if train.id not in {"SRC_2", "SRC_3"} or not in_debug_window:
                continue
            p_curve_kmh = main_gui.ms_to_kmh(train.curves["P"])
            previous = previous_p_curve.get(train.id)
            if previous is not None and previous - p_curve_kmh > 5.0:
                raise AssertionError("ATP P curve should not drop by more than 5 km/h in one open-line tick")
            previous_p_curve[train.id] = p_curve_kmh
            for event in train.pop_pending_events():
                if event["event"] == "ATP_EMERGENCY_INTERVENTION":
                    raise AssertionError("default 195-210s ATP curve regression should not create nuisance EBI")
        if default_curve_sim.sim_time_s > 210.0:
            break

    dense_source_curve_scenario = load_scenario(docs_dir / "default_scenario.yaml")
    dense_source_curve_scenario["source_trains"][0]["capacity"] = 8
    dense_source_curve_scenario["source_trains"][0]["total_trains"] = 8
    dense_source_curve_sim = main_gui.Simulation(dense_source_curve_scenario)
    dense_previous_p_curve: dict[str, float] = {}
    seen_departure_section_after_station_1000 = False
    for _ in range(3600):
        dense_source_curve_sim.step()
        for train in dense_source_curve_sim.trains:
            if train.id in {"SRC_5", "SRC_7"} and 850.0 <= train.pos <= 1005.0:
                p_curve_kmh = main_gui.ms_to_kmh(train.curves["P"])
                previous = dense_previous_p_curve.get(train.id)
                if previous is not None and previous - p_curve_kmh > 8.0:
                    raise AssertionError("dense source ATP P curve should not collapse near STATION_1000")
                dense_previous_p_curve[train.id] = p_curve_kmh
            for event in train.pop_pending_events():
                if event["event"] == "ATP_EMERGENCY_INTERVENTION" and train.id in {"SRC_5", "SRC_7"}:
                    raise AssertionError("dense source followers must not trip on stale station reservation ATP collapse")
            if (
                train.id == "SRC_2"
                and train.station_state == "DEPARTING"
                and train.last_station_idx == 0
                and train.station_lane is not None
                and train.active_scheduled_stop is None
                and 1000.0 <= train.pos <= 1080.0
                and not train.departure_hold
                and train.distance_to_eoa > 100.0
            ):
                seen_departure_section_after_station_1000 = True
        if dense_source_curve_sim.sim_time_s > 360.0:
            break
    if not seen_departure_section_after_station_1000:
        raise AssertionError("departing train must enter the next block section before taking the next station stop target")

    source_sim = main_gui.Simulation(default_scenario)
    source_sim.step()
    expected_source_trains = int(source_sim.source_trains[0]["total_trains"])
    if (
        len(source_sim.trains) != expected_source_trains
        or source_sim.source_trains[0]["generated"] != expected_source_trains
        or round(source_sim.sim_time_s, 1) != 0.1
    ):
        raise AssertionError("main_gui Simulation smoke step failed")
    staged_positions = sorted(round(train.pos, 1) for train in source_sim.trains)
    if staged_positions != [-105.0] * expected_source_trains:
        raise AssertionError("source should stage source-capacity trains at the source staging head")
    if not any(train.eoa >= source_sim.track_end_m for train in source_sim.trains) or not any(train.departure_hold for train in source_sim.trains):
        raise AssertionError("source departure gate should release one lane and hold the other source lanes")
    active_source_trains = sorted(
        [train for train in source_sim.trains if train.protection_zone_id == "SOURCE"],
        key=lambda item: item.pos,
        reverse=True,
    )
    if [train.protection_lane for train in active_source_trains] != list(range(len(active_source_trains))):
        raise AssertionError("source protection lanes must be reassigned front-to-back after a new train spawns")
    front_source = active_source_trains[0]
    if front_source.departure_hold or front_source.atp_action == "EBI":
        raise AssertionError("newly spawned rear source train must not hold or trip the source train ahead")
    dwell_sim = main_gui.Simulation(default_scenario)
    dwell_train = dwell_sim.trains[0]
    dwell_sim.trains = [dwell_train]
    dwell_sim.zc = main_gui.ZoneController(dwell_sim.trains, dwell_sim.track_end_m)
    dwell_train.pos = 1000.0
    dwell_train.reported_pos = 1000.0
    dwell_train.prev_pos = 1000.0
    dwell_train.speed = 0.0
    dwell_train.filtered_speed = 0.0
    dwell_train.eoa = 999.75
    dwell_train.standstill_anchor_pos = 1000.0
    dwell_train.dwell_remaining_s = main_gui.DT
    dwell_train.next_scheduled_stop_idx = 1
    dwell_train.commanded_stop = True
    dwell_train.active_scheduled_stop = dwell_train.scheduled_stops[0]
    dwell_sim.step()
    if dwell_train.atp_action == "EBI" or dwell_train.emg_latch or dwell_train.eoa <= 1000.0:
        raise AssertionError("train must not trip on stale station EOA when dwell releases")

    manual_ack_sim = main_gui.Simulation(default_scenario)
    ack_train = manual_ack_sim.trains[0]
    ack_train.dwell_remaining_s = 0.2
    ack_train.next_scheduled_stop_idx = 1
    ack_train.enter_trip_mode("TEST EBI LATCH", ack_train.reported_pos)
    ack_train.speed = 0.0
    ack_train.filtered_speed = 0.0
    for _ in range(5):
        manual_ack_sim._update_train_stop_schedule(ack_train)
        if not ack_train.emg_latch:
            raise AssertionError("EBI latch must not clear automatically during dwell/schedule updates")
    ack_train.acknowledge_emergency_safe()
    if ack_train.emg_latch or not ack_train.emergency_recovery_hold:
        raise AssertionError("manual acknowledgement should be required to leave EBI latch")

    station_route_scenario = normalize_scenario(
        {
            "display": {"track_min_m": 0, "track_max_m": 1200, "track_labels": [0, 500, 1000, 1200]},
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 80}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500, "length_m": 160, "capacity": 2, "dwell_s": 10}],
            "trains": [{"id": "FRONT", "start_pos": 500}, {"id": "FOLLOW", "start_pos": 250}],
        }
    )
    route_sim = main_gui.Simulation(station_route_scenario)
    front, follower = route_sim.trains
    front.pos = front.reported_pos = 500.0
    front.speed = front.filtered_speed = 0.0
    front.station_lane = 0
    front.active_scheduled_stop = front.scheduled_stops[0]
    front.commanded_stop = True
    front.standstill_required = True
    front.standstill_anchor_pos = front.pos
    follower.pos = follower.reported_pos = 250.0
    follower.active_scheduled_stop = follower.scheduled_stops[0]
    follower.commanded_stop = True
    route_sim._update_station_routes()
    route_sim._dispatch_safe_packets(with_delay=False)
    follower.step(route_sim.sim_time_s)
    if follower.station_lane != 1:
        raise AssertionError("station route should assign the following train to an unoccupied station line")
    if follower.departure_hold:
        raise AssertionError("station route on a free line must not be overridden by the old departure hold gate")
    if follower.eoa < 499.0:
        raise AssertionError("green station route should replace old moving-block limit with the routed stop EOA")

    gui_sim = main_gui.Simulation(dense_scenario)
    gui_sim.step()
    if len(gui_sim.trains) != 4 or round(gui_sim.sim_time_s, 1) != 0.1:
        raise AssertionError("dense Simulation smoke step failed")
    for train in gui_sim.trains:
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
            raise AssertionError("ATP supervision hierarchy must be P <= W <= OFF <= SBI <= SBD <= EBI <= EBD")

    def assert_curve_hierarchy(train):
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
            raise AssertionError("ATP supervision hierarchy must hold during approach/release/jog")
    mal_map = gui_sim.zc.compute_mal()
    ordered = sorted(gui_sim.trains, key=lambda t: t.reported_pos, reverse=True)
    if len(ordered) >= 3:
        second_follower_mal = mal_map[ordered[2].id]
        if abs(second_follower_mal.protected_rear_m - ordered[1].safe_rear_end_pos()) > 1e-9:
            raise AssertionError("ZC MAL must reference the nearest train ahead, not the leading train")
    if len(ordered) >= 2:
        front = ordered[0]
        follower = ordered[1]
        if mal_map[follower.id].mal_m >= front.safe_rear_end_pos():
            raise AssertionError("ZC MAL must stay behind the front train safe rear")
        follower_mal = mal_map[follower.id]
        minimum_reserve = follower_mal.safety_margin_m + follower_mal.overlap_m
        if follower_mal.mal_m > follower_mal.protected_rear_m - minimum_reserve:
            raise AssertionError("ZC MAL must reserve both safety margin and overlap before the protected point")
        if abs(follower_mal.mal_m - (follower_mal.protected_rear_m - minimum_reserve)) > 1e-9:
            raise AssertionError("ZC EOA should be protected point minus static safety buffer; braking is handled by onboard ATP curves")
        if follower_mal.follower_projection_m <= 0.0:
            raise AssertionError("ZC MAL should still expose dynamic follower projection for diagnostics")
        pre_trip_mal = follower_mal.mal_m
        front.enter_trip_mode("TEST TRIP", front.reported_pos)
        trip_mal = gui_sim.zc.compute_mal()[follower.id]
        if trip_mal.mal_m > pre_trip_mal + 1e-9:
            raise AssertionError("Front train trip must not extend follower EOA beyond the previous safe-rear authority")
        if trip_mal.protected_rear_m > front.safe_rear_end_pos() + 1e-9:
            raise AssertionError("Tripped front train must be protected by its safe rear, not its head position")
        follower.speed = main_gui.kmh_to_ms(5.0)
        low_speed_mal = gui_sim.zc.compute_mal()[follower.id].mal_m
        follower.speed = main_gui.kmh_to_ms(45.0)
        high_speed_mal = gui_sim.zc.compute_mal()[follower.id].mal_m
        if abs(low_speed_mal - high_speed_mal) > 1e-9:
            raise AssertionError("Follower EOA behind a tripped/stopped front train should not oscillate with follower speed")

    for _ in range(2000):
        if len(gui_sim.trains) >= 2:
            break
        gui_sim.step()
    if len(gui_sim.trains) < 2:
        raise AssertionError("source train should release a second train after the first clears the source spacing")

    stopped_train = gui_sim.trains[1]
    stopped_train.speed = 0.0
    stopped_train.filtered_speed = 0.0
    stopped_train.emg_latch = True
    stopped_train.emergency_stop = False
    stopped_train.trip_mode = False
    stopped_train.safe_packet_valid = True
    stopped_train.cc.pending_packets.clear()
    stopped_train.receive_safe_packet(
        main_gui.SafeMovementPacket(
            eoa_m=stopped_train.reported_pos + 120.0,
            tsr_kmh=80.0,
            variants={
                "gradient": 0.0,
                "next_speed_limit_kmh": 80.0,
                "next_speed_limit_dist_m": float("inf"),
            },
        ),
        gui_sim.sim_time_s,
    )
    stopped_train.step(gui_sim.sim_time_s)
    if not stopped_train.emg_latch or stopped_train.atp_action != "EBI":
        raise AssertionError("Emergency latch must not auto-release without safe confirmation")
    stopped_train.acknowledge_emergency_safe()
    if stopped_train.emg_latch or not stopped_train.emergency_recovery_hold:
        raise AssertionError("Safe confirmation should clear EBI latch but keep train locked in recovery hold")
    stopped_train.resume_after_emergency()
    if stopped_train.emergency_recovery_hold or stopped_train.standstill_required:
        raise AssertionError("Resume should release recovery hold after safe confirmation")

    stopped_train.commanded_stop = True
    stopped_train.zero_speed_detected = True
    stopped_train.speed = 0.0
    stopped_train.eoa = 100.0
    stopped_train.pos = 98.4
    stopped_train.stop_target_pos = 100.25
    stopped_train.jog_active = False
    stopped_train.jog_used = False
    stopped_train.door_authorized = False
    stopped_train.emergency_stop = False
    stopped_train.emg_latch = False
    stopped_train.trip_mode = False
    if not stopped_train.request_precise_jog():
        raise AssertionError("Precise jog should be requestable after first stop within 2m of EOA")
    if not stopped_train.precise_jog_in_progress or not stopped_train.jog_used:
        raise AssertionError("Precise jog request should enter a single in-progress jog phase")
    if stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must reject a second request during the same stop")
    now_s = gui_sim.sim_time_s
    for step_idx in range(240):
        stopped_train.receive_safe_packet(
            main_gui.SafeMovementPacket(
                eoa_m=100.0,
                tsr_kmh=80.0,
                variants={
                    "gradient": 0.0,
                    "next_speed_limit_kmh": 80.0,
                    "next_speed_limit_dist_m": float("inf"),
                },
            ),
            now_s + step_idx * main_gui.DT,
        )
        stopped_train.step(now_s + step_idx * main_gui.DT)
        if stopped_train.precise_jog_completed:
            break
    if not stopped_train.precise_jog_completed:
        raise AssertionError("Precise jog should complete in one continuous phase")
    if abs(stopped_train.pos - stopped_train.stop_target_pos) > 1e-9 or stopped_train.speed > 0.0:
        raise AssertionError("Precise jog should clamp exactly to the stop target and zero speed")
    if stopped_train.emg_latch or stopped_train.atp_action == "EBI":
        raise AssertionError("Precise jog success must not create ATP emergency")
    if not stopped_train.door_authorized:
        raise AssertionError("Door should authorize after successful precise jog")
    started_events = [
        event for event in stopped_train.event_records
        if event["event"] == "JOG_STARTED" and event["stop_id"] == stopped_train.current_stop_id
    ]
    if len(started_events) != 1:
        raise AssertionError("Precise jog must start exactly once for the current stop")
    if stopped_train.can_request_precise_jog() or stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must be single-shot for one commanded stop")
    if not any(event["event"] == "JOG_IGNORED_ALREADY_USED" for event in stopped_train.event_records):
        raise AssertionError("Second jog request should log JOG_IGNORED_ALREADY_USED")

    stable_pos = stopped_train.pos
    stable_time = now_s + 240 * main_gui.DT
    for step_idx in range(80):
        stopped_train.receive_safe_packet(
            main_gui.SafeMovementPacket(
                eoa_m=100.0,
                tsr_kmh=80.0,
                variants={
                    "gradient": 0.0,
                    "next_speed_limit_kmh": 80.0,
                    "next_speed_limit_dist_m": float("inf"),
                },
            ),
            stable_time + step_idx * main_gui.DT,
        )
        stopped_train.step(stable_time + step_idx * main_gui.DT)
        assert_curve_hierarchy(stopped_train)
    if abs(stopped_train.pos - stable_pos) > 1e-9 or stopped_train.speed > 0.0:
        raise AssertionError("Train must not pulse or drift after precise jog completion")
    stable_started_events = [
        event for event in stopped_train.event_records
        if event["event"] == "JOG_STARTED" and event["stop_id"] == stopped_train.current_stop_id
    ]
    if len(stable_started_events) != 1:
        raise AssertionError("Jog must not restart after completion")

    stopped_train.precise_jog_in_progress = False
    stopped_train.precise_jog_completed = False
    stopped_train.jog_state = main_gui.JOG_STATE_IDLE
    stopped_train.jog_used_for_current_stop = False
    stopped_train.jog_used_stop_key = None
    stopped_train.jog_used = False
    stopped_train.door_authorized = False
    stopped_train.pos = 97.9
    stopped_train.stop_target_pos = 100.25
    if stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must reject remaining distance outside the 2m jog window")

    stopped_train.pos = 98.25
    stopped_train.stop_target_pos = 100.25
    if stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must reject remaining distance equal to the 2m jog window")

    stopped_train.pos = 100.05
    stopped_train.stop_target_pos = 100.25
    if stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must reject stops already within stop accuracy tolerance")

    stopped_train.jog_state = main_gui.JOG_STATE_IDLE
    stopped_train.jog_used_for_current_stop = False
    stopped_train.jog_used_stop_key = None
    stopped_train.jog_used = False
    stopped_train.door_authorized = False
    stopped_train.emergency_stop = False
    stopped_train.emg_latch = False
    stopped_train.trip_mode = False
    stopped_train.emergency_recovery_hold = False
    stopped_train.zero_speed_detected = True
    stopped_train.speed = 0.0
    stopped_train.pos = 100.35
    stopped_train.stop_target_pos = 100.25
    if stopped_train.request_precise_jog():
        raise AssertionError("Precise jog must reject overrun stops")
    if stopped_train.jog_state != main_gui.JOG_STATE_FAILED_LOCKED:
        raise AssertionError("Overrun jog request must lock the current stop")
    if stopped_train.request_precise_jog():
        raise AssertionError("Failed locked jog must not restart for the same stop")

    stopped_train.pos = 99.94
    stopped_train.reported_pos = stopped_train.pos
    stopped_train.prev_pos = stopped_train.pos
    stopped_train.speed = 0.0
    stopped_train.filtered_speed = 0.0
    stopped_train.estimated_speed = 0.0
    stopped_train.vital_speed = 0.0
    stopped_train.stop_target_pos = 100.25
    stopped_train.eoa = 100.0
    stopped_train.jog_state = main_gui.JOG_STATE_IDLE
    stopped_train.jog_used_for_current_stop = False
    stopped_train.jog_used_stop_key = None
    stopped_train.jog_used = False
    stopped_train.emg_latch = False
    stopped_train.door_authorized = False
    if not stopped_train.request_precise_jog():
        raise AssertionError("Precise jog should accept a short remaining distance above tolerance")
    overshoot_time = stable_time + 100 * main_gui.DT
    for step_idx in range(80):
        stopped_train.receive_safe_packet(
            main_gui.SafeMovementPacket(
                eoa_m=100.0,
                tsr_kmh=80.0,
                variants={
                    "gradient": 0.0,
                    "next_speed_limit_kmh": 80.0,
                    "next_speed_limit_dist_m": float("inf"),
                },
            ),
            overshoot_time + step_idx * main_gui.DT,
        )
        stopped_train.step(overshoot_time + step_idx * main_gui.DT)
        if stopped_train.jog_state == main_gui.JOG_STATE_COMPLETED:
            break
    if stopped_train.pos > stopped_train.stop_target_pos + 1e-9:
        raise AssertionError("Precise jog must clamp without overshooting the stop target")

    stopped_train.jog_state = main_gui.JOG_STATE_IDLE
    stopped_train.jog_used_for_current_stop = False
    stopped_train.jog_used_stop_key = None
    stopped_train.jog_used = False
    stopped_train.door_authorized = False
    stopped_train.emergency_stop = False
    stopped_train.emg_latch = False
    stopped_train.trip_mode = False
    stopped_train.emergency_recovery_hold = False
    stopped_train.commanded_stop = True
    stopped_train.active_scheduled_stop = {"name": "SPEED_TRACE", "pos_m": 100.0, "dwell_s": 0.0}
    stopped_train.eoa = 99.75
    stopped_train.stop_target_pos = 100.0
    stopped_train.pos = 98.1
    stopped_train.reported_pos = stopped_train.pos
    stopped_train.prev_pos = stopped_train.pos
    stopped_train.speed = 0.0
    stopped_train.zero_speed_detected = True
    stopped_train.prev_accel = 0.0
    if not stopped_train.request_precise_jog():
        raise AssertionError("Precise jog should accept a 1.9m remaining speed trace case")
    speed_trace_time = overshoot_time + 100 * main_gui.DT
    trace_speeds = []
    zero_before_brake = False
    for step_idx in range(120):
        remaining_before = stopped_train.stop_target_pos - stopped_train.pos
        stopped_train.update_jog(main_gui.DT)
        trace_speeds.append(main_gui.ms_to_kmh(stopped_train.speed))
        if (
            stopped_train.jog_state == main_gui.JOG_STATE_ACTIVE
            and remaining_before > main_gui.STOP_ACCURACY_TOL_M
            and stopped_train.pos - stopped_train.jog_start_pos < 0.5 * stopped_train.jog_profile_distance_m
            and stopped_train.speed <= main_gui.STANDSTILL_SPEED_EPS
        ):
            zero_before_brake = True
        if stopped_train.jog_state != main_gui.JOG_STATE_ACTIVE:
            break
    if max(trace_speeds) < 0.9 * main_gui.JOG_SPEED_KMH:
        raise AssertionError("Jog speed should reach near configured JOG_SPEED_KMH when distance allows")
    if zero_before_brake:
        raise AssertionError("Jog speed must not be cut to zero before the braking half")
    starts_for_speed_trace = [
        event for event in stopped_train.event_records
        if event["event"] == "JOG_STARTED" and event["stop_id"] == stopped_train.current_stop_id
    ]
    if len(starts_for_speed_trace) != 1:
        raise AssertionError("Speed trace jog should start once")

    stopped_train.jog_state = main_gui.JOG_STATE_IDLE
    stopped_train.jog_used_for_current_stop = False
    stopped_train.jog_used_stop_key = None
    stopped_train.jog_used = False
    stopped_train.commanded_stop = True
    stopped_train.active_scheduled_stop = {"name": "WAIT_JOG", "pos_m": 100.0, "dwell_s": 0.0}
    stopped_train.eoa = 99.75
    stopped_train.stop_target_pos = 100.0
    stopped_train.pos = 98.8
    stopped_train.reported_pos = stopped_train.pos
    stopped_train.speed = main_gui.kmh_to_ms(0.55)
    stopped_train.filtered_speed = stopped_train.speed
    stopped_train.estimated_speed = stopped_train.speed
    stopped_train.vital_speed = main_gui.kmh_to_ms(0.50)
    stopped_train.hidden_curves["SBI"] = 0.0
    stopped_train.hidden_curves["EBI"] = main_gui.kmh_to_ms(5.0)
    stopped_train.curves["EBD"] = main_gui.kmh_to_ms(5.0)
    stopped_train.curves["P"] = main_gui.kmh_to_ms(40.0)
    stopped_train.curves["W"] = main_gui.kmh_to_ms(40.0)
    stopped_train.distance_to_eoa = 0.5
    stopped_train.release_active = False
    stopped_train.service_brake_latch = False
    stopped_train.emg_latch = False
    stopped_train.emergency_stop = False
    stopped_train.trip_mode = False
    before_events = len(stopped_train.event_records)
    stopped_train.step(speed_trace_time + 20.0)
    if stopped_train.atp_action == "SBI" or stopped_train.service_brake_latch:
        raise AssertionError("Low-speed station jog wait must not pulse ATP service only because SBI collapsed to zero")
    if any(event["event"] == "ATP_SERVICE_INTERVENTION" for event in list(stopped_train.event_records)[before_events:]):
        raise AssertionError("Low-speed station jog wait should not log ATP service intervention")

    stopped_train.commanded_stop = True
    stopped_train.emergency_recovery_hold = True
    stopped_train.standstill_required = True
    stopped_train.standstill_anchor_pos = stopped_train.pos - main_gui.STANDSTILL_DRIFT_M - 1.0
    stopped_train.resume_after_emergency()
    if stopped_train.standstill_required:
        raise AssertionError("Resume must release emergency standstill hold even while commanded_stop is set")
    report = reporting.build_simulation_report(gui_sim)
    if "analytics" not in report or "trains" not in report:
        raise AssertionError("simulation report should include analytics and train states")

    station_stop_scenario = normalize_scenario(
        {
            "display": {"track_min_m": 0, "track_max_m": 1200, "track_labels": [0, 500, 1000, 1200]},
            "track": {"segments": [{"start_m": 0, "end_m": 1200, "gradient": 0.0, "psr_kmh": 80}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500, "dwell_s": 10}],
            "trains": [{"id": "A1", "start_pos": 0}],
        }
    )
    station_sim = main_gui.Simulation(station_stop_scenario)
    reached_dwell = False
    dwell_seen = False
    for _ in range(3000):
        station_sim.step()
        station_train = station_sim.trains[0]
        assert_curve_hierarchy(station_train)
        if station_train.atp_action == "EBI":
            raise AssertionError("Normal station braking/release/docking must not create nuisance ATP emergency")
        if station_train.station_state == "DOCKING" and station_train.distance_to_stop_target() <= 25.0:
            if station_train.hidden_curves["SBI"] <= 0.0:
                raise AssertionError("Final 25m station docking must keep a valid SBI curve")
            if station_train.hidden_curves["EBI"] < station_train.hidden_curves["SBI"]:
                raise AssertionError("Final 25m station docking must keep EBI above SBI")
        if station_train.next_scheduled_stop_idx > 0 and station_train.dwell_remaining_s > 0.0:
            reached_dwell = True
            dwell_seen = True
            if not station_train.door_authorized:
                raise AssertionError("Station dwell should start only after door authorization")
            if station_train.station_state != "DWELLING":
                raise AssertionError("Station dwell must drive per-train station state to DWELLING")
            break
    if not reached_dwell:
        raise AssertionError("Station stop scenario should reach dwell without emergency")
    if not dwell_seen:
        raise AssertionError("Station stop scenario should show dwell countdown")

    multi_station_scenario = normalize_scenario(
        {
            "display": {"track_min_m": 0, "track_max_m": 1400, "track_labels": [0, 500, 1000, 1400]},
            "track": {"segments": [{"start_m": 0, "end_m": 1400, "gradient": 0.0, "psr_kmh": 65}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500, "length_m": 160, "capacity": 2, "dwell_s": 5}],
            "trains": [{"id": "L1", "start_pos": 360}, {"id": "L2", "start_pos": 180}, {"id": "L3", "start_pos": 0}],
        }
    )
    multi_station_sim = main_gui.Simulation(multi_station_scenario)
    seen_dwells: set[str] = set()
    seen_receive_route: set[str] = set()
    for _ in range(5000):
        multi_station_sim.step()
        for station_train in multi_station_sim.trains:
            if station_train.dwell_remaining_s > 0.0:
                seen_dwells.add(station_train.id)
            for event in station_train.pop_pending_events():
                if event["event"] == "STATION_EVENT" and event["reason"] == "RECEIVE_ROUTE_ASSIGNED":
                    seen_receive_route.add(station_train.id)
    if {"L1", "L2"} - seen_dwells:
        raise AssertionError("Following trains must still receive station routes and dwell at the scheduled station")
    if "L2" not in seen_receive_route:
        raise AssertionError("Follower train must receive an explicit station route before dwelling")

    danger_scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 400, "gradient": 0.0, "psr_kmh": 80}]},
            "trains": [{"id": "DANGER", "start_pos": 98.0}],
        }
    )
    danger_sim = main_gui.Simulation(danger_scenario)
    danger_train = danger_sim.trains[0]
    danger_train.commanded_stop = True
    danger_train.active_scheduled_stop = {"name": "DANGER_STOP", "pos_m": 100.0, "dwell_s": 0.0}
    danger_train.eoa = 99.75
    danger_train.stop_target_pos = 100.0
    danger_train.pos = 99.9
    danger_train.reported_pos = 99.9
    danger_train.speed = main_gui.kmh_to_ms(20.0)
    danger_train.filtered_speed = danger_train.speed
    danger_train.estimated_speed = danger_train.speed
    danger_train.vital_speed = danger_train.speed
    danger_train.receive_safe_packet(
        main_gui.SafeMovementPacket(
            eoa_m=99.75,
            tsr_kmh=80.0,
            variants={"gradient": 0.0, "next_speed_limit_kmh": 80.0, "next_speed_limit_dist_m": float("inf")},
        ),
        danger_sim.sim_time_s,
    )
    danger_train.step(danger_sim.sim_time_s)
    if danger_train.atp_action != "EBI":
        raise AssertionError("ATP must still emergency on real EBI/SvL danger")

    degraded_sim = main_gui.Simulation(degraded_scenario)
    for _ in range(260):
        degraded_sim.step()
    if degraded_sim.analytics["ebi_count"] <= 0:
        raise AssertionError("DCS mute degraded scenario should trigger fail-safe EBI")

    dcs_transition_scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 600, "gradient": 0.0, "psr_kmh": 60}]},
            "trains": [
                {
                    "id": "D1",
                    "start_pos": 0,
                    "drive_mode": "ATO",
                    "dcs_mute_windows": [{"start_s": 0.2, "end_s": 2.5}],
                }
            ],
        }
    )
    dcs_transition_sim = main_gui.Simulation(dcs_transition_scenario)
    for _ in range(20):
        dcs_transition_sim.step()
    dcs_train = dcs_transition_sim.trains[0]
    if dcs_train.drive_mode != "CMD25" or not dcs_train.dcs_degraded_requested:
        raise AssertionError("ATO train should request restricted recovery after DCS timeout")
    if not dcs_train.trip_mode or dcs_train.atp_action != "EBI":
        raise AssertionError("DCS timeout transition must remain fail-safe until recovered")

    print("smoke ok: scenarios and main_gui simulation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


