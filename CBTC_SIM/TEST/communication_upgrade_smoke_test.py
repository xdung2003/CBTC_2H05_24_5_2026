from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
PACKAGE_PARENT = ROOT.parent
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from CONFIG.scenario_loader import load_scenario
from SUBSYSTEMS.communication.messages import AtsOperationCommandMessage, MovementAuthorityMessage, PositionReportMessage
from SUBSYSTEMS.communication.opcua import OpcUaSupervisionFrame
from SUBSYSTEMS.communication.rasta import VitalSafePacket
from SUBSYSTEMS.runtime import Simulation


def make_packet(sim: Simulation, train, seq: int, eoa_m: float, timestamp_ms: int | None = None, ttl_ms: int = 1000):
    timestamp_ms = int(sim.sim_time_s * 1000) if timestamp_ms is None else timestamp_ms
    payload = MovementAuthorityMessage(
        train_id=train.id,
        eoa_m=eoa_m,
        psr_kmh=70.0,
        gradient=0.0,
        next_speed_limit_kmh=0.0,
        next_speed_limit_dist_m=float("inf"),
        issued_time_s=sim.sim_time_s,
        reason="test",
    ).to_payload()
    return VitalSafePacket.create(
        source_id="ZC_01",
        destination_id=train.id,
        session_id=f"ZC_01:{train.id}",
        message_type="MA_UPDATE",
        sequence_number=seq,
        timestamp_ms=timestamp_ms,
        ttl_ms=ttl_ms,
        payload=payload,
        key_id="SIM_KEY_01",
        secret=train.cc.vital_session.secret,
    )


def make_position_packet(
    sim: Simulation,
    train,
    seq: int,
    timestamp_ms: int | None = None,
    ttl_ms: int = 1000,
    train_integrity_ok: bool = True,
):
    timestamp_ms = int(sim.sim_time_s * 1000) if timestamp_ms is None else timestamp_ms
    payload = PositionReportMessage(
        train_id=train.id,
        safe_front_m=float(train.safe_front_end_pos),
        safe_rear_m=float(train.safe_rear_end_pos()),
        speed_mps=float(train.speed),
        direction="FORWARD",
        localization_uncertainty_m=float(train.effective_position_uncertainty_m()),
        train_integrity_ok=train_integrity_ok,
        timestamp_ms=timestamp_ms,
    ).to_payload()
    return VitalSafePacket.create(
        source_id=train.id,
        destination_id="ZC_01",
        session_id=f"{train.id}:ZC_01",
        message_type="POSITION_REPORT",
        sequence_number=seq,
        timestamp_ms=timestamp_ms,
        ttl_ms=ttl_ms,
        payload=payload,
        key_id="SIM_KEY_01",
        secret=sim.zc_vital_sessions[train.id].secret,
    )


def run_steps(sim: Simulation, count: int):
    for _ in range(count):
        sim.step()


def main():
    import CBTC_SIM  # noqa: F401

    default_scenario = load_scenario()
    assert default_scenario["communication"]["use_vital_position_report_for_zc"], "ZC must default to DCS-delivered POSITION_REPORT"
    sim = Simulation(default_scenario)
    assert not hasattr(sim.zc, "trains"), "ZC must not retain direct Train objects"
    assert len(sim.trains[0].cc.channels) == 2, "each train should have redundant dual CC channels"
    assert sim.trains[0].cc.redundancy_state in {"DUAL_ACTIVE", "DEGRADED"}, "dual CC should expose redundancy state"
    run_steps(sim, 20)
    train = sim.trains[0]
    assert train.safe_packet_valid, "normal MA flow should keep vital packet valid"

    sim_bad_ats = Simulation(load_scenario())
    target = sim_bad_ats.trains[0]
    payload = AtsOperationCommandMessage(
        command="EMERGENCY_STOP",
        train_id=target.id,
        value=None,
        reason="bad_hmac_test",
    ).to_payload()
    bad_packet = VitalSafePacket.create(
        source_id="ATS",
        destination_id="OPERATIONS",
        session_id="ATS:OPERATIONS",
        message_type="ATS_OPERATION_COMMAND",
        sequence_number=1,
        timestamp_ms=int(sim_bad_ats.sim_time_s * 1000),
        ttl_ms=1500,
        payload=payload,
        key_id="SIM_KEY_01",
        secret="cbtc-sim-shared-secret",
    ).with_hmac_corruption()
    sim_bad_ats.pending_ats_operation_packets.append((sim_bad_ats.sim_time_s, bad_packet))
    sim_bad_ats._process_ats_operation_frames()
    assert not target.emergency_stop and not target.trip_mode, "bad HMAC ATS command must not affect train state"
    assert any(
        event.msg_type == "MA_UPDATE" and event.result == "ACCEPTED" and event.destination_id == train.id
        for event in sim.dcs_transport.events
    ), "normal MA_UPDATE should be accepted"

    original_eoa = train.eoa
    expired = make_packet(sim, train, 10_000, original_eoa + 999.0, timestamp_ms=int((sim.sim_time_s - 3.0) * 1000), ttl_ms=100)
    train.receive_vital_packet(expired, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    assert train.eoa == original_eoa, "expired packet must not update EOA"
    assert train.vital_packet_result == "TIMEOUT", "expired packet should be rejected as TIMEOUT"

    valid = make_packet(sim, train, 10_001, original_eoa + 111.0)
    assert valid.encryption_enabled and valid.encrypted_payload, "MA_UPDATE should carry encrypted payload"
    train.receive_vital_packet(valid, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    accepted_eoa = train.eoa
    assert accepted_eoa == original_eoa + 111.0, "encrypted MA_UPDATE should decrypt and update EOA"

    decrypt_bad = make_packet(sim, train, 10_004, accepted_eoa + 444.0).with_decrypt_error(train.cc.vital_session.secret)
    train.receive_vital_packet(decrypt_bad, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    assert train.eoa == accepted_eoa, "decrypt error must not update EOA"
    assert train.vital_packet_result == "DECRYPT_ERROR", "bad encrypted payload should be rejected as DECRYPT_ERROR"

    train.receive_vital_packet(valid, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    assert train.eoa == accepted_eoa, "replay packet must not update EOA"
    assert train.vital_packet_result == "REPLAY", "duplicate packet should be rejected as REPLAY"

    crc_bad = make_packet(sim, train, 10_002, accepted_eoa + 222.0).with_crc_corruption()
    train.receive_vital_packet(crc_bad, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    assert train.eoa == accepted_eoa, "CRC-corrupt packet must not update EOA"
    assert train.vital_packet_result == "CRC_ERROR", "CRC-corrupt packet should be rejected"

    hmac_bad = make_packet(sim, train, 10_003, accepted_eoa + 333.0).with_hmac_corruption()
    train.receive_vital_packet(hmac_bad, sim.sim_time_s)
    train.cc.apply_to_train(train, sim.sim_time_s)
    assert train.eoa == accepted_eoa, "HMAC-corrupt packet must not update EOA"
    assert train.vital_packet_result == "HMAC_ERROR", "HMAC-corrupt packet should be rejected"

    failover_count_before = sum(1 for event in sim.dcs_transport.events if event.result == "FAILOVER")
    sim.dcs_transport.set_path_state("RED", "LOST")
    sim._dispatch_safe_packets(with_delay=False)
    assert sim.dcs_transport.active_path == "BLUE", "RED failure should fail over to BLUE"
    assert any(event.result == "FAILOVER" for event in sim.dcs_transport.events), "failover should be logged"
    failover_count_after = sum(1 for event in sim.dcs_transport.events if event.result == "FAILOVER")
    assert failover_count_after - failover_count_before == 1, "failover should be logged exactly once per path switch"

    sim_ber = Simulation(load_scenario())
    run_steps(sim_ber, 20)
    ber_train = sim_ber.trains[0]
    before_ber_eoa = ber_train.eoa
    ber_train.cc.pending_vital_packets.clear()
    sim_ber.dcs_transport.set_fault("ber_corruption", True)
    sim_ber._dispatch_safe_packets(with_delay=False)
    ber_train.cc.apply_to_train(ber_train, sim_ber.sim_time_s + 0.5)
    assert ber_train.eoa == before_ber_eoa, "BER-corrupted vital packet must not update EOA"
    assert ber_train.vital_packet_result in {"CRC_ERROR", "HMAC_ERROR"}, "BER corruption should fail CRC/HMAC"
    inspector_event = next(event for event in reversed(sim_ber.dcs_transport.events) if event.details)
    assert {"route", "message", "frame", "radio", "chain"}.issubset(inspector_event.details), "packet inspector details should contain all packet layers"

    sim_loss = Simulation(load_scenario())
    run_steps(sim_loss, 20)
    sim_loss.dcs_transport.set_fault("radio_coverage_loss", True)
    run_steps(sim_loss, 20)
    loss_train = sim_loss.trains[0]
    assert not loss_train.safe_packet_valid, "radio loss should make vital packet invalid after watchdog timeout"
    assert loss_train.trip_mode or loss_train.emg_latch, "vital timeout should trigger ATP fail-safe"

    sim_opc = Simulation(load_scenario())
    run_steps(sim_opc, 20)
    opc_train = sim_opc.trains[0]
    before_valid = opc_train.safe_packet_valid
    frame = OpcUaSupervisionFrame(
        request_id="REQ_1",
        response_id="",
        source_id="ATS",
        destination_id=opc_train.id,
        method_name="DWELL_EXTEND",
        timestamp_ms=int(sim_opc.sim_time_s * 1000),
        timeout_ms=500,
        retry_count=0,
        encrypted_flag=True,
        certificate_id="SIM_CERT",
        payload={"seconds": 15},
    )
    sim_opc.dcs_transport.transport_supervision(frame, sim_opc.sim_time_s)
    run_steps(sim_opc, 5)
    assert opc_train.safe_packet_valid == before_valid, "OPC UA-like supervision should not break ATP safety while vital is alive"

    forbidden = OpcUaSupervisionFrame(
        request_id="REQ_2",
        response_id="",
        source_id="ATS",
        destination_id=opc_train.id,
        method_name="MA_UPDATE",
        timestamp_ms=int(sim_opc.sim_time_s * 1000),
        timeout_ms=500,
        retry_count=0,
        encrypted_flag=True,
        certificate_id="SIM_CERT",
        payload={"eoa_m": 99999},
    )
    delivered, _arrival, event = sim_opc.dcs_transport.transport_supervision(forbidden, sim_opc.sim_time_s)
    assert delivered is None and event.result == "REJECTED", "OPC UA-like must not carry MA/EOA"

    sim_pos = Simulation(load_scenario())
    run_steps(sim_pos, 3)
    pos_train = sim_pos.trains[0]
    pos_packet = make_position_packet(sim_pos, pos_train, 50_000)
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, pos_packet))
    sim_pos._process_zc_position_reports()
    assert pos_train.id in sim_pos.last_valid_position_report, "accepted POSITION_REPORT should update ZC store"
    assert pos_packet.encryption_enabled and pos_packet.encrypted_payload, "POSITION_REPORT should be encrypted"
    stored_front = sim_pos.last_valid_position_report[pos_train.id]["safe_front_m"]

    expired_pos = make_position_packet(
        sim_pos,
        pos_train,
        50_001,
        timestamp_ms=int((sim_pos.sim_time_s - 3.0) * 1000),
        ttl_ms=100,
    )
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, expired_pos))
    sim_pos._process_zc_position_reports()
    assert sim_pos.last_valid_position_report[pos_train.id]["safe_front_m"] == stored_front, "expired POSITION_REPORT must not update ZC store"

    corrupt_pos = make_position_packet(sim_pos, pos_train, 50_002).with_hmac_corruption()
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, corrupt_pos))
    sim_pos._process_zc_position_reports()
    assert sim_pos.last_valid_position_report[pos_train.id]["safe_front_m"] == stored_front, "corrupt POSITION_REPORT must not update ZC store"

    replay_pos = make_position_packet(sim_pos, pos_train, 50_003)
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, replay_pos))
    sim_pos._process_zc_position_reports()
    accepted_replay_front = sim_pos.last_valid_position_report[pos_train.id]["safe_front_m"]
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, replay_pos))
    sim_pos._process_zc_position_reports()
    assert sim_pos.last_valid_position_report[pos_train.id]["safe_front_m"] == accepted_replay_front, "replay POSITION_REPORT must not update ZC store"

    out_of_order = make_position_packet(sim_pos, pos_train, 50_002)
    sim_pos.pending_zc_position_packets.append((sim_pos.sim_time_s, out_of_order))
    sim_pos._process_zc_position_reports()
    assert any(event.result == "OUT_OF_ORDER" for event in sim_pos.dcs_transport.events), "out-of-order vital packet should be rejected"

    sim_integrity = Simulation(load_scenario())
    run_steps(sim_integrity, 3)
    integrity_train = max(sim_integrity.trains, key=lambda item: item.reported_pos)
    integrity_train.pos = 800.0
    integrity_train.reported_pos = 800.0
    integrity_train.safe_front_end_pos = 800.0
    integrity_train.speed = 0.0
    integrity_train.set_fault("INTEGRITY", True, sim_integrity.sim_time_s)
    unsafe_report = sim_integrity._position_report_message(integrity_train)
    assert not unsafe_report.train_integrity_ok, "POSITION_REPORT should expose virtual integrity line loss"
    integrity_status = sim_integrity._train_status_message(integrity_train)
    assert integrity_status.fault_flags["INTEGRITY"], "TRAIN_STATUS should report train integrity loss"
    assert not integrity_status.fault_flags["DCS"], "train integrity loss should not be mislabeled as a DCS fault"
    unsafe_packet = sim_integrity._vital_position_packet(integrity_train)
    sim_integrity.pending_zc_position_packets.append((sim_integrity.sim_time_s, unsafe_packet))
    sim_integrity._process_zc_position_reports()
    assert sim_integrity.position_report_freshness[integrity_train.id] == "UNSAFE", "ZC should mark failed train integrity as UNSAFE"
    unsafe_packets = sim_integrity.zc.build_safe_packets(
        sim_integrity.track_profile,
        sim_integrity.tsr_zones,
        sim_integrity.track_end_m,
        {},
        trains_for_authority=sim_integrity._authority_trains_from_position_reports(),
    )
    assert integrity_train.id not in unsafe_packets, "ZC must not issue MA to a failed-integrity train"
    assert unsafe_packets, "ZC should keep issuing MA to healthy trains protected by the unsafe train report"
    zc_payload = sim_integrity._zc_status_message().to_payload()
    assert zc_payload["virtual_obstacles"], "ZC_STATUS should publish virtual obstacle blocks for ATS display"
    virtual_obstacle = next(item for item in zc_payload["virtual_obstacles"] if item["train_id"] == integrity_train.id)
    occupied_span_m = float(virtual_obstacle["occupied_end_m"]) - float(virtual_obstacle["occupied_start_m"])
    protected_span_m = float(virtual_obstacle["end_m"]) - float(virtual_obstacle["start_m"])
    assert occupied_span_m >= integrity_train.length, "virtual obstacle occupied core should cover the failed train"
    assert protected_span_m >= occupied_span_m, "virtual obstacle display should not be smaller than its occupied core"
    run_steps(sim_integrity, 20)
    healthy_dcs_faults = [
        train.id
        for train in sim_integrity.trains
        if train.id != integrity_train.id and sim_integrity._train_status_message(train).fault_flags["DCS"]
    ]
    assert not healthy_dcs_faults, "one failed-integrity train must not trigger DCS faults for the whole line"

    sim_lost_report = Simulation(load_scenario())
    run_steps(sim_lost_report, 6)
    non_comm_train = max(sim_lost_report.trains, key=lambda item: item.reported_pos)
    sim_lost_report.position_report_freshness[non_comm_train.id] = "LOST"
    sim_lost_report.zc.mark_position_report_freshness(non_comm_train.id, "LOST")
    lost_report_packets = sim_lost_report.zc.build_safe_packets(
        sim_lost_report.track_profile,
        sim_lost_report.tsr_zones,
        sim_lost_report.track_end_m,
        {},
        trains_for_authority=sim_lost_report._authority_trains_from_position_reports(),
    )
    assert non_comm_train.id not in lost_report_packets, "ZC must not issue MA to a non-communicating train"
    assert lost_report_packets, "ZC should keep healthy trains moving behind a last-known non-communicating obstacle"

    scenario_reports = load_scenario()
    scenario_reports["communication"]["use_vital_position_report_for_zc"] = True
    sim_reports = Simulation(scenario_reports)
    lead, follow = sim_reports.trains[0], sim_reports.trains[1]
    lead.protection_zone_id = None
    follow.protection_zone_id = None
    lead.protection_lane = 0
    follow.protection_lane = 0
    sim_reports.last_valid_position_report[lead.id] = {
        "safe_front_m": 1000.0,
        "safe_rear_m": 900.0,
        "speed_mps": 0.0,
        "localization_uncertainty_m": 1.0,
        "train_integrity_ok": True,
    }
    sim_reports.last_valid_position_report[follow.id] = {
        "safe_front_m": 500.0,
        "safe_rear_m": 400.0,
        "speed_mps": 0.0,
        "localization_uncertainty_m": 1.0,
        "train_integrity_ok": True,
    }
    sim_reports.position_report_freshness[lead.id] = "FRESH"
    sim_reports.position_report_freshness[follow.id] = "FRESH"
    mal = sim_reports.zc.compute_mal(sim_reports._authority_trains_from_position_reports())
    assert mal[follow.id].protected_rear_m == 900.0, "ZC should compute MA from vital position report when enabled"
    sim_reports.last_valid_position_report[lead.id]["train_integrity_ok"] = False
    sim_reports.position_report_freshness[lead.id] = "UNSAFE"
    obstacle_packets = sim_reports.zc.build_safe_packets(
        sim_reports.track_profile,
        sim_reports.tsr_zones,
        sim_reports.track_end_m,
        {},
        trains_for_authority=[
            view
            for view in sim_reports._authority_trains_from_position_reports()
            if view.id in {lead.id, follow.id}
        ],
    )
    assert lead.id not in obstacle_packets, "ZC must not issue MA to the unsafe obstacle train"
    assert obstacle_packets[follow.id].variants.get("ma_reason") == "OBSTACLE_PROTECTION", "ZC should label MA cutbacks caused by protected obstacles"
    sim_reports.last_valid_position_report[lead.id]["train_integrity_ok"] = True
    sim_reports.position_report_freshness[lead.id] = "FRESH"
    sim_reports.position_report_freshness[lead.id] = "LOST"
    no_report_packets = sim_reports.zc.build_safe_packets(
        sim_reports.track_profile,
        sim_reports.tsr_zones,
        sim_reports.track_end_m,
        {},
        trains_for_authority=sim_reports._authority_trains_from_position_reports(),
    )
    assert no_report_packets == {}, "ZC must not issue new MA when any POSITION_REPORT is not fresh"

    sim_status = Simulation(load_scenario())
    run_steps(sim_status, 5)
    assert sim_status.ats_received_train_state, "TRAIN_STATUS should reach ATS through OPC UA-like"
    status_event = next(event for event in reversed(sim_status.dcs_transport.events) if event.protocol == "OPCUA_SUPERVISION" and event.msg_type == "TRAIN_STATUS" and event.details)
    assert status_event.details["frame"]["encryption_enabled"], "OPC UA-like TRAIN_STATUS should be encrypted"
    status_train = sim_status.trains[0]
    vital_before = status_train.safe_packet_valid
    displayed_pos_before_loss = sim_status.ats_received_train_state[status_train.id]["position_m"]
    sim_status.dcs_transport.set_fault("opcua_loss", True)
    sim_status.pending_ats_status_frames.clear()
    run_steps(sim_status, 35)
    assert sim_status.ats_train_freshness[status_train.id] in {"STALE", "LOST"}, "OPC UA loss should stale/lost ATS status"
    assert (
        sim_status.ats_received_train_state[status_train.id]["position_m"] == displayed_pos_before_loss
    ), "ATS must not refresh train position by reading Train directly when OPC UA is lost"
    assert status_train.safe_packet_valid == vital_before, "OPC UA loss must not affect ATP vital safety while RaSTA is alive"

    sim_radio_status = Simulation(load_scenario())
    run_steps(sim_radio_status, 5)
    radio_status_train = sim_radio_status.trains[0]
    assert sim_radio_status.ats_train_freshness[radio_status_train.id] == "FRESH", "TRAIN_STATUS should initially be fresh"
    sim_radio_status.dcs_transport.set_fault("radio_coverage_loss", True)
    sim_radio_status.pending_ats_status_frames.clear()
    run_steps(sim_radio_status, 35)
    assert sim_radio_status.ats_train_freshness[radio_status_train.id] in {"STALE", "LOST"}, "radio loss should stale/lost TRAIN_STATUS at ATS"

    sim_lock = Simulation(load_scenario())
    station_idx = 0
    sim_lock.station_route_states[station_idx]["lock_remaining_s"] = 5.0
    can_accept, reason = sim_lock.can_accept_train(station_idx, sim_lock.trains[0])
    assert not can_accept and reason == "TURNOUT_LOCKING", "station route must wait during 5s turnout lock"

    print("communication-upgrade-smoke-ok")


if __name__ == "__main__":
    main()
