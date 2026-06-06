from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from CONFIG.scenario_loader import load_scenario
from SUBSYSTEMS.communication.messages import MovementAuthorityMessage, PositionReportMessage
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


def make_position_packet(sim: Simulation, train, seq: int, timestamp_ms: int | None = None, ttl_ms: int = 1000):
    timestamp_ms = int(sim.sim_time_s * 1000) if timestamp_ms is None else timestamp_ms
    payload = PositionReportMessage(
        train_id=train.id,
        safe_front_m=float(train.safe_front_end_pos),
        safe_rear_m=float(train.safe_rear_end_pos()),
        speed_mps=float(train.speed),
        direction="FORWARD",
        localization_uncertainty_m=float(train.effective_position_uncertainty_m()),
        train_integrity_ok=True,
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
    sim = Simulation(load_scenario())
    run_steps(sim, 20)
    train = sim.trains[0]
    assert train.safe_packet_valid, "normal MA flow should keep vital packet valid"
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

    sim.dcs_transport.set_path_state("RED", "LOST")
    sim._dispatch_safe_packets(with_delay=False)
    assert sim.dcs_transport.active_path == "BLUE", "RED failure should fail over to BLUE"
    assert any(event.result == "FAILOVER" for event in sim.dcs_transport.events), "failover should be logged"

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
    }
    sim_reports.last_valid_position_report[follow.id] = {
        "safe_front_m": 500.0,
        "safe_rear_m": 400.0,
        "speed_mps": 0.0,
        "localization_uncertainty_m": 1.0,
    }
    sim_reports.position_report_freshness[lead.id] = "FRESH"
    sim_reports.position_report_freshness[follow.id] = "FRESH"
    mal = sim_reports.zc.compute_mal(sim_reports._authority_trains_from_position_reports())
    assert mal[follow.id].protected_rear_m == 900.0, "ZC should compute MA from vital position report when enabled"

    sim_status = Simulation(load_scenario())
    run_steps(sim_status, 5)
    assert sim_status.ats_received_train_state, "TRAIN_STATUS should reach ATS through OPC UA-like"
    status_event = next(event for event in reversed(sim_status.dcs_transport.events) if event.protocol == "OPCUA_SUPERVISION" and event.msg_type == "TRAIN_STATUS" and event.details)
    assert status_event.details["frame"]["encryption_enabled"], "OPC UA-like TRAIN_STATUS should be encrypted"
    status_train = sim_status.trains[0]
    vital_before = status_train.safe_packet_valid
    sim_status.dcs_transport.set_fault("opcua_loss", True)
    run_steps(sim_status, 35)
    assert sim_status.ats_train_freshness[status_train.id] in {"STALE", "LOST"}, "OPC UA loss should stale/lost ATS status"
    assert status_train.safe_packet_valid == vital_before, "OPC UA loss must not affect ATP vital safety while RaSTA is alive"

    print("communication-upgrade-smoke-ok")


if __name__ == "__main__":
    main()
