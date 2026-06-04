from __future__ import annotations

from collections import deque

from SUBSYSTEMS.signalling import SafeMovementPacket


class DCSWatchdog:
    """Fail-safe communication watchdog for the onboard CC."""

    def __init__(self, timeout_s: float, startup_grace_s: float):
        self.timeout_s = timeout_s
        self.startup_grace_s = startup_grace_s
        self.last_receive_time_s = 0.0

    def mark_received(self, now_s: float):
        self.last_receive_time_s = now_s

    def age_s(self, now_s: float) -> float:
        return max(0.0, now_s - self.last_receive_time_s)

    def packet_is_valid(self, now_s: float, packet_integrity_ok: bool) -> bool:
        if not packet_integrity_ok:
            return False
        return self.age_s(now_s) <= self.timeout_s or now_s <= self.startup_grace_s


class OnboardControlCenter:
    """Train-local CC that only consumes safe packets from ZC/DCS."""

    def __init__(self, train_id: str, timeout_s: float, startup_grace_s: float):
        self.train_id = train_id
        self.latest_packet = SafeMovementPacket(
            eoa_m=0.0,
            tsr_kmh=25.0,
            variants={
                "gradient": 0.0,
                "next_speed_limit_kmh": 0.0,
                "next_speed_limit_dist_m": float("inf"),
            },
        )
        self.pending_packets: deque[tuple[float, SafeMovementPacket]] = deque()
        self.watchdog = DCSWatchdog(timeout_s, startup_grace_s)
        self.latest_packet_issued_time_s = -1.0

    def receive_safe_packet(self, packet: SafeMovementPacket, arrival_time_s: float):
        self.pending_packets.append((arrival_time_s, packet))
        self.pending_packets = deque(sorted(self.pending_packets, key=lambda item: item[0]))

    def apply_to_train(self, train: object, now_s: float):
        while self.pending_packets and self.pending_packets[0][0] <= now_s:
            _, packet = self.pending_packets.popleft()
            if packet.issued_time_s < self.latest_packet_issued_time_s:
                continue
            self.latest_packet = packet
            self.latest_packet_issued_time_s = packet.issued_time_s
            self.watchdog.mark_received(now_s)
        packet = self.latest_packet
        train.eoa = packet.eoa_m
        train.psr_kmh = min(packet.tsr_kmh, 25.0) if train.drive_mode == "CMD25" else packet.tsr_kmh
        train.gradient = float(packet.variants.get("gradient", 0.0))
        train.limit_ahead_speed_kmh = float(packet.variants.get("next_speed_limit_kmh", 0.0))
        train.limit_ahead_dist = float(packet.variants.get("next_speed_limit_dist_m", float("inf")))
        packet_valid = (
            packet.tsr_kmh >= 0.0
            and train.limit_ahead_dist >= 0.0
            and not (packet.eoa_m != packet.eoa_m)
            and not (train.gradient != train.gradient)
        )
        train.safe_packet_age_s = self.watchdog.age_s(now_s)
        train.safe_packet_valid = self.watchdog.packet_is_valid(now_s, packet_valid)


__all__ = ["DCSWatchdog", "OnboardControlCenter", "SafeMovementPacket"]
