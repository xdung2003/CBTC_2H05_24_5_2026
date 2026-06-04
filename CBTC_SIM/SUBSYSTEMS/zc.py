from __future__ import annotations

from typing import Dict, List, Tuple

from SUBSYSTEMS.signalling import (
    AuthorityManager,
    MovementAuthorityLimit,
    SafeMovementPacket,
    STOP_SVL_OFFSET_M,
    get_track_info,
    next_lower_limit,
)
from SUBSYSTEMS.control_common import STOP_ACCURACY_TOL_M


class ZoneController:
    """Wayside/ZC logic that prepares safe packets for onboard CCs."""

    def __init__(self, trains: List[Train], track_end_m: float, block_mode: str = "moving_block", fixed_blocks: List[Dict[str, float]] | None = None):
        self.trains = trains
        self.block_mode = block_mode
        self.fixed_blocks = fixed_blocks or []
        self.authority_manager = AuthorityManager(track_end_m)

    def compute_mal(self) -> Dict[str, MovementAuthorityLimit]:
        if self.block_mode == "fixed_block":
            mal_map: Dict[str, MovementAuthorityLimit] = {}
            occupancy = self.fixed_block_occupancy()
            reservations: Dict[str, str] = {}
            for train in sorted(self.trains, key=lambda item: item.reported_pos, reverse=True):
                mal_m = self._fixed_block_limit_for_train(train, occupancy, reservations)
                self._reserve_fixed_blocks_for_authority(train, mal_m, reservations, occupancy)
                mal_map[train.id] = MovementAuthorityLimit(
                    train_id=train.id,
                    mal_m=mal_m,
                    protected_rear_m=mal_m + STOP_SVL_OFFSET_M,
                    follower_braking_m=0.0,
                    follower_projection_m=0.0,
                    safety_margin_m=0.0,
                    overlap_m=0.0,
                    reason="FIXED_BLOCK",
                )
            return mal_map
        return self.authority_manager.compute_mal(self.trains)

    def fixed_block_occupancy(self) -> Dict[str, List[str]]:
        occupancy = {str(block["id"]): [] for block in self.fixed_blocks}
        for block in self.fixed_blocks:
            start = float(block["start_m"])
            end = float(block["end_m"])
            block_id = str(block["id"])
            for train in self.trains:
                if train.safe_rear_end_pos() < end and train.reported_pos > start:
                    occupancy[block_id].append(train.id)
        return occupancy

    def _fixed_block_for_pos(self, pos_m: float) -> Dict[str, float] | None:
        if self.fixed_blocks and pos_m < float(self.fixed_blocks[0]["start_m"]):
            return self.fixed_blocks[0]
        for block in self.fixed_blocks:
            start_m = float(block["start_m"])
            end_m = float(block["end_m"])
            if start_m <= pos_m < end_m or (
                abs(pos_m - self.authority_manager.track_end_m) <= 1e-6
                and abs(end_m - self.authority_manager.track_end_m) <= 1e-6
            ):
                return block
            if pos_m < start_m:
                return block
        return None

    def _fixed_block_for_train_from_occupancy(
        self,
        train: Train,
        occupancy: Dict[str, List[str]],
    ) -> Dict[str, float] | None:
        occupied_blocks = [
            block
            for block in self.fixed_blocks
            if train.id in occupancy.get(str(block["id"]), [])
        ]
        if occupied_blocks:
            return max(occupied_blocks, key=lambda item: int(item["index"]))
        return self._fixed_block_for_pos(float(train.reported_pos))

    def _fixed_block_limit_for_train(
        self,
        train: Train,
        occupancy: Dict[str, List[str]] | None = None,
        reserved_blocks: Dict[str, str] | None = None,
    ) -> float:
        zone_id = getattr(train, "protection_zone_id", None)
        if (
            isinstance(zone_id, str)
            and zone_id.startswith("STATION:")
            and getattr(train, "active_scheduled_stop", None) is not None
            and getattr(train, "commanded_stop", False)
        ):
            return self.authority_manager.track_end_m
        occupancy = occupancy if occupancy is not None else self.fixed_block_occupancy()
        current_block = self._fixed_block_for_train_from_occupancy(train, occupancy)
        if current_block is None:
            return self.authority_manager.track_end_m
        reserved_blocks = reserved_blocks if reserved_blocks is not None else {}
        current_index = int(current_block["index"])
        candidate_blocks = [
            block for block in self.fixed_blocks
            if current_index <= int(block["index"]) <= current_index + 1
        ]
        candidate_blocks.sort(key=lambda item: int(item["index"]))
        authority_end_m = float(current_block["end_m"])
        for block in candidate_blocks:
            block_id = str(block["id"])
            occupants = [train_id for train_id in occupancy.get(block_id, []) if train_id != train.id]
            reserved_by = reserved_blocks.get(block_id)
            blocked = bool(occupants) or (reserved_by is not None and reserved_by != train.id)
            if not blocked:
                authority_end_m = float(block["end_m"])
                continue
            return float(block["start_m"]) - STOP_SVL_OFFSET_M
        if authority_end_m >= self.authority_manager.track_end_m - STOP_ACCURACY_TOL_M:
            return self.authority_manager.track_end_m
        return authority_end_m - STOP_SVL_OFFSET_M

    def _reserve_fixed_blocks_for_authority(
        self,
        train: Train,
        mal_m: float,
        reserved_blocks: Dict[str, str],
        occupancy: Dict[str, List[str]] | None = None,
    ) -> None:
        occupancy = occupancy if occupancy is not None else self.fixed_block_occupancy()
        current_block = self._fixed_block_for_train_from_occupancy(train, occupancy)
        if current_block is None:
            return
        current_index = int(current_block["index"])
        authority_svl_m = mal_m + STOP_SVL_OFFSET_M
        for block in self.fixed_blocks:
            block_index = int(block["index"])
            if block_index < current_index:
                continue
            if float(block["start_m"]) > authority_svl_m + STOP_ACCURACY_TOL_M:
                break
            reserved_blocks.setdefault(str(block["id"]), train.id)

    def compute_eoa(self) -> Dict[str, float]:
        return {train_id: mal.mal_m for train_id, mal in self.compute_mal().items()}

    def build_safe_packets(
        self,
        track_profile: List[Tuple[float, float, float, float]],
        tsr_zones: List[Dict[str, float]],
        track_end_m: float,
        stop_eoa_map: Dict[str, float],
    ) -> Dict[str, SafeMovementPacket]:
        mal_map = {} if self.block_mode == "fixed_block" else self.compute_mal()
        packets: Dict[str, SafeMovementPacket] = {}
        fixed_occupancy = self.fixed_block_occupancy() if self.block_mode == "fixed_block" else {}
        fixed_reservations: Dict[str, str] = {}
        packet_order = sorted(self.trains, key=lambda item: item.reported_pos, reverse=True)
        for train in packet_order:
            pos_for_limits = train.reported_pos
            if pos_for_limits < 0:
                gradient, psr = get_track_info(track_profile, track_profile[0][0])
            else:
                gradient, base_psr = get_track_info(track_profile, pos_for_limits)
                psr = base_psr
            for zone in tsr_zones:
                if zone["start"] <= pos_for_limits <= zone["end"]:
                    psr = min(psr, zone["speed"])
            next_speed, next_dist = next_lower_limit(track_profile, pos_for_limits, psr, tsr_zones)
            mal = mal_map.get(train.id)
            mal_m = mal.mal_m if mal is not None else track_end_m
            if self.block_mode == "fixed_block":
                mal_m = self._fixed_block_limit_for_train(train, fixed_occupancy, fixed_reservations)
            stop_eoa = stop_eoa_map.get(train.id)
            # Station stop/holding EOA is a constraint on top of moving-block MA,
            # not a replacement for leader protection on the open line.
            packet_eoa = min(mal_m, stop_eoa if stop_eoa is not None else track_end_m)
            if self.block_mode == "fixed_block":
                self._reserve_fixed_blocks_for_authority(train, packet_eoa, fixed_reservations, fixed_occupancy)
            packets[train.id] = SafeMovementPacket(
                eoa_m=packet_eoa,
                tsr_kmh=psr,
                variants={
                    "gradient": gradient,
                    "next_speed_limit_kmh": next_speed,
                    "next_speed_limit_dist_m": next_dist,
                },
            )
        return packets
