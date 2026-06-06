from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict


@dataclass(frozen=True)
class MovementAuthorityMessage:
    train_id: str
    eoa_m: float
    psr_kmh: float
    gradient: float = 0.0
    next_speed_limit_kmh: float = 0.0
    next_speed_limit_dist_m: float = float("inf")
    issued_time_s: float = 0.0
    reason: str = ""

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PositionReportMessage:
    train_id: str
    safe_front_m: float
    safe_rear_m: float
    speed_mps: float
    direction: str
    localization_uncertainty_m: float
    train_integrity_ok: bool
    timestamp_ms: int

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TrainStatusMessage:
    train_id: str
    position_m: float
    speed_mps: float
    mode: str
    atp_state: str
    ato_state: str
    door_state: str
    brake_state: str
    fault_flags: Dict[str, bool] = field(default_factory=dict)
    timestamp_ms: int = 0
    freshness: str = "FRESH"

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class AtsOperationCommandMessage:
    command: str
    train_id: str
    value: Any = None
    reason: str = ""

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DcsHealthMessage:
    red_state: str
    blue_state: str
    active_path: str
    latency_ms_avg: float
    packet_loss_count: int
    timeout_count: int
    freshness: str = "FRESH"

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)
