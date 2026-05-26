from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Literal


DispatchDecision = Literal["HOLD", "RELEASE_DISPATCH"]


@dataclass(frozen=True)
class HeadwayDecision:
    train_id: str
    decision: DispatchDecision
    reason: str
    target_headway_s: float
    planned_dispatch_time_s: float | None
    dispatch_delay_s: float


@dataclass
class HeadwayStats:
    target_headway_s: float = 0.0
    actual_headways_s: List[float] = field(default_factory=list)
    actual_headway_pairs: List[Dict[str, Any]] = field(default_factory=list)
    dispatch_delays_s: List[float] = field(default_factory=list)
    dispatch_times_s: Dict[str, float] = field(default_factory=dict)
    release_times_s: Dict[str, float] = field(default_factory=dict)

    @property
    def min_actual_headway_s(self) -> float | None:
        return min(self.actual_headways_s) if self.actual_headways_s else None

    @property
    def avg_actual_headway_s(self) -> float | None:
        if not self.actual_headways_s:
            return None
        return sum(self.actual_headways_s) / len(self.actual_headways_s)

    @property
    def max_actual_headway_s(self) -> float | None:
        return max(self.actual_headways_s) if self.actual_headways_s else None

    @property
    def avg_dispatch_delay_s(self) -> float | None:
        if not self.dispatch_delays_s:
            return None
        return sum(self.dispatch_delays_s) / len(self.dispatch_delays_s)


class HeadwayManager:
    """Operational dispatch regulator.

    This manager only decides whether a staged train may be released from the
    dispatch gate. It never grants authority, never computes EOA and never
    overrides ATP.
    """

    def __init__(
        self,
        mode: str = "off",
        target_headway_s: float = 0.0,
        timetable_s: List[float] | None = None,
        adaptive_min_headway_s: float | None = None,
        adaptive_tsr_extra_s: float = 20.0,
        adaptive_min_gap_m: float = 0.0,
    ):
        self.mode = str(mode or "off").lower()
        self.target_headway_s = max(0.0, float(target_headway_s))
        self.timetable_s = sorted(float(item) for item in (timetable_s or []))
        self.adaptive_min_headway_s = (
            max(0.0, float(adaptive_min_headway_s))
            if adaptive_min_headway_s is not None
            else self.target_headway_s
        )
        self.adaptive_tsr_extra_s = max(0.0, float(adaptive_tsr_extra_s))
        self.adaptive_min_gap_m = max(0.0, float(adaptive_min_gap_m))
        self.released_train_ids: set[str] = set()
        self.stats = HeadwayStats(target_headway_s=self.nominal_target_headway_s())

    @classmethod
    def from_scenario(cls, scenario: Dict[str, Any]) -> "HeadwayManager":
        cfg = scenario.get("headway", {}) or {}
        if not isinstance(cfg, dict):
            cfg = {}
        adaptive = cfg.get("adaptive", {}) or {}
        if not isinstance(adaptive, dict):
            adaptive = {}
        return cls(
            mode=str(cfg.get("mode", "off")),
            target_headway_s=float(cfg.get("target_headway_s", cfg.get("fixed_headway_s", 0.0))),
            timetable_s=list(cfg.get("timetable_s", cfg.get("timetable", [])) or []),
            adaptive_min_headway_s=adaptive.get("min_headway_s", cfg.get("adaptive_min_headway_s")),
            adaptive_tsr_extra_s=float(adaptive.get("tsr_extra_s", cfg.get("adaptive_tsr_extra_s", 20.0))),
            adaptive_min_gap_m=float(adaptive.get("min_gap_m", cfg.get("adaptive_min_gap_m", 0.0))),
        )

    def nominal_target_headway_s(self) -> float:
        if self.mode == "timetable" and len(self.timetable_s) >= 2:
            gaps = [right - left for left, right in zip(self.timetable_s, self.timetable_s[1:]) if right > left]
            if gaps:
                return sum(gaps) / len(gaps)
        if self.mode == "adaptive":
            return self.adaptive_min_headway_s
        return self.target_headway_s

    def enabled(self) -> bool:
        return self.mode in {"fixed", "timetable", "adaptive"} and (
            self.nominal_target_headway_s() > 0.0 or bool(self.timetable_s)
        )

    def _planned_time_for_sequence(self, sequence_index: int) -> float | None:
        if self.mode == "timetable":
            if 0 <= sequence_index < len(self.timetable_s):
                return self.timetable_s[sequence_index]
            if self.timetable_s and self.nominal_target_headway_s() > 0.0:
                extra = sequence_index - len(self.timetable_s) + 1
                return self.timetable_s[-1] + extra * self.nominal_target_headway_s()
            return None
        target = self.nominal_target_headway_s()
        return sequence_index * target if target > 0.0 else None

    def _effective_target_headway_s(self, tsr_active: bool) -> float:
        target = self.nominal_target_headway_s()
        if self.mode == "adaptive" and tsr_active:
            target += self.adaptive_tsr_extra_s
        return target

    def decide(
        self,
        train: Any,
        now_s: float,
        dispatched_front_pos_m: float | None = None,
        tsr_active: bool = False,
    ) -> HeadwayDecision:
        if train.id in self.released_train_ids or not self.enabled():
            return HeadwayDecision(train.id, "RELEASE_DISPATCH", "HEADWAY_DISABLED_OR_ALREADY_RELEASED", 0.0, None, 0.0)

        sequence_index = len(self.released_train_ids)
        planned_time = None
        if self.mode == "timetable":
            train_planned_time = getattr(train, "schedule_planned_dispatch_s", None)
            if train_planned_time is not None:
                planned_time = float(train_planned_time)
        if planned_time is None:
            planned_time = self._planned_time_for_sequence(sequence_index)
        target = self._effective_target_headway_s(tsr_active)
        self.stats.target_headway_s = target

        if planned_time is not None and now_s + 1e-9 < planned_time:
            return HeadwayDecision(
                train.id,
                "HOLD",
                "TIMETABLE_NOT_DUE" if self.mode == "timetable" else "HEADWAY_NOT_DUE",
                target,
                planned_time,
                0.0,
            )

        last_release = max(self.stats.release_times_s.values(), default=None)
        if self.mode != "timetable" and last_release is not None and target > 0.0 and now_s - last_release + 1e-9 < target:
            return HeadwayDecision(
                train.id,
                "HOLD",
                "TARGET_HEADWAY_ACTIVE",
                target,
                planned_time,
                0.0,
            )

        if (
            self.mode == "adaptive"
            and self.adaptive_min_gap_m > 0.0
            and dispatched_front_pos_m is not None
            and dispatched_front_pos_m < self.adaptive_min_gap_m
        ):
            return HeadwayDecision(train.id, "HOLD", "ADAPTIVE_GAP_ACTIVE", target, planned_time, 0.0)

        delay = max(0.0, now_s - planned_time) if planned_time is not None else 0.0
        self.released_train_ids.add(train.id)
        self.stats.release_times_s[train.id] = now_s
        self.stats.dispatch_delays_s.append(delay)
        return HeadwayDecision(train.id, "RELEASE_DISPATCH", "HEADWAY_RELEASED", target, planned_time, delay)

    def mark_actual_dispatch(self, train_id: str, now_s: float):
        if train_id in self.stats.dispatch_times_s:
            return
        previous_train_id = None
        previous = None
        if self.stats.dispatch_times_s:
            previous_train_id, previous = max(self.stats.dispatch_times_s.items(), key=lambda item: item[1])
        self.stats.dispatch_times_s[train_id] = now_s
        if previous is not None:
            actual_headway_s = max(0.0, now_s - previous)
            self.stats.actual_headways_s.append(actual_headway_s)
            self.stats.actual_headway_pairs.append(
                {
                    "front_train_id": previous_train_id,
                    "following_train_id": train_id,
                    "front_dispatch_time_s": previous,
                    "following_dispatch_time_s": now_s,
                    "actual_headway_s": actual_headway_s,
                }
            )

    def snapshot(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "target_headway_s": self.stats.target_headway_s,
            "actual_headways_s": list(self.stats.actual_headways_s),
            "actual_headway_pairs": [dict(pair) for pair in self.stats.actual_headway_pairs],
            "min_actual_headway_s": self.stats.min_actual_headway_s,
            "avg_actual_headway_s": self.stats.avg_actual_headway_s,
            "max_actual_headway_s": self.stats.max_actual_headway_s,
            "dispatch_delays_s": list(self.stats.dispatch_delays_s),
            "avg_dispatch_delay_s": self.stats.avg_dispatch_delay_s,
            "dispatch_times_s": dict(self.stats.dispatch_times_s),
            "release_times_s": dict(self.stats.release_times_s),
        }
