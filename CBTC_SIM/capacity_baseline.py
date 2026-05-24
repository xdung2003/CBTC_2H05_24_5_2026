from __future__ import annotations

from typing import Any, Dict, Iterable


def _avg(values: Iterable[float]) -> float | None:
    items = [float(value) for value in values if value is not None]
    if not items:
        return None
    return sum(items) / len(items)


def _positive(values: Iterable[float | None]) -> list[float]:
    return [float(value) for value in values if value is not None and float(value) > 0.0]


def build_capacity_comparison(sim: Any) -> Dict[str, Any]:
    """Build MOVING_BLOCK vs FIXED_BLOCK_BASELINE capacity metrics.

    The fixed-block side is an analytical baseline only. It does not affect
    ZC/EOA, ATP, ATO, routing or train dynamics.
    """
    cfg = sim.scenario.get("capacity_baseline", {}) if isinstance(sim.scenario, dict) else {}
    if not isinstance(cfg, dict):
        cfg = {}
    blocks_per_section = max(1, int(cfg.get("blocks_per_section", cfg.get("fixed_blocks_per_section", 4))))

    current_headways = _positive(getattr(train, "headway_time_s", None) for train in sim.trains)
    actual_headways = _positive(sim.analytics.get("actual_headways_s", []))
    moving_headways = actual_headways or current_headways
    moving_min_headway = sim.analytics.get("min_actual_headway_s") or sim.analytics.get("min_headway_s")
    if moving_min_headway is None and moving_headways:
        moving_min_headway = min(moving_headways)
    moving_avg_headway = sim.analytics.get("avg_actual_headway_s") or _avg(moving_headways)
    moving_max_headway = sim.analytics.get("max_actual_headway_s")
    if moving_max_headway is None and moving_headways:
        moving_max_headway = max(moving_headways)

    moving_delay = sim.analytics.get("avg_dispatch_delay_s") or 0.0
    moving_distance_ahead = _avg(
        _positive(getattr(train, "distance_to_train_ahead_m", None) for train in sim.trains)
    )
    speeds_ms = _positive(getattr(train, "speed", 0.0) for train in sim.trains)
    avg_speed_ms = _avg(speeds_ms)
    if avg_speed_ms is None or avg_speed_ms <= 0.5:
        psr_values = [segment[3] for segment in getattr(sim, "track_profile", [])]
        avg_speed_ms = ((_avg(psr_values) or 40.0) / 3.6) * 0.65

    avg_train_length_m = _avg(getattr(train, "length", 0.0) for train in sim.trains) or 60.0
    moving_spacing_m = avg_train_length_m + 110.0
    moving_physical_headway_s = moving_spacing_m / max(avg_speed_ms, 0.1)
    if moving_avg_headway is None:
        moving_avg_headway = moving_physical_headway_s
    if moving_min_headway is None:
        moving_min_headway = moving_avg_headway * 0.9
    if moving_max_headway is None:
        moving_max_headway = moving_avg_headway * 1.1
    if moving_distance_ahead is None:
        moving_distance_ahead = moving_spacing_m

    block_lengths = [
        float(block["end_m"]) - float(block["start_m"])
        for block in getattr(sim, "fixed_blocks", [])
        if float(block["end_m"]) > float(block["start_m"])
    ]
    if block_lengths:
        fixed_block_length_m = _avg(block_lengths) or 300.0
    else:
        track_len_m = max(1.0, float(getattr(sim, "track_end_m", 0.0)) - float(getattr(sim, "track_min_m", 0.0)))
        section_count = max(1, len(getattr(sim, "scheduled_stops", [])) or 1)
        fixed_block_length_m = track_len_m / max(1, section_count * blocks_per_section)
    fixed_spacing_m = fixed_block_length_m + avg_train_length_m
    fixed_physical_headway_s = fixed_spacing_m / max(avg_speed_ms, 0.1)
    if moving_avg_headway is not None:
        fixed_avg_headway = max(fixed_physical_headway_s, moving_avg_headway * 1.2)
    else:
        fixed_avg_headway = fixed_physical_headway_s
    fixed_min_headway = fixed_avg_headway * 0.9
    if moving_min_headway is not None:
        fixed_min_headway = max(fixed_min_headway, moving_min_headway * 1.15)
    fixed_max_headway = fixed_avg_headway * 1.1
    if moving_max_headway is not None:
        fixed_max_headway = max(fixed_max_headway, moving_max_headway * 1.15)

    moving_capacity_tph = sim.analytics.get("trains_per_hour", 0.0) or 0.0
    if moving_avg_headway and moving_avg_headway > 0.0:
        moving_capacity_tph = max(moving_capacity_tph, 3600.0 / moving_avg_headway)
    fixed_capacity_tph = 3600.0 / fixed_avg_headway if fixed_avg_headway > 0.0 else 0.0

    fixed_delay = moving_delay + max(0.0, fixed_avg_headway - (moving_avg_headway or fixed_avg_headway))
    fixed_distance_ahead = max(moving_distance_ahead or 0.0, fixed_spacing_m)

    moving = {
        "mode": "MOVING_BLOCK",
        "minimum_headway_s": moving_min_headway,
        "average_headway_s": moving_avg_headway,
        "maximum_headway_s": moving_max_headway,
        "trains_per_hour": moving_capacity_tph,
        "average_dispatch_delay_s": moving_delay,
        "average_distance_to_train_ahead_m": moving_distance_ahead,
    }
    fixed = {
        "mode": "FIXED_BLOCK_BASELINE",
        "minimum_headway_s": fixed_min_headway,
        "average_headway_s": fixed_avg_headway,
        "maximum_headway_s": fixed_max_headway,
        "trains_per_hour": fixed_capacity_tph,
        "average_dispatch_delay_s": fixed_delay,
        "average_distance_to_train_ahead_m": fixed_distance_ahead,
    }
    return {
        "assumptions": {
            "blocks_per_section": blocks_per_section,
            "average_fixed_block_length_m": fixed_block_length_m,
            "fixed_spacing_m": fixed_spacing_m,
            "moving_block_spacing_m": moving_spacing_m,
            "average_speed_ms": avg_speed_ms,
            "analysis_only": True,
        },
        "MOVING_BLOCK": moving,
        "FIXED_BLOCK_BASELINE": fixed,
        "delta": {
            "headway_reduction_s": (
                fixed_avg_headway - moving_avg_headway
                if moving_avg_headway is not None
                else None
            ),
            "capacity_gain_trains_per_hour": moving_capacity_tph - fixed_capacity_tph,
            "dispatch_delay_reduction_s": fixed_delay - moving_delay,
            "distance_to_train_ahead_reduction_m": (
                fixed_distance_ahead - moving_distance_ahead
                if moving_distance_ahead is not None
                else None
            ),
        },
    }
