from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from CONFIG.config import (
    BRAKE_BUILDUP_S,
    BRAKE_FORCE_N,
    DT,
    EMERGENCY_FORCE_N,
    MAX_JERK_MS3,
)
from SUBSYSTEMS.core_engine import (
    ATP_BRAKE_BUILDUP_S,
    ATP_EMERGENCY_BRAKE_FACTOR,
    ATP_MIN_DECEL_MS2,
    VitalBrakeModel,
    conservative_brake_decel_ms2,
    max_entry_speed_with_buildup,
    max_speed_with_buildup,
    stopping_distance_with_buildup,
    vital_delay_margin_m,
)
from SUBSYSTEMS.physics import (
    equivalent_mass_adjusted_accel,
    kmh_to_ms,
    ms_to_kmh,
    traction_acceleration_ms2,
)

G = 9.81

POS_UNCERT_M = 4.0
BALISE_POS_UNCERT_M = 0.8
STATION_POS_UNCERT_M = 0.5
PRECISE_STOP_POS_UNCERT_M = 0.2
# Conservative ATP assumptions used by the vital braking model.
ATP_P_REACTION_S = 2.5
ATP_W_REACTION_S = 0.8
ATP_SBI_REACTION_S = 1.2
ATP_EBI_REACTION_S = 1.6
ATP_INDICATION_DELAY_S = 0.8
ATP_MA_EXTRAPOLATION_S = 1.5
ATP_POS_REPORT_LATENCY_S = 0.6
ATP_ADHESION_FACTOR = 0.82
ATP_SERVICE_BRAKE_FACTOR = 1.03
ATP_EMERGENCY_BRAKE_FACTOR = 1.05
ATP_BRAKE_BUILDUP_S = max(1.2, BRAKE_BUILDUP_S - 0.2)
ATP_MIN_DECEL_MS2 = 0.15
# Time margins (s) for target supervision. Kept tighter so the curves stay closer.
W_TIME_S = 1.0
SBI_TIME_S = 1.5
EBI_TIME_S = 2.0
P_TIME_S = 2.0
# Speed tolerances (km/h) for ceiling supervision
W_SPEED_TOL_KMH = 1.0
SBI_SPEED_TOL_KMH = 2.0
EBD_SPEED_TOL_KMH = 3.0
EBI_SPEED_MARGIN_KMH = 0.5
STOP_EBI_SUPERVISION_FLOOR_KMH = 0.5
SBI_VIOLATION_EPS_KMH = 0.03
# Additional widening of ATP supervision curves at higher speeds
HIGH_SPEED_CURVE_BLEND_FROM_KMH = 45.0
HIGH_SPEED_CURVE_BLEND_TO_KMH = 100.0
HIGH_SPEED_TIME_MARGIN_GAIN = 0.35
HIGH_SPEED_SPEED_TOL_GAIN_KMH = 3.0
TARGET_CURVE_RESERVE_LOW_M = 20.0
TARGET_CURVE_RESERVE_HIGH_M = 140.0
ATO_TARGET_PREP_MAX_M = 280.0
ATP_TARGET_INTERVENTION_MIN_M = 400.0
STOP_TARGET_MIN_ACTIVATION_M = 280.0
STOP_TARGET_BUFFER_M = 80.0
# Moving-block overlap beyond the granted EOA, reserved by ZC before the protected point.
# Small stop SvL offset used for precise stopping at an authority end
STOP_SVL_OFFSET_M = 1.0
# Rollback / reverse protection distance when train moves without authority
ROLLBACK_PROTECT_M = 1.0
# Minimum separation between curves (km/h), except near zero
CURVE_EPS_KMH = 0.5
# UI/display smoothing for ATP curves. Raw curves still drive ATP interventions;
# these rates only prevent chart traces from collapsing on one packet jitter tick.
CURVE_DISPLAY_DROP_RATE_KMH_S = {
    "P": 12.0,
    "I": 12.0,
    "W": 14.0,
    "OFF": 14.0,
    "SBI": 16.0,
    "SBD": 16.0,
    "EBI": 20.0,
    "EBD": 20.0,
}
CURVE_DISPLAY_RISE_RATE_KMH_S = {
    "P": 18.0,
    "I": 18.0,
    "W": 20.0,
    "OFF": 20.0,
    "SBI": 24.0,
    "SBD": 24.0,
    "EBI": 28.0,
    "EBD": 28.0,
}
# Additional ATO tracking reserve to keep high-speed running away from ATP intervention curves.
ATO_TRACKING_MARGIN_LOW_KMH = 0.3
ATO_TRACKING_MARGIN_HIGH_KMH = 6.0
ATO_TRACKING_MARGIN_BLEND_FROM_KMH = 35.0
ATO_TRACKING_MARGIN_BLEND_TO_KMH = 100.0
LOW_SPEED_FLEX_FULL_KMH = 15.0
LOW_SPEED_FLEX_NONE_KMH = 45.0
ATO_TRACKING_MARGIN_LOW_SPEED_RELIEF_KMH = 0.05
ATO_TRACKING_MARGIN_MIN_LOW_KMH = 0.25
I_CURVE_MARGIN_LOW_KMH = 2.0
I_CURVE_MARGIN_HIGH_KMH = 12.0
ATO_EBI_GUARD_LOW_KMH = 3.5
ATO_EBI_GUARD_HIGH_KMH = 10.0
# Precise stop target used by ATO close to authority end
STOP_TARGET_OFFSET_M = 0.75
# Stop position tolerance for final accuracy
STOP_ACCURACY_TOL_M = 0.3
# Low-speed docking zone near the stop target
DOCKING_ZONE_M = 12.0
DOCKING_SPEED_KMH = 4.0
FINAL_CREEP_ZONE_M = 2.0
FINAL_CREEP_MIN_SPEED_KMH = 1.8
ATO_TARGET_DROP_RATE_KMH_S = 8.0
FINAL_APPROACH_MAX_SPEED_KMH = 5.0
FINAL_APPROACH_MIN_SPEED_KMH = 2.5
FINAL_APPROACH_SBI_FLOOR_KMH = 3.5
PRECISE_STOP_EBI_FLOOR_KMH = 3.0
PRECISE_STOP_SERVICE_BAND_M = 2.0
PRECISE_STOP_SBI_ENTRY_KMH = 2.8
PRECISE_STOP_EBI_GAP_KMH = 2.0
# Speed estimation / supervision resolution
SPEED_ESTIMATION_RES_KMH = 1.0
ATO_CONTROL_RES_KMH = 0.5
VITAL_SPEED_MARGIN_KMH = 0.5
# Release speed supervision near the end of authority
# This allows the train to creep into the stop point without early EBI intervention.
RELEASE_ZONE_M = 100.0
RELEASE_SPEED_KMH = 18.0
RELEASE_SCAN_FAST_KMH = 10.0
RELEASE_SCAN_FINE_KMH = 6.0
RELEASE_BLEND_EXP = 1.35
RELEASE_HANDOVER_START_M = 16.0
FINAL_STOP_BRAKE_ZONE_M = 8.0
RELEASE_ENTRY_MARGIN_KMH = 1.5
RELEASE_I_MARGIN_KMH = 0.35
# Precise stop beacon and forward jog
STOP_BEACON_OFFSET_M = 25.0
JOG_MAX_DIST_M = 15.0
JOG_SPEED_KMH = 5.0
JOG_PROFILE_ACCEL_MS2 = 1.0
MANUAL_JOG_WINDOW_M = 2.0
AUTO_DOCKING_JOG_WINDOW_M = 3.0
JOG_WINDOW_EPS_M = 0.05
JOG_STATE_IDLE = "IDLE"
JOG_STATE_REQUESTED = "REQUESTED"
JOG_STATE_ACTIVE = "ACTIVE"
JOG_STATE_COMPLETED = "COMPLETED"
JOG_STATE_FAILED_LOCKED = "FAILED_LOCKED"
LOW_SPEED_ATP_GUARD_KMH = 2.0
CREEP_MAX_SPEED_KMH = 25.0
CREEP_RELEASE_CAP_KMH = 25.0
# Standstill monitoring
STANDSTILL_DRIFT_M = 2.0
STANDSTILL_SPEED_EPS = 0.05
# Natural deceleration when traction is cut off
COAST_BASE_DECEL = 0.12
COAST_SPEED_GAIN = 0.03
# Additional operating margins on steep downgrade
DOWNHILL_P_BUFFER_KMH_PER_GRAD = 40.0
SBI_RELEASE_HYST_KMH = 1.0

# Balise-based position correction
BALISE_SPACING_M = 200.0
BALISE_ERROR_MAX_M = BALISE_POS_UNCERT_M
STATION_BALISE_ZONE_M = 120.0
STATION_BALISE_SPACING_M = 25.0
STATION_BALISE_ERROR_MAX_M = 0.35
BEACON_LOCK_RELEASE_MARGIN_M = 35.0
# Odometer drift grows with distance until balise/beacon correction.
ODOMETER_ERROR_RATE = 0.05
# Simple non-vital DCS timing model.
DCS_DELAY_MIN_S = 0.05
DCS_DELAY_MAX_S = 0.35
DCS_TIMEOUT_S = 1.0
DCS_STARTUP_GRACE_S = 1.0
# Low-pass filtering of measured speed before quantization.
SPEED_FILTER_TAU_S = 0.35
# ATO PID baseline by speed band.
ATO_PID_KP_LOW = 0.85
ATO_PID_KP_HIGH = 0.55
ATO_PID_KI_LOW = 0.08
ATO_PID_KI_HIGH = 0.03
ATO_PID_KD_LOW = 0.10
ATO_PID_KD_HIGH = 0.04
ATO_PID_BLEND_FROM_KMH = 10.0
ATO_PID_BLEND_TO_KMH = 65.0
ATO_PID_INT_LIMIT_MS = kmh_to_ms(8.0)


def max_speed_for_target(v_target_ms: float, distance_m: float, decel: float) -> float:
    if distance_m <= 0 or decel <= 0:
        return v_target_ms
    return (v_target_ms * v_target_ms + 2.0 * decel * distance_m) ** 0.5


def quantize_speed_ms(speed_ms: float, resolution_kmh: float) -> float:
    if speed_ms <= 0.0 or resolution_kmh <= 0.0:
        return 0.0
    resolution_ms = kmh_to_ms(resolution_kmh)
    steps = round(speed_ms / resolution_ms)
    return max(0.0, steps * resolution_ms)


def lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, ratio))


def low_pass_step(previous: float, measurement: float, tau_s: float, dt_s: float) -> float:
    if tau_s <= 0.0 or dt_s <= 0.0:
        return measurement
    alpha = max(0.0, min(1.0, dt_s / (tau_s + dt_s)))
    return previous + alpha * (measurement - previous)


def release_transition_ratio(distance_m: float) -> float:
    if distance_m <= FINAL_STOP_BRAKE_ZONE_M:
        return 1.0
    if distance_m >= RELEASE_HANDOVER_START_M:
        return 0.0
    span_m = max(0.1, RELEASE_HANDOVER_START_M - FINAL_STOP_BRAKE_ZONE_M)
    ratio = 1.0 - ((distance_m - FINAL_STOP_BRAKE_ZONE_M) / span_m)
    # Smoothstep keeps the final release handover gradual instead of dropping
    # the ATP curves abruptly while ATO is still braking toward the marker.
    return ratio * ratio * (3.0 - 2.0 * ratio)


def release_speed_profile(distance_m: float, release_speed_ms: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    fine_scan_speed_ms = kmh_to_ms(RELEASE_SCAN_FINE_KMH)
    fast_scan_speed_ms = min(release_speed_ms, kmh_to_ms(RELEASE_SCAN_FAST_KMH))
    min_creep_speed_ms = kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH)
    if distance_m >= RELEASE_HANDOVER_START_M:
        return fast_scan_speed_ms
    if distance_m >= FINAL_STOP_BRAKE_ZONE_M:
        span_m = max(0.1, RELEASE_HANDOVER_START_M - FINAL_STOP_BRAKE_ZONE_M)
        ratio = max(0.0, min(1.0, (distance_m - FINAL_STOP_BRAKE_ZONE_M) / span_m))
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        return lerp(fine_scan_speed_ms, fast_scan_speed_ms, smooth)
    span_m = max(0.1, FINAL_STOP_BRAKE_ZONE_M - STOP_ACCURACY_TOL_M)
    ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
    smooth = ratio * ratio * (3.0 - 2.0 * ratio)
    return lerp(min_creep_speed_ms, fine_scan_speed_ms, smooth)


def release_entry_speed_limit(distance_m: float, release_speed_ms: float, decel: float) -> float:
    if distance_m <= 0.0:
        return 0.0
    if distance_m <= FINAL_STOP_BRAKE_ZONE_M:
        return release_speed_profile(distance_m, release_speed_ms)
    cruise_distance_m = max(0.0, distance_m - FINAL_STOP_BRAKE_ZONE_M)
    return max_entry_speed_with_buildup(release_speed_ms, cruise_distance_m, decel, BRAKE_BUILDUP_S)


def precise_stop_sbi_limit_ms(distance_m: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    if distance_m <= PRECISE_STOP_SERVICE_BAND_M:
        span_m = max(0.1, PRECISE_STOP_SERVICE_BAND_M - STOP_ACCURACY_TOL_M)
        ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
        return kmh_to_ms(PRECISE_STOP_SBI_ENTRY_KMH) * (ratio ** 0.8)
    if distance_m <= JOG_MAX_DIST_M:
        span_m = max(0.1, JOG_MAX_DIST_M - PRECISE_STOP_SERVICE_BAND_M)
        ratio = max(0.0, min(1.0, (distance_m - PRECISE_STOP_SERVICE_BAND_M) / span_m))
        sbi_kmh = PRECISE_STOP_SBI_ENTRY_KMH + (FINAL_APPROACH_SBI_FLOOR_KMH - PRECISE_STOP_SBI_ENTRY_KMH) * ratio
        return kmh_to_ms(sbi_kmh)
    return kmh_to_ms(FINAL_APPROACH_SBI_FLOOR_KMH)


def precise_stop_gap_ms(distance_m: float, full_gap_kmh: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    span_m = max(0.1, PRECISE_STOP_SERVICE_BAND_M - STOP_ACCURACY_TOL_M)
    ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
    return kmh_to_ms(full_gap_kmh) * (ratio ** 0.8)


def precise_stop_profile_active(train: "Train", distance_m: float) -> bool:
    if not train.commanded_stop or train.trip_mode:
        return False
    if distance_m < 0.0 or distance_m > JOG_MAX_DIST_M:
        return False
    return (
        distance_m <= PRECISE_STOP_SERVICE_BAND_M
        or train.zero_speed_detected
        or train.speed <= STANDSTILL_SPEED_EPS
    )


def emergency_speed_curve(
    target_speed_ms: float,
    distance_m: float,
    decel: float,
    reaction_margin_m: float,
) -> float:
    """Compute the emergency intervention curve (EBI/EBD) from an EBD baseline."""
    if decel <= 0 or distance_m <= 0:
        return 0.0
    d = distance_m - (POS_UNCERT_M + reaction_margin_m)
    return max(0.0, max_entry_speed_with_buildup(target_speed_ms, max(0.0, d), decel, BRAKE_BUILDUP_S))


def ato_tracking_margin_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = ATO_TRACKING_MARGIN_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = ATO_TRACKING_MARGIN_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = ATO_TRACKING_MARGIN_LOW_KMH + ratio * (
            ATO_TRACKING_MARGIN_HIGH_KMH - ATO_TRACKING_MARGIN_LOW_KMH
        )
    low_speed_relief = ATO_TRACKING_MARGIN_LOW_SPEED_RELIEF_KMH * low_speed_flexibility_scale(speed_ms)
    margin_kmh = max(ATO_TRACKING_MARGIN_MIN_LOW_KMH, margin_kmh - low_speed_relief)
    return kmh_to_ms(margin_kmh)


def i_curve_margin_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = I_CURVE_MARGIN_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = I_CURVE_MARGIN_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = I_CURVE_MARGIN_LOW_KMH + ratio * (
            I_CURVE_MARGIN_HIGH_KMH - I_CURVE_MARGIN_LOW_KMH
        )
    return kmh_to_ms(margin_kmh)


def ato_ebi_guard_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = ATO_EBI_GUARD_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = ATO_EBI_GUARD_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = ATO_EBI_GUARD_LOW_KMH + ratio * (
            ATO_EBI_GUARD_HIGH_KMH - ATO_EBI_GUARD_LOW_KMH
        )
    return kmh_to_ms(margin_kmh)


def low_speed_flexibility_scale(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= LOW_SPEED_FLEX_FULL_KMH:
        return 1.0
    if speed_kmh >= LOW_SPEED_FLEX_NONE_KMH:
        return 0.0
    return 1.0 - (
        (speed_kmh - LOW_SPEED_FLEX_FULL_KMH)
        / (LOW_SPEED_FLEX_NONE_KMH - LOW_SPEED_FLEX_FULL_KMH)
    )

def ato_brake_gain(speed_ms: float) -> float:
    speed_ratio = min(1.0, max(0.0, (ms_to_kmh(speed_ms) - 40.0) / 60.0))
    low_speed_relief = 0.2 * low_speed_flexibility_scale(speed_ms)
    return max(0.85, 1.05 + 0.45 * speed_ratio - low_speed_relief)


def ato_pid_gains(speed_ms: float) -> Tuple[float, float, float]:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_PID_BLEND_FROM_KMH:
        return ATO_PID_KP_LOW, ATO_PID_KI_LOW, ATO_PID_KD_LOW
    if speed_kmh >= ATO_PID_BLEND_TO_KMH:
        return ATO_PID_KP_HIGH, ATO_PID_KI_HIGH, ATO_PID_KD_HIGH
    ratio = (speed_kmh - ATO_PID_BLEND_FROM_KMH) / max(0.1, ATO_PID_BLEND_TO_KMH - ATO_PID_BLEND_FROM_KMH)
    return (
        lerp(ATO_PID_KP_LOW, ATO_PID_KP_HIGH, ratio),
        lerp(ATO_PID_KI_LOW, ATO_PID_KI_HIGH, ratio),
        lerp(ATO_PID_KD_LOW, ATO_PID_KD_HIGH, ratio),
    )


def high_speed_curve_scale(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= HIGH_SPEED_CURVE_BLEND_FROM_KMH:
        return 0.0
    if speed_kmh >= HIGH_SPEED_CURVE_BLEND_TO_KMH:
        return 1.0
    return (
        (speed_kmh - HIGH_SPEED_CURVE_BLEND_FROM_KMH)
        / (HIGH_SPEED_CURVE_BLEND_TO_KMH - HIGH_SPEED_CURVE_BLEND_FROM_KMH)
    )


def required_brake_rate_for_target(current_speed_ms: float, target_speed_ms: float, distance_m: float) -> float:
    """Minimum constant deceleration needed to reach target speed within the remaining distance."""
    if current_speed_ms <= target_speed_ms:
        return 0.0
    if distance_m <= 0.0:
        return float("inf")
    return max(0.0, (current_speed_ms * current_speed_ms - target_speed_ms * target_speed_ms) / (2.0 * distance_m))


def target_curve_reserve_m(current_speed_ms: float, target_speed_ms: float) -> float:
    speed_gap_ratio = 0.0
    if current_speed_ms > 0.0:
        speed_gap_ratio = max(0.0, min(1.0, (current_speed_ms - target_speed_ms) / current_speed_ms))
    high_speed_scale = high_speed_curve_scale(current_speed_ms)
    blend = max(high_speed_scale, speed_gap_ratio)
    return TARGET_CURVE_RESERVE_LOW_M + (TARGET_CURVE_RESERVE_HIGH_M - TARGET_CURVE_RESERVE_LOW_M) * blend


def next_lower_limit(
    track_profile: List[Tuple[float, float, float, float]],
    pos_m: float,
    current_psr: float,
    tsr_zones,
) -> Tuple[float, float]:
    best_dist = float("inf")
    best_speed = current_psr

    # PSR segments ahead
    for start, _end, _grad, psr in track_profile:
        if start <= pos_m:
            continue
        if psr < current_psr:
            dist = start - pos_m
            if dist < best_dist or (dist == best_dist and psr < best_speed):
                best_dist = dist
                best_speed = psr

    # TSR zones ahead
    for zone in tsr_zones:
        z_start = zone["start"]
        z_speed = zone["speed"]
        if z_start <= pos_m:
            continue
        if z_speed < current_psr:
            dist = z_start - pos_m
            if dist < best_dist or (dist == best_dist and z_speed < best_speed):
                best_dist = dist
                best_speed = z_speed

    if best_dist == float("inf"):
        return current_psr, float("inf")
    return best_speed, best_dist


def elevation_at(track_profile: List[Tuple[float, float, float, float]], pos_m: float) -> float:
    if pos_m <= track_profile[0][0]:
        start, _, gradient, _ = track_profile[0]
        return (pos_m - start) * gradient

    elev = 0.0
    for start, end, gradient, _ in track_profile:
        if pos_m >= end:
            elev += (end - start) * gradient
        else:
            elev += (pos_m - start) * gradient
            break
    return elev


def indication_speed_delta_ms(service_decel_ms2: float, delay_s: float) -> float:
    if service_decel_ms2 <= 0.0 or delay_s <= 0.0:
        return kmh_to_ms(CURVE_EPS_KMH)
    return max(kmh_to_ms(CURVE_EPS_KMH), service_decel_ms2 * delay_s)


def train_color(train_cfg: Dict[str, float | str | None], index: int, palette: List[str]) -> str:
    color = train_cfg.get("color")
    if isinstance(color, str) and color:
        return color
    return palette[index % len(palette)]

__all__ = [name for name in globals() if not name.startswith("_")]
