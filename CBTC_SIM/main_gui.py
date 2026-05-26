from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, ttk
import random
import time
import math
import re
import sys
import threading
import queue
import yaml
from dataclasses import dataclass, replace
from datetime import datetime, timezone, timedelta
from typing import Any, Dict, List, Tuple
from collections import deque
import ctypes
from copy import deepcopy

from config import (
    AW3_MASS_KG,
    DT,
    OVERLAP_M,
    SAFETY_MARGIN_M,
    BRAKE_FORCE_N,
    EMERGENCY_FORCE_N,
    BRAKE_BUILDUP_S,
    MAX_JERK_MS3,
)
from physics import (
    kmh_to_ms,
    ms_to_kmh,
    braking_distance_m,
    traction_acceleration_ms2,
    running_resistance_accel_ms2,
    equivalent_mass_adjusted_accel,
    limit_jerk,
)
from scenario_loader import DEFAULT_SCENARIO_PATH, load_scenario, normalize_scenario, save_scenario_file, scenario_to_yaml_data
from reporting import save_simulation_report
from headway_manager import HeadwayManager
from monte_carlo import MonteCarloConfig, run_batch
from core_engine import (
    AuthorityManager,
    DCSWatchdog,
    MovementAuthorityLimit,
    OnboardControlCenter,
    SafeMovementPacket,
    VitalBrakeModel,
)

G = 9.81

TSR_COLOR = "#c94a36"
SOURCE_TRAIN_SPACING_M = 120.0
SOURCE_TRAIN_LENGTH_M = 200.0
SOURCE_TRAIN_START_M = -SOURCE_TRAIN_LENGTH_M
SOURCE_TRAIN_EXIT_M = 0.0
SOURCE_TRAIN_STAGING_CLEARANCE_M = 35.0
SOURCE_VISIBLE_ACTIVE_TRAINS = 2
MIN_PASSENGER_DWELL_S = 25.0
MIN_TIMETABLE_RECOVERY_DWELL_S = 20.0
PARALLEL_ROMAN_LABELS = ["I", "II", "III", "IV", "V", "VI", "VII", "VIII", "IX", "X"]
PARALLEL_RELEASE_MARGIN_M = 5.0
DEPARTURE_RELEASE_MIN_AUTHORITY_M = 30.0
STATION_ROUTE_APPROACH_M = 800.0
TURNOUT_LOCK_S = 5.0
LINE_CENTER_SPACING_M = 4.0
APP_THEME = {
    "bg": "#f5ebe9",
    "workspace": "#fff4ef",
    "panel": "#ffe2c2",
    "panel_alt": "#f4ba72",
    "card": "#fff8ed",
    "card_alt": "#ffe7c7",
    "canvas": "#fbf3ef",
    "canvas_grid": "#dec8bf",
    "border": "#6b2e35",
    "text": "#5a2630",
    "muted": "#8a5b52",
    "button": "#f4a63c",
    "button_hover": "#ffc15c",
    "button_pressed": "#d8782d",
    "button_active": "#ffd06f",
    "accent": "#b85c2d",
    "accent_pressed": "#8f3f24",
    "run_active": "#f0a132",
    "pause_active": "#ffd166",
    "button_inactive": "#d9a876",
    "danger": "#d84b3c",
    "danger_pressed": "#9b2c2b",
    "ok": "#74b65d",
    "warning": "#f0a132",
    "log_bg": "#fffaf2",
}

CURVE_COLORS = {
    "actual": "#5a2630",
    "P": "#2f7f8f",
    "I": "#8a4f9f",
    "W": "#b86f00",
    "SBD": "#4f8f3a",
    "EBD": "#c94a36",
}

ACTION_COLORS = {
    "WARN": CURVE_COLORS["W"],
    "OFF": "#b86f00",
    "SBI": CURVE_COLORS["SBD"],
    "EBI": CURVE_COLORS["EBD"],
}

# Position uncertainty (m). Fail-safe defaults remain conservative until a fixed
# transponder reference is available.
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


def get_track_info(track_profile: List[Tuple[float, float, float, float]], pos_m: float) -> Tuple[float, float]:
    for start_m, end_m, gradient, psr_kmh in track_profile:
        if start_m <= pos_m < end_m:
            return gradient, psr_kmh
    return track_profile[-1][2], track_profile[-1][3]


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


def braking_curve_profile(decel: float, build_s: float) -> Tuple[float, float]:
    """Return the residual delay and jerk-ramp duration used by ATP braking curves.

    The train dynamics already apply jerk-limited deceleration. For curve synthesis we model:
    1. A residual brake application delay not covered by the jerk ramp.
    2. A linear deceleration ramp until the commanded brake rate is reached.
    This keeps ATP supervision closer to the actual simulated stop performance.
    """
    if decel <= 0.0:
        return 0.0, 0.0
    if MAX_JERK_MS3 <= 0.0:
        return max(0.0, build_s), 0.0
    ramp_s = decel / MAX_JERK_MS3
    residual_delay_s = max(0.0, build_s - ramp_s)
    return residual_delay_s, ramp_s


def max_speed_with_buildup(distance_m: float, decel: float, build_s: float) -> float:
    if distance_m <= 0 or decel <= 0:
        return 0.0
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    linear_term = 2.0 * decel * (residual_delay_s + 0.5 * ramp_s)
    constant_term = 2.0 * decel * distance_m + ((decel * ramp_s) ** 2) / 12.0
    term = linear_term * linear_term + 4.0 * constant_term
    return max(0.0, 0.5 * (-linear_term + term ** 0.5))


def max_entry_speed_with_buildup(v_target_ms: float, distance_m: float, decel: float, build_s: float) -> float:
    if decel <= 0:
        return v_target_ms
    if distance_m <= 0:
        return v_target_ms
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    linear_term = 2.0 * decel * (residual_delay_s + 0.5 * ramp_s)
    constant_term = (
        v_target_ms * v_target_ms
        + 2.0 * decel * distance_m
        + ((decel * ramp_s) ** 2) / 12.0
    )
    term = linear_term * linear_term + 4.0 * constant_term
    return max(v_target_ms, 0.5 * (-linear_term + term ** 0.5))


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


def stopping_distance_with_buildup(speed_ms: float, decel: float, build_s: float) -> float:
    if speed_ms <= 0.0 or decel <= 0.0:
        return 0.0
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    return (
        speed_ms * residual_delay_s
        + (speed_ms * speed_ms) / (2.0 * decel)
        + 0.5 * speed_ms * ramp_s
        - decel * ramp_s * ramp_s / 24.0
    )


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


def worst_gradient_in_range(
    track_profile: List[Tuple[float, float, float, float]],
    start_pos_m: float,
    distance_m: float,
) -> float:
    if not track_profile:
        return 0.0
    if distance_m <= 0.0:
        return get_track_info(track_profile, start_pos_m)[0]
    end_pos_m = start_pos_m + distance_m
    worst_gradient = get_track_info(track_profile, start_pos_m)[0]
    for start_m, end_m, gradient, _psr in track_profile:
        if end_m < start_pos_m or start_m > end_pos_m:
            continue
        worst_gradient = min(worst_gradient, gradient)
    return worst_gradient


def conservative_brake_decel_ms2(force_n: float, mass_kg: float, brake_factor: float) -> float:
    worst_mass_kg = max(mass_kg, AW3_MASS_KG)
    if worst_mass_kg <= 0.0:
        return ATP_MIN_DECEL_MS2
    base_decel = equivalent_mass_adjusted_accel(force_n / worst_mass_kg) * brake_factor * ATP_ADHESION_FACTOR
    return max(ATP_MIN_DECEL_MS2, base_decel)


def gradient_adjusted_decel_ms2(
    base_decel_ms2: float,
    track_profile: List[Tuple[float, float, float, float]],
    start_pos_m: float,
    distance_m: float,
) -> float:
    worst_gradient = worst_gradient_in_range(track_profile, start_pos_m, distance_m)
    aiding_accel = max(0.0, -worst_gradient) * G
    return max(ATP_MIN_DECEL_MS2, base_decel_ms2 - aiding_accel)


def vital_delay_margin_m(speed_ms: float, delay_s: float) -> float:
    if delay_s <= 0.0:
        return 0.0
    return max(0.0, speed_ms) * delay_s


def indication_speed_delta_ms(service_decel_ms2: float, delay_s: float) -> float:
    if service_decel_ms2 <= 0.0 or delay_s <= 0.0:
        return kmh_to_ms(CURVE_EPS_KMH)
    return max(kmh_to_ms(CURVE_EPS_KMH), service_decel_ms2 * delay_s)


def train_color(train_cfg: Dict[str, float | str | None], index: int, palette: List[str]) -> str:
    color = train_cfg.get("color")
    if isinstance(color, str) and color:
        return color
    return palette[index % len(palette)]


@dataclass
class ATPEnvelopeResult:
    control_speed: float
    actual_distance_to_stop: float
    distance_to_svl: float
    svl_m: float
    target_active: bool
    stop_target_active: bool
    release_active: bool
    release_blend: float
    p_t: float
    p_r: float
    a_service: float
    a_emergency: float
    a_traction: float
    curves: Dict[str, float]
    hidden_curves: Dict[str, float]
    curve_mode: str
    cutoff_threshold: float
    margin_dyn_m: float
    atp_service_brake_decel: float
    atp_emergency_brake_decel: float
    release_speed_kmh: float = 0.0


@dataclass
class ATOPilotingResult:
    ato_brake_prepare: bool
    ato_curve_speed: float
    ato_piloting_speed: float
    ato_target_speed: float
    jog_active: bool
    jog_used: bool


class ATPEnvelopeEngine:
    """Vital supervision engine. Computes ATP curves independently from ATO piloting."""

    def compute(self, train: "Train") -> ATPEnvelopeResult:
        psr_ms = kmh_to_ms(train.psr_kmh)
        a_service = equivalent_mass_adjusted_accel(BRAKE_FORCE_N / train.mass)
        a_emergency = equivalent_mass_adjusted_accel(EMERGENCY_FORCE_N / train.mass)
        a_traction = traction_acceleration_ms2(train.speed)
        atp_service_brake_decel = conservative_brake_decel_ms2(
            BRAKE_FORCE_N,
            train.mass,
            ATP_SERVICE_BRAKE_FACTOR,
        )
        atp_emergency_brake_decel = conservative_brake_decel_ms2(
            EMERGENCY_FORCE_N,
            train.mass,
            ATP_EMERGENCY_BRAKE_FACTOR,
        )
        estimated_speed = quantize_speed_ms(train.filtered_speed, SPEED_ESTIMATION_RES_KMH)
        control_speed = quantize_speed_ms(train.filtered_speed, ATO_CONTROL_RES_KMH)
        if train.filtered_speed <= STANDSTILL_SPEED_EPS:
            # Do not keep a residual vital-speed margin at standstill; it prevents
            # the final jog from re-applying traction and can deadlock a station stop.
            estimated_speed = 0.0
            control_speed = 0.0
            vital_speed = 0.0
        else:
            vital_speed = estimated_speed + kmh_to_ms(VITAL_SPEED_MARGIN_KMH)
        position_uncertainty_m = train.effective_position_uncertainty_m()

        brake_model = VitalBrakeModel(
            train.track_profile,
            train.safe_front_end_pos,
            position_uncertainty_m,
            vital_speed,
            ATP_BRAKE_BUILDUP_S,
        )

        svl_pos = brake_model.supervised_location_m(train.eoa)
        distance_to_svl = svl_pos - train.reported_pos
        actual_distance_to_stop = train.stop_target_pos - train.pos
        high_speed_scale = high_speed_curve_scale(vital_speed)
        time_margin_boost = 1.0 + HIGH_SPEED_TIME_MARGIN_GAIN * high_speed_scale
        extra_tol_kmh = HIGH_SPEED_SPEED_TOL_GAIN_KMH * high_speed_scale
        i_margin_ms = indication_speed_delta_ms(
            atp_service_brake_decel,
            ATP_INDICATION_DELAY_S * time_margin_boost,
        )

        margin_p = vital_delay_margin_m(vital_speed, ATP_P_REACTION_S * time_margin_boost)
        margin_w = vital_delay_margin_m(vital_speed, ATP_W_REACTION_S * time_margin_boost)
        margin_sbi = vital_delay_margin_m(vital_speed, ATP_SBI_REACTION_S * time_margin_boost)
        margin_ebi = vital_delay_margin_m(vital_speed, ATP_EBI_REACTION_S * time_margin_boost)
        margin_dyn_m = position_uncertainty_m + margin_w

        p_c = psr_ms
        w_c = psr_ms + kmh_to_ms(W_SPEED_TOL_KMH + 0.4 * extra_tol_kmh)
        off_c = psr_ms + kmh_to_ms(1.5 + 0.7 * extra_tol_kmh)
        sbi_c = psr_ms + kmh_to_ms(SBI_SPEED_TOL_KMH + extra_tol_kmh)
        sbd_c = psr_ms + kmh_to_ms(SBI_SPEED_TOL_KMH + 0.5 + 1.2 * extra_tol_kmh)
        ebd_c = psr_ms + kmh_to_ms(EBD_SPEED_TOL_KMH + 1.4 * extra_tol_kmh)
        ebi_c = max(0.0, ebd_c - kmh_to_ms(EBI_SPEED_MARGIN_KMH + 0.35 * extra_tol_kmh))

        speed_target_active = train.limit_ahead_dist != float("inf")
        if speed_target_active:
            limit_ahead_speed_ms = kmh_to_ms(train.limit_ahead_speed_kmh)
            p_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_P_REACTION_S * time_margin_boost,
            )
            w_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_W_REACTION_S * time_margin_boost,
            )
            off_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                0.5 * (ATP_W_REACTION_S + ATP_SBI_REACTION_S) * time_margin_boost,
            )
            sbi_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            sbd_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                0.0,
            )
            ebd_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_emergency_brake_decel,
                0.0,
            )
            ebi_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            ebi_speed = min(ebi_speed, max(0.0, ebd_speed - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
        else:
            p_speed = p_c
            w_speed = w_c
            off_speed = off_c
            sbi_speed = sbi_c
            sbd_speed = sbd_c
            ebi_speed = ebi_c
            ebd_speed = ebd_c

        stop_activation_distance = max(
            STOP_TARGET_MIN_ACTIVATION_M,
            stopping_distance_with_buildup(train.speed, a_service, BRAKE_BUILDUP_S) + STOP_TARGET_BUFFER_M,
        )
        stop_activation_distance = max(stop_activation_distance, ATO_TARGET_PREP_MAX_M)
        stop_target_active = train.commanded_stop or train.distance_to_eoa <= stop_activation_distance
        if stop_target_active:
            sbd_stop = brake_model.speed_for_stop(train.distance_to_eoa, atp_service_brake_decel, 0.0)
            ebd_stop = brake_model.speed_for_stop(distance_to_svl, atp_emergency_brake_decel, 0.0)
            ebi_stop = brake_model.speed_for_target(
                0.0,
                distance_to_svl,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            ebi_stop = min(ebi_stop, max(0.0, ebd_stop - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
            w_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_W_REACTION_S * time_margin_boost,
            )
            off_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                0.5 * (ATP_W_REACTION_S + ATP_SBI_REACTION_S) * time_margin_boost,
            )
            sbi_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            p_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_P_REACTION_S * time_margin_boost,
            )
        else:
            sbd_stop = float("inf")
            ebd_stop = float("inf")
            w_stop = float("inf")
            off_stop = float("inf")
            sbi_stop = float("inf")
            p_stop = float("inf")
            ebi_stop = float("inf")

        p_t = min(p_speed, p_stop)
        w_t = min(w_speed, w_stop)
        off_t = min(off_speed, off_stop)
        sbi_t = min(sbi_speed, sbi_stop)
        sbd_t = min(sbd_speed, sbd_stop)
        ebi_t = min(ebi_speed, ebi_stop)
        ebd_t = min(ebd_speed, ebd_stop)
        if ebd_t <= sbd_t + kmh_to_ms(CURVE_EPS_KMH):
            ebd_t = sbd_t + kmh_to_ms(CURVE_EPS_KMH)
        if ebi_t <= sbi_t + kmh_to_ms(CURVE_EPS_KMH):
            ebi_t = sbi_t + kmh_to_ms(CURVE_EPS_KMH)

        release_speed = kmh_to_ms(RELEASE_SPEED_KMH)
        release_base = release_speed_profile(actual_distance_to_stop, release_speed)
        release_speed_kmh = ms_to_kmh(release_base)  # Actual release speed for display
        release_service_entry = release_entry_speed_limit(
            actual_distance_to_stop,
            release_speed,
            atp_service_brake_decel,
        )
        release_emergency_entry = release_entry_speed_limit(
            actual_distance_to_stop,
            release_speed,
            atp_emergency_brake_decel,
        )
        p_r = release_base
        w_r = p_r + kmh_to_ms(1.0)
        off_r = p_r + kmh_to_ms(1.5)
        sbi_r = p_r + kmh_to_ms(2.5)
        release_ebd = max(release_emergency_entry, release_base)
        release_ebi = max(0.0, release_ebd - kmh_to_ms(EBI_SPEED_MARGIN_KMH))
        release_blend = release_transition_ratio(actual_distance_to_stop)
        p_release = min(p_c, lerp(p_t, p_r, release_blend))
        w_release = min(w_c, lerp(w_t, w_r, release_blend))
        off_release = min(off_c, lerp(off_t, off_r, release_blend))
        sbi_release = min(sbi_c, lerp(sbi_t, sbi_r, release_blend))
        sbd_release = min(sbd_t, lerp(sbd_t, max(sbd_t, release_service_entry), release_blend))
        ebi_release = min(ebi_t, lerp(ebi_t, max(ebi_t, release_ebi), release_blend))
        ebd_release = min(ebd_t, lerp(ebd_t, max(ebd_t, release_ebd), release_blend))
        use_precise_profile = precise_stop_profile_active(train, actual_distance_to_stop)
        if use_precise_profile:
            profile_sbi = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            profile_sbd_gap = precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH)
            profile_ebi_gap = precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            profile_ebd_gap = precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH)
            sbi_release = profile_sbi
            sbd_release = profile_sbi + profile_sbd_gap
            ebi_release = profile_sbi + profile_ebi_gap
            ebd_release = ebi_release + profile_ebd_gap
        elif train.commanded_stop and 0.0 < actual_distance_to_stop <= JOG_MAX_DIST_M:
            sbi_floor = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            ebi_floor = sbi_floor + precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            sbi_release = max(sbi_release, sbi_floor)
            sbd_release = max(
                sbd_release,
                sbi_release + precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH),
            )
            ebi_release = max(ebi_release, ebi_floor)
            ebd_release = max(
                ebd_release,
                ebi_release + precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH),
            )
        i_release = max(0.0, p_release - i_margin_ms)

        target_active = speed_target_active or stop_target_active or p_t < p_c - kmh_to_ms(0.1)
        near_release_zone = STOP_ACCURACY_TOL_M < actual_distance_to_stop <= RELEASE_ZONE_M
        release_entry_ok = vital_speed <= min(
            kmh_to_ms(CREEP_RELEASE_CAP_KMH),
            max(release_service_entry, p_t) + kmh_to_ms(RELEASE_ENTRY_MARGIN_KMH),
        )
        release_active = (
            near_release_zone
            and stop_target_active
            and (release_entry_ok or (train.release_active and train.commanded_stop))
            and not train.emergency_stop
            and not train.emg_latch
        )

        if train.trip_mode:
            trip_distance = max(0.0, train.trip_protect_pos - train.safe_front_end_pos)
            trip_sbd = brake_model.speed_for_stop(trip_distance, atp_service_brake_decel, 0.0)
            trip_sbi = brake_model.speed_for_stop(
                trip_distance,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            trip_ebd = brake_model.speed_for_stop(trip_distance, atp_emergency_brake_decel, 0.0)
            trip_ebi = brake_model.speed_for_target(
                0.0,
                trip_distance,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            trip_ebi = min(trip_ebi, max(0.0, trip_ebd - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
            curve_mode = "TRIP"
            curves = {"P": 0.0, "W": 0.0, "SBD": trip_sbd, "EBD": trip_ebd}
            hidden_curves = {"I": 0.0, "OFF": 0.0, "SBI": trip_sbi, "EBI": trip_ebi}
        elif release_active:
            curve_mode = "RELEASE"
            curves = {"P": p_release, "W": w_release, "SBD": sbd_release, "EBD": ebd_release}
            hidden_curves = {"I": i_release, "OFF": off_release, "SBI": sbi_release, "EBI": ebi_release}
        elif target_active:
            curve_mode = "TARGET"
            curves = {
                "P": min(p_c, p_t),
                "W": min(w_c, w_t),
                "SBD": min(sbd_c, sbd_t),
                "EBD": min(ebd_c, ebd_t),
            }
            hidden_curves = {
                "I": max(0.0, min(p_c, p_t) - i_margin_ms),
                "OFF": min(off_c, off_t),
                "SBI": min(sbi_c, sbi_t),
                "EBI": min(ebi_c, ebi_t),
            }
        else:
            curve_mode = "CEILING"
            curves = {"P": p_c, "W": w_c, "SBD": sbd_c, "EBD": ebd_c}
            hidden_curves = {"I": max(0.0, p_c - i_margin_ms), "OFF": off_c, "SBI": sbi_c, "EBI": ebi_c}

        if use_precise_profile:
            sbi_limit = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            sbd_gap = precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH)
            ebi_gap = precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            ebd_gap = precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH)
            hidden_curves["SBI"] = sbi_limit
            curves["SBD"] = hidden_curves["SBI"] + sbd_gap
            hidden_curves["EBI"] = hidden_curves["SBI"] + ebi_gap
            curves["EBD"] = hidden_curves["EBI"] + ebd_gap
        elif (
            train.commanded_stop
            and not train.trip_mode
            and PRECISE_STOP_SERVICE_BAND_M < actual_distance_to_stop <= JOG_MAX_DIST_M
        ):
            sbi_floor = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            ebi_floor = sbi_floor + precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            hidden_curves["SBI"] = max(hidden_curves["SBI"], sbi_floor)
            curves["SBD"] = max(
                curves["SBD"],
                hidden_curves["SBI"] + precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH),
            )
            hidden_curves["EBI"] = max(hidden_curves["EBI"], ebi_floor)
            curves["EBD"] = max(
                curves["EBD"],
                hidden_curves["EBI"] + precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH),
            )
        if release_active and train.commanded_stop and actual_distance_to_stop > STOP_ACCURACY_TOL_M:
            release_floor = min(p_c, release_speed_profile(actual_distance_to_stop, release_speed))
            curves["P"] = max(curves["P"], release_floor)
            curves["W"] = max(curves["W"], min(w_c, curves["P"] + kmh_to_ms(1.0)))
            hidden_curves["OFF"] = max(hidden_curves["OFF"], curves["W"] + kmh_to_ms(0.5))
            hidden_curves["SBI"] = max(hidden_curves["SBI"], curves["P"] + kmh_to_ms(2.5))
            curves["SBD"] = max(curves["SBD"], hidden_curves["SBI"] + kmh_to_ms(CURVE_EPS_KMH))
            hidden_curves["EBI"] = max(hidden_curves["EBI"], curves["SBD"] + kmh_to_ms(CURVE_EPS_KMH))
            curves["EBD"] = max(curves["EBD"], hidden_curves["EBI"] + kmh_to_ms(EBI_SPEED_MARGIN_KMH))

        def keep_below(value: float, upper: float) -> float:
            if upper <= 0.0:
                return 0.0
            if value >= upper:
                return max(0.0, upper - kmh_to_ms(CURVE_EPS_KMH))
            return value

        hidden_curves["EBI"] = keep_below(hidden_curves["EBI"], curves["EBD"])
        curves["SBD"] = keep_below(curves["SBD"], hidden_curves["EBI"])
        hidden_curves["SBI"] = keep_below(hidden_curves["SBI"], curves["SBD"])
        hidden_curves["OFF"] = keep_below(hidden_curves["OFF"], hidden_curves["SBI"])
        curves["W"] = keep_below(curves["W"], hidden_curves["OFF"])
        curves["P"] = keep_below(curves["P"], curves["W"])
        i_gap_limit = max(kmh_to_ms(CURVE_EPS_KMH), i_margin_ms)
        hidden_curves["I"] = min(hidden_curves["I"], max(0.0, curves["P"] - i_gap_limit))
        cutoff_threshold = hidden_curves["OFF"]

        train.estimated_speed = estimated_speed
        train.vital_speed = vital_speed

        return ATPEnvelopeResult(
            control_speed=control_speed,
            actual_distance_to_stop=actual_distance_to_stop,
            distance_to_svl=distance_to_svl,
            svl_m=svl_pos,
            target_active=target_active,
            stop_target_active=stop_target_active,
            release_active=release_active,
            release_blend=release_blend,
            p_t=p_t,
            p_r=p_r,
            a_service=a_service,
            a_emergency=a_emergency,
            a_traction=a_traction,
            curves=curves,
            hidden_curves=hidden_curves,
            curve_mode=curve_mode,
            cutoff_threshold=cutoff_threshold,
            margin_dyn_m=margin_dyn_m,
            atp_service_brake_decel=atp_service_brake_decel,
            atp_emergency_brake_decel=atp_emergency_brake_decel,
            release_speed_kmh=release_speed_kmh,
        )


class ATOPilotingEngine:
    """Non-vital operating target generator that runs beneath the ATP envelope."""

    def compute(self, train: "Train", atp: ATPEnvelopeResult) -> ATOPilotingResult:
        ato_guard_limit = max(0.0, atp.hidden_curves["EBI"] - ato_ebi_guard_ms(train.vital_speed))
        ato_brake_prepare = atp.target_active and train.vital_speed >= max(0.0, atp.curves["P"] - kmh_to_ms(0.2))
        ato_curve_speed = min(atp.curves["P"], ato_guard_limit)
        ato_stop_limit = max_speed_with_buildup(
            atp.actual_distance_to_stop,
            atp.a_service,
            BRAKE_BUILDUP_S,
        )
        ato_tracking_margin = ato_tracking_margin_ms(atp.control_speed)
        ato_piloting_speed = max(
            0.0,
            ato_curve_speed
            - ato_tracking_margin
            - kmh_to_ms(max(0.0, -train.gradient) * DOWNHILL_P_BUFFER_KMH_PER_GRAD),
        )
        release_target = min(
            ato_piloting_speed,
            release_speed_profile(atp.actual_distance_to_stop, kmh_to_ms(RELEASE_SPEED_KMH)),
        )
        desired_target = ato_piloting_speed
        if atp.target_active or train.commanded_stop or atp.release_active:
            desired_target = min(desired_target, ato_stop_limit)
        if atp.target_active and not train.commanded_stop and not atp.release_active:
            if train.vital_speed > atp.curves["W"]:
                desired_target = min(desired_target, max(0.0, atp.curves["P"] - kmh_to_ms(1.0)))
            elif train.vital_speed > atp.curves["P"]:
                desired_target = min(desired_target, max(0.0, atp.curves["P"] - kmh_to_ms(2.0)))
        if atp.release_active:
            desired_target = min(desired_target, release_target)
        if atp.actual_distance_to_stop <= DOCKING_ZONE_M:
            if atp.actual_distance_to_stop <= STOP_ACCURACY_TOL_M:
                docking_limit = 0.0
            elif atp.actual_distance_to_stop <= FINAL_CREEP_ZONE_M:
                span_m = max(0.1, FINAL_CREEP_ZONE_M - STOP_ACCURACY_TOL_M)
                ratio = max(0.0, min(1.0, (atp.actual_distance_to_stop - STOP_ACCURACY_TOL_M) / span_m))
                docking_limit = lerp(0.0, kmh_to_ms(FINAL_CREEP_MIN_SPEED_KMH), ratio ** 0.7)
            else:
                span_m = max(0.1, DOCKING_ZONE_M - FINAL_CREEP_ZONE_M)
                ratio = max(0.0, min(1.0, (atp.actual_distance_to_stop - FINAL_CREEP_ZONE_M) / span_m))
                docking_limit = lerp(kmh_to_ms(FINAL_CREEP_MIN_SPEED_KMH), kmh_to_ms(DOCKING_SPEED_KMH), ratio ** 0.9)
            desired_target = min(desired_target, docking_limit)
        if atp.release_active:
            desired_target = min(desired_target, kmh_to_ms(CREEP_MAX_SPEED_KMH))
        if train.commanded_stop and STOP_ACCURACY_TOL_M < atp.actual_distance_to_stop <= RELEASE_HANDOVER_START_M:
            desired_target = min(
                desired_target,
                release_speed_profile(atp.actual_distance_to_stop, kmh_to_ms(RELEASE_SCAN_FAST_KMH)),
            )
        if (
            train.commanded_stop
            and train.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and atp.actual_distance_to_stop > STOP_ACCURACY_TOL_M
            and train.speed <= STANDSTILL_SPEED_EPS
        ):
            # After the single permitted jog has fully finished and the train is
            # back at standstill, do not let the final-approach minimum-speed
            # floor start another unintended creep.
            desired_target = 0.0
        ato_target_speed = train.ato_target_speed
        if desired_target > ato_target_speed:
            max_rise = traction_acceleration_ms2(train.speed) * DT
            ato_target_speed = min(desired_target, ato_target_speed + max_rise)
        else:
            max_drop = kmh_to_ms(ATO_TARGET_DROP_RATE_KMH_S) * DT if train.commanded_stop else float("inf")
            ato_target_speed = max(desired_target, ato_target_speed - max_drop)

        jog_active = False
        jog_used = train.jog_used
        if (
            train.commanded_stop
            and train.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE)
            and STOP_ACCURACY_TOL_M < atp.actual_distance_to_stop <= JOG_MAX_DIST_M
        ):
            jog_active = True
            jog_used = True
            ato_target_speed = max(ato_target_speed, kmh_to_ms(JOG_SPEED_KMH))
        elif atp.actual_distance_to_stop <= STOP_ACCURACY_TOL_M or atp.actual_distance_to_stop < 0.0:
            jog_active = False
        if not train.commanded_stop:
            jog_used = False

        return ATOPilotingResult(
            ato_brake_prepare=ato_brake_prepare,
            ato_curve_speed=ato_curve_speed,
            ato_piloting_speed=ato_piloting_speed,
            ato_target_speed=ato_target_speed,
            jog_active=jog_active,
            jog_used=jog_used,
        )


class Train:
    def __init__(self, train_cfg: Dict[str, float | str | None]):
        train_id = str(train_cfg["id"])
        start_pos = float(train_cfg["start_pos"])
        self.id = train_id
        self.pos = start_pos
        self.reported_pos = start_pos
        self.speed = 0.0
        self.filtered_speed = 0.0
        self.estimated_speed = 0.0
        self.vital_speed = 0.0
        self.length = float(train_cfg["length_m"])
        self.mass = float(train_cfg["mass_kg"])
        self.drive_mode = str(train_cfg.get("drive_mode", "ATO")).upper()
        self.requested_drive_mode = str(train_cfg.get("requested_drive_mode", self.drive_mode)).upper()
        self.mode_transition_reason = ""
        self.dcs_degraded_requested = False
        self.max_manual_speed_kmh = float(train_cfg.get("max_manual_speed_kmh", 45.0))
        self.dcs_mute_windows = [dict(window) for window in train_cfg.get("dcs_mute_windows", [])]
        self.dcs_muted = False
        self.track_profile = train_cfg["track_profile"]
        self.color = str(train_cfg["color"])
        self.eoa = 0.0
        self.psr_kmh = 25.0
        self.gradient = 0.0
        self.commanded_stop = False
        self.emergency_stop = False
        self.ato_state = "ATO_IDLE"
        self.atp_state = "ATP_OK"
        self.atp_brake = "NONE"
        self.atp_alert = "OK"
        self.traction_cutoff = False
        self.service_brake_latch = False
        self.emg_latch = False
        self.emg_ack = False
        self.emergency_recovery_hold = False
        self.trip_mode = False
        self.trip_reason = ""
        self.trip_protect_pos = start_pos
        self.trip_protect_rear_pos = start_pos - self.length
        self.atp_action = ""
        self.curves = {
            "P": 0.0,
            "W": 0.0,
            "SBD": 0.0,
            "EBD": 0.0,
        }
        self.hidden_curves = {
            "I": 0.0,
            "OFF": 0.0,
            "SBI": 0.0,
            "EBI": 0.0,
        }
        self.raw_curves = dict(self.curves)
        self.raw_hidden_curves = dict(self.hidden_curves)
        self._display_curves_initialized = False
        self._previous_display_curves_for_smoothing: Dict[str, float] | None = None
        self._previous_display_hidden_for_smoothing: Dict[str, float] | None = None
        self.curve_mode = "CEILING"
        self.cutoff_threshold = 0.0
        self.distance_to_eoa = 0.0
        self.safe_front_end_pos = start_pos
        self.ato_target_speed = 0.0
        self.ato_curve_speed = 0.0
        self.ato_piloting_speed = 0.0
        self.margin_dyn_m = 0.0
        self.atp_service_brake_decel = 0.0
        self.atp_emergency_brake_decel = 0.0
        self.limit_ahead_speed_kmh = 0.0
        self.limit_ahead_dist = float("inf")
        self.constraint_type = "NONE"
        self.constraint_target_speed_kmh = 0.0
        self.distance_to_constraint_m = float("inf")
        self.last_balise_pos = start_pos
        self.next_balise_pos = start_pos + BALISE_SPACING_M
        self.has_balise_fix = False
        self.pos_error_m = 0.0
        self.odometer_error_sign = 1.0 if (sum(ord(ch) for ch in self.id) % 2 == 0) else -1.0
        self.prev_pos = start_pos
        self.rollback_protection = False
        self.door_open_allowed = False
        self.headway_time_s = None
        self.headway_target_s = 0.0
        self.headway_planned_dispatch_s = None
        self.headway_dispatch_delay_s = 0.0
        self.headway_dispatch_released = False
        self.headway_actual_dispatched = False
        self.headway_hold_reason = ""
        self.protection_zone_id = None
        self.protection_lane = 0
        self.source_name = train_cfg.get("source_name")
        self.source_lane = None
        self.schedule_service_id = train_cfg.get("schedule_service_id")
        self.schedule_profile = str(train_cfg.get("schedule_profile", "") or "")
        self.schedule_records = [dict(record) for record in train_cfg.get("schedule_records", [])]
        self.schedule_planned_dispatch_s = train_cfg.get("schedule_planned_dispatch_s")
        self.station_lane = None
        self.assigned_station_id = None
        self.assigned_station_line_id = None
        self.departure_hold = False
        self.standstill_required = False
        self.standstill_anchor_pos = start_pos
        self.stop_target_pos = start_pos
        self.stop_beacon_pos = start_pos
        self.stop_beacon_seen = False
        self.beacon_lock_stop_pos = None
        self.jog_active = False
        self.jog_used = False
        self.manual_jog_requested = False
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_state = JOG_STATE_IDLE
        self.jog_used_for_current_stop = False
        self.current_stop_id = None
        self.active_stop_key = None
        self.jog_used_stop_key = None
        self.jog_event_counts = {}
        self.jog_stop_target_pos = None
        self.jog_start_pos = start_pos
        self.jog_profile_distance_m = 0.0
        self.prev_commanded_stop = False
        self.release_active = False
        self.release_speed_kmh = 0.0
        self.release_blend = 0.0
        self.release_p_target_ms = 0.0
        self.release_p_release_ms = 0.0
        self.prev_accel = 0.0
        self.zero_speed_detected = True
        self.standstill_monitoring = False
        self.rollback_monitoring = True
        self.door_authorized = False
        self.ato_hold_active = True
        self.ato_door_mode = "LOCKED"
        self.precise_stop_state = "ALIGNED"
        self.ato_brake_prepare = False
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.scheduled_stops = [dict(stop) for stop in train_cfg.get("scheduled_stops", [])]
        self.next_scheduled_stop_idx = 0
        self.active_scheduled_stop = None
        self.dwell_remaining_s = 0.0
        self.station_state = "COMPLETED_STOP"
        self.station_state_stop_key = None
        self.last_station_state_reason = ""
        self.last_station_idx = None
        self.assigned_platform = None
        self.last_dispatched_eoa = None
        self.last_dispatched_eoa_reason = ""
        self.station_reject_reason = ""
        self.beacon_position_locked = False
        self.safe_packet_age_s = 0.0
        self.safe_packet_valid = True
        self.dcs_fault_active = False
        self.ato_fault_active = False
        self.atp_fault_active = False
        self.collision_latched = False
        self.collision_partner_id = ""
        self.collision_overlap_m = 0.0
        self.analytics_distance_m = 0.0
        self.analytics_traction_work_j = 0.0
        self.analytics_brake_work_j = 0.0
        self.runtime_traction_force_n = 0.0
        self.runtime_brake_force_n = 0.0
        self.runtime_used_legacy_fallback = False
        self.ato_brake_mode = "none"
        self.last_sim_time_s = 0.0
        self.event_records = deque(maxlen=240)
        self.pending_event_records = deque(maxlen=80)
        self._last_atp_service_active = False
        self._last_atp_emergency_active = False
        self._last_door_authorized = False
        self._low_speed_ebi_counter = 0
        self._low_speed_guard_active = False
        self.cc = OnboardControlCenter(self.id, DCS_TIMEOUT_S, DCS_STARTUP_GRACE_S)
        self.atp_engine = ATPEnvelopeEngine()
        self.ato_engine = ATOPilotingEngine()

    def receive_safe_packet(self, packet: SafeMovementPacket, arrival_time_s: float):
        self.cc.receive_safe_packet(packet, arrival_time_s)

    def set_fault(self, subsystem: str, active: bool, now_s: float = 0.0):
        subsystem = subsystem.upper()
        if subsystem == "DCS":
            self.dcs_fault_active = bool(active)
            if active:
                self.dcs_mute_windows.append({"start_s": now_s, "end_s": now_s + 3600.0})
                self.dcs_muted = True
            else:
                self.dcs_mute_windows = []
                self.dcs_muted = False
                self.cc.watchdog.mark_received(now_s)
                self.safe_packet_valid = True
                self.dcs_degraded_requested = False
        elif subsystem == "ATO":
            self.ato_fault_active = bool(active)
            if active:
                self.drive_mode = "LMD"
                self.mode_transition_reason = "ATO fault: degraded to limited manual"
                self.ato_state = "ATO_FAULT"
            elif self.requested_drive_mode == "ATO" and self.safe_packet_valid:
                self.drive_mode = "ATO"
                self.mode_transition_reason = ""
        elif subsystem == "ATP":
            self.atp_fault_active = bool(active)
            if active:
                self.enter_trip_mode("ATP FAULT", self.reported_pos)
                self.atp_state = "ATP_TRIP"
                self.atp_alert = "ATP FAULT"
                self.atp_action = "EBI"
                self.atp_brake = "EMERGENCY"
                self.emg_latch = True
            else:
                self.atp_fault_active = False

    def compute_ato_pid_accel(self, error: float, control_speed: float, a_service: float, a_traction: float) -> float:
        if abs(error) < kmh_to_ms(0.2):
            error = 0.0
        if error == 0.0:
            self.ato_pid_integral *= 0.6
        else:
            self.ato_pid_integral += error * DT
            self.ato_pid_integral = max(-ATO_PID_INT_LIMIT_MS, min(ATO_PID_INT_LIMIT_MS, self.ato_pid_integral))
        derivative = (error - self.ato_pid_prev_error) / max(DT, 1e-6)
        self.ato_pid_prev_error = error
        kp, ki, kd = ato_pid_gains(control_speed)
        pid_accel = kp * error + ki * self.ato_pid_integral + kd * derivative
        cmd_with_grade = pid_accel + G * self.gradient
        unsaturated_cmd = cmd_with_grade
        if cmd_with_grade > 0.0:
            cmd_with_grade = min(a_traction, cmd_with_grade)
        else:
            cmd_with_grade = max(-a_service, cmd_with_grade)
        saturated_high = unsaturated_cmd > a_traction and error > 0.0
        saturated_low = unsaturated_cmd < -a_service and error < 0.0
        if saturated_high or saturated_low:
            self.ato_pid_integral *= 0.85
        return cmd_with_grade

    def enter_trip_mode(self, reason: str, protect_pos: float | None = None):
        if protect_pos is None:
            protect_pos = self.reported_pos
        self.trip_mode = True
        self.trip_reason = reason
        self.trip_protect_pos = min(self.eoa, protect_pos)
        self.trip_protect_rear_pos = min(self.safe_rear_end_pos(), self.trip_protect_pos - self.length)
        self.emergency_stop = True
        self.emg_latch = True
        self.emg_ack = False
        self.release_active = False
        self.service_brake_latch = False
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0

    def clear_trip_mode(self):
        self.trip_mode = False
        self.trip_reason = ""
        self.emergency_stop = False
        self.trip_protect_rear_pos = self.safe_rear_end_pos()

    def acknowledge_emergency_safe(self):
        if self.speed > 0.01:
            return
        self.emg_latch = False
        self.emg_ack = False
        self.emergency_stop = False
        self.service_brake_latch = False
        self.clear_trip_mode()
        self.emergency_recovery_hold = True
        self.standstill_required = True
        self.standstill_anchor_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0

    def reset_non_emergency_stop_latches(self):
        self.service_brake_latch = False
        if not self.emg_latch and not self.trip_mode:
            self.emergency_stop = False
            self.emergency_recovery_hold = False
            self.clear_trip_mode()

    def apply_display_curve_smoothing(self, raw_curves: Dict[str, float], raw_hidden_curves: Dict[str, float]):
        if not self._display_curves_initialized:
            self.curves = dict(raw_curves)
            self.hidden_curves = dict(raw_hidden_curves)
            self._display_curves_initialized = True
            return

        precise_stop_display = (
            self.commanded_stop
            and not self.trip_mode
            and (
                0.0 <= self.distance_to_stop_target() <= PRECISE_STOP_SERVICE_BAND_M
                or (
                    0.0 <= self.distance_to_stop_target() <= JOG_MAX_DIST_M
                    and (self.zero_speed_detected or self.jog_state == JOG_STATE_ACTIVE)
                )
            )
        )
        if precise_stop_display:
            self.curves = dict(raw_curves)
            self.hidden_curves = dict(raw_hidden_curves)
            return

        previous_curves = self._previous_display_curves_for_smoothing or self.curves
        previous_hidden = self._previous_display_hidden_for_smoothing or self.hidden_curves
        previous = {
            "P": previous_curves.get("P", 0.0),
            "W": previous_curves.get("W", 0.0),
            "SBD": previous_curves.get("SBD", 0.0),
            "EBD": previous_curves.get("EBD", 0.0),
            "I": previous_hidden.get("I", 0.0),
            "OFF": previous_hidden.get("OFF", 0.0),
            "SBI": previous_hidden.get("SBI", 0.0),
            "EBI": previous_hidden.get("EBI", 0.0),
        }
        raw_all = {
            "P": raw_curves.get("P", 0.0),
            "W": raw_curves.get("W", 0.0),
            "SBD": raw_curves.get("SBD", 0.0),
            "EBD": raw_curves.get("EBD", 0.0),
            "I": raw_hidden_curves.get("I", 0.0),
            "OFF": raw_hidden_curves.get("OFF", 0.0),
            "SBI": raw_hidden_curves.get("SBI", 0.0),
            "EBI": raw_hidden_curves.get("EBI", 0.0),
        }

        smoothed: Dict[str, float] = {}
        bypass_all = self.trip_mode or self.emergency_stop
        for key, raw_value in raw_all.items():
            prev_value = previous.get(key, raw_value)
            if bypass_all or raw_value <= 0.0 or prev_value <= 0.0:
                smoothed[key] = raw_value
                continue
            if raw_value >= prev_value:
                max_rise = kmh_to_ms(CURVE_DISPLAY_RISE_RATE_KMH_S.get(key, 24.0)) * DT
                smoothed[key] = min(raw_value, prev_value + max_rise)
                continue

            safety_gap_kmh = 1.0 if key in ("EBI", "EBD") else 0.5
            raw_too_close_to_train = raw_value <= self.vital_speed + kmh_to_ms(safety_gap_kmh)
            if raw_too_close_to_train:
                smoothed[key] = raw_value
                continue

            max_drop = kmh_to_ms(CURVE_DISPLAY_DROP_RATE_KMH_S.get(key, 16.0)) * DT
            smoothed[key] = max(raw_value, prev_value - max_drop)

        def keep_below_display(value: float, upper: float) -> float:
            if upper <= 0.0:
                return 0.0
            if value >= upper:
                return max(0.0, upper - kmh_to_ms(CURVE_EPS_KMH))
            return value

        smoothed["EBI"] = keep_below_display(smoothed["EBI"], smoothed["EBD"])
        smoothed["SBD"] = keep_below_display(smoothed["SBD"], smoothed["EBI"])
        smoothed["SBI"] = keep_below_display(smoothed["SBI"], smoothed["SBD"])
        smoothed["OFF"] = keep_below_display(smoothed["OFF"], smoothed["SBI"])
        smoothed["W"] = keep_below_display(smoothed["W"], smoothed["OFF"])
        smoothed["P"] = keep_below_display(smoothed["P"], smoothed["W"])
        smoothed["I"] = min(smoothed["I"], max(0.0, smoothed["P"] - kmh_to_ms(CURVE_EPS_KMH)))

        self.curves = {"P": smoothed["P"], "W": smoothed["W"], "SBD": smoothed["SBD"], "EBD": smoothed["EBD"]}
        self.hidden_curves = {
            "I": smoothed["I"],
            "OFF": smoothed["OFF"],
            "SBI": smoothed["SBI"],
            "EBI": smoothed["EBI"],
        }

    def resume_after_emergency(self):
        if not self.emergency_recovery_hold:
            return
        self.emergency_recovery_hold = False
        self.emg_ack = False
        self.emergency_stop = False
        self.service_brake_latch = False
        self.clear_trip_mode()
        self.standstill_required = False
        self.standstill_anchor_pos = self.pos
        self.prev_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0
        if self.requested_drive_mode == "ATO" and self.safe_packet_valid and not self.dcs_degraded_requested:
            self.drive_mode = "ATO"
            self.mode_transition_reason = ""

    def distance_to_active_stop(self) -> float:
        if self.active_scheduled_stop is None:
            return float("inf")
        return float(self.active_scheduled_stop["pos_m"]) - self.pos

    def in_station_calibration_zone(self) -> bool:
        return self.commanded_stop and 0.0 <= self.distance_to_active_stop() <= STATION_BALISE_ZONE_M

    def effective_position_uncertainty_m(self) -> float:
        if self.beacon_position_locked:
            return PRECISE_STOP_POS_UNCERT_M
        if self.in_station_calibration_zone():
            return STATION_POS_UNCERT_M
        if self.has_balise_fix:
            return BALISE_POS_UNCERT_M
        return POS_UNCERT_M

    def safe_rear_end_pos(self) -> float:
        """Conservative rear-end location used by ZC for moving-block MAL."""
        return self.reported_pos - self.length - self.effective_position_uncertainty_m()

    def effective_balise_spacing_m(self) -> float:
        if self.in_station_calibration_zone():
            return STATION_BALISE_SPACING_M
        return BALISE_SPACING_M

    def effective_balise_error_max_m(self) -> float:
        if self.beacon_position_locked:
            return 0.0
        if self.in_station_calibration_zone():
            return STATION_BALISE_ERROR_MAX_M
        return BALISE_ERROR_MAX_M

    def sync_reported_position(self):
        self.reported_pos = self.pos
        self.pos_error_m = 0.0

    def distance_to_stop_target(self) -> float:
        return self.stop_target_pos - self.pos

    def scheduled_stop_target_pos(self) -> float | None:
        if self.active_scheduled_stop is None:
            return None
        return float(self.active_scheduled_stop["pos_m"])

    def resolve_stop_target(self, authority_target_pos: float) -> float:
        station_target_pos = self.scheduled_stop_target_pos()
        if (
            self.active_scheduled_stop is not None
            and self.station_state in {
                "APPROACHING_STATION",
                "ROUTE_ASSIGNED",
                "DOCKING",
                "STOPPED_AT_PLATFORM",
                "DWELLING",
                "READY_TO_DEPART",
            }
            and station_target_pos is not None
            and authority_target_pos + JOG_WINDOW_EPS_M >= station_target_pos
        ):
            # For a station stop, keep the platform marker as the commanded target
            # while authority still covers it. EOA/SvL continues to protect ATP.
            return station_target_pos
        return authority_target_pos

    def stop_identity(self) -> str:
        if self.active_scheduled_stop is not None:
            name = self.active_scheduled_stop.get("name", "STOP")
            pos_m = float(self.active_scheduled_stop["pos_m"])
            return f"{name}@{pos_m:.3f}"
        return f"target@{self.stop_target_pos:.3f}"

    def get_current_stop_key(self) -> str | None:
        if not self.commanded_stop:
            return None
        return self.stop_identity()

    def jog_limit_m(self) -> float:
        return min(JOG_MAX_DIST_M, MANUAL_JOG_WINDOW_M)

    def docking_jog_limit_m(self) -> float:
        return min(JOG_MAX_DIST_M, max(MANUAL_JOG_WINDOW_M, AUTO_DOCKING_JOG_WINDOW_M))

    def _curve_snapshot(self) -> Dict[str, float]:
        return {
            "P": ms_to_kmh(self.curves.get("P", 0.0)),
            "W": ms_to_kmh(self.curves.get("W", 0.0)),
            "SBI": ms_to_kmh(self.hidden_curves.get("SBI", 0.0)),
            "EBI": ms_to_kmh(self.hidden_curves.get("EBI", 0.0)),
            "EBD": ms_to_kmh(self.curves.get("EBD", 0.0)),
        }

    def log_event(self, event: str, reason: str = "", **extra):
        stop_error_m = self.pos - self.stop_target_pos
        record = {
            "sim_time": self.last_sim_time_s,
            "train_id": self.id,
            "event": event,
            "reason": reason,
            "stop_id": self.current_stop_id or self.stop_identity(),
            "current_pos": self.pos,
            "stop_target_pos": self.stop_target_pos,
            "remaining": self.stop_target_pos - self.pos,
            "stop_error": stop_error_m,
            "speed_kmh": ms_to_kmh(self.speed),
            "vital_speed_kmh": ms_to_kmh(self.vital_speed),
            "eoa_m": self.eoa,
            "distance_to_eoa": self.distance_to_eoa,
            "station_state": self.station_state,
            "assigned_line": self.assigned_platform,
            "dwell_remaining": self.dwell_remaining_s,
            "curve_mode": self.curve_mode,
            "jog_state": self.jog_state,
            "curves": self._curve_snapshot(),
        }
        record.update(extra)
        self.event_records.append(record)
        self.pending_event_records.append(record)

    def pop_pending_events(self):
        events = list(self.pending_event_records)
        self.pending_event_records.clear()
        return events

    def _set_jog_state(self, state: str, reason: str = ""):
        if self.jog_state == state and not reason:
            return
        self.jog_state = state
        self.precise_jog_in_progress = state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE)
        self.precise_jog_completed = state == JOG_STATE_COMPLETED
        self.jog_active = state == JOG_STATE_ACTIVE
        if state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE, JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED):
            self.jog_used = True
            self.jog_used_for_current_stop = True
            self.jog_used_stop_key = self.active_stop_key or self.current_stop_id or self.stop_identity()
        event_by_state = {
            JOG_STATE_REQUESTED: "JOG_REQUESTED",
            JOG_STATE_ACTIVE: "JOG_STARTED",
            JOG_STATE_COMPLETED: "JOG_COMPLETED",
            JOG_STATE_FAILED_LOCKED: "JOG_FAILED_LOCKED",
        }
        event = event_by_state.get(state)
        if event:
            self.log_event(event, reason)
            if event == "JOG_STARTED":
                key = self.current_stop_id or self.stop_identity()
                self.jog_event_counts[key] = self.jog_event_counts.get(key, 0) + 1

    def _reset_jog_for_stop(self, stop_id: str | None):
        self.current_stop_id = stop_id
        self.active_stop_key = stop_id
        self.jog_state = JOG_STATE_IDLE
        self.jog_used = False
        self.jog_used_for_current_stop = False
        self.jog_used_stop_key = None
        self.manual_jog_requested = False
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_active = False
        self.jog_stop_target_pos = None
        self.jog_start_pos = self.pos
        self.jog_profile_distance_m = 0.0

    def can_request_precise_jog(self) -> bool:
        key = self.get_current_stop_key()
        remaining_m = self.distance_to_stop_target()
        eoa_error_m = abs(self.eoa - self.pos)
        jog_limit = self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
        station_target_pos = self.scheduled_stop_target_pos()
        if station_target_pos is not None and self.stop_target_pos < station_target_pos - JOG_WINDOW_EPS_M:
            return False
        return (
            key is not None
            and self.commanded_stop
            and (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS)
            and not self.door_authorized
            and not self.jog_active
            and not self.jog_used
            and self.jog_state == JOG_STATE_IDLE
            and not self.jog_used_for_current_stop
            and self.jog_used_stop_key != key
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
            and STOP_ACCURACY_TOL_M < remaining_m <= jog_limit
            and eoa_error_m <= jog_limit
        )

    def request_precise_jog(self) -> bool:
        stop_id = self.get_current_stop_key()
        if stop_id is None:
            self.log_event("JOG_IGNORED_NO_STOP")
            return False
        if self.jog_used_stop_key == stop_id or self.jog_used_for_current_stop:
            self.log_event("JOG_IGNORED_ALREADY_USED", f"state={self.jog_state}")
            return False
        if self.jog_state != JOG_STATE_IDLE:
            self.log_event("JOG_IGNORED_LOCKED", f"state={self.jog_state}")
            return False
        if not (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS):
            self.log_event("JOG_IGNORED_NOT_ZERO_SPEED")
            return False
        if self.door_authorized:
            self.log_event("JOG_IGNORED_DOOR_ALREADY_AUTHORIZED")
            return False
        if self.emergency_stop or self.emg_latch or self.trip_mode or self.emergency_recovery_hold:
            self.log_event("JOG_IGNORED_ATP_TRIP")
            return False
        remaining_m = self.distance_to_stop_target()
        if remaining_m <= 0.0:
            self.current_stop_id = stop_id
            self.active_stop_key = stop_id
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, f"remaining={remaining_m:.3f}")
            self.log_event("JOG_IGNORED_OVERRUN", f"remaining={remaining_m:.3f}")
            return False
        if remaining_m <= STOP_ACCURACY_TOL_M:
            self.log_event("JOG_IGNORED_ALREADY_ALIGNED", f"remaining={remaining_m:.3f}")
            return False
        if remaining_m > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M:
            self.log_event("JOG_IGNORED_TOO_FAR", f"remaining={remaining_m:.3f}")
            return False
        if abs(self.eoa - self.pos) > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M or not self.can_request_precise_jog():
            self.log_event("JOG_IGNORED_INVALID_REMAINING", "preconditions")
            return False
        self.current_stop_id = stop_id
        self.active_stop_key = stop_id
        self.manual_jog_requested = False
        self.jog_used = True
        self.jog_used_for_current_stop = True
        self.jog_used_stop_key = stop_id
        self.jog_stop_target_pos = self.stop_target_pos
        self.jog_start_pos = self.pos
        self.jog_profile_distance_m = remaining_m
        self.standstill_required = False
        self.standstill_anchor_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0
        self._set_jog_state(JOG_STATE_ACTIVE)
        return True

    def retry_failed_jog_if_safe(self) -> bool:
        if self.jog_state != JOG_STATE_FAILED_LOCKED:
            return False
        key = self.get_current_stop_key()
        remaining_m = self.distance_to_stop_target()
        jog_limit = self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
        if not (
            key is not None
            and self.commanded_stop
            and (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS)
            and not self.door_authorized
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
            and STOP_ACCURACY_TOL_M < remaining_m <= jog_limit
            and abs(self.eoa - self.pos) <= jog_limit
        ):
            return False
        self.log_event("JOG_RETRY_SAFE", f"remaining={remaining_m:.3f}")
        self.jog_state = JOG_STATE_IDLE
        self.jog_used = False
        self.jog_used_for_current_stop = False
        self.jog_used_stop_key = None
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_active = False
        return True

    def _update_door_authorization(self):
        brake_applied_for_hold = self.standstill_required or self.service_brake_latch or self.emg_latch or self.commanded_stop
        station_target_pos = self.scheduled_stop_target_pos()
        station_stop_ready = (
            station_target_pos is None
            or (
                self.station_lane is not None
                and self.stop_target_pos >= station_target_pos - JOG_WINDOW_EPS_M
                and abs(self.pos - station_target_pos) <= STOP_ACCURACY_TOL_M
            )
        )
        self.door_authorized = (
            self.zero_speed_detected
            and brake_applied_for_hold
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and station_stop_ready
            and abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M
        )
        self.door_open_allowed = self.door_authorized
        if self.door_authorized and not self._last_door_authorized and self.jog_state == JOG_STATE_COMPLETED:
            self.log_event("DOOR_AUTHORIZED_AFTER_JOG")
        self._last_door_authorized = self.door_authorized

    def _finalize_jog_step(self):
        self.zero_speed_detected = self.speed <= STANDSTILL_SPEED_EPS
        self.standstill_monitoring = self.standstill_required or self.zero_speed_detected
        self.rollback_monitoring = self.zero_speed_detected or self.commanded_stop or self.emg_latch
        self._update_door_authorization()
        self.ato_hold_active = self.zero_speed_detected and (
            self.standstill_required or self.ato_target_speed <= STANDSTILL_SPEED_EPS
        )
        if self.door_authorized:
            self.ato_door_mode = "ENABLE"
        elif self.ato_hold_active:
            self.ato_door_mode = "READY"
        else:
            self.ato_door_mode = "LOCKED"
        if self.jog_state == JOG_STATE_ACTIVE:
            self.precise_stop_state = "JOG"
            self.ato_state = "ATO_JOG"
        elif self.jog_state == JOG_STATE_FAILED_LOCKED:
            self.precise_stop_state = "JOG_FAILED"
            self.ato_state = "ATO_HOLD"
        elif abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M:
            self.precise_stop_state = "ALIGNED"
            self.ato_state = "ATO_HOLD"
        else:
            self.precise_stop_state = "WAIT_JOG"
            self.ato_state = "ATO_HOLD"
        if self.trip_mode:
            self.atp_state = "ATP_TRIP"
            self.atp_alert = self.trip_reason or "TRAIN TRIP"
            self.atp_action = "EBI"
        elif self.emg_latch or self.emergency_stop:
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "JOG PROTECTION"
            self.atp_action = "EBI"
        elif self.door_authorized:
            self.atp_state = "ATP_STANDSTILL"
            self.atp_alert = "DOOR ENABLE"
            self.atp_action = ""
        else:
            self.atp_state = "ATP_STANDSTILL" if self.zero_speed_detected else "ATP_OK"
            self.atp_alert = "STANDSTILL MON" if self.zero_speed_detected else "OK"
            self.atp_action = ""
        self.prev_pos = self.pos
        self.prev_commanded_stop = self.commanded_stop

    def update_jog(self, dt_s: float) -> bool:
        if self.jog_state != JOG_STATE_ACTIVE:
            return False
        key = self.get_current_stop_key()
        target = self.stop_target_pos
        remaining_m = target - self.pos
        if key is None:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "no_stop")
            self.log_event("JOG_FAILED_LOCKED", "no_stop")
            self._finalize_jog_step()
            return True
        if self.emergency_stop or self.emg_latch or self.trip_mode:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "atp_trip")
            self.log_event("JOG_FAILED_LOCKED", "atp_trip")
            self._finalize_jog_step()
            return True
        if remaining_m <= STOP_ACCURACY_TOL_M and remaining_m >= 0.0:
            self.pos = target
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "within_tolerance")
            if abs(self.pos - target) > STOP_ACCURACY_TOL_M:
                self.log_event("JOG_COMPLETED_NOT_ALIGNED")
            self._finalize_jog_step()
            return True
        if remaining_m < 0.0:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "overrun")
            self.log_event("JOG_FAILED_OVERRUN", f"remaining={remaining_m:.3f}")
            self._finalize_jog_step()
            return True
        if remaining_m > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "too_far")
            self.log_event("JOG_FAILED_TRUE_TOO_FAR", f"remaining={remaining_m:.3f}")
            self._finalize_jog_step()
            return True
        total_m = max(self.jog_profile_distance_m, target - self.jog_start_pos, remaining_m)
        travelled_m = max(0.0, self.pos - self.jog_start_pos)
        half_m = max(0.001, 0.5 * total_m)
        configured_jog_speed_ms = kmh_to_ms(JOG_SPEED_KMH)
        distance_limited_speed_ms = (2.0 * JOG_PROFILE_ACCEL_MS2 * half_m) ** 0.5
        max_profile_speed_ms = min(configured_jog_speed_ms, distance_limited_speed_ms)
        speed_limit_source = "configured_jog_speed" if configured_jog_speed_ms <= distance_limited_speed_ms else "distance_accel_profile"
        if travelled_m < half_m:
            commanded_accel = JOG_PROFILE_ACCEL_MS2
            profile_phase = "ACCEL"
        else:
            commanded_accel = -JOG_PROFILE_ACCEL_MS2
            profile_phase = "BRAKE"
        next_speed = max(0.0, min(max_profile_speed_ms, self.speed + commanded_accel * dt_s))
        target_speed_reason = speed_limit_source
        if travelled_m >= half_m and remaining_m <= STOP_ACCURACY_TOL_M + max(0.0, self.speed * dt_s):
            next_speed = 0.0
            target_speed_reason = "final_tolerance_stop"
        self.log_event(
            "JOG_PROFILE_TRACE",
            target_speed_reason,
            configured_jog_speed_kmh=JOG_SPEED_KMH,
            jog_target_speed_kmh=ms_to_kmh(max_profile_speed_ms),
            distance_limited_speed_kmh=ms_to_kmh(distance_limited_speed_ms),
            actual_speed_kmh=ms_to_kmh(self.speed),
            next_speed_kmh=ms_to_kmh(next_speed),
            commanded_accel_ms2=commanded_accel,
            profile_phase=profile_phase,
            travelled_m=travelled_m,
            profile_distance_m=total_m,
            half_distance_m=half_m,
            atp_p_kmh=ms_to_kmh(self.curves.get("P", 0.0)),
            atp_w_kmh=ms_to_kmh(self.curves.get("W", 0.0)),
            atp_sbi_kmh=ms_to_kmh(self.hidden_curves.get("SBI", 0.0)),
            atp_ebi_kmh=ms_to_kmh(self.hidden_curves.get("EBI", 0.0)),
            selected_reference="jog_motion_profile",
        )
        step_m = 0.5 * (self.speed + next_speed) * dt_s
        if step_m >= remaining_m or next_speed <= STANDSTILL_SPEED_EPS and travelled_m >= half_m:
            self.pos = target
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "clamped_at_target")
            self.log_event("JOG_COMPLETED_CLAMPED_TO_TARGET")
            self._finalize_jog_step()
            return True
        self.pos += step_m
        self.reported_pos = self.pos
        self.pos_error_m = 0.0
        self.speed = next_speed
        self.prev_accel = commanded_accel
        self.ato_target_speed = 0.0
        self.standstill_required = False
        self.jog_active = True
        self.log_event("JOG_CREEP_ACTIVE", f"remaining={remaining_m:.3f}")
        self.log_event("JOG_ACTIVE_STEP")
        self._finalize_jog_step()
        return True

    def update_reported_position(self):
        spacing_m = self.effective_balise_spacing_m()
        error_cap_m = self.effective_balise_error_max_m()
        if self.pos >= self.next_balise_pos:
            self.last_balise_pos = self.next_balise_pos
            self.next_balise_pos += spacing_m
            self.has_balise_fix = True
            self.pos_error_m = 0.0
            self.odometer_error_sign *= -1.0

        if self.beacon_position_locked:
            self.sync_reported_position()
            return

        dist_since = max(0.0, self.pos - self.last_balise_pos)
        error = min(error_cap_m, ODOMETER_ERROR_RATE * dist_since)
        self.pos_error_m = self.odometer_error_sign * error
        self.reported_pos = self.pos + self.pos_error_m

    def step(self, now_s: float):
        self.last_sim_time_s = now_s
        self.cc.apply_to_train(self, now_s)
        station_stop_eoa = None
        if self.commanded_stop and self.active_scheduled_stop is not None and self.station_lane is not None:
            stop_pos = float(self.active_scheduled_stop["pos_m"])
            station_end = stop_pos + float(self.active_scheduled_stop.get("length_m", 160.0)) / 2.0
            if self.pos <= station_end:
                station_stop_eoa = stop_pos - (STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                self.eoa = min(self.eoa, station_stop_eoa)
                remaining_to_station_stop = stop_pos - self.pos
                final_station_capture = (
                    -STOP_ACCURACY_TOL_M <= remaining_to_station_stop <= self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
                    and self.speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH + 0.5)
                    and not self.trip_mode
                    and not self.emergency_stop
                    and not self.emg_latch
                )
                if final_station_capture:
                    self.eoa = max(self.eoa, station_stop_eoa)
        hard_departure_hold = self.departure_hold and self.protection_zone_id == "SOURCE"
        stop_short_pending = (
            self.commanded_stop
            and self.distance_to_stop_target() > STOP_ACCURACY_TOL_M
        )
        if hard_departure_hold and self.speed <= kmh_to_ms(1.0) and not stop_short_pending:
            self.speed = 0.0
            self.filtered_speed = 0.0
            self.prev_accel = 0.0
            if not self.standstill_required:
                self.standstill_anchor_pos = self.pos
            self.standstill_required = True
        elif (
            stop_short_pending
            and self.dwell_remaining_s <= 0.0
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
        ):
            self.standstill_required = False
        elif not (self.commanded_stop or self.emg_latch or self.trip_mode or self.emergency_recovery_hold):
            self.standstill_required = False
        self.filtered_speed = low_pass_step(self.filtered_speed, self.speed, SPEED_FILTER_TAU_S, DT)
        self.atp_brake = "NONE"
        self.atp_action = ""
        self.traction_cutoff = False
        if self.ato_fault_active:
            self.drive_mode = "LMD"
            self.ato_state = "ATO_FAULT"
            self.mode_transition_reason = "ATO fault: degraded to limited manual"
        if self.atp_fault_active:
            self.enter_trip_mode("ATP FAULT", self.reported_pos)
            self.atp_state = "ATP_TRIP"
            self.atp_alert = "ATP FAULT"
            self.atp_action = "EBI"
            self.atp_brake = "EMERGENCY"
            self.emg_latch = True
        if not self.safe_packet_valid:
            if self.drive_mode == "ATO":
                self.drive_mode = "CMD25"
                self.dcs_degraded_requested = True
                self.mode_transition_reason = "DCS timeout: ATO inhibited, restricted recovery required"
            self.enter_trip_mode("DCS TIMEOUT", self.reported_pos)

        position_uncertainty_m = self.effective_position_uncertainty_m()
        safe_margin = max(position_uncertainty_m, abs(self.pos_error_m))
        self.safe_front_end_pos = self.reported_pos + safe_margin
        protected_eoa = self.eoa
        if self.trip_mode:
            protected_eoa = min(protected_eoa, self.trip_protect_pos)
        dwell_stop_hold = self.dwell_remaining_s > 0.0 and self.standstill_required
        if dwell_stop_hold:
            held_eoa = self.standstill_anchor_pos + STOP_TARGET_OFFSET_M - STOP_SVL_OFFSET_M
            protected_eoa = max(protected_eoa, held_eoa)
        target_eoa = protected_eoa - 1.0
        self.distance_to_eoa = target_eoa - self.safe_front_end_pos
        if self.distance_to_eoa < self.limit_ahead_dist:
            self.constraint_type = "STOP"
            self.constraint_target_speed_kmh = 0.0
            self.distance_to_constraint_m = self.distance_to_eoa
        elif self.limit_ahead_dist != float("inf"):
            self.constraint_type = "SPEED_REDUCTION"
            self.constraint_target_speed_kmh = self.limit_ahead_speed_kmh
            self.distance_to_constraint_m = self.limit_ahead_dist
        else:
            self.constraint_type = "NONE"
            self.constraint_target_speed_kmh = self.psr_kmh
            self.distance_to_constraint_m = float("inf")

        active_stop_eoa = protected_eoa if dwell_stop_hold else self.eoa
        svl_pos = active_stop_eoa + STOP_SVL_OFFSET_M
        distance_to_svl = svl_pos - self.reported_pos
        self.stop_target_pos = self.resolve_stop_target(svl_pos - STOP_TARGET_OFFSET_M)
        self.stop_beacon_pos = self.stop_target_pos - STOP_BEACON_OFFSET_M
        actual_distance_to_stop = self.stop_target_pos - self.pos
        self.release_active = False
        current_stop_id = self.stop_identity() if self.commanded_stop else None
        if current_stop_id is None:
            if self.current_stop_id is not None or self.jog_state != JOG_STATE_IDLE or self.jog_used_for_current_stop:
                self._reset_jog_for_stop(None)
        elif self.current_stop_id != current_stop_id:
            self._reset_jog_for_stop(current_stop_id)

        if self.pos >= self.stop_beacon_pos and not self.stop_beacon_seen:
            # Station stop beacon clears accumulated odometry error near the platform.
            self.sync_reported_position()
            self.has_balise_fix = True
            self.last_balise_pos = self.pos
            self.next_balise_pos = self.pos + STATION_BALISE_SPACING_M
            self.stop_beacon_seen = True
            self.beacon_position_locked = True
            self.beacon_lock_stop_pos = self.stop_target_pos
        elif self.pos < self.stop_beacon_pos - 2.0:
            self.stop_beacon_seen = False
        if self.beacon_position_locked:
            locked_stop_pos = self.beacon_lock_stop_pos if self.beacon_lock_stop_pos is not None else self.stop_target_pos
            distance_from_locked_stop = self.pos - locked_stop_pos
            if distance_from_locked_stop < -STATION_BALISE_ZONE_M or distance_from_locked_stop > BEACON_LOCK_RELEASE_MARGIN_M:
                self.beacon_position_locked = False
                self.beacon_lock_stop_pos = None
            else:
                self.sync_reported_position()

        atp = self.atp_engine.compute(self)
        self.release_active = atp.release_active
        self.release_speed_kmh = atp.release_speed_kmh
        self.release_blend = atp.release_blend
        self.release_p_target_ms = atp.p_t
        self.release_p_release_ms = atp.p_r
        self._previous_display_curves_for_smoothing = dict(self.curves)
        self._previous_display_hidden_for_smoothing = dict(self.hidden_curves)
        self.raw_curves = dict(atp.curves)
        self.raw_hidden_curves = dict(atp.hidden_curves)
        self.apply_display_curve_smoothing(self.raw_curves, self.raw_hidden_curves)
        stop_ebi_floor_active = (
            self.commanded_stop
            and abs(atp.actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if stop_ebi_floor_active:
            self.hidden_curves["EBI"] = max(self.hidden_curves["EBI"], kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            self.curves["EBD"] = max(self.curves["EBD"], self.hidden_curves["EBI"])
            self.raw_hidden_curves["EBI"] = self.hidden_curves["EBI"]
            self.raw_curves["EBD"] = self.curves["EBD"]
        atp_supervision = replace(
            atp,
            curves=dict(self.curves),
            hidden_curves=dict(self.hidden_curves),
            cutoff_threshold=self.hidden_curves.get("OFF", atp.cutoff_threshold),
        )
        self.curve_mode = atp.curve_mode
        self.cutoff_threshold = atp_supervision.cutoff_threshold
        self.margin_dyn_m = atp.margin_dyn_m
        self.atp_service_brake_decel = atp.atp_service_brake_decel
        self.atp_emergency_brake_decel = atp.atp_emergency_brake_decel

        self.retry_failed_jog_if_safe()
        if self.can_request_precise_jog():
            self.request_precise_jog()

        if self.jog_state == JOG_STATE_ACTIVE and self.update_jog(DT):
            return

        ato = self.ato_engine.compute(self, atp_supervision)
        self.ato_brake_prepare = ato.ato_brake_prepare
        self.ato_curve_speed = ato.ato_curve_speed
        self.ato_piloting_speed = ato.ato_piloting_speed
        self.ato_target_speed = ato.ato_target_speed
        self.jog_active = ato.jog_active
        self.jog_used = ato.jog_used
        if self.jog_state == JOG_STATE_REQUESTED and self.jog_active:
            self._set_jog_state(JOG_STATE_ACTIVE)
        if self.jog_active or self.jog_used or not self.commanded_stop:
            self.manual_jog_requested = False
        if self.drive_mode == "LMD":
            manual_limit = min(kmh_to_ms(self.max_manual_speed_kmh), atp_supervision.curves["P"] - ato_tracking_margin_ms(atp_supervision.control_speed))
            self.ato_target_speed = max(0.0, manual_limit)
            self.jog_active = False
        elif self.drive_mode == "CMD25":
            degraded_limit = min(kmh_to_ms(25.0), atp_supervision.curves["P"] - ato_tracking_margin_ms(atp_supervision.control_speed))
            self.ato_target_speed = max(0.0, degraded_limit)
        if self.emergency_recovery_hold:
            self.ato_target_speed = 0.0
            self.jog_active = False
        if hard_departure_hold:
            self.ato_target_speed = 0.0
            self.jog_active = False
        if self.emergency_recovery_hold or self.emg_latch or self.trip_mode:
            if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, "interrupted_by_protection")
        if self.precise_jog_in_progress and self.jog_stop_target_pos is None:
            self.jog_stop_target_pos = self.stop_target_pos

        control_speed = atp_supervision.control_speed
        actual_distance_to_stop = atp_supervision.actual_distance_to_stop
        distance_to_svl = atp_supervision.distance_to_svl
        a_service = atp_supervision.a_service
        a_emergency = atp_supervision.a_emergency
        a_traction = atp_supervision.a_traction
        coast_decel = COAST_BASE_DECEL + COAST_SPEED_GAIN * control_speed

        stop_within_tolerance = abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
        stop_overshoot = actual_distance_to_stop < -STOP_ACCURACY_TOL_M
        if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
            if stop_within_tolerance:
                self.pos = self.stop_target_pos
                self.speed = 0.0
                self.ato_target_speed = 0.0
                self.prev_accel = 0.0
                self.standstill_required = True
                self.standstill_anchor_pos = self.pos
                self._set_jog_state(JOG_STATE_COMPLETED, "within_tolerance")
            elif actual_distance_to_stop <= 0.0:
                self.speed = 0.0
                self.ato_target_speed = 0.0
                self.prev_accel = 0.0
                reason = "overshot_target" if self.pos > self.stop_target_pos else "invalid_remaining"
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, reason)
            elif actual_distance_to_stop > JOG_MAX_DIST_M or not self.commanded_stop:
                self.speed = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else self.speed
                self.ato_target_speed = 0.0
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, "invalid_remaining")
        # For commanded station stops, allow the train to finish braking with SBI while it is
        # still within the final stop tolerance. Only escalate to EB once it has genuinely
        # overshot the permitted stopping window.
        zero_speed_at_stop_limit = self.zero_speed_detected and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
        scheduled_stop_target = self.scheduled_stop_target_pos()
        temporary_authority_stop = (
            self.commanded_stop
            and scheduled_stop_target is not None
            and self.stop_target_pos < scheduled_stop_target - JOG_WINDOW_EPS_M
        )
        if temporary_authority_stop and self.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED):
            self._reset_jog_for_stop(self.current_stop_id or self.stop_identity())
        low_speed_temporary_hold = (
            temporary_authority_stop
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and distance_to_svl > 0.0
        )
        stop_position_emergency = self.distance_to_eoa <= 0.0 and (
            not zero_speed_at_stop_limit
            and not low_speed_temporary_hold
            and (not self.commanded_stop or stop_overshoot)
        )
        emergency = stop_position_emergency or self.emergency_stop
        downhill_buffer = kmh_to_ms(max(0.0, -self.gradient) * DOWNHILL_P_BUFFER_KMH_PER_GRAD)
        service_release_limit = max(0.0, self.curves["P"] - downhill_buffer - kmh_to_ms(SBI_RELEASE_HYST_KMH))
        service_speed_violation = self.vital_speed > self.hidden_curves["SBI"] + kmh_to_ms(SBI_VIOLATION_EPS_KMH)
        service_stop_distance = stopping_distance_with_buildup(
            self.vital_speed,
            max(a_service, ATP_MIN_DECEL_MS2),
            BRAKE_BUILDUP_S,
        )
        projected_service_stop_safe_before_svl = service_stop_distance <= max(0.0, distance_to_svl)
        station_jog_wait_guard = (
            self.commanded_stop
            and self.get_current_stop_key() is not None
            and self.jog_state == JOG_STATE_IDLE
            and self.jog_used_stop_key != self.get_current_stop_key()
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
            and self.vital_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and distance_to_svl > 0.0
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        low_speed_service_guard = (
            service_speed_violation
            and self.commanded_stop
            and 0.0 < actual_distance_to_stop <= RELEASE_HANDOVER_START_M
            and self.vital_speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH + 0.8)
            and distance_to_svl > 0.0
            and projected_service_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        precise_stop_service_intervention = (
            service_speed_violation
            and self.commanded_stop
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= PRECISE_STOP_SERVICE_BAND_M
            and self.vital_speed > STANDSTILL_SPEED_EPS
            and distance_to_svl > 0.0
            and service_stop_distance > max(STOP_ACCURACY_TOL_M, actual_distance_to_stop)
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if not self.commanded_stop:
            if (not self.release_active) and service_speed_violation:
                self.service_brake_latch = True
            elif self.service_brake_latch and self.vital_speed <= service_release_limit:
                self.service_brake_latch = False
        elif station_jog_wait_guard:
            if self.service_brake_latch:
                self.log_event("ATP_NUISANCE_GUARD_LOW_SPEED", "station_jog_wait_service_release")
            self.service_brake_latch = False
        elif self.release_active and self.vital_speed <= kmh_to_ms(RELEASE_SPEED_KMH):
            self.service_brake_latch = False
        elif (
            self.commanded_stop
            and self.service_brake_latch
            and self.vital_speed <= service_release_limit
            and not precise_stop_service_intervention
        ):
            self.service_brake_latch = False

        door_enable_service_guard = (
            service_speed_violation
            and self.door_authorized
            and self.zero_speed_detected
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and distance_to_svl > 0.0
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if door_enable_service_guard and self.service_brake_latch:
            self.service_brake_latch = False

        service_intervention = precise_stop_service_intervention or self.service_brake_latch or (
            service_speed_violation
            and not station_jog_wait_guard
            and not low_speed_service_guard
            and not door_enable_service_guard
        )

        stop_ebi_floor_active = (
            self.commanded_stop
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        ebi_limit_for_violation = self.hidden_curves["EBI"]
        ebd_limit_for_violation = self.curves["EBD"]
        if stop_ebi_floor_active:
            ebi_limit_for_violation = max(ebi_limit_for_violation, kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            ebd_limit_for_violation = max(ebd_limit_for_violation, ebi_limit_for_violation)
            self.hidden_curves["EBI"] = ebi_limit_for_violation
            self.raw_hidden_curves["EBI"] = ebi_limit_for_violation
        ebi_speed_violation = (
            self.vital_speed > ebi_limit_for_violation + 0.05
            or self.vital_speed > ebd_limit_for_violation + 0.05
        )
        if ebi_speed_violation:
            self._low_speed_ebi_counter += 1
        else:
            self._low_speed_ebi_counter = 0
            self._low_speed_guard_active = False
        emergency_stop_distance = stopping_distance_with_buildup(
            self.vital_speed,
            max(a_emergency, ATP_MIN_DECEL_MS2),
            BRAKE_BUILDUP_S,
        )
        projected_stop_safe_before_svl = emergency_stop_distance <= max(0.0, distance_to_svl)
        low_speed_station_guard = (
            ebi_speed_violation
            and self.commanded_stop
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and -STOP_ACCURACY_TOL_M <= actual_distance_to_stop <= RELEASE_ZONE_M
            and distance_to_svl > 0.0
            and projected_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
        )
        low_speed_departure_hold_guard = (
            ebi_speed_violation
            and self.departure_hold
            and not hard_departure_hold
            and self.station_state in {"READY_TO_DEPART", "DEPARTING"}
            and self.vital_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and distance_to_svl > 0.0
            and projected_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        precise_stop_service_guard = (
            ebi_speed_violation
            and service_intervention
            and self.commanded_stop
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= PRECISE_STOP_SERVICE_BAND_M
            and distance_to_svl > 0.0
            and projected_service_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if low_speed_station_guard and not self._low_speed_guard_active:
            self.log_event("ATP_NUISANCE_GUARD_LOW_SPEED", "safe_projected_stop_before_svl")
            self._low_speed_guard_active = True
        current_ebi_violation = (
            ebi_speed_violation
            and not low_speed_station_guard
            and not low_speed_departure_hold_guard
            and not precise_stop_service_guard
        )
        emergency_condition = emergency or current_ebi_violation
        atp_emergency_reason = ""
        if emergency_condition:
            if self.emergency_stop:
                atp_emergency_reason = "requested_emergency"
            elif stop_position_emergency:
                atp_emergency_reason = "stop_position_or_eoa"
            elif current_ebi_violation:
                atp_emergency_reason = "over_ebi_or_ebd"
            else:
                atp_emergency_reason = "protection_condition"
        if emergency_condition:
            self.emg_latch = True
            accel = -a_emergency
            self.atp_brake = "EMERGENCY"
        else:
            if service_intervention:
                accel = -a_service
                self.atp_brake = "SERVICE"
            elif self.commanded_stop:
                if stop_overshoot:
                    accel = -a_service
                    self.atp_brake = "NONE"
                elif (
                    0.0 < actual_distance_to_stop <= max(STOP_ACCURACY_TOL_M + 0.2, STOP_ACCURACY_TOL_M)
                    and control_speed <= kmh_to_ms(3.0)
                    and self.ato_target_speed <= kmh_to_ms(3.0)
                    and self.jog_state == JOG_STATE_IDLE
                ):
                    # Avoid low-speed hunting only in the sub-metre capture band.
                    # Farther out, ATO must keep creeping instead of stopping short.
                    accel = -min(a_service, COAST_BASE_DECEL)
                    self.atp_brake = "NONE"
                else:
                    error = self.ato_target_speed - control_speed
                    accel = self.compute_ato_pid_accel(error, control_speed, a_service, a_traction)
                    self.atp_brake = "NONE"
            else:
                if self.vital_speed > self.cutoff_threshold:
                    # Hidden traction power cut threshold between W and SBD.
                    self.traction_cutoff = True
                    accel = -coast_decel
                    self.atp_brake = "CUT_POWER"
                else:
                    # ATO follows its own piloting curve while ATP keeps supervising above it.
                    error = self.ato_target_speed - control_speed
                    accel = self.compute_ato_pid_accel(error, control_speed, a_service, a_traction)
                    self.atp_brake = "NONE"

        if self.emg_latch:
            accel = -a_emergency
            self.atp_brake = "EMERGENCY"

        if self.emergency_recovery_hold:
            accel = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else -a_service
            self.atp_brake = "HOLD"

        if stop_ebi_floor_active and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M:
            self.ato_target_speed = 0.0
            self.traction_cutoff = True
            accel = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else -min(a_service, COAST_BASE_DECEL)
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        if hard_departure_hold:
            self.ato_target_speed = 0.0
            if self.speed <= kmh_to_ms(1.0):
                accel = 0.0
                self.speed = 0.0
                self.filtered_speed = 0.0
                self.prev_accel = 0.0
                if not self.standstill_required:
                    self.standstill_anchor_pos = self.pos
                self.standstill_required = True
            else:
                accel = -a_service
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        if (
            self.commanded_stop
            and self.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and not temporary_authority_stop
            and self.speed <= STANDSTILL_SPEED_EPS
        ):
            accel = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        hold_position_lock = False
        if (
            (self.dwell_remaining_s > 0.0 or hard_departure_hold)
            and self.standstill_required
            and self.speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
        ):
            hold_position_lock = True
            accel = 0.0
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.pos = self.standstill_anchor_pos
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        accel = limit_jerk(self.prev_accel, accel, MAX_JERK_MS3, DT)
        self.prev_accel = accel
        if self.speed > STANDSTILL_SPEED_EPS or accel > 0.0:
            accel -= running_resistance_accel_ms2(self.speed)
        accel -= G * self.gradient

        prev_speed = self.speed
        next_speed = max(0.0, prev_speed + accel * DT)
        self.speed = next_speed
        moving_this_step = prev_speed > STANDSTILL_SPEED_EPS or next_speed > STANDSTILL_SPEED_EPS

        if prev_speed <= STANDSTILL_SPEED_EPS and next_speed <= STANDSTILL_SPEED_EPS:
            self.pos = self.pos
        else:
            self.pos = self.pos + 0.5 * (prev_speed + next_speed) * DT

        if hold_position_lock:
            self.pos = self.standstill_anchor_pos
            self.speed = 0.0
            self.prev_accel = 0.0
            moving_this_step = False

        # Force emergency if the train rolls back without authorization.
        if not self.commanded_stop and self.pos < self.prev_pos - ROLLBACK_PROTECT_M:
            self.rollback_protection = True
            self.emg_latch = True
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "ROLLBACK"
            self.atp_brake = "EMERGENCY"
            self.atp_action = "EBI"

        # During the single recovery jog, clamp the train exactly onto the stop target
        # as soon as it reaches or crosses it, so no second jog is needed.
        if (
            moving_this_step
            and self.jog_state == JOG_STATE_ACTIVE
            and self.prev_pos < self.stop_target_pos <= self.pos
            and prev_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and not self.standstill_required
        ):
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "clamped_at_target")
            self.jog_active = False
        # Snap to the stop target when the train is already in the final low-speed docking window.
        elif (
            moving_this_step
            and self.jog_state not in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and abs(self.stop_target_pos - self.pos) <= STOP_ACCURACY_TOL_M
            and control_speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH)
            and not self.standstill_required
        ):
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
                self.standstill_required = True
                self.standstill_anchor_pos = self.pos
                self._set_jog_state(JOG_STATE_COMPLETED, "snapped_within_tolerance")
                self.jog_active = False
        elif (
            self.commanded_stop
            and self.jog_state == JOG_STATE_IDLE
            and not self.standstill_required
            and not self.emg_latch
            and not self.trip_mode
            and self.speed <= STANDSTILL_SPEED_EPS
            and 0.0 <= self.stop_target_pos - self.pos <= MANUAL_JOG_WINDOW_M
        ):
            # The final anti-hunting coast can settle just short of the marker.
            # Close that sub-meter gap once stopped so dwell/door logic sees an exact stop.
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
        elif self.jog_state == JOG_STATE_ACTIVE and self.pos > self.stop_target_pos + STOP_ACCURACY_TOL_M:
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "overshot_target")
        # Standstill supervision should only hold the train once it has effectively reached the stop area
        # or after an emergency latch. A scheduled stop far ahead must not anchor standstill at the origin.
        stop_pending = self.commanded_stop and actual_distance_to_stop > STOP_ACCURACY_TOL_M
        stop_hold_zone = self.commanded_stop and actual_distance_to_stop <= max(
            STOP_ACCURACY_TOL_M,
            JOG_MAX_DIST_M,
            RELEASE_HANDOVER_START_M,
        )
        must_hold_standstill = (
            (stop_hold_zone or self.emg_latch)
            and self.speed <= STANDSTILL_SPEED_EPS
            and not stop_pending
        )
        if must_hold_standstill and not self.standstill_required:
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
        elif (
            stop_pending
            and actual_distance_to_stop > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
            and self.dwell_remaining_s <= 0.0
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
        ):
            self.standstill_required = False
        elif not (self.commanded_stop or self.emg_latch):
            self.standstill_required = False

        waiting_precise_jog = (
            self.commanded_stop
            and self.speed <= STANDSTILL_SPEED_EPS
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= JOG_MAX_DIST_M
            and not self.emergency_stop
            and not self.trip_mode
        )
        if waiting_precise_jog and self.standstill_required:
            # A train stopped short of the platform marker is allowed to wait for
            # precise jog. Do not escalate this waiting state into EBI.
            self.standstill_anchor_pos = self.pos

        if (
            self.standstill_required
            and not waiting_precise_jog
            and abs(self.pos - self.standstill_anchor_pos) > STANDSTILL_DRIFT_M
        ):
            self.emg_latch = True
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "STANDSTILL DRIFT"
            self.atp_brake = "EMERGENCY"
            self.atp_action = "EBI"

        self.zero_speed_detected = self.speed <= STANDSTILL_SPEED_EPS
        self.standstill_monitoring = self.standstill_required or self.zero_speed_detected
        self.rollback_monitoring = self.zero_speed_detected or self.commanded_stop or self.emg_latch
        brake_applied_for_hold = self.standstill_required or self.service_brake_latch or self.emg_latch or self.commanded_stop
        station_target_pos = self.scheduled_stop_target_pos()
        station_stop_ready = (
            station_target_pos is None
            or (
                self.station_lane is not None
                and self.stop_target_pos >= station_target_pos - JOG_WINDOW_EPS_M
                and abs(self.pos - station_target_pos) <= STOP_ACCURACY_TOL_M
            )
        )
        self.door_authorized = (
            self.zero_speed_detected
            and brake_applied_for_hold
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and station_stop_ready
            and abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M
        )
        self.door_open_allowed = self.door_authorized
        self.ato_hold_active = self.zero_speed_detected and (
            stop_hold_zone or self.standstill_required or self.ato_target_speed <= STANDSTILL_SPEED_EPS
        )
        if self.door_authorized:
            self.ato_door_mode = "ENABLE"
        elif self.ato_hold_active:
            self.ato_door_mode = "READY"
        else:
            self.ato_door_mode = "LOCKED"

        if self.jog_active:
            self.precise_stop_state = "JOG"
        elif self.jog_state == JOG_STATE_FAILED_LOCKED:
            self.precise_stop_state = "JOG_FAILED"
        elif abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M:
            self.precise_stop_state = "ALIGNED"
        elif self.commanded_stop and self.zero_speed_detected:
            self.precise_stop_state = "WAIT_JOG"
        else:
            self.precise_stop_state = "TRACKING"

        if self.trip_mode:
            self.atp_state = "ATP_TRIP"
        elif emergency or self.emergency_stop or self.emg_latch or current_ebi_violation:
            self.atp_state = "ATP_EMERGENCY"
        elif self.emergency_recovery_hold:
            self.atp_state = "ATP_RECOVERY_HOLD"
        elif self.departure_hold and self.zero_speed_detected:
            self.atp_state = "ATP_STANDSTILL"
        elif stop_ebi_floor_active:
            self.atp_state = "ATP_STANDSTILL"
        elif self.zero_speed_detected and self.standstill_required:
            self.atp_state = "ATP_STANDSTILL"
        elif self.zero_speed_detected:
            self.atp_state = "ATP_STANDBY"
        elif service_intervention and not stop_ebi_floor_active:
            self.atp_state = "ATP_SERVICE"
        elif self.vital_speed > self.cutoff_threshold + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_state = "ATP_CUTOFF"
        elif self.vital_speed > self.curves["W"] + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_state = "ATP_WARNING"
        else:
            self.atp_state = "ATP_OK"

        if self.trip_mode:
            self.atp_alert = self.trip_reason or "TRAIN TRIP"
        elif self.emergency_recovery_hold:
            self.atp_alert = "SAFE CONFIRMED"
        elif self.departure_hold:
            self.atp_alert = "DEPARTURE HOLD"
        elif self.standstill_required and self.emg_latch and abs(self.pos - self.standstill_anchor_pos) > STANDSTILL_DRIFT_M:
            self.atp_alert = "STANDSTILL DRIFT"
        elif stop_ebi_floor_active:
            self.atp_alert = "STANDSTILL MON"
        elif self.door_authorized:
            self.atp_alert = "STANDSTILL MON"
        elif self.zero_speed_detected and self.standstill_required:
            self.atp_alert = "STANDSTILL MON"
        elif self.zero_speed_detected:
            self.atp_alert = "ZERO SPEED"
        elif current_ebi_violation and self.vital_speed > self.hidden_curves["EBI"] + 0.05:
            self.atp_alert = "OVER EBI"
        elif current_ebi_violation and self.vital_speed > self.curves["EBD"] + 0.05:
            self.atp_alert = "OVER EBD"
        elif service_intervention and not stop_ebi_floor_active:
            self.atp_alert = "OVER SBI"
        elif self.vital_speed > self.cutoff_threshold + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_alert = "OFF ENERGY"
        elif self.vital_speed > self.curves["W"] + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_alert = "OVER W"
        else:
            self.atp_alert = "OK"

        if self.atp_state in ("ATP_EMERGENCY", "ATP_TRIP"):
            self.atp_action = "EBI"
        elif self.atp_state == "ATP_SERVICE":
            self.atp_action = "SBI"
        elif self.atp_state == "ATP_CUTOFF":
            self.atp_action = "OFF"
        elif self.atp_state == "ATP_WARNING":
            self.atp_action = "WARN"
        else:
            self.atp_action = ""

        service_active_now = self.atp_action == "SBI"
        emergency_active_now = self.atp_action == "EBI"
        if service_active_now and not self._last_atp_service_active:
            self.log_event("ATP_SERVICE_INTERVENTION", "over_sbi_or_service_latch")
        if emergency_active_now and not self._last_atp_emergency_active:
            self.log_event("ATP_EMERGENCY_INTERVENTION", atp_emergency_reason or self.atp_alert)
        if self.door_authorized and not self._last_door_authorized and self.jog_state == JOG_STATE_COMPLETED:
            self.log_event("DOOR_AUTHORIZED_AFTER_JOG")
        self._last_atp_service_active = service_active_now
        self._last_atp_emergency_active = emergency_active_now
        self._last_door_authorized = self.door_authorized

        if self.emergency_recovery_hold:
            self.ato_state = "ATO_HOLD"
        elif self.drive_mode == "CMD25" and not self.zero_speed_detected:
            self.ato_state = "RM/CMD25"
        elif self.drive_mode == "LMD" and not self.zero_speed_detected:
            self.ato_state = "LMD"
        elif self.jog_active:
            self.ato_state = "ATO_JOG"
        elif self.ato_hold_active:
            self.ato_state = "ATO_HOLD"
        elif self.zero_speed_detected:
            self.ato_state = "ATO_STANDBY"
        elif self.release_active and control_speed <= kmh_to_ms(CREEP_MAX_SPEED_KMH):
            self.ato_state = "ATO_CREEP"
        elif self.commanded_stop:
            self.ato_state = "ATO_STOP"
        elif self.speed < self.ato_target_speed - 0.1:
            self.ato_state = "ATO_TRACTION"
        elif self.speed > self.ato_target_speed + 0.1:
            self.ato_state = "ATO_BRAKE"
        else:
            self.ato_state = "ATO_COAST"
        if (
            self.commanded_stop
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        ):
            self.hidden_curves["EBI"] = max(self.hidden_curves["EBI"], kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            self.curves["EBD"] = max(self.curves["EBD"], self.hidden_curves["EBI"])
        if self.ato_fault_active:
            self.drive_mode = "LMD"
            self.ato_state = "ATO_FAULT"
        if self.atp_fault_active:
            self.atp_state = "ATP_TRIP"
            self.atp_alert = "ATP FAULT"
            self.atp_action = "EBI"
            self.atp_brake = "EMERGENCY"
            self.emg_latch = True
        distance_delta_m = max(0.0, self.pos - self.prev_pos)
        self.analytics_distance_m += distance_delta_m
        if self.atp_brake in {"SERVICE", "EMERGENCY", "HOLD"} or self.ato_brake_mode not in {"none", "coast"}:
            self.runtime_brake_force_n = max(0.0, abs(self.prev_accel) * self.mass)
            self.runtime_traction_force_n = 0.0
        elif self.speed > 0.0 and not self.traction_cutoff:
            self.runtime_traction_force_n = max(0.0, self.prev_accel * self.mass)
            self.runtime_brake_force_n = 0.0
        else:
            self.runtime_traction_force_n = 0.0
            self.runtime_brake_force_n = 0.0
        self.analytics_traction_work_j += self.runtime_traction_force_n * distance_delta_m
        self.analytics_brake_work_j += self.runtime_brake_force_n * distance_delta_m
        self.prev_pos = self.pos
        self.prev_commanded_stop = self.commanded_stop


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


class Simulation:
    def __init__(self, scenario: Dict[str, object]):
        self.load_scenario(scenario)
        self.running = False
        self.last_step = time.time()
        self.tsr_zones = []

    def load_scenario(self, scenario: Dict[str, object]):
        self.scenario = scenario
        self.track_profile = list(scenario["track_profile"])
        self.track_end_m = float(scenario["track_end_m"])
        self.track_min_m = float(scenario["track_min_m"])
        self.track_max_m = float(scenario["track_max_m"])
        self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self.track_labels = list(scenario["track_labels"])
        raw_headway_cfg = scenario.get("headway")
        headway_cfg = raw_headway_cfg if isinstance(raw_headway_cfg, dict) else {}
        requested_block_mode = str(scenario.get("block_mode", "")).lower()
        if requested_block_mode in {"fixed", "fixed_block"}:
            self.block_mode = "fixed_block"
        elif requested_block_mode in {"moving", "moving_block"}:
            self.block_mode = "moving_block"
        else:
            explicit_off_mode = (
                bool(scenario.get("headway_config_present", isinstance(raw_headway_cfg, dict) and bool(raw_headway_cfg)))
                and str(headway_cfg.get("mode", "off")).lower() == "off"
            )
            self.block_mode = "fixed_block" if explicit_off_mode else "moving_block"
        self.headway_manager = HeadwayManager.from_scenario(scenario)
        self.scheduled_stops = [dict(stop) for stop in scenario.get("scheduled_stops", [])]
        self.timetable_services = self._build_timetable_services(scenario)
        headway_runtime = scenario.get("headway", {}) if isinstance(scenario.get("headway", {}), dict) else {}
        self.timetable_wall_clock = bool(headway_runtime.get("timetable_wall_clock", False))
        self.timetable_loaded_clock_s = (
            float(headway_runtime.get("timetable_loaded_clock_s"))
            if headway_runtime.get("timetable_loaded_clock_s") is not None
            else None
        )
        self.timetable_clock_scale = max(1.0, float(headway_runtime.get("timetable_clock_scale", 1.0) or 1.0))
        self._timetable_clock_anchor_real_s = time.monotonic()
        self._timetable_clock_anchor_operational_s = 0.0
        self.fixed_blocks = self._build_fixed_blocks(scenario)
        self.station_route_states: List[Dict[str, Any]] = []
        self.parallel_release_locks: Dict[Tuple[str, int], float] = {}
        self.source_trains = []
        for idx, source in enumerate(scenario.get("source_trains", [])):
            capacity = max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS)))
            total_trains = max(0, int(source.get("total_trains", capacity)))
            self.source_trains.append(
                {
                    "name": str(source.get("name", f"SRC_{idx + 1}")),
                    "start_m": SOURCE_TRAIN_START_M,
                    "length_m": SOURCE_TRAIN_LENGTH_M,
                    "capacity": capacity,
                    "total_trains": total_trains,
                    "generated": min(total_trains, max(0, int(source.get("generated", 0)))),
                }
            )
            self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self.line_conditions = []
        for condition in scenario.get("line_conditions", []):
            self.line_conditions.append(
                {
                    "start": float(condition.get("start", condition.get("start_m", self.track_min_m))),
                    "end": float(condition.get("end", condition.get("end_m", self.track_max_m))),
                    "condition": str(condition.get("condition", "dry")),
                }
            )
        palette = list(scenario["color_palette"])
        self.color_palette = palette
        self.generated_train_counter = 0
        self.train_generation_changed = False
        self.station_last_arrival_s: Dict[int, float] = {}
        self.station_arrival_headway_actual_s: Dict[int, List[float]] = {}
        self.station_last_departure_s: Dict[int, float] = {}
        self.station_next_departure_release_s: Dict[int, float] = {}
        self.station_headway_actual_s: Dict[int, List[float]] = {}
        self.station_headway_deviation_s: Dict[int, List[float]] = {}
        self.trains = []
        for idx, train_cfg in enumerate(scenario["trains"]):
            cfg = dict(train_cfg)
            cfg["color"] = train_color(cfg, idx, palette)
            cfg["track_profile"] = self.track_profile
            cfg["scheduled_stops"] = self.scheduled_stops
            train = Train(cfg)
            train.source_lane = cfg.get("source_lane")
            self.trains.append(train)
        self._stage_initial_source_trains()
        self._sync_station_route_states()
        self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
        self.tsr_zones = []
        self.sim_time_s = 0.0
        self._reset_timetable_clock_anchor()
        self.analytics = {
            "min_headway_s": None,
            "target_headway_s": self.headway_manager.nominal_target_headway_s(),
            "actual_headways_s": [],
            "actual_headway_pairs": [],
            "current_open_headway_s": None,
            "min_actual_headway_s": None,
            "avg_actual_headway_s": None,
            "max_actual_headway_s": None,
            "dispatch_delays_s": [],
            "avg_dispatch_delay_s": None,
            "trains_per_hour": 0.0,
            "station_passenger_metrics": [],
            "station_arrivals": {},
            "traction_work_kwh": 0.0,
            "brake_work_kwh": 0.0,
            "collision_count": 0,
            "active_collision_count": 0,
            "collision_events": [],
            "ebi_count": 0,
            "sbi_count": 0,
            "journey_times": {},
            "last_actions": {},
        }
        self._dispatch_safe_packets(with_delay=False)

    def _build_fixed_blocks(self, scenario: Dict[str, object]) -> List[Dict[str, float]]:
        cfg = scenario.get("capacity_baseline", {}) if isinstance(scenario.get("capacity_baseline", {}), dict) else {}
        blocks_per_section = max(1, int(cfg.get("blocks_per_section", cfg.get("fixed_blocks_per_section", 4))))
        track_start_m = float(self.track_profile[0][0]) if self.track_profile else 0.0
        sections: List[Tuple[float, float]] = []
        section_start = track_start_m
        for stop in self.scheduled_stops:
            stop_pos = float(stop.get("pos_m", track_start_m))
            stop_len = max(0.0, float(stop.get("length_m", 160.0)))
            station_start = max(track_start_m, stop_pos - stop_len / 2.0)
            station_end = min(self.track_end_m, stop_pos + stop_len / 2.0)
            if station_start > section_start + STOP_ACCURACY_TOL_M:
                sections.append((section_start, station_start))
            section_start = max(section_start, station_end)
        if self.track_end_m > section_start + STOP_ACCURACY_TOL_M:
            sections.append((section_start, self.track_end_m))
        if not sections:
            sections.append((track_start_m, self.track_end_m))
        blocks: List[Dict[str, float]] = []
        block_idx = 1
        for section_idx, (start, end) in enumerate(sections, start=1):
            length = max(0.0, end - start)
            if length <= 0.0:
                continue
            block_len = length / blocks_per_section
            for local_idx in range(blocks_per_section):
                block_start = start + block_len * local_idx
                block_end = end if local_idx == blocks_per_section - 1 else start + block_len * (local_idx + 1)
                blocks.append(
                    {
                        "id": f"FB{section_idx}.{local_idx + 1}",
                        "start_m": block_start,
                        "end_m": block_end,
                        "index": block_idx,
                    }
                )
                block_idx += 1
        return blocks

    def _sync_station_route_states(self):
        while len(self.station_route_states) < len(self.scheduled_stops):
            self.station_route_states.append(
                {
                    "route_lane": None,
                    "assigned_train_id": None,
                    "route_state": "FREE",
                    "lines": [],
                    "invariant_block": False,
                    "lock_remaining_s": 0.0,
                    "locking_train_id": None,
                    "switch_started": False,
                }
            )
        if len(self.station_route_states) > len(self.scheduled_stops):
            self.station_route_states = self.station_route_states[:len(self.scheduled_stops)]
        for station_idx, stop in enumerate(self.scheduled_stops):
            self.detect_station_lines(station_idx)

    def _make_source_train_config(self, source: Dict[str, Any], train_id: str, start_pos: float, lane: int = 0) -> Dict[str, Any]:
        sequence = int(source.get("_pending_sequence", 0) or 0)
        service = self._timetable_service_for_sequence(sequence)
        if service:
            train_id = str(service.get("train_id", train_id))
        cfg = {
            "id": train_id,
            "start_pos": start_pos,
            "length_m": float(self.scenario["train_defaults"]["length_m"]),
            "mass_kg": float(self.scenario["train_defaults"]["mass_kg"]),
            "drive_mode": str(self.scenario["train_defaults"].get("drive_mode", "ATO")),
            "requested_drive_mode": str(self.scenario["train_defaults"].get("drive_mode", "ATO")),
            "max_manual_speed_kmh": float(self.scenario["train_defaults"].get("max_manual_speed_kmh", 45.0)),
            "dcs_mute_windows": [],
            "color": self.color_palette[len(self.trains) % len(self.color_palette)],
            "track_profile": self.track_profile,
            "scheduled_stops": self.scheduled_stops,
            "source_name": str(source.get("name", "SRC")),
            "source_lane": lane,
        }
        if service:
            cfg["schedule_service_id"] = service.get("train_id")
            cfg["schedule_profile"] = service.get("profile", "")
            cfg["schedule_records"] = service.get("records", [])
            cfg["schedule_planned_dispatch_s"] = service.get("planned_dispatch_time_s")
        return cfg

    def _build_timetable_services(self, scenario: Dict[str, Any]) -> List[Dict[str, Any]]:
        headway = scenario.get("headway", {}) if isinstance(scenario.get("headway", {}), dict) else {}
        records = [dict(record) for record in headway.get("timetable_records", []) or []]
        services: Dict[str, Dict[str, Any]] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            if not train_id:
                continue
            service = services.setdefault(train_id, {"train_id": train_id, "records": []})
            service["records"].append(record)
            if service.get("planned_dispatch_time_s") is None and record.get("departure_time_s") is not None:
                service["planned_dispatch_time_s"] = float(record.get("departure_time_s"))
            if not service.get("profile") and record.get("profile") not in {None, "", "--"}:
                service["profile"] = str(record.get("profile"))
        result = list(services.values())
        result.sort(
            key=lambda service: min(
                (
                    float(record.get("departure_time_s"))
                    for record in service.get("records", [])
                    if record.get("departure_time_s") is not None
                ),
                default=float("inf"),
            )
        )
        return result

    def _timetable_service_for_sequence(self, sequence: int) -> Dict[str, Any] | None:
        if self.headway_manager.mode != "timetable" or sequence <= 0:
            return None
        idx = sequence - 1
        if 0 <= idx < len(self.timetable_services):
            return self.timetable_services[idx]
        return None

    def _vietnam_clock_seconds(self) -> float:
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        return float(now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 1_000_000.0)

    def _clock_delay_from_reference_s(self, clock_s: float, reference_clock_s: float) -> float:
        day_s = 24.0 * 3600.0
        return (float(clock_s) - float(reference_clock_s)) % day_s

    def _reset_timetable_clock_anchor(self) -> None:
        self._timetable_clock_anchor_real_s = time.monotonic()
        if self.timetable_wall_clock and self.timetable_loaded_clock_s is not None:
            self._timetable_clock_anchor_operational_s = self._clock_delay_from_reference_s(
                self._vietnam_clock_seconds(),
                self.timetable_loaded_clock_s,
            )
        else:
            self._timetable_clock_anchor_operational_s = self.sim_time_s

    def set_timetable_clock_scale(self, scale: float) -> None:
        current_operational_s = self._timetable_operational_time_s()
        self.timetable_clock_scale = max(1.0, float(scale))
        self._timetable_clock_anchor_operational_s = current_operational_s
        self._timetable_clock_anchor_real_s = time.monotonic()

    def _timetable_operational_time_s(self) -> float:
        if (
            self.headway_manager.mode == "timetable"
            and self.timetable_wall_clock
            and self.timetable_loaded_clock_s is not None
        ):
            elapsed_real_s = max(0.0, time.monotonic() - getattr(self, "_timetable_clock_anchor_real_s", time.monotonic()))
            return float(getattr(self, "_timetable_clock_anchor_operational_s", 0.0)) + elapsed_real_s * float(
                getattr(self, "timetable_clock_scale", 1.0)
            )
        return self.sim_time_s

    def timetable_display_clock_s(self) -> float | None:
        if not (self.headway_manager.mode == "timetable" and self.timetable_wall_clock and self.timetable_loaded_clock_s is not None):
            return None
        return (self.timetable_loaded_clock_s + self._timetable_operational_time_s()) % (24.0 * 3600.0)

    def _source_staging_head_pos(self) -> float:
        train_length = float(self.scenario["train_defaults"]["length_m"])
        min_head = SOURCE_TRAIN_START_M + train_length + SOURCE_TRAIN_STAGING_CLEARANCE_M
        max_head = SOURCE_TRAIN_EXIT_M - SOURCE_TRAIN_STAGING_CLEARANCE_M
        return min(max_head, max(min_head, SOURCE_TRAIN_START_M + train_length))

    def _next_source_train_id(self, source: Dict[str, Any], sequence: int) -> str:
        train_id = f"{source.get('name', 'SRC')}_{sequence}"
        existing_ids = {train.id for train in self.trains}
        while train_id in existing_ids:
            self.generated_train_counter += 1
            train_id = f"SRC_{self.generated_train_counter}"
        return train_id

    def _source_train_matches(self, train: Train, source_name: str) -> bool:
        if getattr(train, "source_name", None) == source_name:
            return True
        return train.id.startswith(f"{source_name}_")

    def _source_train_sequence(self, train: Train, source_name: str) -> int:
        prefix = f"{source_name}_"
        if not train.id.startswith(prefix):
            return 0
        try:
            return int(train.id[len(prefix):])
        except ValueError:
            return 0

    def _source_owned_trains(self, source_name: str) -> List[Train]:
        owned = [train for train in self.trains if self._source_train_matches(train, source_name)]
        return sorted(owned, key=lambda item: self._source_train_sequence(item, source_name))

    def _rebuild_after_train_set_change(self):
        self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
        self._dispatch_safe_packets(with_delay=False)
        self.train_generation_changed = True

    def _sync_source_train_count(self, index: int, previous_name: str | None = None):
        if index < 0 or index >= len(self.source_trains):
            return
        source = self.source_trains[index]
        source_name = str(source.get("name", "SRC"))
        match_name = previous_name or source_name
        total = max(0, int(source.get("total_trains", source.get("capacity", 0))))
        owned = self._source_owned_trains(match_name)
        changed = False

        for train in owned:
            train.source_name = source_name

        if len(owned) > total:
            remove_set = set(owned[total:])
            self.trains = [train for train in self.trains if train not in remove_set]
            changed = True
        elif len(owned) < total:
            before = len(self.trains)
            source["generated"] = len(owned)
            self._stage_initial_source_trains()
            changed = len(self.trains) != before

        source["generated"] = min(total, len(self._source_owned_trains(source_name)))
        if changed:
            self._rebuild_after_train_set_change()

    def _stage_initial_source_trains(self):
        for source in self.source_trains:
            total = int(source.get("total_trains", 0))
            generated = int(source.get("generated", 0))
            capacity = max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS)))
            occupied_lanes = {
                int(train.source_lane)
                for train in self.trains
                if train.source_lane is not None and self._source_zone_contains(train)
            }
            free_lanes = [lane for lane in range(capacity) if lane not in occupied_lanes]
            to_stage = min(len(free_lanes), max(0, total - generated))
            for idx, lane in enumerate(free_lanes[:to_stage]):
                sequence = generated + idx + 1
                start_pos = self._source_staging_head_pos()
                train_id = self._next_source_train_id(source, sequence)
                source["_pending_sequence"] = sequence
                train = Train(self._make_source_train_config(source, train_id, start_pos, lane))
                source.pop("_pending_sequence", None)
                train.source_lane = lane
                self.trains.append(train)
            source["generated"] = generated + to_stage

    def add_station(self, name: str, pos_m: float, length_m: float, capacity: int = 3, dwell_s: float = 30.0):
        self.scheduled_stops.append(
            {
                "name": name,
                "pos_m": pos_m,
                "dwell_s": dwell_s,
                "length_m": max(1.0, length_m),
                "capacity": max(1, capacity),
            }
        )
        self.scheduled_stops.sort(key=lambda item: item["pos_m"])
        self._sync_station_route_states()
        for train in self.trains:
            train.scheduled_stops = self.scheduled_stops

    def update_station(self, index: int, name: str, pos_m: float, length_m: float, capacity: int, dwell_s: float):
        if index < 0 or index >= len(self.scheduled_stops):
            return
        self.scheduled_stops[index].update(
            {
                "name": name,
                "pos_m": pos_m,
                "dwell_s": dwell_s,
                "length_m": max(1.0, length_m),
                "capacity": max(1, capacity),
            }
        )
        self.scheduled_stops.sort(key=lambda item: item["pos_m"])
        self._sync_station_route_states()
        for train in self.trains:
            train.scheduled_stops = self.scheduled_stops

    def add_psr_segment(self, start_m: float, end_m: float, psr_kmh: float):
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        updated: List[Tuple[float, float, float, float]] = []
        covered = False
        for seg_start, seg_end, gradient, old_psr in self.track_profile:
            if seg_end <= start_m or seg_start >= end_m:
                updated.append((seg_start, seg_end, gradient, old_psr))
                continue
            covered = True
            if seg_start < start_m:
                updated.append((seg_start, start_m, gradient, old_psr))
            updated.append((max(seg_start, start_m), min(seg_end, end_m), gradient, psr_kmh))
            if seg_end > end_m:
                updated.append((end_m, seg_end, gradient, old_psr))
        if not covered:
            gradient, _old_psr = get_track_info(self.track_profile, start_m)
            updated.append((start_m, end_m, gradient, psr_kmh))
        self.track_profile = sorted(
            [segment for segment in updated if segment[1] > segment[0]],
            key=lambda item: item[0],
        )
        self.track_end_m = max(self.track_end_m, end_m)
        self.track_max_m = max(self.track_max_m, end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def update_track_segment(self, index: int, start_m: float, end_m: float, gradient: float, psr_kmh: float):
        if index < 0 or index >= len(self.track_profile):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.track_profile[index] = (start_m, end_m, gradient, psr_kmh)
        self.track_profile.sort(key=lambda item: item[0])
        self.track_end_m = max(end for _start, end, _gradient, _psr in self.track_profile)
        self.track_max_m = max(self.track_max_m, self.track_end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def add_gradient_segment(self, start_m: float, end_m: float, gradient: float):
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        updated: List[Tuple[float, float, float, float]] = []
        covered = False
        for seg_start, seg_end, old_gradient, old_psr in self.track_profile:
            if seg_end <= start_m or seg_start >= end_m:
                updated.append((seg_start, seg_end, old_gradient, old_psr))
                continue
            covered = True
            if seg_start < start_m:
                updated.append((seg_start, start_m, old_gradient, old_psr))
            updated.append((max(seg_start, start_m), min(seg_end, end_m), gradient, old_psr))
            if seg_end > end_m:
                updated.append((end_m, seg_end, old_gradient, old_psr))
        if not covered:
            _old_gradient, psr = get_track_info(self.track_profile, start_m)
            updated.append((start_m, end_m, gradient, psr))
        self.track_profile = sorted(
            [segment for segment in updated if segment[1] > segment[0]],
            key=lambda item: item[0],
        )
        self.track_end_m = max(self.track_end_m, end_m)
        self.track_max_m = max(self.track_max_m, end_m)
        for train in self.trains:
            train.track_profile = self.track_profile

    def add_line_element(
        self,
        length_m: float,
        gradient_start_m: float,
        gradient_end_m: float,
        gradient: float,
        condition_start_m: float,
        condition_end_m: float,
        condition: str,
        psr_kmh: float,
    ):
        length_m = max(1.0, length_m)
        self.track_end_m = max(self.track_end_m, length_m)
        self.track_max_m = max(self.track_max_m, length_m)
        if gradient_end_m < gradient_start_m:
            gradient_start_m, gradient_end_m = gradient_end_m, gradient_start_m
        if gradient_end_m > gradient_start_m:
            self.track_profile.append((gradient_start_m, gradient_end_m, gradient, psr_kmh))
            self.track_profile.sort(key=lambda item: item[0])
        if condition_end_m < condition_start_m:
            condition_start_m, condition_end_m = condition_end_m, condition_start_m
        if condition_end_m > condition_start_m:
            self.line_conditions.append(
                {
                    "start": condition_start_m,
                    "end": condition_end_m,
                    "condition": condition,
                }
            )
        for train in self.trains:
            train.track_profile = self.track_profile

    def update_line_condition(self, index: int, start_m: float, end_m: float, condition: str):
        if index < 0 or index >= len(self.line_conditions):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.line_conditions[index].update({"start": start_m, "end": end_m, "condition": condition})

    def update_tsr(self, index: int, start_m: float, end_m: float, speed_kmh: float):
        if index < 0 or index >= len(self.tsr_zones):
            return
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            return
        self.tsr_zones[index].update({"start": start_m, "end": end_m, "speed": speed_kmh})

    def update_source_train(self, index: int, name: str, capacity: int, total_trains: int):
        if index < 0 or index >= len(self.source_trains):
            return
        source = self.source_trains[index]
        previous_name = str(source.get("name", name))
        capacity = max(1, capacity)
        total_trains = max(0, total_trains)
        source.update(
            {
                "name": name,
                "start_m": SOURCE_TRAIN_START_M,
                "length_m": SOURCE_TRAIN_LENGTH_M,
                "capacity": capacity,
                "total_trains": total_trains,
            }
        )
        self._sync_source_train_count(index, previous_name)

    def add_source_train(self, name: str, start_m: float, length_m: float, capacity: int, total_trains: int):
        capacity = max(1, capacity)
        total_trains = max(0, total_trains)
        source = {
            "name": name,
            "start_m": SOURCE_TRAIN_START_M,
            "length_m": SOURCE_TRAIN_LENGTH_M,
            "capacity": capacity,
            "total_trains": total_trains,
            "generated": 0,
        }
        self.source_trains.append(source)
        self.track_min_m = min(self.track_min_m, SOURCE_TRAIN_START_M)
        self._sync_source_train_count(len(self.source_trains) - 1)

    def _source_exit_clear(self, source: Dict[str, Any], exit_pos: float, train_length: float) -> bool:
        source_start = SOURCE_TRAIN_START_M
        source_end = SOURCE_TRAIN_EXIT_M
        protected_start = source_start - train_length
        protected_end = source_end + SOURCE_TRAIN_SPACING_M
        return not any((train.pos - train.length) < protected_end and train.pos > protected_start for train in self.trains)

    def _spawn_source_trains(self):
        changed = False
        for source in self.source_trains:
            total = int(source.get("total_trains", 0))
            generated = int(source.get("generated", 0))
            if generated >= total:
                continue
            train_length = float(self.scenario["train_defaults"]["length_m"])
            start_pos = self._source_staging_head_pos()
            if not self._source_exit_clear(source, start_pos, train_length):
                continue
            self.generated_train_counter += 1
            train_id = self._next_source_train_id(source, generated + 1)
            source["_pending_sequence"] = generated + 1
            train = Train(self._make_source_train_config(source, train_id, start_pos, 0))
            source.pop("_pending_sequence", None)
            dispatched_front_gap_m = max(
                (
                    existing.pos - SOURCE_TRAIN_EXIT_M
                    for existing in self.trains
                    if existing.headway_dispatch_released and existing.pos > SOURCE_TRAIN_EXIT_M
                ),
                default=None,
            )
            tsr_active = bool(self.tsr_zones)
            decision = self.headway_manager.decide(
                train,
                self._timetable_operational_time_s(),
                dispatched_front_pos_m=dispatched_front_gap_m,
                tsr_active=tsr_active,
            )
            if decision.decision == "HOLD":
                source["last_hold_reason"] = decision.reason
                source["planned_dispatch_time_s"] = decision.planned_dispatch_time_s
                continue
            train.headway_target_s = decision.target_headway_s
            train.headway_planned_dispatch_s = decision.planned_dispatch_time_s
            train.headway_dispatch_delay_s = decision.dispatch_delay_s
            train.headway_dispatch_released = True
            train.headway_hold_reason = decision.reason
            train.source_lane = 0
            self.trains.append(train)
            source["generated"] = generated + 1
            source["last_hold_reason"] = ""
            changed = True
        if changed:
            self.zc = ZoneController(self.trains, self.track_end_m, self.block_mode, self.fixed_blocks)
            self._dispatch_safe_packets(with_delay=False)
        return changed

    def _scheduled_stop_eoa(self, stop_pos_m: float) -> float:
        return stop_pos_m - (STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)

    def _source_zone_contains(self, train: Train) -> bool:
        if not self.source_trains:
            return False
        if train.source_lane is not None:
            return train.safe_rear_end_pos() <= SOURCE_TRAIN_EXIT_M
        return train.pos >= SOURCE_TRAIN_START_M and train.pos <= SOURCE_TRAIN_EXIT_M + STOP_ACCURACY_TOL_M

    def _update_source_headway_releases(self):
        source_trains = [
            train for train in self.trains
            if train.source_lane is not None and self._source_zone_contains(train)
        ]
        source_trains.sort(key=lambda item: (item.source_lane if item.source_lane is not None else 0, item.id))
        for train in source_trains:
            if train.headway_dispatch_released:
                train.departure_hold = False
                continue
            released_front_gaps = [
                existing.pos - SOURCE_TRAIN_EXIT_M
                for existing in self.trains
                if (
                    existing is not train
                    and existing.headway_dispatch_released
                    and existing.pos > SOURCE_TRAIN_EXIT_M
                )
            ]
            if self.headway_manager.mode == "adaptive" and self.headway_manager.released_train_ids and not released_front_gaps:
                dispatched_front_gap_m = 0.0
            else:
                dispatched_front_gap_m = max(released_front_gaps, default=None)
            decision = self.headway_manager.decide(
                train,
                self._timetable_operational_time_s(),
                dispatched_front_pos_m=dispatched_front_gap_m,
                tsr_active=bool(self.tsr_zones),
            )
            train.headway_target_s = decision.target_headway_s
            train.headway_planned_dispatch_s = decision.planned_dispatch_time_s
            train.headway_dispatch_delay_s = decision.dispatch_delay_s
            train.headway_hold_reason = decision.reason
            if decision.decision == "HOLD":
                train.departure_hold = True
                continue
            train.headway_dispatch_released = True
            train.departure_hold = False

    def _station_for_train(self, train: Train) -> Tuple[int, Dict[str, Any]] | None:
        for idx, stop in enumerate(self.scheduled_stops):
            pos_m = float(stop["pos_m"])
            length_m = float(stop.get("length_m", 160.0))
            station_start = pos_m - length_m / 2.0
            station_end = pos_m + length_m / 2.0
            if station_start <= train.pos and train.safe_rear_end_pos() <= station_end:
                return idx, stop
        return None

    def _station_overlapped_by_train(self, train: Train) -> Tuple[int, Dict[str, Any]] | None:
        for idx, stop in enumerate(self.scheduled_stops):
            if self._train_overlaps_station(train, stop):
                return idx, stop
        return None

    def _station_bounds(self, stop: Dict[str, Any]) -> Tuple[float, float]:
        pos_m = float(stop["pos_m"])
        length_m = float(stop.get("length_m", 160.0))
        return pos_m - length_m / 2.0, pos_m + length_m / 2.0

    def _same_station_stop(self, left: Dict[str, Any] | None, right: Dict[str, Any] | None) -> bool:
        if left is None or right is None:
            return False
        return (
            str(left.get("name", "")) == str(right.get("name", ""))
            and abs(float(left.get("pos_m", 0.0)) - float(right.get("pos_m", 0.0))) <= 1e-9
        )

    def _station_index_for_stop(self, stop: Dict[str, Any] | None) -> int | None:
        for idx, candidate in enumerate(self.scheduled_stops):
            if self._same_station_stop(stop, candidate):
                return idx
        return None

    def _train_overlaps_station(self, train: Train, stop: Dict[str, Any]) -> bool:
        station_start, station_end = self._station_bounds(stop)
        return train.safe_rear_end_pos() < station_end and train.pos > station_start

    def _train_head_in_station(self, train: Train, stop: Dict[str, Any]) -> bool:
        station_start, station_end = self._station_bounds(stop)
        return station_start <= train.pos <= station_end

    def _station_line_id(self, station_idx: int, lane: int) -> str:
        stop = self.scheduled_stops[station_idx]
        return f"{stop.get('name', f'STATION_{station_idx}')}:{int(lane)}"

    def detect_station_lines(self, station_idx: int) -> List[Dict[str, Any]]:
        self._sync_station_route_states() if station_idx >= len(self.station_route_states) else None
        stop = self.scheduled_stops[station_idx]
        state = self.station_route_states[station_idx]
        capacity = max(1, int(stop.get("capacity", 3)))
        station_start, station_end = self._station_bounds(stop)
        stop_point_m = float(stop["pos_m"])
        existing = {int(line.get("lane", idx)): line for idx, line in enumerate(state.get("lines", []))}
        lines: List[Dict[str, Any]] = []
        for lane in range(capacity):
            line = dict(existing.get(lane, {}))
            line.update(
                {
                    "lane": lane,
                    "line_id": self._station_line_id(station_idx, lane),
                    "station_id": str(stop.get("name", station_idx)),
                    "start_m": station_start,
                    "end_m": station_end,
                    "stop_point_m": stop_point_m,
                    "type": "MAIN" if lane == 0 else "PLATFORM",
                    "capacity": int(line.get("capacity", 1)),
                    "reserved_by_train_id": line.get("reserved_by_train_id"),
                    "occupied_by_train_id": line.get("occupied_by_train_id"),
                    "route_state": line.get("route_state", "FREE"),
                }
            )
            lines.append(line)
        state["lines"] = lines
        return lines

    def _station_line_for_lane(self, station_idx: int, lane: int | None) -> Dict[str, Any] | None:
        if lane is None or not (0 <= station_idx < len(self.station_route_states)):
            return None
        for line in self.detect_station_lines(station_idx):
            if int(line["lane"]) == int(lane):
                return line
        return None

    def _train_overlaps_station_line(self, train: Train, line: Dict[str, Any]) -> bool:
        head_pos = train.pos
        tail_pos = train.pos - train.length
        return tail_pos < float(line["end_m"]) and head_pos > float(line["start_m"])

    def _station_line_tail_clear(self, train: Train, line: Dict[str, Any]) -> bool:
        return train.safe_rear_end_pos() > float(line["end_m"]) + PARALLEL_RELEASE_MARGIN_M

    def _train_stably_stopped_in_station_line(self, train: Train, station_idx: int) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        return (
            train.zero_speed_detected
            and train.speed <= STANDSTILL_SPEED_EPS
            and abs(train.pos - float(line["stop_point_m"])) <= STOP_ACCURACY_TOL_M
            and line.get("occupied_by_train_id") == train.id
        )

    def _station_line_reservation_still_valid(
        self,
        station_idx: int,
        stop: Dict[str, Any],
        line: Dict[str, Any],
        train: Train | None,
    ) -> bool:
        if train is None:
            return False
        if self._train_overlaps_station_line(train, line):
            return True
        assigned_to_line = train.station_lane is not None and int(train.station_lane) == int(line["lane"])
        if not assigned_to_line:
            return False
        active_for_station = self._same_station_stop(train.active_scheduled_stop, stop)
        lifecycle_holds_station = (
            train.last_station_idx == station_idx
            and train.station_state in {
                "ROUTE_ASSIGNED",
                "DOCKING",
                "STOPPED_AT_PLATFORM",
                "DWELLING",
                "READY_TO_DEPART",
                "DEPARTING",
            }
        )
        if not active_for_station and not lifecycle_holds_station:
            return False
        return (
            not self._station_line_tail_clear(train, line)
            or train.pos <= float(line["end_m"]) + PARALLEL_RELEASE_MARGIN_M
        )

    def update_station_occupancy(self, station_idx: int) -> None:
        state = self.station_route_states[station_idx]
        stop = self.scheduled_stops[station_idx]
        lines = self.detect_station_lines(station_idx)
        route_lane = state.get("route_lane")
        locking_train_id = state.get("locking_train_id")
        assigned_train_id = state.get("assigned_train_id")
        for line in lines:
            lane = int(line["lane"])
            previous_occupied = line.get("occupied_by_train_id")
            overlapping = [
                train
                for train in self.trains
                if self._train_overlaps_station_line(train, line)
                and (
                    train.station_lane is None
                    or int(train.station_lane) == lane
                    or self._station_physical_lane(train, station_idx) == lane
                )
            ]
            occupying_train = min(overlapping, key=lambda item: abs(float(stop["pos_m"]) - item.pos), default=None)
            line["occupied_by_train_id"] = occupying_train.id if occupying_train is not None else None
            if route_lane == lane and assigned_train_id is not None:
                line["reserved_by_train_id"] = assigned_train_id
            elif line.get("reserved_by_train_id") is not None:
                reserved_train = next((train for train in self.trains if train.id == line.get("reserved_by_train_id")), None)
                if not self._station_line_reservation_still_valid(station_idx, stop, line, reserved_train):
                    line["reserved_by_train_id"] = None
            if occupying_train is not None:
                if occupying_train.station_lane is None:
                    occupying_train.station_lane = lane
                occupying_train.assigned_station_id = str(stop.get("name", station_idx))
                occupying_train.assigned_station_line_id = line["line_id"]
                occupying_train.assigned_platform = lane
                if previous_occupied != occupying_train.id:
                    self.log_station_event(station_idx, occupying_train, line, "STATION_LINE_OCCUPIED")
                line["route_state"] = "OCCUPIED"
            elif locking_train_id is not None and route_lane == lane:
                line["route_state"] = "LOCKED"
            elif route_lane == lane or line.get("reserved_by_train_id") is not None:
                line["route_state"] = "RESERVED"
            elif previous_occupied is not None:
                line["route_state"] = "RELEASE_PENDING"
            else:
                line["route_state"] = "FREE"

    def _station_slot_count(self, station_idx: int) -> int:
        count = 0
        for line in self.detect_station_lines(station_idx):
            if (
                line.get("reserved_by_train_id") is not None
                or line.get("occupied_by_train_id") is not None
                or line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
            ):
                count += 1
        return count

    def _station_reserved_count(self, station_idx: int) -> int:
        return sum(1 for line in self.detect_station_lines(station_idx) if line.get("reserved_by_train_id") is not None)

    def _station_occupied_count(self, station_idx: int) -> int:
        return sum(1 for line in self.detect_station_lines(station_idx) if line.get("occupied_by_train_id") is not None)

    def can_accept_train(self, station_idx: int, train: Train) -> Tuple[bool, str]:
        state = self.station_route_states[station_idx]
        capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
        if state.get("invariant_block"):
            return False, "ROUTE_CONFLICT"
        if self._station_receiving_route_active_for_other_train(station_idx, train):
            return False, "ROUTE_CONFLICT"
        if train.station_lane is not None:
            existing_line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if existing_line is not None and existing_line.get("reserved_by_train_id") == train.id:
                return True, "OK"
        if self._station_slot_count(station_idx) >= capacity:
            return False, "STATION_FULL"
        if train.station_lane is not None:
            line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if line is None:
                return False, "INVALID_LINE_ASSIGNMENT"
            if self._station_line_available_for_receive_route(station_idx, line, train):
                return True, "OK"
            return False, "STATION_LINE_OCCUPIED"
        if any(self._station_line_available_for_receive_route(station_idx, line, train) for line in self.detect_station_lines(station_idx)):
            return True, "OK"
        return False, "NO_FREE_PLATFORM"

    def _station_line_available_for_train(self, line: Dict[str, Any], train: Train) -> bool:
        reserved_by = line.get("reserved_by_train_id")
        occupied_by = line.get("occupied_by_train_id")
        if reserved_by is not None and reserved_by != train.id:
            return False
        if occupied_by is not None and occupied_by != train.id:
            return False
        return line.get("route_state") not in {"LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"} or reserved_by == train.id

    def _station_receiving_route_active_for_other_train(self, station_idx: int, train: Train) -> bool:
        state = self.station_route_states[station_idx]
        assigned_train_id = state.get("assigned_train_id")
        return (
            state.get("route_lane") is not None
            and assigned_train_id is not None
            and assigned_train_id != train.id
        )

    def _station_line_available_for_receive_route(self, station_idx: int, line: Dict[str, Any], train: Train) -> bool:
        if self._station_receiving_route_active_for_other_train(station_idx, train):
            return False
        reserved_by = line.get("reserved_by_train_id")
        occupied_by = line.get("occupied_by_train_id")
        if reserved_by is not None and reserved_by != train.id:
            return False
        if occupied_by is not None and occupied_by != train.id:
            return False
        if line.get("route_state") in {"LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}:
            return False
        if reserved_by == train.id:
            return True
        for other in self.trains:
            if other.id == train.id:
                continue
            if self._train_overlaps_station_line(other, line):
                other_lane = self._station_physical_lane(other, station_idx)
                if other_lane is None and other.station_lane is not None and other.last_station_idx == station_idx:
                    other_lane = int(other.station_lane)
                if other_lane is None or int(other_lane) == int(line["lane"]):
                    return False
                if not self._train_stably_stopped_in_station_line(other, station_idx):
                    return False
            if (
                other.station_lane is not None
                and int(other.station_lane) == int(line["lane"])
                and other.last_station_idx == station_idx
                and not self._station_line_tail_clear(other, line)
            ):
                return False
        return True

    def _station_holding_eoa(self, station_idx: int) -> float:
        station_start, _station_end = self._station_bounds(self.scheduled_stops[station_idx])
        return station_start - STOP_TARGET_BUFFER_M

    def assign_receive_route(self, station_idx: int, train: Train) -> bool:
        can_accept, reason = self.can_accept_train(station_idx, train)
        if not can_accept:
            train.station_reject_reason = reason
            line = self._station_line_for_lane(station_idx, train.station_lane)
            self.log_station_event(station_idx, train, line, reason, reject_reason=reason)
            return False
        lines = self.detect_station_lines(station_idx)
        candidate_line = None
        if train.station_lane is not None:
            line = self._station_line_for_lane(station_idx, int(train.station_lane))
            if line is not None and self._station_line_available_for_receive_route(station_idx, line, train):
                candidate_line = line
        else:
            for line in lines:
                if self._station_line_available_for_receive_route(station_idx, line, train):
                    candidate_line = line
                    break
        if candidate_line is None:
            train.station_reject_reason = "NO_FREE_PLATFORM"
            self.log_station_event(station_idx, train, None, "NO_FREE_PLATFORM", reject_reason="NO_FREE_PLATFORM")
            return False
        lane = int(candidate_line["lane"])
        train.station_reject_reason = ""
        train.station_lane = lane
        train.assigned_platform = lane
        train.assigned_station_id = str(candidate_line["station_id"])
        train.assigned_station_line_id = str(candidate_line["line_id"])
        candidate_line["reserved_by_train_id"] = train.id
        candidate_line["route_state"] = "RESERVED"
        self.lock_station_line(station_idx, lane, train)
        self.assign_stop_target(station_idx, train)
        self.log_station_event(station_idx, train, candidate_line, "RECEIVE_ROUTE_ASSIGNED")
        return True

    def lock_station_line(self, station_idx: int, lane: int, train: Train) -> None:
        state = self.station_route_states[station_idx]
        line = self._station_line_for_lane(station_idx, lane)
        state["route_lane"] = int(lane)
        state["assigned_train_id"] = train.id
        state["route_state"] = "RECEIVE_OPEN"
        if line is not None:
            line["reserved_by_train_id"] = train.id
            line["route_state"] = "RESERVED"

    def assign_stop_target(self, station_idx: int, train: Train) -> float:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        stop_target = float(line["stop_point_m"]) if line is not None else float(self.scheduled_stops[station_idx]["pos_m"])
        train.stop_target_pos = stop_target
        self.log_station_event(station_idx, train, line, "STOP_TARGET_ASSIGNED")
        return stop_target

    def start_dwell_if_stopped_correctly(self, station_idx: int, train: Train, stop: Dict[str, Any]) -> bool:
        if not (train.commanded_stop and train.zero_speed_detected and train.door_authorized):
            return False
        if train.station_lane is None or abs(train.pos - float(stop["pos_m"])) > STOP_ACCURACY_TOL_M:
            return False
        self.assign_stop_target(station_idx, train)
        train.pos = train.stop_target_pos
        train.speed = 0.0
        train.prev_accel = 0.0
        train.reset_non_emergency_stop_latches()
        train.standstill_required = True
        train.standstill_anchor_pos = train.pos
        self._set_train_station_state(train, station_idx, "STOPPED_AT_PLATFORM", "aligned_stop")
        train.dwell_remaining_s = self._station_dwell_time_s(station_idx, stop, train)
        self._record_station_arrival(station_idx, train, train.dwell_remaining_s)
        train.next_scheduled_stop_idx += 1
        self._set_train_station_state(train, station_idx, "DWELLING", "dwell_started")
        self.log_station_event(station_idx, train, self._station_line_for_lane(station_idx, train.station_lane), "DWELL_STARTED")
        return True

    def prepare_departure_route(self, station_idx: int, train: Train) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            self.log_station_event(station_idx, train, None, "INVALID_LINE_ASSIGNMENT", reject_reason="INVALID_LINE_ASSIGNMENT")
            return False
        train.station_reject_reason = ""
        line["route_state"] = "DEPARTING"
        self.log_station_event(station_idx, train, line, "DEPARTURE_ROUTE_ASSIGNED")
        return True

    def release_station_line_only_after_tail_clear(self, station_idx: int, train: Train) -> bool:
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        if not self._station_line_tail_clear(train, line):
            line["route_state"] = "RELEASE_PENDING"
            self.log_station_event(station_idx, train, line, "TAIL_NOT_CLEAR")
            return False
        line["occupied_by_train_id"] = None
        line["reserved_by_train_id"] = None
        line["route_state"] = "FREE"
        state = self.station_route_states[station_idx]
        if state.get("assigned_train_id") == train.id:
            state["assigned_train_id"] = None
        if state.get("locking_train_id") == train.id:
            state["locking_train_id"] = None
        self.log_station_event(station_idx, train, line, "TAIL_CLEAR_RELEASED")
        self.log_station_event(station_idx, train, line, "LINE_RELEASED")
        self.clear_completed_stop_for_train(station_idx, train)
        return True

    def clear_completed_stop_for_train(self, station_idx: int | None, train: Train) -> None:
        train.station_lane = None
        train.assigned_platform = None
        train.assigned_station_id = None
        train.assigned_station_line_id = None
        self._set_train_station_state(train, station_idx, "COMPLETED_STOP", "station_line_released")

    def log_station_event(self, station_idx: int, train: Train | None, line: Dict[str, Any] | None, reason: str, **extra):
        stop = self.scheduled_stops[station_idx]
        station_id = str(stop.get("name", station_idx))
        capacity = max(1, int(stop.get("capacity", 3)))
        if train is None:
            return
        station_start, station_end = self._station_bounds(stop)
        train.log_event(
            "STATION_EVENT",
            reason,
            station_id=station_id,
            station_capacity=capacity,
            station_reserved_count=self._station_reserved_count(station_idx),
            station_occupied_count=self._station_occupied_count(station_idx),
            train_state_at_station=train.station_state,
            assigned_station_line_id=train.assigned_station_line_id,
            line_id=None if line is None else line.get("line_id"),
            line_type=None if line is None else line.get("type"),
            line_state=None if line is None else line.get("route_state"),
            reserved_by_train_id=None if line is None else line.get("reserved_by_train_id"),
            occupied_by_train_id=None if line is None else line.get("occupied_by_train_id"),
            train_head_pos=train.pos,
            train_tail_pos=train.pos - train.length,
            line_start_m=station_start if line is None else line.get("start_m"),
            line_end_m=station_end if line is None else line.get("end_m"),
            stop_target_m=train.stop_target_pos,
            route_lock_state=self._route_state_label(self.station_route_states[station_idx]),
            departure_route_state=None if line is None else line.get("route_state"),
            tail_clear=False if line is None else self._station_line_tail_clear(train, line),
            dwell_remaining=train.dwell_remaining_s,
            **extra,
        )

    def _check_station_invariants(self) -> bool:
        ok = True
        for station_idx, stop in enumerate(self.scheduled_stops):
            self.update_station_occupancy(station_idx)
            state = self.station_route_states[station_idx]
            state["invariant_block"] = False
            capacity = max(1, int(stop.get("capacity", 3)))
            active_slots = self._station_slot_count(station_idx)
            if active_slots > capacity:
                ok = False
                state["invariant_block"] = True
                train = next((item for item in self.trains if self._station_overlapped_by_train(item) and self._station_overlapped_by_train(item)[0] == station_idx), None)
                if train is not None:
                    self.log_station_event(station_idx, train, None, "STATION_FULL", reject_reason="STATION_CAPACITY_INVARIANT")
            for line in self.detect_station_lines(station_idx):
                line_capacity = max(1, int(line.get("capacity", 1)))
                reserved_train_ids = {
                    str(line.get("reserved_by_train_id"))
                    for line_candidate in [line]
                    if line_candidate.get("reserved_by_train_id") is not None
                }
                overlapping = [
                    train
                    for train in self.trains
                    if self._train_overlaps_station_line(train, line)
                    and (train.station_lane is None or int(train.station_lane) == int(line["lane"]))
                ]
                unique_users = set(reserved_train_ids)
                unique_users.update(train.id for train in overlapping)
                if len(unique_users) > line_capacity:
                    ok = False
                    state["invariant_block"] = True
                    train = overlapping[0] if overlapping else next((item for item in self.trains if item.id in unique_users), None)
                    if train is not None:
                        self.log_station_event(station_idx, train, line, "ROUTE_CONFLICT", reject_reason="LINE_CAPACITY_INVARIANT")
                if line.get("route_state") == "FREE" and overlapping:
                    ok = False
                    state["invariant_block"] = True
                    self.log_station_event(station_idx, overlapping[0], line, "TAIL_NOT_CLEAR", reject_reason="FREE_WITH_OVERLAP")
                for train in overlapping:
                    if train.station_state in {"DOCKING", "DWELLING", "READY_TO_DEPART"} and train.assigned_station_line_id not in {None, line.get("line_id")}:
                        ok = False
                        state["invariant_block"] = True
                        self.log_station_event(station_idx, train, line, "INVALID_LINE_ASSIGNMENT", reject_reason="STOP_TARGET_LINE_MISMATCH")
        return ok

    def _station_physical_lane(self, train: Train, station_idx: int) -> int | None:
        zone_id = f"STATION:{station_idx}"
        if train.protection_zone_id == zone_id:
            return int(train.protection_lane)
        station = self._station_overlapped_by_train(train)
        if station is None or station[0] != station_idx:
            return None
        if train.station_lane is not None and self._same_station_stop(train.active_scheduled_stop, station[1]):
            return int(train.station_lane)
        return None

    def _station_lane_occupied(self, station_idx: int, lane: int) -> bool:
        line = self._station_line_for_lane(station_idx, lane)
        if line is not None and (
            line.get("reserved_by_train_id") is not None
            or line.get("occupied_by_train_id") is not None
            or line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
        ):
            return True
        stop = self.scheduled_stops[station_idx]
        return any(
            self._station_physical_lane(train, station_idx) == lane
            and self._train_overlaps_station(train, stop)
            for train in self.trains
        )

    def _station_route_lane_order(self, capacity: int) -> List[int]:
        return list(range(max(1, capacity)))

    def _is_terminal_station(self, station_idx: int | None) -> bool:
        if station_idx is None or not (0 <= station_idx < len(self.scheduled_stops)):
            return False
        stop = self.scheduled_stops[station_idx]
        return float(stop.get("pos_m", 0.0)) >= self.track_end_m - STOP_ACCURACY_TOL_M

    def _is_final_scheduled_station(self, station_idx: int | None) -> bool:
        return station_idx is not None and station_idx == len(self.scheduled_stops) - 1

    def _schedule_record_for_train_station(self, train: Train | None, stop: Dict[str, Any], station_idx: int | None = None) -> Dict[str, Any] | None:
        if train is None or self.headway_manager.mode != "timetable":
            return None
        station_name = str(stop.get("name", "")).strip().lower()
        station_aliases = {station_name}
        if station_idx is not None:
            station_aliases.add(f"s{station_idx + 2}".lower())
        for record in getattr(train, "schedule_records", []) or []:
            if str(record.get("station", "")).strip().lower() in station_aliases:
                return dict(record)
        return None

    def _station_dwell_time_s(self, station_idx: int | None, stop: Dict[str, Any], train: Train | None = None) -> float:
        if self._is_terminal_station(station_idx) or self._is_final_scheduled_station(station_idx):
            return float("inf")
        schedule_record = self._schedule_record_for_train_station(train, stop, station_idx)
        if schedule_record is not None:
            planned_departure_s = schedule_record.get("departure_time_s")
            planned_arrival_s = schedule_record.get("arrival_time_s")
            operational_time_s = self._timetable_operational_time_s()
            late_for_schedule = planned_arrival_s is not None and operational_time_s > float(planned_arrival_s)
            if late_for_schedule:
                late_s = operational_time_s - float(planned_arrival_s)
                scheduled_dwell_s = schedule_record.get("dwell_s")
                if scheduled_dwell_s is None and planned_departure_s is not None:
                    scheduled_dwell_s = max(0.0, float(planned_departure_s) - float(planned_arrival_s))
                if scheduled_dwell_s is not None:
                    return max(MIN_TIMETABLE_RECOVERY_DWELL_S, float(scheduled_dwell_s) - late_s)
                return max(MIN_TIMETABLE_RECOVERY_DWELL_S, MIN_PASSENGER_DWELL_S - late_s)
            if planned_departure_s is not None:
                return max(MIN_PASSENGER_DWELL_S, float(planned_departure_s) - operational_time_s)
            scheduled_dwell_s = schedule_record.get("dwell_s")
            if scheduled_dwell_s is not None:
                return max(MIN_PASSENGER_DWELL_S, float(scheduled_dwell_s))
        minimum_departure_s = self.sim_time_s + MIN_PASSENGER_DWELL_S
        if station_idx is None:
            return MIN_PASSENGER_DWELL_S
        target_headway_s = max(0.0, float(self.headway_manager.nominal_target_headway_s()))
        if target_headway_s <= 0.0:
            return MIN_PASSENGER_DWELL_S
        previous_candidates = [
            value
            for value in (
                self.station_next_departure_release_s.get(station_idx),
                self.station_last_departure_s.get(station_idx),
            )
            if value is not None
        ]
        previous_slot_s = max(previous_candidates) if previous_candidates else None
        if previous_slot_s is None:
            departure_slot_s = minimum_departure_s
        else:
            departure_slot_s = max(minimum_departure_s, previous_slot_s + target_headway_s)
        self.station_next_departure_release_s[station_idx] = departure_slot_s
        return max(MIN_PASSENGER_DWELL_S, departure_slot_s - self.sim_time_s)

    def _record_station_departure_headway(self, station_idx: int, train: Train):
        previous = self.station_last_departure_s.get(station_idx)
        self.station_last_departure_s[station_idx] = self.sim_time_s
        self.station_next_departure_release_s[station_idx] = max(
            self.station_next_departure_release_s.get(station_idx, self.sim_time_s),
            self.sim_time_s,
        )
        if previous is None:
            return
        actual = max(0.0, self.sim_time_s - previous)
        target = self.headway_manager.nominal_target_headway_s()
        self.station_headway_actual_s.setdefault(station_idx, []).append(actual)
        if target > 0.0:
            self.station_headway_deviation_s.setdefault(station_idx, []).append(actual - target)

    def _station_departure_headway_hold_s(self, station_idx: int | None) -> float:
        if station_idx is None:
            return 0.0
        previous = self.station_last_departure_s.get(station_idx)
        if previous is None:
            return 0.0
        target_headway_s = max(0.0, float(self.headway_manager.nominal_target_headway_s()))
        if target_headway_s <= 0.0:
            return 0.0
        return max(0.0, previous + target_headway_s - self.sim_time_s)

    def _station_schedule_departure_hold_s(self, station_idx: int | None, train: Train) -> float:
        if station_idx is None or self.headway_manager.mode != "timetable":
            return 0.0
        if not (0 <= station_idx < len(self.scheduled_stops)):
            return 0.0
        record = self._schedule_record_for_train_station(train, self.scheduled_stops[station_idx], station_idx)
        if record is None or record.get("departure_time_s") is None:
            return 0.0
        operational_time_s = self._timetable_operational_time_s()
        planned_arrival_s = record.get("arrival_time_s")
        if planned_arrival_s is not None and operational_time_s > float(planned_arrival_s):
            return 0.0
        return max(0.0, float(record["departure_time_s"]) - operational_time_s)

    def _record_station_arrival(self, station_idx: int, train: Train, dwell_s: float) -> None:
        previous = self.station_last_arrival_s.get(station_idx)
        self.station_last_arrival_s[station_idx] = self.sim_time_s
        arrival_headway_s = None
        if previous is not None:
            arrival_headway_s = max(0.0, self.sim_time_s - previous)
            self.station_arrival_headway_actual_s.setdefault(station_idx, []).append(arrival_headway_s)
        schedule_record = (
            self._schedule_record_for_train_station(train, self.scheduled_stops[station_idx], station_idx)
            if 0 <= station_idx < len(self.scheduled_stops)
            else None
        )
        schedule_variance_s = None
        if schedule_record is not None and schedule_record.get("arrival_time_s") is not None:
            schedule_variance_s = self._timetable_operational_time_s() - float(schedule_record["arrival_time_s"])
        self.analytics.setdefault("station_arrivals", {}).setdefault(station_idx, []).append(
            {
                "train_id": train.id,
                "arrival_time_s": self.sim_time_s,
                "arrival_headway_s": arrival_headway_s,
                "scheduled_arrival_time_s": None if schedule_record is None else schedule_record.get("arrival_time_s"),
                "schedule_station": None if schedule_record is None else schedule_record.get("station"),
                "schedule_variance_s": schedule_variance_s,
                "planned_dwell_s": dwell_s,
                "passenger_dwell_s": dwell_s,
                "station_wait_s": dwell_s,
            }
        )

    def _set_train_station_state(self, train: Train, station_idx: int | None, state: str, reason: str = ""):
        stop_key = train.stop_identity() if train.active_scheduled_stop is not None else train.station_state_stop_key
        if state == train.station_state and stop_key == train.station_state_stop_key and station_idx == train.last_station_idx:
            return
        train.station_state = state
        train.station_state_stop_key = stop_key
        train.last_station_idx = station_idx
        train.assigned_platform = train.station_lane
        train.last_station_state_reason = reason
        station_id = "--"
        if station_idx is not None and 0 <= station_idx < len(self.scheduled_stops):
            station_id = str(self.scheduled_stops[station_idx].get("name", station_idx))
        train.log_event(
            "STATION_STATE",
            reason or state,
            station_id=station_id,
            station_state=state,
            assigned_line=train.station_lane,
        )

    def _route_state_label(self, state: Dict[str, Any]) -> str:
        if state.get("route_lane") is not None:
            return "RECEIVE_OPEN"
        if state.get("locking_train_id") is not None:
            return "LOCKED"
        return "FREE"

    def _log_eoa_update(self, train: Train, old_eoa: float | None, new_eoa: float, reason: str, station_idx: int | None):
        if old_eoa is not None and abs(old_eoa - new_eoa) <= 1e-6 and reason == train.last_dispatched_eoa_reason:
            return
        station_id = "--"
        route_state = "--"
        if station_idx is not None and 0 <= station_idx < len(self.station_route_states):
            station_id = str(self.scheduled_stops[station_idx].get("name", station_idx))
            route_state = self._route_state_label(self.station_route_states[station_idx])
        train.log_event(
            "EOA_UPDATE",
            reason,
            station_id=station_id,
            station_state=train.station_state,
            assigned_line=train.station_lane,
            route_state=route_state,
            old_eoa=old_eoa,
            new_eoa=new_eoa,
            assigned_station_id=train.assigned_station_id,
            assigned_station_line_id=train.assigned_station_line_id,
            station_receive_state=train.station_state,
            station_reject_reason=train.station_reject_reason,
            stop_target=train.stop_target_pos,
            distance_to_stop=train.distance_to_stop_target(),
            atp_curve_limit_kmh=ms_to_kmh(train.curves.get("P", 0.0)),
            sbi_state=train.service_brake_latch,
            ebi_state=train.emg_latch,
            dwell_remaining=train.dwell_remaining_s,
            tail_clear_state=(
                station_idx is None
                or not self._train_overlaps_station(train, self.scheduled_stops[station_idx])
            ),
        )
        train.last_dispatched_eoa = new_eoa
        train.last_dispatched_eoa_reason = reason

    def _update_station_routes(self) -> bool:
        changed = False
        self._sync_station_route_states()
        for station_idx, stop in enumerate(self.scheduled_stops):
            capacity = max(1, int(stop.get("capacity", 3)))
            state = self.station_route_states[station_idx]
            self.update_station_occupancy(station_idx)
            station_start, station_end = self._station_bounds(stop)
            route_lane = state.get("route_lane")

            if route_lane is not None:
                for train in self.trains:
                    if (
                        self._train_overlaps_station(train, stop)
                        and (
                            train.station_lane == route_lane
                            or self._station_physical_lane(train, station_idx) == route_lane
                        )
                    ):
                        # As soon as the train touches the routed station line, revoke the green aspect.
                        state["route_lane"] = None
                        state["assigned_train_id"] = train.id
                        state["locking_train_id"] = train.id
                        state["route_state"] = "LOCKED"
                        line = self._station_line_for_lane(station_idx, int(route_lane))
                        if line is not None:
                            line["reserved_by_train_id"] = train.id
                            line["occupied_by_train_id"] = train.id
                            line["route_state"] = "OCCUPIED"
                        state["switch_started"] = False
                        state["lock_remaining_s"] = 0.0
                        self._set_train_station_state(train, station_idx, "DOCKING", "route_consumed")
                        route_lane = None
                        changed = True
                        break

            locking_train = next((train for train in self.trains if train.id == state.get("locking_train_id")), None)
            if locking_train is not None:
                if locking_train.speed <= STANDSTILL_SPEED_EPS and self._train_overlaps_station(locking_train, stop):
                    if not state.get("switch_started", False):
                        state["lock_remaining_s"] = TURNOUT_LOCK_S
                        state["switch_started"] = True
                        if locking_train.dwell_remaining_s > 0.0:
                            self._set_train_station_state(locking_train, station_idx, "DWELLING", "switch_lock_started")
                        else:
                            self._set_train_station_state(locking_train, station_idx, "STOPPED_AT_PLATFORM", "switch_lock_started")
                        changed = True
                    else:
                        state["lock_remaining_s"] = max(0.0, float(state.get("lock_remaining_s", 0.0)) - DT)
                if state.get("switch_started") and float(state.get("lock_remaining_s", 0.0)) <= 0.0:
                    state["locking_train_id"] = None
                    state["switch_started"] = False
                    state["route_state"] = "FREE"
                    changed = True
            elif float(state.get("lock_remaining_s", 0.0)) > 0.0:
                state["lock_remaining_s"] = max(0.0, float(state.get("lock_remaining_s", 0.0)) - DT)

            self.update_station_occupancy(station_idx)

            target_zone_id = f"STATION:{station_idx}"
            for train in self.trains:
                if (
                    self._same_station_stop(train.active_scheduled_stop, stop)
                    and train.station_lane is not None
                    and not self._train_overlaps_station(train, stop)
                ):
                    current_station = self._station_for_train(train)
                    if (
                        current_station is not None
                        and not self._same_station_stop(current_station[1], stop)
                        and train.protection_zone_id != target_zone_id
                        and self._station_overlapped_by_train(train) is None
                    ):
                        self.clear_completed_stop_for_train(station_idx, train)

            preassigned = [
                train for train in self.trains
                if self._same_station_stop(train.active_scheduled_stop, stop)
                and train.pos < station_end
                and train.pos >= station_start - STATION_ROUTE_APPROACH_M
                and train.station_lane is not None
                and not self._train_overlaps_station(train, stop)
            ]
            if preassigned:
                preassigned.sort(key=lambda item: abs(float(stop["pos_m"]) - item.pos))
                for candidate in preassigned:
                    lane = int(candidate.station_lane)
                    if 0 <= lane < capacity and self.assign_receive_route(station_idx, candidate):
                        self._set_train_station_state(candidate, station_idx, "ROUTE_ASSIGNED", "preassigned_route_open")
                        changed = True
                        break

            approaching = [
                train for train in self.trains
                if self._same_station_stop(train.active_scheduled_stop, stop)
                and train.pos < station_end
                and train.pos >= station_start - STATION_ROUTE_APPROACH_M
                and train.station_lane is None
            ]
            if not approaching:
                continue
            approaching.sort(key=lambda item: (int(item.protection_lane), abs(float(stop["pos_m"]) - item.pos)))
            for candidate in approaching:
                if self.assign_receive_route(station_idx, candidate):
                    self._set_train_station_state(candidate, station_idx, "ROUTE_ASSIGNED", "route_open")
                    changed = True
                    break
        return changed

    def _train_has_station_route_authority(self, train: Train) -> bool:
        if train.active_scheduled_stop is None or train.station_lane is None:
            return False
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return False
        capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
        if not (0 <= int(train.station_lane) < capacity):
            return False
        state = self.station_route_states[station_idx]
        line = self._station_line_for_lane(station_idx, train.station_lane)
        if line is None:
            return False
        if state.get("route_lane") == train.station_lane and state.get("assigned_train_id") == train.id:
            return True
        if line.get("reserved_by_train_id") == train.id:
            return True
        if line.get("occupied_by_train_id") == train.id:
            return True
        return self._train_overlaps_station_line(train, line)

    def _train_has_station_stop_eoa_authority(self, train: Train) -> bool:
        if not self._train_has_station_route_authority(train):
            return False
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return False
        stop = self.scheduled_stops[station_idx]
        _station_start, station_end = self._station_bounds(stop)
        if train.pos > station_end:
            return False
        if train.commanded_stop:
            return True
        state = self.station_route_states[station_idx] if station_idx < len(self.station_route_states) else {}
        line = self._station_line_for_lane(station_idx, train.station_lane)
        # A reserved receive route is route authority, not yet stop authority.
        # Keep the station stop EOA out until the train enters the commanded-stop
        # approach; otherwise ATP/ATO starts braking hundreds of metres too early.
        if line is not None and line.get("reserved_by_train_id") == train.id:
            return self._train_overlaps_station(train, stop)
        return (
            (
                state.get("route_lane") == train.station_lane
                and state.get("assigned_train_id") == train.id
            )
            and self._train_overlaps_station(train, stop)
        )

    def _zone_cleared_by_train(self, train: Train, zone_end_m: float, follower: Train) -> bool:
        return train.safe_rear_end_pos() > zone_end_m + PARALLEL_RELEASE_MARGIN_M

    def _train_waiting_for_station_departure(self, train: Train, station_idx: int) -> bool:
        if not (0 <= station_idx < len(self.scheduled_stops)):
            return False
        if (
            train.active_scheduled_stop is None
            and train.dwell_remaining_s <= 0.0
            and not (
                train.station_lane is not None
                and train.last_station_idx == station_idx
                and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
            )
        ):
            return False
        stop = self.scheduled_stops[station_idx]
        stop_pos = float(stop["pos_m"])
        station_start, station_end = self._station_bounds(stop)
        if not (station_start <= train.pos <= station_end):
            return False
        if train.dwell_remaining_s > 0.0 or train.departure_hold:
            return True
        if train.zero_speed_detected and train.pos >= stop_pos - STOP_ACCURACY_TOL_M:
            return True
        if train.pos > stop_pos + STOP_ACCURACY_TOL_M and train.speed <= kmh_to_ms(5.0):
            return True
        return False

    def _train_still_holding_previous_station_line(self, train: Train, next_station_idx: int | None) -> bool:
        if train.station_lane is None or train.last_station_idx is None:
            return False
        previous_station_idx = train.last_station_idx
        if previous_station_idx == next_station_idx or not (0 <= previous_station_idx < len(self.scheduled_stops)):
            return False
        line = self._station_line_for_lane(previous_station_idx, train.station_lane)
        if line is None:
            return False
        previous_stop = self.scheduled_stops[previous_station_idx]
        return self._train_overlaps_station(train, previous_stop) or not self._station_line_tail_clear(train, line)

    def _arm_parallel_release_lock(self, zone_id: str, lane: int):
        key = (zone_id, int(lane))
        self.parallel_release_locks[key] = max(float(self.parallel_release_locks.get(key, 0.0)), TURNOUT_LOCK_S)

    def _tick_parallel_release_locks(self):
        expired = []
        for key, remaining_s in self.parallel_release_locks.items():
            next_remaining = max(0.0, remaining_s - DT)
            self.parallel_release_locks[key] = next_remaining
            if next_remaining <= 0.0:
                expired.append(key)
        for key in expired:
            self.parallel_release_locks.pop(key, None)

    def _parallel_departure_holds(self) -> Dict[str, float]:
        holds: Dict[str, float] = {}

        def apply_lane_gate(
            zone_id: str,
            zone_end_m: float,
            trains: List[Train],
            gated_train_ids: set[str] | None = None,
            zone_start_m: float | None = None,
        ):
            lane_map: Dict[int, Train] = {}
            for train in trains:
                lane_map[int(train.protection_lane)] = train
            for lane in sorted(lane_map):
                train = lane_map[lane]
                if gated_train_ids is not None and train.id not in gated_train_ids:
                    continue
                for prior_lane in range(lane):
                    prior_train = lane_map.get(prior_lane)
                    release_lock_active = self.parallel_release_locks.get((zone_id, prior_lane), 0.0) > 0.0
                    prior_train_in_departure_zone = (
                        prior_train is not None
                        and (zone_start_m is None or prior_train.pos >= zone_start_m - STOP_ACCURACY_TOL_M)
                    )
                    prior_train_not_clear = (
                        prior_train_in_departure_zone
                        and prior_train is not None
                        and not self._zone_cleared_by_train(prior_train, zone_end_m, train)
                    )
                    if release_lock_active or prior_train_not_clear:
                        hold_margin = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
                        hold_eoa = train.reported_pos + hold_margin + STOP_SVL_OFFSET_M + PARALLEL_RELEASE_MARGIN_M
                        if isinstance(zone_id, str) and zone_id.startswith("STATION:"):
                            tail_clear_pos = zone_end_m + PARALLEL_RELEASE_MARGIN_M + train.length
                            hold_eoa = max(hold_eoa, tail_clear_pos + STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                        holds[train.id] = hold_eoa
                        break

        source_trains = [train for train in self.trains if train.protection_zone_id == "SOURCE"]
        if source_trains:
            apply_lane_gate("SOURCE", SOURCE_TRAIN_EXIT_M, source_trains)

        station_zone_ids = sorted(
            {
                train.protection_zone_id
                for train in self.trains
                if isinstance(train.protection_zone_id, str) and train.protection_zone_id.startswith("STATION:")
            }
        )
        for zone_id in station_zone_ids:
            try:
                station_idx = int(zone_id.split(":", 1)[1])
            except (IndexError, ValueError):
                continue
            if not (0 <= station_idx < len(self.scheduled_stops)):
                continue
            station_start, station_end = self._station_bounds(self.scheduled_stops[station_idx])
            station_trains = [train for train in self.trains if train.protection_zone_id == zone_id]
            if station_trains:
                gated_train_ids = {
                    train.id
                    for train in station_trains
                    if self._train_waiting_for_station_departure(train, station_idx)
                }
                apply_lane_gate(zone_id, station_end, station_trains, gated_train_ids, station_start)

        return holds

    def _needs_station_departure_authority_hold(self, train: Train, packet: SafeMovementPacket) -> bool:
        if (
            train.station_lane is not None
            and train.last_station_idx is not None
            and 0 <= train.last_station_idx < len(self.scheduled_stops)
            and train.next_scheduled_stop_idx >= len(train.scheduled_stops)
        ):
            _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
            if station_end >= self.track_end_m - STOP_ACCURACY_TOL_M:
                return True
        current_station = self._station_for_train(train)
        if (
            train.station_lane is not None
            and train.last_station_idx is not None
            and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
            and train.speed <= kmh_to_ms(5.0)
        ):
            authority_ahead_m = packet.eoa_m - train.reported_pos
            return authority_ahead_m < DEPARTURE_RELEASE_MIN_AUTHORITY_M
        if current_station is None or train.active_scheduled_stop is None:
            return False
        _current_idx, current_stop = current_station
        if self._same_station_stop(train.active_scheduled_stop, current_stop):
            return False
        if train.dwell_remaining_s > 0.0:
            return False
        if not (train.zero_speed_detected or train.speed <= kmh_to_ms(5.0)):
            return False
        authority_ahead_m = packet.eoa_m - train.reported_pos
        return authority_ahead_m < DEPARTURE_RELEASE_MIN_AUTHORITY_M

    def _enforce_station_cd_routes(self):
        for station_idx, state in enumerate(self.station_route_states):
            route_lane = state.get("route_lane")
            if route_lane is None:
                continue
            occupied_train = next(
                (
                    train for train in self.trains
                    if self._station_physical_lane(train, station_idx) == int(route_lane)
                    and self._train_overlaps_station(train, self.scheduled_stops[station_idx])
                ),
                None,
            )
            if occupied_train is not None:
                state["route_lane"] = None
                state["assigned_train_id"] = occupied_train.id
                state["locking_train_id"] = occupied_train.id
                state["route_state"] = "LOCKED"
                state["switch_started"] = False
                state["lock_remaining_s"] = 0.0

    def _update_parallel_protection_zones(self):
        previous_station_lanes: Dict[str, Tuple[str, int]] = {}
        for train in self.trains:
            if isinstance(train.protection_zone_id, str) and train.protection_zone_id.startswith("STATION:"):
                previous_station_lanes[train.id] = (train.protection_zone_id, int(train.protection_lane))
        for train in self.trains:
            previous_zone_id = train.protection_zone_id
            previous_lane = int(train.protection_lane)
            train.protection_zone_id = None
            train.protection_lane = 0
            train.departure_hold = False
            if not self._source_zone_contains(train):
                if train.source_lane is not None:
                    self._arm_parallel_release_lock("SOURCE", int(train.source_lane))
                train.source_lane = None
            if train.station_lane is not None:
                routed_station = self._station_for_train(train)
                active_station_start = None
                active_station_end = None
                if train.active_scheduled_stop is not None:
                    active_station_start, active_station_end = self._station_bounds(train.active_scheduled_stop)
                active_station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                active_line_id = (
                    None
                    if active_station_idx is None
                    else self._station_line_id(active_station_idx, int(train.station_lane))
                )
                station_assignment_matches_active_stop = (
                    active_station_idx is not None
                    and train.assigned_station_line_id == active_line_id
                )
                approaching_routed_stop = (
                    train.active_scheduled_stop is not None
                    and train.station_lane is not None
                    and active_station_start is not None
                    and station_assignment_matches_active_stop
                    and active_station_start - STATION_ROUTE_APPROACH_M <= train.pos <= active_station_end
                )
                if routed_station is None and not approaching_routed_stop:
                    if isinstance(previous_zone_id, str) and previous_zone_id.startswith("STATION:"):
                        try:
                            previous_station_idx = int(previous_zone_id.split(":", 1)[1])
                        except (IndexError, ValueError):
                            previous_station_idx = None
                        if previous_station_idx is not None and 0 <= previous_station_idx < len(self.scheduled_stops):
                            line = self._station_line_for_lane(previous_station_idx, train.station_lane)
                            if line is not None and not self._station_line_tail_clear(train, line):
                                train.protection_zone_id = previous_zone_id
                                train.protection_lane = previous_lane
                                line["route_state"] = "RELEASE_PENDING"
                                self._set_train_station_state(train, previous_station_idx, "DEPARTING", "tail_not_clear")
                                self.log_station_event(previous_station_idx, train, line, "TAIL_NOT_CLEAR")
                                continue
                            self._arm_parallel_release_lock(previous_zone_id, previous_lane)
                            if self.release_station_line_only_after_tail_clear(previous_station_idx, train):
                                continue
                    train.station_lane = None
                    train.assigned_platform = None
                    train.assigned_station_id = None
                    train.assigned_station_line_id = None
                    if train.active_scheduled_stop is None:
                        self._set_train_station_state(train, None, "COMPLETED_STOP", "station_lane_released")

        source_trains = [train for train in self.trains if self._source_zone_contains(train)]
        source_trains.sort(key=lambda item: item.pos, reverse=True)
        source_capacity = max(
            (max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS))) for source in self.source_trains),
            default=SOURCE_VISIBLE_ACTIVE_TRAINS,
        )
        used_source_lanes: set[int] = set()
        for train in source_trains:
            lane = int(train.source_lane) if train.source_lane is not None else None
            if lane is None or lane < 0 or lane >= source_capacity or lane in used_source_lanes:
                lane = next((candidate for candidate in range(source_capacity) if candidate not in used_source_lanes), None)
            if lane is None:
                continue
            train.source_lane = lane
            used_source_lanes.add(lane)
            train.protection_zone_id = "SOURCE"
            train.protection_lane = int(train.source_lane)

        station_groups: Dict[int, List[Train]] = {}
        for train in self.trains:
            if self._train_has_station_route_authority(train):
                station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                _station_start, station_end = self._station_bounds(train.active_scheduled_stop)
                if (
                    station_idx is not None
                    and train.pos <= station_end
                    and train.protection_zone_id != "SOURCE"
                ):
                    train.protection_zone_id = f"STATION:{station_idx}"
                    train.protection_lane = int(train.station_lane)
            station = self._station_overlapped_by_train(train)
            if station is None:
                continue
            station_idx, _stop = station
            station_groups.setdefault(station_idx, []).append(train)
        for station_idx, trains in station_groups.items():
            capacity = max(1, int(self.scheduled_stops[station_idx].get("capacity", 3)))
            if capacity <= 1:
                continue
            zone_id = f"STATION:{station_idx}"
            current_stop = self.scheduled_stops[station_idx]
            used_lanes: set[int] = set()
            trains.sort(key=lambda item: item.id)
            for train in trains:
                previous_zone_lane = previous_station_lanes.get(train.id)
                previous_lane = previous_zone_lane[1] if previous_zone_lane is not None and previous_zone_lane[0] == zone_id else None
                if previous_lane is not None and 0 <= previous_lane < capacity and previous_lane not in used_lanes:
                    lane = previous_lane
                else:
                    lane = next((candidate for candidate in range(capacity) if candidate not in used_lanes), None)
                if lane is None:
                    continue
                used_lanes.add(lane)
                active_is_current_station = self._same_station_stop(train.active_scheduled_stop, current_stop)
                if train.station_lane is None and active_is_current_station:
                    train.station_lane = lane
                train.protection_zone_id = zone_id
                train.protection_lane = int(train.station_lane if active_is_current_station and train.station_lane is not None else lane)

        self._enforce_station_cd_routes()

    def _update_train_stop_schedule(self, train: Train) -> bool:
        immediate_packet_required = False
        while train.next_scheduled_stop_idx < len(train.scheduled_stops):
            stop = train.scheduled_stops[train.next_scheduled_stop_idx]
            if stop["pos_m"] < train.pos - STOP_ACCURACY_TOL_M and train.dwell_remaining_s <= 0.0:
                train.next_scheduled_stop_idx += 1
                continue
            break

        if train.dwell_remaining_s > 0.0:
            train.active_scheduled_stop = train.scheduled_stops[max(0, train.next_scheduled_stop_idx - 1)]
            train.commanded_stop = True
            station_idx = self._station_index_for_stop(train.active_scheduled_stop)
            if station_idx is None and train.station_lane is not None:
                station_idx = train.last_station_idx
            self._set_train_station_state(train, station_idx, "DWELLING", "dwell_countdown")
            train.pos = train.standstill_anchor_pos
            train.speed = 0.0
            train.prev_accel = 0.0
            train.reset_non_emergency_stop_latches()
            train.standstill_required = True
            train.standstill_anchor_pos = train.pos
            if train.dwell_remaining_s != float("inf"):
                train.dwell_remaining_s = max(0.0, train.dwell_remaining_s - DT)
            if train.dwell_remaining_s <= 0.0:
                schedule_hold_s = self._station_schedule_departure_hold_s(station_idx, train)
                if schedule_hold_s > 0.0:
                    train.dwell_remaining_s = schedule_hold_s
                    return immediate_packet_required
                headway_hold_s = self._station_departure_headway_hold_s(station_idx)
                if headway_hold_s > 0.0:
                    train.dwell_remaining_s = headway_hold_s
                    return immediate_packet_required
                train.commanded_stop = False
                self._set_train_station_state(train, station_idx, "READY_TO_DEPART", "dwell_complete")
                if station_idx is not None:
                    self._record_station_departure_headway(station_idx, train)
                    self.prepare_departure_route(station_idx, train)
                    self.log_station_event(
                        station_idx,
                        train,
                        self._station_line_for_lane(station_idx, train.station_lane),
                        "DWELL_COMPLETED",
                    )
                train.active_scheduled_stop = None
                train.reset_non_emergency_stop_latches()
                train.standstill_required = False
                immediate_packet_required = True
            return immediate_packet_required

        if train.next_scheduled_stop_idx >= len(train.scheduled_stops):
            train.active_scheduled_stop = None
            train.commanded_stop = False
            return False

        stop = train.scheduled_stops[train.next_scheduled_stop_idx]
        station_idx = self._station_index_for_stop(stop)
        distance_to_stop = stop["pos_m"] - train.pos
        if self._train_still_holding_previous_station_line(train, station_idx):
            train.active_scheduled_stop = None
            train.commanded_stop = False
            if train.station_state not in {"READY_TO_DEPART", "DEPARTING"}:
                self._set_train_station_state(train, train.last_station_idx, "DEPARTING", "between_stations")
            return False
        train.active_scheduled_stop = stop
        train.commanded_stop = 0.0 <= distance_to_stop <= STOP_TARGET_MIN_ACTIVATION_M
        if train.station_lane is None and distance_to_stop <= STATION_ROUTE_APPROACH_M:
            self._set_train_station_state(train, station_idx, "APPROACHING_STATION", "scheduled_stop_approach")
        elif train.station_lane is not None and distance_to_stop > 25.0:
            self._set_train_station_state(train, station_idx, "ROUTE_ASSIGNED", "route_assigned")
        elif train.station_lane is not None and distance_to_stop <= 25.0:
            self._set_train_station_state(train, station_idx, "DOCKING", "final_approach")

        aligned_to_scheduled_stop = abs(train.pos - float(stop["pos_m"])) <= STOP_ACCURACY_TOL_M
        if (
            train.commanded_stop
            and train.zero_speed_detected
            and train.door_authorized
            and train.station_lane is not None
            and aligned_to_scheduled_stop
        ):
            # Dwell should start from the same aligned-stop condition that enables
            # doors, otherwise the train can remain held short of the platform or
            # creep past the stop marker before the schedule advances.
            train.pos = train.stop_target_pos
            train.speed = 0.0
            train.prev_accel = 0.0
            train.reset_non_emergency_stop_latches()
            train.standstill_required = True
            train.standstill_anchor_pos = train.pos
            self._set_train_station_state(train, station_idx, "STOPPED_AT_PLATFORM", "aligned_stop")
            train.dwell_remaining_s = self._station_dwell_time_s(station_idx, stop, train)
            self._record_station_arrival(station_idx, train, train.dwell_remaining_s)
            train.next_scheduled_stop_idx += 1
            self._set_train_station_state(train, station_idx, "DWELLING", "dwell_started")
            immediate_packet_required = True
        return False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def _timetable_regulated_speed_cap_kmh(self, train: Train, base_cap_kmh: float) -> Tuple[float, str]:
        if self.headway_manager.mode != "timetable" or train.active_scheduled_stop is None:
            return base_cap_kmh, ""
        station_idx = self._station_index_for_stop(train.active_scheduled_stop)
        if station_idx is None:
            return base_cap_kmh, ""
        record = self._schedule_record_for_train_station(train, train.active_scheduled_stop, station_idx)
        if record is None or record.get("arrival_time_s") is None:
            return base_cap_kmh, ""
        distance_m = max(0.0, float(train.active_scheduled_stop["pos_m"]) - train.pos)
        if distance_m <= STOP_ACCURACY_TOL_M:
            return base_cap_kmh, ""
        remaining_s = float(record["arrival_time_s"]) - self._timetable_operational_time_s()
        profile = str(getattr(train, "schedule_profile", "") or record.get("profile", "") or "").lower()
        if remaining_s <= 0.0:
            return base_cap_kmh, "TIMETABLE_LATE_FAST"
        required_kmh = distance_m / max(remaining_s, 1.0) * 3.6
        if required_kmh >= base_cap_kmh * 0.70:
            return base_cap_kmh, "TIMETABLE_RECOVER"
        slack_ratio = max(0.0, min(1.0, 1.0 - required_kmh / max(base_cap_kmh, 1.0)))
        min_slack_ratio = 0.40 if profile == "eco" else 0.30
        early_time_buffer_s = max(20.0, distance_m / max(kmh_to_ms(base_cap_kmh), 0.1) * 0.35)
        if slack_ratio < min_slack_ratio or remaining_s < early_time_buffer_s:
            return base_cap_kmh, "TIMETABLE_ON_TIME"
        eco_cap = 35.0 if profile == "eco" else base_cap_kmh
        regulated = max(20.0, min(base_cap_kmh, eco_cap, required_kmh * 1.25 + 6.0))
        return regulated, "TIMETABLE_EARLY_COAST"

    def _dispatch_safe_packets(self, with_delay: bool):
        self._update_parallel_protection_zones()
        self._update_source_headway_releases()
        stop_eoa_map: Dict[str, float] = {}
        for train in self.trains:
            if train.active_scheduled_stop is not None and self._train_has_station_stop_eoa_authority(train):
                stop_eoa_map[train.id] = self._scheduled_stop_eoa(float(train.active_scheduled_stop["pos_m"]))
            elif train.active_scheduled_stop is not None and train.station_lane is None:
                station_idx = self._station_index_for_stop(train.active_scheduled_stop)
                if station_idx is not None:
                    self.update_station_occupancy(station_idx)
                    can_accept, reject_reason = self.can_accept_train(station_idx, train)
                    station_start, _station_end = self._station_bounds(self.scheduled_stops[station_idx])
                    if not can_accept and train.pos < station_start:
                        hold_eoa = self._station_holding_eoa(station_idx)
                        stop_eoa_map[train.id] = hold_eoa
                        train.station_reject_reason = reject_reason
                        self.log_station_event(
                            station_idx,
                            train,
                            None,
                            reject_reason,
                            old_EOA=train.last_dispatched_eoa,
                            new_EOA=hold_eoa,
                            EOA_update_reason="STATION_FULL",
                            reject_reason=reject_reason,
                        )
                    elif can_accept:
                        train.station_reject_reason = ""
        safe_packets = self.zc.build_safe_packets(
            self.track_profile,
            self.tsr_zones,
            self.track_end_m,
            stop_eoa_map,
        )
        departure_holds = self._parallel_departure_holds()
        for train in self.trains:
            train.dcs_muted = any(
                window["start_s"] <= self.sim_time_s <= window["end_s"]
                for window in train.dcs_mute_windows
            )
            if train.dcs_muted:
                continue
            delay_s = random.uniform(DCS_DELAY_MIN_S, DCS_DELAY_MAX_S) if with_delay else 0.0
            packet = safe_packets[train.id]
            regulated_cap_kmh, regulation_reason = self._timetable_regulated_speed_cap_kmh(train, packet.tsr_kmh)
            if regulation_reason and regulated_cap_kmh < packet.tsr_kmh - 1e-6:
                variants = dict(packet.variants)
                variants["schedule_regulation"] = regulation_reason
                variants["schedule_speed_cap_kmh"] = regulated_cap_kmh
                packet = SafeMovementPacket(
                    eoa_m=packet.eoa_m,
                    tsr_kmh=regulated_cap_kmh,
                    variants=variants,
                    issued_time_s=packet.issued_time_s,
                )
            if train.departure_hold or train.id in departure_holds or self._needs_station_departure_authority_hold(train, packet):
                hold_eoa = departure_holds.get(train.id)
                terminal_station_hold = (
                    train.station_lane is not None
                    and train.last_station_idx is not None
                    and 0 <= train.last_station_idx < len(self.scheduled_stops)
                    and train.next_scheduled_stop_idx >= len(train.scheduled_stops)
                    and self._station_bounds(self.scheduled_stops[train.last_station_idx])[1] >= self.track_end_m - STOP_ACCURACY_TOL_M
                )
                if hold_eoa is None:
                    fixed_block_station_hold = (
                        self.block_mode == "fixed_block"
                        and train.station_lane is not None
                        and train.last_station_idx is not None
                        and 0 <= train.last_station_idx < len(self.scheduled_stops)
                        and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
                    )
                    if fixed_block_station_hold:
                        _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
                        hold_eoa = station_end - STOP_SVL_OFFSET_M
                    else:
                        hold_margin = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
                        hold_eoa = train.reported_pos + hold_margin + STOP_SVL_OFFSET_M + PARALLEL_RELEASE_MARGIN_M
                    if (
                        not fixed_block_station_hold
                        and
                        train.station_lane is not None
                        and train.last_station_idx is not None
                        and 0 <= train.last_station_idx < len(self.scheduled_stops)
                        and train.station_state in {"READY_TO_DEPART", "DEPARTING"}
                    ):
                        _station_start, station_end = self._station_bounds(self.scheduled_stops[train.last_station_idx])
                        tail_clear_pos = station_end + PARALLEL_RELEASE_MARGIN_M + train.length
                        hold_eoa = max(hold_eoa, tail_clear_pos + STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                packet = SafeMovementPacket(
                    eoa_m=hold_eoa,
                    tsr_kmh=0.0 if train.protection_zone_id == "SOURCE" or terminal_station_hold else packet.tsr_kmh,
                    variants=dict(packet.variants),
                    issued_time_s=packet.issued_time_s,
                )
                train.departure_hold = True
            station_idx = self._station_index_for_stop(train.active_scheduled_stop)
            if station_idx is None and train.station_lane is not None:
                station_idx = train.last_station_idx
            reason = "LEADER_PROTECTION"
            station_stop_eoa = stop_eoa_map.get(train.id)
            if train.trip_mode or train.emergency_stop or train.emg_latch:
                reason = "EMERGENCY"
            elif train.dcs_muted:
                reason = "DCS_TIMEOUT"
            elif train.departure_hold:
                reason = "SAFETY_RESTRICTION"
            elif train.station_reject_reason in {"STATION_FULL", "NO_FREE_PLATFORM", "ALL_LINES_OCCUPIED", "ROUTE_CONFLICT", "STATION_LINE_OCCUPIED"}:
                reason = "SAFETY_RESTRICTION"
            elif self.block_mode == "fixed_block" and station_stop_eoa is None:
                reason = "FIXED_BLOCK"
            elif station_stop_eoa is not None:
                if packet.eoa_m < station_stop_eoa - 1e-6:
                    reason = "FIXED_BLOCK" if self.block_mode == "fixed_block" else "LEADER_PROTECTION"
                elif train.station_state in {"ROUTE_ASSIGNED", "APPROACHING_STATION"}:
                    reason = "ROUTE_ASSIGNED"
                else:
                    reason = "STATION_STOP_TARGET"
            elif train.last_dispatched_eoa_reason in {"ROUTE_ASSIGNED", "STATION_STOP_TARGET"} and train.active_scheduled_stop is not None:
                reason = "ROUTE_INVALIDATED"
            elif train.last_station_state_reason == "dwell_complete" and (
                train.last_dispatched_eoa is None or packet.eoa_m > train.last_dispatched_eoa + 1e-6
            ):
                reason = "DEPARTURE_RELEASE"
            self._log_eoa_update(train, train.last_dispatched_eoa, packet.eoa_m, reason, station_idx)
            packet.issued_time_s = self.sim_time_s
            train.receive_safe_packet(packet, self.sim_time_s + delay_s)

    def _update_analytics(self, include_station_metrics: bool = False):
        headway_snapshot = self.headway_manager.snapshot()
        self.analytics.update(headway_snapshot)
        if headway_snapshot.get("actual_headways_s"):
            self.analytics["min_headway_s"] = headway_snapshot.get("min_actual_headway_s")
            avg_headway = headway_snapshot.get("avg_actual_headway_s")
            self.analytics["trains_per_hour"] = 3600.0 / avg_headway if avg_headway else 0.0
        release_times = headway_snapshot.get("release_times_s", {})
        if release_times:
            self.analytics["current_open_headway_s"] = max(0.0, self.sim_time_s - max(release_times.values()))
        self.analytics["traction_work_kwh"] = sum(t.analytics_traction_work_j for t in self.trains) / 3_600_000.0
        self.analytics["brake_work_kwh"] = sum(t.analytics_brake_work_j for t in self.trains) / 3_600_000.0
        station_metric_indices = sorted(
            set(self.station_arrival_headway_actual_s)
            | set(self.station_headway_actual_s)
            | set(self.analytics.get("station_arrivals", {}))
        )
        station_metrics = []
        for idx in station_metric_indices:
            arrival_headways = list(self.station_arrival_headway_actual_s.get(idx, []))
            departure_headways = list(self.station_headway_actual_s.get(idx, []))
            arrivals = [dict(record) for record in self.analytics.get("station_arrivals", {}).get(idx, [])]
            avg_arrival_headway = (
                sum(arrival_headways) / len(arrival_headways)
                if arrival_headways
                else None
            )
            avg_departure_headway = (
                sum(departure_headways) / len(departure_headways)
                if departure_headways
                else None
            )
            for record in arrivals:
                record["avg_station_arrival_headway_s"] = avg_arrival_headway
            station_metrics.append(
                {
                    "station_index": idx,
                    "station_name": self.scheduled_stops[idx].get("name", f"STATION_{idx}")
                    if idx < len(self.scheduled_stops)
                    else f"STATION_{idx}",
                    "arrival_headways_s": arrival_headways,
                    "departure_headways_s": departure_headways,
                    "avg_arrival_headway_s": avg_arrival_headway,
                    "avg_departure_headway_s": avg_departure_headway,
                    "headway_deviation_s": list(self.station_headway_deviation_s.get(idx, [])),
                    "arrivals": arrivals,
                }
            )
        self.analytics["station_passenger_metrics"] = station_metrics
        ordered_for_collision = sorted(self.trains, key=lambda item: item.pos)
        active_collisions = 0
        for left, right in zip(ordered_for_collision, ordered_for_collision[1:]):
            if left.source_lane is not None and right.source_lane is not None and left.source_lane != right.source_lane:
                continue
            if (
                left.protection_zone_id == right.protection_zone_id
                and left.protection_lane != right.protection_lane
            ):
                continue
            overlap = left.pos - right.safe_rear_end_pos()
            if overlap > 0.0:
                active_collisions += 1
                event = {
                    "time_s": self.sim_time_s,
                    "front_train_id": right.id,
                    "following_train_id": left.id,
                    "overlap_m": overlap,
                }
                if not left.collision_latched or not right.collision_latched:
                    self.analytics["collision_count"] = self.analytics.get("collision_count", 0) + 1
                    self.analytics.setdefault("collision_events", []).append(event)
                for train, partner in ((left, right), (right, left)):
                    train.collision_latched = True
                    train.collision_partner_id = partner.id
                    train.collision_overlap_m = overlap
                    train.enter_trip_mode("COLLISION", train.reported_pos)
                    train.atp_state = "ATP_TRIP"
                    train.atp_alert = "COLLISION"
                    train.atp_action = "EBI"
                    train.atp_brake = "EMERGENCY"
                    train.emg_latch = True
        self.analytics["active_collision_count"] = active_collisions
        for train in self.trains:
            if train.headway_time_s is not None and train.headway_time_s > 0.0:
                current_min = self.analytics["min_headway_s"]
                if current_min is None or train.headway_time_s < current_min:
                    self.analytics["min_headway_s"] = train.headway_time_s

            journey_times = self.analytics["journey_times"]
            if train.id not in journey_times and train.pos >= self.track_end_m:
                journey_times[train.id] = self.sim_time_s

            previous_action = self.analytics["last_actions"].get(train.id, "")
            if train.atp_action == "EBI" and previous_action != "EBI":
                self.analytics["ebi_count"] += 1
            if train.atp_action == "SBI" and previous_action != "SBI":
                self.analytics["sbi_count"] += 1
            self.analytics["last_actions"][train.id] = train.atp_action

    def step(self):
        self._tick_parallel_release_locks()
        self.train_generation_changed = self._spawn_source_trains()
        immediate_packet_required = False
        for t in self.trains:
            t.update_reported_position()
            immediate_packet_required = self._update_train_stop_schedule(t) or immediate_packet_required
        immediate_packet_required = self._update_station_routes() or immediate_packet_required
        self._check_station_invariants()
        self._dispatch_safe_packets(with_delay=not immediate_packet_required)
        ordered = sorted(self.trains, key=lambda t: t.pos, reverse=True)
        for i, t in enumerate(ordered):
            front = min(
                (
                    candidate for candidate in ordered
                    if candidate.pos > t.pos
                    and (
                        t.protection_zone_id is None
                        or candidate.protection_zone_id is None
                        or t.protection_zone_id != candidate.protection_zone_id
                        or t.protection_lane == candidate.protection_lane
                    )
                ),
                key=lambda candidate: candidate.pos,
                default=None,
            )
            if front is None:
                t.headway_time_s = None
                continue
            gap = (front.pos - front.length) - t.pos
            t.headway_time_s = gap / max(t.speed, 0.1)
        for t in self.trains:
            t.step(self.sim_time_s)
            if (
                getattr(t, "headway_dispatch_released", False)
                and not getattr(t, "headway_actual_dispatched", False)
                and not self._source_zone_contains(t)
            ):
                self.headway_manager.mark_actual_dispatch(t.id, self.sim_time_s)
                t.headway_actual_dispatched = True
            if t.active_scheduled_stop is None and t.station_lane is not None and t.speed > STANDSTILL_SPEED_EPS:
                self._set_train_station_state(t, self._station_index_for_stop(self._station_overlapped_by_train(t)[1]) if self._station_overlapped_by_train(t) is not None else t.last_station_idx, "DEPARTING", "departing_from_station")
        self._enforce_station_cd_routes()
        self._update_analytics()
        self.sim_time_s += DT


class TrainPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, train_id: str, on_toggle, on_resume, on_instant_stop, on_precise_jog, color: str, scale_factor: float):
        super().__init__(master, style="Card.TFrame")
        self.train_id = train_id
        self.on_toggle = on_toggle
        self.on_resume = on_resume
        self.on_instant_stop = on_instant_stop
        self.on_precise_jog = on_precise_jog
        self.color = color
        self.scale_factor = scale_factor
        self.track_max_m = 2000.0
        self.emg_state = 0
        self.history_len = 180
        self.hist_actual = deque(maxlen=self.history_len)
        self.hist_curves = {
            "P": deque(maxlen=self.history_len),
            "I": deque(maxlen=self.history_len),
            "W": deque(maxlen=self.history_len),
            "SBI": deque(maxlen=self.history_len),
            "SBD": deque(maxlen=self.history_len),
            "EBI": deque(maxlen=self.history_len),
            "EBD": deque(maxlen=self.history_len),
        }
        self.curve_vars = {
            "actual": tk.StringVar(value="0.0 km/h"),
            "P": tk.StringVar(value="0.0 km/h"),
            "I": tk.StringVar(value="0.0 km/h"),
            "W": tk.StringVar(value="0.0 km/h"),
            "SBI": tk.StringVar(value="0.0 km/h"),
            "SBD": tk.StringVar(value="0.0 km/h"),
            "EBI": tk.StringVar(value="0.0 km/h"),
            "EBD": tk.StringVar(value="0.0 km/h"),
        }
        self.current_release_speed = None

        # Use direct frame container instead of internal scrolling so train panels show full content.
        self.config(width=600, height=420)
        self.pack_propagate(False)
        self.grid_propagate(False)
        container = self
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=0)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(3, weight=0)
        self.rowconfigure(4, weight=0)
        self.rowconfigure(5, weight=0)
        container.columnconfigure(0, weight=0, minsize=int(180 * scale_factor))
        container.columnconfigure(1, weight=1)

        # Header
        header = ttk.Frame(container, style="Card.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(int(10 * scale_factor), 0))
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text=f"Train {train_id}",
            style="CardTitle.TLabel",
            foreground=self.color,
        ).grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value="A")
        self.mode_label = ttk.Label(
            header,
            textvariable=self.mode_var,
            font=("Consolas", int(12 * scale_factor), "bold"),
            foreground=APP_THEME["accent"],
        )
        self.mode_label.grid(row=0, column=1)
        self.badge_var = tk.StringVar(value="OK")
        self.badge_label = tk.Label(
            header,
            textvariable=self.badge_var,
            bg=APP_THEME["ok"],
            fg="#243018",
            font=("Consolas", int(8 * scale_factor), "bold"),
            padx=int(8 * scale_factor),
            pady=int(3 * scale_factor),
        )
        self.badge_label.grid(row=0, column=2, sticky="e")

        # Main content: target/operations on the left, speed curves on the right.
        target_frame = ttk.Frame(container, style="Card.TFrame")
        target_frame.grid(row=1, column=0, sticky="new", padx=(int(10 * scale_factor), int(5 * scale_factor)), pady=(int(6 * scale_factor), int(4 * scale_factor)))
        ttk.Label(target_frame, text="Target Distance", style="CardTitle.TLabel").pack()
        self.target_distance_var = tk.StringVar(value="0 m")
        ttk.Label(target_frame, textvariable=self.target_distance_var, font=("Consolas", int(16 * scale_factor), "bold")).pack()
        self.target_speed_var = tk.StringVar(value="0 km/h")
        ttk.Label(target_frame, textvariable=self.target_speed_var, font=("Consolas", int(12 * scale_factor))).pack()

        # Operational Info under target distance.
        op_frame = ttk.Frame(target_frame, style="Card.TFrame")
        op_frame.pack(fill="x", pady=(int(8 * scale_factor), 0))
        ttk.Label(op_frame, text="Operational Status", style="CardTitle.TLabel").pack()
        self.door_var = tk.StringVar(value="Closed")
        ttk.Label(op_frame, textvariable=self.door_var, font=("Consolas", int(10 * scale_factor))).pack()
        self.docking_var = tk.StringVar(value="Not Docked")
        ttk.Label(op_frame, textvariable=self.docking_var, font=("Consolas", int(10 * scale_factor))).pack()
        self.headway_var = tk.StringVar(value="-- s")
        ttk.Label(op_frame, text="Headway:", font=("Consolas", int(9 * scale_factor))).pack()
        ttk.Label(op_frame, textvariable=self.headway_var, font=("Consolas", int(10 * scale_factor), "bold")).pack()

        # Curves Table (Right)
        curves_frame = ttk.Frame(container, style="Card.TFrame")
        curves_frame.grid(row=1, column=1, sticky="new", padx=(int(5 * scale_factor), int(10 * scale_factor)), pady=(int(6 * scale_factor), int(4 * scale_factor)))
        curves_frame.columnconfigure(0, weight=0)
        curves_frame.columnconfigure(1, weight=1)
        ttk.Label(curves_frame, text="Speed Curves", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, int(6 * scale_factor)))
        
        # Create variables for curves
        self.curve_vars = {
            "Actual": tk.StringVar(value="0.0 km/h"),
            "Permitted": tk.StringVar(value="0.0 km/h"),
            "Warning": tk.StringVar(value="0.0 km/h"),
            "SBI": tk.StringVar(value="0.0 km/h"),
            "EBI": tk.StringVar(value="0.0 km/h"),
        }
        
        for row, (curve_name, var) in enumerate(self.curve_vars.items(), start=1):
            ttk.Label(curves_frame, text=f"{curve_name}:", font=("Consolas", int(10 * scale_factor))).grid(row=row, column=0, sticky="w", pady=1)
            ttk.Label(curves_frame, textvariable=var, font=("Consolas", int(11 * scale_factor), "bold")).grid(row=row, column=1, sticky="w", padx=(int(8 * scale_factor), 0), pady=1)

        # Braking Curves Chart
        chart_frame = ttk.Frame(container, style="Card.TFrame")
        chart_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=int(10 * scale_factor), pady=(0, int(4 * scale_factor)))
        ttk.Label(chart_frame, text="Braking Curves & Speed", style="CardTitle.TLabel").pack(pady=(0, int(5 * scale_factor)))
        self.chart_canvas = tk.Canvas(
            chart_frame,
            width=int(560 * scale_factor),
            height=int(150 * scale_factor),
            background=APP_THEME["card_alt"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.chart_canvas.pack(fill="both", expand=True)

        # Message Area
        self.message_var = tk.StringVar(value="")
        message_label = ttk.Label(
            container,
            textvariable=self.message_var,
            font=("Consolas", int(10 * scale_factor)),
            foreground="#6b2e35",
            background="#ffd6c9",
            padding=int(5 * scale_factor),
        )
        self.release_var = tk.StringVar(value="")
        self.release_label = ttk.Label(
            container,
            textvariable=self.release_var,
            font=("Consolas", int(10 * scale_factor), "bold"),
            foreground="#7a3d1c",
            background="#ffe8b5",
            padding=int(8 * scale_factor),
        )
        self.release_label.grid(row=3, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(0, int(10 * scale_factor)))
        self.release_label.grid_remove()

        message_label.grid(row=4, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(0, int(10 * scale_factor)))

        # Jogging Status (hidden by default)
        self.jog_var = tk.StringVar(value="")
        self.jog_label = ttk.Label(
            container,
            textvariable=self.jog_var,
            font=("Consolas", int(8 * scale_factor), "bold"),
            foreground=APP_THEME["accent"],
            background=APP_THEME["card_alt"],
            padding=int(3 * scale_factor),
        )
        self.jog_label.grid_remove()

        # Buttons
        btn_row = ttk.Frame(container, style="Card.TFrame")
        btn_row.grid(row=5, column=0, columnspan=2, sticky="ew", padx=int(10 * scale_factor), pady=(int(8 * scale_factor), int(10 * scale_factor)))
        btn_row.columnconfigure(0, weight=1)
        btn_row.columnconfigure(1, weight=1)
        self.toggle_btn = ttk.Button(btn_row, text="Emergency Stop", command=self._toggle, style="Accent.TButton")
        self.toggle_btn.grid(row=0, column=0, sticky="ew", padx=(0, int(4 * scale_factor)))
        self.more_btn = ttk.Button(btn_row, text="More Details", command=self._show_details)
        self.more_btn.grid(row=0, column=1, sticky="ew", padx=(int(4 * scale_factor), 0))

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def _bind_detail_scroll(self, window, canvas, scrollable_frame, window_id):
        def sync_scrollregion(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def fit_frame_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        def on_mousewheel(event):
            if getattr(event, "num", None) == 4:
                delta = -1
            elif getattr(event, "num", None) == 5:
                delta = 1
            else:
                delta = int(-1 * (event.delta / 120)) if getattr(event, "delta", 0) else 0
            if delta:
                canvas.yview_scroll(delta, "units")

        scrollable_frame.bind("<Configure>", sync_scrollregion)
        canvas.bind("<Configure>", fit_frame_width)

        for widget in (window, canvas, scrollable_frame):
            widget.bind("<MouseWheel>", on_mousewheel, add="+")
            widget.bind("<Button-4>", on_mousewheel, add="+")
            widget.bind("<Button-5>", on_mousewheel, add="+")

    def _show_details(self):
        # Create a dialog window for detailed information
        details_window = tk.Toplevel(self)
        details_window.title(f"Train {self.train_id} - Detailed Information")
        details_window.geometry("312x647")
        details_window.minsize(312, 647)
        details_window.resizable(True, True)
        
        # Bind close event to cleanup
        details_window.protocol("WM_DELETE_WINDOW", lambda: self._cleanup_details(details_window))
        
        # Create scrollable frame for details
        canvas = tk.Canvas(details_window, highlightthickness=0)
        scrollbar = tk.Scrollbar(
            details_window,
            orient="vertical",
            command=canvas.yview,
            width=16,
            cursor="hand2",
            activebackground="#9db6d8",
        )
        scrollable_frame = ttk.Frame(canvas)

        detail_window_id = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        self._bind_detail_scroll(details_window, canvas, scrollable_frame, detail_window_id)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Add detailed information
        ttk.Label(scrollable_frame, text="Detailed Train Information", style="CardTitle.TLabel").pack(pady=10)
        
        # Metrics
        metrics_frame = ttk.LabelFrame(scrollable_frame, text="Speed Metrics")
        metrics_frame.pack(fill="x", padx=10, pady=5)
        self.detail_metric_vars = {
            "actual": tk.StringVar(value="0.0"),
            "permitted": tk.StringVar(value="0.0"),
            "warning": tk.StringVar(value="0.0"),
            "intervention": tk.StringVar(value="0.0"),
        }
        for key, var in self.detail_metric_vars.items():
            ttk.Label(metrics_frame, text=f"{key.capitalize()}: {var.get()} km/h").pack(anchor="w")
        
        # Target & Planning
        target_frame = ttk.LabelFrame(scrollable_frame, text="Target & Planning")
        target_frame.pack(fill="x", padx=10, pady=5)
        self.detail_target_var = tk.StringVar(value="")
        self.detail_plan_var = tk.StringVar(value="")
        ttk.Label(target_frame, textvariable=self.detail_target_var, justify="left").pack(anchor="w")
        ttk.Separator(target_frame, orient="horizontal").pack(fill="x", pady=5)
        ttk.Label(target_frame, textvariable=self.detail_plan_var, justify="left").pack(anchor="w")
        
        # Operational Status
        status_frame = ttk.LabelFrame(scrollable_frame, text="Operational Status")
        status_frame.pack(fill="x", padx=10, pady=5)
        self.detail_state_var = tk.StringVar(value="")
        self.detail_alarm_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.detail_state_var, justify="left").pack(anchor="w")
        ttk.Separator(status_frame, orient="horizontal").pack(fill="x", pady=5)
        ttk.Label(status_frame, textvariable=self.detail_alarm_var, justify="left").pack(anchor="w")
        
        # Update the details with current data
        if hasattr(self, 'last_train'):
            self._update_details_window(self.last_train)

    def _cleanup_details(self, window):
        # Clean up attributes when details window is closed
        if hasattr(self, 'detail_metric_vars'):
            delattr(self, 'detail_metric_vars')
        if hasattr(self, 'detail_target_var'):
            delattr(self, 'detail_target_var')
        if hasattr(self, 'detail_plan_var'):
            delattr(self, 'detail_plan_var')
        if hasattr(self, 'detail_state_var'):
            delattr(self, 'detail_state_var')
        if hasattr(self, 'detail_alarm_var'):
            delattr(self, 'detail_alarm_var')
        window.destroy()

    def _update_details_window(self, train: Train):
        actual_kmh = ms_to_kmh(train.speed)
        permitted_kmh = min(ms_to_kmh(train.curves["P"]), train.psr_kmh) if train.curves["P"] > 0.0 else train.psr_kmh
        warning_kmh = ms_to_kmh(train.curves["W"])
        sbi_kmh = ms_to_kmh(train.hidden_curves["SBI"])
        ebi_kmh = ms_to_kmh(train.hidden_curves["EBI"])
        intervention_kmh = ebi_kmh
        current_uncertainty_m = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
        target_distance_m = train.distance_to_eoa if train.constraint_type == "STOP" else train.distance_to_constraint_m
        target_speed_kmh = 0.0 if train.constraint_type == "STOP" else train.constraint_target_speed_kmh
        current_tsr_kmh = min(train.psr_kmh, train.limit_ahead_speed_kmh) if train.limit_ahead_dist <= 0.0 else train.psr_kmh

        self.detail_metric_vars["actual"].set(f"{actual_kmh:.1f}")
        self.detail_metric_vars["permitted"].set(f"{permitted_kmh:.1f}")
        self.detail_metric_vars["warning"].set(f"{warning_kmh:.1f}")
        self.detail_metric_vars["intervention"].set(f"{intervention_kmh:.1f}")

        self.detail_target_var.set(
            "\n".join(
                [
                    f"EOA / SvL      : {train.eoa:,.1f} m  /  {train.stop_target_pos:,.1f} m",
                    f"Distance target: {target_distance_m:,.1f} m",
                    f"Target speed   : {target_speed_kmh:.1f} km/h",
                    f"Constraint     : {train.constraint_type}  curve={train.curve_mode}",
                    f"ATO target     : {ms_to_kmh(train.ato_target_speed):.1f} km/h",
                    f"Door stop state: {train.precise_stop_state}",
                ]
            )
        )
        self.detail_plan_var.set(
            "\n".join(
                [
                    f"Current SSP    : {train.psr_kmh:.1f} km/h",
                    f"Next SSP / TSR : {train.limit_ahead_speed_kmh:.1f} km/h @ {train.limit_ahead_dist:,.0f} m",
                    f"Gradient       : {train.gradient:+.3f}  ({train.gradient * 1000.0:+.1f} permille)",
                    f"Balise / Beacon: last {train.last_balise_pos:,.0f} m  next {train.next_balise_pos:,.0f} m",
                    f"Odo CI         : +/-{current_uncertainty_m:.1f} m  rep={train.reported_pos:,.1f} m",
                    f"TSR active     : {current_tsr_kmh:.1f} km/h  release={'ON' if train.release_active else 'OFF'}",
                ]
            )
        )
        integrity_text = "Confirmed" if train.safe_packet_valid and not train.trip_mode else "Degraded"
        self.detail_state_var.set(
            "\n".join(
                [
                    f"Drive mode     : {train.drive_mode}  ({train.ato_state})  /  {train.atp_state}",
                    f"Mode source    : requested={train.requested_drive_mode}  {'AUTO DEGRADED' if train.dcs_degraded_requested else train.mode_transition_reason or 'NORMAL'}",
                    f"Brake channel  : {train.atp_brake}  cutoff={'YES' if train.traction_cutoff else 'NO'}",
                    f"Door interlock : {train.ato_door_mode}  authorized={'YES' if train.door_authorized else 'NO'}",
                    f"Hold brake     : {'ACTIVE' if train.ato_hold_active else 'FREE'}",
                    f"Station state  : {train.station_state}  line={train.station_lane if train.station_lane is not None else '--'}",
                    f"Train integrity: {integrity_text}",
                    f"Zero speed mon : {'ON' if train.zero_speed_detected else 'OFF'}  standstill={'ON' if train.standstill_monitoring else 'OFF'}",
                    f"DCS / ZC link   : {'VALID' if train.safe_packet_valid else 'TIMEOUT'}  age={train.safe_packet_age_s:.1f}s",
                ]
            )
        )
        self.detail_alarm_var.set(
            "\n".join(
                [
                    f"Alert          : {train.atp_alert}",
                    f"ATO prepare    : {'YES' if train.ato_brake_prepare else 'NO'}  jog={train.jog_state}",
                    f"SBI / EBI      : {'ARMED' if train.service_brake_latch else 'IDLE'}  /  {'ARMED' if train.emg_latch else 'IDLE'}",
                    f"DWELL          : {train.dwell_remaining_s:.1f}s",
                    f"Beacon align   : {'LOCKED' if train.beacon_position_locked else 'TRACK'}  seen={'YES' if train.stop_beacon_seen else 'NO'}",
                    f"Headway        : {'--' if train.headway_time_s is None else f'{train.headway_time_s:,.1f}s'}",
                    f"Scheduled stop : {'--' if train.active_scheduled_stop is None else train.active_scheduled_stop['name']}",
                ]
            )
        )

    def update_from_train(self, train: Train, append_history: bool = True):
        self.last_train = train
        actual_kmh = ms_to_kmh(train.speed)
        table_curves = getattr(train, "raw_curves", train.curves)
        table_hidden_curves = getattr(train, "raw_hidden_curves", train.hidden_curves)
        permitted_ms = table_curves.get("P", train.curves["P"])
        permitted_kmh = min(ms_to_kmh(permitted_ms), train.psr_kmh) if permitted_ms > 0.0 else train.psr_kmh
        warning_kmh = ms_to_kmh(table_curves.get("W", train.curves["W"]))
        sbi_kmh = ms_to_kmh(table_hidden_curves.get("SBI", train.hidden_curves["SBI"]))
        ebi_kmh = ms_to_kmh(table_hidden_curves.get("EBI", train.hidden_curves["EBI"]))
        current_uncertainty_m = max(train.effective_position_uncertainty_m(), abs(train.pos_error_m))
        target_distance_m = train.distance_to_eoa if train.constraint_type == "STOP" else train.distance_to_constraint_m
        target_speed_kmh = 0.0 if train.constraint_type == "STOP" else train.constraint_target_speed_kmh
        current_tsr_kmh = min(train.psr_kmh, train.limit_ahead_speed_kmh) if train.limit_ahead_dist <= 0.0 else train.psr_kmh

        # Update target distance and speed
        self.target_distance_var.set(f"{target_distance_m:,.0f} m")
        self.target_speed_var.set(f"{target_speed_kmh:.1f} km/h")

        # Update mode and status
        if train.drive_mode == "ATO" and train.atp_state not in ("ATP_EMERGENCY", "ATP_TRIP"):
            self.mode_var.set("A")
            self.mode_label.configure(foreground="#1f77b4")
        elif train.drive_mode == "CMD25":
            self.mode_var.set("25")
            self.mode_label.configure(foreground="#d19c1d")
        else:
            self.mode_var.set("M")
            self.mode_label.configure(foreground="#ffc107")

        # Update badge
        alert_text = f"ATP {train.atp_alert}"
        badge_bg = "#d7f7dc"
        badge_fg = "#16351f"
        if train.atp_action == "OFF":
            badge_bg = "#ffefc4"
            badge_fg = "#704d00"
        elif train.atp_action == "WARN":
            badge_bg = "#ffd9d2"
            badge_fg = "#7f2319"
        elif train.atp_action == "SBI":
            badge_bg = "#d7f5de"
            badge_fg = "#12552a"
        elif train.atp_action == "EBI":
            badge_bg = "#ffd0d0"
            badge_fg = "#7f0d0d"
        self.badge_var.set(alert_text)
        self.badge_label.configure(bg=badge_bg, fg=badge_fg)

        # Update door status
        if train.door_authorized or train.ato_door_mode == "ENABLE":
            door_text = "Enabled"
        elif train.ato_door_mode == "READY":
            door_text = "Ready"
        else:
            door_text = "Closed"
        self.door_var.set(f"Door: {door_text}")

        # Update docking status
        if train.door_authorized:
            docking_text = "Door Ready"
        elif train.precise_stop_state == "JOG":
            docking_text = "Nhich"
        elif train.precise_stop_state == "JOG_FAILED":
            docking_text = "Jog Locked"
        elif train.precise_stop_state == "ALIGNED":
            docking_text = "Aligned"
        elif train.precise_stop_state == "WAIT_JOG":
            docking_text = "Need Nhich"
        else:
            docking_text = "Not Docked"
        self.docking_var.set(docking_text)

        # Update headway
        headway_text = f"{train.headway_time_s:.1f} s" if train.headway_time_s is not None else "--"
        self.headway_var.set(headway_text)

        # Update message area
        message = ""
        if train.atp_action in ["WARN", "SBI", "EBI"]:
            message = f"ATP {train.atp_action}: {train.atp_alert}"
        elif train.dwell_remaining_s > 0.0:
            message = f"DWELL: {train.dwell_remaining_s:.1f}s"
        self.message_var.set(message)

        # Hide release speed indicator
        self.release_label.grid_remove()

        # Jogging status remains an internal flag; keep the small label hidden for the new layout
        self.jog_label.grid_remove()

        # Update curves table
        self.curve_vars["Actual"].set(f"{actual_kmh:.1f} km/h")
        self.curve_vars["Permitted"].set(f"{permitted_kmh:.1f} km/h")
        self.curve_vars["Warning"].set(f"{warning_kmh:.1f} km/h")
        self.curve_vars["SBI"].set(f"{sbi_kmh:.1f} km/h")
        self.curve_vars["EBI"].set(f"{ebi_kmh:.1f} km/h")

        # Keep charts frozen while simulation time is paused.
        if append_history:
            self._push_history(train)
        self._draw_chart()
        if hasattr(self, 'detail_metric_vars'):
            self._update_details_window(train)

        # Update emergency button state
        if train.emergency_recovery_hold:
            self.emg_state = 2
        elif train.emergency_stop or train.emg_latch or train.trip_mode:
            self.emg_state = max(self.emg_state, 1)
        elif self.emg_state != 2:
            self.emg_state = 0
        if self.emg_state == 0:
            text = "Emergency Stop"
        elif self.emg_state == 1:
            text = "Safe Confirmed"
        elif self.emg_state == 2:
            text = "Resume Train"
        self.toggle_btn.config(text=text)

    def _push_history(self, train: Train):
        chart_curves = getattr(train, "raw_curves", train.curves)
        chart_hidden_curves = getattr(train, "raw_hidden_curves", train.hidden_curves)
        self.hist_actual.append(ms_to_kmh(train.speed))
        self.hist_curves["P"].append(ms_to_kmh(chart_curves.get("P", train.curves["P"])))
        self.hist_curves["I"].append(ms_to_kmh(chart_hidden_curves.get("I", train.hidden_curves["I"])))
        self.hist_curves["W"].append(ms_to_kmh(chart_curves.get("W", train.curves["W"])))
        self.hist_curves["SBI"].append(ms_to_kmh(chart_hidden_curves.get("SBI", train.hidden_curves["SBI"])))
        self.hist_curves["SBD"].append(ms_to_kmh(chart_curves.get("SBD", train.curves["SBD"])))
        self.hist_curves["EBI"].append(ms_to_kmh(chart_hidden_curves.get("EBI", train.hidden_curves["EBI"])))
        self.hist_curves["EBD"].append(ms_to_kmh(chart_curves.get("EBD", train.curves["EBD"])))

    def _draw_chart(self):
        canvas = self.chart_canvas
        w = max(320, int(canvas.winfo_width() or canvas["width"]))
        h = max(120, int(canvas.winfo_height() or canvas["height"]))
        canvas.delete("all")
        canvas.create_rectangle(1, 1, w - 1, h - 1, outline=APP_THEME["border"], fill=APP_THEME["card_alt"])

        if len(self.hist_actual) < 2:
            return

        max_v = max(max(self.hist_actual), 1.0)
        for key in self.hist_curves:
            max_v = max(max_v, max(self.hist_curves[key]) if self.hist_curves[key] else 1.0)
        max_v = max(20.0, math.ceil(max_v / 10.0) * 10.0)

        left = 34
        right = w - 10
        top = 62
        bottom = h - 22
        plot_w = max(1.0, right - left)
        plot_h = max(1.0, bottom - top)
        scale_y = plot_h / max_v

        def to_points(values):
            pts = []
            count = max(1, len(values) - 1)
            for i, v in enumerate(values):
                x = left + (i / count) * plot_w
                y = bottom - min(max_v, max(0.0, v)) * scale_y
                pts.extend([x, y])
            return pts

        # Title
        canvas.create_text(16, 18, anchor="w", text="Braking Curves & Speed", fill=APP_THEME["accent"], font=("Consolas", 11, "bold"))

        # Legend
        legend_x = 20
        legend_y = 30
        legend_items = [
            ("Actual", CURVE_COLORS["actual"]),
            ("P", CURVE_COLORS["P"]),
            ("W", CURVE_COLORS["W"]),
            ("I", CURVE_COLORS["I"]),
            ("SBI", ACTION_COLORS["SBI"]),
            ("SBD", CURVE_COLORS["SBD"]),
            ("EBI", ACTION_COLORS["EBI"]),
            ("EBD", CURVE_COLORS["EBD"]),
        ]
        for i, (label, color) in enumerate(legend_items):
            x = legend_x + (i % 4) * 78
            y = legend_y + (i // 4) * 18
            canvas.create_line(x, y, x + 16, y, fill=color, width=2)
            canvas.create_text(x + 20, y, anchor="w", text=label, fill=APP_THEME["text"], font=("Consolas", 8))

        canvas.create_line(left, top, left, bottom, fill=APP_THEME["canvas_grid"])
        canvas.create_line(left, bottom, right, bottom, fill=APP_THEME["canvas_grid"])
        tick_step = 10.0 if max_v <= 80.0 else 20.0
        tick = 0.0
        while tick <= max_v + 1e-6:
            y = bottom - tick * scale_y
            canvas.create_line(left, y, right, y, fill=APP_THEME["canvas_grid"], dash=(1, 3))
            canvas.create_text(left - 4, y, anchor="e", text=f"{tick:.0f}", fill=APP_THEME["muted"], font=("Consolas", 8))
            tick += tick_step
        canvas.create_text(left, top - 8, anchor="w", text="km/h", fill=APP_THEME["muted"], font=("Consolas", 8))

        # Draw curves
        canvas.create_line(*to_points(self.hist_curves["EBD"]), fill=CURVE_COLORS["EBD"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["EBI"]), fill=ACTION_COLORS["EBI"], dash=(4, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["SBD"]), fill=CURVE_COLORS["SBD"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["SBI"]), fill=ACTION_COLORS["SBI"], dash=(4, 2), width=2)
        canvas.create_line(*to_points(self.hist_curves["W"]), fill=CURVE_COLORS["W"], dash=(2, 4), width=2)
        canvas.create_line(*to_points(self.hist_curves["I"]), fill=CURVE_COLORS["I"], dash=(1, 3), width=2)
        canvas.create_line(*to_points(self.hist_curves["P"]), fill=CURVE_COLORS["P"], dash=(2, 2), width=2)
        canvas.create_line(*to_points(self.hist_actual), fill=CURVE_COLORS["actual"], width=3)

    def _toggle(self):
        if self.emg_state == 0:
            self.on_toggle(self.train_id)
        elif self.emg_state == 1:
            self.on_toggle(self.train_id, emergency=True)
        elif self.emg_state == 2:
            self.on_resume(self.train_id)

    def _instant_stop(self):
        self.on_instant_stop(self.train_id)

    def _precise_jog(self):
        self.on_precise_jog(self.train_id)

    def set_track_range(self, track_max_m: float):
        self.track_max_m = track_max_m


class ATSOverviewPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float, on_select=None, on_edit=None):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.on_select = on_select
        self.on_edit = on_edit
        self.selected_element: str | None = None
        self._element_lookup: Dict[str, Tuple[str, int]] = {}
        self._last_sim: Simulation | None = None
        self.view_zoom = 1.0
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self._drag_start: Tuple[int, int] | None = None
        self._drag_origin: Tuple[float, float] = (0.0, 0.0)
        self._dragged = False
        ttk.Label(self, text="ATS Canvas", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(int(2 * scale_factor), int(6 * scale_factor)))
        self.canvas = tk.Canvas(self, height=int(285 * scale_factor), background=APP_THEME["canvas"], highlightthickness=1, highlightbackground=APP_THEME["border"])
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Double-Button-1>", self._on_double_click)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", self._on_mousewheel)
        self.canvas.bind("<Button-5>", self._on_mousewheel)

    def _element_tag(self, kind: str, index: int) -> str:
        key = f"{kind}:{index}"
        self._element_lookup[key] = (kind, index)
        return f"element:{key}"

    def _element_from_event(self, event) -> str | None:
        item = self.canvas.find_closest(event.x, event.y)
        if not item:
            return None
        for tag in self.canvas.gettags(item[0]):
            if tag.startswith("element:"):
                return tag.split("element:", 1)[1]
        return None

    def _redraw_current_view(self):
        if self._last_sim is not None:
            self._draw(self._last_sim)

    def _on_press(self, event):
        self._drag_start = (event.x, event.y)
        self._drag_origin = (self.view_offset_x, self.view_offset_y)
        self._dragged = False
        self.canvas.config(cursor="fleur")

    def _on_drag(self, event):
        if self._drag_start is None:
            return "break"
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        if abs(dx) > 3 or abs(dy) > 3:
            self._dragged = True
        self.view_offset_x = self._drag_origin[0] + dx
        self.view_offset_y = self._drag_origin[1] + dy
        self._redraw_current_view()
        return "break"

    def _on_release(self, event):
        self.canvas.config(cursor="")
        self._drag_start = None
        if self._dragged:
            self._dragged = False
            return "break"
        element_key = self._element_from_event(event)
        if element_key is None:
            return "break"
        self.selected_element = element_key
        if self.on_select is not None:
            self.on_select(element_key)
        self._redraw_current_view()
        return "break"

    def _on_mousewheel(self, event):
        if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0:
            zoom_step = 1.12
        else:
            zoom_step = 1.0 / 1.12
        old_zoom = self.view_zoom
        new_zoom = max(0.45, min(5.0, old_zoom * zoom_step))
        if abs(new_zoom - old_zoom) < 1e-6:
            return "break"
        anchor_x = event.x - 28.0
        self.view_offset_x = anchor_x - (anchor_x - self.view_offset_x) * (new_zoom / old_zoom)
        self.view_zoom = new_zoom
        self._redraw_current_view()
        return "break"

    def _on_double_click(self, event):
        if self._dragged:
            return "break"
        element_key = self._element_from_event(event)
        if element_key is None:
            return "break"
        self.selected_element = element_key
        if self.on_edit is not None:
            kind, index = self._element_lookup.get(element_key, ("", -1))
            self.on_edit(kind, index)
        return "break"

    def update_data(self, sim: Simulation):
        self._last_sim = sim
        occupancy = sim.zc.fixed_block_occupancy() if getattr(sim, "block_mode", "") == "fixed_block" else {}
        occupied_blocks = sum(1 for train_ids in occupancy.values() if train_ids)
        esa_active = sum(1 for t in sim.trains if t.atp_action == "EBI")
        block_mode = getattr(sim, "block_mode", "fixed")
        block_text = (
            f"occupied fixed blocks={occupied_blocks}/{len(getattr(sim, 'fixed_blocks', []))}"
            if block_mode == "fixed_block"
            else "moving-block authority (PSR segments visible)"
        )
        self.summary_var.set(
            f"OCC view  |  trains={len(sim.trains)}  {block_text}  ESA active={esa_active}"
        )
        self._draw(sim)

    def _draw(self, sim: Simulation):
        c = self.canvas
        w = max(320, c.winfo_width())
        h = max(180, c.winfo_height())
        c.delete("all")
        self._element_lookup = {}
        c.create_rectangle(1, 1, w - 1, h - 1, outline=APP_THEME["border"], fill=APP_THEME["canvas"])
        grid_spacing = 24
        for gx in range(16, w - 8, grid_spacing):
            c.create_line(gx, 18, gx, h - 8, fill="#ead9d2", width=1)
        for gy in range(18, h - 8, grid_spacing):
            c.create_line(16, gy, w - 8, gy, fill="#ead9d2", width=1)
        for gx in range(16, w - 8, grid_spacing):
            for gy in range(18, h - 8, grid_spacing):
                c.create_rectangle(gx - 1, gy - 1, gx + 1, gy + 1, fill=APP_THEME["canvas_grid"], outline="")

        def x_from_pos(pos_m: float) -> float:
            span = max(1.0, sim.track_max_m - sim.track_min_m)
            base_x = 28 + (pos_m - sim.track_min_m) / span * (w - 56)
            return 28 + self.view_offset_x + (base_x - 28) * self.view_zoom

        rail_y = 130 + self.view_offset_y
        block_y1 = 148 + self.view_offset_y
        block_y2 = 172 + self.view_offset_y
        psr_y1 = 184 + self.view_offset_y
        psr_y2 = 206 + self.view_offset_y
        earth_rail = "#8b5a32"
        earth_rail_dark = "#6b2e35"
        earth_zone = "#ffe3b5"
        earth_zone_occupied = "#ffc06b"
        earth_zone_outline = "#9b4d2b"
        main_start_m = min((start for start, _end, _gradient, _psr in sim.track_profile), default=0.0)
        main_end_m = sim.track_end_m
        main_x1 = x_from_pos(main_start_m)
        main_x2 = x_from_pos(main_end_m)
        c.create_line(28, rail_y, main_x1, rail_y, fill=earth_rail_dark, width=2, dash=(8, 5))
        c.create_line(main_x2, rail_y, w - 28, rail_y, fill=earth_rail_dark, width=2, dash=(8, 5))
        c.create_line(main_x1, rail_y, main_x2, rail_y, fill=earth_rail, width=5)

        def main_visual_lane(lane_count: int) -> int:
            return 1 if lane_count > 1 else 0

        def visual_lane_y(base_y: float, lane_count: int, visual_lane: int, spacing: float = 14.0) -> float:
            if lane_count <= 1:
                return base_y
            return base_y + (visual_lane - main_visual_lane(lane_count)) * spacing

        def visual_lane_for_protection(lane_count: int, protection_lane: int) -> int:
            if lane_count <= 1:
                return 0
            if protection_lane == 0:
                return main_visual_lane(lane_count)
            if protection_lane == main_visual_lane(lane_count):
                return 0
            return min(max(0, protection_lane), lane_count - 1)

        def lane_y(base_y: float, lane_count: int, lane: int) -> float:
            return visual_lane_y(base_y, lane_count, visual_lane_for_protection(lane_count, lane))

        def roman_lane_label(visual_lane: int) -> str:
            if visual_lane < len(PARALLEL_ROMAN_LABELS):
                return PARALLEL_ROMAN_LABELS[visual_lane]
            return str(visual_lane + 1)

        def lane_label(visual_lane: int, is_main: bool = False) -> str:
            return roman_lane_label(visual_lane) if is_main else str(visual_lane + 1)

        def protection_lane_label(lane_count: int, protection_lane: int) -> str:
            visual_lane = visual_lane_for_protection(lane_count, protection_lane)
            return lane_label(visual_lane, visual_lane == main_visual_lane(lane_count))

        def selected_width(kind: str, index: int, normal: int = 1) -> int:
            return 3 if self.selected_element == f"{kind}:{index}" else normal

        def draw_parallel_zone(
            kind: str,
            index: int,
            tag: str,
            x1: float,
            x2: float,
            center_y: float,
            lane_count: int,
            title: str,
            outline: str,
            fill: str,
            label_fill: str,
            dashed: bool = False,
            lane_spacing: float = 14.0,
            connect_left: bool = True,
            connect_right: bool = True,
            align_left_ends: bool = False,
            lane_status: Dict[int, str] | None = None,
            keep_main_lane_on_mainline: bool = False,
        ) -> Dict[int, Tuple[float, float, float]]:
            lane_count = max(1, lane_count)
            lane_positions = [visual_lane_y(center_y, lane_count, lane, lane_spacing) for lane in range(lane_count)]
            zone_y1 = min(lane_positions) - 28
            zone_y2 = max(lane_positions) + 28
            dash_option = {"dash": (4, 2)} if dashed else {}
            c.create_rectangle(
                x1,
                zone_y1,
                x2,
                zone_y2,
                outline=outline,
                fill=fill,
                width=selected_width(kind, index, 2),
                tags=(tag,),
                **dash_option,
            )
            c.create_rectangle(
                x1,
                zone_y1 - 18,
                x2,
                zone_y1,
                outline=outline,
                fill=APP_THEME["card"],
                width=selected_width(kind, index),
                tags=(tag,),
                **dash_option,
            )
            c.create_text(
                (x1 + x2) / 2,
                zone_y1 - 9,
                text=title,
                fill=label_fill,
                font=("Consolas", 8, "bold"),
                tags=(tag,),
            )
            c.create_line(x1, zone_y1, x1, zone_y2, fill=outline, width=3, tags=(tag,))
            c.create_line(x2, zone_y1, x2, zone_y2, fill=outline, width=3, tags=(tag,))
            main_lane = main_visual_lane(lane_count)
            span = max(1.0, x2 - x1)
            branch_step = max(12.0, min(20.0, span / 8.0))
            max_inset = max(8.0, span / 2.0 - 8.0)

            def lane_endpoints(visual_lane: int) -> Tuple[float, float]:
                if keep_main_lane_on_mainline and visual_lane == main_lane:
                    return x1, x2
                distance_from_main = abs(visual_lane - main_lane)
                inset = min(max_inset, 8.0 + distance_from_main * branch_step)
                left_inset = 8.0 if align_left_ends else inset
                return x1 + left_inset, x2 - inset

            for visual_lane in range(lane_count):
                y = visual_lane_y(center_y, lane_count, visual_lane, lane_spacing)
                if visual_lane != main_lane:
                    neighbor_lane = visual_lane + (1 if visual_lane < main_lane else -1)
                    neighbor_y = visual_lane_y(center_y, lane_count, neighbor_lane, lane_spacing)
                    left_x, right_x = lane_endpoints(visual_lane)
                    neighbor_left_x, neighbor_right_x = lane_endpoints(neighbor_lane)
                    if connect_left:
                        c.create_line(left_x, y, neighbor_left_x, neighbor_y, fill=earth_rail, width=2, tags=(tag,))
                    if connect_right:
                        c.create_line(neighbor_right_x, neighbor_y, right_x, y, fill=earth_rail, width=2, tags=(tag,))
            for visual_lane in range(lane_count):
                y = visual_lane_y(center_y, lane_count, visual_lane, lane_spacing)
                is_main = visual_lane == main_lane
                protection_lane = next(
                    (
                        lane for lane in range(lane_count)
                        if visual_lane_for_protection(lane_count, lane) == visual_lane
                    ),
                    visual_lane,
                )
                status = (lane_status or {}).get(protection_lane)
                line_width = 6 if is_main else 3
                if status == "GREEN":
                    line_fill = APP_THEME["ok"]
                elif status == "CD":
                    line_fill = APP_THEME["warning"]
                elif status == "RED":
                    line_fill = APP_THEME["danger"]
                else:
                    line_fill = earth_rail if is_main else earth_rail_dark
                left_x, right_x = lane_endpoints(visual_lane)
                c.create_line(left_x, y, right_x, y, fill=line_fill, width=line_width, tags=(tag,))
                c.create_oval(left_x - 3, y - 3, left_x + 3, y + 3, fill=line_fill, outline="", tags=(tag,))
                c.create_oval(right_x - 3, y - 3, right_x + 3, y + 3, fill=line_fill, outline="", tags=(tag,))
                label = lane_label(visual_lane, is_main)
                c.create_text(left_x + 4, y - 8, anchor="w", text=label, fill=label_fill, font=("Consolas", 7, "bold"), tags=(tag,))
            return {
                protection_lane: (
                    *lane_endpoints(visual_lane_for_protection(lane_count, protection_lane)),
                    visual_lane_y(center_y, lane_count, visual_lane_for_protection(lane_count, protection_lane), lane_spacing),
                )
                for protection_lane in range(lane_count)
            }

        for idx, condition in enumerate(getattr(sim, "line_conditions", [])):
            tag = self._element_tag("line_condition", idx)
            x1 = x_from_pos(float(condition["start"]))
            x2 = x_from_pos(float(condition["end"]))
            fill = "#ffeccc" if str(condition.get("condition", "")).lower() == "dry" else "#f4d1bd"
            c.create_rectangle(x1, rail_y + 18, x2, rail_y + 27, fill=fill, outline=APP_THEME["border"], width=selected_width("line_condition", idx), tags=(tag,))
            c.create_text((x1 + x2) / 2, rail_y + 36, text=str(condition.get("condition", "")).upper(), fill=APP_THEME["muted"], font=("Consolas", 7, "bold"), tags=(tag,))

        psr_labels: List[Tuple[float, float, str, str]] = []
        segment_boundary_labels: Dict[int, str] = {}
        for idx, (start, end, _gradient, psr) in enumerate(sim.track_profile):
            tag = self._element_tag("track_segment", idx)
            x1 = x_from_pos(start)
            x2 = x_from_pos(end)
            c.create_rectangle(x1, psr_y1, x2, psr_y2, fill=earth_zone, outline=earth_zone_outline, width=selected_width("track_segment", idx), tags=(tag,))
            segment_mid_x = (x1 + x2) / 2
            psr_labels.append((segment_mid_x, (psr_y1 + psr_y2) / 2, f"PSR {psr:.0f}", tag))
            segment_boundary_labels[int(round(start))] = tag
            segment_boundary_labels[int(round(end))] = tag

        show_fixed_blocks = getattr(sim, "block_mode", "fixed_block") == "fixed_block"
        if show_fixed_blocks:
            fixed_occupancy = sim.zc.fixed_block_occupancy()
            c.create_text(28, block_y1 - 9, anchor="w", text="FIXED BLOCKS", fill=APP_THEME["text"], font=("Consolas", 7, "bold"))
            for idx, block in enumerate(getattr(sim, "fixed_blocks", [])):
                block_id = str(block.get("id", f"FB{idx + 1}"))
                x1 = x_from_pos(float(block["start_m"]))
                x2 = x_from_pos(float(block["end_m"]))
                occupants = fixed_occupancy.get(block_id, [])
                fill = "#ffe0d1" if occupants else "#e5f0c8"
                outline = APP_THEME["danger"] if occupants else APP_THEME["ok"]
                c.create_rectangle(x1, block_y1, x2, block_y2, fill=fill, outline=outline, width=2)
                if x2 - x1 >= 34:
                    label = f"{block_id}" if not occupants else f"{block_id} OCC"
                    c.create_text((x1 + x2) / 2, (block_y1 + block_y2) / 2, text=label, fill=APP_THEME["text"], font=("Consolas", 7, "bold"))
                c.create_line(x1, block_y1 - 3, x1, block_y2 + 3, fill=outline, width=1)
                c.create_line(x2, block_y1 - 3, x2, block_y2 + 3, fill=outline, width=1)

        for idx, zone in enumerate(sim.tsr_zones):
            tag = self._element_tag("tsr", idx)
            x1 = x_from_pos(float(zone["start"]))
            x2 = x_from_pos(float(zone["end"]))
            c.create_rectangle(x1, psr_y2 + 5, x2, psr_y2 + 21, fill="#ffd6c9", outline=TSR_COLOR, width=selected_width("tsr", idx), tags=(tag,))
            c.create_text((x1 + x2) / 2, psr_y2 + 13, text=f"TSR {float(zone['speed']):.0f}", fill=TSR_COLOR, font=("Consolas", 8, "bold"), tags=(tag,))

        for pos_m in sorted(segment_boundary_labels):
            x = x_from_pos(float(pos_m))
            tag = segment_boundary_labels[pos_m]
            c.create_rectangle(x - 18, psr_y1 - 17, x + 18, psr_y1 - 2, fill=APP_THEME["card"], outline="", tags=(tag,))
            c.create_text(x, psr_y1 - 9, text=f"{pos_m}", fill=APP_THEME["text"], font=("Consolas", 8, "bold"), tags=(tag,))

        for psr_x, psr_y, psr_text, tag in psr_labels:
            c.create_rectangle(psr_x - 28, psr_y - 8, psr_x + 28, psr_y + 8, fill=APP_THEME["card"], outline="", tags=(tag,))
            c.create_text(psr_x, psr_y, text=psr_text, fill=APP_THEME["muted"], font=("Consolas", 8), tags=(tag,))

        for idx, stop in enumerate(sim.scheduled_stops):
            tag = self._element_tag("station", idx)
            pos_m = float(stop["pos_m"])
            length_m = float(stop.get("length_m", 160.0))
            capacity = int(stop.get("capacity", 3))
            x1 = x_from_pos(pos_m - length_m / 2.0)
            x2 = x_from_pos(pos_m + length_m / 2.0)
            station_state = sim.station_route_states[idx] if idx < len(getattr(sim, "station_route_states", [])) else {}
            route_lane = station_state.get("route_lane")
            lane_status = {}
            for lane in range(max(1, capacity)):
                line = sim._station_line_for_lane(idx, lane)
                if route_lane == lane:
                    lane_status[lane] = "GREEN"
                elif line is not None and (
                    line.get("occupied_by_train_id") is not None
                    or line.get("reserved_by_train_id") is not None
                    or line.get("route_state") in {"RESERVED", "LOCKED", "OCCUPIED", "DEPARTING", "RELEASE_PENDING"}
                ):
                    lane_status[lane] = "CD"
            draw_parallel_zone(
                "station",
                idx,
                tag,
                x1,
                x2,
                rail_y,
                capacity,
                str(stop.get("name", "STATION")),
                earth_zone_outline,
                "#ffe7b8",
                APP_THEME["text"],
                lane_status=lane_status,
                keep_main_lane_on_mainline=True,
            )
            stop_x = x_from_pos(pos_m)
            lane_positions = [visual_lane_y(rail_y, max(1, capacity), lane) for lane in range(max(1, capacity))]
            c.create_line(stop_x, min(lane_positions) - 10, stop_x, max(lane_positions) + 10, fill=APP_THEME["danger"], width=3, tags=(tag,))

        for label in sim.track_labels:
            x = x_from_pos(label)
            c.create_line(x, block_y2 + 24, x, block_y2 + 32, fill=APP_THEME["muted"], dash=(2, 2))
            c.create_text(x, block_y2 + 42, text=f"{int(label)}m", fill=APP_THEME["muted"], font=("Consolas", 7))

        def visible_train_span(head_x: float, tail_x: float) -> Tuple[float, float]:
            if abs(head_x - tail_x) >= 10.0:
                return min(tail_x, head_x), max(tail_x, head_x)
            center_x = (tail_x + head_x) / 2.0
            return center_x - 12.0, center_x + 12.0

        source_lane_count = max(
            (max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS))) for source in getattr(sim, "source_trains", [])),
            default=SOURCE_VISIBLE_ACTIVE_TRAINS,
        )
        source_lane_slots: Dict[int, Tuple[float, float, float]] = {}
        for idx, source in enumerate(getattr(sim, "source_trains", [])):
            tag = self._element_tag("source_train", idx)
            start_m = float(source["start_m"])
            length_m = float(source["length_m"])
            x1 = x_from_pos(start_m)
            x2 = x_from_pos(start_m + length_m)
            total = int(source.get("total_trains", 0))
            generated = int(source.get("generated", 0))
            lane_slots = draw_parallel_zone(
                "source_train",
                idx,
                tag,
                x1,
                x2,
                rail_y,
                max(1, int(source.get("capacity", SOURCE_VISIBLE_ACTIVE_TRAINS))),
                f"{source.get('name', 'SOURCE')} {generated}/{total}",
                "#000000",
                "#ffe7b8",
                "#000000",
                lane_spacing=26.0,
                connect_left=False,
                align_left_ends=True,
                keep_main_lane_on_mainline=True,
            )
            source_lane_slots.update(lane_slots)

        sorted_trains = sorted(sim.trains, key=lambda t: t.pos)
        for idx, train in enumerate(sorted_trains):
            tail = max(sim.track_min_m, train.pos - train.length)
            x1 = x_from_pos(tail)
            x2 = x_from_pos(train.pos)
            x1, x2 = visible_train_span(x2, x1)
            if train.protection_zone_id == "SOURCE":
                slot = source_lane_slots.get(int(train.protection_lane))
                y = slot[2] if slot is not None and train.pos <= SOURCE_TRAIN_EXIT_M + STOP_ACCURACY_TOL_M else rail_y
            elif isinstance(train.protection_zone_id, str) and train.protection_zone_id.startswith("STATION:"):
                try:
                    station_idx = int(train.protection_zone_id.split(":", 1)[1])
                    station_capacity = max(1, int(sim.scheduled_stops[station_idx].get("capacity", 3)))
                except (IndexError, ValueError):
                    station_capacity = 1
                if 0 <= station_idx < len(sim.scheduled_stops) and sim._train_head_in_station(train, sim.scheduled_stops[station_idx]):
                    y = lane_y(rail_y, station_capacity, int(train.protection_lane))
                else:
                    y = rail_y
            elif train.station_lane is not None and train.active_scheduled_stop is not None:
                station_idx = sim._station_index_for_stop(train.active_scheduled_stop)
                if station_idx is not None:
                    station_capacity = max(1, int(sim.scheduled_stops[station_idx].get("capacity", 3)))
                    y = lane_y(rail_y, station_capacity, int(train.station_lane)) if sim._train_head_in_station(train, sim.scheduled_stops[station_idx]) else rail_y
                else:
                    y = rail_y
            else:
                y = rail_y
            dcs_fault = bool(
                getattr(train, "dcs_fault_active", False)
                or getattr(train, "dcs_muted", False)
                or not getattr(train, "safe_packet_valid", True)
            )
            atp_fault = bool(getattr(train, "atp_fault_active", False))
            ato_fault = bool(getattr(train, "ato_fault_active", False))
            emergency_fault = bool(
                getattr(train, "trip_mode", False)
                or getattr(train, "emergency_stop", False)
                or getattr(train, "emg_latch", False)
                or getattr(train, "emergency_recovery_hold", False)
            )
            fault_active = atp_fault or ato_fault or dcs_fault or emergency_fault
            train_fill = "#6b7d90" if fault_active else train.color
            train_alert_outline = (
                APP_THEME["danger"]
                if atp_fault
                else "#4d5964"
                if emergency_fault
                else "#ff9f1c"
                if ato_fault
                else APP_THEME["warning"]
                if dcs_fault
                else ""
            )
            train_half_height = 5
            c.create_rectangle(
                x1 - 2,
                y - train_half_height - 2,
                x2 + 2,
                y + train_half_height + 2,
                fill="#000000",
                outline="#000000",
                width=2,
            )
            c.create_rectangle(
                x1,
                y - train_half_height,
                x2,
                y + train_half_height,
                fill=train_fill,
                outline="#000000",
                width=1,
            )
            if train_alert_outline:
                c.create_rectangle(
                    x1 - 4,
                    y - train_half_height - 4,
                    x2 + 4,
                    y + train_half_height + 4,
                    outline=train_alert_outline,
                    width=2,
                )
            c.create_text((x1 + x2) / 2, y - 22, text=f"{train.id} {train.pos:.0f}m", fill=APP_THEME["text"], font=("Consolas", 8, "bold"))
            fault_labels = []
            if atp_fault:
                fault_labels.append("ATP")
            if ato_fault:
                fault_labels.append("ATO")
            if dcs_fault:
                fault_labels.append("DCS")
            if emergency_fault and not atp_fault:
                fault_labels.append("EMG")
            label_text = f"{train.id} {'/'.join(fault_labels)}" if fault_labels else train.id
            c.create_text((x1 + x2) / 2, y - 11, text=label_text, fill=train_fill, font=("Consolas", 8, "bold"))
            if train.departure_hold:
                c.create_text((x1 + x2) / 2, y + 14, text="HOLD", fill=APP_THEME["danger"], font=("Consolas", 7, "bold"))
            rep_x = x_from_pos(train.reported_pos)
            c.create_oval(rep_x - 3, rail_y + 18, rep_x + 3, rail_y + 24, outline=train_fill, width=2)
            eoa_x = x_from_pos(train.eoa)
            c.create_line(eoa_x, rail_y - 30, eoa_x, rail_y + 32, fill=train_fill, dash=(3, 3))

            # Display distance to next train
            if idx + 1 < len(sorted_trains):
                next_train = sorted_trains[idx + 1]
                next_tail = max(sim.track_min_m, next_train.pos - next_train.length)
                distance_m = next_tail - train.pos
                
                # Position for distance label (between current train head and next train tail)
                mid_x = (x2 + x_from_pos(next_tail)) / 2
                gap_y = rail_y + 36
                
                distance_color = APP_THEME["ok"] if distance_m > SAFETY_MARGIN_M else APP_THEME["warning"] if distance_m > 0 else APP_THEME["danger"]
                c.create_text(mid_x, gap_y, text=f"gap: {distance_m:.1f}m", fill=distance_color, font=("Consolas", 8, "bold"))
                
                # Draw a thin line connecting between trains to show gap visually
                c.create_line(x2, gap_y - 8, x_from_pos(next_tail), gap_y - 8, fill=distance_color, dash=(1, 1), width=1)


class InfrastructurePanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        ttk.Label(self, text="ATS - Infrastructure State", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=1, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(13 * scale_factor),
            state="disabled",
            wrap="none",
            font=("Consolas", int(8 * scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        def fmt_schedule_variance(value: Any) -> str:
            if value is None:
                return "--"
            try:
                variance = float(value)
            except (TypeError, ValueError):
                return "--"
            if abs(variance) < 0.05:
                return "0.0s"
            if variance < 0.0:
                return f"+{abs(variance):.1f}s"
            return f"-{variance:.1f}s"

        lines = ["Asset                State                 Notes"]
        lines.append("-" * 72)
        if getattr(sim, "block_mode", "moving_block") == "fixed_block":
            blocks = list(getattr(sim, "fixed_blocks", []))
            for idx, block in enumerate(blocks, 1):
                start = float(block.get("start_m", 0.0))
                end = float(block.get("end_m", start))
                _gradient, psr = get_track_info(sim.track_profile, (start + end) / 2.0)
                occupied = any((t.pos - t.length) < end and t.pos > start for t in sim.trains)
                signal = "RED" if occupied else "GREEN"
                axle = "OCC" if occupied else "CLEAR"
                signal_id = str(block.get("id", f"FB{idx:02d}"))
                lines.append(f"{signal_id:<6} {start:>5.0f}-{end:<5.0f}  {axle:<20} fixed block psr={psr:.0f}")
                lines.append(f"SIG-{idx:02d}             {signal:<20} physical lineside signal")
        else:
            for idx, (start, end, gradient, psr) in enumerate(sim.track_profile, 1):
                occupied = any((t.pos - t.length) < end and t.pos > start for t in sim.trains)
                signal = "RED" if occupied else "GREEN"
                axle = "OCC" if occupied else "CLEAR"
                lines.append(f"VB-{idx:02d} {start:>5.0f}-{end:<5.0f}  {axle:<20} gradient={gradient:+.3f} psr={psr:.0f}")
                lines.append(f"SIG-{idx:02d}             {signal:<20} virtual lineside aspect")
        if sim.tsr_zones:
            lines.append("")
            lines.append("Temporary speed restrictions")
            lines.append("-" * 72)
            for idx, zone in enumerate(sim.tsr_zones, 1):
                lines.append(
                    f"TSR-{idx:02d} {float(zone['start']):>5.0f}-{float(zone['end']):<5.0f}  ACTIVE               limit={float(zone['speed']):.0f} km/h"
                )
        timetable_records = list((getattr(sim, "scenario", {}) or {}).get("headway", {}).get("timetable_records", []) or [])
        if timetable_records:
            lines.append("")
            lines.append("Lich trinh chay tau")
            lines.append("-" * 104)
            lines.append("Tau   Ga    Den        Dung   Di         Profile   Som+/Tre-")
            lines.append("-" * 104)
            variance_by_key: Dict[Tuple[str, str], float] = {}
            for station in sim.analytics.get("station_passenger_metrics", []):
                for arrival in station.get("arrivals", []):
                    station_name = str(station.get("station_name", "")).strip().lower()
                    schedule_station = str(arrival.get("schedule_station", "")).strip().lower()
                    variance = arrival.get("schedule_variance_s")
                    train_id = str(arrival.get("train_id", "")).strip().lower()
                    if train_id and variance is not None:
                        if station_name:
                            variance_by_key[(train_id, station_name)] = float(variance)
                        if schedule_station:
                            variance_by_key[(train_id, schedule_station)] = float(variance)
            for record in timetable_records:
                arrival = str(record.get("arrival_text") or "--")
                dwell = str(record.get("dwell_text") or "--")
                departure = str(record.get("departure_text") or "--")
                train_key = str(record.get("train_id", "")).strip().lower()
                station_key = str(record.get("station", "")).strip().lower()
                variance = variance_by_key.get((train_key, station_key))
                lines.append(
                    f"{str(record.get('train_id', '--')):<5} "
                    f"{str(record.get('station', '--')):<5} "
                    f"{arrival:<10} "
                    f"{dwell:<6} "
                    f"{departure:<10} "
                    f"{str(record.get('profile', '--')):<9} "
                    f"{fmt_schedule_variance(variance)}"
                )
        lines.append("")
        for idx, train in enumerate(sim.trains, 1):
            switch_state = "DIVERGING" if train.commanded_stop else "NORMAL"
            esa_state = "ACTIVE" if train.atp_action == "EBI" else "STANDBY"
            lines.append(f"SW-{idx:02d}              {switch_state:<20} simulated route authority")
            lines.append(f"ESA-{idx:02d}             {esa_state:<20} linked to {train.id}")
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")


class TimeDistancePanel(ttk.Frame):
    def __init__(self, master: tk.Widget):
        super().__init__(master, padding=8, style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        ttk.Label(self, text="ATS - Time Distance Graph", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            self,
            text="Actual running graph versus line distance for OCC regulation and dwell supervision",
            style="Muted.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 6))
        self.canvas = tk.Canvas(self, height=300, background=APP_THEME["canvas"], highlightthickness=1, highlightbackground=APP_THEME["border"])
        self.canvas.grid(row=2, column=0, sticky="ew")

    def update_data(self, sim: Simulation, time_history: deque, position_history: Dict[str, deque]):
        c = self.canvas
        w = max(320, c.winfo_width())
        h = max(180, c.winfo_height())
        c.delete("all")
        c.create_rectangle(1, 1, w - 1, h - 1, outline=APP_THEME["border"], fill=APP_THEME["canvas"])
        if len(time_history) < 2:
            return

        min_t = time_history[0]
        max_t = time_history[-1]
        span_t = max(1.0, max_t - min_t)
        span_pos = max(1.0, sim.track_max_m - sim.track_min_m)
        left = 46
        right = w - 16
        top = 16
        bottom = h - 28

        c.create_text(left, 8, anchor="w", text="Distance", fill=APP_THEME["accent"], font=("Consolas", 8, "bold"))
        c.create_text(right, h - 10, anchor="e", text="Time", fill=APP_THEME["accent"], font=("Consolas", 8, "bold"))
        c.create_line(left, top, left, bottom, fill=APP_THEME["border"])
        c.create_line(left, bottom, right, bottom, fill=APP_THEME["border"])

        for label in sim.track_labels:
            y = bottom - ((label - sim.track_min_m) / span_pos) * (bottom - top)
            c.create_line(left, y, right, y, fill="#ead9d2")
            c.create_text(left - 6, y, anchor="e", text=f"{int(label)}", fill=APP_THEME["muted"], font=("Consolas", 8))

        for train in sim.trains:
            series = position_history.get(train.id)
            if series is None or len(series) < 2:
                continue
            pts = []
            for idx, pos in enumerate(series):
                x = left + ((time_history[idx] - min_t) / span_t) * (right - left)
                y = bottom - ((pos - sim.track_min_m) / span_pos) * (bottom - top)
                pts.extend([x, y])
            c.create_line(*pts, fill=train.color, width=2)
            c.create_text(pts[-2] + 4, pts[-1], anchor="w", text=train.id, fill=train.color, font=("Consolas", 8, "bold"))


class EngineeringPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)

        ttk.Label(self, text="Engineering & Config", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")

        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=1, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)

        self.text = tk.Text(
            text_frame,
            state="disabled",
            wrap="none",
            font=("Consolas", int(8 * scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")

        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")

        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        lines = [
            "Static Guideway Database",
            "-" * 72,
        ]
        for idx, (start, end, gradient, psr) in enumerate(sim.track_profile, 1):
            lines.append(
                f"SEG-{idx:02d}  start={start:>6.1f}m  end={end:>6.1f}m  gradient={gradient:+.3f}  SSP={psr:>5.1f} km/h"
            )
        lines.append("")
        lines.append("Balise / Beacon Layout")
        lines.append("-" * 72)
        balise = sim.track_min_m
        while balise <= sim.track_max_m:
            lines.append(f"BALISE  pos={balise:>6.1f}m")
            balise += BALISE_SPACING_M
        lines.append("")
        lines.append("Train Configuration")
        lines.append("-" * 72)
        for train in sim.trains:
            adhesion_pct = ATP_ADHESION_FACTOR * 100.0
            mute_count = len(train.dcs_mute_windows)
            lines.append(
                f"{train.id:<4} mass={train.mass:>8.0f}kg  length={train.length:>5.1f}m  "
                f"mode={train.drive_mode:<5} manual_cap={train.max_manual_speed_kmh:>4.0f}km/h  "
                f"mute_windows={mute_count}  jerk={MAX_JERK_MS3:.2f}m/s3  adhesion={adhesion_pct:.0f}%"
            )
        lines.append("")
        lines.append("Safety Logic Baseline")
        lines.append("-" * 72)
        lines.extend(
            [
                f"Service brake model : factor={ATP_SERVICE_BRAKE_FACTOR:.2f}  buildup={ATP_BRAKE_BUILDUP_S:.2f}s",
                f"Emergency brake     : factor={ATP_EMERGENCY_BRAKE_FACTOR:.2f}  overlap={OVERLAP_M:.1f}m",
                f"Reaction delays      : P={ATP_P_REACTION_S:.1f}s  W={ATP_W_REACTION_S:.1f}s  SBI={ATP_SBI_REACTION_S:.1f}s  EBI={ATP_EBI_REACTION_S:.1f}s",
                f"Odometer error       : rate={ODOMETER_ERROR_RATE:.2f} m/m  base CI={POS_UNCERT_M:.1f}m  balise CI={BALISE_POS_UNCERT_M:.1f}m  station CI={STATION_POS_UNCERT_M:.1f}m  precise CI={PRECISE_STOP_POS_UNCERT_M:.1f}m",
                f"DCS transmission     : min={DCS_DELAY_MIN_S:.2f}s  max={DCS_DELAY_MAX_S:.2f}s  timeout={DCS_TIMEOUT_S:.1f}s",
            ]
        )
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")


class DiagnosticsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.prev_curve_debug: Dict[str, Dict[str, float]] = {}
        ttk.Label(self, text="Diagnostics & Logs", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(int(2 * scale_factor), int(6 * scale_factor)))
        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=2, column=0, sticky="nsew")
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(18 * scale_factor),
            state="disabled",
            wrap="none",
            font=("Consolas", int(8 * scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)
        self.text.bind("<MouseWheel>", self._on_text_mousewheel, add="+")
        self.text.bind("<Button-4>", self._on_text_mousewheel, add="+")
        self.text.bind("<Button-5>", self._on_text_mousewheel, add="+")

    def _on_text_mousewheel(self, event):
        if getattr(event, "num", None) == 4:
            delta = -3
        elif getattr(event, "num", None) == 5:
            delta = 3
        else:
            delta = -3 * int(event.delta / 120) if getattr(event, "delta", 0) else 0
        if delta:
            self.text.yview_scroll(delta, "units")
        return "break"

    def update_data(self, sim: Simulation, event_log: deque):
        dcs_ok = sum(1 for t in sim.trains if t.safe_packet_valid)
        max_error = max((abs(t.pos_error_m) for t in sim.trains), default=0.0)
        total_trains = len(sim.trains)
        self.summary_var.set(
            f"Realtime vital curves, odometry confidence and DCS timing  |  DCS healthy={dcs_ok}/{total_trains}  max odo error={max_error:.1f}m"
        )
        lines = [
            "Train  Actual  P      W      SBI    EBI    OdoErr  DCS Age  Link   Alert",
            "-" * 94,
        ]
        for train in sim.trains:
            lines.append(
                f"{train.id:<5} {ms_to_kmh(train.speed):>6.1f} {ms_to_kmh(train.curves['P']):>6.1f} "
                f"{ms_to_kmh(train.curves['W']):>6.1f} {ms_to_kmh(train.hidden_curves['SBI']):>6.1f} "
                f"{ms_to_kmh(train.hidden_curves['EBI']):>6.1f} {train.pos_error_m:>7.2f} "
                f"{train.safe_packet_age_s:>7.2f}s {'MUTE' if train.dcs_muted else 'OK' if train.safe_packet_valid else 'TRIP':<6} {train.atp_alert}"
            )
        lines.append("")
        lines.append("Final braking debug")
        lines.append("-" * 132)
        lines.append(
            "Train  Rem(m)  dRem/s  Mode     Rel% Prec Jog     RawSBI RawEBI  DspSBI DspEBI  dRawSBI/s dRawEBI/s  Reason"
        )
        for train in sim.trains:
            remaining_m = train.distance_to_stop_target()
            if not train.commanded_stop and remaining_m > JOG_MAX_DIST_M:
                continue

            raw_sbi = ms_to_kmh(train.raw_hidden_curves.get("SBI", 0.0))
            raw_ebi = ms_to_kmh(train.raw_hidden_curves.get("EBI", 0.0))
            display_sbi = ms_to_kmh(train.hidden_curves.get("SBI", 0.0))
            display_ebi = ms_to_kmh(train.hidden_curves.get("EBI", 0.0))
            prev = self.prev_curve_debug.get(train.id)
            dt_s = max(1e-6, sim.sim_time_s - prev["time"]) if prev else 0.0
            drem_s = (remaining_m - prev["remaining_m"]) / dt_s if prev and dt_s > 0.0 else 0.0
            draw_sbi_s = (raw_sbi - prev["raw_sbi"]) / dt_s if prev and dt_s > 0.0 else 0.0
            draw_ebi_s = (raw_ebi - prev["raw_ebi"]) / dt_s if prev and dt_s > 0.0 else 0.0
            display_raw_delta = max(abs(display_sbi - raw_sbi), abs(display_ebi - raw_ebi))
            precise_profile = precise_stop_profile_active(train, remaining_m)

            reason = "OK"
            if display_raw_delta > 0.3:
                reason = "DISPLAY_SMOOTHING"
            elif train.zero_speed_detected and STOP_ACCURACY_TOL_M < remaining_m <= JOG_MAX_DIST_M and abs(drem_s) < 0.02:
                reason = "DISTANCE_HELD_ZERO_SPEED"
            elif train.commanded_stop and 0.0 < remaining_m <= JOG_MAX_DIST_M and not precise_profile:
                reason = "PRECISE_PROFILE_OFF"
            elif train.release_active and train.release_blend > 0.0:
                reason = "RELEASE_BLEND_ACTIVE"
            if draw_sbi_s < -8.0 or draw_ebi_s < -8.0:
                reason = f"ABRUPT_DROP:{reason}"

            lines.append(
                f"{train.id:<5} {remaining_m:>7.2f} {drem_s:>7.2f} {train.curve_mode:<8} "
                f"{train.release_blend * 100.0:>5.0f} {'ON' if precise_profile else 'OFF':<4} {train.jog_state:<7} "
                f"{raw_sbi:>6.2f} {raw_ebi:>6.2f} {display_sbi:>7.2f} {display_ebi:>6.2f} "
                f"{draw_sbi_s:>9.2f} {draw_ebi_s:>9.2f}  {reason}"
            )
            self.prev_curve_debug[train.id] = {
                "time": sim.sim_time_s,
                "remaining_m": remaining_m,
                "raw_sbi": raw_sbi,
                "raw_ebi": raw_ebi,
            }
        lines.append("")
        lines.append("Recent event log")
        lines.append("-" * 94)
        if event_log:
            lines.extend(event_log)
        else:
            lines.append("No event yet.")
        y_first, y_last = self.text.yview()
        near_bottom = y_last >= 0.98
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")
        if near_bottom:
            self.text.yview_moveto(1.0)
        else:
            self.text.yview_moveto(y_first)


class AnalyticsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        ttk.Label(self, text="Scenario Analytics", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(int(2 * scale_factor), int(6 * scale_factor)))
        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=2, column=0, sticky="nsew")
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(18 * scale_factor),
            state="disabled",
            wrap="none",
            font=("Consolas", int(8 * scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        def fmt_seconds(value):
            if value is None:
                return "--"
            if value == float("inf") or (isinstance(value, float) and math.isinf(value)):
                return "terminal"
            return f"{float(value):,.1f}s"

        min_headway = sim.analytics["min_headway_s"]
        min_headway_text = "--" if min_headway is None else f"{min_headway:.1f}s"
        completed = len(sim.analytics["journey_times"])
        total_trains = len(sim.trains)
        self.summary_var.set(
            f"Moving-block comparison metrics  |  min headway={min_headway_text}  "
            f"completed={completed}/{total_trains}  EBI={sim.analytics['ebi_count']}  SBI={sim.analytics['sbi_count']}"
        )

        lines = [
            "Scenario KPI",
            "-" * 88,
            f"Simulation time        : {sim.sim_time_s:,.1f} s",
            f"Minimum headway        : {min_headway_text}",
            f"Emergency interventions: {sim.analytics['ebi_count']}",
            f"Service interventions  : {sim.analytics['sbi_count']}",
            "",
            "Train  Mode   Pos(m)   Speed  Headway   Journey     DCS     ATP",
            "-" * 88,
        ]
        for train in sorted(sim.trains, key=lambda item: item.id):
            journey = sim.analytics["journey_times"].get(train.id)
            journey_text = "--" if journey is None else f"{journey:,.1f}s"
            headway_text = "--" if train.headway_time_s is None else f"{train.headway_time_s:,.1f}s"
            link_text = "MUTE" if train.dcs_muted else "OK" if train.safe_packet_valid else "TIMEOUT"
            lines.append(
                f"{train.id:<5} {train.drive_mode:<6} {train.pos:>7.1f} {ms_to_kmh(train.speed):>6.1f} "
                f"{headway_text:>8} {journey_text:>10} {link_text:<7} {train.atp_state}"
            )

        station_metrics = sorted(
            sim.analytics.get("station_passenger_metrics", []),
            key=lambda item: (int(item.get("station_index", 0)), str(item.get("station_name", ""))),
        )
        lines.extend(["", "Station Headway / Train Wait", "-" * 88])
        if not station_metrics:
            lines.append("No station arrivals recorded yet.")
        else:
            lines.append("Station       Train   Arrive       Headway    Avg HW     Wait")
            lines.append("-" * 88)
            for station in station_metrics:
                station_name = str(station.get("station_name", f"STATION_{station.get('station_index', '')}"))
                avg_headway = station.get("avg_arrival_headway_s")
                arrivals = sorted(
                    station.get("arrivals", []),
                    key=lambda item: (float(item.get("arrival_time_s", 0.0)), str(item.get("train_id", ""))),
                )
                if not arrivals:
                    lines.append(f"{station_name:<13} {'--':<7} {'--':>10} {'--':>10} {fmt_seconds(avg_headway):>9} {'--':>8}")
                    continue
                for record in arrivals:
                    wait_s = record.get("station_wait_s", record.get("passenger_dwell_s", record.get("planned_dwell_s")))
                    lines.append(
                        f"{station_name:<13} {str(record.get('train_id', '--')):<7} "
                        f"{fmt_seconds(record.get('arrival_time_s')):>10} "
                        f"{fmt_seconds(record.get('arrival_headway_s')):>10} "
                        f"{fmt_seconds(avg_headway):>9} "
                        f"{fmt_seconds(wait_s):>8}"
                    )

        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")


class ControlPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, on_apply_psr, on_add_tsr, on_clear_tsr, scale_factor: float):
        super().__init__(master, padding=int(4 * scale_factor), borderwidth=1, relief="solid", width=int(170 * scale_factor), height=int(210 * scale_factor))
        self.on_apply_psr = on_apply_psr
        self.on_add_tsr = on_add_tsr
        self.on_clear_tsr = on_clear_tsr
        self.segment_label = None

        ttk.Label(self, text="Control", font=("Segoe UI", int(11 * scale_factor), "bold")).pack(anchor="w")

        psr_frame = ttk.LabelFrame(self, text="PSR Segment")
        psr_frame.pack(fill="x", pady=(int(8 * scale_factor), int(6 * scale_factor)))
        self.segment_label = ttk.Label(psr_frame, text="Segment")
        self.segment_label.grid(row=0, column=0, sticky="w", padx=int(4 * scale_factor), pady=int(2 * scale_factor))
        ttk.Label(psr_frame, text="PSR km/h").grid(row=1, column=0, sticky="w", padx=int(4 * scale_factor), pady=int(2 * scale_factor))
        self.psr_segment = tk.StringVar(value="0")
        self.psr_value = tk.StringVar(value="40")
        ttk.Entry(psr_frame, textvariable=self.psr_segment, width=8).grid(row=0, column=1, padx=4, pady=2)
        ttk.Entry(psr_frame, textvariable=self.psr_value, width=8).grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(psr_frame, text="Apply PSR", command=self._apply_psr).grid(row=2, column=0, columnspan=2, sticky="we", padx=4, pady=4)

        tsr_frame = ttk.LabelFrame(self, text="TSR Zone")
        tsr_frame.pack(fill="x", pady=(4, 6))
        ttk.Label(tsr_frame, text="Start (m)").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(tsr_frame, text="End (m)").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(tsr_frame, text="Speed km/h").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        self.tsr_start = tk.StringVar(value="600")
        self.tsr_end = tk.StringVar(value="900")
        self.tsr_speed = tk.StringVar(value="25")
        ttk.Entry(tsr_frame, textvariable=self.tsr_start, width=8).grid(row=0, column=1, padx=4, pady=2)
        ttk.Entry(tsr_frame, textvariable=self.tsr_end, width=8).grid(row=1, column=1, padx=4, pady=2)
        ttk.Entry(tsr_frame, textvariable=self.tsr_speed, width=8).grid(row=2, column=1, padx=4, pady=2)
        ttk.Button(tsr_frame, text="Add TSR", command=self._add_tsr).grid(row=3, column=0, columnspan=2, sticky="we", padx=4, pady=4)
        ttk.Button(tsr_frame, text="Clear TSR", command=self._clear_tsr).grid(row=4, column=0, columnspan=2, sticky="we", padx=4, pady=(0, 4))

    def _apply_psr(self):
        self.on_apply_psr(self.psr_segment.get(), self.psr_value.get())

    def _add_tsr(self):
        self.on_add_tsr(self.tsr_start.get(), self.tsr_end.get(), self.tsr_speed.get())

    def _clear_tsr(self):
        self.on_clear_tsr()

    def update_track_profile(self, track_profile: List[Tuple[float, float, float, float]]):
        if not track_profile:
            self.segment_label.config(text="Segment")
            return
        self.segment_label.config(text=f"Segment (0-{len(track_profile) - 1})")


class SpeedLimitsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(4 * scale_factor), borderwidth=1, relief="solid", width=int(240 * scale_factor), height=int(260 * scale_factor))
        ttk.Label(self, text="Speed Limits", font=("Segoe UI", int(12 * scale_factor), "bold")).pack(anchor="w")
        content = ttk.Frame(self)
        content.pack(fill="both", expand=True, pady=(int(4 * scale_factor), 0))
        self.text = tk.Text(
            content,
            width=int(30 * scale_factor),
            height=int(14 * scale_factor),
            state="disabled",
            font=("Consolas", int(9 * scale_factor)),
            wrap="none",
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            insertbackground=APP_THEME["text"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.text.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(content, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        self.text.configure(yscrollcommand=vscroll.set)
        hscroll = ttk.Scrollbar(content, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(xscrollcommand=hscroll.set)
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)

    def update_limits(self, track_profile, tsr_zones):
        lines = []
        lines.append("Track segments:")
        for i, (start, end, grad, psr) in enumerate(track_profile):
            lines.append(f"  {i}: {start:.0f}-{end:.0f} m  grad={grad:+.3f}  PSR={psr:.0f} km/h")
        lines.append("")
        lines.append("TSR (temporary):")
        if not tsr_zones:
            lines.append("  (none)")
        else:
            for idx, z in enumerate(tsr_zones, 1):
                lines.append(f"  {idx}: {z['start']:.0f}-{z['end']:.0f} m  {z['speed']:.0f} km/h")

        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")


class AddElementDialog(tk.Toplevel):
    ADDABLE_ELEMENTS = [
        "Station",
        "Segment PSR/TSR",
        "Gradient Segment",
        "Line",
        "Line Condition",
        "Source Train",
    ]

    FIELD_SETS = {
        "Station": [
            ("name", "Name", "STATION_NEW"),
            ("pos_m", "Stop position (m)", "1000"),
            ("length_m", "Station length (m)", "160"),
            ("capacity", "Train capacity", "3"),
            ("dwell_s", "Dwell (s)", "30"),
        ],
        "Segment PSR/TSR": [
            ("mode", "Mode (PSR/TSR)", "TSR"),
            ("start_m", "Start (m)", "600"),
            ("end_m", "End (m)", "900"),
            ("speed_kmh", "Limit speed (km/h)", "25"),
        ],
        "PSR Segment": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("gradient", "Gradient", "0.00"),
            ("psr_kmh", "PSR km/h", "80"),
        ],
        "TSR": [
            ("start_m", "Start (m)", "600"),
            ("end_m", "End (m)", "900"),
            ("speed_kmh", "Limit speed (km/h)", "25"),
        ],
        "Gradient Segment": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("gradient", "Gradient", "0.00"),
        ],
        "Line": [
            ("length_m", "Line length (m)", "2000"),
        ],
        "Line Condition": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("condition", "Condition (dry/wet)", "dry"),
        ],
        "Source Train": [
            ("name", "Name", "SRC"),
            ("capacity", "Source trains", "3"),
        ],
    }

    def __init__(
        self,
        master: tk.Widget,
        on_add,
        initial_element: str = "Station",
        initial_values: Dict[str, str] | None = None,
        locked_element: bool = False,
        submit_text: str = "Add",
        title: str = "Add ATS Element",
        on_cancel=None,
        element_choices: List[str] | None = None,
    ):
        super().__init__(master)
        self.title(title)
        self.resizable(False, False)
        self.configure(background=APP_THEME["bg"])
        self.on_add = on_add
        self.on_cancel = on_cancel
        self.initial_values = initial_values or {}
        self.values: Dict[str, tk.StringVar] = {}
        self.element_choices = list(element_choices or self.ADDABLE_ELEMENTS)
        if initial_element not in self.element_choices:
            self.element_choices = [initial_element, *self.element_choices]
        self.element_var = tk.StringVar(value=initial_element)

        body = ttk.Frame(self, padding=12, style="Shell.TFrame")
        body.grid(row=0, column=0, sticky="nsew")
        ttk.Label(body, text="Element").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        chooser_state = "disabled" if locked_element else "readonly"
        chooser = ttk.Combobox(body, textvariable=self.element_var, values=self.element_choices, state=chooser_state, width=20)
        chooser.grid(row=0, column=1, sticky="ew", pady=(0, 8))
        chooser.bind("<<ComboboxSelected>>", lambda _event: self._render_fields())

        self.fields_frame = ttk.Frame(body, style="Shell.TFrame")
        self.fields_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        buttons = ttk.Frame(body, style="Shell.TFrame")
        buttons.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        buttons.columnconfigure(0, weight=1)
        ttk.Button(buttons, text="Cancel", command=self._cancel).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(buttons, text=submit_text, command=self._submit).grid(row=0, column=2)

        self.status_var = tk.StringVar(value="")
        ttk.Label(body, textvariable=self.status_var, style="Muted.TLabel").grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self._render_fields()
        self.transient(master)
        self.grab_set()
        self.focus_force()
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self._center_on_master()

    def _render_fields(self):
        for widget in self.fields_frame.winfo_children():
            widget.destroy()
        self.values = {}
        for row, (key, label, default) in enumerate(self.FIELD_SETS[self.element_var.get()]):
            ttk.Label(self.fields_frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
            var = tk.StringVar(value=self.initial_values.get(key, default))
            self.values[key] = var
            ttk.Entry(self.fields_frame, textvariable=var, width=22).grid(row=row, column=1, sticky="ew", pady=3)

    def _submit(self):
        data = {key: var.get().strip() for key, var in self.values.items()}
        try:
            self.on_add(self.element_var.get(), data)
        except ValueError as exc:
            self.status_var.set(str(exc))
            return
        self.destroy()

    def _cancel(self):
        if self.on_cancel is not None:
            self.on_cancel()
        self.destroy()

    def _center_on_master(self):
        self.update_idletasks()
        master = self.master
        if master is not None:
            master.update_idletasks()
            master_x = master.winfo_rootx()
            master_y = master.winfo_rooty()
            master_w = max(1, master.winfo_width())
            master_h = max(1, master.winfo_height())
            x = master_x + (master_w - self.winfo_width()) // 2
            y = master_y + (master_h - self.winfo_height()) // 2
        else:
            x = (self.winfo_screenwidth() - self.winfo_width()) // 2
            y = (self.winfo_screenheight() - self.winfo_height()) // 2
        self.geometry(f"+{max(0, x)}+{max(0, y)}")


class MonteCarloPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, app: "App"):
        super().__init__(master, padding=14, style="Shell.TFrame")
        self.app = app
        self._worker: threading.Thread | None = None
        self._queue: queue.Queue = queue.Queue()
        self._stop_requested = False

        self.columnconfigure(0, weight=1)
        self.rowconfigure(5, weight=1)
        ttk.Label(self, text="Chế độ thống kê Monte Carlo", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")

        controls = ttk.Frame(self, style="Shell.TFrame")
        controls.grid(row=1, column=0, sticky="ew", pady=(10, 6))
        self.runs_var = tk.StringVar(value="100")
        self.max_time_var = tk.StringVar(value="1000")
        self.seed_var = tk.StringVar(value="")
        ttk.Label(controls, text="Runs", style="Status.TLabel").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.runs_var, width=7).grid(row=0, column=1, padx=(0, 10))
        ttk.Label(controls, text="Max sim s", style="Status.TLabel").grid(row=0, column=2, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.max_time_var, width=8).grid(row=0, column=3, padx=(0, 10))
        ttk.Label(controls, text="Replay ID", style="Status.TLabel").grid(row=0, column=4, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.seed_var, width=10).grid(row=0, column=5, padx=(0, 10))
        self.run_btn = ttk.Button(controls, text="Chạy thống kê", command=self.start)
        self.run_btn.grid(row=0, column=6, padx=(10, 4))
        self.stop_btn = ttk.Button(controls, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.grid(row=0, column=7)

        self.progress_var = tk.DoubleVar(value=0.0)
        ttk.Progressbar(self, variable=self.progress_var, maximum=100.0).grid(row=2, column=0, sticky="ew", pady=(2, 6))
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self, textvariable=self.status_var, style="Status.TLabel").grid(row=3, column=0, sticky="w")
        ttk.Label(
            self,
            text="Replay ID is optional: leave it blank for a fresh random batch, or reuse the same number to replay the same batch.",
            style="Muted.TLabel",
        ).grid(row=4, column=0, sticky="w", pady=(0, 4))

        self.output = tk.Text(
            self,
            height=36,
            wrap="none",
            background="#fbfdff",
            foreground=APP_THEME["text"],
            font=("Consolas", 9),
            relief="solid",
            borderwidth=1,
        )
        self.output.grid(row=5, column=0, sticky="nsew", pady=(8, 0))
        self._set_output("Kết quả thống kê sẽ hiển thị tại đây.\n")

    def _config_from_fields(self) -> MonteCarloConfig:
        seed_text = self.seed_var.get().strip()
        seed = int(seed_text) if seed_text else None
        return MonteCarloConfig(
            runs=max(1, int(float(self.runs_var.get()))),
            max_sim_time_s=max(1.0, float(self.max_time_var.get())),
            seed=seed,
        )

    def start(self):
        if self._worker is not None and self._worker.is_alive():
            return
        try:
            config = self._config_from_fields()
        except ValueError as exc:
            self.status_var.set(f"Invalid Monte Carlo input: {exc}")
            return
        self._stop_requested = False
        self.progress_var.set(0.0)
        self.status_var.set("Starting Monte Carlo batch")
        self._set_output("Running...\n")
        self.run_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        scenario = deepcopy(self.app.scenario)

        def progress(summary):
            if self._stop_requested:
                raise RuntimeError("Monte Carlo stopped by user")
            self._queue.put(("summary", summary))

        def status(update):
            if self._stop_requested:
                raise RuntimeError("Monte Carlo stopped by user")
            self._queue.put(("status", dict(update)))

        def worker():
            try:
                summary = run_batch(
                    scenario,
                    config,
                    Simulation,
                    sys.modules[__name__],
                    progress=progress,
                    status=status,
                )
                self._queue.put(("done", summary))
            except Exception as exc:
                self._queue.put(("error", str(exc)))

        self._worker = threading.Thread(target=worker, daemon=True)
        self._worker.start()
        self.after(100, self._poll_queue)

    def stop(self):
        self._stop_requested = True
        self.status_var.set("Stopping after current Monte Carlo callback")

    def _set_output(self, text: str):
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        self.output.insert("1.0", text)
        self.output.configure(state="disabled")

    def _poll_queue(self):
        keep_polling = self._worker is not None and self._worker.is_alive()
        while True:
            try:
                kind, payload = self._queue.get_nowait()
            except queue.Empty:
                break
            if kind == "summary":
                completed = len(payload.results)
                total = max(1, payload.config.runs)
                self.progress_var.set(completed / total * 100.0)
                self.status_var.set(f"{completed}/{total} samples ({completed / total * 100.0:.0f}%)")
                self._set_output("\n".join(payload.table_lines()))
            elif kind == "status":
                run_id = payload.get("run_id", 0)
                runs = payload.get("runs", 0)
                phase = payload.get("phase", "Running")
                sim_time = payload.get("sim_time_s")
                suffix = f"  t={sim_time:.1f}s" if isinstance(sim_time, (int, float)) else ""
                self.status_var.set(f"Run {run_id}/{runs}: {phase}{suffix}")
            elif kind == "done":
                self.progress_var.set(100.0)
                self.status_var.set("Monte Carlo complete")
                self._set_output("\n".join(payload.table_lines()))
                self.run_btn.configure(state="normal")
                self.stop_btn.configure(state="disabled")
                keep_polling = False
            elif kind == "error":
                self.status_var.set(payload)
                self.run_btn.configure(state="normal")
                self.stop_btn.configure(state="disabled")
                keep_polling = False
        if keep_polling:
            self.after(100, self._poll_queue)


class App(tk.Tk):
    def __init__(self):
        super().__init__()

        # Detect DPI and set scaling for responsiveness
        try:
            user32 = ctypes.windll.user32
            user32.SetProcessDPIAware()
            dpi = user32.GetDpiForSystem()
            scale_factor = dpi / 96.0  # 96 is default DPI
            self.tk.call('tk', 'scaling', scale_factor)
        except:
            # Fallback if DPI detection fails
            scale_factor = 1.0

        self.scale_factor = scale_factor
        self.scenario = load_scenario()
        self.title(self.scenario["window_title"])
        # Set fullscreen mode
        self.state('zoomed')  # Windows fullscreen
        self.resizable(True, True)
        self.configure(background=APP_THEME["bg"])
        self._configure_styles()

        self.sim = Simulation(self.scenario)
        self.time_scale = 1
        self.sim_paused = False
        self.current_workspace_mode = "normal"
        self.operation_mode_var = tk.StringVar(value=self._operation_mode_from_scenario())
        self.operation_mechanism_var = tk.StringVar(value="")
        self.operation_selected_status_var = tk.StringVar(value="")
        self.headway_target_var = tk.StringVar(value=str(self.scenario.get("headway", {}).get("target_headway_s", 180.0)))
        self.timetable_file_var = tk.StringVar(value=str(self.scenario.get("headway", {}).get("timetable_file", "")))
        adaptive_cfg = self.scenario.get("headway", {}).get("adaptive", {}) or {}
        adaptive_target_s = float(adaptive_cfg.get("min_headway_s", self.scenario.get("headway", {}).get("target_headway_s", 180.0)) or 180.0)
        self.adaptive_tph_var = tk.StringVar(value=f"{3600.0 / adaptive_target_s:.1f}" if adaptive_target_s > 0.0 else "20")
        self.blocks_per_section_var = tk.StringVar(
            value=str(self.scenario.get("capacity_baseline", {}).get("blocks_per_section", 4))
        )
        self.vn_clock_var = tk.StringVar(value="")
        self.time_scale_buttons: Dict[int, ttk.Button] = {}
        self.time_history = deque(maxlen=240)
        self.position_history: Dict[str, deque] = {}
        self.event_log = deque(maxlen=30)
        self._last_ui_refresh_real_s = 0.0
        self._running_ui_refresh_interval_s = DT
        self._idle_ui_refresh_interval_s = 1.00
        self.edit_undo_stack: List[Dict[str, Any]] = []
        self.edit_redo_stack: List[Dict[str, Any]] = []
        self.prev_train_snapshot: Dict[str, Tuple[str, str, bool, bool, bool]] = {}
        self.child_windows: Dict[str, tk.Toplevel] = {}
        self._reset_runtime_buffers()
        self.pending_line_extension: Tuple[float, float] | None = None
        self.pending_station_prompt_after_limit = False
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        container = ttk.Frame(self, style="Shell.TFrame")
        container.grid(row=0, column=0, sticky="nsew")
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(container, background=APP_THEME["bg"], highlightthickness=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(container, orient="vertical", command=self.scroll_canvas.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        self.scroll_canvas.configure(yscrollcommand=vscroll.set)

        # Bind mouse wheel scrolling
        self.scroll_canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-4>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-5>", self._on_mousewheel)

        self.content = ttk.Frame(self.scroll_canvas, padding=10, style="Shell.TFrame")
        self.content.grid_columnconfigure(0, weight=1)
        self.content.grid_rowconfigure(5, weight=1)
        self.scroll_canvas_frame = self.scroll_canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all")),
        )
        self.scroll_canvas.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.itemconfig(self.scroll_canvas_frame, width=event.width, height=event.height),
        )

        header = ttk.Frame(self.content, padding=10, style="Shell.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        btns = ttk.Frame(header, style="Shell.TFrame")
        btns.grid(row=0, column=0, sticky="ew")
        btns.columnconfigure(0, weight=5)
        sim_group = self._make_button_group(btns, "Simulation Control", 0)
        element_group = self._make_button_group(btns, "Element Editing", 1)
        scenario_group = self._make_button_group(btns, "Scenario I/O", 2)
        mode_group = self._make_button_group(btns, "Mode", 3)

        self.start_btn = ttk.Button(sim_group, text="Start", command=self.on_start, style="Accent.TButton")
        self.start_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.stop_btn = ttk.Button(sim_group, text="II", command=self.on_stop, style="History.TButton", width=3)
        self.stop_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.reset_sim_btn = ttk.Button(sim_group, text="⟳", command=self.on_reset_simulation, style="History.TButton", width=3)
        self.reset_sim_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")
        for idx, scale in enumerate((1, 2, 5, 10, 100), start=3):
            button = ttk.Button(sim_group, text=f"x{scale}", command=lambda value=scale: self.set_time_scale(value))
            button.grid(row=1, column=idx, padx=2, pady=(2, 4), sticky="ew")
            self.time_scale_buttons[scale] = button
        self._update_time_scale_buttons()

        self.add_element_btn = ttk.Button(element_group, text="Add", command=self.open_add_element_dialog)
        self.add_element_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.delete_element_btn = ttk.Button(element_group, text="Delete", command=self.delete_selected_element)
        self.delete_element_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        element_group.grid_remove()
        self.undo_edit_btn = ttk.Button(element_group, text="↶", command=self.undo_canvas_edit, width=3, style="History.TButton")
        self.undo_edit_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")
        self.redo_edit_btn = ttk.Button(element_group, text="↷", command=self.redo_canvas_edit, width=3, style="History.TButton")
        self.redo_edit_btn.grid(row=1, column=3, padx=3, pady=(2, 4), sticky="ew")
        self._update_edit_history_buttons()
        self._update_run_pause_buttons()
        self.undo_edit_btn.grid_remove()
        self.redo_edit_btn.grid_remove()
        self.undo_edit_btn = ttk.Button(sim_group, text="↶", command=self.undo_canvas_edit, width=3, style="History.TButton")
        self.redo_edit_btn = ttk.Button(sim_group, text="↷", command=self.redo_canvas_edit, width=3, style="History.TButton")
        self.undo_edit_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.redo_edit_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.start_btn.grid_configure(column=2)
        self.stop_btn.grid_configure(column=3)
        self.reset_sim_btn.configure(text="⟳")
        self.reset_sim_btn.grid_configure(column=4)
        for idx, scale in enumerate((1, 2, 5, 10, 100), start=5):
            self.time_scale_buttons[scale].grid_configure(column=idx)
        for idx in range(10):
            sim_group.columnconfigure(idx, weight=1)
        for idx in range(3):
            scenario_group.columnconfigure(idx, weight=1)
        for idx in range(1):
            mode_group.columnconfigure(idx, weight=1)
        scenario_group.grid_configure(column=1)
        mode_group.grid_configure(column=2)
        btns.columnconfigure(1, weight=1)
        btns.columnconfigure(2, weight=0)
        btns.columnconfigure(3, weight=0)
        self._update_edit_history_buttons()
        self._update_run_pause_buttons()

        self.load_btn = ttk.Button(scenario_group, text="Load Scenario", command=self.on_load_scenario)
        self.load_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.save_scenario_btn = ttk.Button(scenario_group, text="Save YAML", command=self.on_save_scenario)
        self.save_scenario_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.export_btn = ttk.Button(scenario_group, text="Export Report", command=self.on_export_report)
        self.export_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")

        self.mode_toggle_btn = ttk.Button(mode_group, text="Chế độ thống kê", command=self.toggle_workspace_mode)
        self.mode_toggle_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")

        clock_frame = ttk.Frame(header, padding=(10, 5, 10, 5), style="Clock.TFrame")
        clock_frame.grid(row=0, column=1, sticky="e", padx=(8, 0))
        ttk.Label(clock_frame, text="VIETNAM STANDARD TIME", style="ClockSmall.TLabel").pack(anchor="e")
        ttk.Label(clock_frame, textvariable=self.vn_clock_var, style="Clock.TLabel").pack(anchor="e")

        self.status_var = tk.StringVar(value="Status: stopped")
        self.scenario_var = tk.StringVar(value=f"Scenario: {self.scenario['name']}")
        self.clock_var = tk.StringVar(value="Sim time: 0.0 s")
        self.summary_var = tk.StringVar(value="")
        status_row = ttk.Frame(self.content, padding=(10, 0, 10, 0), style="Shell.TFrame")
        status_row.grid(row=1, column=0, sticky="ew")
        status_row.grid_columnconfigure(0, weight=1)
        ttk.Label(status_row, textvariable=self.status_var, style="Status.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(status_row, textvariable=self.clock_var, style="Status.TLabel").grid(row=0, column=1, sticky="e", padx=(0, 12))
        ttk.Label(status_row, textvariable=self.scenario_var, style="Status.TLabel").grid(row=0, column=2, sticky="e")

        summary_frame = ttk.Frame(self.content, padding=(10, 8, 10, 0), style="Shell.TFrame")
        summary_frame.grid(row=2, column=0, sticky="ew")
        ttk.Label(summary_frame, textvariable=self.summary_var, style="Status.TLabel").pack(anchor="w")

        headway_frame = ttk.LabelFrame(self.content, text="Operation Mode", padding=8)
        headway_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(8, 0))
        ttk.Label(headway_frame, text="Mode").grid(row=0, column=0, padx=(0, 4), sticky="w")
        self.operation_mode_combo = ttk.Combobox(
            headway_frame,
            textvariable=self.operation_mode_var,
            values=("1 Fixed-block", "2 Headway target", "3 Timetable", "4 Adaptive tph"),
            state="readonly",
            width=20,
        )
        self.operation_mode_combo.grid(row=0, column=1, padx=(0, 8), sticky="w")
        self.operation_mode_combo.bind("<<ComboboxSelected>>", lambda _event: self._refresh_operation_mode_controls())
        ttk.Label(headway_frame, text="Mechanism").grid(row=0, column=2, padx=(0, 4), sticky="w")
        ttk.Label(headway_frame, textvariable=self.operation_mechanism_var, style="Status.TLabel").grid(row=0, column=3, padx=(0, 12), sticky="w")
        self.blocks_label = ttk.Label(headway_frame, text="Blocks/section")
        self.blocks_entry = ttk.Entry(headway_frame, textvariable=self.blocks_per_section_var, width=8)
        self.target_label = ttk.Label(headway_frame, text="Target seconds")
        self.target_entry = ttk.Entry(headway_frame, textvariable=self.headway_target_var, width=10)
        self.timetable_label = ttk.Label(headway_frame, text="Schedule file")
        self.timetable_entry = ttk.Entry(headway_frame, textvariable=self.timetable_file_var, width=42)
        self.timetable_button = ttk.Button(headway_frame, text="Load YAML/MD", command=self.load_timetable_file)
        self.timetable_set_after_now_button = ttk.Button(
            headway_frame,
            text="Set +1.5 min",
            command=self.set_timetable_after_now,
        )
        self.tph_label = ttk.Label(headway_frame, text="Trains/hour")
        self.tph_entry = ttk.Entry(headway_frame, textvariable=self.adaptive_tph_var, width=10)
        ttk.Button(headway_frame, text="Apply + Reset", command=self.apply_headway_block_settings).grid(row=0, column=9, padx=(0, 6))
        ttk.Button(headway_frame, text="Reload Values", command=self.reload_headway_block_values).grid(row=0, column=10)
        ttk.Label(headway_frame, textvariable=self.operation_selected_status_var, style="Status.TLabel").grid(
            row=1, column=0, columnspan=11, sticky="w", pady=(6, 0)
        )
        self._refresh_operation_mode_controls()

        workspace = ttk.PanedWindow(self.content, orient=tk.HORIZONTAL)
        workspace.grid(row=4, column=0, sticky="nsew", padx=6, pady=(8, 0))
        self.workspace = workspace
        workspace.bind("<Configure>", lambda _event: self.after_idle(self._fit_workspace_panes), add="+")
        self.content.grid_rowconfigure(4, weight=1)
        self.content.grid_rowconfigure(5, weight=0)

        side_shell = ttk.Frame(workspace, padding=(0, 0, 6, 0), style="Shell.TFrame")
        side_shell.columnconfigure(0, weight=1)
        side_shell.rowconfigure(0, weight=1)
        self.side_toolbar_canvas = tk.Canvas(side_shell, background=APP_THEME["workspace"], highlightthickness=0, width=int(190 * self.scale_factor))
        self.side_toolbar_canvas.grid(row=0, column=0, sticky="nsew")
        side_scrollbar = ttk.Scrollbar(side_shell, orient="vertical", command=self.side_toolbar_canvas.yview)
        side_scrollbar.grid(row=0, column=1, sticky="ns")
        self.side_toolbar_canvas.configure(yscrollcommand=side_scrollbar.set)
        side_toolbar = ttk.Frame(self.side_toolbar_canvas, style="Shell.TFrame")
        self.side_toolbar_window = self.side_toolbar_canvas.create_window((0, 0), window=side_toolbar, anchor="nw")
        side_toolbar.bind("<Configure>", lambda _event: self.side_toolbar_canvas.configure(scrollregion=self.side_toolbar_canvas.bbox("all")))
        self.side_toolbar_canvas.bind("<Configure>", lambda event: self.side_toolbar_canvas.itemconfigure(self.side_toolbar_window, width=event.width))
        side_toolbar.columnconfigure(0, weight=1)
        self._bind_side_toolbar_scroll(self.side_toolbar_canvas)

        element_side = ttk.LabelFrame(side_toolbar, text="Element Editing", padding=6)
        element_side.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        element_side.columnconfigure(0, weight=1)
        self.add_element_btn = ttk.Button(element_side, text="Add", command=self.open_add_element_dialog)
        self.add_element_btn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self.delete_element_btn = ttk.Button(element_side, text="Delete", command=self.delete_selected_element)
        self.delete_element_btn.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(element_side, text="Source + Train", command=self.add_train_to_selected_source).grid(row=2, column=0, sticky="ew", padx=2, pady=(8, 2))
        ttk.Button(element_side, text="Source - Train", command=self.remove_train_from_selected_source).grid(row=3, column=0, sticky="ew", padx=2, pady=2)

        faults_side = ttk.LabelFrame(side_toolbar, text="Selected Train Faults", padding=6)
        faults_side.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        faults_side.columnconfigure(0, weight=1)
        ttk.Button(faults_side, text="DCS Loss", command=self.toggle_all_dcs_loss, style="Danger.TButton").grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(faults_side, text="Clear Faults", command=self.clear_all_faults, style="Inactive.TButton").grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        self.emergency_fault_frame = ttk.LabelFrame(faults_side, text="Emergency Stop", padding=4)
        self.emergency_fault_frame.grid(row=2, column=0, sticky="ew", padx=2, pady=(6, 2))
        self.atp_fault_frame = ttk.LabelFrame(faults_side, text="ATP Fault", padding=4)
        self.atp_fault_frame.grid(row=3, column=0, sticky="ew", padx=2, pady=2)
        self.ato_fault_frame = ttk.LabelFrame(faults_side, text="ATO Fault", padding=4)
        self.ato_fault_frame.grid(row=4, column=0, sticky="ew", padx=2, pady=2)
        for frame in (self.emergency_fault_frame, self.atp_fault_frame, self.ato_fault_frame):
            frame.columnconfigure(0, weight=1)
        self.train_fault_buttons: Dict[str, Dict[str, ttk.Button]] = {}
        self._bind_side_toolbar_tree(side_toolbar)

        workspace.add(side_shell, weight=0)

        center_workspace = ttk.PanedWindow(workspace, orient=tk.VERTICAL)
        self.center_workspace = center_workspace
        workspace.add(center_workspace, weight=4)
        canvas_shell = ttk.Frame(center_workspace, style="Shell.TFrame")
        canvas_shell.columnconfigure(0, weight=1)
        canvas_shell.rowconfigure(0, weight=1)
        self.ats_overview_panel = ATSOverviewPanel(
            canvas_shell,
            self.scale_factor,
            on_select=self.on_ats_element_selected,
            on_edit=self.open_edit_element_dialog,
        )
        self.ats_overview_panel.grid(row=0, column=0, sticky="nsew")
        center_workspace.add(canvas_shell, weight=3)

        trains_shell = ttk.Frame(center_workspace, style="Shell.TFrame")
        self.trains_shell = trains_shell
        self.trains_tab_collapsed = False
        self._trains_restore_sash = None
        trains_shell.columnconfigure(0, weight=1)
        trains_shell.rowconfigure(0, weight=1)
        self.ats_tabs = ttk.Notebook(trains_shell, style="Shell.TNotebook")
        self.ats_tabs.grid(row=0, column=0, sticky="nsew")
        self.ats_tabs.enable_traversal()
        self.ats_tabs.bind("<Button-1>", self._on_trains_tab_click, add="+")
        trains_tab = ttk.Frame(self.ats_tabs, style="Shell.TFrame")
        trains_tab.columnconfigure(0, weight=1)
        trains_tab.rowconfigure(0, weight=1)
        trains_tab.rowconfigure(1, weight=0)
        self.trains_canvas = tk.Canvas(trains_tab, background=APP_THEME["workspace"], highlightthickness=0, height=360)
        self.trains_scrollbar = ttk.Scrollbar(trains_tab, orient="horizontal", command=self.trains_canvas.xview)
        self.trains_scrollable_frame = ttk.Frame(self.trains_canvas, style="Shell.TFrame")
        self.trains_scrollable_frame.bind(
            "<Configure>",
            lambda _event: self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all")),
        )
        self.trains_window = self.trains_canvas.create_window((0, 0), window=self.trains_scrollable_frame, anchor="nw")
        self.trains_canvas.bind("<Configure>", self._resize_train_boards, add="+")
        self.trains_canvas.configure(xscrollcommand=self.trains_scrollbar.set)
        self._bind_train_horizontal_scroll(self.trains_canvas)
        self._bind_train_horizontal_scroll(self.trains_scrollable_frame)
        self.trains_canvas.grid(row=0, column=0, sticky="nsew")
        self.trains_scrollbar.grid(row=1, column=0, sticky="ew")
        self.ats_tabs.add(trains_tab, text="Trains")
        center_workspace.add(trains_shell, weight=2)

        dock_tabs = ttk.Notebook(workspace, style="Shell.TNotebook")
        self.dock_tabs = dock_tabs
        workspace.add(dock_tabs, weight=2)
        infra_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        engineering_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        diagnostics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        analytics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        for tab in (infra_tab, engineering_tab, diagnostics_tab, analytics_tab):
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(0, weight=1)
        self.infrastructure_panel = InfrastructurePanel(infra_tab, self.scale_factor)
        self.infrastructure_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=(int(6 * self.scale_factor), int(3 * self.scale_factor)))
        self.engineering_panel = EngineeringPanel(engineering_tab, self.scale_factor)
        self.engineering_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.diagnostics_panel = DiagnosticsPanel(diagnostics_tab, self.scale_factor)
        self.diagnostics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.analytics_panel = AnalyticsPanel(analytics_tab, self.scale_factor)
        self.analytics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.limits_panel = SpeedLimitsPanel(infra_tab, self.scale_factor)
        dock_tabs.add(infra_tab, text="Infrastructure")
        dock_tabs.add(engineering_tab, text="Engineering")
        dock_tabs.add(diagnostics_tab, text="Diagnostics")
        dock_tabs.add(analytics_tab, text="Analytics")

        button_row = ttk.Frame(self.content, padding=(0, 8, 0, 0), style="Shell.TFrame")
        button_row.grid(row=6, column=0, sticky="ew")
        button_row.grid_columnconfigure(0, weight=1)

        self.panels: Dict[str, TrainPanel] = {}
        self._create_aux_windows()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.engineering_panel.update_data(self.sim)
        self.diagnostics_panel.update_data(self.sim, self.event_log)
        self.analytics_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self.normal_widgets = [status_row, summary_frame, headway_frame, workspace, button_row]
        self.monte_carlo_panel = MonteCarloPanel(self.content, self)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<MouseWheel>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-4>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-5>", self._on_global_mousewheel, add="+")
        self.after(100, self.tick)
        self.after(100, self._update_vietnam_clock)
        self.after_idle(self._fit_workspace_panes)

    def _make_button_group(self, master: tk.Widget, title: str, column: int) -> ttk.Frame:
        group = ttk.Frame(master, padding=(8, 4, 8, 4), style="ToolbarGroup.TFrame")
        group.grid(row=0, column=column, sticky="ew", padx=(0, 8))
        master.columnconfigure(column, weight=1 if title == "Simulation Control" else 0)
        ttk.Label(group, text=title, style="ToolbarTitle.TLabel").grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 2))
        return group

    def _bind_side_toolbar_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_side_scroll_bound", False):
            return
        setattr(widget, "_cbtc_side_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_side_toolbar_mousewheel, add="+")

    def _bind_side_toolbar_tree(self, widget: tk.Widget):
        self._bind_side_toolbar_scroll(widget)
        for child in widget.winfo_children():
            self._bind_side_toolbar_tree(child)

    def _on_side_toolbar_mousewheel(self, event):
        if not hasattr(self, "side_toolbar_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -4
        elif getattr(event, "num", None) == 5:
            delta = 4
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -4 if delta_raw > 0 else 4
        self.side_toolbar_canvas.yview_scroll(delta, "units")
        return "break"

    def _fit_workspace_panes(self):
        if not hasattr(self, "workspace"):
            return
        self.update_idletasks()
        width = max(1, self.workspace.winfo_width())
        height = max(1, getattr(self, "center_workspace", self.workspace).winfo_height())
        left_w = int(210 * self.scale_factor)
        right_w = max(int(390 * self.scale_factor), min(int(520 * self.scale_factor), int(width * 0.28)))
        try:
            self.workspace.sashpos(0, left_w)
            self.workspace.sashpos(1, max(left_w + 480, width - right_w))
        except tk.TclError:
            pass
        try:
            if not getattr(self, "trains_tab_collapsed", False):
                self.center_workspace.sashpos(0, max(int(300 * self.scale_factor), int(height * 0.52)))
        except tk.TclError:
            pass
        self._resize_train_boards()

    def _update_vietnam_clock(self):
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        display_clock_s = self.sim.timetable_display_clock_s() if hasattr(self, "sim") else None
        if display_clock_s is None:
            self.vn_clock_var.set(now.strftime("%H:%M:%S  %Y-%m-%d  UTC+7"))
        else:
            total_s = int(display_clock_s) % (24 * 3600)
            hour = total_s // 3600
            minute = (total_s % 3600) // 60
            second = total_s % 60
            self.vn_clock_var.set(
                f"{hour:02d}:{minute:02d}:{second:02d}  {now:%Y-%m-%d}  UTC+7  x{self.time_scale}"
            )
        self.after(1000, self._update_vietnam_clock)

    def _update_mode_toggle_button(self):
        if not hasattr(self, "mode_toggle_btn"):
            return
        if self.current_workspace_mode == "monte_carlo":
            self.mode_toggle_btn.configure(text="Chế độ mô phỏng")
        else:
            self.mode_toggle_btn.configure(text="Chế độ thống kê")

    def toggle_workspace_mode(self):
        if self.current_workspace_mode == "normal":
            self.show_monte_carlo_mode()
        else:
            self.show_normal_mode()

    def show_monte_carlo_mode(self):
        if self.current_workspace_mode == "monte_carlo":
            return
        self.current_workspace_mode = "monte_carlo"
        self.sim.stop()
        for widget in self.normal_widgets:
            widget.grid_remove()
        self.monte_carlo_panel.grid(row=1, column=0, rowspan=6, sticky="nsew")
        self._update_mode_toggle_button()
        self.status_var.set("Status: chế độ thống kê")

    def show_normal_mode(self):
        if self.current_workspace_mode == "normal":
            return
        self.current_workspace_mode = "normal"
        self.monte_carlo_panel.grid_remove()
        for widget in self.normal_widgets:
            widget.grid()
        self._update_mode_toggle_button()
        self.status_var.set("Status: chế độ mô phỏng")

    def _operation_mode_from_scenario(self) -> str:
        headway = self.scenario.get("headway", {}) if isinstance(self.scenario.get("headway", {}), dict) else {}
        mode = str(headway.get("mode", "off")).lower()
        block_mode = str(getattr(self, "sim", None).block_mode if hasattr(self, "sim") else self.scenario.get("block_mode", "")).lower()
        if block_mode in {"fixed", "fixed_block"}:
            return "1 Fixed-block"
        if mode == "timetable":
            return "3 Timetable"
        if mode == "adaptive":
            return "4 Adaptive tph"
        return "2 Headway target"

    def _operation_mode_key(self) -> str:
        value = self.operation_mode_var.get().strip().lower()
        if value.startswith("1"):
            return "fixed_block"
        if value.startswith("3"):
            return "timetable"
        if value.startswith("4"):
            return "adaptive"
        return "headway_target"

    def _operation_mode_label_for_key(self, key: str) -> str:
        labels = {
            "fixed_block": "1 Fixed-block",
            "headway_target": "2 Headway target",
            "timetable": "3 Timetable",
            "adaptive": "4 Adaptive tph",
        }
        return labels.get(key, "2 Headway target")

    def _active_operation_mode_key(self) -> str:
        if getattr(self.sim, "block_mode", "moving_block") == "fixed_block":
            return "fixed_block"
        mode = str(getattr(self.sim.headway_manager, "mode", "fixed")).lower()
        if mode == "timetable":
            return "timetable"
        if mode == "adaptive":
            return "adaptive"
        return "headway_target"

    def _update_operation_mode_status(self):
        if not hasattr(self, "operation_selected_status_var"):
            return
        selected_key = self._operation_mode_key()
        active_key = self._active_operation_mode_key()
        selected_label = self._operation_mode_label_for_key(selected_key)
        active_label = self._operation_mode_label_for_key(active_key)
        pending = " | Chưa apply" if selected_key != active_key else ""
        self.operation_selected_status_var.set(
            f"Đang chọn: {selected_label} | Sẽ chạy: {selected_label} | Đang chạy: {active_label}{pending}"
        )

    def _refresh_operation_mode_controls(self):
        for widget in (
            getattr(self, "blocks_label", None),
            getattr(self, "blocks_entry", None),
            getattr(self, "target_label", None),
            getattr(self, "target_entry", None),
            getattr(self, "timetable_label", None),
            getattr(self, "timetable_entry", None),
            getattr(self, "timetable_button", None),
            getattr(self, "timetable_set_after_now_button", None),
            getattr(self, "tph_label", None),
            getattr(self, "tph_entry", None),
        ):
            if widget is not None:
                widget.grid_remove()

        mode = self._operation_mode_key()
        self.operation_mechanism_var.set("fixed_block" if mode == "fixed_block" else "moving_block")
        if mode == "fixed_block":
            self.blocks_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.blocks_entry.grid(row=0, column=5, padx=(0, 8), sticky="w")
        elif mode == "headway_target":
            self.target_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.target_entry.grid(row=0, column=5, padx=(0, 8), sticky="w")
        elif mode == "timetable":
            self.timetable_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.timetable_entry.grid(row=0, column=5, columnspan=2, padx=(0, 8), sticky="ew")
            self.timetable_button.grid(row=0, column=7, padx=(0, 8), sticky="w")
            self.timetable_set_after_now_button.grid(row=0, column=8, padx=(0, 8), sticky="w")
        elif mode == "adaptive":
            self.tph_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.tph_entry.grid(row=0, column=5, padx=(0, 8), sticky="w")
        self._update_operation_mode_status()

    def _extract_timetable_seconds(self, payload: Any) -> List[float]:
        if isinstance(payload, dict):
            for key in ("timetable_s", "timetable", "schedule", "departures", "times"):
                if key in payload:
                    return self._extract_timetable_seconds(payload[key])
            return []
        if isinstance(payload, list):
            values: List[float] = []
            for item in payload:
                if isinstance(item, dict):
                    for key in ("time_s", "depart_s", "departure_s", "time", "depart", "departure"):
                        if key in item:
                            try:
                                values.append(float(item[key]))
                                break
                            except (TypeError, ValueError):
                                pass
                else:
                    try:
                        values.append(float(item))
                    except (TypeError, ValueError):
                        pass
            return sorted(value for value in values if value >= 0.0)
        return []

    def _parse_timetable_clock_s(self, value: Any) -> float | None:
        text = str(value or "").strip()
        if not text or text in {"--", "-"}:
            return None
        match = re.search(r"\b(\d{1,2}):(\d{2})(?::(\d{2}))?\b", text)
        if not match:
            return None
        hour = int(match.group(1))
        minute = int(match.group(2))
        second = int(match.group(3) or 0)
        if minute >= 60 or second >= 60:
            return None
        return float(hour * 3600 + minute * 60 + second)

    def _parse_timetable_dwell_s(self, value: Any) -> float | None:
        text = str(value or "").strip().lower()
        if not text or text in {"--", "-"}:
            return None
        match = re.search(r"(\d+(?:\.\d+)?)\s*s", text)
        if match:
            return float(match.group(1))
        try:
            return float(text)
        except ValueError:
            return None

    def _looks_like_train_id(self, text: str) -> bool:
        return re.fullmatch(r"T\d+", text.strip(), flags=re.IGNORECASE) is not None

    def _looks_like_station_id(self, text: str) -> bool:
        return re.fullmatch(r"S\d+", text.strip(), flags=re.IGNORECASE) is not None

    def _normalize_markdown_cell(self, text: str) -> str:
        return re.sub(r"<[^>]+>", " ", text).strip()

    def _append_timetable_record(
        self,
        records: List[Dict[str, Any]],
        train_id: str | None,
        station: str | None,
        arrival: Any,
        dwell: Any,
        departure: Any,
        profile: Any,
        note: Any = "",
    ) -> None:
        if not train_id or not station:
            return
        arrival_clock_s = self._parse_timetable_clock_s(arrival)
        departure_clock_s = self._parse_timetable_clock_s(departure)
        records.append(
            {
                "train_id": str(train_id).strip(),
                "station": str(station).strip(),
                "arrival_text": str(arrival or "").strip(),
                "arrival_clock_s": arrival_clock_s,
                "arrival_time_s": arrival_clock_s,
                "dwell_text": str(dwell or "").strip(),
                "dwell_s": self._parse_timetable_dwell_s(dwell),
                "departure_text": str(departure or "").strip(),
                "departure_clock_s": departure_clock_s,
                "departure_time_s": departure_clock_s,
                "profile": str(profile or "").strip(),
                "note": str(note or "").strip(),
            }
        )

    def _extract_vietnamese_markdown_timetable(self, text: str) -> Tuple[List[float], List[Dict[str, Any]]]:
        records: List[Dict[str, Any]] = []
        current_train: str | None = None
        plain_lines: List[str] = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            if "|" in line:
                cells = [self._normalize_markdown_cell(cell) for cell in line.strip("|").split("|")]
                cells = [cell for cell in cells if cell]
                if not cells or all(set(cell) <= {"-", ":"} for cell in cells):
                    continue
                header_text = " ".join(cells).lower()
                if "tàu" in header_text or "arrival" in header_text or "giờ" in header_text:
                    continue
                if len(cells) >= 6:
                    if self._looks_like_train_id(cells[0]):
                        current_train = cells[0].upper()
                        offset = 1
                    else:
                        offset = 0
                    if len(cells) - offset >= 5 and self._looks_like_station_id(cells[offset]):
                        note = " ".join(cells[offset + 5 :]) if len(cells) - offset > 5 else ""
                        self._append_timetable_record(
                            records,
                            current_train,
                            cells[offset],
                            cells[offset + 1],
                            cells[offset + 2],
                            cells[offset + 3],
                            cells[offset + 4],
                            note,
                        )
                continue
            plain_lines.append(line)

        idx = 0
        while idx < len(plain_lines):
            line = plain_lines[idx]
            if self._looks_like_train_id(line):
                current_train = line.upper()
                idx += 1
                continue
            if self._looks_like_station_id(line) and idx + 4 < len(plain_lines):
                station = line
                arrival = plain_lines[idx + 1]
                dwell = plain_lines[idx + 2]
                departure = plain_lines[idx + 3]
                profile = plain_lines[idx + 4]
                note_parts: List[str] = []
                idx += 5
                while idx < len(plain_lines) and not self._looks_like_train_id(plain_lines[idx]) and not self._looks_like_station_id(plain_lines[idx]):
                    note_parts.append(plain_lines[idx])
                    idx += 1
                self._append_timetable_record(records, current_train, station, arrival, dwell, departure, profile, " ".join(note_parts))
                continue
            idx += 1

        first_departures: Dict[str, float] = {}
        for record in records:
            departure_s = record.get("departure_time_s")
            train_id = str(record.get("train_id", ""))
            if departure_s is None or not train_id:
                continue
            first_departures.setdefault(train_id, float(departure_s))
        if not first_departures:
            return [], records
        origin_s = min(first_departures.values())
        timetable_s = sorted(max(0.0, value - origin_s) for value in first_departures.values())
        for record in records:
            for key in ("arrival_time_s", "departure_time_s"):
                if record.get(key) is not None:
                    record[key] = max(0.0, float(record[key]) - origin_s)
        return timetable_s, records

    def _vietnam_clock_seconds(self) -> float:
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        return float(now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 1_000_000.0)

    def _clock_delay_from_now_s(self, clock_s: float, now_clock_s: float) -> float:
        day_s = 24.0 * 3600.0
        return (float(clock_s) - float(now_clock_s)) % day_s

    def _wall_clock_timetable_values(
        self,
        records: List[Dict[str, Any]],
        fallback_values: List[float],
        now_clock_s: float | None = None,
    ) -> Tuple[List[float], List[Dict[str, Any]]]:
        if not records:
            return fallback_values, records
        now_clock_s = self._vietnam_clock_seconds() if now_clock_s is None else float(now_clock_s)
        first_departures: Dict[str, float] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            clock_s = record.get("departure_clock_s")
            if train_id and clock_s is not None:
                first_departures.setdefault(train_id, float(clock_s))
        if not first_departures:
            return fallback_values, records

        values = sorted(self._clock_delay_from_now_s(clock_s, now_clock_s) for clock_s in first_departures.values())
        adjusted_records: List[Dict[str, Any]] = []
        for record in records:
            adjusted = dict(record)
            for clock_key, time_key in (("arrival_clock_s", "arrival_time_s"), ("departure_clock_s", "departure_time_s")):
                clock_s = adjusted.get(clock_key)
                if clock_s is not None:
                    adjusted[time_key] = self._clock_delay_from_now_s(float(clock_s), now_clock_s)
            adjusted_records.append(adjusted)
        return values, adjusted_records

    def _format_timetable_clock_s(self, clock_s: float) -> str:
        total_s = int(round(float(clock_s))) % int(24.0 * 3600.0)
        hour = total_s // 3600
        minute = (total_s % 3600) // 60
        second = total_s % 60
        return f"{hour:02d}:{minute:02d}:{second:02d}"

    def set_timetable_after_now(self):
        headway = dict(self.scenario.get("headway", {}) or {})
        values = [float(value) for value in (headway.get("timetable_s", []) or [])]
        records = [dict(record) for record in (headway.get("timetable_records", []) or [])]
        if not values and not records:
            self.status_var.set("Status: load a timetable before setting +1.5 min")
            return

        now_clock_s = self._vietnam_clock_seconds()
        target_first_departure_s = 90.0
        first_departures: Dict[str, float] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            departure_s = record.get("departure_time_s")
            if train_id and departure_s is not None:
                first_departures.setdefault(train_id, float(departure_s))
        if first_departures:
            origin_s = min(first_departures.values())
        elif values:
            origin_s = min(values)
        else:
            self.status_var.set("Status: timetable has no departure times to shift")
            return

        if values:
            shifted_values = sorted(max(0.0, target_first_departure_s + value - origin_s) for value in values)
        else:
            shifted_values = sorted(
                max(0.0, target_first_departure_s + value - origin_s)
                for value in first_departures.values()
            )
        shifted_records: List[Dict[str, Any]] = []
        for record in records:
            shifted = dict(record)
            for clock_key, time_key, text_key in (
                ("arrival_clock_s", "arrival_time_s", "arrival_text"),
                ("departure_clock_s", "departure_time_s", "departure_text"),
            ):
                old_time_s = shifted.get(time_key)
                if old_time_s is None:
                    continue
                new_time_s = max(0.0, target_first_departure_s + float(old_time_s) - origin_s)
                new_clock_s = (now_clock_s + new_time_s) % (24.0 * 3600.0)
                shifted[time_key] = new_time_s
                shifted[clock_key] = new_clock_s
                shifted[text_key] = self._format_timetable_clock_s(new_clock_s)
            shifted_records.append(shifted)

        headway["mode"] = "timetable"
        headway["timetable_s"] = shifted_values
        if shifted_records:
            headway["timetable_records"] = shifted_records
            headway["timetable_wall_clock"] = True
        headway["timetable_loaded_clock_s"] = now_clock_s
        self.scenario["headway"] = headway
        self.sim.scenario["headway"] = deepcopy(headway)
        self.operation_mode_var.set("3 Timetable")
        self._refresh_operation_mode_controls()
        self.on_reset_simulation()
        first_clock = self._format_timetable_clock_s(now_clock_s + target_first_departure_s)
        self.status_var.set(f"Status: timetable first departure set to {first_clock} (+1.5 min) and reset simulation")

    def load_timetable_file(self):
        path = filedialog.askopenfilename(
            title="Load timetable",
            filetypes=(("Timetable files", "*.yaml *.yml *.md"), ("All files", "*.*")),
        )
        if not path:
            return
        self.timetable_file_var.set(path)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                text = fh.read()
            if path.lower().endswith((".yaml", ".yml")):
                values = self._extract_timetable_seconds(yaml.safe_load(text) or {})
            else:
                timetable_loaded_clock_s = self._vietnam_clock_seconds()
                values, records = self._extract_vietnamese_markdown_timetable(text)
                if not values:
                    values = []
                    for token in text.replace(",", " ").replace("|", " ").split():
                        try:
                            values.append(float(token))
                        except ValueError:
                            continue
                    values = sorted(value for value in values if value >= 0.0)
                    records = []
                else:
                    values, records = self._wall_clock_timetable_values(records, values, now_clock_s=timetable_loaded_clock_s)
            headway = dict(self.scenario.get("headway", {}) or {})
            headway["mode"] = "timetable"
            headway["timetable_s"] = values
            headway["timetable_file"] = path
            if path.lower().endswith(".md"):
                headway["timetable_records"] = records
                headway["timetable_wall_clock"] = True
                headway["timetable_loaded_clock_s"] = timetable_loaded_clock_s
            self.scenario["headway"] = headway
            self.sim.scenario["headway"] = deepcopy(headway)
            self.operation_mode_var.set("3 Timetable")
            self._refresh_operation_mode_controls()
            self.on_reset_simulation()
            self.status_var.set(f"Status: loaded timetable with {len(values)} train departures and reset simulation")
        except Exception as exc:
            self.status_var.set(f"Status: failed to load timetable: {exc}")

    def reload_headway_block_values(self):
        self.operation_mode_var.set(self._operation_mode_from_scenario())
        headway = self.scenario.get("headway", {}) if isinstance(self.scenario.get("headway", {}), dict) else {}
        self.headway_target_var.set(str(headway.get("target_headway_s", 180.0)))
        self.timetable_file_var.set(str(headway.get("timetable_file", "")))
        adaptive = headway.get("adaptive", {}) if isinstance(headway.get("adaptive", {}), dict) else {}
        adaptive_target_s = float(adaptive.get("min_headway_s", headway.get("target_headway_s", 180.0)) or 180.0)
        self.adaptive_tph_var.set(f"{3600.0 / adaptive_target_s:.1f}" if adaptive_target_s > 0.0 else "20")
        self.blocks_per_section_var.set(str(self.scenario.get("capacity_baseline", {}).get("blocks_per_section", 4)))
        self._refresh_operation_mode_controls()
        self.status_var.set("Status: operation mode values reloaded")

    def apply_headway_block_settings(self):
        headway = dict(self.scenario.get("headway", {}) or {})
        mode = self._operation_mode_key()
        if mode == "fixed_block":
            headway["mode"] = "off"
            self.scenario["block_mode"] = "fixed_block"
        elif mode == "headway_target":
            headway["mode"] = "fixed"
            try:
                headway["target_headway_s"] = max(1.0, float(self.headway_target_var.get()))
            except ValueError:
                headway["target_headway_s"] = 180.0
                self.headway_target_var.set("180.0")
            self.scenario["block_mode"] = "moving_block"
        elif mode == "timetable":
            headway["mode"] = "timetable"
            headway["timetable_file"] = self.timetable_file_var.get()
            self.scenario["block_mode"] = "moving_block"
        else:
            headway["mode"] = "adaptive"
            try:
                tph = max(0.1, float(self.adaptive_tph_var.get()))
            except ValueError:
                tph = 20.0
                self.adaptive_tph_var.set("20")
            target_s = 3600.0 / tph
            headway["target_headway_s"] = target_s
            adaptive = dict(headway.get("adaptive", {}) or {})
            adaptive["min_headway_s"] = target_s
            headway["adaptive"] = adaptive
            self.scenario["block_mode"] = "moving_block"
        self.scenario["headway"] = headway
        capacity = dict(self.scenario.get("capacity_baseline", {}) or {})
        try:
            capacity["blocks_per_section"] = max(1, int(float(self.blocks_per_section_var.get())))
        except ValueError:
            capacity["blocks_per_section"] = 4
            self.blocks_per_section_var.set("4")
        self.scenario["capacity_baseline"] = capacity
        self.on_reset_simulation()
        self.ats_overview_panel.update_data(self.sim)
        self.status_var.set("Status: applied operation mode and reset simulation")

    def _configure_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Scale font sizes based on DPI scaling
        base_font_size = int(9 * self.scale_factor)
        button_font_size = int(11 * self.scale_factor)
        history_button_font_size = int(15 * self.scale_factor)
        header_font_size = int(16 * self.scale_factor)
        section_font_size = int(11 * self.scale_factor)
        status_font_size = int(9 * self.scale_factor)
        muted_font_size = int(8 * self.scale_factor)
        card_title_font_size = int(11 * self.scale_factor)
        tab_font_size = int(10 * self.scale_factor)

        style.configure("TFrame", background=APP_THEME["workspace"])
        style.configure("TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", base_font_size))
        style.configure("Shell.TFrame", background=APP_THEME["workspace"])
        style.configure("Panel.TFrame", background=APP_THEME["workspace"])
        style.configure("Card.TFrame", background=APP_THEME["card"], relief="solid", borderwidth=1)
        style.configure("SubCard.TFrame", background=APP_THEME["card_alt"])
        style.configure("ToolbarGroup.TFrame", background=APP_THEME["panel"], relief="ridge", borderwidth=1)
        style.configure("Clock.TFrame", background=APP_THEME["accent"], relief="solid", borderwidth=1)
        style.configure("TLabelframe", background=APP_THEME["panel"], foreground=APP_THEME["text"], bordercolor=APP_THEME["border"], lightcolor=APP_THEME["border"], darkcolor=APP_THEME["border"])
        style.configure("TLabelframe.Label", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Vertical.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("Horizontal.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("HeaderTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", header_font_size, "bold"))
        style.configure("SectionTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", section_font_size, "bold"))
        style.configure("CardTitle.TLabel", background=APP_THEME["card"], foreground=APP_THEME["accent"], font=("Consolas", card_title_font_size, "bold"))
        style.configure("ToolbarTitle.TLabel", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Shell.TNotebook", background=APP_THEME["workspace"], borderwidth=0)
        style.configure(
            "Shell.TNotebook.Tab",
            padding=(int(10 * self.scale_factor), int(6 * self.scale_factor)),
            font=("Consolas", tab_font_size, "bold"),
            background=APP_THEME["panel_alt"],
            foreground=APP_THEME["muted"],
        )
        style.map(
            "Shell.TNotebook.Tab",
            background=[("selected", APP_THEME["card"]), ("active", APP_THEME["button_hover"])],
            foreground=[("selected", APP_THEME["accent"]), ("active", APP_THEME["text"])],
        )
        style.configure("Muted.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["muted"], font=("Consolas", muted_font_size))
        style.configure("Status.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", status_font_size))
        style.configure("Clock.TLabel", background=APP_THEME["accent"], foreground="#fff8ed", font=("Consolas", int(15 * self.scale_factor), "bold"))
        style.configure("ClockSmall.TLabel", background=APP_THEME["accent"], foreground="#ffe7c7", font=("Consolas", int(7 * self.scale_factor), "bold"))
        style.configure(
            "TEntry",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor=APP_THEME["button_hover"],
            darkcolor=APP_THEME["border"],
        )
        style.configure(
            "TCombobox",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            background=APP_THEME["button"],
            bordercolor=APP_THEME["border"],
            arrowcolor=APP_THEME["border"],
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", APP_THEME["log_bg"])],
            background=[("active", APP_THEME["button_hover"]), ("readonly", APP_THEME["button"])],
        )
        style.configure(
            "TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor="#ffd979",
            darkcolor="#b85c2d",
            relief="raised",
        )
        style.map(
            "TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Accent.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["accent"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Accent.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#d8782d")],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Active.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="ridge",
        )
        style.map(
            "Active.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", "#ffe08a"), ("!disabled", APP_THEME["button_active"])],
            relief=[("pressed", "sunken"), ("!pressed", "ridge")],
        )
        style.configure(
            "History.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "History.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "RunActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["run_active"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "RunActive.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#ffc15c"), ("!disabled", APP_THEME["run_active"])],
            foreground=[("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["pause_active"],
            foreground="#1f2730",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", "#1f2730")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseHistoryActive.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["pause_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseHistoryActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Danger.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["danger"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["danger_pressed"],
            relief="raised",
        )
        style.map(
            "Danger.TButton",
            background=[("pressed", APP_THEME["danger_pressed"]), ("active", "#e85d4f"), ("!disabled", APP_THEME["danger"])],
            foreground=[("disabled", "#f0caca"), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Inactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["muted"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Inactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["muted"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "StartInactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "StartInactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )

    def _reset_runtime_buffers(self):
        self.time_history.clear()
        self.position_history = {train.id: deque(maxlen=240) for train in self.sim.trains}
        self.event_log.clear()
        self.prev_train_snapshot = {}

    def _on_mousewheel(self, event):
        """Handle mouse wheel scroll events (Windows and Linux)."""
        # Windows uses event.delta (positive = up, negative = down)
        # Linux uses event.num (4 = up, 5 = down)
        if event.num == 4:  # Linux scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.num == 5:  # Linux scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        elif event.delta > 0:  # Windows scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.delta < 0:  # Windows scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        return "break"  # Prevent default scrolling

    def _on_global_mousewheel(self, event):
        widget = event.widget
        if isinstance(widget, tk.Text):
            return None
        try:
            if widget.winfo_toplevel() is not self:
                return None
        except tk.TclError:
            return None
        if getattr(event, "num", None) == 4:
            delta = -3
        elif getattr(event, "num", None) == 5:
            delta = 3
        else:
            delta = -3 * int(event.delta / 120) if getattr(event, "delta", 0) else 0
        if delta:
            self.scroll_canvas.yview_scroll(delta, "units")
        return None

    def _record_runtime_history(self):
        self.time_history.append(self.sim.sim_time_s)
        for train in self.sim.trains:
            if train.id not in self.position_history:
                self.position_history[train.id] = deque(maxlen=240)
            self.position_history[train.id].append(train.pos)

    def _update_event_log(self):
        sim_stamp = f"{self.sim.sim_time_s:7.1f}s"
        for train in self.sim.trains:
            for record in train.pop_pending_events():
                curves = record["curves"]
                extra = ""
                if record["event"] == "JOG_PROFILE_TRACE":
                    extra = (
                        f" src={record.get('reason', '--')}"
                        f" target={record.get('jog_target_speed_kmh', 0.0):.2f}km/h"
                        f" next={record.get('next_speed_kmh', 0.0):.2f}km/h"
                        f" accel={record.get('commanded_accel_ms2', 0.0):+.2f}"
                        f" phase={record.get('profile_phase', '--')}"
                    )
                self.event_log.appendleft(
                    f"{record['sim_time']:7.1f}s  {record['train_id']}  {record['event']}"
                    f" reason={record['reason'] or '--'} stop={record['stop_id']}"
                    f" pos={record['current_pos']:.2f} target={record['stop_target_pos']:.2f}"
                    f" rem={record['remaining']:.2f} err={record['stop_error']:.2f}"
                    f" v={record['speed_kmh']:.2f}/{record['vital_speed_kmh']:.2f}km/h"
                    f" eoa={record.get('eoa_m', 0.0):.2f} deoa={record.get('distance_to_eoa', 0.0):.2f}"
                    f" station={record.get('station_state', '--')} line={record.get('assigned_line', '--')}"
                    f" dwell={record.get('dwell_remaining', 0.0):.1f}s"
                    f" P/W/SBI/EBI/EBD={curves['P']:.1f}/{curves['W']:.1f}/{curves['SBI']:.1f}/{curves['EBI']:.1f}/{curves['EBD']:.1f}"
                    f" mode={record['curve_mode']} jog={record['jog_state']}{extra}"
                )
            snapshot = (
                train.atp_action,
                train.atp_alert,
                train.safe_packet_valid,
                train.door_authorized,
                train.commanded_stop,
            )
            previous = self.prev_train_snapshot.get(train.id)
            if previous is None:
                self.prev_train_snapshot[train.id] = snapshot
                continue
            if previous[0] != snapshot[0]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  ATP action {previous[0] or 'NONE'} -> {snapshot[0] or 'NONE'}")
            if previous[1] != snapshot[1]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  alert {previous[1]} -> {snapshot[1]}")
            if previous[2] != snapshot[2]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  DCS {'restored' if snapshot[2] else 'timeout / trip'}")
            if previous[3] != snapshot[3]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  door {'authorized' if snapshot[3] else 'locked'}")
            if previous[4] != snapshot[4]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  scheduled stop {'armed' if snapshot[4] else 'released'}")
            self.prev_train_snapshot[train.id] = snapshot

    def _create_aux_windows(self):
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.child_windows = {}

        self.rebuild_train_panels()
        self._update_control_track_profile()

    def _update_control_track_profile(self):
        panel = getattr(self, "control_panel", None)
        if panel is not None:
            panel.update_track_profile(self.sim.track_profile)

    def rebuild_train_panels(self):
        # Clear existing panels
        for panel in self.panels.values():
            panel.destroy()
        self.panels = {}
        self.position_history = {train.id: deque(maxlen=240) for train in self.sim.trains}

        # Clear the scrollable frame
        for widget in self.trains_scrollable_frame.winfo_children():
            widget.destroy()

        for i, train in enumerate(self.sim.trains):
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def sync_train_panels(self):
        current_ids = {train.id for train in self.sim.trains}
        for train_id, panel in list(self.panels.items()):
            if train_id not in current_ids:
                wrapper = panel.master
                panel.destroy()
                try:
                    wrapper.destroy()
                except tk.TclError:
                    pass
                self.panels.pop(train_id, None)
                self.position_history.pop(train_id, None)

        for train in self.sim.trains:
            if train.id in self.panels:
                self.panels[train.id].set_track_range(self.sim.track_max_m)
                continue
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
            if train.id not in self.position_history:
                self.position_history[train.id] = deque(maxlen=240)
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def _train_board_dimensions(self) -> Tuple[int, int]:
        canvas_w = max(1, int(getattr(self, "trains_canvas", self).winfo_width() or 0))
        canvas_h = max(1, int(getattr(self, "trains_canvas", self).winfo_height() or 0))
        count = max(1, len(getattr(self.sim, "trains", [])))
        visible_count = min(count, 4)
        gap_px = int(10 * self.scale_factor)
        available_w = max(1, canvas_w - gap_px * max(0, visible_count - 1) - int(16 * self.scale_factor))
        board_w = max(int(360 * self.scale_factor), int(available_w / visible_count))
        board_h = max(int(220 * self.scale_factor), canvas_h - int(8 * self.scale_factor))
        return board_w, board_h

    def _resize_train_boards(self, _event=None):
        if not hasattr(self, "trains_scrollable_frame"):
            return
        board_w, board_h = self._train_board_dimensions()
        for panel in self.panels.values():
            wrapper = panel.master
            try:
                wrapper.config(width=board_w, height=board_h)
                panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            except tk.TclError:
                continue
        self.trains_scrollable_frame.update_idletasks()
        try:
            self.trains_canvas.itemconfigure(self.trains_window, height=max(1, self.trains_canvas.winfo_height()))
        except tk.TclError:
            pass
        self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all"))

    def _bind_train_horizontal_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_train_scroll_bound", False):
            return
        setattr(widget, "_cbtc_train_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Shift-MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_train_horizontal_mousewheel, add="+")

    def _bind_train_horizontal_scroll_tree(self, widget: tk.Widget):
        self._bind_train_horizontal_scroll(widget)
        for child in widget.winfo_children():
            self._bind_train_horizontal_scroll_tree(child)

    def _on_train_horizontal_mousewheel(self, event):
        if not hasattr(self, "trains_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -1
        elif getattr(event, "num", None) == 5:
            delta = 1
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -1 if delta_raw > 0 else 1
        if delta:
            self.trains_canvas.xview_scroll(delta * 12, "units")
        return "break"

    def _on_trains_tab_click(self, event):
        try:
            tab_index = self.ats_tabs.index(f"@{event.x},{event.y}")
        except tk.TclError:
            return None
        if self.ats_tabs.tab(tab_index, "text") != "Trains":
            return None
        if tab_index == self.ats_tabs.index("current"):
            self._toggle_trains_tab()
            return "break"
        return None

    def _toggle_trains_tab(self):
        if not hasattr(self, "center_workspace"):
            return
        try:
            total_h = max(1, self.center_workspace.winfo_height())
            if self.trains_tab_collapsed:
                target = self._trains_restore_sash or int(total_h * 0.55)
                self.center_workspace.sashpos(0, max(180, min(total_h - 180, target)))
                self.trains_tab_collapsed = False
            else:
                self._trains_restore_sash = self.center_workspace.sashpos(0)
                self.center_workspace.sashpos(0, max(120, total_h - int(34 * self.scale_factor)))
                self.trains_tab_collapsed = True
        except tk.TclError:
            return
        self.after_idle(self._resize_train_boards)

    def _rebuild_train_fault_buttons(self):
        if not hasattr(self, "emergency_fault_frame"):
            return
        for frame in (self.emergency_fault_frame, self.atp_fault_frame, self.ato_fault_frame):
            for child in frame.winfo_children():
                child.destroy()
        self.train_fault_buttons = {}
        for row, train in enumerate(self.sim.trains):
            emergency_btn = ttk.Button(
                self.emergency_fault_frame,
                text=f"{train.id}: Stop",
                command=lambda train_id=train.id: self.instant_stop_train(train_id),
                style="Danger.TButton",
            )
            emergency_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(emergency_btn)
            atp_btn = ttk.Button(
                self.atp_fault_frame,
                text=f"{train.id}: ATP",
                command=lambda train_id=train.id: self.toggle_train_fault(train_id, "ATP"),
            )
            atp_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(atp_btn)
            ato_btn = ttk.Button(
                self.ato_fault_frame,
                text=f"{train.id}: ATO",
                command=lambda train_id=train.id: self.toggle_train_fault(train_id, "ATO"),
            )
            ato_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(ato_btn)
            self.train_fault_buttons[train.id] = {"emergency": emergency_btn, "atp": atp_btn, "ato": ato_btn}
        self._bind_side_toolbar_tree(self.emergency_fault_frame)
        self._bind_side_toolbar_tree(self.atp_fault_frame)
        self._bind_side_toolbar_tree(self.ato_fault_frame)

    def _hide_all_child_windows(self):
        for window in self.child_windows.values():
            try:
                window.withdraw()
            except tk.TclError:
                pass

    def _any_child_visible(self) -> bool:
        return any(str(window.state()) != "withdrawn" for window in self.child_windows.values())

    def _hide_window(self, window: tk.Toplevel):
        try:
            window.withdraw()
        except tk.TclError:
            return
        if not self._any_child_visible():
            try:
                self.deiconify()
            except tk.TclError:
                pass

    def _show_window(self, key: str):
        window = self.child_windows.get(key)
        if window is None:
            return
        self._hide_all_child_windows()
        try:
            self.withdraw()
        except tk.TclError:
            pass
        window.deiconify()
        window.lift()
        window.focus_force()
        try:
            window.state("zoomed")
        except tk.TclError:
            screen_w = window.winfo_screenwidth()
            screen_h = window.winfo_screenheight()
            window.geometry(f"{screen_w}x{screen_h}+0+0")

    def _update_root_summary(self):
        dcs_ok = sum(1 for train in self.sim.trains if train.safe_packet_valid)
        emergency = sum(1 for train in self.sim.trains if train.atp_action == "EBI")
        moving = sum(1 for train in self.sim.trains if train.speed > 0.1)
        total_trains = len(self.sim.trains)
        self.summary_var.set(
            f"Use the ATS main tabs for full line monitoring and systems functions."
            f"  Trains={total_trains}  moving={moving}  DCS healthy={dcs_ok}/{total_trains}  EBI active={emergency}"
        )

    def _on_close(self):
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.destroy()

    def on_load_scenario(self):
        selected = filedialog.askopenfilename(
            title="Load Scenario YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
            initialdir=str(DEFAULT_SCENARIO_PATH.parent),
        )
        if not selected:
            return
        was_running = self.sim.running
        self.sim.stop()
        try:
            self.scenario = load_scenario(selected)
        except Exception as exc:
            self.status_var.set(f"Status: failed to load scenario ({exc})")
            if was_running:
                self.sim.start()
            return
        self.sim.load_scenario(self.scenario)
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Scenario: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.edit_undo_stack.clear()
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()
        self.rebuild_train_panels()
        self.status_var.set(f"Status: loaded scenario from {selected}")
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()

    def on_start(self):
        self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: running")

    def on_stop(self):
        self.sim.stop()
        self.sim_paused = True
        self._update_run_pause_buttons()
        self.status_var.set("Status: paused")

    def on_reset_simulation(self):
        was_running = self.sim.running
        self.sim.stop()
        self.sim.load_scenario(self.scenario)
        self.sim.set_timetable_clock_scale(self.time_scale)
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.edit_undo_stack.clear()
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()
        self.rebuild_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self._update_operation_mode_status()
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: simulation reset")

    def set_time_scale(self, scale: int):
        self.time_scale = max(1, int(scale))
        self.sim.set_timetable_clock_scale(self.time_scale)
        self._update_time_scale_buttons()
        self.status_var.set(f"Status: time scale x{self.time_scale}")

    def _update_time_scale_buttons(self):
        for scale, button in self.time_scale_buttons.items():
            button.configure(style="Active.TButton" if scale == self.time_scale else "TButton")

    def _update_run_pause_buttons(self):
        if hasattr(self, "start_btn"):
            self.start_btn.configure(style="RunActive.TButton" if self.sim.running else "StartInactive.TButton")
        if hasattr(self, "stop_btn"):
            pause_active = self.sim_paused and not self.sim.running
            self.stop_btn.configure(style="PauseHistoryActive.TButton" if pause_active else "History.TButton")

    def _editable_snapshot(self) -> Dict[str, Any]:
        data = scenario_to_yaml_data(self.sim, self.scenario)
        snapshot = normalize_scenario(data, self.scenario.get("source_path"))
        snapshot["source_path"] = self.scenario.get("source_path")
        snapshot["tsr_zones"] = deepcopy(self.sim.tsr_zones)
        return snapshot

    def _push_edit_undo(self):
        self.edit_undo_stack.append(self._editable_snapshot())
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()

    def _restore_edit_snapshot(self, snapshot: Dict[str, Any]):
        was_running = self.sim.running
        self.sim.stop()
        scenario_snapshot = deepcopy(snapshot)
        tsr_zones = deepcopy(scenario_snapshot.pop("tsr_zones", []))
        self.scenario = scenario_snapshot
        self.sim.load_scenario(self.scenario)
        self.sim.tsr_zones = tsr_zones
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Scenario: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.rebuild_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        if was_running:
            self.sim.start()

    def _update_edit_history_buttons(self):
        if hasattr(self, "undo_edit_btn"):
            self.undo_edit_btn.configure(state=("normal" if self.edit_undo_stack else "disabled"))
        if hasattr(self, "redo_edit_btn"):
            self.redo_edit_btn.configure(state=("normal" if self.edit_redo_stack else "disabled"))

    def undo_canvas_edit(self):
        if not self.edit_undo_stack:
            return
        self.edit_redo_stack.append(self._editable_snapshot())
        snapshot = self.edit_undo_stack.pop()
        self._restore_edit_snapshot(snapshot)
        self._update_edit_history_buttons()
        self.status_var.set("Status: canvas edit undone")

    def redo_canvas_edit(self):
        if not self.edit_redo_stack:
            return
        self.edit_undo_stack.append(self._editable_snapshot())
        snapshot = self.edit_redo_stack.pop()
        self._restore_edit_snapshot(snapshot)
        self._update_edit_history_buttons()
        self.status_var.set("Status: canvas edit redone")

    def on_export_report(self):
        path = save_simulation_report(self.sim)
        self.status_var.set(f"Status: exported report to {path}")

    def on_save_scenario(self):
        target_path = self.scenario.get("source_path")
        if not target_path:
            target_path = filedialog.asksaveasfilename(
                title="Save Scenario YAML",
                filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
                defaultextension=".yaml",
                initialdir=str(DEFAULT_SCENARIO_PATH.parent),
            )
            if not target_path:
                return
        try:
            path = save_scenario_file(self.sim, self.scenario, target_path)
        except Exception as exc:
            self.status_var.set(f"Status: failed to save scenario ({exc})")
            return
        self.scenario["source_path"] = str(path)
        self.status_var.set(f"Status: saved scenario to {path}")

    def open_add_element_dialog(self):
        AddElementDialog(self, self.add_ats_element)

    def _track_has_psr_coverage(self, start_m: float, end_m: float) -> bool:
        return self._first_psr_gap(start_m, end_m) is None

    def _first_psr_gap(self, start_m: float, end_m: float) -> Tuple[float, float] | None:
        if end_m <= start_m:
            return None
        cursor = start_m
        for seg_start, seg_end, _gradient, _psr in sorted(self.sim.track_profile, key=lambda item: item[0]):
            if seg_end <= cursor:
                continue
            if seg_start > cursor + 1e-6:
                return cursor, min(seg_start, end_m)
            cursor = max(cursor, min(seg_end, end_m))
            if cursor >= end_m - 1e-6:
                return None
        return cursor, end_m

    def _clear_pending_line_extension(self):
        self.pending_line_extension = None
        self.pending_station_prompt_after_limit = False

    def _open_extension_speed_limit_dialog(self, start_m: float, end_m: float):
        AddElementDialog(
            self,
            self.add_ats_element,
            initial_element="Segment PSR/TSR",
            initial_values={
                "mode": "PSR",
                "start_m": f"{start_m:.0f}",
                "end_m": f"{end_m:.0f}",
                "speed_kmh": "80",
            },
            locked_element=True,
            title="Add PSR for Uncovered Line",
            on_cancel=self._clear_pending_line_extension,
        )

    def _open_extension_station_dialog(self, _start_m: float, end_m: float):
        AddElementDialog(
            self,
            self.add_ats_element,
            initial_element="Station",
            initial_values={
                "name": f"STATION_{int(round(end_m))}",
                "pos_m": f"{end_m:.0f}",
                "length_m": "160",
                "capacity": "3",
                "dwell_s": "30",
            },
            locked_element=True,
            title="Add Station for Extended Line",
        )

    def on_ats_element_selected(self, element_key: str):
        self.status_var.set(f"Status: selected {element_key}")

    def _selected_source_index(self) -> int | None:
        element_key = self.ats_overview_panel.selected_element
        if element_key:
            kind, index = self.ats_overview_panel._element_lookup.get(element_key, ("", -1))
            if kind == "source_train" and 0 <= index < len(self.sim.source_trains):
                return index
        if self.sim.source_trains:
            return 0
        return None

    def _set_source_train_count(self, index: int, count: int):
        if index < 0 or index >= len(self.sim.source_trains):
            return
        self._push_edit_undo()
        source = self.sim.source_trains[index]
        count = max(0, int(count))
        source["capacity"] = max(1, count)
        source["total_trains"] = count
        self.sim._sync_source_train_count(index)
        self.sync_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self.status_var.set(f"Status: source {source.get('name', index)} trains={count}")

    def add_train_to_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: add a Source Train first")
            return
        source = self.sim.source_trains[index]
        current = int(source.get("total_trains", source.get("capacity", 0)))
        self._set_source_train_count(index, current + 1)

    def remove_train_from_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: no Source Train to remove from")
            return
        source = self.sim.source_trains[index]
        current = int(source.get("total_trains", source.get("capacity", 0)))
        self._set_source_train_count(index, current - 1)

    def open_edit_element_dialog(self, kind: str, index: int):
        element_type, values = self._edit_dialog_data(kind, index)
        if element_type is None:
            return
        AddElementDialog(
            self,
            lambda selected_type, data: self.edit_ats_element(kind, index, selected_type, data),
            initial_element=element_type,
            initial_values=values,
            locked_element=True,
            submit_text="Save",
            title="Edit ATS Element",
        )

    def _edit_dialog_data(self, kind: str, index: int) -> Tuple[str | None, Dict[str, str]]:
        if kind == "station" and 0 <= index < len(self.sim.scheduled_stops):
            stop = self.sim.scheduled_stops[index]
            return "Station", {
                "name": str(stop.get("name", "")),
                "pos_m": str(stop.get("pos_m", 0.0)),
                "length_m": str(stop.get("length_m", 160.0)),
                "capacity": str(stop.get("capacity", 3)),
                "dwell_s": str(stop.get("dwell_s", 30.0)),
            }
        if kind == "track_segment" and 0 <= index < len(self.sim.track_profile):
            start, end, gradient, psr = self.sim.track_profile[index]
            return "PSR Segment", {
                "start_m": str(start),
                "end_m": str(end),
                "gradient": str(gradient),
                "psr_kmh": str(psr),
            }
        if kind == "tsr" and 0 <= index < len(self.sim.tsr_zones):
            zone = self.sim.tsr_zones[index]
            return "TSR", {
                "start_m": str(zone.get("start", 0.0)),
                "end_m": str(zone.get("end", 0.0)),
                "speed_kmh": str(zone.get("speed", 0.0)),
            }
        if kind == "line_condition" and 0 <= index < len(self.sim.line_conditions):
            condition = self.sim.line_conditions[index]
            return "Line Condition", {
                "start_m": str(condition.get("start", 0.0)),
                "end_m": str(condition.get("end", 0.0)),
                "condition": str(condition.get("condition", "dry")),
            }
        if kind == "source_train" and 0 <= index < len(self.sim.source_trains):
            source = self.sim.source_trains[index]
            return "Source Train", {
                "name": str(source.get("name", "SRC")),
                "capacity": str(source.get("capacity", 2)),
            }
        return None, {}

    def delete_selected_element(self):
        element_key = self.ats_overview_panel.selected_element
        if not element_key:
            self.status_var.set("Status: select an ATS element to delete")
            return
        kind, index = self.ats_overview_panel._element_lookup.get(element_key, ("", -1))
        try:
            if kind == "station" and 0 <= index < len(self.sim.scheduled_stops):
                self._push_edit_undo()
                removed = self.sim.scheduled_stops.pop(index)
                for train in self.sim.trains:
                    train.scheduled_stops = self.sim.scheduled_stops
                    if train.active_scheduled_stop is not None and self.sim._station_index_for_stop(train.active_scheduled_stop) is None:
                        train.active_scheduled_stop = None
                        train.commanded_stop = False
                        train.station_lane = None
                self.sim._sync_station_route_states()
                self.status_var.set(f"Status: deleted station {removed.get('name', index)}")
            elif kind == "track_segment" and 0 <= index < len(self.sim.track_profile):
                if len(self.sim.track_profile) <= 1:
                    self.status_var.set("Status: cannot delete the last track segment")
                    return
                self._push_edit_undo()
                self.sim.track_profile.pop(index)
                self.sim.track_end_m = max(end for _start, end, _gradient, _psr in self.sim.track_profile)
                self.sim.track_max_m = max(self.sim.track_max_m, self.sim.track_end_m)
                for train in self.sim.trains:
                    train.track_profile = self.sim.track_profile
                self._update_control_track_profile()
                self.status_var.set(f"Status: deleted track segment {index}")
            elif kind == "tsr" and 0 <= index < len(self.sim.tsr_zones):
                self._push_edit_undo()
                self.sim.tsr_zones.pop(index)
                self.status_var.set(f"Status: deleted TSR {index}")
            elif kind == "line_condition" and 0 <= index < len(self.sim.line_conditions):
                self._push_edit_undo()
                self.sim.line_conditions.pop(index)
                self.status_var.set(f"Status: deleted line condition {index}")
            elif kind == "source_train" and 0 <= index < len(self.sim.source_trains):
                self._push_edit_undo()
                removed = self.sim.source_trains.pop(index)
                source_name = str(removed.get("name", "SRC"))
                self.sim.trains = [
                    train for train in self.sim.trains
                    if not self.sim._source_train_matches(train, source_name)
                ]
                self.sim._rebuild_after_train_set_change()
                self.sync_train_panels()
                self.status_var.set(f"Status: deleted source {removed.get('name', index)}")
            else:
                self.status_var.set(f"Status: cannot delete {element_key}")
                return
        finally:
            self.ats_overview_panel.selected_element = None
            self.ats_overview_panel.update_data(self.sim)
            self.infrastructure_panel.update_data(self.sim)
            self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def edit_ats_element(self, kind: str, index: int, _element_type: str, data: Dict[str, str]):
        self._push_edit_undo()
        if kind == "station":
            self.sim.update_station(
                index,
                data["name"] or f"STATION_{index + 1}",
                float(data["pos_m"]),
                float(data["length_m"]),
                int(float(data["capacity"])),
                float(data["dwell_s"]),
            )
        elif kind == "track_segment":
            self.sim.update_track_segment(
                index,
                float(data["start_m"]),
                float(data["end_m"]),
                float(data["gradient"]),
                float(data["psr_kmh"]),
            )
            self._update_control_track_profile()
        elif kind == "tsr":
            self.sim.update_tsr(index, float(data["start_m"]), float(data["end_m"]), float(data["speed_kmh"]))
        elif kind == "line_condition":
            self.sim.update_line_condition(index, float(data["start_m"]), float(data["end_m"]), data["condition"] or "dry")
        elif kind == "source_train":
            capacity = int(float(data["capacity"]))
            self.sim.update_source_train(
                index,
                data["name"] or f"SRC_{index + 1}",
                capacity,
                capacity,
            )
            self.sync_train_panels()
        else:
            raise ValueError("Unsupported ATS element.")
        self.status_var.set(f"Status: edited {kind}:{index}")
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def add_ats_element(self, element_type: str, data: Dict[str, str]):
        if element_type == "Station":
            self._push_edit_undo()
            name = data["name"] or f"STATION_{len(self.sim.scheduled_stops) + 1}"
            self.sim.add_station(
                name,
                float(data["pos_m"]),
                float(data["length_m"]),
                int(float(data["capacity"])),
                float(data["dwell_s"]),
            )
            self.status_var.set(f"Status: added station {name}")
        elif element_type == "Segment PSR/TSR":
            mode = data["mode"].upper()
            start_m = float(data["start_m"])
            end_m = float(data["end_m"])
            if end_m < start_m:
                start_m, end_m = end_m, start_m
            if mode not in ("PSR", "TSR"):
                raise ValueError("Mode must be PSR or TSR.")
            self._push_edit_undo()
            if mode == "PSR":
                self.sim.add_psr_segment(start_m, end_m, float(data["speed_kmh"]))
                self._update_control_track_profile()
            elif mode == "TSR":
                self.sim.tsr_zones.append({"start": start_m, "end": end_m, "speed": float(data["speed_kmh"])})
            self.status_var.set(f"Status: added {mode} segment")
            if self.pending_station_prompt_after_limit and self.pending_line_extension is not None:
                extension_start, extension_end = self.pending_line_extension
                missing_psr = self._first_psr_gap(0.0, self.sim.track_end_m)
                if missing_psr is not None:
                    missing_start, missing_end = missing_psr
                    self.status_var.set(
                        f"Status: added {mode} segment; add PSR for uncovered {missing_start:.0f}-{missing_end:.0f} m"
                    )
                    self.after(50, lambda: self._open_extension_speed_limit_dialog(missing_start, missing_end))
                else:
                    self.pending_line_extension = None
                    self.pending_station_prompt_after_limit = False
                    self.after(50, lambda: self._open_extension_station_dialog(extension_start, extension_end))
        elif element_type == "Line":
            old_end_m = self.sim.track_end_m
            length_m = max(1.0, float(data["length_m"]))
            self._push_edit_undo()
            self.sim.track_end_m = max(self.sim.track_end_m, length_m)
            self.sim.track_max_m = max(self.sim.track_max_m, length_m)
            missing_psr = self._first_psr_gap(0.0, self.sim.track_end_m)
            if length_m > old_end_m and missing_psr is not None:
                missing_start, missing_end = missing_psr
                self.pending_line_extension = (old_end_m, length_m)
                self.pending_station_prompt_after_limit = True
                self.status_var.set(
                    f"Status: extended line to {length_m:.0f} m; add PSR for uncovered {missing_start:.0f}-{missing_end:.0f} m"
                )
                self.after(50, lambda: self._open_extension_speed_limit_dialog(missing_start, missing_end))
            else:
                self.pending_line_extension = None
                self.pending_station_prompt_after_limit = False
                self.status_var.set(f"Status: extended line to {length_m:.0f} m")
        elif element_type == "Gradient Segment":
            self._push_edit_undo()
            self.sim.add_gradient_segment(
                float(data["start_m"]),
                float(data["end_m"]),
                float(data["gradient"]),
            )
            self._update_control_track_profile()
            self.status_var.set("Status: added gradient segment")
        elif element_type == "Line Condition":
            start_m = float(data["start_m"])
            end_m = float(data["end_m"])
            if end_m < start_m:
                start_m, end_m = end_m, start_m
            if end_m <= start_m:
                raise ValueError("Line condition end must be greater than start.")
            self._push_edit_undo()
            self.sim.line_conditions.append({"start": start_m, "end": end_m, "condition": data["condition"] or "dry"})
            self.status_var.set("Status: added line condition")
        elif element_type == "Source Train":
            self._push_edit_undo()
            name = data["name"] or f"SRC_{len(self.sim.source_trains) + 1}"
            capacity = int(float(data["capacity"]))
            self.sim.add_source_train(
                name,
                SOURCE_TRAIN_START_M,
                SOURCE_TRAIN_LENGTH_M,
                capacity,
                capacity,
            )
            self.sync_train_panels()
            self.status_var.set(f"Status: added source train {name}")
        else:
            raise ValueError("Unsupported element type.")
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def toggle_train(self, train_id: str, emergency: bool = False):
        for t in self.sim.trains:
            if t.id == train_id:
                if emergency:
                    t.emg_ack = True
                    t.acknowledge_emergency_safe()
                else:
                    t.enter_trip_mode("TRAIN TRIP", t.reported_pos)
                    t.emergency_recovery_hold = False
                break

    def resume_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                t.resume_after_emergency()
                break

    def instant_stop_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                t.enter_trip_mode("INSTANT STOP", t.reported_pos)
                t.emergency_recovery_hold = False
                t.ato_target_speed = 0.0
                t.service_brake_latch = False
                t.atp_state = "ATP_TRIP"
                t.atp_alert = "INSTANT STOP"
                t.atp_brake = "EMERGENCY"
                t.atp_action = "EBI"
                break

    def precise_jog_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                if t.request_precise_jog():
                    self.status_var.set(f"Status: {train_id} precise jog requested")
                else:
                    remaining_m = t.distance_to_stop_target()
                    self.status_var.set(
                        f"Status: {train_id} cannot jog "
                        f"(remaining={remaining_m:.2f} m, zero_speed={'YES' if t.zero_speed_detected else 'NO'})"
                    )
                break

    def toggle_train_fault(self, train_id: str, subsystem: str):
        subsystem = subsystem.upper()
        for train in self.sim.trains:
            if train.id != train_id:
                continue
            if subsystem == "ATP":
                train.set_fault("ATP", not train.atp_fault_active, self.sim.sim_time_s)
            elif subsystem == "ATO":
                train.set_fault("ATO", not train.ato_fault_active, self.sim.sim_time_s)
            elif subsystem == "DCS":
                train.set_fault("DCS", not train.dcs_fault_active, self.sim.sim_time_s)
            self.status_var.set(f"Status: toggled {subsystem} fault on {train_id}")
            break

    def toggle_all_dcs_loss(self):
        active = not any(train.dcs_fault_active for train in self.sim.trains)
        for train in self.sim.trains:
            train.set_fault("DCS", active, self.sim.sim_time_s)
        self.status_var.set(
            "Status: DCS loss applied"
            if active
            else "Status: DCS loss cleared; emergency recovery still required for tripped trains"
        )

    def clear_all_faults(self):
        for train in self.sim.trains:
            train.set_fault("DCS", False, self.sim.sim_time_s)
            train.set_fault("ATO", False, self.sim.sim_time_s)
            train.set_fault("ATP", False, self.sim.sim_time_s)
            if not (train.trip_mode or train.emg_latch or train.emergency_stop or train.emergency_recovery_hold):
                train.reset_non_emergency_stop_latches()
        self.status_var.set("Status: cleared fault flags; use Safe Confirmed/Resume for tripped trains")

    def apply_psr(self, segment_str: str, psr_str: str):
        try:
            idx = int(segment_str)
            psr = float(psr_str)
            if idx < 0 or idx >= len(self.sim.track_profile):
                return
            self._push_edit_undo()
            start, end, gradient, _ = self.sim.track_profile[idx]
            self.sim.track_profile[idx] = (start, end, gradient, psr)
        except ValueError:
            return

    def add_tsr(self, start_str: str, end_str: str, speed_str: str):
        try:
            start = float(start_str)
            end = float(end_str)
            speed = float(speed_str)
            if end < start:
                start, end = end, start
            self._push_edit_undo()
            self.sim.tsr_zones.append({"start": start, "end": end, "speed": speed})
        except ValueError:
            return

    def clear_tsr(self):
        if not self.sim.tsr_zones:
            return
        self._push_edit_undo()
        self.sim.tsr_zones.clear()

    def tick(self):
        if self.sim.running:
            steps_this_tick = max(1, int(self.time_scale))
            for _ in range(steps_this_tick):
                self.sim.step()
            if self.sim.train_generation_changed:
                self.sync_train_panels()
        now = time.monotonic()
        refresh_interval = self._running_ui_refresh_interval_s if self.sim.running else self._idle_ui_refresh_interval_s
        if now - self._last_ui_refresh_real_s >= refresh_interval:
            self._last_ui_refresh_real_s = now
            if self.sim.running:
                self._record_runtime_history()
            self._update_event_log()
            self.clock_var.set(f"Sim time: {self.sim.sim_time_s:.1f} s")
            self._update_root_summary()
            for t in self.sim.trains:
                if t.id not in self.panels:
                    self.sync_train_panels()
                self.panels[t.id].update_from_train(t, append_history=self.sim.running)
            self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
            if self.current_workspace_mode == "normal":
                self.ats_overview_panel.update_data(self.sim)
                self.infrastructure_panel.update_data(self.sim)
                self.engineering_panel.update_data(self.sim)
                self.diagnostics_panel.update_data(self.sim, self.event_log)
                self.analytics_panel.update_data(self.sim)
        self.after(int(DT * 1000), self.tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
