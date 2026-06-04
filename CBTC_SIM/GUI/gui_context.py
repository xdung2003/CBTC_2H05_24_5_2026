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

from CONFIG.config import (
    AW3_MASS_KG,
    DT,
    OVERLAP_M,
    SAFETY_MARGIN_M,
    BRAKE_FORCE_N,
    EMERGENCY_FORCE_N,
    BRAKE_BUILDUP_S,
    MAX_JERK_MS3,
)
from SUBSYSTEMS.physics import (
    kmh_to_ms,
    ms_to_kmh,
    braking_distance_m,
    traction_acceleration_ms2,
    running_resistance_accel_ms2,
    equivalent_mass_adjusted_accel,
    limit_jerk,
)
from CONFIG.scenario_loader import DEFAULT_SCENARIO_PATH, load_scenario, normalize_scenario, save_scenario_file, scenario_to_yaml_data
from REPORT.reporting import save_simulation_report
from OPERATION.headway_manager import HeadwayManager
from MONTECARLO.monte_carlo import MonteCarloConfig, run_batch
from SUBSYSTEMS.dcs import DCSWatchdog, OnboardControlCenter
from SUBSYSTEMS.signalling import (
    AuthorityManager,
    MovementAuthorityLimit,
    SafeMovementPacket,
    VitalBrakeModel,
    braking_curve_profile,
    conservative_brake_decel_ms2,
    get_track_info,
    gradient_adjusted_decel_ms2,
    max_entry_speed_with_buildup,
    max_speed_with_buildup,
    stopping_distance_with_buildup,
    vital_delay_margin_m,
    worst_gradient_in_range,
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
SOURCE_RELEASE_LOCK_S = 5.0
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


from SUBSYSTEMS.atp import ATPEnvelopeEngine, ATPEnvelopeResult
from SUBSYSTEMS.ato import ATOPilotingEngine, ATOPilotingResult
from SUBSYSTEMS.control_common import (
    ato_ebi_guard_ms,
    ato_pid_gains,
    ato_tracking_margin_ms,
    high_speed_curve_scale,
    low_speed_flexibility_scale,
    max_speed_for_target,
    precise_stop_gap_ms,
    precise_stop_profile_active,
    precise_stop_sbi_limit_ms,
    release_entry_speed_limit,
    release_speed_profile,
    release_transition_ratio,
    required_brake_rate_for_target,
    target_curve_reserve_m,
)

from SUBSYSTEMS.train import Train, train_color

from SUBSYSTEMS.zc import ZoneController

from SUBSYSTEMS.runtime import Simulation
