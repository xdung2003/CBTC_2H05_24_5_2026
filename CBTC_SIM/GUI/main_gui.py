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
            "actual": tk.StringVar(value="Actual       : 0.0 km/h"),
            "permitted": tk.StringVar(value="Permitted    : 0.0 km/h"),
            "warning": tk.StringVar(value="Warning      : 0.0 km/h"),
            "intervention": tk.StringVar(value="Intervention : 0.0 km/h"),
        }
        for var in self.detail_metric_vars.values():
            ttk.Label(metrics_frame, textvariable=var).pack(anchor="w")
        
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

        self.detail_metric_vars["actual"].set(f"Actual       : {actual_kmh:.1f} km/h")
        self.detail_metric_vars["permitted"].set(f"Permitted    : {permitted_kmh:.1f} km/h")
        self.detail_metric_vars["warning"].set(f"Warning      : {warning_kmh:.1f} km/h")
        self.detail_metric_vars["intervention"].set(f"Intervention : {intervention_kmh:.1f} km/h")

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


class DataFlowPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.scale_factor = scale_factor
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(3, weight=1)
        ttk.Label(self, text="Dataflow Monitor", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(int(2 * scale_factor), int(6 * scale_factor)),
        )
        self.canvas = tk.Canvas(
            self,
            height=int(430 * scale_factor),
            background=APP_THEME["canvas"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.canvas.grid(row=2, column=0, sticky="nsew")

        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=3, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            height=int(10 * scale_factor),
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

    def _node(self, x: float, y: float, w: float, h: float, title: str, body: str, fill: str, outline: str | None = None):
        c = self.canvas
        outline = outline or APP_THEME["border"]
        c.create_rectangle(x, y, x + w, y + h, fill=fill, outline=outline, width=2)
        c.create_text(
            x + 8,
            y + 8,
            anchor="nw",
            text=title,
            fill=APP_THEME["text"],
            font=("Consolas", int(9 * self.scale_factor), "bold"),
        )
        if body:
            c.create_text(
                x + 8,
                y + 28,
                anchor="nw",
                text=body,
                fill=APP_THEME["muted"],
                font=("Consolas", int(8 * self.scale_factor)),
            )

    def _packet_arrow(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        label: str,
        color: str,
        phase: float,
        dashed: bool = False,
    ):
        c = self.canvas
        dash = (4, 3) if dashed else None
        c.create_line(x1, y1, x2, y2, fill=color, width=2, arrow=tk.LAST, dash=dash)
        lx = x1 + (x2 - x1) * 0.5
        ly = y1 + (y2 - y1) * 0.5
        c.create_rectangle(lx - 36, ly - 9, lx + 36, ly + 9, fill=APP_THEME["canvas"], outline="")
        c.create_text(lx, ly, text=label, fill=color, font=("Consolas", int(7 * self.scale_factor), "bold"))
        packet_x = x1 + (x2 - x1) * phase
        packet_y = y1 + (y2 - y1) * phase
        c.create_oval(packet_x - 4, packet_y - 4, packet_x + 4, packet_y + 4, fill=color, outline="")

    def update_data(self, sim: Simulation):
        trains = sorted(sim.trains, key=lambda item: item.id)
        dcs_ok = sum(1 for train in trains if train.safe_packet_valid)
        self.summary_var.set(
            f"Live packet paths  |  trains={len(trains)}  stations={len(sim.scheduled_stops)}  "
            f"sources={len(getattr(sim, 'source_trains', []))}  DCS healthy={dcs_ok}/{len(trains)}"
        )

        c = self.canvas
        c.delete("all")
        width = max(760, int(c.winfo_width() or 760))
        height = max(400, int(c.winfo_height() or 400))
        node_w = max(118, int(128 * self.scale_factor))
        node_h = max(58, int(62 * self.scale_factor))
        zc_x = width * 0.50 - node_w / 2
        top_y = 18
        train_y = 210
        ats_pos = (width * 0.18 - node_w / 2, top_y)
        zc_pos = (zc_x, top_y)
        dcs_pos = (width * 0.82 - node_w / 2, top_y)

        headway_mode = getattr(sim.headway_manager, "mode", "off")
        self._node(ats_pos[0], ats_pos[1], node_w, node_h, "ATS", f"mode={headway_mode}\nroutes/stops", APP_THEME["card"])
        self._node(zc_pos[0], zc_pos[1], node_w, node_h, "ZC", f"{sim.block_mode}\nMA packets", "#ffe7b8")
        self._node(dcs_pos[0], dcs_pos[1], node_w, node_h, "DCS", f"OK {dcs_ok}/{len(trains)}\nradio link", APP_THEME["card_alt"])
        phase_base = (sim.sim_time_s * 0.55) % 1.0
        self._packet_arrow(ats_pos[0] + node_w, ats_pos[1] + node_h / 2, zc_pos[0], zc_pos[1] + node_h / 2, "route/headway", APP_THEME["accent"], phase_base)
        self._packet_arrow(zc_pos[0], zc_pos[1] + node_h / 2 + 12, ats_pos[0] + node_w, ats_pos[1] + node_h / 2 + 12, "line status", "#2f7f8f", (phase_base + 0.45) % 1.0)
        self._packet_arrow(zc_pos[0] + node_w, zc_pos[1] + node_h / 2, dcs_pos[0], dcs_pos[1] + node_h / 2, "safe pkt", "#4f8f3a", (phase_base + 0.2) % 1.0)

        if trains:
            left_margin = 18
            usable_w = max(1, width - left_margin * 2 - node_w)
            count = max(1, len(trains))
            for idx, train in enumerate(trains):
                x = left_margin + (usable_w * idx / max(1, count - 1) if count > 1 else usable_w / 2)
                y = train_y
                link_ok = train.safe_packet_valid and not train.dcs_muted
                outline = APP_THEME["ok"] if link_ok else APP_THEME["danger"]
                body = (
                    f"ATP {train.atp_state.replace('ATP_', '')}\n"
                    f"ATO {train.ato_state.replace('ATO_', '')}"
                )
                self._node(x, y, node_w, node_h, train.id, body, APP_THEME["card"], outline=outline)
                src_x = dcs_pos[0] + node_w / 2
                src_y = dcs_pos[1] + node_h
                dst_x = x + node_w / 2
                dst_y = y
                color = APP_THEME["ok"] if link_ok else APP_THEME["danger"]
                self._packet_arrow(src_x, src_y, dst_x, dst_y, "ZC->CC", color, (phase_base + idx * 0.17) % 1.0, dashed=not link_ok)
                self._packet_arrow(dst_x, dst_y - 10, zc_pos[0] + node_w / 2, zc_pos[1] + node_h, "pos", "#2f7f8f", (phase_base + 0.55 + idx * 0.13) % 1.0)
                self._packet_arrow(x + 16, y + node_h + 10, x + node_w - 16, y + node_h + 10, "ATP<>ATO", "#8a4f9f", (phase_base + idx * 0.21) % 1.0)

        asset_y = max(train_y + node_h + 48, height - max(58, int(56 * self.scale_factor)))
        source_count = len(getattr(sim, "source_trains", []))
        station_count = len(sim.scheduled_stops)
        asset_count = max(1, source_count + station_count)
        asset_w = max(96, int(108 * self.scale_factor))
        for idx, source in enumerate(getattr(sim, "source_trains", [])):
            x = 18 + idx * min(asset_w + 14, max(90, (width - 36) / asset_count))
            body = f"{int(source.get('generated', 0))}/{int(source.get('total_trains', 0))} trains"
            self._node(x, asset_y, asset_w, 46, str(source.get("name", "DEPOT")), body, "#fff1cc", outline="#000000")
            self._packet_arrow(x + asset_w / 2, asset_y, ats_pos[0] + node_w / 2, ats_pos[1] + node_h, "depot", APP_THEME["muted"], (phase_base + 0.35) % 1.0)
        for idx, stop in enumerate(sim.scheduled_stops):
            x_index = source_count + idx
            x = 18 + x_index * min(asset_w + 14, max(90, (width - 36) / asset_count))
            state = sim.station_route_states[idx] if idx < len(getattr(sim, "station_route_states", [])) else {}
            occupied = sum(1 for line in state.get("lines", []) if line.get("occupied_by_train_id"))
            capacity = max(1, int(stop.get("capacity", 1)))
            self._node(x, asset_y, asset_w, 46, str(stop.get("name", f"STA{idx + 1}")), f"occ {occupied}/{capacity}", "#fff1cc")
            self._packet_arrow(x + asset_w / 2, asset_y, ats_pos[0] + node_w / 2, ats_pos[1] + node_h, "station", APP_THEME["muted"], (phase_base + idx * 0.11) % 1.0)

        lines = [
            "Time     From        To          Packet / State",
            "-" * 92,
        ]
        for train in trains:
            link = "MUTE" if train.dcs_muted else "OK" if train.safe_packet_valid else "TIMEOUT"
            lines.append(
                f"{sim.sim_time_s:7.1f}s ZC          {train.id:<10} MA eoa={train.eoa:>7.1f}m psr={train.psr_kmh:>5.1f} "
                f"age={train.safe_packet_age_s:>4.1f}s link={link}"
            )
            lines.append(
                f"{sim.sim_time_s:7.1f}s {train.id:<11} ZC          POS report={train.reported_pos:>7.1f}m "
                f"speed={ms_to_kmh(train.speed):>5.1f}km/h"
            )
            lines.append(
                f"{sim.sim_time_s:7.1f}s {train.id + '/ATP':<11} {train.id + '/ATO':<10} "
                f"ATP={train.atp_state:<14} ATO={train.ato_state:<12} action={train.atp_action or 'NONE'}"
            )
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")


class AnalyticsPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, scale_factor: float):
        super().__init__(master, padding=int(8 * scale_factor), style="Panel.TFrame")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        ttk.Label(self, text="Line Configuration Analytics", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
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
            "Line Configuration KPI",
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
        "Depot",
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
        "Depot": [
            ("name", "Name", "DEPOT"),
            ("capacity", "Depot trains", "3"),
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
        scenario_group = self._make_button_group(btns, "Line Config I/O", 2)
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

        self.load_btn = ttk.Button(scenario_group, text="Load Line Config", command=self.on_load_scenario)
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
        self.scenario_var = tk.StringVar(value=f"Line config: {self.scenario['name']}")
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
            values=("Fixed-block", "Headway target", "Timetable"),
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
        ttk.Button(element_side, text="Depot + Train", command=self.add_train_to_selected_source).grid(row=2, column=0, sticky="ew", padx=2, pady=(8, 2))
        ttk.Button(element_side, text="Depot - Train", command=self.remove_train_from_selected_source).grid(row=3, column=0, sticky="ew", padx=2, pady=2)

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
        dataflow_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        analytics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        for tab in (infra_tab, engineering_tab, diagnostics_tab, dataflow_tab, analytics_tab):
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(0, weight=1)
        self.infrastructure_panel = InfrastructurePanel(infra_tab, self.scale_factor)
        self.infrastructure_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=(int(6 * self.scale_factor), int(3 * self.scale_factor)))
        self.engineering_panel = EngineeringPanel(engineering_tab, self.scale_factor)
        self.engineering_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.diagnostics_panel = DiagnosticsPanel(diagnostics_tab, self.scale_factor)
        self.diagnostics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.dataflow_panel = DataFlowPanel(dataflow_tab, self.scale_factor)
        self.dataflow_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.analytics_panel = AnalyticsPanel(analytics_tab, self.scale_factor)
        self.analytics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.limits_panel = SpeedLimitsPanel(infra_tab, self.scale_factor)
        dock_tabs.add(infra_tab, text="Infrastructure")
        dock_tabs.add(engineering_tab, text="Engineering")
        dock_tabs.add(diagnostics_tab, text="Diagnostics")
        dock_tabs.add(dataflow_tab, text="Dataflow")
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
        self.dataflow_panel.update_data(self.sim)
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
            return "Fixed-block"
        if mode == "timetable":
            return "Timetable"
        return "Headway target"

    def _operation_mode_key(self) -> str:
        value = self.operation_mode_var.get().strip().lower()
        if value.startswith("fixed") or value.startswith("1"):
            return "fixed_block"
        if value.startswith("time") or value.startswith("3"):
            return "timetable"
        return "headway_target"

    def _operation_mode_label_for_key(self, key: str) -> str:
        labels = {
            "fixed_block": "Fixed-block",
            "headway_target": "Headway target",
            "timetable": "Timetable",
        }
        return labels.get(key, "Headway target")

    def _active_operation_mode_key(self) -> str:
        if getattr(self.sim, "block_mode", "moving_block") == "fixed_block":
            return "fixed_block"
        mode = str(getattr(self.sim.headway_manager, "mode", "fixed")).lower()
        if mode == "timetable":
            return "timetable"
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
        self.operation_mode_var.set("Timetable")
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
            self.operation_mode_var.set("Timetable")
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
            title="Load Line Configuration YAML",
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
            self.status_var.set(f"Status: failed to load line config ({exc})")
            if was_running:
                self.sim.start()
            return
        self.sim.load_scenario(self.scenario)
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Line config: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.edit_undo_stack.clear()
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()
        self.rebuild_train_panels()
        self.status_var.set(f"Status: loaded line config from {selected}")
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
        self.scenario_var.set(f"Line config: {self.scenario['name']}")
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
                title="Save Line Configuration YAML",
                filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
                defaultextension=".yaml",
                initialdir=str(DEFAULT_SCENARIO_PATH.parent),
            )
            if not target_path:
                return
        try:
            path = save_scenario_file(self.sim, self.scenario, target_path)
        except Exception as exc:
            self.status_var.set(f"Status: failed to save line config ({exc})")
            return
        self.scenario["source_path"] = str(path)
        self.status_var.set(f"Status: saved line config to {path}")

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
        self.status_var.set(f"Status: depot {source.get('name', index)} trains={count}")

    def add_train_to_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: add a depot first")
            return
        source = self.sim.source_trains[index]
        current = int(source.get("total_trains", source.get("capacity", 0)))
        self._set_source_train_count(index, current + 1)

    def remove_train_from_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: no depot to remove from")
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
            return "Depot", {
                "name": str(source.get("name", "DEPOT")),
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
                source_name = str(removed.get("name", "DEPOT"))
                self.sim.trains = [
                    train for train in self.sim.trains
                    if not self.sim._source_train_matches(train, source_name)
                ]
                self.sim._rebuild_after_train_set_change()
                self.sync_train_panels()
                self.status_var.set(f"Status: deleted depot {removed.get('name', index)}")
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
                data["name"] or f"DEPOT_{index + 1}",
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
        elif element_type == "Depot":
            self._push_edit_undo()
            name = data["name"] or f"DEPOT_{len(self.sim.source_trains) + 1}"
            capacity = int(float(data["capacity"]))
            self.sim.add_source_train(
                name,
                SOURCE_TRAIN_START_M,
                SOURCE_TRAIN_LENGTH_M,
                capacity,
                capacity,
            )
            self.sync_train_panels()
            self.status_var.set(f"Status: added depot {name}")
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
                self.dataflow_panel.update_data(self.sim)
                self.analytics_panel.update_data(self.sim)
        self.after(int(DT * 1000), self.tick)


# GUI startup is kept in run.py. This module remains import-compatible for tests
# and for code that still imports the legacy combined module during refactoring.
