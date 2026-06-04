from __future__ import annotations

from GUI.gui_context import *

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


