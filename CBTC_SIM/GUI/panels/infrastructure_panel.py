from __future__ import annotations

from GUI.main_gui import *

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
        self._last_content = ""
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        lines = ["Static Guideway / Infrastructure"]
        lines.append("-" * 72)
        for idx, (start, end, gradient, psr) in enumerate(sim.track_profile, 1):
            occupied = any((t.pos - t.length) < end and t.pos > start for t in sim.trains)
            signal = "RED" if occupied else "GREEN"
            axle = "OCC" if occupied else "CLEAR"
            lines.append(f"SEG-{idx:02d} {start:>5.0f}-{end:<5.0f}  {axle:<20} gradient={gradient:+.3f} SSP={psr:.0f} km/h")
            lines.append(f"SIG-{idx:02d}             {signal:<20} virtual lineside aspect")
        transport = getattr(sim, "dcs_transport", None)
        rap_items = list(getattr(transport, "radio_access_points", []) or [])
        if rap_items:
            lines.append("")
            lines.append("Radio access points")
            lines.append("-" * 72)
            for rap in rap_items:
                lines.append(f"{rap.id:<8} {rap.start_m:>5.0f}-{rap.end_m:<5.0f}  DCS RADIO COVERAGE")
        if getattr(sim, "balises", []):
            lines.append("")
            lines.append("Balise / Beacon Layout")
            lines.append("-" * 72)
            for balise in sim.balises:
                lines.append(f"{str(balise.get('id', 'BALISE')):<8} pos={float(balise['pos_m']):>6.1f}m")
        if sim.tsr_zones:
            lines.append("")
            lines.append("Temporary speed restrictions")
            lines.append("-" * 72)
            for idx, zone in enumerate(sim.tsr_zones, 1):
                lines.append(
                    f"TSR-{idx:02d} {float(zone['start']):>5.0f}-{float(zone['end']):<5.0f}  ACTIVE               limit={float(zone['speed']):.0f} km/h"
                )
        lines.append("")
        lines.append("Train Configuration")
        lines.append("-" * 72)
        for train in sim.trains:
            adhesion_pct = ATP_ADHESION_FACTOR * 100.0
            mute_count = len(train.dcs_mute_windows)
            lines.append(
                f"{train.id:<10} mass={train.mass:>8.0f}kg  length={train.length:>5.1f}m  "
                f"mode={train.drive_mode:<5} ATO_cap={getattr(train, 'max_ato_speed_kmh', 70.0):>4.0f}km/h  "
                f"manual_cap={train.max_manual_speed_kmh:>4.0f}km/h  mute_windows={mute_count}"
            )
        lines.append("")
        lines.append("Safety Logic Baseline")
        lines.append("-" * 72)
        lines.extend(
            [
                f"Service brake model : factor={ATP_SERVICE_BRAKE_FACTOR:.2f}  buildup={ATP_BRAKE_BUILDUP_S:.2f}s",
                f"Emergency brake     : factor={ATP_EMERGENCY_BRAKE_FACTOR:.2f}  overlap={OVERLAP_M:.1f}m",
                f"Reaction delays     : P={ATP_P_REACTION_S:.1f}s  W={ATP_W_REACTION_S:.1f}s  SBI={ATP_SBI_REACTION_S:.1f}s  EBI={ATP_EBI_REACTION_S:.1f}s",
                f"Odometer error      : rate={ODOMETER_ERROR_RATE:.2f} m/m  base CI={POS_UNCERT_M:.1f}m  balise CI={BALISE_POS_UNCERT_M:.1f}m  station CI={STATION_POS_UNCERT_M:.1f}m",
                f"DCS transmission    : min={DCS_DELAY_MIN_S:.2f}s  max={DCS_DELAY_MAX_S:.2f}s  timeout={DCS_TIMEOUT_S:.1f}s",
            ]
        )
        lines.append("")
        lines.append("Route / Emergency Assets")
        lines.append("-" * 72)
        for idx, train in enumerate(sim.trains, 1):
            switch_state = "DIVERGING" if train.commanded_stop else "NORMAL"
            esa_state = "ACTIVE" if train.atp_action == "EBI" else "STANDBY"
            lines.append(f"SW-{idx:02d}              {switch_state:<20} simulated route authority")
            lines.append(f"ESA-{idx:02d}             {esa_state:<20} linked to {train.id}")
        content = "\n".join(lines)
        if content == self._last_content:
            return
        first, _last = self.text.yview()
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", content)
        self.text.yview_moveto(first)
        self.text.configure(state="disabled")
        self._last_content = content


