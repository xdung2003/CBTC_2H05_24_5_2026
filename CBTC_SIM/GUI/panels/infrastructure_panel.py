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
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def update_data(self, sim: Simulation):
        lines = ["Asset                State                 Notes"]
        lines.append("-" * 72)
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


