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


