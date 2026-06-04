from __future__ import annotations

from GUI.main_gui import *

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


