from __future__ import annotations

from GUI.gui_context import *

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


