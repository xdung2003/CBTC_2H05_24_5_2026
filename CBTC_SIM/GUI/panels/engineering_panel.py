from __future__ import annotations

from GUI.main_gui import *
import json

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
        self.packet_events = []
        self.text.bind("<Button-1>", self._on_packet_row_click)

        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")

        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def _on_packet_row_click(self, event):
        index = self.text.index(f"@{event.x},{event.y}")
        tags = self.text.tag_names(index)
        for tag in tags:
            if tag.startswith("packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.packet_events):
                    self._open_packet_inspector(self.packet_events[event_index])
                    return "break"
        return None

    def _open_packet_inspector(self, event):
        window = tk.Toplevel(self)
        window.title(f"Packet Inspector - {event.protocol} {event.msg_type} #{event.sequence_number}")
        window.geometry(f"{int(860 * self.scale_factor)}x{int(720 * self.scale_factor)}")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        text = tk.Text(
            window,
            wrap="word",
            font=("Consolas", int(9 * self.scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
        )
        text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(window, orient="vertical", command=text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=scroll.set)
        details = event.details or {}
        sections = {
            "A. Route": details.get("route", {}),
            "B. Message layer": details.get("message", {}),
            "C. Frame / packet layer": details.get("frame", {}),
            "D. Protection layer": {
                "result": event.result,
                "action": event.action,
                "reason": event.reason,
                **details.get("protection", {}),
            },
            "E. Radio / modulation layer": details.get("radio", {}),
            "Transformation chain": details.get("chain", []),
        }
        lines = []
        for title, payload in sections.items():
            lines.append(title)
            lines.append("-" * len(title))
            lines.append(json.dumps(payload, indent=2, sort_keys=True, default=str))
            lines.append("")
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")

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
        for balise in getattr(sim, "balises", []):
            lines.append(f"{str(balise.get('id', 'BALISE')):<8} pos={float(balise['pos_m']):>6.1f}m")
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
        self.rowconfigure(4, weight=1)
        ttk.Label(self, text="Dataflow Monitor", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")
        self.summary_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.summary_var, style="Muted.TLabel").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(int(2 * scale_factor), int(6 * scale_factor)),
        )
        diagram_frame = ttk.Frame(self, style="Panel.TFrame")
        diagram_frame.grid(row=2, column=0, sticky="nsew")
        diagram_frame.columnconfigure(0, weight=1)
        diagram_frame.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            diagram_frame,
            height=int(820 * scale_factor),
            background=APP_THEME["canvas"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        canvas_hscroll = ttk.Scrollbar(diagram_frame, orient="horizontal", command=self.canvas.xview)
        canvas_hscroll.grid(row=1, column=0, sticky="ew")
        self.canvas.configure(xscrollcommand=canvas_hscroll.set)
        self.canvas.bind("<Button-1>", self._on_canvas_packet_click)
        self.canvas_packet_events = []
        self._last_sim = None
        self._resize_after_id = None
        self.canvas.bind("<Configure>", self._on_canvas_resize, add="+")

        filter_frame = ttk.Frame(self, style="Panel.TFrame")
        filter_frame.grid(row=3, column=0, sticky="ew", pady=(int(6 * scale_factor), 0))
        filter_frame.columnconfigure(5, weight=1)
        self.packet_filter_var = tk.StringVar(value="All")
        self.packet_search_var = tk.StringVar(value="")
        filter_values = ("All", "Vital only", "OPC UA only", "Rejected only")
        ttk.Label(filter_frame, text="Filter", style="Muted.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 4))
        filter_combo = ttk.Combobox(filter_frame, textvariable=self.packet_filter_var, values=filter_values, width=14, state="readonly")
        filter_combo.grid(row=0, column=1, sticky="w", padx=(0, 8))
        filter_combo.bind("<<ComboboxSelected>>", lambda _event: self.event_generate("<<DataflowFilterChanged>>"))
        ttk.Label(filter_frame, text="Train / Protocol / Result", style="Muted.TLabel").grid(row=0, column=2, sticky="w", padx=(0, 4))
        search_entry = ttk.Entry(filter_frame, textvariable=self.packet_search_var, width=24)
        search_entry.grid(row=0, column=3, sticky="w")
        search_entry.bind("<KeyRelease>", lambda _event: self.event_generate("<<DataflowFilterChanged>>"))

        text_frame = ttk.Frame(self, style="Panel.TFrame")
        text_frame.grid(row=4, column=0, sticky="nsew", pady=(int(6 * scale_factor), 0))
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
        self.packet_events = []
        self.text.bind("<Button-1>", self._on_packet_row_click)
        vscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        hscroll = ttk.Scrollbar(text_frame, orient="horizontal", command=self.text.xview)
        hscroll.grid(row=1, column=0, sticky="ew")
        self.text.configure(yscrollcommand=vscroll.set, xscrollcommand=hscroll.set)

    def _on_canvas_resize(self, _event):
        if self._last_sim is None:
            return
        if self._resize_after_id is not None:
            try:
                self.after_cancel(self._resize_after_id)
            except tk.TclError:
                pass
        self._resize_after_id = self.after(120, self._redraw_last_sim)

    def _redraw_last_sim(self):
        self._resize_after_id = None
        if self._last_sim is not None:
            self.update_data(self._last_sim)

    def _on_canvas_packet_click(self, event):
        item = self.canvas.find_closest(self.canvas.canvasx(event.x), self.canvas.canvasy(event.y))
        if not item:
            return None
        for tag in self.canvas.gettags(item[0]):
            if tag.startswith("canvas_packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.canvas_packet_events):
                    self._open_packet_inspector(self.canvas_packet_events[event_index])
                    return "break"
        return None

    def _on_packet_row_click(self, event):
        index = self.text.index(f"@{event.x},{event.y}")
        tags = self.text.tag_names(index)
        for tag in tags:
            if tag.startswith("packet_event_"):
                try:
                    event_index = int(tag.rsplit("_", 1)[1])
                except ValueError:
                    return None
                if 0 <= event_index < len(self.packet_events):
                    self._open_packet_inspector(self.packet_events[event_index])
                    return "break"
        return None

    def _open_packet_inspector(self, event):
        window = tk.Toplevel(self)
        window.title(f"Packet Inspector - {event.protocol} {event.msg_type} #{event.sequence_number}")
        window.geometry(f"{int(900 * self.scale_factor)}x{int(760 * self.scale_factor)}")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        text = tk.Text(
            window,
            wrap="word",
            font=("Consolas", int(9 * self.scale_factor)),
            background=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
        )
        text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(window, orient="vertical", command=text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=scroll.set)
        details = event.details or {}
        frame = details.get("frame", {})
        sections = {
            "Route": details.get("route", {}),
            "Message": details.get("message", {}),
            "Frame/Header": {
                "header": frame.get("received_header") or frame.get("header") or frame.get("sent_header") or frame,
                "session_id": (frame.get("received_header") or frame.get("header") or {}).get("session_id", ""),
                "sequence_number": event.sequence_number,
                "timestamp": (frame.get("received_header") or frame.get("header") or {}).get("timestamp_ms", frame.get("timestamp_ms", "")),
                "ttl": (frame.get("received_header") or frame.get("header") or {}).get("ttl_ms", frame.get("timeout_ms", "")),
                "packet_uuid": frame.get("safety", {}).get("packet_uuid", ""),
                "payload_length": (frame.get("received_header") or frame.get("header") or {}).get("payload_length", ""),
            },
            "Encryption": {
                "encryption_enabled": frame.get("encryption_enabled", False),
                "encryption_algorithm": frame.get("encryption_algorithm", ""),
                "key_id": frame.get("key_id", frame.get("safety", {}).get("key_id", "")),
                "encrypted_payload": frame.get("encrypted_payload", ""),
                "payload_format": frame.get("payload_format", ""),
            },
            "Protection": {
                "crc32": frame.get("safety", {}).get("crc32", ""),
                "hmac_sha256": frame.get("safety", {}).get("hmac_sha256", ""),
                "validation_result": event.result,
                "reject_reason": event.reason if event.result not in ("ACCEPTED", "DELIVERED", "FAILOVER") else "",
                **details.get("protection", {}),
            },
            "Radio/RAP": details.get("radio", {}),
            "Validation result": {
                "result": event.result,
                "action": event.action,
                "reason": event.reason,
            },
            "State update result": {
                "updated": event.result in ("ACCEPTED", "DELIVERED") and event.action not in ("ignored", "rejected"),
                "action": event.action,
                "reason": event.reason,
            },
            "Transformation chain": details.get("chain", []),
        }
        lines = []
        for title, payload in sections.items():
            lines.append(title)
            lines.append("-" * len(title))
            lines.append(json.dumps(payload, indent=2, sort_keys=True, default=str))
            lines.append("")
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")

    def _filtered_packet_events(self, events):
        mode = self.packet_filter_var.get() if hasattr(self, "packet_filter_var") else "All"
        query = self.packet_search_var.get().strip().lower() if hasattr(self, "packet_search_var") else ""
        rejected_results = {"REJECTED", "TIMEOUT", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"}
        filtered = []
        for event in events:
            if mode == "Vital only" and event.protocol != "RASTA_VITAL":
                continue
            if mode == "OPC UA only" and event.protocol != "OPCUA_SUPERVISION":
                continue
            if mode == "Rejected only" and event.result not in rejected_results and event.action not in ("ignored", "rejected"):
                continue
            if query:
                haystack = " ".join(
                    str(item).lower()
                    for item in (
                        event.source_id,
                        event.destination_id,
                        event.protocol,
                        event.path,
                        event.msg_type,
                        event.result,
                        event.reason,
                    )
                )
                if query not in haystack:
                    continue
            filtered.append(event)
        return filtered

    def _scale(self, value: float) -> int:
        return int(round(value * self.scale_factor))

    def _function_block(self, x: float, y: float, w: float, h: float, title: str, body: str, fill: str) -> dict[str, float]:
        c = self.canvas
        c.create_rectangle(x, y, x + w, y + h, fill=fill, outline="#1f2933", width=2)
        c.create_text(
            x + w / 2,
            y + self._scale(18),
            text=title,
            fill=APP_THEME["text"],
            font=("Consolas", self._scale(10), "bold"),
        )
        if body:
            c.create_text(
                x + self._scale(12),
                y + self._scale(38),
                anchor="nw",
                text=body,
                fill=APP_THEME["muted"],
                font=("Consolas", self._scale(8)),
                width=max(self._scale(90), w - self._scale(24)),
            )
        return {
            "left": x,
            "right": x + w,
            "top": y,
            "bottom": y + h,
            "cx": x + w / 2,
            "cy": y + h / 2,
            "w": w,
            "h": h,
        }

    def _route_label(self, x: float, y: float, text: str, color: str, anchor: str = "center"):
        self.canvas.create_text(
            x,
            y,
            text=text,
            fill=color,
            anchor=anchor,
            font=("Consolas", self._scale(8), "bold"),
        )

    def _lane_arrow(
        self,
        points: list[tuple[float, float]],
        label: str,
        color: str,
        label_segment: int = 0,
        label_offset: float = -16,
        dash: tuple[int, int] | None = None,
        width_px: int = 3,
    ):
        if len(points) < 2:
            return
        flat = [coord for point in points for coord in point]
        self.canvas.create_line(
            *flat,
            fill=color,
            width=width_px,
            arrow=tk.LAST,
            dash=dash,
            capstyle=tk.ROUND,
            joinstyle=tk.ROUND,
        )
        if label:
            segment_index = min(max(0, label_segment), len(points) - 2)
            start = points[segment_index]
            end = points[segment_index + 1]
            lx = (start[0] + end[0]) / 2
            ly = (start[1] + end[1]) / 2
            self._route_label(lx, ly + label_offset, label, color)

    def _update_canvas_scrollregion(self, padding: int | None = None):
        padding = self._scale(70) if padding is None else padding
        bbox = self.canvas.bbox("all")
        if bbox is None:
            width = max(self._scale(980), int(self.canvas.winfo_width() or self._scale(980)))
            height = self._scale(620)
            self.canvas.configure(scrollregion=(0, 0, width, height))
            return
        x1, y1, x2, y2 = bbox
        scrollregion = (
            min(0, x1 - padding),
            min(0, y1 - padding),
            max(int(self.canvas.winfo_width() or 0), x2 + padding),
            y2 + padding,
        )
        self.canvas.configure(scrollregion=scrollregion)

    def _draw_legend(self, x: float, y: float, compact: bool = False):
        rows = [
            ("#5b6fd6", "Vital: CC <-> ZC authority data", None),
            ("#1f8a8a", "Supervision: CC/ZC/Stations -> ATS", None),
            ("#8b5cf6", "ATS/OCC commands", None),
            ("#6b7280", "Station/YAML internal state", None),
        ]
        row_h = self._scale(17 if compact else 20)
        w = self._scale(330 if compact else 360)
        h = self._scale(30) + len(rows) * row_h
        self.canvas.create_rectangle(x, y, x + w, y + h, fill=APP_THEME["canvas"], outline=APP_THEME["border"])
        self.canvas.create_text(
            x + self._scale(10),
            y + self._scale(8),
            anchor="nw",
            text="Legend",
            fill=APP_THEME["text"],
            font=("Consolas", self._scale(8), "bold"),
        )
        for idx, (color, text, dash) in enumerate(rows):
            ly = y + self._scale(34) + idx * row_h
            self.canvas.create_line(x + self._scale(12), ly, x + self._scale(48), ly, fill=color, width=3, dash=dash)
            self.canvas.create_text(
                x + self._scale(58),
                ly,
                anchor="w",
                text=text,
                fill=APP_THEME["muted"],
                font=("Consolas", self._scale(7)),
            )

    def _draw_basic_dataflow_canvas(self, sim: Simulation, events):
        c = self.canvas
        c.delete("all")
        self.canvas_packet_events = []

        width = max(self._scale(720), int(c.winfo_width() or self._scale(980)))
        margin = self._scale(36 if width < self._scale(900) else 54)
        trains = list(getattr(sim, "trains", []))
        train_count = len(trains)
        stops = list(getattr(sim, "scheduled_stops", []))
        visible_trains = trains[:6] if trains else []
        top = self._scale(72)
        cc_w = self._scale(220)
        cc_h = self._scale(82)
        cc_gap = self._scale(22)
        station_w = self._scale(235)
        station_h = self._scale(76)
        station_gap = self._scale(20)
        zc_w = self._scale(220)
        ats_w = self._scale(230)
        side_gap = max(self._scale(70), (width - 2 * margin - cc_w - station_w - zc_w - ats_w) / 3)
        cc_x = margin
        station_x = cc_x + cc_w + side_gap
        zc_x = station_x + station_w + side_gap
        ats_x = zc_x + zc_w + side_gap
        if ats_x + ats_w > width - margin:
            side_gap = self._scale(70)
            ats_x = zc_x + zc_w + side_gap

        zc = self._function_block(
            zc_x,
            top + self._scale(40),
            zc_w,
            self._scale(126),
            "ZC",
            "Computes MA\nEOA + SVL\nuses CC + station state",
            "#eef8f0",
        )
        ats = self._function_block(
            ats_x,
            top + self._scale(40),
            ats_w,
            self._scale(126),
            "ATS",
            "OCC supervision\noperator commands\nno MA/EOA issue",
            "#fff7e8",
        )

        cc_blocks = []
        if visible_trains:
            for idx, train in enumerate(visible_trains):
                y = top + idx * (cc_h + cc_gap)
                link = "OK" if getattr(train, "safe_packet_valid", False) else "STALE"
                body = (
                    f"pos={float(getattr(train, 'pos', 0.0)):6.1f}m\n"
                    f"EOA={float(getattr(train, 'eoa', 0.0)):6.1f}m  link={link}"
                )
                cc_blocks.append(self._function_block(cc_x, y, cc_w, cc_h, f"CC {train.id}", body, "#eef3ff"))
        else:
            cc_blocks.append(self._function_block(cc_x, top, cc_w, cc_h, "CC", "No active train", "#eef3ff"))

        station_blocks = []
        visible_stops = stops[:5] if stops else []
        active_stop_ids = {str(getattr(train, "active_scheduled_stop", "") or "") for train in trains}
        if visible_stops:
            for idx, stop in enumerate(visible_stops):
                y = top + idx * (station_h + station_gap)
                name = str(stop.get("name", f"STATION_{idx + 1}"))
                occupied = name in active_stop_ids
                status = "DWELL/OCCUPIED" if occupied else "AVAILABLE"
                body = f"pos={float(stop.get('pos_m', 0.0)):6.1f}m\nstate={status}"
                station_blocks.append(self._function_block(station_x, y, station_w, station_h, name, body, "#f4f4f5"))
        else:
            station_blocks.append(self._function_block(station_x, top, station_w, station_h, "Station State", "No stations configured", "#f4f4f5"))

        vital_color = "#5b6fd6"
        ma_color = "#b85c00"
        opc_color = "#1f8a8a"
        command_color = "#8b5cf6"
        internal_color = "#6b7280"
        first_cc = cc_blocks[0]
        last_cc = cc_blocks[-1]
        first_station = station_blocks[0]
        last_station = station_blocks[-1]
        cc_bus_x = first_cc["right"] + self._scale(26)
        station_bus_x = first_station["right"] + self._scale(24)
        supervision_y = max(last_cc["bottom"], last_station["bottom"], zc["bottom"], ats["bottom"]) + self._scale(52)
        command_y = top - self._scale(36)

        for idx, cc in enumerate(cc_blocks):
            y = cc["cy"]
            self._lane_arrow([(cc["right"], y), (cc_bus_x, y)], "", vital_color, dash=(6, 4))
        self._lane_arrow(
            [(cc_bus_x, first_cc["cy"]), (cc_bus_x, zc["cy"] - self._scale(28)), (zc["left"], zc["cy"] - self._scale(28))],
            "POSITION_REPORT",
            vital_color,
            label_segment=1,
            label_offset=-self._scale(16),
        )
        self._lane_arrow(
            [(zc["left"], zc["cy"] + self._scale(10)), (cc_bus_x, zc["cy"] + self._scale(10)), (cc_bus_x, last_cc["cy"]), (last_cc["right"], last_cc["cy"])],
            "MA_UPDATE: EOA + SVL",
            ma_color,
            label_segment=0,
            label_offset=self._scale(16),
        )

        self._lane_arrow(
            [(first_cc["right"], first_cc["bottom"] - self._scale(10)), (cc_bus_x, first_cc["bottom"] - self._scale(10)), (cc_bus_x, supervision_y), (ats["left"], supervision_y), (ats["left"], ats["bottom"] - self._scale(24))],
            "TRAIN_STATUS -> ATS",
            opc_color,
            label_segment=2,
            label_offset=self._scale(16),
            dash=(6, 4),
        )
        self._lane_arrow(
            [(zc["right"], zc["bottom"] - self._scale(38)), (ats["left"], zc["bottom"] - self._scale(38))],
            "ZC_STATE -> ATS",
            opc_color,
            label_offset=-self._scale(16),
        )
        self._lane_arrow(
            [(ats["left"], ats["top"] + self._scale(28)), (zc["right"], zc["top"] + self._scale(28))],
            "ATS route/TSR -> ZC",
            command_color,
            label_offset=-self._scale(16),
        )
        self._lane_arrow(
            [(ats["cx"], ats["top"]), (ats["cx"], command_y), (cc_bus_x, command_y), (cc_bus_x, first_cc["top"]), (first_cc["right"], first_cc["top"])],
            "ATS CMD: stop/hold/resume",
            command_color,
            label_segment=1,
            label_offset=-self._scale(16),
            dash=(6, 4),
        )

        for station in station_blocks:
            self._lane_arrow([(station["right"], station["cy"]), (station_bus_x, station["cy"])], "", internal_color)
        self._lane_arrow(
            [(station_bus_x, first_station["cy"]), (station_bus_x, zc["bottom"] + self._scale(28)), (zc["cx"], zc["bottom"] + self._scale(28)), (zc["cx"], zc["bottom"])],
            "Station constraints -> ZC",
            internal_color,
            label_segment=1,
            label_offset=-self._scale(16),
        )
        self._lane_arrow(
            [(station_bus_x, last_station["cy"]), (station_bus_x, supervision_y + self._scale(38)), (ats["right"] - self._scale(26), supervision_y + self._scale(38)), (ats["right"] - self._scale(26), ats["bottom"])],
            "WAYSIDE_STATUS / OPC UA -> ATS",
            opc_color,
            label_segment=1,
            label_offset=self._scale(16),
        )

        note_y = max(supervision_y + self._scale(68), last_station["bottom"] + self._scale(42), last_cc["bottom"] + self._scale(42))
        note_h = self._scale(54)
        c.create_rectangle(margin, note_y, width - margin, note_y + note_h, fill=APP_THEME["canvas"], outline=APP_THEME["border"])
        c.create_text(
            margin + self._scale(12),
            note_y + self._scale(10),
            anchor="nw",
            text="Canvas hides DCS transport by design. Use the packet log below for DCS/RaSTA/OPC UA details; click a packet row to open the Packet Inspector.",
            fill=APP_THEME["text"],
            font=("Consolas", self._scale(8), "bold"),
            width=width - 2 * margin - self._scale(24),
        )

        compact_legend = width < self._scale(1040)
        legend_w = self._scale(330 if compact_legend else 360)
        legend_x = width - margin - legend_w
        legend_y = note_y + note_h + self._scale(20)
        self._draw_legend(legend_x, legend_y, compact=compact_legend)
        c.configure(height=max(self._scale(560), legend_y + self._scale(125)))
        self._update_canvas_scrollregion()

    def update_data(self, sim: Simulation):
        self._last_sim = sim
        trains = sorted(sim.trains, key=lambda item: item.id)
        transport = getattr(sim, "dcs_transport", None)
        raw_events = list(getattr(transport, "events", []))[-120:] if transport is not None else []
        events = self._filtered_packet_events(raw_events)[-80:]
        self.summary_var.set(
            f"Logical dataflow view  |  CC={len(trains)}  stations={len(sim.scheduled_stops)}  "
            f"SGD segments={len(sim.track_profile)}  packet log events={len(events)}  DCS details in packet log"
        )
        self._draw_basic_dataflow_canvas(sim, events)

        header_lines = [
            "time    | from        | to          | protocol          | path       | msg_type        | seq   | latency | ttl | result       | action        | reason",
            "-" * 156,
        ]
        event_lines = []
        if events:
            for event in events:
                event_lines.append(
                    f"{event.time_s:7.3f} | "
                    f"{event.source_id[:11]:<11} | "
                    f"{event.destination_id[:11]:<11} | "
                    f"{event.protocol[:17]:<17} | "
                    f"{event.path[:10]:<10} | "
                    f"{event.msg_type[:15]:<15} | "
                    f"{event.sequence_number:<5} | "
                    f"{event.latency_ms:>6.0f}ms | "
                    f"{event.ttl_state:<3} | "
                    f"{event.result[:12]:<12} | "
                    f"{event.action[:13]:<13} | "
                    f"{event.reason}"
                )
        else:
            event_lines.append("(no packet events yet)")
        footer_lines = ["", "Network Health / DCS-NMS", "-" * 72]
        if transport is not None:
            red = transport.paths.get("RED")
            blue = transport.paths.get("BLUE")
            total_timeout = sum(path.timeout_count for path in transport.paths.values())
            total_lost = sum(path.lost_count for path in transport.paths.values())
            total_sent = sum(path.sent_count for path in transport.paths.values())
            total_loss_pct = (total_lost / total_sent * 100.0) if total_sent else 0.0
            active_path = getattr(transport, "active_path", "")
            rap_text = ", ".join(f"{train_id}={rap_id}" for train_id, rap_id in sorted(getattr(transport, "last_rap_by_train", {}).items())) or "none"
            active = transport.paths.get(active_path)
            latency = f"{active.base_latency_ms:.0f}ms" if active is not None else "n/a"
            jitter = f"{active.jitter_ms:.0f}ms" if active is not None else "n/a"
            footer_lines.extend(
                [
                    f"RED={red.state.value if red else 'N/A':<8} BLUE={blue.state.value if blue else 'N/A':<8} active={active_path:<4} RAP={rap_text}",
                    f"latency={latency:<6} jitter={jitter:<6} packet_loss={total_loss_pct:>5.1f}%  timeout_count={total_timeout:<4} handover_count={getattr(transport, 'handover_count', 0):<4}",
                    f"last_fault={getattr(transport, 'last_fault', '') or 'none'}",
                ]
            )
        else:
            footer_lines.append("DCS transport unavailable")
        footer_lines.extend(["", "Train vital data freshness", "-" * 72])
        for train in trains:
            link = "MUTE" if train.dcs_muted else "OK" if train.safe_packet_valid else "TIMEOUT"
            footer_lines.append(
                f"{train.id:<10} MA={getattr(train, 'ma_freshness', 'FRESH'):<7} "
                f"link={link:<8} result={getattr(train, 'vital_packet_result', ''):<12} "
                f"reason={getattr(train, 'vital_packet_reason', '')}"
            )
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("end", "\n".join(header_lines) + "\n")
        self.packet_events = events
        for idx, line in enumerate(event_lines):
            tag = f"packet_event_{idx}"
            if events:
                self.text.insert("end", line + "\n", (tag,))
            else:
                self.text.insert("end", line + "\n")
            self.text.tag_configure(tag, foreground=APP_THEME["text"], underline=False)
        self.text.insert("end", "\n".join(footer_lines))
        self.text.configure(state="disabled")


