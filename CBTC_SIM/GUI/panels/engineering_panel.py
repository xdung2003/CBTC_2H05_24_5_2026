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
        self.canvas = tk.Canvas(
            self,
            height=int(820 * scale_factor),
            background=APP_THEME["canvas"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.canvas.bind("<Button-1>", self._on_canvas_packet_click)
        self.canvas_packet_events = []

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
        label_offset: float = -14,
    ):
        c = self.canvas
        dash = (5, 4) if dashed else None
        c.create_line(x1, y1, x2, y2, fill=color, width=2, arrow=tk.LAST, dash=dash)
        lx = x1 + (x2 - x1) * 0.5
        ly = y1 + (y2 - y1) * 0.5
        if label:
            c.create_text(lx, ly + label_offset, text=label, fill=color, font=("Consolas", int(7 * self.scale_factor), "bold"))
        packet_x = x1 + (x2 - x1) * phase
        packet_y = y1 + (y2 - y1) * phase
        c.create_oval(packet_x - 4, packet_y - 4, packet_x + 4, packet_y + 4, fill=color, outline="")

    def _path_arrow(
        self,
        points: list[tuple[float, float]],
        label: str,
        color: str,
        phase: float,
        dashed: bool = False,
        label_offset: float = -14,
        label_segment: int | None = None,
    ):
        if len(points) < 2:
            return
        c = self.canvas
        dash = (5, 4) if dashed else None
        flat = [coord for point in points for coord in point]
        c.create_line(*flat, fill=color, width=2, arrow=tk.LAST, dash=dash)

        segments = []
        total_len = 0.0
        for start, end in zip(points, points[1:]):
            length = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
            segments.append((start, end, length))
            total_len += length

        if label:
            if label_segment is None or not (0 <= label_segment < len(segments)):
                label_segment = max(range(len(segments)), key=lambda idx: segments[idx][2])
            start, end, _length = segments[label_segment]
            lx = start[0] + (end[0] - start[0]) * 0.5
            ly = start[1] + (end[1] - start[1]) * 0.5
            c.create_text(lx, ly + label_offset, text=label, fill=color, font=("Consolas", int(7 * self.scale_factor), "bold"))

        target = max(0.0, min(1.0, phase)) * total_len
        travelled = 0.0
        packet_x, packet_y = points[-1]
        for start, end, length in segments:
            if travelled + length >= target:
                ratio = 0.0 if length <= 0.0 else (target - travelled) / length
                packet_x = start[0] + (end[0] - start[0]) * ratio
                packet_y = start[1] + (end[1] - start[1]) * ratio
                break
            travelled += length
        c.create_oval(packet_x - 4, packet_y - 4, packet_x + 4, packet_y + 4, fill=color, outline="")

    def update_data(self, sim: Simulation):
        trains = sorted(sim.trains, key=lambda item: item.id)
        dcs_ok = sum(1 for train in trains if train.safe_packet_valid)
        self.summary_var.set(
            f"Live packet paths  |  trains={len(trains)}  stations={len(sim.scheduled_stops)}  "
            f"sources={len(getattr(sim, 'source_trains', []))}  SGD segments={len(sim.track_profile)}  DCS healthy={dcs_ok}/{len(trains)}"
        )

        c = self.canvas
        c.delete("all")
        width = max(900, int(c.winfo_width() or 900))
        diagram_height = int(650 * self.scale_factor)
        c.configure(scrollregion=(0, 0, width, diagram_height))
        phase_base = (sim.sim_time_s * 0.55) % 1.0
        transport = getattr(sim, "dcs_transport", None)
        active_path = getattr(transport, "active_path", "RED") if transport is not None else "RED"
        raw_events = list(getattr(transport, "events", []))[-120:] if transport is not None else []
        events = self._filtered_packet_events(raw_events)[-80:]

        def status_color(value: str) -> str:
            value = str(value).upper()
            if value in ("OK", "FRESH", "ACCEPTED", "DELIVERED"):
                return APP_THEME["ok"]
            if value in ("DEGRADED", "STALE", "EXPIRED", "FAILOVER"):
                return APP_THEME["warning"]
            if value in ("LOST", "TIMEOUT", "REJECTED", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"):
                return APP_THEME["danger"]
            return APP_THEME["muted"]

        red_net = "#d43f3a"
        blue_net = "#286fd6"
        position_color = "#7b4cc2"
        ma_color = "#c05a00"
        status_color_line = "#248a8d"
        infra_zc_color = "#2f8f5b"
        block_fill = "#ffffff"
        block_outline = "#111111"

        def block(x: float, y: float, w: float, h: float, title: str, body: str = "", fill: str = block_fill):
            c.create_rectangle(x, y, x + w, y + h, fill=block_fill, outline=block_outline, width=3)
            if fill != block_fill:
                c.create_rectangle(x + 2, y + 2, x + w - 2, y + h - 2, fill=fill, outline="")
            c.create_text(
                x + w / 2,
                y + 18,
                text=title,
                fill=APP_THEME["text"],
                font=("Consolas", int(10 * self.scale_factor), "bold"),
            )
            if body:
                c.create_text(
                    x + 10,
                    y + 38,
                    anchor="nw",
                    text=body,
                    fill=APP_THEME["muted"],
                    font=("Consolas", int(8 * self.scale_factor)),
                    width=max(40, w - 20),
                )
            return {"x": x, "y": y, "w": w, "h": h, "left": x, "right": x + w, "top": y, "bottom": y + h, "cx": x + w / 2, "cy": y + h / 2}

        def label(x: float, y: float, text: str, color: str, anchor: str = "center"):
            c.create_text(x, y, text=text, fill=color, font=("Consolas", int(8 * self.scale_factor), "bold"), anchor=anchor)

        def route(points: list[tuple[float, float]], color: str, dash: tuple[int, int] | None = None, arrow: str | None = tk.LAST, width_px: int = 3):
            flat = [coord for point in points for coord in point]
            c.create_line(*flat, fill=color, width=width_px, arrow=arrow, dash=dash)

        def point_on_path(points: list[tuple[float, float]], phase: float) -> tuple[float, float]:
            if len(points) < 2:
                return points[0] if points else (0.0, 0.0)
            segments = []
            total_len = 0.0
            for start, end in zip(points, points[1:]):
                length = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
                segments.append((start, end, length))
                total_len += length
            target = max(0.0, min(1.0, phase)) * total_len
            travelled = 0.0
            x, y = points[-1]
            for start, end, length in segments:
                if travelled + length >= target:
                    ratio = 0.0 if length <= 0.0 else (target - travelled) / length
                    x = start[0] + (end[0] - start[0]) * ratio
                    y = start[1] + (end[1] - start[1]) * ratio
                    break
                travelled += length
            return x, y

        def packet_marker(points: list[tuple[float, float]], color: str, phase: float):
            x, y = point_on_path(points, phase)
            c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color, outline="")

        def flow(points: list[tuple[float, float]], color: str, text: str, phase: float, dash: tuple[int, int] | None = None, label_index: int = 0):
            route(points, color, dash=dash, width_px=3)
            if len(points) >= 2:
                start = points[min(label_index, len(points) - 2)]
                end = points[min(label_index + 1, len(points) - 1)]
                label((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 - 14, text, color)

        stations = list(getattr(sim, "scheduled_stops", []))
        sources = list(getattr(sim, "source_trains", []))
        cc_count = max(1, len(trains))
        infra_count = max(1, len(sources) + len(stations))
        lane_gap = 92
        infra_gap = 88
        left_x = max(36, width * 0.035)
        mid_x = max(360, width * 0.39)
        right_x = max(680, width - 295)
        top_y = 34
        dcs_y = 224
        cc_start_y = 420
        infra_start_y = 415
        cc_w = min(260, max(210, width * 0.23))
        infra_w = min(255, max(220, width * 0.22))
        diagram_height = int(max(760 * self.scale_factor, cc_start_y + cc_count * lane_gap + 245, infra_start_y + infra_count * infra_gap + 245))
        c.configure(height=diagram_height, scrollregion=(0, 0, width, diagram_height))

        ats = block(left_x, top_y, 230, 120, "ATS / OCC", "Non-vital supervision only\nReceives TRAIN_STATUS\nNo direct MA/EOA vital input", "#fff6df")
        zc = block(right_x, top_y, 245, 132, "ZC", "Computes MA / EOA / SVL\nInputs: train position,\nstations/depot/track DB/TSR", "#e8f6e8")
        dcs = block(mid_x, dcs_y, 285, 150, "DCS Transport Layer", f"RED / BLUE redundant paths\nRAP + radio + BTN/fiber\nactive={active_path}", "#f7f1ff")

        cc_blocks: dict[str, dict[str, float]] = {}
        if trains:
            for idx, train in enumerate(trains):
                y = cc_start_y + idx * lane_gap
                mode = getattr(train, "operation_mode", getattr(train, "mode", "AUTO"))
                body = (
                    f"ATP/ATO onboard\n"
                    f"pos={float(getattr(train, 'pos', 0.0)):6.1f}m  mode={mode}\n"
                    f"EOA={float(getattr(train, 'eoa', 0.0)):6.1f}m"
                )
                cc_blocks[train.id] = block(left_x, y, cc_w, 78, f"CC {train.id}", body, "#eef3ff")
        else:
            cc_blocks["NO_TRAIN"] = block(left_x, cc_start_y, cc_w, 78, "CC", "No active train\nwaiting for depot/source", "#eef3ff")

        infra_blocks: list[dict[str, float]] = []
        if sources:
            for idx, source in enumerate(sources):
                y = infra_start_y + len(infra_blocks) * infra_gap
                name = str(source.get("name", f"DEPOT_{idx + 1}"))
                body = (
                    f"source={float(source.get('start_m', 0.0)):6.1f}m\n"
                    f"capacity={int(source.get('capacity', 1))}\n"
                    "source/staging data"
                )
                infra_blocks.append(block(right_x, y, infra_w, 76, name, body, "#fff1df"))
        else:
            infra_blocks.append(block(right_x, infra_start_y, infra_w, 76, "DEPOT", "not configured\nsource data missing", "#fff1df"))
        for idx, stop in enumerate(stations):
            y = infra_start_y + len(infra_blocks) * infra_gap
            name = str(stop.get("name", f"STATION_{idx + 1}"))
            body = (
                f"stop={float(stop.get('pos_m', 0.0)):6.1f}m\n"
                f"capacity={int(stop.get('capacity', 1))}\n"
                "stop/route constraint"
            )
            infra_blocks.append(block(right_x, y, infra_w, 76, name, body, "#fff1df"))

        red_y = dcs["top"] + 62
        blue_y = dcs["top"] + 88
        c.create_line(dcs["left"] + 24, red_y, dcs["right"] - 24, red_y, fill=red_net, width=4)
        c.create_text(dcs["left"] + 30, red_y - 10, anchor="w", text="RED", fill=red_net, font=("Consolas", int(8 * self.scale_factor), "bold"))
        c.create_line(dcs["left"] + 24, blue_y, dcs["right"] - 24, blue_y, fill=blue_net, width=4)
        c.create_text(dcs["left"] + 30, blue_y + 12, anchor="w", text="BLUE", fill=blue_net, font=("Consolas", int(8 * self.scale_factor), "bold"))
        c.create_text(
            dcs["cx"],
            dcs["bottom"] - 22,
            text="Transmission only: delay, loss, failover, radio coverage",
            fill=APP_THEME["muted"],
            font=("Consolas", int(8 * self.scale_factor)),
        )

        pos_routes: dict[str, list[tuple[float, float]]] = {}
        ma_routes: dict[str, list[tuple[float, float]]] = {}
        status_routes: dict[str, list[tuple[float, float]]] = {}
        for idx, (train_id, cc) in enumerate(cc_blocks.items()):
            lane = idx % 5
            pos_y = dcs["top"] + 34 + lane * 7
            ma_y = dcs["bottom"] - 34 - lane * 7
            status_mid_y = max(ats["bottom"] + 24, cc["top"] - 24 - lane * 8)
            pos_routes[train_id] = [(cc["right"], cc["top"] + 22), (dcs["left"], pos_y), (dcs["right"], pos_y), (zc["left"], zc["bottom"] - 46)]
            ma_routes[train_id] = [(zc["left"], zc["bottom"] - 18), (dcs["right"], ma_y), (dcs["left"], ma_y), (cc["right"], cc["top"] + 55)]
            status_routes[train_id] = [(cc["left"] + 30, cc["top"]), (cc["left"] + 30, status_mid_y), (ats["left"] + 52, status_mid_y), (ats["left"] + 52, ats["bottom"])]
            label_text = "POSITION_REPORT / RaSTA_VITAL" if idx == 0 else ""
            flow(pos_routes[train_id], position_color, label_text, (phase_base + 0.05 + idx * 0.06) % 1.0, dash=(7, 5), label_index=1)
            label_text = "MA_UPDATE contains EOA + SVL / RaSTA_VITAL" if idx == 0 else ""
            flow(ma_routes[train_id], ma_color, label_text, (phase_base + 0.45 + idx * 0.06) % 1.0, dash=(7, 5), label_index=1)
            label_text = "TRAIN_STATUS / OPCUA_SUPERVISION" if idx == 0 else ""
            flow(status_routes[train_id], status_color_line, label_text, (phase_base + 0.25 + idx * 0.05) % 1.0, dash=None, label_index=1)

        for idx, infra in enumerate(infra_blocks):
            zc_lane_y = zc["bottom"] - 16 - (idx % 4) * 15
            bend_x = zc["right"] + 32 + (idx % 3) * 16
            infra_points = [(infra["left"], infra["top"] + 34), (bend_x, infra["top"] + 34), (bend_x, zc_lane_y), (zc["right"], zc_lane_y)]
            flow(
                infra_points,
                infra_zc_color,
                "depot/station/track constraints -> ZC" if idx == 0 else "",
                (phase_base + 0.62 + idx * 0.07) % 1.0,
                dash=None,
                label_index=0,
            )

        label(zc["cx"], zc["bottom"] + 22, "EOA is calculated here, then packaged inside MA_UPDATE", ma_color)
        label(ats["cx"], ats["bottom"] + 22, "ATS does not receive vital POSITION_REPORT or MA_UPDATE", status_color_line)

        default_train_id = next(iter(cc_blocks))

        def route_train_id(event, prefer_destination: bool = False) -> str:
            candidates = []
            if prefer_destination:
                candidates.extend([event.destination_id, event.source_id])
            else:
                candidates.extend([event.source_id, event.destination_id])
            for candidate in candidates:
                if candidate in cc_blocks:
                    return candidate
            return default_train_id

        def event_route(event):
            if event.protocol == "RASTA_VITAL" and event.msg_type == "POSITION_REPORT":
                return pos_routes.get(route_train_id(event)), position_color
            if event.protocol == "RASTA_VITAL" and event.msg_type == "MA_UPDATE":
                return ma_routes.get(route_train_id(event, prefer_destination=True)), ma_color
            if event.protocol == "OPCUA_SUPERVISION" and event.msg_type == "TRAIN_STATUS":
                return status_routes.get(route_train_id(event)), status_color_line
            if event.protocol == "OPCUA_SUPERVISION":
                return status_routes.get(route_train_id(event)), status_color_line
            if event.protocol == "RASTA_VITAL":
                return pos_routes.get(route_train_id(event)), position_color
            return None, APP_THEME["muted"]

        self.canvas_packet_events = []
        recent_canvas_events = [
            event
            for event in events[-36:]
            if event.msg_type not in {"HANDOVER", "COMM_FAULT"}
        ]
        for idx, event in enumerate(recent_canvas_events):
            points, color = event_route(event)
            if not points:
                continue
            self.canvas_packet_events.append(event)
            event_index = len(self.canvas_packet_events) - 1
            age_s = max(0.0, float(sim.sim_time_s) - float(event.time_s))
            phase = min(0.98, 0.08 + age_s * 0.8 + (idx % 5) * 0.035)
            x, y = point_on_path(points, phase)
            rejected = event.result in {"REJECTED", "TIMEOUT", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"} or event.action in {"ignored", "rejected"}
            outline = APP_THEME["danger"] if rejected else APP_THEME["text"]
            size = 7 if event.result in {"DELIVERED", "ACCEPTED"} else 6
            tags = ("packet_dot", f"canvas_packet_event_{event_index}")
            c.create_oval(x - size, y - size, x + size, y + size, fill=color, outline=outline, width=2, tags=tags)
            c.create_text(
                x,
                y - 13,
                text=event.msg_type[:3],
                fill=outline,
                font=("Consolas", int(6 * self.scale_factor), "bold"),
                tags=tags,
            )

        note_x = left_x
        note_y = max((block_item["bottom"] for block_item in [*cc_blocks.values(), *infra_blocks]), default=cc_start_y) + 34
        c.create_rectangle(note_x, note_y, min(width - 36, note_x + 780), note_y + 58, fill=APP_THEME["canvas"], outline=APP_THEME["border"])
        c.create_text(
            note_x + 12,
            note_y + 10,
            anchor="nw",
            text="Correct authority chain: Train POSITION_REPORT -> ZC computes MA/EOA/SVL -> MA_UPDATE -> Train CC/ATP. "
            "ATS receives non-vital TRAIN_STATUS after onboard state is updated.",
            fill=APP_THEME["text"],
            font=("Consolas", int(8 * self.scale_factor), "bold"),
            width=max(300, min(width - 80, 740)),
        )

        legend_x, legend_y = min(width - 265, right_x), 184
        c.create_rectangle(legend_x, legend_y, legend_x + 250, legend_y + 138, fill=APP_THEME["canvas"], outline=APP_THEME["border"])
        c.create_text(legend_x + 10, legend_y + 8, anchor="nw", text="Legend", fill=APP_THEME["text"], font=("Consolas", int(8 * self.scale_factor), "bold"))
        legend_rows = [
            (position_color, "Vital: POSITION_REPORT"),
            (ma_color, "Vital: MA_UPDATE with EOA/SVL"),
            (status_color_line, "Non-vital: TRAIN_STATUS"),
            (infra_zc_color, "Infrastructure constraints"),
            (red_net, "DCS RED path"),
            (blue_net, "DCS BLUE path"),
        ]
        for idx, (color, text) in enumerate(legend_rows):
            ly = legend_y + 32 + idx * 17
            c.create_line(legend_x + 12, ly, legend_x + 45, ly, fill=color, width=3)
            c.create_text(legend_x + 54, ly, anchor="w", text=text, fill=APP_THEME["muted"], font=("Consolas", int(7 * self.scale_factor)))

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


