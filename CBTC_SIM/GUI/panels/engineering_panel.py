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
            height=int(430 * scale_factor),
            background=APP_THEME["canvas"],
            highlightthickness=1,
            highlightbackground=APP_THEME["border"],
        )
        self.canvas.grid(row=2, column=0, sticky="nsew")

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
        phase_base = (sim.sim_time_s * 0.55) % 1.0
        transport = getattr(sim, "dcs_transport", None)
        red = transport.paths.get("RED") if transport is not None else None
        blue = transport.paths.get("BLUE") if transport is not None else None
        active_path = getattr(transport, "active_path", "RED") if transport is not None else "RED"
        rap_text = ", ".join(
            f"{train_id}:{rap_id}" for train_id, rap_id in sorted(getattr(transport, "last_rap_by_train", {}).items())
        ) or "none"
        last_fault = getattr(transport, "last_fault", "") if transport is not None else ""
        rejected_count = sum(
            1
            for event in list(getattr(transport, "events", []))[-80:]
            if event.result in {"REJECTED", "TIMEOUT", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"}
            or event.action in {"ignored", "rejected"}
        ) if transport is not None else 0

        margin = 18
        gap = max(10, int(12 * self.scale_factor))

        def status_color(value: str) -> str:
            value = str(value).upper()
            if value in ("OK", "FRESH", "ACCEPTED", "DELIVERED"):
                return APP_THEME["ok"]
            if value in ("DEGRADED", "STALE", "EXPIRED", "FAILOVER"):
                return APP_THEME["warning"]
            if value in ("LOST", "TIMEOUT", "REJECTED", "REPLAY", "CRC_ERROR", "HMAC_ERROR", "OUT_OF_ORDER", "DECRYPT_ERROR"):
                return APP_THEME["danger"]
            return APP_THEME["muted"]

        c.create_text(
            margin,
            8,
            anchor="nw",
            text="CBTC subsystem dataflow: ATS / ZC / Train CC through DCS dual-redundant RED-BLUE radio network",
            fill=APP_THEME["text"],
            font=("Consolas", int(9 * self.scale_factor), "bold"),
        )

        headway_mode = getattr(sim.headway_manager, "mode", "off")
        red_state = red.state.value if red is not None else "N/A"
        blue_state = blue.state.value if blue is not None else "N/A"
        base_y = 34
        panel_h = max(300, min(height - 52, int(342 * self.scale_factor)))
        ats_w = max(126, width * 0.15)
        zc_w = max(138, width * 0.17)
        dcs_w = max(250, width * 0.29)
        cc_w = max(190, width - margin * 2 - ats_w - zc_w - dcs_w - gap * 3)
        ats_x = margin
        zc_x = ats_x + ats_w + gap
        dcs_x = zc_x + zc_w + gap
        cc_x = dcs_x + dcs_w + gap

        def subsystem_box(x: float, y: float, w: float, h: float, title: str, subtitle: str, fill: str, outline: str | None = None):
            c.create_rectangle(x, y, x + w, y + h, fill=fill, outline=outline or APP_THEME["border"], width=2)
            c.create_text(x + 8, y + 8, anchor="nw", text=title, fill=APP_THEME["text"], font=("Consolas", int(10 * self.scale_factor), "bold"))
            c.create_text(x + 8, y + 29, anchor="nw", text=subtitle, fill=APP_THEME["muted"], font=("Consolas", int(8 * self.scale_factor)))

        def inner_box(x: float, y: float, w: float, h: float, title: str, body: str, fill: str, outline: str | None = None):
            c.create_rectangle(x, y, x + w, y + h, fill=fill, outline=outline or APP_THEME["border"], width=1)
            c.create_text(x + 6, y + 5, anchor="nw", text=title, fill=APP_THEME["text"], font=("Consolas", int(8 * self.scale_factor), "bold"))
            c.create_text(x + 6, y + 22, anchor="nw", text=body, fill=APP_THEME["muted"], font=("Consolas", int(7 * self.scale_factor)))

        subsystem_box(ats_x, base_y, ats_w, panel_h, "ATS", f"OPC UA-like\nmode={headway_mode}", APP_THEME["card"])
        subsystem_box(zc_x, base_y, zc_w, panel_h, "ZC", f"MA/EOA authority\n{sim.block_mode}", "#ffe7b8")
        subsystem_box(dcs_x, base_y, dcs_w, panel_h, "DCS", f"dual network + radio\nactive={active_path}", APP_THEME["card_alt"], outline=status_color("OK" if red_state != "LOST" or blue_state != "LOST" else "LOST"))
        subsystem_box(cc_x, base_y, cc_w, panel_h, "Train CC", f"{len(trains)} onboard controllers\nATP/ATO/TX-RX", APP_THEME["card"])

        inner_margin = 10
        ats_inner_w = ats_w - inner_margin * 2
        inner_box(ats_x + inner_margin, base_y + 64, ats_inner_w, 56, "Supervision", "hold / dwell\ntrain status", "#e8f2ff")
        inner_box(ats_x + inner_margin, base_y + 132, ats_inner_w, 56, "ATS Display", "received state\nFRESH/STALE/LOST", "#e8f2ff")
        inner_box(ats_x + inner_margin, base_y + 200, ats_inner_w, 58, "Diagnostics", "network health\nevent log", "#fff4e8")

        zc_inner_w = zc_w - inner_margin * 2
        inner_box(zc_x + inner_margin, base_y + 64, zc_inner_w, 58, "MA Builder", "MA_UPDATE\nEOA / PSR / TSR", "#fff1cc")
        inner_box(zc_x + inner_margin, base_y + 136, zc_inner_w, 58, "Position Store", "last valid report\nfresh/stale/lost", "#f7fff0")
        inner_box(zc_x + inner_margin, base_y + 210, zc_inner_w, 58, "Vital Session", "seq watchdog\nanti-replay", "#e4f7df")

        dcs_inner_w = dcs_w - inner_margin * 2
        red_y = base_y + 62
        blue_y = red_y + 54
        rap_y = blue_y + 62
        inner_box(dcs_x + inner_margin, red_y, dcs_inner_w, 42, "RED Network Path", f"state={red_state}\nlat={red.base_latency_ms:.0f}ms" if red is not None else "state=N/A", "#fff7f7", outline=status_color(red_state))
        inner_box(dcs_x + inner_margin, blue_y, dcs_inner_w, 42, "BLUE Network Path", f"state={blue_state}\nlat={blue.base_latency_ms:.0f}ms" if blue is not None else "state=N/A", "#f4f8ff", outline=status_color(blue_state))
        inner_box(dcs_x + inner_margin, rap_y, dcs_inner_w, 58, "RAP / Radio Layer", f"{rap_text[:30]}\nOFDM-QAM-like BER", "#f7fff0")
        inner_box(dcs_x + inner_margin, rap_y + 72, dcs_inner_w, 62, "DCS-NMS / Router", f"failover RED<->BLUE\nlast={last_fault or 'none'}", "#fff4e8")

        cc_inner_w = cc_w - inner_margin * 2
        cc_col_gap = 8
        cc_col_w = max(68, (cc_inner_w - cc_col_gap) / 2)
        inner_box(cc_x + inner_margin, base_y + 64, cc_col_w, 62, "TX/RX", "encrypt/decrypt\nCRC/HMAC check", "#e4f7df", outline=status_color("OK" if dcs_ok == len(trains) else "DEGRADED"))
        inner_box(cc_x + inner_margin + cc_col_w + cc_col_gap, base_y + 64, cc_col_w, 62, "ATP", f"MA accepted only\nOK {dcs_ok}/{len(trains)}", "#e4f7df", outline=status_color("OK" if dcs_ok == len(trains) else "LOST"))
        inner_box(cc_x + inner_margin, base_y + 142, cc_col_w, 62, "ATO", "uses ATP limit\nnon-vital commands", "#f5ecff")
        inner_box(cc_x + inner_margin + cc_col_w + cc_col_gap, base_y + 142, cc_col_w, 62, "Status Agent", "TrainStatus\nfault/mode/door", "#e8f2ff")
        inner_box(cc_x + inner_margin, base_y + 220, cc_inner_w, 58, "Packet Validation Gate", f"ACCEPTED updates state\nrejected={rejected_count}", "#fff4e8", outline=status_color("OK" if rejected_count == 0 else "REJECTED"))

        ats_mid = ats_x + ats_w
        zc_left = zc_x
        zc_right = zc_x + zc_w
        dcs_left = dcs_x
        dcs_right = dcs_x + dcs_w
        cc_left = cc_x
        cc_right = cc_x + cc_w
        vital_color = "#4f8f3a"
        report_color = "#2f7f8f"
        opc_color = "#4078a0"
        fault_dashed = red_state == "LOST" and blue_state == "LOST"
        opc_dashed = bool(getattr(transport, "faults", {}).get("opcua_loss", False)) if transport is not None else False

        self._packet_arrow(zc_right, base_y + 95, dcs_left, red_y + 20, "RASTA MA_UPDATE", vital_color, phase_base, dashed=fault_dashed)
        self._packet_arrow(dcs_right, red_y + 20, cc_left, base_y + 95, "RED vital", vital_color, (phase_base + 0.15) % 1.0, dashed=red_state == "LOST")
        self._packet_arrow(dcs_right, blue_y + 20, cc_left, base_y + 114, "BLUE standby/failover", vital_color, (phase_base + 0.28) % 1.0, dashed=blue_state == "LOST")
        self._packet_arrow(cc_left, base_y + 250, dcs_right, rap_y + 22, "POSITION_REPORT", report_color, (phase_base + 0.42) % 1.0, dashed=fault_dashed)
        self._packet_arrow(dcs_left, rap_y + 22, zc_right, base_y + 164, "valid position", report_color, (phase_base + 0.55) % 1.0, dashed=fault_dashed)

        self._packet_arrow(ats_mid, base_y + 92, zc_left, base_y + 92, "route/headway", opc_color, (phase_base + 0.08) % 1.0, dashed=opc_dashed)
        self._packet_arrow(cc_left, base_y + 174, dcs_right, blue_y + 20, "TRAIN_STATUS", opc_color, (phase_base + 0.22) % 1.0, dashed=opc_dashed)
        self._packet_arrow(dcs_left, blue_y + 20, ats_mid, base_y + 160, "OPC UA-like status", opc_color, (phase_base + 0.36) % 1.0, dashed=opc_dashed)
        self._packet_arrow(ats_mid, base_y + 116, dcs_left, blue_y + 20, "hold/dwell cmd", opc_color, (phase_base + 0.5) % 1.0, dashed=opc_dashed)
        self._packet_arrow(dcs_right, blue_y + 20, cc_left, base_y + 174, "ATO command", opc_color, (phase_base + 0.64) % 1.0, dashed=opc_dashed)

        legend_y = base_y + panel_h + 12
        if legend_y < height - 22:
            c.create_text(
                margin,
                legend_y,
                anchor="nw",
                text="Green=RASTA-like vital safety, Blue=OPC UA-like supervision, RED/BLUE boxes show DCS redundant paths, RAP box shows radio access coverage.",
                fill=APP_THEME["muted"],
                font=("Consolas", int(8 * self.scale_factor)),
            )

        header_lines = [
            "time    | from        | to          | protocol          | path       | msg_type        | seq   | latency | ttl | result       | action        | reason",
            "-" * 156,
        ]
        transport = getattr(sim, "dcs_transport", None)
        raw_events = list(getattr(transport, "events", []))[-120:]
        events = self._filtered_packet_events(raw_events)[-80:]
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


