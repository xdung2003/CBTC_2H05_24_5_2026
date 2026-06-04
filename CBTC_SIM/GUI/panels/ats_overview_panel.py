from __future__ import annotations

from GUI.main_gui import *

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


