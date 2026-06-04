from __future__ import annotations

from GUI.gui_context import *
from GUI.panels.train_panel import TrainPanel
from GUI.panels.ats_overview_panel import ATSOverviewPanel
from GUI.panels.infrastructure_panel import InfrastructurePanel
from GUI.panels.engineering_panel import DataFlowPanel, EngineeringPanel, TimeDistancePanel
from GUI.panels.diagnostics_panel import DiagnosticsPanel
from GUI.panels.analytics_panel import AnalyticsPanel
from GUI.panels.control_panel import ControlPanel, SpeedLimitsPanel
from GUI.dialogs.scenario_dialog import AddElementDialog
from GUI.panels.monte_carlo_panel import MonteCarloPanel

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        # Detect DPI and set scaling for responsiveness
        try:
            user32 = ctypes.windll.user32
            user32.SetProcessDPIAware()
            dpi = user32.GetDpiForSystem()
            scale_factor = dpi / 96.0  # 96 is default DPI
            self.tk.call('tk', 'scaling', scale_factor)
        except:
            # Fallback if DPI detection fails
            scale_factor = 1.0

        self.scale_factor = scale_factor
        self.scenario = load_scenario()
        self.title(self.scenario["window_title"])
        # Set fullscreen mode
        self.state('zoomed')  # Windows fullscreen
        self.resizable(True, True)
        self.configure(background=APP_THEME["bg"])
        self._configure_styles()

        self.sim = Simulation(self.scenario)
        self.time_scale = 1
        self.sim_paused = False
        self.current_workspace_mode = "normal"
        self.operation_mode_var = tk.StringVar(value=self._operation_mode_from_scenario())
        self.operation_mechanism_var = tk.StringVar(value="")
        self.operation_selected_status_var = tk.StringVar(value="")
        self.headway_target_var = tk.StringVar(value=str(self.scenario.get("headway", {}).get("target_headway_s", 180.0)))
        self.timetable_file_var = tk.StringVar(value=str(self.scenario.get("headway", {}).get("timetable_file", "")))
        self.blocks_per_section_var = tk.StringVar(
            value=str(self.scenario.get("capacity_baseline", {}).get("blocks_per_section", 4))
        )
        self.vn_clock_var = tk.StringVar(value="")
        self.time_scale_buttons: Dict[int, ttk.Button] = {}
        self.time_history = deque(maxlen=240)
        self.position_history: Dict[str, deque] = {}
        self.event_log = deque(maxlen=30)
        self._last_ui_refresh_real_s = 0.0
        self._running_ui_refresh_interval_s = DT
        self._idle_ui_refresh_interval_s = 1.00
        self.edit_undo_stack: List[Dict[str, Any]] = []
        self.edit_redo_stack: List[Dict[str, Any]] = []
        self.prev_train_snapshot: Dict[str, Tuple[str, str, bool, bool, bool]] = {}
        self.child_windows: Dict[str, tk.Toplevel] = {}
        self._reset_runtime_buffers()
        self.pending_line_extension: Tuple[float, float] | None = None
        self.pending_station_prompt_after_limit = False
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        container = ttk.Frame(self, style="Shell.TFrame")
        container.grid(row=0, column=0, sticky="nsew")
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(container, background=APP_THEME["bg"], highlightthickness=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        vscroll = ttk.Scrollbar(container, orient="vertical", command=self.scroll_canvas.yview)
        vscroll.grid(row=0, column=1, sticky="ns")
        self.scroll_canvas.configure(yscrollcommand=vscroll.set)

        # Bind mouse wheel scrolling
        self.scroll_canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-4>", self._on_mousewheel)
        self.scroll_canvas.bind("<Button-5>", self._on_mousewheel)

        self.content = ttk.Frame(self.scroll_canvas, padding=10, style="Shell.TFrame")
        self.content.grid_columnconfigure(0, weight=1)
        self.content.grid_rowconfigure(5, weight=1)
        self.scroll_canvas_frame = self.scroll_canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all")),
        )
        self.scroll_canvas.bind(
            "<Configure>",
            lambda event: self.scroll_canvas.itemconfig(self.scroll_canvas_frame, width=event.width, height=event.height),
        )

        header = ttk.Frame(self.content, padding=10, style="Shell.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        btns = ttk.Frame(header, style="Shell.TFrame")
        btns.grid(row=0, column=0, sticky="ew")
        btns.columnconfigure(0, weight=5)
        sim_group = self._make_button_group(btns, "Simulation Control", 0)
        element_group = self._make_button_group(btns, "Element Editing", 1)
        scenario_group = self._make_button_group(btns, "Line Config I/O", 2)
        mode_group = self._make_button_group(btns, "Mode", 3)

        self.start_btn = ttk.Button(sim_group, text="Start", command=self.on_start, style="Accent.TButton")
        self.start_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.stop_btn = ttk.Button(sim_group, text="II", command=self.on_stop, style="History.TButton", width=3)
        self.stop_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.reset_sim_btn = ttk.Button(sim_group, text="⟳", command=self.on_reset_simulation, style="History.TButton", width=3)
        self.reset_sim_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")
        for idx, scale in enumerate((1, 2, 5, 10, 100), start=3):
            button = ttk.Button(sim_group, text=f"x{scale}", command=lambda value=scale: self.set_time_scale(value))
            button.grid(row=1, column=idx, padx=2, pady=(2, 4), sticky="ew")
            self.time_scale_buttons[scale] = button
        self._update_time_scale_buttons()

        self.add_element_btn = ttk.Button(element_group, text="Add", command=self.open_add_element_dialog)
        self.add_element_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.delete_element_btn = ttk.Button(element_group, text="Delete", command=self.delete_selected_element)
        self.delete_element_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        element_group.grid_remove()
        self.undo_edit_btn = ttk.Button(element_group, text="↶", command=self.undo_canvas_edit, width=3, style="History.TButton")
        self.undo_edit_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")
        self.redo_edit_btn = ttk.Button(element_group, text="↷", command=self.redo_canvas_edit, width=3, style="History.TButton")
        self.redo_edit_btn.grid(row=1, column=3, padx=3, pady=(2, 4), sticky="ew")
        self._update_edit_history_buttons()
        self._update_run_pause_buttons()
        self.undo_edit_btn.grid_remove()
        self.redo_edit_btn.grid_remove()
        self.undo_edit_btn = ttk.Button(sim_group, text="↶", command=self.undo_canvas_edit, width=3, style="History.TButton")
        self.redo_edit_btn = ttk.Button(sim_group, text="↷", command=self.redo_canvas_edit, width=3, style="History.TButton")
        self.undo_edit_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.redo_edit_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.start_btn.grid_configure(column=2)
        self.stop_btn.grid_configure(column=3)
        self.reset_sim_btn.configure(text="⟳")
        self.reset_sim_btn.grid_configure(column=4)
        for idx, scale in enumerate((1, 2, 5, 10, 100), start=5):
            self.time_scale_buttons[scale].grid_configure(column=idx)
        for idx in range(10):
            sim_group.columnconfigure(idx, weight=1)
        for idx in range(3):
            scenario_group.columnconfigure(idx, weight=1)
        for idx in range(1):
            mode_group.columnconfigure(idx, weight=1)
        scenario_group.grid_configure(column=1)
        mode_group.grid_configure(column=2)
        btns.columnconfigure(1, weight=1)
        btns.columnconfigure(2, weight=0)
        btns.columnconfigure(3, weight=0)
        self._update_edit_history_buttons()
        self._update_run_pause_buttons()

        self.load_btn = ttk.Button(scenario_group, text="Load Line Config", command=self.on_load_scenario)
        self.load_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")
        self.save_scenario_btn = ttk.Button(scenario_group, text="Save YAML", command=self.on_save_scenario)
        self.save_scenario_btn.grid(row=1, column=1, padx=3, pady=(2, 4), sticky="ew")
        self.export_btn = ttk.Button(scenario_group, text="Export Report", command=self.on_export_report)
        self.export_btn.grid(row=1, column=2, padx=3, pady=(2, 4), sticky="ew")

        self.mode_toggle_btn = ttk.Button(mode_group, text="Chế độ thống kê", command=self.toggle_workspace_mode)
        self.mode_toggle_btn.grid(row=1, column=0, padx=3, pady=(2, 4), sticky="ew")

        clock_frame = ttk.Frame(header, padding=(10, 5, 10, 5), style="Clock.TFrame")
        clock_frame.grid(row=0, column=1, sticky="e", padx=(8, 0))
        ttk.Label(clock_frame, text="VIETNAM STANDARD TIME", style="ClockSmall.TLabel").pack(anchor="e")
        ttk.Label(clock_frame, textvariable=self.vn_clock_var, style="Clock.TLabel").pack(anchor="e")

        self.status_var = tk.StringVar(value="Status: stopped")
        self.scenario_var = tk.StringVar(value=f"Line config: {self.scenario['name']}")
        self.clock_var = tk.StringVar(value="Sim time: 0.0 s")
        self.summary_var = tk.StringVar(value="")
        status_row = ttk.Frame(self.content, padding=(10, 0, 10, 0), style="Shell.TFrame")
        status_row.grid(row=1, column=0, sticky="ew")
        status_row.grid_columnconfigure(0, weight=1)
        ttk.Label(status_row, textvariable=self.status_var, style="Status.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(status_row, textvariable=self.clock_var, style="Status.TLabel").grid(row=0, column=1, sticky="e", padx=(0, 12))
        ttk.Label(status_row, textvariable=self.scenario_var, style="Status.TLabel").grid(row=0, column=2, sticky="e")

        summary_frame = ttk.Frame(self.content, padding=(10, 8, 10, 0), style="Shell.TFrame")
        summary_frame.grid(row=2, column=0, sticky="ew")
        ttk.Label(summary_frame, textvariable=self.summary_var, style="Status.TLabel").pack(anchor="w")

        headway_frame = ttk.LabelFrame(self.content, text="Operation Mode", padding=8)
        headway_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(8, 0))
        ttk.Label(headway_frame, text="Mode").grid(row=0, column=0, padx=(0, 4), sticky="w")
        self.operation_mode_combo = ttk.Combobox(
            headway_frame,
            textvariable=self.operation_mode_var,
            values=("Fixed-block", "Headway target", "Timetable"),
            state="readonly",
            width=20,
        )
        self.operation_mode_combo.grid(row=0, column=1, padx=(0, 8), sticky="w")
        self.operation_mode_combo.bind("<<ComboboxSelected>>", lambda _event: self._refresh_operation_mode_controls())
        ttk.Label(headway_frame, text="Mechanism").grid(row=0, column=2, padx=(0, 4), sticky="w")
        ttk.Label(headway_frame, textvariable=self.operation_mechanism_var, style="Status.TLabel").grid(row=0, column=3, padx=(0, 12), sticky="w")
        self.blocks_label = ttk.Label(headway_frame, text="Blocks/section")
        self.blocks_entry = ttk.Entry(headway_frame, textvariable=self.blocks_per_section_var, width=8)
        self.target_label = ttk.Label(headway_frame, text="Target seconds")
        self.target_entry = ttk.Entry(headway_frame, textvariable=self.headway_target_var, width=10)
        self.timetable_label = ttk.Label(headway_frame, text="Schedule file")
        self.timetable_entry = ttk.Entry(headway_frame, textvariable=self.timetable_file_var, width=42)
        self.timetable_button = ttk.Button(headway_frame, text="Load YAML/MD", command=self.load_timetable_file)
        self.timetable_set_after_now_button = ttk.Button(
            headway_frame,
            text="Set +1.5 min",
            command=self.set_timetable_after_now,
        )
        ttk.Button(headway_frame, text="Apply + Reset", command=self.apply_headway_block_settings).grid(row=0, column=9, padx=(0, 6))
        ttk.Button(headway_frame, text="Reload Values", command=self.reload_headway_block_values).grid(row=0, column=10)
        ttk.Label(headway_frame, textvariable=self.operation_selected_status_var, style="Status.TLabel").grid(
            row=1, column=0, columnspan=11, sticky="w", pady=(6, 0)
        )
        self._refresh_operation_mode_controls()

        workspace = ttk.PanedWindow(self.content, orient=tk.HORIZONTAL)
        workspace.grid(row=4, column=0, sticky="nsew", padx=6, pady=(8, 0))
        self.workspace = workspace
        workspace.bind("<Configure>", lambda _event: self.after_idle(self._fit_workspace_panes), add="+")
        self.content.grid_rowconfigure(4, weight=1)
        self.content.grid_rowconfigure(5, weight=0)

        side_shell = ttk.Frame(workspace, padding=(0, 0, 6, 0), style="Shell.TFrame")
        side_shell.columnconfigure(0, weight=1)
        side_shell.rowconfigure(0, weight=1)
        self.side_toolbar_canvas = tk.Canvas(side_shell, background=APP_THEME["workspace"], highlightthickness=0, width=int(190 * self.scale_factor))
        self.side_toolbar_canvas.grid(row=0, column=0, sticky="nsew")
        side_scrollbar = ttk.Scrollbar(side_shell, orient="vertical", command=self.side_toolbar_canvas.yview)
        side_scrollbar.grid(row=0, column=1, sticky="ns")
        self.side_toolbar_canvas.configure(yscrollcommand=side_scrollbar.set)
        side_toolbar = ttk.Frame(self.side_toolbar_canvas, style="Shell.TFrame")
        self.side_toolbar_window = self.side_toolbar_canvas.create_window((0, 0), window=side_toolbar, anchor="nw")
        side_toolbar.bind("<Configure>", lambda _event: self.side_toolbar_canvas.configure(scrollregion=self.side_toolbar_canvas.bbox("all")))
        self.side_toolbar_canvas.bind("<Configure>", lambda event: self.side_toolbar_canvas.itemconfigure(self.side_toolbar_window, width=event.width))
        side_toolbar.columnconfigure(0, weight=1)
        self._bind_side_toolbar_scroll(self.side_toolbar_canvas)

        element_side = ttk.LabelFrame(side_toolbar, text="Element Editing", padding=6)
        element_side.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        element_side.columnconfigure(0, weight=1)
        self.add_element_btn = ttk.Button(element_side, text="Add", command=self.open_add_element_dialog)
        self.add_element_btn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self.delete_element_btn = ttk.Button(element_side, text="Delete", command=self.delete_selected_element)
        self.delete_element_btn.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(element_side, text="Depot + Train", command=self.add_train_to_selected_source).grid(row=2, column=0, sticky="ew", padx=2, pady=(8, 2))
        ttk.Button(element_side, text="Depot - Train", command=self.remove_train_from_selected_source).grid(row=3, column=0, sticky="ew", padx=2, pady=2)

        faults_side = ttk.LabelFrame(side_toolbar, text="Selected Train Faults", padding=6)
        faults_side.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        faults_side.columnconfigure(0, weight=1)
        ttk.Button(faults_side, text="DCS Loss", command=self.toggle_all_dcs_loss, style="Danger.TButton").grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(faults_side, text="Clear Faults", command=self.clear_all_faults, style="Inactive.TButton").grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        self.emergency_fault_frame = ttk.LabelFrame(faults_side, text="Emergency Stop", padding=4)
        self.emergency_fault_frame.grid(row=2, column=0, sticky="ew", padx=2, pady=(6, 2))
        self.atp_fault_frame = ttk.LabelFrame(faults_side, text="ATP Fault", padding=4)
        self.atp_fault_frame.grid(row=3, column=0, sticky="ew", padx=2, pady=2)
        self.ato_fault_frame = ttk.LabelFrame(faults_side, text="ATO Fault", padding=4)
        self.ato_fault_frame.grid(row=4, column=0, sticky="ew", padx=2, pady=2)
        for frame in (self.emergency_fault_frame, self.atp_fault_frame, self.ato_fault_frame):
            frame.columnconfigure(0, weight=1)
        self.train_fault_buttons: Dict[str, Dict[str, ttk.Button]] = {}
        self._bind_side_toolbar_tree(side_toolbar)

        workspace.add(side_shell, weight=0)

        center_workspace = ttk.PanedWindow(workspace, orient=tk.VERTICAL)
        self.center_workspace = center_workspace
        workspace.add(center_workspace, weight=4)
        canvas_shell = ttk.Frame(center_workspace, style="Shell.TFrame")
        canvas_shell.columnconfigure(0, weight=1)
        canvas_shell.rowconfigure(0, weight=1)
        self.ats_overview_panel = ATSOverviewPanel(
            canvas_shell,
            self.scale_factor,
            on_select=self.on_ats_element_selected,
            on_edit=self.open_edit_element_dialog,
        )
        self.ats_overview_panel.grid(row=0, column=0, sticky="nsew")
        center_workspace.add(canvas_shell, weight=3)

        trains_shell = ttk.Frame(center_workspace, style="Shell.TFrame")
        self.trains_shell = trains_shell
        self.trains_tab_collapsed = False
        self._trains_restore_sash = None
        trains_shell.columnconfigure(0, weight=1)
        trains_shell.rowconfigure(0, weight=1)
        self.ats_tabs = ttk.Notebook(trains_shell, style="Shell.TNotebook")
        self.ats_tabs.grid(row=0, column=0, sticky="nsew")
        self.ats_tabs.enable_traversal()
        self.ats_tabs.bind("<Button-1>", self._on_trains_tab_click, add="+")
        trains_tab = ttk.Frame(self.ats_tabs, style="Shell.TFrame")
        trains_tab.columnconfigure(0, weight=1)
        trains_tab.rowconfigure(0, weight=1)
        trains_tab.rowconfigure(1, weight=0)
        self.trains_canvas = tk.Canvas(trains_tab, background=APP_THEME["workspace"], highlightthickness=0, height=360)
        self.trains_scrollbar = ttk.Scrollbar(trains_tab, orient="horizontal", command=self.trains_canvas.xview)
        self.trains_scrollable_frame = ttk.Frame(self.trains_canvas, style="Shell.TFrame")
        self.trains_scrollable_frame.bind(
            "<Configure>",
            lambda _event: self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all")),
        )
        self.trains_window = self.trains_canvas.create_window((0, 0), window=self.trains_scrollable_frame, anchor="nw")
        self.trains_canvas.bind("<Configure>", self._resize_train_boards, add="+")
        self.trains_canvas.configure(xscrollcommand=self.trains_scrollbar.set)
        self._bind_train_horizontal_scroll(self.trains_canvas)
        self._bind_train_horizontal_scroll(self.trains_scrollable_frame)
        self.trains_canvas.grid(row=0, column=0, sticky="nsew")
        self.trains_scrollbar.grid(row=1, column=0, sticky="ew")
        self.ats_tabs.add(trains_tab, text="Trains")
        center_workspace.add(trains_shell, weight=2)

        dock_tabs = ttk.Notebook(workspace, style="Shell.TNotebook")
        self.dock_tabs = dock_tabs
        workspace.add(dock_tabs, weight=2)
        infra_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        engineering_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        diagnostics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        dataflow_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        analytics_tab = ttk.Frame(dock_tabs, style="Shell.TFrame")
        for tab in (infra_tab, engineering_tab, diagnostics_tab, dataflow_tab, analytics_tab):
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(0, weight=1)
        self.infrastructure_panel = InfrastructurePanel(infra_tab, self.scale_factor)
        self.infrastructure_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=(int(6 * self.scale_factor), int(3 * self.scale_factor)))
        self.engineering_panel = EngineeringPanel(engineering_tab, self.scale_factor)
        self.engineering_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.diagnostics_panel = DiagnosticsPanel(diagnostics_tab, self.scale_factor)
        self.diagnostics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.dataflow_panel = DataFlowPanel(dataflow_tab, self.scale_factor)
        self.dataflow_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.analytics_panel = AnalyticsPanel(analytics_tab, self.scale_factor)
        self.analytics_panel.grid(row=0, column=0, sticky="nsew", padx=int(6 * self.scale_factor), pady=int(6 * self.scale_factor))
        self.limits_panel = SpeedLimitsPanel(infra_tab, self.scale_factor)
        dock_tabs.add(infra_tab, text="Infrastructure")
        dock_tabs.add(engineering_tab, text="Engineering")
        dock_tabs.add(diagnostics_tab, text="Diagnostics")
        dock_tabs.add(dataflow_tab, text="Dataflow")
        dock_tabs.add(analytics_tab, text="Analytics")

        button_row = ttk.Frame(self.content, padding=(0, 8, 0, 0), style="Shell.TFrame")
        button_row.grid(row=6, column=0, sticky="ew")
        button_row.grid_columnconfigure(0, weight=1)

        self.panels: Dict[str, TrainPanel] = {}
        self._create_aux_windows()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.engineering_panel.update_data(self.sim)
        self.diagnostics_panel.update_data(self.sim, self.event_log)
        self.dataflow_panel.update_data(self.sim)
        self.analytics_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self.normal_widgets = [status_row, summary_frame, headway_frame, workspace, button_row]
        self.monte_carlo_panel = MonteCarloPanel(self.content, self)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind_all("<MouseWheel>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-4>", self._on_global_mousewheel, add="+")
        self.bind_all("<Button-5>", self._on_global_mousewheel, add="+")
        self.after(100, self.tick)
        self.after(100, self._update_vietnam_clock)
        self.after_idle(self._fit_workspace_panes)

    def _make_button_group(self, master: tk.Widget, title: str, column: int) -> ttk.Frame:
        group = ttk.Frame(master, padding=(8, 4, 8, 4), style="ToolbarGroup.TFrame")
        group.grid(row=0, column=column, sticky="ew", padx=(0, 8))
        master.columnconfigure(column, weight=1 if title == "Simulation Control" else 0)
        ttk.Label(group, text=title, style="ToolbarTitle.TLabel").grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 2))
        return group

    def _bind_side_toolbar_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_side_scroll_bound", False):
            return
        setattr(widget, "_cbtc_side_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_side_toolbar_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_side_toolbar_mousewheel, add="+")

    def _bind_side_toolbar_tree(self, widget: tk.Widget):
        self._bind_side_toolbar_scroll(widget)
        for child in widget.winfo_children():
            self._bind_side_toolbar_tree(child)

    def _on_side_toolbar_mousewheel(self, event):
        if not hasattr(self, "side_toolbar_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -4
        elif getattr(event, "num", None) == 5:
            delta = 4
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -4 if delta_raw > 0 else 4
        self.side_toolbar_canvas.yview_scroll(delta, "units")
        return "break"

    def _fit_workspace_panes(self):
        if not hasattr(self, "workspace"):
            return
        self.update_idletasks()
        width = max(1, self.workspace.winfo_width())
        height = max(1, getattr(self, "center_workspace", self.workspace).winfo_height())
        left_w = int(210 * self.scale_factor)
        right_w = max(int(390 * self.scale_factor), min(int(520 * self.scale_factor), int(width * 0.28)))
        try:
            self.workspace.sashpos(0, left_w)
            self.workspace.sashpos(1, max(left_w + 480, width - right_w))
        except tk.TclError:
            pass
        try:
            if not getattr(self, "trains_tab_collapsed", False):
                self.center_workspace.sashpos(0, max(int(300 * self.scale_factor), int(height * 0.52)))
        except tk.TclError:
            pass
        self._resize_train_boards()

    def _update_vietnam_clock(self):
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        display_clock_s = self.sim.timetable_display_clock_s() if hasattr(self, "sim") else None
        if display_clock_s is None:
            self.vn_clock_var.set(now.strftime("%H:%M:%S  %Y-%m-%d  UTC+7"))
        else:
            total_s = int(display_clock_s) % (24 * 3600)
            hour = total_s // 3600
            minute = (total_s % 3600) // 60
            second = total_s % 60
            self.vn_clock_var.set(
                f"{hour:02d}:{minute:02d}:{second:02d}  {now:%Y-%m-%d}  UTC+7  x{self.time_scale}"
            )
        self.after(1000, self._update_vietnam_clock)

    def _update_mode_toggle_button(self):
        if not hasattr(self, "mode_toggle_btn"):
            return
        if self.current_workspace_mode == "monte_carlo":
            self.mode_toggle_btn.configure(text="Chế độ mô phỏng")
        else:
            self.mode_toggle_btn.configure(text="Chế độ thống kê")

    def toggle_workspace_mode(self):
        if self.current_workspace_mode == "normal":
            self.show_monte_carlo_mode()
        else:
            self.show_normal_mode()

    def show_monte_carlo_mode(self):
        if self.current_workspace_mode == "monte_carlo":
            return
        self.current_workspace_mode = "monte_carlo"
        self.sim.stop()
        for widget in self.normal_widgets:
            widget.grid_remove()
        self.monte_carlo_panel.grid(row=1, column=0, rowspan=6, sticky="nsew")
        self._update_mode_toggle_button()
        self.status_var.set("Status: chế độ thống kê")

    def show_normal_mode(self):
        if self.current_workspace_mode == "normal":
            return
        self.current_workspace_mode = "normal"
        self.monte_carlo_panel.grid_remove()
        for widget in self.normal_widgets:
            widget.grid()
        self._update_mode_toggle_button()
        self.status_var.set("Status: chế độ mô phỏng")

    def _operation_mode_from_scenario(self) -> str:
        headway = self.scenario.get("headway", {}) if isinstance(self.scenario.get("headway", {}), dict) else {}
        mode = str(headway.get("mode", "off")).lower()
        block_mode = str(getattr(self, "sim", None).block_mode if hasattr(self, "sim") else self.scenario.get("block_mode", "")).lower()
        if block_mode in {"fixed", "fixed_block"}:
            return "Fixed-block"
        if mode == "timetable":
            return "Timetable"
        return "Headway target"

    def _operation_mode_key(self) -> str:
        value = self.operation_mode_var.get().strip().lower()
        if value.startswith("fixed") or value.startswith("1"):
            return "fixed_block"
        if value.startswith("time") or value.startswith("3"):
            return "timetable"
        return "headway_target"

    def _operation_mode_label_for_key(self, key: str) -> str:
        labels = {
            "fixed_block": "Fixed-block",
            "headway_target": "Headway target",
            "timetable": "Timetable",
        }
        return labels.get(key, "Headway target")

    def _active_operation_mode_key(self) -> str:
        if getattr(self.sim, "block_mode", "moving_block") == "fixed_block":
            return "fixed_block"
        mode = str(getattr(self.sim.headway_manager, "mode", "fixed")).lower()
        if mode == "timetable":
            return "timetable"
        return "headway_target"

    def _update_operation_mode_status(self):
        if not hasattr(self, "operation_selected_status_var"):
            return
        selected_key = self._operation_mode_key()
        active_key = self._active_operation_mode_key()
        selected_label = self._operation_mode_label_for_key(selected_key)
        active_label = self._operation_mode_label_for_key(active_key)
        pending = " | Chưa apply" if selected_key != active_key else ""
        self.operation_selected_status_var.set(
            f"Đang chọn: {selected_label} | Sẽ chạy: {selected_label} | Đang chạy: {active_label}{pending}"
        )

    def _refresh_operation_mode_controls(self):
        for widget in (
            getattr(self, "blocks_label", None),
            getattr(self, "blocks_entry", None),
            getattr(self, "target_label", None),
            getattr(self, "target_entry", None),
            getattr(self, "timetable_label", None),
            getattr(self, "timetable_entry", None),
            getattr(self, "timetable_button", None),
            getattr(self, "timetable_set_after_now_button", None),
        ):
            if widget is not None:
                widget.grid_remove()

        mode = self._operation_mode_key()
        self.operation_mechanism_var.set("fixed_block" if mode == "fixed_block" else "moving_block")
        if mode == "fixed_block":
            self.blocks_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.blocks_entry.grid(row=0, column=5, padx=(0, 8), sticky="w")
        elif mode == "headway_target":
            self.target_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.target_entry.grid(row=0, column=5, padx=(0, 8), sticky="w")
        elif mode == "timetable":
            self.timetable_label.grid(row=0, column=4, padx=(0, 4), sticky="w")
            self.timetable_entry.grid(row=0, column=5, columnspan=2, padx=(0, 8), sticky="ew")
            self.timetable_button.grid(row=0, column=7, padx=(0, 8), sticky="w")
            self.timetable_set_after_now_button.grid(row=0, column=8, padx=(0, 8), sticky="w")
        self._update_operation_mode_status()

    def _extract_timetable_seconds(self, payload: Any) -> List[float]:
        if isinstance(payload, dict):
            for key in ("timetable_s", "timetable", "schedule", "departures", "times"):
                if key in payload:
                    return self._extract_timetable_seconds(payload[key])
            return []
        if isinstance(payload, list):
            values: List[float] = []
            for item in payload:
                if isinstance(item, dict):
                    for key in ("time_s", "depart_s", "departure_s", "time", "depart", "departure"):
                        if key in item:
                            try:
                                values.append(float(item[key]))
                                break
                            except (TypeError, ValueError):
                                pass
                else:
                    try:
                        values.append(float(item))
                    except (TypeError, ValueError):
                        pass
            return sorted(value for value in values if value >= 0.0)
        return []

    def _parse_timetable_clock_s(self, value: Any) -> float | None:
        text = str(value or "").strip()
        if not text or text in {"--", "-"}:
            return None
        match = re.search(r"\b(\d{1,2}):(\d{2})(?::(\d{2}))?\b", text)
        if not match:
            return None
        hour = int(match.group(1))
        minute = int(match.group(2))
        second = int(match.group(3) or 0)
        if minute >= 60 or second >= 60:
            return None
        return float(hour * 3600 + minute * 60 + second)

    def _parse_timetable_dwell_s(self, value: Any) -> float | None:
        text = str(value or "").strip().lower()
        if not text or text in {"--", "-"}:
            return None
        match = re.search(r"(\d+(?:\.\d+)?)\s*s", text)
        if match:
            return float(match.group(1))
        try:
            return float(text)
        except ValueError:
            return None

    def _looks_like_train_id(self, text: str) -> bool:
        return re.fullmatch(r"T\d+", text.strip(), flags=re.IGNORECASE) is not None

    def _looks_like_station_id(self, text: str) -> bool:
        return re.fullmatch(r"S\d+", text.strip(), flags=re.IGNORECASE) is not None

    def _normalize_markdown_cell(self, text: str) -> str:
        return re.sub(r"<[^>]+>", " ", text).strip()

    def _append_timetable_record(
        self,
        records: List[Dict[str, Any]],
        train_id: str | None,
        station: str | None,
        arrival: Any,
        dwell: Any,
        departure: Any,
        profile: Any,
        note: Any = "",
    ) -> None:
        if not train_id or not station:
            return
        arrival_clock_s = self._parse_timetable_clock_s(arrival)
        departure_clock_s = self._parse_timetable_clock_s(departure)
        records.append(
            {
                "train_id": str(train_id).strip(),
                "station": str(station).strip(),
                "arrival_text": str(arrival or "").strip(),
                "arrival_clock_s": arrival_clock_s,
                "arrival_time_s": arrival_clock_s,
                "dwell_text": str(dwell or "").strip(),
                "dwell_s": self._parse_timetable_dwell_s(dwell),
                "departure_text": str(departure or "").strip(),
                "departure_clock_s": departure_clock_s,
                "departure_time_s": departure_clock_s,
                "profile": str(profile or "").strip(),
                "note": str(note or "").strip(),
            }
        )

    def _extract_vietnamese_markdown_timetable(self, text: str) -> Tuple[List[float], List[Dict[str, Any]]]:
        records: List[Dict[str, Any]] = []
        current_train: str | None = None
        plain_lines: List[str] = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            if "|" in line:
                cells = [self._normalize_markdown_cell(cell) for cell in line.strip("|").split("|")]
                cells = [cell for cell in cells if cell]
                if not cells or all(set(cell) <= {"-", ":"} for cell in cells):
                    continue
                header_text = " ".join(cells).lower()
                if "tàu" in header_text or "arrival" in header_text or "giờ" in header_text:
                    continue
                if len(cells) >= 6:
                    if self._looks_like_train_id(cells[0]):
                        current_train = cells[0].upper()
                        offset = 1
                    else:
                        offset = 0
                    if len(cells) - offset >= 5 and self._looks_like_station_id(cells[offset]):
                        note = " ".join(cells[offset + 5 :]) if len(cells) - offset > 5 else ""
                        self._append_timetable_record(
                            records,
                            current_train,
                            cells[offset],
                            cells[offset + 1],
                            cells[offset + 2],
                            cells[offset + 3],
                            cells[offset + 4],
                            note,
                        )
                continue
            plain_lines.append(line)

        idx = 0
        while idx < len(plain_lines):
            line = plain_lines[idx]
            if self._looks_like_train_id(line):
                current_train = line.upper()
                idx += 1
                continue
            if self._looks_like_station_id(line) and idx + 4 < len(plain_lines):
                station = line
                arrival = plain_lines[idx + 1]
                dwell = plain_lines[idx + 2]
                departure = plain_lines[idx + 3]
                profile = plain_lines[idx + 4]
                note_parts: List[str] = []
                idx += 5
                while idx < len(plain_lines) and not self._looks_like_train_id(plain_lines[idx]) and not self._looks_like_station_id(plain_lines[idx]):
                    note_parts.append(plain_lines[idx])
                    idx += 1
                self._append_timetable_record(records, current_train, station, arrival, dwell, departure, profile, " ".join(note_parts))
                continue
            idx += 1

        first_departures: Dict[str, float] = {}
        for record in records:
            departure_s = record.get("departure_time_s")
            train_id = str(record.get("train_id", ""))
            if departure_s is None or not train_id:
                continue
            first_departures.setdefault(train_id, float(departure_s))
        if not first_departures:
            return [], records
        origin_s = min(first_departures.values())
        timetable_s = sorted(max(0.0, value - origin_s) for value in first_departures.values())
        for record in records:
            for key in ("arrival_time_s", "departure_time_s"):
                if record.get(key) is not None:
                    record[key] = max(0.0, float(record[key]) - origin_s)
        return timetable_s, records

    def _vietnam_clock_seconds(self) -> float:
        vietnam_tz = timezone(timedelta(hours=7))
        now = datetime.now(vietnam_tz)
        return float(now.hour * 3600 + now.minute * 60 + now.second + now.microsecond / 1_000_000.0)

    def _clock_delay_from_now_s(self, clock_s: float, now_clock_s: float) -> float:
        day_s = 24.0 * 3600.0
        return (float(clock_s) - float(now_clock_s)) % day_s

    def _wall_clock_timetable_values(
        self,
        records: List[Dict[str, Any]],
        fallback_values: List[float],
        now_clock_s: float | None = None,
    ) -> Tuple[List[float], List[Dict[str, Any]]]:
        if not records:
            return fallback_values, records
        now_clock_s = self._vietnam_clock_seconds() if now_clock_s is None else float(now_clock_s)
        first_departures: Dict[str, float] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            clock_s = record.get("departure_clock_s")
            if train_id and clock_s is not None:
                first_departures.setdefault(train_id, float(clock_s))
        if not first_departures:
            return fallback_values, records

        values = sorted(self._clock_delay_from_now_s(clock_s, now_clock_s) for clock_s in first_departures.values())
        adjusted_records: List[Dict[str, Any]] = []
        for record in records:
            adjusted = dict(record)
            for clock_key, time_key in (("arrival_clock_s", "arrival_time_s"), ("departure_clock_s", "departure_time_s")):
                clock_s = adjusted.get(clock_key)
                if clock_s is not None:
                    adjusted[time_key] = self._clock_delay_from_now_s(float(clock_s), now_clock_s)
            adjusted_records.append(adjusted)
        return values, adjusted_records

    def _format_timetable_clock_s(self, clock_s: float) -> str:
        total_s = int(round(float(clock_s))) % int(24.0 * 3600.0)
        hour = total_s // 3600
        minute = (total_s % 3600) // 60
        second = total_s % 60
        return f"{hour:02d}:{minute:02d}:{second:02d}"

    def set_timetable_after_now(self):
        headway = dict(self.scenario.get("headway", {}) or {})
        values = [float(value) for value in (headway.get("timetable_s", []) or [])]
        records = [dict(record) for record in (headway.get("timetable_records", []) or [])]
        if not values and not records:
            self.status_var.set("Status: load a timetable before setting +1.5 min")
            return

        now_clock_s = self._vietnam_clock_seconds()
        target_first_departure_s = 90.0
        first_departures: Dict[str, float] = {}
        for record in records:
            train_id = str(record.get("train_id", "")).strip()
            departure_s = record.get("departure_time_s")
            if train_id and departure_s is not None:
                first_departures.setdefault(train_id, float(departure_s))
        if first_departures:
            origin_s = min(first_departures.values())
        elif values:
            origin_s = min(values)
        else:
            self.status_var.set("Status: timetable has no departure times to shift")
            return

        if values:
            shifted_values = sorted(max(0.0, target_first_departure_s + value - origin_s) for value in values)
        else:
            shifted_values = sorted(
                max(0.0, target_first_departure_s + value - origin_s)
                for value in first_departures.values()
            )
        shifted_records: List[Dict[str, Any]] = []
        for record in records:
            shifted = dict(record)
            for clock_key, time_key, text_key in (
                ("arrival_clock_s", "arrival_time_s", "arrival_text"),
                ("departure_clock_s", "departure_time_s", "departure_text"),
            ):
                old_time_s = shifted.get(time_key)
                if old_time_s is None:
                    continue
                new_time_s = max(0.0, target_first_departure_s + float(old_time_s) - origin_s)
                new_clock_s = (now_clock_s + new_time_s) % (24.0 * 3600.0)
                shifted[time_key] = new_time_s
                shifted[clock_key] = new_clock_s
                shifted[text_key] = self._format_timetable_clock_s(new_clock_s)
            shifted_records.append(shifted)

        headway["mode"] = "timetable"
        headway["timetable_s"] = shifted_values
        if shifted_records:
            headway["timetable_records"] = shifted_records
            headway["timetable_wall_clock"] = True
        headway["timetable_loaded_clock_s"] = now_clock_s
        self.scenario["headway"] = headway
        self.sim.scenario["headway"] = deepcopy(headway)
        self.operation_mode_var.set("Timetable")
        self._refresh_operation_mode_controls()
        self.on_reset_simulation()
        first_clock = self._format_timetable_clock_s(now_clock_s + target_first_departure_s)
        self.status_var.set(f"Status: timetable first departure set to {first_clock} (+1.5 min) and reset simulation")

    def load_timetable_file(self):
        path = filedialog.askopenfilename(
            title="Load timetable",
            filetypes=(("Timetable files", "*.yaml *.yml *.md"), ("All files", "*.*")),
        )
        if not path:
            return
        self.timetable_file_var.set(path)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                text = fh.read()
            if path.lower().endswith((".yaml", ".yml")):
                values = self._extract_timetable_seconds(yaml.safe_load(text) or {})
            else:
                timetable_loaded_clock_s = self._vietnam_clock_seconds()
                values, records = self._extract_vietnamese_markdown_timetable(text)
                if not values:
                    values = []
                    for token in text.replace(",", " ").replace("|", " ").split():
                        try:
                            values.append(float(token))
                        except ValueError:
                            continue
                    values = sorted(value for value in values if value >= 0.0)
                    records = []
                else:
                    values, records = self._wall_clock_timetable_values(records, values, now_clock_s=timetable_loaded_clock_s)
            headway = dict(self.scenario.get("headway", {}) or {})
            headway["mode"] = "timetable"
            headway["timetable_s"] = values
            headway["timetable_file"] = path
            if path.lower().endswith(".md"):
                headway["timetable_records"] = records
                headway["timetable_wall_clock"] = True
                headway["timetable_loaded_clock_s"] = timetable_loaded_clock_s
            self.scenario["headway"] = headway
            self.sim.scenario["headway"] = deepcopy(headway)
            self.operation_mode_var.set("Timetable")
            self._refresh_operation_mode_controls()
            self.on_reset_simulation()
            self.status_var.set(f"Status: loaded timetable with {len(values)} train departures and reset simulation")
        except Exception as exc:
            self.status_var.set(f"Status: failed to load timetable: {exc}")

    def reload_headway_block_values(self):
        self.operation_mode_var.set(self._operation_mode_from_scenario())
        headway = self.scenario.get("headway", {}) if isinstance(self.scenario.get("headway", {}), dict) else {}
        self.headway_target_var.set(str(headway.get("target_headway_s", 180.0)))
        self.timetable_file_var.set(str(headway.get("timetable_file", "")))
        self.blocks_per_section_var.set(str(self.scenario.get("capacity_baseline", {}).get("blocks_per_section", 4)))
        self._refresh_operation_mode_controls()
        self.status_var.set("Status: operation mode values reloaded")

    def apply_headway_block_settings(self):
        headway = dict(self.scenario.get("headway", {}) or {})
        mode = self._operation_mode_key()
        if mode == "fixed_block":
            headway["mode"] = "off"
            self.scenario["block_mode"] = "fixed_block"
        elif mode == "headway_target":
            headway["mode"] = "fixed"
            try:
                headway["target_headway_s"] = max(1.0, float(self.headway_target_var.get()))
            except ValueError:
                headway["target_headway_s"] = 180.0
                self.headway_target_var.set("180.0")
            self.scenario["block_mode"] = "moving_block"
        elif mode == "timetable":
            headway["mode"] = "timetable"
            headway["timetable_file"] = self.timetable_file_var.get()
            self.scenario["block_mode"] = "moving_block"
        self.scenario["headway"] = headway
        capacity = dict(self.scenario.get("capacity_baseline", {}) or {})
        try:
            capacity["blocks_per_section"] = max(1, int(float(self.blocks_per_section_var.get())))
        except ValueError:
            capacity["blocks_per_section"] = 4
            self.blocks_per_section_var.set("4")
        self.scenario["capacity_baseline"] = capacity
        self.on_reset_simulation()
        self.ats_overview_panel.update_data(self.sim)
        self.status_var.set("Status: applied operation mode and reset simulation")

    def _configure_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Scale font sizes based on DPI scaling
        base_font_size = int(9 * self.scale_factor)
        button_font_size = int(11 * self.scale_factor)
        history_button_font_size = int(15 * self.scale_factor)
        header_font_size = int(16 * self.scale_factor)
        section_font_size = int(11 * self.scale_factor)
        status_font_size = int(9 * self.scale_factor)
        muted_font_size = int(8 * self.scale_factor)
        card_title_font_size = int(11 * self.scale_factor)
        tab_font_size = int(10 * self.scale_factor)

        style.configure("TFrame", background=APP_THEME["workspace"])
        style.configure("TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", base_font_size))
        style.configure("Shell.TFrame", background=APP_THEME["workspace"])
        style.configure("Panel.TFrame", background=APP_THEME["workspace"])
        style.configure("Card.TFrame", background=APP_THEME["card"], relief="solid", borderwidth=1)
        style.configure("SubCard.TFrame", background=APP_THEME["card_alt"])
        style.configure("ToolbarGroup.TFrame", background=APP_THEME["panel"], relief="ridge", borderwidth=1)
        style.configure("Clock.TFrame", background=APP_THEME["accent"], relief="solid", borderwidth=1)
        style.configure("TLabelframe", background=APP_THEME["panel"], foreground=APP_THEME["text"], bordercolor=APP_THEME["border"], lightcolor=APP_THEME["border"], darkcolor=APP_THEME["border"])
        style.configure("TLabelframe.Label", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Vertical.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("Horizontal.TScrollbar", background=APP_THEME["button"], troughcolor=APP_THEME["bg"], bordercolor=APP_THEME["border"], arrowcolor=APP_THEME["border"])
        style.configure("HeaderTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", header_font_size, "bold"))
        style.configure("SectionTitle.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["accent"], font=("Consolas", section_font_size, "bold"))
        style.configure("CardTitle.TLabel", background=APP_THEME["card"], foreground=APP_THEME["accent"], font=("Consolas", card_title_font_size, "bold"))
        style.configure("ToolbarTitle.TLabel", background=APP_THEME["panel"], foreground=APP_THEME["accent"], font=("Consolas", base_font_size, "bold"))
        style.configure("Shell.TNotebook", background=APP_THEME["workspace"], borderwidth=0)
        style.configure(
            "Shell.TNotebook.Tab",
            padding=(int(10 * self.scale_factor), int(6 * self.scale_factor)),
            font=("Consolas", tab_font_size, "bold"),
            background=APP_THEME["panel_alt"],
            foreground=APP_THEME["muted"],
        )
        style.map(
            "Shell.TNotebook.Tab",
            background=[("selected", APP_THEME["card"]), ("active", APP_THEME["button_hover"])],
            foreground=[("selected", APP_THEME["accent"]), ("active", APP_THEME["text"])],
        )
        style.configure("Muted.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["muted"], font=("Consolas", muted_font_size))
        style.configure("Status.TLabel", background=APP_THEME["workspace"], foreground=APP_THEME["text"], font=("Consolas", status_font_size))
        style.configure("Clock.TLabel", background=APP_THEME["accent"], foreground="#fff8ed", font=("Consolas", int(15 * self.scale_factor), "bold"))
        style.configure("ClockSmall.TLabel", background=APP_THEME["accent"], foreground="#ffe7c7", font=("Consolas", int(7 * self.scale_factor), "bold"))
        style.configure(
            "TEntry",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor=APP_THEME["button_hover"],
            darkcolor=APP_THEME["border"],
        )
        style.configure(
            "TCombobox",
            fieldbackground=APP_THEME["log_bg"],
            foreground=APP_THEME["text"],
            background=APP_THEME["button"],
            bordercolor=APP_THEME["border"],
            arrowcolor=APP_THEME["border"],
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", APP_THEME["log_bg"])],
            background=[("active", APP_THEME["button_hover"]), ("readonly", APP_THEME["button"])],
        )
        style.configure(
            "TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            lightcolor="#ffd979",
            darkcolor="#b85c2d",
            relief="raised",
        )
        style.map(
            "TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Accent.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["accent"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Accent.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#d8782d")],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Active.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="ridge",
        )
        style.map(
            "Active.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", "#ffe08a"), ("!disabled", APP_THEME["button_active"])],
            relief=[("pressed", "sunken"), ("!pressed", "ridge")],
        )
        style.configure(
            "History.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["button"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "History.TButton",
            background=[("pressed", APP_THEME["button_pressed"]), ("active", APP_THEME["button_hover"]), ("disabled", APP_THEME["button_inactive"])],
            foreground=[("disabled", APP_THEME["muted"]), ("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "RunActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["run_active"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "RunActive.TButton",
            background=[("pressed", APP_THEME["accent_pressed"]), ("active", "#ffc15c"), ("!disabled", APP_THEME["run_active"])],
            foreground=[("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseActive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["pause_active"],
            foreground="#1f2730",
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", "#1f2730")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "PauseHistoryActive.TButton",
            font=("Consolas", history_button_font_size, "bold"),
            padding=(10, 1),
            background=APP_THEME["pause_active"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "PauseHistoryActive.TButton",
            background=[("pressed", "#d8782d"), ("active", "#ffe08a"), ("!disabled", APP_THEME["pause_active"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Danger.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(9, 5),
            background=APP_THEME["danger"],
            foreground="#fff8ed",
            bordercolor=APP_THEME["danger_pressed"],
            relief="raised",
        )
        style.map(
            "Danger.TButton",
            background=[("pressed", APP_THEME["danger_pressed"]), ("active", "#e85d4f"), ("!disabled", APP_THEME["danger"])],
            foreground=[("disabled", "#f0caca"), ("!disabled", "#fff8ed")],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "Inactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["muted"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "Inactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["muted"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )
        style.configure(
            "StartInactive.TButton",
            font=("Consolas", button_font_size, "bold"),
            padding=(10, 5),
            background=APP_THEME["button_inactive"],
            foreground=APP_THEME["text"],
            bordercolor=APP_THEME["border"],
            relief="raised",
        )
        style.map(
            "StartInactive.TButton",
            background=[("pressed", "#c18a58"), ("active", "#e7bd8c"), ("!disabled", APP_THEME["button_inactive"])],
            foreground=[("!disabled", APP_THEME["text"])],
            relief=[("pressed", "sunken"), ("!pressed", "raised")],
        )

    def _reset_runtime_buffers(self):
        self.time_history.clear()
        self.position_history = {train.id: deque(maxlen=240) for train in self.sim.trains}
        self.event_log.clear()
        self.prev_train_snapshot = {}

    def _on_mousewheel(self, event):
        """Handle mouse wheel scroll events (Windows and Linux)."""
        # Windows uses event.delta (positive = up, negative = down)
        # Linux uses event.num (4 = up, 5 = down)
        if event.num == 4:  # Linux scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.num == 5:  # Linux scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        elif event.delta > 0:  # Windows scroll up
            self.scroll_canvas.yview_scroll(-3, "units")
        elif event.delta < 0:  # Windows scroll down
            self.scroll_canvas.yview_scroll(3, "units")
        return "break"  # Prevent default scrolling

    def _on_global_mousewheel(self, event):
        widget = event.widget
        if isinstance(widget, tk.Text):
            return None
        try:
            if widget.winfo_toplevel() is not self:
                return None
        except tk.TclError:
            return None
        if getattr(event, "num", None) == 4:
            delta = -3
        elif getattr(event, "num", None) == 5:
            delta = 3
        else:
            delta = -3 * int(event.delta / 120) if getattr(event, "delta", 0) else 0
        if delta:
            self.scroll_canvas.yview_scroll(delta, "units")
        return None

    def _record_runtime_history(self):
        self.time_history.append(self.sim.sim_time_s)
        for train in self.sim.trains:
            if train.id not in self.position_history:
                self.position_history[train.id] = deque(maxlen=240)
            self.position_history[train.id].append(train.pos)

    def _update_event_log(self):
        sim_stamp = f"{self.sim.sim_time_s:7.1f}s"
        for train in self.sim.trains:
            for record in train.pop_pending_events():
                curves = record["curves"]
                extra = ""
                if record["event"] == "JOG_PROFILE_TRACE":
                    extra = (
                        f" src={record.get('reason', '--')}"
                        f" target={record.get('jog_target_speed_kmh', 0.0):.2f}km/h"
                        f" next={record.get('next_speed_kmh', 0.0):.2f}km/h"
                        f" accel={record.get('commanded_accel_ms2', 0.0):+.2f}"
                        f" phase={record.get('profile_phase', '--')}"
                    )
                self.event_log.appendleft(
                    f"{record['sim_time']:7.1f}s  {record['train_id']}  {record['event']}"
                    f" reason={record['reason'] or '--'} stop={record['stop_id']}"
                    f" pos={record['current_pos']:.2f} target={record['stop_target_pos']:.2f}"
                    f" rem={record['remaining']:.2f} err={record['stop_error']:.2f}"
                    f" v={record['speed_kmh']:.2f}/{record['vital_speed_kmh']:.2f}km/h"
                    f" eoa={record.get('eoa_m', 0.0):.2f} deoa={record.get('distance_to_eoa', 0.0):.2f}"
                    f" station={record.get('station_state', '--')} line={record.get('assigned_line', '--')}"
                    f" dwell={record.get('dwell_remaining', 0.0):.1f}s"
                    f" P/W/SBI/EBI/EBD={curves['P']:.1f}/{curves['W']:.1f}/{curves['SBI']:.1f}/{curves['EBI']:.1f}/{curves['EBD']:.1f}"
                    f" mode={record['curve_mode']} jog={record['jog_state']}{extra}"
                )
            snapshot = (
                train.atp_action,
                train.atp_alert,
                train.safe_packet_valid,
                train.door_authorized,
                train.commanded_stop,
            )
            previous = self.prev_train_snapshot.get(train.id)
            if previous is None:
                self.prev_train_snapshot[train.id] = snapshot
                continue
            if previous[0] != snapshot[0]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  ATP action {previous[0] or 'NONE'} -> {snapshot[0] or 'NONE'}")
            if previous[1] != snapshot[1]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  alert {previous[1]} -> {snapshot[1]}")
            if previous[2] != snapshot[2]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  DCS {'restored' if snapshot[2] else 'timeout / trip'}")
            if previous[3] != snapshot[3]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  door {'authorized' if snapshot[3] else 'locked'}")
            if previous[4] != snapshot[4]:
                self.event_log.appendleft(f"{sim_stamp}  {train.id}  scheduled stop {'armed' if snapshot[4] else 'released'}")
            self.prev_train_snapshot[train.id] = snapshot

    def _create_aux_windows(self):
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.child_windows = {}

        self.rebuild_train_panels()
        self._update_control_track_profile()

    def _update_control_track_profile(self):
        panel = getattr(self, "control_panel", None)
        if panel is not None:
            panel.update_track_profile(self.sim.track_profile)

    def rebuild_train_panels(self):
        # Clear existing panels
        for panel in self.panels.values():
            panel.destroy()
        self.panels = {}
        self.position_history = {train.id: deque(maxlen=240) for train in self.sim.trains}

        # Clear the scrollable frame
        for widget in self.trains_scrollable_frame.winfo_children():
            widget.destroy()

        for i, train in enumerate(self.sim.trains):
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def sync_train_panels(self):
        current_ids = {train.id for train in self.sim.trains}
        for train_id, panel in list(self.panels.items()):
            if train_id not in current_ids:
                wrapper = panel.master
                panel.destroy()
                try:
                    wrapper.destroy()
                except tk.TclError:
                    pass
                self.panels.pop(train_id, None)
                self.position_history.pop(train_id, None)

        for train in self.sim.trains:
            if train.id in self.panels:
                self.panels[train.id].set_track_range(self.sim.track_max_m)
                continue
            wrapper = ttk.Frame(self.trains_scrollable_frame, padding=8, style="Shell.TFrame")
            board_w, board_h = self._train_board_dimensions()
            wrapper.config(width=board_w, height=board_h)
            wrapper.pack(side="left", fill="y", padx=(0, 10))
            wrapper.pack_propagate(False)
            self._bind_train_horizontal_scroll(wrapper)
            panel = TrainPanel(
                wrapper,
                train.id,
                self.toggle_train,
                self.resume_train,
                self.instant_stop_train,
                self.precise_jog_train,
                train.color,
                self.scale_factor,
            )
            panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            panel.pack(fill="both", expand=True)
            panel.pack_propagate(False)
            self._bind_train_horizontal_scroll_tree(wrapper)
            panel.set_track_range(self.sim.track_max_m)
            self.panels[train.id] = panel
            if train.id not in self.position_history:
                self.position_history[train.id] = deque(maxlen=240)
        self._rebuild_train_fault_buttons()
        self._resize_train_boards()
        self._update_root_summary()

    def _train_board_dimensions(self) -> Tuple[int, int]:
        canvas_w = max(1, int(getattr(self, "trains_canvas", self).winfo_width() or 0))
        canvas_h = max(1, int(getattr(self, "trains_canvas", self).winfo_height() or 0))
        count = max(1, len(getattr(self.sim, "trains", [])))
        visible_count = min(count, 4)
        gap_px = int(10 * self.scale_factor)
        available_w = max(1, canvas_w - gap_px * max(0, visible_count - 1) - int(16 * self.scale_factor))
        board_w = max(int(360 * self.scale_factor), int(available_w / visible_count))
        board_h = max(int(220 * self.scale_factor), canvas_h - int(8 * self.scale_factor))
        return board_w, board_h

    def _resize_train_boards(self, _event=None):
        if not hasattr(self, "trains_scrollable_frame"):
            return
        board_w, board_h = self._train_board_dimensions()
        for panel in self.panels.values():
            wrapper = panel.master
            try:
                wrapper.config(width=board_w, height=board_h)
                panel.config(width=max(280, board_w - 20), height=max(200, board_h - 20))
            except tk.TclError:
                continue
        self.trains_scrollable_frame.update_idletasks()
        try:
            self.trains_canvas.itemconfigure(self.trains_window, height=max(1, self.trains_canvas.winfo_height()))
        except tk.TclError:
            pass
        self.trains_canvas.configure(scrollregion=self.trains_canvas.bbox("all"))

    def _bind_train_horizontal_scroll(self, widget: tk.Widget):
        if getattr(widget, "_cbtc_train_scroll_bound", False):
            return
        setattr(widget, "_cbtc_train_scroll_bound", True)
        widget.bind("<MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Shift-MouseWheel>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-4>", self._on_train_horizontal_mousewheel, add="+")
        widget.bind("<Button-5>", self._on_train_horizontal_mousewheel, add="+")

    def _bind_train_horizontal_scroll_tree(self, widget: tk.Widget):
        self._bind_train_horizontal_scroll(widget)
        for child in widget.winfo_children():
            self._bind_train_horizontal_scroll_tree(child)

    def _on_train_horizontal_mousewheel(self, event):
        if not hasattr(self, "trains_canvas"):
            return None
        if getattr(event, "num", None) == 4:
            delta = -1
        elif getattr(event, "num", None) == 5:
            delta = 1
        else:
            delta_raw = getattr(event, "delta", 0)
            if not delta_raw:
                return "break"
            delta = -1 if delta_raw > 0 else 1
        if delta:
            self.trains_canvas.xview_scroll(delta * 12, "units")
        return "break"

    def _on_trains_tab_click(self, event):
        try:
            tab_index = self.ats_tabs.index(f"@{event.x},{event.y}")
        except tk.TclError:
            return None
        if self.ats_tabs.tab(tab_index, "text") != "Trains":
            return None
        if tab_index == self.ats_tabs.index("current"):
            self._toggle_trains_tab()
            return "break"
        return None

    def _toggle_trains_tab(self):
        if not hasattr(self, "center_workspace"):
            return
        try:
            total_h = max(1, self.center_workspace.winfo_height())
            if self.trains_tab_collapsed:
                target = self._trains_restore_sash or int(total_h * 0.55)
                self.center_workspace.sashpos(0, max(180, min(total_h - 180, target)))
                self.trains_tab_collapsed = False
            else:
                self._trains_restore_sash = self.center_workspace.sashpos(0)
                self.center_workspace.sashpos(0, max(120, total_h - int(34 * self.scale_factor)))
                self.trains_tab_collapsed = True
        except tk.TclError:
            return
        self.after_idle(self._resize_train_boards)

    def _rebuild_train_fault_buttons(self):
        if not hasattr(self, "emergency_fault_frame"):
            return
        for frame in (self.emergency_fault_frame, self.atp_fault_frame, self.ato_fault_frame):
            for child in frame.winfo_children():
                child.destroy()
        self.train_fault_buttons = {}
        for row, train in enumerate(self.sim.trains):
            emergency_btn = ttk.Button(
                self.emergency_fault_frame,
                text=f"{train.id}: Stop",
                command=lambda train_id=train.id: self.instant_stop_train(train_id),
                style="Danger.TButton",
            )
            emergency_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(emergency_btn)
            atp_btn = ttk.Button(
                self.atp_fault_frame,
                text=f"{train.id}: ATP",
                command=lambda train_id=train.id: self.toggle_train_fault(train_id, "ATP"),
            )
            atp_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(atp_btn)
            ato_btn = ttk.Button(
                self.ato_fault_frame,
                text=f"{train.id}: ATO",
                command=lambda train_id=train.id: self.toggle_train_fault(train_id, "ATO"),
            )
            ato_btn.grid(row=row, column=0, sticky="ew", padx=2, pady=2)
            self._bind_side_toolbar_scroll(ato_btn)
            self.train_fault_buttons[train.id] = {"emergency": emergency_btn, "atp": atp_btn, "ato": ato_btn}
        self._bind_side_toolbar_tree(self.emergency_fault_frame)
        self._bind_side_toolbar_tree(self.atp_fault_frame)
        self._bind_side_toolbar_tree(self.ato_fault_frame)

    def _hide_all_child_windows(self):
        for window in self.child_windows.values():
            try:
                window.withdraw()
            except tk.TclError:
                pass

    def _any_child_visible(self) -> bool:
        return any(str(window.state()) != "withdrawn" for window in self.child_windows.values())

    def _hide_window(self, window: tk.Toplevel):
        try:
            window.withdraw()
        except tk.TclError:
            return
        if not self._any_child_visible():
            try:
                self.deiconify()
            except tk.TclError:
                pass

    def _show_window(self, key: str):
        window = self.child_windows.get(key)
        if window is None:
            return
        self._hide_all_child_windows()
        try:
            self.withdraw()
        except tk.TclError:
            pass
        window.deiconify()
        window.lift()
        window.focus_force()
        try:
            window.state("zoomed")
        except tk.TclError:
            screen_w = window.winfo_screenwidth()
            screen_h = window.winfo_screenheight()
            window.geometry(f"{screen_w}x{screen_h}+0+0")

    def _update_root_summary(self):
        dcs_ok = sum(1 for train in self.sim.trains if train.safe_packet_valid)
        emergency = sum(1 for train in self.sim.trains if train.atp_action == "EBI")
        moving = sum(1 for train in self.sim.trains if train.speed > 0.1)
        total_trains = len(self.sim.trains)
        self.summary_var.set(
            f"Use the ATS main tabs for full line monitoring and systems functions."
            f"  Trains={total_trains}  moving={moving}  DCS healthy={dcs_ok}/{total_trains}  EBI active={emergency}"
        )

    def _on_close(self):
        for window in list(self.child_windows.values()):
            try:
                window.destroy()
            except tk.TclError:
                pass
        self.destroy()

    def on_load_scenario(self):
        selected = filedialog.askopenfilename(
            title="Load Line Configuration YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
            initialdir=str(DEFAULT_SCENARIO_PATH.parent),
        )
        if not selected:
            return
        was_running = self.sim.running
        self.sim.stop()
        try:
            self.scenario = load_scenario(selected)
        except Exception as exc:
            self.status_var.set(f"Status: failed to load line config ({exc})")
            if was_running:
                self.sim.start()
            return
        self.sim.load_scenario(self.scenario)
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Line config: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.edit_undo_stack.clear()
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()
        self.rebuild_train_panels()
        self.status_var.set(f"Status: loaded line config from {selected}")
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()

    def on_start(self):
        self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: running")

    def on_stop(self):
        self.sim.stop()
        self.sim_paused = True
        self._update_run_pause_buttons()
        self.status_var.set("Status: paused")

    def on_reset_simulation(self):
        was_running = self.sim.running
        self.sim.stop()
        self.sim.load_scenario(self.scenario)
        self.sim.set_timetable_clock_scale(self.time_scale)
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.edit_undo_stack.clear()
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()
        self.rebuild_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self._update_operation_mode_status()
        if was_running:
            self.sim.start()
        self.sim_paused = False
        self._update_run_pause_buttons()
        self.status_var.set("Status: simulation reset")

    def set_time_scale(self, scale: int):
        self.time_scale = max(1, int(scale))
        self.sim.set_timetable_clock_scale(self.time_scale)
        self._update_time_scale_buttons()
        self.status_var.set(f"Status: time scale x{self.time_scale}")

    def _update_time_scale_buttons(self):
        for scale, button in self.time_scale_buttons.items():
            button.configure(style="Active.TButton" if scale == self.time_scale else "TButton")

    def _update_run_pause_buttons(self):
        if hasattr(self, "start_btn"):
            self.start_btn.configure(style="RunActive.TButton" if self.sim.running else "StartInactive.TButton")
        if hasattr(self, "stop_btn"):
            pause_active = self.sim_paused and not self.sim.running
            self.stop_btn.configure(style="PauseHistoryActive.TButton" if pause_active else "History.TButton")

    def _editable_snapshot(self) -> Dict[str, Any]:
        data = scenario_to_yaml_data(self.sim, self.scenario)
        snapshot = normalize_scenario(data, self.scenario.get("source_path"))
        snapshot["source_path"] = self.scenario.get("source_path")
        snapshot["tsr_zones"] = deepcopy(self.sim.tsr_zones)
        return snapshot

    def _push_edit_undo(self):
        self.edit_undo_stack.append(self._editable_snapshot())
        self.edit_redo_stack.clear()
        self._update_edit_history_buttons()

    def _restore_edit_snapshot(self, snapshot: Dict[str, Any]):
        was_running = self.sim.running
        self.sim.stop()
        scenario_snapshot = deepcopy(snapshot)
        tsr_zones = deepcopy(scenario_snapshot.pop("tsr_zones", []))
        self.scenario = scenario_snapshot
        self.sim.load_scenario(self.scenario)
        self.sim.tsr_zones = tsr_zones
        self.title(self.scenario["window_title"])
        self.scenario_var.set(f"Line config: {self.scenario['name']}")
        self.reload_headway_block_values()
        self._update_control_track_profile()
        self._reset_runtime_buffers()
        self.rebuild_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        if was_running:
            self.sim.start()

    def _update_edit_history_buttons(self):
        if hasattr(self, "undo_edit_btn"):
            self.undo_edit_btn.configure(state=("normal" if self.edit_undo_stack else "disabled"))
        if hasattr(self, "redo_edit_btn"):
            self.redo_edit_btn.configure(state=("normal" if self.edit_redo_stack else "disabled"))

    def undo_canvas_edit(self):
        if not self.edit_undo_stack:
            return
        self.edit_redo_stack.append(self._editable_snapshot())
        snapshot = self.edit_undo_stack.pop()
        self._restore_edit_snapshot(snapshot)
        self._update_edit_history_buttons()
        self.status_var.set("Status: canvas edit undone")

    def redo_canvas_edit(self):
        if not self.edit_redo_stack:
            return
        self.edit_undo_stack.append(self._editable_snapshot())
        snapshot = self.edit_redo_stack.pop()
        self._restore_edit_snapshot(snapshot)
        self._update_edit_history_buttons()
        self.status_var.set("Status: canvas edit redone")

    def on_export_report(self):
        path = save_simulation_report(self.sim)
        self.status_var.set(f"Status: exported report to {path}")

    def on_save_scenario(self):
        target_path = self.scenario.get("source_path")
        if not target_path:
            target_path = filedialog.asksaveasfilename(
                title="Save Line Configuration YAML",
                filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
                defaultextension=".yaml",
                initialdir=str(DEFAULT_SCENARIO_PATH.parent),
            )
            if not target_path:
                return
        try:
            path = save_scenario_file(self.sim, self.scenario, target_path)
        except Exception as exc:
            self.status_var.set(f"Status: failed to save line config ({exc})")
            return
        self.scenario["source_path"] = str(path)
        self.status_var.set(f"Status: saved line config to {path}")

    def open_add_element_dialog(self):
        AddElementDialog(self, self.add_ats_element)

    def _track_has_psr_coverage(self, start_m: float, end_m: float) -> bool:
        return self._first_psr_gap(start_m, end_m) is None

    def _first_psr_gap(self, start_m: float, end_m: float) -> Tuple[float, float] | None:
        if end_m <= start_m:
            return None
        cursor = start_m
        for seg_start, seg_end, _gradient, _psr in sorted(self.sim.track_profile, key=lambda item: item[0]):
            if seg_end <= cursor:
                continue
            if seg_start > cursor + 1e-6:
                return cursor, min(seg_start, end_m)
            cursor = max(cursor, min(seg_end, end_m))
            if cursor >= end_m - 1e-6:
                return None
        return cursor, end_m

    def _clear_pending_line_extension(self):
        self.pending_line_extension = None
        self.pending_station_prompt_after_limit = False

    def _open_extension_speed_limit_dialog(self, start_m: float, end_m: float):
        AddElementDialog(
            self,
            self.add_ats_element,
            initial_element="Segment PSR/TSR",
            initial_values={
                "mode": "PSR",
                "start_m": f"{start_m:.0f}",
                "end_m": f"{end_m:.0f}",
                "speed_kmh": "80",
            },
            locked_element=True,
            title="Add PSR for Uncovered Line",
            on_cancel=self._clear_pending_line_extension,
        )

    def _open_extension_station_dialog(self, _start_m: float, end_m: float):
        AddElementDialog(
            self,
            self.add_ats_element,
            initial_element="Station",
            initial_values={
                "name": f"STATION_{int(round(end_m))}",
                "pos_m": f"{end_m:.0f}",
                "length_m": "160",
                "capacity": "3",
                "dwell_s": "30",
            },
            locked_element=True,
            title="Add Station for Extended Line",
        )

    def on_ats_element_selected(self, element_key: str):
        self.status_var.set(f"Status: selected {element_key}")

    def _selected_source_index(self) -> int | None:
        element_key = self.ats_overview_panel.selected_element
        if element_key:
            kind, index = self.ats_overview_panel._element_lookup.get(element_key, ("", -1))
            if kind == "source_train" and 0 <= index < len(self.sim.source_trains):
                return index
        if self.sim.source_trains:
            return 0
        return None

    def _set_source_train_count(self, index: int, count: int):
        if index < 0 or index >= len(self.sim.source_trains):
            return
        self._push_edit_undo()
        source = self.sim.source_trains[index]
        count = max(0, int(count))
        source["capacity"] = max(1, count)
        source["total_trains"] = count
        self.sim._sync_source_train_count(index)
        self.sync_train_panels()
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
        self.status_var.set(f"Status: depot {source.get('name', index)} trains={count}")

    def add_train_to_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: add a depot first")
            return
        source = self.sim.source_trains[index]
        current = int(source.get("total_trains", source.get("capacity", 0)))
        self._set_source_train_count(index, current + 1)

    def remove_train_from_selected_source(self):
        index = self._selected_source_index()
        if index is None:
            self.status_var.set("Status: no depot to remove from")
            return
        source = self.sim.source_trains[index]
        current = int(source.get("total_trains", source.get("capacity", 0)))
        self._set_source_train_count(index, current - 1)

    def open_edit_element_dialog(self, kind: str, index: int):
        element_type, values = self._edit_dialog_data(kind, index)
        if element_type is None:
            return
        AddElementDialog(
            self,
            lambda selected_type, data: self.edit_ats_element(kind, index, selected_type, data),
            initial_element=element_type,
            initial_values=values,
            locked_element=True,
            submit_text="Save",
            title="Edit ATS Element",
        )

    def _edit_dialog_data(self, kind: str, index: int) -> Tuple[str | None, Dict[str, str]]:
        if kind == "station" and 0 <= index < len(self.sim.scheduled_stops):
            stop = self.sim.scheduled_stops[index]
            return "Station", {
                "name": str(stop.get("name", "")),
                "pos_m": str(stop.get("pos_m", 0.0)),
                "length_m": str(stop.get("length_m", 160.0)),
                "capacity": str(stop.get("capacity", 3)),
                "dwell_s": str(stop.get("dwell_s", 30.0)),
            }
        if kind == "track_segment" and 0 <= index < len(self.sim.track_profile):
            start, end, gradient, psr = self.sim.track_profile[index]
            return "PSR Segment", {
                "start_m": str(start),
                "end_m": str(end),
                "gradient": str(gradient),
                "psr_kmh": str(psr),
            }
        if kind == "tsr" and 0 <= index < len(self.sim.tsr_zones):
            zone = self.sim.tsr_zones[index]
            return "TSR", {
                "start_m": str(zone.get("start", 0.0)),
                "end_m": str(zone.get("end", 0.0)),
                "speed_kmh": str(zone.get("speed", 0.0)),
            }
        if kind == "line_condition" and 0 <= index < len(self.sim.line_conditions):
            condition = self.sim.line_conditions[index]
            return "Line Condition", {
                "start_m": str(condition.get("start", 0.0)),
                "end_m": str(condition.get("end", 0.0)),
                "condition": str(condition.get("condition", "dry")),
            }
        if kind == "source_train" and 0 <= index < len(self.sim.source_trains):
            source = self.sim.source_trains[index]
            return "Depot", {
                "name": str(source.get("name", "DEPOT")),
                "capacity": str(source.get("capacity", 2)),
            }
        return None, {}

    def delete_selected_element(self):
        element_key = self.ats_overview_panel.selected_element
        if not element_key:
            self.status_var.set("Status: select an ATS element to delete")
            return
        kind, index = self.ats_overview_panel._element_lookup.get(element_key, ("", -1))
        try:
            if kind == "station" and 0 <= index < len(self.sim.scheduled_stops):
                self._push_edit_undo()
                removed = self.sim.scheduled_stops.pop(index)
                for train in self.sim.trains:
                    train.scheduled_stops = self.sim.scheduled_stops
                    if train.active_scheduled_stop is not None and self.sim._station_index_for_stop(train.active_scheduled_stop) is None:
                        train.active_scheduled_stop = None
                        train.commanded_stop = False
                        train.station_lane = None
                self.sim._sync_station_route_states()
                self.status_var.set(f"Status: deleted station {removed.get('name', index)}")
            elif kind == "track_segment" and 0 <= index < len(self.sim.track_profile):
                if len(self.sim.track_profile) <= 1:
                    self.status_var.set("Status: cannot delete the last track segment")
                    return
                self._push_edit_undo()
                self.sim.track_profile.pop(index)
                self.sim.track_end_m = max(end for _start, end, _gradient, _psr in self.sim.track_profile)
                self.sim.track_max_m = max(self.sim.track_max_m, self.sim.track_end_m)
                for train in self.sim.trains:
                    train.track_profile = self.sim.track_profile
                self._update_control_track_profile()
                self.status_var.set(f"Status: deleted track segment {index}")
            elif kind == "tsr" and 0 <= index < len(self.sim.tsr_zones):
                self._push_edit_undo()
                self.sim.tsr_zones.pop(index)
                self.status_var.set(f"Status: deleted TSR {index}")
            elif kind == "line_condition" and 0 <= index < len(self.sim.line_conditions):
                self._push_edit_undo()
                self.sim.line_conditions.pop(index)
                self.status_var.set(f"Status: deleted line condition {index}")
            elif kind == "source_train" and 0 <= index < len(self.sim.source_trains):
                self._push_edit_undo()
                removed = self.sim.source_trains.pop(index)
                source_name = str(removed.get("name", "DEPOT"))
                self.sim.trains = [
                    train for train in self.sim.trains
                    if not self.sim._source_train_matches(train, source_name)
                ]
                self.sim._rebuild_after_train_set_change()
                self.sync_train_panels()
                self.status_var.set(f"Status: deleted depot {removed.get('name', index)}")
            else:
                self.status_var.set(f"Status: cannot delete {element_key}")
                return
        finally:
            self.ats_overview_panel.selected_element = None
            self.ats_overview_panel.update_data(self.sim)
            self.infrastructure_panel.update_data(self.sim)
            self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def edit_ats_element(self, kind: str, index: int, _element_type: str, data: Dict[str, str]):
        self._push_edit_undo()
        if kind == "station":
            self.sim.update_station(
                index,
                data["name"] or f"STATION_{index + 1}",
                float(data["pos_m"]),
                float(data["length_m"]),
                int(float(data["capacity"])),
                float(data["dwell_s"]),
            )
        elif kind == "track_segment":
            self.sim.update_track_segment(
                index,
                float(data["start_m"]),
                float(data["end_m"]),
                float(data["gradient"]),
                float(data["psr_kmh"]),
            )
            self._update_control_track_profile()
        elif kind == "tsr":
            self.sim.update_tsr(index, float(data["start_m"]), float(data["end_m"]), float(data["speed_kmh"]))
        elif kind == "line_condition":
            self.sim.update_line_condition(index, float(data["start_m"]), float(data["end_m"]), data["condition"] or "dry")
        elif kind == "source_train":
            capacity = int(float(data["capacity"]))
            self.sim.update_source_train(
                index,
                data["name"] or f"DEPOT_{index + 1}",
                capacity,
                capacity,
            )
            self.sync_train_panels()
        else:
            raise ValueError("Unsupported ATS element.")
        self.status_var.set(f"Status: edited {kind}:{index}")
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def add_ats_element(self, element_type: str, data: Dict[str, str]):
        if element_type == "Station":
            self._push_edit_undo()
            name = data["name"] or f"STATION_{len(self.sim.scheduled_stops) + 1}"
            self.sim.add_station(
                name,
                float(data["pos_m"]),
                float(data["length_m"]),
                int(float(data["capacity"])),
                float(data["dwell_s"]),
            )
            self.status_var.set(f"Status: added station {name}")
        elif element_type == "Segment PSR/TSR":
            mode = data["mode"].upper()
            start_m = float(data["start_m"])
            end_m = float(data["end_m"])
            if end_m < start_m:
                start_m, end_m = end_m, start_m
            if mode not in ("PSR", "TSR"):
                raise ValueError("Mode must be PSR or TSR.")
            self._push_edit_undo()
            if mode == "PSR":
                self.sim.add_psr_segment(start_m, end_m, float(data["speed_kmh"]))
                self._update_control_track_profile()
            elif mode == "TSR":
                self.sim.tsr_zones.append({"start": start_m, "end": end_m, "speed": float(data["speed_kmh"])})
            self.status_var.set(f"Status: added {mode} segment")
            if self.pending_station_prompt_after_limit and self.pending_line_extension is not None:
                extension_start, extension_end = self.pending_line_extension
                missing_psr = self._first_psr_gap(0.0, self.sim.track_end_m)
                if missing_psr is not None:
                    missing_start, missing_end = missing_psr
                    self.status_var.set(
                        f"Status: added {mode} segment; add PSR for uncovered {missing_start:.0f}-{missing_end:.0f} m"
                    )
                    self.after(50, lambda: self._open_extension_speed_limit_dialog(missing_start, missing_end))
                else:
                    self.pending_line_extension = None
                    self.pending_station_prompt_after_limit = False
                    self.after(50, lambda: self._open_extension_station_dialog(extension_start, extension_end))
        elif element_type == "Line":
            old_end_m = self.sim.track_end_m
            length_m = max(1.0, float(data["length_m"]))
            self._push_edit_undo()
            self.sim.track_end_m = max(self.sim.track_end_m, length_m)
            self.sim.track_max_m = max(self.sim.track_max_m, length_m)
            missing_psr = self._first_psr_gap(0.0, self.sim.track_end_m)
            if length_m > old_end_m and missing_psr is not None:
                missing_start, missing_end = missing_psr
                self.pending_line_extension = (old_end_m, length_m)
                self.pending_station_prompt_after_limit = True
                self.status_var.set(
                    f"Status: extended line to {length_m:.0f} m; add PSR for uncovered {missing_start:.0f}-{missing_end:.0f} m"
                )
                self.after(50, lambda: self._open_extension_speed_limit_dialog(missing_start, missing_end))
            else:
                self.pending_line_extension = None
                self.pending_station_prompt_after_limit = False
                self.status_var.set(f"Status: extended line to {length_m:.0f} m")
        elif element_type == "Gradient Segment":
            self._push_edit_undo()
            self.sim.add_gradient_segment(
                float(data["start_m"]),
                float(data["end_m"]),
                float(data["gradient"]),
            )
            self._update_control_track_profile()
            self.status_var.set("Status: added gradient segment")
        elif element_type == "Line Condition":
            start_m = float(data["start_m"])
            end_m = float(data["end_m"])
            if end_m < start_m:
                start_m, end_m = end_m, start_m
            if end_m <= start_m:
                raise ValueError("Line condition end must be greater than start.")
            self._push_edit_undo()
            self.sim.line_conditions.append({"start": start_m, "end": end_m, "condition": data["condition"] or "dry"})
            self.status_var.set("Status: added line condition")
        elif element_type == "Depot":
            self._push_edit_undo()
            name = data["name"] or f"DEPOT_{len(self.sim.source_trains) + 1}"
            capacity = int(float(data["capacity"]))
            self.sim.add_source_train(
                name,
                SOURCE_TRAIN_START_M,
                SOURCE_TRAIN_LENGTH_M,
                capacity,
                capacity,
            )
            self.sync_train_panels()
            self.status_var.set(f"Status: added depot {name}")
        else:
            raise ValueError("Unsupported element type.")
        self.ats_overview_panel.update_data(self.sim)
        self.infrastructure_panel.update_data(self.sim)
        self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)

    def toggle_train(self, train_id: str, emergency: bool = False):
        for t in self.sim.trains:
            if t.id == train_id:
                if emergency:
                    t.emg_ack = True
                    t.acknowledge_emergency_safe()
                else:
                    t.enter_trip_mode("TRAIN TRIP", t.reported_pos)
                    t.emergency_recovery_hold = False
                break

    def resume_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                t.resume_after_emergency()
                break

    def instant_stop_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                t.enter_trip_mode("INSTANT STOP", t.reported_pos)
                t.emergency_recovery_hold = False
                t.ato_target_speed = 0.0
                t.service_brake_latch = False
                t.atp_state = "ATP_TRIP"
                t.atp_alert = "INSTANT STOP"
                t.atp_brake = "EMERGENCY"
                t.atp_action = "EBI"
                break

    def precise_jog_train(self, train_id: str):
        for t in self.sim.trains:
            if t.id == train_id:
                if t.request_precise_jog():
                    self.status_var.set(f"Status: {train_id} precise jog requested")
                else:
                    remaining_m = t.distance_to_stop_target()
                    self.status_var.set(
                        f"Status: {train_id} cannot jog "
                        f"(remaining={remaining_m:.2f} m, zero_speed={'YES' if t.zero_speed_detected else 'NO'})"
                    )
                break

    def toggle_train_fault(self, train_id: str, subsystem: str):
        subsystem = subsystem.upper()
        for train in self.sim.trains:
            if train.id != train_id:
                continue
            if subsystem == "ATP":
                train.set_fault("ATP", not train.atp_fault_active, self.sim.sim_time_s)
            elif subsystem == "ATO":
                train.set_fault("ATO", not train.ato_fault_active, self.sim.sim_time_s)
            elif subsystem == "DCS":
                train.set_fault("DCS", not train.dcs_fault_active, self.sim.sim_time_s)
            self.status_var.set(f"Status: toggled {subsystem} fault on {train_id}")
            break

    def toggle_all_dcs_loss(self):
        active = not any(train.dcs_fault_active for train in self.sim.trains)
        for train in self.sim.trains:
            train.set_fault("DCS", active, self.sim.sim_time_s)
        self.status_var.set(
            "Status: DCS loss applied"
            if active
            else "Status: DCS loss cleared; emergency recovery still required for tripped trains"
        )

    def clear_all_faults(self):
        for train in self.sim.trains:
            train.set_fault("DCS", False, self.sim.sim_time_s)
            train.set_fault("ATO", False, self.sim.sim_time_s)
            train.set_fault("ATP", False, self.sim.sim_time_s)
            if not (train.trip_mode or train.emg_latch or train.emergency_stop or train.emergency_recovery_hold):
                train.reset_non_emergency_stop_latches()
        self.status_var.set("Status: cleared fault flags; use Safe Confirmed/Resume for tripped trains")

    def apply_psr(self, segment_str: str, psr_str: str):
        try:
            idx = int(segment_str)
            psr = float(psr_str)
            if idx < 0 or idx >= len(self.sim.track_profile):
                return
            self._push_edit_undo()
            start, end, gradient, _ = self.sim.track_profile[idx]
            self.sim.track_profile[idx] = (start, end, gradient, psr)
        except ValueError:
            return

    def add_tsr(self, start_str: str, end_str: str, speed_str: str):
        try:
            start = float(start_str)
            end = float(end_str)
            speed = float(speed_str)
            if end < start:
                start, end = end, start
            self._push_edit_undo()
            self.sim.tsr_zones.append({"start": start, "end": end, "speed": speed})
        except ValueError:
            return

    def clear_tsr(self):
        if not self.sim.tsr_zones:
            return
        self._push_edit_undo()
        self.sim.tsr_zones.clear()

    def tick(self):
        if self.sim.running:
            steps_this_tick = max(1, int(self.time_scale))
            for _ in range(steps_this_tick):
                self.sim.step()
            if self.sim.train_generation_changed:
                self.sync_train_panels()
        now = time.monotonic()
        refresh_interval = self._running_ui_refresh_interval_s if self.sim.running else self._idle_ui_refresh_interval_s
        if now - self._last_ui_refresh_real_s >= refresh_interval:
            self._last_ui_refresh_real_s = now
            if self.sim.running:
                self._record_runtime_history()
            self._update_event_log()
            self.clock_var.set(f"Sim time: {self.sim.sim_time_s:.1f} s")
            self._update_root_summary()
            for t in self.sim.trains:
                if t.id not in self.panels:
                    self.sync_train_panels()
                self.panels[t.id].update_from_train(t, append_history=self.sim.running)
            self.limits_panel.update_limits(self.sim.track_profile, self.sim.tsr_zones)
            if self.current_workspace_mode == "normal":
                self.ats_overview_panel.update_data(self.sim)
                self.infrastructure_panel.update_data(self.sim)
                self.engineering_panel.update_data(self.sim)
                self.diagnostics_panel.update_data(self.sim, self.event_log)
                self.dataflow_panel.update_data(self.sim)
                self.analytics_panel.update_data(self.sim)
        self.after(int(DT * 1000), self.tick)


# GUI startup is kept in run.py. This module remains import-compatible for tests
# and for code that still imports the legacy combined module during refactoring.
