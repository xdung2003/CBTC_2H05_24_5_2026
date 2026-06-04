from __future__ import annotations

from GUI.gui_context import *

class AddElementDialog(tk.Toplevel):
    ADDABLE_ELEMENTS = [
        "Station",
        "Segment PSR/TSR",
        "Gradient Segment",
        "Line",
        "Line Condition",
        "Depot",
    ]

    FIELD_SETS = {
        "Station": [
            ("name", "Name", "STATION_NEW"),
            ("pos_m", "Stop position (m)", "1000"),
            ("length_m", "Station length (m)", "160"),
            ("capacity", "Train capacity", "3"),
            ("dwell_s", "Dwell (s)", "30"),
        ],
        "Segment PSR/TSR": [
            ("mode", "Mode (PSR/TSR)", "TSR"),
            ("start_m", "Start (m)", "600"),
            ("end_m", "End (m)", "900"),
            ("speed_kmh", "Limit speed (km/h)", "25"),
        ],
        "PSR Segment": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("gradient", "Gradient", "0.00"),
            ("psr_kmh", "PSR km/h", "80"),
        ],
        "TSR": [
            ("start_m", "Start (m)", "600"),
            ("end_m", "End (m)", "900"),
            ("speed_kmh", "Limit speed (km/h)", "25"),
        ],
        "Gradient Segment": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("gradient", "Gradient", "0.00"),
        ],
        "Line": [
            ("length_m", "Line length (m)", "2000"),
        ],
        "Line Condition": [
            ("start_m", "Start (m)", "0"),
            ("end_m", "End (m)", "500"),
            ("condition", "Condition (dry/wet)", "dry"),
        ],
        "Depot": [
            ("name", "Name", "DEPOT"),
            ("capacity", "Depot trains", "3"),
        ],
    }

    def __init__(
        self,
        master: tk.Widget,
        on_add,
        initial_element: str = "Station",
        initial_values: Dict[str, str] | None = None,
        locked_element: bool = False,
        submit_text: str = "Add",
        title: str = "Add ATS Element",
        on_cancel=None,
        element_choices: List[str] | None = None,
    ):
        super().__init__(master)
        self.title(title)
        self.resizable(False, False)
        self.configure(background=APP_THEME["bg"])
        self.on_add = on_add
        self.on_cancel = on_cancel
        self.initial_values = initial_values or {}
        self.values: Dict[str, tk.StringVar] = {}
        self.element_choices = list(element_choices or self.ADDABLE_ELEMENTS)
        if initial_element not in self.element_choices:
            self.element_choices = [initial_element, *self.element_choices]
        self.element_var = tk.StringVar(value=initial_element)

        body = ttk.Frame(self, padding=12, style="Shell.TFrame")
        body.grid(row=0, column=0, sticky="nsew")
        ttk.Label(body, text="Element").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        chooser_state = "disabled" if locked_element else "readonly"
        chooser = ttk.Combobox(body, textvariable=self.element_var, values=self.element_choices, state=chooser_state, width=20)
        chooser.grid(row=0, column=1, sticky="ew", pady=(0, 8))
        chooser.bind("<<ComboboxSelected>>", lambda _event: self._render_fields())

        self.fields_frame = ttk.Frame(body, style="Shell.TFrame")
        self.fields_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        buttons = ttk.Frame(body, style="Shell.TFrame")
        buttons.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        buttons.columnconfigure(0, weight=1)
        ttk.Button(buttons, text="Cancel", command=self._cancel).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(buttons, text=submit_text, command=self._submit).grid(row=0, column=2)

        self.status_var = tk.StringVar(value="")
        ttk.Label(body, textvariable=self.status_var, style="Muted.TLabel").grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self._render_fields()
        self.transient(master)
        self.grab_set()
        self.focus_force()
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self._center_on_master()

    def _render_fields(self):
        for widget in self.fields_frame.winfo_children():
            widget.destroy()
        self.values = {}
        for row, (key, label, default) in enumerate(self.FIELD_SETS[self.element_var.get()]):
            ttk.Label(self.fields_frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
            var = tk.StringVar(value=self.initial_values.get(key, default))
            self.values[key] = var
            ttk.Entry(self.fields_frame, textvariable=var, width=22).grid(row=row, column=1, sticky="ew", pady=3)

    def _submit(self):
        data = {key: var.get().strip() for key, var in self.values.items()}
        try:
            self.on_add(self.element_var.get(), data)
        except ValueError as exc:
            self.status_var.set(str(exc))
            return
        self.destroy()

    def _cancel(self):
        if self.on_cancel is not None:
            self.on_cancel()
        self.destroy()

    def _center_on_master(self):
        self.update_idletasks()
        master = self.master
        if master is not None:
            master.update_idletasks()
            master_x = master.winfo_rootx()
            master_y = master.winfo_rooty()
            master_w = max(1, master.winfo_width())
            master_h = max(1, master.winfo_height())
            x = master_x + (master_w - self.winfo_width()) // 2
            y = master_y + (master_h - self.winfo_height()) // 2
        else:
            x = (self.winfo_screenwidth() - self.winfo_width()) // 2
            y = (self.winfo_screenheight() - self.winfo_height()) // 2
        self.geometry(f"+{max(0, x)}+{max(0, y)}")


