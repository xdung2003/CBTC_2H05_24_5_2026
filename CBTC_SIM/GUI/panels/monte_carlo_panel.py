from __future__ import annotations

from GUI.main_gui import *

class MonteCarloPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, app: "App"):
        super().__init__(master, padding=14, style="Shell.TFrame")
        self.app = app
        self._worker: threading.Thread | None = None
        self._queue: queue.Queue = queue.Queue()
        self._stop_requested = False

        self.columnconfigure(0, weight=1)
        self.rowconfigure(5, weight=1)
        ttk.Label(self, text="Chế độ thống kê Monte Carlo", style="SectionTitle.TLabel").grid(row=0, column=0, sticky="w")

        controls = ttk.Frame(self, style="Shell.TFrame")
        controls.grid(row=1, column=0, sticky="ew", pady=(10, 6))
        self.runs_var = tk.StringVar(value="100")
        self.max_time_var = tk.StringVar(value="1000")
        self.seed_var = tk.StringVar(value="")
        ttk.Label(controls, text="Runs", style="Status.TLabel").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.runs_var, width=7).grid(row=0, column=1, padx=(0, 10))
        ttk.Label(controls, text="Max sim s", style="Status.TLabel").grid(row=0, column=2, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.max_time_var, width=8).grid(row=0, column=3, padx=(0, 10))
        ttk.Label(controls, text="Replay ID", style="Status.TLabel").grid(row=0, column=4, padx=(0, 4))
        ttk.Entry(controls, textvariable=self.seed_var, width=10).grid(row=0, column=5, padx=(0, 10))
        self.run_btn = ttk.Button(controls, text="Chạy thống kê", command=self.start)
        self.run_btn.grid(row=0, column=6, padx=(10, 4))
        self.stop_btn = ttk.Button(controls, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.grid(row=0, column=7)

        self.progress_var = tk.DoubleVar(value=0.0)
        ttk.Progressbar(self, variable=self.progress_var, maximum=100.0).grid(row=2, column=0, sticky="ew", pady=(2, 6))
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self, textvariable=self.status_var, style="Status.TLabel").grid(row=3, column=0, sticky="w")
        ttk.Label(
            self,
            text="Replay ID is optional: leave it blank for a fresh random batch, or reuse the same number to replay the same batch.",
            style="Muted.TLabel",
        ).grid(row=4, column=0, sticky="w", pady=(0, 4))

        self.output = tk.Text(
            self,
            height=36,
            wrap="none",
            background="#fbfdff",
            foreground=APP_THEME["text"],
            font=("Consolas", 9),
            relief="solid",
            borderwidth=1,
        )
        self.output.grid(row=5, column=0, sticky="nsew", pady=(8, 0))
        self._set_output("Kết quả thống kê sẽ hiển thị tại đây.\n")

    def _config_from_fields(self) -> MonteCarloConfig:
        seed_text = self.seed_var.get().strip()
        seed = int(seed_text) if seed_text else None
        return MonteCarloConfig(
            runs=max(1, int(float(self.runs_var.get()))),
            max_sim_time_s=max(1.0, float(self.max_time_var.get())),
            seed=seed,
        )

    def start(self):
        if self._worker is not None and self._worker.is_alive():
            return
        try:
            config = self._config_from_fields()
        except ValueError as exc:
            self.status_var.set(f"Invalid Monte Carlo input: {exc}")
            return
        self._stop_requested = False
        self.progress_var.set(0.0)
        self.status_var.set("Starting Monte Carlo batch")
        self._set_output("Running...\n")
        self.run_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        scenario = deepcopy(self.app.scenario)

        def progress(summary):
            if self._stop_requested:
                raise RuntimeError("Monte Carlo stopped by user")
            self._queue.put(("summary", summary))

        def status(update):
            if self._stop_requested:
                raise RuntimeError("Monte Carlo stopped by user")
            self._queue.put(("status", dict(update)))

        def worker():
            try:
                summary = run_batch(
                    scenario,
                    config,
                    Simulation,
                    sys.modules[__name__],
                    progress=progress,
                    status=status,
                )
                self._queue.put(("done", summary))
            except Exception as exc:
                self._queue.put(("error", str(exc)))

        self._worker = threading.Thread(target=worker, daemon=True)
        self._worker.start()
        self.after(100, self._poll_queue)

    def stop(self):
        self._stop_requested = True
        self.status_var.set("Stopping after current Monte Carlo callback")

    def _set_output(self, text: str):
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        self.output.insert("1.0", text)
        self.output.configure(state="disabled")

    def _poll_queue(self):
        keep_polling = self._worker is not None and self._worker.is_alive()
        while True:
            try:
                kind, payload = self._queue.get_nowait()
            except queue.Empty:
                break
            if kind == "summary":
                completed = len(payload.results)
                total = max(1, payload.config.runs)
                self.progress_var.set(completed / total * 100.0)
                self.status_var.set(f"{completed}/{total} samples ({completed / total * 100.0:.0f}%)")
                self._set_output("\n".join(payload.table_lines()))
            elif kind == "status":
                run_id = payload.get("run_id", 0)
                runs = payload.get("runs", 0)
                phase = payload.get("phase", "Running")
                sim_time = payload.get("sim_time_s")
                suffix = f"  t={sim_time:.1f}s" if isinstance(sim_time, (int, float)) else ""
                self.status_var.set(f"Run {run_id}/{runs}: {phase}{suffix}")
            elif kind == "done":
                self.progress_var.set(100.0)
                self.status_var.set("Monte Carlo complete")
                self._set_output("\n".join(payload.table_lines()))
                self.run_btn.configure(state="normal")
                self.stop_btn.configure(state="disabled")
                keep_polling = False
            elif kind == "error":
                self.status_var.set(payload)
                self.run_btn.configure(state="normal")
                self.stop_btn.configure(state="disabled")
                keep_polling = False
        if keep_polling:
            self.after(100, self._poll_queue)


