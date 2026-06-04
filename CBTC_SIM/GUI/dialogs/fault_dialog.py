from __future__ import annotations

from GUI.gui_context import *


class FaultDialog(tk.Toplevel):
    """Small fault-control dialog shell reserved for subsystem fault workflows."""

    def __init__(self, master: tk.Widget, title: str = "Fault Control"):
        super().__init__(master)
        self.title(title)
        self.transient(master)
        self.result: dict[str, str] | None = None


__all__ = ["FaultDialog"]
