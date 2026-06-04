from __future__ import annotations

from GUI.gui_context import *


class StationEditorDialog(tk.Toplevel):
    """Small station editor shell reserved for station-specific GUI edits."""

    def __init__(self, master: tk.Widget, title: str = "Station Editor"):
        super().__init__(master)
        self.title(title)
        self.transient(master)
        self.result: dict[str, str] | None = None


__all__ = ["StationEditorDialog"]
