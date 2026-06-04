from __future__ import annotations

from GUI.gui_context import *


class TrainEditorDialog(tk.Toplevel):
    """Small train editor shell reserved for train-specific GUI edits."""

    def __init__(self, master: tk.Widget, title: str = "Train Editor"):
        super().__init__(master)
        self.title(title)
        self.transient(master)
        self.result: dict[str, str] | None = None


__all__ = ["TrainEditorDialog"]
