from __future__ import annotations

from GUI.gui_context import *


class TableView(ttk.Treeview):
    """Thin Treeview wrapper for tabular GUI data."""

    def replace_rows(self, rows: list[tuple]) -> None:
        for item in self.get_children():
            self.delete(item)
        for row in rows:
            self.insert("", "end", values=row)


__all__ = ["TableView"]
