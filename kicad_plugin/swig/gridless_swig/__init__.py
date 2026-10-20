"""GridLess autorouter — KiCad action plugin (SWIG / pcbnew API, KiCad 9–10)."""

from .gridless_action import GridlessAction

GridlessAction().register()
