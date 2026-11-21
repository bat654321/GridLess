"""GridLess action plugin for the legacy SWIG API (works in KiCad 10 without enabling
the IPC API server). The router reads the *saved* board file; save before routing.
KiCad wraps everything done in Run() into one undo step."""

import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gridless_common as gc  # noqa: E402

try:
    # Inside KiCad this is the language chosen in Preferences; empty when headless.
    gc.set_language_hint(pcbnew.GetLanguage())
except Exception:  # noqa: BLE001
    pass


def _vec(d):
    return pcbnew.VECTOR2I(pcbnew.FromMM(d["x"]), pcbnew.FromMM(d["y"]))


def apply_result(board, result, keep_existing):
    """Insert the router result into `board` (also used by the headless test)."""
    if not keep_existing:
        old = [t for t in board.GetTracks() if not t.IsLocked()]
        for t in old:
            board.Remove(t)
    nets = {}

    def net(name):
        if name not in nets:
            ni = board.FindNet(name)
            if ni is None:
                raise ValueError("net %s not found in board" % name)
            nets[name] = ni
        return nets[name]

    added = 0
    for kind, d in gc.iter_items(result):
        if kind == "seg":
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(_vec(d["a"]))
            t.SetEnd(_vec(d["b"]))
            t.SetWidth(pcbnew.FromMM(d["width"]))
            t.SetLayer(board.GetLayerID(d["layer"]))
            t.SetNet(net(d["net"]))
        elif kind == "arc":
            t = pcbnew.PCB_ARC(board)
            t.SetStart(_vec(d["s"]))
            t.SetMid(_vec(d["m"]))
            t.SetEnd(_vec(d["e"]))
            t.SetWidth(pcbnew.FromMM(d["width"]))
            t.SetLayer(board.GetLayerID(d["layer"]))
            t.SetNet(net(d["net"]))
        elif kind == "via":
            t = pcbnew.PCB_VIA(board)
            t.SetPosition(_vec(d["pos"]))
            t.SetViaType(pcbnew.VIATYPE_THROUGH)
            t.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            t.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(d["dia"]))
            t.SetDrill(pcbnew.FromMM(d["drill"]))
            t.SetNet(net(d["net"]))
        else:
            continue
        board.Add(t)
        added += 1
    return added


class GridlessAction(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = gc.tr("GridLess: autoroute")
        self.category = gc.tr("Routing")
        self.description = gc.tr("Gridless topological (any-angle, arcs) autorouter")
        self.show_toolbar_button = True
        icon = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icon.png")
        if os.path.isfile(icon):
            self.icon_file_name = icon

    def Run(self):
        try:
            gc.set_language_hint(pcbnew.GetLanguage())
        except Exception:  # noqa: BLE001
            pass
        board = pcbnew.GetBoard()
        path = board.GetFileName()
        if not path or not os.path.isfile(path):
            gc.message(gc.tr("Save the board first: the router works on the saved .kicad_pcb file."))
            return
        opts = gc.ask_options(config_path=gc.config_for(path))
        if opts is None:
            return
        keep, rounds, kicad_check = opts
        project = os.path.splitext(path)[0] + ".kicad_pro"
        try:
            result = gc.run_router_with_window(path, project, gc.config_for(path), keep, rounds, kicad_check=kicad_check)
        except gc.Cancelled:
            gc.message(gc.tr("Routing cancelled: the board was not changed."))
            return
        except Exception as e:  # noqa: BLE001
            gc.message(gc.tr("Routing failed:\n%s") % e)
            return
        n = apply_result(board, result, keep)
        pcbnew.Refresh()
        gc.message(gc.tr("Added %d items.\n\n%s\n\nRefill zones (B) to update copper pours.") % (n, gc.summary(result)))
