"""GridLess action for the KiCad 10 IPC API (kicad-python / kipy).

Runs out of process: takes the open board *including unsaved edits*
(Board.get_as_string), routes it with the gridless executable and creates the
result in a single commit, i.e. one undo step. Requires
Preferences > Plugins > "Enable KiCad API".
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gridless_common as gc  # noqa: E402

from kipy import KiCad  # noqa: E402
from kipy.board_types import ArcTrack, BoardLayer, Track, Via  # noqa: E402
from kipy.geometry import Vector2  # noqa: E402
from kipy.util import from_mm  # noqa: E402


def layer_of(name):
    return getattr(BoardLayer, "BL_" + name.replace(".", "_"))


def main():
    kicad = KiCad()
    board = kicad.get_board()
    doc = board.document
    proj_dir = doc.project.path
    name = doc.board_filename
    stem = os.path.splitext(name)[0]
    orig_path = os.path.join(proj_dir, name)

    opts = gc.ask_options(config_path=gc.config_for(orig_path))
    if opts is None:
        return
    keep, rounds, kicad_check = opts

    # route a snapshot that contains unsaved edits
    tmp = tempfile.mkdtemp(prefix="gridless_board_")
    tmp_pcb = os.path.join(tmp, name)
    with open(tmp_pcb, "w", encoding="utf-8") as f:
        f.write(board.get_as_string())
    project = os.path.join(proj_dir, stem + ".kicad_pro")
    try:
        result = gc.run_router_with_window(tmp_pcb, project, gc.config_for(orig_path), keep, rounds, kicad_check=kicad_check)
    except gc.Cancelled:
        gc.message(gc.tr("Routing cancelled: the board was not changed."))
        return

    nets = {n.name: n for n in board.get_nets()}
    items = []
    for kind, d in gc.iter_items(result):
        if d.get("net") not in nets:
            continue  # never let the API create a new net
        if kind == "seg":
            t = Track()
            t.start = Vector2.from_xy_mm(d["a"]["x"], d["a"]["y"])
            t.end = Vector2.from_xy_mm(d["b"]["x"], d["b"]["y"])
            t.width = from_mm(d["width"])
            t.layer = layer_of(d["layer"])
            t.net = nets[d["net"]]
        elif kind == "arc":
            t = ArcTrack()
            t.start = Vector2.from_xy_mm(d["s"]["x"], d["s"]["y"])
            t.mid = Vector2.from_xy_mm(d["m"]["x"], d["m"]["y"])
            t.end = Vector2.from_xy_mm(d["e"]["x"], d["e"]["y"])
            t.width = from_mm(d["width"])
            t.layer = layer_of(d["layer"])
            t.net = nets[d["net"]]
        elif kind == "via":
            t = Via()
            t.position = Vector2.from_xy_mm(d["pos"]["x"], d["pos"]["y"])
            t.diameter = from_mm(d["dia"])
            t.drill_diameter = from_mm(d["drill"])
            t.net = nets[d["net"]]
        else:
            continue
        items.append(t)

    commit = board.begin_commit()
    try:
        if not keep:
            old = [t for t in list(board.get_tracks()) + list(board.get_vias()) if not getattr(t, "locked", False)]
            if old:
                board.remove_items(old)
        board.create_items(items)
        board.push_commit(commit, gc.tr("GridLess: autoroute"))
    except Exception:
        board.drop_commit(commit)
        raise
    gc.message(gc.tr("Added %d items.\n\n%s\n\nRefill zones (B) to update copper pours.") % (len(items), gc.summary(result)))


if __name__ == "__main__":
    try:
        import wx

        app = wx.App(False)
    except ImportError:
        app = None
    try:
        main()
    except Exception as e:  # noqa: BLE001
        gc.message(gc.tr("GridLess failed:\n%s") % e)
        raise
