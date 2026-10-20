"""Shared helpers of the GridLess KiCad plugins (IPC and SWIG flavours).

The router itself is the `gridless` executable (Rust). The plugins only
  1. hand it the board file (+ project and optional `<board>.gridless.toml`),
  2. read back the JSON result,
  3. create the tracks / arcs / vias in the open board.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def find_exe():
    """Locate the router executable."""
    cands = []
    env = os.environ.get("GRIDLESS_EXE")
    if env:
        cands.append(env)
    exe = "gridless.exe" if sys.platform == "win32" else "gridless"
    cands.append(os.path.join(HERE, "bin", exe))
    cands.append(os.path.join(os.path.dirname(HERE), "bin", exe))
    w = shutil.which("gridless")
    if w:
        cands.append(w)
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def config_for(board_path):
    """`<board>.gridless.toml` next to the board, if present."""
    cfg = os.path.splitext(board_path)[0] + ".gridless.toml"
    return cfg if os.path.isfile(cfg) else None


def run_router(board_path, project_path=None, config_path=None, keep_existing=False, rounds=8, log=None, kicad_check=False):
    """Run the router on `board_path`; returns the parsed JSON result."""
    exe = find_exe()
    if not exe:
        raise RuntimeError("gridless executable not found (set GRIDLESS_EXE or put gridless.exe into the plugin's bin folder)")
    out_dir = tempfile.mkdtemp(prefix="gridless_")
    json_out = os.path.join(out_dir, "result.json")
    cmd = [exe, "route", board_path, "--json-out", json_out, "--rounds", str(rounds)]
    if project_path and os.path.isfile(project_path):
        cmd += ["--project", project_path]
    if config_path:
        cmd += ["--config", config_path]
    if keep_existing:
        cmd.append("--keep-existing")
    if kicad_check:
        cmd.append("--kicad-check")
    flags = 0x08000000 if sys.platform == "win32" else 0  # CREATE_NO_WINDOW
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace", creationflags=flags)
    lines = []
    for line in proc.stdout:
        line = line.rstrip()
        lines.append(line)
        if log:
            log(line)
    proc.wait()
    if proc.returncode != 0 or not os.path.isfile(json_out):
        raise RuntimeError("router failed (exit %s):\n%s" % (proc.returncode, "\n".join(lines[-20:])))
    with open(json_out, encoding="utf-8") as f:
        return json.load(f)


def iter_items(result):
    """Yield ('seg'|'arc'|'via', dict) from the router JSON."""
    for it in result.get("items", []):
        for kind, d in it.items():
            yield kind.lower(), d


def summary(result):
    v = result.get("violations", [])
    un = result.get("unconnected", [])
    lines = [
        "Items: %d, vias: %d, total length: %.1f mm" % (len(result.get("items", [])), result.get("vias", 0), result.get("length_mm", 0.0)),
        "Unrouted connections: %d" % result.get("unrouted", 0),
        "DRC (router check): %d violations" % len(v),
        "Time: %.1f s" % result.get("seconds", 0.0),
    ]
    if un:
        lines.append("Unconnected: " + ", ".join(un))
    for t in result.get("tuning", []):
        lines.append("Length %s: %.3f -> %.3f mm (target %.3f) %s" % (t["net"], t["before"], t["after"], t["target"], "OK" if t["ok"] else "NOT MET"))
    return "\n".join(lines)


def ask_options(parent=None, default_keep=False, config_path=None):
    """Small wx dialog; returns (keep_existing, rounds, kicad_check) or None if cancelled."""
    try:
        import wx
    except ImportError:
        return (default_keep, 8, True)

    dlg = wx.Dialog(parent, title="GridLess autorouter")
    s = wx.BoxSizer(wx.VERTICAL)
    keep = wx.CheckBox(dlg, label="Keep existing tracks (route only missing connections)")
    keep.SetValue(default_keep)
    s.Add(keep, 0, wx.ALL, 8)
    row = wx.BoxSizer(wx.HORIZONTAL)
    row.Add(wx.StaticText(dlg, label="DRC feedback rounds:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
    rounds = wx.SpinCtrl(dlg, min=1, max=50, initial=8)
    row.Add(rounds, 0)
    s.Add(row, 0, wx.ALL, 8)
    check = wx.CheckBox(dlg, label="Check copper pours with KiCad (plane vias where a pour does not reach a pad)")
    check.SetValue(True)
    s.Add(check, 0, wx.ALL, 8)
    cfg_txt = ("Settings: " + config_path) if config_path else "Settings: KiCad net classes only (no <board>.gridless.toml found)"
    s.Add(wx.StaticText(dlg, label=cfg_txt), 0, wx.ALL, 8)
    s.Add(dlg.CreateButtonSizer(wx.OK | wx.CANCEL), 0, wx.ALL | wx.EXPAND, 8)
    dlg.SetSizerAndFit(s)
    ok = dlg.ShowModal() == wx.ID_OK
    res = (keep.GetValue(), rounds.GetValue(), check.GetValue()) if ok else None
    dlg.Destroy()
    return res


def message(text, title="GridLess", parent=None):
    try:
        import wx

        wx.MessageBox(text, title, parent=parent)
    except ImportError:
        print(text)
