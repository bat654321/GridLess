"""Shared helpers of the GridLess KiCad plugins (IPC and SWIG flavours).

The router itself is the `gridless` executable (Rust). The plugins only
  1. hand it the board file (+ project and optional `<board>.gridless.toml`),
  2. read back the JSON result,
  3. create the tracks / arcs / vias in the open board.
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CREATE_NO_WINDOW = 0x08000000 if sys.platform == "win32" else 0


class Cancelled(Exception):
    """The user stopped the routing; the board is left as it was."""


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


def _start(board_path, project_path, config_path, keep_existing, rounds, kicad_check):
    """Start the router; returns (process, path of the JSON result)."""
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
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace", creationflags=CREATE_NO_WINDOW)
    return proc, json_out


def _result(proc, json_out, lines):
    if proc.returncode != 0 or not os.path.isfile(json_out):
        raise RuntimeError("router failed (exit %s):\n%s" % (proc.returncode, "\n".join(lines[-20:])))
    with open(json_out, encoding="utf-8") as f:
        return json.load(f)


def _kill(proc):
    """Stop the router and whatever it started (kicad-cli during the zone check)."""
    if proc.poll() is not None:
        return
    if sys.platform == "win32":
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=CREATE_NO_WINDOW)
    else:
        proc.kill()
    try:
        proc.wait(timeout=5)
    except Exception:  # noqa: BLE001
        pass


def run_router(board_path, project_path=None, config_path=None, keep_existing=False, rounds=8, log=None, kicad_check=False):
    """Run the router on `board_path`; returns the parsed JSON result."""
    proc, json_out = _start(board_path, project_path, config_path, keep_existing, rounds, kicad_check)
    lines = []
    for line in proc.stdout:
        line = line.rstrip()
        lines.append(line)
        if log:
            log(line)
    proc.wait()
    return _result(proc, json_out, lines)


_LOG_PREFIX = re.compile(r"^\[(\w+)\s+[\w:]+\]\s*")
_TIME_SUFFIX = re.compile(r"\s*\(\d+(\.\d+)?s\)$")


def tidy(line):
    """Router log line without the logger prefix and the timestamp suffix."""
    m = _LOG_PREFIX.match(line)
    if m:
        level = m.group(1)
        line = ("%s: " % level.capitalize() if level in ("WARN", "ERROR") else "") + line[m.end():]
    return _TIME_SUFFIX.sub("", line)


class Progress:
    """Share of the work done, read from the router's log: topology, the DRC feedback
    rounds, finishing, length matching. With the zone check the two passes take half each."""

    def __init__(self, rounds, kicad_check):
        self.rounds = max(1, rounds)
        self.check = kicad_check
        self.second = False
        self.done = 0.0
        self.stage = "Starting..."

    def feed(self, msg):
        first = msg.startswith("[pass 1] ")
        if first:
            msg = msg[len("[pass 1] "):]
        f = None
        m = re.match(r"round (\d+): (\d+) violations", msg)
        if msg.startswith("problem:"):
            f, self.stage = 0.02, "Reading the board"
        elif msg.startswith("meshes:"):
            f, self.stage = 0.05, "Topology"
        elif msg.startswith("topology:"):
            f, self.stage = 0.25, "Geometry"
        elif msg.startswith("length reserve"):
            f, self.stage = 0.3, "Geometry (room reserved for meanders)"
        elif m:
            k = int(m.group(1)) + 1
            f = 0.3 + 0.55 * min(k, self.rounds) / self.rounds
            self.stage = "DRC round %d of up to %d: %s violation(s)" % (k, self.rounds, m.group(2))
        elif msg.startswith(("finishing", "gap equalisation", "smoothing")):
            f, self.stage = 0.9, "Finishing: gap equalisation, smoothing"
        elif msg.startswith("length "):
            f, self.stage = 0.95, "Length matching"
        elif "kicad-cli not found" in msg:
            self.check = False  # one pass only
        elif msg.startswith("KiCad zone check"):
            self.second = True
            f, self.stage = 0.0, "Copper pours checked with KiCad: routing again"
        elif msg.startswith("done in"):
            f, self.stage = 1.0, "Writing the result"
        two = self.second and not first
        if f is not None:
            if self.check:
                f = 0.5 + 0.5 * f if two else 0.5 * f
            self.done = max(self.done, f)
        if self.check:
            return self.done, "Pass %d of 2: %s" % (2 if two else 1, self.stage)
        return self.done, self.stage


def run_router_with_window(board_path, project_path=None, config_path=None, keep_existing=False, rounds=8, kicad_check=False, parent=None, _hook=None):
    """run_router with a progress window: stage, progress bar, elapsed time, the live log
    and a Cancel button. Raises Cancelled when the user stops the routing. Without wx it
    simply runs with the log on the console."""
    try:
        import wx
    except ImportError:
        return run_router(board_path, project_path, config_path, keep_existing, rounds, log=print, kicad_check=kicad_check)

    proc, json_out = _start(board_path, project_path, config_path, keep_existing, rounds, kicad_check)
    lines = []
    prog = Progress(rounds, kicad_check)
    state = {"running": True, "cancelled": False, "error": None, "result": None}
    t0 = time.time()

    dlg = wx.Dialog(parent, title="GridLess autorouter", style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
    sz = wx.BoxSizer(wx.VERTICAL)
    stage = wx.StaticText(dlg, label=prog.stage)
    sz.Add(stage, 0, wx.LEFT | wx.RIGHT | wx.TOP | wx.EXPAND, 10)
    row = wx.BoxSizer(wx.HORIZONTAL)
    gauge = wx.Gauge(dlg, range=1000, size=(-1, 18))
    row.Add(gauge, 1, wx.ALIGN_CENTER_VERTICAL)
    clock = wx.StaticText(dlg, label="0:00")
    row.Add(clock, 0, wx.LEFT | wx.ALIGN_CENTER_VERTICAL, 10)
    sz.Add(row, 0, wx.ALL | wx.EXPAND, 10)
    log = wx.TextCtrl(dlg, style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_DONTWRAP, size=(760, 320))
    log.SetFont(wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
    sz.Add(log, 1, wx.LEFT | wx.RIGHT | wx.EXPAND, 10)
    btn = wx.Button(dlg, wx.ID_CANCEL, "Cancel")
    sz.Add(btn, 0, wx.ALL | wx.ALIGN_RIGHT, 10)
    dlg.SetSizerAndFit(sz)
    dlg.CentreOnParent()

    def tick(_evt=None):
        s = int(time.time() - t0)
        clock.SetLabel("%d:%02d" % (s // 60, s % 60))

    timer = wx.Timer(dlg)
    dlg.Bind(wx.EVT_TIMER, tick, timer)
    timer.Start(500)

    def on_line(line):
        if state["cancelled"]:
            return
        msg = tidy(line)
        log.AppendText(msg + "\n")
        done, st = prog.feed(msg)
        gauge.SetValue(int(done * 1000))
        stage.SetLabel(st)

    def on_exit():
        state["running"] = False
        if state["cancelled"]:
            return
        try:
            state["result"] = _result(proc, json_out, lines)
        except Exception as e:  # noqa: BLE001
            # keep the window and its log open; the button now closes it
            state["error"] = e
            stage.SetLabel("Routing failed")
            log.AppendText("\n%s\n" % e)
            btn.SetLabel("Close")
            return
        gauge.SetValue(1000)
        dlg.EndModal(wx.ID_OK)

    def reader():
        for line in proc.stdout:
            line = line.rstrip()
            lines.append(line)
            wx.CallAfter(on_line, line)
        proc.wait()
        wx.CallAfter(on_exit)

    def stop(_evt=None):
        if state["running"]:
            state["cancelled"] = True
            stage.SetLabel("Cancelling...")
            _kill(proc)
        dlg.EndModal(wx.ID_CANCEL)

    btn.Bind(wx.EVT_BUTTON, stop)
    dlg.Bind(wx.EVT_CLOSE, stop)
    threading.Thread(target=reader, daemon=True).start()
    if _hook:
        _hook(dlg, btn)
    dlg.ShowModal()
    timer.Stop()
    dlg.Destroy()
    if state["cancelled"]:
        raise Cancelled()
    if state["error"] is not None:
        raise state["error"]
    return state["result"]


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
