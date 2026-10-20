"""Routing quality metrics of a .kicad_pcb (via `gridless info --json`).

    python tools/analyze_board.py board.kicad_pcb [--exe target/release/gridless.exe]

Prints vias, nets spread over several layers, layer changes at through-hole pads,
sharp corners (< 120 deg between consecutive straight segments) and detour ratio
(routed length / MST length) per net.
"""

import argparse
import collections
import json
import math
import os
import subprocess
import sys
import tempfile


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("board")
    ap.add_argument("--exe", default=os.path.join(os.path.dirname(__file__), "..", "target", "release", "gridless.exe"))
    ap.add_argument("--nets", default="")
    a = ap.parse_args()
    js = os.path.join(tempfile.mkdtemp(), "m.json")
    subprocess.run([a.exe, "info", a.board, "--json", js], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    m = json.load(open(js))
    L = [l["name"] for l in m["layers"]]
    N = [n["name"] for n in m["nets"]]
    per = collections.defaultdict(lambda: collections.defaultdict(float))
    segs = collections.defaultdict(list)
    for t in m["tracks"]:
        g = t["geom"]
        if "Seg" in g:
            a_, b_ = g["Seg"]["a"], g["Seg"]["b"]
            pa, pb = (a_["x"], a_["y"]), (b_["x"], b_["y"])
            ln = math.dist(pa, pb)
            segs[(t["net"], t["layer"])].append((pa, pb))
        else:
            s, mm, e = g["Arc"]["s"], g["Arc"]["m"], g["Arc"]["e"]
            ln = math.dist((s["x"], s["y"]), (mm["x"], mm["y"])) + math.dist((mm["x"], mm["y"]), (e["x"], e["y"]))
        per[t["net"]][L[t["layer"]]] += ln
    vias = collections.Counter(v["net"] for v in m["vias"])
    # junction points (pad centres, vias) are not "corners"
    anchors = [(p["pos"]["x"], p["pos"]["y"]) for p in m["pads"]] + [(v["pos"]["x"], v["pos"]["y"]) for v in m["vias"]]
    def at_anchor(q):
        return any(abs(q[0] - a[0]) < 0.05 and abs(q[1] - a[1]) < 0.05 for a in anchors)
    sharp = collections.defaultdict(list)
    for (net, layer), ss in segs.items():
        ends = collections.defaultdict(list)
        for i, (p0, p1) in enumerate(ss):
            ends[(round(p0[0], 4), round(p0[1], 4))].append((i, 0))
            ends[(round(p1[0], 4), round(p1[1], 4))].append((i, 1))
        for q, lst in ends.items():
            if len(lst) != 2:
                continue
            (i, ei), (j, ej) = lst
            a1, b1 = ss[i]
            a2, b2 = ss[j]
            d1 = (b1[0] - a1[0], b1[1] - a1[1]) if ei == 0 else (a1[0] - b1[0], a1[1] - b1[1])
            d2 = (b2[0] - a2[0], b2[1] - a2[1]) if ej == 0 else (a2[0] - b2[0], a2[1] - b2[1])
            l1, l2 = math.hypot(*d1), math.hypot(*d2)
            if l1 < 1e-6 or l2 < 1e-6:
                continue
            c = (d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
            if ang < 120 and not at_anchor(q):
                sharp[N[net]].append((round(ang), L[layer], round(q[0], 2), round(q[1], 2)))
    # layer changes at through-hole pads: pads touched by tracks of their net on >1 layer
    tht_switch = 0
    for p in m["pads"]:
        if p["net"] is None or len(p["layers"]) < len(L):
            continue
        q = (p["pos"]["x"], p["pos"]["y"])
        lays = set()
        for (net, layer), ss in segs.items():
            if net != p["net"]:
                continue
            if any(math.dist(q, s[0]) < 1e-3 or math.dist(q, s[1]) < 1e-3 for s in ss):
                lays.add(layer)
        if len(lays) > 1:
            tht_switch += 1
    multi = [N[i] for i in per if len(per[i]) >= 2]
    print("vias: %d   nets on >=2 layers: %d   THT pads used as layer switch: %d   sharp corners: %d"
          % (sum(vias.values()), len(multi), tht_switch, sum(len(v) for v in sharp.values())))
    print("total length: %.1f mm" % sum(sum(v.values()) for v in per.values()))
    want = [n for n in a.nets.split(",") if n]
    for n in want:
        if n in N:
            i = N.index(n)
            print("  %-12s %s vias %d sharp %s" % (n, {k: round(v, 1) for k, v in per[i].items()}, vias.get(i, 0), sharp.get(n, [])[:5]))
    if not want:
        worst = sorted(sharp.items(), key=lambda kv: -len(kv[1]))[:8]
        for n, lst in worst:
            print("  sharp %-12s %s" % (n, lst[:4]))


if __name__ == "__main__":
    sys.exit(main())
