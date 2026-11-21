"""Install the GridLess KiCad plugins.

    python kicad_plugin/install.py            # install into the KiCad 10 user folders
    python kicad_plugin/install.py --dest DIR # assemble the plugin folders into DIR only

Installs
  * the SWIG action plugin  -> Documents/KiCad/<ver>/scripting/plugins/gridless_swig
  * the IPC API plugin      -> Documents/KiCad/<ver>/plugins/gridless
and copies the router executable (target/release/gridless[.exe]) into both.
"""

import argparse
import os
import shutil
import struct
import sys
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.join(ROOT, "kicad_plugin")
EXE = "gridless.exe" if sys.platform == "win32" else "gridless"


def png(path, size, fg, bg=None):
    """Tiny PNG icon: a rounded track bending around a pad (no PIL needed)."""
    import math

    w = h = size
    px = [[bg or (0, 0, 0, 0) for _ in range(w)] for _ in range(h)]

    def dot(x, y, r, c):
        for j in range(max(0, int(y - r - 1)), min(h, int(y + r + 2))):
            for i in range(max(0, int(x - r - 1)), min(w, int(x + r + 2))):
                d = math.hypot(i + 0.5 - x, j + 0.5 - y)
                if d <= r:
                    px[j][i] = c

    s = size / 24.0
    pad = (120, 120, 120, 255)
    dot(12 * s, 12 * s, 3.2 * s, pad)
    # track: straight, arc around the pad, straight
    for t in range(0, 101):
        u = t / 100.0
        if u < 0.35:
            x, y = 2 * s + (u / 0.35) * 8 * s, 6 * s
        elif u < 0.65:
            a = math.pi * 1.5 + (u - 0.35) / 0.3 * math.pi * 0.5
            x, y = 12 * s + 6 * s * math.cos(a) - 2 * s, 12 * s + 6 * s * math.sin(a)
        else:
            x, y = 16 * s, 12 * s + (u - 0.65) / 0.35 * 10 * s
        dot(x, y, 1.6 * s, fg)
    raw = b"".join(b"\x00" + bytes(v for p in row for v in p) for row in px)

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def assemble(dest_swig, dest_ipc, exe):
    for d in (dest_swig, dest_ipc):
        os.makedirs(os.path.join(d, "bin"), exist_ok=True)
        shutil.copy2(os.path.join(HERE, "common", "gridless_common.py"), d)
        shutil.copy2(os.path.join(HERE, "common", "gridless_i18n.py"), d)
        license_path = os.path.join(ROOT, "LICENSE")
        if os.path.isfile(license_path):
            shutil.copy2(license_path, d)
        shutil.copytree(
            os.path.join(HERE, "common", "locales"),
            os.path.join(d, "locales"),
            dirs_exist_ok=True,
        )
        if exe:
            shutil.copy2(exe, os.path.join(d, "bin", EXE))
    for f in ("__init__.py", "gridless_action.py"):
        shutil.copy2(os.path.join(HERE, "swig", "gridless_swig", f), dest_swig)
    for f in ("plugin.json", "requirements.txt", "gridless_action.py"):
        shutil.copy2(os.path.join(HERE, "ipc", f), dest_ipc)
    png(os.path.join(dest_swig, "icon.png"), 24, (200, 60, 50, 255))
    png(os.path.join(dest_ipc, "icon.png"), 24, (200, 60, 50, 255))
    png(os.path.join(dest_ipc, "icon-dark.png"), 24, (240, 120, 100, 255))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kicad-version", default="10.0")
    ap.add_argument("--dest", help="assemble into this folder instead of the KiCad user folders")
    ap.add_argument("--exe", default=os.path.join(ROOT, "target", "release", EXE))
    a = ap.parse_args()
    exe = a.exe if os.path.isfile(a.exe) else None
    if not exe:
        print("warning: %s not found - build it first (cargo build --release)" % a.exe)
    if a.dest:
        swig = os.path.join(a.dest, "scripting", "plugins", "gridless_swig")
        ipc = os.path.join(a.dest, "plugins", "gridless")
    else:
        docs = os.path.join(os.path.expanduser("~"), "Documents", "KiCad", a.kicad_version)
        swig = os.path.join(docs, "scripting", "plugins", "gridless_swig")
        ipc = os.path.join(docs, "plugins", "gridless")
    assemble(swig, ipc, exe)
    print("SWIG action plugin:", swig)
    print("IPC API plugin    :", ipc)
    print("Restart KiCad (or Tools > External Plugins > Refresh). The IPC plugin needs")
    print("Preferences > Plugins > 'Enable KiCad API'.")


if __name__ == "__main__":
    main()
