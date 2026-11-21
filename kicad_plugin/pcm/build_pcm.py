"""Build a KiCad Plugin and Content Manager (PCM) archive for GridLess.

The archive contains the IPC API plugin, the router binary/binaries and all
interface translations.  It follows the layout required by KiCad:

    plugins/       the plugin files (plugin.json, Python sources, bin/, locales/)
    resources/     icon.png (64x64) shown by the PCM
    metadata.json  package control file

Examples
--------
Windows only, with the release build in the repository:

    python kicad_plugin/pcm/build_pcm.py --windows target/release/gridless.exe

One archive for all platforms (build the binaries elsewhere and point to them):

    python kicad_plugin/pcm/build_pcm.py \\
        --windows build/gridless.exe \\
        --linux build/gridless-linux-x86_64 \\
        --macos-arm64 build/gridless-macos-arm64 \\
        --macos-x86_64 build/gridless-macos-x86_64

The script prints the SHA-256 and sizes and writes ``dist/metadata.json`` for
submission to https://gitlab.com/kicad/addons/metadata.  Pass ``--download-url``
with the public release asset URL to put the final link into that file.
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN_ROOT = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(PLUGIN_ROOT)
sys.path.insert(0, PLUGIN_ROOT)

from install import png  # noqa: E402

DEFAULT_IDENTIFIER = "com.github.bat654321.gridless"
DEFAULT_NAME = "GridLess"
DEFAULT_AUTHOR = "bat654321"
DEFAULT_URL = "https://github.com/bat654321/GridLess"

SHORT = (
    "Gridless topological autorouter for KiCad 10: any-angle tracks with arcs, "
    "differential pairs, length matching and DRC feedback."
)

LONG = """GridLess is a gridless, topological autorouter for KiCad 10, in the spirit of TopoR.
It first decides which side of which obstacle every track passes, then draws any-angle
tracks with arcs around pads, concentric bundles, differential pairs and length matching.

Features:
- any-angle tracks and arcs, no 45-degree grid
- differential pairs kept coupled, with skew compensated through intermediate components
- length matching with meanders, and room for them reserved already in the topology
- preferred layer directions, via minimisation, automatic track necking
- planes and split planes; optional zone check through kicad-cli plus plane vias
- a feedback loop driven by KiCad's DRC and by the router's own clearance check
- the plugin interface is translated into all 44 languages shipped with KiCad 10
- a command-line tool plus two KiCad plugins (IPC API and the classic SWIG API)

Status: early testing.  On the author's 4-layer reference board it matches the hand
routing with zero clearance errors and fewer vias; dense boards (0.4 mm QFN, RF
sections) are still an open problem.  Tested with KiCad 10.0 on Windows; Linux and
macOS binaries can be included in the same archive but are tested less.  Save your
board and keep a backup: this is an autorouter, not a guarantee.

The IPC plugin needs Preferences > Plugins > "Enable KiCad API".  The classic
plugin, which routes the saved board file and needs no API server, is included in
the source repository's installer.

The dialogs use wxWidgets, which KiCad's bundled Python provides on Windows and
macOS.  On Linux the external Python may not have it; the plugin then routes with
default options and prints the report (install wxPython for the dialogs).
"""


def workspace_version():
    try:
        with open(os.path.join(REPO_ROOT, "Cargo.toml"), encoding="utf-8") as f:
            text = f.read()
        m = re.search(r'\[workspace\.package\](?:(?!\n\[).)*?\bversion\s*=\s*"([^"]+)"', text, re.S)
        if m:
            return m.group(1)
    except OSError:
        pass
    return "0.1.0"


def binary_arguments(parser):
    parser.add_argument("--windows", help="Windows gridless.exe")
    parser.add_argument("--linux", help="Linux x86_64 gridless binary")
    parser.add_argument("--linux-arm64", help="Linux arm64 gridless binary")
    parser.add_argument("--macos-arm64", help="macOS Apple Silicon gridless binary")
    parser.add_argument("--macos-x86_64", help="macOS Intel gridless binary")


def collect_binaries(args):
    """Return [(archive_name, source_path, platform)].  At least one binary is required."""
    found = []
    specs = [
        ("windows", "gridless.exe", args.windows),
        ("linux", "gridless-linux-x86_64", args.linux),
        ("linux", "gridless-linux-arm64", args.linux_arm64),
        ("macos", "gridless-macos-arm64", args.macos_arm64),
        ("macos", "gridless-macos-x86_64", args.macos_x86_64),
    ]
    for platform, name, path in specs:
        if not path:
            continue
        if not os.path.isfile(path):
            raise SystemExit("binary not found: %s" % path)
        found.append((name, path, platform))

    if not found:
        # Convenient default: use the local release build.
        local = os.path.join(REPO_ROOT, "target", "release", "gridless.exe")
        if not os.path.isfile(local):
            local = os.path.join(REPO_ROOT, "target", "release", "gridless")
        if os.path.isfile(local):
            if local.endswith(".exe"):
                found.append(("gridless.exe", local, "windows"))
            elif sys.platform == "darwin":
                found.append(("gridless-macos-" + ("arm64" if os.uname().machine == "arm64" else "x86_64"), local, "macos"))
            else:
                found.append(("gridless-linux-x86_64", local, "linux"))
    if not found:
        raise SystemExit("no router binary given; pass --windows/--linux/--macos-* or build target/release/gridless")
    return found


def write_metadata(args, version, platforms, icon_dir):
    author = {
        "name": args.author,
        "contact": {"github": args.author_url},
    }
    meta = {
        "$schema": "https://go.kicad.org/pcm/schemas/v2",
        "name": args.name,
        "description": SHORT,
        "description_full": LONG,
        "identifier": args.identifier,
        "type": "plugin",
        "author": author,
        "maintainer": author,
        "license": "MIT",
        "resources": {
            "homepage": args.author_url,
            "repository": args.author_url,
            "documentation": args.author_url + "#readme",
        },
        "tags": ["autorouter", "routing", "pcb", "differential-pairs", "length-matching"],
        "versions": [
            {
                "version": version,
                "status": args.status,
                "kicad_version": args.kicad_version,
                "platforms": platforms,
                "runtime": "ipc",
            }
        ],
    }
    return meta


def make_icon(path, size, colour):
    png(path, size, colour)


def copy_plugin(stage, binaries):
    plugins = os.path.join(stage, "plugins")
    os.makedirs(plugins)
    for name in ("plugin.json", "requirements.txt", "gridless_action.py"):
        shutil.copy2(os.path.join(PLUGIN_ROOT, "ipc", name), plugins)
    shutil.copy2(os.path.join(REPO_ROOT, "LICENSE"), plugins)
    # The IPC action is loaded from the plugin root; the file keeps its name.
    shutil.copy2(os.path.join(PLUGIN_ROOT, "common", "gridless_common.py"), plugins)
    shutil.copy2(os.path.join(PLUGIN_ROOT, "common", "gridless_i18n.py"), plugins)
    locales = os.path.join(plugins, "locales")
    os.makedirs(locales)
    source_locales = os.path.join(PLUGIN_ROOT, "common", "locales")
    for name in sorted(os.listdir(source_locales)):
        if name.endswith(".json"):
            shutil.copy2(os.path.join(source_locales, name), os.path.join(locales, name))
    os.makedirs(os.path.join(plugins, "bin"))
    for archive_name, source, _platform in binaries:
        target = os.path.join(plugins, "bin", archive_name)
        shutil.copy2(source, target)
        if not archive_name.endswith(".exe"):
            os.chmod(target, 0o755)
    make_icon(os.path.join(plugins, "icon.png"), 24, (200, 60, 50, 255))
    make_icon(os.path.join(plugins, "icon-dark.png"), 24, (240, 120, 100, 255))


def tree_size(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            total += os.path.getsize(os.path.join(root, name))
    return total


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def zip_dir(stage, target):
    with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as z:
        for root, _dirs, files in os.walk(stage):
            for name in sorted(files):
                full = os.path.join(root, name)
                rel = os.path.relpath(full, stage)
                z.write(full, rel.replace(os.sep, "/"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", default=workspace_version())
    ap.add_argument("--out", default=os.path.join(REPO_ROOT, "dist"))
    ap.add_argument("--identifier", default=DEFAULT_IDENTIFIER)
    ap.add_argument("--name", default=DEFAULT_NAME)
    ap.add_argument("--author", default=DEFAULT_AUTHOR)
    ap.add_argument("--author-url", default=DEFAULT_URL)
    ap.add_argument("--download-url", help="public URL of the finished archive")
    ap.add_argument("--status", default="testing", choices=["stable", "testing", "development"])
    ap.add_argument("--kicad-version", default="10.0")
    binary_arguments(ap)
    args = ap.parse_args()

    binaries = collect_binaries(args)
    platforms = sorted({p for _n, _s, p in binaries})
    os.makedirs(args.out, exist_ok=True)

    stage = tempfile.mkdtemp(prefix="gridless_pcm_")
    try:
        copy_plugin(stage, binaries)
        resources = os.path.join(stage, "resources")
        os.makedirs(resources)
        make_icon(os.path.join(resources, "icon.png"), 64, (200, 60, 50, 255))

        meta = write_metadata(args, args.version, platforms, resources)
        with open(os.path.join(stage, "metadata.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
            f.write("\n")

        base = "%s-%s-pcm" % (args.name.lower().replace(" ", "-"), args.version)
        archive = os.path.join(args.out, base + ".zip")
        zip_dir(stage, archive)

        digest = sha256(archive)
        download_size = os.path.getsize(archive)
        install_size = tree_size(stage)

        submit = json.loads(json.dumps(meta))
        url = args.download_url or "%s/releases/download/v%s/%s.zip" % (
            args.author_url.rstrip("/"),
            args.version,
            base,
        )
        submit["versions"][0].update(
            {
                "download_url": url,
                "download_sha256": digest,
                "download_size": download_size,
                "install_size": install_size,
            }
        )
        meta_path = os.path.join(args.out, "metadata.json")
        with open(meta_path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(submit, f, ensure_ascii=False, indent=2)
            f.write("\n")

        print("archive      :", archive)
        print("platforms    :", ", ".join(platforms))
        print("download_size:", download_size)
        print("install_size :", install_size)
        print("sha256       :", digest)
        print("metadata     :", meta_path)
        if not args.download_url:
            print("\nSet --download-url (or edit metadata.json) once the archive is uploaded.")
    finally:
        shutil.rmtree(stage, ignore_errors=True)


if __name__ == "__main__":
    main()
