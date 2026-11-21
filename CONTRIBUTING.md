# Contributing to GridLess

Thanks for taking a look! GridLess is an experimental autorouter; a reproducible
board and a clear description are worth more than a long report.

## Reporting a routing problem

Please include:

- the KiCad version and operating system;
- a minimal `.kicad_pcb` and, if used, the `.kicad_pro` and
  `<board>.gridless.toml` (remove confidential data or use a reduced board);
- the command line or plugin options (keep existing tracks, DRC rounds, zone
  check);
- the router's log and JSON output, or at least the final report;
- what you expected and what happened (a screenshot helps).

Before routing, keep a copy of the original board. GridLess writes a routed
copy from the command line, but the plugin replaces tracks in the open board.

## Building and testing

```bash
cargo build --release
cargo test --release
```

The executable is `target/release/gridless` (`.exe` on Windows).

For the plugins:

```bash
python kicad_plugin/install.py --dest build/plugins   # inspect the assembled tree
python kicad_plugin/check_i18n.py                     # validate translations
python -m py_compile kicad_plugin/common/*.py kicad_plugin/ipc/*.py
```

Manual install for development:

```bash
cargo build --release
python kicad_plugin/install.py
```

Then restart KiCad or use **Tools → External Plugins → Refresh**.

## Code

- Rust: keep the pipeline stages separated (`problem`, `topo`, `geometry`,
  `drc`, `pipeline`). New costs or clean-up passes should be measurable on the
  reference boards with `tools/analyze_board.py`.
- Python: the plugins are deliberately thin drivers around the executable. Shared
  code belongs in `kicad_plugin/common/`; both plugin flavours must keep working.
- Determinism matters: a hash map may be used for lookups, but never iterated
  where the order changes the result. Add a test when fixing such a bug.

## Translations

The interface follows KiCad's language setting. Each language is one file in
`kicad_plugin/common/locales/`; `en.json` is the source of truth. To fix a
translation:

1. Edit the values in `<code>.json`; do not translate the keys.
2. Keep printf placeholders (`%d`, `%s`, `%.1f`, `%.3f`) unchanged; they may move.
3. Run `python kicad_plugin/check_i18n.py`.
4. Mention in the pull request which language and which terms you changed.

Native-speaker review is especially welcome for Georgian, Tamil, Telugu and Thai,
and for technical terms such as *autorouter*, *differential pair*, *length
matching* and *copper pour*. The router's live log stays in English because it is
diagnostic output from the executable.

## Submitting a package

See [docs/PUBLISHING.md](docs/PUBLISHING.md) for building the PCM archive,
creating a release and submitting metadata to KiCad's addon repository.
