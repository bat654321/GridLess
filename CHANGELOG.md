# Changelog

All notable changes to GridLess are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions use
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Changed

- Gap equalisation is no longer dropped for a whole layer when it costs a rule:
  it retries at 50 % and 25 % of the pull, so a dense cluster keeps most of the
  centring instead of losing all of it.
- `trust_pours` now applies to outer-layer pours as well: a pad the filled copper
  reaches is left to the pour, with no track. The check uses the zone's filled
  polygons, not its outline.
- A via is moved off its lattice site onto the line between the terminals it
  joins (the median for three or more), within 1.2 mm and clear of foreign
  copper. This removes the horseshoe a fixed via used to force.

## [0.1.0] — 2026-11-21

First public, packaged release.

### Added

- Gridless topological router core in Rust: planar topology, any-angle geometry,
  arcs around pads, concentric bundles, automatic necking.
- Differential pairs, length matching with meanders, length reserve in topology,
  preferred layer directions, via minimisation, planes and split planes.
- Gap equalisation with the pad-distance "radar" and a smoothing pass.
- Built-in clearance/connectivity check and a DRC feedback loop; optional zone
  check through `kicad-cli` with plane vias.
- Deterministic output on the same board and binary.
- Command-line tool `gridless` with `route` and `info` commands, optional
  `<board>.gridless.toml` configuration.
- KiCad 10 plugins: IPC API (unsaved edits, one undo step, progress window) and
  classic SWIG action plugin (works without the API server).
- Interface translations for all 44 languages shipped with KiCad 10, following
  the language selected in KiCad; `GRIDLESS_LANG` override and catalog checker.
- Plugin and Content Manager packaging script, release workflow and submission
  checklist.
- English README, Russian README, articles and contributor documentation.

### Known limitations

- Dense 0.4 mm packages and RF keepouts are not solved yet (Beacon board remains
  partially routed).
- Length matching only on straight runs; may report `NOT MET`.
- Preferred layer directions do not affect the final isotropic rubber band.
- Copper-pour islands are not stitched automatically.
- The SWIG plugin routes the saved board file, not unsaved edits.

[0.1.0]: https://github.com/bat654321/GridLess/releases/tag/v0.1.0
