# Changelog

All notable changes to GridLess are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions use
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- Contour map (`terrain`, on by default): foreign copper is a hill, the path search
  pays for climbing. Tracks run between groups of pads, leave pin rows square to them,
  take the long run on the free layer and drop the via near the crowded end.
- Connection order `logic` (default): knots of short links (a part with its close
  neighbours) first, densest first; then short same-layer links, power, the rest;
  SMD-to-SMD links on different layers last.
- `route --stitch`: join pour islands and reach pads the pour misses right after
  routing. `render --no-fills --nets A,B` for looking at the result.
- Reference of features and algorithms: `docs/REFERENCE.ru.md`; bench scripts for
  comparing with a hand-routed board: `tools/bench`.

- `gridless stitch <board>`: join the copper islands of a net's pours with vias.
  It refills the zones through `kicad-cli` first (so the islands are the ones the
  router actually leaves), builds the island graph from pads, tracks and vias,
  and connects every component to the largest one with a via where two layers'
  islands overlap. `--net GND` limits it to one net, `--no-refill` uses the saved
  fills. This is the step that closes unconnected ground islands on boards where
  signal routing cut the pour.
- Zones nested in footprints (Altium component keepouts, thermal pours) are read now.
  Their coordinates are absolute; keepout flags are kept per kind, so a keepout that
  only blocks pours does not block tracks. The router used to cross XT1's keepout.
- `pour_keepout` (mm): room around a pad that a same-net pour has to reach. Signal
  routing pays an extra cost there, so the pour stays connected to its pad instead
  of being cut away by a track. Off by default.

### Changed

- Defaults: `pad_exit` on (90 degree exits from rectangular pads), via cost 12 mm.
- A part is fine-pitch when a track does not fit between its pins (not by pad width);
  a pin joined to the pin across its own part leaves inwards.
- A via no longer moves across a foreign wire (that produced hairpins).
- A finishing stage that leaves a net unconnected is undone; a net the geometry left
  in pieces is re-routed in the next round.
- NPTH holes keep the hole clearance; slivers under 20 um are merged on output.
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
