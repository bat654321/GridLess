# GridLess: features and algorithms reference

Russian version: [REFERENCE.ru.md](REFERENCE.ru.md).

State as of 23 Nov 2026 (version 0.2.0). This reference describes what the router
does, how it works inside, and which parameters control each stage.
A survey of similar tools and the reasoning behind the architecture is in [ANALYSIS.md](ANALYSIS.md).
An example configuration is in [example.gridless.toml](example.gridless.toml).

## 1. What it is

GridLess is a gridless topological autorouter for KiCad 10, in the style of TopoR.
It first builds the *topology* (which side of each obstacle every trace passes on),
then the exact "rubber band" geometry: segments at any angle and arcs around pads.
The core is written in Rust. It connects to KiCad through two plugins or through the
command line.

## 2. Features

### Routing
* Boards with any number of layers. Inner layers that are almost fully covered by zones
  are treated as planes (including planes split between nets).
* Widths and clearances come from the net classes of the KiCad project. They can be
  overridden by name masks.
* Traces automatically neck down to the minimum rule width in tight spots.
* Differential pairs (detected by the suffixes `_P/_N`, `+/-`, `P/N`, or set
  explicitly): one strip with a constant gap, paired vias side by side, choice of
  equivalent pins (USB-C A6/B6), skew compensation.
* Length matching of net groups with trombone meanders. Space for meanders is reserved
  already at the topology stage (`length_reserve`).
* Preferred layer directions and a per-layer "cost".
* Connecting pads to pours and planes: a pad that the pour of its own net reaches stays
  with the pour. The other pads get a via to the plane.
* Stitching of pour islands after routing (`--stitch`, see §4.9).
* Existing traces can be kept, and only the missing connections are routed (`--keep-existing`).

### Result quality
* Traces go around groups of pads instead of squeezing between pins
  ("contour map", §4.4).
* Routing order follows the designer's approach: first knots of short connections, then
  power, then the rest (§4.5).
* Traces leave rectangular pads at 90° to the pad edge.
* Clearances between neighbouring traces are equalized. A trace between two
  pads runs down the middle ("radar", §4.7).
* Bends are rounded with arcs. The entry into a pad is straight (§4.8).
* Built-in rule check and a "DRC → reroute" feedback loop. If
  `kicad-cli` is available, KiCad itself checks the result.

### Interfaces
* **IPC API plugin (KiCad 10)**: routes the open board including unsaved edits and
  inserts the result as one undo step. Installed through PCM.
* **Classic ActionPlugin (SWIG)**: routes the saved file.
* Both plugins: settings dialog, progress window with a log and a cancel button,
  final report. The interface is translated into all 44 KiCad languages.
* **Command line** `gridless`:

| Command | Purpose |
|---|---|
| `route BOARD -o OUT` | route the board |
| `  -c FILE` | configuration file (default `<board>.gridless.toml`) |
| `  --project FILE` | `.kicad_pro` project with net classes |
| `  --rounds N` | rounds of "geometry → DRC → reroute" (8) |
| `  --keep-existing` | keep existing traces |
| `  --kicad-check` | fill zones through `kicad-cli`, add vias to the plane for pads the pour did not reach, and route again |
| `  --stitch` | stitch pour islands after routing (like `stitch`) |
| `  --json-out FILE` | result as JSON (for the plugins) |
| `  --png FILE --view x0,y0,x1,y1 --scale K` | image of the result |
| `stitch BOARD -o OUT [--net GND]` | stitch pour islands on a finished board |
| `render BOARD --png FILE` | draw the board as is |
| `  --view`, `--scale`, `--layers F.Cu,B.Cu` | window, scale, layers (first one on top) |
| `  --no-fills` | without fills |
| `  --nets A,B` | highlight nets, dim the others |
| `  --markers "x,y;x,y"` | numbered markers |
| `info BOARD [--json FILE]` | board summary, model as JSON |

## 3. Pipeline

```
.kicad_pcb + .kicad_pro (+ .gridless.toml)
  │ 1. Board model and rules             kicad.rs, config.rs
  │ 2. Problem: obstacles, via           problem.rs
  │    sites, connection list
  │ 3. Triangulation of each layer       mesh.rs
  │ 4. Contour map                       terrain.rs
  │ 5. Topology: A*, negotiation,        topo.rs
  │    rip-up, via minimization,
  │    loop cleanup
  │ 6. Rubber band geometry              geometry.rs
  │ 7. DRC → penalties → reroute         drc.rs, pipeline.rs   (up to N rounds)
  │ 8. Finishing: clearance              geometry.rs, smooth.rs
  │    equalization, smoothing
  │ 9. Length matching                   tuning.rs
  │10. Output and pour stitching         output.rs, kicad.rs, stitch.rs
  ▼
.kicad_pcb / JSON
```

## 4. Algorithms and methods

### 4.1. Board model and rules
The `.kicad_pcb` reader parses S-expressions and remembers the position of every node in
the file. Because of this, writing changes only traces and vias, and the rest of the file
stays byte for byte the same. Pads of any shape are represented as "rounded convex
polygons" (a convex core plus a radius). This gives exact distances for clearances. The
reader loads the board outline, zones (including zones nested in footprints: Altium
keepouts), keepouts by type (tracks, vias, pour), and copper graphic shapes.

The rules (`Rules`) are assembled from the project net classes, the board rules and the
configuration file. A non-plated hole is modelled as a circle enlarged so that the
copper-to-hole clearance (`hole_clearance`) holds.

### 4.2. Routing problem
* **Obstacles**: pads, kept traces, vias, copper, keepouts.
* **Via sites**: a hexagonal lattice with a pitch of "diameter + clearance", plus points
  around each SMD pad in 8 directions. Sites that violate clearances to
  obstacles, the board edge or holes are discarded.
* **Connections**: a minimum spanning tree over the pads of each net. An edge between
  pads with no common signal layer costs an extra via (through-hole
  pads and vias change layer for free). A long edge that passes near a
  third pad of the net is split into two edges through that pad.
* **Escape stubs from fine-pitch pads**: a short straight segment along the long axis of
  the pad, pointing away from the package. A package counts as fine-pitch if a track of
  its net does not fit between neighbouring pins (K1: 0.31 mm pads with 0.19 mm clearance).
  A pin connected to the opposite pin of the same package escapes inward, under the package.

### 4.3. Triangulation
A constrained Delaunay triangulation is built for each layer (crate `spade`). The vertices
are obstacle corners and centres, outline points, via sites and pads. Obstacle sides and
outline sides are constraint edges. Each edge stores its "exit distance": the distance taken
by the obstacle at the edge end together with the clearance.

### 4.4. Contour map (terrain)
Copper of other nets is a hill: full height 1 on the copper itself and its clearance
band, then a Gaussian falloff with parameter `terrain_sigma` (0.5 mm). Hills are summed on
a 0.1 mm grid, so a dense group of pins becomes a plateau and the gap between groups
becomes a valley. For a trace, its own net is flat ground: its contribution is subtracted
at query time.

The path search pays for climbing: each step costs `terrain × length × mean height` on
top of the length. A via pays `terrain_via × height` at its site
(averaged over layers). Consequences:
* a trace runs between component groups, not between the pins of one package;
* a trace leaves a row of pins across the row, because this is the fastest way down from the plateau;
* a long section runs on the free layer, and the via is placed at the crowded end;
* vias land in valleys, not among pads.

Debug height image: environment variable `GRIDLESS_TERRAIN_PNG=prefix`.

### 4.5. Routing order (`order`)
`logic` (default) follows the designer's order:
1. **Knots.** A knot is a group of short connections (up to 6 mm) whose ends lie within
   1 mm of each other. A knot has at least 3 connections and is no larger than 8 mm. Knots
   grow from the shortest connections, so they form around a dense core:
   relays with diodes, connector pull-ups, capacitors next to an IC. Knots are routed
   first, the densest ones earlier, each one as a whole.
2. **Short single-layer connections** (up to `short_link`, 3 mm): at least one end is SMD
   and there is a common layer. A layer change here can only do harm.
3. **Power**: nets with raised priority or wider than the default class width.
4. **The remaining** connections, by length. Connections through through-hole pads and
   vias (layer change is free) are also here.
5. **SMD–SMD on different layers** go last: they need a via in any case.

Differential pairs always go before everything else. Older modes: `width` (priority,
width, length), `cost`, `short`, `auto` (route three ways and pick the best one
by KiCad DRC).

### 4.6. Topological routing
A trace is stored as a sequence of crossed triangulation edges. On each edge the
crossings of different traces are kept in strict order, so the sketch of each
layer is planar by construction.

* **Search**: A* over states "edge + gap between already placed
  traces". Moves: through a triangle to the opposite edge, to a vertex
  (target or via site), layer change through a via.
* **Step cost**: length × layer direction multiplier; climbing on the contour map;
  edge overflow (not enough room for everyone) with a growing factor, as in
  PathFinder; accumulated conflict history of the edge; necking penalty;
  penalty for crossing another trace, equal to the cost of rerouting it (rip-up); penalty
  for passing near a pad that the pour must reach (`pour_keepout`); via cost
  (`via.cost`, 12 mm of track) plus the terrain height at its site.
* **Negotiation (negotiated rip-up and reroute)**: connections are routed one by one.
  Crossed traces are removed and queued again. After a pass,
  overflowed edges receive history, half of the connections on them that are cheapest
  to reroute are removed, and the iteration repeats (`iterations`, 40). The
  best state is kept.
* **Via minimization** (like TopoR "layer separation"): each connection with vias
  is retried without them at a prohibitive via cost. The new path is accepted if it is
  no longer than 1.6 times the old one + 3 mm and creates no overflow.
* **Cleanup**: each connection is rerouted alone on the settled board.
  A shorter path or a path with fewer vias is accepted. A connection with a
  large detour is rerouted together with the 1–4 traces that block it
  ("who goes around whom").

### 4.7. Rubber band geometry
For each layer (in parallel) a new triangulation is built from real
obstacles only, including the vias in use. Traces are transferred into it.
The variables are the crossing positions on the edges. Inside a triangle a trace is straight
or wraps around the shared vertex with an arc whose radius accounts for nested traces
(concentric bundles). The initial position comes from the funnel algorithm, followed by
coordinate descent on length with clearance constraints along each edge.

* **Vias are moved** from the lattice node onto the line between the ends they
  connect (the median for three or more), by no more than 1.2 mm, keeping clearances and
  without crossing another trace (otherwise that trace would loop around the new site).
* **Necking**: where there is not enough room, the width is reduced to the rule minimum.
* **Clearance equalization ("radar")**: a trace between two pads runs
  down the middle, and a bundle runs with equal gaps. Each gap is at least its own rule.
  The influence of pads weakens with the distance between them: full up to `centering.full`
  (2 channels), zero from `centering.range` (5 channels). If equalization adds
  violations on a layer, 50 % and 25 % strength are tried.

### 4.8. Checking, feedback, finishing
* **Internal DRC**: copper-to-copper clearances taking both nets' classes into account,
  board edge, net connectivity (taking pours of the same net into account).
* **Feedback**: violation points get a penalty in the edge history, and the responsible
  connections are rerouted. Repeated spots are escalated together with their neighbours.
  A net that the geometry left broken is rerouted as a whole. The best round is kept.
* **Smoothing**: a short escape tail is absorbed if the trace can enter the
  pad directly. The exit from a rectangular pad goes through the middle of the edge at 90°.
  Bends are rounded with tangent arcs with a sag of no more than 0.05 mm. Each
  change is checked for clearances.
* **Finishing safeguard**: a stage that added violations on a layer is rolled back on
  that layer. A stage that broke a net is rolled back entirely.
* **45° snapping** (`angle45`, off): finished traces are snapped to
  0/45/90°. Traces with violations are reverted.

### 4.9. Length matching and pour stitching
* **Lengths**: trombone meanders with rounded corners on straight sections. The amplitude
  is chosen by binary search so that clearances are not violated. Diff pair skew is
  computed along the whole path through intermediate components and compensated on the
  shorter trace.
* **Stitching** (`stitch`, `route --stitch`): zones are refilled through `kicad-cli`,
  and an island graph is built (islands connected by pads, traces and vias).
  Each component is connected to the main one with a via where islands on two layers
  overlap, or with a short track to the pour on the other layer. Pads that the
  pour did not reach (closed thermal barrier) get a short track to the pour.

### 4.10. Output
Segments and arcs are written to `.kicad_pcb`. An arc at a pad or via
ends with a 20 µm segment (otherwise KiCad does not consider it connected). Nearly
straight arcs are written as segments. Fragments shorter than 20 µm are merged with the
neighbouring segment.

## 5. Configuration file parameters

All are optional. Default values are given in parentheses.

| Key | Meaning |
|---|---|
| `order` | routing order: `logic`, `width`, `cost`, `short`, `auto` (`logic`) |
| `short_link` | threshold for a "short" connection, mm (3) |
| `terrain` | cost of climbing on the contour map per mm of track; 0 = off (1.5) |
| `terrain_sigma` | hill falloff beyond the clearance band, mm (0.5) |
| `terrain_via` | via cost per unit of height, mm (2) |
| `pad_exit` | exit from rectangular pads at 90° (`true`) |
| `[via] cost` | via cost in mm of track (12) |
| `[via] diameter`, `drill` | via dimensions (from the Default class) |
| `[[nets]] match, width, clearance, skip, priority` | net rules by mask |
| `[layers."X"] role, direction, direction_weight, cost` | layer role, direction and cost |
| `[[diff_pairs]] p, n, width, gap, max_skew` | differential pairs |
| `auto_diff_pairs` | detect pairs by suffix (`true`) |
| `[[length_groups]] match, target, tolerance` | length matching groups |
| `length_reserve` | space reserved for meanders: `auto`, `always`, `off` (`auto`) |
| `[centering] enabled, full, range` | clearance equalization (on; 2 and 5 channels) |
| `trust_pours` | leave a pad that the pour reaches to the pour (`true`) |
| `pour_keepout` | zone around such a pad where signals pay more, mm (0) |
| `keep_existing` | keep existing traces (`false`) |
| `iterations` | negotiation iterations (40) |
| `mst_via_cost` | include via cost in the connection tree (`true`) |
| `same_layer_bias` | via cost multiplier for nets whose pads are all on one layer (1) |
| `dynamic_terminals` | a connection may end on any unconnected part of the net (`false`) |
| `detour_relax` | relaxation of layer direction during detour cleanup (0) |
| `angle45`, `angle_cost` | 45° snapping and cost of deviation from 45° (0) |
| `ridge`, `ridge_cost`, `center_gap`, `ridge_range`, `ridge_step` | experiments with a distance-to-copper field, off |
| `force_fanout` | `REF.PAD` pads that must get a via to the plane |

Debug environment variables: `GRIDLESS_DEBUG_NET=name` (how a net was routed),
`GRIDLESS_TERRAIN_PNG=prefix` (terrain image), `GRIDLESS_NO_VIA_RELAX` (do not
move vias), `GRIDLESS_DEBUG_DRC`, `GRIDLESS_DEBUG_POINT=x,y`.

## 6. Results on the reference board

StandaloneProgrammer: 2 layers, 93 components, 319 pads, 66 nets, GND pours on
both layers. Hand-routed in Altium. The reference and the empty board are prepared by the scripts in
[tools/bench](../tools/bench/README.md). Checked with KiCad 10 DRC with zone refill.

| | Hand-routed | GridLess (`route --stitch`, no configuration file) |
|---|---|---|
| KiCad DRC errors | 0 (after adjusting the rules to the board) | 0 |
| Unconnected items | 0 | 0 |
| Signal vias | 63 | 51 |
| GND vias (stitching) | 13 | 16 |
| Track length | 1412 mm | 1390 mm |
| Time | — | ≈ 28 s |

Before this work, the same board had 80 vias, 16 internal DRC violations and 13
unconnected items.

## 7. Known limitations

* Power nets are routed as a tree of the shortest links between pads, not as one
  trunk with branches, as a designer would do it.
* Inside a knot the connection order is not searched, and the knot's traces are not
  protected from rerouting by distant connections: NetK1_2 keeps 2 vias, the reference has none.
* In the dense area (DA1/DA2) some traces still loop around other nets' vias.
* The final rubber band is isotropic: orthogonality and 45° are not enforced.
* RF sections and packages with 0.4 mm pitch need more work on escape templates.
