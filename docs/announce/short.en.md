# Short announcement texts

Copy, adapt and attach the before/after image. Do not post all of them at once.

## Reddit (r/KiCad)

**Title:** GridLess: a gridless, TopoR-style autorouter for KiCad 10

**Body:**

I built an autorouter that does not use a grid or 45° segments. It builds a
planar topology first, then draws any-angle tracks with arcs around pads,
concentric bundles, differential pairs and length matching.

On my 4-layer reference board (75 nets): 0 clearance errors, 28 vias vs 30 by
hand, 1286 mm vs 1330 mm, about 11 s with KiCad's zone check. A dense BLE board
with a 0.4 mm QFN and an RF section is still only partially routed — that is the
next problem.

It is open source (MIT), and the interface follows KiCad's language setting
(all 44 languages). Install from the Releases page with **Plugin and Content
Manager → Install from File**, or build from source.

Repo: https://github.com/bat654321/GridLess

Happy to answer questions, and very interested in boards where it fails.

## Mastodon / X (short)

GridLess is a gridless topological autorouter for #KiCad 10: any-angle tracks,
arcs around pads, differential pairs, length matching. On a 4-layer reference
board: 0 clearance errors, 28 vias vs 30 by hand, 11 s. MIT, 44 UI languages.
Still weak on 0.4 mm QFN/RF boards. https://github.com/bat654321/GridLess #PCB #EDA #Rust

## Hacker News (Show HN)

**Title:** Show HN: A gridless TopoR-style autorouter for KiCad

**First comment:**

Hi HN. I built GridLess, a PCB autorouter for KiCad 10 that deliberately avoids
a routing grid. Like TopoR, it first decides on which side of each obstacle every
connection passes, then solves the geometry: straight chords at arbitrary angles,
arcs around pads, concentric bundles.

The measured result on a 4-layer, 75-net board is 0 clearance errors after
KiCad's DRC, 28 vias versus 30 by hand, 1286 mm versus 1330 mm, in about 11 s
including a zone refill. It also handles differential pairs and length matching.

What does not work yet: dense 0.4 mm-pitch packages and RF keepouts. A BLE board
with a QFN and a split power plane still ends with about 12 violations and 3
unconnected nets. I would rather say that up front than ship a demo that only
works on one board.

It is written in Rust, MIT licensed, and the plugin UI follows KiCad's language
setting (44 languages). The repo has the CLI, both plugin flavours, the test
boards and a long write-up about the algorithm and the bugs found on the way.

I would love feedback from people who route dense boards, especially reproducible
failures.
