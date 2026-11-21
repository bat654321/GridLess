# Revised KiCad Forum post

The original post was hidden by community flags as "spam / overly promotional".
This version is written as a technical discussion: it leads with the problem and
method, gives measured results, states the limitations, and asks specific
questions. It has **one** source link and no install instructions or call to
action. Use it to edit the existing topic once staff restore posting rights; do
not repost while the account is silenced.

## Suggested title

```text
Gridless topological autorouting experiment for KiCad 10 — feedback on dense boards
```

## Body

```text
I have been experimenting with a gridless, topological router for KiCad 10 (in
the spirit of TopoR) and would like technical feedback, especially on dense boards.

The idea is to separate routing into two problems:

1. topology: decide which side of which obstacle every connection passes, on a
   constrained Delaunay mesh, with negotiated congestion and rip-up/reroute;
2. geometry: draw the exact copper on the fixed topology — straight chords at
   any angle, arcs around pads, concentric bundles, differential pairs and
   meanders.

![Carrier: same topology without and with gap equalisation](upload://6BxeNNjyBvanMdDuKWthYSnXUcx.png)

On a 4-layer reference board (43 parts, 281 pads, 75 nets, inner layer 1 is GND),
KiCad 10's DRC after a zone refill reports 0 clearance errors, shorts or
crossings. It uses 28 vias versus 30 in my hand routing, 1286 mm versus 1330 mm
of track, and routes in about 11 s with a zone check. The same board without a
config file and without the zone check takes about 4 s with 23 vias. The result is
deterministic.

What I would like to discuss:

- Dense fine-pitch parts. A BLE board with a 0.4 mm QFN, a split plane and an RF
  section still ends with about 12 DRC violations and 3 unconnected nets around
  the RF part. I suspect the escape patterns and keepout handling more than the
  topology itself.
- Length matching. Meanders only fit on straight runs; when there is no room the
  router reports NOT MET. Is reserving length in the topology the right
  trade-off, or should it be handled in geometry?
- Preferred layer directions. They guide layer and corridor choice, but the final
  rubber band is isotropic, so diagonals remain. Has anyone tried an anisotropic
  metric without making the geometry much slower?
- Via minimisation and plane connections. The optional kicad-cli zone check adds
  plane vias where a pour does not reach a pad, then routes again. This is the
  part I am least sure about.

The project is open source (MIT). Source and a longer write-up with the measured
results and the bugs found on the way:

https://github.com/bat654321/GridLess

I am not trying to announce a product; I am interested in whether the
topology-first approach is worth pursuing for boards of this class, and in
reproducible failures. If you have a board where it fails, a minimal .kicad_pcb
would be the most useful feedback.
```

## Message to staff (email or PM)

```text
Hello,

I understand why the post was flagged as promotional. I have revised it to focus
on the technical method, the measured results, the limitations and specific
questions, with a single source link and no installation instructions or call to
action.

Could you restore the account/topic so I can replace the post with the revised
version? I am happy to follow any further guidance about the appropriate format
for project posts in External Plugins.

Thank you,
Mikhail
```

## What changed and why

- Removed the "Install" section, download links and "accept the offer" wording.
- Reduced the number of external links to one (the source repository).
- Reframed the title from a product name to a technical experiment and feedback
  request.
- Kept the measured results, but put them in the context of a method and its
  open questions.
- Made the limitations (dense QFN, RF, length matching) a central part of the
  post rather than a footnote.
- Avoided a signature (the forum profile already identifies the author).
- Did not cross-post the same text to several categories.
