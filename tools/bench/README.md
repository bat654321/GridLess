# Bench: comparing GridLess with a hand-routed board

Scripts used to tune the router against the hand-routed StandaloneProgrammer board
(imported from Altium). They work in a scratch folder, for example `out/sp/`; the
board itself is not part of the repository.

1. Copy the reference board and project into the scratch folder as `ref.kicad_pcb`
   and `ref.kicad_pro`, then run `python prep.py`. It writes
   * `ref_fixed.kicad_pcb`: the reference with the import defects removed
     (doubled Edge.Cuts lines, zero-length tracks) and project rules matched to the
     board (0.19 mm pad gap of a 0.5 mm pitch part, solder mask bridges inside
     footprints, 1 thermal spoke, a Power net class);
   * `bare.kicad_pcb`: the same board without tracks and vias, pours kept.
2. `powershell -ExecutionPolicy Bypass -File run.ps1 TAG CONFIG.toml` routes
   `bare.kicad_pcb` into `TAG.kicad_pcb`, runs KiCad's DRC with zone refill and
   renders `TAG_FB.png` (whole board) and `TAG_R.png` (one dense area). Extra router
   arguments go in `$env:GL_EXTRA`, e.g. `--stitch`. `par.ps1 TAG1 TAG2 ...` runs
   several configs (`TAGn.toml`) in parallel.
3. Judge the pictures first. Numbers only point at places to look:
   * `cmp.py ref_fixed.kicad_pcb TAG.kicad_pcb`: length per layer and vias per net,
     sorted by the difference to the reference;
   * `viad.py`: how far signal vias sit from their own pads;
   * `drcsum.py`, `unc.py`: KiCad DRC summary and unconnected items;
   * `box.py BOARD x0,y0,x1,y1`: every track and via in a window;
   * `gridless render BOARD --png F.png --no-fills --view x0,y0,x1,y1 --nets A,B`
     shows a window and highlights nets.
