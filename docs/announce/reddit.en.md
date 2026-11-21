# Reddit post (r/KiCad)

Post as a **self post** (text), not a link post. Attach
`docs/articles/img/before_after.en.png`. Choose the flair the subreddit offers
for open-source projects or show-and-tell, if any.

## Title

```text
GridLess: a gridless, TopoR-style autorouter for KiCad 10
```

## Body

```text
I built an autorouter that does not use a grid or 45° segments. It is called
GridLess: it builds a planar topology first, then draws any-angle tracks with
arcs around pads, concentric bundles, differential pairs and length matching.

On my 4-layer reference board (43 parts, 281 pads, 75 nets): 0 clearance errors
after KiCad's DRC, 28 vias vs 30 by hand, 1286 mm vs 1330 mm, about 11 s with
KiCad's zone check. The output is deterministic.

What does not work yet: a dense BLE board with a 0.4 mm QFN, a split power plane
and an RF section still ends with about 12 DRC violations and 3 unconnected nets
around the RF part. Dense fine-pitch parts and RF keepouts are the next step.
Treat it as an assistant and keep a backup of the board.

It is open source (MIT). The plugin interface follows KiCad's language setting
and is translated into all 44 languages shipped with KiCad 10.

Install:
- PCM archive from the release (Install from File), or
- third-party PCM repository:
  https://bat654321.github.io/GridLess/repository.json
- command-line builds for Windows, Linux and macOS are on the release page.

Repo, documentation and the longer write-up:
https://github.com/bat654321/GridLess

Official KiCad catalogue submission (open):
https://gitlab.com/kicad/addons/metadata/-/merge_requests/759

Happy to answer questions, and very interested in boards where it fails.
```

## Notes

- Do not post from an account that is currently suspended. Reddit's anti-abuse
  systems may treat a post immediately after a suspension as suspicious. Finish
  the password reset / appeal, wait for the account to be in good standing, and
  post manually.
- Do not create a new account to get around a suspension.
- The same text works for r/PrintedCircuitBoard, but check that subreddit's rules
  first; some communities restrict self-promotion.
- The `docs/announce/short.en.md` file also has short versions for Mastodon/X and
  Hacker News.
