//! End-to-end tests on small synthetic boards.

use gridless::config::{Config, Rules};
use gridless::problem::Problem;

fn pad(num: &str, kind: &str, shape: &str, x: f64, y: f64, w: f64, h: f64, layers: &str, net: &str) -> String {
    let drill = if kind == "thru_hole" { "(drill 0.8)" } else { "" };
    format!("(pad \"{num}\" {kind} {shape} (at {x} {y}) (size {w} {h}) {drill} (layers {layers}) (net \"{net}\"))")
}

fn footprint(r: &str, x: f64, y: f64, pads: &[String]) -> String {
    format!("(footprint \"test:{r}\" (layer \"F.Cu\") (at {x} {y}) (property \"Reference\" \"{r}\") {})\n", pads.join(" "))
}

/// Board with `n` copper layers and the given footprints, 30 x 20 mm.
fn board(n_layers: usize, fps: &[String]) -> String {
    let mut layers = String::from("(0 \"F.Cu\" signal)");
    for i in 1..n_layers - 1 {
        layers += &format!(" ({} \"In{}.Cu\" signal)", 2 + 2 * i, i);
    }
    layers += " (2 \"B.Cu\" signal) (25 \"Edge.Cuts\" user)";
    format!(
        "(kicad_pcb (version 20260206) (generator \"test\") (layers {layers})\n\
         (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type solid)) (fill no) (layer \"Edge.Cuts\"))\n{}\n)",
        fps.join("")
    )
}

fn route(src: &str, cfg: &str) -> (gridless::pipeline::Outcome, gridless::board::Board) {
    let dir = std::env::temp_dir().join(format!("gridless_test_{}_{}", std::process::id(), src.len()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.kicad_pcb");
    std::fs::write(&path, src).unwrap();
    let l = gridless::kicad::load(&path).unwrap();
    let cfg = Config::from_toml(cfg).unwrap();
    let rules = Rules::resolve(&l.board, &cfg);
    let pb = Problem::build(&l.board, &rules);
    let out = gridless::pipeline::route(&l.board, &rules, &pb, 6, &mut |_| {});
    // round-trip through the writer
    let items = gridless::output::items(&l.board, &out.geo, &rules);
    let routed = dir.join("routed.kicad_pcb");
    gridless::kicad::write_routed(&l.source, &routed, &items, true).unwrap();
    let back = gridless::kicad::load(&routed).unwrap();
    assert_eq!(back.board.tracks.len() + back.board.vias.len(), items.len());
    (out, l.board)
}

#[test]
fn crossing_nets_two_layers() {
    // A: left-bottom -> right-top, B: left-top -> right-bottom (SMD on top only)
    let fps = vec![
        footprint("J1", 5.0, 10.0, &[pad("1", "smd", "rect", 0.0, -4.0, 1.0, 1.0, "\"F.Cu\"", "A"), pad("2", "smd", "rect", 0.0, 4.0, 1.0, 1.0, "\"F.Cu\"", "B")]),
        footprint("J2", 25.0, 10.0, &[pad("1", "smd", "rect", 0.0, 4.0, 1.0, 1.0, "\"F.Cu\"", "A"), pad("2", "smd", "rect", 0.0, -4.0, 1.0, 1.0, "\"F.Cu\"", "B")]),
    ];
    let (out, _) = route(&board(2, &fps), "");
    assert_eq!(out.stats.unrouted, 0);
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    assert!(out.drc.unconnected.is_empty(), "{:?}", out.drc.unconnected);
    // the two nets must cross: on one layer only through a via pair (or a detour around a pad)
    assert!(!out.geo.tracks.is_empty());
}

#[test]
fn through_hole_row_and_arcs() {
    // a row of THT pins with a net that has to pass between them
    let mut pins = Vec::new();
    for i in 0..6 {
        let net = if i == 0 { "S" } else { "N" };
        pins.push(pad(&format!("{}", i + 1), "thru_hole", "circle", (i as f64) * 2.54, 0.0, 1.6, 1.6, "\"*.Cu\"", net));
    }
    let fps = vec![
        footprint("J1", 8.0, 10.0, &pins),
        footprint("R1", 26.0, 3.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.8, 0.8, "\"F.Cu\"", "S"), pad("2", "smd", "rect", 1.6, 0.0, 0.8, 0.8, "\"F.Cu\"", "N")]),
        footprint("R2", 4.0, 17.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.8, 0.8, "\"F.Cu\"", "S")]),
    ];
    let (out, _) = route(&board(2, &fps), "");
    assert_eq!(out.stats.unrouted, 0);
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    assert!(out.drc.unconnected.is_empty(), "{:?}", out.drc.unconnected);
}

#[test]
fn length_matching_adds_meanders() {
    let fps = vec![
        footprint("U1", 3.0, 10.0, &[pad("1", "smd", "rect", 0.0, -3.0, 0.6, 0.6, "\"F.Cu\"", "D0"), pad("2", "smd", "rect", 0.0, 3.0, 0.6, 0.6, "\"F.Cu\"", "D1")]),
        footprint("U2", 27.0, 10.0, &[pad("1", "smd", "rect", 0.0, -3.0, 0.6, 0.6, "\"F.Cu\"", "D0"), pad("2", "smd", "rect", 0.0, 3.0, 0.6, 0.6, "\"F.Cu\"", "D1")]),
    ];
    let cfg = "[[length_groups]]\nmatch = [\"D*\"]\ntarget = 30.0\ntolerance = 0.2\n";
    let (out, b) = route(&board(2, &fps), cfg);
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    for n in ["D0", "D1"] {
        let id = b.net_id(n).unwrap();
        let len = gridless::tuning::net_length(&out.geo, id);
        assert!((len - 30.0).abs() < 0.25, "{n}: {len}");
    }
}

/// Polyline points of all tracks of `net` within |y - 10| < 1.5 (around the pin row).
fn near_row(out: &gridless::pipeline::Outcome, b: &gridless::board::Board, net: &str) -> Vec<(gridless::geom::P, gridless::geom::P)> {
    let id = b.net_id(net).unwrap();
    let mut segs = Vec::new();
    for t in out.geo.tracks.iter().filter(|t| t.net == id) {
        for pr in &t.prims {
            let pl = pr.polyline(0.001);
            for s in pl.windows(2) {
                if s[0].y.min(s[1].y) < 11.5 && s[0].y.max(s[1].y) > 8.5 {
                    segs.push((s[0], s[1]));
                }
            }
        }
    }
    assert!(!segs.is_empty(), "{net} does not pass the pin row");
    segs
}

/// Copper gap between a track (width w) and a round pin (centre c, radius r).
fn gap_to_pin(segs: &[(gridless::geom::P, gridless::geom::P)], c: gridless::geom::P, r: f64, w: f64) -> f64 {
    segs.iter().map(|&(a, b)| gridless::geom::point_seg_dist(c, a, b)).fold(f64::INFINITY, f64::min) - r - w / 2.0
}

fn gap_between(s1: &[(gridless::geom::P, gridless::geom::P)], s2: &[(gridless::geom::P, gridless::geom::P)], w: f64) -> f64 {
    let mut d = f64::INFINITY;
    for &(a, b) in s1 {
        for &(c, e) in s2 {
            d = d.min(gridless::geom::seg_seg_dist(a, b, c, e));
        }
    }
    d - w
}

/// A wall of touching THT pins (net W, not routed) across the board with one gap of
/// `gap` mm copper-to-copper between the pins at x = 15 -/+ (gap/2 + 1.1); SMD pads of
/// `nets` above and below it at the given x positions.
fn pin_row_board(gap: f64, nets: &[(&str, f64, f64)]) -> String {
    let r = 1.1;
    let half = gap / 2.0 + r;
    let mut xs = Vec::new();
    let mut x = 15.0 - half;
    while x > 1.7 {
        xs.push(x);
        x -= 2.0 * r;
    }
    xs.push(1.6); // no room for a track at the board edge
    let mut x = 15.0 + half;
    while x < 28.3 {
        xs.push(x);
        x += 2.0 * r;
    }
    xs.push(28.4);
    let pins: Vec<String> = xs.iter().enumerate().map(|(i, &x)| pad(&format!("{}", i + 1), "thru_hole", "circle", x, 0.0, 2.0 * r, 2.0 * r, "\"*.Cu\"", "W")).collect();
    let mut fps = vec![footprint("J1", 0.0, 10.0, &pins)];
    for (k, &(net, xa, xb)) in nets.iter().enumerate() {
        fps.push(footprint(&format!("A{k}"), xa, 3.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.6, 0.6, "\"F.Cu\"", net)]));
        fps.push(footprint(&format!("B{k}"), xb, 17.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.6, 0.6, "\"F.Cu\"", net)]));
    }
    board(2, &fps)
}

/// one routing layer (no way around the gap); the wall net is not routed
const TOP_ONLY: &str = "[layers.\"B.Cu\"]\nrole = \"off\"\n[[nets]]\nmatch = [\"W\"]\nskip = true\n";

#[test]
fn track_centred_between_close_pins() {
    // the straight line would pass the row at x = 14.5, hugging the left pin
    let gap = 1.2;
    let src = pin_row_board(gap, &[("S", 13.0, 16.0)]);
    let (pa, pb) = (gridless::geom::p(15.0 - gap / 2.0 - 1.1, 10.0), gridless::geom::p(15.0 + gap / 2.0 + 1.1, 10.0));
    let w = 0.25; // test board default class
    let (out, b) = route(&src, TOP_ONLY);
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    let s = near_row(&out, &b, "S");
    let (da, db) = (gap_to_pin(&s, pa, 1.1, w), gap_to_pin(&s, pb, 1.1, w));
    eprintln!("centred: {da:.4} / {db:.4}");
    assert!((da - db).abs() < 0.01, "not centred: {da:.4} / {db:.4}");
    // without equalisation the band hugs a pin
    let (out, b) = route(&src, &format!("{TOP_ONLY}[centering]\nenabled = false\n"));
    let s = near_row(&out, &b, "S");
    let (da, db) = (gap_to_pin(&s, pa, 1.1, w), gap_to_pin(&s, pb, 1.1, w));
    eprintln!("tight: {da:.4} / {db:.4}");
    assert!((da - db).abs() > 0.1, "tight band should hug: {da:.4} / {db:.4}");
}

#[test]
fn radar_ignores_far_pins() {
    // 6 mm between the pins: beyond the radar, the band stays tight against one pin
    let gap = 6.0;
    let src = pin_row_board(gap, &[("S", 11.0, 13.5)]);
    let pa = gridless::geom::p(15.0 - gap / 2.0 - 1.1, 10.0);
    let (on, b) = route(&src, TOP_ONLY);
    let (off, _) = route(&src, &format!("{TOP_ONLY}[centering]\nenabled = false\n"));
    let d_on = gap_to_pin(&near_row(&on, &b, "S"), pa, 1.1, 0.25);
    let d_off = gap_to_pin(&near_row(&off, &b, "S"), pa, 1.1, 0.25);
    assert!((d_on - d_off).abs() < 0.005, "far pins moved the track: {d_on:.4} vs {d_off:.4}");
}

#[test]
fn equal_gaps_respect_clearance_rules() {
    // three tracks through one gap; S2 needs 0.25 mm, the others 0.2 mm. Equal shares of
    // the room would be about 0.2375 mm: S2 keeps its 0.25 on both sides, the outer gaps
    // share the rest (about 0.22 each).
    let gap = 1.70;
    let src = pin_row_board(gap, &[("S1", 13.5, 13.5), ("S2", 15.0, 15.0), ("S3", 16.5, 16.5)]);
    let cfg = format!("{TOP_ONLY}[centering]\nfull = 3.0\nrange = 6.0\n[[nets]]\nmatch = [\"S2\"]\nclearance = 0.25\n");
    let (out, b) = route(&src, &cfg);
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    let (pa, pb) = (gridless::geom::p(15.0 - gap / 2.0 - 1.1, 10.0), gridless::geom::p(15.0 + gap / 2.0 + 1.1, 10.0));
    let (s1, s2, s3) = (near_row(&out, &b, "S1"), near_row(&out, &b, "S2"), near_row(&out, &b, "S3"));
    let w = 0.25;
    let g = [gap_to_pin(&s1, pa, 1.1, w), gap_between(&s1, &s2, w), gap_between(&s2, &s3, w), gap_to_pin(&s3, pb, 1.1, w)];
    eprintln!("gaps pin-S1, S1-S2, S2-S3, S3-pin: {g:.4?}");
    let rule = [0.2, 0.25, 0.25, 0.2];
    for i in 0..4 {
        assert!(g[i] >= rule[i] - 1e-3, "gap {i} = {:.4} below its rule {} ({g:?})", g[i], rule[i]);
    }
    assert!((g[0] - g[3]).abs() < 0.01 && (g[1] - g[2]).abs() < 0.01, "gaps not even: {g:?}");
    assert!((g[1] - 0.25).abs() < 0.01, "S2 should keep exactly its rule: {g:?}");
    // the router keeps GEOM_MARGIN on top of every rule
    let m = gridless::geometry::GEOM_MARGIN;
    let outer = (gap - 3.0 * w - 2.0 * (0.25 + m)) / 2.0;
    assert!((g[0] - outer).abs() < 0.003, "outer gaps should share the rest ({outer:.4}): {g:?}");
}

#[test]
fn drc_report_is_deterministic() {
    // a grid of crossing tracks of two nets: many violations with equal gaps; every DRC
    // run hashes with a new random seed, the report must still come out the same
    use gridless::geometry::{Geometry, Prim, Track};
    let fps = vec![
        footprint("U1", 2.0, 2.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.6, 0.6, "\"F.Cu\"", "A")]),
        footprint("U2", 28.0, 18.0, &[pad("1", "smd", "rect", 0.0, 0.0, 0.6, 0.6, "\"F.Cu\"", "B")]),
    ];
    let src = board(2, &fps);
    let dir = std::env::temp_dir().join(format!("gridless_det_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.kicad_pcb");
    std::fs::write(&path, src).unwrap();
    let l = gridless::kicad::load(&path).unwrap();
    let rules = Rules::resolve(&l.board, &Config::default());
    let pb = Problem::build(&l.board, &rules);
    let (a, b) = (l.board.net_id("A").unwrap(), l.board.net_id("B").unwrap());
    let p = gridless::geom::p;
    let mut tracks = Vec::new();
    for i in 0..8 {
        let x = 5.0 + 2.5 * i as f64;
        let y = 3.0 + 2.0 * i as f64;
        tracks.push(Track { layer: 0, net: a, width: 0.25, conn: 0, prims: vec![Prim::Seg(p(x, 1.0), p(x, 19.0))], pw: vec![0.25] });
        tracks.push(Track { layer: 0, net: b, width: 0.25, conn: 1, prims: vec![Prim::Seg(p(1.0, y), p(29.0, y))], pw: vec![0.25] });
    }
    let geo = Geometry { layers: Vec::new(), tracks, vias: Vec::new(), warnings: Vec::new() };
    let key = |r: &gridless::drc::Report| r.violations.iter().map(|v| format!("{} {:.6} {:.6} {} {}", v.kind, v.at.x, v.at.y, v.a, v.b)).collect::<Vec<_>>();
    let first = key(&gridless::drc::check(&l.board, &pb, &rules, &geo));
    assert!(first.len() >= 64, "expected a violation per crossing, got {}", first.len());
    for _ in 0..5 {
        assert_eq!(key(&gridless::drc::check(&l.board, &pb, &rules, &geo)), first);
    }
}

#[test]
fn smoothing_straightens_entries_and_rounds_kinks() {
    use gridless::geometry::{Geometry, Prim, Track};
    // net A: pad at (10,10); its track leaves round the pad's corner (a hook) and then
    // bends sharply in open space. A far pad of net B must not be touched.
    let fps = vec![
        footprint("U1", 10.0, 10.0, &[pad("1", "smd", "rect", 0.0, 0.0, 1.0, 1.0, "\"F.Cu\"", "A")]),
        footprint("U2", 25.0, 10.0, &[pad("1", "smd", "rect", 0.0, 0.0, 1.0, 1.0, "\"F.Cu\"", "B")]),
    ];
    let src = board(2, &fps);
    let dir = std::env::temp_dir().join(format!("gridless_smooth_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("t.kicad_pcb");
    std::fs::write(&path, src).unwrap();
    let l = gridless::kicad::load(&path).unwrap();
    let rules = Rules::resolve(&l.board, &Config::default());
    let pb = Problem::build(&l.board, &rules);
    let a = l.board.net_id("A").unwrap();
    let p = gridless::geom::p;
    let pts = [p(10.0, 10.0), p(9.4, 9.4), p(12.0, 6.0), p(15.0, 6.0), p(17.0, 3.0)];
    let prims: Vec<Prim> = pts.windows(2).map(|w| Prim::Seg(w[0], w[1])).collect();
    let pw = vec![0.25; prims.len()];
    let len0: f64 = prims.iter().map(|q| q.len()).sum();
    let mut geo = Geometry { layers: Vec::new(), tracks: vec![Track { layer: 0, net: a, width: 0.25, conn: 0, prims, pw }], vias: Vec::new(), warnings: Vec::new() };
    let st = gridless::smooth::smooth(&l.board, &pb, &rules, &mut geo);
    let t = &geo.tracks[0];
    assert!(st.entries >= 1, "{st:?}");
    assert!(st.corners >= 1, "{st:?}");
    // the hook round the pad corner is gone: the track leaves the pad straight
    let first = t.prims[0];
    assert!(first.start().dist(p(10.0, 10.0)) < 1e-9);
    assert!(gridless::geom::point_seg_dist(p(9.4, 9.4), first.start(), first.end()) > 0.3, "{:?}", t.prims);
    assert!(t.prims.iter().any(|q| matches!(q, Prim::Arc(_))), "no rounded corner: {:?}", t.prims);
    let len1: f64 = t.prims.iter().map(|q| q.len()).sum();
    assert!(len1 < len0 - 0.5, "{len0} -> {len1}");
    // still one continuous track from the pad to the far end
    for w in t.prims.windows(2) {
        assert!(w[0].end().dist(w[1].start()) < 1e-6);
    }
    assert!(t.prims.last().unwrap().end().dist(p(17.0, 3.0)) < 1e-9);
    let rep = gridless::drc::check(&l.board, &pb, &rules, &geo);
    assert!(rep.violations.is_empty(), "{:?}", rep.violations);
}

/// A channel between two walls of touching pins (net W, not routed) with nets N1, D0, N2
/// running through it on a 0.5 mm pitch from x = 2 to x = 28.
fn channel_board() -> String {
    let r = 1.1;
    let mut pins = Vec::new();
    let mut i = 0;
    for &y in &[6.5, 13.5] {
        let mut x = 1.6;
        while x < 28.5 {
            i += 1;
            pins.push(pad(&format!("{i}"), "thru_hole", "circle", x, y - 10.0, 2.0 * r, 2.0 * r, "\"*.Cu\"", "W"));
            x += 2.0 * r;
        }
    }
    let mut fps = vec![footprint("J1", 0.0, 10.0, &pins)];
    for (k, (net, y)) in [("N1", 9.5), ("D0", 10.0), ("N2", 10.5)].iter().enumerate() {
        fps.push(footprint(&format!("A{k}"), 2.5, *y, &[pad("1", "smd", "rect", 0.0, 0.0, 0.3, 0.3, "\"F.Cu\"", net)]));
        fps.push(footprint(&format!("B{k}"), 27.5, *y, &[pad("1", "smd", "rect", 0.0, 0.0, 0.3, 0.3, "\"F.Cu\"", net)]));
    }
    board(2, &fps)
}

#[test]
fn length_reserve_makes_room_for_meanders() {
    // D0 must grow from 25 to 33 mm, but its neighbours run 0.25 mm away: without a reserve
    // there is no room for meanders; with it the topology keeps a corridor around D0
    let src = channel_board();
    let base = format!("{TOP_ONLY}[[length_groups]]\nmatch = [\"D0\"]\ntarget = 33.0\ntolerance = 0.3\n");
    let res = |cfg: &str| {
        let (out, b) = route(&src, cfg);
        assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
        assert!(out.drc.unconnected.is_empty(), "{:?}", out.drc.unconnected);
        let len = gridless::tuning::net_length(&out.geo, b.net_id("D0").unwrap());
        eprintln!("D0 {len:.3} mm, ok {:?}", out.tuning.iter().map(|t| t.ok).collect::<Vec<_>>());
        (len, out.tuning.iter().all(|t| t.ok))
    };
    let (len_off, ok_off) = res(&format!("length_reserve = \"off\"\n{base}"));
    assert!(!ok_off, "the test board should be too tight without a reserve ({len_off})");
    for mode in ["always", "auto"] {
        let (len, ok) = res(&format!("length_reserve = \"{mode}\"\n{base}"));
        assert!(ok && (len - 33.0).abs() <= 0.3, "{mode}: {len}");
    }
}

#[test]
fn pair_vias_side_by_side() {
    // a wall of F.Cu pads across the board: the USB pair has to change layer, and its two
    // vias must stand side by side, square to the pair (here: same x)
    let wall: Vec<String> = (0..20).map(|k| pad(&format!("{}", k + 1), "smd", "rect", 0.0, k as f64 + 0.5, 1.0, 1.0, "\"F.Cu\"", "W")).collect();
    let ends = |r: &str, x: f64| footprint(r, x, 10.0, &[pad("1", "smd", "rect", 0.0, -0.4, 0.4, 0.4, "\"F.Cu\"", "USB_P"), pad("2", "smd", "rect", 0.0, 0.4, 0.4, 0.4, "\"F.Cu\"", "USB_N")]);
    let fps = vec![footprint("J1", 15.0, 0.0, &wall), ends("U1", 4.0), ends("U2", 26.0)];
    let (out, b) = route(&board(2, &fps), "[[nets]]\nmatch = [\"W\"]\nskip = true\n");
    assert!(out.drc.violations.is_empty(), "{:?}", out.drc.violations);
    assert!(out.drc.unconnected.is_empty(), "{:?}", out.drc.unconnected);
    let (pn, nn) = (b.net_id("USB_P").unwrap(), b.net_id("USB_N").unwrap());
    let pv: Vec<_> = out.geo.vias.iter().filter(|v| v.1 == pn).map(|v| v.0).collect();
    let nv: Vec<_> = out.geo.vias.iter().filter(|v| v.1 == nn).map(|v| v.0).collect();
    assert!(!pv.is_empty() && pv.len() == nv.len(), "{pv:?} {nv:?}");
    for p in &pv {
        let n = nv.iter().min_by(|a, b| a.dist(*p).partial_cmp(&b.dist(*p)).unwrap()).unwrap();
        assert!((n.x - p.x).abs() < 0.05, "pair vias not side by side: {p:?} {n:?}");
        assert!(n.dist(*p) < 1.0, "pair vias too far apart: {p:?} {n:?}");
    }
}
