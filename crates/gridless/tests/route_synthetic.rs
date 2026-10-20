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
