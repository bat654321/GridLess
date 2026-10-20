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
