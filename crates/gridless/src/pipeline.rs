//! End-to-end routing: topology -> geometry -> DRC, with DRC-driven re-routing.

use crate::board::Board;
use crate::config::Rules;
use crate::drc::{self, Report};
use crate::geometry::{realize, Geometry};
use crate::problem::Problem;
use crate::topo::{RouteStats, Router};

pub struct Outcome {
    pub tuning: Vec<crate::tuning::TuneResult>,
    pub geo: Geometry,
    pub drc: Report,
    pub stats: RouteStats,
    pub rounds: usize,
    pub length: f64,
}

/// Lower is better: (weighted defects, unconnected, vias, length).
fn score(rep: &Report, geo: &Geometry, unrouted: usize) -> (usize, usize, usize, i64) {
    let len: f64 = geo.tracks.iter().map(|t| t.prims.iter().map(|p| p.len()).sum::<f64>()).sum();
    let unconnected = unrouted + rep.unconnected.len();
    (3 * unconnected + rep.violations.len(), unconnected, geo.vias.len(), (len * 10.0) as i64)
}

/// Route a board. `rounds` = number of geometry/DRC feedback rounds.
pub fn route(board: &Board, rules: &Rules, pb: &Problem, rounds: usize, progress: &mut dyn FnMut(&str)) -> Outcome {
    let mut r = Router::new(board, pb, rules);
    progress(&format!("meshes: {:?} triangles", r.meshes.iter().map(|m| m.tris.len()).collect::<Vec<_>>()));
    r.run();
    let removed = r.minimise_vias();
    r.protect_pairs = true;
    let mut better = r.improve(3);
    better += r.improve_detours(12);
    better += r.improve(2);
    progress(&format!(
        "topology: {} unrouted, {} vias ({} removed by via minimisation), {} connections improved by clean-up, overflow {:.3} mm",
        r.stats.unrouted, r.stats.vias, removed, better, r.stats.overflow
    ));
    let mut best: Option<((usize, usize, usize, i64), Geometry, Report, RouteStats)> = None;
    let mut done_rounds = 0;
    // violation spots seen in earlier rounds (layer, point, times)
    let mut seen: Vec<(Option<usize>, crate::geom::P, u32)> = Vec::new();
    let mut best_state: Option<crate::topo::State> = None;
    // the feedback rounds work on the plain rubber band; gap equalisation is a final
    // post-processing step on the best topology
    let tight = Rules { centering: None, ..rules.clone() };
    for round in 0..rounds.max(1) {
        done_rounds = round + 1;
        let geo = realize(&r, pb, &tight);
        let rep = drc::check(board, pb, rules, &geo);
        let unrouted = r.st.conns.iter().filter(|c| !c.routed).count();
        let sc = score(&rep, &geo, unrouted);
        progress(&format!("round {round}: {} violations, {} unconnected, {} vias", rep.violations.len(), sc.1, geo.vias.len()));
        let clean = sc.0 == 0;
        let mut feedback: Vec<(Option<usize>, crate::geom::P, Option<usize>)> = Vec::new();
        let is_pair = |c: usize| pb.conns[c].couple.is_some();
        for v in &rep.violations {
            // differential pairs are protected: move the other party when there is one
            let c1 = v.track.map(|(ti, _)| geo.tracks[ti].conn as usize);
            let c2 = v.track2.map(|(ti, _)| geo.tracks[ti].conn as usize);
            let conn = match (c1, c2) {
                (Some(a), Some(b)) if is_pair(a) && !is_pair(b) => Some(b),
                // a pair wire alone, barely too close: penalise the spot, keep the pair
                (Some(a), None) if is_pair(a) && v.need - v.gap < 0.02 => None,
                (Some(a), _) => Some(a),
                (None, b) => b,
            };
            let layer = v.layer.or(v.track.map(|(ti, _)| geo.tracks[ti].layer));
            feedback.push((layer, v.at, conn));
        }
        let improved = best.as_ref().map_or(true, |b| sc < b.0);
        if improved {
            best = Some((sc, geo, rep, r.stats.clone()));
            best_state = Some(r.st.clone());
        } else if let Some(bs) = &best_state {
            // this round made things worse: continue from the best topology, keeping
            // the accumulated history penalties
            r.st = bs.clone();
        }
        if clean || round + 1 == rounds {
            break;
        }
        // feedback: make the trouble spots expensive and re-route the offenders
        let mut to_rip: Vec<usize> = Vec::new();
        for (layer, at, conn) in feedback {
            // persistent trouble spot: escalate and move the neighbours too
            let times = match seen.iter_mut().find(|(l, q, _)| *l == layer && q.dist(at) < 1.0) {
                Some(x) => {
                    x.2 += 1;
                    x.2
                }
                None => {
                    seen.push((layer, at, 1));
                    1
                }
            };
            let amount = (6.0 + 4.0 * round as f32) * times.min(3) as f32;
            let near = match layer {
                Some(l) => r.penalise_near(l, at, 0.7 + 0.3 * times.min(3) as f64, amount),
                None => Vec::new(),
            };
            if let Some(c) = conn {
                if !to_rip.contains(&c) {
                    to_rip.push(c);
                }
            }
            if times >= 2 || conn.is_none() {
                for &c in near.iter().filter(|&&c| !pb.conns[c].couple.is_some()).take(2 + times.min(2) as usize) {
                    if !to_rip.contains(&c) {
                        to_rip.push(c);
                    }
                }
            }
        }
        r.reroute(&to_rip, 10);
        r.minimise_vias();
        r.improve(1);
    }
    let (best_sc, mut geo, mut drc, stats) = best.unwrap();
    // gap equalisation on the best geometry; it must not cost a rule: on a layer where it
    // adds violations the tight tracks are kept
    if rules.centering.is_some() {
        let tight_tracks = geo.tracks.clone();
        crate::geometry::equalise(&r, pb, rules, &mut geo);
        let mut crep = drc::check(board, pb, rules, &geo);
        let per_layer = |rep: &Report, tracks: &[crate::geometry::Track]| {
            let mut n = vec![0usize; board.layers.len()];
            for v in &rep.violations {
                if let Some(l) = v.layer.or(v.track.map(|(ti, _)| tracks[ti].layer)) {
                    n[l] += 1;
                }
            }
            n
        };
        let (before, after) = (per_layer(&drc, &tight_tracks), per_layer(&crep, &geo.tracks));
        let worse: Vec<usize> = (0..board.layers.len()).filter(|&l| after[l] > before[l]).collect();
        if !worse.is_empty() {
            let mut merged: Vec<crate::geometry::Track> = geo.tracks.drain(..).filter(|t| !worse.contains(&t.layer)).collect();
            merged.extend(tight_tracks.iter().filter(|t| worse.contains(&t.layer)).cloned());
            geo.tracks = merged;
            crep = drc::check(board, pb, rules, &geo);
            let names: Vec<&str> = worse.iter().map(|&l| board.layers[l].name.as_str()).collect();
            progress(&format!("gap equalisation kept off {:?}: it added violations there", names));
        }
        let unrouted = r.st.conns.iter().filter(|c| !c.routed).count();
        let sc = score(&crep, &geo, unrouted);
        if (sc.0, sc.1) <= (best_sc.0, best_sc.1) {
            let radar = rules.centering.unwrap();
            progress(&format!(
                "gap equalisation (radar: equal gaps up to {:.2} mm between pads, none from {:.2} mm): {} violations, length {:.1} -> {:.1} mm",
                radar.full,
                radar.range,
                crep.violations.len(),
                best_sc.3 as f64 / 10.0,
                sc.3 as f64 / 10.0
            ));
            drc = crep;
        } else {
            progress(&format!("gap equalisation dropped: {} violations instead of {}", crep.violations.len(), drc.violations.len()));
            geo.tracks = tight_tracks;
        }
    }
    // length matching on the final geometry
    let tuning = if !rules.length_groups.is_empty() || !rules.diff_pairs.is_empty() {
        let t = crate::tuning::tune(board, pb, rules, &mut geo);
        for x in &t {
            progress(&format!("length {}: {:.3} -> {:.3} (target {:.3}) {}", x.net, x.before, x.after, x.target, if x.ok { "ok" } else { "NOT MET" }));
        }
        drc = drc::check(board, pb, rules, &geo);
        t
    } else {
        Vec::new()
    };
    let length = geo.tracks.iter().map(|t| t.prims.iter().map(|p| p.len()).sum::<f64>()).sum();
    Outcome { tuning, geo, drc, stats, rounds: done_rounds, length }
}

/// Locate kicad-cli: $KICAD_CLI, PATH, or the default KiCad 10/9 install folders.
pub fn find_kicad_cli() -> Option<std::path::PathBuf> {
    if let Ok(p) = std::env::var("KICAD_CLI") {
        let p = std::path::PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    let exe = if cfg!(windows) { "kicad-cli.exe" } else { "kicad-cli" };
    if let Ok(path) = std::env::var("PATH") {
        for d in std::env::split_paths(&path) {
            let c = d.join(exe);
            if c.is_file() {
                return Some(c);
            }
        }
    }
    for v in ["10.0", "9.0"] {
        for base in [r"C:\Program Files\KiCad", "/usr/bin", "/Applications/KiCad/KiCad.app/Contents/MacOS"] {
            let c = std::path::Path::new(base).join(v).join("bin").join(exe);
            if c.is_file() {
                return Some(c);
            }
        }
    }
    None
}

/// Run KiCad's DRC (with zone refill) on a routed board and return the pads of plane
/// nets that ended up unconnected, as "REF.PAD" labels.
pub fn kicad_unconnected_plane_pads(cli: &std::path::Path, routed: &std::path::Path, project: Option<&std::path::Path>, board: &Board, rules: &Rules) -> anyhow::Result<Vec<String>> {
    let dir = std::env::temp_dir().join(format!("gridless_drc_{}", std::process::id()));
    std::fs::create_dir_all(&dir)?;
    let pcb = dir.join("check.kicad_pcb");
    std::fs::copy(routed, &pcb)?;
    if let Some(p) = project {
        let _ = std::fs::copy(p, dir.join("check.kicad_pro"));
    }
    let json = dir.join("drc.json");
    let st = std::process::Command::new(cli)
        .args(["pcb", "drc", "--format", "json", "--units", "mm", "--severity-all", "--refill-zones", "-o"])
        .arg(&json)
        .arg(&pcb)
        .output()?;
    if !json.is_file() {
        anyhow::bail!("kicad-cli drc failed: {}", String::from_utf8_lossy(&st.stderr));
    }
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&json)?)?;
    let mut out = Vec::new();
    for u in v["unconnected_items"].as_array().cloned().unwrap_or_default() {
        for it in u["items"].as_array().cloned().unwrap_or_default() {
            let (x, y) = (it["pos"]["x"].as_f64().unwrap_or(f64::NAN), it["pos"]["y"].as_f64().unwrap_or(f64::NAN));
            let q = crate::geom::p(x, y);
            for (i, pd) in board.pads.iter().enumerate() {
                if pd.pos.dist(q) < 0.01 && pd.kind == crate::board::PadKind::Smd {
                    if let Some(n) = pd.net {
                        if rules.nets[n].plane.is_some() {
                            let l = board.pad_label(i);
                            if !out.contains(&l) {
                                out.push(l);
                            }
                        }
                    }
                }
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
    Ok(out)
}
