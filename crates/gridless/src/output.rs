//! Conversion of the routed geometry into KiCad board items.

use crate::board::Board;
use crate::config::Rules;
use crate::geom::Arc;
use crate::geometry::{Geometry, Prim};
use crate::kicad::OutItem;

pub fn items(b: &Board, g: &Geometry, rules: &Rules) -> Vec<OutItem> {
    let mut out = Vec::new();
    for t in &g.tracks {
        let layer = b.layers[t.layer].name.clone();
        let net = b.nets[t.net].name.clone();
        let mut prims: Vec<(Prim, f64)> = t.prims.iter().enumerate().map(|(i, &p)| (p, t.pw.get(i).copied().unwrap_or(t.width))).collect();
        prims = merge_slivers(prims);
        // KiCad does not count an arc that ends right at a pad or via as connected: a track
        // that starts or ends with an arc gets its last 20 um as a straight segment
        // (deviation well under a micrometre)
        const TIP: f64 = 0.02;
        if let Some(&(Prim::Arc(a), w)) = prims.first() {
            if a.len() > 2.0 * TIP {
                let f = TIP / a.len();
                let q = a.at(f);
                prims[0] = (Prim::Arc(Arc { c: a.c, r: a.r, a0: a.a0 + a.sweep * f, sweep: a.sweep * (1.0 - f) }), w);
                prims.insert(0, (Prim::Seg(a.start(), q), w));
            }
        }
        if let Some(&(Prim::Arc(a), w)) = prims.last() {
            if a.len() > 2.0 * TIP {
                let f = 1.0 - TIP / a.len();
                let q = a.at(f);
                let n = prims.len();
                prims[n - 1] = (Prim::Arc(Arc { c: a.c, r: a.r, a0: a.a0, sweep: a.sweep * f }), w);
                prims.push((Prim::Seg(q, a.end()), w));
            }
        }
        for (pr, width) in prims.iter().map(|(p, w)| (p, *w)) {
            match pr {
                Prim::Seg(a, c) => {
                    if a.dist(*c) > 1e-6 {
                        out.push(OutItem::Seg { layer: layer.clone(), net: net.clone(), width, a: *a, b: *c });
                    }
                }
                Prim::Arc(a) => {
                    // an arc that is practically straight (sagitta below 1 um) goes out as a
                    // segment: KiCad's connectivity misjudges such slivers as dangling, and
                    // the deviation stays well inside GEOM_MARGIN
                    let sagitta = a.r * (1.0 - (a.sweep.abs() / 2.0).cos());
                    if sagitta < 1e-3 {
                        if a.start().dist(a.end()) > 1e-6 {
                            out.push(OutItem::Seg { layer: layer.clone(), net: net.clone(), width, a: a.start(), b: a.end() });
                        }
                    } else if a.len() > 1e-6 {
                        out.push(OutItem::Arc { layer: layer.clone(), net: net.clone(), width, s: a.start(), m: a.mid(), e: a.end() });
                    }
                }
            }
        }
    }
    for &(pos, n) in &g.vias {
        out.push(OutItem::Via { net: b.nets[n].name.clone(), pos, dia: rules.via_dia, drill: rules.via_drill });
    }
    out
}

/// Fold slivers (pieces under 20 um left by smoothing and rounding) into a straight
/// neighbour, keeping the chain of end points continuous. A board full of
/// micrometre segments is hard to edit by hand.
fn merge_slivers(prims: Vec<(Prim, f64)>) -> Vec<(Prim, f64)> {
    const MIN: f64 = 0.02;
    let len = |p: &Prim| match p {
        Prim::Seg(a, b) => a.dist(*b),
        Prim::Arc(a) => a.len(),
    };
    let ends = |p: &Prim| match p {
        Prim::Seg(a, b) => (*a, *b),
        Prim::Arc(a) => (a.start(), a.end()),
    };
    let mut src: std::collections::VecDeque<(Prim, f64)> = prims.into();
    let mut out: Vec<(Prim, f64)> = Vec::new();
    while let Some((p, w)) = src.pop_front() {
        if len(&p) >= MIN || (out.is_empty() && src.is_empty()) {
            out.push((p, w));
            continue;
        }
        let (s, e) = ends(&p);
        if let Some((Prim::Seg(a, _), w2)) = src.front().copied() {
            // the next straight piece starts where the sliver started
            let (_, b) = ends(&src.front().unwrap().0);
            let _ = a;
            src[0] = (Prim::Seg(s, b), w2);
        } else if let Some((Prim::Seg(a, _), w2)) = out.last().copied() {
            let n = out.len();
            out[n - 1] = (Prim::Seg(a, e), w2);
        } else {
            out.push((p, w));
        }
    }
    out
}

/// JSON report for plugins: items to create plus statistics.
#[derive(serde::Serialize)]
pub struct JsonResult {
    pub items: Vec<OutItem>,
    pub vias: usize,
    pub length_mm: f64,
    pub violations: Vec<crate::drc::Violation>,
    pub unconnected: Vec<String>,
    pub tuning: Vec<crate::tuning::TuneResult>,
    pub unrouted: usize,
    pub seconds: f64,
}
