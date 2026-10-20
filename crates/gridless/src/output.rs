//! Conversion of the routed geometry into KiCad board items.

use crate::board::Board;
use crate::config::Rules;
use crate::geometry::{Geometry, Prim};
use crate::kicad::OutItem;

pub fn items(b: &Board, g: &Geometry, rules: &Rules) -> Vec<OutItem> {
    let mut out = Vec::new();
    for t in &g.tracks {
        let layer = b.layers[t.layer].name.clone();
        let net = b.nets[t.net].name.clone();
        for (i, pr) in t.prims.iter().enumerate() {
            let width = t.pw.get(i).copied().unwrap_or(t.width);
            match pr {
                Prim::Seg(a, c) => {
                    if a.dist(*c) > 1e-6 {
                        out.push(OutItem::Seg { layer: layer.clone(), net: net.clone(), width, a: *a, b: *c });
                    }
                }
                Prim::Arc(a) => {
                    if a.len() > 1e-6 {
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
