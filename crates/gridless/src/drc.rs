//! Independent design-rule check of the routed result (clearance, board edge,
//! connectivity), used to validate and to drive local repairs.

use crate::board::{Board, LayerId, NetId, PadKind};
use crate::config::Rules;
use crate::geom::{BBox, RoundPoly, P};
use crate::geometry::{Geometry, Prim};
use crate::problem::{dist_to_outline, Problem};
use std::collections::HashMap;

#[derive(Clone, Debug, serde::Serialize)]
pub struct Violation {
    pub kind: String,
    pub layer: Option<LayerId>,
    pub at: P,
    pub gap: f64,
    pub need: f64,
    pub a: String,
    pub b: String,
    /// (track index, primitive index) of a routed item involved, if any
    #[serde(skip)]
    pub track: Option<(usize, usize)>,
    /// the second routed item, for track-track violations
    #[serde(skip)]
    pub track2: Option<(usize, usize)>,
}

#[derive(Clone, Debug)]
struct Item {
    shape: RoundPoly,
    net: Option<NetId>,
    layers: u64,
    desc: String,
    track: Option<(usize, usize)>,
}

pub struct Report {
    pub violations: Vec<Violation>,
    pub unconnected: Vec<String>,
}

fn closest_pair(a: &RoundPoly, b: &RoundPoly) -> P {
    // approximate location: midpoint between centroids clipped is good enough for reporting
    let ca = a.centroid();
    let cb = b.centroid();
    let (qa, _) = a.core_closest(cb);
    let (qb, _) = b.core_closest(ca);
    qa.mid(qb)
}

pub fn check(b: &Board, pb: &Problem, rules: &Rules, g: &Geometry) -> Report {
    let all = (1u64 << b.layers.len()) - 1;
    let mut items: Vec<Item> = Vec::new();
    for (i, pd) in b.pads.iter().enumerate() {
        if pd.kind == PadKind::Npth && pd.layers.is_empty() {
            continue;
        }
        let m = pd.layers.iter().fold(0u64, |m, &l| m | (1 << l));
        items.push(Item { shape: pd.shape.clone(), net: pd.net, layers: m, desc: format!("pad {}", b.pad_label(i)), track: None });
    }
    for cs in &b.copper {
        for pc in &cs.pieces {
            items.push(Item { shape: pc.clone(), net: cs.net, layers: 1 << cs.layer, desc: format!("copper {}", cs.net.map(|n| b.nets[n].name.as_str()).unwrap_or("")), track: None });
        }
    }
    if rules.keep_existing {
        for o in &pb.obstacles {
            if matches!(o.kind, crate::problem::ObsKind::Track | crate::problem::ObsKind::Via) {
                items.push(Item { shape: o.shape.clone(), net: o.net, layers: o.layers, desc: "existing".into(), track: None });
            }
        }
    }
    for (ti, t) in g.tracks.iter().enumerate() {
        for (pi, pr) in t.prims.iter().enumerate() {
            let pts = pr.polyline(0.0002);
            let width = t.pw.get(pi).copied().unwrap_or(t.width);
            for w in pts.windows(2) {
                items.push(Item {
                    shape: RoundPoly::capsule(w[0], w[1], width / 2.0),
                    net: Some(t.net),
                    layers: 1 << t.layer,
                    desc: format!("track {} on {}", b.nets[t.net].name, b.layers[t.layer].name),
                    track: Some((ti, pi)),
                });
            }
        }
    }
    for &(pos, n) in &g.vias {
        items.push(Item { shape: RoundPoly::circle(pos, rules.via_dia / 2.0), net: Some(n), layers: all, desc: format!("via {}", b.nets[n].name), track: None });
    }
    // spatial hash
    let cell = 1.0;
    let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    let bbs: Vec<BBox> = items.iter().map(|it| it.shape.bbox()).collect();
    for (i, bb) in bbs.iter().enumerate() {
        let bb = bb.grow(0.5);
        for x in (bb.min.x / cell).floor() as i32..=(bb.max.x / cell).floor() as i32 {
            for y in (bb.min.y / cell).floor() as i32..=(bb.max.y / cell).floor() as i32 {
                grid.entry((x, y)).or_default().push(i);
            }
        }
    }
    // visit the cells in a fixed order: the report drives the re-routing, so it must not
    // depend on the (per-process random) hash order
    let mut cells: Vec<(i32, i32)> = grid.keys().copied().collect();
    cells.sort_unstable();
    let mut seen = std::collections::HashSet::new();
    let mut violations = Vec::new();
    let tol = 5e-4;
    for cellv in cells.iter().map(|k| &grid[k]) {
        for x in 0..cellv.len() {
            for y in x + 1..cellv.len() {
                let (i, j) = (cellv[x].min(cellv[y]), cellv[x].max(cellv[y]));
                let (a, c) = (&items[i], &items[j]);
                if a.layers & c.layers == 0 {
                    continue;
                }
                if a.track.is_none() && c.track.is_none() && !a.desc.starts_with("via") && !c.desc.starts_with("via") {
                    continue; // pad-pad: placement, not routing
                }
                if a.net.is_some() && a.net == c.net {
                    continue;
                }
                if !seen.insert((i, j)) {
                    continue;
                }
                let need = rules.clr(a.net, c.net);
                if !bbs[i].grow(need).overlaps(&bbs[j]) {
                    continue;
                }
                let d = a.shape.shape_dist(&c.shape);
                if d < need - tol {
                    let layer = (0..b.layers.len()).find(|&l| (a.layers & c.layers) & (1 << l) != 0);
                    violations.push(Violation { kind: "clearance".into(), layer, at: closest_pair(&a.shape, &c.shape), gap: d, need, a: a.desc.clone(), b: c.desc.clone(), track: a.track.or(c.track), track2: if a.track.is_some() { c.track } else { None } });
                }
            }
        }
    }
    // board edge
    for it in items.iter().filter(|it| it.track.is_some() || it.desc.starts_with("via")) {
        let mut d = f64::INFINITY;
        for poly in &pb.outline {
            let n = poly.len();
            for k in 0..n {
                d = d.min(it.shape.seg_dist(poly[k], poly[(k + 1) % n]));
            }
        }
        if d < rules.edge_clearance - tol {
            let c = it.shape.centroid();
            violations.push(Violation { kind: "edge_clearance".into(), layer: None, at: c, gap: d, need: rules.edge_clearance, a: it.desc.clone(), b: "board edge".into(), track: it.track, track2: None });
        }
        let _ = dist_to_outline;
    }
    // collapse duplicates (one per track primitive pair)
    // total order, so that the same duplicate survives every time
    violations.sort_by(|a, b| a.gap.total_cmp(&b.gap).then(a.at.x.total_cmp(&b.at.x)).then(a.at.y.total_cmp(&b.at.y)).then_with(|| a.a.cmp(&b.a)).then_with(|| a.b.cmp(&b.b)));
    let mut dedup: Vec<Violation> = Vec::new();
    for v in violations {
        if !dedup.iter().any(|o| o.a == v.a && o.b == v.b && o.at.dist(v.at) < 0.5) {
            dedup.push(v);
        }
    }
    // ---- connectivity (union-find over touching same-net copper)
    let n = items.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for cellv in cells.iter().map(|k| &grid[k]) {
        for x in 0..cellv.len() {
            for y in x + 1..cellv.len() {
                let (i, j) = (cellv[x], cellv[y]);
                let (a, c) = (&items[i], &items[j]);
                if a.net.is_none() || a.net != c.net || a.layers & c.layers == 0 {
                    continue;
                }
                if a.shape.shape_dist(&c.shape) <= 1e-4 {
                    let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                    parent[ri] = rj;
                }
            }
        }
    }
    // planes: everything of a plane net that reaches the plane layer is connected
    for &net in &pb.plane_nets {
        let pl = match rules.nets[net].plane {
            Some(l) => l,
            None => continue,
        };
        let members: Vec<usize> = (0..n).filter(|&i| items[i].net == Some(net) && items[i].layers & (1 << pl) != 0).collect();
        for w in members.windows(2) {
            let (ri, rj) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
            parent[ri] = rj;
        }
        // pads left to same-net pours are assumed connected by the pour
        if rules.trust_pours {
            let pours: Vec<usize> = (0..n)
                .filter(|&i| {
                    items[i].net == Some(net)
                        && items[i].desc.starts_with("pad")
                        && b.zones.iter().any(|z| z.keepout.is_none() && z.net == Some(net) && z.layers.iter().any(|&l| items[i].layers & (1 << l) != 0) && crate::geom::point_in_poly(items[i].shape.centroid(), &z.outline))
                })
                .collect();
            if let (Some(&f), Some(&m)) = (pours.first(), members.first()) {
                for &i in &pours {
                    let (ri, rj) = (find(&mut parent, i), find(&mut parent, f));
                    parent[ri] = rj;
                }
                let (ri, rj) = (find(&mut parent, f), find(&mut parent, m));
                parent[ri] = rj;
            }
        }
    }
    let mut unconnected = Vec::new();
    for (ni, net) in b.nets.iter().enumerate() {
        if rules.nets[ni].skip {
            continue;
        }
        let pads: Vec<usize> = (0..n).filter(|&i| items[i].net == Some(ni) && items[i].desc.starts_with("pad")).collect();
        if pads.len() < 2 {
            continue;
        }
        let mut groups: std::collections::BTreeMap<usize, Vec<String>> = std::collections::BTreeMap::new();
        for &i in &pads {
            let r = find(&mut parent, i);
            groups.entry(r).or_default().push(items[i].desc.trim_start_matches("pad ").to_string());
        }
        if groups.len() > 1 {
            let mut gs: Vec<Vec<String>> = groups.into_values().collect();
            gs.sort_by_key(|g| g.len());
            let small: Vec<String> = gs[..gs.len() - 1].iter().map(|g| g.join("+")).collect();
            unconnected.push(format!("{} ({} islands; isolated: {})", net.name, gs.len(), small.join(", ")));
        }
    }
    let _ = Prim::Seg;
    Report { violations: dedup, unconnected }
}
