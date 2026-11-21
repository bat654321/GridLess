//! Pour stitching: join the copper islands of a net's pours with vias.
//!
//! Pours are not part of the router's connectivity model: a filled zone is
//! copper that comes from KiCad's filler, and its islands change every time
//! tracks are added. So stitching runs as a post-process on a *refilled* board:
//! build the island graph (islands joined by pads, tracks and vias), pick the
//! main component and connect every other one to it with a via where two
//! layers' islands overlap.

use crate::board::{Board, LayerId, NetId, TrackGeom};
use crate::config::Rules;
use crate::geom::{p, point_in_poly, point_seg_dist, poly_area, P};
use crate::problem::{dist_to_outline, inside_board};

#[derive(Clone, Debug, Default)]
pub struct StitchReport {
    pub islands: usize,
    pub nets: usize,
    pub added: usize,
    pub unresolved: usize,
}

struct Island {
    layer: LayerId,
    net: NetId,
    poly: Vec<P>,
    area: f64,
    center: P,
}

struct Dsu {
    p: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Dsu {
        Dsu { p: (0..n).collect() }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.p[x] != x {
            self.p[x] = self.p[self.p[x]];
            x = self.p[x];
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.p[rb] = ra;
        }
    }
}

fn poly_dist(q: P, poly: &[P]) -> f64 {
    let n = poly.len();
    let mut d = f64::INFINITY;
    for i in 0..n {
        d = d.min(point_seg_dist(q, poly[i], poly[(i + 1) % n]));
    }
    d
}

/// A pad or track is joined by a pour when the filled copper reaches its edge:
/// thermal spokes touch the pad, an isolated pad keeps the clearance gap. Keeping
/// this small is what tells the two apart.
const REACH: f64 = 0.1;

fn touches(poly: &[P], q: P, extent: f64) -> bool {
    point_in_poly(q, poly) || poly_dist(q, poly) <= extent + REACH
}

fn inside_any(poly: &[P], q: P) -> bool {
    point_in_poly(q, poly)
}

fn track_points(t: &crate::board::Track) -> (P, P) {
    match t.geom {
        TrackGeom::Seg { a, b } => (a, b),
        TrackGeom::Arc { s, e, .. } => (s, e),
    }
}

/// Distance from q to a track's centreline.
fn track_dist(t: &crate::board::Track, q: P) -> f64 {
    match t.geom {
        TrackGeom::Seg { a, b } => point_seg_dist(q, a, b),
        TrackGeom::Arc { s, m, e } => {
            // sample the arc; good enough for a clearance test
            let mut d = f64::INFINITY;
            let mut prev = s;
            for k in 1..=12 {
                let u = k as f64 / 12.0;
                let x = p(
                    (1.0 - u) * (1.0 - u) * s.x + 2.0 * (1.0 - u) * u * m.x + u * u * e.x,
                    (1.0 - u) * (1.0 - u) * s.y + 2.0 * (1.0 - u) * u * m.y + u * u * e.y,
                );
                d = d.min(point_seg_dist(q, prev, x));
                prev = x;
            }
            d
        }
    }
}

/// Can a via of `net` stand at q without breaking a rule?
fn via_clear(b: &Board, rules: &Rules, q: P, net: NetId, vr: f64) -> bool {
    if !inside_board(q, &b.outline) || dist_to_outline(q, &b.outline) < vr + rules.edge_clearance.max(0.05) {
        return false;
    }
    for pd in &b.pads {
        if pd.net == Some(net) {
            continue;
        }
        let clr = rules.clr(pd.net, Some(net));
        if pd.shape.dist(q) < vr + clr {
            return false;
        }
    }
    for t in &b.tracks {
        if t.net == Some(net) {
            continue;
        }
        let clr = rules.clr(t.net, Some(net));
        if track_dist(t, q) < vr + t.width / 2.0 + clr {
            return false;
        }
    }
    for v in &b.vias {
        if v.net == Some(net) {
            continue;
        }
        let clr = rules.clr(v.net, Some(net));
        if q.dist(v.pos) < vr + v.dia / 2.0 + clr {
            return false;
        }
    }
    for z in &b.zones {
        if let Some(k) = &z.keepout {
            if (k.tracks || k.vias) && z.layers.iter().any(|&l| l == 0 || l < b.layers.len()) && point_in_poly(q, &z.outline) {
                return false;
            }
        }
    }
    true
}

/// Grid points inside both islands (the smaller one drives the grid).
fn overlap_points(a: &Island, b: &Island) -> Vec<P> {
    let mut min = p(f64::INFINITY, f64::INFINITY);
    let mut max = p(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for q in &a.poly {
        min = p(min.x.min(q.x), min.y.min(q.y));
        max = p(max.x.max(q.x), max.y.max(q.y));
    }
    let step = 0.4;
    let mut out = Vec::new();
    let mut y = min.y;
    while y <= max.y && out.len() < 300 {
        let mut x = min.x;
        while x <= max.x && out.len() < 300 {
            let q = p(x, y);
            if point_in_poly(q, &a.poly) && point_in_poly(q, &b.poly) {
                out.push(q);
            }
            x += step;
        }
        y += step;
    }
    out
}

pub fn stitch(b: &Board, rules: &Rules, only_net: Option<&str>, max_vias: usize) -> (Vec<(P, NetId)>, StitchReport) {
    let mut rep = StitchReport::default();
    let mut islands: Vec<Island> = Vec::new();
    for z in &b.zones {
        let net = match (z.net, &z.keepout) {
            (Some(n), None) => n,
            _ => continue,
        };
        for (layer, poly) in &z.fills {
            if poly.len() < 3 {
                continue;
            }
            let area = poly_area(poly).abs();
            if area < 0.05 {
                continue; // slivers are not worth a via
            }
            let mut c = P::default();
            for q in poly {
                c = c + *q;
            }
            islands.push(Island { layer: *layer, net, poly: poly.clone(), area, center: c / poly.len() as f64 });
        }
    }
    rep.islands = islands.len();
    if islands.is_empty() {
        return (Vec::new(), rep);
    }

    let nets: Vec<NetId> = {
        let mut v: Vec<NetId> = islands.iter().map(|i| i.net).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    let mut vias: Vec<(P, NetId)> = Vec::new();
    let vr = rules.via_dia / 2.0;

    for net in nets {
        if let Some(name) = only_net {
            if b.nets[net].name != name {
                continue;
            }
        }
        rep.nets += 1;
        let idx: Vec<usize> = (0..islands.len()).filter(|&i| islands[i].net == net).collect();
        let mut dsu = Dsu::new(islands.len());

        // islands joined through the net's own copper
        for pd in b.pads.iter().filter(|p| p.net == Some(net)) {
            let extent = pd.shape.pts.iter().map(|q| q.dist(pd.pos)).fold(0.0, f64::max) + pd.shape.r;
            let touched: Vec<usize> = idx
                .iter()
                .copied()
                .filter(|&i| pd.layers.contains(&islands[i].layer) && touches(&islands[i].poly, pd.pos, extent))
                .collect();
            for w in touched.windows(2) {
                dsu.union(w[0], w[1]);
            }
        }
        for t in b.tracks.iter().filter(|t| t.net == Some(net)) {
            let (a, bb) = track_points(t);
            let touched: Vec<usize> = idx
                .iter()
                .copied()
                .filter(|&i| {
                    islands[i].layer == t.layer
                        && (touches(&islands[i].poly, a, t.width / 2.0) || touches(&islands[i].poly, bb, t.width / 2.0))
                })
                .collect();
            for w in touched.windows(2) {
                dsu.union(w[0], w[1]);
            }
        }
        for v in b.vias.iter().filter(|v| v.net == Some(net)) {
            let touched: Vec<usize> = idx.iter().copied().filter(|&i| touches(&islands[i].poly, v.pos, v.dia / 2.0)).collect();
            for w in touched.windows(2) {
                dsu.union(w[0], w[1]);
            }
        }

        let mut added_here = 0;
        loop {
            let mut comps: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
            for &i in &idx {
                let r = dsu.find(i);
                comps.entry(r).or_default().push(i);
            }
            if comps.len() <= 1 {
                break;
            }
            let main = *comps
                .iter()
                .max_by(|a, b| {
                    let sa: f64 = a.1.iter().map(|&i| islands[i].area).sum();
                    let sb: f64 = b.1.iter().map(|&i| islands[i].area).sum();
                    sa.total_cmp(&sb)
                })
                .map(|(r, _)| r)
                .unwrap();
            let mains: Vec<usize> = comps[&main].clone();
            let others: Vec<(usize, Vec<usize>)> = comps.into_iter().filter(|(r, _)| *r != main).collect();
            let mut progress = false;
            for (root, members) in others {
                if vias.len() >= max_vias {
                    break;
                }
                let mut done = false;
                'search: for &i in &members {
                    let a = &islands[i];
                    for &j in &mains {
                        let c = &islands[j];
                        if a.layer == c.layer {
                            continue;
                        }
                        for q in overlap_points(a, c) {
                            if !inside_any(&c.poly, q) {
                                continue;
                            }
                            let mut cands = vec![q];
                            for r in [0.3, 0.6, 0.9] {
                                for k in 0..8 {
                                    let ang = std::f64::consts::TAU * k as f64 / 8.0;
                                    let t = q + p(r * ang.cos(), r * ang.sin());
                                    if point_in_poly(t, &a.poly) && point_in_poly(t, &c.poly) {
                                        cands.push(t);
                                    }
                                }
                            }
                            for t in cands {
                                if !inside_any(&c.poly, t) {
                                    continue;
                                }
                                if vias.iter().any(|(v, _)| v.dist(t) < rules.via_dia + 0.2) {
                                    continue;
                                }
                                if !via_clear(b, rules, t, net, vr) {
                                    continue;
                                }
                                vias.push((t, net));
                                added_here += 1;
                                dsu.union(main, i);
                                progress = true;
                                done = true;
                                break 'search;
                            }
                        }
                    }
                }
                if !done {
                    rep.unresolved += 1;
                }
                let _ = root;
            }
            if !progress {
                break;
            }
        }
        rep.added += added_here;
    }
    (vias, rep)
}
