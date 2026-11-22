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
    pub tracks: usize,
    pub unresolved: usize,
}

/// A short track drawn by the stitcher to join a pad's island to the main pour.
#[derive(Clone, Debug)]
pub struct TrackStub {
    pub layer: LayerId,
    pub net: NetId,
    pub a: P,
    pub b: P,
    pub width: f64,
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

/// A pad or track is joined by a pour when the filled copper reaches its edge.
/// Tracks and vias must really touch the fill (2 um is numeric slack); pads get a
/// wider sampling in `pad_on_pour`, because the filler keeps a clearance moat around
/// a thermal-relieved pad and only the spokes bridge it.
const REACH: f64 = 0.02;

/// Does the filled copper actually reach the pad's copper? Sampling the pad outline
/// (and a hair outside it) is what separates a pad with a thermal spoke from one the
/// filler left in its own clearance moat - the circumscribed radius of a long pad
/// overshoots the gap and makes every pad look connected.
fn pad_on_pour(pd: &crate::board::Pad, poly: &[P]) -> bool {
    if point_in_poly(pd.pos, poly) {
        return true;
    }
    let n = pd.shape.pts.len();
    if n == 0 {
        return poly_dist(pd.pos, poly) <= pd.shape.r + REACH;
    }
    for i in 0..n {
        let a = pd.shape.pts[i];
        let b = pd.shape.pts[(i + 1) % n];
        for k in 0..=6 {
            let q = a + (b - a) * (k as f64 / 6.0);
            if point_in_poly(q, poly) {
                return true;
            }
            let d = q - pd.pos;
            let l = d.len();
            if l > 1e-9 {
                // a thermal spoke is copper from the pad edge outwards, so a probe
                // 0.15 mm beyond the outline still lands in it; a pad sitting in its
                // clearance moat stays outside
                let out = q + d * (0.15 / l);
                if point_in_poly(out, poly) {
                    return true;
                }
            }
        }
    }
    false
}

/// Distance from a pad's copper to a segment (0 where they overlap).
fn pad_track_dist(pd: &crate::board::Pad, a: P, b: P) -> f64 {
    let n = ((a.dist(b) / 0.1).ceil() as usize).clamp(1, 200);
    let mut d = f64::INFINITY;
    for k in 0..=n {
        let q = a + (b - a) * (k as f64 / n as f64);
        d = d.min(pd.shape.dist(q));
    }
    d
}

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
            for k in 1..=24 {
                let u = k as f64 / 24.0;
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
        if pd.shape.dist(q) < vr + clr + 0.05 {
            return false;
        }
    }
    for t in &b.tracks {
        if t.net == Some(net) {
            continue;
        }
        let clr = rules.clr(t.net, Some(net));
        if track_dist(t, q) < vr + t.width / 2.0 + clr + 0.05 {
            return false;
        }
    }
    for v in &b.vias {
        if v.net == Some(net) {
            continue;
        }
        let clr = rules.clr(v.net, Some(net));
        if q.dist(v.pos) < vr + v.dia / 2.0 + clr + 0.05 {
            return false;
        }
    }
    for z in &b.zones {
        if let Some(k) = &z.keepout {
            if (k.tracks || k.vias) && z.layers.iter().any(|&l| l == 0 || l < b.layers.len()) && point_in_poly(q, &z.outline) {
                return false;
            }
            continue;
        }
        if z.net == Some(net) {
            continue;
        }
        let clr = rules.clr(z.net, Some(net));
        for (_, poly) in &z.fills {
            if poly.len() >= 3 && (point_in_poly(q, poly) || poly_dist(q, poly) < vr + clr) {
                return false;
            }
        }
    }
    true
}

/// Can a track of `net` and width `w` on `layer` run from a to c without breaking a
/// rule? Only copper on the same layer takes part: a track on the other side does not
/// block, THT pads and vias do.
fn seg_clear(b: &Board, rules: &Rules, a: P, c: P, net: NetId, w: f64, layer: LayerId) -> bool {
    if !inside_board(a, &b.outline) || !inside_board(c, &b.outline) {
        return false;
    }
    let r = w / 2.0;
    let n = ((a.dist(c) / 0.15).ceil() as usize).max(1);
    for k in 0..=n {
        let q = a + (c - a) * (k as f64 / n as f64);
        for pd in &b.pads {
            if pd.net == Some(net) || !pd.layers.contains(&layer) {
                continue;
            }
            let clr = rules.clr(pd.net, Some(net));
            if pd.shape.dist(q) < r + clr + 0.02 {
                return false;
            }
        }
        for t in &b.tracks {
            if t.net == Some(net) || t.layer != layer {
                continue;
            }
            let clr = rules.clr(t.net, Some(net));
            if track_dist(t, q) < r + t.width / 2.0 + clr + 0.02 {
                return false;
            }
        }
        for v in &b.vias {
            if v.net == Some(net) {
                continue;
            }
            let clr = rules.clr(v.net, Some(net));
            if q.dist(v.pos) < r + v.dia / 2.0 + clr + 0.02 {
                return false;
            }
        }
        for z in &b.zones {
            if z.net == Some(net) {
                continue;
            }
            if z.keepout.is_some() {
                if z.layers.contains(&layer) && (point_in_poly(q, &z.outline) || poly_dist(q, &z.outline) < r + 0.02) {
                    return false;
                }
                continue;
            }
            let clr = rules.clr(z.net, Some(net));
            for (l, poly) in &z.fills {
                if *l != layer {
                    continue;
                }
                if poly.len() >= 3 && (point_in_poly(q, poly) || poly_dist(q, poly) < r + clr + 0.02) {
                    return false;
                }
            }
        }
    }
    true
}

/// Nearest point on a polygon's boundary, plus its distance.
fn nearest_on_poly(poly: &[P], q: P) -> (P, f64) {
    let mut best = poly[0];
    let mut bd = f64::INFINITY;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let ab = b - a;
        let t = if ab.len() < 1e-9 { 0.0 } else { ((q - a).dot(ab) / ab.len()).clamp(0.0, 1.0) };
        let c = a + ab * t;
        let d = q.dist(c);
        if d < bd {
            bd = d;
            best = c;
        }
    }
    (best, bd)
}

/// Grid points inside both islands. The grid covers the intersection of their
/// bounding boxes and the step grows with the area, so a big pour is sampled over
/// its whole overlap, not just its top-left corner.
fn overlap_points(a: &Island, b: &Island) -> Vec<P> {
    let bbox = |poly: &[P]| {
        let mut min = p(f64::INFINITY, f64::INFINITY);
        let mut max = p(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for q in poly {
            min = p(min.x.min(q.x), min.y.min(q.y));
            max = p(max.x.max(q.x), max.y.max(q.y));
        }
        (min, max)
    };
    let (a0, a1) = bbox(&a.poly);
    let (b0, b1) = bbox(&b.poly);
    let min = p(a0.x.max(b0.x), a0.y.max(b0.y));
    let max = p(a1.x.min(b1.x), a1.y.min(b1.y));
    let (w, h) = (max.x - min.x, max.y - min.y);
    if !(w > 0.0 && h > 0.0) {
        return Vec::new();
    }
    let cap = 4000.0;
    let step = (0.5f64).max((w * h / cap).sqrt());
    let mut out = Vec::new();
    let mut y = min.y;
    while y <= max.y {
        let mut x = min.x;
        while x <= max.x {
            let q = p(x, y);
            if point_in_poly(q, &a.poly) && point_in_poly(q, &b.poly) {
                out.push(q);
                if out.len() >= 2000 {
                    return out;
                }
            }
            x += step;
        }
        y += step;
    }
    out
}

pub fn stitch(
    b: &Board,
    rules: &Rules,
    only_net: Option<&str>,
    max_vias: usize,
    orphans: &[(NetId, P)],
) -> (Vec<(P, NetId)>, Vec<TrackStub>, StitchReport) {
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
        return (Vec::new(), Vec::new(), rep);
    }

    let nets: Vec<NetId> = {
        let mut v: Vec<NetId> = islands.iter().map(|i| i.net).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    let mut vias: Vec<(P, NetId)> = Vec::new();
    let mut stubs: Vec<TrackStub> = Vec::new();
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
            let touched: Vec<usize> = idx
                .iter()
                .copied()
                .filter(|&i| pd.layers.contains(&islands[i].layer) && pad_on_pour(pd, &islands[i].poly))
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

        // Pads the pour never reached: the filler can close every thermal spoke and
        // leave the pad sitting in its clearance moat. A person draws a short track
        // from the pad into the copper; same net, so it is legal and cheap. When the
        // caller has KiCad's own unconnected list (`orphans`), that list decides which
        // pads to bridge; otherwise the fill is sampled for pads sitting in a moat.
        {
            let spots: Vec<P> = orphans.iter().filter(|(n, _)| *n == net).map(|(_, q)| *q).collect();
            let candidates: Vec<usize> = if spots.is_empty() {
                (0..b.pads.len())
                    .filter(|&pi| {
                        let pd = &b.pads[pi];
                        pd.net == Some(net)
                            && !idx.iter().any(|&i| pd.layers.contains(&islands[i].layer) && pad_on_pour(pd, &islands[i].poly))
                            && !b.tracks.iter().filter(|t| t.net == Some(net)).any(|t| {
                                let (a, bb) = track_points(t);
                                pd.layers.contains(&t.layer) && pad_track_dist(pd, a, bb) <= t.width / 2.0 + REACH
                            })
                            && !b.vias.iter().filter(|v| v.net == Some(net)).any(|v| pd.shape.dist(v.pos) <= v.dia / 2.0 + REACH)
                    })
                    .collect()
            } else {
                b.pads
                    .iter()
                    .enumerate()
                    .filter(|(_, pd)| pd.net == Some(net) && spots.iter().any(|q| pd.pos.dist(*q) < 0.05))
                    .map(|(pi, _)| pi)
                    .collect()
            };
            let w0 = rules.nets[net].width;
            let from_kicad = !spots.is_empty();
            for pi in candidates {
                let pd = &b.pads[pi];
                if stubs.iter().any(|s| s.a.dist(pd.pos) < 0.01) {
                    continue;
                }
                let mut nd = f64::INFINITY;
                for &i in &idx {
                    if pd.layers.contains(&islands[i].layer) {
                        nd = nd.min(poly_dist(pd.pos, &islands[i].poly));
                    }
                }
                let ext = pd.shape.pts.iter().map(|q| q.dist(pd.pos)).fold(0.0, f64::max) + pd.shape.r;
                // the pour is already at the pad edge: the filler just keeps the moat
                // narrow there, no bridge needed. Only as a heuristic - KiCad's own
                // unconnected list is authoritative.
                if !from_kicad && nd <= ext + 0.12 {
                    continue;
                }
                // narrower escapes are normal for a 0.5 mm pitch connector: the hand board
                // uses 0.25 mm here. Never go under the rules' minimum track width.
                let mut widths: Vec<f64> = [w0, 0.4, 0.3, 0.25, 0.2]
                    .into_iter()
                    .filter(|w| *w <= w0 + 1e-9)
                    .collect();
                widths.dedup();
                if widths.is_empty() {
                    widths.push(w0);
                }
                // target, width, length, layer, island area: land on the biggest piece of
                // the pour, not on a sliver the signals cut off
                let mut bridge: Option<(P, f64, f64, LayerId, f64)> = None;
                for k in 0..180 {
                    let ang = std::f64::consts::TAU * k as f64 / 180.0;
                    let dir = p(ang.cos(), ang.sin());
                    let mut t = 0.0;
                    let mut hit: Option<(P, LayerId, f64)> = None;
                    while t < 4.0 {
                        t += 0.05;
                        let q = pd.pos + dir * t;
                        if let Some(&i) = idx.iter().find(|&&i| {
                            pd.layers.contains(&islands[i].layer) && point_in_poly(q, &islands[i].poly)
                        }) {
                            hit = Some((q, islands[i].layer, islands[i].area));
                            break;
                        }
                    }
                    let Some((q, layer, area)) = hit else { continue };
                    let d = pd.pos.dist(q);
                    for &w in &widths {
                        let w = w.max(0.2);
                        if seg_clear(b, rules, pd.pos, q, net, w, layer) {
                            if bridge.map_or(true, |(_, _bw, bd, _, ba)| {
                                area > ba * 1.05 || (area >= ba * 0.95 && d < bd - 0.05)
                            }) {
                                bridge = Some((q, w, d, layer, area));
                            }
                            break;
                        }
                    }
                }
                if let Some((q, w, _, layer, _)) = bridge {
                    stubs.push(TrackStub { layer, net, a: pd.pos, b: q, width: w });
                    rep.tracks += 1;
                } else {
                    rep.unresolved += 1;
                    log::info!(
                        "stitch: pad {} left in its moat (nearest pour {:.3} mm, pad extent {:.3} mm)",
                        b.pad_label(pi),
                        nd,
                        ext
                    );
                }
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
                    // Last resort: a short track from a pad of this component to the
                    // main pour on the same layer. This is what a person does when the
                    // thermal barrier blocks the pour: draw the pad out to the copper.
                    let w = rules.nets[net].width;
                    'stub: for &i in &members {
                        let isl = &islands[i];
                        for pd in b.pads.iter().filter(|pd| pd.net == Some(net) && pd.layers.contains(&isl.layer)) {
                            let extent = pd.shape.pts.iter().map(|q| q.dist(pd.pos)).fold(0.0, f64::max) + pd.shape.r;
                            if !touches(&isl.poly, pd.pos, extent) {
                                continue;
                            }
                            for &j in &mains {
                                let cm = &islands[j];
                                if cm.layer != isl.layer {
                                    continue;
                                }
                                let (q, d) = nearest_on_poly(&cm.poly, pd.pos);
                                if d > 4.0 {
                                    continue;
                                }
                                if stubs.iter().any(|s| s.a.dist(pd.pos) < 0.01) {
                                    continue;
                                }
                                if seg_clear(b, rules, pd.pos, q, net, w, isl.layer) {
                                    stubs.push(TrackStub { layer: isl.layer, net, a: pd.pos, b: q, width: w });
                                    rep.tracks += 1;
                                    dsu.union(main, i);
                                    progress = true;
                                    done = true;
                                    break 'stub;
                                }
                            }
                        }
                    }
                }
                if !done {
                    rep.unresolved += 1;
                    let a = &islands[members[0]];
                    log::info!(
                        "stitch: island {:.1} mm2 at ({:.1}, {:.1}) on {} stays separate",
                        members.iter().map(|&i| islands[i].area).sum::<f64>(),
                        a.center.x,
                        a.center.y,
                        b.layers[a.layer].name
                    );
                }
                let _ = root;
            }
            if !progress {
                break;
            }
        }
        rep.added += added_here;

        // No island of the plane may be left floating: the island graph can think two
        // fragments are joined through a pad while KiCad's connectivity says they are
        // not. Ground stitching is wanted anyway, so give every island of decent size
        // its own via to the other layer - unless one already sits in that island.
        for &i in &idx {
            let isl = &islands[i];
            if isl.area < 0.5 {
                continue;
            }
            let has_via = vias.iter().any(|(q, _)| point_in_poly(*q, &isl.poly))
                || b.vias.iter().any(|v| v.net == Some(net) && point_in_poly(v.pos, &isl.poly));
            if has_via {
                continue;
            }
            let mut stitched = false;
            let mut partners: Vec<usize> = idx.iter().copied().filter(|&j| islands[j].layer != isl.layer).collect();
            partners.sort_by(|&a, &b| islands[b].area.total_cmp(&islands[a].area));
            partners.truncate(3);
            for j in partners {
                let c = &islands[j];
                let mut cands = overlap_points(isl, c);
                // via-in-pad: a same-net pad inside the island is always a legal spot
                // for the via, and in a dense connector it is the only one left
                for pd in b.pads.iter().filter(|pd| pd.net == Some(net) && pd.kind == crate::board::PadKind::Smd) {
                    if pd.layers.contains(&isl.layer)
                        && point_in_poly(pd.pos, &isl.poly)
                        && point_in_poly(pd.pos, &c.poly)
                    {
                        cands.push(pd.pos);
                    }
                }
                let mut spot = None;
                for q in cands {
                    if !via_clear(b, rules, q, net, vr) {
                        continue;
                    }
                    if vias.iter().any(|(v, _)| v.dist(q) < rules.via_dia + 0.2) {
                        continue;
                    }
                    spot = Some(q);
                    break;
                }
                if let Some(q) = spot {
                    vias.push((q, net));
                    rep.added += 1;
                    stitched = true;
                    break;
                }
            }
            if stitched {
                continue;
            }
            // A THT pad inside the island already ties F and B through its barrel: draw
            // a short track from the pad to the pour on the other layer, and the whole
            // island comes along.
            for pd in b.pads.iter().filter(|pd| {
                pd.net == Some(net) && pd.kind == crate::board::PadKind::Tht && pd.layers.contains(&isl.layer)
            }) {
                if !point_in_poly(pd.pos, &isl.poly) {
                    continue;
                }
                let w = rules.nets[net].width;
                for &layer in pd.layers.iter().filter(|&&l| l != isl.layer) {
                    let mut partners: Vec<usize> = idx.iter().copied().filter(|&j| islands[j].layer == layer).collect();
                    partners.sort_by(|&a, &b| {
                        islands[a]
                            .center
                            .dist(pd.pos)
                            .total_cmp(&islands[b].center.dist(pd.pos))
                    });
                    for j in partners.into_iter().take(3) {
                        let c = &islands[j];
                        let (q, d) = nearest_on_poly(&c.poly, pd.pos);
                        if d > 3.0 || !seg_clear(b, rules, pd.pos, q, net, w, layer) {
                            continue;
                        }
                        stubs.push(TrackStub { layer, net, a: pd.pos, b: q, width: w });
                        rep.tracks += 1;
                        stitched = true;
                        break;
                    }
                    if stitched {
                        break;
                    }
                }
                if stitched {
                    break;
                }
            }
            if stitched {
                continue;
            }
            // No copper on the other layer here: bridge to another island of the same
            // layer with a short track across the gap the signals left.
            let mut same: Vec<usize> = idx
                .iter()
                .copied()
                .filter(|&j| j != i && islands[j].layer == isl.layer && islands[j].area >= isl.area * 0.5)
                .collect();
            same.sort_by(|&a, &b| {
                islands[a]
                    .center
                    .dist(isl.center)
                    .total_cmp(&islands[b].center.dist(isl.center))
            });
            for j in same.into_iter().take(4) {
                let c = &islands[j];
                let (q1, _) = nearest_on_poly(&isl.poly, c.center);
                let (q2, _) = nearest_on_poly(&c.poly, isl.center);
                let w = rules.nets[net].width;
                if seg_clear(b, rules, q1, q2, net, w, isl.layer) {
                    stubs.push(TrackStub { layer: isl.layer, net, a: q1, b: q2, width: w });
                    rep.tracks += 1;
                    break;
                }
            }
        }
    }
    (vias, stubs, rep)
}
