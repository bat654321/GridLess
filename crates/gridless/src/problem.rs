//! Routing problem: obstacles per layer, via sites and the list of connections to route.

use crate::board::*;
use crate::config::{LayerRole, Rules};
use crate::geom::{point_in_poly, point_seg_dist, BBox, RoundPoly, P};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObsKind {
    Pad(usize),
    Track,
    Via,
    Keepout,
    Copper,
    /// straight exit stub of a fine-pitch pad (pad index)
    Escape(usize),
}

#[derive(Clone, Debug)]
pub struct Obstacle {
    pub shape: RoundPoly,
    /// connection point of a pad
    pub anchor: Option<P>,
    pub net: Option<NetId>,
    /// bitmask of copper layers
    pub layers: u64,
    pub kind: ObsKind,
    pub drill: Option<f64>,
}

impl Obstacle {
    pub fn on(&self, l: LayerId) -> bool {
        self.layers & (1u64 << l) != 0
    }
    pub fn on_mask(&self, m: u64) -> bool {
        self.layers & m != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Term {
    Pad(usize),
    /// any free via site (fanout to a plane)
    AnySite,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnKind {
    Signal,
    Fanout,
    /// coupled differential pair routed as one fat band; `pair` indexes Rules::diff_pairs
    Pair { pair: usize },
}

#[derive(Clone, Debug)]
pub struct Conn {
    pub net: NetId,
    /// N net for a differential pair band
    pub net2: Option<NetId>,
    pub a: Term,
    pub b: Term,
    /// for pairs: the N pads matching a and b
    pub a2: Option<usize>,
    pub b2: Option<usize>,
    pub width: f64,
    pub clearance: f64,
    pub kind: ConnKind,
    pub priority: i32,
    pub est_len: f64,
    /// differential pair member: (pair index, is the P/leader net)
    pub couple: Option<(usize, bool)>,
}

#[derive(Clone, Debug)]
pub struct ViaSite {
    pub pos: P,
    /// plane nets whose plane zone contains this site (a fanout via may land here)
    pub plane_nets: Vec<NetId>,
    /// sites closer than the via-to-via spacing
    pub conflicts: Vec<usize>,
}

pub struct Problem {
    pub obstacles: Vec<Obstacle>,
    /// closed board outline polygons (first = outer)
    pub outline: Vec<Vec<P>>,
    pub sites: Vec<ViaSite>,
    pub conns: Vec<Conn>,
    pub route_layers: Vec<LayerId>,
    pub bbox: BBox,
    /// nets that are connected through a plane layer (pads on all layers / vias suffice)
    pub plane_nets: Vec<NetId>,
    /// pads already joined by fixed copper (group representative pad per pad)
    pub pad_group: Vec<usize>,
    /// pads (position, radius) left to a same-net pour: signal routing pays extra
    /// near them so it cannot cut the pour away from the pad
    pub pour_pads: Vec<(P, f64)>,
    /// exit point and stub width of fine-pitch pads (routing starts there)
    pub escape: Vec<Option<(P, f64)>>,
}

fn mask(ls: &[LayerId]) -> u64 {
    ls.iter().fold(0u64, |m, &l| m | (1u64 << l))
}

pub fn dist_to_outline(q: P, outline: &[Vec<P>]) -> f64 {
    let mut d = f64::INFINITY;
    for poly in outline {
        let n = poly.len();
        for i in 0..n {
            d = d.min(point_seg_dist(q, poly[i], poly[(i + 1) % n]));
        }
    }
    d
}

pub fn inside_board(q: P, outline: &[Vec<P>]) -> bool {
    if outline.is_empty() {
        return true;
    }
    if !point_in_poly(q, &outline[0]) {
        return false;
    }
    // holes / cutouts
    !outline[1..].iter().any(|h| h.len() >= 3 && point_in_poly(q, h))
}

struct Grid {
    cell: f64,
    origin: P,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<usize>>,
}

impl Grid {
    fn new(bb: BBox, cell: f64) -> Grid {
        let nx = ((bb.w() / cell).ceil() as usize).max(1) + 1;
        let ny = ((bb.h() / cell).ceil() as usize).max(1) + 1;
        Grid { cell, origin: bb.min, nx, ny, cells: vec![Vec::new(); nx * ny] }
    }
    fn idx(&self, q: P) -> (isize, isize) {
        (((q.x - self.origin.x) / self.cell).floor() as isize, ((q.y - self.origin.y) / self.cell).floor() as isize)
    }
    fn insert_box(&mut self, b: BBox, id: usize) {
        let (x0, y0) = self.idx(b.min);
        let (x1, y1) = self.idx(b.max);
        for y in y0.max(0)..=y1.min(self.ny as isize - 1) {
            for x in x0.max(0)..=x1.min(self.nx as isize - 1) {
                self.cells[y as usize * self.nx + x as usize].push(id);
            }
        }
    }
    fn query(&self, q: P, r: f64, out: &mut Vec<usize>) {
        out.clear();
        let (x0, y0) = self.idx(q - crate::geom::p(r, r));
        let (x1, y1) = self.idx(q + crate::geom::p(r, r));
        for y in y0.max(0)..=y1.min(self.ny as isize - 1) {
            for x in x0.max(0)..=x1.min(self.nx as isize - 1) {
                out.extend_from_slice(&self.cells[y as usize * self.nx + x as usize]);
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

impl Problem {
    pub fn build(b: &Board, rules: &Rules) -> Problem {
        let route_layers = rules.routing_layers();
        let all_cu: Vec<LayerId> = (0..b.layers.len()).collect();
        let mut obstacles = Vec::new();
        for (i, pd) in b.pads.iter().enumerate() {
            let layers = if pd.kind == PadKind::Npth { all_cu.clone() } else { pd.layers.clone() };
            // NPTH: hole only, copper-free; model the hole as an obstacle on all layers
            let shape = if pd.kind == PadKind::Npth && pd.layers.is_empty() {
                RoundPoly::circle(pd.pos, pd.drill.unwrap_or(0.0) / 2.0)
            } else {
                pd.shape.clone()
            };
            obstacles.push(Obstacle { shape, anchor: Some(pd.pos), net: pd.net, layers: mask(&layers), kind: ObsKind::Pad(i), drill: pd.drill });
        }
        let escape = pad_escapes(b, rules, &obstacles);
        for (i, e) in escape.iter().enumerate() {
            if let Some((q, w)) = e {
                let pd = &b.pads[i];
                obstacles.push(Obstacle { anchor: None, shape: RoundPoly::capsule(pd.pos, *q, w / 2.0), net: pd.net, layers: mask(&pd.layers), kind: ObsKind::Escape(i), drill: None });
            }
        }
        if rules.keep_existing {
            for t in &b.tracks {
                let pts = match t.geom {
                    TrackGeom::Seg { a, b } => vec![a, b],
                    TrackGeom::Arc { s, m, e } => match crate::geom::Arc::from_3p(s, m, e) {
                        Some(a) => a.polyline(0.002),
                        None => vec![s, e],
                    },
                };
                for w in pts.windows(2) {
                    obstacles.push(Obstacle { anchor: None, shape: RoundPoly::capsule(w[0], w[1], t.width / 2.0), net: t.net, layers: 1u64 << t.layer, kind: ObsKind::Track, drill: None });
                }
            }
            for v in &b.vias {
                obstacles.push(Obstacle { anchor: None, shape: RoundPoly::circle(v.pos, v.dia / 2.0), net: v.net, layers: mask(&all_cu), kind: ObsKind::Via, drill: Some(v.drill) });
            }
        } else {
            for t in b.tracks.iter().filter(|t| t.locked) {
                if let TrackGeom::Seg { a, b: e } = t.geom {
                    obstacles.push(Obstacle { anchor: None, shape: RoundPoly::capsule(a, e, t.width / 2.0), net: t.net, layers: 1u64 << t.layer, kind: ObsKind::Track, drill: None });
                }
            }
            for v in b.vias.iter().filter(|v| v.locked) {
                obstacles.push(Obstacle { anchor: None, shape: RoundPoly::circle(v.pos, v.dia / 2.0), net: v.net, layers: mask(&all_cu), kind: ObsKind::Via, drill: Some(v.drill) });
            }
        }
        for cs in &b.copper {
            for pc in &cs.pieces {
                obstacles.push(Obstacle { anchor: None, shape: pc.clone(), net: cs.net, layers: 1u64 << cs.layer, kind: ObsKind::Copper, drill: None });
            }
        }
        // pads already joined by fixed copper of their net (copper graphics, and the
        // existing tracks/vias when they are kept)
        let pad_group = {
            let mut items: Vec<(RoundPoly, Option<NetId>, u64, Option<usize>)> = Vec::new();
            for (i, pd) in b.pads.iter().enumerate() {
                if pd.net.is_some() {
                    items.push((pd.shape.clone(), pd.net, mask(&pd.layers), Some(i)));
                }
            }
            for cs in &b.copper {
                for pc in &cs.pieces {
                    items.push((pc.clone(), cs.net, 1u64 << cs.layer, None));
                }
            }
            for o in &obstacles {
                if matches!(o.kind, ObsKind::Track | ObsKind::Via) {
                    items.push((o.shape.clone(), o.net, o.layers, None));
                }
            }
            let n = items.len();
            let mut parent: Vec<usize> = (0..n).collect();
            fn find(g: &mut [usize], mut x: usize) -> usize {
                while g[x] != x {
                    g[x] = g[g[x]];
                    x = g[x];
                }
                x
            }
            let bbs: Vec<BBox> = items.iter().map(|it| it.0.bbox()).collect();
            for i in 0..n {
                for j in i + 1..n {
                    let (a, c) = (&items[i], &items[j]);
                    if a.1.is_none() || a.1 != c.1 || a.2 & c.2 == 0 || !bbs[i].overlaps(&bbs[j]) {
                        continue;
                    }
                    if a.3.is_some() && c.3.is_some() {
                        continue; // pad-pad overlap does not count
                    }
                    if a.0.shape_dist(&c.0) <= 1e-6 {
                        let (x, y) = (find(&mut parent, i), find(&mut parent, j));
                        parent[x] = y;
                    }
                }
            }
            let mut root_pad: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
            let mut g: Vec<usize> = (0..b.pads.len()).collect();
            for i in 0..n {
                if let Some(pi) = items[i].3 {
                    let r = find(&mut parent, i);
                    let rep = *root_pad.entry(r).or_insert(pi);
                    g[pi] = rep;
                }
            }
            g
        };
        for z in &b.zones {
            if let Some(k) = &z.keepout {
                if k.tracks || k.vias {
                    // keepouts are concave in general: split into convex pieces
                    for pc in crate::geom::convex_pieces(&z.outline) {
                        if pc.len() >= 3 {
                            obstacles.push(Obstacle { anchor: None, shape: RoundPoly { pts: pc, r: 0.0 }, net: None, layers: mask(&z.layers), kind: ObsKind::Keepout, drill: None });
                        }
                    }
                }
            }
        }
        let outline = b.outline.clone();
        let bbox = b.bbox();

        // ---- via sites
        let via_r = rules.via_dia / 2.0;
        let mut grid = Grid::new(bbox.grow(1.0), 1.0);
        for (i, o) in obstacles.iter().enumerate() {
            grid.insert_box(o.shape.bbox(), i);
        }
        let mut near = Vec::new();
        let legal = |q: P, near: &mut Vec<usize>| -> bool {
            if !inside_board(q, &outline) || dist_to_outline(q, &outline) < via_r + rules.edge_clearance.max(0.05) {
                return false;
            }
            grid.query(q, via_r + 1.0, near);
            for &oi in near.iter() {
                let o = &obstacles[oi];
                if o.kind == ObsKind::Keepout {
                    if crate::geom::point_in_poly(q, &o.shape.pts) {
                        return false;
                    }
                    continue;
                }
                let clr = rules.clr(o.net, None);
                if o.shape.dist(q) < via_r + clr {
                    return false;
                }
                if let Some(d) = o.drill {
                    if q.dist(o.shape.centroid()) < d / 2.0 + rules.via_drill / 2.0 + rules.hole_to_hole {
                        return false;
                    }
                }
            }
            true
        };
        let mut cand: Vec<P> = Vec::new();
        // hexagonal lattice
        let s = (rules.via_dia + rules.default_clearance + 0.25).max(0.7);
        let mut row = 0;
        let mut y = bbox.min.y + s * 0.5;
        while y < bbox.max.y {
            let off = if row % 2 == 1 { s * 0.5 } else { 0.0 };
            let mut x = bbox.min.x + s * 0.5 + off;
            while x < bbox.max.x {
                cand.push(crate::geom::p(x, y));
                x += s;
            }
            y += s * 0.866;
            row += 1;
        }
        // fanout candidates next to SMD pads
        for pd in b.pads.iter().filter(|p| p.kind == PadKind::Smd && p.net.is_some()) {
            let bb = pd.shape.bbox();
            let ext = (bb.w().max(bb.h())) * 0.5;
            let clr = rules.clr(pd.net, None);
            for k in 0..8 {
                let a = k as f64 * std::f64::consts::FRAC_PI_4;
                for f in [1.0, 1.4] {
                    let dir = crate::geom::p(a.cos(), a.sin());
                    // march outwards until legal distance from the pad itself
                    let mut d = 0.0;
                    while d < ext + 2.0 {
                        let q = pd.pos + dir * d;
                        if pd.shape.dist(q) >= (via_r + clr) * f {
                            cand.push(q);
                            break;
                        }
                        d += 0.05;
                    }
                }
            }
        }
        let mut sites: Vec<ViaSite> = Vec::new();
        let min_sep = rules.via_dia * 0.5; // dedup very close candidates
        let mut sgrid = Grid::new(bbox.grow(1.0), 1.0);
        let mut snear = Vec::new();
        for q in cand {
            if !legal(q, &mut near) {
                continue;
            }
            sgrid.query(q, min_sep, &mut snear);
            if snear.iter().any(|&j| sites[j].pos.dist(q) < min_sep) {
                continue;
            }
            let id = sites.len();
            let pn: Vec<NetId> = b
                .zones
                .iter()
                .filter(|z| z.keepout.is_none() && z.net.map_or(false, |n| rules.nets[n].plane.map_or(false, |pl| z.layers.contains(&pl))) && point_in_poly(q, &z.outline))
                .filter_map(|z| z.net)
                .collect();
            sites.push(ViaSite { pos: q, conflicts: Vec::new(), plane_nets: pn });
            sgrid.insert_box(BBox { min: q, max: q }, id);
        }
        let conflict_d = rules.via_dia + rules.default_clearance.max(rules.hole_to_hole + rules.via_drill - rules.via_dia);
        for i in 0..sites.len() {
            sgrid.query(sites[i].pos, conflict_d, &mut snear);
            let c: Vec<usize> = snear.iter().copied().filter(|&j| j != i && sites[j].pos.dist(sites[i].pos) < conflict_d).collect();
            sites[i].conflicts = c;
        }

        // ---- connections
        let mut conns = Vec::new();
        let mut plane_nets = Vec::new();
        let mut in_pair = vec![false; b.nets.len()];
        for (pi, dp) in rules.diff_pairs.iter().enumerate() {
            in_pair[dp.p] = true;
            in_pair[dp.n] = true;
            plan_pair(b, rules, pi, &mut conns);
        }
        for (ni, nr) in rules.nets.iter().enumerate() {
            if nr.skip || in_pair[ni] {
                continue;
            }
            let pads: Vec<usize> = (0..b.pads.len()).filter(|&i| b.pads[i].net == Some(ni) && b.pads[i].kind != PadKind::Npth && !b.pads[i].layers.is_empty()).collect();
            if let Some(pl) = nr.plane {
                plane_nets.push(ni);
                for &pi in &pads {
                    let pd = &b.pads[pi];
                    if pd.layers.contains(&pl) {
                        continue; // THT: touches the plane
                    }
                    if rules.trust_pours && pour_covers(b, ni, pd) && !rules.force_fanout.contains(&b.pad_label(pi)) {
                        continue;
                    }
                    conns.push(Conn { net: ni, net2: None, a: Term::Pad(pi), b: Term::AnySite, a2: None, b2: None, width: nr.width, clearance: nr.clearance, kind: ConnKind::Fanout, priority: nr.priority + 1, est_len: 0.5, couple: None });
                }
                continue;
            }
            if pads.len() < 2 {
                continue;
            }
            // A pad inside a same-net pour on its own layer is joined by the pour, on
            // outer layers as well: no track is routed between two such pads. Pads the
            // pour does not cover keep their tracks, so a net is never left dangling.
            let covered = |i: usize| {
                rules.trust_pours
                    && pour_covers(b, ni, &b.pads[i])
                    && !rules.force_fanout.contains(&b.pad_label(i))
            };
            let mut left_to_pour = 0;
            for (x, y) in mst_grouped(b, rules, &pads, Some(&pad_group)) {
                if covered(x) && covered(y) {
                    left_to_pour += 1;
                    continue;
                }
                conns.push(Conn {
                    net: ni,
                    net2: None,
                    a: Term::Pad(x),
                    b: Term::Pad(y),
                    a2: None,
                    b2: None,
                    width: nr.width,
                    clearance: nr.clearance,
                    kind: ConnKind::Signal,
                    priority: nr.priority,
                    est_len: b.pads[x].pos.dist(b.pads[y].pos),
                    couple: None,
                });
            }
            if left_to_pour > 0 {
                log::info!("net {}: {left_to_pour} connection(s) left to the pour", b.nets[ni].name);
            }
        }
        let _ = LayerRole::Signal;
        // Pads the pour has to reach: remember them so the topology can charge extra
        // for crossing their escape area. A geometric keepout would enclose the pad and
        // break the mesh; a cost keeps the same promise without touching the geometry.
        let mut pour_pads = Vec::new();
        if rules.trust_pours && rules.pour_keepout > 0.0 {
            for pd in b.pads.iter() {
                let net = match pd.net {
                    Some(n) => n,
                    None => continue,
                };
                if pour_covers(b, net, pd) {
                    let pr = pd.shape.pts.iter().map(|q| q.dist(pd.pos)).fold(0.0, f64::max) + pd.shape.r;
                    pour_pads.push((pd.pos, pr + rules.pour_keepout));
                }
            }
        }
        Problem { obstacles, outline, sites, conns, route_layers, bbox, plane_nets, pad_group, pour_pads, escape }
    }
}

/// A pad is joined by a same-net pour when the filled copper actually reaches it:
/// inside the fill, or within a thermal spoke of it. The zone *outline* is not enough —
/// a pour cut by tracks may pass near a pad without touching it.
fn pour_covers(b: &Board, net: NetId, pd: &Pad) -> bool {
    b.zones.iter().any(|z| {
        if z.keepout.is_some() || z.net != Some(net) {
            return false;
        }
        z.fills.iter().any(|(l, poly)| {
            pd.layers.contains(l)
                && poly.len() >= 3
                && (point_in_poly(pd.pos, poly)
                    || poly
                        .iter()
                        .zip(poly.iter().cycle().skip(1))
                        .take(poly.len())
                        .any(|(a, c)| pd.shape.seg_dist(*a, *c) <= 0.6))
        })
    })
}

/// Euclidean MST over pads (Prim), with a penalty for pad pairs without a common layer.
fn mst(b: &Board, rules: &Rules, pads: &[usize]) -> Vec<(usize, usize)> {
    mst_grouped(b, rules, pads, None)
}

fn mst_grouped(b: &Board, rules: &Rules, pads: &[usize], group: Option<&[usize]>) -> Vec<(usize, usize)> {
    let n = pads.len();
    let same = |i: usize, j: usize| group.map_or(false, |g| g[pads[i]] == g[pads[j]]);
    let cost = |i: usize, j: usize| {
        if same(i, j) {
            return 0.0;
        }
        let a = &b.pads[pads[i]];
        let c = &b.pads[pads[j]];
        let common = a.layers.iter().any(|l| c.layers.contains(l) && rules.layers[*l].role == LayerRole::Signal);
        a.pos.dist(c.pos) + if common { 0.0 } else { rules.via_cost }
    };
    let mut in_tree = vec![false; n];
    let mut best = vec![(f64::INFINITY, usize::MAX); n];
    in_tree[0] = true;
    for j in 1..n {
        best[j] = (cost(0, j), 0);
    }
    let mut edges = Vec::new();
    for _ in 1..n {
        let mut k = usize::MAX;
        let mut kd = f64::INFINITY;
        for j in 0..n {
            if !in_tree[j] && best[j].0 < kd {
                kd = best[j].0;
                k = j;
            }
        }
        if k == usize::MAX {
            break;
        }
        in_tree[k] = true;
        if !same(best[k].1, k) {
            edges.push((pads[best[k].1], pads[k]));
        }
        for j in 0..n {
            if !in_tree[j] {
                let c = cost(k, j);
                if c < best[j].0 {
                    best[j] = (c, k);
                }
            }
        }
    }
    edges
}

/// Differential pair: both nets are routed single-ended along their own MST; the N
/// net is attracted to the P net in the topology and locked to it in the geometry.
fn plan_pair(b: &Board, rules: &Rules, pi: usize, conns: &mut Vec<Conn>) {
    let dp = &rules.diff_pairs[pi];
    for (net, leader) in [(dp.p, true), (dp.n, false)] {
        let pads: Vec<usize> = (0..b.pads.len()).filter(|&i| b.pads[i].net == Some(net) && b.pads[i].kind != PadKind::Npth && !b.pads[i].layers.is_empty()).collect();
        if pads.len() < 2 {
            continue;
        }
        for (x, y) in mst(b, rules, &pads) {
            conns.push(Conn {
                net,
                net2: None,
                a: Term::Pad(x),
                b: Term::Pad(y),
                a2: None,
                b2: None,
                width: dp.width,
                clearance: rules.nets[net].clearance,
                kind: ConnKind::Signal,
                priority: rules.nets[net].priority + if leader { 12 } else { 11 },
                est_len: b.pads[x].pos.dist(b.pads[y].pos),
                couple: Some((pi, leader)),
            });
        }
    }
}

/// Straight exit stubs for fine-pitch, elongated SMD pads of ICs: the track leaves the
/// pad along its long axis, away from the part centre, before it may turn (keeps
/// diagonal escapes from grazing the neighbouring pins).
fn pad_escapes(b: &Board, rules: &Rules, obstacles: &[Obstacle]) -> Vec<Option<(P, f64)>> {
    let mut out: Vec<Option<(P, f64)>> = vec![None; b.pads.len()];
    let mut per_fp = vec![0usize; b.footprints.len()];
    for pd in &b.pads {
        per_fp[pd.footprint] += 1;
    }
    let mut stubs: Vec<(RoundPoly, Option<NetId>, u64)> = Vec::new();
    for (i, pd) in b.pads.iter().enumerate() {
        if pd.kind != PadKind::Smd || pd.net.is_none() || per_fp[pd.footprint] < 6 || pd.shape.pts.len() < 3 {
            continue;
        }
        let net = pd.net.unwrap();
        if rules.nets[net].skip {
            continue;
        }
        // long axis of the pad
        let pts = &pd.shape.pts;
        let mut axis = crate::geom::p(1.0, 0.0);
        let mut best = 0.0;
        for k in 0..pts.len() {
            let d = pts[(k + 1) % pts.len()] - pts[k];
            if d.len() > best {
                best = d.len();
                axis = d.norm();
            }
        }
        let wmin = pd.shape.min_width();
        let half_len = pts.iter().map(|q| (*q - pd.pos).dot(axis).abs()).fold(0.0, f64::max) + pd.shape.r;
        if wmin > 0.3 || 2.0 * half_len < 1.4 * wmin {
            continue; // not a fine-pitch elongated pad
        }
        let away = pd.pos - b.footprints[pd.footprint].pos;
        if away.len() < 0.3 {
            continue; // centre / thermal pad
        }
        if away.dot(axis) < 0.0 {
            axis = -axis;
        }
        let w = rules.nets[net].width.min(wmin.max(b.rules.min_track_width));
        let clr = rules.nets[net].clearance;
        let e = pd.pos + axis * (half_len + clr * 0.5 + w);
        if !inside_board(e, &b.outline) {
            continue;
        }
        let stub = RoundPoly::capsule(pd.pos, e, w / 2.0);
        let lm = mask(&pd.layers);
        let clash = obstacles.iter().any(|o| o.on_mask(lm) && o.net != pd.net && o.kind != ObsKind::Keepout && stub.shape_dist(&o.shape) < rules.clr(pd.net, o.net) + 0.001)
            || stubs.iter().any(|(s2, n2, m2)| m2 & lm != 0 && *n2 != pd.net && stub.shape_dist(s2) < rules.clr(pd.net, *n2) + 0.001);
        if clash {
            continue;
        }
        stubs.push((stub, pd.net, lm));
        out[i] = Some((e, w));
    }
    out
}
