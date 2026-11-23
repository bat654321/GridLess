//! Topological (rubber-band) router.
//!
//! Each wire is a sequence of mesh edges it crosses on one layer; along every edge the
//! crossings are kept in a strict order, so the sketch on each layer is planar by
//! construction. A path search (A*) walks the states "crossing edge e in gap g" and
//! may cross existing wires of other nets only by paying a rip-up cost — the crossed
//! connections are then ripped up and re-queued (negotiated rip-up and reroute with
//! PathFinder-style present/history congestion costs on edge capacities).

use crate::board::{Board, NetId};
use crate::config::{Direction, Rules};
use crate::geom::{line_intersect, P};
use crate::mesh::{Mesh, NONE};
use crate::problem::{ConnKind, Problem, Term};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use crate::field::Field;

/// Extra room reserved per wire when checking edge capacity, so that the geometry
/// (which keeps its own small safety margin) never has to squeeze through a zero-slack gap.
const CAP_MARGIN: f64 = 0.012;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct XRef {
    pub wire: u32,
    pub k: u32,
}

#[derive(Clone, Debug)]
pub struct Wire {
    pub conn: u32,
    /// index into Router::meshes
    pub li: usize,
    pub start: u32,
    pub end: u32,
    pub cross: Vec<u32>,
    /// triangle of each chord (cross.len()+1 entries)
    pub tris: Vec<u32>,
    pub net: NetId,
    pub net2: Option<NetId>,
    pub width: f64,
    pub clr: f64,
    pub alive: bool,
    pub couple: Option<(usize, bool)>,
    /// width at each crossing (necked down below `width` in narrow gaps)
    pub xw: Vec<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct ConnState {
    pub wires: Vec<u32>,
    pub sites: Vec<u32>,
    pub routed: bool,
    pub ripups: u32,
    pub failed: bool,
}

#[derive(Clone)]
pub struct LayerState {
    pub edge_x: Vec<Vec<XRef>>,
    pub tri_ch: Vec<Vec<XRef>>,
}

#[derive(Clone)]
pub struct State {
    pub wires: Vec<Wire>,
    pub free_wires: Vec<u32>,
    pub conns: Vec<ConnState>,
    pub layers: Vec<LayerState>,
    /// via site -> (net, number of users)
    pub site_use: Vec<Option<(NetId, u32)>>,
}

pub struct Router<'a> {
    pub board: &'a Board,
    pub pb: &'a Problem,
    pub rules: &'a Rules,
    pub meshes: Vec<Mesh>,
    pub st: State,
    pub via_r: f64,
    pub stats: RouteStats,
    pub queue: VecDeque<usize>,
    /// cost knobs updated by the negotiation loop
    pres_fac: f64,
    /// multiplier of the via cost (raised by the via-minimisation pass)
    pub via_scale: f64,
    /// after the first routing pass differential pairs are kept where they are if possible
    pub protect_pairs: bool,
    /// PathFinder history cost per layer and edge; survives state restores
    pub hist: Vec<Vec<f32>>,
    /// per connection: room kept free on each side of the track (length matching
    /// reserve); the topology and the geometry treat the wire as this much wider
    pub reserve: Vec<f64>,
    /// extra cost multiplier near pads the pour has to reach, per layer and edge
    pour_e: Vec<Vec<f32>>,
    /// the same for mesh vertices
    pour_v: Vec<Vec<f32>>,
    rip_base: f64,
    /// temporary multiplier of the layer direction weight (detour clean-up relaxes it)
    dir_scale: f64,
    /// distance field to copper per layer (empty when the ridge pull is off)
    field: Vec<Field>,
    /// contour map (None when off)
    terrain: Option<crate::terrain::Terrain>,
}

/// How many millimetres of ordinary track one millimetre through a pour pad's escape
/// area is worth. High enough to send a signal around, not so high that it fails.
const POUR_COST: f64 = 16.0;

fn pour_penalty(pads: &[(P, f64)], q: P) -> f32 {
    let mut m = 0.0f64;
    for (p, r) in pads {
        let d = q.dist(*p);
        if d < *r {
            m = m.max(1.0 - d / *r);
        }
    }
    m as f32
}

#[derive(Clone, Debug, Default, serde::Serialize)]
pub struct RouteStats {
    pub searches: usize,
    pub expansions: usize,
    pub ripups: usize,
    pub iterations: usize,
    pub unrouted: usize,
    pub overflow: f64,
    pub vias: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum SKey {
    Vert(u8, u32),
    Edge(u8, u32, u32, u32),
}

#[derive(Clone, Debug)]
struct Node {
    key: SKey,
    g: f64,
    pos: P,
    parent: u32,
    /// triangle of the chord leading into this node (NONE for via transitions / start)
    tri: u32,
    /// gap index on the edge for Edge nodes
    crossed: Vec<u32>,
    /// width used at this crossing
    xw: f64,
}

#[derive(Clone, Copy)]
struct Open {
    f: f64,
    id: u32,
}
impl PartialEq for Open {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f
    }
}
impl Eq for Open {}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

/// One step of a found path.
#[derive(Clone, Debug)]
pub enum Step {
    Vert { li: usize, v: u32 },
    Cross { li: usize, e: u32, gap: u32, tri: u32, xw: f64 },
}

pub struct Found {
    steps: Vec<(Step, u32)>, // step + triangle of chord leading into it
    crossed: Vec<u32>,
    cost: f64,
}

fn chord_cross(a1: f64, b1: f64, a2: f64, b2: f64) -> bool {
    let (lo, hi) = if a1 < b1 { (a1, b1) } else { (b1, a1) };
    let eps = 1e-9;
    let inside = |x: f64| x > lo + eps && x < hi - eps;
    let outside = |x: f64| x < lo - eps || x > hi + eps;
    (inside(a2) && outside(b2)) || (inside(b2) && outside(a2))
}

impl<'a> Router<'a> {
    pub fn new(board: &'a Board, pb: &'a Problem, rules: &'a Rules) -> Router<'a> {
        let meshes: Vec<Mesh> = pb.route_layers.iter().map(|&l| Mesh::build(pb, l, rules.default_clearance, rules.edge_clearance)).collect();
        let layers = meshes
            .iter()
            .map(|m| LayerState { edge_x: vec![Vec::new(); m.edges.len()], tri_ch: vec![Vec::new(); m.tris.len()] })
            .collect();
        let hist = meshes.iter().map(|m| vec![0.0f32; m.edges.len()]).collect();
        let pour_e = meshes
            .iter()
            .map(|m| {
                m.edges
                    .iter()
                    .map(|e| pour_penalty(&pb.pour_pads, (m.verts[e.v[0] as usize].pos + m.verts[e.v[1] as usize].pos) / 2.0))
                    .collect()
            })
            .collect();
        let pour_v = meshes.iter().map(|m| m.verts.iter().map(|v| pour_penalty(&pb.pour_pads, v.pos)).collect()).collect();
        let field: Vec<Field> = if rules.ridge > 0.0 || rules.ridge_cost > 0.0 {
            meshes.iter().map(|m| Field::from_mesh(m, rules.ridge_step)).collect()
        } else {
            Vec::new()
        };
        let st = State { wires: Vec::new(), free_wires: Vec::new(), conns: vec![ConnState::default(); pb.conns.len()], layers, site_use: vec![None; pb.sites.len()] };
        let terrain = if rules.terrain > 0.0 {
            let t = crate::terrain::Terrain::build(pb, &pb.route_layers, rules.default_clearance, rules.terrain_sigma);
            if let Ok(prefix) = std::env::var("GRIDLESS_TERRAIN_PNG") {
                for li in 0..pb.route_layers.len() {
                    let name = &board.layers[pb.route_layers[li]].name;
                    let _ = t.save_png(li, std::path::Path::new(&format!("{prefix}_{name}.png")), 16.0);
                }
            }
            Some(t)
        } else {
            None
        };
        Router { board, pb, rules, meshes, st, via_r: rules.via_dia / 2.0, stats: RouteStats::default(), queue: VecDeque::new(), pres_fac: 1.0, rip_base: 25.0, via_scale: 1.0, protect_pairs: false, hist, reserve: vec![0.0; pb.conns.len()], pour_e, pour_v, dir_scale: 1.0, field, terrain }
    }

    /// Climbing cost of the step a-b on layer li for a wire of `net` (contour map).
    fn terrain_cost(&self, li: usize, a: P, b: P, net: NetId) -> f64 {
        match &self.terrain {
            Some(t) => self.rules.terrain * a.dist(b) * t.mean(li, a, b, Some(net)),
            None => 0.0,
        }
    }

    /// Cost of a via at q (mean terrain height of all routing layers there).
    fn terrain_via_cost(&self, q: P, net: NetId) -> f64 {
        match &self.terrain {
            Some(t) => {
                let n = self.meshes.len().max(1);
                self.rules.terrain_via * (0..self.meshes.len()).map(|li| t.at(li, q, Some(net))).sum::<f64>() / n as f64
            }
            None => 0.0,
        }
    }

    // ------------------------------------------------------------------ helpers

    fn conn_nets(&self, c: usize) -> (NetId, Option<NetId>) {
        let cn = &self.pb.conns[c];
        (cn.net, cn.net2)
    }

    fn own(&self, net: Option<NetId>, c: usize) -> bool {
        let (a, b) = self.conn_nets(c);
        net.is_some() && (net == Some(a) || net == b)
    }

    /// Net that owns mesh vertex v (obstacle net, or via site owner).
    fn vert_net(&self, li: usize, v: u32) -> Option<NetId> {
        let mv = &self.meshes[li].verts[v as usize];
        if let Some(o) = mv.obs {
            return self.meshes[li].obs[o as usize].net;
        }
        if let Some(s) = mv.site {
            return self.st.site_use[s as usize].map(|x| x.0);
        }
        None
    }

    /// Clearance-inflated extent of the obstacle at endpoint k of edge e (dynamic for vias).
    fn edge_exit(&self, li: usize, e: u32, k: usize) -> f64 {
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        let v = &m.verts[ed.v[k] as usize];
        if let Some(s) = v.site {
            if self.st.site_use[s as usize].is_some() {
                return self.via_r + self.rules.default_clearance;
            }
        }
        ed.exit[k]
    }

    /// Cyclic boundary position (0..3) inside triangle t of the crossing at index j of edge
    /// e (n = number of crossings on the edge, j may be fractional for gaps).
    fn bpos(&self, li: usize, t: u32, e: u32, j: f64, n: usize) -> f64 {
        let m = &self.meshes[li];
        let tri = &m.tris[t as usize];
        let le = tri.edge_idx(e);
        let fwd = m.edges[e as usize].v[0] == tri.v[le];
        let n1 = (n + 1) as f64;
        if fwd {
            le as f64 + (j + 1.0) / n1
        } else {
            le as f64 + (n as f64 - j) / n1
        }
    }

    fn gap_bpos(&self, li: usize, t: u32, e: u32, gap: u32) -> f64 {
        let n = self.st.layers[li].edge_x[e as usize].len();
        self.bpos(li, t, e, gap as f64 - 0.5, n)
    }

    fn xref_index(&self, li: usize, e: u32, x: XRef) -> usize {
        self.st.layers[li].edge_x[e as usize].iter().position(|y| *y == x).expect("xref missing")
    }

    /// Boundary positions of an existing chord (wire w, chord k) in triangle t.
    fn chord_bpos(&self, li: usize, t: u32, ch: XRef) -> (f64, f64) {
        let w = &self.st.wires[ch.wire as usize];
        let tri = &self.meshes[li].tris[t as usize];
        let k = ch.k as usize;
        let a = if k == 0 {
            tri.vert_idx(w.start).map(|i| i as f64).unwrap_or(0.0)
        } else {
            let e = w.cross[k - 1];
            let j = self.xref_index(li, e, XRef { wire: ch.wire, k: (k - 1) as u32 });
            self.bpos(li, t, e, j as f64, self.st.layers[li].edge_x[e as usize].len())
        };
        let b = if k == w.cross.len() {
            tri.vert_idx(w.end).map(|i| i as f64).unwrap_or(0.0)
        } else {
            let e = w.cross[k];
            let j = self.xref_index(li, e, XRef { wire: ch.wire, k: k as u32 });
            self.bpos(li, t, e, j as f64, self.st.layers[li].edge_x[e as usize].len())
        };
        (a, b)
    }

    /// Wires (other conns) crossed by a new chord (pa,pb) in triangle t.
    fn crossings(&self, li: usize, t: u32, pa: f64, pb: f64, c: usize, out: &mut Vec<u32>) -> bool {
        out.clear();
        for ch in &self.st.layers[li].tri_ch[t as usize] {
            let (a, b) = self.chord_bpos(li, t, *ch);
            if chord_cross(pa, pb, a, b) {
                let w = &self.st.wires[ch.wire as usize];
                if w.conn as usize == c {
                    continue;
                }
                if self.own(Some(w.net), c) {
                    continue; // same net: copper may merge
                }
                if !out.contains(&ch.wire) {
                    out.push(ch.wire);
                }
            }
        }
        true
    }

    fn tri_allowed(&self, li: usize, t: u32, c: usize) -> bool {
        if t == NONE {
            return false;
        }
        let tri = &self.meshes[li].tris[t as usize];
        if !tri.alive {
            return false;
        }
        match tri.owner {
            None => true,
            Some(crate::mesh::BLOCKED) => false,
            Some(o) => self.obs_open(li, o, c),
        }
    }

    fn edge_crossable(&self, li: usize, e: u32, c: usize) -> bool {
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        let a = &m.verts[ed.v[0] as usize];
        let b = &m.verts[ed.v[1] as usize];
        if a.outline && b.outline && ed.constraint {
            return false;
        }
        if let (Some(oa), Some(ob)) = (a.obs, b.obs) {
            if oa == ob && !self.obs_open(li, oa, c) {
                return false;
            }
        }
        if let Some(o) = ed.owner {
            if !self.obs_open(li, o, c) {
                return false;
            }
        }
        true
    }

    /// May connection c pass over obstacle o? Only its own copper — and not a pad that
    /// has an escape stub (that pad is reached through the stub's end only).
    fn obs_open(&self, li: usize, o: u32, c: usize) -> bool {
        let ob = &self.meshes[li].obs[o as usize];
        if !self.own(ob.net, c) {
            return false;
        }
        match ob.kind {
            crate::problem::ObsKind::Pad(p) => self.pb.escape[p].is_none(),
            crate::problem::ObsKind::Escape(_) => false,
            _ => true,
        }
    }

    /// Bit mask of routing layers on which wires of `net` (other than connection `skip`)
    /// end at pad `pad`.
    fn pad_layers(&self, net: NetId, pad: u32, skip: usize) -> u32 {
        let mut m = 0u32;
        for w in self.st.wires.iter().filter(|w| w.alive && w.net == net && w.conn as usize != skip) {
            let mesh = &self.meshes[w.li];
            if mesh.verts[w.start as usize].pad == Some(pad) || mesh.verts[w.end as usize].pad == Some(pad) {
                m |= 1 << w.li;
            }
        }
        m
    }

    /// Cost of entering / leaving a through-hole pad on layer li when the net already
    /// uses that pad on other layers only (an unnecessary layer change).
    fn layer_switch_pen(&self, li: usize, v: u32, c: usize) -> f64 {
        let pad = match self.meshes[li].verts[v as usize].pad {
            Some(p) => p,
            None => return 0.0,
        };
        if self.board.pads[pad as usize].layers.len() < self.board.layers.len() {
            return 0.0; // SMD: the layer is fixed anyway
        }
        let m = self.pad_layers(self.pb.conns[c].net, pad, c);
        if m != 0 && m & (1 << li) == 0 {
            0.5 * self.rules.via_cost
        } else {
            0.0
        }
    }

    /// Centre distance of the two vias of a differential pair: the pair pitch, or the
    /// via-to-via and hole-to-hole spacing if that is larger.
    pub fn pair_via_pitch(&self, pi: usize) -> f64 {
        let dp = &self.rules.diff_pairs[pi];
        let clr = self.rules.clr(Some(dp.p), Some(dp.n));
        (dp.width + dp.gap).max(self.rules.via_dia + clr + 0.01).max(self.rules.via_drill + self.rules.hole_to_hole + 0.01)
    }

    /// For a differential pair follower: where its vias belong, beside each via of its
    /// leader, square to the leader's direction there. (leader site, one side, other side)
    pub fn pair_via_targets(&self, c: usize) -> Vec<(u32, P, P)> {
        let (pi, leader) = match self.pb.conns[c].couple {
            Some(x) => x,
            None => return Vec::new(),
        };
        if leader {
            return Vec::new();
        }
        let parts = |k: usize| -> Option<(usize, usize)> {
            match (self.pb.conns[k].a, self.pb.conns[k].b) {
                (Term::Pad(x), Term::Pad(y)) => {
                    let (fa, fb) = (self.board.pads[x].footprint, self.board.pads[y].footprint);
                    Some((fa.min(fb), fa.max(fb)))
                }
                _ => None,
            }
        };
        let key = parts(c);
        let pitch = self.pair_via_pitch(pi);
        let mut out = Vec::new();
        for k in 0..self.pb.conns.len() {
            if self.pb.conns[k].couple != Some((pi, true)) || parts(k) != key || !self.st.conns[k].routed {
                continue;
            }
            let ws = &self.st.conns[k].wires;
            for i in 0..ws.len().saturating_sub(1) {
                let (a, b) = (&self.st.wires[ws[i] as usize], &self.st.wires[ws[i + 1] as usize]);
                let site = match self.meshes[a.li].verts[a.end as usize].site {
                    Some(s) if self.meshes[b.li].verts[b.start as usize].site == Some(s) => s,
                    _ => continue,
                };
                // direction of the pair through the via, from the anchors it runs between
                // (embedded crossings are only rough positions on the mesh edges)
                let at = self.pb.sites[site as usize].pos;
                let prev = self.meshes[a.li].verts[a.start as usize].pos;
                let next = self.meshes[b.li].verts[b.end as usize].pos;
                let din = (at - prev).norm();
                let dout = (next - at).norm();
                let d = if (din + dout).len() > 1e-6 { (din + dout).norm() } else { din };
                let n = d.perp();
                out.push((site, at + n * pitch, at - n * pitch));
            }
        }
        out
    }

    /// Exact spots for the follower vias of the differential pairs: each follower via that
    /// pairs up (in order along the route) with a leader via goes to the target on its own
    /// side. (follower site, position)
    pub fn pair_via_spots(&self) -> Vec<(u32, P)> {
        let mut out = Vec::new();
        for c in 0..self.pb.conns.len() {
            if !matches!(self.pb.conns[c].couple, Some((_, false))) || !self.st.conns[c].routed {
                continue;
            }
            let targets = self.pair_via_targets(c);
            let mine: Vec<u32> = {
                let ws = &self.st.conns[c].wires;
                (0..ws.len().saturating_sub(1))
                    .filter_map(|i| {
                        let (a, b) = (&self.st.wires[ws[i] as usize], &self.st.wires[ws[i + 1] as usize]);
                        match self.meshes[a.li].verts[a.end as usize].site {
                            Some(s) if self.meshes[b.li].verts[b.start as usize].site == Some(s) => Some(s),
                            _ => None,
                        }
                    })
                    .collect()
            };
            if mine.len() != targets.len() {
                continue;
            }
            for (&s, &(_, t1, t2)) in mine.iter().zip(&targets) {
                let q = self.pb.sites[s as usize].pos;
                out.push((s, if q.dist(t1) <= q.dist(t2) { t1 } else { t2 }));
            }
        }
        out
    }

    /// For a differential pair follower: mask of the layers its leader uses.
    fn leader_layers(&self, c: usize) -> u32 {
        let (pi, leader) = match self.pb.conns[c].couple {
            Some(x) => x,
            None => return 0,
        };
        if leader {
            return 0;
        }
        let parts = |k: usize| -> Option<(usize, usize)> {
            match (self.pb.conns[k].a, self.pb.conns[k].b) {
                (Term::Pad(x), Term::Pad(y)) => {
                    let (fa, fb) = (self.board.pads[x].footprint, self.board.pads[y].footprint);
                    Some((fa.min(fb), fa.max(fb)))
                }
                _ => None,
            }
        };
        let key = parts(c);
        let mut m = 0u32;
        for k in 0..self.pb.conns.len() {
            if self.pb.conns[k].couple == Some((pi, true)) && parts(k) == key && self.st.conns[k].routed {
                for &w in &self.st.conns[k].wires {
                    m |= 1 << self.st.wires[w as usize].li;
                }
            }
        }
        m
    }

    /// Interval (lo, hi) of feasible centre positions (distance from v0) for a new wire of
    /// width w in gap `gap`, and the overflow amount.
    /// Net of the copper an edge lies in: both ends on the same obstacle (a pad side or a
    /// diagonal inside a pad). Wires of that net cross it freely — it is their own copper.
    fn edge_copper_net(&self, li: usize, e: u32) -> Option<NetId> {
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        match (m.verts[ed.v[0] as usize].obs, m.verts[ed.v[1] as usize].obs) {
            (Some(a), Some(b)) if a == b => m.obs[a as usize].net,
            _ => ed.owner.and_then(|o| m.obs[o as usize].net),
        }
    }

    fn gap_interval(&self, li: usize, e: u32, gap: u32, w: f64, clr: f64, c: usize) -> (f64, f64, f64) {
        let ed = &self.meshes[li].edges[e as usize];
        if self.own(self.edge_copper_net(li, e), c) {
            return (0.0, ed.len, 0.0);
        }
        let xs = &self.st.layers[li].edge_x[e as usize];
        let dclr = self.rules.default_clearance;
        // width consumed by the obstacle at end k plus wires `ws` packed against it,
        // up to (and including) the separation before the new wire
        let packed = |k: usize, ws: &mut dyn Iterator<Item = &XRef>| -> f64 {
            let mut used = 0.0;
            let mut prev_net = self.vert_net(li, ed.v[k]);
            let mut prev_clr = dclr;
            let mut first = true;
            for x in ws {
                let wi = &self.st.wires[x.wire as usize];
                let same = prev_net.is_some() && prev_net == Some(wi.net);
                if !same {
                    used += if first { self.edge_exit(li, e, k) + (wi.clr - dclr).max(0.0) } else { prev_clr.max(wi.clr) };
                }
                used += wi.xw[x.k as usize] + CAP_MARGIN;
                prev_net = Some(wi.net);
                prev_clr = wi.clr;
                first = false;
            }
            if !self.own(prev_net, c) {
                used += if first { self.edge_exit(li, e, k) + (clr - dclr).max(0.0) } else { prev_clr.max(clr) };
            }
            used + (w + CAP_MARGIN) / 2.0
        };
        let lo = packed(0, &mut xs[..gap as usize].iter());
        let hi = ed.len - packed(1, &mut xs[gap as usize..].iter().rev());
        let over = (lo - hi).max(0.0);
        (lo, hi, over)
    }

    /// Gap interval for a wire of nominal width `w`; narrows down to `wmin` if needed.
    /// Returns (lo, hi, overflow, width used, width reduction).
    fn fit_gap(&self, li: usize, e: u32, gap: u32, w: f64, wmin: f64, clr: f64, c: usize) -> (f64, f64, f64, f64, f64) {
        let (lo, hi, over) = self.gap_interval(li, e, gap, w, clr, c);
        if over <= 0.0 || wmin >= w {
            return (lo, hi, over, w, 0.0);
        }
        let need = (w - over).max(wmin);
        let (lo2, hi2, over2) = self.gap_interval(li, e, gap, need, clr, c);
        (lo2, hi2, over2, need, w - need)
    }

    /// Current overflow of an edge (mm) for the committed wires.
    pub fn edge_overflow(&self, li: usize, e: u32) -> f64 {
        let xs = &self.st.layers[li].edge_x[e as usize];
        if xs.is_empty() {
            return 0.0;
        }
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        let n0 = self.vert_net(li, ed.v[0]);
        let n1 = self.vert_net(li, ed.v[1]);
        // edges inside a net's own copper: only that net crosses them, no capacity limit
        if let Some(cn) = self.edge_copper_net(li, e) {
            if xs.iter().all(|x| self.st.wires[x.wire as usize].net == cn) {
                return 0.0;
            }
        }
        let mut need = 0.0;
        let mut prev = n0;
        for (i, x) in xs.iter().enumerate() {
            let w = &self.st.wires[x.wire as usize];
            if i == 0 {
                if prev != Some(w.net) {
                    need += self.edge_exit(li, e, 0);
                }
            } else if prev != Some(w.net) {
                need += w.clr;
            }
            need += w.xw[x.k as usize] + CAP_MARGIN;
            prev = Some(w.net);
        }
        if prev != n1 {
            need += self.edge_exit(li, e, 1);
        }
        (need - ed.len).max(0.0)
    }

    fn dir_factor(&self, li: usize, a: P, b: P) -> f64 {
        let lr = &self.rules.layers[self.pb.route_layers[li]];
        let d = b - a;
        let l = d.len();
        if l < 1e-9 {
            return lr.cost;
        }
        let off = match lr.dir {
            Direction::Any => 0.0,
            Direction::Horizontal => d.y.abs() / l,
            Direction::Vertical => d.x.abs() / l,
        };
        // gentle preference for 0/45/90 degree paths: the heuristic stays the plain
        // distance, so a cost >= distance keeps A* admissible
        let mut dev = d.y.atan2(d.x).to_degrees() % 45.0;
        if dev < 0.0 {
            dev += 45.0;
        }
        let dev45 = dev.min(45.0 - dev) / 22.5;
        lr.cost * (1.0 + lr.dir_weight * self.dir_scale * off + self.rules.angle_cost * dev45)
    }

    /// Cost factor for being close to copper: 0 on the ridge of the free space,
    /// `ridge` right against a boundary. The path drifts to the middle of a corridor.
    fn ridge_fac(&self, li: usize, q: P) -> f64 {
        if self.rules.ridge_cost <= 0.0 || self.field.is_empty() {
            return 0.0;
        }
        let d = self.field[li].at(q);
        self.rules.ridge_cost * (1.0 - (d / self.rules.ridge_range.max(1e-6)).clamp(0.0, 1.0))
    }

    fn wire_cost(&self, w: u32) -> f64 {
        let c = self.st.wires[w as usize].conn as usize;
        let r = self.st.conns[c].ripups as f64;
        // differential pairs are expensive to displace (much more so once placed)
        let pri = if self.pb.conns[c].couple.is_some() || matches!(self.pb.conns[c].kind, ConnKind::Pair { .. }) {
            if self.protect_pairs {
                30.0
            } else {
                3.0
            }
        } else {
            1.0
        };
        self.rip_base * (1.0 + r * 0.7) * pri
    }

    // ------------------------------------------------------------------ connectivity

    /// Mesh-independent node id for connectivity: pads are 0.., sites are offset.
    fn comp_nodes(&self, net: NetId, net2: Option<NetId>) -> (Vec<u64>, HashMap<u64, u64>) {
        // union-find over pads and used sites of the net(s)
        let mut parent: HashMap<u64, u64> = HashMap::new();
        fn find(p: &mut HashMap<u64, u64>, x: u64) -> u64 {
            let mut r = x;
            while let Some(&q) = p.get(&r) {
                if q == r {
                    break;
                }
                r = q;
            }
            let mut y = x;
            while let Some(&q) = p.get(&y) {
                if q == r {
                    break;
                }
                p.insert(y, r);
                y = q;
            }
            r
        }
        let nets = [Some(net), net2];
        let mut nodes = Vec::new();
        for (i, pd) in self.board.pads.iter().enumerate() {
            if pd.net.is_some() && nets.contains(&pd.net) {
                nodes.push(i as u64);
                parent.insert(i as u64, i as u64);
            }
        }
        // pads already joined by fixed copper
        for &x in &nodes.clone() {
            let g = self.pb.pad_group[x as usize] as u64;
            if g != x && parent.contains_key(&g) {
                let a = find(&mut parent, x);
                let b = find(&mut parent, g);
                parent.insert(a, b);
            }
        }
        let site_base = 1u64 << 32;
        for (s, u) in self.st.site_use.iter().enumerate() {
            if let Some((n, _)) = u {
                if nets.contains(&Some(*n)) {
                    nodes.push(site_base + s as u64);
                    parent.insert(site_base + s as u64, site_base + s as u64);
                }
            }
        }
        // plane: all THT pads and vias of a plane net are connected
        let plane = self.rules.nets[net].plane.is_some();
        if plane {
            let tht: Vec<u64> = nodes.iter().copied().filter(|&x| x >= site_base || self.board.pads[x as usize].layers.len() == self.board.layers.len()).collect();
            for w in tht.windows(2) {
                let a = find(&mut parent, w[0]);
                let b = find(&mut parent, w[1]);
                parent.insert(a, b);
            }
        }
        for (ci, cs) in self.st.conns.iter().enumerate() {
            if !cs.routed || self.pb.conns[ci].net != net {
                continue;
            }
            let mut ends: Vec<u64> = Vec::new();
            for &wi in &cs.wires {
                let w = &self.st.wires[wi as usize];
                for v in [w.start, w.end] {
                    let mv = &self.meshes[w.li].verts[v as usize];
                    if let Some(p) = mv.pad {
                        ends.push(p as u64);
                    } else if let Some(s) = mv.site {
                        ends.push(site_base + s as u64);
                    }
                }
            }
            for &s in &cs.sites {
                ends.push(site_base + s as u64);
            }
            for w in ends.windows(2) {
                if parent.contains_key(&w[0]) && parent.contains_key(&w[1]) {
                    let a = find(&mut parent, w[0]);
                    let b = find(&mut parent, w[1]);
                    parent.insert(a, b);
                }
            }
        }
        let mut comp = HashMap::new();
        for &x in &nodes {
            let r = find(&mut parent, x);
            comp.insert(x, r);
        }
        (nodes, comp)
    }

    /// (layer, vertex) sets for both ends of a connection, or None if already connected.
    fn endpoints(&self, c: usize) -> Option<(Vec<(usize, u32)>, Vec<(usize, u32)>, bool)> {
        let cn = &self.pb.conns[c];
        let (nodes, comp) = self.comp_nodes(cn.net, None);
        let site_base = 1u64 << 32;
        let to_verts = |set: &[u64]| -> Vec<(usize, u32)> {
            let mut v = Vec::new();
            for &x in set {
                for (li, m) in self.meshes.iter().enumerate() {
                    let mv = if x >= site_base { m.site_vert.get(&((x - site_base) as u32)) } else { m.pad_vert.get(&(x as u32)) };
                    if let Some(&mv) = mv {
                        v.push((li, mv));
                    }
                }
            }
            v
        };
        let pa = match cn.a {
            Term::Pad(p) => p as u64,
            Term::AnySite => return None,
        };
        let ra = comp.get(&pa).copied()?;
        let set_a: Vec<u64> = nodes.iter().copied().filter(|x| comp.get(x) == Some(&ra)).collect();
        match cn.b {
            Term::Pad(p) => {
                let rb = comp.get(&(p as u64)).copied()?;
                if ra == rb {
                    return None;
                }
                if matches!(cn.kind, ConnKind::Pair { .. }) {
                    // bands go pad-to-pad (no tree joining)
                    return Some((to_verts(&[pa]), to_verts(&[p as u64]), false));
                }
                let set_b: Vec<u64> = nodes.iter().copied().filter(|x| comp.get(x) == Some(&rb)).collect();
                Some((to_verts(&set_a), to_verts(&set_b), false))
            }
            Term::AnySite => {
                // fanout: done if the pad's component touches the plane (THT pad or via)
                if self.rules.nets[cn.net].plane.is_some() {
                    let touches = set_a.iter().any(|&x| x >= site_base || self.board.pads[x as usize].layers.len() == self.board.layers.len());
                    if touches {
                        return None;
                    }
                    let plane_set: Vec<u64> = nodes.iter().copied().filter(|&x| x >= site_base || self.board.pads[x as usize].layers.len() == self.board.layers.len()).collect();
                    return Some((to_verts(&[pa]), to_verts(&plane_set), true));
                }
                None
            }
        }
    }

    /// Dynamic terminals: all vertices of the net that are not in the source
    /// component. Returns None when the source already covers the whole net, or
    /// when the connection is not a plain pad-to-pad one.
    fn endpoints_any(&self, c: usize) -> Option<(Vec<(usize, u32)>, Vec<(usize, u32)>, bool)> {
        let cn = &self.pb.conns[c];
        if !matches!(cn.b, Term::Pad(_)) || matches!(cn.kind, ConnKind::Pair { .. }) {
            return None;
        }
        let (nodes, comp) = self.comp_nodes(cn.net, None);
        let site_base = 1u64 << 32;
        let to_verts = |set: &[u64]| -> Vec<(usize, u32)> {
            let mut v = Vec::new();
            for &x in set {
                for (li, m) in self.meshes.iter().enumerate() {
                    let mv = if x >= site_base { m.site_vert.get(&((x - site_base) as u32)) } else { m.pad_vert.get(&(x as u32)) };
                    if let Some(&mv) = mv {
                        v.push((li, mv));
                    }
                }
            }
            v
        };
        let pa = match cn.a {
            Term::Pad(p) => p as u64,
            Term::AnySite => return None,
        };
        let ra = comp.get(&pa).copied()?;
        let set_a: Vec<u64> = nodes.iter().copied().filter(|x| comp.get(x) == Some(&ra)).collect();
        let set_b: Vec<u64> = nodes.iter().copied().filter(|x| comp.get(x) != Some(&ra)).collect();
        if set_b.is_empty() {
            return None;
        }
        Some((to_verts(&set_a), to_verts(&set_b), false))
    }

    fn site_usable(&self, s: u32, net: NetId) -> bool {
        if let Some((n, _)) = self.st.site_use[s as usize] {
            return n == net;
        }
        for &o in &self.pb.sites[s as usize].conflicts {
            if let Some((n, _)) = self.st.site_use[o] {
                if n != net {
                    return false;
                }
                return false; // avoid stacking vias of the same net too
            }
        }
        true
    }

    /// Extra cost for turning site s into a via: overflow it creates on all layers.
    fn via_penalty(&self, s: u32, c: usize) -> f64 {
        if self.st.site_use[s as usize].is_some() {
            return 0.0;
        }
        let mut pen = 0.0;
        for (li, m) in self.meshes.iter().enumerate() {
            let v = match m.site_vert.get(&s) {
                Some(&v) => v,
                None => continue,
            };
            for &t in &m.vtris[v as usize] {
                for &e in &m.tris[t as usize].e {
                    let ed = &m.edges[e as usize];
                    if ed.v[0] != v && ed.v[1] != v {
                        continue;
                    }
                    let xs = &self.st.layers[li].edge_x[e as usize];
                    if xs.is_empty() {
                        continue;
                    }
                    let need: f64 = xs.iter().map(|x| self.st.wires[x.wire as usize].width + self.st.wires[x.wire as usize].clr).sum::<f64>() + self.via_r + self.rules.default_clearance;
                    let k = if ed.v[0] == v { 1 } else { 0 };
                    let avail = ed.len - self.edge_exit(li, e, k);
                    if need > avail {
                        // wires owned by other nets would have to move: count as soft cost
                        let other = xs.iter().any(|x| !self.own(Some(self.st.wires[x.wire as usize].net), c));
                        if other {
                            pen += (need - avail) * 40.0 * self.pres_fac;
                        }
                    }
                }
            }
        }
        pen
    }

    // ------------------------------------------------------------------ search

    fn best_point(&self, li: usize, e: u32, lo: f64, hi: f64, prev: P, goal: Option<P>) -> P {
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        let a = m.verts[ed.v[0] as usize].pos;
        let b = m.verts[ed.v[1] as usize].pos;
        let len = ed.len.max(1e-9);
        let (lo, hi) = if lo <= hi { (lo.max(0.0), hi.min(len)) } else { let mid = (0.5 * (lo + hi)).clamp(0.0, len); (mid, mid) };
        let lo = lo.min(len);
        let hi = hi.max(lo);
        let t = match goal {
            Some(g) => match line_intersect(a, b, prev, g) {
                Some((t, u)) if (0.0..=1.0).contains(&u) => t * len,
                _ => {
                    // closest point to the straight line: use projection of the midpoint of prev/goal
                    let q = prev.mid(g);
                    (q - a).dot(b - a) / len
                }
            },
            None => (prev - a).dot(b - a) / len,
        };
        let t = t.clamp(lo, hi);
        a + (b - a) * (t / len)
    }

    pub fn search(&mut self, c: usize, srcs: &[(usize, u32)], dsts: &[(usize, u32)], to_site: bool, max_exp: usize) -> Option<Found> {
        let cn = self.pb.conns[c].clone();
        let net = cn.net;
        let allow_via = !matches!(cn.kind, ConnKind::Pair { .. });
        let mut nodes: Vec<Node> = Vec::new();
        let mut open = BinaryHeap::new();
        let mut best: HashMap<SKey, f64> = HashMap::new();
        let dst_set: HashSet<(usize, u32)> = dsts.iter().copied().collect();
        let dst_pos: Vec<(usize, P)> = dsts.iter().map(|&(li, v)| (li, self.meshes[li].verts[v as usize].pos)).collect();
        let pair_member = cn.couple.is_some();
        // follower of a differential pair stays on its leader's layers, and changes layer
        // right beside the leader's vias (pair vias)
        let lead_mask = self.leader_layers(c);
        let via_targets: Vec<P> = self.pair_via_targets(c).into_iter().flat_map(|(_, a, b)| [a, b]).collect();
        let lay_k = |li: usize| -> f64 {
            if lead_mask != 0 && lead_mask & (1 << li) == 0 {
                6.0
            } else {
                1.0
            }
        };
        // penalties for changing layer at a through-hole pad already used by the net
        let mut end_pen: HashMap<(usize, u32), f64> = HashMap::new();
        for &(li, v) in srcs.iter().chain(dsts.iter()) {
            let pen = self.layer_switch_pen(li, v, c);
            if pen > 0.0 {
                end_pen.insert((li, v), pen);
            }
        }
        let hist_k = if pair_member { 0.2 } else { 1.0 };
        // A net whose pads all share a layer can be routed without any via. Make a
        // layer change expensive for such a connection, so it stays on its own layer
        // instead of escaping to the other side and coming back.
        let mut common: Option<Vec<crate::board::LayerId>> = None;
        for pd in self.board.pads.iter().filter(|p| p.net == Some(net)) {
            common = Some(match common {
                None => pd.layers.clone(),
                Some(c) => c.into_iter().filter(|l| pd.layers.contains(l)).collect(),
            });
        }
        let single_layer = common.as_ref().is_some_and(|c| !c.is_empty());
        let via_cost = self.rules.via_cost * self.via_scale * if pair_member { 5.0 } else { 1.0 }
            * if single_layer { self.rules.same_layer_bias } else { 1.0 };
        let h_scale = if matches!(cn.couple, Some((_, false))) { 0.35 } else { 1.0 };
        let h = |li: usize, q: P| -> f64 {
            if to_site {
                return 0.0;
            }
            let mut hb = f64::INFINITY;
            for &(dl, dp) in &dst_pos {
                let d = q.dist(dp) + if dl == li { 0.0 } else { via_cost };
                hb = hb.min(d);
            }
            hb * h_scale
        };
        let goal_near = |li: usize, q: P| -> Option<P> {
            let mut bp = None;
            let mut bd = f64::INFINITY;
            for &(dl, dp) in &dst_pos {
                let d = q.dist(dp) + if dl == li { 0.0 } else { 1.0 };
                if d < bd {
                    bd = d;
                    bp = Some(dp);
                }
            }
            bp
        };
        for &(li, v) in srcs {
            let pos = self.meshes[li].verts[v as usize].pos;
            let key = SKey::Vert(li as u8, v);
            let g0 = end_pen.get(&(li, v)).copied().unwrap_or(0.0);
            best.insert(key, g0);
            nodes.push(Node { key, g: g0, pos, parent: NONE, tri: NONE, crossed: Vec::new(), xw: 0.0 });
            open.push(Open { f: g0 + h(li, pos), id: (nodes.len() - 1) as u32 });
        }
        let mut crossed = Vec::new();
        let mut exp = 0usize;
        let width = cn.width + 2.0 * self.reserve[c];
        let wmin = cn.width.min(self.board.rules.min_track_width.max(0.1));
        let clr = cn.clearance;
        let mut goal_node = None;
        // differential pair follower: crossing next to the leader is cheaper
        let leader_net = match cn.couple {
            Some((pi, false)) => Some(self.rules.diff_pairs[pi].p),
            _ => None,
        };
        let attract = |li: usize, e: u32, g: u32| -> f64 {
            let ln = match leader_net {
                Some(n) => n,
                None => return 1.0,
            };
            let xs = &self.st.layers[li].edge_x[e as usize];
            let g = g as usize;
            let next_to = (g > 0 && self.st.wires[xs[g - 1].wire as usize].net == ln) || (g < xs.len() && self.st.wires[xs[g].wire as usize].net == ln);
            if next_to {
                0.35
            } else {
                1.0
            }
        };
        while let Some(Open { id, .. }) = open.pop() {
            let nd = nodes[id as usize].clone();
            if best.get(&nd.key).map_or(false, |&g| g < nd.g - 1e-9) {
                continue;
            }
            exp += 1;
            if exp > max_exp {
                break;
            }
            // goal test
            if let SKey::Vert(li, v) = nd.key {
                let li = li as usize;
                let is_dst = dst_set.contains(&(li, v));
                let is_site_goal = to_site
                    && nd.tri != NONE
                    && self.meshes[li].verts[v as usize].site.map_or(false, |s| self.site_usable(s, net) && self.pb.sites[s as usize].plane_nets.contains(&net));
                if (is_dst || is_site_goal) && nd.parent != NONE {
                    goal_node = Some(id);
                    break;
                }
            }
            let push = |nodes: &mut Vec<Node>, open: &mut BinaryHeap<Open>, best: &mut HashMap<SKey, f64>, key: SKey, g: f64, pos: P, tri: u32, cr: &[u32], li: usize, xw: f64| {
                if best.get(&key).map_or(true, |&b| g < b - 1e-9) {
                    best.insert(key, g);
                    nodes.push(Node { key, g, pos, parent: id, tri, crossed: cr.to_vec(), xw });
                    open.push(Open { f: g + h(li, pos), id: (nodes.len() - 1) as u32 });
                }
            };
            match nd.key {
                SKey::Vert(li8, v) => {
                    let li = li8 as usize;
                    let m = &self.meshes[li];
                    let is_source = nd.parent == NONE;
                    let arrived_by_via = !is_source && nd.tri == NONE;
                    // layer change through a via at this vertex (not twice in a row)
                    if allow_via && !arrived_by_via {
                        if let Some(s) = m.verts[v as usize].site {
                            // vias already used earlier on this very path must not clash
                            let mut clash = false;
                            let mut pi = nd.parent;
                            while pi != NONE {
                                let pn = &nodes[pi as usize];
                                if let SKey::Vert(pl, pv) = pn.key {
                                    if let Some(ps) = self.meshes[pl as usize].verts[pv as usize].site {
                                        if ps != s && self.pb.sites[s as usize].conflicts.contains(&(ps as usize)) {
                                            clash = true;
                                            break;
                                        }
                                    }
                                }
                                pi = pn.parent;
                            }
                            if !clash && self.site_usable(s, net) {
                                let pen = self.via_penalty(s, c);
                                let reuse = self.st.site_use[s as usize].is_some();
                                for (lj, mj) in self.meshes.iter().enumerate() {
                                    if lj == li {
                                        continue;
                                    }
                                    if let Some(&vj) = mj.site_vert.get(&s) {
                                        let pair_off = if via_targets.is_empty() {
                                            0.0
                                        } else {
                                            let d = via_targets.iter().map(|t| t.dist(self.pb.sites[s as usize].pos)).fold(f64::INFINITY, f64::min);
                                            via_cost * (d / 0.5).min(4.0)
                                        };
                                        let g = nd.g + if reuse { 0.5 } else { via_cost + self.terrain_via_cost(self.pb.sites[s as usize].pos, net) } + pen + pair_off + if lay_k(lj) > 1.0 { via_cost * 4.0 } else { 0.0 };
                                        push(&mut nodes, &mut open, &mut best, SKey::Vert(lj as u8, vj), g, nd.pos, NONE, &[], lj, 0.0);
                                    }
                                }
                            }
                        }
                    }
                    // wires leave a vertex only at a source or right after a via
                    if !is_source && !arrived_by_via {
                        continue;
                    }
                    for &t in &m.vtris[v as usize] {
                        if !self.tri_allowed(li, t, c) {
                            continue;
                        }
                        let tri = &m.tris[t as usize];
                        let ci = tri.vert_idx(v).unwrap();
                        let pa = ci as f64;
                        // to opposite edge
                        let eo = tri.e[(ci + 1) % 3];
                        let to = m.edges[eo as usize].other_tri(t);
                        if self.edge_crossable(li, eo, c) && self.tri_allowed(li, to, c) {
                            let n = self.st.layers[li].edge_x[eo as usize].len() as u32;
                            for g in 0..=n {
                                let pb = self.gap_bpos(li, t, eo, g);
                                self.crossings(li, t, pa, pb, c, &mut crossed);
                                let (lo, hi, over, xw, neck) = self.fit_gap(li, eo, g, width, wmin, clr, c);
                                let q = self.best_point(li, eo, lo, hi, nd.pos, goal_near(li, nd.pos));
                                let step = nd.pos.dist(q);
                                let mut cost = step * self.dir_factor(li, nd.pos, q) * attract(li, eo, g) * lay_k(li);
                                cost += step * self.ridge_fac(li, q) + self.terrain_cost(li, nd.pos, q, net);
                                cost += crossed.iter().map(|&w| self.wire_cost(w)).sum::<f64>();
                                cost += over * 60.0 * self.pres_fac + neck * 15.0 + self.hist[li][eo as usize] as f64 * hist_k;
                                cost += POUR_COST * self.pour_e[li][eo as usize] as f64 * nd.pos.dist(q);
                                push(&mut nodes, &mut open, &mut best, SKey::Edge(li8, eo, g, to), nd.g + cost, q, t, &crossed, li, xw);
                            }
                        }
                        // to adjacent corners (targets or via sites)
                        for dk in [1usize, 2] {
                            let cj = (ci + dk) % 3;
                            let u = tri.v[cj];
                            let ok = dst_set.contains(&(li, u)) || (m.verts[u as usize].site.map_or(false, |s| (allow_via || to_site) && self.site_usable(s, net)));
                            if !ok {
                                continue;
                            }
                            self.crossings(li, t, pa, cj as f64, c, &mut crossed);
                            let q = m.verts[u as usize].pos;
                            let step = nd.pos.dist(q);
                            let mut cost = step * self.dir_factor(li, nd.pos, q) * lay_k(li);
                            cost += step * self.ridge_fac(li, q) + self.terrain_cost(li, nd.pos, q, net);
                            cost += crossed.iter().map(|&w| self.wire_cost(w)).sum::<f64>();
                            cost += end_pen.get(&(li, u)).copied().unwrap_or(0.0);
                            cost += POUR_COST * self.pour_v[li][u as usize] as f64 * nd.pos.dist(q);
                            push(&mut nodes, &mut open, &mut best, SKey::Vert(li8, u), nd.g + cost, q, t, &crossed, li, 0.0);
                        }
                    }
                }
                SKey::Edge(li8, e, gap, t) => {
                    let li = li8 as usize;
                    let m = &self.meshes[li];
                    let tri = &m.tris[t as usize];
                    let pa = self.gap_bpos(li, t, e, gap);
                    let le = tri.edge_idx(e);
                    for dk in [1usize, 2] {
                        let lj = (le + dk) % 3;
                        let e2 = tri.e[lj];
                        let t2 = m.edges[e2 as usize].other_tri(t);
                        if !self.edge_crossable(li, e2, c) || !self.tri_allowed(li, t2, c) {
                            continue;
                        }
                        let n = self.st.layers[li].edge_x[e2 as usize].len() as u32;
                        for g in 0..=n {
                            let pb = self.gap_bpos(li, t, e2, g);
                            self.crossings(li, t, pa, pb, c, &mut crossed);
                            let (lo, hi, over, xw, neck) = self.fit_gap(li, e2, g, width, wmin, clr, c);
                            let q = self.best_point(li, e2, lo, hi, nd.pos, goal_near(li, nd.pos));
                            let mut cost = nd.pos.dist(q) * self.dir_factor(li, nd.pos, q) * attract(li, e2, g) * lay_k(li);
                            cost += self.terrain_cost(li, nd.pos, q, net);
                            cost += crossed.iter().map(|&w| self.wire_cost(w)).sum::<f64>();
                            cost += over * 60.0 * self.pres_fac + neck * 15.0 + self.hist[li][e2 as usize] as f64 * hist_k;
                            cost += POUR_COST * self.pour_e[li][e2 as usize] as f64 * nd.pos.dist(q);
                            push(&mut nodes, &mut open, &mut best, SKey::Edge(li8, e2, g, t2), nd.g + cost, q, t, &crossed, li, xw);
                        }
                    }
                    for cj in 0..3 {
                        let u = tri.v[cj];
                        let ok = dst_set.contains(&(li, u)) || (m.verts[u as usize].site.map_or(false, |s| (allow_via || to_site) && self.site_usable(s, net)));
                        if !ok {
                            continue;
                        }
                        self.crossings(li, t, pa, cj as f64, c, &mut crossed);
                        let q = m.verts[u as usize].pos;
                        let mut cost = nd.pos.dist(q) * self.dir_factor(li, nd.pos, q) * lay_k(li);
                        cost += self.terrain_cost(li, nd.pos, q, net);
                        cost += crossed.iter().map(|&w| self.wire_cost(w)).sum::<f64>();
                        cost += end_pen.get(&(li, u)).copied().unwrap_or(0.0);
                        push(&mut nodes, &mut open, &mut best, SKey::Vert(li8, u), nd.g + cost, q, t, &crossed, li, 0.0);
                    }
                }
            }
        }
        self.stats.expansions += exp;
        self.stats.searches += 1;
        let gid = goal_node?;
        // reconstruct
        let mut steps = Vec::new();
        let mut all_crossed: Vec<u32> = Vec::new();
        let mut i = gid;
        while i != NONE {
            let nd = &nodes[i as usize];
            let step = match nd.key {
                SKey::Vert(li, v) => Step::Vert { li: li as usize, v },
                SKey::Edge(li, e, gap, t) => Step::Cross { li: li as usize, e, gap, tri: t, xw: nd.xw },
            };
            steps.push((step, nd.tri));
            for &w in &nd.crossed {
                if !all_crossed.contains(&w) {
                    all_crossed.push(w);
                }
            }
            i = nd.parent;
        }
        steps.reverse();
        Some(Found { steps, crossed: all_crossed, cost: nodes[gid as usize].g })
    }

    // ------------------------------------------------------------------ commit / rip-up

    fn alloc_wire(&mut self, w: Wire) -> u32 {
        if let Some(i) = self.st.free_wires.pop() {
            self.st.wires[i as usize] = w;
            i
        } else {
            self.st.wires.push(w);
            (self.st.wires.len() - 1) as u32
        }
    }

    fn commit(&mut self, c: usize, f: &Found) {
        let cn = self.pb.conns[c].clone();
        // split into single-layer pieces at via transitions
        let mut pieces: Vec<(usize, u32, Vec<(u32, u32, f64)>, Vec<u32>, u32)> = Vec::new(); // li, start, crossings(e,gap,width), tris, end
        let mut sites = Vec::new();
        let mut cur: Option<(usize, u32, Vec<(u32, u32, f64)>, Vec<u32>)> = None;
        for (step, tri) in &f.steps {
            match step {
                Step::Vert { li, v } => {
                    match cur.take() {
                        None => cur = Some((*li, *v, Vec::new(), Vec::new())),
                        Some((cli, sv, xs, mut ts)) => {
                            if cli == *li && *tri != NONE {
                                ts.push(*tri);
                                pieces.push((cli, sv, xs, ts, *v));
                                cur = Some((*li, *v, Vec::new(), Vec::new()));
                            } else {
                                // via transition: previous piece ended at the same site
                                if !xs.is_empty() || !ts.is_empty() {
                                    // shouldn't happen: a piece must end on a vertex step
                                }
                                if let Some(s) = self.meshes[*li].verts[*v as usize].site {
                                    sites.push(s);
                                }
                                cur = Some((*li, *v, Vec::new(), Vec::new()));
                            }
                        }
                    }
                }
                Step::Cross { li: _, e, gap, tri: _, xw } => {
                    if let Some((_, _, xs, ts)) = cur.as_mut() {
                        xs.push((*e, *gap, *xw));
                        ts.push(*tri);
                    }
                }
            }
        }
        // fanout / via-site end: the final vertex is a via
        if let Some((Step::Vert { li, v }, _)) = f.steps.last() {
            if let Some(s) = self.meshes[*li].verts[*v as usize].site {
                sites.push(s);
            }
        }
        if let Some((Step::Vert { li, v }, _)) = f.steps.first() {
            if let Some(s) = self.meshes[*li].verts[*v as usize].site {
                sites.push(s);
            }
        }
        sites.sort();
        sites.dedup();
        let mut wires = Vec::new();
        for (li, sv, xs, ts, ev) in pieces {
            let w = Wire { conn: c as u32, li, start: sv, end: ev, cross: xs.iter().map(|x| x.0).collect(), tris: ts.clone(), net: cn.net, net2: cn.net2, width: cn.width + 2.0 * self.reserve[c], clr: cn.clearance, alive: true, couple: cn.couple, xw: xs.iter().map(|x| x.2).collect() };
            let wid = self.alloc_wire(w);
            // insert crossings
            let mut inserted: HashMap<u32, Vec<u32>> = HashMap::new();
            for (k, &(e, gap, _)) in xs.iter().enumerate() {
                let prev = inserted.entry(e).or_default();
                let shift = prev.iter().filter(|&&g| g <= gap).count() as u32;
                prev.push(gap);
                let list = &mut self.st.layers[li].edge_x[e as usize];
                let idx = ((gap + shift) as usize).min(list.len());
                list.insert(idx, XRef { wire: wid, k: k as u32 });
            }
            for (k, &t) in ts.iter().enumerate() {
                self.st.layers[li].tri_ch[t as usize].push(XRef { wire: wid, k: k as u32 });
            }
            wires.push(wid);
        }
        for &s in &sites {
            let u = &mut self.st.site_use[s as usize];
            *u = Some((cn.net, u.map_or(0, |x| x.1) + 1));
        }
        let cs = &mut self.st.conns[c];
        cs.wires = wires;
        cs.sites = sites;
        cs.routed = true;
        cs.failed = false;
    }

    /// Like `commit`, but keeps the wires already placed for this connection:
    /// the dynamic terminals join a second component with the same connection.
    fn commit_more(&mut self, c: usize, f: &Found) {
        let before = self.st.conns[c].wires.clone();
        let mut sites = self.st.conns[c].sites.clone();
        self.commit(c, f);
        let mut wires = before;
        wires.extend(self.st.conns[c].wires.iter().copied());
        sites.extend(self.st.conns[c].sites.iter().copied());
        sites.sort();
        sites.dedup();
        let cs = &mut self.st.conns[c];
        cs.wires = wires;
        cs.sites = sites;
    }

    /// Rip up the connections crossed by a found path (other nets only).
    fn rip_crossed(&mut self, c: usize, f: &Found, ripped: &mut HashSet<usize>) {
        for &w in &f.crossed {
            if !self.st.wires[w as usize].alive {
                continue;
            }
            let oc = self.st.wires[w as usize].conn as usize;
            if oc != c && ripped.insert(oc) {
                for r in self.ripup(oc) {
                    self.st.conns[r].ripups += 1;
                    self.queue.push_back(r);
                }
            }
        }
    }

    pub fn ripup(&mut self, c: usize) -> Vec<usize> {
        if self.pb.conns[c].couple.is_some() && self.st.conns[c].routed && log::log_enabled!(log::Level::Debug) {
            log::debug!("rip pair conn {c} ({})", std::backtrace::Backtrace::force_capture().to_string().lines().filter(|l| l.contains("gridless::")).take(4).map(|l| l.trim().to_string()).collect::<Vec<_>>().join(" <- "));
        }
        let mut out = vec![c];
        let mut stack = vec![c];
        while let Some(c) = stack.pop() {
            if !self.st.conns[c].routed {
                continue;
            }
            let wires = std::mem::take(&mut self.st.conns[c].wires);
            for wid in wires {
                let w = self.st.wires[wid as usize].clone();
                for &e in &w.cross {
                    self.st.layers[w.li].edge_x[e as usize].retain(|x| x.wire != wid);
                }
                for &t in &w.tris {
                    self.st.layers[w.li].tri_ch[t as usize].retain(|x| x.wire != wid);
                }
                self.st.wires[wid as usize].alive = false;
                self.st.free_wires.push(wid);
            }
            let sites = std::mem::take(&mut self.st.conns[c].sites);
            for s in &sites {
                if let Some((n, k)) = self.st.site_use[*s as usize] {
                    self.st.site_use[*s as usize] = if k <= 1 { None } else { Some((n, k - 1)) };
                }
            }
            self.st.conns[c].routed = false;
            self.stats.ripups += 1;
            // dependants: conns of the same net sharing one of these vias
            for (j, cs) in self.st.conns.iter().enumerate() {
                if cs.routed && cs.sites.iter().any(|s| sites.contains(s)) && !out.contains(&j) {
                    out.push(j);
                    stack.push(j);
                }
            }
        }
        out
    }

    // ------------------------------------------------------------------ main loop

    pub fn total_overflow(&self) -> (f64, Vec<(usize, u32, f64)>) {
        let mut tot = 0.0;
        let mut list = Vec::new();
        for li in 0..self.meshes.len() {
            for e in 0..self.meshes[li].edges.len() as u32 {
                let o = self.edge_overflow(li, e);
                if o > 1e-6 {
                    tot += o;
                    list.push((li, e, o));
                }
            }
        }
        (tot, list)
    }

    fn try_route(&mut self, c: usize) -> bool {
        let (srcs, dsts, to_site) = match self.endpoints(c) {
            None => {
                self.st.conns[c].routed = true;
                self.st.conns[c].wires.clear();
                return true;
            }
            Some(x) => x,
        };
        if srcs.is_empty() || (dsts.is_empty() && !to_site) {
            self.st.conns[c].failed = true;
            return false;
        }
        let budget = 60_000;
        // differential pair leader: choose the end pad (equivalent pins of the same part,
        // e.g. USB-C A6/B6) that lets the follower run alongside without a twist
        let mut dsts = dsts;
        if let Some(f) = self.pair_follower(c) {
            let mut pads: Vec<u32> = dsts.iter().filter_map(|&(li, v)| self.meshes[li].verts[v as usize].pad).collect();
            pads.sort();
            pads.dedup();
            log::debug!("pair leader conn {c}: follower {f}, candidate end pads {:?}", pads.iter().map(|&p| self.board.pad_label(p as usize)).collect::<Vec<_>>());
            if pads.len() > 1 && pads.len() <= 4 {
                let snap = self.st.clone();
                let mut best: Option<(f64, u32)> = None;
                for &pd in &pads {
                    let sub: Vec<(usize, u32)> = dsts.iter().copied().filter(|&(li, v)| self.meshes[li].verts[v as usize].pad == Some(pd)).collect();
                    let mut total = f64::INFINITY;
                    if let Some(fl) = self.search(c, &srcs, &sub, to_site, budget) {
                        self.commit(c, &fl);
                        if self.st.conns[f].routed {
                            self.ripup(f);
                        }
                        if let Some((fs, fd, ft)) = self.endpoints(f) {
                            if let Some(ff) = self.search(f, &fs, &fd, ft, budget) {
                                total = fl.cost + ff.cost;
                            }
                        }
                    }
                    self.st = snap.clone();
                    log::debug!("pair trial conn {c} end pad {} -> total cost {total:.2}", self.board.pad_label(pd as usize));
                    if best.map_or(true, |b| total < b.0) {
                        best = Some((total, pd));
                    }
                }
                if let Some((t, pd)) = best {
                    if t.is_finite() {
                        dsts.retain(|&(li, v)| self.meshes[li].verts[v as usize].pad == Some(pd));
                    }
                }
            }
        }
        // Dynamic terminals: end on the nearest unconnected component of the net,
        // then join the planned target to it, so no leaf pad is left behind.
        if self.rules.dynamic_terminals && !to_site && self.pb.conns[c].couple.is_none() {
            if let Some((dsrc, ddst, _)) = self.endpoints_any(c) {
                let snap = self.st.clone();
                if let Some(f) = self.search(c, &dsrc, &ddst, false, budget) {
                    self.commit(c, &f);
                    let mut extra: Option<Found> = None;
                    let mut ok = true;
                    if let Some((s2, d2, ts2)) = self.endpoints(c) {
                        match self.search(c, &s2, &d2, ts2, budget) {
                            Some(f2) => extra = Some(f2),
                            None => ok = false,
                        }
                    }
                    if ok {
                        if let Some(f2) = extra.as_ref() {
                            self.commit_more(c, f2);
                        }
                        let mut ripped = HashSet::new();
                        self.rip_crossed(c, &f, &mut ripped);
                        if let Some(f2) = extra.as_ref() {
                            self.rip_crossed(c, f2, &mut ripped);
                        }
                        return true;
                    }
                    self.st = snap;
                }
            }
        }
        match self.search(c, &srcs, &dsts, to_site, budget) {
            Some(f) => {
                self.commit(c, &f);
                let mut ripped = HashSet::new();
                self.rip_crossed(c, &f, &mut ripped);
                true
            }
            None => {
                self.st.conns[c].failed = true;
                false
            }
        }
    }

    /// For a differential pair leader connection: the follower joining the same parts.
    fn pair_follower(&self, c: usize) -> Option<usize> {
        let (pi, leader) = self.pb.conns[c].couple?;
        if !leader {
            return None;
        }
        let parts = |k: usize| -> Option<(usize, usize)> {
            match (self.pb.conns[k].a, self.pb.conns[k].b) {
                (Term::Pad(x), Term::Pad(y)) => {
                    let (fa, fb) = (self.board.pads[x].footprint, self.board.pads[y].footprint);
                    if fa == fb {
                        None
                    } else {
                        Some((fa.min(fb), fa.max(fb)))
                    }
                }
                _ => None,
            }
        };
        let key = parts(c)?;
        (0..self.pb.conns.len()).find(|&k| self.pb.conns[k].couple == Some((pi, false)) && parts(k) == Some(key))
    }

    /// Connections marked done without wires (their ends were joined through other
    /// connections) are re-checked: rip-ups may have broken that indirect path.
    pub fn revalidate(&mut self) -> usize {
        let mut n = 0;
        for c in 0..self.st.conns.len() {
            let cs = &self.st.conns[c];
            if cs.routed && cs.wires.is_empty() && cs.sites.is_empty() && self.endpoints(c).is_some() {
                self.st.conns[c].routed = false;
                self.queue.push_back(c);
                n += 1;
            }
        }
        n
    }

    pub fn score(&self) -> (usize, f64, usize, f64) {
        let unrouted = self.st.conns.iter().filter(|c| !c.routed).count();
        let (ov, _) = self.total_overflow();
        let vias = self.st.site_use.iter().filter(|x| x.is_some()).count();
        (unrouted, ov, vias, 0.0)
    }

    pub fn run(&mut self) {
        let n = self.pb.conns.len();
        let mut order: Vec<usize> = (0..n).collect();
        let pb = self.pb;
        let short = self.rules.short_link.max(0.0);
        let mode = self.rules.order;
        let board = self.board;
        let rules = self.rules;
        let default_w = board.classes.first().map_or(0.0, |c| c.track_width);
        // designer's category of a link (lower goes first)
        let logic_cat = |c: &crate::problem::Conn| -> u8 {
            if c.couple.is_some() || matches!(c.kind, ConnKind::Pair { .. }) {
                return 0;
            }
            let (pa, pb_) = match (c.a, c.b) {
                (Term::Pad(x), Term::Pad(y)) => (&board.pads[x], &board.pads[y]),
                _ => return 2, // fanout to a plane
            };
            let sig = |l: &usize| rules.layers[*l].role == crate::config::LayerRole::Signal;
            let common = pa.layers.iter().any(|l| sig(l) && pb_.layers.contains(l));
            let smd = |p: &crate::board::Pad| p.kind == crate::board::PadKind::Smd;
            let power = c.priority > 0 || c.width > default_w + 1e-6;
            if common && (smd(pa) || smd(pb_)) && c.est_len <= short {
                1 // a layer change can only make it worse: route it first, where it is
            } else if power {
                2
            } else if common || !smd(pa) || !smd(pb_) {
                3
            } else {
                4 // SMD pads on different layers: a via is inevitable anyway
            }
        };
        order.sort_by(|&a, &b| {
            let (ca, cb) = (&pb.conns[a], &pb.conns[b]);
            match mode {
                crate::config::OrderMode::Logic => logic_cat(ca).cmp(&logic_cat(cb)).then(ca.est_len.partial_cmp(&cb.est_len).unwrap()),
                // short links that need no new via first, then power nets, then the rest
                crate::config::OrderMode::Cost => {
                    let cat = |c: &crate::problem::Conn| -> u8 {
                        if c.couple.is_some() || c.est_cost <= short {
                            0
                        } else if c.priority > 0 {
                            1
                        } else {
                            2
                        }
                    };
                    cat(ca).cmp(&cat(cb)).then(ca.est_cost.partial_cmp(&cb.est_cost).unwrap())
                }
                // pure shortest effective cost, pairs first
                crate::config::OrderMode::Short => ca
                    .couple
                    .is_some()
                    .cmp(&cb.couple.is_some())
                    .reverse()
                    .then(ca.est_cost.partial_cmp(&cb.est_cost).unwrap()),
                // the proven order: net priority, then width, then straight length
                _ => cb
                    .priority
                    .cmp(&ca.priority)
                    .then(cb.width.partial_cmp(&ca.width).unwrap())
                    .then(ca.est_len.partial_cmp(&cb.est_len).unwrap()),
            }
        });
        self.queue = order.into_iter().collect();
        self.negotiate(0, self.rules.iterations);
    }

    /// TopoR-style via minimisation: re-route every connection that uses vias with a
    /// prohibitive via price, without disturbing other wires; keep it if it is not much
    /// longer. Returns the number of vias removed.
    pub fn minimise_vias(&mut self) -> usize {
        let mut removed = 0;
        let order: Vec<usize> = (0..self.st.conns.len()).filter(|&c| !self.st.conns[c].sites.is_empty()).collect();
        for c in order {
            if self.st.conns[c].sites.is_empty() || !self.st.conns[c].routed {
                continue;
            }
            if matches!(self.pb.conns[c].kind, ConnKind::Fanout) {
                continue;
            }
            let before_vias = self.st.conns[c].sites.len();
            let old_len = self.conn_length(c);
            let saved = self.st.clone();
            let saved_rip = self.rip_base;
            for r in self.ripup(c) {
                if r != c {
                    // dependants share a via: too entangled, keep as is
                    self.st = saved.clone();
                    self.rip_base = saved_rip;
                    continue;
                }
            }
            if self.st.conns[c].routed {
                continue;
            }
            self.via_scale = 1000.0;
            self.rip_base = 1e6; // never displace others here
            let ok = match self.endpoints(c) {
                None => true,
                Some((srcs, dsts, to_site)) => match self.search(c, &srcs, &dsts, to_site, 60_000) {
                    Some(f) if f.crossed.is_empty() => {
                        self.commit(c, &f);
                        let (_, list) = self.total_overflow();
                        let new_len = self.conn_length(c);
                        list.is_empty() && self.st.conns[c].sites.len() < before_vias && new_len <= old_len * 1.6 + 3.0
                    }
                    _ => false,
                },
            };
            self.via_scale = 1.0;
            self.rip_base = saved_rip;
            if ok {
                removed += before_vias - self.st.conns[c].sites.len();
            } else {
                self.st = saved;
            }
        }
        self.stats.vias = self.st.site_use.iter().filter(|x| x.is_some()).count();
        removed
    }

    /// Taut (pulled) length of a wire inside its corridor, weighted by the layer's
    /// direction preference: a good proxy of the final geometric length.
    fn wire_taut(&self, wi: u32) -> f64 {
        let w = &self.st.wires[wi as usize];
        let m = &self.meshes[w.li];
        let mut portals = Vec::with_capacity(w.cross.len());
        for (k, &e) in w.cross.iter().enumerate() {
            let ed = &m.edges[e as usize];
            let a = m.verts[ed.v[0] as usize].pos;
            let b = m.verts[ed.v[1] as usize].pos;
            let len = ed.len.max(1e-9);
            let (mut lo, mut hi) = (self.edge_exit(w.li, e, 0), len - self.edge_exit(w.li, e, 1));
            if lo > hi {
                let mid = 0.5 * (lo + hi);
                lo = mid;
                hi = mid;
            }
            let (plo, phi) = (a + (b - a) * (lo / len), a + (b - a) * (hi / len));
            let tri = &m.tris[w.tris[k] as usize];
            let le = tri.edge_idx(e);
            let fwd = ed.v[0] == tri.v[le];
            portals.push(if fwd { (phi, plo) } else { (plo, phi) });
        }
        let path = crate::geometry::funnel(m.verts[w.start as usize].pos, m.verts[w.end as usize].pos, &portals);
        path.windows(2).map(|s| s[0].dist(s[1]) * self.dir_factor(w.li, s[0], s[1]) + self.terrain_cost(w.li, s[0], s[1], w.net)).sum()
    }

    /// Quality score of a routed connection (lower is better).
    fn conn_score(&self, c: usize) -> f64 {
        let cs = &self.st.conns[c];
        let mut sc: f64 = cs.wires.iter().map(|&w| self.wire_taut(w)).sum();
        sc += self.rules.via_cost * cs.sites.len() as f64;
        for &wi in &cs.wires {
            let w = &self.st.wires[wi as usize];
            sc += self.layer_switch_pen(w.li, w.start, c) + self.layer_switch_pen(w.li, w.end, c);
        }
        sc
    }

    /// Final clean-up (TopoR-style improvement): re-route every connection alone on the
    /// settled board, without displacing anything, and keep the new route when it is
    /// shorter / has fewer vias or layer changes. Removes detours and loops left over
    /// from the negotiation. Returns the number of improved connections.
    pub fn improve(&mut self, passes: usize) -> usize {
        let mut improved = 0;
        for _ in 0..passes {
            let mut any = false;
            let mut order: Vec<usize> = (0..self.st.conns.len())
                .filter(|&c| self.st.conns[c].routed && !self.st.conns[c].wires.is_empty() && self.pb.conns[c].couple.is_none())
                .collect();
            order.sort_by(|&a, &b| self.pb.conns[b].est_len.partial_cmp(&self.pb.conns[a].est_len).unwrap());
            let (ov0, _) = self.total_overflow();
            for c in order {
                if !self.st.conns[c].routed || self.st.conns[c].wires.is_empty() {
                    continue;
                }
                let old = self.conn_score(c);
                let saved = self.st.clone();
                let (rb, pf) = (self.rip_base, self.pres_fac);
                if self.ripup(c).len() > 1 {
                    self.st = saved;
                    continue;
                }
                self.rip_base = 1e6;
                self.pres_fac = 50.0;
                let ok = match self.endpoints(c) {
                    None => {
                        self.st.conns[c].routed = true;
                        true
                    }
                    Some((srcs, dsts, to_site)) => match self.search(c, &srcs, &dsts, to_site, 60_000) {
                        Some(f) if f.crossed.is_empty() => {
                            self.commit(c, &f);
                            let (ov, _) = self.total_overflow();
                            ov <= ov0 + 1e-6 && self.conn_score(c) < old - 0.3
                        }
                        _ => false,
                    },
                };
                self.rip_base = rb;
                self.pres_fac = pf;
                if ok {
                    improved += 1;
                    any = true;
                } else {
                    self.st = saved;
                }
            }
            if !any {
                break;
            }
        }
        self.stats.vias = self.st.site_use.iter().filter(|x| x.is_some()).count();
        improved
    }

    /// End points (positions) of a routed connection: first and last wire vertex.
    fn conn_ends(&self, c: usize) -> Option<(P, P)> {
        let ws = &self.st.conns[c].wires;
        let first = &self.st.wires[*ws.first()? as usize];
        let last = &self.st.wires[*ws.last()? as usize];
        Some((self.meshes[first.li].verts[first.start as usize].pos, self.meshes[last.li].verts[last.end as usize].pos))
    }

    /// Second clean-up stage: a connection that makes a big detour is re-routed first and
    /// the connections whose wires block its straight path are re-routed after it; the
    /// group is kept when its total score improves (moves "who goes around whom").
    pub fn improve_detours(&mut self, max_groups: usize) -> usize {
        let mut improved = 0;
        let mut cands: Vec<(f64, usize)> = Vec::new();
        for c in 0..self.st.conns.len() {
            let cs = &self.st.conns[c];
            if !cs.routed || cs.wires.is_empty() || self.pb.conns[c].couple.is_some() {
                continue;
            }
            let (a, b) = match self.conn_ends(c) {
                Some(x) => x,
                None => continue,
            };
            let direct = a.dist(b);
            let sc = self.conn_score(c);
            if sc > 1.25 * direct + 1.5 {
                cands.push((sc - direct, c));
            }
        }
        cands.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
        let emb = self.embed();
        for &(_, c) in cands.iter().take(max_groups) {
            if !self.st.conns[c].routed {
                continue;
            }
            let (a, b) = match self.conn_ends(c) {
                Some(x) => x,
                None => continue,
            };
            // blockers: other nets' wires on the same layer(s) crossing the straight line a-b
            let net = self.pb.conns[c].net;
            let my_layers: u32 = self.st.conns[c].wires.iter().fold(0u32, |m, &w| m | (1 << self.st.wires[w as usize].li));
            let mut blk: Vec<(f64, usize)> = Vec::new();
            for ew in &emb {
                let w = &self.st.wires[ew.wire as usize];
                if !w.alive || w.net == net || w.conn as usize == c || self.pb.conns[w.conn as usize].couple.is_some() || my_layers & (1 << w.li) == 0 {
                    continue;
                }
                for s in ew.pts.windows(2) {
                    if crate::geom::segs_intersect(a, b, s[0], s[1]) {
                        let d = crate::geom::point_seg_dist(a.mid(b), s[0], s[1]);
                        if !blk.iter().any(|x| x.1 == w.conn as usize) {
                            blk.push((d, w.conn as usize));
                        }
                        break;
                    }
                }
            }
            if blk.is_empty() {
                continue;
            }
            blk.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            let group: Vec<usize> = std::iter::once(c).chain(blk.iter().take(4).map(|x| x.1)).collect();
            let old: f64 = group.iter().map(|&k| self.conn_score(k)).sum();
            let saved = self.st.clone();
            let (rb, pf) = (self.rip_base, self.pres_fac);
            let (ov0, _) = self.total_overflow();
            let mut ripped: Vec<usize> = Vec::new();
            for &k in &group {
                for r in self.ripup(k) {
                    if !ripped.contains(&r) {
                        ripped.push(r);
                    }
                }
            }
            self.rip_base = 1e6;
            self.pres_fac = 50.0;
            // A detour often exists because the layer's preferred direction made a
            // short perpendicular hop more expensive than a long way round. Let this
            // re-route ignore the direction for a moment and take the shorter path.
            let ds = self.dir_scale;
            if self.rules.detour_relax > 0.0 {
                self.dir_scale = self.rules.detour_relax;
            }
            let mut ok = true;
            // the detouring connection first, then the others (dependants included)
            let mut order = vec![c];
            order.extend(ripped.iter().copied().filter(|&k| k != c));
            for &k in &order {
                if self.st.conns[k].routed {
                    continue;
                }
                match self.endpoints(k) {
                    None => self.st.conns[k].routed = true,
                    Some((srcs, dsts, ts)) => match self.search(k, &srcs, &dsts, ts, 60_000) {
                        Some(f) if f.crossed.is_empty() => self.commit(k, &f),
                        _ => {
                            ok = false;
                            break;
                        }
                    },
                }
            }
            if ok {
                let (ov, _) = self.total_overflow();
                let new: f64 = ripped.iter().map(|&k| self.conn_score(k)).sum::<f64>();
                let old_all: f64 = old + ripped.iter().filter(|k| !group.contains(k)).count() as f64 * 0.0;
                // a relaxed-direction re-route must earn its place: only a clearly
                // shorter path (>=15 %) may disturb the settled board
                ok = ov <= ov0 + 1e-6
                    && if self.rules.detour_relax > 0.0 { new < old_all * 0.85 } else { new < old_all - 0.5 };
            }
            self.rip_base = rb;
            self.pres_fac = pf;
            self.dir_scale = ds;
            log::debug!(
                "detour group {:?} (old score {:.2}): {}",
                group.iter().map(|&k| self.board.nets[self.pb.conns[k].net].name.clone()).collect::<Vec<_>>(),
                old,
                if ok { "accepted" } else { "rejected" }
            );
            if ok {
                improved += 1;
            } else {
                self.st = saved;
            }
        }
        self.stats.vias = self.st.site_use.iter().filter(|x| x.is_some()).count();
        improved
    }

    /// Rough length of a routed connection from its embedded polylines.
    pub fn conn_length(&self, c: usize) -> f64 {
        let mut l = 0.0;
        for &wi in &self.st.conns[c].wires {
            let w = &self.st.wires[wi as usize];
            let m = &self.meshes[w.li];
            let mut prev = m.verts[w.start as usize].pos;
            for (k, &e) in w.cross.iter().enumerate() {
                let j = self.xref_index(w.li, e, XRef { wire: wi, k: k as u32 });
                let q = self.embed_point(w.li, e, j);
                l += prev.dist(q);
                prev = q;
            }
            l += prev.dist(m.verts[w.end as usize].pos);
        }
        l
    }

    /// Penalise the routing edges of `layer` around point q and return the connections
    /// whose wires pass there.
    pub fn penalise_near(&mut self, layer: crate::board::LayerId, q: P, radius: f64, amount: f32) -> Vec<usize> {
        let mut conns = Vec::new();
        for li in 0..self.meshes.len() {
            if self.meshes[li].layer != layer {
                continue;
            }
            for e in 0..self.meshes[li].edges.len() {
                let ed = &self.meshes[li].edges[e];
                let a = self.meshes[li].verts[ed.v[0] as usize].pos;
                let b = self.meshes[li].verts[ed.v[1] as usize].pos;
                if crate::geom::point_seg_dist(q, a, b) < radius {
                    self.hist[li][e] += amount;
                    for x in &self.st.layers[li].edge_x[e] {
                        let c = self.st.wires[x.wire as usize].conn as usize;
                        if !conns.contains(&c) {
                            conns.push(c);
                        }
                    }
                }
            }
        }
        conns
    }

    /// Rip up the given connections and run more negotiation iterations.
    pub fn reroute(&mut self, conns: &[usize], iters: usize) {
        let mut conns: Vec<usize> = conns.to_vec();
        // a differential pair is always re-routed as a pair
        for c in conns.clone() {
            if let Some((pi, _)) = self.pb.conns[c].couple {
                for k in 0..self.pb.conns.len() {
                    if self.pb.conns[k].couple.map(|x| x.0) == Some(pi) && !conns.contains(&k) {
                        conns.push(k);
                    }
                }
            }
        }
        // leaders first so that the joint pin choice can run
        conns.sort_by_key(|&c| match self.pb.conns[c].couple {
            Some((_, true)) => 0,
            Some((_, false)) => 1,
            None => 2,
        });
        for &c in &conns {
            for r in self.ripup(c) {
                self.st.conns[r].ripups += 1;
                if !self.queue.contains(&r) {
                    self.queue.push_back(r);
                }
            }
        }
        let start = self.stats.iterations;
        self.negotiate(start, iters);
    }

    fn negotiate(&mut self, it0: usize, iters: usize) {
        let n = self.pb.conns.len();
        let pb = self.pb;
        let mut best: Option<(State, (usize, f64, usize))> = None;
        for it in it0..it0 + iters {
            self.stats.iterations = it + 1;
            self.pres_fac = (1.3f64).powi(it as i32).min(30.0);
            self.rip_base = 25.0 + 4.0 * it as f64;
            let mut guard = 0;
            while let Some(c) = self.queue.pop_front() {
                guard += 1;
                if guard > 20 * n + 200 {
                    break;
                }
                if self.st.conns[c].routed {
                    continue;
                }
                self.try_route(c);
            }
            // indirect connections broken by rip-ups: route them now
            for _ in 0..5 {
                if self.revalidate() == 0 {
                    break;
                }
                while let Some(c) = self.queue.pop_front() {
                    if !self.st.conns[c].routed {
                        self.try_route(c);
                    }
                }
            }
            let (ov, list) = self.total_overflow();
            let unrouted: Vec<usize> = (0..n).filter(|&c| !self.st.conns[c].routed).collect();
            let vias = self.st.site_use.iter().filter(|x| x.is_some()).count();
            let sc = (unrouted.len(), ov, vias);
            log::info!("iter {it}: unrouted {} overflow {:.3} mm on {} edges, vias {}", unrouted.len(), ov, list.len(), vias);
            let better = match &best {
                None => true,
                Some((_, b)) => (sc.0, (sc.1 * 1000.0) as i64, sc.2) < (b.0, (b.1 * 1000.0) as i64, b.2),
            };
            if better {
                best = Some((self.st.clone(), sc));
            }
            if unrouted.is_empty() && ov < 1e-6 {
                break;
            }
            // negotiate: grow history on overflowed edges, rip conns crossing them
            let mut to_rip: Vec<usize> = Vec::new();
            for &(li, e, o) in &list {
                self.hist[li][e as usize] += (o * 20.0) as f32 + 2.0;
                for x in &self.st.layers[li].edge_x[e as usize] {
                    let c = self.st.wires[x.wire as usize].conn as usize;
                    if !to_rip.contains(&c) {
                        to_rip.push(c);
                    }
                }
            }
            // rip only the cheapest-to-move half: shortest conns first, pairs last
            to_rip.sort_by(|&a, &b| {
                let (pa, pb_) = (pb.conns[a].couple.is_some(), pb.conns[b].couple.is_some());
                pa.cmp(&pb_).then(pb.conns[a].est_len.partial_cmp(&pb.conns[b].est_len).unwrap())
            });
            if self.protect_pairs && to_rip.iter().any(|&c| pb.conns[c].couple.is_none()) {
                to_rip.retain(|&c| pb.conns[c].couple.is_none());
            }
            let k = (to_rip.len() + 1) / 2;
            for &c in to_rip.iter().take(k.max(1)) {
                for r in self.ripup(c) {
                    self.st.conns[r].ripups += 1;
                    self.queue.push_back(r);
                }
            }
            for c in unrouted {
                if !self.queue.contains(&c) {
                    self.queue.push_back(c);
                }
            }
        }
        if let Some((st, sc)) = best {
            self.st = st;
            log::info!("best: unrouted {} overflow {:.3} vias {}", sc.0, sc.1, sc.2);
        }
        if let Ok(dn) = std::env::var("GRIDLESS_DEBUG_NET") {
            let lab = |li: usize, v: u32| {
                let mv = &self.meshes[li].verts[v as usize];
                if let Some(p) = mv.pad {
                    self.board.pad_label(p as usize)
                } else if let Some(s) = mv.site {
                    format!("site{s}")
                } else {
                    format!("v{v}")
                }
            };
            for (c, cn) in self.pb.conns.iter().enumerate() {
                if self.board.nets[cn.net].name != dn {
                    continue;
                }
                let cs = &self.st.conns[c];
                let a = match cn.a { Term::Pad(p) => self.board.pad_label(p), Term::AnySite => "site".into() };
                let b = match cn.b { Term::Pad(p) => self.board.pad_label(p), Term::AnySite => "site".into() };
                let ws: Vec<String> = cs.wires.iter().map(|&w| { let w = &self.st.wires[w as usize]; format!("L{}:{}->{}", w.li, lab(w.li, w.start), lab(w.li, w.end)) }).collect();
                eprintln!("conn {c} {a}->{b} routed {} wires {:?} sites {:?}", cs.routed, ws, cs.sites);
            }
        }
        let (u, ov, v, _) = self.score();
        self.stats.unrouted = u;
        self.stats.overflow = ov;
        self.stats.vias = v;
    }
}

/// A wire laid out as a planar polyline (crossing points spread along their edges).
#[derive(Clone, Debug)]
pub struct EmbeddedWire {
    pub wire: u32,
    pub li: usize,
    pub pts: Vec<P>,
}

impl<'a> Router<'a> {
    /// Position of crossing index j of edge e (planar, order-preserving).
    pub fn embed_point(&self, li: usize, e: u32, j: usize) -> P {
        let m = &self.meshes[li];
        let ed = &m.edges[e as usize];
        let a = m.verts[ed.v[0] as usize].pos;
        let b = m.verts[ed.v[1] as usize].pos;
        let n = self.st.layers[li].edge_x[e as usize].len();
        let len = ed.len.max(1e-9);
        let mut s0 = self.edge_exit(li, e, 0).min(len * 0.45);
        let mut s1 = (len - self.edge_exit(li, e, 1)).max(len * 0.55);
        if s1 <= s0 {
            s0 = len * 0.45;
            s1 = len * 0.55;
        }
        let t = s0 + (s1 - s0) * (j as f64 + 0.5) / n as f64;
        a + (b - a) * (t / len)
    }

    pub fn embed(&self) -> Vec<EmbeddedWire> {
        let mut out = Vec::new();
        for (wi, w) in self.st.wires.iter().enumerate() {
            if !w.alive {
                continue;
            }
            let m = &self.meshes[w.li];
            let mut pts = vec![m.verts[w.start as usize].pos];
            for (k, &e) in w.cross.iter().enumerate() {
                let j = self.xref_index(w.li, e, XRef { wire: wi as u32, k: k as u32 });
                pts.push(self.embed_point(w.li, e, j));
            }
            pts.push(m.verts[w.end as usize].pos);
            out.push(EmbeddedWire { wire: wi as u32, li: w.li, pts });
        }
        out
    }
}
