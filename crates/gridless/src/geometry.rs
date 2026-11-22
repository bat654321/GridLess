//! Geometric realisation of the topological sketch ("rubber-band" pull-tight).
//!
//! For every routing layer a fresh triangulation of the *real* obstacles (pads, used
//! vias, outline) is built and each wire is re-threaded through it. The variables are
//! the positions of the crossings along mesh edges. Inside a triangle a wire is either
//! a straight chord or wraps the shared corner with an arc whose radius stacks on top of
//! the wires nested inside it (concentric bundles, TopoR style). Total length is
//! minimised by Gauss–Seidel coordinate descent subject to ordering/spacing constraints
//! along every edge. The result is a chain of straight segments and arcs.

use crate::board::{LayerId, NetId};
use crate::config::Rules;
use crate::geom::{orient, p, point_seg_dist, Arc, P};
use crate::mesh::{exit_dist, MObs, Mesh, NONE};
use crate::problem::{ObsKind, Problem};
use crate::topo::Router;

/// Extra clearance kept by the geometry so results survive rounding to KiCad's 1 nm grid.
pub const GEOM_MARGIN: f64 = 0.003;

#[inline]
fn gclr(rules: &Rules, a: Option<NetId>, b: Option<NetId>) -> f64 {
    rules.clr(a, b) + GEOM_MARGIN
}

#[derive(Clone, Debug)]
pub struct GWire {
    pub conn: u32,
    pub net: NetId,
    pub net2: Option<NetId>,
    pub width: f64,
    pub clr: f64,
    pub sv: u32,
    pub ev: u32,
    pub start: P,
    pub end: P,
    pub xe: Vec<u32>,
    pub xt: Vec<f64>,
    /// obstacle-only bounds of each crossing parameter
    pub xlo: Vec<f64>,
    pub xhi: Vec<f64>,
    /// chord triangles (xe.len()+1)
    pub tris: Vec<u32>,
    /// per chord: corner vertex wrapped (NONE for chords touching a terminal) and arc radius
    pub corner: Vec<u32>,
    pub rho: Vec<f64>,
    /// per chord: larger radius around its corner that runs through the centring slots
    /// (see `soft_r` for how it is applied)
    pub crho: Vec<f64>,
    /// differential pair: (partner net, centre distance) when this wire follows its partner
    pub follow: Option<(NetId, f64)>,
    /// track width used on the first / last chord (neck-down into small pads)
    pub neck: [f64; 2],
    /// actual width of every chord (auto neck-down where room is short)
    pub cw: Vec<f64>,
    /// per crossing: window around its equal-gap slot between two pads (centring)
    pub win: Vec<Option<(f64, f64)>>,
    /// room kept free on each side for length-matching meanders: `width` includes it,
    /// the drawn track does not
    pub reserve: f64,
}

impl GWire {
    /// width of chord k (terminal chords may be necked down)
    pub fn chord_w(&self, k: usize) -> f64 {
        self.cw[k]
    }
    fn init_widths(&mut self) {
        let n = self.xe.len();
        self.cw = vec![self.width; n + 1];
        self.cw[0] = self.cw[0].min(self.neck[0]);
        self.cw[n] = self.cw[n].min(self.neck[1]);
    }
}

/// Geometric primitive of a routed wire.
#[derive(Clone, Copy, Debug)]
pub enum Prim {
    Seg(P, P),
    Arc(Arc),
}

impl Prim {
    pub fn len(&self) -> f64 {
        match self {
            Prim::Seg(a, b) => a.dist(*b),
            Prim::Arc(a) => a.len(),
        }
    }
    pub fn start(&self) -> P {
        match self {
            Prim::Seg(a, _) => *a,
            Prim::Arc(a) => a.start(),
        }
    }
    pub fn end(&self) -> P {
        match self {
            Prim::Seg(_, b) => *b,
            Prim::Arc(a) => a.end(),
        }
    }
    pub fn polyline(&self, tol: f64) -> Vec<P> {
        match self {
            Prim::Seg(a, b) => vec![*a, *b],
            Prim::Arc(a) => a.polyline(tol),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Track {
    pub layer: LayerId,
    pub net: NetId,
    pub width: f64,
    pub conn: u32,
    pub prims: Vec<Prim>,
    /// width of each primitive
    pub pw: Vec<f64>,
}

pub struct LayerGeom {
    pub layer: LayerId,
    pub mesh: Mesh,
    pub wires: Vec<GWire>,
    /// per edge: crossings (wire, k) in topological order from v0 to v1
    pub edge_x: Vec<Vec<(u32, u32)>>,
    /// per triangle: chords (wire, k)
    pub tri_ch: Vec<Vec<(u32, u32)>>,
    pub edge_clearance: f64,
    pub outline: Vec<Vec<P>>,
    /// per triangle: obstacles whose inflated shape may reach into it
    pub tri_obs: Vec<Vec<u32>>,
    /// per triangle: board outline segments close to it
    pub tri_out: Vec<Vec<(P, P)>>,
    /// per edge running between the copper of two obstacles: copper extent of each end
    /// along the edge (filled in for centring)
    pub walls: Vec<Option<[f64; 2]>>,
    pub rules: Rules,
}

/// Straight pieces of a chord a->b that wraps (c, rho) (tangent segments only).
fn chord_pieces(a: P, b: P, wrap: Option<(P, f64)>) -> ([(P, P); 2], usize, Option<Arc>) {
    if let Some((c, rho)) = wrap {
        if let (_, Some((t1, t2))) = around(a, b, c, rho) {
            let a0 = (t1 - c).angle();
            let sw = crate::geom::norm_angle((t2 - c).angle() - a0);
            return ([(a, t1), (t2, b)], 2, Some(Arc { c, r: rho, a0, sweep: sw }));
        }
    }
    ([(a, b), (b, b)], 1, None)
}

/// Radius to bend around the obstacle at q: the centring radius `soft`, shrunk to the
/// distance of a chord end that lies inside it, but never below the rule radius `hard`
/// (continuous in the end positions, so the relaxation sees no jumps).
fn soft_r(hard: f64, soft: f64, q: P, a: P, b: P) -> f64 {
    if soft > hard {
        hard.max(soft.min(a.dist(q)).min(b.dist(q)))
    } else {
        hard
    }
}

/// Shortest path P->Q around disk (c, rho) inside the wedge at c: (length, tangent points).
pub fn around(pp: P, q: P, c: P, rho: f64) -> (f64, Option<(P, P)>) {
    if rho <= 1e-9 || point_seg_dist(c, pp, q) >= rho {
        return (pp.dist(q), None);
    }
    let dp = pp.dist(c);
    let dq = q.dist(c);
    if dp < rho - 1e-7 || dq < rho - 1e-7 {
        // endpoint inside the disk: infeasible, penalise smoothly
        let pen = (rho - dp.min(dq)).max(0.0);
        return (pp.dist(q) + 10.0 * pen, None);
    }
    let vp = pp - c;
    let vq = q - c;
    let phi = vp.cross(vq).atan2(vp.dot(vq)); // signed angle P->Q
    let ap = (rho / dp).min(1.0).acos();
    let aq = (rho / dq).min(1.0).acos();
    let arc = phi.abs() - ap - aq;
    if arc <= 0.0 {
        return (pp.dist(q), None);
    }
    let s = phi.signum();
    let t1 = c + vp.norm().rot(s * ap) * rho;
    let t2 = c + vq.norm().rot(-s * aq) * rho;
    ((dp * dp - rho * rho).max(0.0).sqrt() + (dq * dq - rho * rho).max(0.0).sqrt() + rho * arc, Some((t1, t2)))
}

/// Distance along a->b at which the point gets `d` away from the board edge near the
/// outline vertex `a` (only the outline segments through `a` count: the far end of the
/// edge may lie on the outline too).
fn outline_exit(outline: &[Vec<P>], a: P, b: P, d: f64) -> f64 {
    let len = a.dist(b);
    if len < 1e-12 {
        return 0.0;
    }
    let mut local: Vec<(P, P)> = Vec::new();
    for poly in outline {
        let n = poly.len();
        for k in 0..n {
            let (s0, s1) = (poly[k], poly[(k + 1) % n]);
            if point_seg_dist(a, s0, s1) < 1e-6 {
                local.push((s0, s1));
            }
        }
    }
    if local.is_empty() {
        return d;
    }
    let dist = |q: P| local.iter().map(|&(s0, s1)| point_seg_dist(q, s0, s1)).fold(f64::INFINITY, f64::min);
    if dist(b) < d {
        return len;
    }
    let (mut lo, mut hi) = (0.0, len);
    for _ in 0..40 {
        let m = 0.5 * (lo + hi);
        if dist(a.lerp(b, m / len)) < d {
            lo = m;
        } else {
            hi = m;
        }
    }
    hi
}

/// Simple stupid funnel algorithm over portals given as (left, right) points.
pub fn funnel(start: P, end: P, portals: &[(P, P)]) -> Vec<P> {
    let mut pts: Vec<(P, P)> = Vec::with_capacity(portals.len() + 2);
    pts.push((start, start));
    pts.extend_from_slice(portals);
    pts.push((end, end));
    let mut path = vec![start];
    let mut apex = start;
    let (mut left, mut right) = (start, start);
    let (mut li, mut ri) = (0usize, 0usize);
    let mut i = 1;
    let mut guard = 0;
    while i < pts.len() {
        guard += 1;
        if guard > 100_000 {
            break;
        }
        let (l, r) = pts[i];
        // right side: r narrows if it is left of (or on) apex->right
        if orient(apex, right, r) >= 0.0 {
            if apex.dist(right) < 1e-12 || orient(apex, left, r) < 0.0 {
                right = r;
                ri = i;
            } else {
                path.push(left);
                apex = left;
                let ai = li;
                left = apex;
                right = apex;
                li = ai;
                ri = ai;
                i = ai + 1;
                continue;
            }
        }
        if orient(apex, left, l) <= 0.0 {
            if apex.dist(left) < 1e-12 || orient(apex, right, l) > 0.0 {
                left = l;
                li = i;
            } else {
                path.push(right);
                apex = right;
                let ai = ri;
                left = apex;
                right = apex;
                li = ai;
                ri = ai;
                i = ai + 1;
                continue;
            }
        }
        i += 1;
    }
    if path.last().map_or(true, |q| q.dist(end) > 1e-12) {
        path.push(end);
    }
    path
}

impl LayerGeom {
    fn vpos(&self, v: u32) -> P {
        self.mesh.verts[v as usize].pos
    }
    fn edge_pt(&self, e: u32, t: f64) -> P {
        let ed = &self.mesh.edges[e as usize];
        let a = self.vpos(ed.v[0]);
        let b = self.vpos(ed.v[1]);
        a + (b - a) * (t / ed.len.max(1e-12))
    }
    /// Point k of wire w: 0 = start, 1..=n crossings, n+1 = end.
    fn wpt(&self, w: &GWire, k: usize) -> P {
        if k == 0 {
            w.start
        } else if k > w.xe.len() {
            w.end
        } else {
            self.edge_pt(w.xe[k - 1], w.xt[k - 1])
        }
    }
    /// Clearance disk radius required at vertex v for wire w of the given width (0 if
    /// there is no obstacle at v).
    fn obs_rho_w(&self, w: &GWire, v: u32, rules: &Rules, width: f64) -> f64 {
        let mv = &self.mesh.verts[v as usize];
        if let Some(o) = mv.obs {
            let ob = &self.mesh.obs[o as usize];
            if ob.net.is_some() && (ob.net == Some(w.net) || ob.net == w.net2) {
                return 0.0;
            }
            if ob.kind == ObsKind::Keepout {
                return width / 2.0;
            }
            return mv.r + gclr(rules, Some(w.net), ob.net) + width / 2.0;
        }
        if mv.outline {
            return self.edge_clearance + GEOM_MARGIN + width / 2.0;
        }
        0.0
    }

    fn same_net(a: &GWire, b: &GWire) -> bool {
        a.net == b.net || Some(a.net) == b.net2 || a.net2 == Some(b.net) || (a.net2.is_some() && a.net2 == b.net2)
    }

    /// Recompute corner and arc radius of every chord (inner chords first per triangle).
    fn update_rho(&mut self, rules: &Rules) {
        for t in 0..self.tri_ch.len() {
            if self.tri_ch[t].is_empty() {
                continue;
            }
            // chords around a corner: (corner, wire, k, depth key)
            let mut list: Vec<(u32, u32, u32, f64)> = Vec::new();
            for &(wi, k) in &self.tri_ch[t] {
                let w = &self.wires[wi as usize];
                let k = k as usize;
                let corner = if k == 0 || k == w.xe.len() {
                    NONE
                } else {
                    let e1 = &self.mesh.edges[w.xe[k - 1] as usize];
                    let e2 = &self.mesh.edges[w.xe[k] as usize];
                    let shared = e1.v.iter().find(|v| e2.v.contains(v)).copied();
                    shared.unwrap_or(NONE)
                };
                if corner == NONE {
                    self.wires[wi as usize].corner[k] = NONE;
                    self.wires[wi as usize].rho[k] = 0.0;
                    self.wires[wi as usize].crho[k] = 0.0;
                    continue;
                }
                // depth: distance of the crossing on e1 from the corner
                let pt = self.wpt(w, k);
                list.push((corner, wi, k as u32, pt.dist(self.vpos(corner))));
            }
            list.sort_by(|a, b| a.0.cmp(&b.0).then(a.3.partial_cmp(&b.3).unwrap()));
            for i in 0..list.len() {
                let (c, wi, k, _) = list[i];
                let w = &self.wires[wi as usize];
                let cp = self.vpos(c);
                let wk = w.cw[k as usize];
                let mut rho = self.obs_rho_w(w, c, rules, wk);
                for j in 0..i {
                    let (cj, wj, kj, _) = list[j];
                    if cj != c {
                        continue;
                    }
                    let o = &self.wires[wj as usize];
                    if Self::same_net(w, o) {
                        continue;
                    }
                    let a = self.wpt(o, kj as usize);
                    let b = self.wpt(o, kj as usize + 1);
                    let d = self.corner_r(o, kj as usize, cp, a, b).max(point_seg_dist(cp, a, b));
                    rho = rho.max(d + o.cw[kj as usize] / 2.0 + gclr(rules, Some(w.net), Some(o.net)) + wk / 2.0);
                }
                let crho = self.center_rho(w, k as usize, c);
                let wm = &mut self.wires[wi as usize];
                wm.corner[k as usize] = c;
                wm.rho[k as usize] = rho;
                wm.crho[k as usize] = crho;
            }
        }
    }

    /// Radius chord k of wire w (ends a, b) uses around its own corner at cp.
    fn corner_r(&self, w: &GWire, k: usize, cp: P, a: P, b: P) -> f64 {
        soft_r(w.rho[k], w.crho[k], cp, a, b)
    }

    /// Soft push of the ends of chord k out of the centring disks of its triangle's
    /// corners, so that the free crossings make room for the centring arcs.
    fn center_pen(&self, w: &GWire, k: usize, a: P, b: P) -> f64 {
        let n = w.xe.len();
        let windowed = |x: usize| x < n && w.win[x].is_some();
        let t = w.tris[k];
        if t == NONE || !(windowed(k.wrapping_sub(1)) || windowed(k)) {
            return 0.0;
        }
        let mut pen = 0.0;
        for &v in &self.mesh.tris[t as usize].v {
            if (k == 0 && v == w.sv) || (k == n && v == w.ev) || self.obs_rho_w(w, v, &self.rules, w.cw[k]) <= 0.0 {
                continue;
            }
            let rc = if v == w.corner[k] { w.crho[k] } else { self.center_rho(w, k, v) };
            let q = self.vpos(v);
            for e in [a, b] {
                pen += (rc - e.dist(q)).max(0.0);
            }
        }
        10.0 * pen
    }

    /// Radius at which chord k of wire w passes the obstacle at vertex c so that it runs
    /// through the centring windows of its end crossings on the edges at c (0 if none).
    /// Bends at a gap slot thus become arcs around the pad instead of kinks: the arcs
    /// around the two pads of a gap touch exactly at its middle.
    fn center_rho(&self, w: &GWire, k: usize, c: u32) -> f64 {
        let mut r = f64::INFINITY;
        // chord k runs from crossing k-1 to crossing k
        for x in [k.wrapping_sub(1), k] {
            if x >= w.xe.len() {
                continue;
            }
            if let Some((lo, hi)) = w.win[x] {
                let ed = &self.mesh.edges[w.xe[x] as usize];
                if ed.v[0] == c {
                    r = r.min(lo);
                } else if ed.v[1] == c {
                    r = r.min(ed.len - hi);
                }
            }
        }
        if r.is_finite() {
            r
        } else {
            0.0
        }
    }

    /// Disk (centre, radius) that chord k of wire w wraps between a and b: its own corner
    /// (with the stacked radius), or else a side corner of the triangle whose obstacle the
    /// straight chord would cut — then the chord bends around it instead of kinking.
    fn chord_wrap(&self, w: &GWire, k: usize, a: P, b: P) -> Option<(P, f64)> {
        let c = w.corner[k];
        if c != NONE {
            let cp = self.vpos(c);
            let r = self.corner_r(w, k, cp, a, b);
            if around(a, b, cp, r).1.is_some() {
                return Some((cp, r));
            }
        }
        let t = w.tris[k];
        if t == NONE {
            return None;
        }
        let n = w.xe.len();
        let mut best: Option<(f64, P, f64)> = None;
        for &v in &self.mesh.tris[t as usize].v {
            if v == c || (k == 0 && v == w.sv) || (k == n && v == w.ev) {
                continue;
            }
            let r = self.obs_rho_w(w, v, &self.rules, w.cw[k]);
            if r <= 0.0 {
                continue;
            }
            let q = self.vpos(v);
            let r = soft_r(r, self.center_rho(w, k, v), q, a, b);
            let depth = r - point_seg_dist(q, a, b);
            if depth > 1e-7 && a.dist(q) >= r - 1e-7 && b.dist(q) >= r - 1e-7 && best.map_or(true, |bb| depth > bb.0) {
                best = Some((depth, q, r));
            }
        }
        best.map(|(_, q, r)| (q, r))
    }

    /// Pieces of chord k of wire w given its endpoints.
    fn pieces_of(&self, w: &GWire, k: usize, a: P, b: P) -> ([(P, P); 2], usize, Option<Arc>) {
        chord_pieces(a, b, self.chord_wrap(w, k, a, b))
    }

    /// Worst clearance violation (>0) of chord k of wire wi (endpoints a,b) against the
    /// obstacles, board edge and other wires' chords in its triangle.
    fn chord_viol(&self, wi: usize, k: usize, a: P, b: P, rules: &Rules) -> f64 {
        let w = &self.wires[wi];
        let t = w.tris[k];
        if t == NONE {
            return 0.0;
        }
        let (pcs, np, _) = self.pieces_of(w, k, a, b);
        let pcs = &pcs[..np];
        let ww = w.chord_w(k);
        let mut worst = f64::NEG_INFINITY;
        for &o in &self.tri_obs[t as usize] {
            let ob = &self.mesh.obs[o as usize];
            if ob.net.is_some() && (ob.net == Some(w.net) || ob.net == w.net2) {
                continue;
            }
            let need = if ob.kind == ObsKind::Keepout { ww / 2.0 } else { gclr(rules, Some(w.net), ob.net) + ww / 2.0 };
            for &(p0, p1) in pcs {
                worst = worst.max(need - ob.shape.seg_dist(p0, p1));
            }
        }
        for &(o0, o1) in &self.tri_out[t as usize] {
            let need = self.edge_clearance + GEOM_MARGIN + ww / 2.0;
            for &(p0, p1) in pcs {
                worst = worst.max(need - crate::geom::seg_seg_dist(p0, p1, o0, o1));
            }
        }
        for &(oi, ok) in &self.tri_ch[t as usize] {
            if oi as usize == wi {
                continue;
            }
            let o = &self.wires[oi as usize];
            if Self::same_net(w, o) {
                continue;
            }
            let ok = ok as usize;
            let (opcs, onp, oarc) = self.pieces_of(o, ok, self.wpt(o, ok), self.wpt(o, ok + 1));
            let need = (ww + o.chord_w(ok)) / 2.0 + gclr(rules, Some(w.net), Some(o.net));
            for &(p0, p1) in pcs {
                for &(q0, q1) in &opcs[..onp] {
                    worst = worst.max(need - crate::geom::seg_seg_dist(p0, p1, q0, q1));
                }
                if let Some(arc) = oarc {
                    let pl = arc.polyline(0.002);
                    for q in pl.windows(2) {
                        worst = worst.max(need - crate::geom::seg_seg_dist(p0, p1, q[0], q[1]));
                    }
                }
            }
        }
        worst
    }

    fn chord_len(&self, w: &GWire, k: usize, a: P, b: P) -> f64 {
        let l = match self.chord_wrap(w, k, a, b) {
            Some((c, r)) => around(a, b, c, r).0,
            None => a.dist(b),
        };
        l + self.center_pen(w, k, a, b)
    }

    /// Sine of the angle at which wire w crosses the edge of crossing k (clamped, so that
    /// spacings measured along the edge stay finite for glancing crossings).
    fn cross_sin(&self, w: &GWire, k: usize) -> f64 {
        let ed = &self.mesh.edges[w.xe[k] as usize];
        let ev = (self.vpos(ed.v[1]) - self.vpos(ed.v[0])).norm();
        let d = (self.wpt(w, k + 2) - self.wpt(w, k)).norm();
        d.cross(ev).abs().max(0.25)
    }

    /// Centring windows. On every edge that runs between the copper of two obstacles the
    /// crossings get equal-gap slots (pad - track - ... - track - pad); each gap is at least
    /// its own clearance rule and the rest of the room is shared equally ("water filling").
    /// The radar turns the resulting gap into a weight: a crossing may leave its slot only
    /// by (1 - weight) of its spare room, so narrow gaps pin the tracks to the slots and
    /// wide ones leave the rubber band free. Pair partners move as one block.
    fn update_windows(&mut self, rules: &Rules, radar: &crate::config::Centering, strength: f64) {
        for w in self.wires.iter_mut() {
            w.win.iter_mut().for_each(|x| *x = None);
        }
        let mut out: Vec<(u32, u32, (f64, f64))> = Vec::new();
        for e in 0..self.edge_x.len() {
            let list = &self.edge_x[e];
            let ext = match self.walls[e] {
                Some(x) if !list.is_empty() => x,
                _ => continue,
            };
            let ed = &self.mesh.edges[e];
            let wall_net = |end: usize| self.mesh.verts[ed.v[end] as usize].obs.and_then(|o| self.mesh.obs[o as usize].net);
            let el: Vec<(&GWire, usize)> = list.iter().map(|&(wi, k)| (&self.wires[wi as usize], k as usize)).collect();
            let n = el.len();
            let own = |w: &GWire, net: Option<NetId>| net.is_some() && (net == Some(w.net) || net == w.net2);
            // a same-net neighbour is no wall to keep away from
            if own(el[0].0, wall_net(0)) || own(el[n - 1].0, wall_net(1)) || el.windows(2).any(|p| Self::same_net(p[0].0, p[1].0)) {
                continue;
            }
            // the radar looks at the copper-to-copper distance between the two pads
            let wt = radar.weight(ed.len - ext[0] - ext[1]);
            if wt <= 0.0 {
                continue;
            }
            // Gaps are measured across the tracks: track i crosses at distance x_i from v0
            // under an angle with sine s_i; a pad is taken as a disk of its copper extent
            // along the edge. Gap j lies before track j (gap n after the last one):
            //   x_0 = (ext0 + w_0/2 + g_0) / s_0
            //   x_i = x_(i-1) + (w_(i-1)/2 + g_i + w_i/2) / m_i,   m_i = min(s_(i-1), s_i)
            //   len - x_(n-1) = (ext1 + w_(n-1)/2 + g_n) / s_(n-1)
            // so sum(a_j g_j) = room. Pair partners keep their centre distance.
            let sins: Vec<f64> = el.iter().map(|&(w, k)| self.cross_sin(w, k)).collect();
            let wid: Vec<f64> = el.iter().map(|&(w, k)| w.cw[k].max(w.cw[k + 1])).collect();
            let mut coef = vec![0.0; n + 1];
            let mut need = vec![0.0; n + 1];
            let mut pair: Vec<Option<f64>> = vec![None; n + 1];
            coef[0] = 1.0 / sins[0];
            coef[n] = 1.0 / sins[n - 1];
            need[0] = gclr(rules, Some(el[0].0.net), wall_net(0));
            need[n] = gclr(rules, Some(el[n - 1].0.net), wall_net(1));
            let mut room = ed.len - (ext[0] + wid[0] / 2.0) / sins[0] - (ext[1] + wid[n - 1] / 2.0) / sins[n - 1];
            for i in 1..n {
                let (a, b) = (el[i - 1].0, el[i].0);
                coef[i] = 1.0 / sins[i - 1].min(sins[i]);
                need[i] = gclr(rules, Some(a.net), Some(b.net));
                pair[i] = match (a.follow, b.follow) {
                    (_, Some((pn, d))) if pn == a.net => Some(d),
                    (Some((pn, d)), _) if pn == b.net => Some(d),
                    _ => None,
                };
                room -= coef[i] * pair[i].unwrap_or((wid[i - 1] + wid[i]) / 2.0);
            }
            // water filling: every gap gets at least its own clearance rule, the gaps whose
            // rule is below the common level share the rest equally
            let mut free: Vec<usize> = (0..=n).filter(|&j| pair[j].is_none()).collect();
            let mut level;
            loop {
                let a: f64 = free.iter().map(|&j| coef[j]).sum();
                if free.is_empty() || a <= 0.0 {
                    level = f64::NAN;
                    break;
                }
                level = room / a;
                let (over, rest): (Vec<usize>, Vec<usize>) = free.iter().partition(|&&j| need[j] > level);
                if over.is_empty() {
                    break;
                }
                room -= over.iter().map(|&j| coef[j] * need[j]).sum::<f64>();
                free = rest;
            }
            // spare room (across the track) of the tightest equal-share gap
            let spare = free.iter().map(|&j| level - need[j]).fold(f64::INFINITY, f64::min);
            if !(spare >= 0.0) || !level.is_finite() {
                continue; // not even the rules fit: plain bounds decide
            }
            let gap = |j: usize| need[j].max(level);
            let mut x = (ext[0] + wid[0] / 2.0 + gap(0)) / sins[0];
            for i in 0..n {
                if i > 0 {
                    x += coef[i] * pair[i].unwrap_or((wid[i - 1] + wid[i]) / 2.0 + gap(i));
                }
                let half = (1.0 - wt * strength) * spare / sins[i];
                out.push((list[i].0, list[i].1, (x - half, x + half)));
            }
        }
        for (wi, k, x) in out {
            self.wires[wi as usize].win[k as usize] = Some(x);
        }
        // a wire that bends around a pad crosses a fan of edges at it; it keeps from the pad
        // the distance of the narrowest gap of the fan (one arc), not the middle of every
        // edge (the middle of a pin cell's diagonal lies further out and would add kinks)
        for wi in 0..self.wires.len() {
            let n = self.wires[wi].xe.len();
            let mut k = 1;
            while k < n {
                let c = self.shared_vertex(self.wires[wi].xe[k - 1], self.wires[wi].xe[k]);
                if c == NONE {
                    k += 1;
                    continue;
                }
                // run of crossings k0..=k1 on edges at c
                let k0 = k - 1;
                let mut k1 = k;
                while k1 + 1 < n && self.mesh.edges[self.wires[wi].xe[k1 + 1] as usize].v.contains(&c) {
                    k1 += 1;
                }
                let near = |x: usize| -> Option<f64> {
                    let w = &self.wires[wi];
                    let (lo, hi) = w.win[x]?;
                    let ed = &self.mesh.edges[w.xe[x] as usize];
                    Some(if ed.v[0] == c { lo } else { ed.len - hi })
                };
                if let Some(r) = (k0..=k1).filter_map(near).reduce(f64::min) {
                    for x in k0..=k1 {
                        let len = self.mesh.edges[self.wires[wi].xe[x] as usize].len;
                        let at_v0 = self.mesh.edges[self.wires[wi].xe[x] as usize].v[0] == c;
                        if let Some((lo, hi)) = self.wires[wi].win[x].as_mut() {
                            if at_v0 {
                                *lo = lo.min(r);
                            } else {
                                *hi = hi.max(len - r);
                            }
                        }
                    }
                }
                k = k1 + 1;
            }
        }
    }

    /// Vertex shared by two edges (NONE if they have none).
    fn shared_vertex(&self, e1: u32, e2: u32) -> u32 {
        let (a, b) = (&self.mesh.edges[e1 as usize], &self.mesh.edges[e2 as usize]);
        a.v.iter().copied().find(|v| b.v.contains(v)).unwrap_or(NONE)
    }

    /// Spacing bounds for crossing (wire wi, index k) from its neighbours on the edge.
    fn bounds(&self, wi: u32, k: usize, rules: &Rules) -> (f64, f64) {
        let w = &self.wires[wi as usize];
        let e = w.xe[k];
        let ed = &self.mesh.edges[e as usize];
        let list = &self.edge_x[e as usize];
        let pos = list.iter().position(|&(a, b)| a == wi && b as usize == k).unwrap();
        // width at this crossing (max of the two chords meeting here)
        let wx = w.cw[k].max(w.cw[k + 1]);
        let relief = (w.width - wx).max(0.0) / 2.0;
        let mut lo = if w.xlo[k] > 0.0 { (w.xlo[k] - relief).max(0.0) } else { w.xlo[k] };
        let mut hi = if w.xhi[k] < ed.len { (w.xhi[k] + relief).min(ed.len) } else { w.xhi[k] };
        let sin_of = |o: &GWire, kk: usize| self.cross_sin(o, kk);
        let s_self = sin_of(w, k);
        // differential pair follower: lock to the leader crossing right next to it
        if let Some((pn, dist)) = w.follow {
            let near_end = {
                let x = self.wpt(w, k + 1);
                x.dist(w.start).min(x.dist(w.end)) < 1.2
            };
            if !near_end {
                for (j, dir) in [(pos.wrapping_sub(1), 1.0), (pos + 1, -1.0)] {
                    if j >= list.len() {
                        continue;
                    }
                    let (oi, ok) = list[j];
                    let o = &self.wires[oi as usize];
                    if o.net != pn {
                        continue;
                    }
                    let t0 = o.xt[ok as usize] + dir * dist / s_self.min(sin_of(o, ok as usize));
                    let (l2, h2) = (t0 - 0.002, t0 + 0.002);
                    if l2 >= lo && h2 <= hi {
                        return (l2, h2);
                    }
                }
            }
        }
        for j in (0..pos).rev() {
            let (oi, ok) = list[j];
            let o = &self.wires[oi as usize];
            if Self::same_net(w, o) {
                continue;
            }
            let s = (wx + o.cw[ok as usize].max(o.cw[ok as usize + 1])) / 2.0 + gclr(rules, Some(w.net), Some(o.net));
            lo = lo.max(o.xt[ok as usize] + s / s_self.min(sin_of(o, ok as usize)));
            break;
        }
        for j in pos + 1..list.len() {
            let (oi, ok) = list[j];
            let o = &self.wires[oi as usize];
            if Self::same_net(w, o) {
                continue;
            }
            let s = (wx + o.cw[ok as usize].max(o.cw[ok as usize + 1])) / 2.0 + gclr(rules, Some(w.net), Some(o.net));
            hi = hi.min(o.xt[ok as usize] - s / s_self.min(sin_of(o, ok as usize)));
            break;
        }
        (lo, hi)
    }

    /// One Gauss–Seidel sweep. Returns max parameter change and count of infeasible crossings.
    fn sweep(&mut self, rules: &Rules, backward: bool) -> (f64, usize) {
        let mut maxd: f64 = 0.0;
        let mut infeasible = 0;
        // alternating the direction carries changes along long wires both ways
        let nw = self.wires.len();
        for wo in 0..nw {
            let wi = if backward { nw - 1 - wo } else { wo };
            let n = self.wires[wi].xe.len();
            for ko in 0..n {
                let k = if backward { n - 1 - ko } else { ko };
                let (lo, hi) = self.bounds(wi as u32, k, rules);
                let w = &self.wires[wi];
                let e = w.xe[k];
                let prev = self.wpt(w, k);
                let next = self.wpt(w, k + 2);
                let f = |t: f64| self.chord_len(w, k, prev, self.edge_pt(e, t)) + self.chord_len(w, k + 1, self.edge_pt(e, t), next);
                let viol = |t: f64| {
                    let x = self.edge_pt(e, t);
                    self.chord_viol(wi, k, prev, x, rules).max(self.chord_viol(wi, k + 1, x, next, rules))
                };
                let old = w.xt[k];
                // feasible sub-interval w.r.t. corner obstacles: Ok(interval) or
                // Err(least violating point)
                let feasible = |lo: f64, hi: f64| -> Result<(f64, f64), f64> {
                    const NS: usize = 12;
                    let ts: Vec<f64> = (0..=NS).map(|i| lo + (hi - lo) * i as f64 / NS as f64).collect();
                    let vs: Vec<f64> = ts.iter().map(|&t| viol(t)).collect();
                    let feas: Vec<bool> = vs.iter().map(|&v| v <= 1e-6).collect();
                    if feas.iter().all(|&x| x) {
                        return Ok((lo, hi));
                    }
                    let refine = |good: f64, bad: f64| {
                        let (mut g, mut b) = (good, bad);
                        for _ in 0..30 {
                            let m = 0.5 * (g + b);
                            if viol(m) <= 1e-6 {
                                g = m;
                            } else {
                                b = m;
                            }
                        }
                        g
                    };
                    // runs of feasible samples, each with its best sampled objective
                    let mut runs: Vec<(usize, usize, f64)> = Vec::new();
                    let mut i = 0;
                    while i <= NS {
                        if feas[i] {
                            let i0 = i;
                            while i < NS && feas[i + 1] {
                                i += 1;
                            }
                            let best = (i0..=i).map(|j| f(ts[j])).fold(f64::INFINITY, f64::min);
                            runs.push((i0, i, best));
                        }
                        i += 1;
                    }
                    // the run the crossing stands in now: it stays there unless another run
                    // is clearly shorter (else such crossings hop between the two sides of an
                    // obstacle sweep after sweep)
                    let o = old.clamp(lo, hi);
                    let here = if viol(o) <= 1e-6 {
                        match runs.iter().position(|&(i0, i1, _)| ts[i0.saturating_sub(1)] <= o && o <= ts[(i1 + 1).min(NS)]) {
                            Some(r) => Some(runs[r]),
                            None => {
                                // a narrow clean gap between the samples
                                let j = ((o - lo) / (hi - lo).max(1e-12) * NS as f64).floor().clamp(0.0, (NS - 1) as f64) as usize;
                                return Ok((refine(o, ts[j]), refine(o, ts[j + 1])));
                            }
                        }
                    } else {
                        None
                    };
                    let best = runs.iter().copied().min_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
                    let pick = match (here, best) {
                        (Some(h), Some(b)) if h.2 <= b.2 + 1e-3 => Some(h),
                        (_, b) => b,
                    };
                    match pick {
                        Some((i0, i1, _)) => {
                            let a = if i0 > 0 { refine(ts[i0], ts[i0 - 1]) } else { ts[0] };
                            let b = if i1 < NS { refine(ts[i1], ts[i1 + 1]) } else { ts[NS] };
                            Ok((a, b))
                        }
                        None => {
                            // nothing feasible: least violation
                            let i = (0..=NS).min_by(|&i, &j| vs[i].partial_cmp(&vs[j]).unwrap()).unwrap();
                            Err(ts[i])
                        }
                    }
                };
                let (lo, hi, ok) = if lo <= hi {
                    // centring window first (clipped to the bounds); the rules always win:
                    // without a clean spot inside the window the whole interval is used
                    let centred = w.win[k].and_then(|(a, b)| feasible(a.clamp(lo, hi), b.clamp(lo, hi)).ok());
                    match centred.map(Ok).unwrap_or_else(|| feasible(lo, hi)) {
                        Ok((a, b)) => (a, b, true),
                        Err(t) => (t, t, false),
                    }
                } else {
                    (lo, hi, false)
                };
                let t = if lo > hi || !ok {
                    infeasible += 1;
                    if lo > hi {
                        0.5 * (lo + hi)
                    } else {
                        lo
                    }
                } else {
                    // golden section on [lo, hi]
                    let g = 0.618_033_988_75;
                    let (mut a, mut b) = (lo, hi);
                    let mut x1 = b - g * (b - a);
                    let mut x2 = a + g * (b - a);
                    let mut f1 = f(x1);
                    let mut f2 = f(x2);
                    for _ in 0..40 {
                        if b - a < 1e-6 {
                            break;
                        }
                        if f1 < f2 {
                            b = x2;
                            x2 = x1;
                            f2 = f1;
                            x1 = b - g * (b - a);
                            f1 = f(x1);
                        } else {
                            a = x1;
                            x1 = x2;
                            f1 = f2;
                            x2 = a + g * (b - a);
                            f2 = f(x2);
                        }
                    }
                    let mut t = 0.5 * (a + b);
                    // keep the old value if it is as good (stability)
                    if f(old.clamp(lo, hi)) <= f(t) + 1e-9 {
                        t = old.clamp(lo, hi);
                    }
                    t
                };
                maxd = maxd.max((t - old).abs());
                self.wires[wi].xt[k] = t;
            }
        }
        (maxd, infeasible)
    }

    /// GRIDLESS_DEBUG_NET=<net>: dump the wires of that net on this layer.
    fn debug_net(&self, r: &Router, rules: &Rules) {
        if let Ok(dn) = std::env::var("GRIDLESS_DEBUG_NET") {
            let (lg, layer) = (self, self.layer);
            for (wi, w) in lg.wires.iter().enumerate() {
                if r.board.nets[w.net].name != dn {
                    continue;
                }
                eprintln!("== wire {wi} net {dn} layer {layer} start {:?} end {:?}", w.start, w.end);
                for pr in lg.prims(wi) {
                    match pr {
                        Prim::Seg(a, b) => eprintln!("  seg ({:.3},{:.3})-({:.3},{:.3})", a.x, a.y, b.x, b.y),
                        Prim::Arc(a) => eprintln!("  arc c({:.3},{:.3}) r {:.3} a0 {:.1} sw {:.1}  ({:.3},{:.3})->({:.3},{:.3})", a.c.x, a.c.y, a.r, a.a0.to_degrees(), a.sweep.to_degrees(), a.start().x, a.start().y, a.end().x, a.end().y),
                    }
                }
                for k in 0..w.xe.len() {
                    let ed = &lg.mesh.edges[w.xe[k] as usize];
                    let (lo, hi) = lg.bounds(wi as u32, k, rules);
                    let lab = |v: u32| {
                        let mv = &lg.mesh.verts[v as usize];
                        format!("{:?}r{:.2}{}", mv.pos, mv.r, mv.obs.map(|o| format!(" obs{}:{:?}", o, lg.mesh.obs[o as usize].net.map(|n| r.board.nets[n].name.clone()))).unwrap_or_default())
                    };
                    let prev = lg.wpt(w, k);
                    let x = lg.wpt(w, k + 1);
                    let next = lg.wpt(w, k + 2);
                    eprintln!(
                        "  x{k}: t={:.4} [{:.3},{:.3}] win {:?} len {:.3} v0 {} | v1 {} | chord corner {} rho {:.4} crho {:.4} viol {:.3}/{:.3}",
                        w.xt[k], lo, hi, w.win[k], ed.len, lab(ed.v[0]), lab(ed.v[1]), w.corner[k], w.rho[k], w.crho[k],
                        lg.chord_viol(wi, k, prev, x, rules), lg.chord_viol(wi, k + 1, x, next, rules)
                    );
                }
            }
        }
    }

    /// Relaxation of the wire positions: a block step every few sweeps (each wire pulled
    /// straight through its corridor at once, `funnel_step`) and coordinate sweeps that
    /// alternate direction in between (arcs, clearances, bundles). `hook` runs before every
    /// sweep. Stops after `min` sweeps once nothing moves. Returns (sweeps, last change,
    /// crossings without a clean position).
    fn relax(&mut self, rules: &Rules, min: usize, max: usize, mut hook: impl FnMut(&mut Self, usize)) -> (usize, f64, usize) {
        let debug = std::env::var("GRIDLESS_DEBUG_SWEEP").is_ok();
        let mut it = 0;
        let mut prev: Vec<Vec<f64>> = Vec::new();
        let mut last_len = f64::INFINITY;
        let (mut best_len, mut stall) = (f64::INFINITY, 0);
        loop {
            hook(self, it);
            self.update_rho(rules);
            // block steps: the funnel straightens wires early on, extrapolation then
            // takes the long slow creeps in one go
            let df = if it < 12 && it % 4 == 0 {
                self.funnel_step(rules)
            } else if it % 3 == 2 && !prev.is_empty() {
                self.extrapolate_step(rules, &prev)
            } else {
                0.0
            };
            if df > 0.0 {
                self.update_rho(rules);
            }
            prev = self.wires.iter().map(|w| w.xt.clone()).collect();
            let before = if debug && it + 3 >= max { prev.clone() } else { Vec::new() };
            let (ds, inf) = self.sweep(rules, it % 2 == 1);
            for (wi, w) in self.wires.iter().enumerate().filter(|_| !before.is_empty()) {
                for k in 0..w.xt.len() {
                    if (w.xt[k] - before[wi][k]).abs() > 5e-5 {
                        eprintln!("sweep {it} layer {} wire {wi} net {} k {k}: {:.4} -> {:.4} at {:?}", self.layer, w.net, before[wi][k], w.xt[k], self.wpt(w, k + 1));
                    }
                }
            }
            it += 1;
            let d = ds.max(df);
            // done when nothing moves, when what still creeps (well under a micrometre per
            // sweep) no longer shortens the layer measurably, or when the layer has stopped
            // getting shorter at all: squeezed crossings without a clean spot can push each
            // other round in circles, and more sweeps change nothing for the better
            let len = self.total_len();
            let settled = d < 2e-5 || (d < 1e-3 && last_len - len < 1e-5);
            last_len = len;
            if len < best_len - 1e-5 {
                (best_len, stall) = (len, 0);
            } else {
                stall += 1;
            }
            if ((settled || stall >= 8) && it >= min) || it >= max {
                return (it, d, inf);
            }
        }
    }

    /// Objective of all wires on the layer (length with arcs and centring penalties).
    fn total_len(&self) -> f64 {
        let mut len = 0.0;
        for w in &self.wires {
            for k in 0..=w.xe.len() {
                len += self.chord_len(w, k, self.wpt(w, k), self.wpt(w, k + 1));
            }
        }
        len
    }

    /// (objective, total clearance violation) of wire wi as it lies now.
    fn wire_score(&self, wi: usize, rules: &Rules) -> (f64, f64) {
        let w = &self.wires[wi];
        let (mut len, mut bad) = (0.0, 0.0);
        for k in 0..=w.xe.len() {
            let (a, b) = (self.wpt(w, k), self.wpt(w, k + 1));
            len += self.chord_len(w, k, a, b);
            bad += self.chord_viol(wi, k, a, b, rules).max(0.0);
        }
        (len, bad)
    }

    /// Extrapolation step: where the sweeps keep moving a wire the same way (they creep
    /// along long wires, a little per sweep), try 3, 9, 27... times that last move at once
    /// and keep the shortest clean result. `prev` = positions before the last sweep.
    fn extrapolate_step(&mut self, rules: &Rules, prev: &[Vec<f64>]) -> f64 {
        let mut maxd: f64 = 0.0;
        for wi in 0..self.wires.len() {
            let n = self.wires[wi].xe.len();
            let d: Vec<f64> = (0..n).map(|k| self.wires[wi].xt[k] - prev[wi][k]).collect();
            if d.iter().all(|x| x.abs() < 1e-7) {
                continue;
            }
            let lims: Vec<(f64, f64)> = (0..n).map(|k| self.bounds(wi as u32, k, rules)).collect();
            let base = self.wires[wi].xt.clone();
            let (l0, v0) = self.wire_score(wi, rules);
            let mut best: Option<(f64, Vec<f64>)> = None;
            for m in [3.0, 9.0, 27.0, 81.0] {
                let cand: Vec<f64> = (0..n)
                    .map(|k| {
                        let (lo, hi) = lims[k];
                        let t = base[k] + m * d[k];
                        if lo <= hi {
                            t.clamp(lo, hi)
                        } else {
                            base[k]
                        }
                    })
                    .collect();
                self.wires[wi].xt = cand.clone();
                let (l, v) = self.wire_score(wi, rules);
                if v <= v0 + 1e-9 && l < best.as_ref().map_or(l0 - 1e-9, |b| b.0) {
                    best = Some((l, cand));
                } else {
                    break; // the gain has turned
                }
            }
            match best {
                Some((_, xt)) => {
                    for k in 0..n {
                        maxd = maxd.max((xt[k] - base[k]).abs());
                    }
                    self.wires[wi].xt = xt;
                }
                None => self.wires[wi].xt = base,
            }
        }
        maxd
    }

    /// Block step of the relaxation: every wire is pulled straight through its corridor at
    /// once - the funnel (string pulling) over the current bounds of its crossings, the
    /// other wires held still. Crossing-by-crossing descent needs hundreds of sweeps to
    /// straighten a long wire; this does it in one. The funnel knows no arcs, so a wire
    /// takes the new positions only when they are no longer and no less clean (in practice:
    /// early on, while wires are still far from tight). Returns the largest move.
    fn funnel_step(&mut self, rules: &Rules) -> f64 {
        let mut maxd: f64 = 0.0;
        let score = |lg: &Self, wi: usize| lg.wire_score(wi, rules);
        for wi in 0..self.wires.len() {
            let n = self.wires[wi].xe.len();
            if n == 0 {
                continue;
            }
            let mut portals = Vec::with_capacity(n);
            let mut lims = Vec::with_capacity(n);
            for k in 0..n {
                let (mut lo, mut hi) = self.bounds(wi as u32, k, rules);
                if !(lo <= hi) {
                    break;
                }
                if let Some((a, b)) = self.wires[wi].win[k] {
                    (lo, hi) = (a.clamp(lo, hi), b.clamp(lo, hi));
                }
                let w = &self.wires[wi];
                let e = w.xe[k];
                let tri = &self.mesh.tris[w.tris[k] as usize];
                // exiting a CCW triangle: v[le] is on the right
                let fwd = self.mesh.edges[e as usize].v[0] == tri.v[tri.edge_idx(e)];
                let (pl, ph) = (self.edge_pt(e, lo), self.edge_pt(e, hi));
                portals.push(if fwd { (ph, pl) } else { (pl, ph) });
                lims.push((lo, hi));
            }
            if lims.len() < n {
                continue; // a crossing without room: leave the wire to the sweeps
            }
            let w = &self.wires[wi];
            let path = funnel(w.start, w.end, &portals);
            let mut next = w.xt.clone();
            let mut j = 0;
            for k in 0..n {
                let ed = &self.mesh.edges[w.xe[k] as usize];
                let (a, b) = (self.vpos(ed.v[0]), self.vpos(ed.v[1]));
                let (lo, hi) = lims[k];
                for jj in j..path.len().saturating_sub(1) {
                    if let Some((u, sp)) = crate::geom::line_intersect(a, b, path[jj], path[jj + 1]) {
                        let t = u * ed.len;
                        if (-1e-6..=1.0 + 1e-6).contains(&sp) && t >= lo - 1e-6 && t <= hi + 1e-6 {
                            next[k] = t.clamp(lo, hi);
                            j = jj;
                            break;
                        }
                    }
                }
            }
            let (l0, v0) = score(self, wi);
            let old = std::mem::replace(&mut self.wires[wi].xt, next);
            let (l1, v1) = score(self, wi);
            if v1 <= v0 + 1e-9 && l1 <= l0 + 1e-9 {
                for k in 0..n {
                    maxd = maxd.max((self.wires[wi].xt[k] - old[k]).abs());
                }
            } else {
                self.wires[wi].xt = old;
            }
        }
        maxd
    }

    /// Gap equalisation on a relaxed layer: pull the crossings into their centring windows
    /// and let the rest of every wire settle around them.
    ///
    /// The slots are an ideal: in a dense cluster pulling a track all the way into its
    /// slot can leave less than a rule to a neighbour. Instead of dropping the whole layer
    /// ("kept off"), try full strength and, if it costs a rule, repeat at half and a
    /// quarter of the pull — the track still drifts to the middle of the widest free
    /// channel, just not blindly.
    fn equalise_gaps(&mut self, rules: &Rules, radar: &crate::config::Centering) {
        self.walls = edge_walls(&self.mesh);
        let snapshot: Vec<(Vec<f64>, Vec<f64>)> =
            self.wires.iter().map(|w| (w.xt.clone(), w.cw.clone())).collect();
        let before = self.violation_sum(rules);
        for strength in [1.0, 0.5, 0.25] {
            for (w, (xt, cw)) in self.wires.iter_mut().zip(snapshot.iter()) {
                w.xt.clone_from(xt);
                w.cw.clone_from(cw);
            }
            // the slots depend on the crossing angles, which move with the tracks: refresh
            // them a few times, then freeze them so that the relaxation can settle
            const REFRESH: usize = 20;
            let (it, d, _) = self.relax(rules, REFRESH + 1, 50, |lg, it| {
                if it % 5 == 0 && it <= REFRESH {
                    lg.update_windows(rules, radar, strength);
                }
            });
            self.update_rho(rules);
            let after = self.violation_sum(rules);
            if after <= before + 1e-9 {
                log::info!(
                    "layer {}: gaps equalised at {}% in {} sweeps (last change {:.2e})",
                    self.layer, (strength * 100.0).round(), it, d
                );
                return;
            }
        }
        for (w, (xt, cw)) in self.wires.iter_mut().zip(snapshot.iter()) {
            w.xt.clone_from(xt);
            w.cw.clone_from(cw);
        }
        self.update_rho(rules);
        log::info!("layer {}: gap equalisation kept off (it cost clearance)", self.layer);
    }

    /// Total clearance shortfall of the layer's chords (mm), so a finishing pass cannot
    /// trade looks for rules.
    fn violation_sum(&self, rules: &Rules) -> f64 {
        let mut s = 0.0;
        for wi in 0..self.wires.len() {
            for k in 0..=self.wires[wi].xe.len() {
                let a = self.wpt(&self.wires[wi], k);
                let b = self.wpt(&self.wires[wi], k + 1);
                s += self.chord_viol(wi, k, a, b, rules).max(0.0);
            }
        }
        s
    }

    /// Build primitives (segments + arcs) of wire wi.
    /// Narrow chords that still lack clearance (never below `min_w`).
    fn auto_neck(&mut self, rules: &Rules, min_w: f64) {
        for _pass in 0..4 {
            let mut changed = false;
            for wi in 0..self.wires.len() {
                for k in 0..=self.wires[wi].xe.len() {
                    let (a, b) = (self.wpt(&self.wires[wi], k), self.wpt(&self.wires[wi], k + 1));
                    let v = self.chord_viol(wi, k, a, b, rules);
                    if v > 1e-6 {
                        let cur = self.wires[wi].cw[k];
                        let nw = (cur - 2.0 * v - 0.002).max(min_w.min(cur));
                        if nw < cur - 1e-6 {
                            self.wires[wi].cw[k] = nw;
                            changed = true;
                        }
                    }
                }
            }
            if !changed {
                break;
            }
        }
    }

    /// Primitives with their widths; runs of equal width are simplified separately.
    pub fn prims_w(&self, wi: usize) -> (Vec<Prim>, Vec<f64>) {
        let w = &self.wires[wi];
        let n = w.xe.len();
        let mut runs: Vec<(f64, Vec<Prim>)> = Vec::new();
        for k in 0..=n {
            let cw = w.chord_w(k);
            let one = self.chord_prims(wi, k);
            match runs.last_mut() {
                Some((rw, v)) if (*rw - cw).abs() < 1e-9 => v.extend(one),
                _ => runs.push((cw, one)),
            }
        }
        let mut prims = Vec::new();
        let mut pw = Vec::new();
        for (cw, v) in runs {
            for pr in simplify_prims(v) {
                prims.push(pr);
                pw.push(cw);
            }
        }
        (prims, pw)
    }

    fn chord_prims(&self, wi: usize, k: usize) -> Vec<Prim> {
        let w = &self.wires[wi];
        let a = self.wpt(w, k);
        let b = self.wpt(w, k + 1);
        if let Some((cp, r)) = self.chord_wrap(w, k, a, b) {
            if let (_, Some((t1, t2))) = around(a, b, cp, r) {
                let a0 = (t1 - cp).angle();
                let sw = crate::geom::norm_angle((t2 - cp).angle() - a0);
                return vec![Prim::Seg(a, t1), Prim::Arc(Arc { c: cp, r, a0, sweep: sw }), Prim::Seg(t2, b)];
            }
        }
        vec![Prim::Seg(a, b)]
    }

    pub fn prims(&self, wi: usize) -> Vec<Prim> {
        let n = self.wires[wi].xe.len();
        let mut raw: Vec<Prim> = Vec::new();
        for k in 0..=n {
            raw.extend(self.chord_prims(wi, k));
        }
        simplify_prims(raw)
    }
}

/// Width a track may have where it enters the pad / via at mesh vertex v.
fn neck_width(r: &Router, mesh: &Mesh, v: u32, width: f64, rules: &Rules) -> f64 {
    let mv = &mesh.verts[v as usize];
    let lim = if let Some(pd) = mv.pad {
        r.board.pads[pd as usize].shape.min_width()
    } else if mv.site.is_some() {
        rules.via_dia
    } else {
        width
    };
    width.min(lim.max(r.board.rules.min_track_width))
}

/// Start point of a band: for a differential pair the midpoint of its P and N pads.
fn pair_anchor(r: &Router, pb: &Problem, conn: usize, pad: Option<u32>, pos: P) -> P {
    let cn = &pb.conns[conn];
    if cn.net2.is_none() {
        return pos;
    }
    let pad = match pad {
        Some(p) => p as usize,
        None => return pos,
    };
    let partner = match (cn.a, cn.b) {
        (crate::problem::Term::Pad(a), _) if a == pad => cn.a2,
        (_, crate::problem::Term::Pad(b)) if b == pad => cn.b2,
        _ => None,
    };
    match partner {
        Some(n) => pos.mid(r.board.pads[n].pos),
        None => pos,
    }
}

/// Unit direction of a primitive at its start (or end).
pub fn prim_dir(pr: &Prim, at_start: bool) -> P {
    match pr {
        Prim::Seg(a, b) => (*b - *a).norm(),
        Prim::Arc(a) => {
            let ang = if at_start { a.a0 } else { a.a0 + a.sweep };
            let radial = p(ang.cos(), ang.sin());
            if a.sweep >= 0.0 {
                radial.perp()
            } else {
                -radial.perp()
            }
        }
    }
}

/// Offset a chain of primitives sideways by `d` (positive = left of travel direction).
pub fn offset_prims(prims: &[Prim], d: f64) -> Vec<Prim> {
    let mut out: Vec<Prim> = Vec::new();
    for pr in prims {
        let o = match pr {
            Prim::Seg(a, b) => {
                let n = (*b - *a).norm().perp();
                Prim::Seg(*a + n * d, *b + n * d)
            }
            Prim::Arc(a) => {
                // left of travel is towards the centre for a CCW (positive) sweep
                let r = if a.sweep >= 0.0 { a.r - d } else { a.r + d };
                if r <= 1e-6 {
                    let s = a.start();
                    let e = a.end();
                    Prim::Seg(s, e)
                } else {
                    Prim::Arc(Arc { c: a.c, r, a0: a.a0, sweep: a.sweep })
                }
            }
        };
        if let Some(last) = out.last_mut() {
            let le = last.end();
            let ns = o.start();
            if le.dist(ns) > 1e-6 {
                // kink: join segments at their line intersection, else bridge
                let mut joined = false;
                if let (Prim::Seg(a0, a1), Prim::Seg(b0, b1)) = (*last, o) {
                    if let Some((t, u)) = crate::geom::line_intersect(a0, a1, b0, b1) {
                        if t > 0.0 && t < 1.5 && u > -0.5 && u < 1.0 {
                            let x = a0.lerp(a1, t);
                            *last = Prim::Seg(a0, x);
                            out.push(Prim::Seg(x, b1));
                            joined = true;
                        }
                    }
                }
                if joined {
                    continue;
                }
                out.push(Prim::Seg(le, ns));
            }
        }
        out.push(o);
    }
    out
}

/// Merge collinear segments and co-circular arcs, drop degenerate pieces.
pub fn simplify_prims(raw: Vec<Prim>) -> Vec<Prim> {
    let mut out: Vec<Prim> = Vec::new();
    for pr in raw {
        if pr.len() < 1e-5 {
            // keep continuity: extend previous end to this end
            if let Some(last) = out.last_mut() {
                if let Prim::Seg(a, _) = *last {
                    *last = Prim::Seg(a, pr.end());
                }
            }
            continue;
        }
        if let Some(last) = out.last_mut() {
            match (*last, pr) {
                (Prim::Seg(a, b), Prim::Seg(_, c)) => {
                    let d1 = (b - a).norm();
                    let d2 = (c - b).norm();
                    // The mesh splits a straight run at every triangle it crosses, so a
                    // run of collinear prims must merge when the middle vertex lies on
                    // the chord within a small tolerance. Testing the sine of the angle
                    // alone (1e-7) leaves the run split and the track looks jagged.
                    let ac = c - a;
                    let dev = if ac.len() > 1e-9 { (b - a).cross(ac).abs() / ac.len() } else { 0.0 };
                    if d1.dot(d2) > 0.0 && dev < 0.01 {
                        *last = Prim::Seg(a, c);
                        continue;
                    }
                }
                (Prim::Arc(x), Prim::Arc(y)) => {
                    if x.c.dist(y.c) < 1e-6 && (x.r - y.r).abs() < 1e-6 && x.sweep.signum() == y.sweep.signum() {
                        *last = Prim::Arc(Arc { c: x.c, r: x.r, a0: x.a0, sweep: x.sweep + y.sweep });
                        continue;
                    }
                }
                _ => {}
            }
        }
        out.push(pr);
    }
    out
}

/// Snap a polyline to the 45 degree grid: a segment that is not a multiple of 45
/// degrees becomes a 45 degree stub plus a straight one, between the same endpoints.
/// Hand routing is drawn this way; arbitrary angles are what makes a router's board
/// look alien. Arcs are left alone.
pub fn snap45(prims: Vec<Prim>) -> Vec<Prim> {
    let mut out: Vec<Prim> = Vec::with_capacity(prims.len());
    for pr in prims {
        let (a, b) = match pr {
            Prim::Seg(a, b) => (a, b),
            Prim::Arc(_) => {
                out.push(pr);
                continue;
            }
        };
        let d = b - a;
        let l = d.len();
        if l < 1e-6 {
            out.push(Prim::Seg(a, b));
            continue;
        }
        let mut ang = d.y.atan2(d.x).to_degrees() % 45.0;
        if ang < 0.0 {
            ang += 45.0;
        }
        let dev = ang.min(45.0 - ang);
        if dev <= 2.0 {
            out.push(Prim::Seg(a, b));
            continue;
        }
        // the diagonal leg takes the shorter side, the straight leg the rest
        let m = if d.x.abs() >= d.y.abs() {
            p(a.x + d.y.abs() * d.x.signum(), a.y + d.y)
        } else {
            p(a.x + d.x, a.y + d.x.abs() * d.y.signum())
        };
        out.push(Prim::Seg(a, m));
        out.push(Prim::Seg(m, b));
    }
    out
}

/// `snap45` keeping the per-primitive widths aligned with the new primitives.
pub fn snap45_w(prims: Vec<Prim>, pw: Vec<f64>) -> (Vec<Prim>, Vec<f64>) {
    let mut out: Vec<Prim> = Vec::with_capacity(prims.len());
    let mut ow: Vec<f64> = Vec::with_capacity(pw.len());
    for (pr, w) in prims.into_iter().zip(pw) {
        let n = out.len();
        out.extend(snap45(vec![pr]));
        for _ in n..out.len() {
            ow.push(w);
        }
    }
    (out, ow)
}

/// Walk a polyline from vertex sv to vertex ev through the mesh, returning crossed edges,
/// their parameters (distance from edge.v[0]) and the chord triangles.
fn trace(m: &Mesh, sv: u32, ev: u32, pts: &[P]) -> Option<(Vec<u32>, Vec<f64>, Vec<u32>)> {
    let pos = |v: u32| m.verts[v as usize].pos;
    if pts.len() < 2 {
        return None;
    }
    // first triangle: wedge at sv containing the first direction
    let d0 = {
        let mut k = 1;
        while k + 1 < pts.len() && pts[k].dist(pts[0]) < 1e-9 {
            k += 1;
        }
        pts[k] - pts[0]
    };
    let mut cur = NONE;
    let mut best_score = f64::NEG_INFINITY;
    for &t in &m.vtris[sv as usize] {
        let tri = &m.tris[t as usize];
        let ci = tri.vert_idx(sv).unwrap();
        let a = pos(tri.v[(ci + 1) % 3]) - pos(sv);
        let b = pos(tri.v[(ci + 2) % 3]) - pos(sv);
        // inside wedge if left of a and right of b
        let s = a.norm().cross(d0.norm()).min(d0.norm().cross(b.norm()));
        if s > best_score {
            best_score = s;
            cur = t;
        }
    }
    if cur == NONE {
        return None;
    }
    let mut xe = Vec::new();
    let mut xt = Vec::new();
    let mut tris = Vec::new();
    let mut entry_edge;
    let mut from = pts[0];
    for si in 1..pts.len() {
        let to = pts[si];
        let last = si + 1 == pts.len();
        let mut guard = 0;
        entry_edge = NONE;
        loop {
            guard += 1;
            if guard > 100_000 {
                return None;
            }
            let tri = &m.tris[cur as usize];
            if last && tri.v.contains(&ev) {
                break;
            }
            let (a, b, c) = (pos(tri.v[0]), pos(tri.v[1]), pos(tri.v[2]));
            let inside = orient(a, b, to) >= -1e-12 && orient(b, c, to) >= -1e-12 && orient(c, a, to) >= -1e-12;
            if inside && !last {
                break;
            }
            // exit edge: the ray from->to leaves the (CCW) triangle through the edge whose
            // outward normal it points along, at the smallest ray parameter
            let dir = to - from;
            let mut best: Option<(u32, f64, f64)> = None; // edge, param on segment, param on edge
            for li in 0..3 {
                let e = tri.e[li];
                if e == entry_edge {
                    continue;
                }
                let a = pos(tri.v[li]);
                let b = pos(tri.v[(li + 1) % 3]);
                let d = b - a;
                let outward = p(d.y, -d.x);
                if dir.dot(outward) <= 1e-15 {
                    continue;
                }
                let ed = &m.edges[e as usize];
                let (p0, p1) = (pos(ed.v[0]), pos(ed.v[1]));
                if let Some((s, u)) = crate::geom::line_intersect(from, to, p0, p1) {
                    if u > -1e-7 && u < 1.0 + 1e-7 && s > -1e-7 {
                        if best.map_or(true, |bb| s < bb.1) {
                            best = Some((e, s, u.clamp(1e-6, 1.0 - 1e-6) * ed.len));
                        }
                    }
                }
            }
            let (e, s, u) = match best {
                Some(b) => b,
                None => {
                    log::debug!("trace: no exit from tri {:?} seg {}/{} from {:?} to {:?} (entry {})", m.tris[cur as usize].v.map(|v| pos(v)), si, pts.len() - 1, from, to, entry_edge);
                    return None;
                }
            };
            if s > 1.0 + 1e-9 && !last {
                // numerical: target slightly outside; accept current triangle
                break;
            }
            let nt = m.edges[e as usize].other_tri(cur);
            if nt == NONE {
                log::debug!("trace: left the mesh through edge {e} at seg {si}/{}", pts.len() - 1);
                return None;
            }
            tris.push(cur);
            xe.push(e);
            xt.push(u);
            entry_edge = e;
            cur = nt;
        }
        from = to;
        let _ = sv;
    }
    tris.push(cur);
    // remove back-tracking (crossing the same edge twice in a row)
    let mut changed = true;
    while changed {
        changed = false;
        let mut k = 0;
        while k + 1 < xe.len() {
            if xe[k] == xe[k + 1] {
                xe.drain(k..k + 2);
                xt.drain(k..k + 2);
                tris.drain(k + 1..k + 3);
                changed = true;
            } else {
                k += 1;
            }
        }
    }
    Some((xe, xt, tris))
}

/// Can a via of `net` stand at q: inside the board, clear of foreign copper, keepouts,
/// holes and the other vias?
fn via_spot_ok(pb: &Problem, rules: &Rules, q: P, net: NetId, others: &[(P, NetId)]) -> bool {
    let vr = rules.via_dia / 2.0;
    if !crate::problem::inside_board(q, &pb.outline) {
        return false;
    }
    for poly in &pb.outline {
        for k in 0..poly.len() {
            if point_seg_dist(q, poly[k], poly[(k + 1) % poly.len()]) < vr + rules.edge_clearance + GEOM_MARGIN {
                return false;
            }
        }
    }
    for o in &pb.obstacles {
        if o.kind == ObsKind::Keepout {
            if crate::geom::point_in_poly(q, &o.shape.pts) {
                return false;
            }
            continue;
        }
        if let Some(d) = o.drill {
            if q.dist(o.shape.centroid()) < d / 2.0 + rules.via_drill / 2.0 + rules.hole_to_hole {
                return false;
            }
        }
        if o.net != Some(net) && o.shape.dist(q) < vr + gclr(rules, Some(net), o.net) {
            return false;
        }
    }
    others.iter().all(|&(p2, n2)| {
        let d = q.dist(p2);
        d >= rules.via_drill + rules.hole_to_hole && (n2 == net || d >= rules.via_dia + gclr(rules, Some(net), Some(n2)))
    })
}

/// For every edge that runs through free space between the copper of two different
/// obstacles (pads, vias, fixed copper; not keepouts or the board edge): how far the
/// copper of each end reaches along the edge.
fn edge_walls(mesh: &Mesh) -> Vec<Option<[f64; 2]>> {
    mesh.edges
        .iter()
        .map(|ed| {
            if ed.owner.is_some() {
                return None;
            }
            let (va, vb) = (&mesh.verts[ed.v[0] as usize], &mesh.verts[ed.v[1] as usize]);
            let (oa, ob) = (va.obs?, vb.obs?);
            let copper = |o: u32| mesh.obs[o as usize].kind != ObsKind::Keepout;
            if oa == ob || !copper(oa) || !copper(ob) {
                return None;
            }
            let ea = exit_dist(&mesh.obs[oa as usize].shape, va.pos, vb.pos, 0.0);
            let eb = exit_dist(&mesh.obs[ob as usize].shape, vb.pos, va.pos, 0.0);
            (ea + eb < ed.len - 1e-6).then_some([ea, eb])
        })
        .collect()
}

/// For every triangle: obstacles / outline segments whose clearance zone may reach it.
fn near_lists(mesh: &Mesh, outline: &[Vec<P>], rules: &Rules) -> (Vec<Vec<u32>>, Vec<Vec<(P, P)>>) {
    let max_clr = rules.nets.iter().map(|n| n.clearance).fold(rules.default_clearance, f64::max);
    let max_w = rules.nets.iter().map(|n| n.width).fold(0.0, f64::max).max(rules.diff_pairs.iter().map(|d| 2.0 * d.width + d.gap).fold(0.0, f64::max));
    let margin = max_clr.max(rules.edge_clearance) + max_w / 2.0 + 0.05;
    let cell = 1.0;
    let mut grid: std::collections::HashMap<(i32, i32), Vec<u32>> = std::collections::HashMap::new();
    for (i, o) in mesh.obs.iter().enumerate() {
        let bb = o.shape.bbox().grow(margin);
        for x in (bb.min.x / cell).floor() as i32..=(bb.max.x / cell).floor() as i32 {
            for y in (bb.min.y / cell).floor() as i32..=(bb.max.y / cell).floor() as i32 {
                grid.entry((x, y)).or_default().push(i as u32);
            }
        }
    }
    let mut segs: Vec<(P, P)> = Vec::new();
    for poly in outline {
        for k in 0..poly.len() {
            segs.push((poly[k], poly[(k + 1) % poly.len()]));
        }
    }
    let mut tri_obs = Vec::with_capacity(mesh.tris.len());
    let mut tri_out = Vec::with_capacity(mesh.tris.len());
    for t in &mesh.tris {
        if !t.alive {
            tri_obs.push(Vec::new());
            tri_out.push(Vec::new());
            continue;
        }
        let mut bb = crate::geom::BBox::empty();
        for &v in &t.v {
            bb.add(mesh.verts[v as usize].pos);
        }
        let mut list: Vec<u32> = Vec::new();
        for x in (bb.min.x / cell).floor() as i32..=(bb.max.x / cell).floor() as i32 {
            for y in (bb.min.y / cell).floor() as i32..=(bb.max.y / cell).floor() as i32 {
                if let Some(v) = grid.get(&(x, y)) {
                    list.extend_from_slice(v);
                }
            }
        }
        list.sort_unstable();
        list.dedup();
        list.retain(|&o| mesh.obs[o as usize].shape.bbox().grow(margin).overlaps(&bb));
        tri_obs.push(list);
        let g = bb.grow(margin);
        tri_out.push(
            segs.iter()
                .copied()
                .filter(|&(a, b)| {
                    let mut sb = crate::geom::BBox::empty();
                    sb.add(a);
                    sb.add(b);
                    sb.overlaps(&g)
                })
                .collect(),
        );
    }
    (tri_obs, tri_out)
}

pub struct Geometry {
    pub layers: Vec<LayerGeom>,
    pub tracks: Vec<Track>,
    pub vias: Vec<(P, NetId)>,
    pub warnings: Vec<String>,
}

/// Move each via off its lattice site towards the line through the terminals it joins.
///
/// With two neighbours every point of the segment gives the same wire length, so the
/// closest point to the lattice site wins: that is exactly what removes the horseshoe
/// without making the net longer. With more neighbours the median shortens it.
fn relax_via_positions(r: &Router, pb: &Problem, rules: &Rules, used: &mut [(u32, P, NetId)]) {
    let mut nb: Vec<Vec<P>> = vec![Vec::new(); pb.sites.len()];
    for w in &r.st.wires {
        if !w.alive {
            continue;
        }
        let rm = &r.meshes[w.li];
        if let Some(s) = rm.verts[w.start as usize].site {
            nb[s as usize].push(rm.verts[w.end as usize].pos);
        }
        if let Some(s) = rm.verts[w.end as usize].site {
            nb[s as usize].push(rm.verts[w.start as usize].pos);
        }
    }
    let mut moved = 0;
    for i in 0..used.len() {
        let s = used[i].0 as usize;
        let net = used[i].2;
        let q0 = used[i].1;
        let points = &nb[s];
        if points.is_empty() {
            continue;
        }
        let mut cands: Vec<P> = Vec::new();
        if points.len() == 2 {
            let (a, b) = (points[0], points[1]);
            let ab = b - a;
            let len2 = ab.dot(ab);
            if len2 > 1e-9 {
                let t = ((q0 - a).dot(ab) / len2).clamp(0.0, 1.0);
                for dt in [0.0, 0.08, -0.08, 0.16, -0.16, 0.28, -0.28] {
                    cands.push(a + ab * (t + dt).clamp(0.0, 1.0));
                }
            }
        } else {
            let mut c = P::default();
            for q in points {
                c = c + *q;
            }
            let c = c / points.len() as f64;
            cands.push(c);
            for (dx, dy) in [(0.3, 0.0), (-0.3, 0.0), (0.0, 0.3), (0.0, -0.3)] {
                cands.push(c + P { x: dx, y: dy });
            }
        }
        let others: Vec<(P, NetId)> = used
            .iter()
            .enumerate()
            .filter(|&(j, _)| j != i)
            .map(|(_, u)| (u.1, u.2))
            .collect();
        let mut best: Option<(P, f64)> = None;
        let move_cap: f64 = std::env::var("GRIDLESS_VIA_RELAX_R")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(1.2);
        for q in cands {
            let d0 = q.dist(q0);
            if d0 > move_cap || d0 < 1e-6 || !via_spot_ok(pb, rules, q, net, &others) {
                continue;
            }
            let score = points.iter().map(|p| p.dist(q)).sum::<f64>() + 0.02 * d0;
            if best.map_or(true, |(_, s)| score < s) {
                best = Some((q, score));
            }
        }
        if let Some((q, _)) = best {
            used[i].1 = q;
            moved += 1;
        }
    }
    if moved > 0 {
        log::info!("{moved} via(s) moved off the lattice to straighten the wires");
    }
}

pub fn realize(r: &Router, pb: &Problem, rules: &Rules) -> Geometry {
    let emb = r.embed();
    let via_r = rules.via_dia / 2.0;
    let mut used: Vec<(u32, P, NetId)> = r.st.site_use.iter().enumerate().filter_map(|(s, u)| u.map(|(n, _)| (s as u32, pb.sites[s].pos, n))).collect();
    // differential pair vias side by side: a follower's via moves from its lattice site to
    // the exact spot beside its leader's via, when that spot is clear
    for (s, q) in r.pair_via_spots() {
        if let Some(i) = used.iter().position(|u| u.0 == s) {
            let others: Vec<(P, NetId)> = used.iter().enumerate().filter(|&(j, _)| j != i).map(|(_, u)| (u.1, u.2)).collect();
            if via_spot_ok(pb, rules, q, used[i].2, &others) {
                used[i].1 = q;
            }
        }
    }
    // The rubber band never moves a via: its position comes from the via lattice. When a
    // via sits off the line between the terminals it joins, the wires bend around it (a
    // horseshoe) instead of pulling it straight. Put each via on that line, two
    // neighbours, or at their median for more, keeping it clear of foreign copper.
    if std::env::var("GRIDLESS_NO_VIA_RELAX").is_err() {
        relax_via_positions(r, pb, rules, &mut used);
    }
    // layers are independent: one thread each
    use rayon::prelude::*;
    let per_layer: Vec<(LayerGeom, Vec<String>)> = r.meshes.par_iter().enumerate().map(|(li, rm)| {
        let mut warnings = Vec::new();
        let layer = rm.layer;
        let mut obs: Vec<MObs> = rm.obs.clone();
        for &(_, pos, net) in &used {
            obs.push(MObs { anchor: None, shape: crate::geom::RoundPoly::circle(pos, via_r), net: Some(net), pad: None, kind: ObsKind::Via });
        }
        let sites: Vec<(u32, P)> = used.iter().map(|&(s, pos, _)| (s, pos)).collect();
        let mesh = Mesh::build_from(obs, &pb.outline, &sites, layer, rules.default_clearance, rules.edge_clearance);
        if let Ok(pt) = std::env::var("GRIDLESS_DEBUG_POINT") {
            let n: Vec<f64> = pt.split(',').filter_map(|x| x.parse().ok()).collect();
            if n.len() == 2 {
                let q = p(n[0], n[1]);
                for (i, v) in mesh.verts.iter().enumerate() {
                    if v.pos.dist(q) < 0.3 {
                        let ob = v.obs.map(|o| (mesh.obs[o as usize].kind, mesh.obs[o as usize].net.map(|n| r.board.nets[n].name.clone()), mesh.obs[o as usize].shape.clone()));
                        eprintln!("layer {layer} vert {i} {:?} r {:.3} site {:?} pad {:?} obs {:?}", v.pos, v.r, v.site, v.pad, ob);
                    }
                }
            }
        }
        let (tri_obs, tri_out) = near_lists(&mesh, &pb.outline, rules);
        let mut lg = LayerGeom {
            layer,
            edge_x: vec![Vec::new(); mesh.edges.len()],
            tri_ch: vec![Vec::new(); mesh.tris.len()],
            mesh,
            wires: Vec::new(),
            edge_clearance: rules.edge_clearance,
            outline: pb.outline.clone(),
            tri_obs,
            tri_out,
            walls: Vec::new(),
            rules: rules.clone(),
        };
        let map_v = |v: u32| -> Option<u32> {
            let mv = &rm.verts[v as usize];
            if let Some(pd) = mv.pad {
                return lg.mesh.pad_vert.get(&pd).copied();
            }
            if let Some(s) = mv.site {
                return lg.mesh.site_vert.get(&s).copied();
            }
            None
        };
        // ---- thread wires
        let mut tmp_t: Vec<Vec<(u32, u32, f64)>> = vec![Vec::new(); lg.mesh.edges.len()];
        for ew in emb.iter().filter(|e| e.li == li) {
            let tw = &r.st.wires[ew.wire as usize];
            let (sv, ev) = match (map_v(tw.start), map_v(tw.end)) {
                (Some(a), Some(b)) => (a, b),
                _ => {
                    warnings.push(format!("wire {} endpoints not in realisation mesh", ew.wire));
                    continue;
                }
            };
            let mut pts = ew.pts.clone();
            pts[0] = lg.mesh.verts[sv as usize].pos;
            *pts.last_mut().unwrap() = lg.mesh.verts[ev as usize].pos;
            let (xe, xt, tris) = match trace(&lg.mesh, sv, ev, &pts) {
                Some(x) => x,
                None => {
                    warnings.push(format!("cannot thread wire {} on layer {}", ew.wire, layer));
                    continue;
                }
            };
            let wi = lg.wires.len() as u32;
            let n = xe.len();
            for k in 0..n {
                tmp_t[xe[k] as usize].push((wi, k as u32, xt[k]));
            }
            for (k, &t) in tris.iter().enumerate() {
                lg.tri_ch[t as usize].push((wi, k as u32));
            }
            lg.wires.push(GWire {
                conn: tw.conn,
                net: tw.net,
                net2: tw.net2,
                width: tw.width,
                clr: tw.clr,
                sv,
                ev,
                start: pair_anchor(r, pb, tw.conn as usize, lg.mesh.verts[sv as usize].pad, lg.mesh.verts[sv as usize].pos),
                end: pair_anchor(r, pb, tw.conn as usize, lg.mesh.verts[ev as usize].pad, lg.mesh.verts[ev as usize].pos),
                xe,
                xt,
                xlo: vec![0.0; n],
                xhi: vec![0.0; n],
                tris,
                corner: vec![NONE; n + 1],
                rho: vec![0.0; n + 1],
                crho: vec![0.0; n + 1],
                neck: [neck_width(r, &lg.mesh, sv, tw.width, rules), neck_width(r, &lg.mesh, ev, tw.width, rules)],
                cw: Vec::new(),
                win: vec![None; n],
                reserve: r.reserve[tw.conn as usize],
                follow: match tw.couple {
                    Some((pi, false)) => Some((rules.diff_pairs[pi].p, rules.diff_pairs[pi].width + rules.diff_pairs[pi].gap)),
                    _ => None,
                },
            });
        }
        for w in lg.wires.iter_mut() {
            w.init_widths();
        }
        for (e, mut l) in tmp_t.into_iter().enumerate() {
            l.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
            lg.edge_x[e] = l.into_iter().map(|(w, k, _)| (w, k)).collect();
        }
        // ---- obstacle bounds per crossing
        for wi in 0..lg.wires.len() {
            for k in 0..lg.wires[wi].xe.len() {
                let w = &lg.wires[wi];
                let e = w.xe[k];
                let ed = &lg.mesh.edges[e as usize];
                let mut b = [0.0f64; 2];
                for end in 0..2 {
                    let v = ed.v[end];
                    let mv = &lg.mesh.verts[v as usize];
                    let a = mv.pos;
                    let o = lg.mesh.verts[ed.v[1 - end] as usize].pos;
                    b[end] = if let Some(oi) = mv.obs {
                        let ob = &lg.mesh.obs[oi as usize];
                        if ob.net.is_some() && (ob.net == Some(w.net) || ob.net == w.net2) {
                            0.0
                        } else {
                            exit_dist(&ob.shape, a, o, gclr(rules, Some(w.net), ob.net) + w.width / 2.0)
                        }
                    } else if mv.outline {
                        outline_exit(&lg.outline, a, o, rules.edge_clearance + GEOM_MARGIN + w.width / 2.0)
                    } else {
                        0.0
                    };
                }
                let len = ed.len;
                lg.wires[wi].xlo[k] = b[0].min(len);
                lg.wires[wi].xhi[k] = (len - b[1]).max(0.0);
            }
        }
        // ---- initial positions: independent funnel per wire
        for wi in 0..lg.wires.len() {
            let w = &lg.wires[wi];
            let n = w.xe.len();
            if n == 0 {
                continue;
            }
            let mut portals = Vec::with_capacity(n);
            for k in 0..n {
                let e = w.xe[k];
                let ed = &lg.mesh.edges[e as usize];
                let t = w.tris[k];
                let tri = &lg.mesh.tris[t as usize];
                let le = tri.edge_idx(e);
                // exiting CCW triangle through edge (v_le -> v_le+1): left = v_le+1, right = v_le
                let fwd = ed.v[0] == tri.v[le];
                let (lo, hi) = if w.xlo[k] <= w.xhi[k] { (w.xlo[k], w.xhi[k]) } else { let m = 0.5 * (w.xlo[k] + w.xhi[k]); (m, m) };
                let p_lo = lg.edge_pt(e, lo);
                let p_hi = lg.edge_pt(e, hi);
                // param grows from ed.v[0]; if fwd, v[0] is the right endpoint
                portals.push(if fwd { (p_hi, p_lo) } else { (p_lo, p_hi) });
            }
            let path = funnel(w.start, w.end, &portals);
            let mut xt = w.xt.clone();
            for k in 0..n {
                let e = w.xe[k];
                let ed = &lg.mesh.edges[e as usize];
                let a = lg.vpos(ed.v[0]);
                let b = lg.vpos(ed.v[1]);
                for s in path.windows(2) {
                    if let Some((u, sp)) = crate::geom::line_intersect(a, b, s[0], s[1]) {
                        if (-1e-6..=1.0 + 1e-6).contains(&sp) && (-1e-6..=1.0 + 1e-6).contains(&u) {
                            xt[k] = (u * ed.len).clamp(w.xlo[k].min(w.xhi[k]), w.xhi[k].max(w.xlo[k]));
                            break;
                        }
                    }
                }
            }
            lg.wires[wi].xt = xt;
        }
        // ---- enforce ordering along edges (push apart)
        for e in 0..lg.edge_x.len() {
            let list = lg.edge_x[e].clone();
            if list.len() < 2 {
                continue;
            }
            for i in 1..list.len() {
                let (pw, pk) = list[i - 1];
                let (cw, ck) = list[i];
                let (pt, pwid, pnet) = (lg.wires[pw as usize].xt[pk as usize], lg.wires[pw as usize].width, lg.wires[pw as usize].net);
                let c = &lg.wires[cw as usize];
                let s = if pnet == c.net { 0.0 } else { (pwid + c.width) / 2.0 + gclr(rules, Some(pnet), Some(c.net)) };
                if c.xt[ck as usize] < pt + s {
                    lg.wires[cw as usize].xt[ck as usize] = pt + s;
                }
            }
        }
        // ---- relax
        let t_relax = std::time::Instant::now();
        let (it, d, infeasible) = lg.relax(rules, 4, 200, |_, _| {});
        log::info!("layer {}: relaxed in {} sweeps (last change {:.2e}, infeasible crossings {}) in {} ms", layer, it, d, infeasible, t_relax.elapsed().as_millis());
        lg.update_rho(rules);
        // neck down where room is short, then let positions settle with the new widths
        for _ in 0..3 {
            lg.auto_neck(rules, r.board.rules.min_track_width);
            lg.relax(rules, 1, 25, |_, _| {});
        }
        // ---- centring: spread the tracks evenly between neighbouring pads
        if let Some(radar) = rules.centering {
            lg.equalise_gaps(rules, &radar);
        }
        lg.update_rho(rules);
        lg.auto_neck(rules, r.board.rules.min_track_width);
        lg.update_rho(rules);
        lg.debug_net(r, rules);
        if infeasible > 0 {
            warnings.push(format!("layer {layer}: {infeasible} crossings without enough room"));
        }
        (lg, warnings)
    }).collect();
    let mut layers = Vec::new();
    let mut warnings = Vec::new();
    for (lg, w) in per_layer {
        layers.push(lg);
        warnings.extend(w);
    }
    let tracks = build_tracks(r, pb, rules, &layers);
    let vias = used.iter().map(|&(_, pos, n)| (pos, n)).collect();
    let _ = p(0.0, 0.0);
    Geometry { layers, tracks, vias, warnings }
}

/// Tracks of all realised wires (pairs split into their two conductors) plus the escape
/// stubs of the pads they connect.
fn build_tracks(r: &Router, pb: &Problem, rules: &Rules, layers: &[LayerGeom]) -> Vec<Track> {
    let mut tracks = Vec::new();
    for lg in layers {
        for wi in 0..lg.wires.len() {
            let w = &lg.wires[wi];
            let prims = lg.prims(wi);
            match (w.net2, pb.conns[w.conn as usize].kind) {
                (Some(n2), crate::problem::ConnKind::Pair { pair }) => {
                    let dp = &rules.diff_pairs[pair];
                    let cn = &pb.conns[w.conn as usize];
                    let pad_pos = |t: crate::problem::Term| match t {
                        crate::problem::Term::Pad(p) => r.board.pads[p].pos,
                        _ => w.start,
                    };
                    let (pa, pbp) = (pad_pos(cn.a), pad_pos(cn.b));
                    let (na, nb) = (cn.a2.map(|x| r.board.pads[x].pos).unwrap_or(pa), cn.b2.map(|x| r.board.pads[x].pos).unwrap_or(pbp));
                    // the band may run b->a
                    let (p_start, n_start, p_end, n_end) = if w.start.dist(pa.mid(na)) <= w.start.dist(pbp.mid(nb)) { (pa, na, pbp, nb) } else { (pbp, nb, pa, na) };
                    let off = (dp.width + dp.gap) / 2.0;
                    // which side is P: left of the initial direction?
                    let d0 = prims.first().map(|q| prim_dir(q, true)).unwrap_or(p(1.0, 0.0));
                    let p_left = d0.cross(p_start - w.start) > 0.0;
                    let (sp, sn) = if p_left { (off, -off) } else { (-off, off) };
                    for (net, side, ps, pe) in [(w.net, sp, p_start, p_end), (n2, sn, n_start, n_end)] {
                        let mut pr = offset_prims(&prims, side);
                        if let Some(f) = pr.first().map(|q| q.start()) {
                            pr.insert(0, Prim::Seg(ps, f));
                        }
                        if let Some(l) = pr.last().map(|q| q.end()) {
                            pr.push(Prim::Seg(l, pe));
                        }
                        let prs = simplify_prims(pr);
                        let pw = vec![dp.width; prs.len()];
                        tracks.push(Track { layer: lg.layer, net, width: dp.width, conn: w.conn, prims: prs, pw });
                    }
                }
                _ => {
                    let (prims, pw) = lg.prims_w(wi);
                    // the length-matching reserve is room around the track, not copper
                    let real = w.width - 2.0 * w.reserve;
                    let pw = pw.into_iter().map(|x| x.min(real)).collect();
                    tracks.push(Track { layer: lg.layer, net: w.net, width: real, conn: w.conn, prims, pw })
                }
            }
        }
    }
    // escape stubs of the pads that got connected
    let mut stub_done = vec![false; r.board.pads.len()];
    for lg in layers {
        for w in &lg.wires {
            for v in [w.sv, w.ev] {
                if let Some(pd) = lg.mesh.verts[v as usize].pad {
                    let pd = pd as usize;
                    if let Some((e, sw)) = pb.escape[pd] {
                        if !stub_done[pd] {
                            stub_done[pd] = true;
                            let pad = &r.board.pads[pd];
                            tracks.push(Track { layer: lg.layer, net: pad.net.unwrap_or(w.net), width: sw, conn: w.conn, prims: vec![Prim::Seg(pad.pos, e)], pw: vec![sw] });
                        }
                    }
                }
            }
        }
    }
    tracks
}

/// Gap equalisation as post-processing of a realised geometry: spread the tracks evenly
/// between neighbouring pads (radar-weighted) and rebuild them. Topology, vias and the
/// rule clearances stay as they are.
pub fn equalise(r: &Router, pb: &Problem, rules: &Rules, geo: &mut Geometry) {
    let radar = match rules.centering {
        Some(c) => c,
        None => return,
    };
    use rayon::prelude::*;
    geo.layers.par_iter_mut().for_each(|lg| {
        lg.equalise_gaps(rules, &radar);
        lg.update_rho(rules);
        lg.auto_neck(rules, r.board.rules.min_track_width);
        lg.update_rho(rules);
        lg.debug_net(r, rules);
    });
    geo.tracks = build_tracks(r, pb, rules, &geo.layers);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn funnel_straight() {
        // corridor going right with an obstacle corner forcing a bend
        let portals = vec![(p(1.0, 1.0), p(1.0, -1.0)), (p(2.0, 1.0), p(2.0, 0.5)), (p(3.0, 1.0), p(3.0, -1.0))];
        let path = funnel(p(0.0, 0.0), p(4.0, 0.0), &portals);
        assert_eq!(path.len(), 3, "{path:?}");
        assert!(path[1].dist(p(2.0, 0.5)) < 1e-9);
    }
    #[test]
    fn around_disk() {
        let (l, t) = around(p(-2.0, 0.5), p(2.0, 0.5), p(0.0, 0.0), 1.0);
        assert!(t.is_some());
        assert!(l > 4.0);
        let (l2, t2) = around(p(-2.0, 2.0), p(2.0, 2.0), p(0.0, 0.0), 1.0);
        assert!(t2.is_none());
        assert!((l2 - 4.0).abs() < 1e-12);
    }
}
