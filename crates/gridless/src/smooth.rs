//! Final smoothing of the routed tracks.
//!
//! The rubber band is shortest within its topology, which can leave two kinds of blemish:
//! a wire that reaches its pad round a corner of the pad (or turns back over an escape
//! stub) instead of entering it straight, and kinks where a wire bends at a point instead
//! of round an obstacle (typically where a bundle enters a gap at an angle). This pass
//! works on the finished tracks with an exact clearance check against all foreign copper:
//!
//! * straight pad entries: from the pad centre, the farthest point of the track that a
//!   straight segment can reach cleanly replaces the way there;
//! * rounded kinks: a corner between two segments becomes a tangent arc, as large as the
//!   clearances allow, but deviating at most `MAX_BULGE` into the bend (so that tracks
//!   centred in a gap stay centred).
//!
//! Differential pairs are left alone: rounding the two wires independently would break
//! their spacing.

use crate::board::{Board, LayerId, NetId};
use crate::config::Rules;
use crate::geom::{Arc, BBox, RoundPoly, P};
use crate::geometry::{Geometry, Prim, GEOM_MARGIN};
use crate::problem::{ObsKind, Problem};
use std::collections::HashMap;

/// Largest distance a rounded corner may move into its bend (mm).
const MAX_BULGE: f64 = 0.05;
/// Turns smaller than this are left as they are (rad, ~3 deg).
const MIN_TURN: f64 = 0.05;
/// A new pad entry must not turn sharper than this where it joins the track (rad, 60 deg).
const MAX_JOIN_TURN: f64 = std::f64::consts::FRAC_PI_3;
/// Pad entries are straightened only this far along the track (mm): hooks sit at the pad,
/// and a longer shortcut would undo the centring of the track in the gaps beyond.
const ENTRY_REACH: f64 = 2.0;

#[derive(Default, Debug, Clone, Copy)]
pub struct Stats {
    pub entries: usize,
    pub stubs_dropped: usize,
    pub corners: usize,
    /// total track length saved (mm)
    pub saved: f64,
}

struct Item {
    shape: RoundPoly,
    net: Option<NetId>,
    layers: u64,
    keepout: bool,
    track: Option<usize>,
    alive: bool,
}

/// Spatial index of all copper (and keepouts) for clearance queries.
struct Index {
    items: Vec<Item>,
    grid: HashMap<(i32, i32), Vec<u32>>,
    reach: f64,
    outline: Vec<(P, P)>,
    edge: f64,
}

const CELL: f64 = 1.0;

impl Index {
    fn add(&mut self, it: Item) {
        let id = self.items.len() as u32;
        let bb = it.shape.bbox().grow(self.reach);
        for x in (bb.min.x / CELL).floor() as i32..=(bb.max.x / CELL).floor() as i32 {
            for y in (bb.min.y / CELL).floor() as i32..=(bb.max.y / CELL).floor() as i32 {
                self.grid.entry((x, y)).or_default().push(id);
            }
        }
        self.items.push(it);
    }

    fn add_track(&mut self, ti: usize, layer: LayerId, net: NetId, prims: &[Prim], pw: &[f64]) {
        for (pi, pr) in prims.iter().enumerate() {
            let w = pw.get(pi).copied().unwrap_or(0.0);
            for s in pr.polyline(0.002).windows(2) {
                self.add(Item { shape: RoundPoly::capsule(s[0], s[1], w / 2.0), net: Some(net), layers: 1 << layer, keepout: false, track: Some(ti), alive: true });
            }
        }
    }

    fn drop_track(&mut self, ti: usize) {
        for it in self.items.iter_mut().filter(|it| it.track == Some(ti)) {
            it.alive = false;
        }
    }

    /// Can a track of `net` and `width` on `layer` run along `pieces` (ignoring the items of
    /// the tracks in `skip`)?
    fn fits(&self, rules: &Rules, layer: LayerId, net: NetId, width: f64, pieces: &[(P, P)], skip: &[usize]) -> bool {
        let mut seen: Vec<u32> = Vec::new();
        for &(a, b) in pieces {
            seen.clear(); // an item clear of one piece may still be too close to the next
            let mut bb = BBox::empty();
            bb.add(a);
            bb.add(b);
            for x in (bb.min.x / CELL).floor() as i32..=(bb.max.x / CELL).floor() as i32 {
                for y in (bb.min.y / CELL).floor() as i32..=(bb.max.y / CELL).floor() as i32 {
                    for &id in self.grid.get(&(x, y)).map(|v| v.as_slice()).unwrap_or(&[]) {
                        let it = &self.items[id as usize];
                        if !it.alive || it.layers & (1 << layer) == 0 || it.net == Some(net) || it.track.is_some_and(|t| skip.contains(&t)) {
                            continue;
                        }
                        if seen.contains(&id) {
                            continue;
                        }
                        let need = if it.keepout { width / 2.0 } else { rules.clr(Some(net), it.net) + GEOM_MARGIN + width / 2.0 };
                        if it.shape.seg_dist(a, b) < need {
                            return false;
                        }
                        seen.push(id);
                    }
                }
            }
            for &(o0, o1) in &self.outline {
                if crate::geom::seg_seg_dist(a, b, o0, o1) < self.edge + GEOM_MARGIN + width / 2.0 {
                    return false;
                }
            }
        }
        true
    }
}

fn pieces_of(prims: &[Prim]) -> Vec<(P, P)> {
    let mut out = Vec::new();
    for pr in prims {
        for s in pr.polyline(0.002).windows(2) {
            out.push((s[0], s[1]));
        }
    }
    out
}

fn reverse(prims: &[Prim], pw: &[f64]) -> (Vec<Prim>, Vec<f64>) {
    let pr = prims
        .iter()
        .rev()
        .map(|p| match *p {
            Prim::Seg(a, b) => Prim::Seg(b, a),
            Prim::Arc(a) => Prim::Arc(Arc { c: a.c, r: a.r, a0: a.a0 + a.sweep, sweep: -a.sweep }),
        })
        .collect();
    (pr, pw.iter().rev().copied().collect())
}

fn dir_at_start(pr: &Prim) -> P {
    crate::geometry::prim_dir(pr, true)
}

/// Turn angle (0 = straight on) between directions u and v.
fn turn(u: P, v: P) -> f64 {
    u.cross(v).atan2(u.dot(v)).abs()
}

/// Midpoint of the rectangle edge the track should leave through: the edge whose
/// outward normal best matches the direction from the pad centre to `q`. The segment
/// centre -> midpoint is perpendicular to that edge by construction.
fn pad_normals(pd: &crate::board::Pad) -> Vec<P> {
    let pts = &pd.shape.pts;
    if pts.len() != 4 {
        return Vec::new();
    }
    let c = pd.pos;
    let mut out = Vec::new();
    for i in 0..4 {
        let a = pts[i];
        let b = pts[(i + 1) % 4];
        let mid = a.mid(b);
        let e = (b - a).norm();
        if e.len() < 1e-9 {
            continue;
        }
        let mut n = P { x: -e.y, y: e.x };
        if n.dot(mid - c) < 0.0 {
            n = P { x: -n.x, y: -n.y };
        }
        out.push(n);
    }
    out
}

fn pad_edge_mid(pd: &crate::board::Pad, q: P) -> Option<P> {
    let pts = &pd.shape.pts;
    if pts.len() != 4 {
        return None;
    }
    let c = pd.pos;
    let d = q - c;
    if d.len() < 1e-9 {
        return None;
    }
    let u = d.norm();
    let mut best: Option<P> = None;
    let mut bd = -1.0;
    for i in 0..4 {
        let a = pts[i];
        let b = pts[(i + 1) % 4];
        let mid = a.mid(b);
        let e = (b - a).norm();
        let mut n = P { x: -e.y, y: e.x };
        if n.dot(mid - c) < 0.0 {
            n = P { x: -n.x, y: -n.y };
        }
        let al = n.dot(u);
        if al > bd {
            bd = al;
            best = Some(mid);
        }
    }
    best
}

pub fn smooth(board: &Board, pb: &Problem, rules: &Rules, geo: &mut Geometry) -> Stats {
    let max_clr = rules.nets.iter().map(|n| n.clearance).fold(rules.default_clearance, f64::max);
    let max_w = geo.tracks.iter().map(|t| t.width).fold(rules.via_dia, f64::max);
    let mut ix = Index { items: Vec::new(), grid: HashMap::new(), reach: max_clr + max_w + GEOM_MARGIN + 0.05, outline: Vec::new(), edge: rules.edge_clearance };
    for o in &pb.obstacles {
        ix.add(Item { shape: o.shape.clone(), net: o.net, layers: o.layers, keepout: o.kind == ObsKind::Keepout, track: None, alive: true });
    }
    let all = (1u64 << board.layers.len()) - 1;
    for &(pos, n) in &geo.vias {
        ix.add(Item { shape: RoundPoly::circle(pos, rules.via_dia / 2.0), net: Some(n), layers: all, keepout: false, track: None, alive: true });
    }
    for (ti, t) in geo.tracks.iter().enumerate() {
        ix.add_track(ti, t.layer, t.net, &t.prims, &t.pw);
    }
    for poly in &pb.outline {
        for k in 0..poly.len() {
            ix.outline.push((poly[k], poly[(k + 1) % poly.len()]));
        }
    }
    let paired: Vec<NetId> = rules.diff_pairs.iter().flat_map(|d| [d.p, d.n]).collect();
    let mut st = Stats::default();
    let total = |geo: &Geometry| geo.tracks.iter().map(|t| t.prims.iter().map(|p| p.len()).sum::<f64>()).sum::<f64>();
    let len0 = total(geo);

    for ti in 0..geo.tracks.len() {
        let (layer, net) = (geo.tracks[ti].layer, geo.tracks[ti].net);
        if paired.contains(&net) || geo.tracks[ti].prims.is_empty() {
            continue;
        }
        // ---- straight pad entries, at both ends
        for end in 0..2 {
            let t = &geo.tracks[ti];
            let (prims, pw) = if end == 0 { (t.prims.clone(), t.pw.clone()) } else { reverse(&t.prims, &t.pw) };
            let s = prims[0].start();
            // the pad the track starts at: its centre, or the far end of its escape stub
            let mut origin: Option<(P, Option<usize>, usize)> = None;
            for (pi, pd) in board.pads.iter().enumerate() {
                if pd.net != Some(net) || !pd.layers.contains(&layer) {
                    continue;
                }
                // the track starts in the pad (its anchor, or the centroid of an odd-shaped pad)
                if pd.pos.dist(s) < 1e-4 || pd.shape.dist(s) <= 1e-6 {
                    origin = Some((s, None, pi));
                    break;
                }
                if let Some((e, _)) = pb.escape[pi] {
                    if e.dist(s) < 1e-4 {
                        // the stub can go only if no other track of the net starts there
                        let stub = geo.tracks.iter().position(|x| x.net == net && x.layer == layer && x.prims.len() == 1 && x.prims[0].start().dist(pd.pos) < 1e-6 && x.prims[0].end().dist(e) < 1e-6);
                        let others = geo.tracks.iter().enumerate().filter(|&(j, x)| j != ti && Some(j) != stub && x.net == net && x.layer == layer && x.prims.first().is_some_and(|f| f.start().dist(e) < 1e-4 || x.prims.last().unwrap().end().dist(e) < 1e-4)).count();
                        if let (Some(si), 0) = (stub, others) {
                            origin = Some((pd.pos, Some(si), pi));
                        }
                        break;
                    }
                }
            }
            let Some((o, stub, pi)) = origin else { continue };
            let pd = &board.pads[pi];
            // uniform width only (a necked entry stays as it is)
            let w = pw[0];
            // candidates: ends of the primitives (and points along segments), farthest first
            let mut cands: Vec<(usize, P, f64)> = Vec::new(); // (prim index, point, path length to it)
            let mut acc = 0.0;
            for (pi, pr) in prims.iter().enumerate() {
                if (pw[pi] - w).abs() > 1e-9 {
                    break;
                }
                if let Prim::Seg(a, b) = *pr {
                    let len = a.dist(b);
                    let n = (len / 0.1).floor() as usize;
                    for j in 1..n {
                        let f = j as f64 / n as f64;
                        if acc + f * len <= ENTRY_REACH {
                            cands.push((pi, a.lerp(b, f), acc + f * len));
                        }
                    }
                }
                acc += pr.len();
                if acc > ENTRY_REACH {
                    break;
                }
                cands.push((pi, pr.end(), acc));
            }
            let mut skip = vec![ti];
            if let Some(si) = stub {
                skip.push(si);
            }
            let stub_len = if stub.is_some() { o.dist(s) } else { 0.0 };
            for &(pi, q, along) in cands.iter().rev() {
                if o.dist(q) + 0.02 >= along + stub_len {
                    continue; // not worth it
                }
                // what follows q
                let mut rest: Vec<Prim> = Vec::new();
                let mut rest_w: Vec<f64> = Vec::new();
                if let Prim::Seg(_, b) = prims[pi] {
                    if q.dist(b) > 1e-6 {
                        rest.push(Prim::Seg(q, b));
                        rest_w.push(pw[pi]);
                    }
                } else if q.dist(prims[pi].end()) > 1e-6 {
                    continue; // only arc ends
                }
                rest.extend_from_slice(&prims[pi + 1..]);
                rest_w.extend_from_slice(&pw[pi + 1..]);
                let new_dir = (q - o).norm();
                if let Some(nx) = rest.first() {
                    if turn(new_dir, dir_at_start(nx)) > MAX_JOIN_TURN {
                        continue;
                    }
                }
                // Perpendicular pad exit: leave the rectangle through the midpoint of
                // the edge the track heads for. A small extra length is accepted - a
                // clean 90 degree entry is worth more than a hair of copper.
                let mut entry: Vec<Prim> = vec![Prim::Seg(o, q)];
                let mut ok = ix.fits(rules, layer, net, w, &[(o, q)], &skip);
                if rules.pad_exit {
                    if let Some(e) = pad_edge_mid(pd, q) {
                        let extra = o.dist(e) + e.dist(q) - o.dist(q);
                        if extra >= -0.05 && extra <= 0.8 {
                            if ix.fits(rules, layer, net, w, &[(o, e), (e, q)], &skip) {
                                entry = vec![Prim::Seg(o, e), Prim::Seg(e, q)];
                                ok = true;
                            }
                        }
                    }
                }
                if !ok {
                    continue;
                }
                let mut np = entry;
                np.extend(rest);
                let mut nw = vec![w; np.len() - rest_w.len()];
                nw.extend(rest_w);
                let (np, nw) = if end == 0 { (np, nw) } else { reverse(&np, &nw) };
                ix.drop_track(ti);
                geo.tracks[ti].prims = np;
                geo.tracks[ti].pw = nw;
                let t = &geo.tracks[ti];
                ix.add_track(ti, t.layer, t.net, &t.prims, &t.pw);
                if let Some(si) = stub {
                    ix.drop_track(si);
                    geo.tracks[si].prims.clear();
                    geo.tracks[si].pw.clear();
                    st.stubs_dropped += 1;
                }
                st.entries += 1;
                break;
            }
        }
        // ---- perpendicular exits from rectangular pads, whether or not a hook was
        // unwound above: the first segment leaves through the midpoint of the edge the
        // track heads for, so the visible track meets the pad at 90 degrees.
        if rules.pad_exit {
        for end in 0..2 {
            let t = &geo.tracks[ti];
            if t.prims.is_empty() {
                continue;
            }
            let (prims, pw) = if end == 0 { (t.prims.clone(), t.pw.clone()) } else { reverse(&t.prims, &t.pw) };
            let s = prims[0].start();
            let mut found: Option<(usize, P)> = None;
            for (pi, pd) in board.pads.iter().enumerate() {
                if pd.net != Some(net) || !pd.layers.contains(&layer) {
                    continue;
                }
                if pd.pos.dist(s) < 1e-4 {
                    found = Some((pi, pd.pos));
                    break;
                }
            }
            let Some((pi, o)) = found else { continue };
            let pd = &board.pads[pi];
            if pd.shape.pts.len() != 4 || pd.shape.r > 0.3 {
                continue;
            }
            let Prim::Seg(_, q) = prims[0] else { continue };
            let w = pw[0];
            if o.dist(q) < 0.15 {
                continue;
            }
            let u = (q - o).norm();
            let ns = pad_normals(pd);
            if ns.iter().any(|n| n.dot(u) > 0.985) {
                continue; // already within ~10 degrees of an edge normal
            }
            let Some(e) = pad_edge_mid(pd, q) else { continue };
            if o.dist(e) < 1e-3 {
                continue;
            }
            let extra = o.dist(e) + e.dist(q) - o.dist(q);
            if extra > 1.0 {
                continue;
            }
            if !ix.fits(rules, layer, net, w, &[(o, e), (e, q)], &[ti]) {
                continue;
            }
            let mut np = vec![Prim::Seg(o, e), Prim::Seg(e, q)];
            let mut nw = vec![w, w];
            np.extend_from_slice(&prims[1..]);
            nw.extend_from_slice(&pw[1..]);
            let (np, nw) = if end == 0 { (np, nw) } else { reverse(&np, &nw) };
            ix.drop_track(ti);
            geo.tracks[ti].prims = np;
            geo.tracks[ti].pw = nw;
            let t = &geo.tracks[ti];
            ix.add_track(ti, t.layer, t.net, &t.prims, &t.pw);
            st.entries += 1;
        }
        }
        // ---- rounded kinks between two segments
        let mut i = 0;
        while i + 1 < geo.tracks[ti].prims.len() {
            let t = &geo.tracks[ti];
            let (p1, p2) = (t.prims[i], t.prims[i + 1]);
            let (Prim::Seg(a, v), Prim::Seg(v2, b)) = (p1, p2) else {
                i += 1;
                continue;
            };
            let w = t.pw[i];
            if v.dist(v2) > 1e-6 || (t.pw[i + 1] - w).abs() > 1e-9 {
                i += 1;
                continue;
            }
            let (u1, u2) = ((v - a).norm(), (b - v).norm());
            let phi = turn(u1, u2);
            if phi < MIN_TURN || phi > std::f64::consts::PI - 0.05 {
                i += 1;
                continue;
            }
            let tan = (phi / 2.0).tan();
            let bulge = 1.0 / (phi / 2.0).cos() - 1.0;
            let r_cap = (MAX_BULGE / bulge).min(a.dist(v) / tan).min(v.dist(b) / tan);
            let left = u1.cross(u2) > 0.0;
            let mut done = false;
            let mut r = r_cap;
            while r >= 0.03 && !done {
                let tl = r * tan;
                let (q1, q2) = (v - u1 * tl, v + u2 * tl);
                let n1 = if left { u1.perp() } else { -u1.perp() };
                let c = q1 + n1 * r;
                let a0 = (q1 - c).angle();
                let arc = Arc { c, r, a0, sweep: if left { phi } else { -phi } };
                if ix.fits(rules, layer, net, w, &pieces_of(&[Prim::Arc(arc)]), &[ti]) {
                    let t = &mut geo.tracks[ti];
                    let mut np: Vec<Prim> = t.prims[..i].to_vec();
                    let mut nw: Vec<f64> = t.pw[..i].to_vec();
                    if a.dist(q1) > 1e-6 {
                        np.push(Prim::Seg(a, q1));
                        nw.push(w);
                    }
                    np.push(Prim::Arc(arc));
                    nw.push(w);
                    let tail_from = np.len();
                    if q2.dist(b) > 1e-6 {
                        np.push(Prim::Seg(q2, b));
                        nw.push(w);
                    }
                    np.extend_from_slice(&t.prims[i + 2..]);
                    nw.extend_from_slice(&t.pw[i + 2..]);
                    t.prims = np;
                    t.pw = nw;
                    ix.drop_track(ti);
                    let t = &geo.tracks[ti];
                    ix.add_track(ti, t.layer, t.net, &t.prims, &t.pw);
                    st.corners += 1;
                    done = true;
                    i = tail_from;
                } else {
                    r *= 0.7;
                }
            }
            if !done {
                i += 1;
            }
        }
    }
    st.saved = len0 - total(geo);
    st
}
