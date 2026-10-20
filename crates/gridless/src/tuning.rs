//! Length matching: rounded trombone meanders inserted on straight track segments,
//! sized by binary search so that the result stays DRC clean.

use crate::board::{Board, LayerId, NetId};
use crate::config::Rules;
use crate::geom::{p, Arc, BBox, RoundPoly, P};
use crate::geometry::{Geometry, Prim};
use crate::problem::Problem;
use std::collections::HashMap;

#[derive(Clone, Debug, serde::Serialize)]
pub struct TuneResult {
    pub net: String,
    pub before: f64,
    pub after: f64,
    pub target: f64,
    pub ok: bool,
}

/// Routed length of a net (tracks only).
pub fn net_length(g: &Geometry, net: NetId) -> f64 {
    g.tracks.iter().filter(|t| t.net == net).map(|t| t.prims.iter().map(|p| p.len()).sum::<f64>()).sum()
}

struct Obs {
    shape: RoundPoly,
    net: Option<NetId>,
    layers: u64,
}

struct Index {
    items: Vec<Obs>,
    grid: HashMap<(i32, i32), Vec<usize>>,
}

impl Index {
    fn build(b: &Board, pb: &Problem, rules: &Rules, g: &Geometry) -> Index {
        let all = (1u64 << b.layers.len()) - 1;
        let mut items = Vec::new();
        for o in &pb.obstacles {
            items.push(Obs { shape: o.shape.clone(), net: o.net, layers: o.layers });
        }
        for t in &g.tracks {
            for (i, pr) in t.prims.iter().enumerate() {
                let w = t.pw.get(i).copied().unwrap_or(t.width);
                for s in pr.polyline(0.002).windows(2) {
                    items.push(Obs { shape: RoundPoly::capsule(s[0], s[1], w / 2.0), net: Some(t.net), layers: 1 << t.layer });
                }
            }
        }
        for &(pos, n) in &g.vias {
            items.push(Obs { shape: RoundPoly::circle(pos, rules.via_dia / 2.0), net: Some(n), layers: all });
        }
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, it) in items.iter().enumerate() {
            let bb = it.shape.bbox();
            for x in bb.min.x.floor() as i32..=bb.max.x.floor() as i32 {
                for y in bb.min.y.floor() as i32..=bb.max.y.floor() as i32 {
                    grid.entry((x, y)).or_default().push(i);
                }
            }
        }
        Index { items, grid }
    }

    /// Is a track piece (polyline, width w, net, layer) clear of everything else?
    fn clear(&self, pts: &[P], w: f64, net: NetId, layer: LayerId, rules: &Rules, outline: &[Vec<P>]) -> bool {
        let mut bb = BBox::empty();
        for q in pts {
            bb.add(*q);
        }
        let bb = bb.grow(w / 2.0 + 1.0);
        let mut cand: Vec<usize> = Vec::new();
        for x in bb.min.x.floor() as i32..=bb.max.x.floor() as i32 {
            for y in bb.min.y.floor() as i32..=bb.max.y.floor() as i32 {
                if let Some(v) = self.grid.get(&(x, y)) {
                    cand.extend_from_slice(v);
                }
            }
        }
        cand.sort_unstable();
        cand.dedup();
        for s in pts.windows(2) {
            for &i in &cand {
                let it = &self.items[i];
                if it.layers & (1 << layer) == 0 || it.net == Some(net) {
                    continue;
                }
                let need = rules.clr(Some(net), it.net) + w / 2.0 + 0.002;
                if it.shape.seg_dist(s[0], s[1]) < need {
                    return false;
                }
            }
            if !outline.is_empty() {
                for poly in outline {
                    let n = poly.len();
                    for k in 0..n {
                        if crate::geom::seg_seg_dist(s[0], s[1], poly[k], poly[(k + 1) % n]) < rules.edge_clearance + w / 2.0 + 0.002 {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }
}

/// Rounded single-sided trombone along a -> b: `n` bumps of amplitude `amp`, turn radius
/// `r`, on side `side` (+1 left / -1 right of travel direction). Returns the primitives.
pub fn trombone(a: P, b: P, n: usize, amp: f64, r: f64, side: f64) -> Vec<Prim> {
    let len = a.dist(b);
    let d = (b - a) / len;
    let nn = d.perp() * side;
    let bump_w = 4.0 * r;
    let total = n as f64 * bump_w + (n as f64 - 1.0).max(0.0) * 2.0 * r;
    let mut x = (len - total) / 2.0;
    let at = |s: f64, h: f64| a + d * s + nn * h;
    let mut out = vec![Prim::Seg(a, at(x, 0.0))];
    // arc helper: centre, start point, end point, turning towards `ccw`
    let mk = |c: P, s: P, e: P| {
        let a0 = (s - c).angle();
        let sw = crate::geom::norm_angle((e - c).angle() - a0);
        Prim::Arc(Arc { c, r: c.dist(s), a0, sweep: sw })
    };
    for i in 0..n {
        // quarter arc up
        out.push(mk(at(x, r), at(x, 0.0), at(x + r, r)));
        out.push(Prim::Seg(at(x + r, r), at(x + r, amp - r)));
        // semicircle on top (via its midpoint to fix the direction)
        let c = at(x + 2.0 * r, amp - r);
        let s0 = at(x + r, amp - r);
        let e0 = at(x + 3.0 * r, amp - r);
        let top = at(x + 2.0 * r, amp);
        let a0 = (s0 - c).angle();
        let am = (top - c).angle();
        let half1 = crate::geom::norm_angle(am - a0);
        out.push(Prim::Arc(Arc { c, r, a0, sweep: 2.0 * half1 }));
        let _ = e0;
        out.push(Prim::Seg(at(x + 3.0 * r, amp - r), at(x + 3.0 * r, r)));
        // quarter arc down to the base line
        out.push(mk(at(x + 4.0 * r, r), at(x + 3.0 * r, r), at(x + 4.0 * r, 0.0)));
        x += bump_w;
        if i + 1 < n {
            out.push(Prim::Seg(at(x, 0.0), at(x + 2.0 * r, 0.0)));
            x += 2.0 * r;
        }
    }
    out.push(Prim::Seg(at(x, 0.0), b));
    out
}

fn bump_gain(amp: f64, r: f64) -> f64 {
    2.0 * amp + (2.0 * std::f64::consts::PI - 8.0) * r
}

/// Try to add `need` mm to the tracks selected by `sel` (all of net `net`). Returns the
/// length actually added.
fn add_length(g: &mut Geometry, idx: &Index, rules: &Rules, outline: &[Vec<P>], net: NetId, need: f64, max_amp: f64, avoid: Option<NetId>, sel: &dyn Fn(&crate::geometry::Track) -> bool) -> f64 {
    let mut added = 0.0;
    // candidate straight segments, longest first
    let mut cands: Vec<(usize, usize, f64)> = Vec::new();
    for (ti, t) in g.tracks.iter().enumerate() {
        if t.net != net || !sel(t) {
            continue;
        }
        for (pi, pr) in t.prims.iter().enumerate() {
            if let Prim::Seg(a, b) = pr {
                cands.push((ti, pi, a.dist(*b)));
            }
        }
    }
    cands.sort_by(|x, y| y.2.partial_cmp(&x.2).unwrap());
    let mut replaced: Vec<(usize, usize, Vec<Prim>)> = Vec::new();
    for (ti, pi, len) in cands {
        let rem = need - added;
        if rem < 0.005 {
            break;
        }
        let t = &g.tracks[ti];
        let w = t.pw.get(pi).copied().unwrap_or(t.width);
        let r = (w + rules.nets[net].clearance.max(w)) / 2.0;
        let margin = 2.0 * r; // keep away from the segment ends (corners / pads)
        let usable = len - 2.0 * margin;
        let n_max = ((usable + 2.0 * r) / (6.0 * r)).floor() as usize;
        if n_max == 0 {
            continue;
        }
        let (a, b) = match t.prims[pi] {
            Prim::Seg(a, b) => (a, b),
            _ => continue,
        };
        let amp_min = 2.0 * r + 0.05;
        let mut best: Option<(f64, Vec<Prim>)> = None;
        for side in [1.0, -1.0] {
            // skip the side facing the diff-pair partner
            if let Some(pn) = avoid {
                let mid = a.mid(b) + (b - a).norm().perp() * side * (w + 0.3);
                let near = g.tracks.iter().any(|o| o.net == pn && o.layer == t.layer && o.prims.iter().any(|q| q.polyline(0.01).windows(2).any(|s| crate::geom::point_seg_dist(mid, s[0], s[1]) < w + 0.3)));
                if near {
                    continue;
                }
            }
            // largest clean amplitude for the count that covers the remainder
            for n in 1..=n_max {
                let amp_needed = (rem / n as f64 - (2.0 * std::f64::consts::PI - 8.0) * r) / 2.0;
                let amp_hi = amp_needed.min(max_amp);
                if amp_hi < amp_min {
                    break;
                }
                let ok = |amp: f64| {
                    let prs = trombone(a, b, n, amp, r, side);
                    let pts: Vec<P> = prs.iter().flat_map(|q| q.polyline(0.005)).collect();
                    idx.clear(&pts, w, net, t.layer, rules, outline)
                };
                let amp = if ok(amp_hi) {
                    amp_hi
                } else if ok(amp_min) {
                    let (mut lo, mut hi) = (amp_min, amp_hi);
                    for _ in 0..20 {
                        let m = 0.5 * (lo + hi);
                        if ok(m) {
                            lo = m;
                        } else {
                            hi = m;
                        }
                    }
                    lo
                } else {
                    continue;
                };
                let gain = n as f64 * bump_gain(amp, r);
                if best.as_ref().map_or(true, |bb| gain > bb.0 + 1e-9) {
                    best = Some((gain, trombone(a, b, n, amp, r, side)));
                }
                if gain >= rem - 0.005 {
                    break;
                }
            }
        }
        if let Some((gain, prs)) = best {
            added += gain;
            replaced.push((ti, pi, prs));
        }
    }
    // apply replacements (highest prim index first per track)
    replaced.sort_by(|x, y| (x.0, y.1).cmp(&(y.0, x.1)));
    for (ti, pi, prs) in replaced {
        let t = &mut g.tracks[ti];
        let w = t.pw.get(pi).copied().unwrap_or(t.width);
        let n = prs.len();
        t.prims.splice(pi..pi + 1, prs);
        t.pw.splice(pi..pi + 1, std::iter::repeat(w).take(n));
    }
    added
}

/// Match lengths of the configured groups and the skew of differential pairs.
pub fn tune(b: &Board, pb: &Problem, rules: &Rules, g: &mut Geometry) -> Vec<TuneResult> {
    let mut out = Vec::new();
    let outline = pb.outline.clone();
    // length groups
    for grp in &rules.length_groups {
        let lens: Vec<f64> = grp.nets.iter().map(|&n| net_length(g, n)).collect();
        let target = grp.target.unwrap_or_else(|| lens.iter().cloned().fold(0.0, f64::max));
        for (i, &n) in grp.nets.iter().enumerate() {
            let before = lens[i];
            if before <= 0.0 {
                continue;
            }
            if target - before > grp.tolerance / 2.0 {
                let idx = Index::build(b, pb, rules, g);
                add_length(g, &idx, rules, &outline, n, target - before, 3.0, None, &|_| true);
            }
            let after = net_length(g, n);
            out.push(TuneResult { net: b.nets[n].name.clone(), before, after, target, ok: (after - target).abs() <= grp.tolerance || after >= target - grp.tolerance });
        }
    }
    // differential pair skew, measured end to end along the pair's signal path
    // (e.g. connector -> ESD part -> MCU); compensated on the shorter conductor only
    for (pi, dp) in rules.diff_pairs.iter().enumerate() {
        let conn_len = |g: &Geometry, c: usize| -> f64 { g.tracks.iter().filter(|t| t.conn as usize == c).map(|t| t.prims.iter().map(|q| q.len()).sum::<f64>()).sum() };
        let fps = |c: usize| -> Option<(usize, usize)> {
            match (pb.conns[c].a, pb.conns[c].b) {
                (crate::problem::Term::Pad(x), crate::problem::Term::Pad(y)) => {
                    let (fa, fb) = (b.pads[x].footprint, b.pads[y].footprint);
                    if fa == fb {
                        None // local stub inside one part (e.g. USB-C row bridge)
                    } else {
                        Some((fa.min(fb), fa.max(fb)))
                    }
                }
                _ => None,
            }
        };
        let pcs: Vec<usize> = (0..pb.conns.len()).filter(|&c| pb.conns[c].couple == Some((pi, true))).collect();
        let ncs: Vec<usize> = (0..pb.conns.len()).filter(|&c| pb.conns[c].couple == Some((pi, false))).collect();
        // matched segments: (parts, P connection, N connection)
        let mut segs: Vec<((usize, usize), usize, usize)> = Vec::new();
        for &pc in &pcs {
            if let Some(key) = fps(pc) {
                if let Some(nc) = ncs.iter().copied().find(|&c| fps(c) == Some(key)) {
                    if conn_len(g, pc) > 0.0 && conn_len(g, nc) > 0.0 {
                        segs.push((key, pc, nc));
                    }
                }
            }
        }
        if segs.is_empty() {
            continue;
        }
        // do the segments form one chain of parts? then the skew that matters is the sum
        let mut deg: HashMap<usize, usize> = HashMap::new();
        for (k, _, _) in &segs {
            *deg.entry(k.0).or_default() += 1;
            *deg.entry(k.1).or_default() += 1;
        }
        let is_path = deg.values().all(|&d| d <= 2) && segs.len() + 1 == deg.len();
        let groups: Vec<Vec<usize>> = if is_path { vec![(0..segs.len()).collect()] } else { (0..segs.len()).map(|i| vec![i]).collect() };
        for grp in groups {
            let skew = |g: &Geometry| -> f64 { grp.iter().map(|&i| conn_len(g, segs[i].1) - conn_len(g, segs[i].2)).sum() };
            let d0 = skew(g);
            if d0.abs() > dp.max_skew {
                // the shorter conductor gets all the extra length; start on the segment
                // where it lags most (closest to where the mismatch comes from)
                let p_longer = d0 > 0.0;
                let (short_net, other_net) = if p_longer { (dp.n, dp.p) } else { (dp.p, dp.n) };
                let mut order: Vec<usize> = grp.clone();
                let lag = |g: &Geometry, i: usize| {
                    let d = conn_len(g, segs[i].1) - conn_len(g, segs[i].2);
                    if p_longer {
                        d
                    } else {
                        -d
                    }
                };
                order.sort_by(|&x, &y| lag(g, y).partial_cmp(&lag(g, x)).unwrap());
                let mut need = d0.abs();
                for i in order {
                    if need < dp.max_skew / 4.0 {
                        break;
                    }
                    let sc = if p_longer { segs[i].2 } else { segs[i].1 };
                    let idx = Index::build(b, pb, rules, g);
                    need -= add_length(g, &idx, rules, &outline, short_net, need, (dp.gap * 2.0 + dp.width).max(0.4), Some(other_net), &|t| t.conn as usize == sc);
                }
            }
            let d1 = skew(g);
            // label: parts along the chain
            let label = {
                let mut names: Vec<String> = Vec::new();
                if grp.len() == 1 {
                    let k = segs[grp[0]].0;
                    names.push(b.footprints[k.0].reference.clone());
                    names.push(b.footprints[k.1].reference.clone());
                } else {
                    let mut cur = *deg.iter().find(|(_, &d)| d == 1).map(|(k, _)| k).unwrap_or(&segs[0].0 .0);
                    let mut used = vec![false; segs.len()];
                    names.push(b.footprints[cur].reference.clone());
                    while let Some(i) = (0..segs.len()).find(|&i| !used[i] && (segs[i].0 .0 == cur || segs[i].0 .1 == cur)) {
                        used[i] = true;
                        cur = if segs[i].0 .0 == cur { segs[i].0 .1 } else { segs[i].0 .0 };
                        names.push(b.footprints[cur].reference.clone());
                    }
                }
                names.join("-")
            };
            out.push(TuneResult {
                net: format!("{}/{} skew {}", b.nets[dp.p].name, b.nets[dp.n].name, label),
                before: d0.abs(),
                after: d1.abs(),
                target: dp.max_skew,
                ok: d1.abs() <= dp.max_skew,
            });
        }
    }
    let _ = p(0.0, 0.0);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn trombone_gain() {
        let a = p(0.0, 0.0);
        let b = p(10.0, 0.0);
        let prs = trombone(a, b, 2, 1.0, 0.25, 1.0);
        let len: f64 = prs.iter().map(|q| q.len()).sum();
        let expect = 10.0 + 2.0 * bump_gain(1.0, 0.25);
        assert!((len - expect).abs() < 1e-6, "{len} vs {expect}");
        // continuity
        for w in prs.windows(2) {
            assert!(w[0].end().dist(w[1].start()) < 1e-9, "{:?} {:?}", w[0].end(), w[1].start());
        }
    }
}
