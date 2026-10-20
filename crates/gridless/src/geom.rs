//! Basic 2D geometry. All coordinates are in millimetres (f64), Y axis points down
//! (KiCad convention), angles in radians unless stated otherwise.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Div, Mul, Neg, Sub};

pub const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct P {
    pub x: f64,
    pub y: f64,
}

#[inline]
pub const fn p(x: f64, y: f64) -> P {
    P { x, y }
}

impl P {
    #[inline]
    pub fn dot(self, o: P) -> f64 {
        self.x * o.x + self.y * o.y
    }
    #[inline]
    pub fn cross(self, o: P) -> f64 {
        self.x * o.y - self.y * o.x
    }
    #[inline]
    pub fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
    #[inline]
    pub fn len2(self) -> f64 {
        self.x * self.x + self.y * self.y
    }
    #[inline]
    pub fn dist(self, o: P) -> f64 {
        (self - o).len()
    }
    #[inline]
    pub fn norm(self) -> P {
        let l = self.len();
        if l < EPS {
            P::default()
        } else {
            self / l
        }
    }
    /// Left-hand perpendicular in a Y-up frame (rotates +90 deg mathematically).
    #[inline]
    pub fn perp(self) -> P {
        p(-self.y, self.x)
    }
    #[inline]
    pub fn rot(self, a: f64) -> P {
        let (s, c) = a.sin_cos();
        p(self.x * c - self.y * s, self.x * s + self.y * c)
    }
    #[inline]
    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
    #[inline]
    pub fn lerp(self, o: P, t: f64) -> P {
        self + (o - self) * t
    }
    #[inline]
    pub fn mid(self, o: P) -> P {
        (self + o) * 0.5
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Add for P {
    type Output = P;
    #[inline]
    fn add(self, o: P) -> P {
        p(self.x + o.x, self.y + o.y)
    }
}
impl Sub for P {
    type Output = P;
    #[inline]
    fn sub(self, o: P) -> P {
        p(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for P {
    type Output = P;
    #[inline]
    fn mul(self, k: f64) -> P {
        p(self.x * k, self.y * k)
    }
}
impl Div<f64> for P {
    type Output = P;
    #[inline]
    fn div(self, k: f64) -> P {
        p(self.x / k, self.y / k)
    }
}
impl Neg for P {
    type Output = P;
    #[inline]
    fn neg(self) -> P {
        p(-self.x, -self.y)
    }
}

/// Orientation of triangle (a,b,c): >0 counter-clockwise (in a Y-up frame).
#[inline]
pub fn orient(a: P, b: P, c: P) -> f64 {
    (b - a).cross(c - a)
}

/// Parameter t in [0,1] of the point on segment ab closest to q.
#[inline]
pub fn seg_param(a: P, b: P, q: P) -> f64 {
    let d = b - a;
    let l2 = d.len2();
    if l2 < EPS * EPS {
        return 0.0;
    }
    ((q - a).dot(d) / l2).clamp(0.0, 1.0)
}

#[inline]
pub fn closest_on_seg(a: P, b: P, q: P) -> P {
    a.lerp(b, seg_param(a, b, q))
}

#[inline]
pub fn point_seg_dist(q: P, a: P, b: P) -> f64 {
    q.dist(closest_on_seg(a, b, q))
}

/// Proper or touching intersection test of segments ab and cd.
pub fn segs_intersect(a: P, b: P, c: P, d: P) -> bool {
    let d1 = orient(c, d, a);
    let d2 = orient(c, d, b);
    let d3 = orient(a, b, c);
    let d4 = orient(a, b, d);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return true;
    }
    let on = |p0: P, p1: P, q: P, o: f64| o.abs() < 1e-12 && q.x >= p0.x.min(p1.x) - 1e-12 && q.x <= p0.x.max(p1.x) + 1e-12 && q.y >= p0.y.min(p1.y) - 1e-12 && q.y <= p0.y.max(p1.y) + 1e-12;
    on(c, d, a, d1) || on(c, d, b, d2) || on(a, b, c, d3) || on(a, b, d, d4)
}

/// Intersection parameters (t on ab, u on cd) of two lines, if not parallel.
pub fn line_intersect(a: P, b: P, c: P, d: P) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-15 {
        return None;
    }
    let t = (c - a).cross(s) / den;
    let u = (c - a).cross(r) / den;
    Some((t, u))
}

pub fn seg_seg_dist(a: P, b: P, c: P, d: P) -> f64 {
    if segs_intersect(a, b, c, d) {
        return 0.0;
    }
    point_seg_dist(a, c, d)
        .min(point_seg_dist(b, c, d))
        .min(point_seg_dist(c, a, b))
        .min(point_seg_dist(d, a, b))
}

/// Closest points between two segments (pa on ab, pc on cd).
pub fn seg_seg_closest(a: P, b: P, c: P, d: P) -> (P, P) {
    if let Some((t, u)) = line_intersect(a, b, c, d) {
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            let q = a.lerp(b, t);
            return (q, q);
        }
    }
    let cands = [
        (a, closest_on_seg(c, d, a)),
        (b, closest_on_seg(c, d, b)),
        (closest_on_seg(a, b, c), c),
        (closest_on_seg(a, b, d), d),
    ];
    let mut best = cands[0];
    for c in &cands[1..] {
        if c.0.dist(c.1) < best.0.dist(best.1) {
            best = *c;
        }
    }
    best
}

/// Normalise angle into (-pi, pi].
pub fn norm_angle(mut a: f64) -> f64 {
    use std::f64::consts::PI;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a <= -PI {
        a += 2.0 * PI;
    }
    a
}

/// Circle through three points.
pub fn circle_3p(a: P, b: P, c: P) -> Option<(P, f64)> {
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() < 1e-14 {
        return None;
    }
    let a2 = a.len2();
    let b2 = b.len2();
    let c2 = c.len2();
    let ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    let uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    let cen = p(ux, uy);
    Some((cen, cen.dist(a)))
}

/// Circular arc given by centre, radius, start angle and signed sweep.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Arc {
    pub c: P,
    pub r: f64,
    pub a0: f64,
    pub sweep: f64,
}

impl Arc {
    pub fn from_3p(s: P, m: P, e: P) -> Option<Arc> {
        let (c, r) = circle_3p(s, m, e)?;
        let a0 = (s - c).angle();
        let am = (m - c).angle();
        let ae = (e - c).angle();
        let ccw = orient(s, m, e) > 0.0;
        let mut sw = norm_angle(ae - a0);
        if ccw && sw < 0.0 {
            sw += 2.0 * std::f64::consts::PI;
        } else if !ccw && sw > 0.0 {
            sw -= 2.0 * std::f64::consts::PI;
        }
        let _ = am;
        Some(Arc { c, r, a0, sweep: sw })
    }
    pub fn at(&self, t: f64) -> P {
        let a = self.a0 + self.sweep * t;
        self.c + p(a.cos(), a.sin()) * self.r
    }
    pub fn start(&self) -> P {
        self.at(0.0)
    }
    pub fn end(&self) -> P {
        self.at(1.0)
    }
    pub fn mid(&self) -> P {
        self.at(0.5)
    }
    pub fn len(&self) -> f64 {
        self.r * self.sweep.abs()
    }
    /// Is direction angle `a` (from the centre) inside the swept range?
    pub fn contains_angle(&self, a: f64) -> bool {
        let d = norm_angle(a - self.a0);
        if self.sweep >= 0.0 {
            let d = if d < 0.0 { d + 2.0 * std::f64::consts::PI } else { d };
            d <= self.sweep + 1e-12
        } else {
            let d = if d > 0.0 { d - 2.0 * std::f64::consts::PI } else { d };
            d >= self.sweep - 1e-12
        }
    }
    pub fn closest(&self, q: P) -> P {
        let a = (q - self.c).angle();
        if self.contains_angle(a) {
            self.c + p(a.cos(), a.sin()) * self.r
        } else {
            let s = self.start();
            let e = self.end();
            if s.dist(q) < e.dist(q) {
                s
            } else {
                e
            }
        }
    }
    /// Polyline approximation with chord error <= tol.
    pub fn polyline(&self, tol: f64) -> Vec<P> {
        let n = if self.r <= tol {
            2
        } else {
            let step = 2.0 * (1.0 - tol / self.r).clamp(-1.0, 1.0).acos();
            ((self.sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(2, 512)
        };
        (0..=n).map(|i| self.at(i as f64 / n as f64)).collect()
    }
}

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct BBox {
    pub min: P,
    pub max: P,
}

impl BBox {
    pub fn empty() -> BBox {
        BBox { min: p(f64::INFINITY, f64::INFINITY), max: p(f64::NEG_INFINITY, f64::NEG_INFINITY) }
    }
    pub fn add(&mut self, q: P) {
        self.min.x = self.min.x.min(q.x);
        self.min.y = self.min.y.min(q.y);
        self.max.x = self.max.x.max(q.x);
        self.max.y = self.max.y.max(q.y);
    }
    pub fn grow(&self, d: f64) -> BBox {
        BBox { min: self.min - p(d, d), max: self.max + p(d, d) }
    }
    pub fn union(&self, o: &BBox) -> BBox {
        let mut b = *self;
        b.add(o.min);
        b.add(o.max);
        b
    }
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }
    pub fn w(&self) -> f64 {
        self.max.x - self.min.x
    }
    pub fn h(&self) -> f64 {
        self.max.y - self.min.y
    }
    pub fn overlaps(&self, o: &BBox) -> bool {
        self.min.x <= o.max.x && o.min.x <= self.max.x && self.min.y <= o.max.y && o.min.y <= self.max.y
    }
    pub fn contains(&self, q: P) -> bool {
        q.x >= self.min.x && q.x <= self.max.x && q.y >= self.min.y && q.y <= self.max.y
    }
}

/// A convex "rounded polygon": the Minkowski sum of a convex point set (1 point =
/// circle, 2 points = stadium/track segment, >=3 = polygon) with a disk of radius `r`.
/// This single primitive models pads (circle, oval, rect, roundrect, trapezoid),
/// vias and straight track segments exactly.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoundPoly {
    pub pts: Vec<P>,
    pub r: f64,
}

impl RoundPoly {
    pub fn circle(c: P, r: f64) -> Self {
        RoundPoly { pts: vec![c], r }
    }
    pub fn capsule(a: P, b: P, r: f64) -> Self {
        RoundPoly { pts: vec![a, b], r }
    }
    pub fn bbox(&self) -> BBox {
        let mut b = BBox::empty();
        for q in &self.pts {
            b.add(*q);
        }
        b.grow(self.r)
    }
    pub fn centroid(&self) -> P {
        let mut s = P::default();
        for q in &self.pts {
            s = s + *q;
        }
        s / self.pts.len() as f64
    }
    fn inside_core(&self, q: P) -> bool {
        if self.pts.len() < 3 {
            return false;
        }
        // convex polygon, either orientation
        let n = self.pts.len();
        let mut sign = 0.0;
        for i in 0..n {
            let o = orient(self.pts[i], self.pts[(i + 1) % n], q);
            if o.abs() < 1e-15 {
                continue;
            }
            if sign == 0.0 {
                sign = o.signum();
            } else if o.signum() != sign {
                return false;
            }
        }
        true
    }
    /// Closest point on the core (point set hull) to q, and whether q is inside the core.
    pub fn core_closest(&self, q: P) -> (P, bool) {
        match self.pts.len() {
            0 => (q, false),
            1 => (self.pts[0], false),
            2 => (closest_on_seg(self.pts[0], self.pts[1], q), false),
            n => {
                if self.inside_core(q) {
                    return (q, true);
                }
                let mut best = self.pts[0];
                let mut bd = f64::INFINITY;
                for i in 0..n {
                    let c = closest_on_seg(self.pts[i], self.pts[(i + 1) % n], q);
                    let d = c.dist(q);
                    if d < bd {
                        bd = d;
                        best = c;
                    }
                }
                (best, false)
            }
        }
    }
    /// Signed distance from q to the shape boundary (negative inside).
    pub fn dist(&self, q: P) -> f64 {
        let (c, inside) = self.core_closest(q);
        if inside {
            // distance to boundary of core, negative
            let n = self.pts.len();
            let mut bd = f64::INFINITY;
            for i in 0..n {
                bd = bd.min(point_seg_dist(q, self.pts[i], self.pts[(i + 1) % n]));
            }
            return -bd - self.r;
        }
        c.dist(q) - self.r
    }
    /// Edges of the core as segments (for n==1 a degenerate segment).
    pub fn core_segs(&self) -> Vec<(P, P)> {
        match self.pts.len() {
            0 => vec![],
            1 => vec![(self.pts[0], self.pts[0])],
            2 => vec![(self.pts[0], self.pts[1])],
            n => (0..n).map(|i| (self.pts[i], self.pts[(i + 1) % n])).collect(),
        }
    }
    /// Distance between the shape and a segment ab (of zero width). Negative = overlap depth (approx).
    pub fn seg_dist(&self, a: P, b: P) -> f64 {
        if self.pts.len() >= 3 && (self.inside_core(a) || self.inside_core(b)) {
            return -self.r;
        }
        let mut d = f64::INFINITY;
        for (c, e) in self.core_segs() {
            d = d.min(seg_seg_dist(a, b, c, e));
        }
        d - self.r
    }
    /// Distance between two rounded polygons.
    pub fn shape_dist(&self, o: &RoundPoly) -> f64 {
        if self.pts.len() >= 3 {
            for q in &o.pts {
                if self.inside_core(*q) {
                    return -(self.r + o.r);
                }
            }
        }
        if o.pts.len() >= 3 {
            for q in &self.pts {
                if o.inside_core(*q) {
                    return -(self.r + o.r);
                }
            }
        }
        let mut d = f64::INFINITY;
        for (a, b) in self.core_segs() {
            for (c, e) in o.core_segs() {
                d = d.min(seg_seg_dist(a, b, c, e));
            }
        }
        d - self.r - o.r
    }
    /// Smallest caliper width (thickness) of the shape.
    pub fn min_width(&self) -> f64 {
        let n = self.pts.len();
        if n < 3 {
            return 2.0 * self.r;
        }
        let mut best = f64::INFINITY;
        for i in 0..n {
            let a = self.pts[i];
            let b = self.pts[(i + 1) % n];
            let d = (b - a).norm();
            if d.len() < 1e-12 {
                continue;
            }
            let mut far: f64 = 0.0;
            for q in &self.pts {
                far = far.max((*q - a).cross(d).abs());
            }
            best = best.min(far);
        }
        best + 2.0 * self.r
    }

    /// Outline polygon approximation (for rendering / export).
    pub fn outline(&self, tol: f64) -> Vec<P> {
        use std::f64::consts::PI;
        let n = self.pts.len();
        if n == 0 {
            return vec![];
        }
        if self.r <= 1e-12 {
            return self.pts.clone();
        }
        if n == 1 {
            return Arc { c: self.pts[0], r: self.r, a0: 0.0, sweep: 2.0 * PI }.polyline(tol);
        }
        // ensure CCW order of core
        let mut pts = self.pts.clone();
        if n >= 3 {
            let mut area = 0.0;
            for i in 0..n {
                area += pts[i].cross(pts[(i + 1) % n]);
            }
            if area < 0.0 {
                pts.reverse();
            }
        }
        let mut out = Vec::new();
        let m = pts.len();
        for i in 0..m {
            let prev = pts[(i + m - 1) % m];
            let cur = pts[i];
            let next = pts[(i + 1) % m];
            let n_in = if m == 2 { (cur - prev).perp().norm() * -1.0 } else { (cur - prev).norm().perp() * -1.0 };
            let n_out = if m == 2 { (next - cur).perp().norm() * -1.0 } else { (next - cur).norm().perp() * -1.0 };
            let a0 = n_in.angle();
            let mut sw = norm_angle(n_out.angle() - a0);
            if m == 2 {
                sw = PI;
            } else if sw < 0.0 {
                sw += 2.0 * PI;
            }
            let arc = Arc { c: cur, r: self.r, a0, sweep: sw };
            out.extend(arc.polyline(tol));
        }
        out
    }
}

/// Signed area of a simple polygon (positive = CCW in a Y-up frame).
pub fn poly_area(pts: &[P]) -> f64 {
    let n = pts.len();
    let mut a = 0.0;
    for i in 0..n {
        a += pts[i].cross(pts[(i + 1) % n]);
    }
    a * 0.5
}

pub fn point_in_poly(q: P, pts: &[P]) -> bool {
    let n = pts.len();
    let mut inside = false;
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let a = pts[i];
        let b = pts[j];
        if (a.y > q.y) != (b.y > q.y) {
            let x = a.x + (q.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if q.x < x {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn is_convex(pts: &[P]) -> bool {
    let n = pts.len();
    if n < 4 {
        return true;
    }
    let mut sign = 0.0;
    for i in 0..n {
        let o = orient(pts[i], pts[(i + 1) % n], pts[(i + 2) % n]);
        if o.abs() < 1e-12 {
            continue;
        }
        if sign == 0.0 {
            sign = o.signum();
        } else if o.signum() != sign {
            return false;
        }
    }
    true
}

/// Split a simple polygon into convex pieces (the polygon itself if convex, otherwise
/// triangles from ear clipping).
pub fn convex_pieces(poly: &[P]) -> Vec<Vec<P>> {
    let mut pts: Vec<P> = poly.to_vec();
    pts.dedup_by(|a, b| a.dist(*b) < 1e-9);
    if pts.len() > 1 && pts[0].dist(*pts.last().unwrap()) < 1e-9 {
        pts.pop();
    }
    if pts.len() < 3 {
        return vec![pts];
    }
    if is_convex(&pts) {
        return vec![pts];
    }
    if poly_area(&pts) < 0.0 {
        pts.reverse();
    }
    let mut out = Vec::new();
    let mut guard = 0;
    while pts.len() > 3 && guard < 10_000 {
        guard += 1;
        let n = pts.len();
        let mut clipped = false;
        for i in 0..n {
            let (a, b, c) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
            if orient(a, b, c) <= 1e-12 {
                continue;
            }
            let ear = (0..n).all(|j| {
                let q = pts[j];
                if q == a || q == b || q == c {
                    return true;
                }
                !(orient(a, b, q) > 0.0 && orient(b, c, q) > 0.0 && orient(c, a, q) > 0.0)
            });
            if ear {
                out.push(vec![a, b, c]);
                pts.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    out.push(pts);
    out
}

/// Convex hull (Andrew monotone chain), CCW, no collinear points.
pub fn convex_hull(mut pts: Vec<P>) -> Vec<P> {
    pts.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap().then(a.y.partial_cmp(&b.y).unwrap()));
    pts.dedup_by(|a, b| a.dist(*b) < 1e-9);
    if pts.len() < 3 {
        return pts;
    }
    let mut lower: Vec<P> = Vec::new();
    for &q in &pts {
        while lower.len() >= 2 && orient(lower[lower.len() - 2], lower[lower.len() - 1], q) <= 1e-15 {
            lower.pop();
        }
        lower.push(q);
    }
    let mut upper: Vec<P> = Vec::new();
    for &q in pts.iter().rev() {
        while upper.len() >= 2 && orient(upper[upper.len() - 2], upper[upper.len() - 1], q) <= 1e-15 {
            upper.pop();
        }
        upper.push(q);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arc_3p() {
        let a = Arc::from_3p(p(1.0, 0.0), p(0.0, 1.0), p(-1.0, 0.0)).unwrap();
        assert!((a.r - 1.0).abs() < 1e-9);
        assert!((a.len() - std::f64::consts::PI).abs() < 1e-9);
        assert!(a.mid().dist(p(0.0, 1.0)) < 1e-9);
    }

    #[test]
    fn roundpoly_dist() {
        let r = RoundPoly { pts: vec![p(0.0, 0.0), p(2.0, 0.0), p(2.0, 1.0), p(0.0, 1.0)], r: 0.0 };
        assert!((r.dist(p(3.0, 0.5)) - 1.0).abs() < 1e-9);
        assert!(r.dist(p(1.0, 0.5)) < 0.0);
        let c = RoundPoly::circle(p(5.0, 0.5), 0.5);
        assert!((r.shape_dist(&c) - 2.5).abs() < 1e-9);
    }
}
