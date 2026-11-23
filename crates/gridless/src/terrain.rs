//! Contour map ("terrain") of the routing space.
//!
//! Every piece of foreign copper is a hill: full height on the copper and its clearance
//! band, falling off smoothly (Gaussian) with the distance from it. Hills add up, so a
//! dense group of pads becomes a plateau and the space between groups a valley. The path
//! search pays for climbing (cost per millimetre grows with the height), so wires flow
//! like rivers: between groups of pads rather than between the pins of one part, out
//! of a pin row square to it, and on the layer where the ground is flat — the long run
//! goes on the empty layer and the via drops near the crowded end.
//!
//! A wire's own net is not a hill for it: its contribution is subtracted at lookup.

use crate::board::NetId;
use crate::geom::{p, BBox, RoundPoly, P};
use crate::problem::{ObsKind, Problem};
use crate::render::Canvas;
use std::collections::HashMap;

struct Hill {
    shape: RoundPoly,
    bb: BBox,
    clr: f64,
}

pub struct Terrain {
    x0: f64,
    y0: f64,
    step: f64,
    nx: usize,
    ny: usize,
    sigma: f64,
    /// height grid per routing layer (index = mesh index)
    h: Vec<Vec<f32>>,
    /// hills of each net per routing layer, for subtracting a net's own copper
    own: Vec<HashMap<NetId, Vec<Hill>>>,
}

impl Terrain {
    /// `layers`: board layer of each routing layer (mesh order). `clr`: default clearance.
    pub fn build(pb: &Problem, layers: &[usize], clr: f64, sigma: f64) -> Terrain {
        let step = 0.1;
        let bb = pb.bbox.grow(1.0);
        let nx = (bb.w() / step).ceil() as usize + 1;
        let ny = (bb.h() / step).ceil() as usize + 1;
        let mut t = Terrain { x0: bb.min.x, y0: bb.min.y, step, nx, ny, sigma, h: Vec::new(), own: Vec::new() };
        let reach = 3.0 * sigma;
        for &bl in layers {
            let mut g = vec![0.0f32; nx * ny];
            let mut own: HashMap<NetId, Vec<Hill>> = HashMap::new();
            for o in &pb.obstacles {
                if !o.on(bl) {
                    continue;
                }
                match o.kind {
                    ObsKind::Pad(_) | ObsKind::Copper | ObsKind::Track | ObsKind::Via => {}
                    // escape stubs belong to their pad; keepouts are walls, not hills
                    ObsKind::Escape(_) | ObsKind::Keepout => continue,
                }
                let hill = Hill { shape: o.shape.clone(), bb: o.shape.bbox().grow(clr + reach), clr };
                t.splat(&mut g, &hill, 1.0);
                if let Some(n) = o.net {
                    own.entry(n).or_default().push(hill);
                }
            }
            t.h.push(g);
            t.own.push(own);
        }
        t
    }

    fn bump(&self, hill: &Hill, q: P) -> f64 {
        let d = hill.shape.dist(q) - hill.clr;
        if d <= 0.0 {
            1.0
        } else {
            let x = d / self.sigma;
            if x > 3.0 {
                0.0
            } else {
                (-x * x).exp()
            }
        }
    }

    fn splat(&self, g: &mut [f32], hill: &Hill, sign: f32) {
        let ix0 = (((hill.bb.min.x - self.x0) / self.step).floor().max(0.0)) as usize;
        let iy0 = (((hill.bb.min.y - self.y0) / self.step).floor().max(0.0)) as usize;
        let ix1 = (((hill.bb.max.x - self.x0) / self.step).ceil() as usize).min(self.nx - 1);
        let iy1 = (((hill.bb.max.y - self.y0) / self.step).ceil() as usize).min(self.ny - 1);
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let q = p(self.x0 + ix as f64 * self.step, self.y0 + iy as f64 * self.step);
                let b = self.bump(hill, q) as f32;
                if b > 0.0 {
                    g[iy * self.nx + ix] += sign * b;
                }
            }
        }
    }

    fn grid(&self, li: usize, q: P) -> f64 {
        let fx = ((q.x - self.x0) / self.step).clamp(0.0, (self.nx - 1) as f64);
        let fy = ((q.y - self.y0) / self.step).clamp(0.0, (self.ny - 1) as f64);
        let x = (fx.floor() as usize).min(self.nx - 2);
        let y = (fy.floor() as usize).min(self.ny - 2);
        let tx = fx - x as f64;
        let ty = fy - y as f64;
        let g = |xx: usize, yy: usize| self.h[li][yy * self.nx + xx] as f64;
        let a = g(x, y) * (1.0 - tx) + g(x + 1, y) * tx;
        let b = g(x, y + 1) * (1.0 - tx) + g(x + 1, y + 1) * tx;
        a * (1.0 - ty) + b * ty
    }

    /// Height at q on routing layer li as seen by a wire of `net` (its own copper is flat).
    pub fn at(&self, li: usize, q: P, net: Option<NetId>) -> f64 {
        let mut h = self.grid(li, q);
        if let Some(n) = net {
            if let Some(hs) = self.own[li].get(&n) {
                for hl in hs {
                    if hl.bb.contains(q) {
                        h -= self.bump(hl, q);
                    }
                }
            }
        }
        h.max(0.0)
    }

    /// Mean height along the segment a-b (sampled every ~0.25 mm).
    pub fn mean(&self, li: usize, a: P, b: P, net: Option<NetId>) -> f64 {
        let l = a.dist(b);
        let n = ((l / 0.25).ceil() as usize).clamp(1, 24);
        let mut s = 0.0;
        for k in 0..n {
            let t = (k as f64 + 0.5) / n as f64;
            s += self.at(li, a.lerp(b, t), net);
        }
        s / n as f64
    }

    /// Debug picture of one layer: height as brightness with contour lines.
    pub fn save_png(&self, li: usize, path: &std::path::Path, scale: f64) -> anyhow::Result<()> {
        let min = p(self.x0, self.y0);
        let max = p(self.x0 + self.step * (self.nx - 1) as f64, self.y0 + self.step * (self.ny - 1) as f64);
        let mut cv = Canvas::new(min, max, scale);
        let w = cv.pm.width() as usize;
        let hgt = cv.pm.height() as usize;
        let data = cv.pm.data_mut();
        for py in 0..hgt {
            for px in 0..w {
                let q = p(min.x + (px as f64 + 0.5) / scale, min.y + (py as f64 + 0.5) / scale);
                let h = self.grid(li, q);
                let v = (h / 3.0).min(1.0);
                // contour every 0.25 height units
                let frac = (h / 0.25).fract();
                let line = h > 0.02 && (frac < 0.06 || frac > 0.94);
                let (r, g, b) = if line { (255u8, 255u8, 255u8) } else { ((40.0 + 200.0 * v) as u8, (60.0 + 120.0 * (1.0 - v)) as u8, (30.0 + 60.0 * (1.0 - v)) as u8) };
                let i = (py * w + px) * 4;
                data[i] = r;
                data[i + 1] = g;
                data[i + 2] = b;
                data[i + 3] = 255;
            }
        }
        cv.save(path)
    }
}
