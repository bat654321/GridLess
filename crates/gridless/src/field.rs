//! Distance field to the nearest copper, the "contour map" of the free space.
//!
//! Pads and tracks are hills, the track is a river: a point far from every piece of
//! copper has a high value, and the ridges of the field are the medial axes of the
//! free corridors. The router uses it as a mild cost so a path drifts to the middle
//! of a corridor instead of hugging one wall.
//!
//! The field is sampled from the mesh's constraint edges (the raw copper boundary and
//! the board outline), so it costs one rasterisation plus a chamfer pass per layer.

use crate::geom::{p, P};
use crate::mesh::Mesh;

pub struct Field {
    pub x0: f64,
    pub y0: f64,
    pub step: f64,
    pub nx: usize,
    pub ny: usize,
    d: Vec<f32>,
}

impl Field {
    pub fn from_mesh(m: &Mesh, step: f64) -> Field {
        let mut mn = p(f64::INFINITY, f64::INFINITY);
        let mut mx = p(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for v in &m.verts {
            mn = p(mn.x.min(v.pos.x), mn.y.min(v.pos.y));
            mx = p(mx.x.max(v.pos.x), mx.y.max(v.pos.y));
        }
        if !(mx.x > mn.x && mx.y > mn.y) {
            return Field { x0: 0.0, y0: 0.0, step, nx: 1, ny: 1, d: vec![0.0] };
        }
        let pad = step * 3.0;
        let x0 = mn.x - pad;
        let y0 = mn.y - pad;
        let nx = (((mx.x + pad) - x0) / step).ceil() as usize + 1;
        let ny = (((mx.y + pad) - y0) / step).ceil() as usize + 1;
        let mut d = vec![f32::INFINITY; nx * ny];
        let xmax = x0 + step * (nx - 1) as f64;
        let ymax = y0 + step * (ny - 1) as f64;
        let mark = |q: P, d: &mut [f32]| {
            let ix = ((q.x - x0) / step).round();
            let iy = ((q.y - y0) / step).round();
            if ix < 0.0 || iy < 0.0 || ix > (nx - 1) as f64 || iy > (ny - 1) as f64 {
                return;
            }
            d[iy as usize * nx + ix as usize] = 0.0;
        };
        // copper boundary: the constrained edges of the triangulation
        for e in &m.edges {
            if !e.constraint {
                continue;
            }
            let a = m.verts[e.v[0] as usize].pos;
            let b = m.verts[e.v[1] as usize].pos;
            let n = ((a.dist(b) / (step * 0.5)).ceil() as usize).max(1);
            for k in 0..=n {
                mark(a.lerp(b, k as f64 / n as f64), &mut d);
            }
        }
        // corners and isolated dots
        for v in &m.verts {
            mark(v.pos, &mut d);
        }
        let _ = (xmax, ymax);
        // chamfer distance transform (3x3, orthogonal 1, diagonal sqrt 2)
        let w2 = std::f32::consts::SQRT_2;
        let inf = f32::INFINITY;
        for y in 0..ny {
            for x in 0..nx {
                let i = y * nx + x;
                let mut v = d[i];
                let at = |xx: isize, yy: isize| -> f32 {
                    if xx < 0 || yy < 0 || xx >= nx as isize || yy >= ny as isize {
                        inf
                    } else {
                        d[yy as usize * nx + xx as usize]
                    }
                };
                v = v.min(at(x as isize - 1, y as isize) + 1.0);
                v = v.min(at(x as isize, y as isize - 1) + 1.0);
                v = v.min(at(x as isize - 1, y as isize - 1) + w2);
                v = v.min(at(x as isize + 1, y as isize - 1) + w2);
                d[i] = v;
            }
        }
        for y in (0..ny).rev() {
            for x in (0..nx).rev() {
                let i = y * nx + x;
                let mut v = d[i];
                let at = |xx: isize, yy: isize| -> f32 {
                    if xx < 0 || yy < 0 || xx >= nx as isize || yy >= ny as isize {
                        inf
                    } else {
                        d[yy as usize * nx + xx as usize]
                    }
                };
                v = v.min(at(x as isize + 1, y as isize) + 1.0);
                v = v.min(at(x as isize, y as isize + 1) + 1.0);
                v = v.min(at(x as isize + 1, y as isize + 1) + w2);
                v = v.min(at(x as isize - 1, y as isize + 1) + w2);
                d[i] = v;
            }
        }
        for v in d.iter_mut() {
            *v *= step as f32;
        }
        Field { x0, y0, step, nx, ny, d }
    }

    /// Distance to the nearest copper at `q`, in millimetres (bilinear).
    pub fn at(&self, q: P) -> f64 {
        if self.nx < 2 || self.ny < 2 {
            return 0.0;
        }
        let fx = ((q.x - self.x0) / self.step).clamp(0.0, (self.nx - 1) as f64);
        let fy = ((q.y - self.y0) / self.step).clamp(0.0, (self.ny - 1) as f64);
        let x = fx.floor() as usize;
        let y = fy.floor() as usize;
        let tx = fx - x as f64;
        let ty = fy - y as f64;
        let x1 = (x + 1).min(self.nx - 1);
        let y1 = (y + 1).min(self.ny - 1);
        let g = |xx: usize, yy: usize| self.d[yy * self.nx + xx] as f64;
        let a = g(x, y) * (1.0 - tx) + g(x1, y) * tx;
        let b = g(x, y1) * (1.0 - tx) + g(x1, y1) * tx;
        a * (1.0 - ty) + b * ty
    }
}
