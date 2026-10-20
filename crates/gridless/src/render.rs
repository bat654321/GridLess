//! Debug / preview rendering to PNG (tiny-skia).

use crate::geom::{RoundPoly, P};
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Stroke, Transform};

pub struct Canvas {
    pub pm: Pixmap,
    scale: f64,
    origin: P,
}

pub const LAYER_COLORS: [(u8, u8, u8); 6] = [(200, 52, 52), (60, 160, 60), (200, 160, 30), (60, 90, 220), (180, 60, 180), (40, 170, 170)];

impl Canvas {
    pub fn new(min: P, max: P, px_per_mm: f64) -> Canvas {
        let w = ((max.x - min.x) * px_per_mm).ceil().max(16.0) as u32;
        let h = ((max.y - min.y) * px_per_mm).ceil().max(16.0) as u32;
        let mut pm = Pixmap::new(w, h).expect("pixmap");
        pm.fill(Color::from_rgba8(16, 16, 20, 255));
        Canvas { pm, scale: px_per_mm, origin: min }
    }
    fn tx(&self, q: P) -> (f32, f32) {
        (((q.x - self.origin.x) * self.scale) as f32, ((q.y - self.origin.y) * self.scale) as f32)
    }
    fn paint(c: (u8, u8, u8), a: u8) -> Paint<'static> {
        let mut p = Paint::default();
        p.set_color_rgba8(c.0, c.1, c.2, a);
        p.anti_alias = true;
        p
    }
    pub fn polyline(&mut self, pts: &[P], width_mm: f64, c: (u8, u8, u8), a: u8) {
        if pts.len() < 2 {
            return;
        }
        let mut pb = PathBuilder::new();
        let (x, y) = self.tx(pts[0]);
        pb.move_to(x, y);
        for q in &pts[1..] {
            let (x, y) = self.tx(*q);
            pb.line_to(x, y);
        }
        if let Some(path) = pb.finish() {
            let mut s = Stroke::default();
            s.width = ((width_mm * self.scale) as f32).max(1.0);
            s.line_cap = tiny_skia::LineCap::Round;
            s.line_join = tiny_skia::LineJoin::Round;
            self.pm.stroke_path(&path, &Self::paint(c, a), &s, Transform::identity(), None);
        }
    }
    pub fn polygon(&mut self, pts: &[P], c: (u8, u8, u8), a: u8) {
        if pts.len() < 3 {
            return;
        }
        let mut pb = PathBuilder::new();
        let (x, y) = self.tx(pts[0]);
        pb.move_to(x, y);
        for q in &pts[1..] {
            let (x, y) = self.tx(*q);
            pb.line_to(x, y);
        }
        pb.close();
        if let Some(path) = pb.finish() {
            self.pm.fill_path(&path, &Self::paint(c, a), FillRule::Winding, Transform::identity(), None);
        }
    }
    pub fn shape(&mut self, s: &RoundPoly, c: (u8, u8, u8), a: u8) {
        let o = s.outline(0.01);
        if o.len() >= 3 {
            self.polygon(&o, c, a);
        } else if o.len() == 2 {
            self.polyline(&o, 0.02, c, a);
        }
    }
    pub fn circle(&mut self, q: P, r: f64, c: (u8, u8, u8), a: u8) {
        self.shape(&RoundPoly::circle(q, r), c, a);
    }
    /// Polygon fill using the even-odd rule (zone fills with holes).
    pub fn polygon_eo(&mut self, pts: &[P], c: (u8, u8, u8), a: u8) {
        if pts.len() < 3 {
            return;
        }
        let mut pb = PathBuilder::new();
        let (x, y) = self.tx(pts[0]);
        pb.move_to(x, y);
        for q in &pts[1..] {
            let (x, y) = self.tx(*q);
            pb.line_to(x, y);
        }
        pb.close();
        if let Some(path) = pb.finish() {
            self.pm.fill_path(&path, &Self::paint(c, a), FillRule::EvenOdd, Transform::identity(), None);
        }
    }

    /// Ring marker of radius r (mm) with line width lw (mm).
    pub fn ring(&mut self, q: P, r: f64, lw: f64, c: (u8, u8, u8)) {
        let pl = crate::geom::Arc { c: q, r, a0: 0.0, sweep: 2.0 * std::f64::consts::PI }.polyline(r * 0.02);
        self.polyline(&pl, lw, c, 255);
    }

    /// Draw a non-negative integer with 7-segment glyphs; `h` = glyph height in mm.
    pub fn number(&mut self, n: u32, at: P, h: f64, c: (u8, u8, u8)) {
        // segments a..g as (x0, y0, x1, y1) in a 1 x 2 cell
        const SEG: [(f64, f64, f64, f64); 7] = [(0.0, 0.0, 1.0, 0.0), (1.0, 0.0, 1.0, 1.0), (1.0, 1.0, 1.0, 2.0), (0.0, 2.0, 1.0, 2.0), (0.0, 1.0, 0.0, 2.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 1.0, 1.0)];
        const DIG: [u8; 10] = [0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110, 0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111];
        let w = h / 2.0;
        for (i, ch) in n.to_string().chars().enumerate() {
            let d = ch.to_digit(10).unwrap_or(0) as usize;
            let ox = at.x + i as f64 * w * 1.6;
            for (k, sg) in SEG.iter().enumerate() {
                if DIG[d] & (1 << k) != 0 {
                    let a = crate::geom::p(ox + sg.0 * w, at.y + sg.1 * w);
                    let b = crate::geom::p(ox + sg.2 * w, at.y + sg.3 * w);
                    self.polyline(&[a, b], h * 0.14, c, 255);
                }
            }
        }
    }

    pub fn save(&self, path: &std::path::Path) -> anyhow::Result<()> {
        self.pm.save_png(path).map_err(|e| anyhow::anyhow!("png: {e}"))
    }
}
