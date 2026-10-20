//! Per-layer constrained Delaunay triangulation of the routing space.
//!
//! Vertices are the "dots" of obstacles (pad corners / disk centres, outline corners,
//! existing tracks), pad terminals and candidate via sites. Wires are represented
//! topologically by the sequence of mesh edges they cross.

use crate::board::LayerId;
use crate::geom::{orient, point_in_poly, P};
use crate::problem::{inside_board, ObsKind, Problem};
pub use crate::problem::ObsKind as MObsKind;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use std::collections::HashMap;

pub const NONE: u32 = u32::MAX;
/// triangle owner marker: copper of several nets overlaps here, nobody may enter
pub const BLOCKED: u32 = u32::MAX - 1;

#[derive(Clone, Debug, Default)]
pub struct MVert {
    pub pos: P,
    /// obstacle radius at this dot (0 for polygon corners and free points)
    pub r: f64,
    /// obstacle the dot belongs to
    pub obs: Option<u32>,
    pub site: Option<u32>,
    pub pad: Option<u32>,
    pub outline: bool,
}

#[derive(Clone, Debug)]
pub struct MEdge {
    pub v: [u32; 2],
    pub t: [u32; 2],
    pub len: f64,
    pub constraint: bool,
    /// distance from each endpoint to where the edge leaves the endpoint obstacle body
    /// inflated by the default clearance (0 for free points)
    pub exit: [f64; 2],
    /// obstacle whose copper contains the middle of this edge (crossing = entering it)
    pub owner: Option<u32>,
}

#[derive(Clone, Debug)]
pub struct MTri {
    /// counter-clockwise (math orientation)
    pub v: [u32; 3],
    /// e[i] joins v[i] and v[(i+1)%3]
    pub e: [u32; 3],
    pub alive: bool,
    /// obstacle whose core contains this triangle (only its net may enter)
    pub owner: Option<u32>,
}

/// Obstacle as seen by one layer's mesh.
#[derive(Clone, Debug)]
pub struct MObs {
    pub shape: crate::geom::RoundPoly,
    /// pad anchor (connection point) when it differs from the shape centre
    pub anchor: Option<P>,
    pub net: Option<crate::board::NetId>,
    pub pad: Option<u32>,
    pub kind: ObsKind,
}

pub struct Mesh {
    pub layer: LayerId,
    pub obs: Vec<MObs>,
    pub verts: Vec<MVert>,
    pub edges: Vec<MEdge>,
    pub tris: Vec<MTri>,
    /// triangles around each vertex
    pub vtris: Vec<Vec<u32>>,
    pub site_vert: HashMap<u32, u32>,
    pub pad_vert: HashMap<u32, u32>,
}

/// Maximum distance between consecutive dots on outline / obstacle sides.
const SUBDIV: f64 = 1.0;

/// Points of a polyline (closed if `closed`) with extra points so that no gap exceeds `max`.
fn subdivide(pts: &[P], closed: bool, max: f64) -> Vec<P> {
    let n = pts.len();
    let mut out = Vec::with_capacity(n * 2);
    let segs = if closed { n } else { n.saturating_sub(1) };
    for k in 0..segs {
        let a = pts[k];
        let b = pts[(k + 1) % n];
        out.push(a);
        let m = (a.dist(b) / max).floor() as usize;
        for j in 1..=m {
            let t = j as f64 / (m + 1) as f64;
            out.push(a.lerp(b, t));
        }
    }
    if !closed {
        if let Some(l) = pts.last() {
            out.push(*l);
        }
    }
    out
}

/// Coarse uniform grid over obstacle bounding boxes (1 mm cells).
pub struct ObsGrid {
    cells: HashMap<(i32, i32), Vec<u32>>,
}

pub fn obs_grid(obs: &[MObs]) -> ObsGrid {
    let mut cells: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
    for (i, o) in obs.iter().enumerate() {
        let bb = o.shape.bbox();
        for x in bb.min.x.floor() as i32..=bb.max.x.floor() as i32 {
            for y in bb.min.y.floor() as i32..=bb.max.y.floor() as i32 {
                cells.entry((x, y)).or_default().push(i as u32);
            }
        }
    }
    ObsGrid { cells }
}

pub fn grid_near(g: &ObsGrid, q: P) -> &[u32] {
    g.cells.get(&(q.x.floor() as i32, q.y.floor() as i32)).map(|v| v.as_slice()).unwrap_or(&[])
}

impl MEdge {
    pub fn other_tri(&self, t: u32) -> u32 {
        if self.t[0] == t {
            self.t[1]
        } else {
            self.t[0]
        }
    }
}

impl MTri {
    /// local index (0..3) of edge `e` in this triangle
    pub fn edge_idx(&self, e: u32) -> usize {
        self.e.iter().position(|&x| x == e).expect("edge not in triangle")
    }
    pub fn vert_idx(&self, v: u32) -> Option<usize> {
        self.v.iter().position(|&x| x == v)
    }
}

/// Distance along the ray a->b (|ab| = len) at which the point first gets `d` away from
/// obstacle `shape` (assumes the ray starts inside or on the obstacle).
pub fn exit_dist(shape: &crate::geom::RoundPoly, a: P, b: P, d: f64) -> f64 {
    let len = a.dist(b);
    if len < 1e-12 {
        return 0.0;
    }
    let dir = (b - a) / len;
    if shape.dist(a + dir * len) < d {
        return len;
    }
    let (mut lo, mut hi) = (0.0, len);
    for _ in 0..40 {
        let m = 0.5 * (lo + hi);
        if shape.dist(a + dir * m) < d {
            lo = m;
        } else {
            hi = m;
        }
    }
    hi
}

impl Mesh {
    /// Routing mesh: obstacles of the problem on `layer` plus all via sites.
    pub fn build(pb: &Problem, layer: LayerId, clearance: f64, edge_clearance: f64) -> Mesh {
        let obs: Vec<MObs> = pb
            .obstacles
            .iter()
            .filter(|o| o.on(layer))
            .map(|o| MObs {
                anchor: o.anchor,
                shape: o.shape.clone(),
                net: o.net,
                pad: match o.kind {
                    ObsKind::Pad(p) | ObsKind::Escape(p) => Some(p as u32),
                    _ => None,
                },
                kind: o.kind,
            })
            .collect();
        let sites: Vec<(u32, P)> = pb.sites.iter().enumerate().map(|(i, s)| (i as u32, s.pos)).collect();
        Mesh::build_from(obs, &pb.outline, &sites, layer, clearance, edge_clearance)
    }

    pub fn build_from(obs: Vec<MObs>, outline: &[Vec<P>], sites: &[(u32, P)], layer: LayerId, clearance: f64, edge_clearance: f64) -> Mesh {
        let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
        let mut meta: HashMap<usize, MVert> = HashMap::new();
        let ins = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>, meta: &mut HashMap<usize, MVert>, m: MVert| -> Option<usize> {
            if !m.pos.is_finite() {
                return None;
            }
            let h = cdt.insert(Point2::new(m.pos.x, m.pos.y)).ok()?;
            let i = h.index();
            match meta.get_mut(&i) {
                Some(old) => {
                    // merge roles of coincident dots
                    if m.obs.is_some() && (old.obs.is_none() || m.r > old.r) {
                        old.obs = m.obs;
                        old.r = m.r;
                    }
                    old.site = old.site.or(m.site);
                    old.pad = old.pad.or(m.pad);
                    old.outline |= m.outline;
                }
                None => {
                    meta.insert(i, m);
                }
            }
            Some(i)
        };
        let mut constraints: Vec<(usize, usize)> = Vec::new();

        // board outline
        for poly in outline {
            let mut ids = Vec::new();
            for q in subdivide(poly, true, SUBDIV) {
                if let Some(i) = ins(&mut cdt, &mut meta, MVert { pos: q, outline: true, ..Default::default() }) {
                    if ids.last() != Some(&i) {
                        ids.push(i);
                    }
                }
            }
            for k in 0..ids.len() {
                constraints.push((ids[k], ids[(k + 1) % ids.len()]));
            }
        }
        // obstacles on this layer
        // pads whose terminal is moved to the end of an escape stub
        let escaped: Vec<u32> = obs.iter().filter_map(|o| if let ObsKind::Escape(p) = o.kind { Some(p as u32) } else { None }).collect();
        for (oi, o) in obs.iter().enumerate() {
            if let ObsKind::Escape(p) = o.kind {
                // stub: pad centre -> exit point; the exit point is the routing terminal
                let a = ins(&mut cdt, &mut meta, MVert { pos: o.shape.pts[0], r: o.shape.r, obs: Some(oi as u32), ..Default::default() });
                let b = ins(&mut cdt, &mut meta, MVert { pos: o.shape.pts[1], r: o.shape.r, obs: Some(oi as u32), pad: Some(p as u32), ..Default::default() });
                if let (Some(a), Some(b)) = (a, b) {
                    constraints.push((a, b));
                }
                continue;
            }
            let pad = o.pad.filter(|p| !escaped.contains(p));
            let mut ids = Vec::new();
            let n = o.shape.pts.len();
            // long sides get intermediate dots so that narrow gaps along them show up as
            // short mesh edges (capacity is measured on edges)
            let dots = if n >= 3 { subdivide(&o.shape.pts, true, SUBDIV) } else { o.shape.pts.clone() };
            for q in dots {
                if let Some(i) = ins(&mut cdt, &mut meta, MVert { pos: q, r: o.shape.r, obs: Some(oi as u32), pad: if n == 1 { pad } else { None }, ..Default::default() }) {
                    if ids.last() != Some(&i) {
                        ids.push(i);
                    }
                }
            }
            // pad terminal (anchor) inside the shape
            if let Some(p) = pad {
                if n > 1 {
                    let c = o.anchor.filter(|a| o.shape.dist(*a) < 0.0).unwrap_or_else(|| o.shape.centroid());
                    if let Some(i) = ins(&mut cdt, &mut meta, MVert { pos: c, r: 0.0, obs: Some(oi as u32), pad: Some(p), ..Default::default() }) {
                        if n == 2 {
                            // oval: chain a - centre - b
                            if ids.len() == 2 {
                                constraints.push((ids[0], i));
                                constraints.push((i, ids[1]));
                            }
                            continue;
                        }
                    }
                }
            }
            if ids.len() == 2 {
                constraints.push((ids[0], ids[1]));
            } else if ids.len() > 2 {
                for k in 0..ids.len() {
                    constraints.push((ids[k], ids[(k + 1) % ids.len()]));
                }
            }
        }
        // via sites
        for &(si, pos) in sites {
            ins(&mut cdt, &mut meta, MVert { pos, site: Some(si), ..Default::default() });
        }
        for (a, b) in constraints {
            if a != b {
                let ha = spade::handles::FixedVertexHandle::from_index(a);
                let hb = spade::handles::FixedVertexHandle::from_index(b);
                // split at intersections so overlapping pads keep complete boundaries
                let _ = cdt.add_constraint_and_split(ha, hb, |q| q);
            }
        }

        // ---- convert to compact arrays
        let nv = cdt.num_vertices();
        let mut verts = vec![MVert::default(); nv];
        for v in cdt.vertices() {
            let i = v.fix().index();
            let pos = v.position();
            let mut m = meta.remove(&i).unwrap_or_default();
            m.pos = crate::geom::p(pos.x, pos.y);
            verts[i] = m;
        }
        let mut emap: HashMap<(u32, u32), u32> = HashMap::new();
        let mut edges = Vec::new();
        for e in cdt.undirected_edges() {
            let [a, b] = e.vertices();
            let (a, b) = (a.fix().index() as u32, b.fix().index() as u32);
            let key = (a.min(b), a.max(b));
            let id = edges.len() as u32;
            emap.insert(key, id);
            let len = verts[a as usize].pos.dist(verts[b as usize].pos);
            edges.push(MEdge { v: [key.0, key.1], t: [NONE, NONE], len, constraint: cdt.is_constraint_edge(e.fix()), exit: [0.0; 2], owner: None });
        }
        let ogrid = obs_grid(&obs);
        for ed in edges.iter_mut() {
            let m = verts[ed.v[0] as usize].pos.mid(verts[ed.v[1] as usize].pos);
            for &oi in grid_near(&ogrid, m) {
                if obs[oi as usize].shape.dist(m) < -1e-6 {
                    ed.owner = Some(oi);
                    break;
                }
            }
        }
        let mut tris = Vec::new();
        for f in cdt.inner_faces() {
            let vs = f.vertices();
            let mut v = [vs[0].fix().index() as u32, vs[1].fix().index() as u32, vs[2].fix().index() as u32];
            if orient(verts[v[0] as usize].pos, verts[v[1] as usize].pos, verts[v[2] as usize].pos) < 0.0 {
                v.swap(1, 2);
            }
            let mut e = [0u32; 3];
            for i in 0..3 {
                let (a, b) = (v[i], v[(i + 1) % 3]);
                e[i] = emap[&(a.min(b), a.max(b))];
            }
            let c = (verts[v[0] as usize].pos + verts[v[1] as usize].pos + verts[v[2] as usize].pos) / 3.0;
            let alive = inside_board(c, outline);
            // owner: triangle inside an obstacle core polygon
            // owner: the triangle lies inside the copper of an obstacle
            let mut owner: Option<u32> = None;
            for &oi in grid_near(&ogrid, c) {
                let o = &obs[oi as usize];
                if o.shape.dist(c) < -1e-6 && v.iter().all(|&x| o.shape.dist(verts[x as usize].pos) <= 1e-6) {
                    owner = match owner {
                        None => Some(oi),
                        Some(prev) if obs[prev as usize].net.is_some() && obs[prev as usize].net == o.net => Some(prev),
                        Some(_) => Some(BLOCKED),
                    };
                }
            }
            let _ = point_in_poly;
            let ti = tris.len() as u32;
            for &ei in &e {
                let ed = &mut edges[ei as usize];
                if ed.t[0] == NONE {
                    ed.t[0] = ti;
                } else {
                    ed.t[1] = ti;
                }
            }
            tris.push(MTri { v, e, alive, owner });
        }
        // edge exits (obstacle body + default clearance), board edge clearance for outline dots
        for ed in edges.iter_mut() {
            for k in 0..2 {
                let a = &verts[ed.v[k] as usize];
                let b = &verts[ed.v[1 - k] as usize];
                ed.exit[k] = if let Some(o) = a.obs {
                    exit_dist(&obs[o as usize].shape, a.pos, b.pos, clearance)
                } else if a.outline {
                    edge_clearance
                } else {
                    0.0
                };
            }
        }
        let mut vtris = vec![Vec::new(); nv];
        for (ti, t) in tris.iter().enumerate() {
            for &v in &t.v {
                vtris[v as usize].push(ti as u32);
            }
        }
        let mut site_vert = HashMap::new();
        let mut pad_vert = HashMap::new();
        for (i, v) in verts.iter().enumerate() {
            if let Some(s) = v.site {
                site_vert.insert(s, i as u32);
            }
            if let Some(p) = v.pad {
                pad_vert.insert(p, i as u32);
            }
        }
        Mesh { layer, obs, verts, edges, tris, vtris, site_vert, pad_vert }
    }

    /// Cyclic boundary coordinate of the corner `i` of triangle t.
    #[inline]
    pub fn corner_pos(i: usize) -> f64 {
        i as f64
    }

    /// Is edge `e` (a,b) traversed from v[0] to v[1] in the triangle's CCW order?
    #[inline]
    pub fn edge_forward(&self, t: u32, li: usize) -> bool {
        let tri = &self.tris[t as usize];
        self.edges[tri.e[li] as usize].v[0] == tri.v[li]
    }
}
