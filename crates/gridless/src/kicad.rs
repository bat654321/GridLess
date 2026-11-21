//! KiCad 10 (.kicad_pcb version 2026xxxx, also KiCad 7-9) reader and track writer.

use crate::board::*;
use crate::geom::{convex_hull, p, Arc, RoundPoly, P};
use crate::sexpr::{self, fmt_mm, quote, SList, Sx};
use anyhow::{Context, Result};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// KiCad rotation (degrees, counter-clockwise on screen with Y pointing down).
pub fn krot(v: P, deg: f64) -> P {
    if deg == 0.0 {
        return v;
    }
    let a = deg.to_radians();
    let (s, c) = a.sin_cos();
    p(v.x * c + v.y * s, -v.x * s + v.y * c)
}

fn xy(l: &SList) -> Option<P> {
    Some(p(l.argf(0)?, l.argf(1)?))
}

fn at(l: &SList) -> (P, f64) {
    match l.get("at") {
        Some(a) => (p(a.argf(0).unwrap_or(0.0), a.argf(1).unwrap_or(0.0)), a.argf(2).unwrap_or(0.0)),
        None => (P::default(), 0.0),
    }
}

/// Net name of an item: KiCad 10 uses (net "name"), older versions (net 5 "name").
fn item_net(l: &SList, numbered: &HashMap<i64, String>) -> Option<String> {
    let n = l.get("net")?;
    match n.items.len() {
        2 => {
            let t = n.items[1].text()?;
            if let (Sx::Atom(a), Ok(k)) = (&n.items[1], n.items[1].text()?.parse::<i64>()) {
                let _ = a;
                return numbered.get(&k).cloned().filter(|s| !s.is_empty());
            }
            if t.is_empty() {
                None
            } else {
                Some(t.to_string())
            }
        }
        3.. => n.arg(1).map(|s| s.to_string()).filter(|s| !s.is_empty()),
        _ => None,
    }
}

fn is_copper(name: &str) -> bool {
    name.ends_with(".Cu")
}

fn copper_order(name: &str) -> i32 {
    if name == "F.Cu" {
        0
    } else if name == "B.Cu" {
        10000
    } else if let Some(n) = name.strip_prefix("In").and_then(|s| s.strip_suffix(".Cu")) {
        n.parse::<i32>().unwrap_or(5000)
    } else {
        9000
    }
}

pub struct Loaded {
    pub board: Board,
    pub path: PathBuf,
    pub source: String,
}

/// Expand KiCad layer list ("*.Cu", "F&B.Cu", explicit names) into copper layer ids.
fn expand_layers(names: &[&str], layers: &[Layer]) -> Vec<LayerId> {
    let mut out = Vec::new();
    for n in names {
        match *n {
            "*.Cu" => out.extend(0..layers.len()),
            "F&B.Cu" => {
                out.push(0);
                out.push(layers.len() - 1);
            }
            other => {
                if let Some(i) = layers.iter().position(|l| l.name == other) {
                    out.push(i);
                }
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

fn pad_shape(pl: &SList, size: P) -> (Vec<P>, f64) {
    let shape = pl.arg(2).unwrap_or("rect");
    let (w, h) = (size.x, size.y);
    match shape {
        "circle" => (vec![p(0.0, 0.0)], w / 2.0),
        "oval" => {
            if (w - h).abs() < 1e-9 {
                (vec![p(0.0, 0.0)], w / 2.0)
            } else if w > h {
                (vec![p(-(w - h) / 2.0, 0.0), p((w - h) / 2.0, 0.0)], h / 2.0)
            } else {
                (vec![p(0.0, -(h - w) / 2.0), p(0.0, (h - w) / 2.0)], w / 2.0)
            }
        }
        "roundrect" => {
            let ratio = pl.valf("roundrect_rratio").unwrap_or(0.25).clamp(0.0, 0.5);
            let cr = ratio * w.min(h);
            let (hw, hh) = (w / 2.0 - cr, h / 2.0 - cr);
            (rect_pts(hw, hh), cr)
        }
        "trapezoid" => {
            let d = pl.get("rect_delta").and_then(xy).unwrap_or_default();
            let (hw, hh) = (w / 2.0, h / 2.0);
            let (dx, dy) = (d.x / 2.0, d.y / 2.0);
            let pts = vec![p(-hw - dy, hh + dx), p(-hw + dy, -hh - dx), p(hw - dy, -hh + dx), p(hw + dy, hh - dx)];
            (convex_hull(pts), 0.0)
        }
        "custom" => {
            let mut pts = rect_pts(w / 2.0, h / 2.0);
            let anchor_circle = pl.get("options").and_then(|o| o.vals("anchor")) == Some("circle");
            if anchor_circle {
                pts = circle_pts(P::default(), w / 2.0);
            }
            if let Some(prims) = pl.get("primitives") {
                for pr in prims.lists() {
                    let wd = pr.get("width").and_then(|x| x.argf(0)).or_else(|| pr.get("stroke").and_then(|s| s.valf("width"))).unwrap_or(0.0) / 2.0;
                    match pr.head() {
                        "gr_poly" => {
                            if let Some(pts_l) = pr.get("pts") {
                                for x in pts_l.all("xy") {
                                    if let Some(q) = xy(x) {
                                        pts.extend(circle_pts(q, wd));
                                    }
                                }
                            }
                        }
                        "gr_circle" => {
                            if let (Some(c), Some(e)) = (pr.get("center").and_then(xy), pr.get("end").and_then(xy)) {
                                pts.extend(circle_pts(c, c.dist(e) + wd));
                            }
                        }
                        "gr_line" | "gr_rect" | "gr_arc" => {
                            for k in ["start", "mid", "end"] {
                                if let Some(q) = pr.get(k).and_then(xy) {
                                    pts.extend(circle_pts(q, wd));
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
            (convex_hull(pts), 0.0)
        }
        _ => (rect_pts(w / 2.0, h / 2.0), 0.0),
    }
}

fn rect_pts(hw: f64, hh: f64) -> Vec<P> {
    if hw <= 1e-9 && hh <= 1e-9 {
        return vec![p(0.0, 0.0)];
    }
    if hw <= 1e-9 {
        return vec![p(0.0, -hh), p(0.0, hh)];
    }
    if hh <= 1e-9 {
        return vec![p(-hw, 0.0), p(hw, 0.0)];
    }
    vec![p(-hw, -hh), p(hw, -hh), p(hw, hh), p(-hw, hh)]
}

fn circle_pts(c: P, r: f64) -> Vec<P> {
    if r <= 1e-9 {
        return vec![c];
    }
    // circumscribed octagon (conservative)
    let rr = r / (std::f64::consts::PI / 8.0).cos();
    (0..8).map(|i| c + p((i as f64 * 45.0f64.to_radians()).cos(), (i as f64 * 45.0f64.to_radians()).sin()) * rr).collect()
}

/// Collect Edge.Cuts primitives as polylines.
fn edge_prims(l: &SList, xf: &dyn Fn(P) -> P, out: &mut Vec<Vec<P>>) {
    let on_edge = l.vals("layer") == Some("Edge.Cuts");
    if !on_edge {
        return;
    }
    let g = |k: &str| l.get(k).and_then(xy).map(xf);
    let head = l.head();
    let kind = head.trim_start_matches("gr_").trim_start_matches("fp_");
    match kind {
        "line" => {
            if let (Some(a), Some(b)) = (g("start"), g("end")) {
                out.push(vec![a, b]);
            }
        }
        "arc" => {
            if let (Some(a), Some(m), Some(b)) = (g("start"), g("mid"), g("end")) {
                if let Some(arc) = Arc::from_3p(a, m, b) {
                    let mut pl = arc.polyline(0.005);
                    pl[0] = a;
                    *pl.last_mut().unwrap() = b;
                    out.push(pl);
                }
            }
        }
        "rect" => {
            if let (Some(a), Some(b)) = (l.get("start").and_then(xy), l.get("end").and_then(xy)) {
                let pts = vec![a, p(b.x, a.y), b, p(a.x, b.y), a];
                out.push(pts.into_iter().map(xf).collect());
            }
        }
        "circle" => {
            if let (Some(c), Some(e)) = (l.get("center").and_then(xy), l.get("end").and_then(xy)) {
                let arc = Arc { c, r: c.dist(e), a0: 0.0, sweep: 2.0 * std::f64::consts::PI };
                out.push(arc.polyline(0.005).into_iter().map(xf).collect());
            }
        }
        "poly" => {
            if let Some(pts) = l.get("pts") {
                let mut v = poly_pts(pts);
                if let Some(f) = v.first().copied() {
                    v.push(f);
                }
                out.push(v.into_iter().map(xf).collect());
            }
        }
        _ => {}
    }
}

/// Points of a (pts ...) list, arcs approximated.
fn poly_pts(pts: &SList) -> Vec<P> {
    let mut v = Vec::new();
    for it in pts.lists() {
        match it.head() {
            "xy" => {
                if let Some(q) = xy(it) {
                    v.push(q);
                }
            }
            "arc" => {
                if let (Some(a), Some(m), Some(b)) = (it.get("start").and_then(xy), it.get("mid").and_then(xy), it.get("end").and_then(xy)) {
                    if let Some(arc) = Arc::from_3p(a, m, b) {
                        v.extend(arc.polyline(0.005));
                    }
                }
            }
            _ => {}
        }
    }
    v
}

/// Chain open polylines into closed loops.
fn chain_loops(segs_in: Vec<Vec<P>>) -> Vec<Vec<P>> {
    let tol = 0.01;
    // drop duplicated primitives (common in boards imported from other EDA tools)
    let mut segs: Vec<Vec<P>> = Vec::new();
    for s in segs_in {
        let dup = segs.iter().any(|o| {
            o.len() == s.len()
                && ((o[0].dist(s[0]) < tol && o.last().unwrap().dist(*s.last().unwrap()) < tol)
                    || (o[0].dist(*s.last().unwrap()) < tol && o.last().unwrap().dist(s[0]) < tol))
        });
        if !dup {
            segs.push(s);
        }
    }
    let mut loops = Vec::new();
    while let Some(mut cur) = segs.pop() {
        loop {
            let first = cur[0];
            let last = *cur.last().unwrap();
            if cur.len() > 2 && first.dist(last) < tol {
                cur.pop();
                loops.push(cur);
                break;
            }
            let mut found = false;
            for i in 0..segs.len() {
                let s = &segs[i];
                if s[0].dist(last) < tol {
                    let s = segs.swap_remove(i);
                    cur.extend(s.into_iter().skip(1));
                    found = true;
                    break;
                } else if s.last().unwrap().dist(last) < tol {
                    let mut s = segs.swap_remove(i);
                    s.reverse();
                    cur.extend(s.into_iter().skip(1));
                    found = true;
                    break;
                }
            }
            if !found {
                // open chain: keep as-is (closed implicitly)
                if cur.len() >= 3 {
                    loops.push(cur);
                }
                break;
            }
        }
    }
    loops.sort_by(|a, b| crate::geom::poly_area(b).abs().partial_cmp(&crate::geom::poly_area(a).abs()).unwrap());
    loops
}

pub fn load(path: &Path) -> Result<Loaded> {
    load_with_project(path, None)
}

/// Load a board; `project` overrides the .kicad_pro that sits next to it.
pub fn load_with_project(path: &Path, project: Option<&Path>) -> Result<Loaded> {
    let source = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let root = sexpr::parse(&source)?;
    let mut b = Board::default();

    // ---- layers
    let mut cu: Vec<Layer> = Vec::new();
    if let Some(ls) = root.get("layers") {
        for l in ls.lists() {
            let id: i32 = l.head().parse().unwrap_or(-1);
            let name = l.arg(0).unwrap_or("").to_string();
            if is_copper(&name) {
                let alias = l.arg(2).unwrap_or(&name).to_string();
                cu.push(Layer { name, alias, kicad_id: id });
            }
        }
    }
    cu.sort_by_key(|l| copper_order(&l.name));
    b.layers = cu;

    // ---- nets (old numbered table or names collected from items)
    let mut numbered: HashMap<i64, String> = HashMap::new();
    for n in root.all("net") {
        if let (Some(k), Some(name)) = (n.argf(0), n.arg(1)) {
            numbered.insert(k as i64, name.to_string());
        }
    }
    let mut net_index: HashMap<String, NetId> = HashMap::new();
    let mut intern = |name: Option<String>, b: &mut Board| -> Option<NetId> {
        let name = name?;
        if let Some(&i) = net_index.get(&name) {
            return Some(i);
        }
        let i = b.nets.len();
        b.nets.push(Net { name: name.clone(), class: "Default".into() });
        net_index.insert(name, i);
        Some(i)
    };

    // ---- footprints and pads
    let mut edge_segs: Vec<Vec<P>> = Vec::new();
    for fl in root.all("footprint") {
        let (fpos, frot) = at(fl);
        let back = fl.vals("layer") == Some("B.Cu");
        let reference = fl
            .all("property")
            .find(|pr| pr.arg(0) == Some("Reference"))
            .and_then(|pr| pr.arg(1))
            .or_else(|| fl.all("fp_text").find(|t| t.arg(0) == Some("reference")).and_then(|t| t.arg(1)))
            .unwrap_or("")
            .to_string();
        let fi = b.footprints.len();
        b.footprints.push(Footprint { reference, pos: fpos, rot_deg: frot, back, locked: fl.has_atom("locked") || fl.flag("locked") });
        let xf = move |q: P| fpos + krot(q, frot);
        for it in fl.lists() {
            if it.head().starts_with("fp_") {
                edge_prims(it, &xf, &mut edge_segs);
            }
        }
        for pl in fl.all("pad") {
            let number = pl.arg(0).unwrap_or("").to_string();
            let kind = match pl.arg(1) {
                Some("smd") | Some("connect") => PadKind::Smd,
                Some("np_thru_hole") => PadKind::Npth,
                _ => PadKind::Tht,
            };
            if pl.arg(1) == Some("connect") {
                continue; // no copper connection semantics for routing
            }
            let (ppos, prot) = at(pl);
            let size = pl.get("size").and_then(xy).unwrap_or(p(0.0, 0.0));
            let (mut core, r) = pad_shape(pl, size);
            let mut shape_off = P::default();
            let mut drill = None;
            if let Some(d) = pl.get("drill") {
                let nums: Vec<f64> = d.items.iter().skip(1).filter_map(|x| x.num()).collect();
                if let Some(&d0) = nums.first() {
                    drill = Some(if nums.len() > 1 { d0.min(nums[1]) } else { d0 });
                }
                if let Some(o) = d.get("offset").and_then(xy) {
                    shape_off = o;
                }
            }
            let abs = fpos + krot(ppos, frot);
            for q in core.iter_mut() {
                *q = abs + krot(*q + shape_off, prot);
            }
            let layers_l = pl.get("layers").map(|l| l.texts()).unwrap_or_default();
            let layers = expand_layers(&layers_l, &b.layers);
            let net = intern(item_net(pl, &numbered), &mut b);
            if kind == PadKind::Npth && size.x <= drill.unwrap_or(0.0) + 1e-6 {
                // pure hole: copper-free but still an obstacle for vias/holes
            }
            b.pads.push(Pad { footprint: fi, number, net, pos: abs, shape: RoundPoly { pts: core, r }, layers, kind, drill });
        }
        // Zones inside a footprint (Altium component keepouts, thermal pours) are stored
        // in board coordinates. Ignoring them let the router cross XT1's keepout.
        for z in fl.all("zone") {
            let net = intern(item_net(z, &numbered).or_else(|| z.vals("net_name").map(|s| s.to_string()).filter(|s| !s.is_empty())), &mut b);
            let mut names: Vec<&str> = Vec::new();
            if let Some(l) = z.get("layer") {
                names.extend(l.texts());
            }
            if let Some(l) = z.get("layers") {
                names.extend(l.texts());
            }
            let layers = expand_layers(&names, &b.layers);
            let keepout = z.get("keepout").map(|k| Keepout {
                tracks: k.vals("tracks") == Some("not_allowed"),
                vias: k.vals("vias") == Some("not_allowed"),
                pads: k.vals("pads") == Some("not_allowed"),
                pour: k.vals("copperpour") == Some("not_allowed"),
            });
            let outline = z.get("polygon").and_then(|pg| pg.get("pts")).map(poly_pts).unwrap_or_default();
            let priority = z.valf("priority").unwrap_or(0.0) as i32;
            if !layers.is_empty() && outline.len() >= 3 {
                let fills = z
                    .all("filled_polygon")
                    .filter_map(|fp| {
                        let l = fp.vals("layer").and_then(|n| b.layers.iter().position(|x| x.name == n))?;
                        Some((l, fp.get("pts").map(poly_pts).unwrap_or_default()))
                    })
                    .collect();
                b.zones.push(Zone { net, layers, outline, keepout, priority, fills });
            }
        }
    }

    // ---- board-level graphics (outline)
    for it in root.lists() {
        if it.head().starts_with("gr_") {
            edge_prims(it, &|q| q, &mut edge_segs);
        }
    }
    b.outline = chain_loops(edge_segs);
    // copper graphics with a net (fixed fills drawn as polygons / rectangles / circles)
    for it in root.lists() {
        if !it.head().starts_with("gr_") {
            continue;
        }
        let layer = match it.vals("layer").and_then(|n| b.layers.iter().position(|l| l.name == n)) {
            Some(l) => l,
            None => continue,
        };
        let net = intern(item_net(it, &numbered), &mut b);
        let sw = it.get("stroke").and_then(|st| st.valf("width")).or_else(|| it.valf("width")).unwrap_or(0.0) / 2.0;
        let filled = matches!(it.vals("fill"), Some("yes") | Some("solid"));
        let mut pieces = Vec::new();
        match it.head() {
            "gr_poly" | "gr_rect" => {
                let poly = if it.head() == "gr_poly" {
                    it.get("pts").map(poly_pts).unwrap_or_default()
                } else {
                    match (it.get("start").and_then(xy), it.get("end").and_then(xy)) {
                        (Some(a), Some(c)) => vec![a, p(c.x, a.y), c, p(a.x, c.y)],
                        _ => vec![],
                    }
                };
                if poly.len() >= 3 && filled {
                    for pc in crate::geom::convex_pieces(&poly) {
                        pieces.push(RoundPoly { pts: pc, r: sw });
                    }
                } else {
                    let n = poly.len();
                    for k in 0..n {
                        pieces.push(RoundPoly::capsule(poly[k], poly[(k + 1) % n], sw.max(0.01)));
                    }
                }
            }
            "gr_line" => {
                if let (Some(a), Some(c)) = (it.get("start").and_then(xy), it.get("end").and_then(xy)) {
                    pieces.push(RoundPoly::capsule(a, c, sw.max(0.01)));
                }
            }
            "gr_circle" => {
                if let (Some(c), Some(e)) = (it.get("center").and_then(xy), it.get("end").and_then(xy)) {
                    let r = c.dist(e);
                    if filled {
                        pieces.push(RoundPoly::circle(c, r + sw));
                    } else {
                        let pl = Arc { c, r, a0: 0.0, sweep: 2.0 * std::f64::consts::PI }.polyline(0.01);
                        for w in pl.windows(2) {
                            pieces.push(RoundPoly::capsule(w[0], w[1], sw.max(0.01)));
                        }
                    }
                }
            }
            _ => {}
        }
        if !pieces.is_empty() {
            b.copper.push(CopperShape { layer, net, pieces });
        }
    }

    // ---- tracks and vias
    for it in root.lists() {
        match it.head() {
            "segment" | "arc" => {
                let layer = match it.vals("layer").and_then(|n| b.layers.iter().position(|l| l.name == n)) {
                    Some(l) => l,
                    None => continue,
                };
                let net = intern(item_net(it, &numbered), &mut b);
                let width = it.valf("width").unwrap_or(0.25);
                let g = |k: &str| it.get(k).and_then(xy);
                let geom = if it.head() == "segment" {
                    TrackGeom::Seg { a: g("start").unwrap_or_default(), b: g("end").unwrap_or_default() }
                } else {
                    TrackGeom::Arc { s: g("start").unwrap_or_default(), m: g("mid").unwrap_or_default(), e: g("end").unwrap_or_default() }
                };
                b.tracks.push(Track { layer, net, width, geom, locked: it.has_atom("locked") || it.flag("locked") });
            }
            "via" => {
                let (pos, _) = at(it);
                let dia = it.valf("size").or_else(|| it.get("padstack").and_then(|ps| ps.get("layer")).and_then(|l| l.get("size")).and_then(|s| s.argf(0))).unwrap_or(0.6);
                let drill = it.valf("drill").unwrap_or(0.3);
                let net = intern(item_net(it, &numbered), &mut b);
                b.vias.push(Via { pos, dia, drill, net, locked: it.has_atom("locked") || it.flag("locked") });
            }
            "zone" => {
                let net = intern(item_net(it, &numbered).or_else(|| it.vals("net_name").map(|s| s.to_string()).filter(|s| !s.is_empty())), &mut b);
                let mut names: Vec<&str> = Vec::new();
                if let Some(l) = it.get("layer") {
                    names.extend(l.texts());
                }
                if let Some(l) = it.get("layers") {
                    names.extend(l.texts());
                }
                let layers = expand_layers(&names, &b.layers);
                let keepout = it.get("keepout").map(|k| Keepout {
                    tracks: k.vals("tracks") == Some("not_allowed"),
                    vias: k.vals("vias") == Some("not_allowed"),
                    pads: k.vals("pads") == Some("not_allowed"),
                    pour: k.vals("copperpour") == Some("not_allowed"),
                });
                let outline = it.get("polygon").and_then(|pg| pg.get("pts")).map(poly_pts).unwrap_or_default();
                let priority = it.valf("priority").unwrap_or(0.0) as i32;
                if !layers.is_empty() && outline.len() >= 3 {
                    let fills = it
                        .all("filled_polygon")
                        .filter_map(|fp| {
                            let l = fp.vals("layer").and_then(|n| b.layers.iter().position(|x| x.name == n))?;
                            Some((l, fp.get("pts").map(poly_pts).unwrap_or_default()))
                        })
                        .collect();
                    b.zones.push(Zone { net, layers, outline, keepout, priority, fills });
                }
            }
            _ => {}
        }
    }

    // ---- project rules
    b.classes = vec![NetClass { name: "Default".into(), clearance: 0.2, track_width: 0.25, via_dia: 0.6, via_drill: 0.3, dp_width: 0.2, dp_gap: 0.25, dp_via_gap: 0.25 }];
    let pro = project.map(|p| p.to_path_buf()).unwrap_or_else(|| path.with_extension("kicad_pro"));
    if pro.exists() {
        if let Err(e) = load_project(&pro, &mut b) {
            log::warn!("cannot read {}: {e:#}", pro.display());
        }
    }
    Ok(Loaded { board: b, path: path.to_path_buf(), source })
}

/// Simple glob with '*' and '?' wildcards (KiCad netclass patterns).
pub fn glob_match(pat: &str, s: &str) -> bool {
    let p: Vec<char> = pat.chars().collect();
    let t: Vec<char> = s.chars().collect();
    fn rec(p: &[char], t: &[char]) -> bool {
        match (p.first(), t.first()) {
            (None, None) => true,
            (Some('*'), _) => rec(&p[1..], t) || (!t.is_empty() && rec(p, &t[1..])),
            (Some('?'), Some(_)) => rec(&p[1..], &t[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &t[1..]),
            _ => false,
        }
    }
    rec(&p, &t)
}

fn load_project(pro: &Path, b: &mut Board) -> Result<()> {
    let txt = std::fs::read_to_string(pro)?;
    let j: serde_json::Value = serde_json::from_str(&txt)?;
    let ds = &j["board"]["design_settings"];
    let r = &ds["rules"];
    let f = |v: &serde_json::Value, d: f64| v.as_f64().unwrap_or(d);
    b.rules = DesignRules {
        min_clearance: f(&r["min_clearance"], 0.2),
        edge_clearance: f(&r["min_copper_edge_clearance"], 0.3).max(0.0),
        hole_clearance: f(&r["min_hole_clearance"], 0.25),
        hole_to_hole: f(&r["min_hole_to_hole"], 0.25),
        min_track_width: f(&r["min_track_width"], 0.2),
    };
    let ns = &j["net_settings"];
    let mut classes: Vec<NetClass> = Vec::new();
    let mut default = b.classes[0].clone();
    if let Some(arr) = ns["classes"].as_array() {
        for c in arr {
            let name = c["name"].as_str().unwrap_or("").to_string();
            let base = if name == "Default" { default.clone() } else { classes.iter().find(|k| k.name == "Default").cloned().unwrap_or(default.clone()) };
            let nc = NetClass {
                name: name.clone(),
                clearance: f(&c["clearance"], base.clearance),
                track_width: f(&c["track_width"], base.track_width),
                via_dia: f(&c["via_diameter"], base.via_dia),
                via_drill: f(&c["via_drill"], base.via_drill),
                dp_width: f(&c["diff_pair_width"], base.dp_width),
                dp_gap: f(&c["diff_pair_gap"], base.dp_gap),
                dp_via_gap: f(&c["diff_pair_via_gap"], base.dp_via_gap),
            };
            if name == "Default" {
                default = nc.clone();
            }
            classes.push(nc);
        }
    }
    if !classes.iter().any(|c| c.name == "Default") {
        classes.insert(0, default);
    } else {
        let i = classes.iter().position(|c| c.name == "Default").unwrap();
        let d = classes.remove(i);
        classes.insert(0, d);
    }
    b.classes = classes;
    // explicit assignments then patterns
    let mut assigned: HashMap<String, String> = HashMap::new();
    if let Some(m) = ns["netclass_assignments"].as_object() {
        for (net, cls) in m {
            let c = match cls {
                serde_json::Value::String(s) => s.clone(),
                serde_json::Value::Array(a) => a.iter().filter_map(|x| x.as_str()).next().unwrap_or("Default").to_string(),
                _ => continue,
            };
            assigned.insert(net.clone(), c);
        }
    }
    let patterns: Vec<(String, String)> = ns["netclass_patterns"]
        .as_array()
        .map(|a| a.iter().filter_map(|x| Some((x["pattern"].as_str()?.to_string(), x["netclass"].as_str()?.to_string()))).collect())
        .unwrap_or_default();
    for n in b.nets.iter_mut() {
        if let Some(c) = assigned.get(&n.name) {
            n.class = c.clone();
        } else if let Some((_, c)) = patterns.iter().find(|(pat, _)| glob_match(pat, &n.name)) {
            n.class = c.clone();
        }
        if !b.classes.iter().any(|k| k.name == n.class) {
            n.class = "Default".into();
        }
    }
    Ok(())
}

// ------------------------------------------------------------------ writer

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum OutItem {
    Seg { layer: String, net: String, width: f64, a: P, b: P },
    Arc { layer: String, net: String, width: f64, s: P, m: P, e: P },
    Via { net: String, pos: P, dia: f64, drill: f64 },
}

fn uuid_like(seed: u64) -> String {
    // deterministic pseudo-uuid (v4 layout)
    let mut x = seed.wrapping_mul(0x9E3779B97F4A7C15) ^ 0xD1B54A32D192ED03;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    let a = next();
    let b = next();
    format!(
        "{:08x}-{:04x}-4{:03x}-{:04x}-{:012x}",
        (a >> 32) as u32,
        (a >> 16) as u16,
        (a as u16) & 0x0fff,
        ((b >> 48) as u16 & 0x3fff) | 0x8000,
        b & 0xffff_ffff_ffff
    )
}

pub fn format_item(it: &OutItem, seed: u64) -> String {
    let u = uuid_like(seed);
    match it {
        OutItem::Seg { layer, net, width, a, b } => format!(
            "\t(segment\n\t\t(start {} {})\n\t\t(end {} {})\n\t\t(width {})\n\t\t(layer {})\n\t\t(net {})\n\t\t(uuid \"{}\")\n\t)\n",
            fmt_mm(a.x), fmt_mm(a.y), fmt_mm(b.x), fmt_mm(b.y), fmt_mm(*width), quote(layer), quote(net), u
        ),
        OutItem::Arc { layer, net, width, s, m, e } => format!(
            "\t(arc\n\t\t(start {} {})\n\t\t(mid {} {})\n\t\t(end {} {})\n\t\t(width {})\n\t\t(layer {})\n\t\t(net {})\n\t\t(uuid \"{}\")\n\t)\n",
            fmt_mm(s.x), fmt_mm(s.y), fmt_mm(m.x), fmt_mm(m.y), fmt_mm(e.x), fmt_mm(e.y), fmt_mm(*width), quote(layer), quote(net), u
        ),
        OutItem::Via { net, pos, dia, drill } => format!(
            "\t(via\n\t\t(at {} {})\n\t\t(size {})\n\t\t(drill {})\n\t\t(layers \"F.Cu\" \"B.Cu\")\n\t\t(net {})\n\t\t(uuid \"{}\")\n\t)\n",
            fmt_mm(pos.x), fmt_mm(pos.y), fmt_mm(*dia), fmt_mm(*drill), quote(net), u
        ),
    }
}

/// Write a copy of the board with (optionally) unlocked tracks/vias removed and new items appended.
pub fn write_routed(src: &str, out: &Path, items: &[OutItem], strip_unlocked: bool) -> Result<()> {
    let root = sexpr::parse(src)?;
    let mut cut: Vec<(usize, usize)> = Vec::new();
    if strip_unlocked {
        for it in root.lists() {
            if matches!(it.head(), "segment" | "arc" | "via") && !(it.has_atom("locked") || it.flag("locked")) {
                // extend span to swallow the leading indentation and trailing newline
                let mut s = it.span.0;
                while s > 0 && (src.as_bytes()[s - 1] == b'\t' || src.as_bytes()[s - 1] == b' ') {
                    s -= 1;
                }
                let mut e = it.span.1;
                if src.as_bytes().get(e) == Some(&b'\r') {
                    e += 1;
                }
                if src.as_bytes().get(e) == Some(&b'\n') {
                    e += 1;
                }
                cut.push((s, e));
            }
        }
    }
    let close = root.span.1 - 1; // position of final ')'
    let mut o = String::with_capacity(src.len() + items.len() * 200);
    let mut pos = 0;
    for (s, e) in cut {
        o.push_str(&src[pos..s]);
        pos = e;
    }
    o.push_str(&src[pos..close]);
    if !o.ends_with('\n') {
        o.push('\n');
    }
    for (i, it) in items.iter().enumerate() {
        o.push_str(&format_item(it, i as u64 + 1));
    }
    o.push_str(&src[close..]);
    std::fs::write(out, o)?;
    Ok(())
}
