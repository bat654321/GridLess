//! Router configuration (TOML) and resolved per-net rules.
//!
//! Everything has a sensible default derived from the KiCad project, so a config
//! file is optional. Example `board.gridless.toml`:
//!
//! ```toml
//! [layers."F.Cu"]
//! direction = "horizontal"     # horizontal | vertical | any
//! [layers."In1.Cu"]
//! role = "plane"               # signal | plane | off
//! [via]
//! diameter = 0.5
//! drill = 0.3
//! cost = 6.0                   # mm of wire one via is worth
//! [[nets]]
//! match = ["+5", "3V3"]        # glob patterns
//! width = 0.8
//! [[diff_pairs]]
//! p = "USB_P"
//! n = "USB_N"
//! gap = 0.15
//! [[length_groups]]
//! match = ["A_DATA", "B_DATA", "C_DATA"]
//! tolerance = 0.5
//! [centering]                  # equal gaps between pads (on by default)
//! full = 1.3                   # pad distance (mm) with exactly equal gaps
//! range = 3.25                 # pad distance (mm) from which pads no longer pull
//! ```

use crate::board::{Board, LayerId, NetId};
use crate::kicad::glob_match;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Direction {
    #[default]
    Any,
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum ReserveMode {
    #[default]
    Auto,
    Always,
    Off,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LayerRole {
    Signal,
    Plane,
    Off,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerCfg {
    pub role: Option<LayerRole>,
    pub direction: Option<Direction>,
    /// extra cost factor for moving perpendicular to the preferred direction
    pub direction_weight: Option<f64>,
    /// cost multiplier for using the layer at all (>1 discourages it)
    pub cost: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ViaCfg {
    pub diameter: Option<f64>,
    pub drill: Option<f64>,
    pub cost: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NetRuleCfg {
    #[serde(rename = "match")]
    pub patterns: Vec<String>,
    pub width: Option<f64>,
    pub clearance: Option<f64>,
    /// do not route these nets
    pub skip: Option<bool>,
    /// route order priority (higher first)
    pub priority: Option<i32>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DiffPairCfg {
    pub p: String,
    pub n: String,
    pub width: Option<f64>,
    pub gap: Option<f64>,
    pub max_skew: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LengthGroupCfg {
    #[serde(rename = "match")]
    pub patterns: Vec<String>,
    /// absolute target length; default = longest member
    pub target: Option<f64>,
    pub tolerance: Option<f64>,
}

/// Even spreading of tracks between neighbouring pads ("radar").
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CenteringCfg {
    /// default true
    pub enabled: Option<bool>,
    /// distance between pads (copper to copper, mm) up to which the tracks between them
    /// get exactly equal gaps (default 2 channels of the default net class, see below)
    pub full: Option<f64>,
    /// distance between pads from which they no longer pull the tracks ("radar range";
    /// default 5 channels). Channel = track width + 2 x clearance.
    pub range: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// spread tracks evenly between neighbouring pads
    #[serde(default)]
    pub centering: CenteringCfg,
    #[serde(default)]
    pub layers: BTreeMap<String, LayerCfg>,
    #[serde(default)]
    pub via: ViaCfg,
    #[serde(default)]
    pub nets: Vec<NetRuleCfg>,
    #[serde(default)]
    pub diff_pairs: Vec<DiffPairCfg>,
    /// detect diff pairs from net names (KiCad rules: _P/_N, +/-, P/N)
    pub auto_diff_pairs: Option<bool>,
    #[serde(default)]
    pub length_groups: Vec<LengthGroupCfg>,
    /// keep existing tracks/vias (true) or rip everything that is not locked (false)
    pub keep_existing: Option<bool>,
    /// room for length-matching meanders reserved in the topology: "auto" (default: only
    /// when plain routing leaves a length group short), "always" (TopoR style) or "off"
    pub length_reserve: Option<ReserveMode>,
    /// SMD pads of plane nets on a layer with a same-net pour are left to the pour
    pub trust_pours: Option<bool>,
    /// room (mm) around a pad the pour has to reach, where signal routing pays an
    /// extra cost so it cannot cut the pour away from the pad (0 = off)
    pub pour_keepout: Option<f64>,
    /// prefer 0/45/90 degree track directions: 0 = off, 1 = snap the finished
    /// geometry to the 45 degree grid (a hand-routed board looks like this)
    pub angle45: Option<f64>,
    /// extra cost for walking the mesh at an angle off the 0/45/90 grid (0 = off,
    /// 0.2-0.4 is a gentle preference; large values fight the A* heuristic)
    pub angle_cost: Option<f64>,
    /// centre tracks on the ridge of the distance field (0 = off, 1 = on)
    pub ridge: Option<f64>,
    /// A* cost factor for hugging copper (0 = off; the static field does not know
    /// the other wires yet, so this tends to congest the corridors)
    pub ridge_cost: Option<f64>,
    /// soft pull of a crossing to the middle of its legal interval (0 = off,
    /// 1 = noticeable): the free gap between neighbours, wire-aware by construction
    pub center_gap: Option<f64>,
    /// a link at or below this effective cost (mm, via included) is routed first
    pub short_link: Option<f64>,
    /// distance from copper (mm) at which the ridge pull has faded out
    pub ridge_range: Option<f64>,
    /// grid step (mm) of the distance field
    pub ridge_step: Option<f64>,
    /// router effort: number of negotiation iterations
    pub iterations: Option<usize>,
    pub seed: Option<u64>,
    /// pads ("REF.PAD") of plane nets that must get a via to the plane even if a pour
    /// covers them (filled in automatically by the KiCad zone check)
    #[serde(default)]
    pub force_fanout: Vec<String>,
}

impl Config {
    pub fn from_toml(s: &str) -> anyhow::Result<Config> {
        Ok(toml::from_str(s)?)
    }
}

#[derive(Clone, Debug)]
pub struct LayerRule {
    pub role: LayerRole,
    pub dir: Direction,
    pub dir_weight: f64,
    pub cost: f64,
}

#[derive(Clone, Debug)]
pub struct NetRule {
    pub width: f64,
    pub clearance: f64,
    pub skip: bool,
    pub priority: i32,
    /// layer on which this net has a plane (connect by via)
    pub plane: Option<LayerId>,
}

#[derive(Clone, Debug)]
pub struct DiffPair {
    pub p: NetId,
    pub n: NetId,
    pub width: f64,
    pub gap: f64,
    pub max_skew: f64,
}

#[derive(Clone, Debug)]
pub struct LengthGroup {
    pub nets: Vec<NetId>,
    pub target: Option<f64>,
    pub tolerance: f64,
}

/// Radar of the gap equalisation: how strongly the gap between two pads pulls the tracks
/// running through it to equal spacing, as a function of the distance between the pads.
#[derive(Clone, Copy, Debug)]
pub struct Centering {
    pub full: f64,
    pub range: f64,
}

impl Centering {
    /// 1 = exactly equal gaps, 0 = no pull (free rubber band); smooth in between.
    pub fn weight(&self, dist: f64) -> f64 {
        if dist <= self.full {
            1.0
        } else if dist >= self.range {
            0.0
        } else {
            let x = (dist - self.full) / (self.range - self.full);
            1.0 - x * x * (3.0 - 2.0 * x)
        }
    }
}

/// Fully resolved rules for one routing run.
#[derive(Clone, Debug)]
pub struct Rules {
    pub layers: Vec<LayerRule>,
    pub nets: Vec<NetRule>,
    pub via_dia: f64,
    pub via_drill: f64,
    pub via_cost: f64,
    pub diff_pairs: Vec<DiffPair>,
    pub length_groups: Vec<LengthGroup>,
    pub centering: Option<Centering>,
    pub length_reserve: ReserveMode,
    pub keep_existing: bool,
    pub trust_pours: bool,
    pub pour_keepout: f64,
    pub angle45: f64,
    pub angle_cost: f64,
    pub ridge: f64,
    pub ridge_cost: f64,
    pub center_gap: f64,
    pub short_link: f64,
    pub ridge_range: f64,
    pub ridge_step: f64,
    pub force_fanout: Vec<String>,
    pub iterations: usize,
    pub seed: u64,
    pub edge_clearance: f64,
    pub hole_to_hole: f64,
    pub hole_clearance: f64,
    pub default_clearance: f64,
}

/// KiCad's differential pair suffix matching (BOARD::MatchDpSuffix).
/// Returns (base, is_positive) when the name looks like a pair member.
pub fn dp_suffix(name: &str) -> Option<(String, String, bool)> {
    let chars: Vec<char> = name.chars().collect();
    let mut i = chars.len();
    while i > 0 && (chars[i - 1].is_ascii_digit() || chars[i - 1] == '_') {
        i -= 1;
    }
    if i == 0 {
        return None;
    }
    let c = chars[i - 1];
    let (pos, other) = match c {
        '+' => (true, '-'),
        '-' => (false, '+'),
        'P' => (true, 'N'),
        'N' => (false, 'P'),
        _ => return None,
    };
    let mut partner: Vec<char> = chars.clone();
    partner[i - 1] = other;
    let partner: String = partner.into_iter().collect();
    let base: String = chars[..i - 1].iter().collect();
    Some((base, partner, pos))
}

impl Rules {
    pub fn resolve(b: &Board, cfg: &Config) -> Rules {
        let nl = b.layers.len();
        // ---- planes: an inner layer mostly covered by net zones (possibly split
        // between several nets) is a plane, not a routing layer
        let board_area = {
            let a: f64 = b.outline.first().map(|o| crate::geom::poly_area(o).abs()).unwrap_or(0.0) - b.outline.iter().skip(1).map(|o| crate::geom::poly_area(o).abs()).sum::<f64>();
            if a > 1e-6 {
                a
            } else {
                let bb = b.bbox();
                (bb.w() * bb.h()).max(1e-6)
            }
        };
        let mut zone_area = vec![0.0f64; nl];
        let mut zone_nets: Vec<Vec<NetId>> = vec![Vec::new(); nl];
        for z in &b.zones {
            if z.keepout.is_some() || z.net.is_none() {
                continue;
            }
            let a = crate::geom::poly_area(&z.outline).abs();
            for &l in &z.layers {
                if l > 0 && l + 1 < nl {
                    zone_area[l] += a;
                    if !zone_nets[l].contains(&z.net.unwrap()) {
                        zone_nets[l].push(z.net.unwrap());
                    }
                }
            }
        }
        let auto_plane: Vec<bool> = (0..nl).map(|l| zone_area[l] > 0.4 * board_area).collect();
        let mut layers = Vec::new();
        for (i, l) in b.layers.iter().enumerate() {
            let c = cfg.layers.get(&l.name).or_else(|| cfg.layers.get(&l.alias)).cloned().unwrap_or_default();
            let default_role = if auto_plane[i] { LayerRole::Plane } else { LayerRole::Signal };
            let role = c.role.unwrap_or(default_role);
            if role != LayerRole::Plane {
                zone_nets[i].clear();
            }
            layers.push(LayerRule { role, dir: c.direction.unwrap_or_default(), dir_weight: c.direction_weight.unwrap_or(0.6), cost: c.cost.unwrap_or(1.0) });
        }
        // ---- nets
        let mut nets = Vec::new();
        for (ni, n) in b.nets.iter().enumerate() {
            let cls = b.class_of(ni);
            let mut r = NetRule { width: cls.track_width, clearance: cls.clearance.max(b.rules.min_clearance), skip: false, priority: 0, plane: None };
            for nc in &cfg.nets {
                if nc.patterns.iter().any(|p| glob_match(p, &n.name)) {
                    if let Some(w) = nc.width {
                        r.width = w;
                    }
                    if let Some(c) = nc.clearance {
                        r.clearance = c;
                    }
                    if let Some(s) = nc.skip {
                        r.skip = s;
                    }
                    if let Some(p) = nc.priority {
                        r.priority = p;
                    }
                }
            }
            r.width = r.width.max(b.rules.min_track_width);
            r.plane = zone_nets.iter().position(|v| v.contains(&ni));
            nets.push(r);
        }
        let dc = &b.classes[0];
        let via_dia = cfg.via.diameter.unwrap_or(dc.via_dia);
        let via_drill = cfg.via.drill.unwrap_or(dc.via_drill);
        // ---- diff pairs
        let mut diff_pairs = Vec::new();
        let mut used = vec![false; b.nets.len()];
        for d in &cfg.diff_pairs {
            if let (Some(p), Some(n)) = (b.net_id(&d.p), b.net_id(&d.n)) {
                let cls = b.class_of(p);
                diff_pairs.push(DiffPair { p, n, width: d.width.unwrap_or(cls.dp_width), gap: d.gap.unwrap_or(cls.dp_gap), max_skew: d.max_skew.unwrap_or(0.1) });
                used[p] = true;
                used[n] = true;
            }
        }
        if cfg.auto_diff_pairs.unwrap_or(true) {
            for (pi, n) in b.nets.iter().enumerate() {
                if used[pi] {
                    continue;
                }
                if let Some((_, partner, true)) = dp_suffix(&n.name) {
                    if let Some(ni) = b.net_id(&partner) {
                        if !used[ni] {
                            let cls = b.class_of(pi);
                            diff_pairs.push(DiffPair { p: pi, n: ni, width: cls.dp_width, gap: cls.dp_gap, max_skew: 0.1 });
                            used[pi] = true;
                            used[ni] = true;
                        }
                    }
                }
            }
        }
        let length_groups = cfg
            .length_groups
            .iter()
            .map(|g| LengthGroup {
                nets: (0..b.nets.len()).filter(|&i| g.patterns.iter().any(|p| glob_match(p, &b.nets[i].name))).collect(),
                target: g.target,
                tolerance: g.tolerance.unwrap_or(0.5),
            })
            .filter(|g| !g.nets.is_empty())
            .collect();
        let default_clearance = dc.clearance.max(b.rules.min_clearance);
        let centering = if cfg.centering.enabled.unwrap_or(true) {
            let channel = dc.track_width.max(b.rules.min_track_width) + 2.0 * default_clearance;
            let full = cfg.centering.full.unwrap_or(2.0 * channel);
            let range = cfg.centering.range.unwrap_or(5.0 * channel).max(full + 1e-3);
            Some(Centering { full, range })
        } else {
            None
        };
        Rules {
            layers,
            nets,
            via_dia,
            via_drill,
            via_cost: cfg.via.cost.unwrap_or(6.0),
            diff_pairs,
            length_groups,
            centering,
            length_reserve: cfg.length_reserve.unwrap_or_default(),
            keep_existing: cfg.keep_existing.unwrap_or(false),
            trust_pours: cfg.trust_pours.unwrap_or(true),
            pour_keepout: cfg.pour_keepout.unwrap_or(0.0),
            angle45: cfg.angle45.unwrap_or(0.0),
            angle_cost: cfg.angle_cost.unwrap_or(0.0),
            ridge: cfg.ridge.unwrap_or(0.0),
            ridge_cost: cfg.ridge_cost.unwrap_or(0.0),
            center_gap: cfg.center_gap.unwrap_or(0.0),
            short_link: cfg.short_link.unwrap_or(3.0),
            ridge_range: cfg.ridge_range.unwrap_or(1.2),
            ridge_step: cfg.ridge_step.unwrap_or(0.25),
            force_fanout: cfg.force_fanout.clone(),
            iterations: cfg.iterations.unwrap_or(40),
            seed: cfg.seed.unwrap_or(1),
            edge_clearance: b.rules.edge_clearance.max(0.0),
            hole_to_hole: b.rules.hole_to_hole,
            hole_clearance: b.rules.hole_clearance,
            default_clearance,
        }
    }

    pub fn routing_layers(&self) -> Vec<LayerId> {
        (0..self.layers.len()).filter(|&l| self.layers[l].role == LayerRole::Signal).collect()
    }

    /// Clearance between two nets (max of both net clearances).
    pub fn clr(&self, a: Option<NetId>, b: Option<NetId>) -> f64 {
        let ca = a.map(|n| self.nets[n].clearance).unwrap_or(self.default_clearance);
        let cb = b.map(|n| self.nets[n].clearance).unwrap_or(self.default_clearance);
        ca.max(cb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dp() {
        assert_eq!(dp_suffix("USB_P").map(|x| (x.1, x.2)), Some(("USB_N".into(), true)));
        assert_eq!(dp_suffix("CLK_N1").map(|x| (x.1, x.2)), Some(("CLK_P1".into(), false)));
        assert_eq!(dp_suffix("D+").map(|x| (x.1, x.2)), Some(("D-".into(), true)));
        assert!(dp_suffix("GND").is_none());
        assert!(dp_suffix("USB_DM").is_none());
    }
}
