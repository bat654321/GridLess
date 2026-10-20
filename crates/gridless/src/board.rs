//! In-memory board model independent of the KiCad file format.

use crate::geom::{BBox, RoundPoly, P};
use serde::{Deserialize, Serialize};

pub type NetId = usize;
pub type LayerId = usize; // index into Board::layers (copper layers only, top to bottom)

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Layer {
    /// canonical KiCad name, e.g. "F.Cu", "In1.Cu"
    pub name: String,
    /// user alias, e.g. "Top Layer"
    pub alias: String,
    pub kicad_id: i32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Net {
    pub name: String,
    pub class: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PadKind {
    Smd,
    Tht,
    Npth,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pad {
    pub footprint: usize,
    pub number: String,
    pub net: Option<NetId>,
    /// pad anchor / connection point (absolute)
    pub pos: P,
    /// copper shape (absolute), convex approximation
    pub shape: RoundPoly,
    pub layers: Vec<LayerId>,
    pub kind: PadKind,
    /// drill diameter (min of slot dims)
    pub drill: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Footprint {
    pub reference: String,
    pub pos: P,
    pub rot_deg: f64,
    pub back: bool,
    pub locked: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum TrackGeom {
    Seg { a: P, b: P },
    Arc { s: P, m: P, e: P },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Track {
    pub layer: LayerId,
    pub net: Option<NetId>,
    pub width: f64,
    pub geom: TrackGeom,
    pub locked: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Via {
    pub pos: P,
    pub dia: f64,
    pub drill: f64,
    pub net: Option<NetId>,
    pub locked: bool,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Keepout {
    pub tracks: bool,
    pub vias: bool,
    pub pads: bool,
    pub pour: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Zone {
    pub net: Option<NetId>,
    pub layers: Vec<LayerId>,
    pub outline: Vec<P>,
    pub keepout: Option<Keepout>,
    pub priority: i32,
    /// filled copper as saved by KiCad's zone filler (layer, polygon); for display only
    #[serde(default)]
    pub fills: Vec<(LayerId, Vec<P>)>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CopperShape {
    pub layer: LayerId,
    pub net: Option<NetId>,
    /// convex pieces
    pub pieces: Vec<RoundPoly>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetClass {
    pub name: String,
    pub clearance: f64,
    pub track_width: f64,
    pub via_dia: f64,
    pub via_drill: f64,
    pub dp_width: f64,
    pub dp_gap: f64,
    pub dp_via_gap: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DesignRules {
    pub min_clearance: f64,
    pub edge_clearance: f64,
    pub hole_clearance: f64,
    pub hole_to_hole: f64,
    pub min_track_width: f64,
}

impl Default for DesignRules {
    fn default() -> Self {
        DesignRules { min_clearance: 0.2, edge_clearance: 0.3, hole_clearance: 0.25, hole_to_hole: 0.25, min_track_width: 0.2 }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Board {
    pub layers: Vec<Layer>,
    pub nets: Vec<Net>,
    pub footprints: Vec<Footprint>,
    pub pads: Vec<Pad>,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub zones: Vec<Zone>,
    /// closed outline polygons from Edge.Cuts (first = largest = outer)
    pub outline: Vec<Vec<P>>,
    /// fixed copper graphics (filled polygons etc.), split into convex pieces
    pub copper: Vec<CopperShape>,
    pub classes: Vec<NetClass>,
    pub rules: DesignRules,
}

impl Board {
    pub fn net_id(&self, name: &str) -> Option<NetId> {
        self.nets.iter().position(|n| n.name == name)
    }
    pub fn layer_id(&self, name: &str) -> Option<LayerId> {
        self.layers.iter().position(|l| l.name == name || l.alias == name)
    }
    pub fn class_of(&self, net: NetId) -> &NetClass {
        let c = &self.nets[net].class;
        self.classes.iter().find(|k| &k.name == c).unwrap_or(&self.classes[0])
    }
    pub fn bbox(&self) -> BBox {
        let mut b = BBox::empty();
        for poly in &self.outline {
            for q in poly {
                b.add(*q);
            }
        }
        if b.is_empty() {
            for pd in &self.pads {
                b = b.union(&pd.shape.bbox());
            }
        }
        b
    }
    pub fn pad_label(&self, i: usize) -> String {
        let pd = &self.pads[i];
        format!("{}.{}", self.footprints[pd.footprint].reference, pd.number)
    }
}
