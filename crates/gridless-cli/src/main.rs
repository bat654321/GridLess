use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use gridless::config::{Config, Rules};
use gridless::geom::p;
use gridless::problem::Problem;
use gridless::render::{Canvas, LAYER_COLORS};
use std::path::{Path, PathBuf};
use std::time::Instant;

#[derive(Parser)]
#[command(name = "gridless", version, about = "Gridless topological PCB autorouter for KiCad")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Print board summary; optionally dump the parsed model as JSON
    Info {
        board: PathBuf,
        #[arg(long)]
        json: Option<PathBuf>,
    },
    /// Draw a board as it is (zone fills, pads, tracks, vias) with numbered markers
    Render {
        board: PathBuf,
        #[arg(long)]
        png: PathBuf,
        /// window "x0,y0,x1,y1" in mm
        #[arg(long)]
        view: Option<String>,
        #[arg(long, default_value_t = 20.0)]
        scale: f64,
        /// copper layers to draw (names), default all; the first one is drawn on top
        #[arg(long)]
        layers: Option<String>,
        /// markers "x,y;x,y;..." numbered from --first
        #[arg(long)]
        markers: Option<String>,
        #[arg(long, default_value_t = 1)]
        first: u32,
        /// marker radius in mm
        #[arg(long, default_value_t = 0.8)]
        marker_r: f64,
    },
    /// Route a board
    Route {
        board: PathBuf,
        /// output .kicad_pcb
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// router config (TOML); default: <board>.gridless.toml if present
        #[arg(short, long)]
        config: Option<PathBuf>,
        /// write a PNG preview of the topology/geometry here
        #[arg(long)]
        png: Option<PathBuf>,
        /// preview window "x0,y0,x1,y1" in mm
        #[arg(long)]
        view: Option<String>,
        /// preview scale in px/mm
        #[arg(long, default_value_t = 20.0)]
        scale: f64,
        /// geometry/DRC feedback rounds (default 8)
        #[arg(long)]
        rounds: Option<usize>,
        /// write the result (items to add + report) as JSON, for the KiCad plugins
        #[arg(long)]
        json_out: Option<PathBuf>,
        /// KiCad project file with net classes (default: <board>.kicad_pro)
        #[arg(long)]
        project: Option<PathBuf>,
        /// keep existing tracks and vias and only route what is missing
        #[arg(long)]
        keep_existing: bool,
        /// after routing, refill zones with kicad-cli and add plane vias for pads the
        /// copper pours failed to connect (then route again)
        #[arg(long)]
        kicad_check: bool,
    },
    /// Join the islands of a net's copper pours with vias
    Stitch {
        board: PathBuf,
        /// output .kicad_pcb (default: <board>_stitched.kicad_pcb)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// router config (TOML); default: <board>.gridless.toml if present
        #[arg(short, long)]
        config: Option<PathBuf>,
        /// KiCad project file with net classes (default: <board>.kicad_pro)
        #[arg(long)]
        project: Option<PathBuf>,
        /// only this net (default: every net with a pour)
        #[arg(long)]
        net: Option<String>,
        /// do not refill the pours with kicad-cli first; use the fills saved in the board
        #[arg(long)]
        no_refill: bool,
        #[arg(long, default_value_t = 200)]
        max_vias: usize,
    },
}

fn load_config(board: &Path, cfg: &Option<PathBuf>) -> Result<Config> {
    let path = match cfg {
        Some(p) => Some(p.clone()),
        None => {
            let p = board.with_extension("gridless.toml");
            if p.exists() {
                Some(p)
            } else {
                None
            }
        }
    };
    match path {
        Some(p) => {
            log::info!("config: {}", p.display());
            Config::from_toml(&std::fs::read_to_string(&p).with_context(|| format!("reading {}", p.display()))?)
        }
        None => Ok(Config::default()),
    }
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).format_timestamp(None).init();
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Info { board, json } => {
            let l = gridless::kicad::load(&board)?;
            let b = &l.board;
            println!("layers: {:?}", b.layers.iter().map(|l| format!("{} ({})", l.name, l.alias)).collect::<Vec<_>>());
            println!("nets {}  footprints {}  pads {}  tracks {}  vias {}  zones {}", b.nets.len(), b.footprints.len(), b.pads.len(), b.tracks.len(), b.vias.len(), b.zones.len());
            println!("outline loops: {:?}", b.outline.iter().map(|o| o.len()).collect::<Vec<_>>());
            println!("classes: {:?}", b.classes.iter().map(|c| &c.name).collect::<Vec<_>>());
            println!("rules: {:?}", b.rules);
            if let Some(j) = json {
                std::fs::write(j, serde_json::to_string_pretty(b)?)?;
            }
        }
        Cmd::Render { board, png, view, scale, layers, markers, first, marker_r } => {
            let l = gridless::kicad::load(&board)?;
            let b = &l.board;
            let bb = match &view {
                Some(v) => {
                    let n: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                    anyhow::ensure!(n.len() == 4, "--view needs x0,y0,x1,y1");
                    gridless::geom::BBox { min: p(n[0], n[1]), max: p(n[2], n[3]) }
                }
                None => b.bbox().grow(1.0),
            };
            let mut order: Vec<usize> = match &layers {
                Some(s) => s.split(',').filter_map(|n| b.layer_id(n.trim())).collect(),
                None => (0..b.layers.len()).collect(),
            };
            order.reverse(); // the first named layer is drawn last (on top)
            let mut cv = Canvas::new(bb.min, bb.max, scale);
            for poly in &b.outline {
                let mut pl = poly.clone();
                pl.push(poly[0]);
                cv.polyline(&pl, 0.1, (220, 220, 0), 255);
            }
            for &li in &order {
                let col = LAYER_COLORS[li % LAYER_COLORS.len()];
                let dim = (col.0 / 2, col.1 / 2, col.2 / 2);
                for z in &b.zones {
                    for (zl, pts) in &z.fills {
                        if *zl == li {
                            cv.polygon_eo(pts, dim, 160);
                        }
                    }
                }
                for pd in b.pads.iter().filter(|pd| pd.layers.contains(&li)) {
                    cv.shape(&pd.shape, (175, 175, 175), 235);
                }
                for cs in b.copper.iter().filter(|c| c.layer == li) {
                    for pc in &cs.pieces {
                        cv.shape(pc, col, 200);
                    }
                }
                for t in b.tracks.iter().filter(|t| t.layer == li) {
                    let pts = match t.geom {
                        gridless::board::TrackGeom::Seg { a, b } => vec![a, b],
                        gridless::board::TrackGeom::Arc { s, m, e } => gridless::geom::Arc::from_3p(s, m, e).map(|a| a.polyline(0.002)).unwrap_or(vec![s, e]),
                    };
                    cv.polyline(&pts, t.width, col, 235);
                }
            }
            for v in &b.vias {
                cv.circle(v.pos, v.dia / 2.0, (230, 230, 230), 255);
                cv.circle(v.pos, v.drill / 2.0, (16, 16, 20), 255);
            }
            for pd in b.pads.iter().filter(|pd| pd.drill.is_some()) {
                cv.circle(pd.pos, pd.drill.unwrap() / 2.0, (16, 16, 20), 255);
            }
            if let Some(m) = &markers {
                for (i, part) in m.split(';').filter(|s| !s.trim().is_empty()).enumerate() {
                    let n: Vec<f64> = part.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                    if n.len() == 2 {
                        let q = p(n[0], n[1]);
                        cv.ring(q, marker_r, marker_r * 0.15, (255, 0, 255));
                        cv.number(first + i as u32, q + p(marker_r * 0.9, -marker_r * 2.1), marker_r * 1.2, (255, 255, 255));
                    }
                }
            }
            cv.save(&png)?;
        }
        Cmd::Stitch { board, output, config, project, net, no_refill, max_vias } => {
            let out = output.unwrap_or_else(|| {
                let stem = board.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
                board.with_file_name(format!("{stem}_stitched.kicad_pcb"))
            });
            let mut analysis_path = board.clone();
            let mut temp_dir: Option<PathBuf> = None;
            let mut orphan_spots: Vec<(String, gridless::geom::P)> = Vec::new();
            if !no_refill {
                match gridless::pipeline::find_kicad_cli() {
                    None => log::warn!("stitch: kicad-cli not found (set KICAD_CLI); using the fills saved in the board"),
                    Some(cli) => {
                        let dir = std::env::temp_dir().join(format!("gridless_stitch_{}", std::process::id()));
                        std::fs::create_dir_all(&dir)?;
                        let pcb = dir.join("stitch.kicad_pcb");
                        std::fs::copy(&board, &pcb)?;
                        let pro = project.clone().unwrap_or_else(|| board.with_extension("kicad_pro"));
                        if pro.exists() {
                            let _ = std::fs::copy(&pro, dir.join("stitch.kicad_pro"));
                        }
                        let report = dir.join("drc.json");
                        let st = std::process::Command::new(cli)
                            .args(["pcb", "drc", "--format", "json", "--units", "mm", "--refill-zones", "--save-board", "-o"])
                            .arg(&report)
                            .arg(&pcb)
                            .output()?;
                        if pcb.is_file() {
                            analysis_path = pcb.clone();
                            temp_dir = Some(dir);
                            // which pads KiCad itself calls unconnected on the refilled board
                            orphan_spots = gridless::pipeline::kicad_unconnected_spots(&report);
                            if !orphan_spots.is_empty() {
                                log::info!("stitch: KiCad reports {} unconnected spot(s)", orphan_spots.len());
                            }
                        } else {
                            log::warn!("stitch: refill failed: {}", String::from_utf8_lossy(&st.stderr));
                        }
                    }
                }
            }
            let l = gridless::kicad::load_with_project(&analysis_path, project.as_deref())?;
            let cfg = load_config(&board, &config)?;
            let rules = Rules::resolve(&l.board, &cfg);
            let orphans: Vec<(gridless::board::NetId, gridless::geom::P)> = orphan_spots
                .iter()
                .filter_map(|(n, q)| l.board.nets.iter().position(|x| &x.name == n).map(|i| (i, *q)))
                .collect();
            let (vias, stubs, rep) = gridless::stitch::stitch(&l.board, &rules, net.as_deref(), max_vias, &orphans);
            let mut items: Vec<gridless::kicad::OutItem> = vias
                .iter()
                .map(|(q, n)| gridless::kicad::OutItem::Via {
                    net: l.board.nets[*n].name.clone(),
                    pos: *q,
                    dia: rules.via_dia,
                    drill: rules.via_drill,
                })
                .collect();
            // pad-to-pour bridges drawn as tracks where the thermal barrier blocks the pour
            items.extend(stubs.iter().map(|s| gridless::kicad::OutItem::Seg {
                layer: l.board.layers[s.layer].name.clone(),
                net: l.board.nets[s.net].name.clone(),
                width: s.width,
                a: s.a,
                b: s.b,
            }));
            let src = std::fs::read_to_string(&board)?;
            gridless::kicad::write_routed(&src, &out, &items, false)?;
            log::info!(
                "stitch: {} islands, {} net(s), {} via(s) and {} track(s) added, {} unresolved -> {}",
                rep.islands,
                rep.nets,
                rep.added,
                rep.tracks,
                rep.unresolved,
                out.display()
            );
            if let Some(d) = temp_dir {
                let _ = std::fs::remove_dir_all(d);
            }
        }
        Cmd::Route { board, output, config, png, view, scale, rounds, json_out, project, keep_existing, kicad_check } => {
            let t0 = Instant::now();
            let l = gridless::kicad::load_with_project(&board, project.as_deref())?;
            let mut cfg = load_config(&board, &config)?;
            if keep_existing {
                cfg.keep_existing = Some(true);
            }
            let mut rules = Rules::resolve(&l.board, &cfg);
            if kicad_check {
                // first pass, then let KiCad's zone filler tell which pads the pours miss
                match gridless::pipeline::find_kicad_cli() {
                    None => log::warn!("--kicad-check: kicad-cli not found (set KICAD_CLI)"),
                    Some(cli) => {
                        let pb0 = Problem::build(&l.board, &rules);
                        let o0 = gridless::pipeline::route(&l.board, &rules, &pb0, rounds.unwrap_or(8), &mut |m| log::info!("[pass 1] {m}"));
                        let tmp = std::env::temp_dir().join(format!("gridless_pass1_{}.kicad_pcb", std::process::id()));
                        let items = gridless::output::items(&l.board, &o0.geo, &rules);
                        gridless::kicad::write_routed(&l.source, &tmp, &items, !rules.keep_existing)?;
                        let pro = project.clone().unwrap_or_else(|| board.with_extension("kicad_pro"));
                        let pads = gridless::pipeline::kicad_unconnected_plane_pads(&cli, &tmp, Some(&pro), &l.board, &rules)?;
                        let _ = std::fs::remove_file(&tmp);
                        log::info!("KiCad zone check: pads needing a plane via: {:?}", pads);
                        rules.force_fanout.extend(pads);
                    }
                }
            }
            let pb = Problem::build(&l.board, &rules);
            log::info!(
                "problem: {} obstacles, {} via sites, {} connections, routing layers {:?}, plane nets {:?}, diff pairs {}",
                pb.obstacles.len(),
                pb.sites.len(),
                pb.conns.len(),
                pb.route_layers.iter().map(|&i| &l.board.layers[i].name).collect::<Vec<_>>(),
                pb.plane_nets.iter().map(|&n| &l.board.nets[n].name).collect::<Vec<_>>(),
                rules.diff_pairs.len()
            );
            let rounds = rounds.unwrap_or(8);
            let out = gridless::pipeline::route(&l.board, &rules, &pb, rounds, &mut |m| log::info!("{m} ({:.2}s)", t0.elapsed().as_secs_f64()));
            let geo = out.geo;
            for w in &geo.warnings {
                log::warn!("{w}");
            }
            log::info!(
                "done in {:.2}s after {} rounds: {} tracks, {} vias, length {:.1} mm",
                t0.elapsed().as_secs_f64(),
                out.rounds,
                geo.tracks.len(),
                geo.vias.len(),
                out.length
            );
            let rep = out.drc;
            {
                // direction statistics per layer
                let mut st: std::collections::BTreeMap<String, [f64; 4]> = std::collections::BTreeMap::new();
                for t in &geo.tracks {
                    let e = st.entry(l.board.layers[t.layer].name.clone()).or_insert([0.0; 4]);
                    for pr in &t.prims {
                        match pr {
                            gridless::geometry::Prim::Seg(a, b) => {
                                let d = *b - *a;
                                let len = d.len();
                                if d.x.abs() > 3.0 * d.y.abs() {
                                    e[0] += len;
                                } else if d.y.abs() > 3.0 * d.x.abs() {
                                    e[1] += len;
                                } else {
                                    e[2] += len;
                                }
                            }
                            gridless::geometry::Prim::Arc(a) => e[3] += a.len(),
                        }
                    }
                }
                for (k, v) in st {
                    log::info!("  {k}: horizontal {:.0} mm, vertical {:.0} mm, diagonal {:.0} mm, arcs {:.0} mm", v[0], v[1], v[2], v[3]);
                }
            }
            log::info!("DRC: {} violations, unconnected nets: {:?}", rep.violations.len(), rep.unconnected);
            {
                let mut hist: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
                for v in &rep.violations {
                    let ty = |d: &str| d.split_whitespace().next().unwrap_or("").to_string();
                    let band = if v.gap < 0.0 { "<0" } else if v.gap < 0.1 { "0-0.1" } else if v.gap < 0.15 { "0.1-0.15" } else { "0.15+" };
                    *hist.entry(format!("{} {}-{} {}", v.kind, ty(&v.a), ty(&v.b), band)).or_default() += 1;
                }
                for (k, n) in hist {
                    log::info!("  [{n:4}] {k}");
                }
            }
            for v in rep.violations.iter().rev().take(15) {
                log::info!("  {} gap {:.3}/{:.3} at ({:.2},{:.2}): {} <-> {}", v.kind, v.gap, v.need, v.at.x, v.at.y, v.a, v.b);
                if std::env::var("GRIDLESS_DEBUG_DRC").is_ok() {
                    if let Some((ti, pi)) = v.track {
                        log::info!("     prim {:?}", geo.tracks[ti].prims[pi]);
                    }
                }
            }
            if let Some(jp) = &json_out {
                let res = gridless::output::JsonResult {
                    items: gridless::output::items(&l.board, &geo, &rules),
                    vias: geo.vias.len(),
                    length_mm: out.length,
                    violations: rep.violations.clone(),
                    unconnected: rep.unconnected.clone(),
                    tuning: out.tuning.clone(),
                    unrouted: out.stats.unrouted,
                    seconds: t0.elapsed().as_secs_f64(),
                };
                std::fs::write(jp, serde_json::to_string_pretty(&res)?)?;
                log::info!("JSON result written to {}", jp.display());
            }
            if let Some(out) = &output {
                let items = gridless::output::items(&l.board, &geo, &rules);
                gridless::kicad::write_routed(&l.source, out, &items, !rules.keep_existing)?;
                log::info!("written {} items to {}", items.len(), out.display());
            }
            if let Some(png) = png {
                let bb = match &view {
                    Some(v) => {
                        let n: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                        anyhow::ensure!(n.len() == 4, "--view needs x0,y0,x1,y1");
                        gridless::geom::BBox { min: p(n[0], n[1]), max: p(n[2], n[3]) }
                    }
                    None => pb.bbox.grow(1.0),
                };
                let mut cv = Canvas::new(bb.min, bb.max, scale);
                for poly in &pb.outline {
                    let mut pl = poly.clone();
                    pl.push(poly[0]);
                    cv.polyline(&pl, 0.1, (220, 220, 0), 255);
                }
                for o in &pb.obstacles {
                    let c = if o.on(0) { (150, 60, 60) } else if o.on(l.board.layers.len() - 1) { (60, 60, 150) } else { (90, 90, 90) };
                    cv.shape(&o.shape, c, 200);
                }
                for t in &geo.tracks {
                    let col = LAYER_COLORS[t.layer % LAYER_COLORS.len()];
                    for (i, pr) in t.prims.iter().enumerate() {
                        cv.polyline(&pr.polyline(0.005), t.pw.get(i).copied().unwrap_or(t.width), col, 190);
                    }
                }
                for &(pos, _) in &geo.vias {
                    cv.circle(pos, rules.via_dia / 2.0, (230, 230, 230), 255);
                    cv.circle(pos, rules.via_drill / 2.0, (16, 16, 20), 255);
                }
                for v in &rep.violations {
                    cv.circle(v.at, 0.15, (255, 0, 255), 255);
                }
                let _ = p(0.0, 0.0);
                cv.save(&png)?;
            }
            let _ = output;
        }
    }
    Ok(())
}
