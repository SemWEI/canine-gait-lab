//! Application state and layout: controls on the left, the dog canvas in the centre, the
//! synchronised analysis panels at the bottom.

use crate::io::csv_export::{ExportOptions, write_csv};
use crate::io::csv_import::{Imported, parse_csv, recognised_columns};
use crate::model::breed::{BreedSpec, load_breeds};
use crate::model::gait::{ALL_GAITS, Gait};
use crate::model::kinematics::{FORE_FLEXOR_CRANIAL, HIND_FLEXOR_CRANIAL, distal_dir_from_angle};
use crate::model::math::V2;
use crate::model::sim::{Frame, SimParams, Simulation};
use crate::model::{LIMBS, MODEL_VERSION};
use crate::palette::{Palette, palette};
use crate::plots::{self, JointMode, PlotData, Series};
use crate::render::dog::{Cam, DrawOptions, draw_dog, draw_ground, draw_obstacle, draw_trails};
use eframe::egui::{self, Align2, FontId, RichText, Sense, Stroke, Ui, pos2, vec2};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Joints,
    Footfall,
    Com,
    Grf,
    PawPaths,
    Metrics,
    Data,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CamMode {
    Follow,
    Fixed,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Layout {
    Lanes,
    Overlay,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Sync {
    RealTime,
    Phase,
}

struct Prepared {
    sim: Simulation,
    frames: Vec<Frame>,
    xs: Vec<f64>,
}

impl Prepared {
    fn new(sim: Simulation) -> Self {
        let n = if sim.is_periodic() {
            400
        } else {
            ((sim.duration * 200.0) as usize).clamp(200, 1500)
        };
        let frames = sim.series(n);
        let xs = frames
            .iter()
            .map(|f| {
                if sim.is_periodic() {
                    f.phase * 100.0
                } else {
                    f.t
                }
            })
            .collect();
        Self { sim, frames, xs }
    }
}

pub struct GaitLabApp {
    breeds: Vec<BreedSpec>,
    breeds_source: String,
    breed_idx: usize,
    height_cm: f64,
    gait: Gait,
    froude: f64,
    obstacle_cm: f64,

    cmp_enabled: bool,
    cmp_breed_idx: usize,
    cmp_height_cm: f64,
    layout: Layout,
    sync: Sync,

    main: Prepared,
    cmp: Option<Prepared>,
    dirty: bool,

    t: f64,
    playing: bool,
    rate: f64,
    opts: DrawOptions,
    cam_mode: CamMode,
    zoom: f32,

    tab: Tab,
    joint_mode: JointMode,

    imported: Option<Imported>,
    import_error: Option<String>,
    drive_from_import: bool,
    export_hz: f64,
    export_cycles: usize,
    status: Option<(String, bool)>,
}

/// Initial GUI state from the command line (all optional).
#[derive(Default)]
pub struct StartOptions {
    pub breed: Option<String>,
    pub compare: Option<String>,
    pub gait: Option<Gait>,
    pub froude: Option<f64>,
    pub height_cm: Option<f64>,
    pub obstacle_cm: Option<f64>,
    pub tab: Option<String>,
    pub time_s: Option<f64>,
}

fn find_breed(breeds: &[BreedSpec], prefix: &str) -> usize {
    let p = prefix.to_lowercase();
    breeds
        .iter()
        .position(|b| b.name.to_lowercase().starts_with(&p))
        .unwrap_or(0)
}

impl GaitLabApp {
    pub fn new(start: StartOptions) -> Self {
        let (breeds, breeds_source) = load_breeds();
        let breed_idx = find_breed(&breeds, start.breed.as_deref().unwrap_or("Labrador"));
        let cmp_breed_idx = find_breed(&breeds, start.compare.as_deref().unwrap_or("Chihuahua"));
        let height_cm = start
            .height_cm
            .unwrap_or(breeds[breed_idx].withers_height_m * 100.0);
        let cmp_height_cm = breeds[cmp_breed_idx].withers_height_m * 100.0;
        let gait = start.gait.unwrap_or(Gait::Trot);
        let froude = start.froude.unwrap_or(gait.spec().froude.2);
        let main = Prepared::new(Simulation::new(SimParams {
            breed: breeds[breed_idx].clone(),
            withers_height: height_cm / 100.0,
            gait,
            froude,
            obstacle: start.obstacle_cm.unwrap_or(30.0) / 100.0,
        }));
        let mut app = Self {
            breeds,
            breeds_source,
            breed_idx,
            height_cm,
            gait,
            froude,
            obstacle_cm: start.obstacle_cm.unwrap_or(30.0),
            cmp_enabled: start.compare.is_some(),
            cmp_breed_idx,
            cmp_height_cm,
            layout: Layout::Lanes,
            sync: Sync::RealTime,
            main,
            cmp: None,
            dirty: false,
            t: 0.0,
            playing: true,
            rate: 0.25,
            opts: DrawOptions::default(),
            cam_mode: CamMode::Follow,
            zoom: 1.0,
            tab: Tab::Joints,
            joint_mode: JointMode::Angle,
            imported: None,
            import_error: None,
            drive_from_import: false,
            export_hz: 200.0,
            export_cycles: 1,
            status: None,
        };
        if app.cmp_enabled {
            app.dirty = true;
        }
        if let Some(tab) = start.tab.as_deref() {
            app.tab = match tab {
                "footfall" => Tab::Footfall,
                "com" => Tab::Com,
                "grf" => Tab::Grf,
                "paw" => Tab::PawPaths,
                "metrics" => Tab::Metrics,
                "data" => Tab::Data,
                _ => Tab::Joints,
            };
        }
        if let Some(t) = start.time_s {
            app.t = t;
            app.playing = false;
        }
        app
    }

    fn params(&self, breed_idx: usize, height_cm: f64) -> SimParams {
        SimParams {
            breed: self.breeds[breed_idx].clone(),
            withers_height: height_cm / 100.0,
            gait: self.gait,
            froude: self.froude,
            obstacle: self.obstacle_cm / 100.0,
        }
    }

    fn rebuild(&mut self) {
        let phase = self.main.sim.phase_of(self.t);
        self.main = Prepared::new(Simulation::new(self.params(self.breed_idx, self.height_cm)));
        self.cmp = if self.cmp_enabled {
            Some(Prepared::new(Simulation::new(
                self.params(self.cmp_breed_idx, self.cmp_height_cm),
            )))
        } else {
            None
        };
        // keep the same place in the cycle
        self.t = if self.main.sim.is_periodic() {
            phase * self.main.sim.duration
        } else {
            self.t.min(self.main.sim.duration)
        };
        self.dirty = false;
    }

    fn cmp_time(&self) -> f64 {
        let Some(c) = &self.cmp else { return 0.0 };
        match self.sync {
            Sync::RealTime => {
                if c.sim.is_periodic() {
                    self.t
                } else {
                    self.t.min(c.sim.duration)
                }
            }
            Sync::Phase => {
                if self.main.sim.is_periodic() {
                    self.t / self.main.sim.duration * c.sim.duration
                } else {
                    self.main.sim.phase_of(self.t) * c.sim.duration
                }
            }
        }
    }

    fn cursor_x(&self) -> f64 {
        if self.main.sim.is_periodic() {
            self.main.sim.phase_of(self.t) * 100.0
        } else {
            self.t
        }
    }

    fn scrub_to(&mut self, x: f64) {
        let s = &self.main.sim;
        if s.is_periodic() {
            let base = (self.t / s.duration).floor() * s.duration;
            self.t = base + x / 100.0 * s.duration;
        } else {
            self.t = x.clamp(0.0, s.duration);
        }
    }

    fn current_frame(&self) -> Frame {
        let f = self.main.sim.sample(self.t);
        match (&self.imported, self.drive_from_import) {
            (Some(imp), true) if imp.has_all_angles() => frame_from_import(
                &self.main.sim,
                f,
                imp,
                self.main.sim.phase_of(self.t) * 100.0,
            ),
            _ => f,
        }
    }

    fn import_dialog(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("CSV", &["csv", "txt"])
            .set_title("Import kinematics CSV")
            .pick_file()
        else {
            return;
        };
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_default();
        match std::fs::read_to_string(&path) {
            Ok(text) => match parse_csv(&text, &name) {
                Ok(imp) => {
                    self.status =
                        Some((format!("Imported {name}: {} rows.", imp.pct.len()), false));
                    self.drive_from_import = false;
                    self.imported = Some(imp);
                    self.import_error = None;
                }
                Err(e) => {
                    self.import_error = Some(format!("{name}: {e}"));
                }
            },
            Err(e) => self.import_error = Some(format!("Could not read {name}: {e}")),
        }
    }

    fn export_dialog(&mut self) {
        let s = &self.main.sim;
        let default = format!(
            "{}_{}_{}cm_Fr{:.2}.csv",
            s.params.breed.name.replace(' ', "_").to_lowercase(),
            s.params.gait.csv_name(),
            self.height_cm.round(),
            s.froude()
        );
        let Some(path) = rfd::FileDialog::new()
            .add_filter("CSV", &["csv"])
            .set_file_name(&default)
            .set_title("Export model series")
            .save_file()
        else {
            return;
        };
        let opts = ExportOptions {
            hz: self.export_hz,
            cycles: self.export_cycles,
            demo_noise_deg: 0.0,
        };
        let res = std::fs::File::create(&path)
            .and_then(|f| write_csv(s, opts, std::io::BufWriter::new(f)));
        self.status = Some(match res {
            Ok(n) => (format!("Exported {n} rows to {}", path.display()), false),
            Err(e) => (format!("Export failed: {e}"), true),
        });
    }
}

/// Replace modelled joint angles with imported ones and rebuild the limbs by forward
/// kinematics from the modelled girdles. Paws may slip or leave the ground if the measured
/// angles disagree with the model trunk motion; that is shown as is.
fn frame_from_import(sim: &Simulation, mut f: Frame, imp: &Imported, pct: f64) -> Frame {
    let sk = &sim.sk;
    if let Some(y) = imp.value_at("com_y_m", pct) {
        let dy = y - f.com.y;
        f.com.y = y;
        f.withers.y += dy;
        f.hip.y += dy;
        for l in f.limbs.iter_mut() {
            l.pts[0].y += dy;
            l.pts[1].y += dy;
        }
    }
    for (i, limb) in LIMBS.iter().enumerate() {
        let l = &mut f.limbs[i];
        let ang: Vec<f64> = (0..3)
            .map(|j| {
                imp.value_at(&limb.angle_column(j), pct)
                    .unwrap_or(l.angles[j])
            })
            .collect();
        let (lens, flex) = if limb.is_fore() {
            ([sk.humerus, sk.radius, sk.metacarpus], FORE_FLEXOR_CRANIAL)
        } else {
            ([sk.femur, sk.tibia, sk.metatarsus], HIND_FLEXOR_CRANIAL)
        };
        let mut pts = l.pts;
        for k in 0..3 {
            let prox_dir: V2 = (pts[k] - pts[k + 1]).norm();
            pts[k + 2] = pts[k + 1] + distal_dir_from_angle(prox_dir, ang[k], flex[k]) * lens[k];
        }
        l.pts = pts;
        l.angles = [ang[0], ang[1], ang[2]];
        if let Some(c) = imp.value_at(&format!("{}_contact", limb.label()), pct) {
            l.contact = c > 0.5;
        }
        l.clamped = false;
    }
    f
}

fn fmt_speed(v: f64) -> String {
    format!("{v:.2} m/s ({:.1} km/h)", v * 3.6)
}

impl eframe::App for GaitLabApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let pal = palette(ui.visuals().dark_mode);
        if self.dirty {
            self.rebuild();
        }
        if self.playing {
            let dt = ctx.input(|i| i.stable_dt).min(0.1) as f64;
            self.t += dt * self.rate;
            if !self.main.sim.is_periodic() && self.t > self.main.sim.duration {
                self.t = 0.0;
            }
            ctx.request_repaint();
        }

        self.top_bar(ui, &pal);
        egui::Panel::left("controls")
            .resizable(true)
            .default_size(290.0)
            .min_size(250.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.controls(ui, &pal));
            });
        egui::Panel::bottom("analysis")
            .resizable(true)
            .default_size(470.0)
            .min_size(260.0)
            .show(ui, |ui| {
                self.analysis(ui, &pal);
            });
        egui::CentralPanel::default().show(ui, |ui| self.canvas(ui, &pal));
    }
}

impl GaitLabApp {
    fn top_bar(&mut self, ui: &mut Ui, pal: &Palette) {
        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Canine Gait Lab").strong().size(17.0));
                ui.separator();
                let (txt, fill) = match (&self.imported, self.drive_from_import) {
                    (Some(imp), true) => (
                        format!("Animation driven by imported data: {}", imp.file_name),
                        pal.badge_import,
                    ),
                    (Some(imp), false) => (
                        format!("Modelled kinematics, overlaid with {}", imp.file_name),
                        pal.badge_import,
                    ),
                    _ => (
                        "Modelled kinematics, not measured data".to_string(),
                        pal.badge_model,
                    ),
                };
                egui::Frame::new()
                    .fill(fill)
                    .corner_radius(4.0)
                    .inner_margin(vec2(8.0, 3.0))
                    .show(ui, |ui| {
                        ui.label(RichText::new(txt).color(pal.text).size(12.0));
                    });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    egui::widgets::global_theme_preference_switch(ui);
                    ui.label(
                        RichText::new(format!("model v{MODEL_VERSION}"))
                            .color(pal.text_muted)
                            .size(11.0),
                    );
                });
            });
        });
    }

    fn controls(&mut self, ui: &mut Ui, pal: &Palette) {
        let mut changed = false;
        ui.add_space(4.0);
        ui.heading("Subject");
        let prev = self.breed_idx;
        egui::ComboBox::from_id_salt("breed")
            .width(220.0)
            .selected_text(&self.breeds[self.breed_idx].name)
            .show_ui(ui, |ui| {
                for (i, b) in self.breeds.iter().enumerate() {
                    ui.selectable_value(&mut self.breed_idx, i, &b.name);
                }
            });
        if prev != self.breed_idx {
            self.height_cm = self.breeds[self.breed_idx].withers_height_m * 100.0;
            changed = true;
        }
        let refh = self.breeds[self.breed_idx].withers_height_m * 100.0;
        ui.horizontal(|ui| {
            ui.label("Withers height");
            changed |= ui
                .add(
                    egui::Slider::new(&mut self.height_cm, (refh * 0.5)..=(refh * 1.5))
                        .suffix(" cm")
                        .fixed_decimals(1),
                )
                .changed();
        });
        let sk = &self.main.sim.sk;
        ui.label(
            RichText::new(format!(
                "Mass {:.1} kg (scaled ∝ height³) · hip height {:.1} cm · fore/hind load {:.0}/{:.0} %",
                sk.mass,
                sk.hip_h * 100.0,
                sk.fore_frac * 100.0,
                (1.0 - sk.fore_frac) * 100.0
            ))
            .color(pal.text_muted)
            .size(11.5),
        );
        let notes = &self.breeds[self.breed_idx].notes;
        if !notes.is_empty() {
            ui.label(
                RichText::new(notes)
                    .color(pal.text_muted)
                    .size(11.5)
                    .italics(),
            );
        }

        ui.separator();
        ui.heading("Gait");
        let prev_gait = self.gait;
        egui::ComboBox::from_id_salt("gait")
            .width(220.0)
            .selected_text(self.gait.spec().name)
            .show_ui(ui, |ui| {
                for g in ALL_GAITS {
                    ui.selectable_value(&mut self.gait, g, g.spec().name);
                }
            });
        if prev_gait != self.gait {
            self.froude = self.gait.spec().froude.2;
            self.t = 0.0;
            changed = true;
        }
        let spec = self.gait.spec();
        ui.label(
            RichText::new(spec.description)
                .color(pal.text_muted)
                .size(11.5),
        );
        let (lo, hi, _) = spec.froude;
        if hi > lo {
            ui.horizontal(|ui| {
                ui.label("Froude no.").on_hover_text("Fr = v² / (g · hip height). Equal Fr means dynamically similar speed across dog sizes.");
                changed |= ui.add(egui::Slider::new(&mut self.froude, lo..=hi).logarithmic(true).fixed_decimals(2)).changed();
            });
            let v = (self.froude * crate::model::G * self.main.sim.sk.hip_h).sqrt();
            ui.label(RichText::new(format!("Speed {}", fmt_speed(v))).size(12.5));
        }
        if self.gait.is_jump() {
            let hw = self.height_cm;
            ui.horizontal(|ui| {
                ui.label("Obstacle");
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.obstacle_cm, 0.0..=(hw * 1.6))
                            .suffix(" cm")
                            .fixed_decimals(0),
                    )
                    .changed();
            });
        }
        if self.main.sim.duty_reach_limited {
            ui.label(
                RichText::new("Duty factor shortened to keep stance within limb reach.")
                    .color(pal.text_muted)
                    .size(11.5),
            );
        }

        ui.separator();
        ui.heading("Playback");
        ui.horizontal(|ui| {
            if ui
                .button(if self.playing {
                    "⏸ Pause"
                } else {
                    "▶ Play"
                })
                .clicked()
            {
                self.playing = !self.playing;
            }
            if ui
                .button("◀ 5 ms")
                .on_hover_text("Step back one 200 Hz frame")
                .clicked()
            {
                self.playing = false;
                self.t = (self.t - 0.005).max(0.0);
            }
            if ui
                .button("5 ms ▶")
                .on_hover_text("Step forward one 200 Hz frame")
                .clicked()
            {
                self.playing = false;
                self.t += 0.005;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Speed");
            ui.add(
                egui::Slider::new(&mut self.rate, 0.02..=2.0)
                    .logarithmic(true)
                    .suffix("×")
                    .fixed_decimals(2),
            );
        });
        ui.horizontal(|ui| {
            for r in [0.05, 0.1, 0.25, 1.0] {
                if ui
                    .selectable_label((self.rate - r).abs() < 1e-6, format!("{r}×"))
                    .clicked()
                {
                    self.rate = r;
                }
            }
        });
        let periodic = self.main.sim.is_periodic();
        let mut x = self.cursor_x();
        let range = if periodic {
            0.0..=100.0
        } else {
            0.0..=self.main.sim.duration
        };
        ui.horizontal(|ui| {
            ui.label(if periodic { "Cycle" } else { "Time" });
            let suffix = if periodic { " %" } else { " s" };
            if ui
                .add(
                    egui::Slider::new(&mut x, range)
                        .suffix(suffix)
                        .fixed_decimals(if periodic { 1 } else { 3 }),
                )
                .changed()
            {
                self.playing = false;
                self.scrub_to(x);
            }
        });
        ui.label(
            RichText::new(format!("t = {:.3} s", self.t))
                .color(pal.text_muted)
                .monospace()
                .size(11.5),
        );

        ui.separator();
        ui.heading("Display");
        ui.checkbox(&mut self.opts.silhouette, "Body silhouette");
        ui.checkbox(&mut self.opts.skeleton, "Skeleton and joints");
        ui.checkbox(
            &mut self.opts.angle_labels,
            "Joint angle labels (near limbs)",
        );
        ui.checkbox(&mut self.opts.trails, "Paw, carpus/hock and COM trails");
        ui.checkbox(&mut self.opts.com_marker, "Centre of mass");
        ui.checkbox(&mut self.opts.contacts, "Ground contact bars");
        ui.checkbox(
            &mut self.opts.grf_vectors,
            "Vertical GRF arrows (0.5 × withers height = 1 BW)",
        );
        ui.horizontal(|ui| {
            ui.label("Camera");
            ui.selectable_value(&mut self.cam_mode, CamMode::Follow, "Follow dog");
            ui.selectable_value(&mut self.cam_mode, CamMode::Fixed, "Fixed");
        });
        ui.horizontal(|ui| {
            ui.label("Zoom");
            ui.add(
                egui::Slider::new(&mut self.zoom, 0.3..=4.0)
                    .logarithmic(true)
                    .fixed_decimals(2),
            );
        });
        ui.label(
            RichText::new("Scroll on the canvas to zoom; double-click to reset.")
                .color(pal.text_muted)
                .size(11.0),
        );

        ui.separator();
        ui.heading("Compare");
        changed |= ui
            .checkbox(&mut self.cmp_enabled, "Show a second dog")
            .changed();
        ui.add_enabled_ui(self.cmp_enabled, |ui| {
            let prev = self.cmp_breed_idx;
            egui::ComboBox::from_id_salt("cmp-breed")
                .width(220.0)
                .selected_text(&self.breeds[self.cmp_breed_idx].name)
                .show_ui(ui, |ui| {
                    for (i, b) in self.breeds.iter().enumerate() {
                        ui.selectable_value(&mut self.cmp_breed_idx, i, &b.name);
                    }
                });
            if prev != self.cmp_breed_idx {
                self.cmp_height_cm = self.breeds[self.cmp_breed_idx].withers_height_m * 100.0;
                changed = true;
            }
            let refc = self.breeds[self.cmp_breed_idx].withers_height_m * 100.0;
            ui.horizontal(|ui| {
                ui.label("Withers height");
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.cmp_height_cm, (refc * 0.5)..=(refc * 1.5))
                            .suffix(" cm")
                            .fixed_decimals(1),
                    )
                    .changed();
            });
            ui.label(
                RichText::new("Same gait and Froude number, so speeds are dynamically similar.")
                    .color(pal.text_muted)
                    .size(11.0),
            );
            ui.horizontal(|ui| {
                ui.label("Layout");
                ui.selectable_value(&mut self.layout, Layout::Lanes, "Two lanes");
                ui.selectable_value(&mut self.layout, Layout::Overlay, "Overlay");
            });
            ui.horizontal(|ui| {
                ui.label("Clock");
                ui.selectable_value(&mut self.sync, Sync::RealTime, "Real time");
                ui.selectable_value(&mut self.sync, Sync::Phase, "Gait phase");
            });
        });

        ui.separator();
        ui.heading("Data");
        ui.horizontal(|ui| {
            if ui.button("Import CSV…").clicked() {
                self.import_dialog();
                self.tab = Tab::Data;
            }
            if ui.button("Export CSV…").clicked() {
                self.export_dialog();
            }
        });
        if let Some((msg, err)) = &self.status {
            ui.label(
                RichText::new(msg)
                    .color(if *err { pal.warn } else { pal.text_muted })
                    .size(11.5),
            );
        }
        ui.label(
            RichText::new(format!("Breed presets: {}", self.breeds_source))
                .color(pal.text_muted)
                .size(11.0),
        );

        if changed {
            self.dirty = true;
        }
    }

    fn canvas(&mut self, ui: &mut Ui, pal: &Palette) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.allocate_rect(rect, Sense::click_and_drag());
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.zoom = (self.zoom * (scroll * 0.0015).exp()).clamp(0.3, 4.0);
            }
        }
        if resp.double_clicked() {
            self.zoom = 1.0;
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, pal.canvas);

        let main_f = self.current_frame();
        let cmp_t = self.cmp_time();
        let cmp_f = self.cmp.as_ref().map(|c| c.sim.sample(cmp_t));

        let lanes: Vec<(egui::Rect, bool)> = match (&self.cmp, self.layout) {
            (Some(_), Layout::Lanes) => {
                let mid = rect.center().y;
                vec![
                    (
                        egui::Rect::from_min_max(rect.min, pos2(rect.max.x, mid)),
                        true,
                    ),
                    (
                        egui::Rect::from_min_max(pos2(rect.min.x, mid), rect.max),
                        false,
                    ),
                ]
            }
            _ => vec![(rect, true)],
        };

        // one scale for every dog on screen, so sizes are directly comparable
        let sims: Vec<&Simulation> = std::iter::once(&self.main.sim)
            .chain(self.cmp.as_ref().map(|c| &c.sim))
            .collect();
        let hmax = sims.iter().map(|s| s.sk.withers_height).fold(0.0, f64::max);
        let tall = sims
            .iter()
            .map(|s| {
                s.jump().map_or(1.45 * s.sk.withers_height, |j| {
                    (j.y_apex + 0.6 * s.sk.withers_height).max(1.45 * s.sk.withers_height)
                })
            })
            .fold(0.0, f64::max);
        let extent = sims
            .iter()
            .map(|s| s.sk.trunk_length + 1.8 * s.sk.withers_height)
            .fold(0.0, f64::max);
        let lane_h = lanes[0].0.height();
        let ppm =
            ((lane_h - 44.0) / tall as f32).min(rect.width() * 0.7 / extent as f32) * self.zoom;

        for (lane, is_main) in &lanes {
            let lane_painter = painter.with_clip_rect(*lane);
            let mut draw_list: Vec<(&Simulation, &Frame, f64, bool)> = Vec::new();
            if *is_main {
                draw_list.push((&self.main.sim, &main_f, self.t, false));
                if self.layout == Layout::Overlay
                    && let (Some(c), Some(f)) = (&self.cmp, &cmp_f)
                {
                    draw_list.insert(0, (&c.sim, f, cmp_t, true));
                }
            } else if let (Some(c), Some(f)) = (&self.cmp, &cmp_f) {
                draw_list.push((&c.sim, f, cmp_t, false));
            }
            let Some(&(lead_sim, lead_f, _, _)) = draw_list.iter().find(|d| !d.3) else {
                continue;
            };
            let view_w = lane.width() as f64 / ppm as f64;
            let cam_x = match self.cam_mode {
                CamMode::Follow => lead_f.com.x,
                CamMode::Fixed => {
                    let span = view_w * 0.8;
                    (lead_f.com.x / span).floor() * span + span * 0.5
                }
            };
            let anchor = pos2(lane.center().x, lane.bottom() - 30.0);
            let cam = Cam {
                origin: V2::new(cam_x, 0.0),
                anchor,
                ppm,
            };
            draw_ground(&lane_painter, *lane, &cam, pal, hmax);
            if let Some(j) = lead_sim.jump()
                && j.obstacle_h > 0.0
            {
                draw_obstacle(
                    &lane_painter,
                    &cam,
                    j.obstacle_x,
                    j.obstacle_h,
                    lead_sim.sk.withers_height,
                    pal,
                );
            }
            for &(s, f, t, ghost) in &draw_list {
                // in overlay mode, each dog is centred on its own COM so postures superimpose
                let cam_d = if ghost && self.cam_mode == CamMode::Follow {
                    Cam {
                        origin: V2::new(f.com.x, 0.0),
                        ..cam
                    }
                } else {
                    cam
                };
                if self.opts.trails && !ghost {
                    draw_trails(&lane_painter, &cam_d, s, t, pal);
                }
                draw_dog(&lane_painter, &cam_d, s, f, &self.opts, pal, ghost);
            }
            // lane caption
            let s = lead_sim;
            let caption = if let Some(j) = s.jump() {
                format!(
                    "{} · {:.0} cm · {:.1} kg · {} over {:.0} cm · take-off {:.2} m/s at {:.0}°",
                    s.params.breed.name,
                    s.sk.withers_height * 100.0,
                    s.sk.mass,
                    s.spec.name.to_lowercase(),
                    j.obstacle_h * 100.0,
                    j.v0x.hypot(j.v0z),
                    j.v0z.atan2(j.v0x).to_degrees()
                )
            } else {
                format!(
                    "{} · {:.0} cm · {:.1} kg · {} · {} · Fr {:.2}",
                    s.params.breed.name,
                    s.sk.withers_height * 100.0,
                    s.sk.mass,
                    s.spec.name.to_lowercase(),
                    fmt_speed(s.speed),
                    s.froude()
                )
            };
            lane_painter.text(
                lane.left_top() + vec2(10.0, 8.0),
                Align2::LEFT_TOP,
                caption,
                FontId::proportional(13.0),
                pal.text,
            );
            if self.layout == Layout::Overlay
                && *is_main
                && let Some(c) = &self.cmp
            {
                lane_painter.text(
                    lane.left_top() + vec2(10.0, 26.0),
                    Align2::LEFT_TOP,
                    format!(
                        "Ghost: {} · {:.0} cm",
                        c.sim.params.breed.name,
                        c.sim.sk.withers_height * 100.0
                    ),
                    FontId::proportional(12.0),
                    pal.text_muted,
                );
            }
            if lanes.len() > 1 && *is_main {
                lane_painter.line_segment(
                    [lane.left_bottom(), lane.right_bottom()],
                    Stroke::new(1.0, pal.grid),
                );
            }
        }
        let phase_txt = if self.main.sim.is_periodic() {
            format!("{:5.1} % cycle  ·  t {:.3} s", self.cursor_x(), self.t)
        } else {
            format!("t {:.3} s of {:.2} s", self.t, self.main.sim.duration)
        };
        painter.text(
            rect.right_top() + vec2(-10.0, 8.0),
            Align2::RIGHT_TOP,
            phase_txt,
            FontId::monospace(12.0),
            pal.text_muted,
        );
        let clamped = main_f.limbs.iter().any(|l| l.clamped);
        if clamped {
            painter.text(
                rect.right_top() + vec2(-10.0, 26.0),
                Align2::RIGHT_TOP,
                "Red ring: limb at full extension (target out of reach)",
                FontId::proportional(11.0),
                pal.warn,
            );
        }
    }

    fn analysis(&mut self, ui: &mut Ui, pal: &Palette) {
        ui.horizontal(|ui| {
            for (t, name) in [
                (Tab::Joints, "Joint angles"),
                (Tab::Footfall, "Footfall"),
                (Tab::Com, "Centre of mass"),
                (Tab::Grf, "Ground reaction force"),
                (Tab::PawPaths, "Paw paths"),
                (Tab::Metrics, "Metrics"),
                (Tab::Data, "Data"),
            ] {
                ui.selectable_value(&mut self.tab, t, name);
            }
            if self.tab == Tab::Joints {
                ui.separator();
                ui.selectable_value(&mut self.joint_mode, JointMode::Angle, "Angle");
                ui.selectable_value(
                    &mut self.joint_mode,
                    JointMode::Velocity,
                    "Angular velocity",
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    RichText::new("Click or drag in a chart to move the cursor")
                        .color(pal.text_muted)
                        .size(11.0),
                );
            });
        });
        ui.separator();

        let main_label = self.main.sim.params.breed.name.clone();
        let cmp_label = self
            .cmp
            .as_ref()
            .map(|c| c.sim.params.breed.name.clone())
            .unwrap_or_default();
        let cmp_label = if cmp_label == main_label {
            format!("{cmp_label} (2)")
        } else {
            cmp_label
        };
        let cmp_xs: Option<Vec<f64>> = self.cmp.as_ref().map(|c| {
            if self.main.sim.is_periodic() || self.sync == Sync::RealTime {
                c.xs.clone()
            } else {
                c.frames
                    .iter()
                    .map(|f| f.phase * self.main.sim.duration)
                    .collect()
            }
        });
        let d = PlotData {
            main: Series {
                frames: &self.main.frames,
                xs: &self.main.xs,
                label: &main_label,
            },
            compare: self.cmp.as_ref().map(|c| Series {
                frames: &c.frames,
                xs: cmp_xs.as_deref().unwrap(),
                label: &cmp_label,
            }),
            imported: self.imported.as_ref(),
            periodic: self.main.sim.is_periodic(),
            duration: self.main.sim.duration,
            cursor: self.cursor_x(),
            pal: *pal,
        };
        let period_cmp = self.cmp.as_ref().map_or(1.0, |c| c.sim.duration);
        let scrub = match self.tab {
            Tab::Joints => {
                plots::joint_grid(ui, &d, self.joint_mode, self.main.sim.duration, period_cmp)
            }
            Tab::Footfall => plots::footfall(ui, &d),
            Tab::Com => plots::com(ui, &d),
            Tab::Grf => plots::grf(ui, &d),
            Tab::PawPaths => {
                let f = self.main.sim.sample(self.t);
                let fc = self.cmp.as_ref().map(|c| c.sim.sample(self.cmp_time()));
                plots::paw_paths(ui, &d, &f, fc.as_ref());
                None
            }
            Tab::Metrics => {
                egui::ScrollArea::vertical().show(ui, |ui| self.metrics(ui, pal));
                None
            }
            Tab::Data => {
                egui::ScrollArea::vertical().show(ui, |ui| self.data_tab(ui, pal));
                None
            }
        };
        if let Some(x) = scrub {
            self.playing = false;
            self.scrub_to(x);
        }
    }

    fn metrics(&self, ui: &mut Ui, pal: &Palette) {
        let sims: Vec<&Prepared> = std::iter::once(&self.main)
            .chain(self.cmp.as_ref())
            .collect();
        ui.label(RichText::new("Values computed from the model series (400 samples per cycle). ROM = range of motion.").color(pal.text_muted).size(11.5));
        ui.add_space(4.0);
        egui::Grid::new("metrics")
            .striped(true)
            .num_columns(1 + sims.len())
            .spacing([28.0, 4.0])
            .show(ui, |ui| {
                ui.label(RichText::new("Metric").strong());
                for p in &sims {
                    ui.label(RichText::new(&p.sim.params.breed.name).strong());
                }
                ui.end_row();
                let row = |ui: &mut Ui, name: &str, f: &dyn Fn(&Prepared) -> String| {
                    ui.label(name);
                    for p in &sims {
                        ui.label(RichText::new(f(p)).monospace());
                    }
                    ui.end_row();
                };
                row(ui, "Withers height / mass", &|p| {
                    format!(
                        "{:.1} cm / {:.1} kg",
                        p.sim.sk.withers_height * 100.0,
                        p.sim.sk.mass
                    )
                });
                if let Some(_j) = self.main.sim.jump() {
                    row(ui, "Take-off velocity", &|p| {
                        p.sim.jump().map_or("–".into(), |j| {
                            format!(
                                "{:.2} m/s at {:.0}°",
                                j.v0x.hypot(j.v0z),
                                j.v0z.atan2(j.v0x).to_degrees()
                            )
                        })
                    });
                    row(ui, "Horizontal / vertical", &|p| {
                        p.sim
                            .jump()
                            .map_or("–".into(), |j| format!("{:.2} / {:.2} m/s", j.v0x, j.v0z))
                    });
                    row(ui, "COM apex height", &|p| {
                        p.sim.jump().map_or("–".into(), |j| {
                            format!(
                                "{:.1} cm (stand {:.1})",
                                j.y_apex * 100.0,
                                j.y_stand * 100.0
                            )
                        })
                    });
                    row(ui, "Flight time", &|p| {
                        p.sim.jump().map_or("–".into(), |j| {
                            format!("{:.0} ms", (j.t_land - j.t_takeoff) * 1000.0)
                        })
                    });
                    row(ui, "Take-off → landing (COM)", &|p| {
                        p.sim
                            .jump()
                            .map_or("–".into(), |j| format!("{:.2} m", j.x_land - j.x_takeoff))
                    });
                    row(ui, "Peak vGRF, push-off (hind pair)", &|p| {
                        let j = p.sim.jump().unwrap();
                        let pk = p
                            .frames
                            .iter()
                            .filter(|f| f.t < j.t_takeoff)
                            .map(|f| f.limbs[2].grf_bw + f.limbs[3].grf_bw)
                            .fold(0.0, f64::max);
                        format!("{:.0} %BW", pk * 100.0)
                    });
                    row(ui, "Peak vGRF, landing (fore pair)", &|p| {
                        let j = p.sim.jump().unwrap();
                        let pk = p
                            .frames
                            .iter()
                            .filter(|f| f.t >= j.t_land)
                            .map(|f| f.limbs[0].grf_bw + f.limbs[1].grf_bw)
                            .fold(0.0, f64::max);
                        format!("{:.0} %BW", pk * 100.0)
                    });
                } else {
                    row(ui, "Speed", &|p| fmt_speed(p.sim.speed));
                    row(ui, "Froude number", &|p| format!("{:.2}", p.sim.froude()));
                    row(ui, "Stride length", &|p| {
                        format!(
                            "{:.3} m ({:.2} × hip height)",
                            p.sim.stride_len,
                            p.sim.stride_len / p.sim.sk.hip_h
                        )
                    });
                    row(ui, "Stride frequency / period", &|p| {
                        format!("{:.2} Hz / {:.3} s", 1.0 / p.sim.period, p.sim.period)
                    });
                    row(ui, "Duty factor", &|p| {
                        format!(
                            "{:.1} %{}",
                            p.sim.duty * 100.0,
                            if p.sim.duty_reach_limited {
                                " (reach-limited)"
                            } else {
                                ""
                            }
                        )
                    });
                    row(ui, "Stance / swing time", &|p| {
                        format!(
                            "{:.0} / {:.0} ms",
                            p.sim.duty * p.sim.period * 1000.0,
                            (1.0 - p.sim.duty) * p.sim.period * 1000.0
                        )
                    });
                    row(ui, "Touchdown phase LF RF LH RH", &|p| {
                        let o = p.sim.spec.offsets;
                        format!(
                            "{:.0} {:.0} {:.0} {:.0} %",
                            o[0] * 100.0,
                            o[1] * 100.0,
                            o[2] * 100.0,
                            o[3] * 100.0
                        )
                    });
                    row(ui, "Mean trunk lowering vs stance", &|p| {
                        format!("{:.1} cm", p.sim.lowering * 100.0)
                    });
                }
                row(ui, "COM vertical excursion", &|p| {
                    let (a, b) = p.frames.iter().fold((f64::MAX, f64::MIN), |(a, b), f| {
                        (a.min(f.com.y), b.max(f.com.y))
                    });
                    format!("{:.1} cm", (b - a) * 100.0)
                });
                row(ui, "Trunk pitch range", &|p| {
                    let (a, b) = p.frames.iter().fold((f64::MAX, f64::MIN), |(a, b), f| {
                        (a.min(f.pitch), b.max(f.pitch))
                    });
                    format!("{:+.1}° to {:+.1}°", a.to_degrees(), b.to_degrees())
                });
                for (i, limb) in LIMBS.iter().enumerate() {
                    row(ui, &format!("Peak vGRF {}", limb.label()), &|p| {
                        format!(
                            "{:.0} %BW",
                            p.frames
                                .iter()
                                .map(|f| f.limbs[i].grf_bw)
                                .fold(0.0, f64::max)
                                * 100.0
                        )
                    });
                }
                row(ui, "Frames with a limb at full extension", &|p| {
                    let n = p
                        .frames
                        .iter()
                        .filter(|f| f.limbs.iter().any(|l| l.clamped))
                        .count();
                    format!("{n} of {}", p.frames.len())
                });
            });

        ui.add_space(10.0);
        ui.label(RichText::new("Joint range of motion (min – max, ROM), degrees").strong());
        for p in &sims {
            ui.label(RichText::new(&p.sim.params.breed.name).color(pal.text_muted));
            egui::Grid::new(format!("rom-{}", p.sim.params.breed.name))
                .striped(true)
                .spacing([24.0, 3.0])
                .show(ui, |ui| {
                    ui.label("");
                    for limb in LIMBS {
                        ui.label(RichText::new(limb.long_name()).strong());
                    }
                    ui.end_row();
                    for j in 0..3 {
                        ui.label(format!(
                            "{} / {}",
                            crate::model::FORE_JOINTS[j],
                            crate::model::HIND_JOINTS[j]
                        ));
                        for (i, _) in LIMBS.iter().enumerate() {
                            let (a, b) = p.frames.iter().fold((f64::MAX, f64::MIN), |(a, b), f| {
                                (a.min(f.limbs[i].angles[j]), b.max(f.limbs[i].angles[j]))
                            });
                            ui.label(
                                RichText::new(format!("{a:5.1}–{b:5.1} ({:4.1})", b - a))
                                    .monospace(),
                            );
                        }
                        ui.end_row();
                    }
                });
        }
    }

    fn data_tab(&mut self, ui: &mut Ui, pal: &Palette) {
        ui.horizontal(|ui| {
            if ui.button("Import CSV…").clicked() {
                self.import_dialog();
            }
            if self.imported.is_some() && ui.button("Remove imported data").clicked() {
                self.imported = None;
                self.drive_from_import = false;
            }
        });
        if let Some(e) = &self.import_error {
            ui.colored_label(pal.warn, e);
        }
        if let Some(imp) = &self.imported {
            ui.label(format!(
                "{} · {} rows · {} recognised columns{}",
                imp.file_name,
                imp.pct.len(),
                imp.series.len(),
                imp.duration_s
                    .map_or(String::new(), |d| format!(" · {:.3} s", d))
            ));
            if imp.multi_cycle {
                ui.label(RichText::new("cycle_pct wraps: several cycles found; they are pooled and plotted as points.").color(pal.text_muted));
            }
            if !imp.ignored_columns.is_empty() {
                ui.label(
                    RichText::new(format!(
                        "Ignored columns: {}",
                        imp.ignored_columns.join(", ")
                    ))
                    .color(pal.text_muted)
                    .size(11.5),
                );
            }
            for c in imp.comments.iter().take(4) {
                ui.label(
                    RichText::new(format!("# {c}"))
                        .color(pal.text_muted)
                        .monospace()
                        .size(11.0),
                );
            }
            let all = imp.has_all_angles();
            ui.add_enabled(
                all,
                egui::Checkbox::new(
                    &mut self.drive_from_import,
                    "Drive the animation from the imported joint angles",
                ),
            )
            .on_disabled_hover_text("Needs all 12 joint-angle columns.");

            ui.add_space(6.0);
            ui.label(
                RichText::new(format!(
                    "Model ({}) vs imported data",
                    self.main.sim.params.breed.name
                ))
                .strong(),
            );
            if !self.main.sim.is_periodic() {
                ui.label(
                    RichText::new("Comparison uses % of the jump record.").color(pal.text_muted),
                );
            }
            egui::Grid::new("cmp-table")
                .striped(true)
                .spacing([24.0, 3.0])
                .show(ui, |ui| {
                    for h in [
                        "Column",
                        "n",
                        "RMSE (deg)",
                        "Mean diff (data − model)",
                        "Pearson r",
                    ] {
                        ui.label(RichText::new(h).strong());
                    }
                    ui.end_row();
                    for limb in LIMBS {
                        for j in 0..3 {
                            let col = limb.angle_column(j);
                            if !imp.has(&col) {
                                continue;
                            }
                            let li = limb.idx();
                            let pairs: Vec<(f64, f64)> = imp
                                .pct
                                .iter()
                                .zip(&imp.series[&col])
                                .filter(|(_, v)| v.is_finite())
                                .map(|(&x, &v)| {
                                    (
                                        v,
                                        self.main
                                            .sim
                                            .sample(x / 100.0 * self.main.sim.duration)
                                            .limbs[li]
                                            .angles[j],
                                    )
                                })
                                .collect();
                            let s = stats(&pairs);
                            ui.label(RichText::new(&col).monospace());
                            ui.label(format!("{}", pairs.len()));
                            ui.label(RichText::new(format!("{:.2}", s.0)).monospace());
                            ui.label(RichText::new(format!("{:+.2}", s.1)).monospace());
                            ui.label(RichText::new(format!("{:.3}", s.2)).monospace());
                            ui.end_row();
                        }
                    }
                });
        } else {
            ui.label(RichText::new("No data imported. Measured joint angles appear as dashed lines next to the model.").color(pal.text_muted));
        }

        ui.add_space(10.0);
        ui.separator();
        ui.label(RichText::new("Export model series").strong());
        ui.horizontal(|ui| {
            ui.label("Sampling rate");
            ui.add(
                egui::DragValue::new(&mut self.export_hz)
                    .range(10.0..=5000.0)
                    .suffix(" Hz"),
            );
            if self.main.sim.is_periodic() {
                ui.label("Cycles");
                ui.add(egui::DragValue::new(&mut self.export_cycles).range(1..=50));
            }
            if ui.button("Export CSV…").clicked() {
                self.export_dialog();
            }
        });
        if let Some((msg, err)) = &self.status {
            ui.label(RichText::new(msg).color(if *err { pal.warn } else { pal.text_muted }));
        }

        ui.add_space(10.0);
        egui::CollapsingHeader::new("CSV format reference").default_open(false).show(ui, |ui| {
            ui.label("One row per sample. Required: `cycle_pct` (0–100) or `time_s` (then one stride is assumed). Lines starting with # are comments; blank cells are missing values.");
            ui.label("Angles follow the app convention: included angle on the flexor side, 180° = straight, above 180° = hyperextension.");
            ui.label(RichText::new(recognised_columns().join(", ")).monospace().size(11.0).color(pal.text_muted));
        });
    }
}

/// (RMSE, mean difference a−b, Pearson r) for (data, model) pairs.
fn stats(p: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = p.len() as f64;
    if n < 2.0 {
        return (f64::NAN, f64::NAN, f64::NAN);
    }
    let rmse = (p.iter().map(|(a, b)| (a - b).powi(2)).sum::<f64>() / n).sqrt();
    let md = p.iter().map(|(a, b)| a - b).sum::<f64>() / n;
    let ma = p.iter().map(|x| x.0).sum::<f64>() / n;
    let mb = p.iter().map(|x| x.1).sum::<f64>() / n;
    let cov = p.iter().map(|(a, b)| (a - ma) * (b - mb)).sum::<f64>();
    let va = p.iter().map(|(a, _)| (a - ma).powi(2)).sum::<f64>();
    let vb = p.iter().map(|(_, b)| (b - mb).powi(2)).sum::<f64>();
    (rmse, md, cov / (va * vb).sqrt())
}
