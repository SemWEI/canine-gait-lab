//! Time-series panels below the animation. Every panel shares the x-axis (gait-cycle % for
//! periodic gaits, seconds for jumps), the playback cursor, and click/drag-to-scrub.

use crate::io::csv_import::Imported;
use crate::model::sim::Frame;
use crate::model::{LIMBS, Limb};
use crate::palette::{Palette, with_alpha};
use eframe::egui::{self, Color32, Id, Ui};
use egui_plot::{
    Corner, HLine, Legend, Line, LineStyle, MarkerShape, Plot, PlotPoints, PlotUi, Points, Polygon,
    VLine,
};

pub struct Series<'a> {
    pub frames: &'a [Frame],
    pub xs: &'a [f64],
    pub label: &'a str,
}

pub struct PlotData<'a> {
    pub main: Series<'a>,
    pub compare: Option<Series<'a>>,
    pub imported: Option<&'a Imported>,
    pub periodic: bool,
    /// Record length in seconds (for mapping imported % onto a jump record).
    pub duration: f64,
    pub cursor: f64,
    pub pal: Palette,
}

impl PlotData<'_> {
    fn x_label(&self) -> &'static str {
        if self.periodic {
            "% gait cycle"
        } else {
            "time (s)"
        }
    }

    fn x_max(&self) -> f64 {
        if self.periodic { 100.0 } else { self.duration }
    }

    fn import_x(&self, pct: f64) -> f64 {
        if self.periodic {
            pct
        } else {
            pct / 100.0 * self.duration
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum JointMode {
    Angle,
    Velocity,
}

const LINK: &str = "gait-plots";

fn base_plot<'a>(id: &str, d: &PlotData, height: f32, y_label: &str) -> Plot<'a> {
    bare_plot(id, d, height, y_label).legend(
        Legend::default()
            .position(Corner::RightTop)
            .background_alpha(0.75),
    )
}

/// Shared-axis plot without its own legend (used under a common legend row).
fn bare_plot<'a>(id: &str, d: &PlotData, height: f32, y_label: &str) -> Plot<'a> {
    Plot::new(id)
        .height(height)
        .allow_drag(false)
        .allow_zoom(false)
        .allow_scroll(false)
        .allow_boxed_zoom(false)
        .show_background(false)
        .link_cursor(Id::new(LINK), [true, false])
        .include_x(0.0)
        .include_x(d.x_max())
        .x_axis_label(d.x_label())
        .y_axis_label(y_label.to_string())
        .y_axis_min_width(36.0)
}

/// Add the cursor and report a click/drag position as a scrub request.
fn finish(pu: &mut PlotUi, d: &PlotData) -> Option<f64> {
    pu.vline(
        VLine::new("", d.cursor)
            .color(with_alpha(d.pal.text_muted, 160))
            .width(1.0),
    );
    let r = pu.response();
    if r.is_pointer_button_down_on() || r.clicked() {
        pu.pointer_coordinate().map(|p| p.x.clamp(0.0, d.x_max()))
    } else {
        None
    }
}

fn pts(xs: &[f64], frames: &[Frame], f: impl Fn(&Frame) -> f64) -> Vec<[f64; 2]> {
    xs.iter().zip(frames).map(|(&x, fr)| [x, f(fr)]).collect()
}

fn close_cycle(mut v: Vec<[f64; 2]>, periodic: bool) -> Vec<[f64; 2]> {
    if periodic && !v.is_empty() {
        let first = v[0];
        v.push([100.0, first[1]]);
    }
    v
}

fn velocity(xs: &[f64], ys: &[f64], periodic: bool, period_s: f64) -> Vec<f64> {
    let n = ys.len();
    let dt_of = |a: f64, b: f64| {
        if periodic {
            (b - a) / 100.0 * period_s
        } else {
            b - a
        }
    };
    (0..n)
        .map(|i| {
            let (im, ip) = if periodic {
                ((i + n - 1) % n, (i + 1) % n)
            } else {
                (i.saturating_sub(1), (i + 1).min(n - 1))
            };
            let mut xa = xs[im];
            let mut xb = xs[ip];
            if periodic && ip < i {
                xb += 100.0;
            }
            if periodic && im > i {
                xa -= 100.0;
            }
            let dt = dt_of(xa, xb);
            if dt.abs() < 1e-12 {
                0.0
            } else {
                (ys[ip] - ys[im]) / dt
            }
        })
        .collect()
}

/// Model, comparison and imported series of one quantity for one limb.
#[allow(clippy::too_many_arguments)]
fn limb_lines(
    pu: &mut PlotUi,
    d: &PlotData,
    limb: Limb,
    value: &dyn Fn(&Frame) -> f64,
    import_col: Option<&str>,
    mode: JointMode,
    period_main: f64,
    period_cmp: f64,
) {
    let c = d.pal.limb[limb.idx()];
    let mk = |s: &Series, period: f64| -> Vec<[f64; 2]> {
        let ys: Vec<f64> = s.frames.iter().map(value).collect();
        let ys = match mode {
            JointMode::Angle => ys,
            JointMode::Velocity => velocity(s.xs, &ys, d.periodic, period),
        };
        close_cycle(
            s.xs.iter().zip(ys).map(|(&x, y)| [x, y]).collect(),
            d.periodic,
        )
    };
    pu.line(
        Line::new(
            format!("{} · {}", limb.label(), d.main.label),
            PlotPoints::new(mk(&d.main, period_main)),
        )
        .color(c)
        .width(2.0),
    );
    if let Some(cmp) = &d.compare {
        pu.line(
            Line::new(
                format!("{} · {}", limb.label(), cmp.label),
                PlotPoints::new(mk(cmp, period_cmp)),
            )
            .color(c)
            .width(1.5)
            .style(LineStyle::Dotted { spacing: 4.0 }),
        );
    }
    if let (Some(imp), Some(col), JointMode::Angle) = (d.imported, import_col, mode)
        && imp.has(col)
    {
        let v: Vec<[f64; 2]> = imp
            .pct
            .iter()
            .zip(&imp.series[col])
            .filter(|(_, y)| y.is_finite())
            .map(|(&x, &y)| [d.import_x(x), y])
            .collect();
        let name = format!("{} · data", limb.label());
        if imp.multi_cycle {
            pu.points(
                Points::new(name, PlotPoints::new(v))
                    .color(c)
                    .radius(1.5)
                    .shape(MarkerShape::Circle),
            );
        } else {
            pu.line(
                Line::new(name, PlotPoints::new(v))
                    .color(c)
                    .width(1.5)
                    .style(LineStyle::Dashed { length: 6.0 }),
            );
        }
    }
}

/// One legend for a set of small multiples: limb colours plus the line-style key.
pub fn legend_row(ui: &mut Ui, d: &PlotData) {
    ui.horizontal_wrapped(|ui| {
        for limb in LIMBS {
            let (r, p) = ui.allocate_painter(egui::vec2(22.0, 12.0), egui::Sense::hover());
            let y = r.rect.center().y;
            p.line_segment(
                [egui::pos2(r.rect.left(), y), egui::pos2(r.rect.right(), y)],
                egui::Stroke::new(2.5, d.pal.limb[limb.idx()]),
            );
            ui.label(limb.long_name());
            ui.add_space(8.0);
        }
        ui.separator();
        let mut key = format!("solid: {}", d.main.label);
        if let Some(c) = &d.compare {
            key.push_str(&format!(" · dotted: {}", c.label));
        }
        if let Some(imp) = d.imported {
            key.push_str(&format!(" · dashed: {}", imp.file_name));
        }
        ui.label(egui::RichText::new(key).color(d.pal.text_muted));
    });
}

pub fn joint_grid(
    ui: &mut Ui,
    d: &PlotData,
    mode: JointMode,
    period_main: f64,
    period_cmp: f64,
) -> Option<f64> {
    let rows = 3;
    legend_row(ui, d);
    let avail = ui.available_height();
    let h = ((avail - 8.0) / rows as f32 - 4.0).max(90.0);
    let mut scrub = None;
    let col_w = ((ui.available_width() - 12.0) / 2.0).max(160.0);
    let unit = match mode {
        JointMode::Angle => "deg",
        JointMode::Velocity => "deg/s",
    };
    egui::Grid::new("joint-grid")
        .num_columns(2)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            for j in 0..3 {
                for fore in [true, false] {
                    let (a, b) = if fore {
                        (Limb::LF, Limb::RF)
                    } else {
                        (Limb::LH, Limb::RH)
                    };
                    let name = a.joints()[j];
                    let w = col_w;
                    ui.vertical(|ui| {
                        ui.set_width(w);
                        ui.label(
                            egui::RichText::new(format!("{} ({})", capitalise(name), unit))
                                .strong(),
                        );
                        let r = bare_plot(&format!("joint-{fore}-{j}"), d, h - 18.0, unit)
                            .width(w)
                            .show(ui, |pu| {
                                for limb in [a, b] {
                                    let col = limb.angle_column(j);
                                    limb_lines(
                                        pu,
                                        d,
                                        limb,
                                        &|f: &Frame| f.limbs[limb.idx()].angles[j],
                                        Some(&col),
                                        mode,
                                        period_main,
                                        period_cmp,
                                    );
                                }
                                finish(pu, d)
                            });
                        if let Some(x) = r.inner {
                            scrub = Some(x);
                        }
                    });
                }
                ui.end_row();
            }
        });
    scrub
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Contact intervals as (start, end) in x units.
fn contact_runs(xs: &[f64], frames: &[Frame], limb: usize, x_end: f64) -> Vec<(f64, f64)> {
    let mut runs = Vec::new();
    let mut start: Option<f64> = None;
    for (k, f) in frames.iter().enumerate() {
        let c = f.limbs[limb].contact;
        match (c, start) {
            (true, None) => start = Some(xs[k]),
            (false, Some(s)) => {
                runs.push((s, xs[k]));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        runs.push((s, x_end));
    }
    runs
}

fn bar(pu: &mut PlotUi, name: &str, x0: f64, x1: f64, y0: f64, y1: f64, c: Color32) {
    pu.polygon(
        Polygon::new(
            name.to_string(),
            PlotPoints::new(vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]),
        )
        .fill_color(c)
        .stroke(egui::Stroke::new(0.0, c)),
    );
}

/// Hildebrand-style footfall diagram: one row per limb, bars where the paw is on the ground.
pub fn footfall(ui: &mut Ui, d: &PlotData) -> Option<f64> {
    let reps = if d.periodic { 2 } else { 1 };
    let x_max = d.x_max() * reps as f64;
    let mut key = format!("Bars = paw on the ground. Full bars: {}", d.main.label);
    if let Some(c) = &d.compare {
        key.push_str(&format!(" · thin bars: {}", c.label));
    }
    if let Some(imp) = d.imported {
        key.push_str(&format!(" · faint thin bars: {}", imp.file_name));
    }
    if d.periodic {
        key.push_str(" · two cycles shown");
    }
    ui.label(egui::RichText::new(key).color(d.pal.text_muted));
    let h = ui.available_height() - 4.0;
    let row_of = |limb: usize| 3.0 - limb as f64;
    let r = bare_plot("footfall", d, h, "")
        .include_x(x_max)
        .include_y(-0.6)
        .include_y(3.6)
        .show_grid([true, false])
        .y_axis_formatter(|m, _| {
            let v = m.value.round();
            if (m.value - v).abs() > 1e-6 {
                return String::new();
            }
            match v as i64 {
                3 => "LF".into(),
                2 => "RF".into(),
                1 => "LH".into(),
                0 => "RH".into(),
                _ => String::new(),
            }
        })
        .show(ui, |pu| {
            for (i, limb) in LIMBS.iter().enumerate() {
                let c = d.pal.limb[i];
                let y = row_of(i);
                let main_h = if d.compare.is_some() || d.imported.is_some() {
                    (y - 0.05, y + 0.35)
                } else {
                    (y - 0.3, y + 0.3)
                };
                for rep in 0..reps {
                    let off = rep as f64 * 100.0;
                    for (a, b) in contact_runs(d.main.xs, d.main.frames, i, d.x_max()) {
                        bar(
                            pu,
                            &format!("{} stance · {}", limb.label(), d.main.label),
                            a + off,
                            b + off,
                            main_h.0,
                            main_h.1,
                            c,
                        );
                    }
                    if let Some(cmp) = &d.compare {
                        for (a, b) in contact_runs(cmp.xs, cmp.frames, i, d.x_max()) {
                            bar(
                                pu,
                                &format!("{} stance · {}", limb.label(), cmp.label),
                                a + off,
                                b + off,
                                y - 0.33,
                                y - 0.13,
                                with_alpha(c, 120),
                            );
                        }
                    }
                    if let Some(imp) = d.imported {
                        let col = format!("{}_contact", limb.label());
                        if imp.has(&col) {
                            let ys = &imp.series[&col];
                            let mut start: Option<f64> = None;
                            for (k, &v) in ys.iter().enumerate() {
                                let x = d.import_x(imp.pct[k]);
                                match (v > 0.5, start) {
                                    (true, None) => start = Some(x),
                                    (false, Some(s)) => {
                                        bar(
                                            pu,
                                            &format!("{} stance · data", limb.label()),
                                            s + off,
                                            x + off,
                                            y - 0.33,
                                            y - 0.13,
                                            with_alpha(c, 70),
                                        );
                                        start = None;
                                    }
                                    _ => {}
                                }
                            }
                            if let Some(s) = start {
                                bar(
                                    pu,
                                    &format!("{} stance · data", limb.label()),
                                    s + off,
                                    d.x_max() + off,
                                    y - 0.33,
                                    y - 0.13,
                                    with_alpha(c, 70),
                                );
                            }
                        }
                    }
                }
            }
            if d.periodic {
                pu.vline(
                    VLine::new("", 100.0)
                        .color(with_alpha(d.pal.text_muted, 90))
                        .width(1.0)
                        .style(LineStyle::Dashed { length: 4.0 }),
                );
                pu.vline(
                    VLine::new("", d.cursor + 100.0)
                        .color(with_alpha(d.pal.text_muted, 90))
                        .width(1.0),
                );
            }
            finish(pu, d).map(|x| if d.periodic { x % 100.0 } else { x })
        });
    r.inner
}

#[allow(clippy::too_many_arguments)]
fn scalar_plot(
    ui: &mut Ui,
    d: &PlotData,
    id: &str,
    title: &str,
    unit: &str,
    width: f32,
    height: f32,
    value: &dyn Fn(&Frame) -> f64,
    import_col: Option<&str>,
    reference: Option<f64>,
) -> Option<f64> {
    let mut out = None;
    ui.vertical(|ui| {
        ui.set_width(width);
        ui.label(egui::RichText::new(format!("{title} ({unit})")).strong());
        let r = base_plot(id, d, height - 18.0, unit)
            .width(width)
            .show(ui, |pu| {
                if let Some(y) = reference {
                    pu.hline(
                        HLine::new("", y)
                            .color(with_alpha(d.pal.text_muted, 120))
                            .style(LineStyle::Dashed { length: 5.0 }),
                    );
                }
                let c = d.pal.text;
                pu.line(
                    Line::new(
                        d.main.label.to_string(),
                        PlotPoints::new(close_cycle(
                            pts(d.main.xs, d.main.frames, value),
                            d.periodic,
                        )),
                    )
                    .color(c)
                    .width(2.0),
                );
                if let Some(cmp) = &d.compare {
                    pu.line(
                        Line::new(
                            cmp.label.to_string(),
                            PlotPoints::new(close_cycle(
                                pts(cmp.xs, cmp.frames, value),
                                d.periodic,
                            )),
                        )
                        .color(c)
                        .width(1.5)
                        .style(LineStyle::Dotted { spacing: 4.0 }),
                    );
                }
                if let (Some(imp), Some(col)) = (d.imported, import_col)
                    && imp.has(col)
                {
                    let scale = if col.ends_with("_m") { 100.0 } else { 1.0 };
                    let v: Vec<[f64; 2]> = imp
                        .pct
                        .iter()
                        .zip(&imp.series[col])
                        .filter(|(_, y)| y.is_finite())
                        .map(|(&x, &y)| [d.import_x(x), y * scale])
                        .collect();
                    pu.line(
                        Line::new("data", PlotPoints::new(v))
                            .color(c)
                            .width(1.5)
                            .style(LineStyle::Dashed { length: 6.0 }),
                    );
                }
                finish(pu, d)
            });
        out = r.inner;
    });
    out
}

pub fn com(ui: &mut Ui, d: &PlotData) -> Option<f64> {
    let h = ((ui.available_height() - 8.0) / 2.0).max(110.0);
    let w = (ui.available_width() / 2.0 - 8.0).max(160.0);
    let mut scrub = None;
    egui::Grid::new("com-grid")
        .num_columns(2)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "com-y",
                "COM height",
                "cm",
                w,
                h,
                &|f| f.com.y * 100.0,
                Some("com_y_m"),
                None,
            ));
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "com-vy",
                "COM vertical velocity",
                "m/s",
                w,
                h,
                &|f| f.com_v.y,
                None,
                Some(0.0),
            ));
            ui.end_row();
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "pitch",
                "Trunk pitch, nose-up +",
                "deg",
                w,
                h,
                &|f| f.pitch.to_degrees(),
                Some("pitch_deg"),
                Some(0.0),
            ));
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "com-vx",
                "COM forward velocity",
                "m/s",
                w,
                h,
                &|f| f.com_v.x,
                None,
                None,
            ));
            ui.end_row();
        });
    scrub
}

pub fn grf(ui: &mut Ui, d: &PlotData) -> Option<f64> {
    let h = ui.available_height() - 4.0;
    let w = (ui.available_width() / 3.0 - 8.0).max(160.0);
    let mut scrub = None;
    ui.horizontal_top(|ui| {
        for (title, pair) in [
            ("Forelimbs", [Limb::LF, Limb::RF]),
            ("Hindlimbs", [Limb::LH, Limb::RH]),
        ] {
            ui.vertical(|ui| {
                ui.set_width(w);
                ui.label(
                    egui::RichText::new(format!("{title}: vertical GRF (% body weight, estimate)"))
                        .strong(),
                );
                let r = base_plot(&format!("grf-{title}"), d, h - 18.0, "%BW")
                    .width(w)
                    .show(ui, |pu| {
                        for limb in pair {
                            let col = format!("{}_grf_bw", limb.label());
                            limb_lines(
                                pu,
                                d,
                                limb,
                                &|f: &Frame| f.limbs[limb.idx()].grf_bw * 100.0,
                                None,
                                JointMode::Angle,
                                1.0,
                                1.0,
                            );
                            if let Some(imp) = d.imported
                                && imp.has(&col)
                            {
                                let v: Vec<[f64; 2]> = imp
                                    .pct
                                    .iter()
                                    .zip(&imp.series[&col])
                                    .filter(|(_, y)| y.is_finite())
                                    .map(|(&x, &y)| [d.import_x(x), y * 100.0])
                                    .collect();
                                pu.line(
                                    Line::new(
                                        format!("{} · data", limb.label()),
                                        PlotPoints::new(v),
                                    )
                                    .color(d.pal.limb[limb.idx()])
                                    .width(1.5)
                                    .style(LineStyle::Dashed { length: 6.0 }),
                                );
                            }
                        }
                        finish(pu, d)
                    });
                scrub = scrub.or(r.inner);
            });
        }
        scrub = scrub.or(scalar_plot(
            ui,
            d,
            "grf-total",
            "Total vertical GRF",
            "%BW",
            w,
            h,
            &|f| f.total_grf() * 100.0,
            None,
            Some(100.0),
        ));
    });
    scrub
}

/// Dashed overlay of an imported per-limb column, scaled by `scale`.
fn import_overlay(pu: &mut PlotUi, d: &PlotData, limb: Limb, col: &str, scale: f64) {
    if let Some(imp) = d.imported
        && imp.has(col)
    {
        let v: Vec<[f64; 2]> = imp
            .pct
            .iter()
            .zip(&imp.series[col])
            .filter(|(_, y)| y.is_finite())
            .map(|(&x, &y)| [d.import_x(x), y * scale])
            .collect();
        pu.line(
            Line::new(format!("{} · data", limb.label()), PlotPoints::new(v))
                .color(d.pal.limb[limb.idx()])
                .width(1.5)
                .style(LineStyle::Dashed { length: 6.0 }),
        );
    }
}

/// Frontal-plane panels: limb ab/adduction, lateral COM sway, trunk roll and ML GRF.
pub fn frontal(ui: &mut Ui, d: &PlotData) -> Option<f64> {
    legend_row(ui, d);
    let h = ((ui.available_height() - 8.0) / 2.0 - 4.0).max(110.0);
    let w = ((ui.available_width() - 12.0) / 2.0).max(160.0);
    let mut scrub = None;
    let limb_panel = |ui: &mut Ui,
                      id: &str,
                      title: &str,
                      unit: &str,
                      value: &dyn Fn(&Frame, usize) -> f64,
                      col: &dyn Fn(Limb) -> String,
                      scale: f64|
     -> Option<f64> {
        let mut out = None;
        ui.vertical(|ui| {
            ui.set_width(w);
            ui.label(egui::RichText::new(format!("{title} ({unit})")).strong());
            let r = bare_plot(id, d, h - 18.0, unit).width(w).show(ui, |pu| {
                pu.hline(
                    HLine::new("", 0.0)
                        .color(with_alpha(d.pal.text_muted, 120))
                        .style(LineStyle::Dashed { length: 5.0 }),
                );
                for limb in LIMBS {
                    limb_lines(
                        pu,
                        d,
                        limb,
                        &|f: &Frame| value(f, limb.idx()),
                        None,
                        JointMode::Angle,
                        1.0,
                        1.0,
                    );
                    import_overlay(pu, d, limb, &col(limb), scale);
                }
                finish(pu, d)
            });
            out = r.inner;
        });
        out
    };
    egui::Grid::new("frontal-grid")
        .num_columns(2)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            scrub = scrub.or(limb_panel(
                ui,
                "frontal-abd",
                "Limb ab/adduction vs trunk, abduction +",
                "deg",
                &|f, i| f.limbs[i].abduction,
                &|l| l.abduction_column(),
                1.0,
            ));
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "frontal-com-z",
                "Lateral COM position, left +",
                "cm",
                w,
                h,
                &|f| f.com_z * 100.0,
                Some("com_z_m"),
                Some(0.0),
            ));
            ui.end_row();
            scrub = scrub.or(scalar_plot(
                ui,
                d,
                "frontal-roll",
                "Trunk roll, left side up +",
                "deg",
                w,
                h,
                &|f| f.roll.to_degrees(),
                Some("roll_deg"),
                Some(0.0),
            ));
            scrub = scrub.or(limb_panel(
                ui,
                "frontal-grf-ml",
                "Mediolateral GRF, toward left + (estimate)",
                "%BW",
                &|f, i| f.limbs[i].grf_ml_bw * 100.0,
                &|l| format!("{}_grf_ml_bw", l.label()),
                100.0,
            ));
            ui.end_row();
        });
    scrub
}

/// Paw path relative to the scapular pivot (fore) or hip joint (hind), in centimetres:
/// sagittal (cranial, dorsal) or frontal (lateral, dorsal).
pub fn paw_paths(
    ui: &mut Ui,
    d: &PlotData,
    current: &Frame,
    current_cmp: Option<&Frame>,
    frontal: bool,
) {
    let h = ui.available_height() - 4.0;
    let w = (ui.available_width() / 2.0 - 8.0).max(160.0);
    ui.horizontal_top(|ui| {
        for (title, limb) in [
            ("Left fore paw, relative to scapular pivot", Limb::LF),
            ("Left hind paw, relative to hip joint", Limb::LH),
        ] {
            ui.vertical(|ui| {
                ui.set_width(w);
                ui.label(egui::RichText::new(format!("{title} (cm)")).strong());
                let i = limb.idx();
                let rel = |f: &Frame| {
                    let l = &f.limbs[i];
                    let k = if limb.is_fore() { 0 } else { 1 };
                    let r = l.pts[4] - l.pts[k];
                    if frontal {
                        [(l.lat[4] - l.lat[k]) * 100.0, r.y * 100.0]
                    } else {
                        [r.x * 100.0, r.y * 100.0]
                    }
                };
                Plot::new(format!("paw-{title}-{frontal}"))
                    .height(h - 18.0)
                    .width(w)
                    .data_aspect(1.0)
                    .legend(
                        Legend::default()
                            .position(Corner::RightTop)
                            .background_alpha(0.75),
                    )
                    .show_background(false)
                    .x_axis_label(if frontal {
                        "lateral, left + (cm)"
                    } else {
                        "cranial (cm)"
                    })
                    .y_axis_label("dorsal (cm)")
                    .show(ui, |pu| {
                        let c = d.pal.limb[i];
                        let mut v: Vec<[f64; 2]> = d.main.frames.iter().map(rel).collect();
                        if d.periodic && !v.is_empty() {
                            v.push(v[0]);
                        }
                        pu.line(
                            Line::new(d.main.label.to_string(), PlotPoints::new(v))
                                .color(c)
                                .width(2.0),
                        );
                        if let Some(cmp) = &d.compare {
                            let mut v: Vec<[f64; 2]> = cmp.frames.iter().map(rel).collect();
                            if d.periodic && !v.is_empty() {
                                v.push(v[0]);
                            }
                            pu.line(
                                Line::new(cmp.label.to_string(), PlotPoints::new(v))
                                    .color(c)
                                    .width(1.5)
                                    .style(LineStyle::Dotted { spacing: 4.0 }),
                            );
                        }
                        pu.points(
                            Points::new("now", PlotPoints::new(vec![rel(current)]))
                                .color(c)
                                .radius(5.0)
                                .filled(true),
                        );
                        if let Some(fc) = current_cmp {
                            pu.points(
                                Points::new("now (compare)", PlotPoints::new(vec![rel(fc)]))
                                    .color(c)
                                    .radius(4.0)
                                    .filled(false),
                            );
                        }
                    });
            });
        }
    });
}
