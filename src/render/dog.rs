//! Side-view (sagittal) drawing of the dog, its skeleton overlay and the measurement scene,
//! plus the camera and shape helpers shared with the front and top views.

use super::View;
use crate::model::breed::EarType;
use crate::model::math::{V2, rad};
use crate::model::sim::{Frame, Simulation};
use crate::model::{LIMBS, Limb};
use crate::palette::{Palette, rgb, shade, with_alpha};
use eframe::egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, pos2, vec2};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug)]
pub struct DrawOptions {
    pub silhouette: bool,
    pub skeleton: bool,
    pub angle_labels: bool,
    pub grf_vectors: bool,
    pub trails: bool,
    pub com_marker: bool,
    pub contacts: bool,
}

impl Default for DrawOptions {
    fn default() -> Self {
        Self {
            silhouette: true,
            skeleton: true,
            angle_labels: true,
            grf_vectors: false,
            trails: true,
            com_marker: true,
            contacts: true,
        }
    }
}

/// World (metres, y up) → screen mapping.
#[derive(Clone, Copy, Debug)]
pub struct Cam {
    /// World point drawn at `anchor`.
    pub origin: V2,
    pub anchor: Pos2,
    pub ppm: f32,
}

impl Cam {
    pub fn pt(&self, v: V2) -> Pos2 {
        pos2(
            self.anchor.x + ((v.x - self.origin.x) as f32) * self.ppm,
            self.anchor.y - ((v.y - self.origin.y) as f32) * self.ppm,
        )
    }

    pub fn world_x(&self, sx: f32) -> f64 {
        self.origin.x + ((sx - self.anchor.x) / self.ppm) as f64
    }
}

pub(crate) fn capsule(cam: &Cam, a: V2, b: V2, ra: f64, rb: f64) -> Vec<Pos2> {
    let u = (b - a).norm();
    let base = u.y.atan2(u.x);
    let mut pts = Vec::with_capacity(26);
    let seg = 12;
    for i in 0..=seg {
        let ang = base - PI / 2.0 + PI * i as f64 / seg as f64;
        pts.push(cam.pt(b + V2::new(ang.cos(), ang.sin()) * rb));
    }
    for i in 0..=seg {
        let ang = base + PI / 2.0 + PI * i as f64 / seg as f64;
        pts.push(cam.pt(a + V2::new(ang.cos(), ang.sin()) * ra));
    }
    pts
}

pub(crate) fn ellipse(cam: &Cam, c: V2, a: f64, b: f64, rot: f64) -> Vec<Pos2> {
    (0..28)
        .map(|i| {
            let t = 2.0 * PI * i as f64 / 28.0;
            cam.pt(c + V2::new(a * t.cos(), b * t.sin()).rot(rot))
        })
        .collect()
}

pub(crate) fn fill(p: &Painter, pts: Vec<Pos2>, c: Color32) {
    p.add(Shape::convex_polygon(pts, c, Stroke::NONE));
}

/// Paint pass for the silhouette: normal, or an inflated single-colour halo drawn underneath
/// so dark coats stay visible on a dark canvas (no internal seams, unlike per-shape strokes).
#[derive(Clone, Copy)]
pub(crate) struct Ink {
    pub(crate) inflate: f64,
    pub(crate) color: Option<Color32>,
}

impl Ink {
    pub(crate) const NORMAL: Ink = Ink {
        inflate: 0.0,
        color: None,
    };

    pub(crate) fn fill(&self, p: &Painter, pts: Vec<Pos2>, c: Color32) {
        fill(p, pts, self.color.unwrap_or(c));
    }

    pub(crate) fn capsule(&self, cam: &Cam, a: V2, b: V2, ra: f64, rb: f64) -> Vec<Pos2> {
        capsule(cam, a, b, ra + self.inflate, rb + self.inflate)
    }

    pub(crate) fn ellipse(&self, cam: &Cam, c: V2, a: f64, b: f64, rot: f64) -> Vec<Pos2> {
        ellipse(cam, c, a + self.inflate, b + self.inflate, rot)
    }

    pub(crate) fn halo(&self) -> bool {
        self.color.is_some()
    }
}

pub(crate) fn limb_color(pal: &Palette, limb: Limb) -> Color32 {
    pal.limb[limb.idx()]
}

/// Draw one limb's soft tissue (tapered capsules).
#[allow(clippy::too_many_arguments)]
fn draw_limb_flesh(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    i: usize,
    body: Color32,
    legs: Color32,
    ink: Ink,
) {
    let hw = sim.sk.withers_height;
    let l = &f.limbs[i];
    let [top, prox, mid, dist, paw] = l.pts;
    let up = V2::new(0.0, 1.0);
    if LIMBS[i].is_fore() {
        ink.fill(
            p,
            ink.capsule(
                cam,
                top + V2::new(0.0, -0.04 * hw),
                prox,
                0.085 * hw,
                0.075 * hw,
            ),
            body,
        );
        ink.fill(p, ink.capsule(cam, prox, mid, 0.075 * hw, 0.05 * hw), body);
        ink.fill(p, ink.capsule(cam, mid, dist, 0.045 * hw, 0.028 * hw), legs);
        ink.fill(
            p,
            ink.capsule(cam, dist, paw + up * 0.012 * hw, 0.028 * hw, 0.024 * hw),
            legs,
        );
    } else {
        ink.fill(p, ink.capsule(cam, prox, mid, 0.13 * hw, 0.06 * hw), body);
        ink.fill(p, ink.capsule(cam, mid, dist, 0.055 * hw, 0.028 * hw), legs);
        ink.fill(
            p,
            ink.capsule(cam, dist, paw + up * 0.012 * hw, 0.028 * hw, 0.024 * hw),
            legs,
        );
    }
    // paw pad, flat on the ground in stance
    let fwd = (paw - dist).x.signum().max(0.0) * 0.5 + 0.5;
    let c = paw + V2::new(0.028 * hw * fwd, 0.018 * hw);
    ink.fill(p, ink.ellipse(cam, c, 0.042 * hw, 0.02 * hw, 0.0), legs);
}

/// The trunk, neck, head and tail.
fn draw_body(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    ghost_alpha: Option<u8>,
    ink: Ink,
) {
    let sk = &sim.sk;
    let spec = &sim.params.breed;
    let hw = sk.withers_height;
    let a = |c: Color32| match ghost_alpha {
        Some(al) => with_alpha(c, al),
        None => c,
    };
    let body_c = a(rgb(spec.colors.body));
    let head_c = a(rgb(spec.colors.head));
    let pitch = f.pitch;
    let fwd = V2::new(1.0, 0.0).rot(pitch);
    let up = V2::new(0.0, 1.0).rot(pitch);
    let w = f.withers;
    let h = f.hip;
    let top = w + up * 0.07 * hw;
    let wh = w.dist(h);

    // tail first (behind the hindquarters)
    let wag = 4.0 * (2.0 * PI * f.phase).sin();
    let mut ang = rad(180.0 - spec.tail.carriage - wag) + pitch;
    let mut pt = h - fwd * 0.10 * hw + up * 0.09 * hw;
    let seg_len = spec.tail.length * hw / 7.0;
    for k in 0..7 {
        let r0 = 0.036 * hw * (1.0 - k as f64 / 8.5);
        let r1 = 0.036 * hw * (1.0 - (k + 1) as f64 / 8.5);
        let next = pt + V2::new(ang.cos(), ang.sin()) * seg_len;
        ink.fill(p, ink.capsule(cam, pt, next, r0, r1), body_c);
        pt = next;
        ang -= rad(spec.tail.curl);
    }

    // trunk: chest, abdomen, back, hindquarters
    // the chest ends just cranial to the point of the shoulder (manubrium)
    let chest_b = 0.5 * sk.chest_depth;
    let chest_a = 0.20 * hw + 0.08 * sk.trunk_length;
    let front = sk.scapula * sk.scapula_dir0.sin() + 0.05 * hw;
    let chest_c = top - up * chest_b + fwd * (front - chest_a);
    ink.fill(
        p,
        ink.ellipse(cam, chest_c, chest_a, chest_b, pitch),
        body_c,
    );
    let belly_c = w.lerp(h, 0.55) + up * (0.07 * hw - 0.42 * sk.chest_depth);
    ink.fill(
        p,
        ink.ellipse(cam, belly_c, 0.42 * wh, 0.30 * sk.chest_depth, pitch),
        body_c,
    );
    let croup = h + up * 0.11 * hw;
    ink.fill(
        p,
        ink.capsule(
            cam,
            top - up * 0.06 * hw,
            croup - up * 0.05 * hw,
            0.06 * hw,
            0.065 * hw,
        ),
        body_c,
    );
    ink.fill(
        p,
        ink.ellipse(
            cam,
            h + up * 0.03 * hw - fwd * 0.02 * hw,
            0.15 * hw,
            0.14 * hw,
            pitch,
        ),
        body_c,
    );
    if let Some(s) = spec.colors.saddle {
        let c = top.lerp(croup, 0.5) - up * 0.05 * hw;
        ink.fill(
            p,
            ink.ellipse(
                cam,
                c,
                0.40 * top.dist(croup),
                0.055 * hw,
                (croup - top).y.atan2((croup - top).x) + PI,
            ),
            a(rgb(s)),
        );
    }

    // neck and head (gaze partly stabilised against trunk pitch)
    let neck_base = w + fwd * 0.07 * hw - up * 0.06 * hw;
    let neck_ang = rad(spec.head.neck_angle) + 0.4 * pitch;
    let neck_dir = V2::new(neck_ang.cos(), neck_ang.sin());
    let neck_top = neck_base + neck_dir * spec.head.neck * hw;
    ink.fill(
        p,
        ink.capsule(cam, neck_base, neck_top, 0.12 * hw, 0.075 * hw),
        body_c,
    );
    let skull = spec.head.skull * hw;
    let head_ang = rad(-18.0) + 0.2 * pitch;
    let hd = V2::new(head_ang.cos(), head_ang.sin());
    let hu = V2::new(-hd.y, hd.x);
    let skull_c = neck_top + hd * 0.25 * skull + hu * 0.05 * skull;
    // ears behind / on top of the skull
    let ear_c = shade(head_c, 0.8);
    match spec.head.ears {
        EarType::Erect | EarType::Bat => {
            let big = if spec.head.ears == EarType::Bat {
                1.25
            } else {
                1.0
            };
            let base0 = skull_c - hd * 0.32 * skull + hu * 0.25 * skull;
            let base1 = skull_c - hd * 0.02 * skull + hu * 0.35 * skull;
            let tip = skull_c - hd * 0.30 * skull + hu * (0.95 * big) * skull;
            ink.fill(p, vec![cam.pt(base0), cam.pt(base1), cam.pt(tip)], ear_c);
        }
        EarType::Drop => {
            let c = skull_c - hd * 0.18 * skull - hu * 0.12 * skull;
            ink.fill(
                p,
                ink.ellipse(cam, c, 0.16 * skull, 0.36 * skull, head_ang + 0.2),
                ear_c,
            );
        }
        EarType::Rose => {
            let base0 = skull_c - hd * 0.3 * skull + hu * 0.25 * skull;
            let base1 = skull_c - hd * 0.05 * skull + hu * 0.32 * skull;
            let tip = skull_c - hd * 0.62 * skull + hu * 0.38 * skull;
            ink.fill(p, vec![cam.pt(base0), cam.pt(base1), cam.pt(tip)], ear_c);
        }
    }
    ink.fill(
        p,
        ink.ellipse(cam, skull_c, 0.5 * skull, 0.36 * skull, head_ang),
        head_c,
    );
    let muzzle_len = spec.head.muzzle * skull;
    let m0 = skull_c + hd * 0.25 * skull - hu * 0.08 * skull;
    let m1 = m0 + hd * (0.2 * skull + muzzle_len) - hu * 0.06 * skull;
    ink.fill(
        p,
        ink.capsule(cam, m0, m1, 0.26 * skull, 0.15 * skull),
        head_c,
    );
    let dark = a(Color32::from_rgb(24, 20, 18));
    if ink.halo() {
        return;
    }
    p.circle_filled(
        cam.pt(m1 + hd * 0.1 * skull + hu * 0.03 * skull),
        (0.08 * skull) as f32 * cam.ppm,
        dark,
    );
    p.circle_filled(
        cam.pt(skull_c + hd * 0.18 * skull + hu * 0.10 * skull),
        (0.055 * skull) as f32 * cam.ppm,
        dark,
    );
    if spec.head.ears == EarType::Drop {
        // keep the drop ear over the skull
        let c = skull_c - hd * 0.2 * skull - hu * 0.05 * skull;
        ink.fill(
            p,
            ink.ellipse(cam, c, 0.15 * skull, 0.34 * skull, head_ang + 0.25),
            ear_c,
        );
    }
}

pub fn draw_dog(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    o: &DrawOptions,
    pal: &Palette,
    ghost: bool,
) {
    let spec = &sim.params.breed;
    let ghost_alpha = if ghost { Some(90u8) } else { None };
    let a = |c: Color32| match ghost_alpha {
        Some(al) => with_alpha(c, al),
        None => c,
    };
    let body = rgb(spec.colors.body);
    let legs = rgb(spec.colors.legs);
    if o.silhouette {
        if !ghost {
            let halo = Ink {
                inflate: 1.6 / cam.ppm as f64,
                color: Some(with_alpha(pal.text_muted, 150)),
            };
            for i in 0..4 {
                draw_limb_flesh(p, cam, sim, f, i, body, legs, halo);
            }
            draw_body(p, cam, sim, f, None, halo);
        }
        for i in [1usize, 3] {
            draw_limb_flesh(
                p,
                cam,
                sim,
                f,
                i,
                a(shade(body, 0.62)),
                a(shade(legs, 0.62)),
                Ink::NORMAL,
            );
        }
        draw_body(p, cam, sim, f, ghost_alpha, Ink::NORMAL);
        for i in [0usize, 2] {
            draw_limb_flesh(p, cam, sim, f, i, a(body), a(legs), Ink::NORMAL);
        }
    }
    if o.skeleton {
        draw_skeleton(p, cam, sim, f, o, pal, ghost);
    }
    let hw = sim.sk.withers_height;
    if o.contacts && !ghost {
        for (i, l) in f.limbs.iter().enumerate() {
            if l.contact {
                let c = cam.pt(l.pts[4]);
                let half = (0.05 * hw) as f32 * cam.ppm;
                let y = c.y + 3.0 + if LIMBS[i].is_near() { 0.0 } else { 3.0 };
                p.line_segment(
                    [pos2(c.x - half, y), pos2(c.x + half, y)],
                    Stroke::new(3.0, limb_color(pal, LIMBS[i])),
                );
            }
        }
    }
    if o.grf_vectors && !ghost {
        for (i, l) in f.limbs.iter().enumerate() {
            if l.grf_bw > 0.005 {
                let base = cam.pt(l.pts[4]);
                let len = (l.grf_bw * 0.5 * hw) as f32 * cam.ppm;
                p.arrow(
                    base,
                    vec2(0.0, -len),
                    Stroke::new(2.5, limb_color(pal, LIMBS[i])),
                );
            }
        }
    }
    if o.com_marker && !ghost {
        draw_com_marker(p, cam.pt(f.com), pal);
    }
}

/// The conventional quartered-target COM symbol.
pub(crate) fn draw_com_marker(p: &Painter, c: Pos2, pal: &Palette) {
    let r = 6.0;
    p.circle_filled(c, r, pal.joint_fill);
    p.circle_stroke(c, r, Stroke::new(1.5, pal.com));
    let q = |a0: f32| {
        let mut pts = vec![c];
        for k in 0..=6 {
            let ang = a0 + k as f32 / 6.0 * std::f32::consts::FRAC_PI_2;
            pts.push(c + vec2(ang.cos(), -ang.sin()) * r);
        }
        pts
    };
    p.add(Shape::convex_polygon(q(0.0), pal.com, Stroke::NONE));
    p.add(Shape::convex_polygon(
        q(std::f32::consts::PI),
        pal.com,
        Stroke::NONE,
    ));
}

fn draw_skeleton(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    o: &DrawOptions,
    pal: &Palette,
    ghost: bool,
) {
    let alpha_far = if ghost { 70 } else { 150 };
    let alpha_near = if ghost { 110 } else { 255 };
    // spine and pelvis
    let pitch = f.pitch;
    let up = V2::new(0.0, 1.0).rot(pitch);
    let hw = sim.sk.withers_height;
    let spine_mid = f.withers.lerp(f.hip, 0.5) + up * (0.05 * hw + f.spine_flex * 0.6 * hw);
    let spine: Vec<Pos2> = (0..=12)
        .map(|k| {
            let t = k as f64 / 12.0;
            let a = f.withers.lerp(spine_mid, t);
            let b = spine_mid.lerp(f.hip + up * 0.05 * hw, t);
            cam.pt(a.lerp(b, t))
        })
        .collect();
    p.add(Shape::line(
        spine,
        Stroke::new(2.0, with_alpha(pal.bone, alpha_near)),
    ));
    for (i, limb) in LIMBS.iter().enumerate() {
        let l = &f.limbs[i];
        let near = limb.is_near();
        let col = with_alpha(
            limb_color(pal, *limb),
            if near { alpha_near } else { alpha_far },
        );
        let w = if near { 2.6 } else { 1.6 };
        let pts: Vec<Pos2> = l.pts.iter().map(|&v| cam.pt(v)).collect();
        if !limb.is_fore() {
            // pelvis: crest → hip → ischium
            let isch = l.pts[1] + (l.pts[1] - l.pts[0]) * 0.55;
            p.line_segment(
                [pts[0], cam.pt(isch)],
                Stroke::new(
                    w,
                    with_alpha(pal.bone, if near { alpha_near } else { alpha_far }),
                ),
            );
        }
        p.add(Shape::line(pts.clone(), Stroke::new(w, col)));
        for (k, &q) in pts.iter().enumerate().skip(1) {
            let r = if k == 4 {
                2.5
            } else if near {
                4.0
            } else {
                3.0
            };
            p.circle_filled(q, r, pal.joint_fill);
            p.circle_stroke(q, r, Stroke::new(1.5, col));
        }
        if o.angle_labels && near && !ghost {
            for j in 0..3 {
                let at = pts[j + 1];
                let off = if limb.is_fore() {
                    vec2(10.0, -2.0)
                } else {
                    vec2(-10.0, -2.0)
                };
                let anchor = if limb.is_fore() {
                    Align2::LEFT_CENTER
                } else {
                    Align2::RIGHT_CENTER
                };
                p.text(
                    at + off,
                    anchor,
                    format!("{:.0}°", l.angles[j]),
                    FontId::monospace(11.0),
                    pal.text,
                );
            }
        }
        if l.clamped && !ghost {
            p.circle_stroke(pts[3], 7.0, Stroke::new(1.5, pal.warn));
        }
    }
}

/// Recent paths in the current view: the near paws and near carpus/hock (side), all four paws
/// (front), plus the COM.
pub fn draw_trails(p: &Painter, cam: &Cam, sim: &Simulation, t: f64, pal: &Palette, view: View) {
    let window = if sim.is_periodic() {
        sim.period.min(2.0)
    } else {
        t.min(1.5)
    };
    let n = 90;
    let samples: Vec<Frame> = (0..=n)
        .map(|k| sim.sample(t - window + window * k as f64 / n as f64))
        .collect();
    let draw = |getter: &dyn Fn(&Frame) -> V2, col: Color32, width: f32| {
        for k in 1..samples.len() {
            let a = (k as f32 / n as f32 * 200.0) as u8;
            p.line_segment(
                [cam.pt(getter(&samples[k - 1])), cam.pt(getter(&samples[k]))],
                Stroke::new(width, with_alpha(col, a)),
            );
        }
    };
    match view {
        View::Side => {
            for i in [0usize, 2] {
                let c = pal.limb[i];
                draw(&|f: &Frame| f.limbs[i].pts[4], c, 1.6);
                draw(&|f: &Frame| f.limbs[i].pts[3], c, 1.0);
            }
        }
        View::Front => {
            for i in 0..4 {
                draw(&|f: &Frame| view.project(f.limb_pt3(i, 4)), pal.limb[i], 1.4);
            }
        }
        View::Top => {}
    }
    draw(&|f: &Frame| view.project(f.com3()), pal.com, 1.2);
}

/// Ground line, distance ticks and height guides for one lane.
pub fn draw_ground(p: &Painter, lane: Rect, cam: &Cam, pal: &Palette, max_height: f64) {
    let gy = cam.pt(V2::new(cam.origin.x, 0.0)).y;
    p.rect_filled(
        Rect::from_min_max(pos2(lane.left(), gy), lane.right_bottom()),
        0.0,
        pal.ground_fill,
    );
    // height guides every 10 cm (every 5 cm for small dogs)
    let step = if max_height < 0.35 { 0.05 } else { 0.10 };
    let mut hgt = step;
    while hgt <= max_height * 1.45 {
        let y = cam.pt(V2::new(0.0, hgt)).y;
        if y < lane.top() + 30.0 {
            break;
        }
        p.line_segment(
            [pos2(lane.left(), y), pos2(lane.right(), y)],
            Stroke::new(1.0, pal.grid),
        );
        p.text(
            pos2(lane.left() + 4.0, y - 1.0),
            Align2::LEFT_BOTTOM,
            format!("{:.0} cm", hgt * 100.0),
            FontId::proportional(10.0),
            pal.text_muted,
        );
        hgt += step;
    }
    p.line_segment(
        [pos2(lane.left(), gy), pos2(lane.right(), gy)],
        Stroke::new(1.5, pal.ground),
    );
    // distance ticks along the ground
    let x0 = cam.world_x(lane.left());
    let x1 = cam.world_x(lane.right());
    let minor = if (x1 - x0) > 6.0 { 0.5 } else { 0.1 };
    let major = if minor > 0.2 { 2.0 } else { 0.5 };
    let mut x = (x0 / minor).floor() * minor;
    while x <= x1 {
        let sx = cam.pt(V2::new(x, 0.0)).x;
        let is_major = ((x / major).round() * major - x).abs() < 1e-6;
        let len = if is_major { 7.0 } else { 3.5 };
        p.line_segment(
            [pos2(sx, gy), pos2(sx, gy + len)],
            Stroke::new(1.0, pal.ground),
        );
        if is_major {
            p.text(
                pos2(sx, gy + 9.0),
                Align2::CENTER_TOP,
                format!("{x:.1} m"),
                FontId::proportional(10.0),
                pal.text_muted,
            );
        }
        x += minor;
    }
}

pub fn draw_obstacle(p: &Painter, cam: &Cam, x: f64, h: f64, hw: f64, pal: &Palette) {
    let post_w = 0.02 * hw.max(0.3);
    let base = cam.pt(V2::new(x - post_w, 0.0));
    let topp = cam.pt(V2::new(x + post_w, h));
    p.rect_filled(
        Rect::from_two_pos(base, topp),
        1.0,
        with_alpha(pal.obstacle, 200),
    );
    let bar0 = cam.pt(V2::new(x - 4.0 * post_w, h));
    let bar1 = cam.pt(V2::new(x + 4.0 * post_w, h - post_w));
    p.rect_filled(Rect::from_two_pos(bar0, bar1), 1.0, pal.obstacle);
    p.text(
        pos2(topp.x + 6.0, topp.y),
        Align2::LEFT_BOTTOM,
        format!("{:.0} cm", h * 100.0),
        FontId::proportional(11.0),
        pal.text_muted,
    );
}
