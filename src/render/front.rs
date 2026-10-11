//! Front view (frontal plane): the dog walking toward the viewer, its left on screen right.
//! Shows track and step width, lateral COM sway, trunk roll and limb ab/adduction.

use super::View;
use super::dog::{Cam, DrawOptions, Ink, draw_com_marker, limb_color};
use crate::model::LIMBS;
use crate::model::breed::EarType;
use crate::model::math::{V2, rad};
use crate::model::sim::{Frame, Simulation};
use crate::palette::{Palette, rgb, shade, with_alpha};
use eframe::egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, pos2, vec2};

/// Limb point `k` of limb `i` in front-view coordinates (z, y).
fn fp(f: &Frame, i: usize, k: usize) -> V2 {
    View::Front.project(f.limb_pt3(i, k))
}

/// A point fixed to the trunk, `z` from the midline at height `y`, rolled with the trunk about
/// its long axis through the COM.
fn rolled(f: &Frame, z: f64, y: f64) -> V2 {
    V2::new(f.com_z, f.com.y) + V2::new(z, y - f.com.y).rot(f.roll)
}

/// Sagittal positions of the neck base, skull centre and muzzle base / tip, from the same head
/// pose the side view draws.
pub(crate) fn head_points(sim: &Simulation, f: &Frame) -> [V2; 4] {
    let spec = &sim.params.breed;
    let hw = sim.sk.withers_height;
    let pitch = f.pitch;
    let fwd = V2::new(1.0, 0.0).rot(pitch);
    let up = V2::new(0.0, 1.0).rot(pitch);
    let neck_base = f.withers + fwd * 0.07 * hw - up * 0.06 * hw;
    let neck_ang = rad(spec.head.neck_angle) + 0.4 * pitch;
    let neck_top = neck_base + V2::new(neck_ang.cos(), neck_ang.sin()) * spec.head.neck * hw;
    let skull = spec.head.skull * hw;
    let head_ang = rad(-18.0) + 0.2 * pitch;
    let hd = V2::new(head_ang.cos(), head_ang.sin());
    let hu = V2::new(-hd.y, hd.x);
    let skull_c = neck_top + hd * 0.25 * skull + hu * 0.05 * skull;
    let m0 = skull_c + hd * 0.25 * skull - hu * 0.08 * skull;
    let m1 = m0 + hd * (0.2 * skull + spec.head.muzzle * skull) - hu * 0.06 * skull;
    [neck_base, skull_c, m0, m1]
}

#[allow(clippy::too_many_arguments)]
fn draw_limb(
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
    let [top, prox, mid, dist, paw] = std::array::from_fn(|k| fp(f, i, k));
    let up = V2::new(0.0, 1.0);
    if LIMBS[i].is_fore() {
        ink.fill(
            p,
            ink.capsule(cam, top - up * 0.06 * hw, prox, 0.07 * hw, 0.07 * hw),
            body,
        );
        ink.fill(p, ink.capsule(cam, prox, mid, 0.065 * hw, 0.045 * hw), body);
        ink.fill(p, ink.capsule(cam, mid, dist, 0.04 * hw, 0.028 * hw), legs);
    } else {
        ink.fill(p, ink.capsule(cam, prox, mid, 0.10 * hw, 0.055 * hw), body);
        ink.fill(p, ink.capsule(cam, mid, dist, 0.045 * hw, 0.026 * hw), legs);
    }
    ink.fill(
        p,
        ink.capsule(cam, dist, paw + up * 0.012 * hw, 0.026 * hw, 0.024 * hw),
        legs,
    );
    // paw seen end-on
    ink.fill(
        p,
        ink.ellipse(cam, paw + up * 0.018 * hw, 0.032 * hw, 0.02 * hw, 0.0),
        legs,
    );
}

fn draw_body(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    ghost_alpha: Option<u8>,
    ink: Ink,
    hind_then: &dyn Fn(Ink),
) {
    let sk = &sim.sk;
    let spec = &sim.params.breed;
    let hw = sk.withers_height;
    let a = |c: Color32| match ghost_alpha {
        Some(al) => with_alpha(c, al),
        None => c,
    };
    let body_c = a(rgb(spec.colors.body));

    // a high-carried tail shows above the back
    if spec.tail.carriage > 30.0 {
        let base = rolled(f, 0.0, f.hip.y + 0.09 * hw);
        let rise = spec.tail.length * hw * rad(spec.tail.carriage).sin() * 0.8;
        ink.fill(
            p,
            ink.capsule(
                cam,
                base,
                V2::new(base.x, base.y + rise),
                0.03 * hw,
                0.015 * hw,
            ),
            shade(body_c, 0.85),
        );
    }
    // hindquarters, behind the hindlimbs' upper ends
    ink.fill(
        p,
        ink.ellipse(
            cam,
            rolled(f, 0.0, f.hip.y + 0.03 * hw),
            sk.hip_half_w + 0.06 * hw,
            0.14 * hw,
            f.roll,
        ),
        shade(body_c, 0.8),
    );
    hind_then(ink);

    // chest cross-section
    let top_y = f.withers.y + 0.07 * hw;
    let chest_b = 0.5 * sk.chest_depth;
    ink.fill(
        p,
        ink.ellipse(
            cam,
            rolled(f, 0.0, top_y - chest_b),
            0.5 * sk.chest_width,
            chest_b,
            f.roll,
        ),
        body_c,
    );
    if let Some(s) = spec.colors.saddle {
        ink.fill(
            p,
            ink.ellipse(
                cam,
                rolled(f, 0.0, top_y - 0.06 * hw),
                0.32 * sk.chest_width,
                0.05 * hw,
                f.roll,
            ),
            a(rgb(s)),
        );
    }
}

fn draw_head(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    ghost_alpha: Option<u8>,
    ink: Ink,
) {
    let spec = &sim.params.breed;
    let hw = sim.sk.withers_height;
    let a = |c: Color32| match ghost_alpha {
        Some(al) => with_alpha(c, al),
        None => c,
    };
    let body_c = a(rgb(spec.colors.body));
    let head_c = a(rgb(spec.colors.head));
    let ear_c = shade(head_c, 0.8);
    let skull = spec.head.skull * hw;
    let [neck_base, skull_c, m0, m1] = head_points(sim, f);
    let (neck_y, skull_y, muzzle_y) = (neck_base.y, skull_c.y, 0.5 * (m0.y + m1.y));
    let z = f.com_z;

    // neck: from the rolled trunk up to the (gaze-stabilised) head
    let base = rolled(f, 0.0, neck_y);
    ink.fill(
        p,
        ink.capsule(cam, base, V2::new(z, skull_y), 0.11 * hw, 0.07 * hw),
        body_c,
    );
    let top = skull_y + 0.30 * skull;
    let tri = |pts: [V2; 3]| pts.iter().map(|&v| cam.pt(v)).collect::<Vec<Pos2>>();
    for s in [-1.0, 1.0] {
        match spec.head.ears {
            EarType::Erect | EarType::Bat => {
                let big = if spec.head.ears == EarType::Bat {
                    1.3
                } else {
                    1.0
                };
                ink.fill(
                    p,
                    tri([
                        V2::new(z + s * 0.10 * skull, top - 0.04 * skull),
                        V2::new(z + s * 0.34 * skull, top - 0.14 * skull),
                        V2::new(z + s * 0.36 * big * skull, top + 0.62 * big * skull),
                    ]),
                    ear_c,
                );
            }
            EarType::Rose => {
                ink.fill(
                    p,
                    tri([
                        V2::new(z + s * 0.18 * skull, top - 0.02 * skull),
                        V2::new(z + s * 0.36 * skull, top - 0.12 * skull),
                        V2::new(z + s * 0.56 * skull, top + 0.14 * skull),
                    ]),
                    ear_c,
                );
            }
            EarType::Drop => {}
        }
    }
    ink.fill(
        p,
        ink.ellipse(cam, V2::new(z, skull_y), 0.36 * skull, 0.32 * skull, 0.0),
        head_c,
    );
    if spec.head.ears == EarType::Drop {
        for s in [-1.0, 1.0] {
            ink.fill(
                p,
                ink.ellipse(
                    cam,
                    V2::new(z + s * 0.36 * skull, skull_y - 0.12 * skull),
                    0.12 * skull,
                    0.34 * skull,
                    s * 0.15,
                ),
                ear_c,
            );
        }
    }
    // muzzle seen end-on: short muzzles are relatively wider
    let mw = (0.18 + 0.06 / spec.head.muzzle.max(0.3)) * skull;
    let muzzle_c = V2::new(z, muzzle_y.min(skull_y - 0.12 * skull));
    ink.fill(
        p,
        ink.ellipse(cam, muzzle_c, mw.min(0.3 * skull), 0.17 * skull, 0.0),
        head_c,
    );
    if ink.halo() {
        return;
    }
    let dark = a(Color32::from_rgb(24, 20, 18));
    p.circle_filled(
        cam.pt(muzzle_c + V2::new(0.0, 0.05 * skull)),
        (0.075 * skull) as f32 * cam.ppm,
        dark,
    );
    for s in [-1.0, 1.0] {
        p.circle_filled(
            cam.pt(V2::new(z + s * 0.16 * skull, skull_y + 0.06 * skull)),
            (0.05 * skull) as f32 * cam.ppm,
            dark,
        );
    }
}

fn draw_silhouette(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    ghost_alpha: Option<u8>,
    ink: Ink,
) {
    let spec = &sim.params.breed;
    let a = |c: Color32| match ghost_alpha {
        Some(al) => with_alpha(c, al),
        None => c,
    };
    let body = a(rgb(spec.colors.body));
    let legs = a(rgb(spec.colors.legs));
    // hindlimbs are the far pair in this view
    let hind = |ink: Ink| {
        for i in [2usize, 3] {
            draw_limb(p, cam, sim, f, i, shade(body, 0.7), shade(legs, 0.7), ink);
        }
    };
    draw_body(p, cam, sim, f, ghost_alpha, ink, &hind);
    for i in [0usize, 1] {
        draw_limb(p, cam, sim, f, i, body, legs, ink);
    }
    draw_head(p, cam, sim, f, ghost_alpha, ink);
}

pub fn draw_front(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    o: &DrawOptions,
    pal: &Palette,
    ghost: bool,
) {
    let ghost_alpha = if ghost { Some(90u8) } else { None };
    if o.silhouette {
        if !ghost {
            let halo = Ink {
                inflate: 1.6 / cam.ppm as f64,
                color: Some(with_alpha(pal.text_muted, 150)),
            };
            draw_silhouette(p, cam, sim, f, None, halo);
        }
        draw_silhouette(p, cam, sim, f, ghost_alpha, Ink::NORMAL);
    }
    if o.skeleton {
        draw_skeleton(p, cam, f, o, pal, ghost);
    }
    let hw = sim.sk.withers_height;
    if o.contacts && !ghost {
        for (i, l) in f.limbs.iter().enumerate() {
            if l.contact {
                let c = cam.pt(fp(f, i, 4));
                let half = (0.03 * hw) as f32 * cam.ppm;
                let y = c.y + 3.0 + if LIMBS[i].is_fore() { 0.0 } else { 3.0 };
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
                let k = (0.5 * hw) as f32 * cam.ppm;
                p.arrow(
                    cam.pt(fp(f, i, 4)),
                    vec2(l.grf_ml_bw as f32 * k, -(l.grf_bw as f32) * k),
                    Stroke::new(2.5, limb_color(pal, LIMBS[i])),
                );
            }
        }
    }
    if o.com_marker && !ghost {
        draw_com_marker(p, cam.pt(View::Front.project(f.com3())), pal);
    }
    if !ghost {
        draw_step_width(p, cam, f, pal);
    }
}

fn draw_skeleton(p: &Painter, cam: &Cam, f: &Frame, o: &DrawOptions, pal: &Palette, ghost: bool) {
    let alpha_far = if ghost { 70 } else { 150 };
    let alpha_near = if ghost { 110 } else { 255 };
    // shoulder and pelvic girdles: their tilt is the trunk roll
    for (a, b, near) in [(0usize, 1usize, true), (2, 3, false)] {
        let al = if near { alpha_near } else { alpha_far };
        p.line_segment(
            [cam.pt(fp(f, a, 1)), cam.pt(fp(f, b, 1))],
            Stroke::new(2.0, with_alpha(pal.bone, al)),
        );
    }
    for (i, limb) in LIMBS.iter().enumerate() {
        let near = limb.is_fore();
        let col = with_alpha(
            limb_color(pal, *limb),
            if near { alpha_near } else { alpha_far },
        );
        let w = if near { 2.6 } else { 1.6 };
        let pts: Vec<Pos2> = (1..5).map(|k| cam.pt(fp(f, i, k))).collect();
        p.add(Shape::line(pts.clone(), Stroke::new(w, col)));
        for (k, &q) in pts.iter().enumerate() {
            let r = if k == 3 {
                2.5
            } else if near {
                4.0
            } else {
                3.0
            };
            p.circle_filled(q, r, pal.joint_fill);
            p.circle_stroke(q, r, Stroke::new(1.5, col));
        }
        if o.angle_labels && !ghost {
            // left limbs are on screen right, so label outward on each side
            let out = limb.side() as f32;
            let at = if near { pts[0] } else { pts[2] };
            p.text(
                at + vec2(12.0 * out, -2.0),
                if out > 0.0 {
                    Align2::LEFT_CENTER
                } else {
                    Align2::RIGHT_CENTER
                },
                format!("{} {:+.1}°", limb.label(), f.limbs[i].abduction),
                FontId::monospace(11.0),
                pal.text,
            );
        }
        if f.limbs[i].clamped && !ghost {
            p.circle_stroke(pts[2], 7.0, Stroke::new(1.5, pal.warn));
        }
    }
}

/// Dimension lines under the ground for the current fore and hind step width.
fn draw_step_width(p: &Painter, cam: &Cam, f: &Frame, pal: &Palette) {
    let gy = cam.pt(V2::new(0.0, 0.0)).y;
    for (row, (l, r, name)) in [(0usize, 1usize, "fore"), (2, 3, "hind")]
        .into_iter()
        .enumerate()
    {
        let (zl, zr) = (f.limbs[l].lat[4], f.limbs[r].lat[4]);
        let y = gy + 26.0 + 13.0 * row as f32;
        let (a, b) = (cam.pt(V2::new(zr, 0.0)).x, cam.pt(V2::new(zl, 0.0)).x);
        let st = Stroke::new(1.0, pal.text_muted);
        p.line_segment([pos2(a, y), pos2(b, y)], st);
        p.line_segment([pos2(a, y - 3.0), pos2(a, y + 3.0)], st);
        p.line_segment([pos2(b, y - 3.0), pos2(b, y + 3.0)], st);
        p.text(
            pos2(b.max(a) + 6.0, y),
            Align2::LEFT_CENTER,
            format!("{name} {:.1} cm", (zl - zr).abs() * 100.0),
            FontId::proportional(10.0),
            pal.text_muted,
        );
    }
}

/// Ground strip, height guides, the line of travel and lateral distance ticks.
pub fn draw_ground_front(p: &Painter, lane: Rect, cam: &Cam, pal: &Palette, max_height: f64) {
    let gy = cam.pt(V2::new(0.0, 0.0)).y;
    p.rect_filled(
        Rect::from_min_max(pos2(lane.left(), gy), lane.right_bottom()),
        0.0,
        pal.ground_fill,
    );
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
    // line of travel (midline)
    let mx = cam.pt(V2::new(0.0, 0.0)).x;
    let top = cam.pt(V2::new(0.0, max_height * 1.3)).y.max(lane.top() + 30.0);
    let mut y = top;
    while y < gy {
        p.line_segment(
            [pos2(mx, y), pos2(mx, (y + 5.0).min(gy))],
            Stroke::new(1.0, pal.grid),
        );
        y += 9.0;
    }
    let minor = if max_height < 0.35 { 0.01 } else { 0.02 };
    let major = minor * 5.0;
    let z0 = cam.world_x(lane.left());
    let z1 = cam.world_x(lane.right());
    let mut z = (z0 / minor).floor() * minor;
    while z <= z1 {
        let sx = cam.pt(V2::new(z, 0.0)).x;
        let is_major = ((z / major).round() * major - z).abs() < 1e-6;
        p.line_segment(
            [pos2(sx, gy), pos2(sx, gy + if is_major { 7.0 } else { 3.5 })],
            Stroke::new(1.0, pal.ground),
        );
        if is_major {
            let label = if z.abs() < 1e-6 {
                "0".to_string()
            } else {
                format!("{}{:.0} cm", if z > 0.0 { "L " } else { "R " }, z.abs() * 100.0)
            };
            p.text(
                pos2(sx, gy + 9.0),
                Align2::CENTER_TOP,
                label,
                FontId::proportional(10.0),
                pal.text_muted,
            );
        }
        z += minor;
    }
}
