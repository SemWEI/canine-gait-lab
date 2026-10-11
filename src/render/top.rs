//! Top (plan) view: the dog seen from above, travelling to the right with its left side toward
//! the top of the screen. Shows the footfall pattern, step width, tracking and the support
//! polygon against the COM projection.

use super::View;
use super::dog::{Cam, DrawOptions, draw_com_marker, ellipse, fill, limb_color};
use super::front::head_points;
use crate::model::LIMBS;
use crate::model::math::{V2, rad};
use crate::model::sim::{Frame, Simulation};
use crate::palette::{Palette, rgb, shade, with_alpha};
use eframe::egui::{Align2, FontId, Painter, Pos2, Rect, Shape, Stroke, pos2, vec2};

/// Limb point `k` of limb `i` in plan coordinates (x, z).
fn tp(f: &Frame, i: usize, k: usize) -> V2 {
    View::Top.project(f.limb_pt3(i, k))
}

/// Convex hull (counter-clockwise) of a handful of points.
fn hull(mut pts: Vec<V2>) -> Vec<V2> {
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: V2, a: V2, b: V2| (a - o).cross(b - o);
    let mut lower: Vec<V2> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0
        {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<V2> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0
        {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn inside(poly: &[V2], q: V2) -> bool {
    poly.len() >= 3
        && (0..poly.len()).all(|i| (poly[(i + 1) % poly.len()] - poly[i]).cross(q - poly[i]) >= 0.0)
}

/// Contacting paws as a support polygon (a line for two paws).
fn support(f: &Frame) -> Vec<V2> {
    hull(
        (0..4)
            .filter(|&i| f.limbs[i].contact)
            .map(|i| tp(f, i, 4))
            .collect(),
    )
}

/// Paw prints left over the last ~2 strides, oldest first: (limb, position, age 0..1).
fn footprints(sim: &Simulation, t: f64) -> Vec<(usize, V2, f64)> {
    let window = if sim.is_periodic() {
        (2.0 * sim.period).clamp(0.5, 4.0)
    } else {
        t.min(sim.duration)
    };
    let n = 240;
    let mut out: Vec<(usize, V2, f64)> = Vec::new();
    let mut last: [Option<V2>; 4] = [None; 4];
    for k in 0..=n {
        let tk = t - window + window * k as f64 / n as f64;
        let f = sim.sample(tk);
        for (i, l) in f.limbs.iter().enumerate() {
            if l.contact && l.grf_bw > 0.0 {
                let q = tp(&f, i, 4);
                if last[i].is_none_or(|p| p.dist(q) > 0.01 * sim.sk.withers_height) {
                    out.push((i, q, 1.0 - k as f64 / n as f64));
                    last[i] = Some(q);
                }
            } else {
                last[i] = None;
            }
        }
    }
    out
}

#[allow(clippy::too_many_arguments)]
pub fn draw_top(
    p: &Painter,
    cam: &Cam,
    sim: &Simulation,
    f: &Frame,
    t: f64,
    o: &DrawOptions,
    pal: &Palette,
    ghost: bool,
) {
    let sk = &sim.sk;
    let spec = &sim.params.breed;
    let hw = sk.withers_height;
    if o.trails && !ghost {
        for (i, q, age) in footprints(sim, t) {
            let c = with_alpha(limb_color(pal, LIMBS[i]), (40.0 + 140.0 * (1.0 - age)) as u8);
            fill(p, ellipse(cam, q, 0.035 * hw, 0.024 * hw, 0.0), c);
        }
    }
    let poly = support(f);
    if o.contacts && !ghost && poly.len() >= 2 {
        let pts: Vec<Pos2> = poly.iter().map(|&v| cam.pt(v)).collect();
        if pts.len() >= 3 {
            p.add(Shape::convex_polygon(
                pts.clone(),
                with_alpha(pal.com, 40),
                Stroke::NONE,
            ));
        }
        let mut closed = pts.clone();
        closed.push(pts[0]);
        p.add(Shape::line(closed, Stroke::new(1.2, with_alpha(pal.com, 160))));
    }

    let alpha = if ghost { 80 } else { 200 };
    let body = with_alpha(rgb(spec.colors.body), alpha);
    let legs = with_alpha(rgb(spec.colors.legs), alpha);
    let head = with_alpha(rgb(spec.colors.head), alpha);
    let z = f.com_z;
    if o.silhouette {
        // limbs first: from above they are mostly under the trunk
        for i in 0..4 {
            let pts: [V2; 5] = std::array::from_fn(|k| tp(f, i, k));
            for k in 1..4 {
                let r = if k == 1 { 0.05 } else { 0.03 } * hw;
                fill(p, super::dog::capsule(cam, pts[k], pts[k + 1], r, 0.025 * hw), legs);
            }
        }
        // tail
        let tail_len = spec.tail.length * hw * rad(spec.tail.carriage).cos().abs().max(0.3);
        let t0 = V2::new(f.hip.x - 0.08 * hw, z);
        fill(
            p,
            super::dog::capsule(cam, t0, t0 - V2::new(tail_len, 0.0), 0.03 * hw, 0.012 * hw),
            shade(body, 0.85),
        );
        // trunk: a waisted outline from croup to chest
        let half = 0.5 * sk.chest_width;
        fill(
            p,
            ellipse(
                cam,
                V2::new(f.hip.x + 0.02 * hw, z),
                0.16 * hw,
                sk.hip_half_w + 0.05 * hw,
                0.0,
            ),
            body,
        );
        fill(
            p,
            super::dog::capsule(
                cam,
                V2::new(f.hip.x, z),
                V2::new(f.withers.x - 0.05 * hw, z),
                0.8 * half,
                half,
            ),
            body,
        );
        if let Some(s) = spec.colors.saddle {
            fill(
                p,
                super::dog::capsule(
                    cam,
                    V2::new(f.hip.x + 0.05 * hw, z),
                    V2::new(f.withers.x - 0.08 * hw, z),
                    0.4 * half,
                    0.5 * half,
                ),
                with_alpha(rgb(s), alpha),
            );
        }
        // neck and head
        let [neck_base, skull_c, _m0, m1] = head_points(sim, f);
        let skull = spec.head.skull * hw;
        fill(
            p,
            super::dog::capsule(
                cam,
                V2::new(neck_base.x - 0.04 * hw, z),
                V2::new(skull_c.x, z),
                0.10 * hw,
                0.07 * hw,
            ),
            body,
        );
        fill(
            p,
            ellipse(cam, V2::new(skull_c.x, z), 0.45 * skull, 0.34 * skull, 0.0),
            head,
        );
        fill(
            p,
            super::dog::capsule(
                cam,
                V2::new(skull_c.x + 0.2 * skull, z),
                V2::new(m1.x, z),
                0.22 * skull,
                0.13 * skull,
            ),
            head,
        );
        for s in [-1.0, 1.0] {
            fill(
                p,
                ellipse(
                    cam,
                    V2::new(skull_c.x - 0.2 * skull, z + s * 0.32 * skull),
                    0.16 * skull,
                    0.10 * skull,
                    s * 0.4,
                ),
                shade(head, 0.8),
            );
        }
    }
    if o.skeleton {
        let bone = with_alpha(pal.bone, if ghost { 110 } else { 255 });
        p.line_segment(
            [cam.pt(V2::new(f.hip.x, z)), cam.pt(V2::new(f.withers.x, z))],
            Stroke::new(2.0, bone),
        );
        for (a, b) in [(0usize, 1usize), (2, 3)] {
            p.line_segment([cam.pt(tp(f, a, 1)), cam.pt(tp(f, b, 1))], Stroke::new(2.0, bone));
        }
        for (i, limb) in LIMBS.iter().enumerate() {
            let col = with_alpha(limb_color(pal, *limb), if ghost { 110 } else { 255 });
            let pts: Vec<Pos2> = (1..5).map(|k| cam.pt(tp(f, i, k))).collect();
            p.add(Shape::line(pts.clone(), Stroke::new(1.8, col)));
            for &q in &pts[..3] {
                p.circle_filled(q, 3.0, pal.joint_fill);
                p.circle_stroke(q, 3.0, Stroke::new(1.2, col));
            }
        }
    }
    // paws on top so contacts read clearly
    for (i, limb) in LIMBS.iter().enumerate() {
        let c = cam.pt(tp(f, i, 4));
        let col = limb_color(pal, *limb);
        let r = (0.025 * hw) as f32 * cam.ppm;
        let r = r.max(3.0);
        if f.limbs[i].contact && !ghost {
            p.circle_filled(c, r, col);
        } else {
            p.circle_stroke(c, r, Stroke::new(1.5, with_alpha(col, if ghost { 110 } else { 255 })));
        }
        if !ghost && o.contacts {
            p.text(
                c + vec2(0.0, if limb.is_near() { -r - 2.0 } else { r + 2.0 }),
                if limb.is_near() {
                    Align2::CENTER_BOTTOM
                } else {
                    Align2::CENTER_TOP
                },
                limb.label(),
                FontId::proportional(10.0),
                pal.text_muted,
            );
        }
    }
    if o.grf_vectors && !ghost {
        let k = (0.5 * hw) as f32 * cam.ppm;
        for (i, l) in f.limbs.iter().enumerate() {
            if l.grf_ml_bw.abs() > 0.002 {
                p.arrow(
                    cam.pt(tp(f, i, 4)),
                    vec2(0.0, -(l.grf_ml_bw as f32) * k),
                    Stroke::new(2.5, limb_color(pal, LIMBS[i])),
                );
            }
        }
    }
    if o.com_marker && !ghost {
        let c = cam.pt(View::Top.project(f.com3()));
        draw_com_marker(p, c, pal);
        if poly.len() >= 3 && !inside(&poly, View::Top.project(f.com3())) {
            p.circle_stroke(c, 10.0, Stroke::new(1.5, pal.warn));
        }
    }
}

/// Ground plane seen from above: the line of travel, lateral guides and distance ticks.
pub fn draw_ground_top(p: &Painter, lane: Rect, cam: &Cam, pal: &Palette, max_height: f64) {
    p.rect_filled(lane, 0.0, pal.ground_fill);
    let step = if max_height < 0.35 { 0.05 } else { 0.10 };
    for s in [-1.0, 1.0] {
        let mut zz = step;
        loop {
            let y = cam.pt(V2::new(0.0, s * zz)).y;
            if y < lane.top() + 30.0 || y > lane.bottom() - 24.0 {
                break;
            }
            p.line_segment(
                [pos2(lane.left(), y), pos2(lane.right(), y)],
                Stroke::new(1.0, pal.grid),
            );
            p.text(
                pos2(lane.left() + 4.0, y - 1.0),
                Align2::LEFT_BOTTOM,
                format!("{} {:.0} cm", if s > 0.0 { "L" } else { "R" }, zz * 100.0),
                FontId::proportional(10.0),
                pal.text_muted,
            );
            zz += step;
        }
    }
    // line of travel
    let my = cam.pt(V2::new(0.0, 0.0)).y;
    let mut x = lane.left();
    while x < lane.right() {
        p.line_segment(
            [pos2(x, my), pos2((x + 6.0).min(lane.right()), my)],
            Stroke::new(1.0, pal.ground),
        );
        x += 11.0;
    }
    let gy = lane.bottom() - 22.0;
    let x0 = cam.world_x(lane.left());
    let x1 = cam.world_x(lane.right());
    let minor = if (x1 - x0) > 6.0 { 0.5 } else { 0.1 };
    let major = if minor > 0.2 { 2.0 } else { 0.5 };
    let mut x = (x0 / minor).floor() * minor;
    while x <= x1 {
        let sx = cam.pt(V2::new(x, 0.0)).x;
        let is_major = ((x / major).round() * major - x).abs() < 1e-6;
        p.line_segment(
            [pos2(sx, gy), pos2(sx, gy + if is_major { 7.0 } else { 3.5 })],
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
