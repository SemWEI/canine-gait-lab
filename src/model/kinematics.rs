//! Limb kinematics: two-link inverse kinematics, joint-angle conventions and the template that
//! resolves the redundant distal (metacarpal / metatarsal) segment.
//!
//! Joint angle convention: the included angle between adjacent segments measured on the
//! flexor side, in degrees. 180° means the segments are collinear; values above 180° mean
//! hyperextension (the canine carpus is normally hyperextended under load).

use super::math::{V2, ccw_angle, deg, periodic_spline, rad};
use std::f64::consts::TAU;

/// Which side of the limb axis the middle joint bulges toward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bend {
    /// Elbow: points caudally.
    Caudal,
    /// Stifle: points cranially.
    Cranial,
}

#[derive(Clone, Copy, Debug)]
pub struct IkResult {
    pub mid: V2,
    /// Distal joint actually reached (differs from the target only when clamped).
    pub end: V2,
    pub clamped: bool,
}

/// Solve a planar two-link chain from `a` toward `b`.
pub fn two_link(a: V2, b: V2, l1: f64, l2: f64, bend: Bend) -> IkResult {
    let ab = b - a;
    let d_raw = ab.len();
    let d_min = (l1 - l2).abs() + 1e-6;
    let d_max = l1 + l2 - 1e-6;
    let d = d_raw.clamp(d_min, d_max);
    let clamped = (d - d_raw).abs() > 1e-9;
    let u = ab.norm();
    let end = a + u * d;
    let cos_a = ((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d)).clamp(-1.0, 1.0);
    let ang = cos_a.acos();
    // With u pointing roughly down, a counter-clockwise turn swings toward +x (cranial).
    let turn = match bend {
        Bend::Cranial => ang,
        Bend::Caudal => -ang,
    };
    IkResult {
        mid: a + u.rot(turn) * l1,
        end,
        clamped,
    }
}

/// Flexor side of each joint in the order shoulder, elbow, carpus / hip, stifle, tarsus.
/// `true` = cranial flexor side.
pub const FORE_FLEXOR_CRANIAL: [bool; 3] = [false, true, false];
pub const HIND_FLEXOR_CRANIAL: [bool; 3] = [true, false, true];

/// Included angle at `joint` between the proximal and distal segments, measured on the
/// flexor side (degrees).
pub fn joint_angle(joint: V2, proximal: V2, distal: V2, flexor_cranial: bool) -> f64 {
    let ccw = ccw_angle(distal - joint, proximal - joint);
    deg(if flexor_cranial { ccw } else { TAU - ccw })
}

/// Inverse of [`joint_angle`]: unit direction of the distal segment given the proximal
/// segment direction (joint → proximal point) and the joint angle.
pub fn distal_dir_from_angle(prox_dir: V2, angle_deg: f64, flexor_cranial: bool) -> V2 {
    let th = rad(angle_deg);
    if flexor_cranial {
        prox_dir.rot(-th).norm()
    } else {
        prox_dir.rot(th).norm()
    }
}

/// Distal segment orientation keys as offsets (degrees) from the breed's standing angle.
/// `(progress, offset)` pairs for stance then swing; both start at touchdown/lift-off.
/// Shapes follow the qualitative pattern of canine carpal/tarsal kinematics: carpal
/// hyperextension at mid-stance, heel-off before lift-off, deep carpal flexion in early swing
/// and re-extension before touchdown.
const FORE_STANCE_KEYS: [(f64, f64); 4] = [(0.0, -3.0), (0.5, 3.0), (0.8, -10.0), (1.0, -50.0)];
const FORE_SWING_KEYS: [(f64, f64); 2] = [(0.3, -95.0), (0.7, -25.0)];
const HIND_STANCE_KEYS: [(f64, f64); 4] = [(0.0, 1.0), (0.5, -3.0), (0.8, -13.0), (1.0, -40.0)];
const HIND_SWING_KEYS: [(f64, f64); 2] = [(0.35, -60.0), (0.75, -10.0)];

/// Distal segment angle offset (radians) for a limb phase `u` measured from touchdown, given
/// duty factor `duty`.
pub fn distal_template(fore: bool, duty: f64, u: f64) -> f64 {
    let (st, sw) = if fore {
        (&FORE_STANCE_KEYS[..], &FORE_SWING_KEYS[..])
    } else {
        (&HIND_STANCE_KEYS[..], &HIND_SWING_KEYS[..])
    };
    if duty >= 0.999 {
        return rad(st[1].1);
    }
    let mut keys: Vec<(f64, f64)> = Vec::with_capacity(6);
    for &(s, v) in st {
        keys.push((s * duty, v));
    }
    for &(s, v) in sw {
        keys.push((duty + s * (1.0 - duty), v));
    }
    rad(periodic_spline(&keys, u))
}

/// Distal offset within a stance phase (progress 0..1) — used by non-periodic motions.
pub fn distal_stance(fore: bool, s: f64) -> f64 {
    let st = if fore {
        &FORE_STANCE_KEYS[..]
    } else {
        &HIND_STANCE_KEYS[..]
    };
    let s = s.clamp(0.0, 1.0);
    for w in st.windows(2) {
        if s <= w[1].0 {
            let t = (s - w[0].0) / (w[1].0 - w[0].0);
            let t = super::math::cos_ramp(t);
            return rad(w[0].1 + (w[1].1 - w[0].1) * t);
        }
    }
    rad(st[st.len() - 1].1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ik_round_trip() {
        let a = V2::new(0.1, 0.5);
        for &(bx, by) in &[(0.12, 0.15), (0.25, 0.2), (-0.1, 0.1), (0.0, 0.3)] {
            let b = V2::new(bx, by);
            for bend in [Bend::Caudal, Bend::Cranial] {
                let r = two_link(a, b, 0.22, 0.24, bend);
                assert!(!r.clamped);
                assert!((r.mid.dist(a) - 0.22).abs() < 1e-9);
                assert!((r.mid.dist(b) - 0.24).abs() < 1e-9);
                // forward kinematics through the joint angle reproduces the distal joint
                let ang = joint_angle(r.mid, a, b, bend == Bend::Caudal);
                let dir = distal_dir_from_angle((a - r.mid).norm(), ang, bend == Bend::Caudal);
                let b2 = r.mid + dir * 0.24;
                assert!(b2.dist(b) < 1e-9, "{b2:?} vs {b:?}");
            }
        }
    }

    #[test]
    fn bend_direction_matches_anatomy() {
        let a = V2::new(0.0, 0.5);
        let b = V2::new(0.0, 0.1);
        let elbow = two_link(a, b, 0.25, 0.25, Bend::Caudal);
        let stifle = two_link(a, b, 0.25, 0.25, Bend::Cranial);
        assert!(elbow.mid.x < 0.0);
        assert!(stifle.mid.x > 0.0);
    }

    #[test]
    fn standing_angles_are_physiological() {
        let d = |a: f64| V2::from_down_angle(rad(a));
        let w = V2::new(0.0, 1.0);
        let sh = w + d(35.0);
        let el = sh + d(-35.0);
        let ca = el + d(0.0);
        let pf = ca + d(15.0);
        let shoulder = joint_angle(sh, w, el, false);
        let elbow = joint_angle(el, sh, ca, true);
        let carpus = joint_angle(ca, el, pf, false);
        assert!((shoulder - 110.0).abs() < 1e-6, "{shoulder}");
        assert!((elbow - 145.0).abs() < 1e-6, "{elbow}");
        assert!((carpus - 195.0).abs() < 1e-6, "{carpus}");
    }
}
