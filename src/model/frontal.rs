//! Frontal-plane (mediolateral) part of the model: paw placement across the line of travel,
//! the tilted plane each limb flexes in, and the lateral swing arc.
//!
//! Coordinates: z is lateral, positive toward the dog's left. Each limb flexes and extends in a
//! plane that contains its proximal joint (shoulder / hip) and its paw and is parallel to the
//! direction of travel; that plane is tilted from vertical by the limb's ab/adduction. The
//! sagittal two-link solution is computed inside this plane and mapped back to 3-D, so segment
//! lengths are exact and flexion angles are measured where the joints actually flex.
//!
//! Step-width narrowing with speed (paws converging toward the midline, "single tracking")
//! follows the qualitative pattern reported for dogs; the constants below are approximations
//! chosen for orientation. Verify against primary sources before citing.

use super::breed::Skeleton;
use super::gait::Gait;
use super::math::V2;
use std::f64::consts::PI;

/// Peak outward swing arc (circumduction), as a fraction of withers height.
const SWING_ARC: f64 = 0.012;

/// (floor, Froude scale) of the step-width factor `floor + (1 − floor)·exp(−Fr / scale)`.
fn convergence(gait: Gait) -> (f64, f64) {
    match gait {
        Gait::Stand | Gait::StandingJump | Gait::RunningJump => (1.0, 1.0),
        Gait::Walk => (0.75, 1.0),
        Gait::Trot => (0.35, 1.5),
        // pacing dogs keep a wider base: the ipsilateral pairs cannot single-track
        Gait::Pace => (0.60, 1.5),
        Gait::Canter | Gait::TransverseGallop | Gait::RotaryGallop => (0.30, 2.0),
    }
}

/// Lateral distance of the paw centre from the line of travel during stance (m).
pub fn paw_half_base(sk: &Skeleton, gait: Gait, fr: f64, fore: bool) -> f64 {
    let base = if fore {
        sk.fore_paw_half_base
    } else {
        sk.hind_paw_half_base
    };
    let (floor, scale) = convergence(gait);
    base * (floor + (1.0 - floor) * (-fr.max(0.0) / scale).exp())
}

/// Outward lateral excursion of a swinging paw at swing progress `s` (m, always ≥ 0).
pub fn swing_lateral(s: f64, withers_height: f64) -> f64 {
    SWING_ARC * withers_height * (PI * s.clamp(0.0, 1.0)).sin().powi(2)
}

/// The tilted plane a limb flexes in. In-plane coordinates are (x, y_ip) with the same x as the
/// world and y_ip measured along the tilted vertical through the plane origin, so the sagittal
/// kinematics code can work in it unchanged.
#[derive(Clone, Copy, Debug, Default)]
pub struct LimbPlane {
    /// World height and lateral position of the plane origin (the proximal joint).
    pub oy: f64,
    pub oz: f64,
    /// Tilt from vertical, radians: positive when the distal limb points toward +z (left).
    pub tilt: f64,
}

impl LimbPlane {
    /// Plane through the proximal joint at (`oy`, `oz`) and the paw at (`py`, `pz`).
    /// `min_drop` keeps the tilt bounded while a paw is lifted high in swing.
    pub fn through(oy: f64, oz: f64, py: f64, pz: f64, min_drop: f64) -> Self {
        let drop = (oy - py).max(min_drop);
        Self {
            oy,
            oz,
            tilt: (pz - oz).atan2(drop),
        }
    }

    /// World point (sagittal projection x, y) → in-plane point.
    pub fn to_plane(self, p: V2) -> V2 {
        V2::new(p.x, self.oy - (self.oy - p.y) / self.tilt.cos())
    }

    /// In-plane point → world (sagittal projection, lateral z).
    pub fn to_world(self, q: V2) -> (V2, f64) {
        let h = self.oy - q.y;
        let (s, c) = self.tilt.sin_cos();
        (V2::new(q.x, self.oy - h * c), self.oz + h * s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::breed::embedded_breeds;

    #[test]
    fn plane_maps_paw_exactly_and_preserves_lengths() {
        let pl = LimbPlane::through(0.40, 0.08, 0.0, 0.05, 0.1);
        let paw = pl.to_plane(V2::new(0.1, 0.0));
        let (w, z) = pl.to_world(paw);
        assert!(w.y.abs() < 1e-12 && (z - 0.05).abs() < 1e-12);
        // isometry: distances in the plane equal distances in 3-D
        let a = V2::new(0.0, 0.3);
        let b = V2::new(0.12, 0.1);
        let ((wa, za), (wb, zb)) = (pl.to_world(a), pl.to_world(b));
        let d3 = ((wa.x - wb.x).powi(2) + (wa.y - wb.y).powi(2) + (za - zb).powi(2)).sqrt();
        assert!((d3 - a.dist(b)).abs() < 1e-12);
    }

    #[test]
    fn step_width_narrows_with_speed() {
        let b = embedded_breeds().into_iter().next().unwrap();
        let sk = Skeleton::new(&b, b.withers_height_m);
        let slow = paw_half_base(&sk, Gait::Trot, 0.4, true);
        let fast = paw_half_base(&sk, Gait::Trot, 2.5, true);
        assert!(fast < slow && fast > 0.0);
        assert!(paw_half_base(&sk, Gait::Pace, 1.5, true) > paw_half_base(&sk, Gait::Trot, 1.5, true));
    }
}
