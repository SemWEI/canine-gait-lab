//! Estimated vertical ground-reaction force (GRF) profiles.
//!
//! Each limb's stance profile is a normalised shape scaled so that, over one stride, the four
//! limbs together deliver an impulse equal to body weight × stride period (no net vertical
//! acceleration), with the fore/hind split equal to the static weight distribution.
//! This is an estimate for orientation only; it is not a substitute for force-plate data.

use super::gait::GrfShape;
use std::f64::consts::PI;

/// Normalised stance shape on s ∈ [0,1] with unit integral.
pub fn shape(kind: GrfShape, s: f64) -> f64 {
    if !(0.0..=1.0).contains(&s) {
        return 0.0;
    }
    match kind {
        GrfShape::Constant => 1.0,
        GrfShape::HalfSine => 0.5 * PI * (PI * s).sin(),
        GrfShape::DoubleHump { a } => {
            let raw = (PI * s).sin() + a * (3.0 * PI * s).sin();
            let integral = (2.0 / PI) * (1.0 + a / 3.0);
            raw / integral
        }
    }
}

/// Vertical GRF of one limb in units of body weight.
/// `weight_share` is that limb's share of the stride impulse (e.g. 0.30 for each forelimb
/// with a 60/40 split) and `duty` the stance fraction.
pub fn limb_grf_bw(kind: GrfShape, weight_share: f64, duty: f64, stance_progress: f64) -> f64 {
    if duty >= 0.999 {
        return weight_share;
    }
    weight_share / duty * shape(kind, stance_progress)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn integral(kind: GrfShape) -> f64 {
        let n = 10_000;
        (0..n)
            .map(|i| shape(kind, (i as f64 + 0.5) / n as f64))
            .sum::<f64>()
            / n as f64
    }

    #[test]
    fn shapes_have_unit_integral() {
        for k in [
            GrfShape::HalfSine,
            GrfShape::DoubleHump { a: 0.3 },
            GrfShape::Constant,
        ] {
            assert!((integral(k) - 1.0).abs() < 1e-4, "{k:?}");
        }
    }

    #[test]
    fn walk_profile_dips_at_midstance() {
        let k = GrfShape::DoubleHump { a: 0.3 };
        assert!(shape(k, 0.5) < shape(k, 0.27));
    }
}
