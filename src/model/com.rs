//! Centre-of-mass (COM) vertical motion and trunk pitch for periodic gaits, obtained by
//! integrating the estimated limb forces (impulse–momentum), so COM motion is consistent with
//! the GRF curves: walking produces the inverted-pendulum pattern (COM highest at mid-stance)
//! and running gaits the spring–mass pattern (COM lowest at mid-stance).

use super::G;

/// Fraction of the shoulder–hip paw base used as the radius of gyration for trunk pitch.
/// It stands in for the whole-body pitch inertia (head, neck and limbs included).
pub const GYRATION_FRAC: f64 = 0.6;

#[derive(Clone, Debug)]
pub struct PeriodicTable {
    pub values: Vec<f64>,
}

impl PeriodicTable {
    pub fn at(&self, phase: f64) -> f64 {
        let n = self.values.len();
        let x = super::math::wrap01(phase) * n as f64;
        let i = x.floor() as usize % n;
        let j = (i + 1) % n;
        let t = x - x.floor();
        self.values[i] * (1.0 - t) + self.values[j] * t
    }
}

#[derive(Clone, Debug)]
pub struct BodyTables {
    /// Vertical COM displacement about its mean (m).
    pub dy: PeriodicTable,
    /// Vertical COM velocity (m/s).
    pub vy: PeriodicTable,
    /// Trunk pitch about its mean (rad, nose-up positive).
    pub pitch: PeriodicTable,
}

const N: usize = 2048;

/// Integrate twice over one period with zero-mean, drift-free boundary conditions.
fn integrate_periodic(acc: &[f64], dt: f64) -> (Vec<f64>, Vec<f64>) {
    let cumulate = |x: &[f64]| -> Vec<f64> {
        let n = x.len();
        let mut out = vec![0.0; n + 1];
        for i in 0..n {
            out[i + 1] = out[i] + 0.5 * (x[i] + x[(i + 1) % n]) * dt;
        }
        // remove closing error linearly, then the mean
        let drift = out[n];
        let mut v: Vec<f64> = (0..n)
            .map(|i| out[i] - drift * i as f64 / n as f64)
            .collect();
        let mean = v.iter().sum::<f64>() / n as f64;
        v.iter_mut().for_each(|e| *e -= mean);
        v
    };
    let vel = cumulate(acc);
    let pos = cumulate(&vel);
    (vel, pos)
}

/// `forces(phase)` returns (total vertical GRF in body weights, Σ GRF_bw × moment arm in m).
pub fn body_tables(period: f64, gyration: f64, forces: impl Fn(f64) -> (f64, f64)) -> BodyTables {
    let dt = period / N as f64;
    let mut acc = vec![0.0; N];
    let mut ang = vec![0.0; N];
    for i in 0..N {
        let (f, m) = forces(i as f64 / N as f64);
        acc[i] = G * (f - 1.0);
        ang[i] = G * m / (gyration * gyration);
    }
    let (vy, dy) = integrate_periodic(&acc, dt);
    let (_, pitch) = integrate_periodic(&ang, dt);
    BodyTables {
        dy: PeriodicTable { values: dy },
        vy: PeriodicTable { values: vy },
        pitch: PeriodicTable { values: pitch },
    }
}
