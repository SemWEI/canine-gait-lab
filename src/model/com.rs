//! Centre-of-mass (COM) vertical motion and trunk pitch for periodic gaits, obtained by
//! integrating the estimated limb forces (impulse–momentum), so COM motion is consistent with
//! the GRF curves: walking produces the inverted-pendulum pattern (COM highest at mid-stance)
//! and running gaits the spring–mass pattern (COM lowest at mid-stance).
//!
//! The frontal plane is treated the same way. Each limb's ground force is taken to point at
//! the COM across the body, so a paw placed off the midline adds a mediolateral force
//! `−F_y · (z_paw − z̄) / h_COM`; integrating it gives lateral COM sway. Trunk roll is driven by
//! the vertical-load moment about the long axis, `Σ F_y · (z_paw − z̄)`, exactly as pitch is
//! driven by the fore–aft moment. `z̄`, the mean lateral COM position, is the load-weighted mean
//! paw position (static balance), which also makes the stride's ML impulse zero.

use super::G;

/// Fraction of the shoulder–hip paw base used as the radius of gyration for trunk pitch.
/// It stands in for the whole-body pitch inertia (head, neck and limbs included).
pub const GYRATION_FRAC: f64 = 0.6;
/// Radius of gyration for trunk roll as a fraction of withers height. Like `GYRATION_FRAC` it
/// stands in for whole-body inertia plus the passive resistance of the limbs to rolling.
pub const ROLL_GYRATION_FRAC: f64 = 0.8;

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
    /// Lateral COM displacement about `z_mean` (m, + = left).
    pub dz: PeriodicTable,
    /// Lateral COM velocity (m/s).
    pub vz: PeriodicTable,
    /// Trunk roll (rad, left side up positive).
    pub roll: PeriodicTable,
    /// Mean lateral COM position (m).
    pub z_mean: f64,
}

/// Loads at one instant, all in body weights (moments in body weights × metres).
#[derive(Clone, Copy, Debug, Default)]
pub struct Loads {
    /// Total vertical GRF.
    pub f: f64,
    /// Σ GRF × fore–aft moment arm (pitch).
    pub m: f64,
    /// Σ GRF × lateral paw position.
    pub fz: f64,
}

/// Mediolateral GRF (body weights) of a limb with vertical GRF `grf` at lateral position
/// `z_paw`, for a COM at mean lateral position `z_mean` and height `com_h`.
pub fn ml_force(grf: f64, z_paw: f64, z_mean: f64, com_h: f64) -> f64 {
    -grf * (z_paw - z_mean) / com_h
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

/// Integrate the loads `forces(phase)` over one period into COM and trunk motion.
/// `gyration` / `roll_gyration` are the pitch / roll radii of gyration and `com_h` the COM
/// height (m).
pub fn body_tables(
    period: f64,
    gyration: f64,
    roll_gyration: f64,
    com_h: f64,
    forces: impl Fn(f64) -> Loads,
) -> BodyTables {
    let dt = period / N as f64;
    let loads: Vec<Loads> = (0..N).map(|i| forces(i as f64 / N as f64)).collect();
    let f_sum: f64 = loads.iter().map(|l| l.f).sum();
    let z_mean = if f_sum > 1e-9 {
        loads.iter().map(|l| l.fz).sum::<f64>() / f_sum
    } else {
        0.0
    };
    let mut acc = vec![0.0; N];
    let mut ang = vec![0.0; N];
    let mut acc_z = vec![0.0; N];
    let mut ang_roll = vec![0.0; N];
    for (i, l) in loads.iter().enumerate() {
        let lat = l.fz - z_mean * l.f;
        acc[i] = G * (l.f - 1.0);
        ang[i] = G * l.m / (gyration * gyration);
        acc_z[i] = -G * lat / com_h;
        ang_roll[i] = G * lat / (roll_gyration * roll_gyration);
    }
    let (vy, dy) = integrate_periodic(&acc, dt);
    let (_, pitch) = integrate_periodic(&ang, dt);
    let (vz, dz) = integrate_periodic(&acc_z, dt);
    let (_, roll) = integrate_periodic(&ang_roll, dt);
    BodyTables {
        dy: PeriodicTable { values: dy },
        vy: PeriodicTable { values: vy },
        pitch: PeriodicTable { values: pitch },
        dz: PeriodicTable { values: dz },
        vz: PeriodicTable { values: vz },
        roll: PeriodicTable { values: roll },
        z_mean,
    }
}
