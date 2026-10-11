//! CSV export of simulated series. The column names are the same ones the importer reads,
//! so an export can be re-imported for comparison.

use crate::model::sim::{Frame, Simulation};
use crate::model::{LIMBS, MODEL_VERSION};
use std::io::Write;

#[derive(Clone, Copy, Debug)]
pub struct ExportOptions {
    pub hz: f64,
    /// Number of cycles to write for periodic gaits (ignored for jumps).
    pub cycles: usize,
    /// Deterministic pseudo-noise added to joint angles (degrees, s.d.). Only for producing
    /// demonstration files; a non-zero value is stated in the header.
    pub demo_noise_deg: f64,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            hz: 200.0,
            cycles: 1,
            demo_noise_deg: 0.0,
        }
    }
}

pub fn columns() -> Vec<String> {
    let mut c: Vec<String> = [
        "time_s",
        "cycle_pct",
        "com_x_m",
        "com_y_m",
        "com_vx_m_s",
        "com_vy_m_s",
        "pitch_deg",
        "com_z_m",
        "com_vz_m_s",
        "roll_deg",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    for limb in LIMBS {
        for j in 0..3 {
            c.push(limb.angle_column(j));
        }
        c.push(limb.abduction_column());
    }
    for limb in LIMBS {
        let l = limb.label();
        c.push(format!("{l}_paw_x_m"));
        c.push(format!("{l}_paw_y_m"));
        c.push(format!("{l}_paw_z_m"));
        c.push(format!("{l}_contact"));
        c.push(format!("{l}_grf_bw"));
        c.push(format!("{l}_grf_ml_bw"));
    }
    c.push("total_grf_bw".into());
    c.push("total_grf_ml_bw".into());
    c
}

fn row(f: &Frame, cycle_pct: f64, noise: &mut Noise, sd: f64) -> Vec<String> {
    let mut r = vec![
        format!("{:.6}", f.t),
        format!("{:.6}", cycle_pct),
        format!("{:.6}", f.com.x),
        format!("{:.6}", f.com.y),
        format!("{:.6}", f.com_v.x),
        format!("{:.6}", f.com_v.y),
        format!("{:.6}", f.pitch.to_degrees()),
        format!("{:.6}", f.com_z),
        format!("{:.6}", f.com_vz),
        format!("{:.6}", f.roll.to_degrees()),
    ];
    for l in &f.limbs {
        for a in l.angles {
            let n = if sd > 0.0 { noise.gauss() * sd } else { 0.0 };
            r.push(format!("{:.6}", a + n));
        }
        r.push(format!("{:.6}", l.abduction));
    }
    for l in &f.limbs {
        r.push(format!("{:.6}", l.pts[4].x));
        r.push(format!("{:.6}", l.pts[4].y));
        r.push(format!("{:.6}", l.lat[4]));
        r.push(if l.contact { "1".into() } else { "0".into() });
        r.push(format!("{:.6}", l.grf_bw));
        r.push(format!("{:.6}", l.grf_ml_bw));
    }
    r.push(format!("{:.6}", f.total_grf()));
    r.push(format!("{:.6}", f.total_grf_ml()));
    r
}

pub fn write_csv(
    sim: &Simulation,
    opts: ExportOptions,
    mut w: impl Write,
) -> std::io::Result<usize> {
    let p = &sim.params;
    let sk = &sim.sk;
    writeln!(w, "# Canine Gait Lab model export")?;
    writeln!(w, "# model_version={MODEL_VERSION}")?;
    writeln!(
        w,
        "# breed={}; withers_height_m={:.4}; mass_kg={:.2}",
        p.breed.name, sk.withers_height, sk.mass
    )?;
    writeln!(
        w,
        "# gait={}; froude={:.4}; speed_m_s={:.4}; stride_length_m={:.4}; stride_period_s={:.4}; duty_factor={:.4}",
        p.gait.csv_name(),
        sim.froude(),
        sim.speed,
        sim.stride_len,
        sim.period,
        sim.duty
    )?;
    if let Some(j) = sim.jump() {
        writeln!(
            w,
            "# obstacle_m={:.4}; takeoff_v_m_s=({:.4},{:.4}); apex_com_m={:.4}",
            j.obstacle_h, j.v0x, j.v0z, j.y_apex
        )?;
    }
    writeln!(w, "# DATA ARE MODELLED KINEMATICS, NOT MEASUREMENTS.")?;
    if opts.demo_noise_deg > 0.0 {
        writeln!(
            w,
            "# SYNTHETIC DEMO FILE: joint angles include {:.2} deg s.d. pseudo-random noise.",
            opts.demo_noise_deg
        )?;
    }
    writeln!(
        w,
        "# angles: included angle on the flexor side; 180 = straight, >180 = hyperextension"
    )?;
    writeln!(
        w,
        "# coordinates: sagittal plane, x cranial (direction of travel), y up, metres; GRF in body weights (estimate)"
    )?;

    let mut wr = csv::Writer::from_writer(w);
    wr.write_record(columns())?;
    let dur = if sim.is_periodic() {
        sim.duration * opts.cycles.max(1) as f64
    } else {
        sim.duration
    };
    let n = (dur * opts.hz).round().max(2.0) as usize;
    let mut noise = Noise(0x9E37_79B9_7F4A_7C15);
    for k in 0..n {
        let t = k as f64 / opts.hz;
        let f = sim.sample(t);
        let pct = sim.phase_of(t) * 100.0;
        wr.write_record(row(&f, pct, &mut noise, opts.demo_noise_deg))?;
    }
    wr.flush()?;
    Ok(n)
}

/// Tiny deterministic generator (xorshift64*) so demo files are reproducible.
struct Noise(u64);

impl Noise {
    fn next(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        let u1 = self.next().max(1e-12);
        let u2 = self.next();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}
