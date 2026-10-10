//! Jump model: crouch (or gather), constant-acceleration push-off, ballistic flight,
//! constant-deceleration forelimb-first landing and recovery steps.
//!
//! The COM trajectory is analytic in every phase, so vertical GRF follows by inverse dynamics
//! (F = m·(ÿ + g)) and is distributed over the limbs that are on the ground.

use super::breed::Skeleton;
use super::kinematics::distal_stance;
use super::math::{V2, cos_ramp, lerp, min_jerk, rad};
use super::{G, LIMBS};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub td: f64,
    pub lo: f64,
    /// Extra cranial offset of the footfall from neutral (m).
    pub x_off: f64,
    /// The contact begins at the start of the record (standing / already planted).
    pub initial: bool,
    /// The contact lasts to the end of the record.
    pub terminal: bool,
}

#[derive(Clone, Debug)]
pub struct JumpPlan {
    pub total: f64,
    pub t_crouch: f64,
    pub t_push: f64,
    pub t_takeoff: f64,
    pub t_land: f64,
    pub t_absorbed: f64,
    pub t_recovered: f64,
    pub y_stand: f64,
    pub y_crouch: f64,
    pub y_takeoff: f64,
    pub y_apex: f64,
    pub y_low: f64,
    pub v0x: f64,
    pub v0z: f64,
    pub obstacle_h: f64,
    pub obstacle_x: f64,
    pub x_takeoff: f64,
    pub x_land: f64,
    pitch_keys: Vec<(f64, f64)>,
    pub contacts: [Vec<Contact>; 4],
    /// Piecewise-linear horizontal velocity: (t0, t1, v0, v1).
    vx_segments: Vec<(f64, f64, f64, f64)>,
    fore_n: f64,
    hind_n: f64,
    tuck: [V2; 2],
}

/// Per-limb drive returned to the simulation.
#[derive(Clone, Copy, Debug)]
pub struct LimbDrive {
    pub paw: V2,
    pub alpha_offset: f64,
    pub contact: bool,
    pub grf: f64,
}

impl JumpPlan {
    pub fn new(sk: &Skeleton, running: bool, v_run: f64, obstacle: f64) -> Self {
        let hw = sk.withers_height;
        let h = sk.com_h;
        let y_stand = h;
        let y_crouch = h - if running { 0.12 } else { 0.22 } * h;
        let y_takeoff = h * 1.12;
        let y_low = h * 0.85;

        // COM-to-belly distance at standing posture.
        let withers_top = h + sk.withers_b.y + 0.07 * hw;
        let half_depth = (h - (withers_top - sk.chest_depth)).max(0.05 * hw);
        // Clear the bar with the belly and with the tucked paws (tuck sits 0.45–0.5 COM heights
        // below the COM).
        let below_com = half_depth.max(0.5 * h);
        let y_apex = (obstacle + 0.06 * hw + below_com).max(y_takeoff + 0.05 * h);

        let v0z = (2.0 * G * (y_apex - y_takeoff)).sqrt();
        let t_push = 2.0 * (y_takeoff - y_crouch) / v0z;
        let t_flight = 2.0 * v0z / G;
        let a_land = v0z * v0z / (2.0 * (y_takeoff - y_low));
        let t_abs = v0z / a_land;

        let v0x = if running {
            v_run.max(0.5)
        } else {
            let clear = 1.4 * (sk.trunk_length + 0.3 * hw);
            (clear / t_flight).max(0.4)
        };

        let (t_crouch, t_c) = if running { (0.0, 0.10) } else { (0.35, 0.45) };
        let t_push0 = t_crouch + t_c;
        let t_takeoff = t_push0 + t_push;
        let t_land = t_takeoff + t_flight;
        let t_absorbed = t_land + t_abs;
        let t_recovered = t_absorbed + if running { 0.35 + 0.1 * v0x } else { 0.45 };
        let total = t_recovered + 0.45;

        let mut vx_segments = Vec::new();
        if running {
            vx_segments.push((0.0, t_land, v0x, v0x));
        } else {
            vx_segments.push((0.0, t_push0, 0.0, 0.0));
            vx_segments.push((t_push0, t_takeoff, 0.0, v0x));
            vx_segments.push((t_takeoff, t_land, v0x, v0x));
        }
        vx_segments.push((t_land, t_recovered, v0x, 0.0));
        vx_segments.push((t_recovered, f64::INFINITY, 0.0, 0.0));

        let pitch_keys = if running {
            vec![
                (0.0, 0.0),
                (t_push0, 4.0),
                (t_takeoff, 20.0),
                (t_land, -18.0),
                (t_absorbed, -5.0),
                (t_recovered, 0.0),
                (total, 0.0),
            ]
        } else {
            vec![
                (0.0, 0.0),
                (t_crouch, 0.0),
                (t_push0, 6.0),
                (t_takeoff, 26.0),
                (t_land, -20.0),
                (t_absorbed, -6.0),
                (t_recovered, 0.0),
                (total, 0.0),
            ]
        };

        let c = |td: f64, lo: f64, x_off: f64| Contact {
            td,
            lo,
            x_off,
            initial: false,
            terminal: false,
        };
        let fore_lift = t_absorbed + 0.05;
        let fore_back = fore_lift + 0.22;
        let hind_land = t_land + 0.5 * t_abs;
        let hind_back = fore_back + 0.22;
        let fore_off = if running {
            t_push0 + 0.4 * t_push
        } else {
            t_push0 + 0.55 * t_push
        };
        let start_fore = Contact {
            initial: true,
            ..c(0.0, fore_off, 0.0)
        };
        let start_hind = Contact {
            initial: true,
            ..c(0.0, t_takeoff, if running { 0.05 * hw } else { 0.0 })
        };
        let fore = vec![
            start_fore,
            c(t_land, fore_lift, 0.08 * hw),
            Contact {
                terminal: true,
                ..c(fore_back, f64::INFINITY, 0.0)
            },
        ];
        let hind = vec![
            start_hind,
            c(hind_land, fore_back, 0.0),
            Contact {
                terminal: true,
                ..c(hind_back, f64::INFINITY, 0.0)
            },
        ];

        let mut plan = JumpPlan {
            total,
            t_crouch,
            t_push: t_push0,
            t_takeoff,
            t_land,
            t_absorbed,
            t_recovered,
            y_stand,
            y_crouch,
            y_takeoff,
            y_apex,
            y_low,
            v0x,
            v0z,
            obstacle_h: obstacle,
            obstacle_x: 0.0,
            x_takeoff: 0.0,
            x_land: 0.0,
            pitch_keys,
            contacts: [fore.clone(), fore, hind.clone(), hind],
            vx_segments,
            fore_n: sk.fore_paw_n,
            hind_n: sk.hind_paw_n,
            tuck: [
                V2::new(sk.fore_paw_n + 0.02 * hw, -0.45 * h),
                V2::new(sk.hind_paw_n - 0.05 * hw, -0.5 * h),
            ],
        };
        plan.x_takeoff = plan.com_x(t_takeoff);
        plan.x_land = plan.com_x(t_land);
        plan.obstacle_x = plan.com_x(t_takeoff + 0.5 * t_flight);
        plan
    }

    pub fn com_x(&self, t: f64) -> f64 {
        let mut x = 0.0;
        for &(t0, t1, v0, v1) in &self.vx_segments {
            if t <= t0 {
                break;
            }
            let te = t.min(t1);
            let dur = t1 - t0;
            let tau = te - t0;
            let slope = if dur.is_finite() && dur > 0.0 {
                (v1 - v0) / dur
            } else {
                0.0
            };
            x += v0 * tau + 0.5 * slope * tau * tau;
        }
        x
    }

    pub fn com_vx(&self, t: f64) -> f64 {
        for &(t0, t1, v0, v1) in &self.vx_segments {
            if t >= t0 && t < t1 {
                let dur = t1 - t0;
                return if dur.is_finite() && dur > 0.0 {
                    lerp(v0, v1, (t - t0) / dur)
                } else {
                    v0
                };
            }
        }
        0.0
    }

    /// Vertical COM position, velocity and acceleration.
    pub fn com_y(&self, t: f64) -> (f64, f64, f64) {
        let ramp = |a: f64, b: f64, t0: f64, t1: f64, t: f64| {
            let d = t1 - t0;
            let u = ((t - t0) / d).clamp(0.0, 1.0);
            let span = b - a;
            (
                a + span * cos_ramp(u),
                span * 0.5 * PI * (PI * u).sin() / d,
                span * 0.5 * PI * PI * (PI * u).cos() / (d * d),
            )
        };
        let a_push = self.v0z * self.v0z / (2.0 * (self.y_takeoff - self.y_crouch));
        let a_land = self.v0z * self.v0z / (2.0 * (self.y_takeoff - self.y_low));
        if t < self.t_crouch {
            (self.y_stand, 0.0, 0.0)
        } else if t < self.t_push {
            ramp(self.y_stand, self.y_crouch, self.t_crouch, self.t_push, t)
        } else if t < self.t_takeoff {
            let tau = t - self.t_push;
            (
                self.y_crouch + 0.5 * a_push * tau * tau,
                a_push * tau,
                a_push,
            )
        } else if t < self.t_land {
            let tau = t - self.t_takeoff;
            (
                self.y_takeoff + self.v0z * tau - 0.5 * G * tau * tau,
                self.v0z - G * tau,
                -G,
            )
        } else if t < self.t_absorbed {
            let tau = t - self.t_land;
            (
                self.y_takeoff - self.v0z * tau + 0.5 * a_land * tau * tau,
                -self.v0z + a_land * tau,
                a_land,
            )
        } else if t < self.t_recovered {
            ramp(
                self.y_low,
                self.y_stand,
                self.t_absorbed,
                self.t_recovered,
                t,
            )
        } else {
            (self.y_stand, 0.0, 0.0)
        }
    }

    pub fn pitch(&self, t: f64) -> f64 {
        let k = &self.pitch_keys;
        if t <= k[0].0 {
            return rad(k[0].1);
        }
        for w in k.windows(2) {
            let ((t0, p0), (t1, p1)) = (w[0], w[1]);
            if t <= t1 {
                let u = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
                // Free flight: constant angular velocity (angular momentum conserved).
                let shaped = if (t0 - self.t_takeoff).abs() < 1e-9 {
                    u
                } else {
                    cos_ramp(u)
                };
                return rad(lerp(p0, p1, shaped));
            }
        }
        rad(k[k.len() - 1].1)
    }

    fn com(&self, t: f64) -> V2 {
        V2::new(self.com_x(t), self.com_y(t).0)
    }

    fn neutral(&self, fore: bool) -> f64 {
        if fore { self.fore_n } else { self.hind_n }
    }

    /// World x of the footfall for a contact: neutral position at the contact's mid-time.
    fn footfall_x(&self, c: &Contact, fore: bool) -> f64 {
        let lo = c.lo.min(self.total);
        let mid = 0.5 * (c.td + lo);
        self.com_x(mid) + self.neutral(fore) + c.x_off
    }

    pub fn drive(
        &self,
        t: f64,
        limb_idx: usize,
        total_grf_bw: f64,
        contacts_now: [bool; 4],
        fore_frac: f64,
    ) -> LimbDrive {
        let limb = LIMBS[limb_idx];
        let fore = limb.is_fore();
        let cs = &self.contacts[limb_idx];
        // stance?
        for c in cs {
            if t >= c.td && t < c.lo {
                let paw = V2::new(self.footfall_x(c, fore), 0.0);
                let dur = (c.lo.min(self.total) - c.td).max(1e-6);
                let mut s = (t - c.td) / dur;
                if c.initial {
                    s = 0.5 + 0.5 * s;
                }
                if c.terminal {
                    s = (2.0 * s).min(1.0) * 0.5;
                }
                if c.initial && c.terminal {
                    s = 0.5;
                }
                // late-stance heel-off only near lift-off of push contacts
                let grf = share(limb_idx, total_grf_bw, contacts_now, fore_frac);
                return LimbDrive {
                    paw,
                    alpha_offset: distal_stance(fore, s),
                    contact: true,
                    grf,
                };
            }
        }
        // swing between contact k and k+1
        for w in cs.windows(2) {
            let (a, b) = (&w[0], &w[1]);
            if t >= a.lo && t < b.td {
                let s = (t - a.lo) / (b.td - a.lo);
                let p_lo = V2::new(self.footfall_x(a, fore), 0.0);
                let p_td = V2::new(self.footfall_x(b, fore), 0.0);
                let r_lo = p_lo - self.com(a.lo);
                let r_td = p_td - self.com(b.td);
                let base = r_lo.lerp(r_td, min_jerk(s));
                let flight = (b.td - a.lo) > 0.3;
                let bump = (PI * s).sin().powi(2);
                let r = if flight {
                    let tuck = self.tuck[if fore { 0 } else { 1 }].rot(self.pitch(t));
                    base * (1.0 - bump) + tuck * bump
                } else {
                    base + V2::new(0.0, 0.06 * self.y_stand * bump)
                };
                let mut paw = self.com(t) + r;
                paw.y = paw.y.max(0.0);
                let alpha = distal_swing(fore, s, flight);
                return LimbDrive {
                    paw,
                    alpha_offset: alpha,
                    contact: false,
                    grf: 0.0,
                };
            }
        }
        // before first or after last contact: hold the nearest footfall
        let c = cs.last().unwrap();
        LimbDrive {
            paw: V2::new(self.footfall_x(c, fore), 0.0),
            alpha_offset: distal_stance(fore, 0.5),
            contact: true,
            grf: 0.0,
        }
    }

    pub fn contact_at(&self, t: f64, limb_idx: usize) -> bool {
        self.contacts[limb_idx]
            .iter()
            .any(|c| t >= c.td && t < c.lo)
    }
}

fn share(limb_idx: usize, total: f64, contacts: [bool; 4], fore_frac: f64) -> f64 {
    let nf = contacts[0] as usize + contacts[1] as usize;
    let nh = contacts[2] as usize + contacts[3] as usize;
    let fore = limb_idx < 2;
    if !contacts[limb_idx] {
        return 0.0;
    }
    match (nf, nh) {
        (0, 0) => 0.0,
        (_, 0) => total / nf as f64,
        (0, _) => total / nh as f64,
        _ => {
            if fore {
                total * fore_frac / nf as f64
            } else {
                total * (1.0 - fore_frac) / nh as f64
            }
        }
    }
}

/// Distal segment offset during a swing (radians); flight swings fold the paw tightly.
fn distal_swing(fore: bool, s: f64, flight: bool) -> f64 {
    let keys: &[(f64, f64)] = match (fore, flight) {
        (true, false) => &[(0.0, -50.0), (0.3, -95.0), (0.7, -25.0), (1.0, -3.0)],
        (true, true) => &[(0.0, -50.0), (0.35, -105.0), (0.75, -30.0), (1.0, -3.0)],
        (false, false) => &[(0.0, -40.0), (0.35, -60.0), (0.75, -10.0), (1.0, 1.0)],
        (false, true) => &[(0.0, -40.0), (0.4, -75.0), (0.8, -15.0), (1.0, 1.0)],
    };
    let s = s.clamp(0.0, 1.0);
    for w in keys.windows(2) {
        if s <= w[1].0 {
            let u = cos_ramp((s - w[0].0) / (w[1].0 - w[0].0));
            return rad(lerp(w[0].1, w[1].1, u));
        }
    }
    rad(keys[keys.len() - 1].1)
}
