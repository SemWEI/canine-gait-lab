//! The simulation: turns (breed, height, gait, Froude number) into a time-indexed pose.

use super::breed::{BreedSpec, Skeleton};
use super::com::{BodyTables, GYRATION_FRAC, Loads, ROLL_GYRATION_FRAC, body_tables, ml_force};
use super::frontal::{LimbPlane, paw_half_base, swing_lateral};
use super::gait::{Gait, GaitSpec};
use super::grf::limb_grf_bw;
use super::jump::JumpPlan;
use super::kinematics::{
    Bend, FORE_FLEXOR_CRANIAL, HIND_FLEXOR_CRANIAL, distal_template, joint_angle, two_link,
};
use super::math::{V2, V3, rad, wrap01};
use super::{G, LIMBS};
use std::f64::consts::{PI, TAU};

/// Share of forelimb protraction/retraction produced by scapular rotation.
const SCAPULA_GAIN: f64 = 0.6;
/// Extra carpal / tarsal extension under load, degrees per body weight.
const LOAD_EXT_FORE: f64 = 6.0;
const LOAD_EXT_HIND: f64 = 3.0;
/// Pelvic rotation per unit spinal flexion (radians per unit shortening).
const PELVIS_FLEX_GAIN: f64 = 2.6;
/// Stance excursion may not exceed this fraction of the shorter functional limb length.
const STANCE_REACH: f64 = 1.0;
/// Girdle give, as a fraction of withers height, and the reach at which it starts.
const SCAPULAR_GLIDE: f64 = 0.06;
const PELVIC_GLIDE: f64 = 0.035;
const GLIDE_ONSET: f64 = 0.97;
/// Fraction of the trunk-relative retraction velocity kept at lift-off and touchdown.
const SWING_TANGENT: f64 = 0.35;
/// Largest shoulder→carpus (hip→hock) distance allowed in stance, as a fraction of the
/// two-segment length, before the trunk is lowered (≈168° elbow / stifle).
const MAX_STANCE_REACH: f64 = 0.998;

#[derive(Clone, Debug)]
pub struct SimParams {
    pub breed: BreedSpec,
    pub withers_height: f64,
    pub gait: Gait,
    pub froude: f64,
    /// Obstacle height for jumps (m).
    pub obstacle: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LimbFrame {
    /// Fore: scapular pivot, shoulder, elbow, carpus, paw.
    /// Hind: iliac crest, hip, stifle, hock, paw.
    pub pts: [V2; 5],
    /// Fore: shoulder, elbow, carpus. Hind: hip, stifle, tarsus. Degrees, flexor side.
    pub angles: [f64; 3],
    pub contact: bool,
    pub grf_bw: f64,
    pub clamped: bool,
    /// Demanded proximal→distal-joint distance / (upper + middle segment length).
    pub reach: f64,
    /// Lateral position (z, + = left) of each point in `pts`; `pts` is the sagittal projection.
    pub lat: [f64; 5],
    /// Limb ab/adduction relative to the trunk, degrees; positive = abducted (paw away from
    /// the midline). This is the tilt of the plane the limb flexes in.
    pub abduction: f64,
    /// Mediolateral GRF in body weights (+ = toward the dog's left).
    pub grf_ml_bw: f64,
    /// The tilted plane the limb flexes in (for rebuilding the limb from joint angles).
    pub plane: LimbPlane,
}

#[derive(Clone, Debug, Default)]
pub struct Frame {
    pub t: f64,
    /// 0..1 position in the gait cycle (or in the jump record).
    pub phase: f64,
    pub com: V2,
    pub com_v: V2,
    pub pitch: f64,
    pub spine_flex: f64,
    pub withers: V2,
    pub hip: V2,
    /// Lateral COM position and velocity (+ = left).
    pub com_z: f64,
    pub com_vz: f64,
    /// Trunk roll about the long axis (rad, left side up positive).
    pub roll: f64,
    pub limbs: [LimbFrame; 4],
}

impl Frame {
    pub fn total_grf(&self) -> f64 {
        self.limbs.iter().map(|l| l.grf_bw).sum()
    }

    pub fn total_grf_ml(&self) -> f64 {
        self.limbs.iter().map(|l| l.grf_ml_bw).sum()
    }

    /// Point `k` of limb `i` in 3-D.
    pub fn limb_pt3(&self, i: usize, k: usize) -> V3 {
        let l = &self.limbs[i];
        V3::new(l.pts[k].x, l.pts[k].y, l.lat[k])
    }

    pub fn com3(&self) -> V3 {
        V3::new(self.com.x, self.com.y, self.com_z)
    }
}

#[derive(Clone, Debug)]
enum Kind {
    Cyclic(BodyTables),
    Jump(Box<JumpPlan>),
}

#[derive(Clone, Debug)]
pub struct Simulation {
    pub params: SimParams,
    pub sk: Skeleton,
    pub spec: GaitSpec,
    pub speed: f64,
    pub stride_len: f64,
    pub period: f64,
    pub duty: f64,
    /// True when the duty factor had to be shortened to keep stance within limb reach.
    pub duty_reach_limited: bool,
    /// Cycle period for periodic gaits; record length for jumps.
    pub duration: f64,
    /// How far the mean trunk height was lowered below standing so stance stays within reach (m).
    pub lowering: f64,
    /// Lateral distance of the fore / hind paws from the line of travel in stance (m).
    pub half_base: [f64; 2],
    kind: Kind,
}

struct Drive {
    paw: V2,
    paw_z: f64,
    alpha: f64,
    contact: bool,
    grf: f64,
}

/// Whole-body state handed to [`Simulation::assemble`].
struct Body {
    com: V2,
    com_v: V2,
    pitch: f64,
    flex: f64,
    com_z: f64,
    com_vz: f64,
    roll: f64,
    /// Mean lateral COM position, the reference for mediolateral forces.
    z_mean: f64,
}

impl Simulation {
    pub fn new(params: SimParams) -> Self {
        let sk = Skeleton::new(&params.breed, params.withers_height);
        let spec = params.gait.spec();
        let gait = params.gait;
        let fr = params.froude.max(0.0);
        let half_base = [
            paw_half_base(&sk, gait, fr, true),
            paw_half_base(&sk, gait, fr, false),
        ];

        if gait.is_jump() {
            let running = gait == Gait::RunningJump;
            let v_run = (fr * G * sk.hip_h).sqrt();
            let plan = JumpPlan::new(&sk, running, v_run, params.obstacle);
            return Self {
                speed: plan.v0x,
                stride_len: plan.x_land - plan.x_takeoff,
                period: plan.total,
                duty: 1.0,
                duty_reach_limited: false,
                duration: plan.total,
                lowering: 0.0,
                half_base,
                kind: Kind::Jump(Box::new(plan)),
                params,
                sk,
                spec,
            };
        }

        let (speed, stride_len, period) = if gait == Gait::Stand || fr <= 0.0 {
            (0.0, 0.0, 1.0)
        } else {
            // Dynamic similarity (Alexander & Jayes 1983, J Zool): relative stride length
            // λ/h ≈ 2.3·Fr^0.3, with h the hip height and Fr = v²/(g·h).
            let v = (fr * G * sk.hip_h).sqrt();
            let lambda = 2.3 * fr.powf(0.3) * sk.hip_h;
            (v, lambda, lambda / v)
        };
        let mut duty = spec.duty_at(fr);
        let mut limited = false;
        if stride_len > 0.0 {
            let cap = STANCE_REACH * (sk.fore_len + sk.scapula).min(sk.hind_len) / stride_len;
            if duty > cap {
                duty = cap;
                limited = true;
            }
        }

        let tables = {
            let gyr = GYRATION_FRAC * (sk.fore_paw_n - sk.hind_paw_n);
            let offsets = spec.offsets;
            let ff = sk.fore_frac;
            let grf_kind = spec.grf;
            let (fn_, hn) = (sk.fore_paw_n, sk.hind_paw_n);
            let roll_gyr = ROLL_GYRATION_FRAC * sk.withers_height;
            body_tables(period, gyr, roll_gyr, sk.com_h, |phase| {
                let mut f = 0.0;
                let mut m = 0.0;
                let mut fz = 0.0;
                for (i, limb) in LIMBS.iter().enumerate() {
                    let u = wrap01(phase - offsets[i]);
                    if u < duty || duty >= 0.999 {
                        let s = if duty >= 0.999 { 0.5 } else { u / duty };
                        let w = if limb.is_fore() {
                            0.5 * ff
                        } else {
                            0.5 * (1.0 - ff)
                        };
                        let g = limb_grf_bw(grf_kind, w, duty, s);
                        let n = if limb.is_fore() { fn_ } else { hn };
                        let arm = n + stride_len * duty * (0.5 - s);
                        let z = limb.side() * half_base[if limb.is_fore() { 0 } else { 1 }];
                        f += g;
                        m += g * arm;
                        fz += g * z;
                    }
                }
                Loads { f, m, fz }
            })
        };

        let mut sim = Self {
            params,
            sk,
            spec,
            speed,
            stride_len,
            period,
            duty,
            duty_reach_limited: limited,
            duration: period,
            lowering: 0.0,
            half_base,
            kind: Kind::Cyclic(tables),
        };
        // Dogs carry the trunk lower as stride lengthens; lower it just enough that no stance
        // limb has to over-extend.
        let step = 0.005 * sim.sk.com_h;
        for _ in 0..60 {
            let worst = sim
                .series(240)
                .iter()
                .flat_map(|f| f.limbs.iter().filter(|l| l.contact).map(|l| l.reach))
                .fold(0.0, f64::max);
            if worst <= MAX_STANCE_REACH {
                break;
            }
            sim.lowering += step;
        }
        sim
    }

    pub fn is_periodic(&self) -> bool {
        matches!(self.kind, Kind::Cyclic(_))
    }

    pub fn jump(&self) -> Option<&JumpPlan> {
        match &self.kind {
            Kind::Jump(p) => Some(p),
            Kind::Cyclic(_) => None,
        }
    }

    pub fn froude(&self) -> f64 {
        self.speed * self.speed / (G * self.sk.hip_h)
    }

    /// Phase (0..1) of time `t` within the cycle or record.
    pub fn phase_of(&self, t: f64) -> f64 {
        if self.is_periodic() {
            wrap01(t / self.duration)
        } else {
            (t / self.duration).clamp(0.0, 1.0)
        }
    }

    pub fn sample(&self, t: f64) -> Frame {
        match &self.kind {
            Kind::Cyclic(tb) => self.sample_cyclic(tb, t),
            Kind::Jump(plan) => self.sample_jump(plan, t),
        }
    }

    fn sample_cyclic(&self, tb: &BodyTables, t: f64) -> Frame {
        let sk = &self.sk;
        let spec = &self.spec;
        let phase = wrap01(t / self.period);
        let (v, lambda, d, tp) = (self.speed, self.stride_len, self.duty, self.period);
        let _ = lambda;
        let com = V2::new(v * t, sk.com_h - self.lowering + tb.dy.at(phase));
        let com_v = V2::new(v, tb.vy.at(phase));
        let pitch = tb.pitch.at(phase);
        let flex = spec.spine_flex * (TAU * (phase - 0.02)).cos();
        let hw = sk.withers_height;

        let drives: [Drive; 4] = std::array::from_fn(|i| {
            let limb = LIMBS[i];
            let fore = limb.is_fore();
            let n = if fore { sk.fore_paw_n } else { sk.hind_paw_n };
            let w = if fore {
                0.5 * sk.fore_frac
            } else {
                0.5 * (1.0 - sk.fore_frac)
            };
            let dir0 = if fore {
                sk.metacarpus_dir0
            } else {
                sk.metatarsus_dir0
            };
            let side = limb.side();
            let hb = self.half_base[if fore { 0 } else { 1 }];
            let u = wrap01(phase - spec.offsets[i]);
            let tmpl = distal_template(fore, d, u);
            if d >= 0.999 || u < d {
                let s = if d >= 0.999 { 0.5 } else { u / d };
                let t_td = t - u * tp;
                let paw = V2::new(v * (t_td + 0.5 * d * tp) + n, 0.0);
                let grf = limb_grf_bw(spec.grf, w, d, s);
                let ext = rad(if fore { LOAD_EXT_FORE } else { LOAD_EXT_HIND }) * grf;
                Drive {
                    paw,
                    paw_z: side * hb,
                    alpha: dir0 + tmpl + ext,
                    contact: true,
                    grf,
                }
            } else {
                let s = (u - d) / (1.0 - d);
                // Protraction relative to the trunk: cubic Hermite from the lift-off to the
                // touchdown position whose end slopes carry part of the stance retraction
                // velocity, giving the brief late-stance-like retraction after lift-off and the
                // "swing retraction" before touchdown without large overshoot.
                let m = -SWING_TANGENT * lambda * (1.0 - d);
                let h01 = 3.0 * s * s - 2.0 * s * s * s;
                let rel =
                    -0.5 * lambda * d + lambda * d * h01 + m * s * (1.0 - s) * (1.0 - 2.0 * s);
                let h = sk.withers_height
                    * if fore {
                        spec.swing_height.0
                    } else {
                        spec.swing_height.1
                    };
                let bump = (PI * s.powf(0.75)).sin().powi(2);
                let paw = V2::new(v * t + n + rel, h * bump);
                Drive {
                    paw,
                    paw_z: side * (hb + swing_lateral(s, hw)),
                    alpha: dir0 + tmpl,
                    contact: false,
                    grf: 0.0,
                }
            }
        });
        let body = Body {
            com,
            com_v,
            pitch,
            flex,
            com_z: tb.z_mean + tb.dz.at(phase),
            com_vz: tb.vz.at(phase),
            roll: tb.roll.at(phase),
            z_mean: tb.z_mean,
        };
        self.assemble(t, phase, &body, drives)
    }

    fn sample_jump(&self, plan: &JumpPlan, t: f64) -> Frame {
        let sk = &self.sk;
        let t = t.clamp(0.0, plan.total);
        let (y, vy, ay) = plan.com_y(t);
        let com = V2::new(plan.com_x(t), y);
        let com_v = V2::new(plan.com_vx(t), vy);
        let pitch = plan.pitch(t);
        let total = (1.0 + ay / G).max(0.0);
        let contacts: [bool; 4] = std::array::from_fn(|i| plan.contact_at(t, i));
        let drives: [Drive; 4] = std::array::from_fn(|i| {
            let fore = LIMBS[i].is_fore();
            let dir0 = if fore {
                sk.metacarpus_dir0
            } else {
                sk.metatarsus_dir0
            };
            let hb = self.half_base[if fore { 0 } else { 1 }];
            let ld = plan.drive(t, i, total, contacts, sk.fore_frac);
            let ext = rad(if fore { LOAD_EXT_FORE } else { LOAD_EXT_HIND }) * ld.grf.min(3.0);
            Drive {
                paw: ld.paw,
                // jumps are bilaterally symmetric: paws stay on their standing track
                paw_z: LIMBS[i].side() * hb,
                alpha: dir0 + ld.alpha_offset + ext,
                contact: ld.contact,
                grf: ld.grf,
            }
        });
        let body = Body {
            com,
            com_v,
            pitch,
            flex: 0.0,
            com_z: 0.0,
            com_vz: 0.0,
            roll: 0.0,
            z_mean: 0.0,
        };
        self.assemble(t, t / plan.total, &body, drives)
    }

    fn assemble(&self, t: f64, phase: f64, b: &Body, drives: [Drive; 4]) -> Frame {
        let sk = &self.sk;
        let Body {
            com,
            com_v,
            pitch,
            flex,
            roll,
            ..
        } = *b;
        let (sin_r, cos_r) = roll.sin_cos();
        let withers = com + sk.withers_b.rot(pitch);
        let hip_b = sk.hip_b + (sk.withers_b - sk.hip_b) * flex;
        let hip = com + hip_b.rot(pitch);
        let pelvis_a = sk.pelvis_dir0 + pitch - flex * PELVIS_FLEX_GAIN;
        let crest = hip + V2::new(pelvis_a.cos(), pelvis_a.sin()) * sk.pelvis;

        let limbs = std::array::from_fn(|i| {
            let dr = &drives[i];
            let limb = LIMBS[i];
            let fore = limb.is_fore();
            let up = V2::new(-dr.alpha.sin(), dr.alpha.cos());
            let (dist_len, l1, l2) = if fore {
                (sk.metacarpus, sk.humerus, sk.radius)
            } else {
                (sk.metatarsus, sk.femur, sk.tibia)
            };
            let (mut top, mut prox) = if fore {
                let prot =
                    (dr.paw.x - withers.x).atan2(withers.y - dr.paw.y) - pitch - sk.fore_prot0;
                let a = sk.scapula_dir0 + pitch + SCAPULA_GAIN * prot;
                (withers, withers + V2::from_down_angle(a) * sk.scapula)
            } else {
                (crest, hip)
            };
            // Trunk roll carries the proximal joint around the long axis through the COM.
            let half_w = limb.side() * if fore { sk.shoulder_half_w } else { sk.hip_half_w };
            let dy_c = prox.y - com.y;
            let prox_z = b.com_z + half_w * cos_r - dy_c * sin_r;
            let lift = dy_c * (cos_r - 1.0) + half_w * sin_r;
            top.y += lift;
            prox.y += lift;
            // From here on everything is in the limb's own (tilted) plane.
            let plane = LimbPlane::through(prox.y, prox_z, dr.paw.y, dr.paw_z, 0.3 * (l1 + l2));
            top = plane.to_plane(top);
            let paw_ip = plane.to_plane(dr.paw);
            let target = paw_ip + up * dist_len;
            // Girdle give: the scapula glides on the thoracic sling (no clavicle) and the
            // pelvis rotates at the lumbosacral joint, both moving the proximal joint toward a
            // distant paw before the limb itself has to straighten.
            let max_glide = sk.withers_height * if fore { SCAPULAR_GLIDE } else { PELVIC_GLIDE };
            let glide = (prox.dist(target) - GLIDE_ONSET * (l1 + l2)).clamp(0.0, max_glide);
            if glide > 0.0 {
                let shift = (target - prox).norm() * glide;
                top += shift;
                prox += shift;
            }
            let bend = if fore { Bend::Caudal } else { Bend::Cranial };
            let ik = two_link(prox, target, l1, l2, bend);
            let distal = ik.end;
            let paw = if ik.clamped {
                distal - up * dist_len
            } else {
                paw_ip
            };
            let flex_cr = if fore {
                FORE_FLEXOR_CRANIAL
            } else {
                HIND_FLEXOR_CRANIAL
            };
            let angles = [
                joint_angle(prox, top, ik.mid, flex_cr[0]),
                joint_angle(ik.mid, prox, distal, flex_cr[1]),
                joint_angle(distal, ik.mid, paw, flex_cr[2]),
            ];
            let mut pts = [top, prox, ik.mid, distal, paw];
            let mut lat = [0.0; 5];
            for k in 0..5 {
                (pts[k], lat[k]) = plane.to_world(pts[k]);
            }
            LimbFrame {
                pts,
                angles,
                contact: dr.contact,
                grf_bw: dr.grf,
                clamped: ik.clamped,
                reach: prox.dist(target) / (l1 + l2),
                lat,
                abduction: limb.side() * (plane.tilt - roll).to_degrees(),
                grf_ml_bw: ml_force(dr.grf, dr.paw_z, b.z_mean, sk.com_h),
                plane,
            }
        });

        Frame {
            t,
            phase,
            com,
            com_v,
            pitch,
            spine_flex: flex,
            withers,
            hip,
            com_z: b.com_z,
            com_vz: b.com_vz,
            roll,
            limbs,
        }
    }

    /// `n` evenly spaced frames over one cycle / record (end-point excluded for cycles).
    pub fn series(&self, n: usize) -> Vec<Frame> {
        let n = n.max(2);
        let step = if self.is_periodic() {
            self.duration / n as f64
        } else {
            self.duration / (n - 1) as f64
        };
        (0..n).map(|i| self.sample(i as f64 * step)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::breed::embedded_breeds;
    use crate::model::gait::ALL_GAITS;

    fn sim(name: &str, gait: Gait, fr: f64) -> Simulation {
        let b = embedded_breeds()
            .into_iter()
            .find(|b| b.name.starts_with(name))
            .unwrap();
        let h = b.withers_height_m;
        Simulation::new(SimParams {
            breed: b,
            withers_height: h,
            gait,
            froude: fr,
            obstacle: 0.4 * h,
        })
    }

    #[test]
    fn stance_fraction_equals_duty_factor() {
        for gait in [
            Gait::Walk,
            Gait::Trot,
            Gait::Pace,
            Gait::Canter,
            Gait::TransverseGallop,
            Gait::RotaryGallop,
        ] {
            let s = sim("Labrador", gait, gait.spec().froude.2);
            let frames = s.series(4000);
            for i in 0..4 {
                let frac = frames.iter().filter(|f| f.limbs[i].contact).count() as f64
                    / frames.len() as f64;
                assert!(
                    (frac - s.duty).abs() < 2e-3,
                    "{gait:?} limb {i}: {frac} vs {}",
                    s.duty
                );
            }
        }
    }

    #[test]
    fn paws_do_not_slip_in_stance() {
        for name in ["Chihuahua", "Dachshund", "Labrador", "Great Dane"] {
            for gait in [Gait::Walk, Gait::Trot, Gait::Canter] {
                let s = sim(name, gait, gait.spec().froude.2);
                let dt = s.period / 2000.0;
                let mut prev: Option<Frame> = None;
                for k in 0..4000 {
                    let f = s.sample(k as f64 * dt);
                    if let Some(p) = &prev {
                        for i in 0..4 {
                            let (a, b) = (&p.limbs[i], &f.limbs[i]);
                            if a.contact && b.contact && !a.clamped && !b.clamped {
                                assert!(
                                    (a.pts[4].x - b.pts[4].x).abs() < 1e-3,
                                    "{name} {gait:?} limb {i} slipped"
                                );
                                assert!(b.pts[4].y.abs() < 1e-9);
                            }
                        }
                    }
                    prev = Some(f);
                }
            }
        }
    }

    #[test]
    fn grf_impulse_balances_body_weight() {
        for gait in [
            Gait::Walk,
            Gait::Trot,
            Gait::Pace,
            Gait::Canter,
            Gait::TransverseGallop,
            Gait::RotaryGallop,
        ] {
            let s = sim("Beagle", gait, gait.spec().froude.2);
            let frames = s.series(4000);
            let mean = frames.iter().map(|f| f.total_grf()).sum::<f64>() / frames.len() as f64;
            assert!((mean - 1.0).abs() < 2e-3, "{gait:?}: mean GRF {mean}");
            let fore: f64 = frames
                .iter()
                .map(|f| f.limbs[0].grf_bw + f.limbs[1].grf_bw)
                .sum::<f64>()
                / frames.len() as f64;
            assert!((fore - s.sk.fore_frac).abs() < 2e-3);
        }
    }

    #[test]
    fn same_froude_gives_same_dimensionless_stride() {
        let a = sim("Chihuahua", Gait::Trot, 1.0);
        let b = sim("Great Dane", Gait::Trot, 1.0);
        let ra = a.stride_len / a.sk.hip_h;
        let rb = b.stride_len / b.sk.hip_h;
        assert!((ra - rb).abs() < 1e-9);
        assert!((a.froude() - 1.0).abs() < 1e-9 && (b.froude() - 1.0).abs() < 1e-9);
        // and the small dog takes more strides per second
        assert!(a.period < b.period);
    }

    #[test]
    fn jump_apex_reaches_target() {
        for gait in [Gait::StandingJump, Gait::RunningJump] {
            for name in ["Jack Russell", "Border Collie", "Great Dane"] {
                let s = sim(name, gait, 1.2);
                let plan = s.jump().unwrap().clone();
                let n = 20_000;
                let apex = (0..=n)
                    .map(|k| s.sample(k as f64 / n as f64 * s.duration).com.y)
                    .fold(f64::MIN, f64::max);
                assert!(
                    (apex - plan.y_apex).abs() / plan.y_apex < 0.01,
                    "{name} {gait:?}: {apex} vs {}",
                    plan.y_apex
                );
                // no limb is loaded while airborne
                let mid = s.sample(0.5 * (plan.t_takeoff + plan.t_land));
                assert!(mid.limbs.iter().all(|l| !l.contact && l.grf_bw == 0.0));
            }
        }
    }

    #[test]
    fn every_gait_and_breed_samples_finite_values() {
        for b in embedded_breeds() {
            for gait in ALL_GAITS {
                let h = b.withers_height_m;
                let s = Simulation::new(SimParams {
                    breed: b.clone(),
                    withers_height: h,
                    gait,
                    froude: gait.spec().froude.2,
                    obstacle: 0.3 * h,
                });
                for f in s.series(200) {
                    assert!(f.com.y.is_finite() && f.pitch.is_finite());
                    assert!(f.com_z.is_finite() && f.roll.is_finite());
                    for l in &f.limbs {
                        assert!(l.lat.iter().all(|z| z.is_finite()) && l.abduction.is_finite());
                        assert!(
                            l.angles.iter().all(|a| a.is_finite()),
                            "{} {gait:?}",
                            b.name
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn pitch_and_com_ranges_are_plausible() {
        for gait in [
            Gait::Walk,
            Gait::Trot,
            Gait::Canter,
            Gait::TransverseGallop,
            Gait::RotaryGallop,
        ] {
            let s = sim("Greyhound", gait, gait.spec().froude.2);
            let fr = s.series(1000);
            let (pmin, pmax) = fr.iter().fold((f64::MAX, f64::MIN), |(a, b), f| {
                (a.min(f.pitch), b.max(f.pitch))
            });
            let (ymin, ymax) = fr.iter().fold((f64::MAX, f64::MIN), |(a, b), f| {
                (a.min(f.com.y), b.max(f.com.y))
            });
            let clamped = fr
                .iter()
                .filter(|f| f.limbs.iter().any(|l| l.clamped))
                .count();
            println!(
                "{gait:?}: pitch range {:.1}°, COM excursion {:.1} cm, duty {:.2}, lowered {:.1} cm, clamped frames {clamped}",
                (pmax - pmin).to_degrees(),
                (ymax - ymin) * 100.0,
                s.duty,
                s.lowering * 100.0
            );
            assert!((pmax - pmin).to_degrees() < 40.0);
            assert!((ymax - ymin) < 0.3 * s.sk.withers_height);
        }
    }

    #[test]
    fn paws_do_not_slip_laterally_and_segments_keep_length() {
        for gait in [Gait::Walk, Gait::Trot, Gait::Pace, Gait::RotaryGallop] {
            let s = sim("Labrador", gait, gait.spec().froude.2);
            let sk = &s.sk;
            let frames = s.series(2000);
            for w in frames.windows(2) {
                for i in 0..4 {
                    let (a, b) = (&w[0].limbs[i], &w[1].limbs[i]);
                    if a.contact && b.contact && !a.clamped && !b.clamped {
                        assert!((a.lat[4] - b.lat[4]).abs() < 1e-9, "{gait:?} limb {i}");
                    }
                }
            }
            for f in &frames {
                for i in 0..4 {
                    let (l1, l2) = if LIMBS[i].is_fore() {
                        (sk.humerus, sk.radius)
                    } else {
                        (sk.femur, sk.tibia)
                    };
                    let (p, m, d) = (f.limb_pt3(i, 1), f.limb_pt3(i, 2), f.limb_pt3(i, 3));
                    assert!((p.dist(m) - l1).abs() < 1e-9);
                    assert!((m.dist(d) - l2).abs() < 1e-9);
                }
            }
        }
    }

    fn range(v: impl Iterator<Item = f64>) -> f64 {
        let (lo, hi) = v.fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(x), b.max(x)));
        hi - lo
    }

    #[test]
    fn symmetric_gaits_sway_symmetrically_and_balance_ml_impulse() {
        for gait in [Gait::Walk, Gait::Trot, Gait::Pace] {
            let s = sim("Beagle", gait, gait.spec().froude.2);
            let n = 2000;
            let frames = s.series(n);
            let mean_z = frames.iter().map(|f| f.com_z).sum::<f64>() / n as f64;
            assert!(mean_z.abs() < 1e-6, "{gait:?}: mean com_z {mean_z}");
            for k in 0..n / 2 {
                let (a, b) = (&frames[k], &frames[k + n / 2]);
                assert!((a.com_z + b.com_z).abs() < 1e-4, "{gait:?} mirror");
                assert!((a.roll + b.roll).abs() < 1e-4, "{gait:?} roll mirror");
            }
            let ml = frames.iter().map(|f| f.total_grf_ml()).sum::<f64>() / n as f64;
            assert!(ml.abs() < 1e-3, "{gait:?}: mean ML GRF {ml}");
        }
        // asymmetric gaits still have zero net ML impulse
        let s = sim("Beagle", Gait::Canter, 1.6);
        let frames = s.series(2000);
        let ml = frames.iter().map(|f| f.total_grf_ml()).sum::<f64>() / 2000.0;
        assert!(ml.abs() < 1e-3, "canter: mean ML GRF {ml}");
    }

    #[test]
    fn pace_sways_and_rolls_more_than_trot() {
        let fr = 0.8;
        let p = sim("Labrador", Gait::Pace, fr).series(1000);
        let t = sim("Labrador", Gait::Trot, fr).series(1000);
        let sway = |v: &[Frame]| range(v.iter().map(|f| f.com_z));
        let roll = |v: &[Frame]| range(v.iter().map(|f| f.roll));
        assert!(sway(&p) > 2.0 * sway(&t));
        assert!(roll(&p) > 2.0 * roll(&t));
        println!(
            "pace: sway {:.2} cm, roll {:.2}°; trot: sway {:.2} cm, roll {:.2}°",
            sway(&p) * 100.0,
            roll(&p).to_degrees(),
            sway(&t) * 100.0,
            roll(&t).to_degrees()
        );
        let ab = p.iter().flat_map(|f| f.limbs.iter().map(|l| l.abduction.abs()));
        assert!(ab.fold(0.0, f64::max) < 15.0);
    }

    #[test]
    fn frontal_values_are_plausible_for_every_gait() {
        for gait in ALL_GAITS {
            let s = sim("Labrador", gait, gait.spec().froude.2);
            let fr = s.series(800);
            let sway = range(fr.iter().map(|f| f.com_z));
            let roll = range(fr.iter().map(|f| f.roll)).to_degrees();
            let abd = range(fr.iter().map(|f| f.limbs[0].abduction));
            println!(
                "{gait:?}: half-base fore {:.1} cm, sway {:.2} cm, roll {:.2}°, LF abd range {:.1}°",
                s.half_base[0] * 100.0,
                sway * 100.0,
                roll,
                abd
            );
            assert!(sway < 0.1 * s.sk.withers_height);
            assert!(roll < 15.0);
        }
    }
}
