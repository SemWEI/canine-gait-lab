//! Gait definitions: footfall timing, duty factor and speed range.
//!
//! Footfall phase offsets are the touchdown time of each limb as a fraction of the stride,
//! with the reference hind limb at 0 (after Hildebrand 1965, Science; Hildebrand 1977 on
//! asymmetrical gaits). Duty factors fall with speed within each gait; the end-points below are
//! approximations informed by dog data in Maes et al. 2008 and Hudson et al. 2012 (both J Exp
//! Biol), interpolated in log(Froude number). Verify against primary sources before citing.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Gait {
    Stand,
    Walk,
    Trot,
    Pace,
    Canter,
    TransverseGallop,
    RotaryGallop,
    StandingJump,
    RunningJump,
}

pub const ALL_GAITS: [Gait; 9] = [
    Gait::Stand,
    Gait::Walk,
    Gait::Trot,
    Gait::Pace,
    Gait::Canter,
    Gait::TransverseGallop,
    Gait::RotaryGallop,
    Gait::StandingJump,
    Gait::RunningJump,
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GrfShape {
    /// Single half-sine, typical of running gaits.
    HalfSine,
    /// Two-humped "M" profile of walking (dip at mid-stance), `a` sets the dip depth.
    DoubleHump { a: f64 },
    /// Constant load (standing).
    Constant,
}

#[derive(Clone, Debug)]
pub struct GaitSpec {
    pub name: &'static str,
    /// Touchdown phase for LF, RF, LH, RH.
    pub offsets: [f64; 4],
    /// Duty factor at the lowest and highest Froude number of the range.
    pub duty: (f64, f64),
    /// Froude number range and default.
    pub froude: (f64, f64, f64),
    /// Peak paw clearance in swing / withers height (fore, hind).
    pub swing_height: (f64, f64),
    /// Peak fractional shortening of the shoulder–hip distance by spinal flexion.
    pub spine_flex: f64,
    pub grf: GrfShape,
    pub description: &'static str,
}

impl Gait {
    pub fn is_jump(self) -> bool {
        matches!(self, Gait::StandingJump | Gait::RunningJump)
    }

    pub fn csv_name(self) -> &'static str {
        match self {
            Gait::Stand => "stand",
            Gait::Walk => "walk",
            Gait::Trot => "trot",
            Gait::Pace => "pace",
            Gait::Canter => "canter",
            Gait::TransverseGallop => "transverse_gallop",
            Gait::RotaryGallop => "rotary_gallop",
            Gait::StandingJump => "standing_jump",
            Gait::RunningJump => "running_jump",
        }
    }

    pub fn from_name(s: &str) -> Option<Gait> {
        let s = s.trim().to_lowercase().replace([' ', '-'], "_");
        ALL_GAITS
            .into_iter()
            .find(|g| g.csv_name() == s || g.spec().name.to_lowercase().replace(' ', "_") == s)
    }

    pub fn spec(self) -> GaitSpec {
        match self {
            Gait::Stand => GaitSpec {
                name: "Stand",
                offsets: [0.0; 4],
                duty: (1.0, 1.0),
                froude: (0.0, 0.0, 0.0),
                swing_height: (0.0, 0.0),
                spine_flex: 0.0,
                grf: GrfShape::Constant,
                description: "Quiet standing; static weight distribution.",
            },
            Gait::Walk => GaitSpec {
                name: "Walk",
                // Lateral-sequence walk: LH → LF → RH → RF at quarter-cycle intervals.
                offsets: [0.25, 0.75, 0.0, 0.5],
                duty: (0.72, 0.58),
                froude: (0.05, 0.45, 0.20),
                swing_height: (0.07, 0.06),
                spine_flex: 0.0,
                grf: GrfShape::DoubleHump { a: 0.3 },
                description: "Four-beat lateral-sequence walk; always ≥2 feet on the ground.",
            },
            Gait::Trot => GaitSpec {
                name: "Trot",
                // Diagonal pairs LF+RH and RF+LH in unison, half a cycle apart.
                offsets: [0.5, 0.0, 0.0, 0.5],
                duty: (0.50, 0.36),
                froude: (0.40, 2.50, 0.90),
                swing_height: (0.12, 0.10),
                spine_flex: 0.0,
                grf: GrfShape::HalfSine,
                description: "Two-beat diagonal gait; brief suspension at faster speeds.",
            },
            Gait::Pace => GaitSpec {
                name: "Pace",
                // Ipsilateral pairs in unison.
                offsets: [0.0, 0.5, 0.0, 0.5],
                duty: (0.52, 0.40),
                froude: (0.30, 1.50, 0.70),
                swing_height: (0.10, 0.09),
                spine_flex: 0.0,
                grf: GrfShape::HalfSine,
                description: "Two-beat lateral gait; common in fatigue and in long-bodied dogs.",
            },
            Gait::Canter => GaitSpec {
                name: "Canter",
                // Left lead: RH, then diagonal LH+RF, then leading LF.
                offsets: [0.45, 0.25, 0.25, 0.0],
                duty: (0.45, 0.33),
                froude: (1.0, 3.0, 1.6),
                swing_height: (0.16, 0.14),
                spine_flex: 0.04,
                grf: GrfShape::HalfSine,
                description: "Three-beat asymmetric gait, left lead.",
            },
            Gait::TransverseGallop => GaitSpec {
                name: "Transverse gallop",
                // Left lead: RH, LH, RF, LF — the lead is on the same side fore and hind.
                offsets: [0.57, 0.45, 0.12, 0.0],
                duty: (0.36, 0.24),
                froude: (2.0, 12.0, 4.0),
                swing_height: (0.20, 0.18),
                spine_flex: 0.07,
                grf: GrfShape::HalfSine,
                description: "Four-beat gallop, one gathered suspension.",
            },
            Gait::RotaryGallop => GaitSpec {
                name: "Rotary gallop",
                // RH, LH, LF, RF — footfalls rotate around the body; the fore lead switches side.
                offsets: [0.45, 0.57, 0.12, 0.0],
                duty: (0.32, 0.18),
                froude: (3.0, 50.0, 10.0),
                swing_height: (0.22, 0.20),
                spine_flex: 0.10,
                grf: GrfShape::HalfSine,
                description: "Sighthound sprint gait with extended and gathered suspensions.",
            },
            Gait::StandingJump => GaitSpec {
                name: "Standing jump",
                offsets: [0.0; 4],
                duty: (1.0, 1.0),
                froude: (0.0, 0.0, 0.0),
                swing_height: (0.0, 0.0),
                spine_flex: 0.0,
                grf: GrfShape::HalfSine,
                description: "Crouch, hindlimb push-off, ballistic flight, forelimb-first landing.",
            },
            Gait::RunningJump => GaitSpec {
                name: "Running jump",
                offsets: [0.0; 4],
                duty: (1.0, 1.0),
                froude: (0.3, 4.0, 1.2),
                swing_height: (0.0, 0.0),
                spine_flex: 0.0,
                grf: GrfShape::HalfSine,
                description: "Short gather at approach speed, then ballistic flight and landing.",
            },
        }
    }
}

impl GaitSpec {
    /// Duty factor before any reach limit, interpolated in log(Fr) across the gait's range.
    pub fn duty_at(&self, fr: f64) -> f64 {
        let (lo, hi, _) = self.froude;
        if hi <= lo || lo <= 0.0 {
            return self.duty.0;
        }
        let t = ((fr.max(1e-6).ln() - lo.ln()) / (hi.ln() - lo.ln())).clamp(0.0, 1.0);
        self.duty.0 + (self.duty.1 - self.duty.0) * t
    }
}
