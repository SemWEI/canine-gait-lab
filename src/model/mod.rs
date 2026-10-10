//! Sagittal-plane biomechanical model of canine locomotion.
//!
//! Everything here is a deterministic function of (breed, height, gait, speed, time), so the
//! animation, the plots and exported CSV all read exactly the same numbers.

pub mod breed;
pub mod com;
pub mod gait;
pub mod grf;
pub mod jump;
pub mod kinematics;
pub mod math;
pub mod sim;

/// Standard gravity (m/s²).
pub const G: f64 = 9.81;

/// Bumped whenever model equations or constants change; written into every export.
pub const MODEL_VERSION: &str = "1.0.0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Limb {
    LF,
    RF,
    LH,
    RH,
}

pub const LIMBS: [Limb; 4] = [Limb::LF, Limb::RF, Limb::LH, Limb::RH];

pub const FORE_JOINTS: [&str; 3] = ["shoulder", "elbow", "carpus"];
pub const HIND_JOINTS: [&str; 3] = ["hip", "stifle", "tarsus"];

impl Limb {
    pub fn idx(self) -> usize {
        self as usize
    }

    pub fn is_fore(self) -> bool {
        matches!(self, Limb::LF | Limb::RF)
    }

    /// Left limbs face the viewer: the dog walks to the right, so we see its left side.
    pub fn is_near(self) -> bool {
        matches!(self, Limb::LF | Limb::LH)
    }

    pub fn label(self) -> &'static str {
        match self {
            Limb::LF => "LF",
            Limb::RF => "RF",
            Limb::LH => "LH",
            Limb::RH => "RH",
        }
    }

    pub fn long_name(self) -> &'static str {
        match self {
            Limb::LF => "Left fore",
            Limb::RF => "Right fore",
            Limb::LH => "Left hind",
            Limb::RH => "Right hind",
        }
    }

    pub fn joints(self) -> [&'static str; 3] {
        if self.is_fore() {
            FORE_JOINTS
        } else {
            HIND_JOINTS
        }
    }

    /// CSV column name for a joint angle, e.g. `LF_elbow_deg`.
    pub fn angle_column(self, j: usize) -> String {
        format!("{}_{}_deg", self.label(), self.joints()[j])
    }
}
