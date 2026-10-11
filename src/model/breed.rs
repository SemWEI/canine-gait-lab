//! Breed conformation presets and the derived, height-scaled skeleton.
//!
//! Segment lengths are *relative* proportions. They are scaled so that the standing forelimb
//! chain reaches the ground from the scapular pivot at 93 % of withers height, which keeps every
//! preset geometrically consistent at any height the user chooses.

use super::math::{V2, rad};
use serde::{Deserialize, Serialize};

const EMBEDDED_BREEDS: &str = include_str!("../../assets/breeds.json");

/// Fraction of withers height at which the dorsal scapular pivot sits (skin and spinous
/// processes make up the rest).
const PIVOT_HEIGHT_FRAC: f64 = 0.93;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Segments {
    pub scapula: f64,
    pub humerus: f64,
    pub radius: f64,
    pub metacarpus: f64,
    pub femur: f64,
    pub tibia: f64,
    pub metatarsus: f64,
}

/// Standing segment orientations, degrees from vertical (positive = distal end cranial).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StanceDirs {
    pub scapula: f64,
    pub humerus: f64,
    pub radius: f64,
    pub metacarpus: f64,
    pub femur: f64,
    pub tibia: f64,
    pub metatarsus: f64,
}

impl Default for StanceDirs {
    fn default() -> Self {
        // Typical mesomorph standing posture: shoulder ≈110°, elbow ≈145°, carpus ≈195°,
        // hip ≈95°, stifle ≈120°, tarsus ≈140° (order of magnitude of published canine goniometry, e.g. Jaegger et al. 2002).
        Self {
            scapula: 35.0,
            humerus: -35.0,
            radius: 0.0,
            metacarpus: 15.0,
            femur: 25.0,
            tibia: -35.0,
            metatarsus: 5.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EarType {
    Erect,
    Drop,
    Rose,
    Bat,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HeadSpec {
    /// Skull length (occiput to stop) / withers height.
    pub skull: f64,
    /// Muzzle length / skull length.
    pub muzzle: f64,
    /// Neck length / withers height.
    pub neck: f64,
    /// Neck elevation above horizontal, degrees.
    pub neck_angle: f64,
    pub ears: EarType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TailSpec {
    /// Tail length / withers height.
    pub length: f64,
    /// Carriage at the root, degrees above the caudal horizontal.
    pub carriage: f64,
    /// Additional bend per tail segment, degrees (positive curls up).
    pub curl: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Colors {
    pub body: [u8; 3],
    pub head: [u8; 3],
    pub legs: [u8; 3],
    #[serde(default)]
    pub saddle: Option<[u8; 3]>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BreedSpec {
    pub name: String,
    /// Reference withers height (m) for the breed standard.
    pub withers_height_m: f64,
    /// Reference body mass (kg) at that height; scaled isometrically (∝ h³) with height.
    pub mass_kg: f64,
    /// Horizontal distance scapular pivot → hip joint / withers height.
    pub trunk_length: f64,
    /// Withers-to-sternum depth / withers height.
    pub chest_depth: f64,
    /// Fraction of body weight on the forelimbs while standing.
    pub fore_weight_fraction: f64,
    /// Chest width / withers height.
    #[serde(default = "default_chest_width")]
    pub chest_width: f64,
    /// Standing fore / hind track (centre-to-centre paw spacing) / withers height.
    #[serde(default = "default_fore_track")]
    pub fore_track: f64,
    #[serde(default = "default_hind_track")]
    pub hind_track: f64,
    pub segments: Segments,
    #[serde(default)]
    pub stance: StanceDirs,
    pub head: HeadSpec,
    pub tail: TailSpec,
    pub colors: Colors,
    #[serde(default)]
    pub chondrodystrophic: bool,
    #[serde(default)]
    pub notes: String,
}

fn default_chest_width() -> f64 {
    0.30
}
fn default_fore_track() -> f64 {
    0.26
}
fn default_hind_track() -> f64 {
    0.24
}

/// Load presets from `assets/breeds.json` in the working directory if present (so researchers
/// can edit conformation without recompiling), otherwise from the copy embedded at build time.
pub fn load_breeds() -> (Vec<BreedSpec>, String) {
    if let Ok(txt) = std::fs::read_to_string("assets/breeds.json") {
        match serde_json::from_str::<Vec<BreedSpec>>(&txt) {
            Ok(v) if !v.is_empty() => return (v, "assets/breeds.json".into()),
            Ok(_) => {}
            Err(e) => {
                let v = embedded_breeds();
                return (
                    v,
                    format!("embedded presets (assets/breeds.json invalid: {e})"),
                );
            }
        }
    }
    (embedded_breeds(), "embedded presets".into())
}

pub fn embedded_breeds() -> Vec<BreedSpec> {
    serde_json::from_str(EMBEDDED_BREEDS).expect("embedded breeds.json is valid")
}

/// Height-scaled skeleton in the trunk ("body") frame: origin at the centre of mass,
/// x cranial, y dorsal, at zero pitch.
#[derive(Clone, Debug)]
pub struct Skeleton {
    pub withers_height: f64,
    pub mass: f64,
    pub fore_frac: f64,

    pub scapula: f64,
    pub humerus: f64,
    pub radius: f64,
    pub metacarpus: f64,
    pub femur: f64,
    pub tibia: f64,
    pub metatarsus: f64,
    pub pelvis: f64,

    /// Dorsal scapular pivot relative to COM.
    pub withers_b: V2,
    /// Hip joint relative to COM.
    pub hip_b: V2,
    /// Neutral (mid-stance) paw x relative to COM.
    pub fore_paw_n: f64,
    pub hind_paw_n: f64,
    /// Standing COM height above ground.
    pub com_h: f64,
    /// Standing hip-joint height (the "leg length" used for Froude scaling).
    pub hip_h: f64,

    pub scapula_dir0: f64,
    pub metacarpus_dir0: f64,
    pub metatarsus_dir0: f64,
    /// Pelvic axis (hip → iliac crest) angle above horizontal, radians.
    pub pelvis_dir0: f64,
    /// Standing protraction angle of the paw seen from the scapular pivot, radians from vertical.
    pub fore_prot0: f64,

    pub fore_len: f64,
    pub hind_len: f64,
    pub chest_depth: f64,
    pub trunk_length: f64,

    pub chest_width: f64,
    /// Lateral offset of each shoulder / hip joint from the midline.
    pub shoulder_half_w: f64,
    pub hip_half_w: f64,
    /// Standing lateral offset of the fore / hind paw centres from the midline.
    pub fore_paw_half_base: f64,
    pub hind_paw_half_base: f64,
}

impl Skeleton {
    pub fn new(spec: &BreedSpec, withers_height: f64) -> Self {
        let s = &spec.segments;
        let d = &spec.stance;
        let hw = withers_height;
        let dir = |a: f64| V2::from_down_angle(rad(a));

        let fore_rel = s.scapula * rad(d.scapula).cos()
            + s.humerus * rad(d.humerus).cos()
            + s.radius * rad(d.radius).cos()
            + s.metacarpus * rad(d.metacarpus).cos();
        let pivot_h = PIVOT_HEIGHT_FRAC * hw;
        let k = pivot_h / fore_rel;

        let (sc, hu, ra, mc) = (s.scapula * k, s.humerus * k, s.radius * k, s.metacarpus * k);
        let (fe, ti, mt) = (s.femur * k, s.tibia * k, s.metatarsus * k);

        let w = V2::new(0.0, pivot_h);
        let sh = w + dir(d.scapula) * sc;
        let el = sh + dir(d.humerus) * hu;
        let ca = el + dir(d.radius) * ra;
        let pf = ca + dir(d.metacarpus) * mc;

        let hip_h =
            fe * rad(d.femur).cos() + ti * rad(d.tibia).cos() + mt * rad(d.metatarsus).cos();
        let hip = V2::new(-spec.trunk_length * hw, hip_h);
        let st = hip + dir(d.femur) * fe;
        let ho = st + dir(d.tibia) * ti;
        let ph = ho + dir(d.metatarsus) * mt;

        let ff = spec.fore_weight_fraction;
        // Static balance: the COM lies over the point dividing the paw base in the ratio of
        // the fore/hind weight split.
        let com = V2::new(ph.x + ff * (pf.x - ph.x), 0.5 * (sh.y + hip.y) - 0.02 * hw);

        let scale = withers_height / spec.withers_height_m;
        Self {
            withers_height: hw,
            mass: spec.mass_kg * scale.powi(3),
            fore_frac: ff,
            scapula: sc,
            humerus: hu,
            radius: ra,
            metacarpus: mc,
            femur: fe,
            tibia: ti,
            metatarsus: mt,
            pelvis: 0.16 * hw,
            withers_b: w - com,
            hip_b: hip - com,
            fore_paw_n: pf.x - com.x,
            hind_paw_n: ph.x - com.x,
            com_h: com.y,
            hip_h,
            scapula_dir0: rad(d.scapula),
            metacarpus_dir0: rad(d.metacarpus),
            metatarsus_dir0: rad(d.metatarsus),
            pelvis_dir0: rad(30.0),
            fore_prot0: (pf.x - w.x).atan2(w.y - pf.y),
            fore_len: hu + ra + mc,
            hind_len: fe + ti + mt,
            chest_depth: spec.chest_depth * hw,
            trunk_length: spec.trunk_length * hw,
            chest_width: spec.chest_width * hw,
            shoulder_half_w: 0.45 * spec.chest_width * hw,
            hip_half_w: 0.40 * spec.chest_width * hw,
            fore_paw_half_base: 0.5 * spec.fore_track * hw,
            hind_paw_half_base: 0.5 * spec.hind_track * hw,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_presets_parse_and_stand_on_ground() {
        let breeds = embedded_breeds();
        assert!(breeds.len() >= 10);
        for b in &breeds {
            let sk = Skeleton::new(b, b.withers_height_m);
            assert!(sk.com_h > 0.2 * b.withers_height_m, "{}", b.name);
            assert!(sk.fore_paw_n > sk.hind_paw_n, "{}", b.name);
            assert!(
                sk.hip_h > 0.0 && sk.hip_h < b.withers_height_m * 1.1,
                "{}",
                b.name
            );
        }
    }
}
