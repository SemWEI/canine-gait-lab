pub mod dog;
pub mod front;
pub mod top;

use crate::model::math::{V2, V3};

/// Which way the canvas looks at the dog. All views are orthographic projections of the same
/// 3-D frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum View {
    /// From the dog's left: x cranial to the right, y up.
    Side,
    /// From in front, looking caudally: the dog's left is on screen right, y up.
    Front,
    /// From above: x cranial to the right, the dog's left toward the top of the screen.
    Top,
}

impl View {
    pub fn project(self, p: V3) -> V2 {
        match self {
            View::Side => V2::new(p.x, p.y),
            View::Front => V2::new(p.z, p.y),
            View::Top => V2::new(p.x, p.z),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            View::Side => "Side",
            View::Front => "Front",
            View::Top => "Top",
        }
    }

    pub fn from_name(s: &str) -> Option<View> {
        match s.trim().to_lowercase().as_str() {
            "side" | "sagittal" | "lateral" => Some(View::Side),
            "front" | "frontal" | "cranial" => Some(View::Front),
            "top" | "dorsal" | "plan" => Some(View::Top),
            _ => None,
        }
    }
}
