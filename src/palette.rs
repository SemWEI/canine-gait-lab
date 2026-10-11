//! Colour roles for both themes. Limb hues are the validated categorical slots 1–4
//! (blue, orange, aqua, yellow) stepped separately for light and dark surfaces; every chart
//! that uses them also carries a legend or row labels, so identity never rests on colour alone.

use eframe::egui::Color32;

#[derive(Clone, Copy, Debug)]
pub struct Palette {
    pub limb: [Color32; 4],
    pub text: Color32,
    pub text_muted: Color32,
    pub canvas: Color32,
    pub grid: Color32,
    pub ground: Color32,
    pub ground_fill: Color32,
    pub bone: Color32,
    pub joint_fill: Color32,
    pub com: Color32,
    pub obstacle: Color32,
    pub warn: Color32,
    pub badge_model: Color32,
    pub badge_import: Color32,
}

pub fn palette(dark: bool) -> Palette {
    if dark {
        Palette {
            limb: [
                Color32::from_rgb(0x39, 0x87, 0xe5),
                Color32::from_rgb(0xd9, 0x59, 0x26),
                Color32::from_rgb(0x19, 0x9e, 0x70),
                Color32::from_rgb(0xc9, 0x85, 0x00),
            ],
            text: Color32::from_rgb(0xf2, 0xf1, 0xec),
            text_muted: Color32::from_rgb(0xa9, 0xa8, 0x9f),
            canvas: Color32::from_rgb(0x1b, 0x1d, 0x1f),
            grid: Color32::from_rgba_unmultiplied(0xff, 0xff, 0xff, 0x14),
            ground: Color32::from_rgb(0x8c, 0x8a, 0x80),
            ground_fill: Color32::from_rgb(0x24, 0x26, 0x27),
            bone: Color32::from_rgb(0xf0, 0xea, 0xd8),
            joint_fill: Color32::from_rgb(0x1b, 0x1d, 0x1f),
            com: Color32::from_rgb(0xf2, 0xf1, 0xec),
            obstacle: Color32::from_rgb(0xb8, 0x4a, 0x4a),
            warn: Color32::from_rgb(0xe6, 0x67, 0x67),
            badge_model: Color32::from_rgb(0x3a, 0x3f, 0x47),
            badge_import: Color32::from_rgb(0x1f, 0x4d, 0x3c),
        }
    } else {
        Palette {
            limb: [
                Color32::from_rgb(0x2a, 0x78, 0xd6),
                Color32::from_rgb(0xeb, 0x68, 0x34),
                Color32::from_rgb(0x1b, 0xaf, 0x7a),
                Color32::from_rgb(0xed, 0xa1, 0x00),
            ],
            text: Color32::from_rgb(0x16, 0x17, 0x18),
            text_muted: Color32::from_rgb(0x5a, 0x59, 0x55),
            canvas: Color32::from_rgb(0xf7, 0xf7, 0xf4),
            grid: Color32::from_rgba_unmultiplied(0x00, 0x00, 0x00, 0x12),
            ground: Color32::from_rgb(0x6d, 0x6b, 0x63),
            ground_fill: Color32::from_rgb(0xec, 0xeb, 0xe5),
            bone: Color32::from_rgb(0x3a, 0x35, 0x2c),
            joint_fill: Color32::from_rgb(0xff, 0xff, 0xff),
            com: Color32::from_rgb(0x16, 0x17, 0x18),
            obstacle: Color32::from_rgb(0xb0, 0x3a, 0x3a),
            warn: Color32::from_rgb(0xc0, 0x36, 0x36),
            badge_model: Color32::from_rgb(0xe3, 0xe6, 0xec),
            badge_import: Color32::from_rgb(0xd6, 0xf0, 0xe4),
        }
    }
}

pub fn rgb(c: [u8; 3]) -> Color32 {
    Color32::from_rgb(c[0], c[1], c[2])
}

pub fn shade(c: Color32, k: f32) -> Color32 {
    let f = |v: u8| ((v as f32) * k).clamp(0.0, 255.0) as u8;
    Color32::from_rgba_unmultiplied(f(c.r()), f(c.g()), f(c.b()), c.a())
}

pub fn with_alpha(c: Color32, a: u8) -> Color32 {
    Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), a)
}
