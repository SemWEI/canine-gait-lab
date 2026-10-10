//! Small 2-D vector toolkit for the sagittal-plane model (x forward, y up, metres).

use std::f64::consts::PI;
use std::ops::{Add, AddAssign, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

impl V2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Unit vector pointing "distally" at `a` radians from straight down, positive = cranial (+x).
    pub fn from_down_angle(a: f64) -> Self {
        Self::new(a.sin(), -a.cos())
    }

    pub fn len(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn norm(self) -> Self {
        let l = self.len();
        if l > 1e-12 {
            self * (1.0 / l)
        } else {
            V2::new(0.0, -1.0)
        }
    }

    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }

    /// Counter-clockwise rotation by `a` radians.
    pub fn rot(self, a: f64) -> Self {
        let (s, c) = a.sin_cos();
        V2::new(c * self.x - s * self.y, s * self.x + c * self.y)
    }

    pub fn lerp(self, o: V2, t: f64) -> Self {
        self + (o - self) * t
    }

    pub fn dist(self, o: V2) -> f64 {
        (o - self).len()
    }
}

impl Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        V2::new(self.x + o.x, self.y + o.y)
    }
}
impl AddAssign for V2 {
    fn add_assign(&mut self, o: V2) {
        self.x += o.x;
        self.y += o.y;
    }
}
impl Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for V2 {
    type Output = V2;
    fn mul(self, k: f64) -> V2 {
        V2::new(self.x * k, self.y * k)
    }
}
impl Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        V2::new(-self.x, -self.y)
    }
}

/// Angle swept counter-clockwise from `from` to `to`, in [0, 2π).
pub fn ccw_angle(from: V2, to: V2) -> f64 {
    let a = from.cross(to).atan2(from.dot(to));
    if a < 0.0 { a + 2.0 * PI } else { a }
}

pub fn wrap01(x: f64) -> f64 {
    let r = x - x.floor();
    if r >= 1.0 { 0.0 } else { r }
}

/// Minimum-jerk position profile on [0,1] (Flash & Hogan 1985).
pub fn min_jerk(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

/// Raised-cosine ramp 0→1 with zero slope at both ends.
pub fn cos_ramp(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    0.5 - 0.5 * (PI * s).cos()
}

pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

pub fn deg(r: f64) -> f64 {
    r.to_degrees()
}

pub fn rad(d: f64) -> f64 {
    d.to_radians()
}

/// Periodic cubic Hermite (Catmull–Rom slopes) through `(u, value)` keys on u ∈ [0,1).
/// Keys must be sorted by `u`; the curve wraps from the last key to the first.
pub fn periodic_spline(keys: &[(f64, f64)], u: f64) -> f64 {
    let n = keys.len();
    debug_assert!(n >= 2);
    let u = wrap01(u);
    let key = |i: isize| -> (f64, f64) {
        let len = n as isize;
        let wraps = i.div_euclid(len);
        let (ku, kv) = keys[i.rem_euclid(len) as usize];
        (ku + wraps as f64, kv)
    };
    // find segment k such that key(k).u <= u < key(k+1).u
    let k: isize = keys
        .iter()
        .position(|kv| kv.0 > u)
        .map_or(n as isize - 1, |i| i as isize - 1);
    let (u0, p0) = key(k);
    let (u1, p1) = key(k + 1);
    let (um, pm) = key(k - 1);
    let (u2, p2) = key(k + 2);
    let uu = if u < u0 { u + 1.0 } else { u };
    let h = (u1 - u0).max(1e-9);
    let m0 = (p1 - pm) / (u1 - um).max(1e-9) * h;
    let m1 = (p2 - p0) / (u2 - u0).max(1e-9) * h;
    let t = ((uu - u0) / h).clamp(0.0, 1.0);
    let t2 = t * t;
    let t3 = t2 * t;
    (2.0 * t3 - 3.0 * t2 + 1.0) * p0
        + (t3 - 2.0 * t2 + t) * m0
        + (-2.0 * t3 + 3.0 * t2) * p1
        + (t3 - t2) * m1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spline_hits_keys_and_wraps() {
        let keys = [(0.0, 1.0), (0.3, 5.0), (0.7, -2.0)];
        for &(u, v) in &keys {
            assert!((periodic_spline(&keys, u) - v).abs() < 1e-9);
        }
        let a = periodic_spline(&keys, 0.999_999);
        assert!((a - 1.0).abs() < 1e-3);
    }

    #[test]
    fn ccw_angle_quadrants() {
        let down = V2::new(0.0, -1.0);
        let fwd = V2::new(1.0, 0.0);
        assert!((ccw_angle(down, fwd) - PI / 2.0).abs() < 1e-12);
        assert!((ccw_angle(fwd, down) - 1.5 * PI).abs() < 1e-12);
    }
}
