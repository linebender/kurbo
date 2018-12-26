//! A simple 2D vector.

use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// A 2D vector.
///
/// This can be interpreted as a point in 2-space, a translation, a
/// vector, or a complex number; the interpretation is not indicated
/// in the type.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    /// Create a new vector.
    pub fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    /// Dot product of two vectors.
    pub fn dot(&self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Cross product of two vectors.
    ///
    /// This is signed so that (0, 1) × (1, 0) = 1.
    pub fn cross(&self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Magnitude of vector.
    pub fn hypot(&self) -> f64 {
        self.x.hypot(self.y)
    }

    /// Magnitude squared of vector.
    pub fn hypot2(&self) -> f64 {
        self.dot(*self)
    }

    /// Angle of vector.
    ///
    /// If the vector is interpreted as a complex number, this is the argument.
    pub fn atan2(&self) -> f64 {
        self.y.atan2(self.x)
    }

    /// A unit vector of the given angle.
    pub fn from_angle(th: f64) -> Vec2 {
        Vec2 { x: th.cos(), y: th.sin() }
    }

    /// Linearly interpolate between two points.
    pub fn lerp(&self, other: Vec2, t: f64) -> Vec2 {
        *self + t * (other - *self)
    }
}

impl From<(f64, f64)> for Vec2 {
    fn from(v: (f64, f64)) -> Vec2 {
        Vec2 { x: v.0, y: v.1 }
    }
}

impl From<Vec2> for (f64, f64) {
    fn from(v: Vec2) -> (f64, f64) {
        (v.x, v.y)
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    fn add(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, other: Vec2) {
        *self = Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;

    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x - other.x, y: self.y - other.y }
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, other: Vec2) {
        *self = Vec2 { x: self.x - other.x, y: self.y - other.y }
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;

    fn mul(self, other: f64) -> Vec2 {
        Vec2 { x: self.x * other, y: self.y * other}
    }
}

impl MulAssign<f64> for Vec2 {
    fn mul_assign(&mut self, other: f64) {
        *self = Vec2 { x: self.x * other, y: self.y * other};
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;

    fn mul(self, other: Vec2) -> Vec2 {
        other * self
    }
}

impl Div<f64> for Vec2 {
    type Output = Vec2;

    /// Note: division by a scalar is implemented by multiplying by the reciprocal.
    ///
    /// This is more efficient but has different roundoff behavior than division.
    fn div(self, other: f64) -> Vec2 {
        self * other.recip()
    }
}

impl DivAssign<f64> for Vec2 {
    fn div_assign(&mut self, other: f64) {
        *self *= other.recip();
    }
}

impl Neg for Vec2 {
    type Output = Vec2;

    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}
