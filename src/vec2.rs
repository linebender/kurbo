//! A simple 2D vector.

use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::{Point, Size};

/// A 2D vector.
///
/// This is intended primarily for a vector in the mathematical sense,
/// but it can be interpreted as a translation, and converted to and
/// from a point (vector relative to the origin) and size.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    /// Create a new vector.
    #[inline]
    pub fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    /// Convert this vector into a `Point`.
    #[inline]
    pub fn to_point(self) -> Point {
        Point::new(self.x, self.y)
    }

    /// Convert this vector into a `Size`.
    #[inline]
    pub fn to_size(self) -> Size {
        Size::new(self.x, self.y)
    }

    /// Dot product of two vectors.
    #[inline]
    pub fn dot(&self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Cross product of two vectors.
    ///
    /// This is signed so that (0, 1) × (1, 0) = 1.
    #[inline]
    pub fn cross(&self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Magnitude of vector.
    #[inline]
    pub fn hypot(&self) -> f64 {
        self.x.hypot(self.y)
    }

    /// Magnitude squared of vector.
    #[inline]
    pub fn hypot2(&self) -> f64 {
        self.dot(*self)
    }

    /// Angle of vector.
    ///
    /// If the vector is interpreted as a complex number, this is the argument.
    /// The angle is expressed in radians.
    #[inline]
    pub fn atan2(&self) -> f64 {
        self.y.atan2(self.x)
    }

    /// A unit vector of the given angle.
    ///
    /// With `th` at zero, the result is the positive X unit vector, and
    /// at π/2, it is the positive Y unit vector. The angle is expressed
    /// in radians.
    ///
    /// Thus, in a Y-down coordinate system (as is common for graphics),
    /// it is a clockwise rotation, and in Y-up (traditional for math), it
    /// is anti-clockwise. This convention is consistent with
    /// [`Affine::rotate`](struct.Affine.html#method.rotate).
    #[inline]
    pub fn from_angle(th: f64) -> Vec2 {
        Vec2 {
            x: th.cos(),
            y: th.sin(),
        }
    }

    /// Linearly interpolate between two vectors.
    #[inline]
    pub fn lerp(&self, other: Vec2, t: f64) -> Vec2 {
        *self + t * (other - *self)
    }

    /// Returns a vector of magnitude 1.0 with the same angle as `self`; i.e.
    /// a unit/direction vector.
    ///
    /// This produces `NaN` values when the magnitutde is `0`.
    #[inline]
    pub fn normalize(self) -> Vec2 {
        self / self.hypot()
    }
}

impl From<(f64, f64)> for Vec2 {
    #[inline]
    fn from(v: (f64, f64)) -> Vec2 {
        Vec2 { x: v.0, y: v.1 }
    }
}

impl From<Vec2> for (f64, f64) {
    #[inline]
    fn from(v: Vec2) -> (f64, f64) {
        (v.x, v.y)
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    #[inline]
    fn add(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, other: Vec2) {
        *self = Vec2 {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;

    #[inline]
    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, other: Vec2) {
        *self = Vec2 {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;

    #[inline]
    fn mul(self, other: f64) -> Vec2 {
        Vec2 {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

impl MulAssign<f64> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, other: f64) {
        *self = Vec2 {
            x: self.x * other,
            y: self.y * other,
        };
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;

    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        other * self
    }
}

impl Div<f64> for Vec2 {
    type Output = Vec2;

    /// Note: division by a scalar is implemented by multiplying by the reciprocal.
    ///
    /// This is more efficient but has different roundoff behavior than division.
    #[inline]
    fn div(self, other: f64) -> Vec2 {
        self * other.recip()
    }
}

impl DivAssign<f64> for Vec2 {
    #[inline]
    fn div_assign(&mut self, other: f64) {
        *self *= other.recip();
    }
}

impl Neg for Vec2 {
    type Output = Vec2;

    #[inline]
    fn neg(self) -> Vec2 {
        Vec2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

// Conversions to and from mint
#[cfg(feature = "mint")]
impl From<Vec2> for mint::Vector2<f64> {
    #[inline]
    fn from(p: Vec2) -> mint::Vector2<f64> {
        mint::Vector2 { x: p.x, y: p.y }
    }
}

#[cfg(feature = "mint")]
impl From<mint::Vector2<f64>> for Vec2 {
    #[inline]
    fn from(p: mint::Vector2<f64>) -> Vec2 {
        Vec2 { x: p.x, y: p.y }
    }
}
