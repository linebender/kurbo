//! A simple 2D vector.

use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::common::FloatExt;
use crate::{Point, Size};

/// A 2D vector.
///
/// This is intended primarily for a vector in the mathematical sense,
/// but it can be interpreted as a translation, and converted to and
/// from a point (vector relative to the origin) and size.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec2 {
    /// The x-coordinate.
    pub x: f64,
    /// The y-coordinate.
    pub y: f64,
}

impl Vec2 {
    /// The vector (0, 0).
    pub const ZERO: Vec2 = Vec2::new(0., 0.);

    /// Create a new vector.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    /// Convert this vector into a `Point`.
    #[inline]
    pub const fn to_point(self) -> Point {
        Point::new(self.x, self.y)
    }

    /// Convert this vector into a `Size`.
    #[inline]
    pub const fn to_size(self) -> Size {
        Size::new(self.x, self.y)
    }

    /// Create a `Vec2` with the same value for x and y
    pub(crate) const fn splat(v: f64) -> Self {
        Vec2 { x: v, y: v }
    }

    /// Dot product of two vectors.
    #[inline]
    pub fn dot(self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Cross product of two vectors.
    ///
    /// This is signed so that (0, 1) × (1, 0) = 1.
    #[inline]
    pub fn cross(self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Magnitude of vector.
    #[inline]
    pub fn hypot(self) -> f64 {
        self.x.hypot(self.y)
    }

    /// Magnitude squared of vector.
    #[inline]
    pub fn hypot2(self) -> f64 {
        self.dot(self)
    }

    /// Angle of vector.
    ///
    /// If the vector is interpreted as a complex number, this is the argument.
    /// The angle is expressed in radians.
    #[inline]
    pub fn atan2(self) -> f64 {
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
    pub fn lerp(self, other: Vec2, t: f64) -> Vec2 {
        self + t * (other - self)
    }

    /// Returns a vector of magnitude 1.0 with the same angle as `self`; i.e.
    /// a unit/direction vector.
    ///
    /// This produces `NaN` values when the magnitutde is `0`.
    #[inline]
    pub fn normalize(self) -> Vec2 {
        self / self.hypot()
    }

    /// Returns a new `Vec2`,
    /// with `x` and `y` rounded to the nearest integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Vec2;
    /// let a = Vec2::new(3.3, 3.6).round();
    /// let b = Vec2::new(3.0, -3.1).round();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn round(self) -> Vec2 {
        Vec2::new(self.x.round(), self.y.round())
    }

    /// Returns a new `Vec2`,
    /// with `x` and `y` rounded up to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Vec2;
    /// let a = Vec2::new(3.3, 3.6).ceil();
    /// let b = Vec2::new(3.0, -3.1).ceil();
    /// assert_eq!(a.x, 4.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn ceil(self) -> Vec2 {
        Vec2::new(self.x.ceil(), self.y.ceil())
    }

    /// Returns a new `Vec2`,
    /// with `x` and `y` rounded down to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Vec2;
    /// let a = Vec2::new(3.3, 3.6).floor();
    /// let b = Vec2::new(3.0, -3.1).floor();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 3.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -4.0);
    /// ```
    #[inline]
    pub fn floor(self) -> Vec2 {
        Vec2::new(self.x.floor(), self.y.floor())
    }

    /// Returns a new `Vec2`,
    /// with `x` and `y` rounded away from zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Vec2;
    /// let a = Vec2::new(3.3, 3.6).expand();
    /// let b = Vec2::new(3.0, -3.1).expand();
    /// assert_eq!(a.x, 4.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -4.0);
    /// ```
    #[inline]
    pub fn expand(self) -> Vec2 {
        Vec2::new(self.x.expand(), self.y.expand())
    }

    /// Returns a new `Vec2`,
    /// with `x` and `y` rounded towards zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Vec2;
    /// let a = Vec2::new(3.3, 3.6).trunc();
    /// let b = Vec2::new(3.0, -3.1).trunc();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 3.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn trunc(self) -> Vec2 {
        Vec2::new(self.x.trunc(), self.y.trunc())
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
    #[allow(clippy::suspicious_arithmetic_impl)]
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

impl fmt::Display for Vec2 {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "𝐯=(")?;
        fmt::Display::fmt(&self.x, formatter)?;
        write!(formatter, ", ")?;
        fmt::Display::fmt(&self.y, formatter)?;
        write!(formatter, ")")
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

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn display() {
        let v = Vec2::new(1.2332421, 532.10721213123);
        let s = format!("{:.2}", v);
        assert_eq!(s.as_str(), "𝐯=(1.23, 532.11)");
    }
}
