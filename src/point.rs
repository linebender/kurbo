//! A 2D point.

use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

use crate::common::FloatExt;
use crate::Vec2;

/// A 2D point.
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Point {
    /// The x coordinate.
    pub x: f64,
    /// The y coordinate.
    pub y: f64,
}

impl Point {
    /// The point (0, 0).
    pub const ZERO: Point = Point::new(0., 0.);

    /// The point at the origin; (0, 0).
    pub const ORIGIN: Point = Point::new(0., 0.);

    /// Create a new `Point` with the provided `x` and `y` coordinates.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }

    /// Convert this point into a `Vec2`.
    #[inline]
    pub const fn to_vec2(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }

    /// Linearly interpolate between two points.
    #[inline]
    pub fn lerp(self, other: Point, t: f64) -> Point {
        self.to_vec2().lerp(other.to_vec2(), t).to_point()
    }

    /// Determine the midpoint of two points.
    #[inline]
    pub fn midpoint(self, other: Point) -> Point {
        Point::new(0.5 * (self.x + other.x), 0.5 * (self.y + other.y))
    }

    /// Euclidean distance.
    #[inline]
    pub fn distance(self, other: Point) -> f64 {
        (self - other).hypot()
    }

    /// Squared Euclidean distance.
    #[inline]
    pub fn distance_squared(self, other: Point) -> f64 {
        (self - other).hypot2()
    }

    /// Returns a new `Point`,
    /// with `x` and `y` rounded to the nearest integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Point;
    /// let a = Point::new(3.3, 3.6).round();
    /// let b = Point::new(3.0, -3.1).round();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn round(self) -> Point {
        Point::new(self.x.round(), self.y.round())
    }

    /// Returns a new `Point`,
    /// with `x` and `y` rounded up to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Point;
    /// let a = Point::new(3.3, 3.6).ceil();
    /// let b = Point::new(3.0, -3.1).ceil();
    /// assert_eq!(a.x, 4.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn ceil(self) -> Point {
        Point::new(self.x.ceil(), self.y.ceil())
    }

    /// Returns a new `Point`,
    /// with `x` and `y` rounded down to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Point;
    /// let a = Point::new(3.3, 3.6).floor();
    /// let b = Point::new(3.0, -3.1).floor();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 3.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -4.0);
    /// ```
    #[inline]
    pub fn floor(self) -> Point {
        Point::new(self.x.floor(), self.y.floor())
    }

    /// Returns a new `Point`,
    /// with `x` and `y` rounded away from zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Point;
    /// let a = Point::new(3.3, 3.6).expand();
    /// let b = Point::new(3.0, -3.1).expand();
    /// assert_eq!(a.x, 4.0);
    /// assert_eq!(a.y, 4.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -4.0);
    /// ```
    #[inline]
    pub fn expand(self) -> Point {
        Point::new(self.x.expand(), self.y.expand())
    }

    /// Returns a new `Point`,
    /// with `x` and `y` rounded towards zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Point;
    /// let a = Point::new(3.3, 3.6).trunc();
    /// let b = Point::new(3.0, -3.1).trunc();
    /// assert_eq!(a.x, 3.0);
    /// assert_eq!(a.y, 3.0);
    /// assert_eq!(b.x, 3.0);
    /// assert_eq!(b.y, -3.0);
    /// ```
    #[inline]
    pub fn trunc(self) -> Point {
        Point::new(self.x.trunc(), self.y.trunc())
    }

    /// Is this point finite?
    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    /// Is this point NaN?
    #[inline]
    pub fn is_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan()
    }
}

impl From<(f64, f64)> for Point {
    #[inline]
    fn from(v: (f64, f64)) -> Point {
        Point { x: v.0, y: v.1 }
    }
}

impl From<Point> for (f64, f64) {
    #[inline]
    fn from(v: Point) -> (f64, f64) {
        (v.x, v.y)
    }
}

impl Add<Vec2> for Point {
    type Output = Point;

    #[inline]
    fn add(self, other: Vec2) -> Self {
        Point::new(self.x + other.x, self.y + other.y)
    }
}

impl AddAssign<Vec2> for Point {
    #[inline]
    fn add_assign(&mut self, other: Vec2) {
        *self = Point::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub<Vec2> for Point {
    type Output = Point;

    #[inline]
    fn sub(self, other: Vec2) -> Self {
        Point::new(self.x - other.x, self.y - other.y)
    }
}

impl SubAssign<Vec2> for Point {
    #[inline]
    fn sub_assign(&mut self, other: Vec2) {
        *self = Point::new(self.x - other.x, self.y - other.y)
    }
}

impl Add<(f64, f64)> for Point {
    type Output = Point;

    #[inline]
    fn add(self, (x, y): (f64, f64)) -> Self {
        Point::new(self.x + x, self.y + y)
    }
}

impl AddAssign<(f64, f64)> for Point {
    #[inline]
    fn add_assign(&mut self, (x, y): (f64, f64)) {
        *self = Point::new(self.x + x, self.y + y)
    }
}

impl Sub<(f64, f64)> for Point {
    type Output = Point;

    #[inline]
    fn sub(self, (x, y): (f64, f64)) -> Self {
        Point::new(self.x - x, self.y - y)
    }
}

impl SubAssign<(f64, f64)> for Point {
    #[inline]
    fn sub_assign(&mut self, (x, y): (f64, f64)) {
        *self = Point::new(self.x - x, self.y - y)
    }
}

impl Sub<Point> for Point {
    type Output = Vec2;

    #[inline]
    fn sub(self, other: Point) -> Vec2 {
        Vec2::new(self.x - other.x, self.y - other.y)
    }
}

impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({:?}, {:?})", self.x, self.y)
    }
}

impl fmt::Display for Point {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "(")?;
        fmt::Display::fmt(&self.x, formatter)?;
        write!(formatter, ", ")?;
        fmt::Display::fmt(&self.y, formatter)?;
        write!(formatter, ")")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn point_arithmetic() {
        assert_eq!(
            Point::new(0., 0.) - Vec2::new(10., 0.),
            Point::new(-10., 0.)
        );
        assert_eq!(
            Point::new(0., 0.) - Point::new(-5., 101.),
            Vec2::new(5., -101.)
        );
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn distance() {
        let p1 = Point::new(0., 10.);
        let p2 = Point::new(0., 5.);
        assert_eq!(p1.distance(p2), 5.);

        let p1 = Point::new(-11., 1.);
        let p2 = Point::new(-7., -2.);
        assert_eq!(p1.distance(p2), 5.);
    }

    #[test]
    fn display() {
        let p = Point::new(0.12345, 9.87654);
        assert_eq!(format!("{}", p), "(0.12345, 9.87654)");

        let p = Point::new(0.12345, 9.87654);
        assert_eq!(format!("{:.2}", p), "(0.12, 9.88)");
    }
}

#[cfg(feature = "mint")]
impl From<Point> for mint::Point2<f64> {
    #[inline]
    fn from(p: Point) -> mint::Point2<f64> {
        mint::Point2 { x: p.x, y: p.y }
    }
}

#[cfg(feature = "mint")]
impl From<mint::Point2<f64>> for Point {
    #[inline]
    fn from(p: mint::Point2<f64>) -> Point {
        Point { x: p.x, y: p.y }
    }
}
