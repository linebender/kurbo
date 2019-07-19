//! A 2d point.

use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

use crate::Vec2;

/// A 2d point.
#[derive(Clone, Copy, Default, PartialEq)]
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
    pub fn to_vec2(self) -> Vec2 {
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

    /// A new `Point`, with each of x and y rounded to the nearest integer value.
    #[inline]
    pub fn round(self) -> Point {
        Point::new(self.x.round(), self.y.round())
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
        write!(formatter, "({}, {})", self.x, self.y)
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
    fn distance() {
        let p1 = Point::new(0., 10.);
        let p2 = Point::new(0., 5.);
        assert_eq!(p1.distance(p2), 5.);

        let p1 = Point::new(-11., 1.);
        let p2 = Point::new(-7., -2.);
        assert_eq!(p1.distance(p2), 5.);
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
