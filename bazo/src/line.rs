// Copyright 2018 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Lines.

use core::ops::{Add, Mul, Sub};

use crate::{Affine, Point, Vec2};

/// A single line.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    /// The line's start point.
    pub p0: Point,
    /// The line's end point.
    pub p1: Point,
}

impl Line {
    /// Create a new line.
    #[inline(always)]
    pub fn new(p0: impl Into<Point>, p1: impl Into<Point>) -> Line {
        Line {
            p0: p0.into(),
            p1: p1.into(),
        }
    }

    /// Returns a copy of this `Line` with the end points swapped so that it
    /// points in the opposite direction.
    #[must_use]
    #[inline(always)]
    pub fn reversed(&self) -> Line {
        Self {
            p0: self.p1,
            p1: self.p0,
        }
    }

    /// The length of the line.
    #[inline]
    pub fn length(self) -> f64 {
        (self.p1 - self.p0).hypot()
    }

    /// The midpoint of the line.
    ///
    /// This is the same as calling [`Point::midpoint`] with
    /// the endpoints of this line.
    #[must_use]
    #[inline]
    pub fn midpoint(&self) -> Point {
        self.p0.midpoint(self.p1)
    }

    /// Computes the point where two lines, if extended to infinity, would cross.
    pub fn crossing_point(self, other: Line) -> Option<Point> {
        let ab = self.p1 - self.p0;
        let cd = other.p1 - other.p0;
        let pcd = ab.cross(cd);
        if pcd == 0.0 {
            return None;
        }
        let h = ab.cross(self.p0 - other.p0) / pcd;
        Some(other.p0 + cd * h)
    }

    /// Is this line `finite`?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(self) -> bool {
        self.p0.is_finite() && self.p1.is_finite()
    }

    /// Is this line `NaN`?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(self) -> bool {
        self.p0.is_nan() || self.p1.is_nan()
    }
}

impl From<(Point, Point)> for Line {
    #[inline(always)]
    fn from((from, to): (Point, Point)) -> Self {
        Line::new(from, to)
    }
}

impl From<(Point, Vec2)> for Line {
    #[inline(always)]
    fn from((origin, displacement): (Point, Vec2)) -> Self {
        Line::new(origin, origin + displacement)
    }
}

/// A trivial "curve" that is just a constant.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConstPoint(Point);

impl ConstPoint {
    /// Is this point [finite]?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(self) -> bool {
        self.0.is_finite()
    }

    /// Is this point [NaN]?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(self) -> bool {
        self.0.is_nan()
    }
}

impl Mul<Line> for Affine {
    type Output = Line;

    #[inline]
    fn mul(self, other: Line) -> Line {
        Line {
            p0: self * other.p0,
            p1: self * other.p1,
        }
    }
}

impl Add<Vec2> for Line {
    type Output = Line;

    #[inline]
    fn add(self, v: Vec2) -> Line {
        Line::new(self.p0 + v, self.p1 + v)
    }
}

impl Sub<Vec2> for Line {
    type Output = Line;

    #[inline]
    fn sub(self, v: Vec2) -> Line {
        Line::new(self.p0 - v, self.p1 - v)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Line, Point};

    #[test]
    fn line_reversed() {
        let l = Line::new((0.0, 0.0), (1.0, 1.0));
        let f = l.reversed();

        assert_eq!(l.p0, f.p1);
        assert_eq!(l.p1, f.p0);

        // Reversing it again should result in the original line
        assert_eq!(l, f.reversed());
    }

    #[test]
    fn line_midpoint() {
        let l = Line::new((0.0, 0.0), (2.0, 4.0));
        assert_eq!(l.midpoint(), Point::new(1.0, 2.0));
    }

    #[test]
    fn line_is_finite() {
        assert!((Line {
            p0: Point { x: 0., y: 0. },
            p1: Point { x: 1., y: 1. }
        })
        .is_finite());

        assert!(!(Line {
            p0: Point { x: 0., y: 0. },
            p1: Point {
                x: f64::INFINITY,
                y: 1.
            }
        })
        .is_finite());

        assert!(!(Line {
            p0: Point { x: 0., y: 0. },
            p1: Point {
                x: 0.,
                y: f64::INFINITY
            }
        })
        .is_finite());
    }
}
