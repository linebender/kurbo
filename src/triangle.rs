// Copyright 2024 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Triangle shape
use crate::{PathEl, Point, Rect, Shape, Vec2};

use core::cmp::*;
use core::f64::consts::FRAC_PI_4;
use core::ops::{Add, Sub};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// Triangle
//     A
//     *
//    / \
//   /   \
//  *-----*
//  B     C
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Triangle {
    /// vertex a
    pub a: Point,
    /// vertex b
    pub b: Point,
    /// vertex c
    pub c: Point,
}

impl Triangle {
    /// The empty [`Triangle`] at the origin
    pub const ZERO: Self = Self::from_coords((0., 0.), (0., 0.), (0., 0.));

    /// empty [`Triangle`] at (1.0, 1.0)
    pub const ONE: Self = Self::from_coords((1.0, 1.0), (1.0, 1.0), (1.0, 1.0));

    /// equilateral [`Triangle`]
    pub const EQUILATERAL: Self = Self::from_coords(
        (
            1.0 / 2.0,
            1.732050807568877293527446341505872367_f64 / 2.0, /* (sqrt 3)/2 */
        ),
        (0.0, 0.0),
        (1.0, 0.0),
    );

    /// A new [`Triangle`] from three vertices ([`Points`])
    #[inline]
    pub fn new(a: impl Into<Point>, b: impl Into<Point>, c: impl Into<Point>) -> Self {
        Self {
            a: a.into(),
            b: b.into(),
            c: c.into(),
        }
    }

    /// A new [`Triangle`] from three float vertex coordinates
    #[inline]
    pub const fn from_coords(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Self {
        Self {
            a: Point::new(a.0, a.1),
            b: Point::new(b.0, b.1),
            c: Point::new(c.0, c.1),
        }
    }

    /// The centroid of the [`Triangle`]
    #[inline]
    pub fn centroid(&self) -> Point {
        (1.0 / 3.0 * (self.a.to_vec2() + self.b.to_vec2() + self.c.to_vec2())).to_point()
    }

    /// The circumcenter of the [`Triangle`]
    #[inline]
    pub fn circumcenter(&self) -> Point {
        let d = 2.0
            * (self.a.x * (self.b.y - self.c.y)
                + self.b.x * (self.c.y - self.a.y)
                + self.c.x * (self.a.y - self.b.y));

        let ux = ((self.a.x.powi(2) + self.a.y.powi(2)) * (self.b.y - self.c.y)
            + (self.b.x.powi(2) + self.b.y.powi(2)) * (self.c.y - self.a.y)
            + (self.c.x.powi(2) + self.c.y.powi(2)) * (self.a.y - self.b.y))
            / d;

        let uy = ((self.a.x.powi(2) + self.a.y.powi(2)) * (self.c.x - self.b.x)
            + (self.b.x.powi(2) + self.b.y.powi(2)) * (self.a.x - self.c.x)
            + (self.c.x.powi(2) + self.c.y.powi(2)) * (self.b.x - self.a.x))
            / d;

        Point::new(ux, uy)
    }

    /// The offset of each vertex from the centroid
    #[inline]
    pub fn offsets(&self) -> [Vec2; 3] {
        let centroid = self.centroid().to_vec2();

        [
            (self.a.to_vec2() - centroid),
            (self.b.to_vec2() - centroid),
            (self.c.to_vec2() - centroid),
        ]
    }

    /// The area of the [`Triangle`]
    #[inline]
    pub fn area(&self) -> f64 {
        0.5 * (self.b - self.a).cross(self.c - self.a)
    }

    /// Whether this [`Triangle`] has zero area
    #[doc(alias = "is_empty")]
    #[inline]
    pub fn is_zero_area(&self) -> bool {
        self.area() == 0.0
    }

    /// Inradius of [`Triangle`] (the greatest radius of a circle that is within the [`Triangle`])
    /// with center [`Triangle::circumcenter`]
    #[inline]
    pub fn inradius(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let bc = self.b.distance(self.c);
        let ac = self.a.distance(self.c);

        2.0 * self.area() / (ab + bc + ac)
    }

    /// Circumradius of [`Triangle`] (the smallest radius of a circle such that it intercepts each vertex)
    /// with center [`Triangle::circumcenter`]
    #[inline]
    pub fn circumradius(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let bc = self.b.distance(self.c);
        let ac = self.a.distance(self.c);

        (ab * bc * ac) / (4.0 * self.area())
    }

    /// Expand the triangle by a constant amount (`scalar`) in all directions
    pub fn inflate(&self, scalar: f64) -> Self {
        let centroid = self.centroid();

        Self::new(
            centroid + (0.0, scalar),
            centroid + scalar * Vec2::from_angle(5.0 * FRAC_PI_4),
            centroid + scalar * Vec2::from_angle(7.0 * FRAC_PI_4),
        )
    }

    /// Is this [`Triangle`] [finite]?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.a.is_finite() && self.b.is_finite() && self.c.is_finite()
    }

    /// Is this [`Triangle`] [NaN]?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.a.is_nan() || self.b.is_nan() || self.c.is_nan()
    }

    /// [`Triangle`]'s vertices as an array of Points
    #[inline]
    pub fn as_array(&self) -> [Point; 3] {
        [self.a, self.b, self.c]
    }
}

impl From<(Point, Point, Point)> for Triangle {
    fn from(points: (Point, Point, Point)) -> Triangle {
        Triangle::new(points.0, points.1, points.2)
    }
}

impl Add<Vec2> for Triangle {
    type Output = Triangle;

    #[inline]
    fn add(self, v: Vec2) -> Triangle {
        Triangle::new(self.a + v, self.b + v, self.c + v)
    }
}

impl Sub<Vec2> for Triangle {
    type Output = Triangle;

    #[inline]
    fn sub(self, v: Vec2) -> Triangle {
        Triangle::new(self.a - v, self.b - v, self.c - v)
    }
}

#[doc(hidden)]
pub struct TrianglePathIter {
    triangle: Triangle,
    ix: usize,
}

impl Shape for Triangle {
    type PathElementsIter<'iter> = TrianglePathIter;

    fn path_elements(&self, _tolerance: f64) -> TrianglePathIter {
        TrianglePathIter {
            triangle: *self,
            ix: 0,
        }
    }

    #[inline]
    fn area(&self) -> f64 {
        Triangle::area(self)
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        self.a.distance(self.b) + self.b.distance(self.c) + self.c.distance(self.a)
    }

    #[inline]
    fn winding(&self, pt: Point) -> i32 {
        let s0 = (self.b - self.a).cross(pt - self.a).signum();
        let s1 = (self.c - self.b).cross(pt - self.b).signum();
        let s2 = (self.a - self.c).cross(pt - self.c).signum();

        if s0 == s1 && s1 == s2 {
            s0 as i32
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        Rect::new(
            self.a.x.min(self.b.x.min(self.c.x)),
            self.a.y.min(self.b.y.min(self.c.y)),
            self.a.x.max(self.b.x.max(self.c.x)),
            self.a.y.max(self.b.y.max(self.c.y)),
        )
    }

    #[inline]
    fn as_rect(&self) -> Option<Rect> {
        Some(self.bounding_box())
    }
}

// NOTE: vertices a, b and c are not guaranteed to be in order as described in the struct comments
//       (i.e. as "vertex a is topmost, vertex b is leftmost, and vertex c is rightmost")
impl Iterator for TrianglePathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.ix += 1;
        match self.ix {
            1 => Some(PathEl::MoveTo(self.triangle.a)),
            2 => Some(PathEl::LineTo(self.triangle.b)),
            3 => Some(PathEl::LineTo(self.triangle.c)),
            4 => Some(PathEl::ClosePath),
            _ => None,
        }
    }
}

// TODO: better and more tests
#[cfg(test)]
mod tests {
    use crate::{Point, Triangle, Vec2};

    fn assert_approx_eq(x: f64, y: f64) {
        assert!((x - y).abs() < 1e-7);
    }

    #[test]
    fn centroid() {
        let test = Triangle::from_coords((-90.02, 3.5), (7.2, -9.3), (8.0, 9.1)).centroid();
        let expected = Point::new(-24.939999999999998, 1.0999999999999996);

        assert_eq!(test, expected);
    }

    #[test]
    fn offsets() {
        let test = Triangle::from_coords((-20.0, 180.2), (1.2, 0.0), (290.0, 100.0)).offsets();
        let expected = [
            Vec2::new(-110.39999999999999, 86.8),
            Vec2::new(-89.19999999999999, -93.39999999999999),
            Vec2::new(199.60000000000002, 6.6000000000000085),
        ];

        assert_eq!(test, expected);
    }

    #[test]
    fn area() {
        let test = Triangle::new(
            (12123.423, 2382.7834),
            (7892.729, 238.459),
            (7820.2, 712.23),
        );
        let expected = 1079952.91574081;

        // initial
        assert_approx_eq(test.area(), -expected);
        // permutate vertex
        let test = Triangle::new(test.b, test.a, test.c);
        assert_approx_eq(test.area(), expected);
    }

    #[test]
    fn circumcenter() {
        let test = Triangle::EQUILATERAL.circumcenter();
        let expected = Point::new(0.5, 0.2886751345948128);

        assert_eq!(test.x, expected.x);
        assert_approx_eq(test.y, expected.y);
    }

    #[test]
    fn inradius() {
        let test = Triangle::EQUILATERAL.inradius();
        let expected = 0.28867513459481287;

        assert_approx_eq(test, expected);
    }

    #[test]
    fn circumradius() {
        let test = Triangle::EQUILATERAL.circumradius();
        let expected = 0.5773502691896258;

        assert_approx_eq(test, expected);
    }
}
