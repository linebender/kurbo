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

    /// A new [`Triangle`] from centroid and offsets ([`Vec2`]) - offset of each vertex from the centroid
    #[inline]
    fn from_centroid_offsets(centroid: impl Into<Point>, offsets: [impl Into<Vec2>; 3]) -> Self {
        let centroid = centroid.into();

        let mut points: [Point; 3] = Default::default();
        for (i, offset) in offsets.into_iter().enumerate() {
            points[i] = centroid + offset.into();
        }

        Self::new(points[0], points[1], points[2])
    }

    /// The centroid of the [`Triangle`]
    #[inline]
    pub fn centroid(&self) -> Point {
        (1.0 / 3.0 * (self.a.to_vec2() + self.b.to_vec2() + self.c.to_vec2())).to_point()
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

    /// Whether this [`Triangle`] has no (zero) area
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.area() == 0.0
    }

    /// The greatest radius of a circle that is within the [`Triangle`]
    #[inline]
    pub fn incircle(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let bc = self.b.distance(self.c);
        let ac = self.a.distance(self.c);

        self.area() / (ab + bc + ac) / 2.0
    }

    /// Circumcircle of the [`Triangle`]
    #[inline]
    pub fn circumcircle(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let bc = self.b.distance(self.c);
        let ac = self.a.distance(self.c);

        ab * bc * ac / (((ab + bc + ac) * (bc + ac - ab) * (ab + ac - bc) * (ab + bc - ac)).sqrt())
    }

    /// Expand the triangle by a constant amount (`sizes`) in all directions
    pub fn inflate(&self, scalar: f64) -> Self {
        let centroid = self.centroid();

        Self::new(
            centroid + (0.0, scalar),
            centroid + scalar * Vec2::from_angle(5.0 * FRAC_PI_4),
            centroid + scalar * Vec2::from_angle(7.0 * FRAC_PI_4),
        )
    }

    /// Is this [`Triangle`] finite?
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.a.is_finite() && self.b.is_finite() && self.c.is_finite()
    }

    /// Is this [`Triangle`] NaN?
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
        Triangle::new(
            (self.a.to_vec2() + v).to_point(),
            (self.b.to_vec2() + v).to_point(),
            (self.c.to_vec2() + v).to_point(),
        )
    }
}

impl Sub<Vec2> for Triangle {
    type Output = Triangle;

    #[inline]
    fn sub(self, v: Vec2) -> Triangle {
        Triangle::new(
            (self.a.to_vec2() - v).to_point(),
            (self.b.to_vec2() - v).to_point(),
            (self.c.to_vec2() - v).to_point(),
        )
    }
}

// TODO: Insets

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
        if s0 == s1 && s1 == s2 { s0 as i32 } else { 0 }
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

// NOTE: vertices a, b and c are not garunteed to be in order as described in the struct comments
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

#[cfg(test)]
mod tests {
    use crate::{Point, Triangle, Vec2};

    fn assert_approx_eq(x: f64, y: f64) {
        assert!((x - y).abs() < 1e-7);
    }

    #[test]
    fn from_radius() {
        let test = Triangle::from_radius(10.1);
        let expected = Triangle::from_coords(
            (0.0, 20.199999999999996),
            (-17.493713156445658, -10.099999999999998),
            (17.493713156445658, -10.099999999999998),
        );

        assert_eq!(test, expected);
    }

    #[test]
    fn with_centroid() {
        let test = Triangle::EQUILATERAL.with_centroid((12.2, -2.3));
        let expected = Triangle::from_coords((12.2, 1.7000000000000002), (9.2, -4.3), (15.2, -4.3));

        assert_eq!(test, expected);
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
    fn perimeter() {
        let test = Triangle::new(
            (781239.273894, 789234.234789),
            (234897.823471, 378902.234789),
            (789241.789234, 789234.234897),
        )
        .perimeter();
        let expected = 1380963.0638877784;

        assert_approx_eq(test, expected);
    }

    #[test]
    fn area() {
        let test = Triangle::new(
            (12123.423, 2382.7834),
            (7892.729, 238.459),
            (7820.2, 712.23),
        )
        .area();
        let expected = 1079952.91574081;

        assert_approx_eq(test, expected);
    }

    #[test]
    fn right_angled_area() {
        let test = Triangle::from_coords((1.2, 5.3), (1.2, 1.6), (10.0, 1.6)).right_angled_area();
        let expected = 16.28;

        assert_approx_eq(test, expected);
    }

    #[test]
    fn vertex_from_x() {
        let test = Triangle::from_coords((1.0, 3.2), (63.8, 30.2), (2.0, 2.0))
            .vertex_from_x(63.8)
            .unwrap();
        let expected = Point::new(63.8, 30.2);

        assert_eq!(test, expected);
    }

    #[test]
    fn organise() {
        let test = Triangle::from_coords((-20.1, 3.2), (12.3, 12.3), (9.2, -100.8)).organise();
        let expected = Triangle::from_coords((12.3, 12.3), (-20.1, 3.2), (9.2, -100.8));

        assert_eq!(test, expected);
    }

    #[test]
    fn radius() {
        let test = Triangle::EQUILATERAL.radius();
        let expected = 1.8541019662496845;

        assert_approx_eq(test, expected);
    }
}
