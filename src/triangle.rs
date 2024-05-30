// Copyright 2024 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Triangle shape
use crate::{Ellipse, PathEl, Point, Rect, Shape, Vec2};

use core::f64::consts::FRAC_PI_3;
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
#[derive(Clone, Copy, PartialEq)]
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

impl Default for Triangle {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Triangle {
    /// The empty triangle at the origin
    pub const ZERO: Self = Self::from_coords((0., 0.), (0., 0.), (0., 0.));

    /// Triangle identity
    pub const IDENTITY: Self = Self::from_coords((3.0, 6.0), (0.0, 0.0), (6.0, 0.0));

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

    /// A new `Triangle` from centroid and sizes
    #[inline]
    pub fn from_centroid_sizes(centroid: impl Into<Point>, sizes: [f64; 3]) -> Self {
        let centroid = centroid.into();
        Self::new(
            (centroid.x, centroid.y + sizes[0]),
            centroid + Vec2::IDENTITY * sizes[1],
            centroid + Vec2::IDENTITY * sizes[2],
        )
    }

    /// A new equilateral [`Triangle`]
    /// takes the center and a point equidistant to the midpoints of the vertices (radius)
    #[inline]
    pub fn from_center_size(center: impl Into<Point>, radius: f64) -> Self {
        const THETA: f64 = FRAC_PI_3; // equilateral triangle guarantee
        let center = center.into().to_vec2();

        let h = (radius / ((THETA / 2.0).tan()) * radius / ((THETA / 2.0).tan()) + radius * radius)
            .sqrt();
        let a = center.y + radius;
        let b = center + Vec2::IDENTITY * h;
        let c = center - Vec2::IDENTITY * h;

        Self::new((center.x, a), b.to_point(), c.to_point())
    }

    /// A new [`Triangle`] moved to `centroid`
    #[inline]
    pub fn with_centroid(self, centroid: impl Into<Point>) -> Self {
        Self::from_centroid_sizes(centroid, self.sizes())
    }

    /// Creates a new [`Triangle`] with new `sizes`
    #[inline]
    pub fn with_sizes(self, sizes: [f64; 3]) -> Self {
        Self::from_centroid_sizes(self.centroid(), sizes)
    }

    /// The centroid of the [`Triangle`]
    #[inline]
    pub fn centroid(&self) -> Point {
        Point::new(
            (self.a.x + self.b.x + self.c.x) / 3.0,
            (self.a.y + self.b.y + self.c.y) / 3.0,
        )
    }

    /// The distance of each vertex from the centroid
    #[inline]
    pub fn sizes(&self) -> [f64; 3] {
        let centroid = self.centroid();

        [
            self.a.distance(centroid),
            self.b.distance(centroid),
            self.c.distance(centroid),
        ]
    }

    /// The area of the [`Triangle`]
    #[inline]
    pub fn area(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let ac = self.a.distance(self.c);
        let bc = self.b.distance(self.c);

        // cos rule
        let theta = ((bc * bc - ab * ab - ac * ac) / -2.0 * ab * ac).acos();

        // A = 1/2*a*b*sin(C)
        (ab * ac * theta.sin()) / 2.0
    }

    // TODO: maybe make height functions for right and non right angled triangles

    /// The area of a right angled [`Triangle`]
    #[inline]
    pub fn right_angled_area(&self) -> f64 {
        // A = 1/2*b*h
        (self.b.distance(self.c) * self.b.midpoint(self.c).distance(self.a)) / 2.0
    }

    /// Maximum x-coordinate of the [`Triangle`]'s vertices
    #[inline]
    pub fn max_x(&self) -> f64 {
        self.a.x.max(self.b.x.max(self.c.x))
    }

    /// Minimum x-coordinate of the [`Triangle`]'s vertices
    #[inline]
    pub fn min_x(&self) -> f64 {
        self.a.x.min(self.b.x.min(self.c.x))
    }

    /// Mayimum y-coordinate of the [`Triangle`]'s vertices
    #[inline]
    pub fn max_y(&self) -> f64 {
        self.a.y.max(self.b.y.max(self.c.y))
    }

    /// Minimum y-coordinate of the [`Triangle`]'s vertices
    #[inline]
    pub fn min_y(&self) -> f64 {
        self.a.y.min(self.b.y.min(self.c.y))
    }

    /// Whether this [`Triangle`] has no (zero) area
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.area() == 0.0
    }

    /// Whether this right angled [`Triangle`] has no (zero) area
    #[inline]
    pub fn is_empty_right_angled(&self) -> bool {
        self.right_angled_area() == 0.0
    }

    /// `true` if `point` lies within `self`
    #[inline]
    pub fn contains(&self, point: Point) -> bool {
        point.x >= self.min_x()
            && point.x < self.max_x()
            && point.y >= self.min_y()
            && point.y < self.max_y()
    }

    /// The greatest radius of a circle that is within the [`Triangle`]
    #[inline]
    pub fn radius(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let bc = self.b.distance(self.c);
        let ac = self.a.distance(self.c);

        let s = (ab + bc + ac) / 2.0;

        // Heron's formula
        let area = (s * (s - ab) * (s - bc) * (s - ac)).sqrt();

        area / s
    }

    /// Expand the triangle by a constant amount (`sizes`) in all directions
    pub fn inflate(&self, sizes: [f64; 3]) -> Self {
        Self::new(
            (self.a.to_vec2() + Vec2::IDENTITY * sizes[0]).to_point(),
            (self.b.to_vec2() + Vec2::IDENTITY * sizes[1]).to_point(),
            (self.c.to_vec2() + Vec2::IDENTITY * sizes[2]).to_point(),
        )
    }

    /// A new [`Triangle`] with each vertecie's ordinates rounded to the nearest integer
    #[inline]
    pub fn round(self) -> Self {
        Self::new(self.a.round(), self.b.round(), self.c.round())
    }

    /// A new [`Triangle`] with each vertecie's ordinates rounded up to the nearest integer
    #[inline]
    pub fn ceil(self) -> Self {
        Self::new(self.a.ceil(), self.b.ceil(), self.c.ceil())
    }

    /// A new [`Triangle`] with each vertecie's ordinates rounded down to the nearest integer
    #[inline]
    pub fn floor(self) -> Self {
        Self::new(self.a.floor(), self.b.floor(), self.c.floor())
    }

    /// A new [`Triangle`],
    /// with each coordinate value rounded towards the center of the [`Triangle`]
    /// to the nearest integer, unless they are already an integer.
    /// That is to say this function will return the biggest possible [`Triangle`]
    /// with integer coordinates that is a subset of `self`.
    #[inline]
    pub fn expand(self) -> Self {
        Self::new(self.a.expand(), self.b.expand(), self.c.expand())
    }

    /// Returns a new [`Triangle`],
    /// with each coordinate value rounded towards the center of the [`Triangle`]
    /// to the nearest integer, unless they are already an integer.
    /// That is to say this function will return the biggest possible [`Triangle`]
    /// with integer coordinates that is a subset of `self`.
    #[inline]
    pub fn trunc(self) -> Self {
        Self::new(self.a.trunc(), self.b.trunc(), self.c.trunc())
    }

    /// Scales the [`Triangle`] by a `factor`
    #[inline]
    pub fn scale(self, factor: f64) -> Self {
        Self::new(
            (self.a.to_vec2() * factor).to_point(),
            (self.b.to_vec2() * factor).to_point(),
            (self.c.to_vec2() * factor).to_point(),
        )
    }

    /// Returns the [`Ellipse`] that is bounded by this [`Triangle`].
    #[inline]
    pub fn to_ellipse(self) -> Ellipse {
        Ellipse::from_triangle(self)
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
}

impl From<(Point, Point, Point)> for Triangle {
    fn from(points: (Point, Point, Point)) -> Triangle {
        Triangle::new(points.0, points.1, points.2)
    }
}

impl From<(Point, f64)> for Triangle {
    fn from(params: (Point, f64)) -> Triangle {
        Triangle::from_center_size(params.0, params.1)
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
        let xmin = self.min_x();
        let xmax = self.max_x();
        let ymin = self.min_y();
        let ymax = self.max_y();

        // TODO: may not be correct
        if pt.x >= xmin && pt.x < xmax && pt.y >= ymin && pt.y < ymax {
            if (self.c.x > self.b.x) ^ (self.a.y > self.b.y || self.a.y > self.c.y) {
                -1
            } else {
                1
            }
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        Rect::new(self.min_x(), self.min_y(), self.max_x(), self.max_y())
    }

    #[inline]
    fn as_rect(&self) -> Option<Rect> {
        Some(self.bounding_box())
    }

    #[inline]
    fn contains(&self, pt: Point) -> bool {
        self.contains(pt)
    }
}

// Anticlockwise direction from vertices a, b, c
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

// TODO: tests
