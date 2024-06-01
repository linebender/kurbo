// Copyright 2024 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Triangle shape
use crate::{Ellipse, PathEl, Point, Rect, Shape, Size, Vec2};

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

impl Default for Triangle {
    fn default() -> Self {
        Self::EQUILATERAL
    }
}

macro_rules! most_ordering {
    ($x: ident, $by: block) => {
        *$x.as_array().iter().max_by($by).unwrap()
    };
}

impl Triangle {
    /// The empty [`Triangle`] at the origin
    pub const ZERO: Self = Self::from_coords((0., 0.), (0., 0.), (0., 0.));

    /// empty [`Triangle`] at (1.0, 1.0)
    pub const ONE: Self = Self::from_coords((1.0, 1.0), (1.0, 1.0), (1.0, 1.0));

    /// equilateral [`Triangle`] identity
    pub const EQUILATERAL: Self = Self::from_coords((3.0, 6.0), (0.0, 0.0), (6.0, 0.0));

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

    /// A new [`Triangle`] from centroid and sizes ([`Size`]) - offset of each vertex from the centroid
    #[inline]
    pub fn from_centroid_sizes(centroid: impl Into<Point>, sizes: [impl Into<Size>; 3]) -> Self {
        let centroid = centroid.into();

        let mut points: [Point; 3] = Default::default();
        for (i, size) in sizes.into_iter().enumerate() {
            points[i] = centroid + size.into().to_vec2();
        }

        Self::new(points[0], points[1], points[2])
    }

    /// A new [`Triangle`] with vertex distances of `distances` from the `centroid`
    ///
    /// NOTE: the new [`Triangle`] does not keep `centroid`
    #[inline]
    pub fn from_centroid_distances(centroid: impl Into<Point>, distances: [f64; 3]) -> Self {
        let centroid = centroid.into();

        Self::new(
            centroid + (0.0, distances[0]),
            centroid + distances[1] * Vec2::from_angle(5.0 * FRAC_PI_4),
            centroid + distances[2] * Vec2::from_angle(7.0 * FRAC_PI_4),
        )
    }

    /// A new equilateral [`Triangle`]
    /// takes the center and a point equidistant to the midpoints of the vertices (radius)
    #[inline]
    pub fn from_radius(radius: f64) -> Self {
        // sqrt(3) (until https://github.com/rust-lang/rust/issues/103883)
        pub const SQRT_3: f64 = 1.732050807568877293527446341505872367_f64;

        let center = Point::new(0.0, 0.0); // self.centroid().to_vec2();

        let length = 2.0 * SQRT_3 * radius;
        let height = length * SQRT_3 / 2.0;
        let distance = height / 3.0;

        let sizes: [Size; 3] = [
            (0.0, 2.0 * distance).into(),
            (-length / 2.0, -height / 3.0).into(),
            (length / 2.0, -height / 3.0).into(),
        ];

        // let a = (center.x, center.y + 2.0 * distance);
        // let b = (center + Vec2::ONE * h;
        // let c = center - Vec2::ONE * h;

        // Self::new(a, b.to_point(), c.to_point())
        Self::from_centroid_sizes(center, sizes)
    }

    /// A new [`Triangle`] moved to `centroid`
    #[inline]
    pub fn with_centroid(&self, centroid: impl Into<Point>) -> Self {
        Self::from_centroid_sizes(centroid, self.sizes())
    }

    /// Creates a new [`Triangle`] with `sizes`
    #[inline]
    pub fn with_sizes(self, sizes: [impl Into<Size>; 3]) -> Self {
        Self::from_centroid_sizes(self.centroid(), sizes)
    }

    /// Creates a new [`Triangle`] with each vertice's `distances` offset from centroid
    #[inline]
    pub fn with_distances(self, distances: [f64; 3]) -> Self {
        Self::from_centroid_distances(self.centroid(), distances)
    }

    /// The centroid of the [`Triangle`]
    #[inline]
    pub fn centroid(&self) -> Point {
        Point::new(
            (self.a.x + self.b.x + self.c.x) / 3.0,
            (self.a.y + self.b.y + self.c.y) / 3.0,
        )
    }

    /// The euclidean distance of each vertex from the centroid
    #[inline]
    pub fn distance(&self) -> [f64; 3] {
        let centroid = self.centroid();

        [
            self.a.distance(centroid),
            self.b.distance(centroid),
            self.c.distance(centroid),
        ]
    }

    /// The offset of each vertex from the centroid
    #[inline]
    pub fn sizes(&self) -> [Size; 3] {
        let centroid = self.centroid().to_vec2();

        [
            (self.a.to_vec2() - centroid).to_size(),
            (self.b.to_vec2() - centroid).to_size(),
            (self.c.to_vec2() - centroid).to_size(),
        ]
    }

    /// The perimeter of the [`Triangle`]
    #[inline]
    fn perimeter(&self) -> f64 {
        self.a.distance(self.b) + self.b.distance(self.c) + self.c.distance(self.a)
    }

    /// The area of the [`Triangle`]
    #[inline]
    pub fn area(&self) -> f64 {
        let ab = self.a.distance(self.b);
        let ac = self.a.distance(self.c);
        let bc = self.b.distance(self.c);

        // cos rule
        let theta = ((bc * bc - ab * ab - ac * ac) / (-2.0 * ab * ac)).acos();

        // A = 1/2*a*b*sin(C)
        (ab * ac * theta.sin()) / 2.0
    }

    // TODO: maybe make height functions for right and non right angled triangles

    /// The area of a right angled [`Triangle`]
    ///
    /// NOTE: assumed [`Triangle::organise`]d positions
    #[inline]
    pub fn right_angled_area(&self) -> f64 {
        // A = 1/2*b*h
        (self.b.distance(self.c) * (self.a.y - self.b.y)) / 2.0
    }

    /// Updates [`Triangle`]'s vertice positions
    /// such that [`Triangle::a`] is topmost, [`Triangle::b`] is leftmost, and [`Triangle::c`] is rightmost
    #[inline]
    pub fn organise(self) -> Self {
        let t = Self::new(self.topmost(), self.leftmost(), self.rightmost());

        t
    }

    /// Vertex coordinate ([`Point`]) of [`Triangle`] with `x` ordinate
    #[inline]
    pub fn vertex_from_x(&self, x: f64) -> Option<Point> {
        self.as_array().iter().find(|&v| v.x == x).copied()
    }

    /// Vertex coordinate ([`Point`]) of [`Triangle`] with `y` ordinate
    #[inline]
    pub fn vertex_from_y(&self, y: f64) -> Option<Point> {
        self.as_array().iter().find(|&v| v.y == y).copied()
    }

    /// The topmost vertex of `self` as a [`Point`]
    #[inline]
    pub fn topmost(&self) -> Point {
        most_ordering!(self, { |x, y| x.topmost(y) })
    }

    /// The bottommost vertex of `self` as a [`Point`]
    #[inline]
    pub fn bottommost(&self) -> Point {
        most_ordering!(self, { |x, y| x.bottommost(y) })
    }

    /// The rightmost vertex of `self` as a [`Point`]
    #[inline]
    pub fn rightmost(&self) -> Point {
        most_ordering!(self, { |x, y| x.x.total_cmp(&y.x) })
    }

    /// The leftmost vertex of `self` as a [`Point`]
    #[inline]
    pub fn leftmost(&self) -> Point {
        most_ordering!(self, { |x, y| y.x.total_cmp(&x.x) })
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

        // semi perimeter
        let s = (ab + bc + ac) / 2.0;

        // Heron's formula
        let area = (s * (s - ab) * (s - bc) * (s - ac)).sqrt();

        area / s
    }

    /// Expand the triangle by a constant amount (`sizes`) in all directions
    pub fn inflate(&self, size: impl Into<Size>) -> Self {
        let size = size.into();

        Self::new(
            (self.a.to_vec2() + size.to_vec2()).to_point(),
            (self.b.to_vec2() + size.to_vec2()).to_point(),
            (self.c.to_vec2() + size.to_vec2()).to_point(),
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

impl From<(Point, f64)> for Triangle {
    fn from(params: (Point, f64)) -> Triangle {
        Triangle::from_radius(params.1).with_centroid(params.0)
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
        self.perimeter()
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

#[cfg(test)]
mod tests {
    use crate::{Point, Size, Triangle};

    #[test]
    fn from_centroid_sizes() {
        let test = Triangle::from_centroid_distances((0.06, 12.2), [11.9, 1.2, 12.3]);
        let expected = Triangle::from_coords(
            (0.06, 24.1),
            (-0.7885281374238573, 11.351471862576142),
            (8.757413408594534, 3.5025865914054624),
        );

        assert_eq!(test, expected);
    }

    #[test]
    fn from_centroid_distances() {
        let test = Triangle::from_centroid_distances((0.6, 12.2), [11.9, 1.2, 12.3]);
        let expected = Triangle::from_coords(
            (0.6, 24.1),
            (-0.24852813742385726, 11.351471862576142),
            (9.297413408594533, 3.5025865914054624),
        );

        assert_eq!(test, expected);
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
    fn sizes() {
        let test = Triangle::from_coords((-20.0, 180.2), (1.2, 0.0), (290.0, 100.0)).sizes();
        let expected = [
            Size::new(-110.39999999999999, 86.8),
            Size::new(-89.19999999999999, -93.39999999999999),
            Size::new(199.60000000000002, 6.6000000000000085),
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

        assert_eq!(test, expected);
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

        assert_eq!(test, expected);
    }

    #[test]
    fn right_angled_area() {
        let test = Triangle::from_coords((1.2, 5.3), (1.2, 1.6), (10.0, 1.6)).right_angled_area();
        let expected = 16.28;

        assert_eq!(test, expected);
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

        assert_eq!(test, expected);
    }
}
