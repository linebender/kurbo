// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! An ellipse arc.

use crate::{Affine, Ellipse, ParamCurve, ParamCurveDeriv, PathEl, Point, Rect, Shape, Vec2};
use core::{
    f64::consts::{FRAC_PI_2, PI},
    iter,
    ops::{Mul, Range},
};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// A single elliptical arc segment.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Arc {
    /// The arc's centre point.
    pub center: Point,
    /// The arc's radii, where the vector's x-component is the radius in the
    /// positive x direction after applying `x_rotation`.
    pub radii: Vec2,
    /// The start angle in radians.
    pub start_angle: f64,
    /// The angle between the start and end of the arc, in radians.
    pub sweep_angle: f64,
    /// How much the arc is rotated, in radians.
    pub x_rotation: f64,
}

impl Arc {
    /// Create a new `Arc`.
    #[inline(always)]
    pub fn new(
        center: impl Into<Point>,
        radii: impl Into<Vec2>,
        start_angle: f64,
        sweep_angle: f64,
        x_rotation: f64,
    ) -> Self {
        Self {
            center: center.into(),
            radii: radii.into(),
            start_angle,
            sweep_angle,
            x_rotation,
        }
    }

    /// Returns a copy of this `Arc` in the opposite direction.
    ///
    /// The new `Arc` will sweep towards the original `Arc`s
    /// start angle.
    #[must_use]
    #[inline]
    pub fn reversed(&self) -> Arc {
        Self {
            center: self.center,
            radii: self.radii,
            start_angle: self.start_angle + self.sweep_angle,
            sweep_angle: -self.sweep_angle,
            x_rotation: self.x_rotation,
        }
    }

    #[inline]
    fn angle_at(&self, t: f64) -> f64 {
        self.start_angle + self.sweep_angle * t
    }

    /// Create an iterator generating Bezier path elements.
    ///
    /// The generated elements can be appended to an existing bezier path.
    pub fn append_iter(&self, tolerance: f64) -> ArcAppendIter {
        let sign = self.sweep_angle.signum();
        let scaled_err = self.radii.x.max(self.radii.y) / tolerance;
        // Number of subdivisions per ellipse based on error tolerance.
        // Note: this may slightly underestimate the error for quadrants.
        let n_err = (1.1163 * scaled_err).powf(1.0 / 6.0).max(3.999_999);
        let n = (n_err * self.sweep_angle.abs() * (1.0 / (2.0 * PI))).ceil();
        let angle_step = self.sweep_angle / n;
        let n = n as usize;
        let arm_len = (4.0 / 3.0) * (0.25 * angle_step).abs().tan() * sign;
        let angle0 = self.start_angle;
        let p0 = sample_ellipse(self.radii, self.x_rotation, angle0);

        ArcAppendIter {
            idx: 0,

            center: self.center,
            radii: self.radii,
            x_rotation: self.x_rotation,
            n,
            arm_len,
            angle_step,

            p0,
            angle0,
        }
    }

    /// Converts an `Arc` into a series of cubic bezier segments.
    ///
    /// The closure `p` will be invoked with the control points for each segment.
    pub fn to_cubic_beziers<P>(self, tolerance: f64, mut p: P)
    where
        P: FnMut(Point, Point, Point),
    {
        let mut path = self.append_iter(tolerance);
        while let Some(PathEl::CurveTo(p1, p2, p3)) = path.next() {
            p(p1, p2, p3);
        }
    }
}

#[doc(hidden)]
pub struct ArcAppendIter {
    idx: usize,

    center: Point,
    radii: Vec2,
    x_rotation: f64,
    n: usize,
    arm_len: f64,
    angle_step: f64,

    p0: Vec2,
    angle0: f64,
}

impl Iterator for ArcAppendIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.n {
            return None;
        }

        let angle1 = self.angle0 + self.angle_step;
        let p0 = self.p0;
        let p1 = p0
            + self.arm_len * sample_ellipse(self.radii, self.x_rotation, self.angle0 + FRAC_PI_2);
        let p3 = sample_ellipse(self.radii, self.x_rotation, angle1);
        let p2 =
            p3 - self.arm_len * sample_ellipse(self.radii, self.x_rotation, angle1 + FRAC_PI_2);

        self.angle0 = angle1;
        self.p0 = p3;
        self.idx += 1;

        Some(PathEl::CurveTo(
            self.center + p1,
            self.center + p2,
            self.center + p3,
        ))
    }
}

/// Take the ellipse radii, how the radii are rotated, and the sweep angle, and return a point on
/// the ellipse.
fn sample_ellipse(radii: Vec2, x_rotation: f64, angle: f64) -> Vec2 {
    let (angle_sin, angle_cos) = angle.sin_cos();
    let u = radii.x * angle_cos;
    let v = radii.y * angle_sin;
    rotate_pt(Vec2::new(u, v), x_rotation)
}

/// Rotate `pt` about the origin by `angle` radians.
fn rotate_pt(pt: Vec2, angle: f64) -> Vec2 {
    let (angle_sin, angle_cos) = angle.sin_cos();
    Vec2::new(
        pt.x * angle_cos - pt.y * angle_sin,
        pt.x * angle_sin + pt.y * angle_cos,
    )
}

impl ParamCurve for Arc {
    fn eval(&self, t: f64) -> Point {
        self.center + sample_ellipse(self.radii, self.x_rotation, self.angle_at(t))
    }

    fn subsegment(&self, range: Range<f64>) -> Self {
        Self {
            center: self.center,
            radii: self.radii,
            start_angle: self.angle_at(range.start),
            sweep_angle: self.sweep_angle * (range.end - range.start),
            x_rotation: self.x_rotation,
        }
    }
}

/// The "derivative curve" of an [`Arc`].
///
/// Note that, like the documentation of [`ParamCurveDeriv::deriv`] states, implementations on this
/// type are semantically somewhat inaccurate: the derivative of a curve is not, strictly speaking,
/// a curve itself.
///
/// The difference with [`Arc`] is that this object has no `center`: that's a constant that
/// differentiates to zero.
pub struct ArcDeriv {
    /// The arc's radii, where the vector's x-component is the radius in the
    /// positive x direction after applying `x_rotation`.
    pub radii: Vec2,
    /// The start angle in radians.
    pub start_angle: f64,
    /// The angle between the start and end of the arc, in radians.
    pub sweep_angle: f64,
    /// How much the arc is rotated, in radians.
    pub x_rotation: f64,
}

impl ArcDeriv {
    #[inline(always)]
    const fn as_arc_with_center(&self, center: Point) -> Arc {
        Arc {
            center,
            radii: self.radii,
            start_angle: self.start_angle,
            sweep_angle: self.sweep_angle,
            x_rotation: self.x_rotation,
        }
    }
}

impl ParamCurveDeriv for Arc {
    type DerivResult = ArcDeriv;

    fn deriv(&self) -> ArcDeriv {
        // The center is constant over `t` and differentiates away, and as the arc's derivative is
        // represented by another arc (but without center), we can just delegate to `ArcDeriv`
        // directly.
        ArcDeriv {
            radii: self.radii,
            start_angle: self.start_angle,
            sweep_angle: self.sweep_angle,
            x_rotation: self.x_rotation,
        }
        .deriv()
    }
}

impl ParamCurveDeriv for ArcDeriv {
    type DerivResult = ArcDeriv;

    fn deriv(&self) -> ArcDeriv {
        // Given the axis-aligned parametric elliptical arc
        //
        // ```
        // p(a) = center + (radii.x * cos(a), radii.y * sin(a))
        // ```
        //
        // we have tangents
        //
        // ```
        // p'(a) = (-radii.x sin(a), radii.y * cos(a)).
        // ```
        //
        // Our parameterization is over `t` with `a = angle_at(t) = start_angle + sweep_angle * t`.
        // Hence,
        //
        // ```
        // p'(t) = sweep_angle * (-radii.x sin(angle_at(t)), radii.y * cos(angle_at(t)))
        // ```
        //
        // or equivalently
        //
        // ```
        // p'(t) = sweep_angle * (radii.x cos(angle_at(t) + pi/2), radii.y * sin(angle_at(t) + pi/2)).
        // ```
        //
        // Instead of being axis-aligned, our ellipse is rotated by the 2x2 rotation matrix
        // `R(x_rotation)`. This has the effect of rotating the tangents by the same amount. Hence,
        // the derivative is given by
        //
        // ```
        // sweep_angle * R(x_rotation) * (radii.x cos(angle_at(t) + pi/2), radii.y * sin(angle_at(t) + pi/2)).
        // ```
        //
        // This is exactly another `Arc`, but with zero center, precisely what `ArcDeriv` models.
        // As this has the same form again, `ArcDeriv` is closed under repeated differentiation.
        ArcDeriv {
            radii: self.radii * self.sweep_angle,
            start_angle: self.start_angle + FRAC_PI_2,
            sweep_angle: self.sweep_angle,
            x_rotation: self.x_rotation,
        }
    }
}

impl ParamCurve for ArcDeriv {
    fn eval(&self, t: f64) -> Point {
        self.as_arc_with_center(Point::ZERO).eval(t)
    }

    fn subsegment(&self, range: Range<f64>) -> Self {
        let arc = self.as_arc_with_center(Point::ZERO).subsegment(range);

        Self {
            radii: arc.radii,
            start_angle: arc.start_angle,
            sweep_angle: arc.sweep_angle,
            x_rotation: arc.x_rotation,
        }
    }
}

impl Shape for Arc {
    type PathElementsIter<'iter> = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn path_elements(&self, tolerance: f64) -> Self::PathElementsIter<'_> {
        iter::once(PathEl::MoveTo(self.start())).chain(self.append_iter(tolerance))
    }

    /// Note: shape isn't closed so area is not well defined.
    #[inline]
    fn area(&self) -> f64 {
        let Vec2 { x, y } = self.radii;
        PI * x * y
    }

    /// The perimeter of the arc.
    ///
    /// For now we just approximate by using the bezier curve representation.
    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        self.path_segments(0.1).perimeter(accuracy)
    }

    /// Note: shape isn't closed, so a point's winding number is not well defined.
    #[inline]
    fn winding(&self, pt: Point) -> i32 {
        self.path_segments(0.1).winding(pt)
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        self.path_segments(0.1).bounding_box()
    }
}

impl Mul<Arc> for Affine {
    type Output = Arc;

    fn mul(self, arc: Arc) -> Self::Output {
        let ellipse = self * Ellipse::new(arc.center, arc.radii, arc.x_rotation);
        let center = ellipse.center();
        let (radii, rotation) = ellipse.radii_and_rotation();
        Arc {
            center,
            radii,
            x_rotation: rotation,
            start_angle: arc.start_angle,
            sweep_angle: arc.sweep_angle,
        }
    }
}

#[cfg(test)]
mod tests {
    use core::f64::consts::{FRAC_PI_4, FRAC_PI_6};

    use crate::ParamCurve;

    use super::*;

    fn assert_point_near(actual: Point, expected: Point) {
        let epsilon = 1e-12;
        assert!(
            actual.distance(expected) <= epsilon,
            "expected {expected:?}, got {actual:?}"
        );
    }

    fn assert_subsegment_matches(arc: Arc, range: Range<f64>) {
        let subsegment = arc.subsegment(range.clone());
        for t in [0.0, 0.25, 0.5, 1.0] {
            let expected_t = range.start + (range.end - range.start) * t;
            assert_point_near(subsegment.eval(t), arc.eval(expected_t));
        }
        assert_point_near(subsegment.start(), arc.eval(range.start));
        assert_point_near(subsegment.end(), arc.eval(range.end));
    }

    #[test]
    fn reversed_arc() {
        let a = Arc::new((0., 0.), (1., 0.), 0., PI, 0.);
        let f = a.reversed();

        // Most fields should be unchanged:
        assert_eq!(a.center, f.center);
        assert_eq!(a.radii, f.radii);
        assert_eq!(a.x_rotation, f.x_rotation);

        // Sweep angle should be in reverse
        assert_eq!(a.sweep_angle, -f.sweep_angle);

        // Reversing it again should result in the original arc
        assert_eq!(a, f.reversed());
    }

    #[test]
    fn eval_endpoints_and_midpoint() {
        let arc = Arc::new((3.0, -2.0), (4.0, 1.5), FRAC_PI_6, PI, 0.0);
        assert_point_near(arc.eval(0.0), arc.start());
        assert_point_near(arc.eval(1.0), arc.end());
        assert_point_near(
            arc.eval(0.5),
            arc.center + sample_ellipse(arc.radii, arc.x_rotation, arc.angle_at(0.5)),
        );
    }

    #[test]
    fn eval_rotated_ellipse() {
        let arc = Arc::new((5.0, 7.0), (3.0, 2.0), FRAC_PI_6, PI / 3.0, FRAC_PI_4);
        assert_point_near(
            arc.start(),
            arc.center + sample_ellipse(arc.radii, arc.x_rotation, arc.start_angle),
        );
        assert_point_near(
            arc.end(),
            arc.center
                + sample_ellipse(arc.radii, arc.x_rotation, arc.start_angle + arc.sweep_angle),
        );
    }

    #[test]
    fn eval_reversed_arc() {
        let arc = Arc::new((2.0, 3.0), (4.0, 1.0), FRAC_PI_6, PI / 2.0, FRAC_PI_4);
        let reversed = arc.reversed();
        assert_point_near(reversed.start(), arc.end());
        assert_point_near(reversed.end(), arc.start());
        assert_point_near(reversed.eval(0.5), arc.eval(0.5));
    }

    #[test]
    fn subsegment_matches_original() {
        let arc = Arc::new((1.0, -4.0), (3.0, 2.0), -FRAC_PI_4, PI, FRAC_PI_6);
        assert_subsegment_matches(arc, 0.2..0.7);
    }

    #[test]
    fn subsegment_negative_sweep_matches_original() {
        let arc = Arc::new((1.0, 2.0), (5.0, 3.0), PI, -0.75 * PI, FRAC_PI_6);
        assert_subsegment_matches(arc, 0.1..0.8);
    }

    #[test]
    fn subsegment_tiny_sweep_matches_original() {
        let arc = Arc::new((0.0, 0.0), (2.0, 1.0), 1.0, 1e-9, FRAC_PI_4);
        assert_subsegment_matches(arc, 0.25..0.75);
    }

    #[test]
    fn arc_first_and_second_derivatives() {
        let arc = Arc::new((1.0, -2.0), (2.0, 1.0), 0.0, PI, FRAC_PI_2);
        let d1 = arc.deriv();
        let d2 = d1.deriv();

        // Unrotated: (2 cos(pi t), sin(pi t))
        // Rotated by pi/2: (-sin(pi t), 2 cos(pi t))
        // So:
        // p'(t)  = (-pi cos(pi t), -2 pi sin(pi t))
        // p''(t) = (pi^2 sin(pi t), -2 pi^2 cos(pi t))

        assert_point_near(d1.eval(0.0), Point::new(-PI, 0.0));
        assert_point_near(d1.eval(0.5), Point::new(0.0, -2.0 * PI));
        assert_point_near(d1.eval(1.0), Point::new(PI, 0.0));

        assert_point_near(d2.eval(0.0), Point::new(0.0, -2.0 * PI * PI));
        assert_point_near(d2.eval(0.5), Point::new(PI * PI, 0.0));
        assert_point_near(d2.eval(1.0), Point::new(0.0, 2.0 * PI * PI));
    }
}
