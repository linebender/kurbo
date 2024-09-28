// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! An ellipse arc.

use crate::{Affine, Ellipse, ParamCurve, ParamCurveArclen, PathEl, Point, Rect, Shape, Vec2};
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
        let angle = self.start_angle + (self.sweep_angle * t);
        sample_ellipse(self.radii, self.x_rotation, angle).to_point()
    }

    fn subsegment(&self, range: Range<f64>) -> Self {
        Self {
            center: self.center,
            radii: self.radii,
            start_angle: self.start_angle + (self.sweep_angle * range.start),
            sweep_angle: self.sweep_angle - (self.sweep_angle * (range.end - range.start)),
            x_rotation: self.x_rotation,
        }
    }

    fn start(&self) -> Point {
        sample_ellipse(self.radii, self.x_rotation, self.start_angle).to_point()
    }

    fn end(&self) -> Point {
        sample_ellipse(
            self.radii,
            self.x_rotation,
            self.start_angle + self.sweep_angle,
        )
        .to_point()
    }
}

impl ParamCurveArclen for Arc {
    fn arclen(&self, _accuracy: f64) -> f64 {
        // TODO: wire up accuracy. The Carlson numerical approximation provides a bound on the relative
        // error
        let relative_error = 1e-20;

        // Normalize ellipse to have radius x >= radius y
        let (radii, mut start_angle) = if self.radii.x >= self.radii.y {
            (self.radii, self.start_angle)
        } else {
            (
                Vec2::new(self.radii.y, self.radii.x),
                self.start_angle + PI / 2.,
            )
        };

        // Normalize sweep angle to be non-negative
        let mut sweep_angle = self.sweep_angle;
        if sweep_angle < 0. {
            start_angle -= sweep_angle;
            sweep_angle = -sweep_angle;
        }

        start_angle = start_angle.rem_euclid(PI);

        let mut arclen = 0.;
        let half_turns = (sweep_angle / PI).floor();
        if half_turns > 0. {
            // Half of the ellipse circumference is a complete elliptic integral and could be special-cased
            arclen += half_turns * half_ellipse_arc_length(relative_error, radii, 0., PI);
            sweep_angle = sweep_angle.rem_euclid(PI);
        }

        if start_angle + sweep_angle > PI {
            arclen += half_ellipse_arc_length(relative_error, radii, start_angle, PI);
            arclen +=
                half_ellipse_arc_length(relative_error, radii, 0., start_angle + sweep_angle - PI);
        } else {
            arclen += half_ellipse_arc_length(
                relative_error,
                radii,
                start_angle,
                start_angle + sweep_angle,
            );
        }

        arclen
    }
}

impl Shape for Arc {
    type PathElementsIter<'iter> = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn path_elements(&self, tolerance: f64) -> Self::PathElementsIter<'_> {
        let p0 = sample_ellipse(self.radii, self.x_rotation, self.start_angle);
        iter::once(PathEl::MoveTo(self.center + p0)).chain(self.append_iter(tolerance))
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

/// Approximation of Carlson RF function as defined in "Numerical computation of real or complex
/// elliptic integrals" (Carlson, Bille C.): https://arxiv.org/abs/math/9409227v1
fn carlson_rf(relative_error: f64, x: f64, y: f64, z: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    let mut z = z;

    let a0 = (x + y + z) / 3.;
    let q = (3. * relative_error).powf(-1. / 6.)
        * (a0 - x).abs().max((a0 - y).abs()).max((a0 - z).abs());

    let mut a = a0;
    let mut m = 0;
    loop {
        if 4f64.powi(-m) * q <= a.abs() {
            break;
        }

        let lambda = x.sqrt() * y.sqrt() + x.sqrt() * z.sqrt() + y.sqrt() * z.sqrt();

        a = (a + lambda) / 4.;
        x = (x + lambda) / 4.;
        y = (y + lambda) / 4.;
        z = (z + lambda) / 4.;

        m += 1;
    }

    let x = (a0 - x) / 4f64.powi(m) * a;
    let y = (a0 - y) / 4f64.powi(m) * a;
    let z = -x - y;

    let e2 = x * y - z.powi(2);
    let e3 = x * y * z;

    a.powf(-1. / 2.)
        * (1. - 1. / 10. * e2 + 1. / 14. * e3 + 1. / 24. * e2.powi(2) - 3. / 44. * e2 * e3)
}

/// Approximation of Carlson RD function as defined in "Numerical computation of real or complex
/// elliptic integrals" (Carlson, Bille C.): https://arxiv.org/abs/math/9409227v1
fn carlson_rd(relative_error: f64, x: f64, y: f64, z: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    let mut z = z;

    let a0 = (x + y + 3. * z) / 5.;
    let q = (relative_error / 4.).powf(-1. / 6.)
        * (a0 - x).abs().max((a0 - y).abs()).max((a0 - z).abs());

    let mut sum = 0.;
    let mut a = a0;
    let mut m = 0;
    loop {
        if 4f64.powi(-m) * q <= a.abs() {
            break;
        }

        let lambda = x.sqrt() * y.sqrt() + x.sqrt() * z.sqrt() + y.sqrt() * z.sqrt();
        sum += 4f64.powi(-m) / (z.sqrt() * (z + lambda));
        a = (a + lambda) / 4.;
        x = (x + lambda) / 4.;
        y = (y + lambda) / 4.;
        z = (z + lambda) / 4.;

        m += 1;
    }

    let x = (a0 - x) / (4f64.powi(4) * a);
    let y = (a0 - y) / (4f64.powi(4) * a);
    let z = -(x + y) / 3.;

    let e2 = x * y - 6. * z.powi(2);
    let e3 = (3. * x * y - 8. * z.powi(2)) * z;
    let e4 = 3. * x * y - z.powi(2) * z.powi(2);
    let e5 = x * y * z.powi(3);

    4f64.powi(-m)
        * a.powf(-3. / 2.)
        * (1. - 3. / 14. * e2 + 1. / 6. * e3 + 9. / 88. * e2.powi(2)
            - 3. / 22. * e4
            - 9. / 52. * e2 * e3
            + 3. / 26. * e5)
        + 3. * sum
}

/// Numerically approximate the incomplete elliptic integral of the second kind
/// parameterized by `phi` and `m = k^2` in Legendre's trigonometric form.
fn incomplete_elliptic_integral_second_kind(relative_error: f64, phi: f64, m: f64) -> f64 {
    // Approximate the incomplete elliptic integral through Carlson symmetric forms:
    // https://en.wikipedia.org/w/index.php?title=Carlson_symmetric_form&oldid=1223277638#Incomplete_elliptic_integrals

    debug_assert!(phi >= -PI / 2.);
    debug_assert!(phi <= PI / 2.);
    debug_assert!(m * phi.sin().powi(2) >= 0.);
    debug_assert!(m * phi.sin().powi(2) <= 1.);

    phi.sin()
        * carlson_rf(
            relative_error,
            phi.cos().powi(2),
            1. - m * phi.sin().powi(2),
            1.,
        )
        - 1. / 3.
            * m
            * phi.sin().powi(3)
            * carlson_rd(
                relative_error,
                phi.cos().powi(2),
                1. - m * phi.sin().powi(2),
                1.,
            )
}

/// Calculate the length of an arc along an ellipse defined by `radii`, from `start_angle`
/// to `end_angle`, with the angles being inside the first two quadrants, i.e.,
/// 0 <= start_angle <= end_angle <= PI.
///
/// This assumes radii.x >= radii.y
fn half_ellipse_arc_length(
    relative_error: f64,
    radii: Vec2,
    start_angle: f64,
    end_angle: f64,
) -> f64 {
    debug_assert!(radii.x >= radii.y);
    debug_assert!(start_angle >= 0.);
    debug_assert!(end_angle >= start_angle);
    debug_assert!(end_angle <= PI);

    let radii = Vec2::new(radii.y, radii.x);
    let start_angle = start_angle - PI / 2.;
    let end_angle = end_angle - PI / 2.;

    // Ellipse arc length calculated through the incomplete elliptic integral of the second
    // kind:
    // https://en.wikipedia.org/w/index.php?title=Ellipse&oldid=1248023575#Arc_length

    let m = 1. - (radii.x / radii.y).powi(2);

    radii.y
        * (incomplete_elliptic_integral_second_kind(relative_error, end_angle, m)
            - incomplete_elliptic_integral_second_kind(relative_error, start_angle, m))
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn length() {
        // TODO: when arclen actually uses specified accuracy, update EPSILON and the accuracy
        // params
        const EPSILON: f64 = 1e-6;

        // Circular checks:
        for (start_angle, sweep_angle, length) in [
            (0., 1., 1.),
            (0., 2., 2.),
            (0., 5., 5.),
            (1.0, 3., 3.),
            (1.5, 10., 10.),
        ] {
            let a = Arc::new((0., 0.), (1., 1.), start_angle, sweep_angle, 0.);
            let arc_length = a.arclen(0.000_1);
            assert!(
                (arc_length - length).abs() <= EPSILON,
                "Got arc length {arc_length}, expected {length} for circular arc {a:?}"
            );
        }

        let a = Arc::new((0., 0.), (1., 1.), 0., PI * 4., 0.);
        assert!((a.arclen(0.000_1) - PI * 4.).abs() <= EPSILON);

        let a = Arc::new((0., 0.), (2.23, 3.05), 0., 0.2, 0.);
        assert!((a.arclen(0.000_1) - 0.60811714277).abs() <= EPSILON);

        let a = Arc::new((0., 0.), (3.05, 2.23), 0., 0.2, 0.);
        assert!((a.arclen(0.000_1) - 0.448555).abs() <= EPSILON);
    }

    #[test]
    fn carlson_numerical_checks() {
        // TODO: relative bound on error doesn't seem to be quite correct yet, use a large epsilon
        // for now
        const EPSILON: f64 = 1e-6;

        // Numerical checks from section 3 of "Numerical computation of real or complex elliptic
        // integrals" (Carlson, Bille C.): https://arxiv.org/abs/math/9409227v1 (real-valued calls)
        assert!((carlson_rf(1e-20, 1., 2., 0.) - 1.3110_28777_1461).abs() <= EPSILON);
        assert!((carlson_rf(1e-20, 2., 3., 4.) - 0.58408_28416_7715).abs() <= EPSILON);

        assert!((carlson_rd(1e-20, 0., 2., 1.) - 1.7972_10352_1034).abs() <= EPSILON);
        assert!((carlson_rd(1e-20, 2., 3., 4.) - 0.16510_52729_4261).abs() <= EPSILON);
    }

    #[test]
    fn elliptic_e_numerical_checks() {
        const EPSILON: f64 = 1e-6;

        for (phi, m, elliptic_e) in [
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.5),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (1.0, 1.0, 0.84147098),
        ] {
            let elliptic_e_approx = incomplete_elliptic_integral_second_kind(1e-20, phi, m);
            assert!(
                (elliptic_e_approx - elliptic_e).abs() < EPSILON,
                "Approximated elliptic e {elliptic_e_approx} does not match known value {elliptic_e} for E({phi}|{m})"
            );
        }
    }
}
