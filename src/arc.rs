// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! An ellipse arc.

use crate::{
    ellipse::complete_elliptic_perimeter, Affine, Ellipse, ParamCurve, ParamCurveArclen, PathEl,
    Point, Rect, Shape, Vec2,
};
use core::{
    f64::{
        self,
        consts::{FRAC_PI_2, PI},
    },
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
    fn arclen(&self, accuracy: f64) -> f64 {
        // Normalize ellipse to have radius y >= radius x, required for the parameter assumptions
        // of `incomplete_elliptic_integral_second_kind`.
        let (radii, mut start_angle) = if self.radii.y >= self.radii.x {
            (self.radii, self.start_angle)
        } else {
            (
                Vec2::new(self.radii.y, self.radii.x),
                self.start_angle + PI / 2.,
            )
        };
        let m = 1. - (radii.x / radii.y).powi(2);

        // Normalize sweep angle to be non-negative
        let mut sweep_angle = self.sweep_angle;
        if sweep_angle < 0. {
            start_angle = -start_angle;
            sweep_angle = -sweep_angle;
        }

        // Normalize start angle to be on the upper half of the ellipse
        let start_angle = start_angle.rem_euclid(PI);
        let end_angle = start_angle + sweep_angle;

        let mut quarter_turns = (2. / PI * end_angle).trunc() - (2. / PI * start_angle).trunc();
        let end_angle = end_angle % PI;

        // The elliptic arc length is equal to radii.y * (E(end_angle | m) - E(start_angle | m))
        // with E the incomplete elliptic integral of the second kind and parameter
        // m = 1 - (radii.x / radii.y)^2 = k^2.
        //
        // See also:
        // https://en.wikipedia.org/w/index.php?title=Ellipse&oldid=1248023575#Arc_length
        //
        // The implementation here allows calculating the incomplete elliptic integral in the range
        // 0 <= phi <= 1/2 pi (the first elliptic quadrant), so split the arc into segments in
        // that range.
        let mut arclen = 0.;

        // The available accuracy (tolerance) is distributed over the calculation of the two
        // incomplete and one complete elliptic integrals.
        let accuracy_per_incomplete_integral = 1. / 3. * accuracy / radii.y;
        if start_angle >= PI / 2. {
            arclen += incomplete_elliptic_integral_second_kind(
                accuracy_per_incomplete_integral,
                PI - start_angle,
                m,
            );
            quarter_turns -= 1.;
        } else {
            arclen -= incomplete_elliptic_integral_second_kind(
                accuracy_per_incomplete_integral,
                start_angle,
                m,
            );
        }

        if end_angle >= PI / 2. {
            arclen -= incomplete_elliptic_integral_second_kind(
                accuracy_per_incomplete_integral,
                PI - end_angle,
                m,
            );
            quarter_turns += 1.;
        } else {
            arclen += incomplete_elliptic_integral_second_kind(
                accuracy_per_incomplete_integral,
                end_angle,
                m,
            );
        }
        arclen *= radii.y;

        arclen += 1. / 4.
            * quarter_turns
            * complete_elliptic_perimeter(radii, 1. / 4. / 3. * accuracy * quarter_turns.max(1.));

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

/// Approximation of the Carlson RF function as defined in "Numerical computation of real or complex
/// elliptic integrals" (Carlson, Bille C.): <https://arxiv.org/abs/math/9409227v1>
///
/// RF = 1/2 ∫ 1 / ( sqrt(t+x) sqrt(t+y) sqrt(t+z) ) dt from 0 to inf
fn carlson_rf(accuracy: f64, x: f64, y: f64, z: f64) -> f64 {
    // At most one of (x, y, z) may be 0.
    debug_assert!((x == 0.) as u8 + (y == 0.) as u8 + (z == 0.) as u8 <= 1);

    // This mostly follows "Numerical computation of real or complex elliptic integrals", but using
    // an absolute upper error bound rather than a relative one.
    //
    // From "Numerical computation of real or complex elliptic integrals" we have
    //
    // X_n = (a_0 - x_0) / (4^n a_n)
    // (and the same for variables (Y,y), (Z,z)).
    //
    // From "Computing Elliptic Integrals by Duplication" we have an upper error bound of
    //
    // |err_n| < a_n^(-1/2) epsilon_n^6 / (4 (1 - epsilon_n))
    // with epsilon_n = max(X_n, Y_n, Z_n)
    //                = max(a_0 - x_0, a_0 - y_0, a_0 - z_0) / (4^n a_n).
    //
    // Define e_0 = max(a_0 - x_0, a_0 - y_0, a_0 - z_0). Rewrite for ease of computation,
    //
    //    |err_n| < a_n^(-1/2) epsilon_n^6 / (4 (1 - epsilon_n))
    //            = a_n^(-1/2) e_0^6 / (4^n a_n)^6 / (4 (1 - epsilon_n))
    // -> |err_n| a_n^(1/2) (4^n a_n)^6  / e_0^6 < 1 / (4 (1 - epsilon_n))
    // -> |err_n| a_n^(1/2) a_n^6 4^(6n + 1) / e_0^6 < 1 / (1 - epsilon_n)
    // -> |err_n| a_n^(1/2) a_n^6 4^(6n + 1) / e_0^6 (1 - epsilon_n) < 1.
    //
    // To reach an error upper bound of `accuracy`, iterate until
    // 1 <= accuracy * a_n^(1/2) a_n^6 4^(6n + 1) / e_0^6 (1 - epsilon_n).

    let mut x = x;
    let mut y = y;
    let mut z = z;

    let mut a = (x + y + z) * (1. / 3.);

    // These are partial terms of the inequality derived above. The multiply by (powers of) 4 are
    // performed per iteration for computational efficiency.
    let mut e = a - x.min(y).min(z);
    let mut r = accuracy * 4. * e.powi(-6);

    loop {
        if 1. <= r * a.powi(6) * a.sqrt() * (1. - e / a) {
            break;
        }

        let lambda = (x * y).sqrt() + (x * z).sqrt() + (y * z).sqrt();
        a = (a + lambda) / 4.;
        x = (x + lambda) / 4.;
        y = (y + lambda) / 4.;
        z = (z + lambda) / 4.;

        r *= 4f64.powi(6);
        e /= 4.;
    }

    let x = 1. - x / a;
    let y = 1. - y / a;
    let z = -x - y;

    let e2 = x * y - z.powi(2);
    let e3 = x * y * z;

    (1. + (-1. / 10. * e2 + 1. / 14. * e3 + 1. / 24. * e2.powi(2) - 3. / 44. * e2 * e3)) / a.sqrt()
}

/// Approximation of the Carlson RD function as defined in "Numerical computation of real or
/// complex elliptic integrals" (Carlson, Bille C.): <https://arxiv.org/abs/math/9409227v1>
///
/// RD = 3/2 ∫ 1 / ( sqrt(t+x) sqrt(t+y) (t+z)^(3/2) ) dt from 0 to inf
fn carlson_rd(accuracy: f64, x: f64, y: f64, z: f64) -> f64 {
    // At most one of (x, y) may be 0, z must be nonzero.
    debug_assert!(z != 0.);
    debug_assert!(x != 0. || y != 0.);

    // As above for RF, find the absolute upper error bound rather than a relative one, the
    // derivation of which is along the same lines.
    //
    // Again,
    //
    // X_n = (a_0 - x_0) / (4^n a_n)
    // (and the same for variables (Y,y), (Z,z)).
    //
    // From "Computing Elliptic Integrals by Duplication" we have
    //
    // |err_n| < 4^-n a_n^(-3/2) 3 epsilon_n^6 / (1 - epsilon_n)^(3/2)
    // with epsilon_n = max(X_n, Y_n, Z_n)
    //                = max(a_0 - x_0, a_0 - y_0, a_0 - z_0) / (4^n a_n).
    //
    // Define e_0 = max(a_0 - x_0, a_0 - y_0, a_0 - z_0). Rewriting for ease of computation,
    //
    // |err_n| < 4^-n a_n^(-3/2) 3 epsilon_n^6 / (1 - epsilon_n)^(3/2)
    //         = 4^-n a_n^(-3/2) 3 e_0^6 / 4^(6n) a_n^6 / (1 - epsilon_n)^(3/2)
    // -> |err_n| 4^(7n) a_n^(3/2) a_n^6 / (3 e_0^6) < 1 / (1 - epsilon_n)^(3/2)
    // -> |err_n| 4^(7n) a_n^(3/2) a_n^6 (1/3) / e_0^6 < (1 / 1 - epsilon_n)^(3/2),
    // raise to the power 2/3,
    // -> |err_n|^(2/3) 4^(14/3 n) a_n a_n^4 (1/3)^(2/3) / e_0^4 < 1 / (1 - epsilon_n)
    // -> |err_n|^(2/3) 4^(14/3 n) a_n^5 (1/3)^(2/3) / e_0^4 (1 - epsilon_n) < 1
    //
    // That means, to reach an error upper bound of `accuracy`, iterate until
    // 1 <= accuracy^(2/3) 4^(14/3 n) a_n^5 (1/3)^(2/3) / e_0^4 (1 - epsilon)

    let mut x = x;
    let mut y = y;
    let mut z = z;

    let a0 = (x + y + 3. * z) * (1. / 5.);
    let mut a = a0;

    let mut sum = 0.;
    let mut mul = 1.;

    // These are partial terms of the inequality derived above. The multiply by (powers of) 4 are
    // performed per iteration for computational efficiency.
    let mut e = a - x.min(y).min(z);
    let mut r = (accuracy / 3.).powf(2. / 3.) * e.powi(-4);

    loop {
        if 1. <= r * a.powi(5) * (1. - e / a) {
            break;
        }

        let lambda = (x * y).sqrt() + (x * z).sqrt() + (y * z).sqrt();
        sum += mul / (z.sqrt() * (z + lambda));
        a = (a + lambda) / 4.;
        x = (x + lambda) / 4.;
        y = (y + lambda) / 4.;
        z = (z + lambda) / 4.;

        r *= 4f64.powf(14. / 3.);
        e /= 4.;
        mul /= 4.;
    }

    let x = 1. - x / a;
    let y = 1. - y / a;
    let z = (-x - y) / 3.;

    let e2 = x * y - 6. * z.powi(2);
    let e3 = (3. * x * y - 8. * z.powi(2)) * z;
    let e4 = 3. * (x * y - z.powi(2)) * z.powi(2);
    let e5 = x * y * z.powi(3);

    (1. - 3. / 14. * e2 + 1. / 6. * e3 + 9. / 88. * e2.powi(2) - 3. / 22. * e4 - 9. / 52. * e2 * e3
        + 3. / 26. * e5)
        * mul
        / (a * a.sqrt())
        + 3. * sum
}

/// Numerically approximate the incomplete elliptic integral of the second kind from 0 to `phi`
/// parameterized by `m = k^2` in Legendre's trigonometric form.
///
/// The absolute error between the calculated integral and the true integral is bounded by
/// `accuracy` (modulo floating point rounding errors).
///
/// Assumes:
/// 0 <= phi <= pi / 2
/// and 0 <= m sin^2(phi) <= 1
fn incomplete_elliptic_integral_second_kind(accuracy: f64, phi: f64, m: f64) -> f64 {
    // Approximate the incomplete elliptic integral through Carlson symmetric forms:
    // https://en.wikipedia.org/w/index.php?title=Carlson_symmetric_form&oldid=1223277638#Incomplete_elliptic_integrals

    debug_assert!(phi >= -PI / 2.);
    debug_assert!(phi <= PI / 2.);
    debug_assert!(m * phi.sin().powi(2) >= 0.);
    debug_assert!(m * phi.sin().powi(2) <= 1.);

    let (sin, cos) = phi.sin_cos();
    let sin2 = sin.powi(2);
    let sin3 = sin.powi(3);
    let cos2 = cos.powi(2);

    // note: this actually allows calculating from -1/2 pi <= phi <= 1/2 pi, but there are some
    // alternative translations from the Legendre form that are potentially better, that do
    // restrict the domain to 0 <= phi <= 1/2 pi.
    let term1 = if sin == 0. {
        0.
    } else {
        sin * carlson_rf(
            accuracy / (2. * sin),
            // 1e-30,
            cos2,
            1. - m * sin2,
            1.,
        )
    };

    let term2 = if sin == 0. || m == 0. {
        0.
    } else {
        1. / 3. * m * sin3 * carlson_rd(accuracy * (3. / 2.) / (m * sin3), cos2, 1. - m * sin2, 1.)
    };

    term1 - term2
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
        // Circular checks:
        for (start_angle, sweep_angle, length) in [
            (0., 1., 1.),
            (0., 2., 2.),
            (0., 5., 5.),
            (1.0, 3., 3.),
            (1.5, 10., 10.),
            (2.5, 10., 10.),
        ] {
            let a = Arc::new((0., 0.), (1., 1.), start_angle, sweep_angle, 0.);
            let arc_length = a.arclen(1e-7);
            assert!(
                (arc_length - length).abs() <= 1e-6,
                "Got arc length {arc_length}, expected {length} for circular arc {a:?}"
            );
        }

        let a = Arc::new((0., 0.), (1., 1.), 0., PI * 4., 0.);
        assert!((a.arclen(1e-13) - PI * 4.).abs() <= 1e-12);

        let a = Arc::new((0., 0.), (2.23, 3.05), 0., 0.2, 0.);
        assert!((a.arclen(1e-13) - 0.608_117_142_773_153_8).abs() <= 1e-12);

        let a = Arc::new((0., 0.), (3.05, 2.23), 0., 0.2, 0.);
        assert!((a.arclen(1e-13) - 0.448_554_961_296_305_9).abs() <= 1e-12);
    }

    #[test]
    fn length_compare_with_bez_length() {
        for radii in [(1., 1.), (0.5, 1.), (2., 1.)] {
            for start_angle in [0., 0.5, 1., 2., PI, -1.] {
                for sweep_angle in [0., 0.5, 1., 2., PI, -1.] {
                    let a = Arc::new((0., 0.), radii, start_angle, sweep_angle, 0.);

                    let arc_length = a.arclen(1e-8);
                    let bez_length = a.path_segments(1e-8).perimeter(1e-8);

                    assert!(
                        (arc_length - bez_length).abs() < 1e-7,
                        "Numerically approximated arc length ({arc_length}) does not match bezier segment perimeter length ({bez_length}) for arc {a:?}"
                        );
                }
            }
        }
    }

    #[test]
    fn carlson_numerical_checks() {
        // Numerical checks from section 3 of "Numerical computation of real or complex elliptic
        // integrals" (Carlson, Bille C.): https://arxiv.org/abs/math/9409227v1 (real-valued calls)
        assert!((carlson_rf(1e-13, 1., 2., 0.) - 1.311_028_777_146_1).abs() <= 1e-12);
        assert!((carlson_rf(1e-13, 2., 3., 4.) - 0.584_082_841_677_15).abs() <= 1e-12);

        assert!((carlson_rd(1e-13, 0., 2., 1.) - 1.797_210_352_103_4).abs() <= 1e-12);
        assert!((carlson_rd(1e-13, 2., 3., 4.) - 0.165_105_272_942_61).abs() <= 1e-12);
    }

    #[test]
    fn elliptic_e_numerical_checks() {
        for (phi, m, elliptic_e) in [
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.5),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (1.0, 1.0, 0.841_470_984_807_896_5),
        ] {
            let elliptic_e_approx = incomplete_elliptic_integral_second_kind(1e-13, phi, m);
            assert!(
                (elliptic_e_approx - elliptic_e).abs() < 1e-12,
                "Approximated elliptic e {elliptic_e_approx} does not match known value {elliptic_e} for E({phi}|{m})"
            );
        }
    }
}
