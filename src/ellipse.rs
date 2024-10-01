// Copyright 2020 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Implementation of ellipse shape.

use core::f64::consts::PI;
use core::{
    iter,
    ops::{Add, Mul, Sub},
};

use crate::{Affine, Arc, ArcAppendIter, Circle, PathEl, Point, Rect, Shape, Size, Vec2};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// An ellipse.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    /// All ellipses can be represented as an affine map of the unit circle,
    /// centred at (0, 0). Therefore we can store the ellipse as an affine map,
    /// with the implication that it be applied to the unit circle to recover the
    /// actual shape.
    inner: Affine,
}

impl Ellipse {
    /// Create A new ellipse with a given center, radii, and rotation.
    ///
    /// The returned ellipse will be the result of taking a circle, stretching
    /// it by the `radii` along the x and y axes, then rotating it from the
    /// x axis by `rotation` radians, before finally translating the center
    /// to `center`.
    ///
    /// Rotation is clockwise in a y-down coordinate system. For more on
    /// rotation, see [`Affine::rotate`].
    #[inline]
    pub fn new(center: impl Into<Point>, radii: impl Into<Vec2>, x_rotation: f64) -> Ellipse {
        let Point { x: cx, y: cy } = center.into();
        let Vec2 { x: rx, y: ry } = radii.into();
        Ellipse::private_new(Vec2 { x: cx, y: cy }, rx, ry, x_rotation)
    }

    /// Returns the largest ellipse that can be bounded by this [`Rect`].
    ///
    /// This uses the absolute width and height of the rectangle.
    ///
    /// This ellipse is always axis-aligned; to apply rotation you can call
    /// [`with_rotation`] with the result.
    ///
    /// [`with_rotation`]: Ellipse::with_rotation
    #[inline]
    pub fn from_rect(rect: Rect) -> Self {
        let center = rect.center().to_vec2();
        let Size { width, height } = rect.size() / 2.0;
        Ellipse::private_new(center, width, height, 0.0)
    }

    /// Create an ellipse from an affine transformation of the unit circle.
    #[inline]
    pub fn from_affine(affine: Affine) -> Self {
        Ellipse { inner: affine }
    }

    /// Create a new `Ellipse` centered on the provided point.
    #[inline]
    #[must_use]
    pub fn with_center(self, new_center: Point) -> Ellipse {
        let Point { x: cx, y: cy } = new_center;
        Ellipse {
            inner: self.inner.with_translation(Vec2 { x: cx, y: cy }),
        }
    }

    /// Create a new `Ellipse` with the provided radii.
    #[must_use]
    pub fn with_radii(self, new_radii: Vec2) -> Ellipse {
        let rotation = self.inner.svd().1;
        let translation = self.inner.translation();
        Ellipse::private_new(translation, new_radii.x, new_radii.y, rotation)
    }

    /// Create a new `Ellipse`, with the rotation replaced by `rotation`
    /// radians.
    ///
    /// The rotation is clockwise, for a y-down coordinate system. For more
    /// on rotation, See [`Affine::rotate`].
    #[must_use]
    pub fn with_rotation(self, rotation: f64) -> Ellipse {
        let scale = self.inner.svd().0;
        let translation = self.inner.translation();
        Ellipse::private_new(translation, scale.x, scale.y, rotation)
    }

    #[deprecated(since = "0.7.0", note = "use with_rotation instead")]
    #[must_use]
    #[doc(hidden)]
    pub fn with_x_rotation(self, rotation_radians: f64) -> Ellipse {
        self.with_rotation(rotation_radians)
    }

    /// This gives us an internal method without any type conversions.
    #[inline]
    fn private_new(center: Vec2, scale_x: f64, scale_y: f64, x_rotation: f64) -> Ellipse {
        // Since the circle is symmetric about the x and y axes, using absolute values for the
        // radii results in the same ellipse. For simplicity we make this change here.
        Ellipse {
            inner: Affine::translate(center)
                * Affine::rotate(x_rotation)
                * Affine::scale_non_uniform(scale_x.abs(), scale_y.abs()),
        }
    }

    // Getters and setters.

    /// Returns the center of this ellipse.
    #[inline]
    pub fn center(&self) -> Point {
        let Vec2 { x: cx, y: cy } = self.inner.translation();
        Point { x: cx, y: cy }
    }

    /// Returns the two radii of this ellipse.
    ///
    /// The first number is the horizontal radius and the second is the vertical
    /// radius, before rotation.
    pub fn radii(&self) -> Vec2 {
        self.inner.svd().0
    }

    /// The ellipse's rotation, in radians.
    ///
    /// This allows all possible ellipses to be drawn by always starting with
    /// an ellipse with the two radii on the x and y axes.
    pub fn rotation(&self) -> f64 {
        self.inner.svd().1
    }

    /// Returns the radii and the rotation of this ellipse.
    ///
    /// Equivalent to `(self.radii(), self.rotation())` but more efficient.
    pub fn radii_and_rotation(&self) -> (Vec2, f64) {
        self.inner.svd()
    }

    /// Is this ellipse [finite]?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.inner.is_finite()
    }

    /// Is this ellipse [NaN]?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.inner.is_nan()
    }

    #[doc(hidden)]
    #[deprecated(since = "0.7.0", note = "use rotation() instead")]
    pub fn x_rotation(&self) -> f64 {
        self.rotation()
    }
}

impl Add<Vec2> for Ellipse {
    type Output = Ellipse;

    /// In this context adding a `Vec2` applies the corresponding translation to the ellipse.
    #[inline]
    #[allow(clippy::suspicious_arithmetic_impl)]
    fn add(self, v: Vec2) -> Ellipse {
        Ellipse {
            inner: Affine::translate(v) * self.inner,
        }
    }
}

impl Sub<Vec2> for Ellipse {
    type Output = Ellipse;

    /// In this context subtracting a `Vec2` applies the corresponding translation to the ellipse.
    #[inline]
    fn sub(self, v: Vec2) -> Ellipse {
        Ellipse {
            inner: Affine::translate(-v) * self.inner,
        }
    }
}

impl Mul<Ellipse> for Affine {
    type Output = Ellipse;
    fn mul(self, other: Ellipse) -> Self::Output {
        Ellipse {
            inner: self * other.inner,
        }
    }
}

impl From<Circle> for Ellipse {
    fn from(circle: Circle) -> Self {
        Ellipse::new(circle.center, Vec2::splat(circle.radius), 0.0)
    }
}

impl Shape for Ellipse {
    type PathElementsIter<'iter> = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn path_elements(&self, tolerance: f64) -> Self::PathElementsIter<'_> {
        let (radii, x_rotation) = self.inner.svd();
        Arc {
            center: self.center(),
            radii,
            start_angle: 0.0,
            sweep_angle: 2.0 * PI,
            x_rotation,
        }
        .path_elements(tolerance)
    }

    #[inline]
    fn area(&self) -> f64 {
        let Vec2 { x, y } = self.radii();
        PI * x * y
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        let radii = self.radii();

        if radii.is_nan() {
            return f64::NAN;
        }

        // Check for the trivial case where the ellipse has radii (0,0), as the numerical method
        // used breaks down with this extreme.
        if radii == Vec2::ZERO {
            return 0.;
        }

        kummer_elliptic_perimeter(accuracy, radii)
    }

    fn winding(&self, pt: Point) -> i32 {
        // Strategy here is to apply the inverse map to the point and see if it is in the unit
        // circle.
        let inv = self.inner.inverse();
        if (inv * pt).to_vec2().hypot2() < 1.0 {
            1
        } else {
            0
        }
    }

    // Compute a tight bounding box of the ellipse.
    //
    // See https://www.iquilezles.org/www/articles/ellipses/ellipses.htm. We can get the two
    // radius vectors by applying the affine map to the two impulses (1, 0) and (0, 1) which gives
    // (a, b) and (c, d) if the affine map is
    //
    //  a | c | e
    // -----------
    //  b | d | f
    //
    //  We can then use the method in the link with the translation to get the bounding box.
    #[inline]
    fn bounding_box(&self) -> Rect {
        let aff = self.inner.as_coeffs();
        let a2 = aff[0] * aff[0];
        let b2 = aff[1] * aff[1];
        let c2 = aff[2] * aff[2];
        let d2 = aff[3] * aff[3];
        let cx = aff[4];
        let cy = aff[5];
        let range_x = (a2 + c2).sqrt();
        let range_y = (b2 + d2).sqrt();
        Rect {
            x0: cx - range_x,
            y0: cy - range_y,
            x1: cx + range_x,
            y1: cy + range_y,
        }
    }
}

/// Calculates circumference C of an ellipse with radii (x, y) as the infinite series
///
/// C = pi (x+y) * ∑ binom(1/2, n)^2 * h^n from n = 0 to inf
/// with h = (x - y)^2 / (x + y)^2
/// and binom(.,.) the binomial coefficient
///
/// as described by Kummer ("Über die Hypergeometrische Reihe", 1837) and rediscovered by
/// Linderholm and Segal ("An Overlooked Series for the Elliptic Perimeter", 1995).
///
/// This converges quadratically in the worst case, and converges very quickly for ellipses with
/// only moderate eccentricity (`h` not close to 1).
fn kummer_elliptic_perimeter(accuracy: f64, radii: Vec2) -> f64 {
    let Vec2 { x, y } = radii;

    let accuracy = accuracy / (PI * (x + y));

    let mut h = (x - y).powi(2) / (x + y).powi(2);
    let mut sum = 1.;
    let mut m = 1;

    // The binomial coefficient binom(1/2, n), calculated as
    // ∏ (1.5 - i) / i for i = 1 to n
    let mut binom = 1.;

    loop {
        binom *= (1.5 - m as f64) / m as f64;
        let binom2 = binom.powi(2);
        let term = binom2 * h;
        sum += term;

        m += 1;
        h *= h;

        // Stop iterating when the terms become small enough.
        //
        // Perhaps there's a somewhat stricter bound we could use, but I believe
        // binom(1/2, x)^2 ≥ ∑ binom(1/2, n)^2 for n = x+1 to inf,
        // and the following terms are multiplied by `h` or less.
        if binom2 * h <= accuracy {
            break;
        }
    }

    PI * (x + y) * sum
}

#[cfg(test)]
mod tests {
    use crate::{Circle, Ellipse, Point, Shape};
    use std::f64::consts::PI;

    fn assert_approx_eq(x: f64, y: f64) {
        // Note: we might want to be more rigorous in testing the accuracy
        // of the conversion into Béziers. But this seems good enough.
        assert!((x - y).abs() < 1e-7, "{x} != {y}");
    }

    #[test]
    fn circular_perimeter() {
        for radius in [1.0, 0., 1.5, PI, 10.0, -1.0, 1_234_567_890.1] {
            let circle = Circle::new((0., 0.), radius);
            let ellipse = Ellipse::new((0., 0.), (radius, radius), 0.);

            let circle_p = circle.perimeter(0.);
            let ellipse_p = ellipse.perimeter(0.000_000_000_000_1);

            assert!(
                 (circle_p - ellipse_p).abs() <= 0.000_000_000_000_1,
                 "Expected circular ellipse radius {ellipse_p} to be equal to circle radius {circle_p} for radius {radius}"
             )
        }
    }

    #[test]
    fn bez_perimeter() {
        const EPSILON: f64 = 1e-3;

        for radii in [(0.5, 1.), (2., 1.), (0.000_000_1, 1.), (0., 1.), (1., 0.)] {
            let ellipse = Ellipse::new((0., 0.), radii, 0.);

            let ellipse_p = ellipse.perimeter(0.000_1);
            let bez_p = ellipse.path_segments(0.000_05).perimeter(0.000_05);

            assert!(
                 (ellipse_p - bez_p).abs() < EPSILON,
                 "Numerically approximated ellipse perimeter ({ellipse_p}) does not match bezier segment perimeter length ({bez_p}) for radii {radii:?}"
             );
        }
    }

    #[test]
    fn area_sign() {
        let center = Point::new(5.0, 5.0);
        let e = Ellipse::new(center, (5.0, 5.0), 1.0);
        assert_approx_eq(e.area(), 25.0 * PI);
        let e = Ellipse::new(center, (5.0, 10.0), 1.0);
        assert_approx_eq(e.area(), 50.0 * PI);

        assert_eq!(e.winding(center), 1);

        let p = e.to_path(1e-9);
        assert_approx_eq(e.area(), p.area());
        assert_eq!(e.winding(center), p.winding(center));

        let e_neg_radius = Ellipse::new(center, (-5.0, 10.0), 1.0);
        assert_approx_eq(e_neg_radius.area(), 50.0 * PI);

        assert_eq!(e_neg_radius.winding(center), 1);

        let p_neg_radius = e_neg_radius.to_path(1e-9);
        assert_approx_eq(e_neg_radius.area(), p_neg_radius.area());
        assert_eq!(e_neg_radius.winding(center), p_neg_radius.winding(center));
    }
}
