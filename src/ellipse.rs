//! Implementation of ellipse shape.

use std::f64::consts::PI;
use std::{
    iter,
    ops::{Add, Mul, Sub},
};

use crate::{Affine, Arc, ArcAppendIter, Circle, PathEl, Point, Rect, Shape, Size, Vec2};

/// An ellipse.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    /// All ellipses can be represented as an affine map of the unit circle, centred at (0, 0).
    /// Therefore we can store the ellipse as an affine map, with the implication it be applied to
    /// the unit circle to recover the actual shape.
    inner: Affine,
}

impl Ellipse {
    /// A new ellipse from center, radii, and x_rotation.
    ///
    /// The returned ellipse will be the result of taking a circle, stretching it by the `radii`
    /// along the x and y axes, then rotating it clockwise by `x_rotation` radians, before finally
    /// translating the center to `center`.
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
    /// [`with_x_rotation`] with the result.
    ///
    /// [`Rect`]: struct.Rect.html
    /// [`with_x_rotation`]: #method.with_x_rotation
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
        let Vec2 { x: cx, y: cy } = self.inner.get_translation();
        Point { x: cx, y: cy }
    }

    /// Sets the center of this ellipse.
    #[inline]
    pub fn with_center(self, new_center: Point) -> Ellipse {
        let Point { x: cx, y: cy } = new_center;
        Ellipse {
            inner: self.inner.set_translation(Vec2 { x: cx, y: cy }),
        }
    }

    /// Returns the two radii of this ellipse.
    ///
    /// The first number is the horizontal radius and the second is the vertical radius, before
    /// rotating by `x_rotation`.
    pub fn radii(&self) -> Vec2 {
        self.inner.svd().0
    }

    /// Set the radii of this ellipse.
    #[must_use]
    pub fn with_radii(self, new_radii: Vec2) -> Ellipse {
        let rotation = self.inner.svd().1;
        let translation = self.inner.get_translation();
        Ellipse::private_new(translation, new_radii.x, new_radii.y, rotation)
    }

    /// The amount (in radians) that the ellipse should be rotated by (clockwise).
    ///
    /// This allows all possible ellipses to be drawn by always starting with an ellipse with the
    /// two radii on the x and y axes.
    pub fn x_rotation(&self) -> f64 {
        self.inner.svd().1
    }

    /// Set the amount (in radians) that the ellipse should be rotated by (clockwise).
    pub fn with_x_rotation(self, new_x_rotation: f64) -> Ellipse {
        let scale = self.inner.svd().0;
        let translation = self.inner.get_translation();
        Ellipse::private_new(translation, scale.x, scale.y, new_x_rotation)
    }
}

impl Add<Vec2> for Ellipse {
    type Output = Ellipse;

    /// In this context adding a `Vec2` applies the corresponding translation to the eliipse.
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

    /// In this context subtracting a `Vec2` applies the corresponding translation to the eliipse.
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
    type PathElementsIter = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn to_path_elements(&self, tolerance: f64) -> Self::PathElementsIter {
        let (radii, x_rotation) = self.inner.svd();
        Arc {
            center: self.center(),
            radii,
            start_angle: 0.0,
            sweep_angle: 2.0 * PI,
            x_rotation,
        }
        .to_path_elements(tolerance)
    }

    #[inline]
    fn area(&self) -> f64 {
        let Vec2 { x, y } = self.radii();
        PI * x * y
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        // TODO rather than delegate to the bezier path, it is possible to use various series
        // expansions to compute the perimeter to any accuracy. I believe Ramanujan authored the
        // quickest to converge. See
        // https://www.mathematica-journal.com/2009/11/23/on-the-perimeter-of-an-ellipse/
        // and https://en.wikipedia.org/wiki/Ellipse#Circumference
        //
        self.to_path_segments(0.1).perimeter(accuracy)
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

#[cfg(test)]
mod tests {
    use crate::{Ellipse, Point, Shape};
    use std::f64::consts::PI;

    fn assert_approx_eq(x: f64, y: f64) {
        // Note: we might want to be more rigorous in testing the accuracy
        // of the conversion into Béziers. But this seems good enough.
        assert!((x - y).abs() < 1e-7, "{} != {}", x, y);
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
