//! Implementation of ellipse shape.

use std::f64::consts::PI;
use std::{
    iter,
    ops::{Add, Mul, Sub},
};

use crate::{Affine, Arc, ArcAppendIter, Circle, PathEl, Point, Rect, Shape, Vec2};

/// An ellipse.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    /// All ellipses can be represented as an affine map of the unit circle, centred at (0, 0).
    /// Therefore we can store the ellipse as an affine map, with the implication it be applied to
    /// the unit circle to recover the actual shape.
    inner: Affine,
}

// TODO check all the descriptions match actual behavior.
impl Ellipse {
    /// A new ellipse from center, radii, and x_rotation.
    ///
    /// TODO explain how these parameters form a specific ellipse.
    #[inline]
    pub fn new(center: impl Into<Point>, radii: impl Into<Vec2>, x_rotation: f64) -> Ellipse {
        let Point { x: cx, y: cy } = center.into();
        let Vec2 { x: rx, y: ry } = radii.into();
        Ellipse::private_new(Vec2 { x: cx, y: cy }, rx, ry, x_rotation)
    }

    /// This gives us an internal method without any type conversions.
    #[inline]
    fn private_new(center: Vec2, scale_x: f64, scale_y: f64, x_rotation: f64) -> Ellipse {
        Ellipse {
            inner: Affine::translate(center)
                * Affine::rotate(x_rotation)
                * Affine::scale_non_uniform(scale_x, scale_y),
        }
    }

    /// Create an ellipse from an affine transformation of the unit circle.
    #[inline]
    pub fn from_affine(affine: Affine) -> Self {
        Ellipse { inner: affine }
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

    // NOTE: I'm not 100% sure about stability if you repeatedly do SVDs, which these methods
    // rely on.
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

// Delegate to an bezier path approximation for all measurements, except for area, since the
// formula there is simple.
impl Shape for Ellipse {
    type BezPathIter = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter {
        let (radii, x_rotation) = self.inner.svd();
        Arc {
            center: self.center(),
            radii,
            start_angle: 0.0,
            sweep_angle: 2.0 * PI,
            x_rotation,
        }
        .to_bez_path(tolerance)
    }

    #[inline]
    fn area(&self) -> f64 {
        let Vec2 { x, y } = self.radii();
        PI * x * y
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        self.clone()
            .into_bez_path(0.1)
            .elements()
            .perimeter(accuracy)
    }

    fn winding(&self, pt: Point) -> i32 {
        self.clone().into_bez_path(0.1).elements().winding(pt)
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        self.clone().into_bez_path(0.1).elements().bounding_box()
    }
}

/*
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
        let c = Ellipse::new(center, 5.0);
        assert_approx_eq(c.area(), 25.0 * PI);

        assert_eq!(c.winding(center), 1);

        let p = c.into_bez_path(1e-9);
        assert_approx_eq(c.area(), p.area());
        assert_eq!(c.winding(center), p.winding(center));

        let c_neg_radius = Ellipse::new(center, -5.0);
        assert_approx_eq(c_neg_radius.area(), 25.0 * PI);

        assert_eq!(c_neg_radius.winding(center), 1);

        let p_neg_radius = c_neg_radius.into_bez_path(1e-9);
        assert_approx_eq(c_neg_radius.area(), p_neg_radius.area());
        assert_eq!(c_neg_radius.winding(center), p_neg_radius.winding(center));
    }
}
*/
