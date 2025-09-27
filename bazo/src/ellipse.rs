// Copyright 2020 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Implementation of ellipse shape.

use core::ops::{Add, Mul, Sub};

use crate::{Affine, Circle, Point, Rect, Size, Vec2};

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
    #[inline(always)]
    pub fn from_affine(affine: Affine) -> Self {
        Ellipse { inner: affine }
    }

    /// Create a new `Ellipse` centered on the provided point.
    #[inline]
    #[must_use]
    pub fn with_center(self, new_center: impl Into<Point>) -> Ellipse {
        let Point { x: cx, y: cy } = new_center.into();
        Ellipse {
            inner: self.inner.with_translation(Vec2 { x: cx, y: cy }),
        }
    }

    /// Create a new `Ellipse` with the provided radii.
    #[inline]
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
    #[inline]
    #[must_use]
    pub fn with_rotation(self, rotation: f64) -> Ellipse {
        let scale = self.inner.svd().0;
        let translation = self.inner.translation();
        Ellipse::private_new(translation, scale.x, scale.y, rotation)
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
    #[inline(always)]
    pub fn center(&self) -> Point {
        self.inner.translation().to_point()
    }

    /// Returns the two radii of this ellipse.
    ///
    /// The first number is the horizontal radius and the second is the vertical
    /// radius, before rotation.
    ///
    /// If you are only interested in the value of the greatest or smallest radius of this ellipse,
    /// consider using [`Ellipse::major_radius`] or [`Ellipse::minor_radius`] instead.
    #[inline]
    pub fn radii(&self) -> Vec2 {
        self.inner.svd().0
    }

    /// Returns the major radius of this ellipse.
    ///
    /// This metric is also known as the semi-major axis.
    #[inline]
    pub fn major_radius(&self) -> f64 {
        self.inner.svd().0.x
    }

    /// Returns the minor radius of this ellipse.
    ///
    /// This metric is also known as the semi-minor axis.
    #[inline]
    pub fn minor_radius(&self) -> f64 {
        self.inner.svd().0.y
    }

    /// The ellipse's rotation, in radians.
    ///
    /// This allows all possible ellipses to be drawn by always starting with
    /// an ellipse with the two radii on the x and y axes.
    #[inline]
    pub fn rotation(&self) -> f64 {
        self.inner.svd().1
    }

    /// Returns the radii and the rotation of this ellipse.
    ///
    /// Equivalent to `(self.radii(), self.rotation())` but more efficient.
    #[inline]
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
    #[inline]
    fn mul(self, other: Ellipse) -> Self::Output {
        Ellipse {
            inner: self * other.inner,
        }
    }
}

impl From<Circle> for Ellipse {
    #[inline]
    fn from(circle: Circle) -> Self {
        Ellipse::new(circle.center, Vec2::splat(circle.radius), 0.0)
    }
}
