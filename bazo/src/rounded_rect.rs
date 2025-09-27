// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! A rectangle with rounded corners.

use core::ops::{Add, Sub};

use crate::{Point, Rect, RoundedRectRadii, Size, Vec2};

#[allow(unused_imports)] // This is unused in later versions of Rust because of additions to core::f32
#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// A rectangle with rounded corners.
///
/// By construction the rounded rectangle will have
/// non-negative dimensions and radii clamped to half size of the rect.
/// The rounded rectangle can have different radii for each corner.
///
/// The easiest way to create a `RoundedRect` is often to create a [`Rect`],
/// and then call [`to_rounded_rect`].
///
/// ```
/// use bazo::{RoundedRect, RoundedRectRadii};
///
/// // Create a rounded rectangle with a single radius for all corners:
/// RoundedRect::new(0.0, 0.0, 10.0, 10.0, 5.0);
///
/// // Or, specify different radii for each corner, clockwise from the top-left:
/// RoundedRect::new(0.0, 0.0, 10.0, 10.0, (1.0, 2.0, 3.0, 4.0));
/// ```
///
/// [`to_rounded_rect`]: Rect::to_rounded_rect
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoundedRect {
    /// Coordinates of the rectangle.
    rect: Rect,
    /// Radius of all four corners.
    radii: RoundedRectRadii,
}

impl RoundedRect {
    /// A new rectangle from minimum and maximum coordinates.
    ///
    /// The result will have non-negative width, height and radii.
    #[inline]
    pub fn new(
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
        radii: impl Into<RoundedRectRadii>,
    ) -> RoundedRect {
        RoundedRect::from_rect(Rect::new(x0, y0, x1, y1), radii)
    }

    /// A new rounded rectangle from a rectangle and corner radii.
    ///
    /// The result will have non-negative width, height and radii.
    ///
    /// See also [`Rect::to_rounded_rect`], which offers the same utility.
    #[inline]
    pub fn from_rect(rect: Rect, radii: impl Into<RoundedRectRadii>) -> RoundedRect {
        let rect = rect.abs();
        let shortest_side_length = (rect.width()).min(rect.height());
        let radii = radii.into().abs().clamp(shortest_side_length / 2.0);

        RoundedRect { rect, radii }
    }

    /// A new rectangle from two [`Point`]s.
    ///
    /// The result will have non-negative width, height and radius.
    #[inline]
    pub fn from_points(
        p0: impl Into<Point>,
        p1: impl Into<Point>,
        radii: impl Into<RoundedRectRadii>,
    ) -> RoundedRect {
        Rect::from_points(p0, p1).to_rounded_rect(radii)
    }

    /// A new rectangle from origin and size.
    ///
    /// The result will have non-negative width, height and radius.
    #[inline]
    pub fn from_origin_size(
        origin: impl Into<Point>,
        size: impl Into<Size>,
        radii: impl Into<RoundedRectRadii>,
    ) -> RoundedRect {
        Rect::from_origin_size(origin, size).to_rounded_rect(radii)
    }

    /// The width of the rectangle.
    #[inline]
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// The height of the rectangle.
    #[inline]
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Radii of the rounded corners.
    #[inline(always)]
    pub fn radii(&self) -> RoundedRectRadii {
        self.radii
    }

    /// The (non-rounded) rectangle.
    #[inline(always)]
    pub fn rect(&self) -> Rect {
        self.rect
    }

    /// The origin of the rectangle.
    ///
    /// This is the top left corner in a y-down space.
    #[inline(always)]
    pub fn origin(&self) -> Point {
        self.rect.origin()
    }

    /// The center point of the rectangle.
    #[inline]
    pub fn center(&self) -> Point {
        self.rect.center()
    }

    /// Is this rounded rectangle finite?
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.rect.is_finite() && self.radii.is_finite()
    }

    /// Is this rounded rectangle NaN?
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.rect.is_nan() || self.radii.is_nan()
    }
}

impl Add<Vec2> for RoundedRect {
    type Output = RoundedRect;

    #[inline]
    fn add(self, v: Vec2) -> RoundedRect {
        RoundedRect::from_rect(self.rect + v, self.radii)
    }
}

impl Sub<Vec2> for RoundedRect {
    type Output = RoundedRect;

    #[inline]
    fn sub(self, v: Vec2) -> RoundedRect {
        RoundedRect::from_rect(self.rect - v, self.radii)
    }
}
