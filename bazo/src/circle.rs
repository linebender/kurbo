// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Implementation of circle shape.

use core::ops::{Add, Mul, Sub};

use crate::{Affine, Arc, Ellipse, Point, Vec2};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// A circle.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    /// The center.
    pub center: Point,
    /// The radius.
    pub radius: f64,
}

impl Circle {
    /// A new circle from center and radius.
    #[inline(always)]
    pub fn new(center: impl Into<Point>, radius: f64) -> Circle {
        Circle {
            center: center.into(),
            radius,
        }
    }

    /// Create a [`CircleSegment`] by cutting out parts of this circle.
    #[inline(always)]
    pub fn segment(self, inner_radius: f64, start_angle: f64, sweep_angle: f64) -> CircleSegment {
        CircleSegment {
            center: self.center,
            outer_radius: self.radius,
            inner_radius,
            start_angle,
            sweep_angle,
        }
    }

    /// Is this circle [finite]?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.center.is_finite() && self.radius.is_finite()
    }

    /// Is this circle [NaN]?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.center.is_nan() || self.radius.is_nan()
    }
}

impl Add<Vec2> for Circle {
    type Output = Circle;

    #[inline]
    fn add(self, v: Vec2) -> Circle {
        Circle {
            center: self.center + v,
            radius: self.radius,
        }
    }
}

impl Sub<Vec2> for Circle {
    type Output = Circle;

    #[inline]
    fn sub(self, v: Vec2) -> Circle {
        Circle {
            center: self.center - v,
            radius: self.radius,
        }
    }
}

impl Mul<Circle> for Affine {
    type Output = Ellipse;
    fn mul(self, other: Circle) -> Self::Output {
        self * Ellipse::from(other)
    }
}

/// A segment of a circle.
///
/// If `inner_radius > 0`, then the shape will be a doughnut segment.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CircleSegment {
    /// The center.
    pub center: Point,
    /// The outer radius.
    pub outer_radius: f64,
    /// The inner radius.
    pub inner_radius: f64,
    /// The angle to start drawing the segment (in radians).
    pub start_angle: f64,
    /// The arc length of the segment (in radians).
    pub sweep_angle: f64,
}

impl CircleSegment {
    /// Create a `CircleSegment` out of its constituent parts.
    #[inline(always)]
    pub fn new(
        center: impl Into<Point>,
        outer_radius: f64,
        inner_radius: f64,
        start_angle: f64,
        sweep_angle: f64,
    ) -> Self {
        CircleSegment {
            center: center.into(),
            outer_radius,
            inner_radius,
            start_angle,
            sweep_angle,
        }
    }

    /// Return an arc representing the outer radius.
    #[must_use]
    #[inline(always)]
    pub fn outer_arc(&self) -> Arc {
        Arc {
            center: self.center,
            radii: Vec2::new(self.outer_radius, self.outer_radius),
            start_angle: self.start_angle,
            sweep_angle: self.sweep_angle,
            x_rotation: 0.0,
        }
    }

    /// Return an arc representing the inner radius.
    ///
    /// This is [reversed] from the outer arc, so that it is in the
    /// same direction as the arc that would be drawn (as the path
    /// elements for this circle segment produce a closed path).
    ///
    /// [reversed]: Arc::reversed
    #[must_use]
    #[inline]
    pub fn inner_arc(&self) -> Arc {
        Arc {
            center: self.center,
            radii: Vec2::new(self.inner_radius, self.inner_radius),
            start_angle: self.start_angle + self.sweep_angle,
            sweep_angle: -self.sweep_angle,
            x_rotation: 0.0,
        }
    }

    /// Is this circle segment [finite]?
    ///
    /// [finite]: f64::is_finite
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.center.is_finite()
            && self.outer_radius.is_finite()
            && self.inner_radius.is_finite()
            && self.start_angle.is_finite()
            && self.sweep_angle.is_finite()
    }

    /// Is this circle segment [NaN]?
    ///
    /// [NaN]: f64::is_nan
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.center.is_nan()
            || self.outer_radius.is_nan()
            || self.inner_radius.is_nan()
            || self.start_angle.is_nan()
            || self.sweep_angle.is_nan()
    }
}

impl Add<Vec2> for CircleSegment {
    type Output = CircleSegment;

    #[inline]
    fn add(self, v: Vec2) -> Self {
        Self {
            center: self.center + v,
            ..self
        }
    }
}

impl Sub<Vec2> for CircleSegment {
    type Output = CircleSegment;

    #[inline]
    fn sub(self, v: Vec2) -> Self {
        Self {
            center: self.center - v,
            ..self
        }
    }
}
