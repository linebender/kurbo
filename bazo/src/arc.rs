// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! An ellipse arc.

use crate::{Affine, Ellipse, Point, Vec2};
use core::ops::Mul;

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
    use super::*;
    use core::f64::consts::PI;
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
}
