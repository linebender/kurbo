// Copyright 2018 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! SVG path representation.

use core::f64::consts::PI;

use crate::{Arc, Point, Vec2};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

// Note: the SVG arc logic is heavily adapted from https://github.com/nical/lyon

/// A single SVG arc segment.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SvgArc {
    /// The arc's start point.
    pub from: Point,
    /// The arc's end point.
    pub to: Point,
    /// The arc's radii, where the vector's x-component is the radius in the
    /// positive x direction after applying `x_rotation`.
    pub radii: Vec2,
    /// How much the arc is rotated, in radians.
    pub x_rotation: f64,
    /// Does this arc sweep through more than π radians?
    pub large_arc: bool,
    /// Determines if the arc should begin moving at positive angles.
    pub sweep: bool,
}

impl SvgArc {
    /// Checks that arc is actually a straight line.
    ///
    /// In this case, it can be replaced with a `LineTo`.
    pub fn is_straight_line(&self) -> bool {
        self.radii.x.abs() <= 1e-5 || self.radii.y.abs() <= 1e-5 || self.from == self.to
    }
}

impl Arc {
    /// Creates an `Arc` from a `SvgArc`.
    ///
    /// Returns `None` if `arc` is actually a straight line.
    pub fn from_svg_arc(arc: &SvgArc) -> Option<Arc> {
        // Have to check this first, otherwise `sum_of_sq` will be 0.
        if arc.is_straight_line() {
            return None;
        }

        let mut rx = arc.radii.x.abs();
        let mut ry = arc.radii.y.abs();

        let xr = arc.x_rotation % (2.0 * PI);
        let (sin_phi, cos_phi) = xr.sin_cos();
        let hd_x = (arc.from.x - arc.to.x) * 0.5;
        let hd_y = (arc.from.y - arc.to.y) * 0.5;
        let hs_x = (arc.from.x + arc.to.x) * 0.5;
        let hs_y = (arc.from.y + arc.to.y) * 0.5;

        // F6.5.1
        let p = Vec2::new(
            cos_phi * hd_x + sin_phi * hd_y,
            -sin_phi * hd_x + cos_phi * hd_y,
        );

        // Sanitize the radii.
        // If rf > 1 it means the radii are too small for the arc to
        // possibly connect the end points. In this situation we scale
        // them up according to the formula provided by the SVG spec.

        // F6.6.2
        let rf = p.x * p.x / (rx * rx) + p.y * p.y / (ry * ry);
        if rf > 1.0 {
            let scale = rf.sqrt();
            rx *= scale;
            ry *= scale;
        }

        let rxry = rx * ry;
        let rxpy = rx * p.y;
        let rypx = ry * p.x;
        let sum_of_sq = rxpy * rxpy + rypx * rypx;

        debug_assert!(sum_of_sq != 0.0);

        // F6.5.2
        let sign_coe = if arc.large_arc == arc.sweep {
            -1.0
        } else {
            1.0
        };
        let coe = sign_coe * ((rxry * rxry - sum_of_sq) / sum_of_sq).abs().sqrt();
        let transformed_cx = coe * rxpy / ry;
        let transformed_cy = -coe * rypx / rx;

        // F6.5.3
        let center = Point::new(
            cos_phi * transformed_cx - sin_phi * transformed_cy + hs_x,
            sin_phi * transformed_cx + cos_phi * transformed_cy + hs_y,
        );

        let start_v = Vec2::new((p.x - transformed_cx) / rx, (p.y - transformed_cy) / ry);
        let end_v = Vec2::new((-p.x - transformed_cx) / rx, (-p.y - transformed_cy) / ry);

        let start_angle = start_v.atan2();

        let mut sweep_angle = (end_v.atan2() - start_angle) % (2.0 * PI);

        if arc.sweep && sweep_angle < 0.0 {
            sweep_angle += 2.0 * PI;
        } else if !arc.sweep && sweep_angle > 0.0 {
            sweep_angle -= 2.0 * PI;
        }

        Some(Arc {
            center,
            radii: Vec2::new(rx, ry),
            start_angle,
            sweep_angle,
            x_rotation: arc.x_rotation,
        })
    }
}
