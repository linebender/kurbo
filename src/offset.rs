// Copyright 2022 The kurbo Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Offset curve computation.
//! Currently work in progress.

use std::ops::Range;

use crate::{
    common::solve_itp, CubicBez, CurveFitSample, ParamCurve, ParamCurveDeriv, ParamCurveFit, Point,
    QuadBez, Vec2,
};

/// The offset curve of a cubic Bézier.
///
/// This is a representation of the offset curve of a cubic Bézier segment, for
/// purposes of curve fitting.
pub struct CubicOffset {
    /// Source curve.
    c: CubicBez,
    /// Derivative of source curve.
    q: QuadBez,
    /// Offset.
    d: f64,
    // c0 + c1 t + c2 t^2 is the cross product of second and first
    // derivatives of the underlying cubic (for computing curvature).
    c0: f64,
    c1: f64,
    c2: f64,
}

impl CubicOffset {
    /// Create a new curve from Bézier segment and offset.
    pub fn new(c: CubicBez, d: f64) -> Self {
        let q = c.deriv();
        let d0 = q.p0.to_vec2();
        let d1 = 2.0 * (q.p1 - q.p0);
        let d2 = q.p0.to_vec2() - 2.0 * q.p1.to_vec2() + q.p2.to_vec2();
        let c0 = d1.cross(d0);
        let c1 = 2.0 * d2.cross(d0);
        let c2 = d2.cross(d1);
        CubicOffset {
            c,
            q,
            d,
            c0,
            c1,
            c2,
        }
    }

    fn eval_offset(&self, t: f64) -> Vec2 {
        let dp = self.q.eval(t).to_vec2();
        let norm = Vec2::new(-dp.y, dp.x);
        // TODO: deal with hypot = 0
        norm * self.d / dp.hypot()
    }

    fn eval(&self, t: f64) -> Point {
        // Point on source curve.
        self.c.eval(t) + self.eval_offset(t)
    }

    /// Evaluate derivative of curve.
    fn eval_deriv(&self, t: f64) -> Vec2 {
        let dp = self.q.eval(t).to_vec2();
        let ddp = self.q.deriv().eval(t).to_vec2();
        // TODO: I think this computes the same value as the cusp quadratic.
        let turn = ddp.cross(dp) * self.d / (dp.hypot() * dp.hypot2());
        (1.0 + turn) * dp
    }

    // Compute a function which has a zero-crossing at cusps, and is
    // positive at low curvatures on the source curve.
    fn cusp_sign(&self, t: f64) -> f64 {
        let ds2 = self.q.eval(t).to_vec2().hypot2();
        let k = ((self.c2 * t + self.c1) * t + self.c0) / (ds2 * ds2.sqrt());
        k * self.d + 1.0
    }
}

impl ParamCurveFit for CubicOffset {
    fn sample_pt_tangent(&self, t: f64, sign: f64) -> CurveFitSample {
        let p = self.eval(t);
        const CUSP_EPS: f64 = 1e-8;
        let mut cusp = self.cusp_sign(t);
        if cusp.abs() < CUSP_EPS {
            // This is a numerical derivative, which is probably good enough
            // for all practical purposes, but an analytical derivative would
            // be more elegant.
            //
            // Also, we're not dealing with second or higher order cusps.
            cusp = sign * (self.cusp_sign(t + CUSP_EPS) - self.cusp_sign(t - CUSP_EPS));
        }
        let tangent = self.q.eval(t).to_vec2() * cusp.signum();
        CurveFitSample { p, tangent }
    }

    fn sample_pt_deriv(&self, t: f64) -> (Point, Vec2) {
        (self.eval(t), self.eval_deriv(t))
    }

    fn break_cusp(&self, range: Range<f64>) -> Option<f64> {
        const CUSP_EPS: f64 = 1e-8;
        // When an endpoint is on (or very near) a cusp, move just far enough
        // away from the cusp that we're confident we have the right sign.
        let break_cusp_help = |mut x, mut d| {
            let mut cusp = self.cusp_sign(x);
            while cusp.abs() < CUSP_EPS && d < 1.0 {
                x += d;
                let old_cusp = cusp;
                cusp = self.cusp_sign(x);
                if cusp.abs() > old_cusp.abs() {
                    break;
                }
                d *= 2.0;
            }
            (x, cusp)
        };
        let (a, cusp0) = break_cusp_help(range.start, 1e-12);
        let (b, cusp1) = break_cusp_help(range.end, -1e-12);
        if a >= b || cusp0 * cusp1 >= 0.0 {
            // Discussion point: maybe we should search for double cusps in the interior
            // of the range.
            return None;
        }
        let s = cusp1.signum();
        let f = |t| s * self.cusp_sign(t);
        let k1 = 0.2 / (b - a);
        const ITP_EPS: f64 = 1e-12;
        let x = solve_itp(f, a, b, ITP_EPS, 1, k1, s * cusp0, s * cusp1);
        Some(x)
    }
}
