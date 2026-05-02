// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::ops::Mul;

use crate::{
    BezPath, CubicBez, Join, ParamCurve, ParamCurveDeriv, PathEl, PathSeg, Point, QuadBez, Vec2,
};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// A diagonal matrix, suitable for representing anisotropic scaling.
#[derive(Clone, Copy, Debug)]
pub struct Diagonal2 {
    xx: f64,
    yy: f64,
}

struct ExpandCtx {
    expand: Diagonal2,
    join: Join,
    miter_limit: f64,
    tolerance: f64,
    result: BezPath,
    first_n: Option<Vec2>,
    first_tan: Vec2,
    last_pt: Point,
    last_n: Option<Vec2>,
    last_tan: Vec2,
}

struct CubicCtx {
    q: QuadBez,
    utan0: Vec2,
    utan1: Vec2,
}

struct TwoPointSample {
    a_n: f64,
    b_n: f64,
    c_n: f64,
}

impl Diagonal2 {
    /// Create a diagonal matrix.
    pub fn new(xx: f64, yy: f64) -> Self {
        Self { xx, yy }
    }

    /// Matrix inverse.
    ///
    /// Will of course produce infinities if a component is zero.
    pub fn inv(self) -> Self {
        Diagonal2::new(1.0 / self.xx, 1.0 / self.yy)
    }

    /// Scale a normal vector.
    ///
    /// This is mathematically equivalent to `self * (self * n).normalize()`
    /// but handles zeros.
    ///
    /// Note that `n` need not be unit length.
    pub fn scale_normal(self, n: Vec2) -> Vec2 {
        let z = self * n;
        let z_hypot2 = z.hypot2();
        if z_hypot2 == 0.0 {
            Vec2::ZERO
        } else {
            let inv_scale = 1.0 / z_hypot2.sqrt();
            self * z * inv_scale
        }
    }
}

impl Mul<Vec2> for Diagonal2 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.xx * rhs.x, self.yy * rhs.y)
    }
}

impl Mul<Point> for Diagonal2 {
    type Output = Point;

    fn mul(self, rhs: Point) -> Point {
        Point::new(self.xx * rhs.x, self.yy * rhs.y)
    }
}

// Note: a bunch more ops could be implemented, but for now we'll stick to what we need.

/// Expand a path.
///
/// Expands a filled path by the expansion, which allows separate x and y factors. The
/// path (and the result) is interpreted according to the nonzero winding rule.
///
/// Each subpath must be closed.
///
/// The tolerance is mostly for joins and robustness; it is not used to guide
/// subdivision. Rather, each Bézier segment in the input generally results in one
/// cubic Bézier in the output. Thus, it is not expected to work well when the
/// expansion factor is large compared with the radius of curvature on the input.
pub fn expand(
    path: impl IntoIterator<Item = PathEl>,
    expand: Diagonal2,
    join: Join,
    miter_limit: f64,
    tolerance: f64,
) -> BezPath {
    // TODO: evaluate area, adjust sign of expand
    let result = BezPath::new();
    let mut ctx = ExpandCtx {
        expand,
        join,
        miter_limit,
        tolerance,
        result,
        first_n: None,
        first_tan: Vec2::default(),
        last_pt: Point::default(),
        last_n: None,
        last_tan: Vec2::default(),
    };
    let mut first_pt = Point::default();
    for el in path {
        match el {
            PathEl::MoveTo(point) => {
                if ctx.last_n.is_some() {
                    ctx.do_close_path(first_pt);
                }
                first_pt = point;
                ctx.last_pt = point;
            }
            PathEl::LineTo(p1) => ctx.do_line(p1),
            PathEl::QuadTo(p1, p2) => ctx.do_quad(p1, p2),
            PathEl::CurveTo(p1, p2, p3) => ctx.do_cubic(p1, p2, p3),
            PathEl::ClosePath => ctx.do_close_path(first_pt),
        }
    }
    ctx.result
}

impl ExpandCtx {
    /// Helper function to determine if a distance is within tolerance.
    fn in_tolerance(&self, v: Vec2) -> bool {
        v.hypot2() < self.tolerance * self.tolerance
    }

    /// Process a line segment. Includes initial join.
    fn do_line(&mut self, p1: Point) {
        let tan = p1 - self.last_pt;
        let n = self.expand.scale_normal(tan.turn_90());
        self.do_join(n, tan, true);
        let out_p1 = p1 + n;
        self.result.line_to(out_p1);
        self.last_n = Some(n);
        self.last_tan = tan;
        self.last_pt = p1;
    }

    /// Process a quadratic Bézier segment. Includes initial join.
    fn do_quad(&mut self, p1: Point, p2: Point) {
        let q0 = p1 - self.last_pt;
        let q1 = p2 - p1;
        if self.in_tolerance(q0) || self.in_tolerance(q1) {
            self.do_line(p2);
            return;
        }
        let einv = self.expand.inv();
        let utan0 = (einv * q0).normalize();
        let utan1 = (einv * q1).normalize();
        let det = utan1.cross(utan0);
        if det.abs() < 1e-10 {
            self.do_line(p2);
            return;
        }
        let utanm = (einv * (p2 - self.last_pt)).normalize();
        let n0 = self.expand * utan0.turn_90();
        let n1 = self.expand * utan1.turn_90();
        let mid_chord = self.last_pt.midpoint(p2);
        let m = mid_chord.midpoint(p1) + self.expand * utanm.turn_90();
        let out_p0 = self.last_pt + n0;
        let out_p3 = p2 + n1;
        let rhs = einv * (m - out_p0.midpoint(out_p3));
        let idet = (8. / 3.) / det;
        let a = (utan1.cross(rhs) * idet).max(0.0);
        let b = (utan0.cross(rhs) * idet).max(0.0);
        self.do_join(n0, q0, false);
        let out_p1 = out_p0 + self.expand * (a * utan0);
        let out_p2 = out_p3 - self.expand * (b * utan1);
        self.result.curve_to(out_p1, out_p2, out_p3);
        self.last_n = Some(n1);
        self.last_tan = q1;
        self.last_pt = p2;
    }

    /// Process a cubic segment. Includes initial join.
    fn do_cubic(&mut self, p1: Point, p2: Point, p3: Point) {
        let einv = self.expand.inv();
        let c = CubicBez::new(einv * self.last_pt, einv * p1, einv * p2, einv * p3);
        let (tan0, tan1) = PathSeg::Cubic(c).tangents();
        let utan0 = tan0.normalize();
        let utan1 = tan1.normalize();
        let q = c.deriv();
        let p1xp0 = q.p1.to_vec2().cross(q.p0.to_vec2());
        let p2xp1 = q.p2.to_vec2().cross(q.p1.to_vec2());
        let cx = CubicCtx { q, utan0, utan1 };
        // First we try one-point, but only if there isn't one inflection point.
        let mut soln = if p1xp0 * p2xp1 >= 0.0 {
            try_one_point(&cx)
        } else {
            None
        };
        if soln.is_none() {
            // We try two-point linear if we don't have a one-point solution. A
            // more sophisticated approach would be to evaluate error and pick a
            // minimum, but that would be more complexity and take time. This is
            // very likely good enough for the purpose.
            soln = two_point_linear(&cx);
        }
        if let Some((a, b)) = soln {
            let n0 = self.expand * utan0.turn_90();
            let n1 = self.expand * utan1.turn_90();
            self.do_join(n0, self.expand * utan0, false);
            let out_p3 = p3 + n1;
            // TODO: clamp to correct direction
            let out_p1 = p1 + n0 + self.expand * (a * utan0);
            let out_p2 = p2 + n1 + self.expand * (b * utan1);
            self.result.curve_to(out_p1, out_p2, out_p3);
            self.last_n = Some(n1);
            self.last_tan = self.expand * utan1;
            self.last_pt = p3;
        } else {
            self.do_line(p3);
        }
    }

    /// Do a join.
    ///
    /// The `tan` parameter is a vector tangent to the start of the new segment.
    /// The `n` parameter is the normal vector (turned tangent) scaled by the expansion.
    fn do_join(&mut self, n: Vec2, tan: Vec2, is_line: bool) {
        // TODO: other join types etc
        if let Some(last_n) = self.last_n {
            let p = self.last_pt + n;
            if !self.in_tolerance(n - last_n) {
                if self.join != Join::Bevel {
                    let cross = self.last_tan.cross(tan);
                    let dot = self.last_tan.dot(tan);
                    let hypot = cross.hypot(dot);
                    match self.join {
                        Join::Bevel => unreachable!(),
                        Join::Miter => {
                            if cross < 0.0
                                && 2.0 * hypot < (hypot + dot) * self.miter_limit * self.miter_limit
                            {
                                let h = (n - last_n).cross(tan) / self.last_tan.cross(tan);
                                let miter_pt = self.last_pt + last_n + h * self.last_tan;
                                // A cheap optimization to reduce line segments with joins to lines
                                if let Some(PathEl::LineTo(p)) =
                                    self.result.elements_mut().last_mut()
                                {
                                    *p = miter_pt;
                                } else {
                                    self.result.line_to(miter_pt);
                                }
                                if is_line {
                                    return;
                                }
                            }
                        }
                        _ => todo!(),
                    }
                }
                // Bevel case
                self.result.line_to(p);
            }
        } else {
            self.result.move_to(self.last_pt + n);
            self.first_n = Some(n);
            self.first_tan = tan;
        }
    }

    fn do_close_path(&mut self, first_pt: Point) {
        // maybe do this test inside do_line for all lines
        if first_pt.distance_squared(self.last_pt) > self.tolerance * self.tolerance {
            self.do_line(first_pt);
        }
        self.do_join(self.first_n.unwrap(), self.first_tan, true);
        self.result.close_path();
        self.last_n = None;
    }
}

// Note: here is our own copy of sophisticated cubic Bézier offset logic. We have
// the ability to apply anisotropic expansion, but not cusp detection or subdivision,
// and I've also stripped out all the error evaluation. At some point, we want to
// redo stroking, and there may be an opportunity to share code.

/// Try to compute one-point shape control for a cubic.
///
/// Result is (a, b) parameters.
fn try_one_point(cx: &CubicCtx) -> Option<(f64, f64)> {
    // TODO: possibly reduce duplication with quadratic case.
    let tan = cx.q.eval(0.5).to_vec2();
    let tan_hypot2 = tan.hypot2();
    if tan_hypot2 < 1e-12 {
        return None;
    }
    let utan = tan / tan_hypot2.sqrt();
    let z = (utan - 0.5 * (cx.utan0 + cx.utan1)).turn_90();
    let cross = cx.utan0.cross(cx.utan1);
    if cross.abs() < 1e-12 {
        return None;
    }
    let idet = (8. / 3.) / cross;
    let a = z.cross(cx.utan1) * idet;
    let b = cx.utan0.cross(z) * idet;
    //let delta_tan = 0.75 * (b * cx.utan1 - a * cx.utan0) + 1.5 * (cx.utan1 - cx.utan0).turn_90();
    //let angle_err = delta_tan.cross(utan);
    //let err_est = 0.16 * angle_err.abs();
    Some((a, b))
}

fn two_point_linear(cx: &CubicCtx) -> Option<(f64, f64)> {
    const T0: f64 = 0.35;
    let s = [T0, 1.0 - T0].map(|t| {
        let q = cx.q.eval(t).to_vec2();
        let qscale = 1.0 / q.length();
        let utan = q * qscale;
        let n = utan.turn_90();
        let utan0_n = cx.utan0.dot(n);
        let utan1_n = cx.utan1.dot(n);
        let utan0xn = cx.utan0.dot(utan);
        let utan1xn = cx.utan1.dot(utan);
        let mt = 1.0 - t;
        let b0 = mt * mt * mt;
        let b1 = 3.0 * mt * t * mt;
        let b2 = 3.0 * mt * t * t;
        let b3 = t * t * t;
        let a_n = b1 * utan0_n;
        let b_n = b2 * utan1_n;
        let c_n = (b0 + b1) * utan0xn + (b2 + b3) * utan1xn - 1.0;
        TwoPointSample { a_n, b_n, c_n }
    });
    let det = s[0].a_n * s[1].b_n - s[1].a_n * s[0].b_n;
    if det.abs() < 1e-12 {
        return None;
    }
    let idet = -1.0 / det;
    let a = idet * (s[0].c_n * s[1].b_n - s[1].c_n * s[0].b_n);
    let b = idet * (s[0].a_n * s[1].c_n - s[1].a_n * s[0].c_n);
    Some((a, b))
}
