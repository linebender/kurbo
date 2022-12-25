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

use arrayvec::ArrayVec;

use crate::{
    common::{
        factor_quartic_inner, solve_cubic, solve_quadratic, solve_quartic, GAUSS_LEGENDRE_COEFFS_16,
    },
    Affine, CubicBez, CurveFitSample, ParamCurve, ParamCurveArclen, ParamCurveArea,
    ParamCurveCurvature, ParamCurveDeriv, ParamCurveFit, Point, QuadBez, Shape, Vec2,
};

/// The offset curve of a cubic Bézier.
///
/// This is a representation of the offset curve of a cubic Bézier segment, for
/// purposes of curve fitting.
pub struct CubicOffset {
    inner: CubicOffsetInner,
    // c0 + c1 t + c2 t^2 is the cross product of second and first
    // derivatives of the underlying cubic (for computing curvature).
    c0: f64,
    c1: f64,
    c2: f64,
}

struct CubicOffsetInner {
    /// Source curve.
    c: CubicBez,
    /// Derivative of source curve.
    q: QuadBez,
    /// Offset.
    d: f64,
}

struct CurveSample {
    p: Point,
    offset: Vec2,
}

impl CubicOffsetInner {
    fn new(c: CubicBez, d: f64) -> Self {
        let q = c.deriv();
        CubicOffsetInner { c, q, d }
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
        let turn = ddp.cross(dp) * self.d / (dp.hypot() * dp.hypot2());
        (1.0 + turn) * dp
    }

    /// Approximate signed area.
    ///
    /// This is computed using Green's theorem integrating x' * y, which
    /// is a different treatment than the signed_area trait.
    fn area(&self) -> f64 {
        GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|&(wi, xi)| {
                let t = 0.5 * (1.0 + xi);
                let dx = self.eval_deriv(t).x;
                let y = self.eval(t).y;
                (0.5 * wi) * dx * y
            })
            .sum::<f64>()
    }

    /// Approximate x moment, useful for curve fitting.
    ///
    /// If we're optimizing, this shares a whole bunch with area and the
    /// two can be computed jointly. But for exploring, maybe better to
    /// keep things simple.
    fn x_moment(&self) -> f64 {
        GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|&(wi, xi)| {
                let t = 0.5 * (1.0 + xi);
                let dx = self.eval_deriv(t).x;
                //let delta = 500. * (self.eval(t + 0.001) - self.eval(t - 0.001));
                let p = self.eval(t);
                (0.5 * wi) * dx * p.x * p.y
            })
            .sum::<f64>()
    }

    /// Approximate arclength.
    ///
    /// As with all these, accuracy is ad hoc, but should be handled with
    /// carefully determined error bounds.
    fn arclen(&self) -> f64 {
        GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|&(wi, xi)| {
                let t = 0.5 * (1.0 + xi);
                let ds = self.eval_deriv(t).hypot();
                (0.5 * wi) * ds
            })
            .sum::<f64>()
    }

    /// Rotate the curve so its chord is on the x baseline.
    ///
    /// Returns rotated curve, rotation angle, and translation.
    fn rotate_to_x(&self) -> (CubicOffsetInner, f64, Vec2) {
        let p0 = self.c.p0 + self.eval_offset(0.0);
        let p1 = self.c.p3 + self.eval_offset(1.0);
        let dp = p0.to_vec2();
        let th = -(p1 - p0).atan2();
        let ct = CubicBez::new(
            self.c.p0 - dp,
            self.c.p1 - dp,
            self.c.p2 - dp,
            self.c.p3 - dp,
        );
        let c = Affine::rotate(th) * ct;
        (CubicOffsetInner::new(c, self.d), th, p0.to_vec2())
    }

    /// Compute cubic approximation of curve.
    ///
    /// Discussion: should also return an error metric?
    fn cubic_approx(&self, sign: f64, tol: f64) -> CubicBez {
        let (r, th, dp) = self.rotate_to_x();
        let end_x = r.c.p3.x + r.eval_offset(1.0).x;
        // these divisions don't work in the zero-length chord case.
        let area = r.area() / end_x.powi(2);
        let x_moment = r.x_moment() / end_x.powi(3);
        let th0 = (r.q.p0.to_vec2() * sign).atan2();
        let th1 = -(r.q.p2.to_vec2() * sign).atan2();
        let mut best_c = None;
        let mut best_err = None;
        let samples = self.sample_pts(10);
        let a = Affine::translate(dp) * Affine::rotate(-th) * Affine::scale(end_x);
        for cand in cubic_fit(th0, th1, area, x_moment) {
            let c = a * cand;
            let err = Self::est_cubic_err(c, &samples, best_err.unwrap_or(tol));
            if best_err.map(|best_err| err < best_err).unwrap_or(true) {
                best_err = Some(err);
                best_c = Some(c);
            }
        }
        best_c.unwrap()
    }

    fn sample_pts(&self, n: usize) -> Vec<CurveSample> {
        // TODO: this samples evenly by t, but that's not always great. It would be
        // better to sample approximately evenly by arc length, but without the
        // cost of inverse arc length solving.
        let dt = ((n + 1) as f64).recip();
        (0..n)
            .map(|i| {
                let t = (i + 1) as f64 * dt;
                let offset = self.eval_offset(t);
                let p = self.c.eval(t) + offset;
                CurveSample { p, offset }
            })
            .collect()
    }

    fn est_cubic_err(c: CubicBez, samples: &[CurveSample], tol: f64) -> f64 {
        let mut max_err = 0.0;
        let tol2 = tol * tol;
        for sample in samples {
            // Project sample point onto approximate curve along normal.
            let mut intersections = sample.intersect(c);
            if intersections.is_empty() {
                // maybe should just return some number greater than tol
                intersections.extend([0., 1.]);
            }
            let mut best_err = None;
            for t in intersections {
                let this_err = sample.p.distance_squared(c.eval(t));
                if best_err.map(|err| this_err < err).unwrap_or(true) {
                    best_err = Some(this_err);
                }
            }
            max_err = best_err.unwrap().max(max_err);
            if max_err > tol2 {
                break;
            }
        }
        max_err
    }

    fn find_offset_cusps(&self) {
        let d0 = self.q.p0.to_vec2();
        let d1 = 2.0 * (self.q.p1 - self.q.p0);
        let d2 = self.q.p0.to_vec2() - 2.0 * self.q.p1.to_vec2() + self.q.p1.to_vec2();
        let c0 = d1.cross(d0);
        let c1 = 2.0 * d2.cross(d0);
        let c2 = d2.cross(d1);
        // Compute a function which does zero-crossings at cusps.
        let calc = |t| {
            let ds2 = self.q.eval(t).to_vec2().hypot2();
            // Could avoid a division by multiplying by ds2^1.5, but go for clarity.
            let k = ((c2 * t + c1) * t + c0) / (ds2 * ds2.sqrt());
            k * self.d + 1.0
        };
    }

    fn subsegment(&self, range: Range<f64>) -> CubicOffsetInner {
        Self::new(self.c.subsegment(range), self.d)
    }
}

impl CurveSample {
    /// Intersect a ray with the given cubic.
    ///
    /// Returns a vector of `t` values on the cubic.
    fn intersect(&self, c: CubicBez) -> ArrayVec<f64, 3> {
        let p1 = 3.0 * (c.p1 - c.p0);
        let p2 = 3.0 * c.p2.to_vec2() - 6.0 * c.p1.to_vec2() + 3.0 * c.p0.to_vec2();
        let p3 = (c.p3 - c.p0) - 3.0 * (c.p2 - c.p1);
        let c0 = (c.p0 - self.p).cross(self.offset);
        let c1 = p1.cross(self.offset);
        let c2 = p2.cross(self.offset);
        let c3 = p3.cross(self.offset);
        solve_cubic(c0, c1, c2, c3)
            .into_iter()
            .filter(|&t| t >= 0. && t <= 1.)
            .collect()
    }
}

/// Returns curves matching area and moment, given unit chord.
fn cubic_fit(th0: f64, th1: f64, area: f64, mx: f64) -> ArrayVec<CubicBez, 4> {
    // Note: maybe we want to take unit vectors instead of angle? Shouldn't
    // matter much either way though.
    let (s0, c0) = th0.sin_cos();
    let (s1, c1) = th1.sin_cos();
    let a4 = -9.
        * c0
        * (((2. * s1 * c1 * c0 + s0 * (2. * c1 * c1 - 1.)) * c0 - 2. * s1 * c1) * c0
            - c1 * c1 * s0);
    let a3 = 12.
        * ((((c1 * (30. * area * c1 - s1) - 15. * area) * c0 + 2. * s0
            - c1 * s0 * (c1 + 30. * area * s1))
            * c0
            + c1 * (s1 - 15. * area * c1))
            * c0
            - s0 * c1 * c1);
    let a2 = 12.
        * ((((70. * mx + 15. * area) * s1 * s1 + c1 * (9. * s1 - 70. * c1 * mx - 5. * c1 * area))
            * c0
            - 5. * s0 * s1 * (3. * s1 - 4. * c1 * (7. * mx + area)))
            * c0
            - c1 * (9. * s1 - 70. * c1 * mx - 5. * c1 * area));
    let a1 = 16.
        * (((12. * s0 - 5. * c0 * (42. * mx - 17. * area)) * s1
            - 70. * c1 * (3. * mx - area) * s0
            - 75. * c0 * c1 * area * area)
            * s1
            - 75. * c1 * c1 * area * area * s0);
    let a0 = 80. * s1 * (42. * s1 * mx - 25. * area * (s1 - c1 * area));
    let mut roots = ArrayVec::<f64, 4>::new();
    const EPS: f64 = 1e-12;
    if a4.abs() > EPS {
        let a = a3 / a4;
        let b = a2 / a4;
        let c = a1 / a4;
        let d = a0 / a4;
        if let Some(quads) = factor_quartic_inner(a, b, c, d, false) {
            for (qc1, qc0) in quads {
                let qroots = solve_quadratic(qc0, qc1, 1.0);
                if qroots.is_empty() {
                    // Real part of pair of complex roots
                    roots.push(-0.5 * qc1);
                } else {
                    roots.extend(qroots);
                }
            }
        }
    } else if a3.abs() > EPS {
        roots.extend(solve_cubic(a0, a1, a2, a3));
    } else {
        roots.extend(solve_quadratic(a0, a1, a2));
    }

    let s01 = s0 * c1 + s1 * c0;
    roots
        .iter()
        .filter_map(|&d0| {
            let (d0, d1) = if d0 > 0.0 {
                let d1 = (d0 * s0 - area * (10. / 3.)) / (0.5 * d0 * s01 - s1);
                if d1 > 0.0 {
                    (d0, d1)
                } else {
                    (s0 / s01, 0.0)
                }
            } else {
                (0.0, s1 / s01)
            };
            if d0 >= 0.0 && d1 >= 0.0 {
                Some(CubicBez::new(
                    (0.0, 0.0),
                    (d0 * c0, d0 * s0),
                    (1.0 - d1 * c1, d1 * s1),
                    (1.0, 0.0),
                ))
            } else {
                None
            }
        })
        .collect()
}

#[test]
fn offset() {
    //let c = CubicBez::new((200., 450.), (400., 450.), (500., 100.), (600., 50.));
    let c = CubicBez::new((200., 350.), (400., 450.), (500., 450.), (600., 150.));
    println!(
        "  <path d=\"{}\" stroke=\"#000\" fill=\"none\" />",
        c.to_path(1e-9).to_svg()
    );
    let co = CubicOffsetInner::new(c, -200.0);
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        let p = co.eval(t);
        println!("  <circle cx=\"{}\" cy = \"{}\" r=\"2\" />", p.x, p.y);
        let d = co.eval_deriv(t);
        //println!("{:?}", d);
        let p2 = p + 0.01 * d;
        println!(
            "  <line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#008\" />",
            p.x, p.y, p2.x, p2.y
        )
    }
}

impl CubicOffset {
    /// Create a new curve from Bézier segment and offset.
    pub fn new(c: CubicBez, d: f64) -> Self {
        let inner = CubicOffsetInner::new(c, d);
        let q = inner.q;
        let d0 = q.p0.to_vec2();
        let d1 = 2.0 * (q.p1 - q.p0);
        let d2 = q.p0.to_vec2() - 2.0 * q.p1.to_vec2() + q.p2.to_vec2();
        let c0 = d1.cross(d0);
        let c1 = 2.0 * d2.cross(d0);
        let c2 = d2.cross(d1);
        CubicOffset { inner, c0, c1, c2 }
    }

    // Compute a function which has a zero-crossing at cusps, and is
    // positive at low curvatures on the source curve.
    fn cusp_sign(&self, t: f64) -> f64 {
        let ds2 = self.inner.q.eval(t).to_vec2().hypot2();
        let k = ((self.c2 * t + self.c1) * t + self.c0) / (ds2 * ds2.sqrt());
        k * self.inner.d + 1.0
    }
}

impl ParamCurveFit for CubicOffset {
    fn sample(&self, t: f64, sign: f64) -> CurveFitSample {
        let p = self.inner.eval(t);
        const EPSILON: f64 = 1e-9;
        let mut cusp = self.cusp_sign(t);
        if cusp.abs() < EPSILON {
            // This is a numerical derivative, which is probably good enough
            // for all practical purposes, but an analytical derivative would
            // be more elegant.
            //
            // Also, we're not dealing with second or higher order cusps.
            cusp = sign * (self.cusp_sign(t + EPSILON) - self.cusp_sign(t - EPSILON));
        }
        let tangent = self.inner.q.eval(t).to_vec2() * cusp.signum();
        CurveFitSample { p, tangent }
    }

    fn area_moment(&self, range: Range<f64>) -> (f64, f64) {
        let seg = self.inner.c.subsegment(range.clone());
        let p0 = seg.p0 + self.inner.eval_offset(range.start);
        let p1 = seg.p3 + self.inner.eval_offset(range.end);
        let d = (p1 - p0).normalize();
        let a = Affine::new([d.x, -d.y, d.y, d.x, 0.0, 0.0]) * Affine::translate(-p0.to_vec2());
        // Rotate and translate segment so chord is on the X axis.
        let c_rotated = a * seg;
        let rotated = CubicOffsetInner::new(c_rotated, self.inner.d);
        GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|&(wi, xi)| {
                let t = 0.5 * (1.0 + xi);
                let dx = rotated.eval_deriv(t).x;
                let p = rotated.eval(t);
                let a = (0.5 * wi) * dx * p.y;
                let m = a * p.x;
                (a, m)
            })
            .fold((0.0, 0.0), |(a0, m0), (a1, m1)| (a0 + a1, m0 + m1))
    }

    fn break_cusp(&self, range: Range<f64>) -> Option<f64> {
        None
    }
}

#[test]
fn foo() {
    let l = crate::Line::new((100.0, 100.0), (100.0, 200.0));
    let q = crate::QuadBez::new(l.p0, l.p0.lerp(l.p1, 0.5), l.p1);
    let c = q.raise();
    println!("l area {}", l.signed_area());
    println!("c area {}", c.signed_area());
    println!("offset area {}", CubicOffsetInner::new(c, 10.0).area());
    for c in cubic_fit(
        1.7278759594743864,
        1.4137166941154067,
        0.39161109109787334,
        0.1866475683300104,
    ) {
        println!("{:?}", c);
    }
    solve_quartic(0.048084, -0.140921, 0.302089, -0.334960, 0.113110);
}

#[test]
fn off_metrics() {
    let c = CubicBez::new((0., 0.), (10., 10.), (20., 10.), (30., 0.));
    let co = CubicOffsetInner::new(c, 0.0);
    println!(
        "area = {}, moment_x = {}, arclen = {}",
        co.area(),
        co.x_moment(),
        co.arclen()
    );
}

#[test]
fn try_offset() {
    let c = CubicBez::new((100., 100.), (150., 75.), (300., 50.), (400., 200.));
    println!(
        "  <path d='{}' stroke='#000' fill='none' />",
        c.to_path(1e-9).to_svg()
    );
    for i in 1..=20 {
        let co = CubicOffsetInner::new(c, i as f64 * 15.0);
        let new_c = co.cubic_approx(1.0, 1e3);
        println!(
            "  <path d='{}' stroke='#008' fill='none' />",
            new_c.to_path(1e-9).to_svg()
        );
    }
}
