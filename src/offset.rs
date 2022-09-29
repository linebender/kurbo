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

use crate::{
    common::{solve_quadratic, solve_quartic, GAUSS_LEGENDRE_COEFFS_16},
    Affine, CubicBez, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveDeriv, Point,
    QuadBez, Shape, Vec2,
};

struct CubicOffset {
    /// Source curve.
    c: CubicBez,
    /// Derivative of source curve.
    q: QuadBez,
    /// Offset.
    d: f64,
}

impl CubicOffset {
    fn new(c: CubicBez, d: f64) -> Self {
        let q = c.deriv();
        CubicOffset { c, q, d }
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
                let delta = 500. * (self.eval(t + 0.001) - self.eval(t - 0.001));
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
    fn rotate_to_x(&self) -> (CubicOffset, f64, Vec2) {
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
        (CubicOffset::new(c, self.d), th, p0.to_vec2())
    }

    /// Compute cubic approximation of curve.
    ///
    /// Discussion: should also return an error metric?
    fn cubic_approx(&self) -> CubicBez {
        let (r, th, dp) = self.rotate_to_x();
        let end_x = r.c.p3.x + r.eval_offset(1.0).x;
        // these divisions don't work in the zero-length chord case.
        let arclen = r.arclen() / end_x;
        let area = r.area() / end_x.powi(2);
        let x_moment = r.x_moment() / end_x.powi(3);
        let th0 = r.q.p0.to_vec2().atan2();
        let th1 = -r.q.p2.y.atan2(r.q.p2.x);
        let mut best_c = None;
        let mut best_err = None;
        for cand in cubic_fit(th0, th1, area, x_moment) {
            let cand_arclen = cand.arclen(1e-9);
            let err = (cand_arclen - arclen).abs();
            if best_err.map(|best_err| err < best_err).unwrap_or(true) {
                best_err = Some(err);
                best_c = Some(cand);
            }
            //println!("{:?} {}", cand, cand_arclen - arclen);
        }
        let c = best_c.unwrap();
        Affine::translate(dp) * Affine::rotate(-th) * Affine::scale(end_x) * c
    }
}

/// Returns curves matching area and moment, given unit chord.
fn cubic_fit(th0: f64, th1: f64, area: f64, mx: f64) -> Vec<CubicBez> {
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
    // TODO: set a3 and/or a4 to 0 if nearly so, as solver isn't robust.
    // Alternatively, make solver robust.
    let roots = solve_quartic(a0, a1, a2, a3, a4);
    let s01 = s0 * c1 + s1 * c0;
    let mut roots = roots.iter().copied().collect::<Vec<_>>();
    roots.push(0.0);
    roots.push(area * (10. / 3.) / s0);
    println!("{:?}", roots);
    roots
        .iter()
        .filter_map(|&d0| {
            //println!("d0 = {}", d0);
            // TODO: deal with d0, d1 near 0. Maybe want negative values across cusps?
            if true || d0 >= 0.0 {
                let d1 = (2. * d0 * s0 - area * (20. / 3.)) / (d0 * s01 - 2. * s1);
                //println!("d0 = {}, d1 = {}", d0, d1);
                if true || d1 >= -0.01 {
                    Some(CubicBez::new(
                        (0.0, 0.0),
                        (d0 * c0, d0 * s0),
                        (1.0 - d1 * c1, d1 * s1),
                        (1.0, 0.0),
                    ))
                } else {
                    None
                }
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
    let co = CubicOffset::new(c, -200.0);
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

#[test]
fn foo() {
    let l = crate::Line::new((100.0, 100.0), (100.0, 200.0));
    let q = crate::QuadBez::new(l.p0, l.p0.lerp(l.p1, 0.5), l.p1);
    let c = q.raise();
    println!("l area {}", l.signed_area());
    println!("c area {}", c.signed_area());
    println!("offset area {}", CubicOffset::new(c, 10.0).area());
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
    let co = CubicOffset::new(c, 0.0);
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
        let co = CubicOffset::new(c, i as f64 * 15.0);
        let new_c = co.cubic_approx();
        println!(
            "  <path d='{}' stroke='#008' fill='none' />",
            new_c.to_path(1e-9).to_svg()
        );
    }
}
