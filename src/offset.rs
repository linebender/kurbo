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
    common::GAUSS_LEGENDRE_COEFFS_16, CubicBez, ParamCurve, ParamCurveArea, ParamCurveDeriv, Point,
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

    fn eval(&self, t: f64) -> Point {
        // Point on source curve.
        let p = self.c.eval(t);
        let dp = self.q.eval(t).to_vec2();
        let norm = Vec2::new(-dp.y, dp.x);
        // TODO: deal with hypot = 0
        p + norm * self.d / dp.hypot()
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

    fn x_moment(&self) -> f64 {
        GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|&(wi, xi)| {
                let t = 0.5 * (1.0 + xi);
                let dx = self.eval_deriv(t).x;
                let p = self.eval(t);
                (0.5 * wi) * dx * p.x * p.y
            })
            .sum::<f64>()
    }
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
}
