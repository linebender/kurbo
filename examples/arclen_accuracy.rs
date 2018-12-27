//! A test program to plot the error of arclength approximation.

use std::env;

use kurbo::{ParamCurve, ParamCurveArclen, QuadBez};

/// Calculate arclenth using Gauss-Legendre quadrature using formula from Behdad
/// in https://github.com/Pomax/BezierInfo-2/issues/77
fn gauss_arclen(q: QuadBez) -> f64 {
    let v0 = (-0.492943519233745*q.p0 + 0.430331482911935*q.p1 + 0.0626120363218102*q.p2).hypot();
    let v1 = ((q.p2-q.p0)*0.4444444444444444).hypot();
    let v2 = (-0.0626120363218102*q.p0 - 0.430331482911935*q.p1 + 0.492943519233745*q.p2).hypot();
    v0 + v1 + v2
}

/// Calculate the L0 metric from "Adaptive subdivision and the length and
/// energy of Bézier curves" by Jens Gravesen.
fn calc_l0(q: QuadBez) -> f64 {
    let lc = (q.p2 - q.p0).hypot();
    let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
    (2.0 / 3.0) * lc + (1.0 / 3.0) * lp
}

fn with_subdiv(q: QuadBez, f: &Fn(QuadBez) -> f64, depth: usize) -> f64 {
    if depth == 0 {
        f(q)
    } else {
        let (q0, q1) = q.subdivide();
        with_subdiv(q0, f, depth - 1) + with_subdiv(q1, f, depth - 1)
    }
}

/// Generate map data suitable for plotting in Gnuplot.
fn main() {
    let mut n_subdiv = 0;
    let mut func: &Fn(QuadBez) -> f64 = &gauss_arclen;
    for arg in env::args().skip(1) {
        if arg == "gauss" {
            func = &gauss_arclen;
        } else if arg == "l0" {
            func = &calc_l0;
        } else if let Ok(n) = arg.parse() {
            n_subdiv = n;
        } else {
            println!("usage: arclen_accuracy [func] n_subdiv");
            std::process::exit(1);
        }
    }
    let n = 400;
    for i in 0..=n {
        let x = 2.0 * (i as f64) * (n as f64).recip();
        for j in 0..=n {
            let y = 2.0 * (j as f64) * (n as f64).recip();
            let q = QuadBez::new((-1.0, 0.0), (x, y), (1.0, 0.0));
            let accurate_arclen = q.arclen(1e-15);
            let est = with_subdiv(q, func, n_subdiv);
            let error = est - accurate_arclen;
            println!("{} {} {}", x, y, (error.abs() + 1e-18).log10());
        }
        println!("");
    }
}
