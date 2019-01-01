//! Research testbed for arclengths of cubic Bézier segments.

// Lots of stuff is commented out or was just something to try.
#![allow(unused)]

use std::env;

use kurbo::common::*;
use kurbo::{
    Affine, CubicBez, ParamCurve, ParamCurveArclen, ParamCurveCurvature, ParamCurveDeriv, Vec2,
};

/// Calculate arclength using Gauss-Legendre quadrature using formula from Behdad
/// in https://github.com/Pomax/BezierInfo-2/issues/77
fn gauss_arclen_5(c: CubicBez) -> f64 {
    let v0 = (c.p1 - c.p0).hypot() * 0.15;
    let v1 = (-0.558983582205757 * c.p0
        + 0.325650248872424 * c.p1
        + 0.208983582205757 * c.p2
        + 0.024349751127576 * c.p3)
        .hypot();
    let v2 = (c.p3 - c.p0 + c.p2 - c.p1).hypot() * 0.26666666666666666;
    let v3 = (-0.024349751127576 * c.p0 - 0.208983582205757 * c.p1 - 0.325650248872424 * c.p2
        + 0.558983582205757 * c.p3)
        .hypot();
    let v4 = (c.p3 - c.p2).hypot() * 0.15;

    v0 + v1 + v2 + v3 + v4
}

fn gauss_arclen_7<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_7)
}

fn est_gauss5_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    let d2 = c.deriv().deriv();
    let d3 = d2.deriv();
    let lmi = 2.0 / (lp + lc);
    7e-8 * (d3.eval(0.5).hypot() * lmi + 5.0 * d2.eval(0.5).hypot() * lmi).powi(5) * lp
}

fn gauss_errnorm_n<C: ParamCurveDeriv>(c: C, coeffs: &[(f64, f64)]) -> f64
where
    C::DerivResult: ParamCurveDeriv,
{
    let d = c.deriv().deriv();
    coeffs
        .iter()
        .map(|(wi, xi)| wi * d.eval(0.5 * (xi + 1.0)).hypot2())
        .sum::<f64>()
}

// Squared L2 norm of the second derivative of the cubic.
fn cubic_errnorm(c: CubicBez) -> f64 {
    let d = c.deriv().deriv();
    let dd = d.end() - d.start();
    d.start().hypot2() + d.start().dot(dd) + dd.hypot2() * (1.0 / 3.0)
}

fn est_gauss7_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    8e-9 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(6) * lp
}

fn gauss_arclen_4<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_4)
}

fn gauss_arclen_8<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_8)
}

fn gauss_arclen_9<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_9)
}

fn gauss_arclen_11<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_11)
}

fn gauss_arclen_16<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_16)
}

fn gauss_arclen_24<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_24)
}

fn gauss_arclen_32<C: ParamCurveDeriv>(c: C) -> f64 {
    c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_32)
}

// A common method for quadrature of even order, exploiting symmetry
fn arclen_quadrature_core(coeffs: &[(f64, f64)], dm: Vec2, dm1: Vec2, dm2: Vec2) -> f64 {
    coeffs
        .iter()
        .map(|&(wi, xi)| {
            let d = dm + dm2 * (xi * xi);
            let dpx = (d + dm1 * xi).hypot();
            let dmx = (d - dm1 * xi).hypot();
            (2.25f64.sqrt() * wi) * (dpx + dmx)
        })
        .sum::<f64>()
}

fn est_gauss9_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    (1e-10 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(8) * lp) //.min(0.03 * (lp - lc))
}

fn est_gauss11_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    1e-12 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(11) * lp
}

// A new approach based on integrating local error.
fn est_gauss11_error_2(c: CubicBez) -> f64 {
    let d = c.deriv();
    let d2 = d.deriv();
    GAUSS_LEGENDRE_COEFFS_11
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                let v = d.eval(t).hypot();
                let a2 = d2.eval(t).hypot2();
                a2.powi(3) / v.powi(5)
            }
        })
        .sum::<f64>()
}

fn est_max_curvature(c: CubicBez) -> f64 {
    let n = 10;
    let mut max = 0.0;
    for i in 0..=n {
        let t = (i as f64) * (n as f64).recip();
        let k = c.curvature(t).abs();
        if !(k < max) {
            max = k;
        }
    }
    max
}

fn est_min_deriv_norm2(c: CubicBez) -> f64 {
    let d = c.deriv();
    let n = 100;
    let mut min = d.eval(1.0).hypot2();
    for i in 0..n {
        let t = (i as f64) * (n as f64).recip();
        min = min.min(d.eval(t).hypot2())
    }
    min
}

fn est_gauss11_error_3(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let pc_err = (lp - lc) * 0.02;
    let ks = est_max_curvature(c) * lp;
    let est = ks.powi(3) * lp * 8e-9;
    if est < pc_err {
        est
    } else {
        pc_err
    }
}

fn est_gauss9_error_3(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let pc_err = (lp - lc) * 0.02;
    let ks = est_max_curvature(c) * lp;
    let est = ks.powi(3) * lp * 5e-8;
    if est < pc_err {
        est
    } else {
        pc_err
    }
}

// A new approach based on integrating local error; the cost of evaluating the
// error metric is likely to dominate unless the accuracy buys a lot of subdivisions.
fn est_gauss9_error_2(c: CubicBez) -> f64 {
    let d = c.deriv();
    let d2 = d.deriv();
    let p = 10;
    GAUSS_LEGENDRE_COEFFS_9
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                let v = d.eval(t).hypot();
                let a = d2.eval(t).hypot();
                (1.0e-1 * a / v).tanh().powi(p) * v
            }
        })
        .sum::<f64>()
        * 3.0
}

fn est_gauss9_error_4(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let est = gauss_arclen_9(c);
    let d = c.deriv();
    let v2 = GAUSS_LEGENDRE_COEFFS_9
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                d.eval(t).hypot2()
            }
        })
        .sum::<f64>()
        * 0.5;
    let v4 = GAUSS_LEGENDRE_COEFFS_9
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                d.eval(t).hypot2().powi(2)
            }
        })
        .sum::<f64>()
        * 0.5;
    //1e0 * ((v2 - est.powi(2))/est.powi(2)).powi(3) * lp
    1e0 * ((v4 - v2.powi(2)) / v2.powi(2)).powf(3.5) * lp
}

fn est_gauss9_error_5(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let min_v2 = est_min_deriv_norm2(c);
    let lm = lp; //0.5 * (lp + lc);

    let d2 = c.deriv().deriv();

    let norm_d2 = cubic_errnorm(c);

    ((1.0 - (min_v2 / lm.powi(2))) * norm_d2 / lm.powi(2)).powi(6) * 4e-7 * lm
    //(lp - lc) * 0.03
}

fn est_gauss9_error_6(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let lm = 0.5 * (lp + lc);
    let d = c.deriv();
    let d2 = d.deriv();
    let est = GAUSS_LEGENDRE_COEFFS_7
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                (d2.eval(t).hypot2() / d.eval(t).hypot2()).powi(1)
            }
        })
        .sum::<f64>();
    (est.powf(3.5) * 2e-9).min(0.03) * (lp - lc)
}

fn est_gauss24_error_6(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
    let d = c.deriv();
    let d2 = d.deriv();
    let est = GAUSS_LEGENDRE_COEFFS_8
        .iter()
        .map(|(wi, xi)| {
            wi * {
                let t = 0.5 * (xi + 1.0);
                (d2.eval(t).hypot2() / d.eval(t).hypot2()).powi(1)
            }
        })
        .sum::<f64>();
    (est.powf(12.0) * 5e-24).min(4e-3) * (lp - lc)
}

/// Computes core parameters of error estimate.
///
/// Returns: quadrature of |x''|^2 / |x'|^2, and also lp - lc
fn cubic_err_est_core(c: CubicBez) -> (f64, f64) {
    let d03 = c.p3 - c.p0;
    let d01 = c.p1 - c.p0;
    let d12 = c.p2 - c.p1;
    let d23 = c.p3 - c.p2;
    let lc = d03.hypot();
    let lp = d01.hypot() + d12.hypot() + d23.hypot();
    let dd1 = d12 - d01;
    let dd2 = d23 - d12;
    // It might be faster to do direct multiplies, the data dependencies would be shorter.
    let dm = 0.25 * (d01 + d23) + 0.5 * d12; // first derivative at midpoint
    let dm1 = 0.5 * (dd2 + dd1); // second derivative at midpoint
    let dm2 = 0.25 * (dd2 - dd1); // 0.5 * (third derivative at midpoint)

    let est = GAUSS_LEGENDRE_COEFFS_8
        .iter()
        .map(|&(wi, xi)| {
            wi * {
                let d_norm2 = (dm + dm1 * xi + dm2 * (xi * xi)).hypot2();
                let dd_norm2 = (dm1 + dm2 * (2.0 * xi)).hypot2();
                dd_norm2 / d_norm2
            }
        })
        .sum::<f64>();
    (est, lp - lc)
}

fn est_gauss8_error(c: CubicBez) -> f64 {
    let (est, lp_lc) = cubic_err_est_core(c);
    (est.powi(3) * 2.5e-6).min(3e-2) * lp_lc
}

fn est_gauss16_error(c: CubicBez) -> f64 {
    let (est, lp_lc) = cubic_err_est_core(c);
    (est.powi(6) * 1.5e-11).min(9e-3) * lp_lc
}

fn est_gauss24_error(c: CubicBez) -> f64 {
    let (est, lp_lc) = cubic_err_est_core(c);
    (est.powi(9) * 3.5e-16).min(3.5e-3) * lp_lc
}

fn est_gauss32_error(c: CubicBez) -> f64 {
    let (est, lp_lc) = cubic_err_est_core(c);
    (est.powi(12) * 1.2e-20).min(1.8e-3) * lp_lc
}

fn my_arclen(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss5_error(c) < accuracy {
        *count += 1;
        gauss_arclen_5(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen(c1, accuracy * 0.5, depth + 1, count)
    }
}

fn my_arclen7(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss7_error(c) < accuracy {
        *count += 1;
        gauss_arclen_7(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen7(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen7(c1, accuracy * 0.5, depth + 1, count)
    }
}

// Should make this generic instead of copy+paste, but we need only one when we're done.
fn my_arclen9(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss9_error(c) < accuracy {
        *count += 1;
        gauss_arclen_9(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen9(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen9(c1, accuracy * 0.5, depth + 1, count)
    }
}

// This doesn't help; we can't really get a more accurate error bound, so all this
// does is overkill the accuracy.
fn my_arclen11(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss9_error(c) < accuracy {
        *count += 1;
        gauss_arclen_11(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen11(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen11(c1, accuracy * 0.5, depth + 1, count)
    }
}

fn my_arclen_new(
    c: CubicBez,
    accuracy: f64,
    depth: usize,
    c8: &mut usize,
    c16: &mut usize,
    c24: &mut usize,
) -> f64 {
    let d03 = c.p3 - c.p0;
    let d01 = c.p1 - c.p0;
    let d12 = c.p2 - c.p1;
    let d23 = c.p3 - c.p2;
    let lp_lc = d01.hypot() + d12.hypot() + d23.hypot() - d03.hypot();
    let dd1 = d12 - d01;
    let dd2 = d23 - d12;
    // It might be faster to do direct multiplies, the data dependencies would be shorter.
    let dm = 0.25 * (d01 + d23) + 0.5 * d12; // first derivative at midpoint
    let dm1 = 0.5 * (dd2 + dd1); // second derivative at midpoint
    let dm2 = 0.25 * (dd2 - dd1); // 0.5 * (third derivative at midpoint)

    let est = GAUSS_LEGENDRE_COEFFS_8
        .iter()
        .map(|&(wi, xi)| {
            wi * {
                let d_norm2 = (dm + dm1 * xi + dm2 * (xi * xi)).hypot2();
                let dd_norm2 = (dm1 + dm2 * (2.0 * xi)).hypot2();
                dd_norm2 / d_norm2
            }
        })
        .sum::<f64>();
    let est_gauss8_error = (est.powi(3) * 2.5e-6).min(3e-2) * lp_lc;
    if est_gauss8_error < accuracy {
        *c8 += 1;
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_8_HALF, dm, dm1, dm2);
    }
    let est_gauss16_error = (est.powi(6) * 1.5e-11).min(9e-3) * lp_lc;
    if est_gauss16_error < accuracy {
        *c16 += 1;
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_16_HALF, dm, dm1, dm2);
    }
    let est_gauss24_error = (est.powi(9) * 3.5e-16).min(3.5e-3) * lp_lc;
    if est_gauss24_error < accuracy || depth >= 20 {
        *c24 += 1;
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_24_HALF, dm, dm1, dm2);
    }
    let (c0, c1) = c.subdivide();
    my_arclen_new(c0, accuracy * 0.5, depth + 1, c8, c16, c24)
        + my_arclen_new(c1, accuracy * 0.5, depth + 1, c8, c16, c24)
}

fn randpt() -> Vec2 {
    Vec2::new(rand::random(), rand::random())
}

fn randbez() -> CubicBez {
    CubicBez::new(randpt(), randpt(), randpt(), randpt())
}

fn report_stats() {
    for i in 0..=15 {
        let accuracy = 0.1f64.powi(i);
        let mut c8 = 0;
        let mut c16 = 0;
        let mut c24 = 0;
        let mut c32 = 0;
        for _ in 0..1000_000 {
            let c = randbez();
            let t: f64 = rand::random();
            let c = c.subsegment(0.0..t);
            let mut count = 0;
            let accurate_arclen = my_arclen9(c, 1e-15, 0, &mut count);
            let c = Affine::scale(accurate_arclen.recip()) * c; // normalize to mean vel = 1

            let est = my_arclen_new(c, accuracy, 0, &mut c8, &mut c16, &mut c24);
            let _err = (1.0 - est).abs();
        }
        //println!("1e-{}: {} {} {}, total {}", i, c8, c16, c24, c8 + c16 + c24);
        let est_time_estimating = 50 * (c8 + c16 + c24);
        let est_time = 50 * c8 + 80 * c16 + 110 * c24 + est_time_estimating;
        println!(
            "1e-{}: est time {}ns, {}% estimating",
            i,
            1e-6 * (est_time as f64),
            100.0 * (est_time_estimating as f64) / (est_time as f64)
        );
    }
}

fn plot_accuracy() {
    // TODO: make this a runtime parameter
    let accuracy = 1e-6;
    let mut c8 = 0;
    let mut c16 = 0;
    let mut c24 = 0;
    for _ in 0..100_000 {
        let c = randbez();
        let t: f64 = rand::random();
        let c = c.subsegment(0.0..t);
        let mut count = 0;
        let accurate_arclen = my_arclen9(c, 1e-15, 0, &mut count);
        let c = Affine::scale(accurate_arclen.recip()) * c; // normalize to mean vel = 1

        let est = gauss_arclen_32(c);
        let est_err = est_gauss32_error(c);
        // The arclength has been normalized to 1.0.
        let err = (1.0 - est).abs();
        println!("{} {}", est_err, err);
    }
}

fn main() {
    for arg in env::args().skip(1) {
        if arg == "report_stats" {
            return report_stats();
        } else if arg == "plot_accuracy" {
            return plot_accuracy();
        }
    }
}
