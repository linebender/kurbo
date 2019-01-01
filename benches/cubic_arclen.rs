//! Benchmarks of cubic arclength approaches.

#![feature(test)]
extern crate test;
use test::Bencher;

use kurbo::common::*;
use kurbo::{CubicBez, ParamCurve, ParamCurveArclen, Vec2};

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

fn my_arclen_new(c: CubicBez, accuracy: f64, depth: usize) -> f64 {
    let d03 = c.p3 - c.p0;
    let d01 = c.p1 - c.p0;
    let d12 = c.p2 - c.p1;
    let d23 = c.p3 - c.p2;
    let lp_lc = d01.hypot() + d12.hypot() + d23.hypot() - d03.hypot();
    let dd1 = d12 - d01;
    let dd2 = d23 - d12;
    // It might be faster to do direct multiplies, the data dependencies would be shorter.
    // The following values don't have the factor of 3 for first deriv
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
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_8_HALF, dm, dm1, dm2);
    }
    let est_gauss16_error = (est.powi(6) * 1.5e-11).min(9e-3) * lp_lc;
    if est_gauss16_error < accuracy {
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_16_HALF, dm, dm1, dm2);
    }
    let est_gauss24_error = (est.powi(9) * 3.5e-16).min(3.5e-3) * lp_lc;
    if est_gauss24_error < accuracy || depth >= 20 {
        return arclen_quadrature_core(&GAUSS_LEGENDRE_COEFFS_24_HALF, dm, dm1, dm2);
    }
    let (c0, c1) = c.subdivide();
    my_arclen_new(c0, accuracy * 0.5, depth + 1) + my_arclen_new(c1, accuracy * 0.5, depth + 1)
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

#[bench]
fn bench_cubic_arclen_1e_4(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-4))
}

#[bench]
fn bench_cubic_arclen_1e_5(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-5))
}

#[bench]
fn bench_cubic_arclen_1e_6(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-6))
}

#[bench]
fn bench_cubic_arclen_1e_7(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-7))
}

#[bench]
fn bench_cubic_arclen_1e_8(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-8))
}

#[bench]
fn bench_cubic_arclen_1e_9(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| test::black_box(c).arclen(1e-9))
}

#[bench]
fn bench_cubic_arclen_new_1e_4(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-4, 0))
}

#[bench]
fn bench_cubic_arclen_new_1e_5(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-5, 0))
}

#[bench]
fn bench_cubic_arclen_new_1e_6(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-6, 0))
}

#[bench]
fn bench_cubic_arclen_new_1e_7(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-7, 0))
}

#[bench]
fn bench_cubic_arclen_new_1e_8(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-8, 0))
}

#[bench]
fn bench_cubic_arclen_new_1e_9(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| my_arclen_new(test::black_box(c), 1e-9, 0))
}

#[bench]
fn bench_cubic_est_err(b: &mut Bencher) {
    let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 1.0), (1.0, 1.0));
    b.iter(|| cubic_err_est_core(test::black_box(c)))
}
