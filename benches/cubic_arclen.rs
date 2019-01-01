//! Benchmarks of cubic arclength approaches.

#![feature(test)]
extern crate test;
use test::Bencher;

use kurbo::common::*;
use kurbo::{CubicBez, ParamCurve, ParamCurveArclen, Vec2};

use std::arch::x86_64::*;

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

#[target_feature(enable = "avx")]
unsafe fn arclen_quadrature_core_avx2(coeffs: &[f64; 16], dm: Vec2, dm1: Vec2, dm2: Vec2) -> f64 {
    let dmx = _mm256_set1_pd(dm.x);
    let dmy = _mm256_set1_pd(dm.y);
    let dm1x = _mm256_set1_pd(dm1.x);
    let dm1y = _mm256_set1_pd(dm1.y);
    let dm2x = _mm256_set1_pd(dm2.x);
    let dm2y = _mm256_set1_pd(dm2.y);
    let w0 = _mm256_loadu_pd(&coeffs[0]);
    let x0 = _mm256_loadu_pd(&coeffs[4]);
    let x20 = _mm256_mul_pd(x0, x0);
    let d0x0 = _mm256_add_pd(dmx, _mm256_mul_pd(dm2x, x20));
    let d0y0 = _mm256_add_pd(dmy, _mm256_mul_pd(dm2y, x20));
    let dm1x0 = _mm256_mul_pd(dm1x, x0);
    let dm1y0 = _mm256_mul_pd(dm1y, x0);
    let dpx0 = _mm256_add_pd(d0x0, dm1x0);
    let dpy0 = _mm256_add_pd(d0y0, dm1y0);
    let dp0 = _mm256_sqrt_pd(_mm256_add_pd(
        _mm256_mul_pd(dpx0, dpx0),
        _mm256_mul_pd(dpy0, dpy0),
    ));
    let dmx0 = _mm256_sub_pd(d0x0, dm1x0);
    let dmy0 = _mm256_sub_pd(d0y0, dm1y0);
    let dm0 = _mm256_sqrt_pd(_mm256_add_pd(
        _mm256_mul_pd(dmx0, dmx0),
        _mm256_mul_pd(dmy0, dmy0),
    ));
    let s0 = _mm256_mul_pd(w0, _mm256_add_pd(dp0, dm0));

    let w1 = _mm256_loadu_pd(&coeffs[8]);
    let x1 = _mm256_loadu_pd(&coeffs[12]);
    let x21 = _mm256_mul_pd(x1, x1);
    let d0x1 = _mm256_add_pd(dmx, _mm256_mul_pd(dm2x, x21));
    let d0y1 = _mm256_add_pd(dmy, _mm256_mul_pd(dm2y, x21));
    let dm1x1 = _mm256_mul_pd(dm1x, x1);
    let dm1y1 = _mm256_mul_pd(dm1y, x1);
    let dpx1 = _mm256_add_pd(d0x1, dm1x1);
    let dpy1 = _mm256_add_pd(d0y1, dm1y1);
    let dp1 = _mm256_sqrt_pd(_mm256_add_pd(
        _mm256_mul_pd(dpx1, dpx1),
        _mm256_mul_pd(dpy1, dpy1),
    ));
    let dmx1 = _mm256_sub_pd(d0x1, dm1x1);
    let dmy1 = _mm256_sub_pd(d0y1, dm1y1);
    let dm1 = _mm256_sqrt_pd(_mm256_add_pd(
        _mm256_mul_pd(dmx1, dmx1),
        _mm256_mul_pd(dmy1, dmy1),
    ));
    let s1 = _mm256_mul_pd(w1, _mm256_add_pd(dp1, dm1));

    let s = _mm256_hadd_pd(s0, s1);
    // vpermpd might be a slightly better instruction
    let t = _mm_add_pd(_mm256_extractf128_pd(s, 1), _mm256_castpd256_pd128(s));
    let u = _mm_hadd_pd(t, t);
    _mm_cvtsd_f64(u)
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

#[bench]
fn bench_avx2_quadrature(b: &mut Bencher) {
    let coef = [0.0; 16];
    let dm = Vec2::new(0.0, 0.0);
    let dm1 = Vec2::new(0.0, 0.0);
    let dm2 = Vec2::new(0.0, 0.0);
    unsafe {
        b.iter(|| {
            arclen_quadrature_core_avx2(
                test::black_box(&coef),
                test::black_box(dm),
                test::black_box(dm1),
                test::black_box(dm2),
            )
        })
    }
}
