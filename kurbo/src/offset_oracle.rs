// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Measurement of the error of an offset approximation, for the tests.
//!
//! This is an oracle for testing, not part of the API. It is based on the
//! Fréchet distance measurement of the min-Fréchet work in fast-stroke
//! (github.com/raphlinus/fast-stroke, `find_error_extrema` on the
//! `failed_k4` branch), by way of its port to `frechet.rs` in bezfit (branch
//! `slop1`), with three changes: robust endpoint tangents where the
//! derivative of the source vanishes at an endpoint; an explicit check of
//! the endpoints, which bezfit assumed to coincide; and [`result_error`],
//! which divides a complete result into its pieces.
//!
//! For a single piece, the error is measured along the normal rays of the
//! approximation. If the feet of the rays advance monotonically along the
//! offset, they define a monotone correspondence, so the largest ray error
//! is an upper bound on the Fréchet distance. The critical points of the ray
//! error, where the two tangents are parallel, are located and refined, so
//! that the bound is attained, up to the angle between the normals
//! elsewhere. If the feet do not advance monotonically, the ray error does
//! not measure the Fréchet distance, and the piece is reported as
//! unmeasured rather than underestimated.

use alloc::vec::Vec;

use crate::common::solve_itp;
use crate::{BezPath, CubicBez, Line, ParamCurve, ParamCurveDeriv, PathSeg, Point, QuadBez, Vec2};

/// The number of coarse samples used to search for error extrema, and to
/// check that the feet of the rays advance. bezfit used 32, which can miss
/// a short interval near a cusp of the offset where the feet reverse.
const N_COARSE: usize = 128;

/// The offset of a source cubic, with its derivatives precomputed.
struct SourceOffset {
    c: CubicBez,
    q: QuadBez,
    qq: Line,
    d: f64,
    utan0: Vec2,
    utan1: Vec2,
}

impl SourceOffset {
    fn new(c: CubicBez, d: f64) -> Self {
        let q = c.deriv();
        let qq = q.deriv();
        let (tan0, tan1) = end_tangents(&c);
        Self {
            c,
            q,
            qq,
            d,
            utan0: tan0.normalize(),
            utan1: tan1.normalize(),
        }
    }

    /// The unit tangent at `s`, falling back to the endpoint tangents where
    /// the derivative vanishes.
    fn utan(&self, s: f64) -> Vec2 {
        let q = self.q.eval(s).to_vec2();
        let len = q.hypot();
        if len > 1e-12 {
            q / len
        } else if s < 0.5 {
            self.utan0
        } else {
            self.utan1
        }
    }

    fn normal(&self, s: f64) -> Vec2 {
        self.utan(s).turn_90()
    }

    /// The signed curvature, `C' × C'' / |C'|^3`. This is the opposite sign
    /// convention from `crate::offset`, so the cusp value here is
    /// `1 - kappa d`.
    fn curvature(&self, s: f64) -> f64 {
        let q = self.q.eval(s).to_vec2();
        let qq = self.qq.eval(s).to_vec2();
        q.cross(qq) / q.hypot().powi(3)
    }

    fn eval(&self, s: f64) -> Point {
        self.c.eval(s) + self.d * self.normal(s)
    }

    /// The derivative of the offset with respect to `s`.
    fn deriv(&self, s: f64) -> Vec2 {
        (1.0 - self.curvature(s) * self.d) * self.q.eval(s).to_vec2()
    }
}

/// The endpoint tangents of `c`, as computed by `PathSeg::tangents`, but
/// with a handle treated as zero only when it vanishes to within rounding
/// relative to the size of the curve.
///
/// `PathSeg::tangents` treats a handle shorter than about `1e-6` as zero,
/// regardless of the size of the curve. On a short piece adjacent to an
/// endpoint where the derivative vanishes, it then falls back to the chord,
/// whose direction differs from the endpoint tangent, and the endpoints of
/// the offset are computed incorrectly.
fn end_tangents(c: &CubicBez) -> (Vec2, Vec2) {
    let size = [c.p1 - c.p0, c.p2 - c.p0, c.p3 - c.p0]
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.hypot2()));
    let eps = 1e-28 * size;
    let first = |vs: [Vec2; 3]| *vs.iter().find(|v| v.hypot2() > eps).unwrap_or(&vs[2]);
    (
        first([c.p1 - c.p0, c.p2 - c.p0, c.p3 - c.p0]),
        first([c.p3 - c.p2, c.p3 - c.p1, c.p3 - c.p0]),
    )
}

#[derive(Clone, Copy, Debug, Default)]
struct ErrReport {
    /// The parameter on the source.
    s: f64,
    /// The parameter on the approximation.
    t: f64,
    /// The signed error, positive in the direction of the offset normal.
    error: f64,
}

/// An approximation, with its derivatives precomputed.
struct Approx {
    c: CubicBez,
    q: QuadBez,
    qq: Line,
}

impl Approx {
    fn new(c: CubicBez) -> Self {
        let q = c.deriv();
        let qq = q.deriv();
        Self { c, q, qq }
    }

    fn deriv(&self, t: f64) -> Vec2 {
        self.q.eval(t).to_vec2()
    }

    fn curvature(&self, t: f64) -> f64 {
        let q = self.q.eval(t).to_vec2();
        let qq = self.qq.eval(t).to_vec2();
        q.cross(qq) / q.hypot().powi(3)
    }
}

/// The error along the normal ray of the approximation at `t`.
///
/// The foot of the ray is found by Newton's method from `s_guess`, and if
/// that fails, by bisection in the bracket nearest to `s_guess`. Newton's
/// method fails near a cusp of the offset, where its speed vanishes; bezfit
/// had no fallback, and reported such pieces as unmeasured.
fn error_by_ray(co: &SourceOffset, approx: &Approx, t: f64, s_guess: f64) -> Option<ErrReport> {
    const MAX_ITER: usize = 16;
    let pa = approx.c.eval(t);
    let tan = approx.deriv(t);
    let g = |s: f64| (pa - co.eval(s)).dot(tan);
    let report = |s: f64| {
        let error = (pa - co.eval(s)).dot(co.normal(s));
        Some(ErrReport { s, t, error })
    };
    let mut s = s_guess;
    for _ in 0..MAX_ITER {
        let dg = -co.deriv(s).dot(tan);
        let step = g(s) / dg;
        if !step.is_finite() {
            break;
        }
        s = (s - step).clamp(0.0, 1.0);
        if step.abs() < 1e-12 {
            return report(s);
        }
    }
    // Bisection, in the bracket whose root is nearest to the guess.
    const N: usize = 256;
    let mut best: Option<(f64, f64, f64)> = None;
    let mut prev = g(0.0);
    for j in 1..=N {
        let s1 = j as f64 / N as f64;
        let here = g(s1);
        if prev * here <= 0.0 {
            let s0 = (j - 1) as f64 / N as f64;
            let distance = (0.5 * (s0 + s1) - s_guess).abs();
            if best.is_none_or(|(_, _, b)| distance < b) {
                best = Some((s0, s1, distance));
            }
        }
        prev = here;
    }
    let (mut lo, mut hi, _) = best?;
    let g_lo = g(lo);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if g(mid) * g_lo > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    report(0.5 * (lo + hi))
}

/// The ray error at `n + 1` evenly spaced values of `t`, including the
/// endpoints.
fn coarse_errors(co: &SourceOffset, approx: &Approx, n: usize) -> Vec<ErrReport> {
    let mut result = Vec::with_capacity(n + 1);
    result.push(ErrReport::default());
    let mut s_prev = 0.0;
    for i in 1..n {
        let t = i as f64 / n as f64;
        let report = error_by_ray(co, approx, t, s_prev)
            .or_else(|| error_by_ray(co, approx, t, t))
            .unwrap_or(ErrReport {
                s: t,
                t,
                error: f64::NAN,
            });
        s_prev = report.s;
        result.push(report);
    }
    result.push(ErrReport {
        s: 1.0,
        t: 1.0,
        error: 0.0,
    });
    result
}

/// Refine an initial guess to a critical point of the error, by Newton's
/// method in both parameters.
fn find_extremum(co: &SourceOffset, approx: &Approx, mut s: f64, mut t: f64) -> Option<ErrReport> {
    const MAX_ITER: usize = 10;
    for _ in 0..MAX_ITER {
        let pa = approx.c.eval(t);
        let qa = approx.deriv(t);
        let ua = qa.normalize();
        let qo = co.q.eval(s).to_vec2();
        let uo = qo.normalize();
        let k_src = co.curvature(s);
        let k_approx = approx.curvature(t);
        let dp = pa - co.eval(s);
        let dot = ua.dot(uo);
        let f1 = dp.dot(uo);
        let f2 = ua.cross(uo);
        let df1_ds =
            -qo.length() * (1.0 - k_src * co.d) + k_src * qo.length() * dp.dot(uo.turn_90());
        let df1_dt = qa.dot(uo);
        let df2_ds = k_src * qo.length() * dot;
        let df2_dt = -k_approx * qa.length() * dot;
        let det = df1_ds * df2_dt - df1_dt * df2_ds;
        let ds = (f1 * df2_dt - df1_dt * f2) / det;
        let dt = (df1_ds * f2 - f1 * df2_ds) / det;
        if !(ds.is_finite() && dt.is_finite()) {
            return None;
        }
        s -= ds;
        t -= dt;
        if !(0.0..=1.0).contains(&s) || !(0.0..=1.0).contains(&t) {
            return None;
        }
        if ds.abs() + dt.abs() < 1e-12 {
            let error = (approx.c.eval(t) - co.eval(s)).dot(co.normal(s));
            return Some(ErrReport { s, t, error });
        }
    }
    None
}

/// Whether the coarse samples measure the error: every ray reached the
/// offset, and the feet advance monotonically along it.
fn rays_valid(samples: &[ErrReport]) -> bool {
    samples.iter().all(|e| e.error.is_finite())
        && samples.windows(2).all(|w| w[1].s >= w[0].s - 1e-9)
}

/// The interior critical points bracketed by the coarse samples.
fn extrema_from_samples(
    co: &SourceOffset,
    approx: &Approx,
    samples: &[ErrReport],
) -> Vec<ErrReport> {
    let n = samples.len() - 1;
    // Slope indicators at the interior samples and just inside each
    // endpoint. At the endpoints the tangents agree and the indicator
    // vanishes, but just inside, its sign is that of the slope of the
    // error.
    let t_end = 1.0 / (64 * n) as f64;
    let mut pts: Vec<(f64, f64, f64)> = Vec::with_capacity(n + 1);
    let near = |t: f64, s: f64| slope_at(co, approx, t, s).map(|(f, r)| (t, r.s, f));
    pts.extend(near(t_end, samples[1].s / 64.0));
    for r in &samples[1..n] {
        pts.push((r.t, r.s, approx.deriv(r.t).normalize().cross(co.utan(r.s))));
    }
    pts.extend(near(1.0 - t_end, 1.0 - (1.0 - samples[n - 1].s) / 64.0));
    let mut result: Vec<ErrReport> = Vec::new();
    for w in pts.windows(2) {
        let ((t0, s0, f0), (t1, s1, f1)) = (w[0], w[1]);
        if !(f0 * f1 < 0.0 || f1 == 0.0) {
            continue;
        }
        let u = if f1 == 0.0 { 1.0 } else { f0 / (f0 - f1) };
        let (t, s) = (t0 + (t1 - t0) * u, s0 + (s1 - s0) * u);
        let extremum = refine_bracket(co, approx, (t0, s0), (t1, s1))
            .or_else(|| find_extremum(co, approx, s, t))
            .or_else(|| error_by_ray(co, approx, t, s));
        if let Some(extremum) = extremum {
            if result
                .last()
                .is_none_or(|last: &ErrReport| (extremum.t - last.t).abs() > 1e-9)
            {
                result.push(extremum);
            }
        }
    }
    result
}

/// The slope indicator at `t`: the cross product of the unit tangents of
/// the approximation and of the offset at the foot of the ray.
fn slope_at(co: &SourceOffset, approx: &Approx, t: f64, s_guess: f64) -> Option<(f64, ErrReport)> {
    let r = error_by_ray(co, approx, t, s_guess)?;
    Some((approx.deriv(t).normalize().cross(co.utan(r.s)), r))
}

/// Find a critical point in a bracket, by the ITP method applied to the
/// slope indicator.
fn refine_bracket(
    co: &SourceOffset,
    approx: &Approx,
    lo: (f64, f64),
    hi: (f64, f64),
) -> Option<ErrReport> {
    let (f_lo, r_lo) = slope_at(co, approx, lo.0, lo.1)?;
    let (f_hi, _) = slope_at(co, approx, hi.0, hi.1)?;
    if f_lo == 0.0 {
        return Some(r_lo);
    }
    if f_lo * f_hi > 0.0 {
        return None;
    }
    let sign = -f_lo.signum();
    let mut s_warm = lo.1;
    let mut failed = false;
    let f = |t: f64| match slope_at(co, approx, t, s_warm) {
        Some((f, r)) => {
            s_warm = r.s;
            sign * f
        }
        None => {
            failed = true;
            0.0
        }
    };
    let k1 = 0.2 / (hi.0 - lo.0);
    let t = solve_itp(f, lo.0, hi.0, 1e-12, 1, k1, sign * f_lo, sign * f_hi);
    if failed {
        return None;
    }
    error_by_ray(co, approx, t, s_warm)
}

/// The Fréchet distance between `approx` and the offset of `source` by `d`,
/// or `None` if the ray error does not measure it.
///
/// The endpoints of `approx` need not lie on the offset; their distances
/// from the endpoints of the offset are included.
pub(crate) fn piece_error(source: CubicBez, d: f64, approx: CubicBez) -> Option<f64> {
    let co = SourceOffset::new(source, d);
    let approx = Approx::new(approx);
    let ends = (approx.c.p0 - co.c.p0 - d * co.utan0.turn_90())
        .hypot()
        .max((approx.c.p3 - co.c.p3 - d * co.utan1.turn_90()).hypot());
    let samples = coarse_errors(&co, &approx, N_COARSE);
    if !rays_valid(&samples) {
        return None;
    }
    let extrema = extrema_from_samples(&co, &approx, &samples);
    // Include the samples as well, in case an extremum was not bracketed.
    // bezfit used only the extrema when any were found.
    let worst = samples
        .iter()
        .chain(&extrema)
        .fold(0.0, |m: f64, r| m.max(r.error.abs()));
    Some(worst.max(ends))
}

/// The result of measuring a complete offset result.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ResultError {
    /// The largest error over the pieces that could be measured.
    pub error: f64,
    /// The number of pieces that could not be measured, because the ray
    /// error does not measure their Fréchet distance or their source range
    /// could not be found.
    pub unmeasured: usize,
    /// The largest one-sided distance from an unmeasured piece to the offset
    /// of its source range, by sampling. This is a lower bound on the
    /// Fréchet distance, not an upper bound.
    pub unmeasured_lower: f64,
    /// The numbers of unmeasured pieces whose source range was not found,
    /// whose source range is empty, and whose ray feet do not advance.
    pub unmeasured_reasons: [usize; 3],
}

/// The curve whose offset `crate::offset` approximates: `c`, regularized
/// for `tolerance`.
pub(crate) fn regularized(c: CubicBez, tolerance: f64) -> CubicBez {
    const DIM_TUNE: f64 = 0.25;
    c.regularize_cusp(tolerance * DIM_TUNE)
}

/// The error of one piece of a result.
#[derive(Clone, Copy, Debug)]
pub(crate) enum PieceError {
    /// The Fréchet distance, as measured by [`piece_error`].
    Measured(f64),
    /// The piece could not be measured: its source range was not found,
    /// the range is empty, or the feet of the rays do not advance. The value
    /// is a sampled one-sided distance, a lower bound on its error.
    Unmeasured(Unmeasured, f64),
}

/// Why a piece could not be measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Unmeasured {
    /// Its source range was not found.
    NoRange,
    /// Its source range is empty.
    EmptyRange,
    /// The feet of the rays do not advance.
    RaysTurnBack,
}

/// Measure each piece of `result`, an approximation of the offset of `c`
/// by `d` at `tolerance`, as computed by `crate::offset`.
///
/// The approximation is of the offset of the regularized curve, so the
/// error is measured against that, and the endpoints of every piece lie on
/// that offset. The source range of each piece is recovered from its
/// endpoints, in order, and the piece is measured with [`piece_error`].
pub(crate) fn piece_errors(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    result: &BezPath,
) -> Vec<PieceError> {
    let regularized = regularized(c, tolerance);
    let full = SourceOffset::new(regularized, d);
    let cusps = cusp_params(&full);
    let segments: Vec<PathSeg> = result.segments().collect();
    let mut errors = Vec::with_capacity(segments.len());
    let mut s_start = 0.0;
    for (i, seg) in segments.iter().enumerate() {
        let PathSeg::Cubic(approx) = *seg else {
            errors.push(PieceError::Unmeasured(Unmeasured::NoRange, 0.0));
            continue;
        };
        let s_end = if i + 1 == segments.len() {
            Some(1.0)
        } else {
            find_param(&full, &cusps, approx.p3, s_start)
        };
        let Some(s_end) = s_end else {
            let lower = sampled_distance(&full, &approx, s_start, 1.0);
            errors.push(PieceError::Unmeasured(Unmeasured::NoRange, lower));
            continue;
        };
        let error = if s_end - s_start > 1e-12 {
            match piece_error(regularized.subsegment(s_start..s_end), d, approx) {
                Some(error) => PieceError::Measured(error),
                None => PieceError::Unmeasured(
                    Unmeasured::RaysTurnBack,
                    sampled_distance(&full, &approx, s_start, s_end),
                ),
            }
        } else {
            PieceError::Unmeasured(
                Unmeasured::EmptyRange,
                sampled_distance(&full, &approx, s_start, s_end),
            )
        };
        errors.push(error);
        s_start = s_end;
    }
    errors
}

/// Measure the error of a complete result; see [`piece_errors`].
pub(crate) fn result_error(c: CubicBez, d: f64, tolerance: f64, result: &BezPath) -> ResultError {
    let mut report = ResultError::default();
    for error in piece_errors(c, d, tolerance, result) {
        match error {
            PieceError::Measured(error) => report.error = report.error.max(error),
            PieceError::Unmeasured(reason, lower) => {
                report.unmeasured += 1;
                report.unmeasured_lower = report.unmeasured_lower.max(lower);
                let i = match reason {
                    Unmeasured::NoRange => 0,
                    Unmeasured::EmptyRange => 1,
                    Unmeasured::RaysTurnBack => 2,
                };
                report.unmeasured_reasons[i] += 1;
            }
        }
    }
    report
}

/// The parameters at which the offset has a cusp: the zeros of
/// `1 - kappa d`.
fn cusp_params(co: &SourceOffset) -> Vec<f64> {
    const N: usize = 4000;
    let f = |s: f64| 1.0 - co.curvature(s) * co.d;
    let mut result = Vec::new();
    let mut prev = f(0.0);
    for j in 1..=N {
        let s = j as f64 / N as f64;
        let here = f(s);
        if prev * here < 0.0 {
            let (mut lo, mut hi) = ((j - 1) as f64 / N as f64, s);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if f(mid) * prev > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            result.push(0.5 * (lo + hi));
        }
        prev = here;
    }
    result
}

/// The smallest `s` at or after `s_min` whose offset point is `p`, to
/// within a small distance.
///
/// Near a cusp of the offset, the distance varies slowly with `s`, so a
/// point located by distance may lie past the cusp, where the offset runs
/// backward. As `crate::offset` subdivides exactly at cusps, a point that
/// matches a nearby cusp is snapped to it.
fn find_param(co: &SourceOffset, cusps: &[f64], p: Point, s_min: f64) -> Option<f64> {
    let s = find_param_by_distance(co, p, s_min)?;
    let scale = 1.0 + co.d.abs() + p.to_vec2().hypot();
    for &cusp in cusps {
        if cusp > s_min && (cusp - s).abs() < 1e-2 && p.distance(co.eval(cusp)) <= 1e-7 * scale {
            return Some(cusp);
        }
    }
    Some(s)
}

fn find_param_by_distance(co: &SourceOffset, p: Point, s_min: f64) -> Option<f64> {
    const N: usize = 4000;
    let scale = 1.0 + co.d.abs() + p.to_vec2().hypot();
    let threshold = 1e-7 * scale;
    let n = ((1.0 - s_min) * N as f64).ceil().max(2.0) as usize;
    let at = |j: usize| s_min + (1.0 - s_min) * j as f64 / n as f64;
    let dist: Vec<f64> = (0..=n).map(|j| p.distance(co.eval(at(j)))).collect();
    for j in 0..=n {
        let left = j == 0 || dist[j] <= dist[j - 1];
        let right = j == n || dist[j] <= dist[j + 1];
        if !(left && right) {
            continue;
        }
        let (s, distance) = refine_nearest(co, p, at(j.saturating_sub(1)), at((j + 1).min(n)));
        if distance <= threshold {
            return Some(s);
        }
    }
    None
}

/// The point of the offset nearest to `p` for `s` in `[lo, hi]`, by ternary
/// search, assuming a single minimum in that interval.
fn refine_nearest(co: &SourceOffset, p: Point, mut lo: f64, mut hi: f64) -> (f64, f64) {
    for _ in 0..80 {
        let s1 = lo + (hi - lo) / 3.0;
        let s2 = hi - (hi - lo) / 3.0;
        if p.distance_squared(co.eval(s1)) < p.distance_squared(co.eval(s2)) {
            hi = s2;
        } else {
            lo = s1;
        }
    }
    let s = 0.5 * (lo + hi);
    (s, p.distance(co.eval(s)))
}

/// The largest distance from points of `approx` to the offset for `s` in
/// `[lo, hi]`, by sampling and refinement around every local minimum.
fn sampled_distance(co: &SourceOffset, approx: &CubicBez, lo: f64, hi: f64) -> f64 {
    const N: usize = 400;
    let at = |j: usize| lo + (hi - lo) * j as f64 / N as f64;
    let mut worst = 0.0_f64;
    for i in 0..=20 {
        let p = approx.eval(i as f64 / 20.0);
        let dist: Vec<f64> = (0..=N).map(|j| p.distance(co.eval(at(j)))).collect();
        let mut nearest = f64::INFINITY;
        for j in 0..=N {
            let left = j == 0 || dist[j] <= dist[j - 1];
            let right = j == N || dist[j] <= dist[j + 1];
            if left && right {
                let (_, d) = refine_nearest(co, p, at(j.saturating_sub(1)), at((j + 1).min(N)));
                nearest = nearest.min(d);
            }
        }
        worst = worst.max(nearest);
    }
    worst
}
