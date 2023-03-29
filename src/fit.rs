// Copyright 2022 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! An implementation of cubic Bézier curve fitting based on a quartic
//! solver making signed area and moment match the source curve.

use std::ops::Range;

use arrayvec::ArrayVec;

use crate::{
    common::{
        factor_quartic_inner, solve_cubic, solve_itp, solve_quadratic, GAUSS_LEGENDRE_COEFFS_16,
    },
    Affine, BezPath, CubicBez, ParamCurve, Point, Vec2,
};

/// The source curve for curve fitting.
///
/// Note general similarities to [`ParamCurve`] but also important differences.
/// Instead of separate [`eval`] and evaluation of derivative, have a single
/// [`sample`] method which can be more efficient and also handles cusps more
/// robustly. Also there is no method for subsegment, as that is not needed and
/// would be annoying to implement.
///
/// [`ParamCurve`]: crate::ParamCurve
/// [`eval`]: crate::ParamCurve::eval
/// [`sample`]: ParamCurveFit::sample
pub trait ParamCurveFit {
    /// Evaluate the curve and its tangent at parameter `t`.
    ///
    /// For a regular curve (one not containing a cusp or corner), the
    /// derivative is a good choice for the tangent vector and the `sign`
    /// parameter can be ignored. Otherwise, the `sign` parameter selects which
    /// side of the discontinuity the tangent will be sampled from.
    ///
    /// Generally `t` is in the range [0..1].
    fn sample_pt_tangent(&self, t: f64, sign: f64) -> CurveFitSample;

    /// Evaluate the point and derivative at parameter `t`.
    ///
    /// In curves with cusps, the derivative can go to zero.
    fn sample_pt_deriv(&self, t: f64) -> (Point, Vec2);

    /// Compute moment integrals.
    ///
    /// This method computes the integrals of y dx, x y dx, and y^2 dx over the
    /// length of this curve. From these integrals it is fairly straightforward
    /// to derive the moments needed for curve fitting.
    ///
    /// A default implementation is proved which does quadrature integration
    /// with Green's theorem, in terms of samples evaulated with
    /// [`sample_pt_deriv`].
    ///
    /// [`sample_pt_deriv`]: ParamCurveFit::sample_pt_deriv
    fn moment_integrals(&self, range: Range<f64>) -> (f64, f64, f64) {
        let t0 = 0.5 * (range.start + range.end);
        let dt = 0.5 * (range.end - range.start);
        let (a, x, y) = GAUSS_LEGENDRE_COEFFS_16
            .iter()
            .map(|(wi, xi)| {
                let t = t0 + xi * dt;
                let (p, d) = self.sample_pt_deriv(t);
                let a = wi * d.x * p.y;
                let x = p.x * a;
                let y = p.y * a;
                (a, x, y)
            })
            .fold((0.0, 0.0, 0.0), |(a0, x0, y0), (a1, x1, y1)| {
                (a0 + a1, x0 + x1, y0 + y1)
            });
        (a * dt, x * dt, y * dt)
    }

    /// Find a cusp or corner within the given range.
    ///
    /// If the range contains a corner or cusp, return it. If there is more
    /// than one such discontinuity, any can be reported, as the function will
    /// be called repeatedly after subdivision of the range.
    ///
    /// Do not report cusps at the endpoints of the range. (TODO: talk about
    /// this)
    ///
    /// The definition of what exactly constitutes a cusp is somewhat loose.
    /// If a cusp is missed, then the curve fitting algorithm will attempt to
    /// fit the curve with a smooth curve, which is generally not a disaster
    /// will usually result in more subdivision. Conversely, it might be useful
    /// to report near-cusps, specifically points of curvature maxima where the
    /// curvature is large but still mathematically finite.
    fn break_cusp(&self, range: Range<f64>) -> Option<f64>;
}

/// A sample point of a curve for fitting.
pub struct CurveFitSample {
    /// A point on the curve at the sample location.
    pub p: Point,
    /// A vector tangent to the curve at the sample location.
    pub tangent: Vec2,
}

impl CurveFitSample {
    /// Intersect a ray orthogonal to the tangent with the given cubic.
    ///
    /// Returns a vector of `t` values on the cubic.
    fn intersect(&self, c: CubicBez) -> ArrayVec<f64, 3> {
        let p1 = 3.0 * (c.p1 - c.p0);
        let p2 = 3.0 * c.p2.to_vec2() - 6.0 * c.p1.to_vec2() + 3.0 * c.p0.to_vec2();
        let p3 = (c.p3 - c.p0) - 3.0 * (c.p2 - c.p1);
        let c0 = (c.p0 - self.p).dot(self.tangent);
        let c1 = p1.dot(self.tangent);
        let c2 = p2.dot(self.tangent);
        let c3 = p3.dot(self.tangent);
        solve_cubic(c0, c1, c2, c3)
            .into_iter()
            .filter(|t| (0.0..=1.0).contains(t))
            .collect()
    }
}

/// Generate a Bézier path that fits the source curve.
///
/// Discussion question: should this be a method on the trait instead?
///
/// Another direction this could go would be taking a dyn reference.
pub fn fit_to_bezpath(source: &impl ParamCurveFit, accuracy: f64) -> BezPath {
    let mut path = BezPath::new();
    fit_to_bezpath_rec(source, 0.0..1.0, accuracy, &mut path);
    path
}

// Discussion question: possibly should take endpoint samples, to avoid
// duplication of that work.
fn fit_to_bezpath_rec(
    source: &impl ParamCurveFit,
    range: Range<f64>,
    accuracy: f64,
    path: &mut BezPath,
) {
    let start = range.start;
    let end = range.end;
    if let Some(t) = source.break_cusp(start..end) {
        fit_to_bezpath_rec(source, start..t, accuracy, path);
        fit_to_bezpath_rec(source, t..end, accuracy, path);
    } else if let Some((c, _)) = fit_to_cubic(source, start..end, accuracy) {
        if path.is_empty() {
            path.move_to(c.p0);
        }
        path.curve_to(c.p1, c.p2, c.p3);
    } else {
        // A smarter approach is possible than midpoint subdivision, but would be
        // a significant increase in complexity.
        let t = 0.5 * (start + end);
        fit_to_bezpath_rec(source, start..t, accuracy, path);
        fit_to_bezpath_rec(source, t..end, accuracy, path);
    }
}

/// Fit a single cubic to a range of the source curve.
///
/// Returns the cubic segment and the square of the error.
/// Discussion question: should this be a method on the trait instead?
pub fn fit_to_cubic(
    source: &impl ParamCurveFit,
    range: Range<f64>,
    accuracy: f64,
) -> Option<(CubicBez, f64)> {
    // TODO: handle case where chord is short; return `None` if subdivision
    // will be useful (ie resulting chord is longer), otherwise simple short
    // cubic (maybe raised line).

    let start = source.sample_pt_tangent(range.start, 1.0);
    let end = source.sample_pt_tangent(range.end, -1.0);
    let d = end.p - start.p;
    let th = d.atan2();
    let chord2 = d.hypot2();
    let th0 = start.tangent.atan2() - th;
    let th1 = th - end.tangent.atan2();

    let (mut area, mut x, mut y) = source.moment_integrals(range.clone());
    let (x0, y0) = (start.p.x, start.p.y);
    let (dx, dy) = (d.x, d.y);
    // Subtract off area of chord
    area -= dx * (y0 + 0.5 * dy);
    // `area` is signed area of closed curve segment.
    // This quantity is invariant to translation and rotation.

    // Subtract off moment of chord
    let dy_3 = dy * (1. / 3.);
    x -= dx * (x0 * y0 + 0.5 * (x0 * dy + y0 * dx) + dy_3 * dx);
    y -= dx * (y0 * y0 + y0 * dy + dy_3 * dy);
    // Translate start point to origin; convert raw integrals to moments.
    x -= x0 * area;
    y = 0.5 * y - y0 * area;
    // Rotate into place (this also scales up by chordlength for efficiency).
    let moment = d.x * x + d.y * y;
    // `moment` is the chordlength times the x moment of the curve translated
    // so its start point is on the origin, and rotated so its end point is on the
    // x axis.

    let chord2_inv = chord2.recip();
    let unit_area = area * chord2_inv;
    let mx = moment * chord2_inv.powi(2);
    // `unit_area` is signed area scaled to unit chord; `mx` is scaled x moment

    let chord = chord2.sqrt();
    let aff = Affine::translate(start.p.to_vec2()) * Affine::rotate(th) * Affine::scale(chord);
    // A few things here.
    //
    // First, it's not awesome to sample `t` uniformly, it would be better to
    // do an approximate arc length parametrization.
    //
    // Second, rejection of lower accuracy curves would be faster if we permuted
    // the order of samples, for example bit-reversing.
    const N_SAMPLE: usize = 10;
    let step: f64 = (range.end - range.start) * (1.0 / (N_SAMPLE + 1) as f64);
    let samples: ArrayVec<_, 10> = (0..N_SAMPLE)
        .map(|i| source.sample_pt_tangent(range.start + (i + 1) as f64 * step, 1.0))
        .collect();
    let acc2 = accuracy * accuracy;
    let mut best_c = None;
    let mut best_err2 = None;
    for cand in cubic_fit(th0, th1, unit_area, mx) {
        let c = aff * cand;
        let mut max_err2 = 0.0;
        for sample in &samples {
            let intersections = sample.intersect(c);
            if intersections.is_empty() {
                max_err2 = acc2 + 1.0;
                break;
            }
            let mut best = None;
            for t in intersections {
                let err = sample.p.distance_squared(c.eval(t));
                if best.map(|best| err < best).unwrap_or(true) {
                    best = Some(err);
                }
            }
            max_err2 = best.unwrap().max(max_err2);
            if max_err2 > acc2 {
                break;
            }
        }
        if max_err2 < acc2 && best_err2.map(|best| max_err2 < best).unwrap_or(true) {
            best_c = Some(c);
            best_err2 = Some(max_err2);
        }
    }
    match (best_c, best_err2) {
        (Some(c), Some(err2)) => Some((c, err2)),
        _ => None,
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
    // TODO: "roots" is not a good name for this variable, as it also contains
    // the real part of complex conjugate pairs.
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
                    (s1 / s01, 0.0)
                }
            } else {
                (0.0, s0 / s01)
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

const HUGE: f64 = 1e12;

/// Optimized curve fitting.
///
/// Note: right now this only handles smooth curves (no cusps).
pub fn fit_to_bezpath_opt(source: &impl ParamCurveFit, accuracy: f64) -> BezPath {
    let mut path = BezPath::new();
    let (c, err2) = fit_to_cubic(source, 0.0..1.0, HUGE).unwrap();
    let err = err2.sqrt();
    if err < accuracy {
        path.move_to(c.p0);
        path.curve_to(c.p1, c.p2, c.p3);
        return path;
    }
    let mut t0 = 0.0;
    let mut n = 0;
    let last_err;
    loop {
        n += 1;
        match fit_opt_segment(source, accuracy, t0) {
            Ok(t1) => t0 = t1,
            Err(err) => {
                last_err = err;
                break;
            }
        }
    }
    const EPS: f64 = 1e-9;
    let f = |x| fit_opt_err_delta(source, x, n);
    let k1 = 0.2 / accuracy;
    let ya = -err;
    let yb = accuracy - last_err;
    let x = solve_itp(f, 0.0, accuracy, EPS, 1, k1, ya, yb);
    println!("got fit with n={}, err={}", n, x);
    t0 = 0.0;
    for i in 0..n {
        let t1 = if i < n - 1 {
            fit_opt_segment(source, x, t0).unwrap()
        } else {
            1.0
        };
        let (c, _) = fit_to_cubic(source, t0..t1, HUGE).unwrap();
        if i == 0 {
            path.move_to(c.p0);
        }
        path.curve_to(c.p1, c.p2, c.p3);
        t0 = t1;
    }
    path
}

fn measure_one_seg(source: &impl ParamCurveFit, range: Range<f64>) -> Option<f64> {
    fit_to_cubic(source, range, HUGE).map(|(_, err2)| err2.sqrt())
}

/// An Ok return is the t1 that hits the desired accuracy.
/// An Err return is the error of t0..1.
fn fit_opt_segment(source: &impl ParamCurveFit, accuracy: f64, t0: f64) -> Result<f64, f64> {
    let missing_err = accuracy * 2.0;
    let err = measure_one_seg(source, t0..1.0).unwrap_or(missing_err);
    if err <= accuracy {
        return Err(err);
    }
    let f = |x| {
        let err = measure_one_seg(source, t0..x).unwrap_or(missing_err);
        err - accuracy
    };
    const EPS: f64 = 1e-9;
    let k1 = 2.0 / (1.0 - t0);
    let t1 = solve_itp(f, t0, 1.0, EPS, 1, k1, -accuracy, err - accuracy);
    Ok(t1)
}

// Returns delta error (accuracy - error of last seg)
fn fit_opt_err_delta(source: &impl ParamCurveFit, accuracy: f64, n: usize) -> f64 {
    let mut t0 = 0.0;
    for _ in 0..n - 1 {
        // If this returns Err, then n - 1 will work, which of course means the
        // error is highly non-monotonic. We should probably harvest that solution.
        t0 = fit_opt_segment(source, accuracy, t0).unwrap();
    }
    let err = measure_one_seg(source, t0..1.0).unwrap_or(accuracy * 2.0);
    accuracy - err
}
