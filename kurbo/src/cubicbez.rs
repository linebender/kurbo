// Copyright 2018 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Cubic Bézier segments.

use alloc::vec;
use alloc::vec::Vec;
use core::ops::{Mul, Range};

use crate::MAX_EXTREMA;
use crate::{Line, QuadSpline, Vec2};
use arrayvec::ArrayVec;

use crate::common::{
    GAUSS_LEGENDRE_COEFFS_8, GAUSS_LEGENDRE_COEFFS_8_HALF, GAUSS_LEGENDRE_COEFFS_16_HALF,
    GAUSS_LEGENDRE_COEFFS_24_HALF, solve_cubic, solve_quadratic, solve_quartic,
};
use crate::{
    Affine, Nearest, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveCurvature,
    ParamCurveDeriv, ParamCurveExtrema, ParamCurveNearest, PathEl, Point, QuadBez, Rect, Shape,
};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

const MAX_SPLINE_SPLIT: usize = 100;

/// A single cubic Bézier segment.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[allow(missing_docs)]
pub struct CubicBez {
    pub p0: Point,
    pub p1: Point,
    pub p2: Point,
    pub p3: Point,
}

/// An iterator which produces quadratic Bézier segments.
struct ToQuads {
    c: CubicBez,
    i: usize,
    n: usize,
}

/// Classification result for cusp detection.
#[derive(Debug)]
pub enum CuspType {
    /// Cusp is a loop.
    Loop,
    /// Cusp has two closely spaced inflection points.
    DoubleInflection,
}

/// The largest width `|C'| / |C''|`, in parameter units, of a minimum of
/// the speed that [`CubicBez::regularize_cusp_at`] treats as a cusp.
///
/// A near-cusp turns through nearly a half turn within a few widths, so its
/// width is small whatever the size of the curve. The curvature of a curve
/// small relative to the dimension is large everywhere, but its width is
/// not, and regularizing it would only distort it.
const CUSP_MAX_WIDTH: f64 = 0.03;

impl CubicBez {
    /// Create a new cubic Bézier segment.
    #[inline(always)]
    pub fn new<P: Into<Point>>(p0: P, p1: P, p2: P, p3: P) -> CubicBez {
        CubicBez {
            p0: p0.into(),
            p1: p1.into(),
            p2: p2.into(),
            p3: p3.into(),
        }
    }

    /// Convert to quadratic Béziers.
    ///
    /// The iterator returns the start and end parameter in the cubic of each quadratic
    /// segment, along with the quadratic.
    ///
    /// Note that the resulting quadratic Béziers are not in general G1 continuous;
    /// they are optimized for minimizing distance error.
    ///
    /// This iterator will always produce at least one `QuadBez`.
    #[inline]
    pub fn to_quads(&self, accuracy: f64) -> impl Iterator<Item = (f64, f64, QuadBez)> {
        // The maximum error, as a vector from the cubic to the best approximating
        // quadratic, is proportional to the third derivative, which is constant
        // across the segment. Thus, the error scales down as the third power of
        // the number of subdivisions. Our strategy then is to subdivide `t` evenly.
        //
        // This is an overestimate of the error because only the component
        // perpendicular to the first derivative is important. But the simplicity is
        // appealing.

        // This magic number is the square of 36 / sqrt(3).
        // See: http://caffeineowl.com/graphics/2d/vectorial/cubic2quad01.html
        let max_hypot2 = 432.0 * accuracy * accuracy;
        let p1x2 = 3.0 * self.p1.to_vec2() - self.p0.to_vec2();
        let p2x2 = 3.0 * self.p2.to_vec2() - self.p3.to_vec2();
        let err = (p2x2 - p1x2).hypot2();
        let n = ((err / max_hypot2).powf(1. / 6.0).ceil() as usize).max(1);

        ToQuads { c: *self, n, i: 0 }
    }

    /// Return a [`QuadSpline`] approximating this cubic Bézier.
    ///
    /// Returns `None` if no suitable approximation is found within the given
    /// tolerance.
    pub fn approx_spline(&self, accuracy: f64) -> Option<QuadSpline> {
        (1..=MAX_SPLINE_SPLIT).find_map(|n| self.approx_spline_n(n, accuracy))
    }

    // Approximate a cubic curve with a quadratic spline of `n` curves
    fn approx_spline_n(&self, n: usize, accuracy: f64) -> Option<QuadSpline> {
        if n == 1 {
            return self
                .try_approx_quadratic(accuracy)
                .map(|quad| QuadSpline::new(vec![quad.p0, quad.p1, quad.p2]));
        }
        let mut cubics = self.split_into_n(n);

        // The above function guarantees that the iterator returns n items,
        // which is why we're unwrapping things with wild abandon.
        let mut next_cubic = cubics.next().unwrap();
        let mut next_q1: Point = next_cubic.approx_quad_control(0.0);
        let mut q2 = self.p0;
        let mut d1 = Vec2::ZERO;
        let mut spline = vec![self.p0, next_q1];
        for i in 1..=n {
            let current_cubic: CubicBez = next_cubic;
            let q0 = q2;
            let q1 = next_q1;
            q2 = if i < n {
                next_cubic = cubics.next().unwrap();
                next_q1 = next_cubic.approx_quad_control(i as f64 / (n - 1) as f64);

                spline.push(next_q1);
                q1.midpoint(next_q1)
            } else {
                current_cubic.p3
            };
            let d0 = d1;
            d1 = q2.to_vec2() - current_cubic.p3.to_vec2();

            if d1.hypot() > accuracy
                || !CubicBez::new(
                    d0.to_point(),
                    q0.lerp(q1, 2.0 / 3.0) - current_cubic.p1.to_vec2(),
                    q2.lerp(q1, 2.0 / 3.0) - current_cubic.p2.to_vec2(),
                    d1.to_point(),
                )
                .fit_inside(accuracy)
            {
                return None;
            }
        }
        spline.push(self.p3);
        Some(QuadSpline::new(spline))
    }

    fn approx_quad_control(&self, t: f64) -> Point {
        let p1 = self.p0 + (self.p1 - self.p0) * 1.5;
        let p2 = self.p3 + (self.p2 - self.p3) * 1.5;
        p1.lerp(p2, t)
    }

    /// Approximate a cubic with a single quadratic
    ///
    /// Returns a quadratic approximating the given cubic that maintains
    /// endpoint tangents if that is within tolerance, or `None` otherwise.
    fn try_approx_quadratic(&self, accuracy: f64) -> Option<QuadBez> {
        if let Some(q1) = Line::new(self.p0, self.p1)
            .crossing_point(Line::new(self.p2, self.p3))
            .or_else(|| {
                // with 3 to 4 consecutive equal points (with no valid crossing_point),
                // we can try to use the common point as the quadratic control point.
                // This matches fonttools' cu2qu: https://github.com/fonttools/fonttools/pull/3904
                (self.p1 == self.p2 && (self.p0 == self.p1 || self.p2 == self.p3))
                    .then_some(self.p1)
            })
        {
            let c1 = self.p0.lerp(q1, 2.0 / 3.0);
            let c2 = self.p3.lerp(q1, 2.0 / 3.0);
            if !CubicBez::new(
                Point::ZERO,
                c1 - self.p1.to_vec2(),
                c2 - self.p2.to_vec2(),
                Point::ZERO,
            )
            .fit_inside(accuracy)
            {
                return None;
            }
            return Some(QuadBez::new(self.p0, q1, self.p3));
        }
        None
    }

    fn split_into_n(&self, n: usize) -> impl Iterator<Item = CubicBez> {
        // for certain small values of `n` we precompute all our values.
        // if we have six or fewer items we precompute them.
        let mut storage = ArrayVec::<_, 6>::new();

        match n {
            1 => storage.push(*self),
            2 => {
                let (l, r) = self.subdivide();
                storage.try_extend_from_slice(&[r, l]).unwrap();
            }
            3 => {
                let (left, mid, right) = self.subdivide_3();
                storage.try_extend_from_slice(&[right, mid, left]).unwrap();
            }
            4 => {
                let (l, r) = self.subdivide();
                let (ll, lr) = l.subdivide();
                let (rl, rr) = r.subdivide();
                storage.try_extend_from_slice(&[rr, rl, lr, ll]).unwrap();
            }
            6 => {
                let (l, r) = self.subdivide();
                let (l1, l2, l3) = l.subdivide_3();
                let (r1, r2, r3) = r.subdivide_3();
                storage
                    .try_extend_from_slice(&[r3, r2, r1, l3, l2, l1])
                    .unwrap();
            }
            _ => (),
        }

        // a limitation of returning 'impl Trait' is that the implementation
        // can only return a single concrete type; that is you cannot return
        // Vec::into_iter() from one branch, and then HashSet::into_iter from
        // another branch.
        //
        // This means we have to get a bit fancy, and have a single concrete
        // type that represents both of our possible cases.

        let mut storage = if storage.is_empty() {
            None
        } else {
            Some(storage)
        };

        // used in the fallback case
        let mut i = 0;
        let (a, b, c, d) = self.parameters();
        let dt = 1.0 / n as f64;
        let delta_2 = dt * dt;
        let delta_3 = dt * delta_2;

        core::iter::from_fn(move || {
            // if storage exists, we use it exclusively
            if let Some(storage) = storage.as_mut() {
                return storage.pop();
            }

            // if storage does not exist, we are exclusively working down here.
            if i >= n {
                return None;
            }

            let t1 = i as f64 * dt;
            let t1_2 = t1 * t1;
            let a1 = a * delta_3;
            let b1 = (3.0 * a * t1 + b) * delta_2;
            let c1 = (2.0 * b * t1 + c + 3.0 * a * t1_2) * dt;
            let d1 = a * t1 * t1_2 + b * t1_2 + c * t1 + d;
            let result = CubicBez::from_parameters(a1, b1, c1, d1);
            i += 1;
            Some(result)
        })
    }

    fn parameters(&self) -> (Vec2, Vec2, Vec2, Vec2) {
        let c = (self.p1 - self.p0) * 3.0;
        let b = (self.p2 - self.p1) * 3.0 - c;
        let d = self.p0.to_vec2();
        let a = self.p3.to_vec2() - d - c - b;
        (a, b, c, d)
    }

    /// Rust port of cu2qu [calc_cubic_points](https://github.com/fonttools/fonttools/blob/3b9a73ff8379ab49d3ce35aaaaf04b3a7d9d1655/Lib/fontTools/cu2qu/cu2qu.py#L63-L68)
    fn from_parameters(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Self {
        let p0 = d.to_point();
        let p1 = c.div_exact(3.0).to_point() + d;
        let p2 = (b + c).div_exact(3.0).to_point() + p1.to_vec2();
        let p3 = (a + d + c + b).to_point();
        CubicBez::new(p0, p1, p2, p3)
    }

    fn subdivide_3(&self) -> (CubicBez, CubicBez, CubicBez) {
        let (p0, p1, p2, p3) = (
            self.p0.to_vec2(),
            self.p1.to_vec2(),
            self.p2.to_vec2(),
            self.p3.to_vec2(),
        );
        // The original Python cu2qu code here does not use division operator to divide by 27 but
        // instead uses multiplication by the reciprocal 1 / 27. We want to match it exactly
        // to avoid any floating point differences, hence in this particular case we do not use div_exact.
        // I could directly use the Vec2 Div trait (also implemented as multiplication by reciprocal)
        // but I prefer to be explicit here.
        // Source: https://github.com/fonttools/fonttools/blob/85c80be/Lib/fontTools/cu2qu/cu2qu.py#L215-L218
        // See also: https://github.com/linebender/kurbo/issues/272
        let one_27th = 27.0_f64.recip();
        let mid1 = ((8.0 * p0 + 12.0 * p1 + 6.0 * p2 + p3) * one_27th).to_point();
        let deriv1 = (p3 + 3.0 * p2 - 4.0 * p0) * one_27th;
        let mid2 = ((p0 + 6.0 * p1 + 12.0 * p2 + 8.0 * p3) * one_27th).to_point();
        let deriv2 = (4.0 * p3 - 3.0 * p1 - p0) * one_27th;

        let left = CubicBez::new(
            self.p0,
            (2.0 * p0 + p1).div_exact(3.0).to_point(),
            mid1 - deriv1,
            mid1,
        );
        let mid = CubicBez::new(mid1, mid1 + deriv1, mid2 - deriv2, mid2);
        let right = CubicBez::new(
            mid2,
            mid2 + deriv2,
            (p2 + 2.0 * p3).div_exact(3.0).to_point(),
            self.p3,
        );
        (left, mid, right)
    }

    /// Does this curve fit inside the given distance from the origin?
    ///
    /// Rust port of cu2qu [cubic_farthest_fit_inside](https://github.com/fonttools/fonttools/blob/3b9a73ff8379ab49d3ce35aaaaf04b3a7d9d1655/Lib/fontTools/cu2qu/cu2qu.py#L281)
    fn fit_inside(&self, distance: f64) -> bool {
        if self.p2.to_vec2().hypot() <= distance && self.p1.to_vec2().hypot() <= distance {
            return true;
        }
        let mid =
            (self.p0.to_vec2() + 3.0 * (self.p1.to_vec2() + self.p2.to_vec2()) + self.p3.to_vec2())
                * 0.125;
        if mid.hypot() > distance {
            return false;
        }
        // Split in two. Note that cu2qu here uses a 3/8 subdivision. I don't know why.
        let (left, right) = self.subdivide();
        left.fit_inside(distance) && right.fit_inside(distance)
    }

    /// Is this cubic Bezier curve finite?
    #[inline]
    pub const fn is_finite(&self) -> bool {
        self.p0.is_finite() && self.p1.is_finite() && self.p2.is_finite() && self.p3.is_finite()
    }

    /// Is this cubic Bezier curve NaN?
    #[inline]
    pub const fn is_nan(&self) -> bool {
        self.p0.is_nan() || self.p1.is_nan() || self.p2.is_nan() || self.p3.is_nan()
    }

    /// Determine the inflection points.
    ///
    /// Return value is t parameter for the inflection points of the curve segment.
    /// There are a maximum of two for a cubic Bézier.
    ///
    /// See <https://www.caffeineowl.com/graphics/2d/vectorial/cubic-inflexion.html>
    /// for the theory.
    pub fn inflections(&self) -> ArrayVec<f64, 2> {
        let a = self.p1 - self.p0;
        let b = (self.p2 - self.p1) - a;
        let c = (self.p3 - self.p0) - 3. * (self.p2 - self.p1);
        solve_quadratic(a.cross(b), a.cross(c), b.cross(c))
            .iter()
            .copied()
            .filter(|t| *t >= 0.0 && *t <= 1.0)
            .collect()
    }

    /// Find points on the curve where the tangent line passes through the
    /// given point.
    ///
    /// Result is array of t values such that the tangent line from the curve
    /// evaluated at that point goes through the argument point.
    pub fn tangents_to_point(&self, p: Point) -> ArrayVec<f64, 4> {
        let (a, b, c, d_orig) = self.parameters();
        let d = d_orig - p.to_vec2();
        // coefficients of x(t) \cross x'(t)
        let c4 = b.cross(a);
        let c3 = 2.0 * c.cross(a);
        let c2 = c.cross(b) + 3.0 * d.cross(a);
        let c1 = 2.0 * d.cross(b);
        let c0 = d.cross(c);
        solve_quartic(c0, c1, c2, c3, c4)
            .iter()
            .copied()
            .filter(|t| *t >= 0.0 && *t <= 1.0)
            .collect()
    }

    /// Preprocess a cubic Bézier to ease numerical robustness.
    ///
    /// If the cubic Bézier segment has zero or near-zero derivatives as an interior
    /// cusp, perturb the control points to make curvature finite, avoiding
    /// numerical robustness problems in offset and stroke.
    ///
    /// The offset code calls the constituent functions separately, so that
    /// tip detection can share the minima of the speed; the tests use this
    /// combined form as the reference against which errors are measured.
    #[cfg(test)]
    pub(crate) fn regularize_cusp(&self, dimension: f64) -> CubicBez {
        if !self.may_have_slow_point(0.0, dimension) {
            return *self;
        }
        let minima = self.slow_points(self.cusp_speed_squared(dimension));
        self.regularize_cusp_at(dimension, &minima).unwrap_or(*self)
    }

    /// An upper bound on `|C'|^2` at a minimum of the speed for which
    /// `Self::regularize_cusp` might detect a cusp.
    ///
    /// At a minimum of the speed, the derivative is perpendicular to the
    /// second derivative, so the curvature is `|C''| / |C'|^2`. It exceeds
    /// `1 / dimension` where `|C'|^2 <= |C''| dimension`, and `|C''|` attains
    /// its maximum at an endpoint.
    #[inline(always)]
    pub(crate) fn cusp_speed_squared(&self, dimension: f64) -> f64 {
        let d01 = self.p1 - self.p0;
        let d12 = self.p2 - self.p1;
        let d23 = self.p3 - self.p2;
        6.0 * (d12 - d01).hypot2().max((d23 - d12).hypot2()).sqrt() * dimension
    }

    /// `Self::regularize_cusp`, given the minima of the speed computed by
    /// [`Self::slow_points`] with a limit of at least
    /// [`Self::cusp_speed_squared`].
    ///
    /// Returns `None` if no control point was moved.
    #[cold]
    pub(crate) fn regularize_cusp_at(&self, dimension: f64, minima: &[f64]) -> Option<CubicBez> {
        let cusp_type = self.detect_cusp(dimension, minima)?;
        let mut c = *self;
        // If a control point is too near its endpoint, displace it along the
        // tangent.
        let d01 = c.p1 - c.p0;
        let d01h = d01.hypot();
        let d23 = c.p3 - c.p2;
        let d23h = d23.hypot();
        match cusp_type {
            CuspType::Loop => {
                c.p1 += (dimension / d01h) * d01;
                c.p2 -= (dimension / d23h) * d23;
                Some(c)
            }
            CuspType::DoubleInflection => {
                // Avoid making control distance smaller than dimension
                let nudge0 = d01h > 2.0 * dimension;
                let nudge1 = d23h > 2.0 * dimension;
                if nudge0 {
                    c.p1 -= (dimension / d01h) * d01;
                }
                if nudge1 {
                    c.p2 += (dimension / d23h) * d23;
                }
                (nudge0 || nudge1).then_some(c)
            }
        }
    }

    /// Detect whether there is a cusp at one of the given minima of the
    /// speed.
    ///
    /// Return a cusp classification if there is a cusp with curvature greater than
    /// the reciprocal of the given dimension.
    fn detect_cusp(&self, dimension: f64, minima: &[f64]) -> Option<CuspType> {
        let d01 = self.p1 - self.p0;
        let d23 = self.p3 - self.p2;
        // A zero-length handle means a vanishing derivative at the endpoint,
        // which is not regularized; the displacements in
        // `regularize_cusp_at` divide by the handle lengths.
        if d01.hypot2() == 0.0 || d23.hypot2() == 0.0 {
            return None;
        }
        let q = self.deriv();
        for &t in minima {
            // The curvature is `|C''| / |C'|^2` here; see
            // `cusp_speed_squared`. Unlike the cross product, this form is
            // robust at an exact cusp, where the derivative consists of
            // rounding error that may be parallel to the second derivative.
            let d = q.eval(t).to_vec2();
            let d2 = q.deriv().eval(t).to_vec2();
            if d.hypot2() <= d2.hypot() * dimension
                && d.hypot2() < (CUSP_MAX_WIDTH * CUSP_MAX_WIDTH) * d2.hypot2()
            {
                let d02 = self.p2 - self.p0;
                let d03 = self.p3 - self.p0;
                let det_012 = d01.cross(d02);
                let det_013 = d01.cross(d03);
                let det_023 = d02.cross(d03);
                let a = 3. * det_012 + det_023 - 2. * det_013;
                let b = -3. * det_012 + det_013;
                let c = det_012;
                let d = b * b - 4. * a * c;
                if d > 0.0 {
                    return Some(CuspType::DoubleInflection);
                } else {
                    return Some(CuspType::Loop);
                }
            }
        }
        None
    }

    /// Whether the speed `|C'|` may have an interior local minimum at which
    /// it is small: either `|C'| < max_width |C''|`, or both
    /// `|C'|^2 <= |C''| dimension` and `|C'| < CUSP_MAX_WIDTH |C''|`, with
    /// `|C''|` the larger of its values at the endpoints. The first condition
    /// indicates a sharp turn (see `crate::offset`), and the second a cusp to
    /// be regularized (see [`Self::detect_cusp`]).
    ///
    /// This is an inexpensive conservative test, which nearly every curve
    /// fails, to be followed by [`Self::slow_points`] for those that pass. It
    /// compares the limits with a lower bound on `|C'|^2`, namely the
    /// smallest of its Bernstein coefficients. With a zero-length handle that
    /// coefficient is zero, and a different bound is used: then
    /// `C' / 3 = t (a + t m)`, and a minimum lies at the larger root of a
    /// quadratic (see [`Self::slow_points`]). That root is no less than the
    /// minimizer of the quadratic, and `|a + t m|^2` is no less than its
    /// minimum on `[0, 1]`.
    #[inline(always)]
    pub(crate) fn may_have_slow_point(&self, max_width: f64, dimension: f64) -> bool {
        // The control points of the hodograph over 3.
        let q0 = self.p1 - self.p0;
        let q1 = self.p2 - self.p1;
        let q2 = self.p3 - self.p2;
        // With `c2` the square of `|C''| / 6`, the limit on `|C'|^2 / 9` is
        // `sharp c2` for a sharp turn, and for a cusp it is both `narrow c2`
        // and the value whose square is `cusp c2`.
        let sharp = 4.0 * max_width * max_width;
        let narrow = 4.0 * CUSP_MAX_WIDTH * CUSP_MAX_WIDTH;
        let cusp = (4.0 / 9.0) * dimension * dimension;
        if q0 == Vec2::ZERO || q2 == Vec2::ZERO {
            let (a, m) = Self::zero_handle_factor(q0, q1, q2);
            let (a_a, a_m, m_m) = (a.hypot2(), a.dot(m), m.hypot2());
            // Check that the larger root of `|a|^2 + 3 a.m t + 2 |m|^2 t^2`
            // lies in (0, 1): the roots are real and positive, and the
            // larger one is below 1. Written so that a NaN yields `false`.
            if !(a_m < 0.0
                && 9.0 * a_m * a_m > 8.0 * a_a * m_m
                && -3.0 * a_m < 4.0 * m_m
                && a_a + 3.0 * a_m + 2.0 * m_m > 0.0)
            {
                return false;
            }
            // The quadratic attains its minimum at `-3 a.m / (4 |m|^2)`, and
            // `|a + t m|^2` at `-a.m / |m|^2` if that is below 1, where its
            // value is `(|a|^2 |m|^2 - (a.m)^2) / |m|^2`, and otherwise at 1.
            let (smallest, denominator) = if -a_m < m_m {
                (a_a * m_m - a_m * a_m, m_m)
            } else {
                (a_a + 2.0 * a_m + m_m, 1.0)
            };
            // A lower bound on `|C'|^2 / 9` at the minimum, as a numerator and
            // denominator.
            let bound = 9.0 * a_m * a_m * smallest;
            let denominator = 16.0 * m_m * m_m * denominator;
            // `C'' / 6` is `a / 2` at the endpoint with the zero-length
            // handle, and `a / 2 + m` at the other.
            let c2 = (0.25 * a_a).max(0.25 * a_a + a_m + m_m);
            return !(bound > sharp * c2 * denominator
                && (bound > narrow * c2 * denominator
                    || bound * bound > cusp * c2 * denominator * denominator));
        }
        let (q0_q0, q1_q1, q2_q2) = (q0.hypot2(), q1.hypot2(), q2.hypot2());
        let (q0_q1, q1_q2, q0_q2) = (q0.dot(q1), q1.dot(q2), q0.dot(q2));
        // The Bernstein coefficients of `|C'|^2 / 9`.
        let smallest = q0_q0
            .min(q0_q1)
            .min((2.0 * q1_q1 + q0_q2) * (1.0 / 3.0))
            .min(q1_q2)
            .min(q2_q2);
        let c2 = (q0_q0 - 2.0 * q0_q1 + q1_q1).max(q1_q1 - 2.0 * q1_q2 + q2_q2);
        !(smallest > sharp * c2 && (smallest > narrow * c2 || smallest * smallest > cusp * c2))
    }

    /// With a zero-length handle, the hodograph divided by 3 is
    /// `t (a + t m)`, with `t` measured from the endpoint with that handle.
    /// This returns `a` and `m`, given the control points of the hodograph
    /// divided by 3.
    #[inline(always)]
    fn zero_handle_factor(q0: Vec2, q1: Vec2, q2: Vec2) -> (Vec2, Vec2) {
        let a = 2.0 * q1;
        (a, if q0 == Vec2::ZERO { q2 - a } else { q0 - a })
    }

    /// The interior parameters at which the speed `|C'|` has a local minimum
    /// whose square may be as small as `speed_squared`, in increasing order.
    /// A cubic has at most two minima of the speed.
    ///
    /// This returns every minimum at which the speed is that small, and may
    /// also return others, or parameters close to them; the caller is
    /// expected to test the results. As this is relatively expensive, it
    /// should only be called when [`Self::may_have_slow_point`] passes.
    ///
    /// A minimum is a root of the cubic `C'.C''` at which it changes from
    /// negative to positive. With a zero-length handle, `C'(t) / 3` is
    /// `t (a + t m)`, so `C'.C''` is `t` times the quadratic
    /// `|a|^2 + 3 a.m t + 2 |m|^2 t^2`, and the speed has an interior minimum
    /// at its larger root, if there is one.
    #[cold]
    pub(crate) fn slow_points(&self, speed_squared: f64) -> ArrayVec<f64, 2> {
        let mut minima = ArrayVec::new();
        // The control points of the hodograph over 3.
        let q0 = self.p1 - self.p0;
        let q1 = self.p2 - self.p1;
        let q2 = self.p3 - self.p2;
        let limit = speed_squared * (1.0 / 9.0);
        if q0 == Vec2::ZERO || q2 == Vec2::ZERO {
            let (a, m) = Self::zero_handle_factor(q0, q1, q2);
            let (a_a, a_m, m_m) = (a.hypot2(), a.dot(m), m.hypot2());
            let discriminant = 9.0 * a_m * a_m - 8.0 * a_a * m_m;
            // Both roots are positive only if `a.m` is negative. Written so
            // that a NaN yields no minimum. When both handles are zero, the
            // larger root is 1 up to rounding, and a parameter just below 1
            // may be returned; the caller discards such a point.
            if a_m < 0.0 && discriminant > 0.0 {
                let t = (-3.0 * a_m + discriminant.sqrt()) / (4.0 * m_m);
                if t < 1.0 && t * t * (a + t * m).hypot2() <= limit {
                    minima.push(if q0 == Vec2::ZERO { t } else { 1.0 - t });
                }
            }
            return minima;
        }
        let (q0_q1, q1_q2, q0_q2) = (q0.dot(q1), q1.dot(q2), q0.dot(q2));
        let (q0_q0, q1_q1, q2_q2) = (q0.hypot2(), q1.hypot2(), q2.hypot2());
        // The Bernstein coefficients of `C'.C''`, times a positive constant.
        // With `e0 = q1 - q0` and `e1 = q2 - q1`, they are `3 q0.e0`,
        // `q0.e1 + 2 q1.e0`, `2 q1.e1 + q2.e0` and `3 q2.e1`. By the variation
        // diminishing property, a rising root requires a negative coefficient
        // followed by a positive one.
        let b = [
            3.0 * (q0_q1 - q0_q0),
            q0_q2 - 3.0 * q0_q1 + 2.0 * q1_q1,
            3.0 * q1_q2 - 2.0 * q1_q1 - q0_q2,
            3.0 * (q2_q2 - q1_q2),
        ];
        let mut negative = false;
        let mut rising = false;
        for x in b {
            negative |= x < 0.0;
            rising |= negative && x > 0.0;
        }
        if !rising {
            return minima;
        }
        // With a nearly zero-length handle a root lies very close to an
        // endpoint, and the solver may return it as a parameter just inside
        // the curve.
        const INSIDE: f64 = 1e-12;
        // The cubic `C'.C''`, times a positive constant, in the power basis.
        let c0 = b[0];
        let c1 = 3.0 * (b[1] - b[0]);
        let c2 = 3.0 * (b[0] - 2.0 * b[1] + b[2]);
        let c3 = b[3] - 3.0 * b[2] + 3.0 * b[1] - b[0];
        for t in solve_cubic(c0, c1, c2, c3) {
            let slope = c1 + t * (2.0 * c2 + t * 3.0 * c3);
            if t > INSIDE && t < 1.0 - INSIDE && slope > 0.0 && !minima.is_full() {
                minima.push(t);
            }
        }
        minima.sort_unstable_by(|a, b| a.total_cmp(b));
        minima
    }

    /// Finds the value of `t` for which `self.eval(t)` is about `y`.
    ///
    /// Assumes that this segment is monotonic in `y` and that it crosses
    /// the height `y`. (Under these assumptions, there is a unique answer.)
    pub(crate) fn solve_monotonic_for_y(&self, y: f64) -> f64 {
        let start = self.start();
        let end = self.end();

        debug_assert!(start.y.min(end.y) <= y && y <= start.y.max(end.y));

        let p1 = self.p1;
        let p2 = self.p2;
        let a = end.y - 3.0 * p2.y + 3.0 * p1.y - start.y;
        let b = 3.0 * (p2.y - 2.0 * p1.y + start.y);
        let c = 3.0 * (p1.y - start.y);
        let d = start.y - y;
        for t in solve_cubic(d, c, b, a) {
            if (0.0..=1.0).contains(&t) {
                return t;
            }
        }

        // Even though we asserted that our y range contains `y`, it's possible
        // that we failed to find a solution numerically. (For example, rounding
        // of a, b, or c might have pushed the root outside of [0.0, 1.0].)
        // If we failed to find a solution, the real solution should be close
        // to one of the endpoints. So just find whichever endpoint was closer.
        if (start.y - y).abs() <= (end.y - y).abs() {
            0.0
        } else {
            1.0
        }
    }
}

/// An iterator for cubic beziers.
pub struct CubicBezIter {
    cubic: CubicBez,
    ix: usize,
}

impl Shape for CubicBez {
    type PathElementsIter<'iter> = CubicBezIter;

    #[inline]
    fn path_elements(&self, _tolerance: f64) -> CubicBezIter {
        CubicBezIter {
            cubic: *self,
            ix: 0,
        }
    }

    #[inline(always)]
    fn area(&self) -> f64 {
        0.0
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        self.arclen(accuracy)
    }

    #[inline(always)]
    fn winding(&self, _pt: Point) -> i32 {
        0
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        ParamCurveExtrema::bounding_box(self)
    }
}

impl Iterator for CubicBezIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.ix += 1;
        match self.ix {
            1 => Some(PathEl::MoveTo(self.cubic.p0)),
            2 => Some(PathEl::CurveTo(self.cubic.p1, self.cubic.p2, self.cubic.p3)),
            _ => None,
        }
    }
}

impl ParamCurve for CubicBez {
    #[inline]
    fn eval(&self, t: f64) -> Point {
        let mt = 1.0 - t;
        let v = self.p0.to_vec2() * (mt * mt * mt)
            + (self.p1.to_vec2() * (mt * mt * 3.0)
                + (self.p2.to_vec2() * (mt * 3.0) + self.p3.to_vec2() * t) * t)
                * t;
        v.to_point()
    }

    fn subsegment(&self, range: Range<f64>) -> CubicBez {
        let (t0, t1) = (range.start, range.end);
        let p0 = self.eval(t0);
        let p3 = self.eval(t1);
        let d = self.deriv();
        let scale = (t1 - t0) * (1.0 / 3.0);
        let p1 = p0 + scale * d.eval(t0).to_vec2();
        let p2 = p3 - scale * d.eval(t1).to_vec2();
        CubicBez { p0, p1, p2, p3 }
    }

    /// Subdivide into halves, using de Casteljau.
    #[inline]
    fn subdivide(&self) -> (CubicBez, CubicBez) {
        let pm = self.eval(0.5);
        (
            CubicBez::new(
                self.p0,
                self.p0.midpoint(self.p1),
                ((self.p0.to_vec2() + self.p1.to_vec2() * 2.0 + self.p2.to_vec2()) * 0.25)
                    .to_point(),
                pm,
            ),
            CubicBez::new(
                pm,
                ((self.p1.to_vec2() + self.p2.to_vec2() * 2.0 + self.p3.to_vec2()) * 0.25)
                    .to_point(),
                self.p2.midpoint(self.p3),
                self.p3,
            ),
        )
    }

    #[inline(always)]
    fn start(&self) -> Point {
        self.p0
    }

    #[inline(always)]
    fn end(&self) -> Point {
        self.p3
    }
}

impl ParamCurveDeriv for CubicBez {
    type DerivResult = QuadBez;

    #[inline]
    fn deriv(&self) -> QuadBez {
        QuadBez::new(
            (3.0 * (self.p1 - self.p0)).to_point(),
            (3.0 * (self.p2 - self.p1)).to_point(),
            (3.0 * (self.p3 - self.p2)).to_point(),
        )
    }
}

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

fn arclen_rec(c: &CubicBez, accuracy: f64, depth: usize) -> f64 {
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
        return arclen_quadrature_core(GAUSS_LEGENDRE_COEFFS_8_HALF, dm, dm1, dm2);
    }
    let est_gauss16_error = (est.powi(6) * 1.5e-11).min(9e-3) * lp_lc;
    if est_gauss16_error < accuracy {
        return arclen_quadrature_core(GAUSS_LEGENDRE_COEFFS_16_HALF, dm, dm1, dm2);
    }
    let est_gauss24_error = (est.powi(9) * 3.5e-16).min(3.5e-3) * lp_lc;
    if est_gauss24_error < accuracy || depth >= 20 {
        return arclen_quadrature_core(GAUSS_LEGENDRE_COEFFS_24_HALF, dm, dm1, dm2);
    }
    let (c0, c1) = c.subdivide();
    arclen_rec(&c0, accuracy * 0.5, depth + 1) + arclen_rec(&c1, accuracy * 0.5, depth + 1)
}

impl ParamCurveArclen for CubicBez {
    /// Arclength of a cubic Bézier segment.
    ///
    /// This is an adaptive subdivision approach using Legendre-Gauss quadrature
    /// in the base case, and an error estimate to decide when to subdivide.
    fn arclen(&self, accuracy: f64) -> f64 {
        arclen_rec(self, accuracy, 0)
    }
}

impl ParamCurveArea for CubicBez {
    #[inline]
    fn signed_area(&self) -> f64 {
        (self.p0.x * (6.0 * self.p1.y + 3.0 * self.p2.y + self.p3.y)
            + 3.0
                * (self.p1.x * (-2.0 * self.p0.y + self.p2.y + self.p3.y)
                    - self.p2.x * (self.p0.y + self.p1.y - 2.0 * self.p3.y))
            - self.p3.x * (self.p0.y + 3.0 * self.p1.y + 6.0 * self.p2.y))
            * (1.0 / 20.0)
    }
}

impl ParamCurveNearest for CubicBez {
    /// Find the nearest point using a quintic solver.
    ///
    /// The polynomial `|self - p|^2` has degree 6, so we find its critical
    /// points and evaluate them all to find the best one.
    fn nearest(&self, p: Point, accuracy: f64) -> Nearest {
        fn eval_t(p: Point, t_best: &mut f64, r_best: &mut Option<f64>, t: f64, p0: Point) {
            let r = (p0 - p).hypot2();
            if r_best.map(|r_best| r < r_best).unwrap_or(true) {
                *r_best = Some(r);
                *t_best = t;
            }
        }

        let mut r_best = None;
        let mut t_best = 0.0;

        // Reparameterize `self - p` as q0 + q1 t + q2 t^2 + q3 t^3.
        let q0 = self.p0 - p;
        let q1 = 3.0 * (self.p1 - self.p0);
        let q2 = 3.0 * (self.p0.to_vec2() - 2.0 * self.p1.to_vec2() + self.p2.to_vec2());
        let q3 = -self.p0.to_vec2() + 3.0 * self.p1.to_vec2() - 3.0 * self.p2.to_vec2()
            + self.p3.to_vec2();

        // Coefficients of the degree-5 polynomial (self - p) \cdot tangent.
        let c0 = q0.dot(q1);
        let c1 = q1.hypot2() + 2.0 * q2.dot(q0);
        let c2 = 3.0 * (q2.dot(q1) + q3.dot(q0));
        let c3 = 4.0 * q3.dot(q1) + 2.0 * q2.hypot2();
        let c4 = 5.0 * q3.dot(q2);
        let c5 = 3.0 * q3.hypot2();

        let roots = polycool::Poly::new([c0, c1, c2, c3, c4, c5]).roots_between(0.0, 1.0, accuracy);

        for &t in &roots {
            eval_t(p, &mut t_best, &mut r_best, t, self.eval(t));
        }

        // If we found all 5 critical points, we can skip evaluating the endpoints.
        if roots.len() != 5 {
            eval_t(p, &mut t_best, &mut r_best, 0.0, self.p0);
            eval_t(p, &mut t_best, &mut r_best, 1.0, self.p3);
        }

        Nearest {
            t: t_best,
            distance_sq: r_best.unwrap(),
        }
    }
}

impl ParamCurveCurvature for CubicBez {}

impl ParamCurveExtrema for CubicBez {
    fn extrema(&self) -> ArrayVec<f64, MAX_EXTREMA> {
        fn one_coord(result: &mut ArrayVec<f64, MAX_EXTREMA>, d0: f64, d1: f64, d2: f64) {
            let a = d0 - 2.0 * d1 + d2;
            let b = 2.0 * (d1 - d0);
            let c = d0;
            let roots = solve_quadratic(c, b, a);
            for &t in &roots {
                if t > 0.0 && t < 1.0 {
                    result.push(t);
                }
            }
        }
        let mut result = ArrayVec::new();
        let d0 = self.p1 - self.p0;
        let d1 = self.p2 - self.p1;
        let d2 = self.p3 - self.p2;
        one_coord(&mut result, d0.x, d1.x, d2.x);
        one_coord(&mut result, d0.y, d1.y, d2.y);
        result.sort_by(|a, b| a.partial_cmp(b).unwrap());
        result
    }
}

impl Mul<CubicBez> for Affine {
    type Output = CubicBez;

    #[inline]
    fn mul(self, c: CubicBez) -> CubicBez {
        CubicBez {
            p0: self * c.p0,
            p1: self * c.p1,
            p2: self * c.p2,
            p3: self * c.p3,
        }
    }
}

impl Iterator for ToQuads {
    type Item = (f64, f64, QuadBez);

    #[inline]
    fn next(&mut self) -> Option<(f64, f64, QuadBez)> {
        if self.i == self.n {
            return None;
        }
        let t0 = self.i as f64 / self.n as f64;
        let t1 = (self.i + 1) as f64 / self.n as f64;
        let seg = self.c.subsegment(t0..t1);
        let p1x2 = 3.0 * seg.p1.to_vec2() - seg.p0.to_vec2();
        let p2x2 = 3.0 * seg.p2.to_vec2() - seg.p3.to_vec2();
        let result = QuadBez::new(seg.p0, ((p1x2 + p2x2) / 4.0).to_point(), seg.p3);
        self.i += 1;
        Some((t0, t1, result))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.n - self.i;
        (remaining, Some(remaining))
    }
}

/// Convert multiple cubic Bézier curves to quadratic splines.
///
/// Ensures that the resulting splines have the same number of control points.
///
/// Rust port of cu2qu [cubic_approx_quadratic](https://github.com/fonttools/fonttools/blob/3b9a73ff8379ab49d3ce35aaaaf04b3a7d9d1655/Lib/fontTools/cu2qu/cu2qu.py#L322)
pub fn cubics_to_quadratic_splines(curves: &[CubicBez], accuracy: f64) -> Option<Vec<QuadSpline>> {
    let mut result = Vec::new();
    let mut split_order = 0;

    while split_order <= MAX_SPLINE_SPLIT {
        split_order += 1;
        result.clear();

        for curve in curves {
            match curve.approx_spline_n(split_order, accuracy) {
                Some(spline) => result.push(spline),
                None => break,
            }
        }

        if result.len() == curves.len() {
            return Some(result);
        }
    }
    None
}
#[cfg(test)]
mod tests {
    use super::CUSP_MAX_WIDTH;
    use crate::{
        Affine, CubicBez, Nearest, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveDeriv,
        ParamCurveExtrema, ParamCurveNearest, Point, QuadBez, QuadSpline, Vec2,
        cubics_to_quadratic_splines,
    };

    #[test]
    fn cubicbez_deriv() {
        // y = x^2
        let c = CubicBez::new(
            (0.0, 0.0),
            (1.0 / 3.0, 0.0),
            (2.0 / 3.0, 1.0 / 3.0),
            (1.0, 1.0),
        );
        let deriv = c.deriv();

        let n = 10;
        for i in 0..=n {
            let t = (i as f64) * (n as f64).recip();
            let delta = 1e-6;
            let p = c.eval(t);
            let p1 = c.eval(t + delta);
            let d_approx = (p1 - p) * delta.recip();
            let d = deriv.eval(t).to_vec2();
            assert!((d - d_approx).hypot() < delta * 2.0);
        }
    }

    #[test]
    fn cubicbez_arclen() {
        // y = x^2
        let c = CubicBez::new(
            (0.0, 0.0),
            (1.0 / 3.0, 0.0),
            (2.0 / 3.0, 1.0 / 3.0),
            (1.0, 1.0),
        );
        let true_arclen = 0.5 * 5.0f64.sqrt() + 0.25 * (2.0 + 5.0f64.sqrt()).ln();
        for i in 0..12 {
            let accuracy = 0.1f64.powi(i);
            let error = c.arclen(accuracy) - true_arclen;
            assert!(error.abs() < accuracy);
        }
    }

    #[test]
    fn cubicbez_inv_arclen() {
        // y = x^2 / 100
        let c = CubicBez::new(
            (0.0, 0.0),
            (100.0 / 3.0, 0.0),
            (200.0 / 3.0, 100.0 / 3.0),
            (100.0, 100.0),
        );
        let true_arclen = 100.0 * (0.5 * 5.0f64.sqrt() + 0.25 * (2.0 + 5.0f64.sqrt()).ln());
        for i in 0..12 {
            let accuracy = 0.1f64.powi(i);
            let n = 10;
            for j in 0..=n {
                let arc = (j as f64) * ((n as f64).recip() * true_arclen);
                let t = c.inv_arclen(arc, accuracy * 0.5);
                let actual_arc = c.subsegment(0.0..t).arclen(accuracy * 0.5);
                assert!(
                    (arc - actual_arc).abs() < accuracy,
                    "at accuracy {accuracy:e}, wanted {actual_arc} got {arc}"
                );
            }
        }
        // corner case: user passes accuracy larger than total arc length
        let accuracy = true_arclen * 1.1;
        let arc = true_arclen * 0.5;
        let t = c.inv_arclen(arc, accuracy);
        let actual_arc = c.subsegment(0.0..t).arclen(accuracy);
        assert!(
            (arc - actual_arc).abs() < 2.0 * accuracy,
            "at accuracy {accuracy:e}, want {actual_arc} got {arc}"
        );
    }

    #[test]
    fn cubicbez_inv_arclen_accuracy() {
        let c = CubicBez::new((0.2, 0.73), (0.35, 1.08), (0.85, 1.08), (1.0, 0.73));
        let true_t = c.inv_arclen(0.5, 1e-12);
        for i in 1..12 {
            let accuracy = (0.1f64).powi(i);
            let approx_t = c.inv_arclen(0.5, accuracy);
            assert!((approx_t - true_t).abs() <= accuracy);
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn cubicbez_signed_area_linear() {
        // y = 1 - x
        let c = CubicBez::new(
            (1.0, 0.0),
            (2.0 / 3.0, 1.0 / 3.0),
            (1.0 / 3.0, 2.0 / 3.0),
            (0.0, 1.0),
        );
        let epsilon = 1e-12;
        assert_eq!((Affine::rotate(0.5) * c).signed_area(), 0.5);
        assert!(((Affine::rotate(0.5) * c).signed_area() - 0.5).abs() < epsilon);
        assert!(((Affine::translate((0.0, 1.0)) * c).signed_area() - 1.0).abs() < epsilon);
        assert!(((Affine::translate((1.0, 0.0)) * c).signed_area() - 1.0).abs() < epsilon);
    }

    #[test]
    fn cubicbez_signed_area() {
        // y = 1 - x^3
        let c = CubicBez::new((1.0, 0.0), (2.0 / 3.0, 1.0), (1.0 / 3.0, 1.0), (0.0, 1.0));
        let epsilon = 1e-12;
        assert!((c.signed_area() - 0.75).abs() < epsilon);
        assert!(((Affine::rotate(0.5) * c).signed_area() - 0.75).abs() < epsilon);
        assert!(((Affine::translate((0.0, 1.0)) * c).signed_area() - 1.25).abs() < epsilon);
        assert!(((Affine::translate((1.0, 0.0)) * c).signed_area() - 1.25).abs() < epsilon);
    }

    #[test]
    fn cubicbez_nearest() {
        fn verify(result: Nearest, expected: f64) {
            assert!(
                (result.t - expected).abs() < 1e-6,
                "got {result:?} expected {expected}"
            );
        }
        // y = x^3
        let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 0.0), (1.0, 1.0));
        verify(c.nearest((0.1, 0.001).into(), 1e-6), 0.1);
        verify(c.nearest((0.2, 0.008).into(), 1e-6), 0.2);
        verify(c.nearest((0.3, 0.027).into(), 1e-6), 0.3);
        verify(c.nearest((0.4, 0.064).into(), 1e-6), 0.4);
        verify(c.nearest((0.5, 0.125).into(), 1e-6), 0.5);
        verify(c.nearest((0.6, 0.216).into(), 1e-6), 0.6);
        verify(c.nearest((0.7, 0.343).into(), 1e-6), 0.7);
        verify(c.nearest((0.8, 0.512).into(), 1e-6), 0.8);
        verify(c.nearest((0.9, 0.729).into(), 1e-6), 0.9);
        verify(c.nearest((1.0, 1.0).into(), 1e-6), 1.0);
        verify(c.nearest((1.1, 1.1).into(), 1e-6), 1.0);
        verify(c.nearest((-0.1, 0.0).into(), 1e-6), 0.0);
        let a = Affine::rotate(0.5);
        verify((a * c).nearest(a * Point::new(0.1, 0.001), 1e-6), 0.1);

        // Here's a case that tripped up the old solver because the start is close to
        // degenerate and the end is actually degenerate; see #446.
        let curve = CubicBez::new(
            (461.0, 123.0),
            (460.99999999999994, 123.00000000000004),
            (111.0, 319.0),
            (111.0, 319.0),
        );
        let p = Point::new(282.0379003395483, 223.21877580985594);
        let eps = 0.0005;
        let nearest = curve.nearest(p, eps);
        let q = curve.eval(nearest.t);
        let r = curve.eval(0.5075474297354187);

        assert!((q - p).hypot() <= (r - p).hypot() + eps);
    }

    // ensure to_quads returns something given collinear points
    #[test]
    fn degenerate_to_quads() {
        let c = CubicBez::new((0., 9.), (6., 6.), (12., 3.0), (18., 0.0));
        let quads = c.to_quads(1e-6).collect::<Vec<_>>();
        assert_eq!(quads.len(), 1, "{:?}", quads);
    }

    #[test]
    fn cubicbez_extrema() {
        // y = x^2
        let q = CubicBez::new((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0));
        let extrema = q.extrema();
        assert_eq!(extrema.len(), 1);
        assert!((extrema[0] - 0.5).abs() < 1e-6);

        let q = CubicBez::new((0.4, 0.5), (0.0, 1.0), (1.0, 0.0), (0.5, 0.4));
        let extrema = q.extrema();
        assert_eq!(extrema.len(), 4);
    }

    #[test]
    fn cubicbez_toquads() {
        // y = x^3
        let c = CubicBez::new((0.0, 0.0), (1.0 / 3.0, 0.0), (2.0 / 3.0, 0.0), (1.0, 1.0));
        for i in 0..10 {
            let accuracy = 0.1f64.powi(i);
            let mut worst: f64 = 0.0;
            for (t0, t1, q) in c.to_quads(accuracy) {
                let epsilon = 1e-12;
                assert!((q.start() - c.eval(t0)).hypot() < epsilon);
                assert!((q.end() - c.eval(t1)).hypot() < epsilon);
                let n = 4;
                for j in 0..=n {
                    let t = (j as f64) * (n as f64).recip();
                    let p = q.eval(t);
                    let err = (p.y - p.x.powi(3)).abs();
                    worst = worst.max(err);
                    assert!(err < accuracy, "got {err} wanted {accuracy}");
                }
            }
        }
    }

    #[test]
    fn cubicbez_approx_spline() {
        let c1 = CubicBez::new(
            (550.0, 258.0),
            (1044.0, 482.0),
            (2029.0, 1841.0),
            (1934.0, 1554.0),
        );

        let quad = c1.try_approx_quadratic(344.0);
        let expected = QuadBez::new(
            Point::new(550.0, 258.0),
            Point::new(1673.665720592873, 767.5164401068898),
            Point::new(1934.0, 1554.0),
        );
        assert!(quad.is_some());
        assert_eq!(quad.unwrap(), expected);

        let quad = c1.try_approx_quadratic(343.0);
        assert!(quad.is_none());

        let spline = c1.approx_spline_n(2, 343.0);
        assert!(spline.is_some());
        let spline = spline.unwrap();
        let expected = [
            Point::new(550.0, 258.0),
            Point::new(920.5, 426.0),
            Point::new(2005.25, 1769.25),
            Point::new(1934.0, 1554.0),
        ];
        assert_eq!(spline.points().len(), expected.len());
        for (got, &wanted) in spline.points().iter().zip(expected.iter()) {
            assert!(got.distance(wanted) < 5.0);
        }

        let spline = c1.approx_spline(5.0);
        let expected = [
            Point::new(550.0, 258.0),
            Point::new(673.5, 314.0),
            Point::new(984.8777777777776, 584.2666666666667),
            Point::new(1312.6305555555557, 927.825),
            Point::new(1613.1194444444443, 1267.425),
            Point::new(1842.7055555555555, 1525.8166666666666),
            Point::new(1957.75, 1625.75),
            Point::new(1934.0, 1554.0),
        ];
        assert!(spline.is_some());
        let spline = spline.unwrap();
        assert_eq!(spline.points().len(), expected.len());
        for (got, &wanted) in spline.points().iter().zip(expected.iter()) {
            assert!(got.distance(wanted) < 5.0);
        }
    }

    #[test]
    fn cubicbez_cubics_to_quadratic_splines() {
        let curves = vec![
            CubicBez::new(
                (550.0, 258.0),
                (1044.0, 482.0),
                (2029.0, 1841.0),
                (1934.0, 1554.0),
            ),
            CubicBez::new(
                (859.0, 384.0),
                (1998.0, 116.0),
                (1596.0, 1772.0),
                (8.0, 1824.0),
            ),
            CubicBez::new(
                (1090.0, 937.0),
                (418.0, 1300.0),
                (125.0, 91.0),
                (104.0, 37.0),
            ),
        ];
        let converted = cubics_to_quadratic_splines(&curves, 5.0);
        assert!(converted.is_some());
        let converted = converted.unwrap();
        assert_eq!(converted[0].points().len(), 8);
        assert_eq!(converted[1].points().len(), 8);
        assert_eq!(converted[2].points().len(), 8);
        assert!(converted[0].points()[1].distance(Point::new(673.5, 314.0)) < 0.0001);
        assert!(
            converted[0].points()[2].distance(Point::new(88639.0 / 90.0, 52584.0 / 90.0)) < 0.0001
        );
    }

    #[test]
    fn cubicbez_approx_spline_div_exact() {
        // Ensure rounding behavior for division matches fonttools
        // cu2qu.
        // See <https://github.com/linebender/kurbo/issues/272>
        let cubic = CubicBez::new(
            Point::new(408.0, 321.0),
            Point::new(408.0, 452.0),
            Point::new(342.0, 560.0),
            Point::new(260.0, 560.0),
        );
        let spline = cubic.approx_spline(1.0).unwrap();
        assert_eq!(
            spline.points(),
            &[
                Point::new(408.0, 321.0),
                // Previous behavior produced 386.49999999999994 for the
                // y coordinate leading to inconsistent rounding.
                Point::new(408.0, 386.5),
                Point::new(368.16666666666663, 495.0833333333333),
                Point::new(301.0, 560.0),
                Point::new(260.0, 560.0)
            ]
        );
    }

    #[test]
    fn cubicbez_inflections() {
        let c = CubicBez::new((0., 0.), (0.8, 1.), (0.2, 1.), (1., 0.));
        let inflections = c.inflections();
        assert_eq!(inflections.len(), 2);
        assert!((inflections[0] - 0.311018).abs() < 1e-6);
        assert!((inflections[1] - 0.688982).abs() < 1e-6);
        let c = CubicBez::new((0., 0.), (1., 1.), (2., -1.), (3., 0.));
        let inflections = c.inflections();
        assert_eq!(inflections.len(), 1);
        assert!((inflections[0] - 0.5).abs() < 1e-6);
        let c = CubicBez::new((0., 0.), (1., 1.), (2., 1.), (3., 0.));
        let inflections = c.inflections();
        assert_eq!(inflections.len(), 0);
    }

    #[test]
    fn cubic_to_quadratic_matches_python() {
        // from https://github.com/googlefonts/fontmake-rs/issues/217
        let cubic = CubicBez {
            p0: (796.0, 319.0).into(),
            p1: (727.0, 314.0).into(),
            p2: (242.0, 303.0).into(),
            p3: (106.0, 303.0).into(),
        };

        // FontTools can approximate this curve successfully in 7 splits, we can too
        assert!(cubic.approx_spline_n(7, 1.0).is_some());

        // FontTools can solve this with accuracy 0.001, we can too
        assert!(cubics_to_quadratic_splines(&[cubic], 0.001).is_some());
    }

    #[test]
    fn cubic_to_quadratic_all_points_equal() {
        let pt = Point::new(5.0, 5.0);
        let cubic = CubicBez::new(pt, pt, pt, pt);
        let quads = cubics_to_quadratic_splines(&[cubic], 0.1).unwrap();
        assert_eq!(quads, [QuadSpline::new(vec![pt, pt, pt])]);
    }

    #[test]
    fn cubic_to_quadratic_3_points_equal_single_quad_within_tolerance() {
        let p0 = Point::new(5.0, 5.0);
        let p1 = Point::new(5.0, 5.1);
        let cubic = CubicBez::new(p0, p0, p0, p1);
        let quads = cubics_to_quadratic_splines(&[cubic], 0.1).unwrap();
        // a single quadratic bezier approximates this cubic for the given tolerance
        assert_eq!(quads, [QuadSpline::new(vec![p0, p0, p1])]);
    }

    #[test]
    fn slow_points_finds_every_slow_minimum() {
        // Compare with the minima of the speed found by sampling.
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1_u64 << 53) as f64
        };
        let mut n_slow = 0;
        for i in 0..1000 {
            let mut point = || Point::new(next() * 100.0, next() * 100.0);
            let mut c = CubicBez::new(point(), point(), point(), point());
            match i % 4 {
                // A near cusp at a random parameter, from the hodograph
                // `(t - t_cusp)(a + b t)` plus a small constant.
                1 => {
                    let t_cusp = 0.05 + 0.9 * next();
                    let a = c.p1.to_vec2() * 3.0 - Vec2::new(150.0, 150.0);
                    let b = c.p2.to_vec2() * 3.0 - Vec2::new(150.0, 150.0);
                    let e = Vec2::new(next() - 0.5, next() - 0.5) * 10.0_f64.powf(-6.0 * next());
                    let (pow2, pow1, pow0) = (b, a - b * t_cusp, e - a * t_cusp);
                    let p1 = c.p0 + pow0 / 3.0;
                    let p2 = p1 + (pow0 + 0.5 * pow1) / 3.0;
                    c = CubicBez::new(c.p0, p1, p2, p2 + (pow0 + pow1 + pow2) / 3.0);
                }
                // A zero-length handle at one end.
                2 => c.p1 = c.p0,
                3 => c.p2 = c.p3,
                _ => {}
            }
            let q = c.deriv();
            let scale = q.p0.to_vec2().hypot2().max(q.p2.to_vec2().hypot2());
            const N: usize = 20_000;
            let speed_squared: Vec<f64> = (0..=N)
                .map(|j| q.eval(j as f64 / N as f64).to_vec2().hypot2())
                .collect();
            // The sampled minima, as parameters with the squared speed there.
            let minima: Vec<(f64, f64)> = (1..N)
                .filter(|&j| {
                    speed_squared[j] <= speed_squared[j - 1]
                        && speed_squared[j] < speed_squared[j + 1]
                })
                .map(|j| (j as f64 / N as f64, speed_squared[j]))
                .collect();
            for limit in [1e-6 * scale, 1e-3 * scale, 0.1 * scale] {
                let found = c.slow_points(limit);
                for &(t, here) in &minima {
                    if here <= limit {
                        n_slow += 1;
                        assert!(
                            found.iter().any(|&u| (u - t).abs() < 2.0 / N as f64),
                            "minimum near {t} under {limit} not in {found:?} for {c:?}"
                        );
                    }
                }
            }
            // The cheap test must pass every curve with a slow minimum.
            let c2 = q
                .deriv()
                .p0
                .to_vec2()
                .hypot()
                .max(q.deriv().p1.to_vec2().hypot());
            for (max_width, dimension) in [(0.01, 0.0), (0.0, 0.01), (0.1, 1e-4)] {
                if c.may_have_slow_point(max_width, dimension) {
                    continue;
                }
                for &(t, here) in &minima {
                    let slow = here.sqrt() < max_width * c2
                        || (here <= c2 * dimension && here.sqrt() < CUSP_MAX_WIDTH * c2);
                    assert!(
                        !slow,
                        "slow minimum near {t} missed, width {max_width}, dimension {dimension}, for {c:?}"
                    );
                }
            }
        }
        assert!(n_slow > 1000, "{n_slow}");
    }

    #[test]
    fn regularize_exact_cusp() {
        // The derivative vanishes at the midpoint (#615), to rounding, and
        // the rounding left it along the second derivative, so the curvature
        // there computed as zero and the cusp was not detected.
        let c = CubicBez::new(
            (1.600_000_000_000_000_8, 310.787_665_198_237_9),
            (5.406_167_400_881_057, 306.981_497_797_356_84),
            (5.406_167_400_881_058, 310.787_665_198_237_9),
            (1.6, 306.981_497_797_356_84),
        );
        for dimension in [5e-3, 2.5e-5] {
            let r = c.regularize_cusp(dimension);
            assert_ne!(r, c, "dimension {dimension}");
            let moved = (r.p1 - c.p1).hypot().max((r.p2 - c.p2).hypot());
            assert!(moved <= 1.000_001 * dimension, "moved {moved}");
        }
    }

    #[test]
    fn cubic_to_quadratic_3_points_equal_exceeding_tolerance() {
        let p0 = Point::new(5.0, 5.0);
        let p1 = Point::new(5.0, 5.1);
        let cubic = CubicBez::new(p0, p0, p0, p1);
        let quads = cubics_to_quadratic_splines(&[cubic], 0.01).unwrap();
        // 2 quadratic off-curves are required to approximate the same cubic
        // given the smaller tolerance
        assert_eq!(
            quads,
            [QuadSpline::new(vec![p0, p0, (5.0, 5.025).into(), p1])]
        );
    }

    #[test]
    fn cubics_to_quadratic_splines_matches_python() {
        // https://github.com/linebender/kurbo/pull/273
        let light = CubicBez::new((378., 608.), (378., 524.), (355., 455.), (266., 455.));
        let regular = CubicBez::new((367., 607.), (367., 511.), (338., 472.), (243., 472.));
        let bold = CubicBez::new(
            (372.425, 593.05),
            (372.425, 524.95),
            (355.05, 485.95),
            (274., 485.95),
        );
        let qsplines = cubics_to_quadratic_splines(&[light, regular, bold], 1.0).unwrap();
        assert_eq!(
            qsplines,
            [
                QuadSpline::new(vec![
                    (378.0, 608.0).into(),
                    (378.0, 566.0).into(),
                    (359.0833333333333, 496.5).into(),
                    (310.5, 455.0).into(),
                    (266.0, 455.0).into(),
                ]),
                QuadSpline::new(vec![
                    (367.0, 607.0).into(),
                    (367.0, 559.0).into(),
                    // Previous behavior produced 496.5 for the y coordinate
                    (344.5833333333333, 499.49999999999994).into(),
                    (290.5, 472.0).into(),
                    (243.0, 472.0).into(),
                ]),
                QuadSpline::new(vec![
                    (372.425, 593.05).into(),
                    (372.425, 559.0).into(),
                    (356.98333333333335, 511.125).into(),
                    (314.525, 485.95).into(),
                    (274.0, 485.95).into(),
                ]),
            ]
        );
    }
}
