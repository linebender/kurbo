//! Cubic Bézier segments.

use std::ops::{Mul, Range};

use crate::MAX_EXTREMA;
use arrayvec::ArrayVec;

use crate::common::solve_quadratic;
use crate::common::GAUSS_LEGENDRE_COEFFS_9;
use crate::{
    Affine, Nearest, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveCurvature,
    ParamCurveDeriv, ParamCurveExtrema, ParamCurveNearest, PathEl, Point, QuadBez, Rect, Shape,
};

/// A single cubic Bézier segment.
#[derive(Clone, Copy, Debug, PartialEq)]
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

impl CubicBez {
    /// Create a new cubic Bézier segment.
    #[inline]
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
    pub fn to_quads(self, accuracy: f64) -> impl Iterator<Item = (f64, f64, QuadBez)> {
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

        ToQuads { c: self, n, i: 0 }
    }


    fn split_into_n(&self, n: usize) -> Vec<CubicBez> {
        match n {
            1 => {
                vec![*self]
            }
            2 => {
                let (l, r) = self.subdivide();
                vec![l, r]
            }
            3 => {
                let (left, mid, right) = self.subdivide_3();
                vec![left, mid, right]
            }
            4 => {
                let (l, r) = self.subdivide();
                let (ll, lr) = l.subdivide();
                let (rl, rr) = r.subdivide();
                vec![ll, lr, rl, rr]
            }
            6 => {
                let (l, r) = self.subdivide();
                let (l1, l2, l3) = l.subdivide_3();
                let (r1, r2, r3) = r.subdivide_3();
                vec![l1, l2, l3, r1, r2, r3]
            }
            _ => self._split_into_n_generic(n),
        }
    }

    fn _split_into_n_generic(&self, n: usize) -> Vec<CubicBez> {
        let mut r = vec![];
        let (a, b, c, d) = self.parameters();
        let dt = 1.0 / n as f64;
        let delta_2 = dt * dt;
        let delta_3 = dt * delta_2;
        for i in 0..n {
            let t1 = i as f64 * dt;
            let t1_2 = t1 * t1;
            let a1 = a * delta_3;
            let b1 = (3.0 * a * t1 + b) * delta_2;
            let c1 = (2.0 * b * t1 + c + 3.0 * a * t1_2) * dt;
            let d1 = a * t1 * t1_2 + b * t1_2 + c * t1 + d;

            r.push(CubicBez::from_parameters(a1, b1, c1, d1))
        }
        r
    }

    fn parameters(&self) -> (Vec2, Vec2, Vec2, Vec2) {
        let c = (self.p1 - self.p0) * 3.0;
        let b = (self.p2 - self.p1) * 3.0 - c;
        let d = self.p0.to_vec2();
        let a = self.p3.to_vec2() - d - c - b;
        (a, b, c, d)
    }

    fn from_parameters(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Self {
        CubicBez::new(
            d.to_point(),
            d.to_point() + (c / 3.0),
            d.to_point() + (c / 3.0) + (b + c) / 3.0,
            d.to_point() + (c / 3.0) + (b + c) / 3.0 + a,
        )
    }

    fn subdivide_3(&self) -> (CubicBez, CubicBez, CubicBez) {
        let (p0, p1, p2, p3) = (
            self.p0.to_vec2(),
            self.p1.to_vec2(),
            self.p2.to_vec2(),
            self.p3.to_vec2(),
        );
        let mid1 = ((8.0 * p0 + 12.0 * p1 + 6.0 * p2 + p3) / 27.0).to_point();
        let deriv1 = (p3 + 3.0 * p2 - 4.0 * p0) / 27.0;
        let mid2 = ((p0 + 6.0 * p1 + 12.0 * p2 + 8.0 * p3) / 27.0).to_point();
        let deriv2 = (4.0 * p3 - 3.0 * p1 - p0) / 27.0;
        let left = CubicBez::new(
            self.p0,
            ((2.0 * p0 + p1) / 3.0).to_point(),
            mid1 - deriv1,
            mid1,
        );
        let mid = CubicBez::new(mid1, mid1 + deriv1, mid2 - deriv2, mid2);
        let right = CubicBez::new(
            mid2,
            mid2 + deriv2,
            ((p2 + 2.0 * p3) / 3.0).to_point(),
            self.p3,
        );
        (left, mid, right)
    }

    /// Does this curve fit inside the given distance from the origin?
    pub fn fit_inside(&self, distance: f64) -> bool {
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
    pub fn is_finite(&self) -> bool {
        self.p0.is_finite() && self.p1.is_finite() && self.p2.is_finite() && self.p3.is_finite()
    }

    /// Is this cubic Bezier curve NaN?
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.p0.is_nan() || self.p1.is_nan() || self.p2.is_nan() || self.p3.is_nan()
    }
}

/// An iterator for cubic beziers.
pub struct CubicBezIter {
    cubic: CubicBez,
    ix: usize,
}

impl Shape for CubicBez {
    type PathElementsIter = CubicBezIter;

    #[inline]
    fn path_elements(&self, _tolerance: f64) -> CubicBezIter {
        CubicBezIter {
            cubic: *self,
            ix: 0,
        }
    }

    fn area(&self) -> f64 {
        0.0
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        self.arclen(accuracy)
    }

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

    #[inline]
    fn start(&self) -> Point {
        self.p0
    }

    #[inline]
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

impl ParamCurveArclen for CubicBez {
    /// Arclength of a cubic Bézier segment.
    ///
    /// This is an adaptive subdivision approach using Legendre-Gauss quadrature
    /// in the base case, and an error estimate to decide when to subdivide.
    fn arclen(&self, accuracy: f64) -> f64 {
        // Squared L2 norm of the second derivative of the cubic.
        fn cubic_errnorm(c: &CubicBez) -> f64 {
            let d = c.deriv().deriv();
            let dd = d.end() - d.start();
            d.start().to_vec2().hypot2() + d.start().to_vec2().dot(dd) + dd.hypot2() * (1.0 / 3.0)
        }
        fn est_gauss9_error(c: &CubicBez) -> f64 {
            let lc2 = (c.p3 - c.p0).hypot2();
            let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

            2.56e-8 * (cubic_errnorm(c) / lc2).powi(8) * lp
        }
        const MAX_DEPTH: usize = 16;
        fn rec(c: &CubicBez, accuracy: f64, depth: usize) -> f64 {
            if depth == MAX_DEPTH || est_gauss9_error(c) < accuracy {
                c.gauss_arclen(GAUSS_LEGENDRE_COEFFS_9)
            } else {
                let (c0, c1) = c.subdivide();
                rec(&c0, accuracy * 0.5, depth + 1) + rec(&c1, accuracy * 0.5, depth + 1)
            }
        }

        // Check if the bezier curve is degenerate, or almost degenerate
        // A degenerate curve where all points are identical will cause infinite recursion in the rec function (well, until MAX_DEPTH at least) in all branches.
        // This test will in addition be true if the bezier curve is just a simple line (i.e. p0=p1 and p2=p3).
        // The constant 0.5 has not been mathematically proven to be small enough, but from empirical tests
        // a value of about 0.87 should be enough. Thus 0.5 is a conservative value.
        // See https://github.com/linebender/kurbo/pull/100 for more info.
        if (self.p1 - self.p0).hypot2() + (self.p2 - self.p3).hypot2() <= 0.5 * accuracy * accuracy
        {
            (self.p0 - self.p3).hypot()
        } else {
            rec(self, accuracy, 0)
        }
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
    /// Find the nearest point, using subdivision.
    fn nearest(&self, p: Point, accuracy: f64) -> Nearest {
        let mut best_r = None;
        let mut best_t = 0.0;
        for (t0, t1, q) in self.to_quads(accuracy) {
            let nearest = q.nearest(p, accuracy);
            if best_r
                .map(|best_r| nearest.distance_sq < best_r)
                .unwrap_or(true)
            {
                best_t = t0 + nearest.t * (t1 - t0);
                best_r = Some(nearest.distance_sq);
            }
        }
        Nearest {
            t: best_t,
            distance_sq: best_r.unwrap(),
        }
    }
}

impl ParamCurveCurvature for CubicBez {}

impl ParamCurveExtrema for CubicBez {
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]> {
        fn one_coord(result: &mut ArrayVec<[f64; MAX_EXTREMA]>, d0: f64, d1: f64, d2: f64) {
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

#[cfg(test)]
mod tests {
    use crate::{
        Affine, CubicBez, Nearest, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveDeriv,
        ParamCurveExtrema, ParamCurveNearest, Point,
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
                    "at accuracy {:e}, wanted {} got {}",
                    accuracy,
                    actual_arc,
                    arc
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
            "at accuracy {:e}, want {} got {}",
            accuracy,
            actual_arc,
            arc
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
                "got {:?} expected {}",
                result,
                expected
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
    }

    // ensure to_quads returns something given colinear points
    #[test]
    fn degenerate_to_quads() {
        let c = CubicBez::new((0., 9.), (6., 6.), (12., 3.0), (18., 0.0));
        let quads = c.to_quads(1e-6).collect::<Vec<_>>();
        assert_eq!(quads.len(), 1, "{:?}", &quads);
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
            for (_count, (t0, t1, q)) in c.to_quads(accuracy).enumerate() {
                let epsilon = 1e-12;
                assert!((q.start() - c.eval(t0)).hypot() < epsilon);
                assert!((q.end() - c.eval(t1)).hypot() < epsilon);
                let n = 4;
                for j in 0..=n {
                    let t = (j as f64) * (n as f64).recip();
                    let p = q.eval(t);
                    let err = (p.y - p.x.powi(3)).abs();
                    worst = worst.max(err);
                    assert!(err < accuracy, "got {} wanted {}", err, accuracy);
                }
            }
        }
    }
}
