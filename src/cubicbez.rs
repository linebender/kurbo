//! Cubic Bézier segments.

use std::ops::{Mul, Range};

use arrayvec::ArrayVec;
use crate::MAX_EXTREMA;

use crate::{Affine, ParamCurve, ParamCurveArea, ParamCurveArclen, ParamCurveCurvature,
    ParamCurveDeriv, ParamCurveExtrema, ParamCurveNearest, QuadBez, Vec2};
use crate::common::solve_quadratic;

/// A single cubic Bézier segment.
#[derive(Clone, Copy, Debug)]
pub struct CubicBez {
    pub p0: Vec2,
    pub p1: Vec2,
    pub p2: Vec2,
    pub p3: Vec2,
}

/// An iterator which produces quadratic Bézier segments.
struct ToQuads {
    c: CubicBez,
    max_hypot2: f64,
    t: f64,
}

impl CubicBez {
    /// Create a new cubic Bézier segment.
    #[inline]
    pub fn new<V: Into<Vec2>>(p0: V, p1: V, p2: V, p3: V) -> CubicBez {
        CubicBez { p0: p0.into(), p1: p1.into(), p2: p2.into(), p3: p3.into() }
    }

    /// Convert to quadratic Béziers.
    ///
    /// The iterator returns the start and end parameter in the cubic of each quadratic
    /// segment, along with the quadratic.
    ///
    /// Note that the resulting quadratic Béziers are not in general G1 continuous;
    /// they are optimized for minimizing distance error.
    #[inline]
    pub fn to_quads(&self, accuracy: f64) -> impl Iterator<Item = (f64, f64, QuadBez)> {
        // This magic number is the square of 36 / sqrt(3).
        // See: http://caffeineowl.com/graphics/2d/vectorial/cubic2quad01.html
        let max_hypot2 = 432.0 * accuracy * accuracy;
        ToQuads { c: *self, max_hypot2, t: 0.0 }
    }
}

impl ParamCurve for CubicBez {
    #[inline]
    fn eval(&self, t: f64) -> Vec2 {
        let mt = 1.0 - t;
        self.p0 * (mt * mt * mt)
            + (self.p1 * (mt * mt * 3.0) + (self.p2 * (mt * 3.0) + self.p3 * t) * t) * t
    }

    #[inline]
    fn start(&self) -> Vec2 {
        self.p0
    }

    #[inline]
    fn end(&self) -> Vec2 {
        self.p3
    }

    fn subsegment(&self, range: Range<f64>) -> CubicBez {
        let (t0, t1) = (range.start, range.end);
        let p0 = self.eval(t0);
        let p3 = self.eval(t1);
        let d = self.deriv();
        let scale = (t1 - t0) * (1.0 / 3.0);
        let p1 = p0 + scale * d.eval(t0);
        let p2 = p3 - scale * d.eval(t1);
        CubicBez { p0, p1, p2, p3 }
    }

    /// Subdivide into halves, using de Casteljau.
    #[inline]
    fn subdivide(&self) -> (CubicBez, CubicBez) {
        let pm = self.eval(0.5);
        (
            CubicBez::new(self.p0,
                (self.p0 + self.p1) / 2.0,
                (self.p0 + self.p1 * 2.0 + self.p2) * 0.25,
                pm),
            CubicBez::new(pm,
                (self.p1 + self.p2 * 2.0 + self.p3) * 0.25,
                (self.p2 + self.p3) / 2.0,
                self.p3)
        )
    }
}

impl ParamCurveDeriv for CubicBez {
    type DerivResult = QuadBez;

    #[inline]
    fn deriv(&self) -> QuadBez {
        QuadBez::new(
            3.0 * (self.p1 - self.p0),
            3.0 * (self.p2 - self.p1),
            3.0 * (self.p3 - self.p2)
        )
    }
}

impl ParamCurveArclen for CubicBez {
    /// Arclength of a cubic Bézier segment.
    ///
    /// This algorithm is based on "Adaptive subdivision and the length and
    /// energy of Bézier curves" by Jens Gravesen.
    fn arclen(&self, accuracy: f64) -> f64 {
        // Estimate for a single segment.
        fn calc_l0(c: &CubicBez) -> f64 {
            let lc = (c.p3 - c.p0).hypot();
            let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();
            (lc + lp) * 0.5
        }
        const MAX_DEPTH: usize = 16;
        fn rec(c: &CubicBez, l0: f64, accuracy: f64, depth: usize) -> f64 {
            let (c0, c1) = c.subdivide();
            let l0_c0 = calc_l0(&c0);
            let l0_c1 = calc_l0(&c1);
            let l1 = l0_c0 + l0_c1;
            let error = (l0 - l1) * (1.0 / 15.0);
            if error.abs() < accuracy || depth == MAX_DEPTH {
                l1 - error
            } else {
                rec(&c0, l0_c0, accuracy * 0.5, depth + 1)
                    + rec(&c1, l0_c1, accuracy * 0.5, depth + 1)
            }
        }
        rec(self, calc_l0(self), accuracy, 0)
    }
}

impl ParamCurveArea for CubicBez {
    #[inline]
    fn signed_area(&self) -> f64 {
        (self.p0.x * (6.0 * self.p1.y + 3.0 * self.p2.y + self.p3.y)
            + 3.0 * (self.p1.x * (-2.0 * self.p0.y + self.p2.y + self.p3.y)
                - self.p2.x * (self.p0.y + self.p1.y - 2.0 * self.p3.y))
            - self.p3.x * (self.p0.y + 3.0 * self.p1.y + 6.0 * self.p2.y)) * (1.0 / 20.0)
    }
}

impl ParamCurveNearest for CubicBez {
    /// Find nearest point, using subdivision.
    fn nearest(&self, p: Vec2, accuracy: f64) -> (f64, f64) {
        let mut best_r = None;
        let mut best_t = 0.0;
        for (t0, t1, q) in self.to_quads(accuracy) {
            let (t, r) = q.nearest(p, accuracy);
            if best_r.map(|best_r| r < best_r).unwrap_or(true) {
                best_t = t0 + t * (t1 - t0);
                best_r = Some(r);
            }
        }
        (best_t, best_r.unwrap())
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
        CubicBez { p0: self * c.p0, p1: self * c.p1, p2: self * c.p2, p3: self * c.p3 }
    }
}

impl Iterator for ToQuads {
    type Item = (f64, f64, QuadBez);

    fn next(&mut self) -> Option<(f64, f64, QuadBez)> {
        let t0 = self.t;
        let mut t1 = 1.0;
        if t0 == t1 { return None; }
        loop {
            let seg = self.c.subsegment(t0..t1);
            // Compute error for candidate quadratic.
            let p1x2 = 3.0 * seg.p1 - seg.p0;
            let p2x2 = 3.0 * seg.p2 - seg.p3;
            let err = (p2x2 - p1x2).hypot2();
            //println!("{:?} {} {}", t0..t1, err, if err < self.max_hypot2 { "ok" } else { "" });
            if err < self.max_hypot2 {
                let result = QuadBez::new(seg.p0, (p1x2 + p2x2) / 4.0, seg.p3);
                self.t = t1;
                return Some((t0, t1, result));
            } else {
                let shrink = if t1 == 1.0 && err < 64.0 * self.max_hypot2 {
                    0.5
                } else {
                    0.999_999 * (self.max_hypot2 / err).powf(1./6.0)
                };
                t1 = t0 + shrink * (t1 - t0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Affine, CubicBez, ParamCurve, ParamCurveArea, ParamCurveArclen, ParamCurveDeriv,
        ParamCurveExtrema, ParamCurveNearest, Vec2};

    #[test]
    fn cubicbez_deriv() {
        // y = x^2
        let c = CubicBez::new((0.0, 0.0), (1.0/3.0, 0.0), (2.0/3.0, 1.0/3.0), (1.0, 1.0));
        let deriv = c.deriv();

        let n = 10;
        for i in 0..=n {
            let t = (i as f64) * (n as f64).recip();
            let delta = 1e-6;
            let p = c.eval(t);
            let p1 = c.eval(t + delta);
            let d_approx = (p1 - p) * delta.recip();
            let d = deriv.eval(t);
            assert!((d - d_approx).hypot() < delta * 2.0);
        }
    }

    #[test]
    fn cubicbez_arclen() {
        // y = x^2
        let c = CubicBez::new((0.0, 0.0), (1.0/3.0, 0.0), (2.0/3.0, 1.0/3.0), (1.0, 1.0));
        let true_arclen = 0.5 * 5.0f64.sqrt() + 0.25 * (2.0 + 5.0f64.sqrt()).ln();
        for i in 0..12 {
            let accuracy = 0.1f64.powi(i);
            let error = c.arclen(accuracy) - true_arclen;
            //println!("{:e}: {:e}", accuracy, error);
            assert!(error.abs() < accuracy);
        }
    }

    #[test]
    fn cubicbez_inv_arclen() {
        // y = x^2
        let c = CubicBez::new((0.0, 0.0), (1.0/3.0, 0.0), (2.0/3.0, 1.0/3.0), (1.0, 1.0));
        let true_arclen = 0.5 * 5.0f64.sqrt() + 0.25 * (2.0 + 5.0f64.sqrt()).ln();
        for i in 0..12 {
            let accuracy = 0.1f64.powi(i);
            let n = 10;
            for j in 0..=n {
                let arc = (j as f64) * ((n as f64).recip() * true_arclen);
                let t = c.inv_arclen(arc, accuracy * 0.5);
                let actual_arc = c.subsegment(0.0 .. t).arclen(accuracy * 0.5);
                assert!((arc - actual_arc).abs() < accuracy,
                    "at accuracy {:e}, wanted {} got {}", accuracy, actual_arc, arc);
            }
        }
    }

    #[test]
    fn cubicbez_signed_area_linear() {
        // y = 1 - x
        let c = CubicBez::new((1.0, 0.0), (2.0/3.0, 1.0/3.0), (1.0/3.0, 2.0/3.0), (0.0, 1.0));
        let epsilon = 1e-12;
        assert_eq!((Affine::rotate(0.5) * c).signed_area(), 0.5);
        assert!(((Affine::rotate(0.5) * c).signed_area() - 0.5).abs() < epsilon);
        assert!(((Affine::translate((0.0, 1.0)) * c).signed_area() - 1.0).abs() < epsilon);
        assert!(((Affine::translate((1.0, 0.0)) * c).signed_area() - 1.0).abs() < epsilon);
    }

    #[test]
    fn cubicbez_signed_area() {
        // y = 1 - x^3
        let c = CubicBez::new((1.0, 0.0), (2.0/3.0, 1.0), (1.0/3.0, 1.0), (0.0, 1.0));
        let epsilon = 1e-12;
        assert!((c.signed_area() - 0.75).abs() < epsilon);
        assert!(((Affine::rotate(0.5) * c).signed_area() - 0.75).abs() < epsilon);
        assert!(((Affine::translate((0.0, 1.0)) * c).signed_area() - 1.25).abs() < epsilon);
        assert!(((Affine::translate((1.0, 0.0)) * c).signed_area() - 1.25).abs() < epsilon);
    }

    #[test]
    fn cubicbez_nearest() {
        fn verify(result: (f64, f64), expected: f64) {
            assert!((result.0 - expected).abs() < 1e-6, "got {:?} expected {}", result, expected);
        }
        // y = x^3
        let c = CubicBez::new((0.0, 0.0), (1.0/3.0, 0.0), (2.0/3.0, 0.0), (1.0, 1.0));
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
        verify((a * c).nearest(a * Vec2::new(0.1, 0.001), 1e-6), 0.1);
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
        let c = CubicBez::new((0.0, 0.0), (1.0/3.0, 0.0), (2.0/3.0, 0.0), (1.0, 1.0));
        for i in 0..10 {
            let accuracy = 0.1f64.powi(i);
            let mut _count = 0;
            let mut worst: f64 = 0.0;
            for (t0, t1, q) in c.to_quads(accuracy) {
                _count += 1;
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
            //println!("accuracy {:e}: got {:e}, {} quads", accuracy, worst, _count);
        }
    }
}
