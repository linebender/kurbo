//! Quadratic Bézier segments.

use std::ops::{Mul, Range};

use arrayvec::ArrayVec;

use crate::{Affine, CubicBez, Line, ParamCurve, ParamCurveArea, ParamCurveArclen, ParamCurveCurvature,
    ParamCurveDeriv, ParamCurveExtrema, ParamCurveNearest, Vec2};
use crate::MAX_EXTREMA;
use crate::common::solve_cubic;

/// A single quadratic Bézier segment.
#[derive(Clone, Copy, Debug)]
pub struct QuadBez {
    pub p0: Vec2,
    pub p1: Vec2,
    pub p2: Vec2,
}

impl QuadBez {
    /// Create a new quadratic Bézier segment.
    pub fn new<V: Into<Vec2>>(p0: V, p1: V, p2: V) -> QuadBez {
        QuadBez { p0: p0.into(), p1: p1.into(), p2: p2.into() }
    }

    /// Raise the order by 1.
    ///
    /// Returns a cubic Bézier segment that exactly represents this quadratic.
    pub fn raise(&self) -> CubicBez {
        CubicBez::new(
            self.p0,
            self.p0 + (2.0/3.0) * (self.p1 - self.p0),
            self.p2 + (2.0/3.0) * (self.p1 - self.p2),
            self.p2)
    }
}

impl ParamCurve for QuadBez {
    fn eval(&self, t: f64) -> Vec2 {
        let mt = 1.0 - t;
        self.p0 * (mt * mt) + (self.p1 * (mt * 2.0) + self.p2 * t) * t
    }

    fn start(&self) -> Vec2 {
        self.p0
    }

    fn end(&self) -> Vec2 {
        self.p2
    }

    /// Subdivide into halves, using de Casteljau.
    fn subdivide(&self) -> (QuadBez, QuadBez) {
        let pm = self.eval(0.5);
        (QuadBez::new(self.p0, (self.p0 + self.p1) / 2.0, pm),
            QuadBez::new(pm, (self.p1 + self.p2) / 2.0, self.p2))
    }

    fn subsegment(&self, range: Range<f64>) -> QuadBez {
        let (t0, t1) = (range.start, range.end);
        let p0 = self.eval(t0);
        let p2 = self.eval(t1);
        let p1 = p0 + (self.p1 - self.p0).lerp(self.p2 - self.p1, t0) * (t1 - t0);
        QuadBez { p0, p1, p2 }
    }
}

impl ParamCurveDeriv for QuadBez {
    type DerivResult = Line;

    fn deriv(&self) -> Line {
        Line::new(2.0 * (self.p1 - self.p0), 2.0 * (self.p2 - self.p1))
    }
}

impl ParamCurveArclen for QuadBez {
    /// Arclength of a quadratic Bézier segment.
    ///
    /// This algorithm is based on "Adaptive subdivision and the length and
    /// energy of Bézier curves" by Jens Gravesen.
    fn arclen(&self, accuracy: f64) -> f64 {
        // Estimate for a single segment.
        fn calc_l0(q: &QuadBez) -> f64 {
            let lc = (q.p2 - q.p0).hypot();
            let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
            (2.0 * lc + lp) * (1.0 / 3.0)
        }
        fn rec(q: &QuadBez, l0: f64, accuracy: f64) -> f64 {
            let (q0, q1) = q.subdivide();
            let l0_q0 = calc_l0(&q0);
            let l0_q1 = calc_l0(&q1);
            let l1 = l0_q0 + l0_q1;
            let error = (l0 - l1) * (1.0 / 15.0);
            if error.abs() < accuracy {
                l1 - error
            } else {
                rec(&q0, l0_q0, accuracy * 0.5) + rec(&q1, l0_q1, accuracy * 0.5)
            }
        }
        rec(self, calc_l0(self), accuracy)
    }
}

impl ParamCurveArea for QuadBez {
    fn signed_area(&self) -> f64 {
        (self.p0.x * (2.0 * self.p1.y + self.p2.y)
            + 2.0 * self.p1.x * (self.p2.y - self.p0.y)
            - self.p2.x * (self.p0.y + 2.0 * self.p1.y)) * (1.0 / 6.0)
    }
}

impl ParamCurveNearest for QuadBez {
    /// Find nearest point, using analytical algorithm based on cubic root finding.
    fn nearest(&self, p: Vec2, _accuracy: f64) -> (f64, f64) {
        fn eval_t(p: Vec2, t_best: &mut f64, r_best: &mut Option<f64>, t: f64, p0: Vec2) {
            let r = (p0 - p).hypot2();
            if r_best.map(|r_best| r < r_best).unwrap_or(true) {
                *r_best = Some(r);
                *t_best = t;
            }
        }
        fn try_t(q: &QuadBez, p: Vec2, t_best: &mut f64, r_best: &mut Option<f64>, t: f64)
            -> bool
        {
            if t < 0.0 || t > 1.0 {
                return true;
            }
            eval_t(p, t_best, r_best, t, q.eval(t));
            false
        }
        let d0 = self.p1 - self.p0;
        let d1 = self.p0 + self.p2 - 2.0 * self.p1;
        let d = self.p0 - p;
        let c0 = d.dot(d0);
        let c1 = 2.0 * d0.hypot2() + d.dot(d1);
        let c2 = 3.0 * d1.dot(d0);
        let c3 = d1.hypot2();
        let roots = solve_cubic(c0, c1, c2, c3);
        let mut r_best = None;
        let mut t_best = 0.0;
        let mut need_ends = false;
        for &t in &roots {
            need_ends |= try_t(self, p, &mut t_best, &mut r_best, t);
        }
        if need_ends {
            eval_t(p, &mut t_best, &mut r_best, 0.0, self.p0);
            eval_t(p, &mut t_best, &mut r_best, 1.0, self.p2);
        }
        (t_best, r_best.unwrap())
    }
}

impl ParamCurveCurvature for QuadBez {}

impl ParamCurveExtrema for QuadBez {
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]> {
        let mut result = ArrayVec::new();
        let d0 = self.p1 - self.p0;
        let d1 = self.p2 - self.p1;
        let dd = d1 - d0;
        if dd.x != 0.0 {
            let t = -d0.x / dd.x;
            if t > 0.0 && t < 1.0 {
                result.push(t);
            }
        }
        if dd.y != 0.0 {
            let t = -d0.y / dd.y;
            if t > 0.0 && t < 1.0 {
                result.push(t);
                if result.len() == 2 && result[0] > t {
                    result.swap(0, 1);
                }
            }
        }
        result
    }
}

impl Mul<QuadBez> for Affine {
    type Output = QuadBez;

    fn mul(self, other: QuadBez) -> QuadBez {
        QuadBez { p0: self * other.p0, p1: self * other.p1, p2: self * other.p2 }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Affine, ParamCurve, ParamCurveArea, ParamCurveArclen, ParamCurveDeriv,
        ParamCurveExtrema, ParamCurveNearest, QuadBez, Vec2};

    fn assert_near(p0: Vec2, p1: Vec2, epsilon: f64) {
        assert!((p1 - p0).hypot() < epsilon, "{:?} != {:?}", p0, p1);
    }

    #[test]
    fn quadbez_deriv() {
        let q = QuadBez::new((0.0, 0.0), (0.0, 0.5), (1.0, 1.0));
        let deriv = q.deriv();

        let n = 10;
        for i in 0..=n {
            let t = (i as f64) * (n as f64).recip();
            let delta = 1e-6;
            let p = q.eval(t);
            let p1 = q.eval(t + delta);
            let d_approx = (p1 - p) * delta.recip();
            let d = deriv.eval(t);
            assert!((d - d_approx).hypot() < delta * 2.0);
        }
    }

    #[test]
    fn quadbez_arclen() {
        let q = QuadBez::new((0.0, 0.0), (0.0, 0.5), (1.0, 1.0));
        let true_arclen = 0.5 * 5.0f64.sqrt() + 0.25 * (2.0 + 5.0f64.sqrt()).ln();
        for i in 0..12 {
            let accuracy = 0.1f64.powi(i);
            let error = q.arclen(accuracy) - true_arclen;
            //println!("{:e}: {:e}", accuracy, error);
            assert!(error.abs() < accuracy);
        }
    }

    #[test]
    fn quadbez_subsegment() {
        let q = QuadBez::new((3.1, 4.1), (5.9, 2.6), (5.3, 5.8));
        let t0 = 0.1;
        let t1 = 0.8;
        let qs = q.subsegment(t0..t1);
        let epsilon = 1e-12;
        let n = 10;
        for i in 0..=n {
            let t = (i as f64) * (n as f64).recip();
            let ts = t0 + t * (t1 - t0);
            assert_near(q.eval(ts), qs.eval(t), epsilon);
        }
    }

    #[test]
    fn quadbez_raise() {
        let q = QuadBez::new((3.1, 4.1), (5.9, 2.6), (5.3, 5.8));
        let c = q.raise();
        let qd = q.deriv();
        let cd = c.deriv();
        let epsilon = 1e-12;
        let n = 10;
        for i in 0..=n {
            let t = (i as f64) * (n as f64).recip();
            assert_near(q.eval(t), c.eval(t), epsilon);
            assert_near(qd.eval(t), cd.eval(t), epsilon);
        }
    }

    #[test]
    fn quadbez_signed_area() {
        // y = 1 - x^2
        let q = QuadBez::new((1.0, 0.0), (0.5, 1.0), (0.0, 1.0));
        let epsilon = 1e-12;
        assert!((q.signed_area() - 2.0/3.0).abs() < epsilon);
        assert!(((Affine::rotate(0.5) * q).signed_area() - 2.0/3.0).abs() < epsilon);
        assert!(((Affine::translate((0.0, 1.0)) * q).signed_area() - 3.5/3.0).abs() < epsilon);
        assert!(((Affine::translate((1.0, 0.0)) * q).signed_area() - 3.5/3.0).abs() < epsilon);
    }

    #[test]
    fn quadbez_nearest() {
        fn verify(result: (f64, f64), expected: f64) {
            assert!((result.0 - expected).abs() < 1e-6, "got {:?} expected {}", result, expected);
        }
        // y = x^2
        let q = QuadBez::new((-1.0, 1.0), (0.0, -1.0), (1.0, 1.0));
        verify(q.nearest((0.0, 0.0).into(), 1e-3), 0.5);
        verify(q.nearest((0.0, 0.1).into(), 1e-3), 0.5);
        verify(q.nearest((0.0, -0.1).into(), 1e-3), 0.5);
        verify(q.nearest((0.5, 0.25).into(), 1e-3), 0.75);
        verify(q.nearest((1.0, 1.0).into(), 1e-3), 1.0);
        verify(q.nearest((1.1, 1.1).into(), 1e-3), 1.0);
        verify(q.nearest((-1.1, 1.1).into(), 1e-3), 0.0);
        let a = Affine::rotate(0.5);
        verify((a * q).nearest(a * Vec2::new(0.5, 0.25), 1e-3), 0.75);
    }

    #[test]
    fn quadbez_extrema() {
        // y = x^2
        let q = QuadBez::new((-1.0, 1.0), (0.0, -1.0), (1.0, 1.0));
        let extrema = q.extrema();
        assert_eq!(extrema.len(), 1);
        assert!((extrema[0] - 0.5).abs() < 1e-6);

        let q = QuadBez::new((0.0, 0.5), (1.0, 1.0), (0.5, 0.0));
        let extrema = q.extrema();
        assert_eq!(extrema.len(), 2);
        assert!((extrema[0] - 1.0 / 3.0).abs() < 1e-6);
        assert!((extrema[1] - 2.0 / 3.0).abs() < 1e-6);

        // Reverse direction
        let q = QuadBez::new((0.5, 0.0), (1.0, 1.0), (0.0, 0.5));
        let extrema = q.extrema();
        assert_eq!(extrema.len(), 2);
        assert!((extrema[0] - 1.0 / 3.0).abs() < 1e-6);
        assert!((extrema[1] - 2.0 / 3.0).abs() < 1e-6);
    }
}
