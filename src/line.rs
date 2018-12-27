//! Lines.

use std::ops::{Mul, Range};

use arrayvec::ArrayVec;

use crate::{Affine, ParamCurve, ParamCurveArea, ParamCurveArclen, ParamCurveCurvature,
    ParamCurveDeriv, ParamCurveExtrema, ParamCurveNearest, Vec2};
use crate::MAX_EXTREMA;

/// A single line.
#[derive(Clone, Copy)]
pub struct Line {
    pub p0: Vec2,
    pub p1: Vec2,
}

impl Line {
    pub fn new<V: Into<Vec2>>(p0: V, p1: V) -> Line {
        Line { p0: p0.into(), p1: p1.into() }
    }
}

impl ParamCurve for Line {
    fn eval(&self, t: f64) -> Vec2 {
        self.p0.lerp(self.p1, t)
    }

    fn start(&self) -> Vec2 {
        self.p0
    }

    fn end(&self) -> Vec2 {
        self.p1
    }

    fn subsegment(&self, range: Range<f64>) -> Line {
        Line { p0: self.eval(range.start), p1: self.eval(range.end) }
    }
}

impl ParamCurveDeriv for Line {
    type DerivResult = ConstVec2;

    fn deriv(&self) -> ConstVec2 {
        ConstVec2(self.p1 - self.p0)
    }
}

impl ParamCurveArclen for Line {
    fn arclen(&self, _accuracy: f64) -> f64 {
        (self.p1 - self.p0).hypot()
    }
}

impl ParamCurveArea for Line {
    fn signed_area(&self) -> f64 {
        self.p0.cross(self.p1) * 0.5
    }
}

impl ParamCurveNearest for Line {
    fn nearest(&self, p: Vec2, _accuracy: f64) -> (f64, f64) {
        let d = self.p1 - self.p0;
        let dotp = d.dot(p - self.p0);
        let d_squared = d.dot(d);
        if dotp <= 0.0 {
            (0.0, (p - self.p0).hypot2())
        } else if dotp >= d_squared {
            (1.0, (p - self.p1).hypot2())
        } else {
            let t = dotp / d_squared;
            let dist = (p - self.eval(t)).hypot2();
            (t, dist)
        }
    }
}

impl ParamCurveCurvature for Line {
    fn curvature(&self, _t: f64) -> f64 {
        0.0
    }
}

impl ParamCurveExtrema for Line {
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]> {
        ArrayVec::new()
    }
}

/// A trivial "curve" that is just a constant.
#[derive(Clone, Copy)]
pub struct ConstVec2(Vec2);

impl ParamCurve for ConstVec2 {
    fn eval(&self, _t: f64) -> Vec2 {
        self.0
    }

    fn subsegment(&self, _range: Range<f64>) -> ConstVec2 {
        *self
    }
}

impl ParamCurveDeriv for ConstVec2 {
    type DerivResult = ConstVec2;

    fn deriv(&self) -> ConstVec2 {
        ConstVec2(Vec2::new(0.0, 0.0))
    }
}

impl ParamCurveArclen for ConstVec2 {
    fn arclen(&self, _accuracy: f64) -> f64 {
        0.0
    }
}

impl Mul<Line> for Affine {
    type Output = Line;

    fn mul(self, other: Line) -> Line {
        Line { p0: self * other.p0, p1: self * other.p1 }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Line, ParamCurveArclen};

    #[test]
    fn line_arclen() {
        let l = Line::new((0.0, 0.0), (1.0, 1.0));
        let true_len = 2.0f64.sqrt();
        let epsilon = 1e-9;
        assert!(l.arclen(epsilon) - true_len < epsilon);

        let t = l.inv_arclen(true_len / 3.0, epsilon);
        assert!((t - 1.0 / 3.0).abs() < epsilon);
        //println!("{}", t);
    }
}
