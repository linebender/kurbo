//! Lines.

use std::ops::{Mul, Range};

use arrayvec::ArrayVec;

use crate::MAX_EXTREMA;
use crate::{
    Affine, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveCurvature, ParamCurveDeriv,
    ParamCurveExtrema, ParamCurveNearest, PathEl, Point, Rect, Shape,
};

/// A single line.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    /// The line's start point.
    pub p0: Point,
    /// The line's end point.
    pub p1: Point,
}

impl Line {
    /// Create a new line.
    #[inline]
    pub fn new(p0: impl Into<Point>, p1: impl Into<Point>) -> Line {
        Line {
            p0: p0.into(),
            p1: p1.into(),
        }
    }
}

impl ParamCurve for Line {
    #[inline]
    fn eval(&self, t: f64) -> Point {
        self.p0.lerp(self.p1, t)
    }

    #[inline]
    fn start(&self) -> Point {
        self.p0
    }

    #[inline]
    fn end(&self) -> Point {
        self.p1
    }

    #[inline]
    fn subsegment(&self, range: Range<f64>) -> Line {
        Line {
            p0: self.eval(range.start),
            p1: self.eval(range.end),
        }
    }
}

impl ParamCurveDeriv for Line {
    type DerivResult = ConstPoint;

    #[inline]
    fn deriv(&self) -> ConstPoint {
        ConstPoint((self.p1 - self.p0).to_point())
    }
}

impl ParamCurveArclen for Line {
    #[inline]
    fn arclen(&self, _accuracy: f64) -> f64 {
        (self.p1 - self.p0).hypot()
    }
}

impl ParamCurveArea for Line {
    #[inline]
    fn signed_area(&self) -> f64 {
        self.p0.to_vec2().cross(self.p1.to_vec2()) * 0.5
    }
}

impl ParamCurveNearest for Line {
    fn nearest(&self, p: Point, _accuracy: f64) -> (f64, f64) {
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
    #[inline]
    fn curvature(&self, _t: f64) -> f64 {
        0.0
    }
}

impl ParamCurveExtrema for Line {
    #[inline]
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]> {
        ArrayVec::new()
    }
}

/// A trivial "curve" that is just a constant.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ConstPoint(Point);

impl ParamCurve for ConstPoint {
    #[inline]
    fn eval(&self, _t: f64) -> Point {
        self.0
    }

    #[inline]
    fn subsegment(&self, _range: Range<f64>) -> ConstPoint {
        *self
    }
}

impl ParamCurveDeriv for ConstPoint {
    type DerivResult = ConstPoint;

    #[inline]
    fn deriv(&self) -> ConstPoint {
        ConstPoint(Point::new(0.0, 0.0))
    }
}

impl ParamCurveArclen for ConstPoint {
    #[inline]
    fn arclen(&self, _accuracy: f64) -> f64 {
        0.0
    }
}

impl Mul<Line> for Affine {
    type Output = Line;

    #[inline]
    fn mul(self, other: Line) -> Line {
        Line {
            p0: self * other.p0,
            p1: self * other.p1,
        }
    }
}

/// An iterator yielding the path for a single line.
#[doc(hidden)]
pub struct LinePathIter {
    line: Line,
    ix: usize,
}

impl Shape for Line {
    type BezPathIter = LinePathIter;

    #[inline]
    fn to_bez_path(&self, _tolerance: f64) -> LinePathIter {
        LinePathIter { line: *self, ix: 0 }
    }

    /// Returning zero here is consistent with the contract (area is
    /// only meaningful for closed shapes), but an argument can be made
    /// that the contract should be tightened to include the Green's
    /// theorem contribution.
    fn area(&self) -> f64 {
        0.0
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        (self.p1 - self.p0).hypot()
    }

    /// Same consideration as `area`.
    fn winding(&self, _pt: Point) -> i32 {
        0
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        Rect::from_points(self.p0, self.p1)
    }

    #[inline]
    fn as_line(&self) -> Option<Line> {
        Some(*self)
    }
}

impl Iterator for LinePathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.ix += 1;
        match self.ix {
            1 => Some(PathEl::MoveTo(self.line.p0)),
            2 => Some(PathEl::LineTo(self.line.p1)),
            _ => None,
        }
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
    }
}
