//! A trait for curves parametrized by a scalar.

use std::ops::Range;

use crate::Vec2;

/// A curve parametrized by a scalar.
///
/// If the result is interpreted as a point, this represents a curve.
/// But the result can be interpreted as a vector as well.
pub trait ParamCurve: Sized {
    /// Evaluate the curve at parameter `t`.
    ///
    /// Generally `t` is in the range [0..1].
    fn eval(&self, t: f64) -> Vec2;

    /// Get a subsegment of the curve for the given parameter range.
    fn subsegment(&self, range: Range<f64>) -> Self;

    /// Subdivide into (roughly) halves.
    fn subdivide(&self) -> (Self, Self) {
        (self.subsegment(0.0 .. 0.5), self.subsegment(0.5 .. 1.0))
    }

    /// The start point.
    fn start(&self) -> Vec2 { self.eval(0.0) }

    /// The end point.
    fn end(&self) -> Vec2 { self.eval(1.0) }
}

// TODO: I might not want to have separate traits for all these.

/// A differentiable parametrized curve.
pub trait ParamCurveDeriv {
    type DerivResult: ParamCurve;

    /// The derivative of the curve.
    fn deriv(&self) -> Self::DerivResult;
}

/// A parametrized curve that can have its arc length measured.
pub trait ParamCurveArclen: ParamCurve {
    /// The arc length of the curve.
    ///
    /// The result is accurate to the given accuracy (subject to
    /// roundoff errors for ridiculously low values). Compute time
    /// may vary with accuracy, if the curve needs to be subdivided.
    fn arclen(&self, accuracy: f64) -> f64;

    /// Solve for the parameter that has the given arclength from the start.
    ///
    /// This implementation is bisection, which is very robust but not
    /// necessarily the fastest. It does measure increasingly short
    /// segments, though, which should be good for subdivision algorithms.
    fn inv_arclen(&self, arclen: f64, accuracy: f64) -> f64 {
        // invariant: the curve's arclen on [0..t_last] + remaining = arclen
        let mut remaining = arclen;
        let mut t_last = 0.0;
        let mut t0 = 0.0;
        let mut t1 = 1.0;
        let n = (-accuracy.log2()).ceil();
        let inner_accuracy = accuracy / n;
        let n = n as usize;
        for i in 0..n {
            let tm = 0.5 * (t0 + t1);
            let (range, dir) = if tm > t_last {
                (t_last .. tm, 1.0)
            } else {
                (tm .. t_last, -1.0)
            };
            let range_size = range.end - range.start;
            let arc = self.subsegment(range).arclen(inner_accuracy);
            //println!("tm={}, arc={}, remaining={}", tm, arc, remaining);
            remaining -= arc * dir;
            if i == n - 1 || (remaining).abs() < accuracy {
                // Allocate remaining arc evenly.
                return tm + range_size * remaining / arc;
            }
            if remaining > 0.0 {
                t0 = tm;
            } else {
                t1 = tm;
            }
            t_last = tm;
        }
        unreachable!();
    }
}

/// A parametrized curve that can have its signed area measured.
pub trait ParamCurveArea {
    /// Compute the signed area under the curve.
    ///
    /// For a closed path, the signed area of the path is the sum of signed
    /// areas of the segments. This is a variant of the "shoelace formula."
    /// See:
    /// <https://github.com/Pomax/bezierinfo/issues/44> and
    /// <http://ich.deanmcnamee.com/graphics/2016/03/30/CurveArea.html>
    ///
    /// This can be computed exactly for Béziers thanks to Green's theorem,
    /// and also for simple curves such as circular arcs. For more exotic
    /// curves, it's probably best to subdivide to cubics. We leave that
    /// to the caller, which is why we don't give an accuracy param here.
    fn signed_area(&self) -> f64;
}

/// A parametrized curve that reports the nearest point.
pub trait ParamCurveNearest {
    /// Find the point on the curve nearest the given point.
    ///
    /// Returns the parameter and the square of the distance.
    fn nearest(&self, p: Vec2, accuracy: f64) -> (f64, f64);
}

/// A parametrized curve that reports its curvature.
pub trait ParamCurveCurvature: ParamCurveDeriv
    where Self::DerivResult: ParamCurveDeriv
{
    /// Compute the signed curvature at parameter `t`.
    fn curvature(&self, t: f64) -> f64 {
        let deriv = self.deriv();
        let deriv2 = deriv.deriv();
        let d = deriv.eval(t);
        let d2 = deriv2.eval(t);
        // TODO: What's the convention for sign? I think it should match signed
        // area - a positive area curve should have positive curvature.
        d2.cross(d) * d.hypot2().powf(-1.5)
    }
}
