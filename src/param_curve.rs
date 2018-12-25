//! A trait for curves parametrized by a scalar.

use crate::Vec2;

/// A curve parametrized by a scalar.
///
/// If the result is interpreted as a point, this represents a curve.
/// But the result can be interpreted as a vector as well.
pub trait ParamCurve {
    /// Evaluate the curve at parameter `t`.
    ///
    /// Generally `t` is in the range [0..1].
    fn eval(&self, t: f64) -> Vec2;

    /// The start point.
    fn start(&self) -> Vec2 { self.eval(0.0) }

    /// The end point.
    fn end(&self) -> Vec2 { self.eval(1.0) }
}

/// A differentiable parametrized curve.
pub trait ParamCurveDeriv {
    type DerivResult: ParamCurve;

    /// The derivative of the curve.
    fn deriv(&self) -> Self::DerivResult;
}

/// A parametrized curve that can have its arc length measured.
pub trait ParamCurveArclen {
    /// The arc length of the curve.
    ///
    /// The result is accurate to the given accuracy (subject to
    /// roundoff errors for ridiculously low values). Compute time
    /// may vary with accuracy, if the curve needs to be subdivided.
    fn arclen(&self, accuracy: f64) -> f64;
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
