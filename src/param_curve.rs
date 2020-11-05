//! A trait for curves parametrized by a scalar.

use std::ops::Range;

use arrayvec::ArrayVec;

use crate::{Point, Rect};

/// A default value for methods that take an 'accuracy' argument.
///
/// This value is intended to be suitable for general-purpose use, such as
/// 2d graphics.
pub const DEFAULT_ACCURACY: f64 = 1e-6;

/// A curve parametrized by a scalar.
///
/// If the result is interpreted as a point, this represents a curve.
/// But the result can be interpreted as a vector as well.
pub trait ParamCurve: Sized {
    /// Evaluate the curve at parameter `t`.
    ///
    /// Generally `t` is in the range [0..1].
    fn eval(&self, t: f64) -> Point;

    /// Get a subsegment of the curve for the given parameter range.
    fn subsegment(&self, range: Range<f64>) -> Self;

    /// Subdivide into (roughly) halves.
    #[inline]
    fn subdivide(&self) -> (Self, Self) {
        (self.subsegment(0.0..0.5), self.subsegment(0.5..1.0))
    }

    /// The start point.
    fn start(&self) -> Point {
        self.eval(0.0)
    }

    /// The end point.
    fn end(&self) -> Point {
        self.eval(1.0)
    }
}

// TODO: I might not want to have separate traits for all these.

/// A differentiable parametrized curve.
pub trait ParamCurveDeriv {
    /// The parametric curve obtained by taking the derivative of this one.
    type DerivResult: ParamCurve;

    /// The derivative of the curve.
    ///
    /// Note that the type of the return value is somewhat inaccurate, as
    /// the derivative of a curve (mapping of param to point) is a mapping
    /// of param to vector. We choose to accept this rather than have a
    /// more complex type scheme.
    fn deriv(&self) -> Self::DerivResult;

    /// Estimate arclength using Gaussian quadrature.
    ///
    /// The coefficients are assumed to cover the range (-1..1), which is
    /// traditional.
    #[inline]
    fn gauss_arclen(&self, coeffs: &[(f64, f64)]) -> f64 {
        let d = self.deriv();
        coeffs
            .iter()
            .map(|(wi, xi)| wi * d.eval(0.5 * (xi + 1.0)).to_vec2().hypot())
            .sum::<f64>()
            * 0.5
    }
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
        let n = (self.arclen(accuracy) / accuracy).log2().ceil().max(1.0);
        let inner_accuracy = accuracy / n;
        let n = n as usize;
        for i in 0..n {
            let tm = 0.5 * (t0 + t1);
            let (range, dir) = if tm > t_last {
                (t_last..tm, 1.0)
            } else {
                (tm..t_last, -1.0)
            };
            let range_size = range.end - range.start;
            let arc = self.subsegment(range).arclen(inner_accuracy);
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
    fn nearest(&self, p: Point, accuracy: f64) -> (f64, f64);
}

/// A parametrized curve that reports its curvature.
pub trait ParamCurveCurvature: ParamCurveDeriv
where
    Self::DerivResult: ParamCurveDeriv,
{
    /// Compute the signed curvature at parameter `t`.
    #[inline]
    fn curvature(&self, t: f64) -> f64 {
        let deriv = self.deriv();
        let deriv2 = deriv.deriv();
        let d = deriv.eval(t).to_vec2();
        let d2 = deriv2.eval(t).to_vec2();
        // TODO: What's the convention for sign? I think it should match signed
        // area - a positive area curve should have positive curvature.
        d2.cross(d) * d.hypot2().powf(-1.5)
    }
}

/// The maximum number of extrema that can be reported in the `ParamCurveExtrema` trait.
///
/// This is 4 to support cubic Béziers. If other curves are used, they should be
/// subdivided to limit the number of extrema.
pub const MAX_EXTREMA: usize = 4;

/// A parametrized curve that reports its extrema.
pub trait ParamCurveExtrema: ParamCurve {
    /// Compute the extrema of the curve.
    ///
    /// Only extrema within the interior of the curve count.
    /// At most four extrema can be reported, which is sufficient for
    /// cubic Béziers.
    ///
    /// The extrema should be reported in increasing parameter order.
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]>;

    /// Return parameter ranges, each of which is monotonic within the range.
    fn extrema_ranges(&self) -> ArrayVec<[Range<f64>; MAX_EXTREMA + 1]> {
        let mut result = ArrayVec::new();
        let mut t0 = 0.0;
        for t in self.extrema() {
            result.push(t0..t);
            t0 = t;
        }
        result.push(t0..1.0);
        result
    }

    /// The smallest rectangle that encloses the curve in the range (0..1).
    fn bounding_box(&self) -> Rect {
        let mut bbox = Rect::from_points(self.start(), self.end());
        for t in self.extrema() {
            bbox = bbox.union_pt(self.eval(t))
        }
        bbox
    }
}
