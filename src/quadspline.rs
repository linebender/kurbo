//! Quadratic Bézier splines.
use crate::Point;

#[derive(Clone, Debug, PartialEq)]
#[allow(missing_docs)]
pub struct QuadSpline(Vec<Point>);

impl QuadSpline {
    /// Construct a new quadratic spline from an array of points.
    #[inline]
    pub fn new(points: Vec<Point>) -> Self {
        Self(points)
    }

    /// Return the control points of the spline.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.0
    }
}
