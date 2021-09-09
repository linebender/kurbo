//! Quadratic Bézier splines.
use crate::Point;

#[derive(Clone, Debug, PartialEq)]
/// A quadratic Bézier spline.
pub struct QuadSpline(Vec<Point>);

impl QuadSpline {
    /// Construct a new `QuadSpline` from an array of [`Point`]s.
    #[inline]
    pub fn new(points: Vec<Point>) -> Self {
        Self(points)
    }

    /// Return the spline's control [`Point`]s.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.0
    }
}
