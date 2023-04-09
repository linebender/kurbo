use crate::Point;

/// An SVG path
pub struct SvgPath(Vec<PathEl>);

/// An SVG path element
pub enum PathEl {
    /// Start a new sub-path at the given (x,y) coordinates.
    MoveTo(Point),
    /// Close the current subpath by connecting it back to the current subpath's initial point.
    ClosePath,
    /// Draw a line from the current point to the given point.
    LineTo(Point),
    /// Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`.
    CurveTo {
        to: Point,
        ctrl1: Point,
        ctrl2: Point,
    },
}

struct CurveTo {
    to: Point,
    ctrl1: Point,
    ctrl2: Point,
}
