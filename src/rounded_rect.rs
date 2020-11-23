//! A rectangle with rounded corners.

use crate::{arc::ArcAppendIter, Arc, PathEl, Point, Rect, Shape, Size, Vec2};
use std::f64::consts::{FRAC_PI_2, PI};

/// A rectangle with equally rounded corners.
///
/// By construction the rounded rectangle will have
/// non-negative dimensions and radius clamped to half size of the rect.
///
/// The easiest way to create a `RoundedRect` is often to create a [`Rect`],
/// and then call [`to_rounded_rect`].
///
/// [`to_rounded_rect`]: Rect::to_rounded_rect
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoundedRect {
    /// Coordinates of the rectangle.
    rect: Rect,
    /// Radius of all four corners.
    radius: f64,
}

impl RoundedRect {
    /// A new rectangle from minimum and maximum coordinates.
    ///
    /// The result will have non-negative width, height and radius.
    #[inline]
    pub fn new(x0: f64, y0: f64, x1: f64, y1: f64, radius: f64) -> RoundedRect {
        RoundedRect::from_rect(Rect::new(x0, y0, x1, y1), radius)
    }

    /// A new rounded rectangle from a rectangle and corner radius.
    ///
    /// The result will have non-negative width, height and radius.
    ///
    /// See also [`Rect::to_rounded_rect`], which offers the same utility.
    #[inline]
    pub fn from_rect(rect: Rect, radius: f64) -> RoundedRect {
        let rect = rect.abs();
        let radius = radius
            .abs()
            .min(rect.width() / 2.0)
            .min(rect.height() / 2.0);

        RoundedRect { rect, radius }
    }

    /// A new rectangle from two [`Point`]s.
    ///
    /// The result will have non-negative width, height and radius.
    #[inline]
    pub fn from_points(p0: impl Into<Point>, p1: impl Into<Point>, radius: f64) -> RoundedRect {
        Rect::from_points(p0, p1).to_rounded_rect(radius)
    }

    /// A new rectangle from origin and size.
    ///
    /// The result will have non-negative width, height and radius.
    #[inline]
    pub fn from_origin_size(
        origin: impl Into<Point>,
        size: impl Into<Size>,
        radius: f64,
    ) -> RoundedRect {
        Rect::from_origin_size(origin, size).to_rounded_rect(radius)
    }

    /// The width of the rectangle.
    #[inline]
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// The height of the rectangle.
    #[inline]
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Radius of the rounded corners.
    #[inline]
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The (non-rounded) rectangle.
    pub fn rect(&self) -> Rect {
        self.rect
    }

    /// The origin of the rectangle.
    ///
    /// This is the top left corner in a y-down space.
    #[inline]
    pub fn origin(&self) -> Point {
        self.rect.origin()
    }

    /// The center point of the rectangle.
    #[inline]
    pub fn center(&self) -> Point {
        self.rect.center()
    }
}

#[doc(hidden)]
pub struct RoundedRectPathIter {
    idx: usize,
    rect: RectPathIter,
    arcs: [ArcAppendIter; 4],
}

impl Shape for RoundedRect {
    type PathElementsIter = RoundedRectPathIter;

    fn path_elements(&self, tolerance: f64) -> RoundedRectPathIter {
        let radius = self.radius();
        let radii = Vec2 {
            x: self.radius,
            y: self.radius,
        };

        let build_arc_iter = |i, center| {
            let arc = Arc {
                center,
                radii,
                start_angle: FRAC_PI_2 * i as f64,
                sweep_angle: FRAC_PI_2,
                x_rotation: 0.0,
            };
            arc.append_iter(tolerance)
        };

        // Note: order follows the rectangle path iterator.
        let arcs = [
            build_arc_iter(
                2,
                Point {
                    x: self.rect.x0 + radius,
                    y: self.rect.y0 + radius,
                },
            ),
            build_arc_iter(
                3,
                Point {
                    x: self.rect.x1 - radius,
                    y: self.rect.y0 + radius,
                },
            ),
            build_arc_iter(
                0,
                Point {
                    x: self.rect.x1 - radius,
                    y: self.rect.y1 - radius,
                },
            ),
            build_arc_iter(
                1,
                Point {
                    x: self.rect.x0 + radius,
                    y: self.rect.y1 - radius,
                },
            ),
        ];

        let rect = RectPathIter {
            rect: self.rect,
            ix: 0,
            radius: self.radius,
        };

        RoundedRectPathIter { idx: 0, rect, arcs }
    }

    #[inline]
    fn area(&self) -> f64 {
        let radius = self.radius();
        self.rect.area() - (4.0 - PI) * radius * radius
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        let radius = self.radius();
        2.0 * (self.width() + self.height()) - 8.0 * radius + 2.0 * PI * radius
    }

    #[inline]
    fn winding(&self, mut pt: Point) -> i32 {
        // The rounded rectangle can be seen as minkowski sum of an inner rectangle
        // and circle specified by the radius of the corners.

        let center = self.center();
        let radius = self.radius();
        let inside_half_width = (self.width() / 2.0 - radius).max(0.0);
        let inside_half_height = (self.height() / 2.0 - radius).max(0.0);

        // 1. Translate the point relative to the center of the rectangle.
        pt.x -= center.x;
        pt.y -= center.y;

        // 2. Project point out of the inner rectangle (positive quadrant)
        //    This basically 'substracts' the inner rectangle.
        let px = (pt.x.abs() - inside_half_width).max(0.0);
        let py = (pt.y.abs() - inside_half_height).max(0.0);

        // 3. The test reduced to calculate the winding of the circle.
        let inside = px * px + py * py <= radius * radius;
        if inside {
            1
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        self.rect.bounding_box()
    }

    #[inline]
    fn as_rounded_rect(&self) -> Option<RoundedRect> {
        Some(*self)
    }
}

struct RectPathIter {
    rect: Rect,
    radius: f64,
    ix: usize,
}

// This is clockwise in a y-down coordinate system for positive area.
impl Iterator for RectPathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.ix += 1;
        match self.ix {
            1 => Some(PathEl::MoveTo(Point::new(
                self.rect.x0,
                self.rect.y0 + self.radius,
            ))),
            2 => Some(PathEl::LineTo(Point::new(
                self.rect.x1 - self.radius,
                self.rect.y0,
            ))),
            3 => Some(PathEl::LineTo(Point::new(
                self.rect.x1,
                self.rect.y1 - self.radius,
            ))),
            4 => Some(PathEl::LineTo(Point::new(
                self.rect.x0 + self.radius,
                self.rect.y1,
            ))),
            5 => Some(PathEl::ClosePath),
            _ => None,
        }
    }
}

// This is clockwise in a y-down coordinate system for positive area.
impl Iterator for RoundedRectPathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        if self.idx > 4 {
            return None;
        }

        // Iterate between rectangle and arc iterators.
        // Rect iterator will start and end the path.

        // Initial point set by the rect iterator
        if self.idx == 0 {
            self.idx += 1;
            return self.rect.next();
        }

        // Generate the arc curve elements.
        // If we reached the end of the arc, add a line towards next arc (rect iterator).
        match self.arcs[self.idx - 1].next() {
            Some(elem) => Some(elem),
            None => {
                self.idx += 1;
                self.rect.next()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Circle, Point, Rect, RoundedRect, Shape};

    #[test]
    fn area() {
        let epsilon = 1e-9;

        // Extremum: 0.0 radius corner -> rectangle
        let rect = Rect::new(0.0, 0.0, 100.0, 100.0);
        let rounded_rect = RoundedRect::new(0.0, 0.0, 100.0, 100.0, 0.0);
        assert!((rect.area() - rounded_rect.area()).abs() < epsilon);

        // Extremum: half-size radius corner -> circle
        let circle = Circle::new((0.0, 0.0), 50.0);
        let rounded_rect = RoundedRect::new(0.0, 0.0, 100.0, 100.0, 50.0);
        assert!((circle.area() - rounded_rect.area()).abs() < epsilon);
    }

    #[test]
    fn winding() {
        let rect = RoundedRect::new(-5.0, -5.0, 10.0, 20.0, 5.0);
        assert_eq!(rect.winding(Point::new(0.0, 0.0)), 1);
        assert_eq!(rect.winding(Point::new(-5.0, 0.0)), 1); // left edge
        assert_eq!(rect.winding(Point::new(0.0, 20.0)), 1); // bottom edge
        assert_eq!(rect.winding(Point::new(10.0, 20.0)), 0); // bottom-right corner
        assert_eq!(rect.winding(Point::new(-10.0, 0.0)), 0);

        let rect = RoundedRect::new(-10.0, -20.0, 10.0, 20.0, 0.0); // rectangle
        assert_eq!(rect.winding(Point::new(10.0, 20.0)), 1); // bottom-right corner
    }

    #[test]
    fn bez_conversion() {
        let rect = RoundedRect::new(-5.0, -5.0, 10.0, 20.0, 5.0);
        let p = rect.to_path(1e-9);
        // Note: could be more systematic about tolerance tightness.
        let epsilon = 1e-7;
        assert!((rect.area() - p.area()).abs() < epsilon);
        assert_eq!(p.winding(Point::new(0.0, 0.0)), 1);
    }
}
