//! A rectangle with rounded corners.

use crate::{
    arc::{Arc, ArcAppendIter},
    PathEl, Rect, Shape, Vec2,
};
use std::f64::consts::{FRAC_PI_2, PI};

/// A rectangle with rounded corners.
#[derive(Clone, Copy, Default, Debug)]
pub struct RoundedRect {
    /// Coordinates of the rectangle.
    pub rect: Rect,
    /// Radius of all four corners.
    pub radius: f64,
}

impl RoundedRect {
    /// A new rectangle from minimum and maximum coordinates.
    #[inline]
    pub fn new(x0: f64, y0: f64, x1: f64, y1: f64, radius: f64) -> RoundedRect {
        RoundedRect {
            rect: Rect::new(x0, y0, x1, y1),
            radius,
        }
    }

    /// A new rectangle from two points.
    ///
    /// The result will have non-negative width and height.
    #[inline]
    pub fn from_points(p0: Vec2, p1: Vec2, radius: f64) -> RoundedRect {
        RoundedRect {
            rect: Rect::from_points(p0, p1),
            radius,
        }
    }

    /// A new rectangle from origin and size.
    ///
    /// The result will have non-negative width and height.
    #[inline]
    pub fn from_origin_size(origin: Vec2, size: Vec2, radius: f64) -> RoundedRect {
        RoundedRect {
            rect: Rect::from_points(origin, origin + size),
            radius,
        }
    }

    /// The width of the rectangle.
    ///
    /// Note: nothing forbids negative width.
    #[inline]
    pub fn width(&self) -> f64 {
        self.rect.width()
    }

    /// The height of the rectangle.
    ///
    /// Note: nothing forbids negative height.
    #[inline]
    pub fn height(&self) -> f64 {
        self.rect.height()
    }

    /// Absolute radius of the rounded corners.
    ///
    /// Clamped to half the size of the rectangle.
    pub fn radius(&self) -> f64 {
        self.radius
            .abs()
            .min(self.width().abs() / 2.0)
            .min(self.height().abs() / 2.0)
    }

    /// The origin of the vector.
    ///
    /// This is the top left corner in a y-down space and with
    /// non-negative width and height.
    #[inline]
    pub fn origin(&self) -> Vec2 {
        self.rect.origin()
    }

    /// The center point of the rectangle.
    #[inline]
    pub fn center(&self) -> Vec2 {
        self.rect.center()
    }

    /// Take absolute value of width, height and radius.
    ///
    /// The resulting rect has the same extents as the original, but is
    /// guaranteed to have non-negative properties.
    #[inline]
    pub fn abs(&self) -> RoundedRect {
        RoundedRect {
            rect: self.rect.abs(),
            radius: self.radius(),
        }
    }
}

#[doc(hidden)]
pub struct RoundedRectPathIter {
    idx: usize,
    rect: RectPathIter,
    arcs: [ArcAppendIter; 4],
}

impl Shape for RoundedRect {
    type BezPathIter = RoundedRectPathIter;

    fn to_bez_path(&self, tolerance: f64) -> RoundedRectPathIter {
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
                Vec2 {
                    x: self.rect.x0 + radius,
                    y: self.rect.y0 + radius,
                },
            ),
            build_arc_iter(
                3,
                Vec2 {
                    x: self.rect.x1 - radius,
                    y: self.rect.y0 + radius,
                },
            ),
            build_arc_iter(
                0,
                Vec2 {
                    x: self.rect.x1 - radius,
                    y: self.rect.y1 - radius,
                },
            ),
            build_arc_iter(
                1,
                Vec2 {
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
        2.0 * (self.width().abs() + self.height().abs()) - 8.0 * radius + 2.0 * PI * radius
    }

    #[inline]
    fn winding(&self, pt: Vec2) -> i32 {
        let radius = self.radius();
        let inside_half_width = (self.width() / 2.0 - self.radius()).max(0.0);
        let inside_half_height = (self.height() / 2.0 - self.radius()).max(0.0);

        // project point out of the inner rectangle (positive quadrant)
        let px = (pt.x.abs() - inside_half_width).max(0.0);
        let py = (pt.y.abs() - inside_half_height).max(0.0);

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
            1 => Some(PathEl::Moveto(Vec2::new(
                self.rect.x0,
                self.rect.y0 + self.radius,
            ))),
            2 => Some(PathEl::Lineto(Vec2::new(
                self.rect.x1 - self.radius,
                self.rect.y0,
            ))),
            3 => Some(PathEl::Lineto(Vec2::new(
                self.rect.x1,
                self.rect.y1 - self.radius,
            ))),
            4 => Some(PathEl::Lineto(Vec2::new(
                self.rect.x0 + self.radius,
                self.rect.y1,
            ))),
            5 => Some(PathEl::Closepath),
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
        if self.idx == 0 {
            self.idx += 1;
            self.rect.next()
        } else {
            match self.arcs[self.idx - 1].next() {
                Some(elem) => Some(elem),
                None => {
                    self.idx += 1;
                    self.rect.next()
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{Circle, Rect, RoundedRect, Shape, Vec2};

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
        let rect = RoundedRect::new(-10.0, -20.0, 10.0, 20.0, 5.0);
        assert_eq!(rect.winding(Vec2::new(0.0, 0.0)), 1); // center
        assert_eq!(rect.winding(Vec2::new(-10.0, 0.0)), 1); // left edge
        assert_eq!(rect.winding(Vec2::new(0.0, 20.0)), 1); // top edge
        assert_eq!(rect.winding(Vec2::new(10.0, 20.0)), 0); // bottom-right corner

        let rect = RoundedRect::new(-10.0, -20.0, 10.0, 20.0, 0.0); // rectangle
        assert_eq!(rect.winding(Vec2::new(10.0, 20.0)), 1); // bottom-right corner
    }
}
