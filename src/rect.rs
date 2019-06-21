//! A rectangle.

use std::ops::{Add, Sub};

use crate::{PathEl, Point, Shape, Size, Vec2};

/// A rectangle.
#[derive(Clone, Copy, Default, Debug)]
pub struct Rect {
    /// The minimum x coordinate (left edge).
    pub x0: f64,
    /// The minimum y coordinate (top edge in y-down spaces).
    pub y0: f64,
    /// The maximum x coordinate (right edge).
    pub x1: f64,
    /// The maximum y coordinate (bottom edge in y-down spaces).
    pub y1: f64,
}

impl Rect {
    /// A new rectangle from minimum and maximum coordinates.
    #[inline]
    pub fn new(x0: f64, y0: f64, x1: f64, y1: f64) -> Rect {
        Rect { x0, y0, x1, y1 }
    }

    /// A new rectangle from two points.
    ///
    /// The result will have non-negative width and height.
    #[inline]
    pub fn from_points(p0: impl Into<Point>, p1: impl Into<Point>) -> Rect {
        let p0 = p0.into();
        let p1 = p1.into();
        Rect {
            x0: p0.x,
            y0: p0.y,
            x1: p1.x,
            y1: p1.y,
        }
        .abs()
    }

    /// A new rectangle from origin and size.
    ///
    /// The result will have non-negative width and height.
    #[inline]
    pub fn from_origin_size(origin: impl Into<Point>, size: impl Into<Size>) -> Rect {
        let origin = origin.into();
        Rect::from_points(origin, origin + size.into().to_vec2())
    }

    /// Create a new `Rect` with the same size as `self` and a new origin.
    #[inline]
    pub fn with_origin(self, origin: impl Into<Point>) -> Rect {
        Rect::from_origin_size(origin, self.size())
    }

    /// Create a new `Rect` with the same origin as `self` and a new size.
    #[inline]
    pub fn with_size(self, size: impl Into<Size>) -> Rect {
        Rect::from_origin_size(self.origin(), size)
    }

    /// The width of the rectangle.
    ///
    /// Note: nothing forbids negative width.
    #[inline]
    pub fn width(&self) -> f64 {
        self.x1 - self.x0
    }

    /// The height of the rectangle.
    ///
    /// Note: nothing forbids negative height.
    #[inline]
    pub fn height(&self) -> f64 {
        self.y1 - self.y0
    }

    /// The origin of the vector.
    ///
    /// This is the top left corner in a y-down space and with
    /// non-negative width and height.
    #[inline]
    pub fn origin(&self) -> Point {
        Point::new(self.x0, self.y0)
    }

    /// The size of the rectangle, as a vector.
    #[inline]
    pub fn size(&self) -> Size {
        Size::new(self.width(), self.height())
    }

    /// The area of the rectangle.
    #[inline]
    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    /// The center point of the rectangle.
    #[inline]
    pub fn center(&self) -> Point {
        Point::new(0.5 * (self.x0 + self.x1), 0.5 * (self.y0 + self.y1))
    }

    /// Take absolute value of width and height.
    ///
    /// The resulting rect has the same extents as the original, but is
    /// guaranteed to have non-negative width and height.
    #[inline]
    pub fn abs(&self) -> Rect {
        let Rect { x0, y0, x1, y1 } = *self;
        Rect {
            x0: x0.min(x1),
            y0: y0.min(y1),
            x1: x0.max(x1),
            y1: y0.max(y1),
        }
    }

    /// The smallest rectangle enclosing two rectangles.
    ///
    /// Results are valid only if width and height are non-negative.
    #[inline]
    pub fn union(&self, other: Rect) -> Rect {
        Rect {
            x0: self.x0.min(other.x0),
            y0: self.y0.min(other.y0),
            x1: self.x1.max(other.x1),
            y1: self.y1.max(other.y1),
        }
    }

    /// Compute the union with one point.
    ///
    /// This method includes the perimeter of zero-area rectangles.
    /// Thus, a succession of `union_pt` operations on a series of
    /// points yields their enclosing rectangle.
    ///
    /// Results are valid only if width and height are non-negative.
    pub fn union_pt(&self, pt: Point) -> Rect {
        Rect::new(
            self.x0.min(pt.x),
            self.y0.min(pt.y),
            self.x1.max(pt.x),
            self.y1.max(pt.y),
        )
    }

    /// The intersection of two rectangles.
    ///
    /// The result is zero-area if either input has negative width or
    /// height. The result always has non-negative width and height.
    #[inline]
    pub fn intersect(&self, other: Rect) -> Rect {
        let x0 = self.x0.max(other.x0);
        let y0 = self.y0.max(other.y0);
        let x1 = self.x1.min(other.x1);
        let y1 = self.y1.min(other.y1);
        Rect {
            x0,
            y0,
            x1: x1.max(x0),
            y1: y1.max(y0),
        }
    }

    /// Expand a rectangle by a constant amount in both directions.
    ///
    /// The logic simply applies the amount in each direction. If rectangle
    /// area or added dimensions are negative, this could give odd results.
    pub fn inflate(&self, width: f64, height: f64) -> Rect {
        Rect {
            x0: self.x0 - width,
            y0: self.y0 - height,
            x1: self.x1 + width,
            y1: self.y1 + height,
        }
    }
}

impl From<((f64, f64), (f64, f64))> for Rect {
    fn from(coords: ((f64, f64), (f64, f64))) -> Rect {
        let ((x0, y0), (x1, y1)) = coords;
        Rect { x0, y0, x1, y1 }
    }
}

impl From<Rect> for ((f64, f64), (f64, f64)) {
    fn from(r: Rect) -> ((f64, f64), (f64, f64)) {
        ((r.x0, r.y0), (r.x1, r.y1))
    }
}

impl From<(Point, Point)> for Rect {
    fn from(points: (Point, Point)) -> Rect {
        Rect::from_points(points.0, points.1)
    }
}

impl From<(Point, Size)> for Rect {
    fn from(params: (Point, Size)) -> Rect {
        Rect::from_origin_size(params.0, params.1)
    }
}

impl Add<Vec2> for Rect {
    type Output = Rect;

    #[inline]
    fn add(self, v: Vec2) -> Rect {
        Rect {
            x0: self.x0 + v.x,
            y0: self.y0 + v.y,
            x1: self.x1 + v.x,
            y1: self.y1 + v.y,
        }
    }
}

impl Sub<Vec2> for Rect {
    type Output = Rect;

    #[inline]
    fn sub(self, v: Vec2) -> Rect {
        Rect {
            x0: self.x0 - v.x,
            y0: self.y0 - v.y,
            x1: self.x1 - v.x,
            y1: self.y1 - v.y,
        }
    }
}

#[doc(hidden)]
pub struct RectPathIter {
    rect: Rect,
    ix: usize,
}

impl Shape for Rect {
    type BezPathIter = RectPathIter;

    fn to_bez_path(&self, _tolerance: f64) -> RectPathIter {
        RectPathIter { rect: *self, ix: 0 }
    }

    // It's a bit of duplication having both this and the impl method, but
    // removing that would require using the trait. We'll leave it for now.
    #[inline]
    fn area(&self) -> f64 {
        Rect::area(self)
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        2.0 * (self.width().abs() + self.height().abs())
    }

    /// Note: this function is carefully designed so that if the plane is
    /// tiled with rectangles, the winding number will be nonzero for exactly
    /// one of them.
    #[inline]
    fn winding(&self, pt: Point) -> i32 {
        let xmin = self.x0.min(self.x1);
        let xmax = self.x0.max(self.x1);
        let ymin = self.y0.min(self.y1);
        let ymax = self.y0.max(self.y1);
        if pt.x >= xmin && pt.x < xmax && pt.y >= ymin && pt.y < ymax {
            if (self.x1 > self.x0) ^ (self.y1 > self.y0) {
                -1
            } else {
                1
            }
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        self.abs()
    }

    #[inline]
    fn as_rect(&self) -> Option<Rect> {
        Some(*self)
    }
}

// This is clockwise in a y-down coordinate system for positive area.
impl Iterator for RectPathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.ix += 1;
        match self.ix {
            1 => Some(PathEl::MoveTo(Point::new(self.rect.x0, self.rect.y0))),
            2 => Some(PathEl::LineTo(Point::new(self.rect.x1, self.rect.y0))),
            3 => Some(PathEl::LineTo(Point::new(self.rect.x1, self.rect.y1))),
            4 => Some(PathEl::LineTo(Point::new(self.rect.x0, self.rect.y1))),
            5 => Some(PathEl::ClosePath),
            _ => None,
        }
    }
}
