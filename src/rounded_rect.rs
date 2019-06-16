//! A rectangle with rounded corners.

use crate::{PathEl, Rect, Shape, Vec2};
use std::f64::consts::PI;

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
            radius: self.radius.abs(),
        }
    }
}

#[doc(hidden)]
pub struct RoundedRectPathIter {
    rect: RoundedRect,
    ix: usize,
}

impl Shape for RoundedRect {
    type BezPathIter = RoundedRectPathIter;

    fn to_bez_path(&self, _tolerance: f64) -> RoundedRectPathIter {
        RoundedRectPathIter { rect: *self, ix: 0 }
    }

    #[inline]
    fn area(&self) -> f64 {
        self.rect.area() // TODO: upper bound only
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        2.0 * (self.width().abs() + self.height().abs()) - 8.0 * self.radius.abs()
            + 2.0 * PI * self.radius.abs()
    }

    #[inline]
    fn winding(&self, pt: Vec2) -> i32 {
        unimplemented!() // TODO
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

// This is clockwise in a y-down coordinate system for positive area.
impl Iterator for RoundedRectPathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        unimplemented!()
    }
}
