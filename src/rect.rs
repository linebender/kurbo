//! A rectangle.

use std::fmt;
use std::ops::{Add, Sub};

use crate::common::FloatExt;
use crate::{Insets, PathEl, Point, RoundedRect, Shape, Size, Vec2};

/// A rectangle.
#[derive(Clone, Copy, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// The empty rectangle at the origin.
    pub const ZERO: Rect = Rect::new(0., 0., 0., 0.);

    /// A new rectangle from minimum and maximum coordinates.
    #[inline]
    pub const fn new(x0: f64, y0: f64, x1: f64, y1: f64) -> Rect {
        Rect { x0, y0, x1, y1 }
    }

    /// A new rectangle from two points.
    ///
    /// The result will have non-negative width and height.
    #[inline]
    pub fn from_points(p0: impl Into<Point>, p1: impl Into<Point>) -> Rect {
        let p0 = p0.into();
        let p1 = p1.into();
        Rect::new(p0.x, p0.y, p1.x, p1.y).abs()
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

    /// Create a new `Rect` by applying the [`Insets`].
    ///
    /// This will not preserve negative width and height.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let inset_rect = Rect::new(0., 0., 10., 10.,).inset(2.);
    /// assert_eq!(inset_rect.width(), 14.0);
    /// assert_eq!(inset_rect.x0, -2.0);
    /// assert_eq!(inset_rect.x1, 12.0);
    /// ```
    ///
    /// [`Insets`]: struct.Insets.html
    #[inline]
    pub fn inset(self, insets: impl Into<Insets>) -> Rect {
        self + insets.into()
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

    /// Returns the minimum value for the x-coordinate of the rectangle.
    #[inline]
    pub fn min_x(&self) -> f64 {
        self.x0.min(self.x1)
    }

    /// Returns the maximum value for the x-coordinate of the rectangle.
    #[inline]
    pub fn max_x(&self) -> f64 {
        self.x0.max(self.x1)
    }

    /// Returns the minimum value for the y-coordinate of the rectangle.
    #[inline]
    pub fn min_y(&self) -> f64 {
        self.y0.min(self.y1)
    }

    /// Returns the maximum value for the y-coordinate of the rectangle.
    #[inline]
    pub fn max_y(&self) -> f64 {
        self.y0.max(self.y1)
    }

    /// The origin of the rectangle.
    ///
    /// This is the top left corner in a y-down space and with
    /// non-negative width and height.
    #[inline]
    pub fn origin(&self) -> Point {
        Point::new(self.x0, self.y0)
    }

    /// The size of the rectangle.
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

    /// Returns `true` if `point` lies within `self`.
    #[inline]
    pub fn contains(&self, point: Point) -> bool {
        point.x >= self.x0 && point.x < self.x1 && point.y >= self.y0 && point.y < self.y1
    }

    /// Take absolute value of width and height.
    ///
    /// The resulting rect has the same extents as the original, but is
    /// guaranteed to have non-negative width and height.
    #[inline]
    pub fn abs(&self) -> Rect {
        let Rect { x0, y0, x1, y1 } = *self;
        Rect::new(x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1))
    }

    /// The smallest rectangle enclosing two rectangles.
    ///
    /// Results are valid only if width and height are non-negative.
    #[inline]
    pub fn union(&self, other: Rect) -> Rect {
        Rect::new(
            self.x0.min(other.x0),
            self.y0.min(other.y0),
            self.x1.max(other.x1),
            self.y1.max(other.y1),
        )
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
        Rect::new(x0, y0, x1.max(x0), y1.max(y0))
    }

    /// Expand a rectangle by a constant amount in both directions.
    ///
    /// The logic simply applies the amount in each direction. If rectangle
    /// area or added dimensions are negative, this could give odd results.
    pub fn inflate(&self, width: f64, height: f64) -> Rect {
        Rect::new(
            self.x0 - width,
            self.y0 - height,
            self.x1 + width,
            self.y1 + height,
        )
    }

    /// Returns a new `Rect`,
    /// with each coordinate value rounded to the nearest integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let rect = Rect::new(3.3, 3.6, 3.0, -3.1).round();
    /// assert_eq!(rect.x0, 3.0);
    /// assert_eq!(rect.y0, 4.0);
    /// assert_eq!(rect.x1, 3.0);
    /// assert_eq!(rect.y1, -3.0);
    /// ```
    #[inline]
    pub fn round(self) -> Rect {
        Rect::new(
            self.x0.round(),
            self.y0.round(),
            self.x1.round(),
            self.y1.round(),
        )
    }

    /// Returns a new `Rect`,
    /// with each coordinate value rounded up to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let rect = Rect::new(3.3, 3.6, 3.0, -3.1).ceil();
    /// assert_eq!(rect.x0, 4.0);
    /// assert_eq!(rect.y0, 4.0);
    /// assert_eq!(rect.x1, 3.0);
    /// assert_eq!(rect.y1, -3.0);
    /// ```
    #[inline]
    pub fn ceil(self) -> Rect {
        Rect::new(
            self.x0.ceil(),
            self.y0.ceil(),
            self.x1.ceil(),
            self.y1.ceil(),
        )
    }

    /// Returns a new `Rect`,
    /// with each coordinate value rounded down to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let rect = Rect::new(3.3, 3.6, 3.0, -3.1).floor();
    /// assert_eq!(rect.x0, 3.0);
    /// assert_eq!(rect.y0, 3.0);
    /// assert_eq!(rect.x1, 3.0);
    /// assert_eq!(rect.y1, -4.0);
    /// ```
    #[inline]
    pub fn floor(self) -> Rect {
        Rect::new(
            self.x0.floor(),
            self.y0.floor(),
            self.x1.floor(),
            self.y1.floor(),
        )
    }

    /// Returns a new `Rect`,
    /// with each coordinate value rounded away from zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let rect = Rect::new(3.3, 3.6, 3.0, -3.1).expand();
    /// assert_eq!(rect.x0, 4.0);
    /// assert_eq!(rect.y0, 4.0);
    /// assert_eq!(rect.x1, 3.0);
    /// assert_eq!(rect.y1, -4.0);
    /// ```
    #[inline]
    pub fn expand(self) -> Rect {
        Rect::new(
            self.x0.expand(),
            self.y0.expand(),
            self.x1.expand(),
            self.y1.expand(),
        )
    }

    /// Returns a new `Rect`,
    /// with each coordinate value rounded towards zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Rect;
    /// let rect = Rect::new(3.3, 3.6, 3.0, -3.1).trunc();
    /// assert_eq!(rect.x0, 3.0);
    /// assert_eq!(rect.y0, 3.0);
    /// assert_eq!(rect.x1, 3.0);
    /// assert_eq!(rect.y1, -3.0);
    /// ```
    #[inline]
    pub fn trunc(self) -> Rect {
        Rect::new(
            self.x0.trunc(),
            self.y0.trunc(),
            self.x1.trunc(),
            self.y1.trunc(),
        )
    }

    /// Creates a new [`RoundedRect`] from this `Rect` and the provided
    /// corner radius.
    ///
    /// [`RoundedRect`]: struct.RoundedRect.html
    #[inline]
    pub fn to_rounded_rect(self, radius: f64) -> RoundedRect {
        RoundedRect::from_rect(self, radius)
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
        Rect::new(self.x0 + v.x, self.y0 + v.y, self.x1 + v.x, self.y1 + v.y)
    }
}

impl Sub<Vec2> for Rect {
    type Output = Rect;

    #[inline]
    fn sub(self, v: Vec2) -> Rect {
        Rect::new(self.x0 - v.x, self.y0 - v.y, self.x1 - v.x, self.y1 - v.y)
    }
}

impl Sub for Rect {
    type Output = Insets;

    #[inline]
    fn sub(self, other: Rect) -> Insets {
        let x0 = other.x0 - self.x0;
        let y0 = other.y0 - self.y0;
        let x1 = self.x1 - other.x1;
        let y1 = self.y1 - other.y1;
        Insets { x0, y0, x1, y1 }
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

impl fmt::Debug for Rect {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if f.alternate() {
            write!(
                f,
                "Rect {{ origin: {:?}, size: {:?} }}",
                self.origin(),
                self.size()
            )
        } else {
            write!(
                f,
                "Rect {{ x0 {:?}, y0: {:?}, x1: {:?}, y1: {:?} }}",
                self.x0, self.y0, self.x1, self.y1
            )
        }
    }
}

impl fmt::Display for Rect {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Rect {{ ")?;
        fmt::Display::fmt(&self.origin(), f)?;
        write!(f, " ")?;
        fmt::Display::fmt(&self.size(), f)?;
        write!(f, " }}")
    }
}

#[cfg(test)]
mod tests {
    use crate::{Point, Rect, Shape};

    fn assert_approx_eq(x: f64, y: f64) {
        assert!((x - y).abs() < 1e-7);
    }

    #[test]
    fn area_sign() {
        let r = Rect::new(0.0, 0.0, 10.0, 10.0);
        let center = r.center();
        assert_approx_eq(r.area(), 100.0);

        assert_eq!(r.winding(center), 1);

        let p = r.into_bez_path(1e-9);
        assert_approx_eq(r.area(), p.area());
        assert_eq!(r.winding(center), p.winding(center));

        let r_flip = Rect::new(0.0, 10.0, 10.0, 0.0);
        assert_approx_eq(r_flip.area(), -100.0);

        assert_eq!(r_flip.winding(Point::new(5.0, 5.0)), -1);
        let p_flip = r_flip.into_bez_path(1e-9);
        assert_approx_eq(r_flip.area(), p_flip.area());
        assert_eq!(r_flip.winding(center), p_flip.winding(center));
    }

    #[test]
    fn display() {
        let r = Rect::from_origin_size((10., 12.23214), (22.222222222, 23.1));
        assert_eq!(
            format!("{}", r),
            "Rect { (10, 12.23214) (22.222222222×23.1) }"
        );
        assert_eq!(format!("{:.2}", r), "Rect { (10.00, 12.23) (22.22×23.10) }");
    }
}
