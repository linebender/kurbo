//! A generic trait for shapes.

use crate::{BezPath, Circle, Line, PathEl, Point, Rect, RoundedRect};

/// A generic trait for open and closed shapes.
pub trait Shape: Sized {
    /// The iterator resulting from `to_bez_path`.
    type BezPathIter: Iterator<Item = PathEl>;

    /// Convert to a Bézier path, as an iterator over path elements.
    ///
    /// Callers should exhaust the `as_` methods first, as those are
    /// likely to be more efficient; in the general case, this
    /// allocates.
    ///
    /// The `tolerance` parameter controls the accuracy of
    /// conversion of geometric primitives to Bézier curves, as
    /// curves such as circles cannot be represented exactly but
    /// only approximated. For drawing as in UI elements, a value
    /// of 0.1 is appropriate, as it is unlikely to be visible to
    /// the eye. For scientific applications, a smaller value
    /// might be appropriate. Note that in general the number of
    /// cubic Bézier segments scales as `tolerance ^ (-1/6)`.
    ///
    /// TODO: When [GAT's] land, the type of this can be changed to
    /// contain a `&'a self` reference, which would let us take
    /// iterators from complex shapes without cloning.
    ///
    /// [GAT's]: https://github.com/rust-lang/rust/issues/44265
    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter;

    /// Convert into a Bézier path.
    ///
    /// Currently, this always allocates. It is appropriate when
    /// the resulting path is to be retained.
    ///
    /// The `tolerance` parameter is the same as
    /// [`to_bez_path()`](#tymethod.to_bez_path).
    fn into_bez_path(self, tolerance: f64) -> BezPath {
        let vec = if let Some(slice) = self.as_path_slice() {
            Vec::from(slice)
        } else {
            self.to_bez_path(tolerance).collect()
        };
        BezPath::from_vec(vec)
    }

    /// Signed area.
    ///
    /// This method only produces meaningful results with closed shapes.
    ///
    /// The convention for positive area is that y increases when x is
    /// positive. Thus, it is clockwise when down is increasing y (the
    /// usual convention for graphics), and anticlockwise when
    /// up is increasing y (the usual convention for math).
    fn area(&self) -> f64;

    /// Total length of perimeter.
    fn perimeter(&self, accuracy: f64) -> f64;

    /// Winding number of point.
    ///
    /// This method only produces meaningful results with closed shapes.
    ///
    /// The sign of the winding number is consistent with that of [`area`],
    /// meaning it is +1 when the point is inside a positive area shape
    /// and -1 when it is inside a negative area shape. Of course, greater
    /// magnitude values are also possible when the shape is more complex.
    ///
    /// [`area`]: #tymethod.area
    fn winding(&self, pt: Point) -> i32;

    /// The smallest rectangle that encloses the shape.
    fn bounding_box(&self) -> Rect;

    /// If the shape is a line, make it available.
    fn as_line(&self) -> Option<Line> {
        None
    }

    /// If the shape is a rectangle, make it available.
    fn as_rect(&self) -> Option<Rect> {
        None
    }

    /// If the shape is a rounded rectangle, make it available.
    fn as_rounded_rect(&self) -> Option<RoundedRect> {
        None
    }

    /// If the shape is a circle, make it available.
    fn as_circle(&self) -> Option<Circle> {
        None
    }

    /// If the shape is stored as a slice of path elements, make
    /// that available.
    ///
    /// Note: when GAT's land, a method like `to_bez_path` would be
    /// able to iterate through the slice with no extra allocation,
    /// without making any assumption that storage is contiguous.
    fn as_path_slice(&self) -> Option<&[PathEl]> {
        None
    }

    // TODO: we'll have as_circle and probably as_rounded_rect,
    // as it's likely renderers will special-case on those.
}

/// Blanket implementation so `impl Shape` will accept owned or reference.
impl<'a, T: Shape> Shape for &'a T {
    type BezPathIter = T::BezPathIter;

    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter {
        (*self).to_bez_path(tolerance)
    }

    fn area(&self) -> f64 {
        (*self).area()
    }

    fn perimeter(&self, accuracy: f64) -> f64 {
        (*self).perimeter(accuracy)
    }

    fn winding(&self, pt: Point) -> i32 {
        (*self).winding(pt)
    }

    fn bounding_box(&self) -> Rect {
        (*self).bounding_box()
    }

    fn as_circle(&self) -> Option<Circle> {
        (*self).as_circle()
    }

    fn as_line(&self) -> Option<Line> {
        (*self).as_line()
    }

    fn as_rect(&self) -> Option<Rect> {
        (*self).as_rect()
    }

    fn as_rounded_rect(&self) -> Option<RoundedRect> {
        (*self).as_rounded_rect()
    }

    fn as_path_slice(&self) -> Option<&[PathEl]> {
        (*self).as_path_slice()
    }
}
