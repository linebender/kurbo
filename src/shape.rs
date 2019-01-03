//! A generic trait for shapes.

use crate::{PathEl, Rect, Vec2};

/// A generic trait for closed shapes.
pub trait Shape {
    /// The iterator resulting from `to_bez_path`.
    type BezPathIter: Iterator<Item = PathEl>;

    /// Convert to a Bézier path, as an iterator over path elements.
    ///
    /// Callers should exhaust the `as_` methods first, as those are
    /// likely to be more efficient; in the general case, this
    /// allocates.
    ///
    /// TODO: When GAT's land, the type of this can be changed to
    /// contain a `&'a self` reference, which would let us take
    /// iterators from complex shapes without cloning.
    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter;

    /// Signed area.
    ///
    /// TODO: figure out sign convention, see #4.
    fn area(&self) -> f64;

    /// Total length of perimeter.
    fn perimeter(&self, accuracy: f64) -> f64;

    /// Winding number of point.
    ///
    /// TODO: figure out sign convention, see #4.
    fn winding(&self, pt: Vec2) -> i32;

    /// The smallest rectangle that encloses the shape.
    fn bounding_box(&self) -> Rect;

    // TODO: centroid would be a good method to add.

    /// If the shape is a rectangle, make it available.
    fn as_rect(&self) -> Option<Rect> {
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
