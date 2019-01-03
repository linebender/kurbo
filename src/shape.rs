//! A generic trait for shapes.

use crate::{BezPath, PathEl, Rect, Vec2};

/// A generic trait for closed shapes.
pub trait Shape {
    /// The iterator resulting from `to_bez_path`.
    ///
    /// TODO: When GAT's land, the type of this can be changed to
    /// contain a `&'a self` reference, which would let us take
    /// iterators from complex shapes without cloning.
    type BezPathIter: Iterator<Item = PathEl>;

    /// Convert to a Bézier path.
    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter;

    /// Convert to an owned Bézier path.
    ///
    /// This might be less copying, and less computation in the
    /// iterator. I'm not yet sure this method is worth having.
    fn to_owned_bez_path(&self, tolerance: f64) -> BezPath {
        BezPath::from_vec(self.to_bez_path(tolerance).collect())
    }

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

    // TODO: centroid would be a good method to add.

    /// If the shape is a rectangle, report that.
    fn as_rect(&self) -> Option<Rect> {
        None
    }

    // TODO: we'll have as_circle and probably as_rounded_rect,
    // as it's likely renderers will special-case on those.
}
