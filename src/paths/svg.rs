//! This module provides type [`Path`] representing SVG path data, and associated types.
use crate::{PathEl as KurboPathEl, Point, Shape, Vec2};
use anyhow::{anyhow, Result};
use std::{fmt, ops::Deref, vec};

pub use self::one_vec::*;

/// An SVG path
///
/// A path *MUST* begin with a `MoveTo` element. For this and other invalid inputs, we return
/// an error value explaining the problem with the input.
///
/// Based on [the SVG path data specification](https://svgwg.org/specs/paths/#PathData).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Path {
    /// The elements that make up this path.
    elements: Vec<PathEl>,
}

/// An SVG path element
///
/// # Bearing and relative position
///
/// Relative path elements are affected by the end position of the previous path and the bearing.
/// You can think of the relative elements as being specified in the coordinate space that consists
/// of a translation to the previous point, followed by a rotation by the bearing. The bearing
/// rotation is at *0* along the *x* axis, and then proceeds in the positive *y* direction. In the
/// y-down SVG coordinate system, this correspond to clockwise rotation.
// TODO think about if we can store things on the stack more.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathEl {
    /// `M`: Start a new sub-path at the given point.
    ///
    /// Subsequent points indicate that straight lines should be drawn (equivalent to `LineTo`).
    /// This can be confusing, and only saves 1 character per polyline, but it is in
    /// [the spec](https://svgwg.org/specs/paths/#PathDataMovetoCommands).
    MoveTo(OneVec<Point>),
    /// `m`: Start a new sub-path at the given point, relative to the previous end point.
    ///
    /// If there is no previous end point (because this is the first element of the path), then the
    /// value is interpreted as an absolute location. The points are interpreted taking into account
    /// the bearing and previous position, as detailed in the [type documentation][PathEl].
    MoveToRel(OneVec<Point>),
    /// `z`, `Z`: Close the current subpath by connecting it back to the current subpath's initial
    /// point.
    ///
    /// This is one of the places where this struct is lossy - it will always output `Z` even if the
    /// input was `z`. These are both semantically equivalent.
    ClosePath,
    /// `L`: Draw a line from the current point to the given point.
    ///
    /// Each point is interpreted as the endpoint of another line.
    LineTo(OneVec<Point>),
    /// `l`: Draw a line from the current point to the given point, with the offsets given relative
    /// to the current position and bearing.
    ///
    /// Each offset is interpreted as another line. The points are interpreted taking into account
    /// the bearing and previous position, as detailed in the [type documentation][PathEl].
    LineToRel(OneVec<Point>),
    /// `H`: Draw a horizontal line with the given distance.
    ///
    /// Multiple values are interpreted as multiple horizontal lines.
    Horiz(OneVec<f64>),
    /// `h`: Draw a line with the given distance along the bearing direction.
    ///
    /// Multiple values are interpreted as multiple lines. The points are interpreted taking into
    /// account the bearing and previous position, as detailed in the [type documentation][PathEl].
    HorizRel(OneVec<f64>),
    /// `V`: Draw a vertical line with the given distance.
    ///
    /// Multiple values are interpreted as multiple lines.
    Vert(OneVec<f64>),
    /// `v`: Draw a line with the given distance at right angles to the bearing direction.
    ///
    /// Multiple values are interpreted as multiple lines. The points are interpreted taking into
    /// account the bearing and previous position, as detailed in the [type documentation][PathEl].
    VertRel(OneVec<f64>),
    /// `C`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`.
    CubicTo(OneVec<CubicTo>),
    /// `c`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`. The points are interpreted taking into account the bearing and previous position,
    /// as detailed in the [type documentation][PathEl].
    CubicToRel(OneVec<CubicTo>),
    /// `S`: Draw a smooth cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl2` is the control point nearest to `to`. The first control point is calculated from the
    /// previous input (see [`SmoothCubicTo`]).
    SmoothCubicTo(OneVec<SmoothCubicTo>),
    /// `s`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`. The first control point is calculated from the previous input (see
    /// [`SmoothCubicTo`]). Both the actual and inferred points are interpreted taking into account
    /// the bearing and previous position, as detailed in the [type documentation][PathEl].
    SmoothCubicToRel(OneVec<SmoothCubicTo>),
    /// `Q`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`.
    QuadTo(OneVec<QuadTo>),
    /// `q`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// `ctrl1` is the control point nearest to the start, and `ctrl2` is the control point nearest
    /// to `to`. The points are interpreted taking into account the bearing and previous position,
    /// as detailed in the [type documentation][PathEl].
    QuadToRel(OneVec<QuadTo>),
    /// `T`: Draw a smooth cubic Bezier curve from the current point to `to`.
    ///
    /// The control point is calculated from the previous input, either a reflection of the previous
    /// control point, or the start point if this is the first segment, or the previous segment was
    /// not a quadratic Bezier.
    SmoothQuadTo(OneVec<Point>),
    /// `t`: Draw a cubic Bezier curve from the current point to `to`.
    ///
    /// The control point is calculated from the previous input, either a reflection of the previous
    /// control point, or the start point if this is the first segment, or the previous segment was
    /// not a quadratic Bezier. Both the actual and inferred points are interpreted taking into
    /// account the bearing and previous position, as detailed in the [type documentation][PathEl].
    SmoothQuadToRel(OneVec<Point>),
    /// `A`: Draw an elliptical arc.
    ///
    /// See the documentation for [`Arc`] for more details.
    EllipticArc(OneVec<Arc>),
    /// `a`: Draw an elliptical arc.
    ///
    /// See the documentation for [`Arc`] for more details. The points are interpreted taking into
    /// account the bearing and previous position, as detailed in the [type documentation][PathEl].
    /// In particular, this affects the x-axis rotation (which will be the sum of the given x-axis
    /// rotation and the bearing).
    EllipticArcRel(OneVec<Arc>),
    // TODO catmull-rom. These curves look interesting but I don't know anything about them and
    // so it makes sense to tackle them later.
    /// Set the bearing.
    ///
    /// This overwrites the existing bearing. This is another place where this implementation is
    /// lossy: the specification allows for multiple bearing commands in sequence, but if we
    /// encounter this we collapse them down to a single bearing. In the absolute case this means
    /// taking the last bearing and discarding the others.
    Bearing(f64),
    /// Set the bearing.
    ///
    /// This differs from `Bearing` in that it adds the parameter to the current bearing, rather
    /// than overwriting it. This is another place where this implementation is lossy: the
    /// specification allows for multiple bearing commands in sequence, but if we encounter this we
    /// collapse them down to a single bearing. In the relative case this means summing the
    /// bearings (mod 2pi).
    BearingRel(f64),
}

/// The parameters of a `CubicTo` or `CubicToRel` element.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CubicTo {
    /// The point that this curve ends at
    pub to: Point,
    /// The first control point (from the start)
    pub ctrl1: Point,
    /// The second control point (from the start)
    pub ctrl2: Point,
}

/// The parameters of a `SmoothCubicTo` or `CubicToRel` element.
///
/// The first control point is the reflection of the control point from the previous curve, or the
/// start point if there was no previous point, or the previous point was not a cubic Bezier.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SmoothCubicTo {
    /// The point that this curve ends at
    pub to: Point,
    /// The second control point (from the start)
    pub ctrl2: Point,
}

/// The parameters of a `QuadTo` or `QuadToRel` element.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct QuadTo {
    /// The point that this curve ends at
    pub to: Point,
    /// The control point
    pub ctrl: Point,
}

// Note: the SVG arc logic is heavily adapted from https://github.com/nical/lyon
/// An SVG arc segment.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Arc {
    /// The arc's end point.
    pub to: Point,
    /// The arc's radii, where the vector's x-component is the radius in the
    /// positive x direction after applying `x_rotation`.
    pub radii: Vec2,
    /// How much the arc is rotated, in radians.
    pub x_rotation: f64,
    /// Does this arc sweep through more than π radians?
    pub large_arc: bool,
    /// Determines if the arc should begin moving at positive angles.
    pub sweep: bool,
}

impl Path {
    /// Create a new path object.
    pub fn new() -> Self {
        Self { elements: vec![] }
    }

    /// Push an element onto the end of an array.
    ///
    /// All elements apart from `MoveTo` and `Bearing` must be preceeded by a `MoveTo`.
    pub fn push(&mut self, el: PathEl) -> Result<()> {
        // bearings and moveto are always allowed
        if let PathEl::MoveTo(_) = &el {
            self.elements.push(el);
            return Ok(());
        }
        if let PathEl::Bearing(_) = &el {
            self.elements.push(el);
            return Ok(());
        }

        // other elements are only allowed if they come after a moveto
        if self
            .elements
            .iter()
            .find(|el| matches!(*el, PathEl::MoveTo(_)))
            .is_none()
        {
            return Err(anyhow!(
                "all line and curve elements must be preceeded by a moveto"
            ));
        }

        self.elements.push(el);
        Ok(())
    }

    /// Write out a text representation of the string.
    pub fn write(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut iter = self.elements.iter();
        if let Some(el) = iter.next() {
            el.write(f)?;
        }
        for el in iter {
            write!(f, " ")?;
            el.write(f)?;
        }
        Ok(())
    }
}

impl Deref for Path {
    type Target = [PathEl];

    fn deref(&self) -> &[PathEl] {
        &self.elements
    }
}

impl Shape for Path {
    type PathElementsIter<'iter> = PathElementsIter<'iter>;

    fn path_elements(&self, tolerance: f64) -> Self::PathElementsIter<'_> {
        PathElementsIter {
            path: self,
            tolerance,
            path_start_point: Point::ZERO,
            current_point: Point::ZERO,
            current_bearing: 0.,
        }
    }

    fn area(&self) -> f64 {
        todo!()
    }

    fn perimeter(&self, accuracy: f64) -> f64 {
        self.elements.iter().map(PathEl::length).sum()
    }

    fn winding(&self, pt: Point) -> i32 {
        todo!()
    }

    fn bounding_box(&self) -> crate::Rect {
        todo!()
    }
}

impl PathEl {
    fn length(&self) -> f64 {
        todo!()
    }

    fn write(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            PathEl::MoveTo(points) => {
                write!(f, "M")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::MoveToRel(points) => {
                write!(f, "m")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::ClosePath => {
                write!(f, "Z")?;
            }
            PathEl::LineTo(points) => {
                write!(f, "L")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::LineToRel(points) => {
                write!(f, "l")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::Horiz(amts) => {
                write!(f, "H")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::HorizRel(amts) => {
                write!(f, "h")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::Vert(amts) => {
                write!(f, "V")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::VertRel(amts) => {
                write!(f, "v")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::CubicTo(cubic_tos) => {
                write!(f, "C")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::CubicToRel(cubic_tos) => {
                write!(f, "c")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::SmoothCubicTo(cubic_tos) => {
                write!(f, "S")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::SmoothCubicToRel(cubic_tos) => {
                write!(f, "s")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::QuadTo(quad_tos) => {
                write!(f, "Q")?;
                quad_tos.write_spaced(QuadTo::write_vals, f)?;
            }
            PathEl::QuadToRel(quad_tos) => {
                write!(f, "q")?;
                quad_tos.write_spaced(QuadTo::write_vals, f)?;
            }
            PathEl::SmoothQuadTo(quad_tos) => {
                write!(f, "T")?;
                quad_tos.write_spaced(write_point, f)?;
            }
            PathEl::SmoothQuadToRel(quad_tos) => {
                write!(f, "t")?;
                quad_tos.write_spaced(write_point, f)?;
            }
            PathEl::EllipticArc(_) => todo!(),
            PathEl::EllipticArcRel(_) => todo!(),
            PathEl::Bearing(bearing) => {
                write!(f, "B{bearing}",)?;
            }
            PathEl::BearingRel(bearing) => {
                write!(f, "b{bearing}",)?;
            }
        }
        Ok(())
    }
}

impl CubicTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{} {},{}",
            self.ctrl1.x, self.ctrl1.y, self.ctrl2.x, self.ctrl2.y, self.to.x, self.to.y
        )
    }
}

impl SmoothCubicTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{}",
            self.ctrl2.x, self.ctrl2.y, self.to.x, self.to.y
        )
    }
}

impl QuadTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{}",
            self.ctrl.x, self.ctrl.y, self.to.x, self.to.y
        )
    }
}

/// An iterator over the path elements of an SVG path.
///
/// This structure could be `Copy`, but we don't implement it to avoid hidden clones.
#[derive(Clone)]
pub struct PathElementsIter<'iter> {
    /// The path we are traversing.
    path: &'iter [PathEl],
    /// Tolerance parameter
    tolerance: f64,
    /// The start point of the current sub-path (this resets for every `MoveTo`).
    path_start_point: Point,
    /// The end point of the previous segment.
    current_point: Point,
    /// The current bearing.
    current_bearing: f64,
}

impl<'iter> Iterator for PathElementsIter<'iter> {
    type Item = KurboPathEl;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

fn write_point(Point { x, y }: &Point, f: &mut fmt::Formatter) -> fmt::Result {
    write!(f, "{x},{y}")
}

mod one_vec {
    use anyhow::anyhow;
    use std::{fmt, iter, slice, vec};

    /// A vector that has at least 1 element.
    ///
    /// It stores the first element on the stack, and the rest on the heap.
    #[derive(Debug, Clone)]
    pub struct OneVec<T> {
        pub first: T,
        pub rest: Vec<T>,
    }

    impl<T> OneVec<T> {
        pub fn single(val: T) -> Self {
            Self {
                first: val,
                rest: vec![],
            }
        }

        pub fn iter(&self) -> iter::Chain<iter::Once<&T>, slice::Iter<'_, T>> {
            self.into_iter()
        }

        /// Write out the vector with spaces between each element
        pub(crate) fn write_spaced(
            &self,
            mut cb: impl FnMut(&T, &mut fmt::Formatter) -> fmt::Result,
            f: &mut fmt::Formatter,
        ) -> fmt::Result {
            cb(&self.first, f)?;
            for v in &self.rest {
                cb(v, f)?;
            }
            Ok(())
        }
    }

    impl<T> TryFrom<Vec<T>> for OneVec<T> {
        type Error = anyhow::Error;
        fn try_from(mut v: Vec<T>) -> Result<Self, Self::Error> {
            // Annoyingly the `Vec::remove` method can panic, so we have to check
            // the vec is non-empty
            if v.is_empty() {
                return Err(anyhow!("vector must not be empty"));
            }
            let first = v.remove(0);
            Ok(OneVec { first, rest: v })
        }
    }

    impl<'a, T> IntoIterator for &'a OneVec<T> {
        type IntoIter = iter::Chain<iter::Once<&'a T>, slice::Iter<'a, T>>;
        type Item = &'a T;

        fn into_iter(self) -> Self::IntoIter {
            iter::once(&self.first).chain(&self.rest)
        }
    }

    impl<T> IntoIterator for OneVec<T> {
        type IntoIter = iter::Chain<iter::Once<T>, vec::IntoIter<T>>;
        type Item = T;

        fn into_iter(self) -> Self::IntoIter {
            iter::once(self.first).chain(self.rest)
        }
    }
}
