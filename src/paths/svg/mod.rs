//! This module provides type [`Path`] representing SVG path data, and associated types.
//!
//! In addition to providing SVG path support, this path type serves as a proof-of-concept and
//! template for custom path types that implement [`kurbo::Shape`], thereby allowing it to be drawn
//! by any rendering engine that supports [`BezPath`][kurbo::BezPath] rendering.
use crate::{Affine, PathEl as KurboPathEl, Point, Rect, Shape, Vec2};
use anyhow::{anyhow, ensure, Result};
use std::{f64::consts::PI, io, mem, ops::Deref, vec};

mod onevec;
mod string_repr;

use self::onevec::*;
pub use self::string_repr::{parse, SvgParseError};

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
    /// A cache to record we've seen a moveto
    has_moveto: bool,
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
    EllipticArc(OneVec<ArcTo>),
    /// `a`: Draw an elliptical arc.
    ///
    /// See the documentation for [`Arc`] for more details. The points are interpreted taking into
    /// account the bearing and previous position, as detailed in the [type documentation][PathEl].
    /// In particular, this affects the x-axis rotation (which will be the sum of the given x-axis
    /// rotation and the bearing).
    EllipticArcRel(OneVec<ArcTo>),
    // TODO catmull-rom. These curves look interesting but I don't know anything about them and
    // so it makes sense to tackle them later. Update: they are based on defining the location and
    // derivative at a set of points.
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
pub struct ArcTo {
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
        Self {
            elements: vec![],
            has_moveto: false,
        }
    }

    /// Create a new path object with space for `length` path elements.
    pub fn with_capacity(length: usize) -> Self {
        Self {
            elements: Vec::with_capacity(length),
            has_moveto: false,
        }
    }

    /// Create a new path object from a vec of elements.
    ///
    /// This function will return an error if the resulting path would be invalid.
    pub fn from_elements(elements: Vec<PathEl>) -> Result<Self> {
        let mut this = Self {
            elements,
            has_moveto: false,
        };
        this.validate()?;
        Ok(this)
    }

    /// Try to parse a string as an SVG path.
    pub fn parse(input: &str) -> Result<Self, SvgParseError> {
        parse(input)
    }

    /// Push an element onto the end of an array.
    ///
    /// All elements apart from `MoveTo` and `Bearing` must be preceeded by a `MoveTo`.
    pub fn push(&mut self, el: PathEl) -> Result<()> {
        // bearings and moveto are always allowed
        if matches!(&el, PathEl::MoveTo(_)) {
            self.elements.push(el);
            self.has_moveto = true;
            return Ok(());
        }
        if matches!(&el, PathEl::Bearing(_)) {
            self.elements.push(el);
            return Ok(());
        }

        // other elements are only allowed if they come after a moveto
        if !self.has_moveto {
            return Err(anyhow!(
                "all line and curve elements must be preceeded by a moveto"
            ));
        }

        self.elements.push(el);
        Ok(())
    }

    /// Write out the text representation of this path to anything implementing `io::Write`.
    ///
    /// `Path` also implements [`Display`][std::fmt::Display], which can be used when you need an
    /// in-memory string representation (so you can e.g. `path.to_string()`).
    ///
    /// Note that this call will produce a lot of write calls under the hood, so it is recommended
    /// to use a buffer (e.g. [`BufWriter`][std::io::BufWriter]) if your writer's
    /// [`write`][io::Write::write] calls are expensive.
    pub fn write_to(&self, mut w: impl io::Write) -> io::Result<()> {
        write!(w, "{}", self)
    }

    /// Returns an error if the path is invalid
    ///
    /// Used to check invariants when a path is constructed from a list of path elemenets.
    fn validate(&mut self) -> Result<()> {
        let Path {
            ref mut has_moveto,
            ref mut elements,
        } = self;
        for (idx, el) in elements.into_iter().enumerate() {
            match el {
                PathEl::MoveTo(_) => {
                    *has_moveto = true;
                }
                PathEl::Bearing(_) => {
                    // fall through
                }
                _ => {
                    // all other elements must be preceeded by moveto
                    ensure!(
                        *has_moveto,
                        "path element {el:?} at index {idx} was not preceeded by a \
                            `PathEl::MoveTo(_)`"
                    )
                }
            }
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

impl TryFrom<Vec<PathEl>> for Path {
    type Error = anyhow::Error;

    fn try_from(value: Vec<PathEl>) -> Result<Self, Self::Error> {
        Path::from_elements(value)
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
            state: IterState::None,
            previous_cubic: None,
            previous_quad: None,
            seen_moveto: false,
        }
    }

    fn area(&self) -> f64 {
        todo!()
    }

    fn perimeter(&self, _accuracy: f64) -> f64 {
        todo!()
    }

    fn winding(&self, _pt: Point) -> i32 {
        todo!()
    }

    fn bounding_box(&self) -> Rect {
        todo!()
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
    #[allow(unused)]
    tolerance: f64,
    /// The start point of the current sub-path (this resets for every `MoveTo`).
    path_start_point: Point,
    /// The end point of the previous segment.
    current_point: Point,
    /// The current bearing.
    current_bearing: f64,
    /// This flag tracks whether we have seen a moveto command yet. It affects the behavior of
    /// `MoveToRel`.
    seen_moveto: bool,
    /// If there was a previous cubic bezier value, store its ctrl2 here.
    ///
    /// The point is ctrl2 in absolute coordinates (since this is all that's needed for smooth
    /// cubics).
    previous_cubic: Option<Point>,
    /// If there was a previous quad bezier value, store its ctrl here.
    ///
    /// Format is ctrl in absolute coordinates.
    previous_quad: Option<Point>,
    /// Iterator state machine
    state: IterState<'iter>,
}

#[derive(Clone)]
enum IterState<'iter> {
    /// We aren't part-way through any data type.
    None,
    /// We're in the middle of a lineto or moveto.
    ///
    /// These values are actually for drawing lines to. See the spec for details of why this is the
    /// state for a `MoveTo` as well.
    LineTo {
        transform: bool,
        rest: &'iter [Point],
    },
    /// Horizontal lines with the given distance
    HorizTo { transform: bool, rest: &'iter [f64] },
    /// Vertical lines with the given distance
    VertTo { transform: bool, rest: &'iter [f64] },
    /// Cubic Beziers
    CubicTo {
        transform: bool,
        rest: &'iter [CubicTo],
    },
    /// Smooth cubic Beziers
    SmoothCubicTo {
        transform: bool,
        rest: &'iter [SmoothCubicTo],
    },
    /// Quad Beziers
    QuadTo {
        transform: bool,
        rest: &'iter [QuadTo],
    },
    /// Smooth quad Beziers
    SmoothQuadTo {
        transform: bool,
        rest: &'iter [Point],
    },
}

impl<'iter> PathElementsIter<'iter> {
    /// Handle the next element
    ///
    /// Only call this if we finished handling the previous one (i.e. `state = IterState::None`).
    fn next_el(&mut self, el: &'iter PathEl) -> Option<KurboPathEl> {
        match el {
            PathEl::MoveTo(points) => {
                self.seen_moveto = true;
                let (first, rest) = points.split();
                self.current_point = *first;
                self.path_start_point = *first;
                self.state = IterState::LineTo {
                    transform: false,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::MoveTo(*first))
            }
            PathEl::MoveToRel(points) => {
                let (&first, rest) = points.split();
                self.reset_prev_beziers();
                if self.seen_moveto {
                    let first = self.transform() * first;
                    self.current_point = first;
                    self.path_start_point = first;
                    self.state = IterState::LineTo {
                        transform: true,
                        rest,
                    };
                    Some(KurboPathEl::MoveTo(first))
                } else {
                    self.seen_moveto = true;
                    self.current_point = first;
                    self.path_start_point = first;
                    // Even though we treat the first element as absolute, we still treat
                    // subsequent points as `LineToRel`s. This might make a difference if the
                    // user added a `Bearing` before the first `MoveTo`.
                    self.state = IterState::LineTo {
                        transform: true,
                        rest,
                    };
                    Some(KurboPathEl::MoveTo(first))
                }
            }
            PathEl::ClosePath => {
                self.current_point = self.path_start_point;
                Some(KurboPathEl::ClosePath)
            }
            PathEl::LineTo(points) => {
                let (&first, rest) = points.split();
                self.current_point = first;
                self.state = IterState::LineTo {
                    transform: false,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::LineToRel(points) => {
                let (&first, rest) = points.split();
                let first = self.transform() * first;
                self.current_point = first;
                self.state = IterState::LineTo {
                    transform: true,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::Horiz(dists) => {
                let (&first, rest) = dists.split();
                let first = Point::new(first, 0.);
                self.current_point = first;
                self.state = IterState::HorizTo {
                    transform: false,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::HorizRel(dists) => {
                let (&first, rest) = dists.split();
                let first = self.transform() * Point::new(first, 0.);
                self.current_point = first;
                self.state = IterState::HorizTo {
                    transform: true,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::Vert(dists) => {
                let (&first, rest) = dists.split();
                let first = Point::new(0., first);
                self.current_point = first;
                self.state = IterState::VertTo {
                    transform: false,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::VertRel(dists) => {
                let (&first, rest) = dists.split();
                let first = self.transform() * Point::new(0., first);
                self.current_point = first;
                self.state = IterState::VertTo {
                    transform: true,
                    rest,
                };
                self.reset_prev_beziers();
                Some(KurboPathEl::LineTo(first))
            }
            PathEl::CubicTo(cubics) => {
                let (CubicTo { to, ctrl1, ctrl2 }, rest) = cubics.split();
                self.current_point = *to;
                self.state = IterState::CubicTo {
                    transform: false,
                    rest,
                };
                self.previous_quad = None;
                self.previous_cubic = Some(*ctrl2);
                Some(KurboPathEl::CurveTo(*ctrl1, *ctrl2, *to))
            }
            PathEl::CubicToRel(cubics) => {
                let (CubicTo { to, ctrl1, ctrl2 }, rest) = cubics.split();
                let (to, ctrl1, ctrl2) = {
                    let trans = self.transform();
                    (trans * *to, trans * *ctrl1, trans * *ctrl2)
                };
                self.current_point = to;
                self.state = IterState::CubicTo {
                    transform: true,
                    rest,
                };
                self.previous_quad = None;
                self.previous_cubic = Some(ctrl2);
                Some(KurboPathEl::CurveTo(ctrl1, ctrl2, to))
            }
            PathEl::SmoothCubicTo(cubics) => {
                let (SmoothCubicTo { to, ctrl2 }, rest) = cubics.split();
                let ctrl1 = self.get_smooth_cubic_ctrl1();
                self.current_point = *to;
                self.state = IterState::SmoothCubicTo {
                    transform: false,
                    rest,
                };
                self.previous_quad = None;
                self.previous_cubic = Some(*ctrl2);
                Some(KurboPathEl::CurveTo(ctrl1, *ctrl2, *to))
            }
            PathEl::SmoothCubicToRel(cubics) => {
                let (SmoothCubicTo { to, ctrl2 }, rest) = cubics.split();

                let (to, ctrl2) = {
                    let trans = self.transform();
                    (trans * *to, trans * *ctrl2)
                };
                let ctrl1 = self.get_smooth_cubic_ctrl1();
                self.current_point = to;
                self.state = IterState::SmoothCubicTo {
                    transform: true,
                    rest,
                };
                self.previous_quad = None;
                self.previous_cubic = Some(ctrl2);
                Some(KurboPathEl::CurveTo(ctrl1, ctrl2, to))
            }
            PathEl::QuadTo(cubics) => {
                let (QuadTo { to, ctrl }, rest) = cubics.split();
                self.current_point = *to;
                self.state = IterState::QuadTo {
                    transform: false,
                    rest,
                };
                self.previous_quad = Some(*ctrl);
                self.previous_cubic = None;
                Some(KurboPathEl::QuadTo(*ctrl, *to))
            }
            PathEl::QuadToRel(cubics) => {
                let (QuadTo { to, ctrl }, rest) = cubics.split();
                let (to, ctrl) = {
                    let trans = self.transform();
                    (trans * *to, trans * *ctrl)
                };

                self.current_point = to;
                self.state = IterState::QuadTo {
                    transform: true,
                    rest,
                };
                self.previous_quad = Some(ctrl);
                self.previous_cubic = None;
                Some(KurboPathEl::QuadTo(ctrl, to))
            }
            PathEl::SmoothQuadTo(cubics) => {
                let (to, rest) = cubics.split();
                let ctrl = self.get_smooth_quad_ctrl();

                self.current_point = *to;
                self.state = IterState::SmoothQuadTo {
                    transform: false,
                    rest,
                };
                self.previous_quad = Some(ctrl);
                self.previous_cubic = None;
                Some(KurboPathEl::QuadTo(ctrl, *to))
            }
            PathEl::SmoothQuadToRel(cubics) => {
                let (to, rest) = cubics.split();
                let to = self.transform() * *to;
                let ctrl = self.get_smooth_quad_ctrl();

                self.current_point = to;
                self.state = IterState::SmoothQuadTo {
                    transform: true,
                    rest,
                };
                self.previous_quad = Some(ctrl);
                self.previous_cubic = None;
                Some(KurboPathEl::QuadTo(ctrl, to))
            }
            PathEl::EllipticArc(_) => todo!(),
            PathEl::EllipticArcRel(_) => todo!(),
            PathEl::Bearing(bearing) => {
                self.current_bearing = bearing.rem_euclid(2. * PI);
                None
            }
            PathEl::BearingRel(bearing_rel) => {
                self.current_bearing = (self.current_bearing + bearing_rel).rem_euclid(2. * PI);
                None
            }
        }
    }

    fn handle_state(&mut self, state: IterState<'iter>) -> Option<KurboPathEl> {
        match state {
            IterState::None => None,
            IterState::LineTo { transform, rest } => {
                let Some((first, rest)) = rest.split_first() else {
                    return None;
                };
                let mut first = *first;

                if transform {
                    first = self.transform() * first;
                }
                self.state = IterState::LineTo { transform, rest };
                self.current_point = first;
                Some(KurboPathEl::LineTo(first))
            }
            IterState::HorizTo { transform, rest } => {
                let Some((first, rest)) = rest.split_first() else {
                    return None;
                };
                let mut first = Point::new(*first, 0.);

                if transform {
                    first = self.transform() * first;
                }
                self.state = IterState::HorizTo { transform, rest };
                self.current_point = first;
                Some(KurboPathEl::LineTo(first))
            }
            IterState::VertTo { transform, rest } => {
                let Some((first, rest)) = rest.split_first() else {
                    return None;
                };
                let mut first = Point::new(0., *first);

                if transform {
                    first = self.transform() * first;
                }
                self.state = IterState::VertTo { transform, rest };
                self.current_point = first;
                Some(KurboPathEl::LineTo(first))
            }
            IterState::CubicTo { transform, rest } => {
                let Some((CubicTo { ctrl1, ctrl2, to }, rest)) = rest.split_first() else {
                    return None;
                };
                let mut ctrl1 = *ctrl1;
                let mut ctrl2 = *ctrl2;
                let mut to = *to;
                if transform {
                    let trans = self.transform();
                    ctrl1 = trans * ctrl1;
                    ctrl2 = trans * ctrl2;
                    to = trans * to;
                }
                self.current_point = to;
                self.state = IterState::CubicTo { transform, rest };
                // no need to set quad as we already did it for the first element
                self.previous_cubic = Some(ctrl2);
                Some(KurboPathEl::CurveTo(ctrl1, ctrl2, to))
            }
            IterState::SmoothCubicTo { transform, rest } => {
                let Some((SmoothCubicTo { ctrl2, to }, rest)) = rest.split_first() else {
                    return None;
                };
                let ctrl1 = self.get_smooth_cubic_ctrl1();
                let mut ctrl2 = *ctrl2;
                let mut to = *to;
                if transform {
                    let trans = self.transform();
                    ctrl2 = trans * ctrl2;
                    to = trans * to;
                }

                self.current_point = to;
                self.state = IterState::SmoothCubicTo { transform, rest };
                // no need to set quad as we already did it for the first element
                self.previous_cubic = Some(ctrl2);
                Some(KurboPathEl::CurveTo(ctrl1, ctrl2, to))
            }
            IterState::QuadTo { transform, rest } => {
                let Some((QuadTo { ctrl, to }, rest)) = rest.split_first() else {
                    return None;
                };
                let mut ctrl = *ctrl;
                let mut to = *to;
                if transform {
                    let trans = self.transform();
                    ctrl = trans * ctrl;
                    to = trans * to;
                }
                self.current_point = to;
                self.state = IterState::QuadTo { transform, rest };
                // no need to set quad as we already did it for the first element
                self.previous_quad = Some(ctrl);
                Some(KurboPathEl::QuadTo(ctrl, to))
            }
            IterState::SmoothQuadTo { transform, rest } => {
                let Some((to, rest)) = rest.split_first() else {
                    return None;
                };
                let ctrl = self.get_smooth_quad_ctrl();
                let mut to = *to;
                if transform {
                    let trans = self.transform();
                    to = trans * to;
                }

                self.current_point = to;
                self.state = IterState::SmoothQuadTo { transform, rest };
                // no need to set quad as we already did it for the first element
                self.previous_quad = Some(ctrl);
                Some(KurboPathEl::QuadTo(ctrl, to))
            }
        }
    }

    /// Get the transform for the current start position and heading.
    fn transform(&self) -> Affine {
        // XXX I think this is correct, but not yet 100%
        Affine::translate(self.current_point.to_vec2()) * Affine::rotate(self.current_bearing)
    }

    fn reset_prev_beziers(&mut self) {
        self.previous_cubic = None;
        self.previous_quad = None;
    }

    /// The ctrl1 of a smooth bezier is the reflection of the previous ctrl2 through the previous
    /// endpoint, or just the previous endpoint if the previous segment wasn't a cubic.
    fn get_smooth_cubic_ctrl1(&self) -> Point {
        let Some(ctrl2) = self.previous_cubic else {
            return self.current_point;
        };
        Point {
            x: 2. * self.current_point.x - ctrl2.x,
            y: 2. * self.current_point.y - ctrl2.y,
        }
    }

    /// The ctrl of a smooth quad is the reflection of the previous ctrl through the previous
    /// endpoint, or just the previous endpoint if the previous segment wasn't a quad.
    fn get_smooth_quad_ctrl(&self) -> Point {
        let Some(ctrl) = self.previous_quad else {
            return self.current_point;
        };
        Point {
            x: 2. * self.current_point.x - ctrl.x,
            y: 2. * self.current_point.y - ctrl.y,
        }
    }
}

impl<'iter> Iterator for PathElementsIter<'iter> {
    type Item = KurboPathEl;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // Remember to but the state back if necessary.
            let existing_state = mem::replace(&mut self.state, IterState::None);
            if let Some(el) = self.handle_state(existing_state) {
                return Some(el);
            }
            let Some((first, rest)) = self.path.split_first() else {
                // This break is hit if we have exhaused the vec of svg path els.
                break;
            };
            self.path = rest;
            if let Some(kurbo_path) = self.next_el(first) {
                return Some(kurbo_path);
            }
        }
        None
    }
}
