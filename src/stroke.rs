// Copyright 2023 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::borrow::Borrow;

use smallvec::SmallVec;

use crate::{
    fit_to_bezpath, offset::CubicOffset, BezPath, CubicBez, PathEl, PathSeg, Point, QuadBez, Vec2,
};

/// Defines the connection between two segments of a stroke.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Join {
    /// A straight line connecting the segments.
    Bevel,
    /// The segments are extended to their natural intersection point.
    Miter,
    /// An arc between the segments.
    Round,
}

/// Defines the shape to be drawn at the ends of a stroke.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Cap {
    /// Flat cap.
    Butt,
    /// Square cap with dimensions equal to half the stroke width.
    Square,
    /// Rounded cap with radius equal to half the stroke width.
    Round,
}

/// Describes the visual style of a stroke.
#[derive(Clone, Debug)]
pub struct Stroke {
    /// Width of the stroke.
    pub width: f64,
    /// Style for connecting segments of the stroke.
    pub join: Join,
    /// Limit for miter joins.
    pub miter_limit: f64,
    /// Style for capping the beginning of an open subpath.
    pub start_cap: Cap,
    /// Style for capping the end of an open subpath.
    pub end_cap: Cap,
    /// Lengths of dashes in alternating on/off order.
    pub dash_pattern: Dashes,
    /// Offset of the first dash.
    pub dash_offset: f64,
    /// True if the stroke width should be affected by the scale of a
    /// transform.
    ///
    /// Discussion question: does this make sense here?
    pub scale: bool,
}

impl Default for Stroke {
    fn default() -> Self {
        Self {
            width: 1.0,
            join: Join::Round,
            miter_limit: 4.0,
            start_cap: Cap::Round,
            end_cap: Cap::Round,
            dash_pattern: Default::default(),
            dash_offset: 0.0,
            scale: true,
        }
    }
}

impl Stroke {
    /// Creates a new stroke with the specified width.
    pub fn new(width: f64) -> Self {
        Self {
            width,
            ..Default::default()
        }
    }

    /// Builder method for setting the join style.
    pub fn with_join(mut self, join: Join) -> Self {
        self.join = join;
        self
    }

    /// Builder method for setting the limit for miter joins.
    pub fn with_miter_limit(mut self, limit: f64) -> Self {
        self.miter_limit = limit;
        self
    }

    /// Builder method for setting the cap style for the start of the stroke.
    pub fn with_start_cap(mut self, cap: Cap) -> Self {
        self.start_cap = cap;
        self
    }

    /// Builder method for setting the cap style for the end of the stroke.
    pub fn with_end_cap(mut self, cap: Cap) -> Self {
        self.end_cap = cap;
        self
    }

    /// Builder method for setting the cap style.
    pub fn with_caps(mut self, cap: Cap) -> Self {
        self.start_cap = cap;
        self.end_cap = cap;
        self
    }

    /// Builder method for setting the dashing parameters.
    pub fn with_dashes<P>(mut self, offset: f64, pattern: P) -> Self
    where
        P: IntoIterator,
        P::Item: Borrow<f64>,
    {
        self.dash_offset = offset;
        self.dash_pattern.clear();
        self.dash_pattern
            .extend(pattern.into_iter().map(|dash| *dash.borrow()));
        self
    }

    /// Builder method for setting whether or not the stroke should be affected
    /// by the scale of any applied transform.
    pub fn with_scale(mut self, yes: bool) -> Self {
        self.scale = yes;
        self
    }
}

/// Collection of values representing lengths in a dash pattern.
pub type Dashes = SmallVec<[f64; 4]>;

/// Internal structure used for creating strokes.
#[derive(Default)]
struct StrokeCtx {
    // Probably don't need both output and forward, can just concat
    output: BezPath,
    forward_path: BezPath,
    backward_path: BezPath,
    last_pt: Point,
    last_tan: Vec2,
}

/// Expand a stroke into a fill.
pub fn stroke(path: impl IntoIterator<Item = PathEl>, style: &Stroke, tolerance: f64) -> BezPath {
    let mut ctx = StrokeCtx::default();
    for el in path {
        let p0 = ctx.last_pt;
        match el {
            PathEl::MoveTo(p) => {
                ctx.finish();
                ctx.last_pt = p;
            }
            PathEl::LineTo(p1) => {
                if p1 != ctx.last_pt {
                    let tangent = p1 - p0;
                    ctx.do_tangents(style, tangent, tangent, p1);
                    ctx.do_line(style, tangent, p1);
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != p0 && p2 != p0 {
                    let q = QuadBez::new(p0, p1, p2);
                    let (tan0, tan1) = PathSeg::Quad(q).tangents();
                    ctx.do_tangents(style, tan0, tan1, p2);
                    ctx.do_cubic(style, q.raise(), tolerance);
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != p0 && p2 != p0 && p3 != p0 {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
                    ctx.do_tangents(style, tan0, tan1, p3);
                    ctx.do_cubic(style, c, tolerance);
                }
            }
            _ => todo!(),
        }
    }
    todo!()
}

fn get_end(el: &PathEl) -> Point {
    match el {
        PathEl::MoveTo(p) => *p,
        PathEl::LineTo(p1) => *p1,
        PathEl::QuadTo(_, p2) => *p2,
        PathEl::CurveTo(_, _, p3) => *p3,
        _ => unreachable!(),
    }
}

impl StrokeCtx {
    /// Append forward and backward paths to output.
    fn finish(&mut self) {
        if self.forward_path.is_empty() {
            return;
        }
        self.output.extend(&self.forward_path);
        let back_els = self.backward_path.elements();
        // TODO: this is "butt" end, but we want to do other styles
        self.output.line_to(get_end(&back_els[back_els.len() - 1]));
        for i in (1..back_els.len()).rev() {
            let end = get_end(&back_els[i - 1]);
            match back_els[i] {
                PathEl::LineTo(_) => self.output.line_to(end),
                PathEl::QuadTo(p1, _) => self.output.quad_to(p1, end),
                PathEl::CurveTo(p1, p2, _) => self.output.curve_to(p2, p1, end),
                _ => unreachable!(),
            }
        }
        // Same, this is butt end
        self.output.close_path();

        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
    }

    fn do_tangents(&mut self, style: &Stroke, tan0: Vec2, tan1: Vec2, p1: Point) {
        let scale = 0.5 * style.width / tan0.hypot();
        let norm = scale * Vec2::new(-tan0.y, tan0.x);
        if self.forward_path.is_empty() {
            self.forward_path.move_to(p1 + norm);
            self.backward_path.move_to(p1 - norm);
        } else {
            // TODO: this represents miter joins, handle other styles
            self.forward_path.line_to(p1 + norm);
            self.backward_path.line_to(p1 - norm);
        }
        self.last_tan = tan1;
    }

    fn do_line(&mut self, style: &Stroke, tangent: Vec2, p1: Point) {
        let scale = 0.5 * style.width / tangent.hypot();
        let norm = scale * Vec2::new(-tangent.y, tangent.x);
        self.forward_path.line_to(p1 + norm);
        self.backward_path.line_to(p1 - norm);
        self.last_pt = p1;
    }

    fn do_cubic(&mut self, style: &Stroke, c: CubicBez, tolerance: f64) {
        let co = CubicOffset::new(c, 0.5 * style.width);
        let forward = fit_to_bezpath(&co, tolerance);
        self.forward_path.extend(forward.into_iter().skip(1));
        let co = CubicOffset::new(c, -0.5 * style.width);
        let backward = fit_to_bezpath(&co, tolerance);
        self.backward_path.extend(backward.into_iter().skip(1));
        self.last_pt = c.p3;
    }
}
