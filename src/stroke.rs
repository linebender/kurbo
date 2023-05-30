// Copyright 2023 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::{borrow::Borrow, f64::consts::PI};

use smallvec::SmallVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{
    fit_to_bezpath, offset::CubicOffset, Affine, Arc, BezPath, CubicBez, PathEl, PathSeg, Point,
    QuadBez, Vec2,
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
    start_pt: Point,
    start_norm: Vec2,
    start_tan: Vec2,
    last_pt: Point,
    last_tan: Vec2,
    // if hypot < (hypot + dot) * bend_thresh, omit join altogether
    join_thresh: f64,
}

/// Expand a stroke into a fill.
pub fn stroke(path: impl IntoIterator<Item = PathEl>, style: &Stroke, tolerance: f64) -> BezPath {
    let mut ctx = StrokeCtx::default();
    ctx.join_thresh = 2.0 * tolerance / style.width;
    for el in path {
        let p0 = ctx.last_pt;
        match el {
            PathEl::MoveTo(p) => {
                ctx.finish(style);
                ctx.start_pt = p;
                ctx.last_pt = p;
            }
            PathEl::LineTo(p1) => {
                if p1 != p0 {
                    let tangent = p1 - p0;
                    ctx.do_join(style, tangent);
                    ctx.last_tan = tangent;
                    ctx.do_line(style, tangent, p1);
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != p0 && p2 != p0 {
                    let q = QuadBez::new(p0, p1, p2);
                    let (tan0, tan1) = PathSeg::Quad(q).tangents();
                    ctx.do_join(style, tan0);
                    ctx.last_tan = tan1;
                    ctx.do_cubic(style, q.raise(), tolerance);
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != p0 && p2 != p0 && p3 != p0 {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
                    ctx.do_join(style, tan0);
                    ctx.last_tan = tan1;
                    ctx.do_cubic(style, c, tolerance);
                }
            }
            PathEl::ClosePath => {
                if p0 != ctx.start_pt {
                    let tangent = ctx.start_pt - p0;
                    ctx.do_join(style, tangent);
                    ctx.last_tan = tangent;
                    ctx.do_line(style, tangent, ctx.start_pt);
                }
                ctx.finish_closed(style);
            }
        }
    }
    ctx.finish(style);
    ctx.output
}

fn round_cap(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2) {
    round_join(out, tolerance, center, norm, PI);
}

fn round_join(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2, angle: f64) {
    let a = Affine::new([norm.x, norm.y, -norm.y, norm.x, center.x, center.y]);
    let arc = Arc::new(Point::ORIGIN, (1.0, 1.0), PI - angle, angle, 0.0);
    arc.to_cubic_beziers(tolerance, |p1, p2, p3| out.curve_to(a * p1, a * p2, a * p3));
}

fn round_join_rev(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2, angle: f64) {
    let a = Affine::new([norm.x, norm.y, norm.y, -norm.x, center.x, center.y]);
    let arc = Arc::new(Point::ORIGIN, (1.0, 1.0), PI - angle, angle, 0.0);
    arc.to_cubic_beziers(tolerance, |p1, p2, p3| out.curve_to(a * p1, a * p2, a * p3));
}

fn square_cap(out: &mut BezPath, close: bool, center: Point, norm: Vec2) {
    let a = Affine::new([norm.x, norm.y, -norm.y, norm.x, center.x, center.y]);
    out.line_to(a * Point::new(1.0, 1.0));
    out.line_to(a * Point::new(-1.0, 1.0));
    if close {
        out.close_path();
    } else {
        out.line_to(a * Point::new(-1.0, 0.0));
    }
}

fn extend_reversed(out: &mut BezPath, elements: &[PathEl]) {
    for i in (1..elements.len()).rev() {
        let end = elements[i - 1].end_point().unwrap();
        match elements[i] {
            PathEl::LineTo(_) => out.line_to(end),
            PathEl::QuadTo(p1, _) => out.quad_to(p1, end),
            PathEl::CurveTo(p1, p2, _) => out.curve_to(p2, p1, end),
            _ => unreachable!(),
        }
    }
}

impl StrokeCtx {
    /// Append forward and backward paths to output.
    fn finish(&mut self, style: &Stroke) {
        // TODO: scale
        let tolerance = 1e-3;
        if self.forward_path.is_empty() {
            return;
        }
        self.output.extend(&self.forward_path);
        let back_els = self.backward_path.elements();
        let return_p = back_els[back_els.len() - 1].end_point().unwrap();
        let d = self.last_pt - return_p;
        match style.end_cap {
            Cap::Butt => self.output.line_to(return_p),
            Cap::Round => round_cap(&mut self.output, tolerance, self.last_pt, d),
            Cap::Square => square_cap(&mut self.output, false, self.last_pt, d),
        }
        extend_reversed(&mut self.output, back_els);
        match style.start_cap {
            Cap::Butt => self.output.close_path(),
            Cap::Round => round_cap(&mut self.output, tolerance, self.start_pt, self.start_norm),
            Cap::Square => square_cap(&mut self.output, true, self.start_pt, self.start_norm),
        }

        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
    }

    /// Finish a closed path
    fn finish_closed(&mut self, style: &Stroke) {
        self.do_join(style, self.start_tan);
        self.output.extend(&self.forward_path);
        self.output.close_path();
        let back_els = self.backward_path.elements();
        let last_pt = back_els[back_els.len() - 1].end_point().unwrap();
        self.output.move_to(last_pt);
        extend_reversed(&mut self.output, back_els);
        self.output.close_path();
        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
    }

    fn do_join(&mut self, style: &Stroke, tan0: Vec2) {
        // TODO: scale
        let tolerance = 1e-3;
        let scale = 0.5 * style.width / tan0.hypot();
        let norm = scale * Vec2::new(-tan0.y, tan0.x);
        let p0 = self.last_pt;
        if self.forward_path.is_empty() {
            self.forward_path.move_to(p0 - norm);
            self.backward_path.move_to(p0 + norm);
            self.start_tan = tan0;
            self.start_norm = norm;
        } else {
            let ab = self.last_tan;
            let cd = tan0;
            let cross = ab.cross(cd);
            let dot = ab.dot(cd);
            let hypot = cross.hypot(dot);
            // possible TODO: a minor speedup could be squaring both sides
            if cross.abs() >= hypot * self.join_thresh {
                match style.join {
                    Join::Bevel => {
                        self.forward_path.line_to(p0 - norm);
                        self.backward_path.line_to(p0 + norm);
                    }
                    Join::Miter => {
                        if 2.0 * hypot < (hypot + dot) * style.miter_limit.powi(2) {
                            // TODO: maybe better to store last_norm or derive from path?
                            let last_scale = 0.5 * style.width / ab.hypot();
                            let last_norm = last_scale * Vec2::new(-ab.y, ab.x);
                            if cross > 0.0 {
                                let fp_last = p0 - last_norm;
                                let fp_this = p0 - norm;
                                let h = ab.cross(fp_this - fp_last) / cross;
                                let miter_pt = fp_this - cd * h;
                                self.forward_path.line_to(miter_pt);
                            } else if cross < 0.0 {
                                let fp_last = p0 + last_norm;
                                let fp_this = p0 + norm;
                                let h = ab.cross(fp_this - fp_last) / cross;
                                let miter_pt = fp_this - cd * h;
                                self.backward_path.line_to(miter_pt);
                            }
                        }
                        self.forward_path.line_to(p0 - norm);
                        self.backward_path.line_to(p0 + norm);
                    }
                    Join::Round => {
                        let angle = cross.atan2(dot);
                        if cross > 0.0 {
                            self.backward_path.line_to(p0 + norm);
                            round_join(&mut self.forward_path, tolerance, p0, norm, angle);
                        } else {
                            self.forward_path.line_to(p0 - norm);
                            round_join_rev(&mut self.backward_path, tolerance, p0, -norm, -angle);
                        }
                    }
                }
            }
        }
    }

    fn do_line(&mut self, style: &Stroke, tangent: Vec2, p1: Point) {
        let scale = 0.5 * style.width / tangent.hypot();
        let norm = scale * Vec2::new(-tangent.y, tangent.x);
        self.forward_path.line_to(p1 - norm);
        self.backward_path.line_to(p1 + norm);
        self.last_pt = p1;
    }

    fn do_cubic(&mut self, style: &Stroke, c: CubicBez, tolerance: f64) {
        let co = CubicOffset::new(c, -0.5 * style.width);
        let forward = fit_to_bezpath(&co, tolerance);
        self.forward_path.extend(forward.into_iter().skip(1));
        let co = CubicOffset::new(c, 0.5 * style.width);
        let backward = fit_to_bezpath(&co, tolerance);
        self.backward_path.extend(backward.into_iter().skip(1));
        self.last_pt = c.p3;
    }
}
