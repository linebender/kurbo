// Copyright 2023 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::{borrow::Borrow, f64::consts::PI};

use alloc::vec::Vec;

use smallvec::SmallVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{
    fit_to_bezpath, fit_to_bezpath_opt, offset::CubicOffset, Affine, Arc, BezPath, CubicBez, Line,
    ParamCurve, ParamCurveArclen, PathEl, PathSeg, Point, QuadBez, Vec2,
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

/// Options for path stroking.
pub struct StrokeOpts {
    opt_level: StrokeOptLevel,
}

/// Optimization level for computing
pub enum StrokeOptLevel {
    /// Adaptively subdivide segments in half.
    Subdivide,
    /// Compute optimized subdivision points to minimize error.
    Optimized,
}

impl Default for StrokeOpts {
    fn default() -> Self {
        let opt_level = StrokeOptLevel::Subdivide;
        StrokeOpts { opt_level }
    }
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

impl StrokeOpts {
    /// Set optimization level for computing stroke outlines.
    pub fn opt_level(mut self, opt_level: StrokeOptLevel) -> Self {
        self.opt_level = opt_level;
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
pub fn stroke(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    opts: &StrokeOpts,
    tolerance: f64,
) -> BezPath {
    if style.dash_pattern.is_empty() {
        stroke_undashed(path, style, tolerance, opts)
    } else {
        let dashed = DashIterator::new(path.into_iter(), style.dash_offset, &style.dash_pattern);
        stroke_undashed(dashed, style, tolerance, opts)
    }
}

/// Version of stroke expansion for styles with no dashes.
fn stroke_undashed(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    tolerance: f64,
    opts: &StrokeOpts,
) -> BezPath {
    let mut ctx = StrokeCtx {
        join_thresh: 2.0 * tolerance / style.width,
        ..Default::default()
    };
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
                    ctx.do_cubic(style, q.raise(), tolerance, opts);
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != p0 && p2 != p0 && p3 != p0 {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
                    ctx.do_join(style, tan0);
                    ctx.last_tan = tan1;
                    ctx.do_cubic(style, c, tolerance, opts);
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

fn fit_with_opts(co: &CubicOffset, tolerance: f64, opts: &StrokeOpts) -> BezPath {
    match opts.opt_level {
        StrokeOptLevel::Subdivide => fit_to_bezpath(co, tolerance),
        StrokeOptLevel::Optimized => fit_to_bezpath_opt(co, tolerance),
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

    fn do_cubic(&mut self, style: &Stroke, c: CubicBez, tolerance: f64, opts: &StrokeOpts) {
        let co = CubicOffset::new(c, -0.5 * style.width);
        let forward = fit_with_opts(&co, tolerance, opts);
        self.forward_path.extend(forward.into_iter().skip(1));
        let co = CubicOffset::new(c, 0.5 * style.width);
        let backward = fit_with_opts(&co, tolerance, opts);
        self.backward_path.extend(backward.into_iter().skip(1));
        self.last_pt = c.p3;
    }
}

/// Iterator for dashing.
pub struct DashIterator<'a, T> {
    inner: T,
    input_done: bool,
    closepath_pending: bool,
    dashes: &'a [f64],
    dash_ix: usize,
    init_dash_ix: usize,
    init_dash_remaining: f64,
    init_is_active: bool,
    is_active: bool,
    state: DashState,
    current_seg: PathSeg,
    t: f64,
    dash_remaining: f64,
    seg_remaining: f64,
    start_pt: Point,
    last_pt: Point,
    stash: Vec<PathEl>,
    stash_ix: usize,
}

#[derive(PartialEq, Eq)]
enum DashState {
    NeedInput,
    ToStash,
    Working,
    FromStash,
}

impl<'a, T: Iterator<Item = PathEl>> Iterator for DashIterator<'a, T> {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        loop {
            match self.state {
                DashState::NeedInput => {
                    if self.input_done {
                        return None;
                    }
                    self.get_input();
                    if self.input_done {
                        return None;
                    }
                    self.state = DashState::ToStash;
                }
                DashState::ToStash => {
                    if let Some(el) = self.step() {
                        self.stash.push(el);
                    }
                }
                DashState::Working => {
                    if let Some(el) = self.step() {
                        return Some(el);
                    }
                }
                DashState::FromStash => {
                    if let Some(el) = self.stash.get(self.stash_ix) {
                        self.stash_ix += 1;
                        return Some(*el);
                    } else {
                        self.stash.clear();
                        self.stash_ix = 0;
                        if self.input_done {
                            return None;
                        }
                        if self.closepath_pending {
                            self.closepath_pending = false;
                            self.state = DashState::NeedInput;
                        } else {
                            self.state = DashState::ToStash;
                        }
                    }
                }
            }
        }
    }
}

fn seg_to_el(el: &PathSeg) -> PathEl {
    match el {
        PathSeg::Line(l) => PathEl::LineTo(l.p1),
        PathSeg::Quad(q) => PathEl::QuadTo(q.p1, q.p2),
        PathSeg::Cubic(c) => PathEl::CurveTo(c.p1, c.p2, c.p3),
    }
}

const DASH_ACCURACY: f64 = 1e-6;

impl<'a, T: Iterator<Item = PathEl>> DashIterator<'a, T> {
    /// Create a new dashing iterator.
    pub fn new(inner: T, dash_offset: f64, dashes: &'a [f64]) -> Self {
        let mut dash_ix = 0;
        let mut dash_remaining = dash_offset;
        let mut is_active = true;
        // Find place in dashes array for initial offset.
        while dash_remaining > 0.0 {
            let dash_len = dashes[dash_ix];
            if dash_remaining < dash_len {
                break;
            }
            dash_remaining -= dash_len;
            dash_ix = (dash_ix + 1) % dashes.len();
            is_active = !is_active;
        }
        DashIterator {
            inner,
            input_done: false,
            closepath_pending: false,
            dashes,
            dash_ix,
            init_dash_ix: dash_ix,
            init_dash_remaining: dash_remaining,
            init_is_active: is_active,
            is_active,
            state: DashState::NeedInput,
            current_seg: PathSeg::Line(Line::new(Point::ORIGIN, Point::ORIGIN)),
            t: 0.0,
            dash_remaining,
            seg_remaining: 0.0,
            start_pt: Point::ORIGIN,
            last_pt: Point::ORIGIN,
            stash: Vec::new(),
            stash_ix: 0,
        }
    }

    fn get_input(&mut self) {
        loop {
            if self.closepath_pending {
                self.handle_closepath();
                break;
            }
            let Some(next_el) = self.inner.next() else {
                self.input_done = true;
                self.state = DashState::FromStash;
                return;
            };
            let p0 = self.last_pt;
            match next_el {
                PathEl::MoveTo(p) => {
                    if !self.stash.is_empty() {
                        self.state = DashState::FromStash;
                    }
                    self.start_pt = p;
                    self.last_pt = p;
                    self.reset_phase();
                    continue;
                }
                PathEl::LineTo(p1) => {
                    let l = Line::new(p0, p1);
                    self.seg_remaining = l.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Line(l);
                    self.last_pt = p1;
                }
                PathEl::QuadTo(p1, p2) => {
                    let q = QuadBez::new(p0, p1, p2);
                    self.seg_remaining = q.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Quad(q);
                    self.last_pt = p2;
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    self.seg_remaining = c.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Cubic(c);
                    self.last_pt = p3;
                }
                PathEl::ClosePath => {
                    self.closepath_pending = true;
                    if p0 != self.start_pt {
                        let l = Line::new(p0, self.start_pt);
                        self.seg_remaining = l.arclen(DASH_ACCURACY);
                        self.current_seg = PathSeg::Line(l);
                        self.last_pt = self.start_pt;
                    } else {
                        self.handle_closepath();
                    }
                }
            }
            break;
        }
        self.t = 0.0;
    }

    /// Move arc length forward to next event.
    fn step(&mut self) -> Option<PathEl> {
        let mut result = None;
        if self.state == DashState::ToStash && self.stash.is_empty() {
            if self.is_active {
                result = Some(PathEl::MoveTo(self.current_seg.start()));
            } else {
                self.state = DashState::Working;
            }
        } else if self.dash_remaining < self.seg_remaining {
            // next transition is a dash transition
            let seg = self.current_seg.subsegment(self.t..1.0);
            let t1 = seg.inv_arclen(self.dash_remaining, DASH_ACCURACY);
            if self.is_active {
                let subseg = seg.subsegment(0.0..t1);
                result = Some(seg_to_el(&subseg));
                self.state = DashState::Working;
            } else {
                let p = seg.eval(t1);
                result = Some(PathEl::MoveTo(p));
            }
            self.is_active = !self.is_active;
            self.t += t1 * (1.0 - self.t);
            self.seg_remaining -= self.dash_remaining;
            self.dash_ix += 1;
            if self.dash_ix == self.dashes.len() {
                self.dash_ix = 0;
            }
            self.dash_remaining = self.dashes[self.dash_ix];
        } else {
            if self.is_active {
                let seg = self.current_seg.subsegment(self.t..1.0);
                result = Some(seg_to_el(&seg));
            }
            self.dash_remaining -= self.seg_remaining;
            self.get_input();
        }
        result
    }

    fn handle_closepath(&mut self) {
        if self.state == DashState::ToStash {
            // Have looped back without breaking a dash, just play it back
            self.stash.push(PathEl::ClosePath);
        } else if self.is_active {
            // connect with path in stash, skip MoveTo.
            self.stash_ix = 1;
        }
        self.state = DashState::FromStash;
        self.reset_phase();
    }

    fn reset_phase(&mut self) {
        self.dash_ix = self.init_dash_ix;
        self.dash_remaining = self.init_dash_remaining;
        self.is_active = self.init_is_active;
    }
}
