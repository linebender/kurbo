// Copyright 2023 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::{borrow::Borrow, f64::consts::PI};

use alloc::vec::Vec;

use smallvec::SmallVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{
    Affine, Arc, BezPath, CubicBez, Line, ParamCurve, ParamCurveArclen, PathEl, PathSeg, Point,
    QuadBez, Vec2, common::solve_quadratic,
};

/// Defines the connection between two segments of a stroke.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Cap {
    /// Flat cap.
    Butt,
    /// Square cap with dimensions equal to half the stroke width.
    Square,
    /// Rounded cap with radius equal to half the stroke width.
    Round,
}

/// The visual style of a stroke.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
}

/// Options for path stroking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrokeOpts {
    opt_level: StrokeOptLevel,
    /// When `true`, dashes are emitted in the order they appear along the path. The default `false` delays a subpath's first dash so it can be merged with a dash that wraps through `ClosePath`; this option disables both the reorder and the wraparound merge, so a dash spanning the seam is truncated there.
    stable_dash_order: bool,
}

/// Optimization level for computing stroke outlines.
///
/// Note that in the current implementation, this setting has no effect.
/// However, having a tradeoff between optimization of number of segments
/// and speed makes sense and may be added in the future, so applications
/// should set it appropriately. For real time rendering, the appropriate
/// value is `Subdivide`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StrokeOptLevel {
    /// Adaptively subdivide segments in half.
    Subdivide,
    /// Compute optimized subdivision points to minimize error.
    Optimized,
}

impl Default for StrokeOpts {
    fn default() -> Self {
        let opt_level = StrokeOptLevel::Subdivide;
        StrokeOpts {
            opt_level,
            stable_dash_order: false,
        }
    }
}

impl Default for Stroke {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl Stroke {
    /// Creates a new stroke with the specified width.
    pub const fn new(width: f64) -> Self {
        Self {
            width,
            join: Join::Round,
            miter_limit: 4.0,
            start_cap: Cap::Round,
            end_cap: Cap::Round,
            dash_pattern: SmallVec::new_const(),
            dash_offset: 0.0,
        }
    }

    /// Builder method for setting the join style.
    pub const fn with_join(mut self, join: Join) -> Self {
        self.join = join;
        self
    }

    /// Builder method for setting the limit for miter joins.
    pub const fn with_miter_limit(mut self, limit: f64) -> Self {
        self.miter_limit = limit;
        self
    }

    /// Builder method for setting the cap style for the start of the stroke.
    pub const fn with_start_cap(mut self, cap: Cap) -> Self {
        self.start_cap = cap;
        self
    }

    /// Builder method for setting the cap style for the end of the stroke.
    pub const fn with_end_cap(mut self, cap: Cap) -> Self {
        self.end_cap = cap;
        self
    }

    /// Builder method for setting the cap style.
    pub const fn with_caps(mut self, cap: Cap) -> Self {
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

    /// Returns `true` if all floating-point stroke parameters are [finite].
    ///
    /// [finite]: f64::is_finite
    pub fn is_finite(&self) -> bool {
        self.width.is_finite()
            && self.miter_limit.is_finite()
            && self.dash_offset.is_finite()
            && self.dash_pattern.iter().all(|dash| dash.is_finite())
    }

    /// Returns `true` if any floating-point stroke parameter is [`NaN`].
    ///
    /// [`NaN`]: f64::is_nan
    pub fn is_nan(&self) -> bool {
        self.width.is_nan()
            || self.miter_limit.is_nan()
            || self.dash_offset.is_nan()
            || self.dash_pattern.iter().any(|dash| dash.is_nan())
    }
}

impl StrokeOpts {
    /// Set optimization level for computing stroke outlines.
    pub fn opt_level(mut self, opt_level: StrokeOptLevel) -> Self {
        self.opt_level = opt_level;
        self
    }

    /// When `true`, dashes are emitted in the order they appear along the path.
    pub fn stable_dash_order(mut self, stable: bool) -> Self {
        self.stable_dash_order = stable;
        self
    }
}

/// Collection of values representing lengths in a dash pattern.
pub type Dashes = SmallVec<[f64; 4]>;

/// A structure that is used for creating strokes.
///
/// See also [`stroke_with`].
#[derive(Default, Debug)]
pub struct StrokeCtx {
    // As a possible future optimization, we might not need separate storage
    // for forward and backward paths, we can add forward to the output in-place.
    // However, this structure is clearer and the cost fairly modest.
    output: BezPath,
    forward_path: BezPath,
    backward_path: BezPath,
    result_path: BezPath,
    start_pt: Point,
    start_norm: Vec2,
    start_tan: Vec2,
    last_pt: Point,
    last_tan: Vec2,
    // True when the current subpath has drawing commands but no nonzero-length
    // segment yet, so finishing it should emit cap-only geometry.
    degenerate_subpath: bool,
    // Tangent used to orient cap-only geometry. Explicit degenerate subpaths use
    // the SVG fallback direction; zero-length dashes can provide a path tangent.
    degenerate_tangent: Vec2,
    // Precomputation of the join threshold, to optimize per-join logic.
    // If hypot < (hypot + dot) * join_thresh, omit join altogether.
    join_thresh: f64,
}

impl StrokeCtx {
    /// Return the path that defines the expanded stroke.
    pub fn output(&self) -> &BezPath {
        &self.output
    }
}

impl StrokeCtx {
    fn reset(&mut self) {
        self.output.truncate(0);
        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
        self.start_pt = Point::default();
        self.start_norm = Vec2::default();
        self.start_tan = Vec2::default();
        self.last_pt = Point::default();
        self.last_tan = Vec2::default();
        self.degenerate_subpath = false;
        self.join_thresh = 0.0;
    }
}

/// Expand a stroke into a fill.
///
/// The `tolerance` parameter controls the accuracy of the result. In general,
/// the number of subdivisions in the output scales at least to the -1/4 power
/// of the parameter, for example making it 1/16 as big generates twice as many
/// segments. Currently the algorithm is not tuned for extremely fine tolerances.
/// The theoretically optimum scaling exponent is -1/6, but achieving this may
/// require slow numerical techniques (currently a subject of research). The
/// appropriate value depends on the application; if the result of the stroke
/// will be scaled up, a smaller value is needed.
///
/// This method attempts a fairly high degree of correctness, but ultimately
/// is based on computing parallel curves and adding joins and caps, rather than
/// computing the rigorously correct parallel sweep (which requires evolutes in
/// the general case). See [Nehab 2020] for more discussion.
///
/// Zero-length subpaths are stroked according to their caps, matching
/// [SVG 1.1 stroke properties]: butt caps produce no output, while round and
/// square caps produce paintable geometry. When a zero-length subpath has no
/// other direction, it is treated as pointing along the positive x-axis.
///
/// [SVG 1.1 stroke properties]: https://www.w3.org/TR/SVG11/painting.html#StrokeProperties
///
/// [Nehab 2020]: https://dl.acm.org/doi/10.1145/3386569.3392392
pub fn stroke(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    opts: &StrokeOpts,
    tolerance: f64,
) -> BezPath {
    let mut ctx = StrokeCtx::default();
    stroke_with(path, style, opts, tolerance, &mut ctx);

    ctx.output
}

/// Expand a stroke into a fill.
///
/// This is the same as [`stroke`], except for the fact that you can explicitly pass a
/// `StrokeCtx`. By doing so, you can reuse the same context over multiple calls and ensure
/// that the number of reallocations is minimized.
///
/// Unlike [`stroke`], this method doesn't return an owned version of the expanded stroke as a
/// [`BezPath`]. Instead, you can get a reference to the resulting path by calling
/// [`StrokeCtx::output`].
pub fn stroke_with(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    opts: &StrokeOpts,
    tolerance: f64,
    ctx: &mut StrokeCtx,
) {
    if style.dash_pattern.is_empty() {
        stroke_undashed(
            path.into_iter().map(StrokePathEl::Path),
            style,
            tolerance,
            ctx,
        );
    } else {
        let dashed = dash_stroke_iter(
            path.into_iter(),
            style.dash_offset,
            &style.dash_pattern,
            opts.stable_dash_order,
        );
        stroke_undashed(dashed, style, tolerance, ctx);
    }
}

#[derive(Clone, Copy)]
enum StrokePathEl {
    Path(PathEl),
    Degenerate { point: Point, tangent: Vec2 },
}

impl StrokePathEl {
    fn into_path_el(self) -> PathEl {
        match self {
            StrokePathEl::Path(el) => el,
            StrokePathEl::Degenerate { point, .. } => PathEl::LineTo(point),
        }
    }
}

/// Version of stroke expansion for styles with no dashes.
fn stroke_undashed(
    path: impl IntoIterator<Item = StrokePathEl>,
    style: &Stroke,
    tolerance: f64,
    ctx: &mut StrokeCtx,
) {
    ctx.reset();
    ctx.join_thresh = 2.0 * tolerance / style.width;

    for el in path {
        let p0 = ctx.last_pt;
        match el {
            StrokePathEl::Path(PathEl::MoveTo(p)) => {
                ctx.finish(style);
                ctx.start_pt = p;
                ctx.last_pt = p;
            }
            StrokePathEl::Path(PathEl::LineTo(p1)) => {
                if p1 != p0 {
                    let tangent = p1 - p0;
                    ctx.do_join(style, tangent);
                    ctx.last_tan = tangent;
                    ctx.do_line(style, tangent, p1);
                } else {
                    ctx.mark_degenerate();
                }
            }
            StrokePathEl::Path(PathEl::QuadTo(p1, p2)) => {
                if p1 != p0 || p2 != p0 {
                    let q = QuadBez::new(p0, p1, p2);
                    let (tan0, tan1) = PathSeg::Quad(q).tangents();
                    ctx.do_join(style, tan0);
                    ctx.do_cubic(style, q.raise(), tolerance);
                    ctx.last_tan = tan1;
                } else {
                    ctx.mark_degenerate();
                }
            }
            StrokePathEl::Path(PathEl::CurveTo(p1, p2, p3)) => {
                if p1 != p0 || p2 != p0 || p3 != p0 {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
                    ctx.do_join(style, tan0);
                    ctx.do_cubic(style, c, tolerance);
                    ctx.last_tan = tan1;
                } else {
                    ctx.mark_degenerate();
                }
            }
            StrokePathEl::Path(PathEl::ClosePath) => {
                if p0 != ctx.start_pt {
                    let tangent = ctx.start_pt - p0;
                    ctx.do_join(style, tangent);
                    ctx.last_tan = tangent;
                    ctx.do_line(style, tangent, ctx.start_pt);
                } else {
                    ctx.mark_degenerate();
                }
                ctx.finish_closed(style);
            }
            StrokePathEl::Degenerate { point, tangent } => {
                ctx.last_pt = point;
                ctx.mark_degenerate_with(tangent);
            }
        }
    }
    ctx.finish(style);
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
            self.finish_degenerate(style, tolerance);
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
        self.degenerate_subpath = false;
    }

    /// Finish a closed path
    fn finish_closed(&mut self, style: &Stroke) {
        // TODO: scale
        let tolerance = 1e-3;
        if self.forward_path.is_empty() {
            self.finish_degenerate(style, tolerance);
            return;
        }
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
        self.degenerate_subpath = false;
    }

    fn mark_degenerate(&mut self) {
        self.mark_degenerate_with(Vec2::new(1.0, 0.0));
    }

    fn mark_degenerate_with(&mut self, tangent: Vec2) {
        if self.forward_path.is_empty() {
            self.degenerate_subpath = true;
            self.degenerate_tangent = if tangent.hypot2() > 0.0 {
                tangent
            } else {
                Vec2::new(1.0, 0.0)
            };
        }
    }

    fn finish_degenerate(&mut self, style: &Stroke, tolerance: f64) {
        if !self.degenerate_subpath {
            return;
        }
        self.degenerate_subpath = false;

        if style.start_cap == Cap::Butt && style.end_cap == Cap::Butt {
            return;
        }

        let scale = 0.5 * style.width / self.degenerate_tangent.hypot();
        let norm = scale * Vec2::new(-self.degenerate_tangent.y, self.degenerate_tangent.x);
        self.output.move_to(self.last_pt - norm);
        match style.end_cap {
            Cap::Butt => self.output.line_to(self.last_pt + norm),
            Cap::Round => round_cap(&mut self.output, tolerance, self.last_pt, -norm),
            Cap::Square => square_cap(&mut self.output, false, self.last_pt, -norm),
        }
        match style.start_cap {
            Cap::Butt => self.output.close_path(),
            Cap::Round => round_cap(&mut self.output, tolerance, self.last_pt, norm),
            Cap::Square => square_cap(&mut self.output, true, self.last_pt, norm),
        }
    }

    fn do_join(&mut self, style: &Stroke, tan0: Vec2) {
        // TODO: scale
        let tolerance = 1e-3;
        let scale = 0.5 * style.width / tan0.hypot();
        let norm = scale * Vec2::new(-tan0.y, tan0.x);
        let p0 = self.last_pt;
        if self.forward_path.elements().is_empty() {
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
            if dot <= 0.0 || cross.abs() >= hypot * self.join_thresh {
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
                                self.backward_path.line_to(p0);
                            } else if cross < 0.0 {
                                let fp_last = p0 + last_norm;
                                let fp_this = p0 + norm;
                                let h = ab.cross(fp_this - fp_last) / cross;
                                let miter_pt = fp_this - cd * h;
                                self.backward_path.line_to(miter_pt);
                                self.forward_path.line_to(p0);
                            }
                        }
                        self.forward_path.line_to(p0 - norm);
                        self.backward_path.line_to(p0 + norm);
                    }
                    Join::Round => {
                        let angle = cross.atan2(dot);
                        if angle > 0.0 {
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
        // First, detect degenerate linear case

        // Ordinarily, this is the direction of the chord, but if the chord is very
        // short, we take the longer control arm.
        let chord = c.p3 - c.p0;
        let mut chord_ref = chord;
        let mut chord_ref_hypot2 = chord_ref.hypot2();
        let d01 = c.p1 - c.p0;
        if d01.hypot2() > chord_ref_hypot2 {
            chord_ref = d01;
            chord_ref_hypot2 = chord_ref.hypot2();
        }
        let d23 = c.p3 - c.p2;
        if d23.hypot2() > chord_ref_hypot2 {
            chord_ref = d23;
            chord_ref_hypot2 = chord_ref.hypot2();
        }
        // Project Bézier onto chord
        let p0 = c.p0.to_vec2().dot(chord_ref);
        let p1 = c.p1.to_vec2().dot(chord_ref);
        let p2 = c.p2.to_vec2().dot(chord_ref);
        let p3 = c.p3.to_vec2().dot(chord_ref);
        const ENDPOINT_D: f64 = 0.01;
        if p3 <= p0
            || p1 > p2
            || p1 < p0 + ENDPOINT_D * (p3 - p0)
            || p2 > p3 - ENDPOINT_D * (p3 - p0)
        {
            // potentially a cusp inside
            let x01 = d01.cross(chord_ref);
            let x23 = d23.cross(chord_ref);
            let x03 = chord.cross(chord_ref);
            let thresh = tolerance.powi(2) * chord_ref_hypot2;
            if x01 * x01 < thresh && x23 * x23 < thresh && x03 * x03 < thresh {
                // control points are nearly co-linear
                let midpoint = c.p0.midpoint(c.p3);
                // Mapping back from projection of reference chord
                let ref_vec = chord_ref / chord_ref_hypot2;
                let ref_pt = midpoint - 0.5 * (p0 + p3) * ref_vec;
                self.do_linear(style, c, [p0, p1, p2, p3], ref_pt, ref_vec);
                return;
            }
        }

        crate::offset::offset_cubic(c, -0.5 * style.width, tolerance, &mut self.result_path);
        self.forward_path.extend(self.result_path.iter().skip(1));
        crate::offset::offset_cubic(c, 0.5 * style.width, tolerance, &mut self.result_path);
        self.backward_path.extend(self.result_path.iter().skip(1));
        self.last_pt = c.p3;
    }

    /// Do a cubic which is actually linear.
    ///
    /// The `p` argument is the control points projected to the reference chord.
    /// The ref arguments are the inverse map of a projection back to the client
    /// coordinate space.
    fn do_linear(
        &mut self,
        style: &Stroke,
        c: CubicBez,
        p: [f64; 4],
        ref_pt: Point,
        ref_vec: Vec2,
    ) {
        // Always do round join, to model cusp as limit of finite curvature (see Nehab).
        let style = Stroke::new(style.width).with_join(Join::Round);
        // Tangents of endpoints (for connecting to joins)
        let (tan0, tan1) = PathSeg::Cubic(c).tangents();
        self.last_tan = tan0;
        // find cusps
        let c0 = p[1] - p[0];
        let c1 = 2.0 * p[2] - 4.0 * p[1] + 2.0 * p[0];
        let c2 = p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0];
        let roots = solve_quadratic(c0, c1, c2);
        // discard cusps right at endpoints
        const EPSILON: f64 = 1e-6;
        for t in roots {
            if t > EPSILON && t < 1.0 - EPSILON {
                let mt = 1.0 - t;
                let z = mt * (mt * mt * p[0] + 3.0 * t * (mt * p[1] + t * p[2])) + t * t * t * p[3];
                let p = ref_pt + z * ref_vec;
                let tan = p - self.last_pt;
                self.do_join(&style, tan);
                self.do_line(&style, tan, p);
                self.last_tan = tan;
            }
        }
        let tan = c.p3 - self.last_pt;
        self.do_join(&style, tan);
        self.do_line(&style, tan, c.p3);
        self.last_tan = tan;
        self.do_join(&style, tan1);
    }
}

/// An implementation of dashing as an iterator-to-iterator transformation.
struct DashIterator<'a, T> {
    inner: T,
    pass_through: bool,
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
    subpath_has_drawing: bool,
    subpath_has_nonzero_segment: bool,
    stash: Vec<StrokePathEl>,
    stash_ix: usize,
    stable_dash_order: bool,
    needs_moveto: bool,
}

#[derive(PartialEq, Eq)]
enum DashState {
    NeedInput,
    ToStash,
    Working,
    FromStash,
}

impl<T: Iterator<Item = PathEl>> Iterator for DashIterator<'_, T> {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        self.next_stroke_el().map(StrokePathEl::into_path_el)
    }
}

struct DashStrokeIterator<'a, T> {
    inner: DashIterator<'a, T>,
}

impl<T: Iterator<Item = PathEl>> Iterator for DashStrokeIterator<'_, T> {
    type Item = StrokePathEl;

    fn next(&mut self) -> Option<StrokePathEl> {
        self.inner.next_stroke_el()
    }
}

impl<'a, T: Iterator<Item = PathEl>> DashIterator<'a, T> {
    fn next_stroke_el(&mut self) -> Option<StrokePathEl> {
        if self.pass_through {
            return self.inner.next().map(StrokePathEl::Path);
        }

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
                        if self.stable_dash_order {
                            return Some(el);
                        }
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

/// Create a new dashing iterator.
///
/// Handling of dashes is fairly orthogonal to stroke expansion. This iterator
/// is an internal detail of the stroke expansion logic, but is also available
/// separately, and is expected to be useful when doing stroke expansion on
/// GPU.
///
/// It is implemented as an iterator-to-iterator transform. Because it consumes
/// the input sequentially and produces consistent output with correct joins,
/// it requires internal state and may allocate.
///
/// Zero-length dashes are emitted as `MoveTo(p)` followed by `LineTo(p)` so
/// downstream strokers can apply caps. Patterns whose total length is zero are
/// treated as undashed.
///
/// Accuracy is currently hard-coded to 1e-6. This is better than generally
/// expected, and care is taken to get cusps correct, among other things.
pub fn dash<'a>(
    inner: impl Iterator<Item = PathEl> + 'a,
    dash_offset: f64,
    dashes: &'a [f64],
) -> impl Iterator<Item = PathEl> + 'a {
    dash_iter(inner, dash_offset, dashes, false)
}

fn dash_iter<'a>(
    inner: impl Iterator<Item = PathEl> + 'a,
    dash_offset: f64,
    dashes: &'a [f64],
    stable_dash_order: bool,
) -> DashIterator<'a, impl Iterator<Item = PathEl> + 'a> {
    // Ensure that offset is positive and minimal by normalization using period
    let period: f64 = dashes.iter().sum();
    // The SVG spec requires odd-length dash arrays to be doubled to become even-length:
    // <https://www.w3.org/TR/SVG11/painting.html#StrokeDasharrayProperty>
    // This prevents gaps and dashes from swapping with one another as the offset increases.
    let period = if dashes.len() % 2 == 1 {
        2.0 * period
    } else {
        period
    };
    let pass_through = period == 0.0 || !period.is_finite() || !dash_offset.is_finite();
    let (dash_ix, dash_remaining, is_active) = if pass_through {
        (0, 0.0, true)
    } else {
        let dash_offset = dash_offset.rem_euclid(period);
        let mut dash_ix = 0;
        let mut dash_remaining = dashes[dash_ix] - dash_offset;
        let mut is_active = true;
        // Find place in dashes array for initial offset.
        while dash_remaining < 0.0 || (dash_remaining == 0.0 && dashes[dash_ix] != 0.0) {
            dash_ix = (dash_ix + 1) % dashes.len();
            dash_remaining += dashes[dash_ix];
            is_active = !is_active;
        }
        (dash_ix, dash_remaining, is_active)
    };
    DashIterator {
        inner,
        pass_through,
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
        subpath_has_drawing: false,
        subpath_has_nonzero_segment: false,
        stash: Vec::new(),
        stash_ix: 0,
        stable_dash_order,
        needs_moveto: true,
    }
}

fn dash_stroke_iter<'a>(
    inner: impl Iterator<Item = PathEl> + 'a,
    dash_offset: f64,
    dashes: &'a [f64],
    stable_dash_order: bool,
) -> DashStrokeIterator<'a, impl Iterator<Item = PathEl> + 'a> {
    DashStrokeIterator {
        inner: dash_iter(inner, dash_offset, dashes, stable_dash_order),
    }
}

impl<'a, T: Iterator<Item = PathEl>> DashIterator<'a, T> {
    fn advance_dash_phase(&mut self) {
        self.is_active = !self.is_active;
        self.dash_ix += 1;
        if self.dash_ix == self.dashes.len() {
            self.dash_ix = 0;
        }
        self.dash_remaining = self.dashes[self.dash_ix];
    }

    fn reset_subpath_tracking(&mut self) {
        self.subpath_has_drawing = false;
        self.subpath_has_nonzero_segment = false;
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
                    self.reset_subpath_tracking();
                    self.reset_phase();
                    continue;
                }
                PathEl::LineTo(p1) => {
                    let l = Line::new(p0, p1);
                    self.seg_remaining = l.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Line(l);
                    self.last_pt = p1;
                    self.subpath_has_drawing = true;
                    if p1 != p0 {
                        self.subpath_has_nonzero_segment = true;
                    }
                }
                PathEl::QuadTo(p1, p2) => {
                    let q = QuadBez::new(p0, p1, p2);
                    self.seg_remaining = q.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Quad(q);
                    self.last_pt = p2;
                    self.subpath_has_drawing = true;
                    if p1 != p0 || p2 != p0 {
                        self.subpath_has_nonzero_segment = true;
                    }
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    self.seg_remaining = c.arclen(DASH_ACCURACY);
                    self.current_seg = PathSeg::Cubic(c);
                    self.last_pt = p3;
                    self.subpath_has_drawing = true;
                    if p1 != p0 || p2 != p0 || p3 != p0 {
                        self.subpath_has_nonzero_segment = true;
                    }
                }
                PathEl::ClosePath => {
                    if p0 != self.start_pt {
                        let l = Line::new(p0, self.start_pt);
                        self.seg_remaining = l.arclen(DASH_ACCURACY);
                        self.current_seg = PathSeg::Line(l);
                        self.last_pt = self.start_pt;
                        self.subpath_has_drawing = true;
                        self.subpath_has_nonzero_segment = true;
                        self.closepath_pending = true;
                    } else if self.subpath_has_nonzero_segment {
                        self.closepath_pending = true;
                        self.handle_closepath();
                    } else if self.subpath_has_drawing {
                        // The existing zero-length drawing command already
                        // represents this degenerate closed subpath.
                        continue;
                    } else {
                        // ClosePath itself makes this a zero-length subpath.
                        let l = Line::new(p0, self.start_pt);
                        self.seg_remaining = 0.0;
                        self.current_seg = PathSeg::Line(l);
                        self.last_pt = self.start_pt;
                        self.subpath_has_drawing = true;
                    }
                }
            }
            break;
        }
        self.t = 0.0;
    }

    /// Move arc length forward to next event.
    fn step(&mut self) -> Option<StrokePathEl> {
        let mut result = None;
        if self.state == DashState::ToStash && self.needs_moveto {
            self.needs_moveto = false;
            if self.is_active {
                result = Some(StrokePathEl::Path(PathEl::MoveTo(self.current_seg.start())));
            } else {
                self.state = DashState::Working;
            }
        } else if self.dash_remaining < self.seg_remaining {
            // next transition is a dash transition
            let seg = self.current_seg.subsegment(self.t..1.0);
            let t1 = seg.inv_arclen(self.dash_remaining, DASH_ACCURACY);
            if self.is_active {
                let subseg = seg.subsegment(0.0..t1);
                result = if self.dash_remaining == 0.0 {
                    Some(StrokePathEl::Degenerate {
                        point: subseg.start(),
                        tangent: seg.tangents().0,
                    })
                } else {
                    Some(StrokePathEl::Path(seg_to_el(&subseg)))
                };
                self.state = DashState::Working;
            } else {
                let p = seg.eval(t1);
                result = Some(StrokePathEl::Path(PathEl::MoveTo(p)));
            }
            self.t += t1 * (1.0 - self.t);
            self.seg_remaining -= self.dash_remaining;
            self.advance_dash_phase();
        } else {
            let was_active = self.is_active;
            if self.is_active {
                let seg = self.current_seg.subsegment(self.t..1.0);
                result = Some(StrokePathEl::Path(seg_to_el(&seg)));
            }
            self.dash_remaining -= self.seg_remaining;
            if was_active && self.dash_remaining <= 0.0 {
                self.advance_dash_phase();
            }
            self.get_input();
        }
        result
    }

    fn handle_closepath(&mut self) {
        let first_dash_is_degenerate =
            matches!(self.stash.get(1), Some(StrokePathEl::Degenerate { .. }));
        if self.state == DashState::ToStash {
            // Have looped back without breaking a dash, just play it back
            self.stash.push(StrokePathEl::Path(PathEl::ClosePath));
        } else if self.is_active && !self.stable_dash_order && !first_dash_is_degenerate {
            // connect with path in stash, skip MoveTo.
            self.stash_ix = 1;
        }
        self.state = DashState::FromStash;
        self.reset_subpath_tracking();
        self.reset_phase();
    }

    fn reset_phase(&mut self) {
        self.dash_ix = self.init_dash_ix;
        self.dash_remaining = self.init_dash_remaining;
        self.is_active = self.init_is_active;
        self.needs_moveto = true;
    }
}

#[cfg(test)]
mod tests {
    use super::{DASH_ACCURACY, dash_iter};
    use crate::{
        BezPath,
        Cap::{Butt, Round, Square},
        CubicBez,
        Join::Miter,
        Line, PathEl, PathSeg, Point, Rect, Shape, Stroke, StrokeOpts, dash, segments, stroke,
    };

    // A degenerate stroke with a cusp at the endpoint.
    #[test]
    fn pathological_stroke() {
        let curve = CubicBez::new(
            (602.469, 286.585),
            (641.975, 286.585),
            (562.963, 286.585),
            (562.963, 286.585),
        );
        let path = curve.into_path(0.1);
        let stroke_style = Stroke::new(1.);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);
        assert!(stroked.is_finite());
    }

    #[test]
    /// <https://github.com/linebender/kurbo/issues/482>
    fn dash_miter_join() {
        let path = BezPath::from_vec(vec![
            PathEl::MoveTo((70.0, 80.0).into()),
            PathEl::LineTo((0.0, 80.0).into()),
            PathEl::LineTo((0.0, 77.0).into()),
        ]);
        let expected_stroke = BezPath::from_vec(vec![
            PathEl::MoveTo((70.0, 90.0).into()),
            PathEl::LineTo((0.0, 90.0).into()),
            // Miter join point on forward path
            PathEl::LineTo((-10.0, 90.0).into()),
            PathEl::LineTo((-10.0, 80.0).into()),
            PathEl::LineTo((-10.0, 77.0).into()),
            PathEl::LineTo((10.0, 77.0).into()),
            PathEl::LineTo((10.0, 80.0).into()),
            // Miter join point on backward path
            PathEl::LineTo((0.0, 80.0).into()),
            PathEl::LineTo((0.0, 70.0).into()),
            PathEl::LineTo((70.0, 70.0).into()),
            PathEl::ClosePath,
        ]);
        let stroke_style = Stroke::new(20.0)
            .with_join(Miter)
            .with_caps(Butt)
            .with_dashes(0.0, [73.0, 12.0]);
        let stroke = stroke(path, &stroke_style, &StrokeOpts::default(), 0.25);
        assert_eq!(stroke, expected_stroke);
    }

    // Test cases adapted from https://github.com/linebender/vello/pull/388
    #[test]
    fn broken_strokes() {
        let broken_cubics = [
            [
                (465.24423, 107.11105),
                (475.50754, 107.11105),
                (475.50754, 107.11105),
                (475.50754, 107.11105),
            ],
            [(0., -0.01), (128., 128.001), (128., -0.01), (0., 128.001)], // Near-cusp
            [(0., 0.), (0., -10.), (0., -10.), (0., 10.)],                // Flat line with 180
            [(10., 0.), (0., 0.), (20., 0.), (10., 0.)],                  // Flat line with 2 180s
            [(39., -39.), (40., -40.), (40., -40.), (0., 0.)],            // Flat diagonal with 180
            [(40., 40.), (0., 0.), (200., 200.), (0., 0.)],               // Diag w/ an internal 180
            [(0., 0.), (1e-2, 0.), (-1e-2, 0.), (0., 0.)],                // Circle
            // Flat line with no turns:
            [
                (400.75, 100.05),
                (400.75, 100.05),
                (100.05, 300.95),
                (100.05, 300.95),
            ],
            [(0.5, 0.), (0., 0.), (20., 0.), (10., 0.)], // Flat line with 2 180s
            [(10., 0.), (0., 0.), (10., 0.), (10., 0.)], // Flat line with a 180
        ];
        let stroke_style = Stroke::new(30.).with_caps(Butt).with_join(Miter);
        for cubic in &broken_cubics {
            let path = CubicBez::new(cubic[0], cubic[1], cubic[2], cubic[3]).into_path(0.1);
            let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);
            assert!(stroked.is_finite());
        }
    }

    fn assert_rect_approx(actual: Rect, expected: Rect) {
        const EPSILON: f64 = 1e-9;
        assert!(
            (actual.x0 - expected.x0).abs() < EPSILON
                && (actual.y0 - expected.y0).abs() < EPSILON
                && (actual.x1 - expected.x1).abs() < EPSILON
                && (actual.y1 - expected.y1).abs() < EPSILON,
            "actual: {actual:?}, expected: {expected:?}",
        );
    }

    // SVG 1.1 requires zero-length subpaths to be stroked according to their caps:
    // <https://www.w3.org/TR/SVG11/painting.html#StrokeProperties>
    #[test]
    fn zero_length_subpath_round_caps_stroke_as_circle() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.line_to((10.0, 20.0));

        let stroke_style = Stroke::new(6.0).with_caps(Round);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
        assert_eq!(stroked.winding((10.0, 20.0).into()), 1);
    }

    #[test]
    fn zero_length_subpath_square_caps_stroke_as_square() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.line_to((10.0, 20.0));

        let stroke_style = Stroke::new(6.0).with_caps(Square);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
        assert_eq!(stroked.winding((10.0, 20.0).into()), 1);
    }

    #[test]
    fn zero_length_subpath_butt_caps_do_not_stroke() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.line_to((10.0, 20.0));

        let stroke_style = Stroke::new(6.0).with_caps(Butt);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        assert!(stroked.is_empty());
    }

    #[test]
    fn move_only_subpath_does_not_stroke() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));

        let stroke_style = Stroke::new(6.0).with_caps(Round);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        assert!(stroked.is_empty());
    }

    #[test]
    fn zero_length_closed_subpath_strokes() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.close_path();

        let stroke_style = Stroke::new(6.0).with_caps(Round);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
        assert_eq!(stroked.winding((10.0, 20.0).into()), 1);
    }

    #[test]
    fn zero_length_closed_subpath_respects_nonround_caps() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.close_path();

        let square = stroke(
            path.iter(),
            &Stroke::new(6.0).with_caps(Square),
            &StrokeOpts::default(),
            0.001,
        );
        assert_rect_approx(square.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
        assert_eq!(square.winding((10.0, 20.0).into()), 1);

        let butt = stroke(
            path,
            &Stroke::new(6.0).with_caps(Butt),
            &StrokeOpts::default(),
            0.001,
        );
        assert!(butt.is_empty());
    }

    #[test]
    fn zero_length_curve_subpaths_respect_caps() {
        let point = Point::new(10.0, 20.0);
        let paths = [
            BezPath::from_vec(vec![PathEl::MoveTo(point), PathEl::QuadTo(point, point)]),
            BezPath::from_vec(vec![
                PathEl::MoveTo(point),
                PathEl::CurveTo(point, point, point),
            ]),
        ];

        for path in paths {
            for cap in [Round, Square] {
                let stroked = stroke(
                    path.iter(),
                    &Stroke::new(6.0).with_caps(cap),
                    &StrokeOpts::default(),
                    0.001,
                );
                assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
                assert_eq!(stroked.winding(point), 1);
            }

            let butt = stroke(
                path,
                &Stroke::new(6.0).with_caps(Butt),
                &StrokeOpts::default(),
                0.001,
            );
            assert!(butt.is_empty());
        }
    }

    #[test]
    fn zero_length_subpath_respects_mixed_caps() {
        let point = Point::new(10.0, 20.0);
        let mut path = BezPath::new();
        path.move_to(point);
        path.line_to(point);

        // With the fallback +x tangent, an end cap extends right and a start
        // cap extends left. A butt cap closes the other half at the point.
        let round_end = stroke(
            path.iter(),
            &Stroke::new(6.0).with_start_cap(Butt).with_end_cap(Round),
            &StrokeOpts::default(),
            0.001,
        );
        assert_rect_approx(round_end.bounding_box(), Rect::new(10.0, 17.0, 13.0, 23.0));
        assert_eq!(round_end.winding((11.0, 20.0).into()), 1);
        assert_eq!(round_end.winding((9.0, 20.0).into()), 0);

        let round_start = stroke(
            path,
            &Stroke::new(6.0).with_start_cap(Round).with_end_cap(Butt),
            &StrokeOpts::default(),
            0.001,
        );
        assert_rect_approx(round_start.bounding_box(), Rect::new(7.0, 17.0, 10.0, 23.0));
        assert_eq!(round_start.winding((9.0, 20.0).into()), 1);
        assert_eq!(round_start.winding((11.0, 20.0).into()), 0);
    }

    #[test]
    fn dashed_zero_length_closed_subpath_strokes_at_subpath_point() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.close_path();

        // ClosePath supplies the drawing command for this otherwise empty
        // subpath; the active dash must preserve it at the MoveTo point.
        let stroke_style = Stroke::new(6.0)
            .with_caps(Round)
            .with_dashes(0.0, [1.0, 1.0]);
        for stable_dash_order in [false, true] {
            let opts = StrokeOpts::default().stable_dash_order(stable_dash_order);
            let stroked = stroke(path.iter(), &stroke_style, &opts, 0.001);

            assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
            assert_eq!(stroked.winding((10.0, 20.0).into()), 1);
            assert_eq!(stroked.winding((0.0, 0.0).into()), 0);
        }
    }

    #[test]
    fn dashed_zero_length_closed_subpath_respects_inactive_phase() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.close_path();

        let stroke_style = Stroke::new(6.0)
            .with_caps(Round)
            .with_dashes(1.5, [1.0, 1.0]);
        for stable_dash_order in [false, true] {
            let opts = StrokeOpts::default().stable_dash_order(stable_dash_order);
            let stroked = stroke(path.iter(), &stroke_style, &opts, 0.001);

            assert!(stroked.is_empty());
        }
    }

    #[test]
    fn dashed_explicit_zero_length_segment_then_close_strokes_once() {
        let mut path = BezPath::new();
        path.move_to((10.0, 20.0));
        path.line_to((10.0, 20.0));
        path.close_path();

        // The explicit LineTo already represents the degenerate subpath, so
        // ClosePath must not synthesize a second cap at the same point.
        let stroke_style = Stroke::new(6.0)
            .with_caps(Round)
            .with_dashes(0.0, [1.0, 1.0]);
        for stable_dash_order in [false, true] {
            let opts = StrokeOpts::default().stable_dash_order(stable_dash_order);
            let stroked = stroke(path.iter(), &stroke_style, &opts, 0.001);

            assert_rect_approx(stroked.bounding_box(), Rect::new(7.0, 17.0, 13.0, 23.0));
            assert_eq!(stroked.winding((10.0, 20.0).into()), 1);
        }
    }

    #[test]
    fn zero_length_dash_round_caps_stroke_as_dots() {
        let line = Line::new((0.0, 0.0), (21.0, 0.0));
        let stroke_style = Stroke::new(4.0)
            .with_caps(Round)
            .with_dashes(0.0, [0.0, 10.0]);
        let stroked = stroke(
            line.path_elements(0.001),
            &stroke_style,
            &StrokeOpts::default(),
            0.001,
        );

        assert_rect_approx(stroked.bounding_box(), Rect::new(-2.0, -2.0, 22.0, 2.0));
        for x in [0.0, 10.0, 20.0] {
            assert_ne!(stroked.winding((x, 0.0).into()), 0);
        }
        for x in [5.0, 15.0] {
            assert_eq!(stroked.winding((x, 0.0).into()), 0);
        }
    }

    #[test]
    fn zero_length_dash_at_open_endpoint_is_excluded() {
        let stroke_style = Stroke::new(4.0)
            .with_caps(Round)
            .with_dashes(0.0, [0.0, 10.0]);
        // Dash starts use the half-open range [0, path_length): a dot exactly
        // at 20 is excluded, but appears as soon as the path extends past it.
        for (length, endpoint_is_stroked) in [(19.999, false), (20.0, false), (20.001, true)] {
            let line = Line::new((0.0, 0.0), (length, 0.0));
            let stroked = stroke(
                line.path_elements(0.001),
                &stroke_style,
                &StrokeOpts::default(),
                0.001,
            );

            for x in [0.0, 10.0] {
                assert_ne!(stroked.winding((x, 0.0).into()), 0);
            }
            assert_eq!(
                stroked.winding((20.0, 0.0).into()) != 0,
                endpoint_is_stroked
            );
        }
    }

    #[test]
    fn zero_length_dash_at_each_open_subpath_endpoint_is_excluded() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((20.0, 0.0));
        path.move_to((100.0, 0.0));
        path.line_to((120.0, 0.0));
        let stroke_style = Stroke::new(4.0)
            .with_caps(Round)
            .with_dashes(0.0, [0.0, 10.0]);
        let stroked = stroke(path, &stroke_style, &StrokeOpts::default(), 0.001);

        for x in [0.0, 10.0, 100.0, 110.0] {
            assert_ne!(stroked.winding((x, 0.0).into()), 0);
        }
        for x in [20.0, 120.0] {
            assert_eq!(stroked.winding((x, 0.0).into()), 0);
        }
    }

    #[test]
    fn zero_length_dash_square_caps_follow_line_tangent() {
        let line = Line::new((0.0, 0.0), (21.0, 21.0));
        let gap = 10.0 * 2.0_f64.sqrt();
        let extent = 2.0 * 2.0_f64.sqrt();
        let stroke_style = Stroke::new(4.0)
            .with_caps(Square)
            .with_dashes(0.0, [0.0, gap]);
        let stroked = stroke(
            line.path_elements(0.001),
            &stroke_style,
            &StrokeOpts::default(),
            0.001,
        );

        assert_rect_approx(
            stroked.bounding_box(),
            Rect::new(-extent, -extent, 20.0 + extent, 20.0 + extent),
        );
        for point in [(0.0, 2.5), (10.0, 12.5), (20.0, 22.5)] {
            assert_ne!(stroked.winding(point.into()), 0);
        }
    }

    // Regression coverage for <https://github.com/linebender/kurbo/pull/578>:
    // exact dash boundaries should not create accidental zero-length output.
    #[test]
    fn dash_transition_on_vertex_does_not_emit_zero_length_line() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((3.0, 0.0));
        path.line_to((3.0, 3.0));
        let dashes = [3.0, 1.0];
        let expected = [
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((3.0, 0.0).into()),
            PathEl::MoveTo((3.0, 1.0).into()),
            PathEl::LineTo((3.0, 3.0).into()),
        ];

        let result = dash_iter(path.into_iter(), 0.0, &dashes, false).collect::<Vec<PathEl>>();

        assert_eq!(result, expected);
    }

    #[test]
    fn dash_preserves_small_initial_dash() {
        let tiny_remainder = DASH_ACCURACY * 0.5;
        let shape = Line::new((0.0, 0.0), (1.0, 0.0));
        let dashes = [3.0, 1.0];
        let dash_offset = 3.0 - tiny_remainder;
        let expected_dash_end = dashes[0] - dash_offset;
        let expected = [
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((expected_dash_end, 0.0).into()),
        ];

        let result = dash(shape.path_elements(0.0), dash_offset, &dashes).collect::<Vec<PathEl>>();

        assert_eq!(result, expected);
    }

    #[test]
    fn dash_preserves_small_remainder_across_vertex() {
        let tiny_remainder = DASH_ACCURACY * 0.5;
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((3.0, 0.0));
        path.line_to((3.0, 1.0));
        let dashes = [3.0 + tiny_remainder, 1.0];
        let expected_dash_end = dashes[0] - 3.0;
        let expected = [
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((3.0, 0.0).into()),
            PathEl::LineTo((3.0, expected_dash_end).into()),
        ];

        let result = dash(path.iter(), 0.0, &dashes).collect::<Vec<PathEl>>();

        assert_eq!(result, expected);
    }

    #[test]
    fn dash_preserves_explicit_zero_length_on_dash() {
        let shape = Line::new((0.0, 0.0), (21.0, 0.0));
        let dashes = [0.0, 10.0];
        let result =
            dash_iter(shape.path_elements(0.0), 0.0, &dashes, true).collect::<Vec<PathEl>>();
        let expected = [
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((0.0, 0.0).into()),
            PathEl::MoveTo((10.0, 0.0).into()),
            PathEl::LineTo((10.0, 0.0).into()),
            PathEl::MoveTo((20.0, 0.0).into()),
            PathEl::LineTo((20.0, 0.0).into()),
        ];

        assert_eq!(result, expected);
    }

    #[test]
    fn degenerate_dash_patterns_pass_through_undashed() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((10.0, 0.0));
        path.move_to((20.0, 0.0));
        path.quad_to((25.0, 5.0), (30.0, 0.0));
        // These inputs previously became solid only because normalization
        // produced NaN. Pin the intentional pass-through behavior instead.
        let patterns: &[(f64, &[f64])] = &[
            (0.0, &[0.0, 0.0]),
            (0.0, &[1.0, -1.0]),
            (f64::NAN, &[1.0, 1.0]),
            (0.0, &[1.0, f64::NAN]),
        ];

        for &(offset, pattern) in patterns {
            let actual = dash(path.iter(), offset, pattern).collect::<Vec<_>>();
            assert_eq!(actual, path.elements());
        }
    }

    #[test]
    fn all_zero_dash_pattern_strokes_as_solid() {
        let line = Line::new((0.0, 0.0), (20.0, 0.0));
        let solid_style = Stroke::new(4.0).with_caps(Square);
        // stroke() consumes the internal dash iterator rather than public
        // PathEl output, so verify its fallback matches the direct path too.
        let expected = stroke(
            line.path_elements(0.001),
            &solid_style,
            &StrokeOpts::default(),
            0.001,
        );
        let actual = stroke(
            line.path_elements(0.001),
            &solid_style.with_dashes(0.0, [0.0, 0.0]),
            &StrokeOpts::default(),
            0.001,
        );

        assert_eq!(actual, expected);
    }

    #[test]
    fn dash_keeps_zero_length_first_dash_separate_at_closed_seam() {
        let shape = Rect::from_points((0.0, 0.0), (10.0, 10.0));
        let dashes = [0.0, 30.0, 20.0, 5.0];
        // The final dash reaches the seam, but the initial zero-length dash
        // still needs its own MoveTo so downstream strokers apply its caps.
        let expected = [
            PathEl::MoveTo((0.0, 10.0).into()),
            PathEl::LineTo((0.0, 0.0).into()),
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((0.0, 0.0).into()),
        ];

        let actual = dash(shape.path_elements(0.0), 0.0, &dashes).collect::<Vec<_>>();

        assert_eq!(actual, expected);
    }

    #[test]
    fn stroke_keeps_zero_length_first_dash_caps_at_closed_seam() {
        let shape = Rect::from_points((0.0, 0.0), (10.0, 10.0));
        let style = Stroke::new(4.0)
            .with_start_cap(Round)
            .with_end_cap(Butt)
            .with_dashes(0.0, [0.0, 30.0, 20.0, 5.0]);
        // The round start cap of the zero-length dash covers this point below
        // the seam; the final dash ends there with a butt cap and does not.
        for stable_dash_order in [false, true] {
            let opts = StrokeOpts::default().stable_dash_order(stable_dash_order);
            let stroked = stroke(shape.path_elements(0.001), &style, &opts, 0.001);

            assert_eq!(stroked.winding((-1.0, -1.0).into()), 1);
        }
    }

    #[test]
    fn dash_ending_on_closepath_vertex_does_not_merge_across_seam() {
        let shape = Rect::from_points((0.0, 0.0), (3.0, 2.5));
        let dashes = [3.0, 1.0];
        let expected = [
            PathEl::MoveTo((0.5, 2.5).into()),
            PathEl::LineTo((0.0, 2.5).into()),
            PathEl::LineTo((0.0, 0.0).into()),
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((3.0, 0.0).into()),
            PathEl::MoveTo((3.0, 1.0).into()),
            PathEl::LineTo((3.0, 2.5).into()),
            PathEl::LineTo((1.5, 2.5).into()),
        ];

        let result = dash(shape.path_elements(0.0), 0.0, &dashes).collect::<Vec<PathEl>>();

        assert_eq!(result, expected);
    }

    #[test]
    fn dash_sequence() {
        let shape = Line::new((0.0, 0.0), (21.0, 0.0));
        let dashes = [1., 5., 2., 5.];
        let expansion = [
            PathSeg::Line(Line::new((6., 0.), (8., 0.))),
            PathSeg::Line(Line::new((13., 0.), (14., 0.))),
            PathSeg::Line(Line::new((19., 0.), (21., 0.))),
            PathSeg::Line(Line::new((0., 0.), (1., 0.))),
        ];
        let iter = segments(dash(shape.path_elements(0.), 0., &dashes));
        assert_eq!(iter.collect::<Vec<PathSeg>>(), expansion);
    }

    #[test]
    fn dash_sequence_closed_path() {
        let shape = Rect::from_points((0.0, 0.0), (4.0, 4.0));
        let dashes = [5., 1.];
        let expansion = [
            PathEl::MoveTo((4.0, 2.0).into()),
            PathEl::LineTo((4.0, 4.0).into()),
            PathEl::LineTo((1.0, 4.0).into()),
            PathEl::MoveTo((0.0, 4.0).into()),
            PathEl::LineTo((0.0, 0.0).into()),
            PathEl::LineTo((4.0, 0.0).into()),
            PathEl::LineTo((4.0, 1.0).into()),
        ];
        let iter = dash(shape.path_elements(0.), 0., &dashes);
        assert_eq!(iter.collect::<Vec<PathEl>>(), expansion);
    }

    #[test]
    fn dash_sequence_stable_order() {
        let shape = Line::new((0.0, 0.0), (21.0, 0.0));
        let dashes = [1., 5., 2., 5.];
        let expansion = [
            PathSeg::Line(Line::new((0., 0.), (1., 0.))),
            PathSeg::Line(Line::new((6., 0.), (8., 0.))),
            PathSeg::Line(Line::new((13., 0.), (14., 0.))),
            PathSeg::Line(Line::new((19., 0.), (21., 0.))),
        ];
        let iter = segments(dash_iter(shape.path_elements(0.), 0., &dashes, true));
        assert_eq!(iter.collect::<Vec<PathSeg>>(), expansion);
    }

    #[test]
    fn dash_sequence_closed_path_stable_order() {
        let shape = Rect::from_points((0.0, 0.0), (4.0, 4.0));
        let dashes = [5., 1.];
        let expansion = [
            PathEl::MoveTo((0.0, 0.0).into()),
            PathEl::LineTo((4.0, 0.0).into()),
            PathEl::LineTo((4.0, 1.0).into()),
            PathEl::MoveTo((4.0, 2.0).into()),
            PathEl::LineTo((4.0, 4.0).into()),
            PathEl::LineTo((1.0, 4.0).into()),
            PathEl::MoveTo((0.0, 4.0).into()),
            PathEl::LineTo((0.0, 0.0).into()),
        ];
        let iter = dash_iter(shape.path_elements(0.), 0., &dashes, true);
        assert_eq!(iter.collect::<Vec<PathEl>>(), expansion);
    }

    #[test]
    fn dash_sequence_offset() {
        // Same as dash_sequence, but with a dash offset
        // of 3, which skips the first dash and cuts into
        // the first gap.
        let shape = Line::new((0.0, 0.0), (21.0, 0.0));
        let dashes = [1., 5., 2., 5.];
        let expansion = [
            PathSeg::Line(Line::new((3., 0.), (5., 0.))),
            PathSeg::Line(Line::new((10., 0.), (11., 0.))),
            PathSeg::Line(Line::new((16., 0.), (18., 0.))),
        ];
        let iter = segments(dash(shape.path_elements(0.), 3., &dashes));
        assert_eq!(iter.collect::<Vec<PathSeg>>(), expansion);
    }

    // Differently-sized subpaths verify stable mode restarts the dash pattern per subpath without leaking state.
    #[test]
    fn dash_stable_order_multi_subpath() {
        let mut path = BezPath::new();
        path.move_to((0., 0.));
        path.line_to((2., 0.));
        path.line_to((2., 2.));
        path.line_to((0., 2.));
        path.close_path();
        path.move_to((10., 10.));
        path.line_to((15., 10.));
        path.line_to((15., 14.));
        path.line_to((10., 14.));
        path.close_path();
        let dashes = [3., 1.];
        let expansion = [
            PathEl::MoveTo((0., 0.).into()),
            PathEl::LineTo((2., 0.).into()),
            PathEl::LineTo((2., 1.).into()),
            PathEl::MoveTo((2., 2.).into()),
            PathEl::LineTo((0., 2.).into()),
            PathEl::LineTo((0., 1.).into()),
            PathEl::MoveTo((10., 10.).into()),
            PathEl::LineTo((13., 10.).into()),
            PathEl::MoveTo((14., 10.).into()),
            PathEl::LineTo((15., 10.).into()),
            PathEl::LineTo((15., 12.).into()),
            PathEl::MoveTo((15., 13.).into()),
            PathEl::LineTo((15., 14.).into()),
            PathEl::LineTo((13., 14.).into()),
            PathEl::MoveTo((12., 14.).into()),
            PathEl::LineTo((10., 14.).into()),
            PathEl::LineTo((10., 13.).into()),
            PathEl::MoveTo((10., 12.).into()),
            PathEl::LineTo((10., 10.).into()),
        ];
        let iter = dash_iter(path.into_iter(), 0., &dashes, true);
        assert_eq!(iter.collect::<Vec<PathEl>>(), expansion);
    }

    #[test]
    fn dash_negative_offset() {
        let shape = Line::new((0.0, 0.0), (28.0, 0.0));
        let dashes = [4., 2.];
        let pos = segments(dash(shape.path_elements(0.), 60., &dashes)).collect::<Vec<PathSeg>>();
        let neg = segments(dash(shape.path_elements(0.), -60., &dashes)).collect::<Vec<PathSeg>>();
        assert_eq!(neg, pos);
    }

    #[test]
    fn dash_odd_length_matches_doubled() {
        let shape = Line::new((0.0, 0.0), (50.0, 0.0));
        let odd = [10.];
        let doubled = [10., 10.];
        for offset in [0., 5., 9., 10., 11., 15., 20., 25., 100., -7.] {
            let from_odd =
                segments(dash(shape.path_elements(0.), offset, &odd)).collect::<Vec<PathSeg>>();
            let from_doubled =
                segments(dash(shape.path_elements(0.), offset, &doubled)).collect::<Vec<PathSeg>>();
            assert_eq!(from_odd, from_doubled, "mismatch at offset {offset}");
        }
    }

    #[test]
    fn dash_three_element_matches_doubled() {
        let shape = Line::new((0.0, 0.0), (200.0, 0.0));
        let three = [20., 10., 3.];
        let doubled = [20., 10., 3., 20., 10., 3.];
        for offset in [0., 15., 32., 33., 34., 50., 66., 99.] {
            let from_three =
                segments(dash(shape.path_elements(0.), offset, &three)).collect::<Vec<PathSeg>>();
            let from_doubled =
                segments(dash(shape.path_elements(0.), offset, &doubled)).collect::<Vec<PathSeg>>();
            assert_eq!(from_three, from_doubled, "mismatch at offset {offset}");
        }
    }

    #[test]
    fn stroke_is_finite_fields() {
        let finite = Stroke::new(2.0)
            .with_miter_limit(4.0)
            .with_dashes(0.0, [1.0, 2.0]);
        assert!(finite.is_finite());

        let non_finite_width = Stroke::new(f64::INFINITY);
        assert!(!non_finite_width.is_finite());

        let non_finite_miter = Stroke::new(2.0).with_miter_limit(f64::NAN);
        assert!(!non_finite_miter.is_finite());

        let non_finite_dash_offset = Stroke::new(2.0).with_dashes(f64::NEG_INFINITY, [1.0, 2.0]);
        assert!(!non_finite_dash_offset.is_finite());

        let non_finite_dash_pattern = Stroke::new(2.0).with_dashes(0.0, [1.0, f64::NAN]);
        assert!(!non_finite_dash_pattern.is_finite());
    }

    #[test]
    fn stroke_is_nan_fields() {
        let finite = Stroke::new(2.0)
            .with_miter_limit(4.0)
            .with_dashes(0.0, [1.0, 2.0]);
        assert!(!finite.is_nan());

        let nan_width = Stroke::new(f64::NAN);
        assert!(nan_width.is_nan());

        let nan_miter = Stroke::new(2.0).with_miter_limit(f64::NAN);
        assert!(nan_miter.is_nan());

        let nan_dash_offset = Stroke::new(2.0).with_dashes(f64::NAN, [1.0, 2.0]);
        assert!(nan_dash_offset.is_nan());

        let nan_dash_pattern = Stroke::new(2.0).with_dashes(0.0, [1.0, f64::NAN]);
        assert!(nan_dash_pattern.is_nan());

        let infinite_width = Stroke::new(f64::INFINITY);
        assert!(!infinite_width.is_nan());
    }
}
