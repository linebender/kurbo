// Copyright 2023 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::{borrow::Borrow, f64::consts::PI};

use alloc::vec::Vec;

use smallvec::SmallVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{
    common::solve_quadratic, Affine, Arc, BezPath, CubicBez, Line, ParamCurve, ParamCurveArclen,
    PathEl, PathSeg, Point, QuadBez, Shape, Vec2,
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
        StrokeOpts { opt_level }
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
/// [Nehab 2020]: https://dl.acm.org/doi/10.1145/3386569.3392392
pub fn stroke(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    _opts: &StrokeOpts,
    tolerance: f64,
) -> BezPath {
    let mut ctx = StrokeCtx::default();
    stroke_with(path, style, _opts, tolerance, &mut ctx);

    ctx.output
}

/// Expand a filled path.
///
/// This applies a contour-aware offset to each subpath, which is useful for
/// expanding or shrinking glyph outlines. Positive values expand the filled
/// region and negative values shrink it.
///
/// The interpretation of "outward" depends on subpath winding. Each contour is
/// offset according to its own winding direction, so the result is most
/// predictable when the input uses consistent fill winding conventions.
///
/// For font outlines, that convention is usually already present: outer
/// contours and counters are typically wound in opposite directions. In that
/// case, expanding grows the outer contour and shrinks counters, while negative
/// expansion does the reverse.
///
/// If different subpaths use inconsistent winding, `expand` will still offset
/// each contour according to the winding it observes, but the result may not
/// match the caller's intended notion of "outer" and "hole". In particular,
/// two visually nested contours with the same winding will both expand in the
/// same winding-relative direction rather than being treated automatically as
/// outer contour and hole.
pub fn expand(
    path: &BezPath,
    expand: Vec2,
    join: Join,
    miter_limit: f64,
    tolerance: f64,
) -> BezPath {
    let params = ExpandParams {
        expand,
        join,
        miter_limit,
    };
    let contour_sign = if path.area() < 0.0 { -1.0 } else { 1.0 };
    // If `expand` has the same sign on both axes, scale into a space where the
    // requested anisotropic expand becomes a unit-radius scalar offset.
    if let Some(normalized) = normalized_expand_space(expand, tolerance) {
        expand_same_sign(path, params, normalized, contour_sign, tolerance)
    } else {
        // Mixed-sign or zero-axis expansion stays in original space and uses the flattened path.
        expand_anisotropic_flattened(path.iter(), params, tolerance, contour_sign)
    }
}

/// Expand a same-sign anisotropic fill, using normalized-space curves when they are beneficial.
fn expand_same_sign(
    path: &BezPath,
    params: ExpandParams,
    normalized: NormalizedExpandSpace,
    contour_sign: f64,
    tolerance: f64,
) -> BezPath {
    if !path
        .elements()
        .iter()
        .any(|el| matches!(el, PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _)))
    {
        // Purely linear contours do not need normalized scalar handling; the
        // original-space anisotropic polyline path keeps support-face corner
        // selection consistent for these contours.
        return expand_anisotropic_flattened(path.iter(), params, tolerance, contour_sign);
    }

    let mut result = BezPath::new();
    if append_normalized_curves(
        path,
        normalized,
        contour_sign,
        params.join,
        params.miter_limit,
        &mut result,
    ) {
        // Prefer the curve-preserving result when the quadratic/cubic approximation succeeds.
        result
    } else {
        // Fall back to the flattened normalized-space implementation for hard curve cases.
        expand_normalized_flattened(
            path,
            normalized,
            contour_sign,
            params.join,
            params.miter_limit,
        )
    }
}

/// Expand in normalized space by flattening first, then offsetting the
/// resulting polyline with scalar-radius join logic.
fn expand_normalized_flattened(
    path: &BezPath,
    normalized: NormalizedExpandSpace,
    contour_sign: f64,
    join: Join,
    miter_limit: f64,
) -> BezPath {
    let scalar_style = Stroke::new(2.0 * normalized.radius.abs())
        .with_caps(Cap::Round)
        .with_join(join)
        .with_miter_limit(miter_limit);
    expand_normalized_flattened_impl(
        path.iter().map(|el| normalized.to_unit * el),
        scalar_style,
        normalized.radius,
        normalized.from_unit,
        normalized.tolerance,
        contour_sign,
    )
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
    _opts: &StrokeOpts,
    tolerance: f64,
    ctx: &mut StrokeCtx,
) {
    if style.dash_pattern.is_empty() {
        stroke_undashed(path, style, tolerance, ctx);
    } else {
        let dashed = dash(path.into_iter(), style.dash_offset, &style.dash_pattern);
        stroke_undashed(dashed, style, tolerance, ctx);
    }
}

/// Version of stroke expansion for styles with no dashes.
fn stroke_undashed(
    path: impl IntoIterator<Item = PathEl>,
    style: &Stroke,
    tolerance: f64,
    ctx: &mut StrokeCtx,
) {
    ctx.reset();
    ctx.join_thresh = 2.0 * tolerance / style.width.max(f64::EPSILON);

    for el in path {
        let p0 = ctx.last_pt;
        match el {
            PathEl::MoveTo(_) | PathEl::LineTo(_) | PathEl::ClosePath => {
                stroke_undashed_line_el(el, p0, style, ctx);
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != p0 || p2 != p0 {
                    let q = QuadBez::new(p0, p1, p2);
                    let (tan0, tan1) = PathSeg::Quad(q).tangents();
                    ctx.do_join(style, tan0);
                    ctx.do_cubic(style, q.raise(), tolerance);
                    ctx.last_tan = tan1;
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != p0 || p2 != p0 || p3 != p0 {
                    let c = CubicBez::new(p0, p1, p2, p3);
                    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
                    ctx.do_join(style, tan0);
                    ctx.do_cubic(style, c, tolerance);
                    ctx.last_tan = tan1;
                }
            }
        }
    }
    ctx.finish(style);
}

fn stroke_undashed_line_el(el: PathEl, p0: Point, style: &Stroke, ctx: &mut StrokeCtx) {
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
        PathEl::ClosePath => {
            if p0 != ctx.start_pt {
                let tangent = ctx.start_pt - p0;
                ctx.do_join(style, tangent);
                ctx.last_tan = tangent;
                ctx.do_line(style, tangent, ctx.start_pt);
            }
            ctx.finish_closed(style);
        }
        PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => unreachable!(),
    }
}

#[derive(Copy, Clone)]
struct NormalizedExpandSpace {
    to_unit: Affine,
    from_unit: Affine,
    radius: f64,
    tolerance: f64,
}

#[derive(Copy, Clone)]
enum OffsetSeg {
    Line(Point),
    Cubic(CubicBez),
}

struct OffsetSegInfo {
    center: Point,
    start: Point,
    tan_start: Vec2,
    tan_end: Vec2,
    seg: OffsetSeg,
}

#[derive(Copy, Clone)]
struct ExpandParams {
    expand: Vec2,
    join: Join,
    miter_limit: f64,
}

/// Emit one completed original-space contour for flattened anisotropic expansion.
fn flush_anisotropic_flattened_subpath(
    out: &mut BezPath,
    tolerance: f64,
    contour_sign: f64,
    params: ExpandParams,
    points: &mut Vec<Point>,
) {
    match points.as_slice() {
        [p0, p1] => {
            let degenerate_style = Stroke::new(0.0)
                .with_caps(Cap::Round)
                .with_join(params.join)
                .with_miter_limit(params.miter_limit);
            let stroked = stroke(
                [PathEl::MoveTo(*p0), PathEl::LineTo(*p1)],
                &degenerate_style,
                &StrokeOpts::default(),
                tolerance,
            );
            out.extend(stroked.iter());
        }
        [_, _, ..] => offset_closed_polyline_into(points, params, contour_sign, tolerance, out),
        _ => {}
    }
    points.clear();
}

/// Flatten the path once and expand each contour in the original coordinate space.
fn expand_anisotropic_flattened(
    path: impl IntoIterator<Item = PathEl>,
    params: ExpandParams,
    tolerance: f64,
    contour_sign: f64,
) -> BezPath {
    let mut result = BezPath::new();
    let mut points = Vec::new();

    crate::flatten(path, tolerance, |el| match el {
        PathEl::MoveTo(p) => {
            // Each closed contour is expanded independently once its flattened
            // point sequence is complete.
            flush_anisotropic_flattened_subpath(
                &mut result,
                tolerance,
                contour_sign,
                params,
                &mut points,
            );
            points.push(p);
        }
        PathEl::LineTo(p) => points.push(p),
        PathEl::ClosePath => flush_anisotropic_flattened_subpath(
            &mut result,
            tolerance,
            contour_sign,
            params,
            &mut points,
        ),
        PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => unreachable!(),
    });
    flush_anisotropic_flattened_subpath(&mut result, tolerance, contour_sign, params, &mut points);
    result
}

/// Emit one completed normalized-space contour for flattened scalar expansion.
fn flush_scalar_flattened_subpath(
    out: &mut BezPath,
    tolerance: f64,
    contour_sign: f64,
    style: &Stroke,
    radius: f64,
    from_unit: Affine,
    points: &mut Vec<Point>,
) {
    match points.as_slice() {
        [p0, p1] => {
            let stroked = stroke(
                [PathEl::MoveTo(*p0), PathEl::LineTo(*p1)],
                style,
                &StrokeOpts::default(),
                tolerance,
            );
            out.extend(stroked.iter().map(|el| from_unit * el));
        }
        [_, _, ..] => offset_closed_polyline_scalar_into(
            points,
            contour_sign,
            radius,
            style.join,
            style.miter_limit,
            tolerance,
            from_unit,
            out,
        ),
        _ => {}
    }
    points.clear();
}

/// Flatten the normalized path once and expand each contour with scalar-radius logic.
fn expand_normalized_flattened_impl(
    path: impl IntoIterator<Item = PathEl>,
    style: Stroke,
    radius: f64,
    from_unit: Affine,
    tolerance: f64,
    contour_sign: f64,
) -> BezPath {
    let mut result = BezPath::new();
    let mut points = Vec::new();

    crate::flatten(path, tolerance, |el| match el {
        PathEl::MoveTo(p) => {
            flush_scalar_flattened_subpath(
                &mut result,
                tolerance,
                contour_sign,
                &style,
                radius,
                from_unit,
                &mut points,
            );
            points.push(p);
        }
        PathEl::LineTo(p) => points.push(p),
        PathEl::ClosePath => flush_scalar_flattened_subpath(
            &mut result,
            tolerance,
            contour_sign,
            &style,
            radius,
            from_unit,
            &mut points,
        ),
        PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => unreachable!(),
    });
    flush_scalar_flattened_subpath(
        &mut result,
        tolerance,
        contour_sign,
        &style,
        radius,
        from_unit,
        &mut points,
    );
    result
}

/// Build the affine-normalized scalar-offset representation for a same-sign anisotropic expand.
fn normalized_expand_space(expand: Vec2, tolerance: f64) -> Option<NormalizedExpandSpace> {
    // Same-sign axis-aligned expansion is equivalent to unit-radius offsetting
    // after scaling into ellipse-normalized coordinates.
    let common_sign = if expand.x != 0.0 {
        expand.x.signum()
    } else {
        expand.y.signum()
    };
    if common_sign == 0.0 {
        return None;
    }
    if expand.x != 0.0 && expand.y != 0.0 && expand.y.signum() != common_sign {
        return None;
    }
    let max_scale = expand.x.abs().max(expand.y.abs());
    let zero_axis_scale = (max_scale * 1e-3).max(tolerance * 1e-3).max(f64::EPSILON);
    let scale_x = if expand.x == 0.0 {
        zero_axis_scale
    } else {
        expand.x.abs()
    };
    let scale_y = if expand.y == 0.0 {
        zero_axis_scale
    } else {
        expand.y.abs()
    };
    Some(NormalizedExpandSpace {
        to_unit: Affine::scale_non_uniform(1.0 / scale_x, 1.0 / scale_y),
        from_unit: Affine::scale_non_uniform(scale_x, scale_y),
        radius: common_sign,
        tolerance: tolerance / scale_x.max(scale_y),
    })
}

/// Append all contours using the curve-preserving normalized-space path.
fn append_normalized_curves(
    path: &BezPath,
    normalized: NormalizedExpandSpace,
    contour_sign: f64,
    join: Join,
    miter_limit: f64,
    out: &mut BezPath,
) -> bool {
    let mut subpath = Vec::new();

    for el in path.iter().map(|el| normalized.to_unit * el) {
        if matches!(el, PathEl::MoveTo(_)) && !subpath.is_empty() {
            if !append_subpath_scalar_curve(
                &subpath,
                contour_sign,
                normalized.radius,
                join,
                miter_limit,
                normalized.tolerance,
                normalized.from_unit,
                out,
            ) {
                return false;
            }
            subpath.clear();
        }
        subpath.push(el);
    }

    append_subpath_scalar_curve(
        &subpath,
        contour_sign,
        normalized.radius,
        join,
        miter_limit,
        normalized.tolerance,
        normalized.from_unit,
        out,
    )
}

/// Expand a single normalized-space contour using line offsets and quadratic-derived curve offsets.
fn append_subpath_scalar_curve(
    subpath: &[PathEl],
    contour_sign: f64,
    radius: f64,
    join: Join,
    miter_limit: f64,
    tolerance: f64,
    from_unit: Affine,
    out: &mut BezPath,
) -> bool {
    let mut current = match subpath.first().copied() {
        None => return true,
        Some(PathEl::MoveTo(p)) => p,
        Some(PathEl::LineTo(p)) | Some(PathEl::QuadTo(_, p)) | Some(PathEl::CurveTo(_, _, p)) => p,
        Some(PathEl::ClosePath) => return true,
    };
    let start = current;
    let mut segs = Vec::new();
    // Positive contour winding and positive radius should expand outward, so
    // the scalar offset distance in normalized space is the opposite sign.
    let signed_distance = -radius * contour_sign;

    for &el in &subpath[1..] {
        match el {
            PathEl::MoveTo(_) => break,
            PathEl::LineTo(p1) => {
                if p1 != current {
                    segs.push(offset_line_segment(current, p1, contour_sign, radius));
                    current = p1;
                }
            }
            PathEl::QuadTo(p1, p2) => {
                let quad = QuadBez::new(current, p1, p2);
                let Some(seg) = offset_quad_segment(quad, contour_sign, signed_distance, radius)
                else {
                    return false;
                };
                segs.push(seg);
                current = p2;
            }
            PathEl::CurveTo(p1, p2, p3) => {
                let cubic = CubicBez::new(current, p1, p2, p3);
                // Cubics are approximated by quadratic pieces so the same
                // quadratic offset fit can be reused for all curved segments.
                let Some(spline) = cubic.approx_spline(tolerance) else {
                    return false;
                };
                for quad in spline.to_quads() {
                    let Some(seg) =
                        offset_quad_segment(quad, contour_sign, signed_distance, radius)
                    else {
                        return false;
                    };
                    segs.push(seg);
                }
                current = p3;
            }
            PathEl::ClosePath => {
                if current != start {
                    segs.push(offset_line_segment(current, start, contour_sign, radius));
                    current = start;
                }
            }
        }
    }

    if segs.len() == 1 {
        if let OffsetSeg::Line(line_end) = segs[0].seg {
            let scalar_style = Stroke::new(2.0 * radius.abs())
                .with_caps(Cap::Round)
                .with_join(join)
                .with_miter_limit(miter_limit);
            let stroked = stroke(
                [PathEl::MoveTo(segs[0].start), PathEl::LineTo(line_end)],
                &scalar_style,
                &StrokeOpts::default(),
                tolerance,
            );
            out.extend(stroked.iter().map(|el| from_unit * el));
            return true;
        }
    }

    if let Some((first, rest)) = segs.split_first() {
        let last = segs.last().unwrap();
        let start_norm = scalar_normal_for_tangent(last.tan_end, radius);
        out.move_to(from_unit * (start - start_norm));
        offset_polyline_join_scalar(
            out,
            first.center,
            last.tan_end,
            first.tan_start,
            contour_sign,
            radius,
            join,
            miter_limit,
            tolerance / radius.abs().max(f64::EPSILON),
            tolerance,
            from_unit,
        );
        append_offset_seg_info(out, first, from_unit);
        let mut prev_tan = first.tan_end;
        for seg in rest {
            offset_polyline_join_scalar(
                out,
                seg.center,
                prev_tan,
                seg.tan_start,
                contour_sign,
                radius,
                join,
                miter_limit,
                tolerance / radius.abs().max(f64::EPSILON),
                tolerance,
                from_unit,
            );
            append_offset_seg_info(out, seg, from_unit);
            prev_tan = seg.tan_end;
        }
        out.close_path();
    }
    true
}

/// Append one already-offset segment, folding the preceding join into cubic starts when possible.
fn append_offset_seg_info(out: &mut BezPath, seg: &OffsetSegInfo, from_unit: Affine) {
    match seg.seg {
        OffsetSeg::Line(end) => out.line_to(from_unit * end),
        OffsetSeg::Cubic(cubic) => {
            let start = from_unit * seg.start;
            let current = out
                .elements()
                .last()
                .and_then(PathEl::end_point)
                .unwrap_or(start);
            let delta = current - start;
            // When the join lands on the cubic's tangent line, absorb that
            // join point into the cubic start instead of emitting a visible
            // straight segment back to the fitted offset start.
            out.curve_to(
                from_unit * cubic.p1 + delta,
                from_unit * cubic.p2,
                from_unit * cubic.p3,
            );
        }
    }
}

/// Offset a single line segment in normalized space and record its tangent data for joins.
fn offset_line_segment(p0: Point, p1: Point, contour_sign: f64, radius: f64) -> OffsetSegInfo {
    let tan = contour_sign * (p1 - p0);
    let norm = scalar_normal_for_tangent(tan, radius);
    OffsetSegInfo {
        center: p0,
        start: p0 - norm,
        tan_start: tan,
        tan_end: tan,
        seg: OffsetSeg::Line(p1 - norm),
    }
}

/// Offset a quadratic segment and package the result for contour assembly.
fn offset_quad_segment(
    quad: QuadBez,
    contour_sign: f64,
    signed_distance: f64,
    radius: f64,
) -> Option<OffsetSegInfo> {
    let (raw_tan0, raw_tan1) = PathSeg::Quad(quad).tangents();
    let tan0 = contour_sign * nonzero_tangent(raw_tan0, quad.p2 - quad.p0)?;
    let tan1 = contour_sign * nonzero_tangent(raw_tan1, quad.p2 - quad.p0)?;

    let cubic = offset_quad_cubic(quad, signed_distance)?;
    let start_norm = scalar_normal_for_tangent(tan0, radius);
    Some(OffsetSegInfo {
        center: quad.p0,
        start: quad.p0 - start_norm,
        tan_start: tan0,
        tan_end: tan1,
        seg: OffsetSeg::Cubic(cubic),
    })
}

/// Prefer a nonzero tangent, falling back to a chord direction for degenerate endpoints.
fn nonzero_tangent(tangent: Vec2, fallback: Vec2) -> Option<Vec2> {
    if tangent.hypot2() > f64::EPSILON {
        Some(tangent)
    } else if fallback.hypot2() > f64::EPSILON {
        Some(fallback)
    } else {
        None
    }
}

fn offset_quad_cubic(quad: QuadBez, distance: f64) -> Option<CubicBez> {
    let tan0 = nonzero_tangent(quad.p1 - quad.p0, quad.p2 - quad.p0)?;
    let tan1 = nonzero_tangent(quad.p2 - quad.p1, quad.p2 - quad.p0)?;
    let chord = nonzero_tangent(quad.p2 - quad.p0, tan0)?;
    let utan0 = tan0.normalize();
    let utan1 = tan1.normalize();

    let p0 = quad.p0 + distance * utan0.turn_90();
    let p3 = quad.p2 + distance * utan1.turn_90();
    let m = quad.eval(0.5) + distance * chord.normalize().turn_90();

    let rhs = m - p0.midpoint(p3);
    let det = utan1.cross(utan0);
    let (p1, p2) = if det.abs() > 1e-10 {
        // Fit a cubic that matches offset endpoints, endpoint tangents, and
        // the midpoint sample of the offset quadratic.
        let idet = (8.0 / 3.0) / det;
        let a = (utan1.cross(rhs) * idet).max(0.0);
        let b = (utan0.cross(rhs) * idet).max(0.0);
        (p0 + a * utan0, p3 - b * utan1)
    } else {
        // Near-parallel endpoint tangents are numerically unstable; fall back
        // to a chord-thirds cubic, which is exact for straight lines.
        let chord = p3 - p0;
        (p0 + chord * (1.0 / 3.0), p3 - chord * (1.0 / 3.0))
    };
    Some(CubicBez::new(p0, p1, p2, p3))
}

/// Offset a flattened anisotropic contour directly in the original coordinate space.
fn offset_closed_polyline_into(
    points: &[Point],
    params: ExpandParams,
    contour_sign: f64,
    tolerance: f64,
    out: &mut BezPath,
) {
    let effective_width = 2.0 * params.expand.x.max(params.expand.y);
    let join_thresh = 2.0 * tolerance / effective_width.max(f64::EPSILON);
    let n = points.len();
    let first_tan = contour_sign * (points[1] - points[0]);
    let mut last_tan = contour_sign * (points[0] - points[n - 1]);
    let start_norm = expand_support_for_edge(params.expand, last_tan, first_tan);
    let start_pt = selected_offset_point(points[0], start_norm, true);

    out.move_to(start_pt);
    for i in 0..n {
        let p0 = points[i];
        let p1 = points[(i + 1) % n];
        let tan0 = contour_sign * (p1 - p0);
        let next_tan = contour_sign * (points[(i + 2) % n] - p1);
        offset_polyline_join(
            out,
            p0,
            last_tan,
            tan0,
            params,
            true,
            join_thresh,
            tolerance,
        );
        let norm = expand_support_for_edge(params.expand, tan0, next_tan);
        let end_pt = selected_offset_point(p1, norm, true);
        if i + 1 != n || end_pt != start_pt {
            line_to_if_needed(out, end_pt);
        }
        last_tan = tan0;
    }
    out.close_path();
}

/// Compute the scalar-radius normal used in normalized-space offsetting.
fn scalar_normal_for_tangent(tangent: Vec2, radius: f64) -> Vec2 {
    radius / tangent.hypot() * Vec2::new(-tangent.y, tangent.x)
}

/// Return the sign of a scalar, but preserve exact zeros instead of inheriting signed-zero quirks.
fn axis_sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Compute the support point of the anisotropic expand box for the given tangent.
fn expand_support_for_tangent(expand: Vec2, tangent: Vec2) -> Vec2 {
    Vec2::new(
        -expand.x * axis_sign(tangent.y),
        expand.y * axis_sign(tangent.x),
    )
}

/// Pick a support point for an edge, falling back to the next tangent when the edge lies on a face.
fn expand_support_for_edge(expand: Vec2, tangent: Vec2, fallback_tangent: Vec2) -> Vec2 {
    Vec2::new(
        -expand.x
            * if tangent.y != 0.0 {
                axis_sign(tangent.y)
            } else {
                axis_sign(fallback_tangent.y)
            },
        expand.y
            * if tangent.x != 0.0 {
                axis_sign(tangent.x)
            } else {
                axis_sign(fallback_tangent.x)
            },
    )
}

/// Offset a flattened normalized-space contour and map the result back through `from_unit`.
fn offset_closed_polyline_scalar_into(
    points: &[Point],
    contour_sign: f64,
    radius: f64,
    join: Join,
    miter_limit: f64,
    tolerance: f64,
    from_unit: Affine,
    out: &mut BezPath,
) {
    let join_thresh = tolerance / radius.abs().max(f64::EPSILON);
    let n = points.len();
    let mut last_tan = contour_sign * (points[0] - points[n - 1]);
    let start_norm = scalar_normal_for_tangent(last_tan, radius);
    let start_pt = from_unit * (points[0] - start_norm);

    out.move_to(start_pt);
    for i in 0..n {
        let p0 = points[i];
        let p1 = points[(i + 1) % n];
        let tan0 = contour_sign * (p1 - p0);
        offset_polyline_join_scalar(
            out,
            p0,
            last_tan,
            tan0,
            contour_sign,
            radius,
            join,
            miter_limit,
            join_thresh,
            tolerance,
            from_unit,
        );
        let norm = scalar_normal_for_tangent(tan0, radius);
        let end_pt = from_unit * (p1 - norm);
        if i + 1 != n || end_pt != start_pt {
            line_to_if_needed(out, end_pt);
        }
        last_tan = tan0;
    }
    out.close_path();
}

/// Choose the forward- or backward-side offset point around a contour vertex.
fn selected_offset_point(center: Point, norm: Vec2, use_forward: bool) -> Point {
    if use_forward {
        center - norm
    } else {
        center + norm
    }
}

/// Append a line only when it changes the current point.
fn line_to_if_needed(out: &mut BezPath, point: Point) {
    let current = out.elements().last().and_then(PathEl::end_point);
    if current != Some(point) {
        out.line_to(point);
    }
}

/// Emit the join geometry for an anisotropic polyline offset in original coordinates.
fn offset_polyline_join(
    out: &mut BezPath,
    center: Point,
    prev_tan: Vec2,
    next_tan: Vec2,
    params: ExpandParams,
    use_forward: bool,
    join_thresh: f64,
    tolerance: f64,
) {
    let norm = expand_support_for_tangent(params.expand, next_tan);
    let prev_norm = expand_support_for_tangent(params.expand, prev_tan);
    let cross = prev_tan.cross(next_tan);
    let dot = prev_tan.dot(next_tan);
    let hypot = cross.hypot(dot);
    if hypot == 0.0 || dot > 0.0 && cross.abs() < hypot * join_thresh {
        return;
    }
    let stable_cross = cross.abs() >= hypot * join_thresh;

    let outer = if use_forward {
        cross > 0.0
    } else {
        cross < 0.0
    };
    let next_pt = selected_offset_point(center, norm, use_forward);

    if !outer {
        return;
    }

    match params.join {
        Join::Bevel => line_to_if_needed(out, next_pt),
        Join::Miter => {
            if stable_cross && 2.0 * hypot < (hypot + dot) * params.miter_limit.powi(2) {
                let prev_pt = selected_offset_point(center, prev_norm, use_forward);
                let h = prev_tan.cross(next_pt - prev_pt) / cross;
                line_to_if_needed(out, next_pt - next_tan * h);
            } else {
                line_to_if_needed(out, next_pt);
            }
        }
        Join::Round => {
            let angle = cross.atan2(dot);
            if use_forward {
                round_join(out, tolerance, center, norm, angle);
            } else {
                round_join_rev(out, tolerance, center, -norm, -angle);
            }
        }
    }
}

/// Emit the join geometry for a scalar-radius polyline offset in normalized space.
fn offset_polyline_join_scalar(
    out: &mut BezPath,
    center: Point,
    prev_tan: Vec2,
    next_tan: Vec2,
    contour_sign: f64,
    radius: f64,
    join: Join,
    miter_limit: f64,
    join_thresh: f64,
    tolerance: f64,
    from_unit: Affine,
) {
    let norm = scalar_normal_for_tangent(next_tan, radius);
    let prev_norm = scalar_normal_for_tangent(prev_tan, radius);
    let cross = prev_tan.cross(next_tan);
    let dot = prev_tan.dot(next_tan);
    let hypot = cross.hypot(dot);
    if hypot == 0.0 || dot > 0.0 && cross.abs() < hypot * join_thresh {
        return;
    }
    let stable_cross = cross.abs() >= hypot * join_thresh;

    let outer = if contour_sign > 0.0 {
        cross > 0.0
    } else {
        cross < 0.0
    };
    if !outer {
        return;
    }

    let next_pt = center - norm;
    match join {
        Join::Bevel => line_to_if_needed(out, from_unit * next_pt),
        Join::Miter => {
            if stable_cross && 2.0 * hypot < (hypot + dot) * miter_limit.powi(2) {
                let prev_pt = center - prev_norm;
                let h = prev_tan.cross(next_pt - prev_pt) / cross;
                line_to_if_needed(out, from_unit * (next_pt - next_tan * h));
            } else {
                line_to_if_needed(out, from_unit * next_pt);
            }
        }
        Join::Round => {
            round_join_transformed(
                out,
                tolerance,
                from_unit,
                center,
                norm,
                contour_sign * cross.atan2(dot),
            );
        }
    }
}

/// Emit a round join after an affine map back out of normalized space.
fn round_join_transformed(
    out: &mut BezPath,
    tolerance: f64,
    transform: Affine,
    center: Point,
    norm: Vec2,
    angle: f64,
) {
    let a = transform * Affine::new([norm.x, norm.y, -norm.y, norm.x, center.x, center.y]);
    let arc = Arc::new(Point::ORIGIN, (1.0, 1.0), PI - angle, angle, 0.0);
    arc.to_cubic_beziers(tolerance, |p1, p2, p3| out.curve_to(a * p1, a * p2, a * p3));
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
        if self.forward_path.is_empty() {
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

impl<T: Iterator<Item = PathEl>> Iterator for DashIterator<'_, T> {
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
/// Accuracy is currently hard-coded to 1e-6. This is better than generally
/// expected, and care is taken to get cusps correct, among other things.
pub fn dash<'a>(
    inner: impl Iterator<Item = PathEl> + 'a,
    dash_offset: f64,
    dashes: &'a [f64],
) -> impl Iterator<Item = PathEl> + 'a {
    // ensure that offset is positive and minimal by normalization using period
    let period = dashes.iter().sum();
    let dash_offset = dash_offset.rem_euclid(period);

    let mut dash_ix = 0;
    let mut dash_remaining = dashes[dash_ix] - dash_offset;
    let mut is_active = true;
    // Find place in dashes array for initial offset.
    while dash_remaining < 0.0 {
        dash_ix = (dash_ix + 1) % dashes.len();
        dash_remaining += dashes[dash_ix];
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

impl<'a, T: Iterator<Item = PathEl>> DashIterator<'a, T> {
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

#[cfg(test)]
mod tests {
    use crate::{
        dash, expand, segments, stroke, BezPath,
        Cap::Butt,
        Circle, CubicBez,
        Join::{Bevel, Miter, Round},
        Line, PathEl, PathSeg, Point, Rect, Shape, Stroke, StrokeOpts, Vec2,
    };

    fn assert_path_eq(path: &BezPath, expected: &[PathEl]) {
        assert_eq!(expected, path.elements());
    }

    fn assert_point_close(actual: Point, expected: Point) {
        assert!((actual.x - expected.x).abs() < 1e-9);
        assert!((actual.y - expected.y).abs() < 1e-9);
    }

    fn assert_path_similar(actual: &BezPath, expected: &BezPath, tolerance: f64) {
        let actual = actual.elements();
        let expected = expected.elements();
        assert_eq!(actual.len(), expected.len());
        for (actual, expected) in actual.iter().zip(expected.iter()) {
            match (*actual, *expected) {
                (PathEl::MoveTo(a), PathEl::MoveTo(b)) | (PathEl::LineTo(a), PathEl::LineTo(b)) => {
                    assert!((a.x - b.x).abs() <= tolerance);
                    assert!((a.y - b.y).abs() <= tolerance);
                }
                (PathEl::QuadTo(a1, a2), PathEl::QuadTo(b1, b2)) => {
                    assert!((a1.x - b1.x).abs() <= tolerance);
                    assert!((a1.y - b1.y).abs() <= tolerance);
                    assert!((a2.x - b2.x).abs() <= tolerance);
                    assert!((a2.y - b2.y).abs() <= tolerance);
                }
                (PathEl::CurveTo(a1, a2, a3), PathEl::CurveTo(b1, b2, b3)) => {
                    assert!((a1.x - b1.x).abs() <= tolerance);
                    assert!((a1.y - b1.y).abs() <= tolerance);
                    assert!((a2.x - b2.x).abs() <= tolerance);
                    assert!((a2.y - b2.y).abs() <= tolerance);
                    assert!((a3.x - b3.x).abs() <= tolerance);
                    assert!((a3.y - b3.y).abs() <= tolerance);
                }
                (PathEl::ClosePath, PathEl::ClosePath) => {}
                _ => panic!("path element kinds differ"),
            }
        }
    }

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

    #[test]
    fn directional_expand_fill_preserves_hole_winding() {
        let path = BezPath::from_svg("M0,0 L10,0 L10,10 L0,10 Z M3,3 L3,7 L7,7 L7,3 Z").unwrap();
        let expanded = expand(&path, Vec2::splat(1.0), Miter, 4.0, 0.1);
        let mut subpaths = expanded.subpaths();
        let outer = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        let inner = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        assert!(outer.area() > 100.0);
        assert!(inner.area() < 0.0);
        assert!(inner.area().abs() < 16.0);
    }

    #[test]
    fn directional_expand_bevel_shrinks_hole() {
        let path = BezPath::from_svg("M0,0 L10,0 L10,10 L0,10 Z M3,3 L3,7 L7,7 L7,3 Z").unwrap();
        let expanded = expand(&path, Vec2::splat(1.0), Bevel, 4.0, 0.1);
        let mut subpaths = expanded.subpaths();
        let outer = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        let inner = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        assert!(outer.area() > 100.0);
        assert!(inner.area() < 0.0);
        assert!(inner.area().abs() < 16.0);
    }

    #[test]
    fn directional_expand_round_shrinks_hole() {
        let path = BezPath::from_svg("M0,0 L10,0 L10,10 L0,10 Z M3,3 L3,7 L7,7 L7,3 Z").unwrap();
        let expanded = expand(&path, Vec2::splat(1.0), Round, 4.0, 0.1);
        let mut subpaths = expanded.subpaths();
        let outer = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        let inner = BezPath::from_vec(subpaths.next().unwrap().to_vec());
        assert!(outer.area() > 100.0);
        assert!(inner.area() < 0.0);
        assert!(inner.area().abs() < 16.0);
    }

    #[test]
    fn directional_expand_miter_square_svg() {
        let rect = Rect::new(0.0, 0.0, 50.0, 50.0);
        let expanded = expand(&rect.to_path(0.1), Vec2::splat(5.0), Miter, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-5.0, -5.0)),
                PathEl::LineTo(Point::new(55.0, -5.0)),
                PathEl::LineTo(Point::new(55.0, 55.0)),
                PathEl::LineTo(Point::new(-5.0, 55.0)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_miter_square_svg_anisotropic() {
        let rect = Rect::new(0.0, 0.0, 50.0, 50.0);
        let expanded = expand(&rect.to_path(0.1), Vec2::new(5.0, 10.0), Miter, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-5.0, -10.0)),
                PathEl::LineTo(Point::new(55.0, -10.0)),
                PathEl::LineTo(Point::new(55.0, 60.0)),
                PathEl::LineTo(Point::new(-5.0, 60.0)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_bevel_square_svg() {
        let rect = Rect::new(0.0, 0.0, 50.0, 50.0);
        let expanded = expand(&rect.to_path(0.1), Vec2::splat(5.0), Bevel, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-5.0, -5.0)),
                PathEl::LineTo(Point::new(0.0, -5.0)),
                PathEl::LineTo(Point::new(55.0, -5.0)),
                PathEl::LineTo(Point::new(55.0, 0.0)),
                PathEl::LineTo(Point::new(55.0, 55.0)),
                PathEl::LineTo(Point::new(50.0, 55.0)),
                PathEl::LineTo(Point::new(-5.0, 55.0)),
                PathEl::LineTo(Point::new(-5.0, 50.0)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_round_rect_points() {
        let rect = Rect::new(0.0, 0.0, 50.0, 50.0);
        let expanded = expand(&rect.to_path(0.1), Vec2::splat(5.0), Round, 4.0, 0.1);
        let els = expanded.elements();
        assert_eq!(9, els.len());
        assert_eq!(PathEl::MoveTo(Point::new(-5.0, -5.0)), els[0]);
        match els[1] {
            PathEl::CurveTo(p1, p2, p3) => {
                assert_point_close(p1, Point::new(-5.0, -2.7614237491539666));
                assert_point_close(p2, Point::new(-2.761423749153968, -5.0));
                assert_point_close(p3, Point::new(0.0, -5.0));
            }
            _ => panic!("expected first corner curve"),
        }
        assert_eq!(PathEl::LineTo(Point::new(55.0, -5.0)), els[2]);
        match els[3] {
            PathEl::CurveTo(_, _, p) => assert_point_close(p, Point::new(55.0, 0.0)),
            _ => panic!("expected second corner curve"),
        }
        assert_eq!(PathEl::LineTo(Point::new(55.0, 55.0)), els[4]);
        match els[5] {
            PathEl::CurveTo(_, _, p) => assert_point_close(p, Point::new(50.0, 55.0)),
            _ => panic!("expected third corner curve"),
        }
        assert_eq!(PathEl::LineTo(Point::new(-5.0, 55.0)), els[6]);
        match els[7] {
            PathEl::CurveTo(_, _, p) => assert_point_close(p, Point::new(-5.0, 50.0)),
            _ => panic!("expected fourth corner curve"),
        }
        assert_eq!(PathEl::ClosePath, els[8]);
    }

    #[test]
    fn directional_expand_quad_emits_curves() {
        let path = BezPath::from_vec(vec![
            PathEl::MoveTo(Point::new(0.0, 0.0)),
            PathEl::QuadTo(Point::new(25.0, 40.0), Point::new(50.0, 0.0)),
            PathEl::LineTo(Point::new(0.0, 0.0)),
            PathEl::ClosePath,
        ]);
        let expanded = expand(&path, Vec2::splat(10.0), Round, 4.0, 0.1);
        assert!(expanded
            .elements()
            .iter()
            .any(|el| matches!(el, PathEl::CurveTo(_, _, _))));
    }

    #[test]
    fn directional_expand_zero_axis_ring_does_not_disappear() {
        let outer = Circle::new((0.0, 0.0), 10.0).to_path(0.1);
        let inner = Circle::new((0.0, 0.0), 5.0).to_path(0.1).reverse_subpaths();
        let mut ring = outer;
        ring.extend(inner.iter());

        let expanded = expand(&ring, Vec2::new(0.0, 2.0), Miter, 4.0, 0.1);

        assert!(!expanded.is_empty());
        assert!(expanded.area().abs() > ring.area().abs());
        assert_eq!(expanded.subpaths().count(), 2);
    }

    #[test]
    fn directional_expand_zero_y_capital_l() {
        let path = BezPath::from_svg("M0,0 L0,10 L3,10 L3,3 L10,3 L10,0 Z").unwrap();
        let expanded = expand(&path, Vec2::new(1.5, 0.0), Miter, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-1.5, 0.0)),
                PathEl::LineTo(Point::new(-1.5, 10.0)),
                PathEl::LineTo(Point::new(4.5, 10.0)),
                PathEl::LineTo(Point::new(4.5, 3.0)),
                PathEl::LineTo(Point::new(11.5, 3.0)),
                PathEl::LineTo(Point::new(11.5, 0.0)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_near_zero_y_capital_l() {
        let path = BezPath::from_svg("M0,0 L0,10 L3,10 L3,3 L10,3 L10,0 Z").unwrap();
        let expanded = expand(&path, Vec2::new(1.5, 0.001), Miter, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-1.5, -0.001)),
                PathEl::LineTo(Point::new(-1.5, 10.001)),
                PathEl::LineTo(Point::new(4.5, 10.001)),
                PathEl::LineTo(Point::new(4.5, 3.001)),
                PathEl::LineTo(Point::new(11.5, 3.001)),
                PathEl::LineTo(Point::new(11.5, -0.001)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_capital_l_isotropic() {
        let path = BezPath::from_svg("M0,0 L0,10 L3,10 L3,3 L10,3 L10,0 Z").unwrap();
        let expanded = expand(&path, Vec2::splat(1.5), Miter, 4.0, 0.1);
        assert_path_eq(
            &expanded,
            &[
                PathEl::MoveTo(Point::new(-1.5, -1.5)),
                PathEl::LineTo(Point::new(-1.5, 11.5)),
                PathEl::LineTo(Point::new(4.5, 11.5)),
                PathEl::LineTo(Point::new(4.5, 4.5)),
                PathEl::LineTo(Point::new(11.5, 4.5)),
                PathEl::LineTo(Point::new(11.5, -1.5)),
                PathEl::ClosePath,
            ],
        );
    }

    #[test]
    fn directional_expand_roboto_e_is_continuous_across_small_x_threshold() {
        let path = BezPath::from_svg(
            "M10.359375,-0.359375 Q6.484375,-0.359375 4.0625,2.1875 Q1.640625,4.734375 1.640625,8.984375 \
             L1.640625,9.578125 Q1.640625,12.40625 2.71875,14.625 Q3.796875,16.859375 5.734375,18.109375 \
             Q7.6875,19.375 9.953125,19.375 Q13.65625,19.375 15.703125,16.921875 Q17.765625,14.484375 17.765625,9.9375 \
             L17.765625,8.578125 L4.890625,8.578125 Q4.953125,5.765625 6.53125,4.03125 Q8.109375,2.296875 10.53125,2.296875 \
             Q12.25,2.296875 13.4375,3 Q14.640625,3.703125 15.546875,4.875 L17.53125,3.328125 Q15.140625,-0.359375 10.359375,-0.359375 Z \
             M9.953125,16.703125 Q7.984375,16.703125 6.640625,15.265625 Q5.3125,13.828125 5,11.25 L14.515625,11.25 L14.515625,11.5 \
             Q14.375,13.96875 13.171875,15.328125 Q11.984375,16.703125 9.953125,16.703125 Z",
        )
        .unwrap();

        let a = expand(&path, Vec2::new(0.099, 0.5), Miter, 4.0, 0.1);
        let b = expand(&path, Vec2::new(0.101, 0.5), Miter, 4.0, 0.1);
        assert_path_similar(&a, &b, 0.01);
    }

    #[test]
    fn directional_expand_roboto_e_is_stable_at_zero_x() {
        let path = BezPath::from_svg(
            "M10.359375,-0.359375 Q6.484375,-0.359375 4.0625,2.1875 Q1.640625,4.734375 1.640625,8.984375 \
             L1.640625,9.578125 Q1.640625,12.40625 2.71875,14.625 Q3.796875,16.859375 5.734375,18.109375 \
             Q7.6875,19.375 9.953125,19.375 Q13.65625,19.375 15.703125,16.921875 Q17.765625,14.484375 17.765625,9.9375 \
             L17.765625,8.578125 L4.890625,8.578125 Q4.953125,5.765625 6.53125,4.03125 Q8.109375,2.296875 10.53125,2.296875 \
             Q12.25,2.296875 13.4375,3 Q14.640625,3.703125 15.546875,4.875 L17.53125,3.328125 Q15.140625,-0.359375 10.359375,-0.359375 Z \
             M9.953125,16.703125 Q7.984375,16.703125 6.640625,15.265625 Q5.3125,13.828125 5,11.25 L14.515625,11.25 L14.515625,11.5 \
             Q14.375,13.96875 13.171875,15.328125 Q11.984375,16.703125 9.953125,16.703125 Z",
        )
        .unwrap();

        let a = expand(&path, Vec2::new(0.0, 0.5), Miter, 4.0, 0.1);
        let b = expand(&path, Vec2::new(0.001, 0.5), Miter, 4.0, 0.1);
        assert!((a.area() - b.area()).abs() < 0.05);
    }

    #[test]
    fn directional_expand_roboto_e_bar_is_stable_at_zero_x() {
        let path = BezPath::from_svg(
            "M10.359375,-0.359375 Q6.484375,-0.359375 4.0625,2.1875 Q1.640625,4.734375 1.640625,8.984375 \
             L1.640625,9.578125 Q1.640625,12.40625 2.71875,14.625 Q3.796875,16.859375 5.734375,18.109375 \
             Q7.6875,19.375 9.953125,19.375 Q13.65625,19.375 15.703125,16.921875 Q17.765625,14.484375 17.765625,9.9375 \
             L17.765625,8.578125 L4.890625,8.578125 Q4.953125,5.765625 6.53125,4.03125 Q8.109375,2.296875 10.53125,2.296875 \
             Q12.25,2.296875 13.4375,3 Q14.640625,3.703125 15.546875,4.875 L17.53125,3.328125 Q15.140625,-0.359375 10.359375,-0.359375 Z \
             M9.953125,16.703125 Q7.984375,16.703125 6.640625,15.265625 Q5.3125,13.828125 5,11.25 L14.515625,11.25 L14.515625,11.5 \
             Q14.375,13.96875 13.171875,15.328125 Q11.984375,16.703125 9.953125,16.703125 Z",
        )
        .unwrap();

        let a = expand(&path, Vec2::new(0.0, 1.5), Miter, 4.0, 0.1);
        let b = expand(&path, Vec2::new(0.001, 1.5), Miter, 4.0, 0.1);
        assert!((a.area() - b.area()).abs() < 0.1);
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

    #[test]
    fn dash_negative_offset() {
        let shape = Line::new((0.0, 0.0), (28.0, 0.0));
        let dashes = [4., 2.];
        let pos = segments(dash(shape.path_elements(0.), 60., &dashes)).collect::<Vec<PathSeg>>();
        let neg = segments(dash(shape.path_elements(0.), -60., &dashes)).collect::<Vec<PathSeg>>();
        assert_eq!(neg, pos);
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
