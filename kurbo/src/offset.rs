// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Computation of offset curves of cubic Béziers.
//!
//! Notes on the design of this module, and on its origins, are in
//! [linebender/kurbo#627](https://github.com/linebender/kurbo/issues/627).
//!
//! # The approximation
//!
//! Every approximation here is a linear perturbation of the source curve,
//! `A(t) = C(t) + d D(t)`, where `C` is the source piece, `d` is the offset
//! distance, and the delta `D` is a cubic Bézier approximating the unit
//! normal of `C`. The endpoints of `D` are the unit normals at the endpoints
//! of `C`, and its inner control points are displaced from them along the
//! endpoint tangents, so that `A` is G1 continuous with the true offset.
//! This leaves two parameters, the displacements `a` and `b`, and the three
//! approximations differ only in how these are determined:
//!
//! * arc drawing, in which `D` is the standard cubic approximation of the
//!   unit circular arc between the endpoint normals;
//! * one-point shape control, which solves `D(1/2) = n(1/2)`;
//! * two-point linear, which solves `D(t) · n(t) = 1` at `t = 1/3` and
//!   `t = 2/3`, and is used when the piece contains an inflection.
//!
//! As `D` does not depend on `d`, it is shared by both sides of a stroke
//! wherever no handle needs to be clamped.
//!
//! # Error estimation and subdivision
//!
//! The error of an approximation is estimated from the osculating parabola
//! of the offset curve at eight sample points (see `OsculatingSamples`).
//! Arc drawing can also be accepted on geometric bounds, when the piece is
//! small, thin, or nearly straight relative to the tolerance.
//!
//! Before the recursion begins, the source is split at each sharp turn, and
//! in particular at each near-cusp, and on either side of it (the root
//! subdivision; see `regularize_and_split_tips`). A piece that is not
//! accepted is subdivided at a cusp of the offset if it contains one, and
//! otherwise at its parameter midpoint, except for a piece extending toward
//! a near-cusp from some distance away (see `CubicOffset::split_toward_tip`).

#![allow(
    clippy::neg_cmp_op_on_partial_ord,
    reason = "The tests on which a bound depends are written so that a NaN fails them."
)]

use arrayvec::ArrayVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, PathSeg, QuadBez, Vec2};

/// The minimum distance between a subdivision point and either endpoint of
/// the piece, as a fraction of its parameter range.
const SPLIT_MARGIN: f64 = 1e-6;

/// The allowance for rounding error added to every bound, in units in the
/// last place of the largest coordinate; see [`CubicOffset::accepts`].
const ROUND_ULPS: f64 = 8.0;

/// Maximum recursion depth.
///
/// This is a robustness guard. A piece at this depth is emitted regardless
/// of whether it was accepted, so the limit should only be reached for input
/// that cannot be approximated. Tight tolerances nonetheless require a
/// fairly deep limit: near a sharp turn of the source, the offset traces an
/// arc of radius `|d|` over a very small parameter interval, and repeated
/// bisection takes many levels to resolve it. The value was determined
/// empirically on a corpus of random curves.
const MAX_DEPTH: usize = 12;

/// We never let cusp values have an absolute value smaller than this.
const CUSP_EPSILON: f64 = 1e-12;

/// The largest parameter distance between an endpoint at which the
/// derivative vanishes and the adjacent cusp of the offset, for which the
/// leading-order estimate of the stub is used. See
/// [`CubicOffset::end_cusp`].
const STUB_MAX_PARAM: f64 = 0.05;

/// The largest estimated extent of a stub, as a fraction of the tolerance,
/// for which the stub is smoothed over rather than fitted. The estimate is
/// only to leading order, so this leaves a margin for the higher-order
/// terms.
const STUB_MAX_EXTENT: f64 = 1.0 / 8.0;

/// The smallest determinant for which a linear fit is attempted.
///
/// The determinant of the one-point fit is `sin(theta)`, where `theta` is
/// the angle between the endpoint tangents, so the fit is singular both for
/// a straight piece and for a half turn.
const DET_EPSILON: f64 = 1e-9;

/// Compute an approximate offset curve.
///
/// The offset of `c` by the distance `d` is written to `result`, beginning
/// with a `move_to`. An offset distance of zero reproduces the source curve.
/// See [`offset_cubic_pair`] for computing both sides at once.
pub fn offset_cubic(c: CubicBez, d: f64, tolerance: f64, result: &mut BezPath) {
    result.truncate(0);
    if d == 0.0 {
        result.move_to(c.p0);
        result.curve_to(c.p1, c.p2, c.p3);
        return;
    }
    if c.may_have_slow_point(TURN_MAX_WIDTH, tolerance * DIM_TUNE) {
        offset_cubic_at_slow_point(c, d, tolerance, result);
    } else {
        offset_cubic_root(&c, &c, &[], NO_TIPS, d, tolerance, result);
    }
}

/// Compute approximate offset curves on both sides of a cubic at once.
///
/// This appends an approximation of the offset of `c` by `d` to `positive`,
/// and of the offset by `-d` to `negative`, as `curve_to` segments. Each
/// path must already have a current point, which should be the start of its
/// offset: `c.p0` plus `d` or `-d` times the unit normal there. This matches
/// the needs of a stroker, which builds the two sides of a stroke in
/// separate paths.
///
/// The result is identical to that of [`offset_cubic`] with `d` and with
/// `-d`, apart from the initial `move_to`, but the work that does not depend
/// on the side is shared: regularization and root subdivision of the source,
/// the choice of subdivision points, and the fits themselves.
pub fn offset_cubic_pair(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
) {
    if d == 0.0 {
        positive.curve_to(c.p1, c.p2, c.p3);
        negative.curve_to(c.p1, c.p2, c.p3);
        return;
    }
    if c.may_have_slow_point(TURN_MAX_WIDTH, tolerance * DIM_TUNE) {
        offset_cubic_pair_at_slow_point(c, d, tolerance, positive, negative);
    } else {
        offset_cubic_pair_root(&c, &c, &[], NO_TIPS, d, tolerance, positive, negative);
    }
}

/// [`offset_cubic_pair`] for a curve that may contain a sharp turn; see
/// [`regularize_and_split_tips`].
#[cold]
#[inline(never)]
fn offset_cubic_pair_at_slow_point(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
) {
    let (regularized, tips) = regularize_and_split_tips(&c, tolerance);
    offset_cubic_pair_root(
        &c,
        &regularized,
        &tips.splits,
        tips.tips,
        d,
        tolerance,
        positive,
        negative,
    );
}

/// [`offset_cubic_pair`], given the regularized curve and the parameters of
/// its root subdivision.
#[inline]
#[allow(clippy::too_many_arguments, reason = "the two roots share this")]
fn offset_cubic_pair_root(
    c: &CubicBez,
    c_regularized: &CubicBez,
    splits: &[f64],
    tips: [f64; 2],
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
) {
    let mut pos = CubicOffset::new(*c_regularized, d, tolerance);
    let mut neg = CubicOffset::new(*c_regularized, -d, tolerance);
    pos.tips = tips;
    neg.tips = tips;
    let (tan0, tan1) = PathSeg::Cubic(*c).tangents();
    let utan0 = tan0.normalize();
    let utan1 = tan1.normalize();
    let root = |co: &mut CubicOffset| OffsetRec {
        t0: 0.0,
        t1: 1.0,
        utan0,
        utan1,
        cusp0: co.end_cusp(EndpointSide::Start),
        cusp1: co.end_cusp(EndpointSide::End),
        depth: 0,
    };
    let rec_pos = root(&mut pos);
    let rec_neg = root(&mut neg);
    if splits.is_empty() {
        offset_rec_pair(&pos, &neg, &rec_pos, &rec_neg, positive, negative);
    } else {
        offset_root_pieces_pair(&pos, &neg, &rec_pos, &rec_neg, splits, positive, negative);
    }
}

/// [`offset_rec_pair`] applied to each piece of the root subdivision; the
/// counterpart of [`CubicOffset::offset_root_pieces`] for both sides.
#[cold]
#[inline(never)]
fn offset_root_pieces_pair(
    pos: &CubicOffset,
    neg: &CubicOffset,
    root_pos: &OffsetRec,
    root_neg: &OffsetRec,
    splits: &[f64],
    out_pos: &mut BezPath,
    out_neg: &mut BezPath,
) {
    // Depth 0 is the whole curve.
    let mut rest_pos = OffsetRec {
        depth: 1,
        ..*root_pos
    };
    let mut rest_neg = OffsetRec {
        depth: 1,
        ..*root_neg
    };
    for &t in splits {
        let q_t = pos.q.eval(t).to_vec2();
        if !(q_t.hypot2() > 0.0) {
            continue;
        }
        let utan = q_t.normalize();
        let piece = |co: &CubicOffset, rest: &mut OffsetRec| {
            let cusp = co.cusp_sign(t);
            let piece = OffsetRec {
                t1: t,
                utan1: utan,
                cusp1: cusp,
                ..*rest
            };
            *rest = OffsetRec {
                t0: t,
                utan0: utan,
                cusp0: cusp,
                ..*rest
            };
            piece
        };
        let piece_pos = piece(pos, &mut rest_pos);
        let piece_neg = piece(neg, &mut rest_neg);
        offset_rec_pair(pos, neg, &piece_pos, &piece_neg, out_pos, out_neg);
    }
    offset_rec_pair(pos, neg, &rest_pos, &rest_neg, out_pos, out_neg);
}

/// The recursion of [`CubicOffset::offset_rec`], for both sides at once.
///
/// As long as both sides reject a piece, they are subdivided at the same
/// point, which depends only on the source. Once one side accepts, the
/// other continues on its own, and both continue separately when either
/// side has a cusp in the piece, as cusps of the offset are found and split
/// for each side independently.
fn offset_rec_pair(
    pos: &CubicOffset,
    neg: &CubicOffset,
    rec_pos: &OffsetRec,
    rec_neg: &OffsetRec,
    out_pos: &mut BezPath,
    out_neg: &mut BezPath,
) {
    let mut todo = [true; 2];
    if pos.tries_arc(rec_pos) {
        let seg = pos.piece(rec_pos);
        let mut hull = None;
        for (i, (co, rec, out)) in [(pos, rec_pos, &mut *out_pos), (neg, rec_neg, &mut *out_neg)]
            .into_iter()
            .enumerate()
        {
            if let Some(approx) = co.accept_arc(&seg, rec, &mut hull) {
                out.curve_to(approx.p1, approx.p2, approx.p3);
                todo[i] = false;
            }
        }
    }
    if todo != [true; 2] || rec_pos.has_cusp() || rec_neg.has_cusp() {
        if todo[0] {
            pos.offset_rec_past_arc(rec_pos, out_pos);
        }
        if todo[1] {
            neg.offset_rec_past_arc(rec_neg, out_neg);
        }
        return;
    }
    let seg = pos.piece(rec_pos);
    let [approx_pos, approx_neg] = match pos.approximate_pair(neg, &seg, rec_pos, rec_neg) {
        Some(both) => both,
        None => [
            pos.approximate(&seg, rec_pos),
            neg.approximate(&seg, rec_neg),
        ],
    };
    // A side whose estimate found a pair of cusps is split at the point it
    // reported, independently of the other side.
    if approx_pos.accepted.is_none()
        && approx_neg.accepted.is_none()
        && approx_pos.split.is_none()
        && approx_neg.split.is_none()
        && rec_pos.depth < MAX_DEPTH
    {
        let SubdivisionPoint { t, utan } = pos.find_subdivision_point(rec_pos);
        let kappa_d = pos.curvature_times_d(t);
        let (cusp_pos, cusp_neg) = (kappa_d + 1.0, -kappa_d + 1.0);
        let (pos0, pos1) = rec_pos.split(t, utan, cusp_pos, cusp_pos);
        let (neg0, neg1) = rec_neg.split(t, utan, cusp_neg, cusp_neg);
        offset_rec_pair(pos, neg, &pos0, &neg0, out_pos, out_neg);
        offset_rec_pair(pos, neg, &pos1, &neg1, out_pos, out_neg);
    } else {
        pos.emit_or_subdivide(rec_pos, approx_pos, out_pos);
        neg.emit_or_subdivide(rec_neg, approx_neg, out_neg);
    }
}

/// [`offset_cubic`] for a curve that may contain a sharp turn; see
/// [`regularize_and_split_tips`].
#[cold]
#[inline(never)]
fn offset_cubic_at_slow_point(c: CubicBez, d: f64, tolerance: f64, result: &mut BezPath) {
    let (regularized, tips) = regularize_and_split_tips(&c, tolerance);
    offset_cubic_root(
        &c,
        &regularized,
        &tips.splits,
        tips.tips,
        d,
        tolerance,
        result,
    );
}

/// [`offset_cubic`], given the regularized curve and the parameters of its
/// root subdivision.
#[inline]
fn offset_cubic_root(
    c: &CubicBez,
    c_regularized: &CubicBez,
    splits: &[f64],
    tips: [f64; 2],
    d: f64,
    tolerance: f64,
    result: &mut BezPath,
) {
    let mut co = CubicOffset::new(*c_regularized, d, tolerance);
    co.tips = tips;
    let (tan0, tan1) = PathSeg::Cubic(*c).tangents();
    let utan0 = tan0.normalize();
    let utan1 = tan1.normalize();
    let cusp0 = co.end_cusp(EndpointSide::Start);
    let cusp1 = co.end_cusp(EndpointSide::End);
    result.move_to(c.p0 + d * utan0.turn_90());
    let rec = OffsetRec {
        t0: 0.0,
        t1: 1.0,
        utan0,
        utan1,
        cusp0,
        cusp1,
        depth: 0,
    };
    if splits.is_empty() {
        co.offset_rec(&rec, result);
    } else {
        co.offset_root_pieces(&rec, splits, result);
    }
}

/// The largest width of a tip, in parameter units; see [`TipSplits`].
const TIP_MAX_WIDTH: f64 = 0.01;

/// The largest width of a sharp turn, in parameter units; see
/// [`regularize_and_split_tips`].
///
/// The samples of the error estimate are an eighth of the piece apart, so
/// a turn narrower than about 0.06 can fall between them. The value balances
/// the cost of the additional splits against accuracy, and was determined
/// empirically on the strokes of the Ghostscript tiger.
const TURN_MAX_WIDTH: f64 = 0.03;

/// The distance from a tip to the root subdivision points on either side
/// of it, as a multiple of the width of the tip; see [`TipSplits`].
const TIP_REACH: f64 = 8.0;

/// The counterpart of [`TIP_REACH`] for a sharp turn that is not a tip.
///
/// If the curve were split at the turn alone, the adjacent pieces would
/// have nearly all of their curvature concentrated at that endpoint, before
/// the first sample of the error estimate. The value was determined
/// empirically on a corpus of random curves with sharp turns.
const TURN_REACH: f64 = 4.0;

/// The largest ratio between the distances of the two endpoints of a piece
/// from a tip for which the piece is subdivided at its midpoint; see
/// [`CubicOffset::split_toward_tip`].
const TIP_SPLIT_RATIO: f64 = 2.0;

/// The dimension below which a cusp of the source is regularized, as a
/// fraction of the tolerance. Regularization perturbs the control points so
/// that the derivative does not vanish in the interior of the curve; too
/// large a value distorts the curve.
const DIM_TUNE: f64 = 0.25;

/// Regularize the curve and determine its root subdivision (tip detection).
///
/// This is called only for a curve whose speed may have a pronounced local
/// minimum, `c.may_have_slow_point(TURN_MAX_WIDTH, tolerance * DIM_TUNE)`;
/// for every other curve, which is nearly every curve, the result would be
/// the curve itself with no subdivision points.
///
/// The returned curve is the regularized input. Its root subdivision points
/// are at each sharp turn, and at [`TURN_REACH`] widths on either side of
/// it, or [`TIP_REACH`] widths for a tip.
///
/// A sharp turn is an interior minimum of the speed `|C'|` at which the
/// tangent direction changes rapidly. At the minimum, `C'` is perpendicular
/// to `C''`, and nearby the tangent turns through `atan(s / w)` over a
/// parameter distance `s`, where `w = |C'| / |C''|` is the width of the
/// minimum: nearly a half turn in total, most of it within a few widths.
/// The minimum is a sharp turn if its width is below [`TURN_MAX_WIDTH`], and
/// a tip if it is below [`TIP_MAX_WIDTH`]; see [`TipSplits`].
///
/// A sharp turn in the interior of a piece is invisible to the recursion.
/// The samples of the error estimate can straddle it, and the guards on the
/// estimate compare the cusp values at the endpoints of the piece, while
/// the curvature peaks between them; the offset may even contain two cusps
/// with the same sign of the cusp value at both endpoints. The turns are
/// therefore detected once, here, and made endpoints of pieces.
///
/// The additional subdivision points on either side are needed because a
/// piece that ends at a turn and extends far from it is no easier: nearly
/// all of its curvature is concentrated at that endpoint, before the first
/// sample, and a midpoint subdivision again leaves one half adjacent to the
/// turn. Each piece between a turn and the neighboring subdivision points
/// turns through most of a quarter turn, with the turning distributed
/// evenly enough in the parameter for the estimate and for midpoint
/// subdivision. The parameters of the tips are also recorded, for the
/// subdivision of pieces further away; see [`CubicOffset::split_toward_tip`].
///
/// Regularization looks for a cusp at the same minima of the speed, so the
/// minima are computed once for both purposes.
fn regularize_and_split_tips(c: &CubicBez, tolerance: f64) -> (CubicBez, TipSplits) {
    let dimension = tolerance * DIM_TUNE;
    let minima = c.slow_points(c.cusp_speed_squared(dimension).max(turn_speed_squared(c)));
    if minima.is_empty() {
        return (*c, TipSplits::default());
    }
    match c.regularize_cusp_at(dimension, &minima) {
        None => (*c, tip_splits_at(c, &minima)),
        // Regularization moved the control points, and with them the
        // minima of the speed.
        Some(regularized) => {
            let minima = regularized.slow_points(turn_speed_squared(&regularized));
            (regularized, tip_splits_at(&regularized, &minima))
        }
    }
}

/// The tips of a curve, and its root subdivision points for the tips and
/// for its other sharp turns.
///
/// A tip is the point of a near-cusp of the source: an interior minimum of
/// the speed `|C'|` whose width `w = |C'| / |C''|` is below
/// [`TIP_MAX_WIDTH`]. It lies very near the maximum of the curvature; the
/// two coincide to leading order in the width. Within a few widths of the
/// tip the tangent turns through nearly a half turn, so that the curve
/// resembles a cusp with a rounded point, and on the outside of the turn
/// the offset traces an arc of radius `|d|` about the tip. A sharp turn is
/// defined in the same way with a width below [`TURN_MAX_WIDTH`], so every
/// tip is also a sharp turn.
struct TipSplits {
    /// The parameters of the tips; NaN where there is none.
    tips: [f64; 2],
    /// The root subdivision points, in increasing order.
    splits: ArrayVec<f64, 6>,
}

impl Default for TipSplits {
    fn default() -> Self {
        TipSplits {
            tips: NO_TIPS,
            splits: ArrayVec::new(),
        }
    }
}

/// The tips of a curve that has none.
const NO_TIPS: [f64; 2] = [f64::NAN; 2];

/// An upper bound on `|C'|^2` at a minimum of the speed that is a sharp
/// turn: there `|C'| < TURN_MAX_WIDTH |C''|`, and `|C''|` attains its
/// maximum at an endpoint.
fn turn_speed_squared(c: &CubicBez) -> f64 {
    let c2_start = (c.p2 - c.p1) - (c.p1 - c.p0);
    let c2_end = (c.p3 - c.p2) - (c.p2 - c.p1);
    (36.0 * TURN_MAX_WIDTH * TURN_MAX_WIDTH) * c2_start.hypot2().max(c2_end.hypot2())
}

/// Classify the given minima of the speed, and compute the root subdivision
/// points for the tips and other sharp turns among them; see
/// [`regularize_and_split_tips`].
fn tip_splits_at(c: &CubicBez, minima: &[f64]) -> TipSplits {
    // Subdivision points closer together than this are merged.
    const MIN_GAP: f64 = 1e-14;
    let q = c.deriv();
    let mut result = TipSplits::default();
    let mut n_tips = 0;
    let mut last = 0.0;
    for &t in minima {
        let speed_squared = q.eval(t).to_vec2().hypot2();
        let width_squared = speed_squared / q.deriv().eval(t).to_vec2().hypot2();
        // Written so that a NaN is not classified as a turn.
        if !(width_squared < TURN_MAX_WIDTH * TURN_MAX_WIDTH) {
            continue;
        }
        let reach = if width_squared < TIP_MAX_WIDTH * TIP_MAX_WIDTH {
            result.tips[n_tips] = t;
            n_tips += 1;
            TIP_REACH
        } else {
            TURN_REACH
        } * width_squared.sqrt();
        for split in [t - reach, t, t + reach] {
            if split > last + MIN_GAP && split < 1.0 - MIN_GAP {
                result.splits.push(split);
                last = split;
            }
        }
    }
    result
}

/// State used for computing an offset curve of a single cubic.
struct CubicOffset {
    /// The cubic being offset. This has been regularized.
    c: CubicBez,
    /// The derivative of `c`.
    q: QuadBez,
    /// The offset distance.
    d: f64,
    /// `c0 + c1 t + c2 t^2` is the cross product of the second and first
    /// derivatives of the source curve, multiplied by the offset distance.
    c0: f64,
    c1: f64,
    c2: f64,
    /// The tolerance.
    tolerance: f64,
    /// Whether the offset has a stub at the start and at the end of the
    /// curve that is smoothed over rather than fitted, indexed by
    /// [`EndpointSide`]; see [`CubicOffset::end_cusp`].
    smoothed_stub: [bool; 2],
    /// The parameters of the tips of the curve, or NaN where there are fewer
    /// than two; see [`TipSplits`].
    tips: [f64; 2],
}

/// One endpoint of a curve or of a piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EndpointSide {
    /// The start, at the smaller parameter value.
    Start,
    /// The end, at the larger parameter value.
    End,
}

/// State local to a piece in the recursive subdivision.
struct OffsetRec {
    t0: f64,
    t1: f64,
    /// Unit tangent at `t0`.
    utan0: Vec2,
    /// Unit tangent at `t1`.
    utan1: Vec2,
    /// Cusp value (curvature times offset distance, plus one) at `t0`.
    cusp0: f64,
    /// Cusp value at `t1`.
    cusp1: f64,
    /// Recursion depth.
    depth: usize,
}

impl OffsetRec {
    /// The two pieces resulting from subdivision at `t`.
    ///
    /// The cusp values on either side of `t` are given separately. In the
    /// case of subdividing at a cusp, the cusp value at the subdivision point
    /// is mathematically zero, but we treat it as a signed infinitesimal:
    /// `cusp_t_minus` and `cusp_t_plus` represent the values just before and
    /// just after `t`; see [`CubicOffset::split_at_cusp`].
    fn split(
        &self,
        t: f64,
        utan_t: Vec2,
        cusp_t_minus: f64,
        cusp_t_plus: f64,
    ) -> (OffsetRec, OffsetRec) {
        (
            OffsetRec {
                t0: self.t0,
                t1: t,
                utan0: self.utan0,
                utan1: utan_t,
                cusp0: self.cusp0,
                cusp1: cusp_t_minus,
                depth: self.depth + 1,
            },
            OffsetRec {
                t0: t,
                t1: self.t1,
                utan0: utan_t,
                utan1: self.utan1,
                cusp0: cusp_t_plus,
                cusp1: self.cusp1,
                depth: self.depth + 1,
            },
        )
    }

    /// Whether the cusp value changes sign over the piece.
    fn has_cusp(&self) -> bool {
        self.cusp0 * self.cusp1 < 0.0
    }
}

/// The result of choosing a subdivision point.
struct SubdivisionPoint {
    /// Parameter value on the source curve.
    t: f64,
    /// Unit tangent of the source curve at that parameter.
    utan: Vec2,
}

/// What [`CubicOffset::approximate`] returns.
struct Approximation {
    /// An approximation that was accepted, if there is one.
    accepted: Option<CubicBez>,
    /// The approximation to fall back on if none was accepted and the piece
    /// cannot be subdivided further.
    fallback: CubicBez,
    /// Where to split the piece, if the estimate found two cusps of the
    /// offset inside it.
    split: Option<f64>,
}

impl Approximation {
    /// The approximation `approx`, accepted or rejected according to
    /// `accepted`.
    fn decided(approx: CubicBez, accepted: bool) -> Self {
        Approximation {
            accepted: accepted.then_some(approx),
            fallback: approx,
            split: None,
        }
    }

    /// The approximation `approx`, rejected, with the piece to be subdivided
    /// at `t`.
    fn split_at(approx: CubicBez, t: f64) -> Self {
        Approximation {
            accepted: None,
            fallback: approx,
            split: Some(t),
        }
    }
}

/// An approximation to the offset of one piece, with its delta.
struct Fit {
    /// The approximation itself.
    approx: CubicBez,
    /// The delta `D`, where the approximation is `A = C + d D`. Its control
    /// points are vectors, represented as points.
    delta: CubicBez,
    /// Whether a handle was clamped to zero length, in which case the
    /// approximation differs from the one the parameters describe.
    clamped: bool,
}

impl CubicOffset {
    fn new(c: CubicBez, d: f64, tolerance: f64) -> Self {
        let q = c.deriv();
        let d2 = 2.0 * d;
        let p1xp0 = q.p1.to_vec2().cross(q.p0.to_vec2());
        let p2xp0 = q.p2.to_vec2().cross(q.p0.to_vec2());
        let p2xp1 = q.p2.to_vec2().cross(q.p1.to_vec2());
        CubicOffset {
            c,
            q,
            d,
            c0: d2 * p1xp0,
            c1: d2 * (p2xp0 - 2.0 * p1xp0),
            c2: d2 * (p2xp1 - p2xp0 + p1xp0),
            tolerance,
            smoothed_stub: [false; 2],
            tips: NO_TIPS,
        }
    }

    /// Compute curvature of the source curve times offset plus 1.
    ///
    /// This quantity is called the cusp value because the offset has a cusp
    /// where it crosses zero, that is, where the radius of curvature of the
    /// source equals the offset distance. It equals one at an inflection of
    /// the source. There is a potential division by zero where the derivative
    /// vanishes; this is avoided for interior points by regularizing the
    /// curve beforehand, and for endpoints by using [`Self::end_cusp`].
    fn cusp_sign(&self, t: f64) -> f64 {
        self.curvature_times_d(t) + 1.0
    }

    /// The curvature of the source at `t` times the offset distance. For the
    /// other side of the source, the value is negated.
    #[inline(always)]
    fn curvature_times_d(&self, t: f64) -> f64 {
        let ds2 = self.q.eval(t).to_vec2().hypot2();
        ((self.c2 * t + self.c1) * t + self.c0) / (ds2 * ds2.sqrt())
    }

    /// The cusp value times `|C'|^3`, and its derivative with respect to `t`.
    ///
    /// This has the same sign as the cusp value, but unlike the cusp value it
    /// has no pole where the derivative of the source vanishes, which makes
    /// it better suited to Newton's method.
    fn scaled_cusp_and_deriv(&self, t: f64) -> (f64, f64) {
        let q = self.q.eval(t).to_vec2();
        let q_deriv = self.q.deriv().eval(t).to_vec2();
        let m = q.hypot2();
        let m_deriv = 2.0 * q.dot(q_deriv);
        let n = (self.c2 * t + self.c1) * t + self.c0;
        let n_deriv = 2.0 * self.c2 * t + self.c1;
        let speed = m.sqrt();
        (n + m * speed, n_deriv + 1.5 * speed * m_deriv)
    }

    /// Find a zero of the cusp value in `[a, b]`, given that `s` times the
    /// cusp value is negative at `a` and positive at `b`.
    ///
    /// This applies Newton's method to [`Self::scaled_cusp_and_deriv`],
    /// safeguarded by bisection, to an accuracy of about `1e-12` in `t`. It
    /// requires far fewer evaluations than the ITP solver previously applied
    /// to the cusp value itself, and none of them involves a division.
    fn find_cusp(&self, mut a: f64, mut b: f64, s: f64) -> f64 {
        const EPS: f64 = 1e-12;
        let mut t = 0.5 * (a + b);
        let mut step_old = b - a;
        for _ in 0..100 {
            #[cfg(test)]
            tests::CUSP_ITERATIONS.with(|count| count.set(count.get() + 1));
            let (y, dy) = self.scaled_cusp_and_deriv(t);
            let (y, dy) = (s * y, s * dy);
            if y > 0.0 {
                b = t;
            } else if y < 0.0 {
                a = t;
            } else {
                return t;
            }
            if b - a <= 2.0 * EPS {
                break;
            }
            let step = y / dy;
            // A step this small terminates the search regardless of where it
            // leads. Newton's method typically approaches the root from one
            // side, leaving the other end of the bracket at its initial
            // position, and its final steps are lost to rounding. Treating
            // those as leaving the bracket would revert to bisection of the
            // whole bracket, long after the root has been found.
            if step.abs() < EPS {
                return (t - step).clamp(a, b);
            }
            let newton = t - step;
            // Bisect if the Newton step leaves the bracket, or if it is not
            // converging faster than bisection would.
            if newton > a && newton < b && (2.0 * y).abs() <= (step_old * dy).abs() {
                step_old = step.abs();
                t = newton;
            } else {
                step_old = 0.5 * (b - a);
                t = 0.5 * (a + b);
            }
        }
        0.5 * (a + b)
    }

    /// The cusp value at the start (`t = 0`) or the end (`t = 1`) of the
    /// curve.
    ///
    /// Where the derivative vanishes at the endpoint, the curvature there is
    /// infinite, and so is the cusp value. Its numerator `c0 + c1 t + c2 t^2`
    /// is then exactly `c2 (t - t_end)^2`, so the cusp value tends to
    /// infinity with the sign of `c2`. When that sign is negative, the offset
    /// has a cusp near the endpoint, on the inside of the curve, and between
    /// the cusp and the endpoint the offset runs backward. We call this short
    /// backward segment a stub. Only the sign of the cusp value matters to
    /// the recursion, so its magnitude is taken as 1.
    ///
    /// The stub is reported as a negative cusp value, so that the cusp is
    /// found and the stub is fitted, unless the stub is too small to matter.
    /// The error estimate does not otherwise detect it, and the result can
    /// then be far out of tolerance.
    ///
    /// This also records whether the stub at that endpoint is smoothed over,
    /// for use by [`Self::handles`], and so it is called exactly once for
    /// each endpoint, at the root.
    fn end_cusp(&mut self, side: EndpointSide) -> f64 {
        let (tan, y) = match side {
            EndpointSide::Start => (self.q.p0.to_vec2(), self.c0),
            EndpointSide::End => (self.q.p2.to_vec2(), self.c0 + self.c1 + self.c2),
        };
        const TAN_DIST_EPSILON: f64 = 1e-12;
        let tan_dist = tan.hypot();
        if tan_dist >= TAN_DIST_EPSILON {
            let rsqrt = 1.0 / tan_dist;
            return y * (rsqrt * rsqrt * rsqrt) + 1.0;
        }
        let smoothed = self.smooths_stub(side);
        self.smoothed_stub[side as usize] = smoothed;
        if !(self.c2 < 0.0) || smoothed {
            1.0
        } else {
            -1.0
        }
    }

    /// Whether the offset has a stub at the given endpoint that is small
    /// enough to be smoothed over. The derivative is known to vanish at that
    /// endpoint.
    ///
    /// Let `q1` be the middle control point of the derivative and `q_other`
    /// the one at the opposite endpoint. To leading order, the displacement
    /// of the offset along the endpoint tangent is `|q1| u^2 - d k u`, where
    /// `u` is the parameter distance from the endpoint and
    /// `k = (q1 x q_other) / (2 |q1|^2)` is the rate of change of the tangent
    /// direction. The offset reverses direction at the cusp,
    /// `u = d k / (2 |q1|)`, having traveled a distance `(d k)^2 / (4 |q1|)`.
    ///
    /// A smoothed stub keeps the cusp value 1 at that endpoint, but the fit
    /// retains a handle pointing back along the stub rather than clamping it.
    /// With the handle clamped, the fit would fall back to arc drawing, which
    /// could exceed the tolerance by several times even for a stub much
    /// smaller than the tolerance.
    fn smooths_stub(&self, side: EndpointSide) -> bool {
        if !(self.c2 < 0.0) {
            return false;
        }
        let q1 = self.q.p1.to_vec2();
        let other = match side {
            EndpointSide::Start => self.q.p2,
            EndpointSide::End => self.q.p0,
        }
        .to_vec2();
        // With `m = |d (q1 x q_other)|`, the cusp is at `u = m / (4 |q1|^3)`
        // and the extent of the stub is `m^2 / (16 |q1|^5)`; both are
        // compared without division.
        let len2 = q1.hypot2();
        let len = len2.sqrt();
        let m = (self.d * q1.cross(other)).abs();
        len2 > 0.0
            && m <= 4.0 * STUB_MAX_PARAM * len2 * len
            && m * m <= 16.0 * STUB_MAX_EXTENT * self.tolerance * len2 * len2 * len
    }

    /// Primary entry point for recursive subdivision.
    ///
    /// Arc drawing is tried first, on the bounds that need nothing but the
    /// fit; see [`Self::accept_arc`].
    fn offset_rec(&self, rec: &OffsetRec, result: &mut BezPath) {
        if self.tries_arc(rec) {
            if let Some(approx) = self.accept_arc(&self.piece(rec), rec, &mut None) {
                result.curve_to(approx.p1, approx.p2, approx.p3);
                return;
            }
        }
        self.offset_rec_past_arc(rec, result);
    }

    /// [`Self::offset_rec`] for a piece that [`Self::accept_arc`] did not
    /// accept: split at a cusp of the offset if there is one, and otherwise
    /// approximate the piece and either emit or subdivide it.
    fn offset_rec_past_arc(&self, rec: &OffsetRec, result: &mut BezPath) {
        // If the sign of the cusp value differs between the endpoints, there
        // is a cusp in the interior. Find it and subdivide there.
        if rec.has_cusp() {
            let a = rec.t0;
            let b = rec.t1;
            let s = rec.cusp1.signum();
            let t = self.find_cusp(a, b, s);
            // A root at an endpoint of the piece indicates that the sign
            // there is spurious, as at an endpoint where the derivative
            // vanishes and the stub was smoothed over: the whole piece is in
            // fact past the cusp. Instead of splitting off a piece of zero
            // length, give that endpoint the sign of the other one.
            let margin = SPLIT_MARGIN * (b - a);
            if t <= a + margin || t >= b - margin {
                let side = if t <= a + margin {
                    EndpointSide::Start
                } else {
                    EndpointSide::End
                };
                self.offset_rec_without_cusp(rec, side, result);
                return;
            }
            // The derivative may vanish near an endpoint of the piece, and
            // the solver may converge there. Use the endpoint tangent in that
            // case.
            const UTAN_EPSILON: f64 = 1e-12;
            let q_t = self.q.eval(t).to_vec2();
            let utan_t = if q_t.length_squared() >= UTAN_EPSILON {
                q_t.normalize()
            } else if t - rec.t0 < rec.t1 - t {
                rec.utan0
            } else {
                rec.utan1
            };
            let cusp_t_minus = CUSP_EPSILON.copysign(rec.cusp0);
            let cusp_t_plus = CUSP_EPSILON.copysign(rec.cusp1);
            self.split_at_cusp(rec, result, t, utan_t, cusp_t_minus, cusp_t_plus);
            return;
        }

        let seg = self.piece(rec);
        let approximation = self.approximate(&seg, rec);
        self.emit_or_subdivide(rec, approximation, result);
    }

    /// [`Self::offset_rec`] applied to each piece of the root subdivision
    /// computed by [`regularize_and_split_tips`].
    #[cold]
    #[inline(never)]
    fn offset_root_pieces(&self, root: &OffsetRec, splits: &[f64], result: &mut BezPath) {
        // Depth 0 is the whole curve.
        let mut rest = OffsetRec { depth: 1, ..*root };
        for &t in splits {
            let q_t = self.q.eval(t).to_vec2();
            // At an exact cusp of the source there is no tangent, so no
            // subdivision is made there.
            if !(q_t.hypot2() > 0.0) {
                continue;
            }
            let utan = q_t.normalize();
            let cusp = self.cusp_sign(t);
            let piece = OffsetRec {
                t1: t,
                utan1: utan,
                cusp1: cusp,
                ..rest
            };
            self.offset_rec(&piece, result);
            rest = OffsetRec {
                t0: t,
                utan0: utan,
                cusp0: cusp,
                ..rest
            };
        }
        self.offset_rec(&rest, result);
    }

    /// Subdivide at a cusp of the offset, without counting the subdivision
    /// toward the depth limit.
    ///
    /// Neither resulting piece contains that cusp, so the number of such
    /// subdivisions is bounded by the number of cusps of the offset. Counting
    /// them would leave the pieces adjacent to a cusp short of depth when the
    /// offset has several. Depth 0 remains reserved for the whole curve.
    ///
    /// This is kept out of line, as inlining it into [`Self::offset_rec`]
    /// made curves without cusps measurably slower.
    #[inline(never)]
    fn split_at_cusp(
        &self,
        rec: &OffsetRec,
        result: &mut BezPath,
        t: f64,
        utan_t: Vec2,
        cusp_t_minus: f64,
        cusp_t_plus: f64,
    ) {
        let (mut rec0, mut rec1) = rec.split(t, utan_t, cusp_t_minus, cusp_t_plus);
        rec0.depth = rec.depth.max(1);
        rec1.depth = rec0.depth;
        self.offset_rec(&rec0, result);
        self.offset_rec(&rec1, result);
    }

    /// [`Self::offset_rec`] for a piece whose cusp lies at one of its
    /// endpoints, with the cusp value at that endpoint given the sign of the
    /// other one.
    #[cold]
    #[inline(never)]
    fn offset_rec_without_cusp(&self, rec: &OffsetRec, side: EndpointSide, result: &mut BezPath) {
        let mut rec = OffsetRec { ..*rec };
        match side {
            EndpointSide::Start => rec.cusp0 = CUSP_EPSILON.copysign(rec.cusp1),
            EndpointSide::End => rec.cusp1 = CUSP_EPSILON.copysign(rec.cusp0),
        }
        self.offset_rec(&rec, result);
    }

    /// Approximate the offset of one piece.
    ///
    /// See [`Approximation`] for what this returns.
    #[inline]
    fn approximate(&self, seg: &CubicBez, rec: &OffsetRec) -> Approximation {
        let d = self.d;
        let tolerance = self.tolerance;

        // A piece whose chord is shorter than the tolerance lies at or near
        // the tip of a near-cusp of the source. Its offset is nearly a
        // circular arc about the tip, however poorly parametrized, and arc
        // drawing with a purely geometric bound handles it.
        if (seg.p3 - seg.p0).hypot2() <= tolerance * tolerance {
            let arc = self.draw_arc(seg, rec);
            let middle = self.hodograph_middle(rec);
            if let Some(bound) = small_piece_bound(seg, rec, middle, &arc, d) {
                if self.accepts(bound, seg, &arc.approx) {
                    return Approximation::decided(arc.approx, true);
                }
            }
        }

        // An inflection is a sign change of the curvature, which shows as the
        // cusp value crossing one. When the piece contains an inflection its
        // endpoint tangents may be parallel, where one-point is singular, so
        // two-point linear is used. Otherwise one-point is used, as it is of
        // higher order and also controls the tangential error.
        let has_inflection = (rec.cusp0 - 1.0) * (rec.cusp1 - 1.0) < 0.0;
        let linear_fit = if has_inflection {
            two_point(seg, rec)
        } else {
            one_point(seg, rec)
        };
        // Arc drawing replaces a linear fit that would require clamping a
        // handle.
        let fit = match linear_fit {
            Some((a, b)) if !self.clamps(seg, rec, a, b) => self.apply(seg, rec, a, b),
            _ => self.draw_arc(seg, rec),
        };
        // The osculating parabola estimate is trusted where its leading-order
        // model is valid: the curvature times `d` varies little over the
        // piece, there is no cusp at the samples, and a two-point fit does
        // not slip far along the offset.
        if self.tries_osculating(rec) {
            let limit = tolerance / (OSCULATING_SAFETY * d.abs());
            // Only the two-point fit leaves the slip unconstrained. With
            // one-point and with arc drawing, a large slip was not observed
            // to make the estimate fall short, and limiting it sent most
            // pieces with a vanishing endpoint derivative to the fallback.
            let two_point_fit = has_inflection && linear_fit.is_some() && !fit.clamped;
            let max_slip = if two_point_fit {
                OSCULATING_MAX_SLIP
            } else {
                f64::INFINITY
            };
            let sign = 1.0_f64.copysign(rec.cusp0);
            match osculating_decision(seg, &fit.delta, d, limit, max_slip, sign) {
                Osculating::Decided(accepted) => {
                    return Approximation::decided(fit.approx, accepted);
                }
                // The cusp value at a sample has the opposite sign from the
                // one at both endpoints, so the piece contains at least two
                // cusps. Subdivide at the sample, so that each half contains
                // a detectable sign change.
                Osculating::Cusp(t) => {
                    return Approximation::split_at(fit.approx, rec.t0 + t * (rec.t1 - rec.t0));
                }
                Osculating::Untrusted => {}
            }
        }
        Approximation::decided(fit.approx, self.accept_fallback(seg, rec, &fit))
    }

    /// Whether [`Self::accept_arc`] is worth trying on a piece.
    ///
    /// Both of its bounds measure the distance at equal parameter, and the
    /// hull bound can only succeed if the endpoint normals, scaled by `d`,
    /// are within the tolerance of each other. The hairline bound succeeds
    /// for any piece when the stroke is thin enough.
    #[inline(always)]
    fn tries_arc(&self, rec: &OffsetRec) -> bool {
        self.d * self.d * (rec.utan1 - rec.utan0).hypot2() <= self.tolerance * self.tolerance
            || 2.0 * self.d.abs() <= self.tolerance
    }

    /// The arc drawing approximation of a piece, if one of the two strict
    /// geometric bounds, which require no error estimate, accepts it.
    ///
    /// For a thin stroke, a piece of any shape can be accepted: the hairline
    /// bound is at least `2 |d|`, so it is only tried when that is within
    /// the tolerance. A piece with very little turning can be accepted by the
    /// hull bound, which also covers straight and nearly straight pieces,
    /// for which the linear fits are ill-conditioned.
    ///
    /// Both bounds are on the distance at equal parameter, which remains
    /// valid across cusps of the offset, so this test precedes subdivision
    /// at a cusp.
    ///
    /// With no handle clamped, `D` is the same on both sides of the source,
    /// and so is the hull bound; `hull` caches it for the other side.
    #[inline(always)]
    fn accept_arc(
        &self,
        seg: &CubicBez,
        rec: &OffsetRec,
        hull: &mut Option<Option<f64>>,
    ) -> Option<CubicBez> {
        let d = self.d;
        let arc = self.draw_arc(seg, rec);
        let hairline = 2.0 * d.abs() <= self.tolerance
            && self.accepts(hairline_bound(&arc.delta, d), seg, &arc.approx);
        let accepted = hairline || {
            let bound = if arc.clamped {
                hull_bound(seg, rec, &arc.delta, d)
            } else {
                *hull.get_or_insert_with(|| hull_bound(seg, rec, &arc.delta, d))
            };
            bound.is_some_and(|bound| self.accepts(bound, seg, &arc.approx))
        };
        accepted.then_some(arc.approx)
    }

    /// Whether to try the osculating parabola estimate on a piece: the cusp
    /// value has the same sign at both endpoints, and its magnitude varies by
    /// at most a factor of [`OSCULATING_MAX_CUSP_RATIO`] between them.
    ///
    /// When the cusp value is negative at both endpoints, the piece lies past
    /// a cusp of the offset, which runs backward there. The estimate applies
    /// equally in that case, with the signs reversed.
    #[inline(always)]
    fn tries_osculating(&self, rec: &OffsetRec) -> bool {
        let cusp0 = rec.cusp0.abs();
        let cusp1 = rec.cusp1.abs();
        let cusp_min = cusp0.min(cusp1);
        let cusp_max = cusp0.max(cusp1);
        // A piece produced by subdivision at a cusp has the cusp value
        // `CUSP_EPSILON` at that endpoint, to which the ratio does not apply.
        let cusp_ratio_ok =
            cusp_max <= OSCULATING_MAX_CUSP_RATIO * cusp_min || cusp_min == CUSP_EPSILON;
        rec.cusp0 * rec.cusp1 > 0.0 && cusp_ratio_ok
    }

    /// [`Self::approximate`] for both sides of the source at once, when they
    /// take the same path: `self` is one side, and `other` is the other side,
    /// at the negated offset distance.
    ///
    /// With the same fit on both sides and no handle clamped on either, `D`
    /// is the same on both. The osculating parabola estimate then differs
    /// only in a few signs, and a single pass over the samples decides both
    /// sides. This covers nearly every piece of an ordinary stroke not
    /// already accepted by [`Self::accept_arc`].
    ///
    /// The results are bit-for-bit identical to those of
    /// [`Self::approximate`] on each side. Returns `None` when the sides
    /// diverge, or when either side needs more than the estimate; the caller
    /// then handles each side separately.
    fn approximate_pair(
        &self,
        other: &CubicOffset,
        seg: &CubicBez,
        rec: &OffsetRec,
        rec_other: &OffsetRec,
    ) -> Option<[Approximation; 2]> {
        let d = self.d;
        let tolerance = self.tolerance;
        if (seg.p3 - seg.p0).hypot2() <= tolerance * tolerance {
            return None;
        }

        let has_inflection = (rec.cusp0 - 1.0) * (rec.cusp1 - 1.0) < 0.0;
        let has_inflection_other = (rec_other.cusp0 - 1.0) * (rec_other.cusp1 - 1.0) < 0.0;
        if has_inflection != has_inflection_other
            || !self.tries_osculating(rec)
            || !other.tries_osculating(rec_other)
        {
            return None;
        }
        let linear_fit = if has_inflection {
            two_point(seg, rec)
        } else {
            one_point(seg, rec)
        };
        let unclamped = match linear_fit {
            Some((a, b)) => {
                let clamps = self.clamps(seg, rec, a, b);
                if clamps != other.clamps(seg, rec_other, a, b) {
                    return None;
                }
                (!clamps).then_some((a, b))
            }
            None => None,
        };
        // Arc drawing replaces a linear fit that would require clamping a
        // handle.
        let (fit, fit_other) = match unclamped {
            Some((a, b)) => (
                self.apply(seg, rec, a, b),
                other.apply(seg, rec_other, a, b),
            ),
            None => (self.draw_arc(seg, rec), other.draw_arc(seg, rec_other)),
        };
        if fit.clamped || fit_other.clamped {
            return None;
        }

        let limit = tolerance / (OSCULATING_SAFETY * d.abs());
        // As in `approximate`.
        let max_slip = if has_inflection && linear_fit.is_some() {
            OSCULATING_MAX_SLIP
        } else {
            f64::INFINITY
        };
        let samples = OsculatingSamples::new(seg, &fit.delta, d, max_slip);
        let result = |decision, fit: &Fit, rec: &OffsetRec| match decision {
            Osculating::Decided(accepted) => Some(Approximation::decided(fit.approx, accepted)),
            Osculating::Cusp(t) => Some(Approximation::split_at(
                fit.approx,
                rec.t0 + t * (rec.t1 - rec.t0),
            )),
            Osculating::Untrusted => None,
        };
        Some([
            result(samples.decide(limit, rec.cusp0), &fit, rec)?,
            result(
                samples.decide_other(limit, rec_other.cusp0),
                &fit_other,
                rec_other,
            )?,
        ])
    }

    /// Whether a `bound` on the error of `approx` establishes that it is
    /// within tolerance, allowing for rounding of its control points.
    ///
    /// The bounds are computed for the exact `D`, but the control points of
    /// `approx` are rounded in absolute coordinates, which no bound on `D`
    /// accounts for. As in `bound/cheap.rs` of bezfit (`ROUND_ULPS`), we
    /// allow a few ulps of the largest coordinate of the source and of the
    /// approximation. This is only significant when the tolerance approaches
    /// that floor.
    #[inline(always)]
    fn accepts(&self, bound: f64, seg: &CubicBez, approx: &CubicBez) -> bool {
        let mut largest = 0.0_f64;
        for p in [
            seg.p0, seg.p1, seg.p2, seg.p3, approx.p0, approx.p1, approx.p2, approx.p3,
        ] {
            largest = largest.max(p.x.abs()).max(p.y.abs());
        }
        bound + ROUND_ULPS * f64::EPSILON * largest <= self.tolerance
    }

    /// Decide whether to accept a piece that [`Self::tries_osculating`]
    /// excludes, or for which the estimate was not trusted.
    ///
    /// This applies the osculating parabola estimate again, with the limit
    /// on the slip applied to every fit, and a looser limit on the ratio of
    /// the cusp values ([`FALLBACK_MAX_CUSP_RATIO`]). Unlike in
    /// [`Self::tries_osculating`], the ratio is only applied when the cusp
    /// value is positive at both endpoints; a piece past a cusp of the
    /// offset is not subject to it. The argument for the limit applies
    /// equally to both cases, but empirically it is only needed in the
    /// first: applying it past a cusp increases the number of segments on
    /// near-cusps and sharp turns without reducing the error.
    fn accept_fallback(&self, seg: &CubicBez, rec: &OffsetRec, fit: &Fit) -> bool {
        let d = self.d;
        // When the curvature times `d` is concentrated at one endpoint, so is
        // the error, and it can fall between the samples. A piece produced
        // by subdivision at a cusp of the offset has the cusp value
        // `CUSP_EPSILON` at that endpoint, to which the ratio does not apply.
        let cusp_min = rec.cusp0.min(rec.cusp1);
        let cusp_max = rec.cusp0.max(rec.cusp1);
        if cusp_min > 0.0
            && cusp_min != CUSP_EPSILON
            && cusp_max > FALLBACK_MAX_CUSP_RATIO * cusp_min
        {
            return false;
        }
        // The estimate requires the same sign of the cusp value at both
        // endpoints.
        if !(rec.cusp0 * rec.cusp1 > 0.0) {
            return false;
        }
        let limit = self.tolerance / (OSCULATING_SAFETY * d.abs());
        let sign = 1.0_f64.copysign(rec.cusp0);
        // A cusp at a sample, or excessive slip, counts as a rejection, and
        // the piece is then subdivided.
        matches!(
            osculating_decision(seg, &fit.delta, d, limit, OSCULATING_MAX_SLIP, sign),
            Osculating::Decided(true)
        )
    }

    /// Recursively subdivide.
    ///
    /// Subdivide at `t` and recurse on both pieces. `cusp_t_minus` and
    /// `cusp_t_plus` are the cusp values just before and just after `t`;
    /// they are equal here, since subdivision at a cusp of the offset goes
    /// through [`Self::split_at_cusp`]. Unit tangents are passed down
    /// explicitly, so that they can be computed robustly where the derivative
    /// nearly vanishes.
    fn subdivide(
        &self,
        rec: &OffsetRec,
        result: &mut BezPath,
        t: f64,
        utan_t: Vec2,
        cusp_t_minus: f64,
        cusp_t_plus: f64,
    ) {
        let (rec0, rec1) = rec.split(t, utan_t, cusp_t_minus, cusp_t_plus);
        self.offset_rec(&rec0, result);
        self.offset_rec(&rec1, result);
    }

    /// The piece of the source curve covered by `rec`.
    #[inline(always)]
    fn piece(&self, rec: &OffsetRec) -> CubicBez {
        if rec.depth == 0 {
            self.c
        } else {
            self.c.subsegment(rec.t0..rec.t1)
        }
    }

    /// Emit an accepted approximation, or subdivide a rejected piece, or, at
    /// the maximum depth, emit the fallback approximation regardless.
    #[inline]
    fn emit_or_subdivide(
        &self,
        rec: &OffsetRec,
        approximation: Approximation,
        result: &mut BezPath,
    ) {
        const UTAN_EPSILON: f64 = 1e-12;
        let Approximation {
            accepted,
            fallback,
            split,
        } = approximation;
        if let Some(approx) = accepted {
            result.curve_to(approx.p1, approx.p2, approx.p3);
        } else if let Some((t, q_t)) = split
            .filter(|_| rec.depth < MAX_DEPTH)
            .map(|t| (t, self.q.eval(t).to_vec2()))
            .filter(|(_, q_t)| q_t.length_squared() >= UTAN_EPSILON)
        {
            // At the interior sample whose cusp value has the opposite sign.
            let cusp = self.cusp_sign(t);
            self.subdivide(rec, result, t, q_t.normalize(), cusp, cusp);
        } else if rec.depth < MAX_DEPTH {
            let SubdivisionPoint { t, utan } = self.find_subdivision_point(rec);
            let cusp = self.cusp_sign(t);
            self.subdivide(rec, result, t, utan, cusp, cusp);
        } else {
            // The maximum depth has been reached. Emit the fallback
            // approximation, even though it was not accepted.
            #[cfg(test)]
            tests::OUT_OF_DEPTH.with(|count| count.set(count.get() + 1));
            result.curve_to(fallback.p1, fallback.p2, fallback.p3);
        }
    }

    /// Convert from (a, b) parameter space to an approximation.
    ///
    /// The control points of `D` are `n0`, `n0 + a utan0`, `n1 + b utan1` and
    /// `n1`, where `n0` and `n1` are the endpoint normals.
    ///
    /// A handle of the approximation that would point against the direction
    /// of travel of the offset is clamped to zero length.
    #[inline(always)]
    fn apply(&self, seg: &CubicBez, rec: &OffsetRec, a: f64, b: f64) -> Fit {
        let d = self.d;
        let n0 = rec.utan0.turn_90();
        let n1 = rec.utan1.turn_90();
        let p0 = seg.p0 + d * n0;
        let p3 = seg.p3 + d * n1;
        let (l0, l1, clamp0, clamp1) = self.handles(seg, rec, a, b);

        // Form `D` directly, rather than by subtracting control points of `A`
        // and `C`, whose coordinates may be much larger than the offset
        // distance.
        let mut p1 = p0 + l0 * rec.utan0;
        let mut delta1 = n0 + a * rec.utan0;
        if clamp0 {
            p1 = p0;
            delta1 = (p1 - seg.p1) / d;
        }
        let mut p2 = p3 - l1 * rec.utan1;
        let mut delta2 = n1 + b * rec.utan1;
        if clamp1 {
            p2 = p3;
            delta2 = (p2 - seg.p2) / d;
        }
        Fit {
            approx: CubicBez::new(p0, p1, p2, p3),
            delta: CubicBez::new(
                n0.to_point(),
                delta1.to_point(),
                delta2.to_point(),
                n1.to_point(),
            ),
            clamped: clamp0 || clamp1,
        }
    }

    /// The handle lengths of the approximation for the given parameters, and
    /// whether [`Self::apply`] clamps each of them to zero.
    #[inline(always)]
    fn handles(&self, seg: &CubicBez, rec: &OffsetRec, a: f64, b: f64) -> (f64, f64, bool, bool) {
        let d = self.d;
        // The handle lengths of the source piece and of the approximation.
        let source_l0 = (seg.p1 - seg.p0).dot(rec.utan0);
        let source_l1 = (seg.p3 - seg.p2).dot(rec.utan1);
        let l0 = source_l0 + a * d;
        let l1 = source_l1 - b * d;
        // At a smoothed stub the offset arrives traveling backward,
        // irrespective of the cusp value.
        let flip0 = rec.t0 == 0.0 && self.smoothed_stub[EndpointSide::Start as usize];
        let flip1 = rec.t1 == 1.0 && self.smoothed_stub[EndpointSide::End as usize];
        let clamp0 = l0 * rec.cusp0 * if flip0 { -1.0 } else { 1.0 } <= 0.0;
        let clamp1 = l1 * rec.cusp1 * if flip1 { -1.0 } else { 1.0 } <= 0.0;
        (l0, l1, clamp0, clamp1)
    }

    /// Whether [`Self::apply`] would clamp a handle for these parameters,
    /// which is considerably cheaper to determine than the fit itself.
    #[inline(always)]
    fn clamps(&self, seg: &CubicBez, rec: &OffsetRec, a: f64, b: f64) -> bool {
        let (_, _, clamp0, clamp1) = self.handles(seg, rec, a, b);
        clamp0 || clamp1
    }

    /// The arc drawing approximation of a piece.
    #[inline(always)]
    fn draw_arc(&self, seg: &CubicBez, rec: &OffsetRec) -> Fit {
        let (a, b) = draw_arc(rec.utan0, rec.utan1);
        self.apply(seg, rec, a, b)
    }

    /// Choose the subdivision point for a piece that was not accepted.
    ///
    /// This is the parameter midpoint, except for a piece extending toward a
    /// tip of the curve from some distance away; see
    /// [`Self::split_toward_tip`].
    ///
    /// The previous version subdivided where the tangent bisects the
    /// endpoint tangents, when the tangent at the midpoint was far from
    /// that bisector, and by the integral of absolute curvature when the
    /// piece contained an inflection. Near-cusps were the worst case for
    /// those uneven subdivisions, and once they are handled by the root
    /// subdivision, the midpoint performs as well, by evaluation on the
    /// strokes of the Ghostscript tiger and on a corpus of random curves
    /// with sharp turns.
    fn find_subdivision_point(&self, rec: &OffsetRec) -> SubdivisionPoint {
        if !self.tips[0].is_nan() {
            if let Some(point) = self.split_toward_tip(rec) {
                return point;
            }
        }
        let t = 0.5 * (rec.t0 + rec.t1);
        let q_t = self.q.eval(t).to_vec2();
        // Near a tip the speed is very small but the tangent is still well
        // defined, so only an exact zero is rejected.
        let utan = if q_t.hypot2() > 0.0 {
            q_t.normalize()
        } else {
            rec.utan0
        };
        SubdivisionPoint { t, utan }
    }

    /// The subdivision point for a piece extending toward a tip of the curve
    /// from some distance away, if the piece is of that kind.
    ///
    /// Beyond a few widths from a tip, the curvature decays as the inverse
    /// cube of the parameter distance from it. The relevant measure of a
    /// piece is therefore the ratio of the distances of its endpoints from
    /// the tip, and a piece spanning a large ratio is subdivided at the
    /// geometric mean of the two distances. Subdivision at the midpoint
    /// would halve the distance from the far endpoint at each level, and
    /// require many levels to resolve the region near the tip, where most
    /// of the turning occurs.
    #[cold]
    #[inline(never)]
    fn split_toward_tip(&self, rec: &OffsetRec) -> Option<SubdivisionPoint> {
        // The tip nearest to the piece, with the distances of the endpoints
        // of the piece from it and the direction away from it. A tip is never
        // in the interior of a piece, as the root subdivision includes it.
        let mut nearest: Option<(f64, f64, f64, f64)> = None;
        for tip in self.tips {
            let (near, far, away) = if tip <= rec.t0 {
                (rec.t0 - tip, rec.t1 - tip, 1.0)
            } else if tip >= rec.t1 {
                (tip - rec.t1, tip - rec.t0, -1.0)
            } else {
                // No tip (NaN), or a tip in the interior of the piece.
                continue;
            };
            if nearest.is_none_or(|(nearest, ..)| near < nearest) {
                nearest = Some((near, far, tip, away));
            }
        }
        let (near, far, tip, away) = nearest?;
        // A piece that begins at the tip, or that is not long compared with
        // its distance from the tip, is subdivided at its midpoint.
        if !(near > 0.0 && far > TIP_SPLIT_RATIO * near) {
            return None;
        }
        let t = tip + away * (near * far).sqrt();
        let q_t = self.q.eval(t).to_vec2();
        (q_t.hypot2() > 0.0).then(|| SubdivisionPoint {
            t,
            utan: q_t.normalize(),
        })
    }

    /// The middle control point of the hodograph of the piece, up to scale:
    /// the derivative at its start, plus half its parameter length times the
    /// second derivative there.
    ///
    /// The control points of the piece yield the same quantity by
    /// subtraction, but that is dominated by rounding error for a piece only
    /// a few ulps across, as occurs near the tip of a near-cusp.
    #[inline(always)]
    fn hodograph_middle(&self, rec: &OffsetRec) -> Vec2 {
        self.q.eval(rec.t0).to_vec2()
            + (0.5 * (rec.t1 - rec.t0)) * self.q.deriv().eval(rec.t0).to_vec2()
    }
}

// ---------------------------------------------------------------------------
// Fits
//
// Each of these returns the parameters (a, b) of an approximation, in the
// form taken by `CubicOffset::apply`. None of them depends on the offset
// distance.

/// Arc drawing.
///
/// `D` is the standard cubic approximation of the arc of the unit circle
/// from one endpoint normal to the other. It is always defined, and exact
/// for a straight piece. Its error is fourth order in the length of the
/// piece.
#[inline(always)]
fn draw_arc(utan0: Vec2, utan1: Vec2) -> (f64, f64) {
    // With theta the angle from utan1 to utan0, the handle length of the
    // standard arc approximation is
    // 4/3 tan(theta / 4) = 4/3 sin(theta / 2) / (1 + cos(theta / 2)).
    let cos_th = utan1.dot(utan0);
    let sin_th = utan1.cross(utan0);
    let cos_half = (0.5 * (1.0 + cos_th)).max(0.0).sqrt();
    let sin_half = (0.5 * (1.0 - cos_th)).max(0.0).sqrt().copysign(sin_th);
    let a = (4.0 / 3.0) * sin_half / (1.0 + cos_half);
    (a, -a)
}

/// One-point shape control.
///
/// This solves `D(1/2) = n(1/2)`, so that at `t = 1/2` the approximation
/// passes through the point of the offset at the same parameter, with no
/// tangential error there. Its error is fifth order in the length of the
/// piece. This is the shape control of Yang and Huang, "A New Shape Control
/// and Classification for Cubic Bézier Curves" (1993).
///
/// Returns `None` if the endpoint tangents are parallel, where the system
/// is singular, as for a symmetric S-shaped piece, or if the derivative of
/// the source vanishes at the midpoint.
#[inline(always)]
fn one_point(seg: &CubicBez, rec: &OffsetRec) -> Option<(f64, f64)> {
    let det = rec.utan0.cross(rec.utan1);
    if det.abs() < DET_EPSILON {
        return None;
    }
    let n0 = rec.utan0.turn_90();
    let n1 = rec.utan1.turn_90();
    let n_mid = unit_normal(seg, 0.5)?;
    // D(1/2) = (n0 + n1) / 2 + 3/8 (a utan0 + b utan1), which is required to
    // equal n_mid.
    let rhs = (8.0 / 3.0) * (n_mid - 0.5 * (n0 + n1));
    let a = rhs.cross(rec.utan1) / det;
    let b = rec.utan0.cross(rhs) / det;
    Some((a, b))
}

/// Two-point linear.
///
/// This solves `D(t) · n(t) = 1` at `t = 1/3` and `t = 2/3`: the component
/// of `D` along the source normal at the same parameter is one. Unlike
/// one-point, it remains well-conditioned when the endpoint tangents are
/// parallel, as they can be when the piece contains an inflection. It does
/// not control the tangential error, and its error is in general fourth
/// order in the length of the piece, but fifth order when the piece contains
/// an inflection, which is where it is used: the error due to slip is
/// proportional to the curvature, which vanishes there.
///
/// Returns `None` if the system is singular, which happens when the source
/// is a parabola.
#[inline(always)]
fn two_point(seg: &CubicBez, rec: &OffsetRec) -> Option<(f64, f64)> {
    let n0 = rec.utan0.turn_90();
    let n1 = rec.utan1.turn_90();
    let mut matrix = [[0.0; 2]; 2];
    let mut rhs = [0.0; 2];
    for (i, t) in [1.0 / 3.0, 2.0 / 3.0].into_iter().enumerate() {
        let n = unit_normal(seg, t)?;
        let mt = 1.0 - t;
        // The Bernstein basis functions of the four control points at t.
        let w0 = mt * mt * mt;
        let w1 = 3.0 * mt * mt * t;
        let w2 = 3.0 * mt * t * t;
        let w3 = t * t * t;
        matrix[i] = [w1 * rec.utan0.dot(n), w2 * rec.utan1.dot(n)];
        rhs[i] = 1.0 - ((w0 + w1) * n0 + (w2 + w3) * n1).dot(n);
    }
    let det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if det.abs() < DET_EPSILON {
        return None;
    }
    let a = (rhs[0] * matrix[1][1] - matrix[0][1] * rhs[1]) / det;
    let b = (matrix[0][0] * rhs[1] - rhs[0] * matrix[1][0]) / det;
    Some((a, b))
}

/// The unit normal of a cubic at `t`, or `None` if its derivative vanishes.
fn unit_normal(c: &CubicBez, t: f64) -> Option<Vec2> {
    const TAN_EPSILON: f64 = 1e-12;
    let tan = c.deriv().eval(t).to_vec2();
    let len = tan.hypot();
    if len < TAN_EPSILON {
        return None;
    }
    Some(tan.turn_90() / len)
}

/// Bound the distance between the approximation and the offset at equal
/// parameter, using only convex hulls.
///
/// The error at equal parameter is `|d| |D(t) - n(t)|`. `D(t)` lies in the
/// convex hull of its control points. The direction of `C'(t)` lies in the
/// cone spanned by the control points of the hodograph, so `n(t)` lies on
/// an arc of the unit circle between the corresponding normals. The
/// distance between a point of the hull and a point of the arc is
/// maximized at a vertex of the former and an endpoint of the latter.
///
/// The bound is first order in the turning of the piece, so it is only
/// useful for a piece that is nearly straight relative to the tolerance.
/// It depends on directions only, so it remains valid when the derivative
/// vanishes at an endpoint, given robust endpoint tangents.
///
/// Returns `None` if the cone spans a quarter turn or more.
fn hull_bound(seg: &CubicBez, rec: &OffsetRec, delta: &CubicBez, d: f64) -> Option<f64> {
    const TAN_EPSILON: f64 = 1e-12;
    // The directions of the control points of the hodograph. The outer ones
    // are the endpoint tangents; the middle one is used if it is nonzero.
    let mut directions: ArrayVec<Vec2, 3> = ArrayVec::new();
    directions.push(rec.utan0);
    let middle = seg.p2 - seg.p1;
    let middle_len = middle.hypot();
    if middle_len > TAN_EPSILON {
        directions.push(middle / middle_len);
    }
    directions.push(rec.utan1);
    for i in 0..directions.len() {
        for j in 0..i {
            if !(directions[i].dot(directions[j]) > 0.0) {
                return None;
            }
        }
    }

    let mut worst = 0.0_f64;
    for direction in &directions {
        let normal = direction.turn_90();
        for vertex in [delta.p0, delta.p1, delta.p2, delta.p3] {
            let vertex = vertex.to_vec2();
            // The argument requires each vertex to be within a quarter turn
            // of each normal.
            if !(vertex.dot(normal) > 0.0) {
                return None;
            }
            worst = worst.max((vertex - normal).hypot2());
        }
    }
    Some(d.abs() * worst.sqrt())
}

/// Bound the distance between any approximation and the offset at equal
/// parameter, by the triangle inequality alone.
///
/// `D(t)` lies in the convex hull of its control points, and `n(t)` has
/// unit length, so `|d D(t) - d n(t)|` is at most `|d|` times one plus the
/// largest norm of a control point. This holds for any piece and any fit,
/// across cusps and regardless of the turning, but it is never less than
/// `2 |d|`, so it is only useful when the stroke is thin relative to the
/// tolerance. In that case it accepts, even at the root, pieces that turn
/// too far for [`hull_bound`].
#[inline(always)]
fn hairline_bound(delta: &CubicBez, d: f64) -> f64 {
    let mut largest = 0.0_f64;
    for p in [delta.p0, delta.p1, delta.p2, delta.p3] {
        largest = largest.max(p.to_vec2().hypot2());
    }
    d.abs() * (largest.sqrt() + 1.0)
}

/// Bound the error of arc drawing on a small piece.
///
/// Suppose the tangent direction of the source turns monotonically over the
/// piece. Then the source normal traverses the arc of the unit circle from
/// one endpoint normal to the other exactly once. Arc drawing makes `D` the
/// standard cubic approximation of that arc, whose direction traverses the
/// same arc once, and whose length is within `epsilon` of one. Pair each
/// point of the approximation with the point of the offset whose normal
/// has the same direction; the two differ by at most the distance between
/// two points of the source piece, plus `|d| epsilon`.
///
/// The Fréchet distance is therefore at most the diameter of the source
/// piece plus `|d| epsilon`. The bound is independent of the
/// parametrization, which is essential: near a cusp of the source the
/// tangent turns very unevenly in the parameter, and every bound that
/// compares the curves at related parameter values fails.
///
/// Returns `None` if the tangent might not turn monotonically, if it turns
/// through more than about a right angle, or if the approximation is not
/// plain arc drawing.
#[inline(always)]
fn small_piece_bound(
    seg: &CubicBez,
    rec: &OffsetRec,
    middle: Vec2,
    arc: &Fit,
    d: f64,
) -> Option<f64> {
    if arc.clamped {
        return None;
    }
    // The tangent turns monotonically if the control points of the
    // hodograph are in angular order. The outer ones may vanish, so the
    // endpoint tangents are used for their directions. `middle` is the
    // middle one, up to scale; see `CubicOffset::hodograph_middle`.
    let turn_0 = rec.utan0.cross(middle);
    let turn_1 = middle.cross(rec.utan1);
    let turn = rec.utan0.cross(rec.utan1);
    let all_one_way = (turn_0 >= 0.0 && turn_1 >= 0.0 && turn >= 0.0)
        || (turn_0 <= 0.0 && turn_1 <= 0.0 && turn <= 0.0);
    // Restrict to turns for which the angular order is unambiguous.
    let cos_th = rec.utan0.dot(rec.utan1);
    if !all_one_way || !(cos_th > 0.0) || !(rec.utan0.dot(middle) >= 0.0) {
        return None;
    }

    // The diameter of the piece is at most the largest distance between
    // two of its control points.
    let points = [seg.p0, seg.p1, seg.p2, seg.p3];
    let mut diameter_squared = 0.0_f64;
    for i in 0..4 {
        for j in 0..i {
            diameter_squared = diameter_squared.max((points[i] - points[j]).hypot2());
        }
    }

    // The largest radial error of the standard approximation of an arc of
    // angle theta is 2/27 sin^6(theta / 4) / cos^2(theta / 4).
    let cos_half = (0.5 * (1.0 + cos_th)).sqrt();
    let cos_quarter_squared = 0.5 * (1.0 + cos_half);
    let sin_quarter_squared = 0.5 * (1.0 - cos_half);
    let epsilon = (2.0 / 27.0) * sin_quarter_squared.powi(3) / cos_quarter_squared;
    Some(diameter_squared.sqrt() + d.abs() * epsilon)
}

// ---------------------------------------------------------------------------
// The osculating parabola estimate

/// The sample parameters of the osculating parabola estimate, at the
/// midpoints of 8 equal intervals.
///
/// The decision of [`osculating_decision`] is made by the first sample, in
/// this order, whose cusp value has the wrong sign or whose error exceeds
/// the limit, and a cusp found at a sample becomes the subdivision point.
/// The order places the samples nearest the endpoints last, so that such a
/// point tends to lie away from the endpoints; ascending order was found
/// empirically to give slightly more segments.
///
/// With fewer samples, the number of missed errors increased markedly in
/// testing.
const OSCULATING_SAMPLES: [f64; 8] = [
    0.3125, 0.6875, 0.1875, 0.8125, 0.4375, 0.5625, 0.0625, 0.9375,
];

/// The safety factor applied to the osculating parabola estimate before
/// comparison with the tolerance.
///
/// The value was determined empirically, by comparison with the Fréchet
/// error of fitted pieces from a corpus of random curves.
const OSCULATING_SAFETY: f64 = 1.2;

/// The largest slip, the tangential component of `D`, for which the
/// osculating parabola estimate is trusted.
///
/// The two-point fit does not constrain the slip, and when it is large the
/// estimate can substantially underestimate the error. On the first
/// application of the estimate only the two-point fit is subject to this
/// limit; [`CubicOffset::accept_fallback`] applies it to every fit.
const OSCULATING_MAX_SLIP: f64 = 0.5;

/// The largest ratio between the cusp values at the endpoints of a piece
/// for which the osculating parabola estimate is trusted.
///
/// At the interior samples only the sign of the cusp value is checked.
/// Applying the ratio there as well sent most pieces ending where the
/// derivative vanishes, whose cusp value at that endpoint is not its limit,
/// to [`CubicOffset::accept_fallback`].
///
/// Where the curvature varies rapidly, as at the tip of a sharp turn, the
/// error is concentrated near one endpoint, between the samples. The value
/// was determined empirically, with some margin, by comparison with the
/// Fréchet error of fitted pieces from a corpus of random curves.
const OSCULATING_MAX_CUSP_RATIO: f64 = 3.0;

/// The largest ratio between the cusp values at the endpoints of a piece
/// for which the fallback estimate is trusted; see
/// [`CubicOffset::accept_fallback`].
///
/// Above this ratio the curvature is concentrated at one endpoint. The value
/// was inherited from the sampled estimate that the osculating estimate
/// replaced in this role, where it was determined empirically on pieces
/// adjacent to regularized near-cusps. The first application of the
/// osculating estimate uses the much tighter [`OSCULATING_MAX_CUSP_RATIO`].
const FALLBACK_MAX_CUSP_RATIO: f64 = 64.0;

/// Accept or reject a piece by the osculating parabola estimate.
///
/// At each sample, the estimate compares the approximation with the
/// osculating parabola of the offset, whose vertex is at the corresponding
/// point of the offset: the error consists of the radial error of `D`, and
/// of the deviation of the offset from its tangent over the slip. This is
/// the estimate of `tests::osculating_estimate` at [`OSCULATING_SAMPLES`],
/// rearranged to require one square root per sample and no division. With
/// `A = C'^⊥ · D`, `B = C' · D`, `K = C'' × C'`, `s = |C'|` and
/// `E = s^3 + K d` (which is `s^3` times the cusp value), the error in units
/// of `d` is
///
/// ```text
/// A / s - 1 + K d B^2 / (2 s^2 E)
/// ```
///
/// and it is within `limit` when `|A s E - s^2 E + K d B^2 / 2|` is at most
/// `limit |s^2 E|`. The slip is within `max_slip` when
/// `B^2 <= max_slip^2 s^2`, and the cusp value has the sign of `E`.
///
/// The decision is made by the first sample whose cusp value does not have
/// the sign of `cusp_sign`, or whose error exceeds the limit. The caller has
/// verified that the cusp value has that sign at both endpoints, so in the
/// former case the piece contains at least two cusps, which the signs at
/// the endpoints do not reveal. Past a cusp of the offset the sign is
/// negative, and nothing else changes: the offset runs backward, along a
/// curve that is equally smooth. Excessive slip at a sample only matters if
/// no sample rejects: the estimate is then not trusted to accept, whereas a
/// sample over the limit still rejects.
fn osculating_decision(
    seg: &CubicBez,
    delta: &CubicBez,
    d: f64,
    limit: f64,
    max_slip: f64,
    cusp_sign: f64,
) -> Osculating {
    OsculatingSamples::new(seg, delta, d, max_slip).decide(limit, cusp_sign)
}

/// The quantities of [`osculating_decision`] at each sample that are shared
/// with the offset at `-d`, since the other side of the source has the same
/// `D` when both use the same fit. In the notation there, these are `A s`,
/// `s^2`, `s^3`, `K d`, `K d B^2 / 2`, `B^2`, and the limit on `B^2`.
///
/// On the other side only `d` changes sign, so `E` becomes `s^3 - K d`, and
/// the numerator `A s E - s^2 E - K d B^2 / 2` in terms of these values.
struct OsculatingSamples {
    a_s: [f64; 8],
    s2: [f64; 8],
    s3: [f64; 8],
    k_d: [f64; 8],
    half_k_bb: [f64; 8],
    bb: [f64; 8],
    slip: [f64; 8],
    /// `E`, and the numerator and denominator of the error, for the side on
    /// which the samples were taken.
    e: [f64; 8],
    numerator: [f64; 8],
    denominator: [f64; 8],
}

impl OsculatingSamples {
    #[inline(always)]
    fn new(seg: &CubicBez, delta: &CubicBez, d: f64, max_slip: f64) -> Self {
        let c1 = [
            3.0 * (seg.p1 - seg.p0),
            3.0 * (seg.p2 - seg.p1),
            3.0 * (seg.p3 - seg.p2),
        ];
        let c2 = [2.0 * (c1[1] - c1[0]), 2.0 * (c1[2] - c1[1])];
        let delta = [
            delta.p0.to_vec2(),
            delta.p1.to_vec2(),
            delta.p2.to_vec2(),
            delta.p3.to_vec2(),
        ];
        let slip_scale = max_slip * max_slip;
        let mut a_s = [0.0; 8];
        let mut s2s = [0.0; 8];
        let mut s3 = [0.0; 8];
        let mut k_d = [0.0; 8];
        let mut half_k_bb = [0.0; 8];
        let mut bb = [0.0; 8];
        let mut slip = [0.0; 8];
        let mut e = [0.0; 8];
        let mut numerator = [0.0; 8];
        let mut denominator = [0.0; 8];
        // All samples are evaluated before any is examined, so that this
        // loop is free of branches and vectorizes. The quantities needed only
        // for the other side are eliminated by the compiler when that side is
        // not requested.
        for i in 0..8 {
            let t = OSCULATING_SAMPLES[i];
            let mt = 1.0 - t;
            let d_t = delta[0] * (mt * mt * mt)
                + delta[1] * (3.0 * mt * mt * t)
                + delta[2] * (3.0 * mt * t * t)
                + delta[3] * (t * t * t);
            let q = c1[0] * (mt * mt) + c1[1] * (2.0 * mt * t) + c1[2] * (t * t);
            let qq = c2[0] * mt + c2[1] * t;
            let s2 = q.hypot2();
            let b = q.dot(d_t);
            let s = s2.sqrt();
            let a = q.turn_90().dot(d_t);
            let k = qq.cross(q);
            a_s[i] = a * s;
            s2s[i] = s2;
            s3[i] = s2 * s;
            k_d[i] = k * d;
            half_k_bb[i] = 0.5 * k_d[i] * b * b;
            bb[i] = b * b;
            slip[i] = slip_scale * s2;
            e[i] = s3[i] + k_d[i];
            denominator[i] = s2 * e[i];
            numerator[i] = a_s[i] * e[i] - denominator[i] + half_k_bb[i];
        }
        OsculatingSamples {
            a_s,
            s2: s2s,
            s3,
            k_d,
            half_k_bb,
            bb,
            slip,
            e,
            numerator,
            denominator,
        }
    }

    /// The decision for the side on which the samples were taken.
    /// `cusp_sign` has the sign of the cusp value at the endpoints of the
    /// piece.
    #[inline(always)]
    fn decide(&self, limit: f64, cusp_sign: f64) -> Osculating {
        self.decision(
            &self.e,
            &self.numerator,
            &self.denominator,
            limit,
            cusp_sign,
        )
    }

    /// The decision for the other side of the source.
    #[inline(always)]
    fn decide_other(&self, limit: f64, cusp_sign: f64) -> Osculating {
        let mut e = [0.0; 8];
        let mut numerator = [0.0; 8];
        let mut denominator = [0.0; 8];
        for i in 0..8 {
            e[i] = self.s3[i] - self.k_d[i];
            denominator[i] = self.s2[i] * e[i];
            numerator[i] = self.a_s[i] * e[i] - denominator[i] - self.half_k_bb[i];
        }
        self.decision(&e, &numerator, &denominator, limit, cusp_sign)
    }

    /// The decision from the values at the samples, examined in order.
    #[inline(always)]
    fn decision(
        &self,
        e: &[f64; 8],
        numerator: &[f64; 8],
        denominator: &[f64; 8],
        limit: f64,
        cusp_sign: f64,
    ) -> Osculating {
        let mut trusted = true;
        for i in 0..8 {
            // The cusp value here is `e / s3`, and `s3` is nonnegative.
            if e[i] * cusp_sign <= 0.0 {
                return Osculating::Cusp(OSCULATING_SAMPLES[i]);
            }
            // Written so that a NaN results in rejection.
            if !(numerator[i].abs() <= limit * denominator[i].abs()) {
                return Osculating::Decided(false);
            }
            trusted &= self.bb[i] <= self.slip[i];
        }
        if trusted {
            Osculating::Decided(true)
        } else {
            Osculating::Untrusted
        }
    }
}

/// The outcome of [`osculating_decision`].
enum Osculating {
    /// `true` if every sample is within the limit and the slip is within
    /// its limit, and `false` if some sample exceeds the limit.
    Decided(bool),
    /// Every sample is within the limit, but the slip exceeds its limit at
    /// one or more samples, so the estimate is not trusted.
    Untrusted,
    /// The cusp value at this sample does not have the sign of the cusp
    /// value at the endpoints, so the piece contains at least two cusps.
    Cusp(f64),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PathEl, Point, Rect};

    /// Estimate the error by the osculating parabola, at `n` samples.
    ///
    /// This is the estimate of `est_arc_error_dot` on the `failed_k4` branch
    /// of fast-stroke. At each sample, with `D` decomposed into components
    /// `dot` and `cross` along the normal and tangent of the source, the error
    /// in units of `d` is `(dot - 1) + kappa_off d cross^2 / 2`: the radial
    /// error, plus the deviation over the slip `d cross` of the offset, whose
    /// curvature is `kappa_off = kappa / (1 + kappa d)`, from its tangent. It
    /// is not a bound: there is no remainder term, and nothing is measured
    /// between the samples, which are at the midpoints of `n` equal
    /// intervals.
    ///
    /// Returns the estimate, and the largest `|cross|`, the slip in units of
    /// `d`, beyond which the estimate is not trusted. Only the tests use this
    /// form; [`osculating_decision`] is the one used in the algorithm.
    fn osculating_estimate(seg: &CubicBez, delta: &CubicBez, d: f64, n: usize) -> (f64, f64) {
        let c1 = [
            3.0 * (seg.p1 - seg.p0),
            3.0 * (seg.p2 - seg.p1),
            3.0 * (seg.p3 - seg.p2),
        ];
        let c2 = [2.0 * (c1[1] - c1[0]), 2.0 * (c1[2] - c1[1])];
        let mut worst = 0.0_f64;
        let mut slip = 0.0_f64;
        for i in 0..n {
            let t = (i as f64 + 0.5) / n as f64;
            let mt = 1.0 - t;
            let w = delta.eval(t).to_vec2();
            let q = c1[0] * (mt * mt) + c1[1] * (2.0 * mt * t) + c1[2] * (t * t);
            let qq = c2[0] * mt + c2[1] * t;
            let speed2 = q.hypot2();
            let speed = speed2.sqrt();
            let normal = q.turn_90() / speed;
            let dot = normal.dot(w);
            let cross = normal.cross(w);
            // The curvature in the convention of this module,
            // `C'' × C' / |C'|^3`, so that `1 + kappa d` is the cusp value.
            let kappa = qq.cross(q) / (speed2 * speed);
            let kappa_off = kappa / (1.0 + kappa * d);
            let error = (dot - 1.0) + 0.5 * kappa_off * d * cross * cross;
            worst = worst.max(error.abs());
            slip = slip.max(cross.abs());
        }
        (worst * d.abs(), slip)
    }

    thread_local! {
        /// The number of pieces emitted at the maximum depth without being
        /// accepted.
        pub(super) static OUT_OF_DEPTH: core::cell::Cell<usize> = const { core::cell::Cell::new(0) };
        /// The number of evaluations in `find_cusp`.
        pub(super) static CUSP_ITERATIONS: core::cell::Cell<usize> = const { core::cell::Cell::new(0) };
    }

    /// A small deterministic random number generator.
    struct Rng(u64);

    impl Rng {
        /// A uniform value in [0, 1).
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1_u64 << 53) as f64
        }

        fn cubic(&mut self) -> CubicBez {
            let mut point = || Point::new(self.next() * 100.0, self.next() * 100.0);
            CubicBez::new(point(), point(), point(), point())
        }
    }

    /// The state for a single piece, as `offset_rec` would receive it.
    fn setup(c: CubicBez, d: f64) -> (CubicOffset, OffsetRec) {
        let mut co = CubicOffset::new(c, d, 0.0);
        let (tan0, tan1) = PathSeg::Cubic(c).tangents();
        let rec = OffsetRec {
            t0: 0.0,
            t1: 1.0,
            utan0: tan0.normalize(),
            utan1: tan1.normalize(),
            cusp0: co.end_cusp(EndpointSide::Start),
            cusp1: co.end_cusp(EndpointSide::End),
            depth: 0,
        };
        (co, rec)
    }

    /// The smallest and largest cusp value over a piece, by sampling.
    fn cusp_range(co: &CubicOffset) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for i in 0..=64 {
            let cusp = co.cusp_sign(i as f64 / 64.0);
            min = min.min(cusp);
            max = max.max(cusp);
        }
        (min, max)
    }

    /// The error of a single piece: the Fréchet distance from the oracle
    /// where it can be measured, and otherwise `fallback`, a one-sided
    /// distance.
    fn piece_frechet(
        source: &CubicBez,
        approx: &CubicBez,
        d: f64,
        fallback: impl FnOnce() -> f64,
    ) -> f64 {
        #[cfg(feature = "std")]
        if let Some(error) = crate::offset_oracle::piece_error(*source, d, *approx) {
            return error;
        }
        fallback()
    }

    /// The error of a complete result of `offset_cubic`, measured against the
    /// offset of the regularized curve.
    ///
    /// With `std`, this is the Fréchet oracle, using the lower bound for any
    /// piece it cannot measure; without, the sampled one-sided distance.
    fn whole_result_error(c: CubicBez, d: f64, tolerance: f64, result: &BezPath) -> f64 {
        #[cfg(feature = "std")]
        {
            let report = crate::offset_oracle::result_error(c, d, tolerance, result);
            report.error.max(report.unmeasured_lower)
        }
        #[cfg(not(feature = "std"))]
        {
            result_error(&c.regularize_cusp(tolerance * 0.25), d, result)
        }
    }

    /// Measure the distance between an approximation and the offset of `c`
    /// at equal parameter.
    fn equal_parameter_distance(c: &CubicBez, approx: &CubicBez, d: f64) -> f64 {
        let c1 = c.deriv();
        let mut worst = 0.0_f64;
        const N: usize = 400;
        for i in 0..=N {
            let t = i as f64 / N as f64;
            let n = c1.eval(t).to_vec2().normalize().turn_90();
            worst = worst.max(approx.eval(t).distance(c.eval(t) + d * n));
        }
        worst
    }

    /// An allowance for rounding in the bounds and in the measurement.
    fn slack(d: f64) -> f64 {
        1e-9 * d.abs() + 1e-11
    }

    /// Visit pieces of random cubics, at several offset distances.
    fn for_each_piece(mut f: impl FnMut(CubicBez, f64)) {
        let mut rng = Rng(0x2545_f491_4f6c_dd1d);
        for _ in 0..40 {
            let c = rng.cubic();
            for n in [2, 4, 8, 16] {
                for i in 0..n {
                    let piece = c.subsegment(i as f64 / n as f64..(i + 1) as f64 / n as f64);
                    for d in [0.3, -0.3, 3.0, -3.0, 30.0, -30.0] {
                        f(piece, d);
                    }
                }
            }
        }
    }

    #[test]
    fn hull_and_hairline_bounds_are_strict() {
        let mut n_hull = 0;
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
            // These bounds hold regardless of the cusp values, and for any
            // fit.
            let (arc_a, arc_b) = draw_arc(rec.utan0, rec.utan1);
            let mut fits = vec![co.apply(&piece, &rec, arc_a, arc_b)];
            if let Some((a, b)) = one_point(&piece, &rec) {
                fits.push(co.apply(&piece, &rec, a, b));
            }
            for fit in &fits {
                let distance = equal_parameter_distance(&piece, &fit.approx, d);
                if let Some(bound) = hull_bound(&piece, &rec, &fit.delta, d) {
                    assert!(
                        bound >= distance - slack(d),
                        "hull bound {bound:e} below distance {distance:e} for {piece:?}, d = {d}"
                    );
                    n_hull += 1;
                }
                let bound = hairline_bound(&fit.delta, d);
                assert!(
                    bound >= distance - slack(d),
                    "hairline bound {bound:e} below distance {distance:e} for {piece:?}, d = {d}"
                );
            }
        });
        assert!(n_hull > 1000, "{n_hull}");
    }

    #[test]
    fn small_piece_bound_is_strict() {
        // Pieces around the minimum of the speed of curves with a near-cusp.
        let mut rng = Rng(4242);
        let mut n_checked = 0;
        for _ in 0..200 {
            // A hairpin: the inner control points lie beyond the endpoints.
            let p0 = Point::new(rng.next() * 10.0, rng.next() * 10.0);
            let p3 = p0 + Vec2::new(rng.next() * 2.0 - 1.0, rng.next() * 2.0 - 1.0);
            let far = Vec2::new(40.0 + rng.next() * 40.0, rng.next() * 20.0);
            let spread = Vec2::new(rng.next() - 0.5, rng.next() - 0.5) * 0.2;
            let c = CubicBez::new(p0, p3 + far + spread, p0 + far - spread, p3);
            let q = c.deriv();
            let mut t_slow = 0.5;
            for i in 0..=1000 {
                let t = i as f64 / 1000.0;
                if q.eval(t).to_vec2().hypot2() < q.eval(t_slow).to_vec2().hypot2() {
                    t_slow = t;
                }
            }
            for width in [1e-2, 3e-3, 1e-3] {
                let t0 = (t_slow - width * rng.next()).max(0.0);
                let t1 = (t0 + width).min(1.0);
                let piece = c.subsegment(t0..t1);
                for d in [1.0, -1.0, 5.0, -5.0] {
                    let (co, rec) = setup(piece, d);
                    let (a, b) = draw_arc(rec.utan0, rec.utan1);
                    let fit = co.apply(&piece, &rec, a, b);
                    let middle = co.hodograph_middle(&rec);
                    let Some(bound) = small_piece_bound(&piece, &rec, middle, &fit, d) else {
                        continue;
                    };
                    let mut result = BezPath::new();
                    result.move_to(fit.approx.p0);
                    result.curve_to(fit.approx.p1, fit.approx.p2, fit.approx.p3);
                    let error =
                        piece_frechet(&piece, &fit.approx, d, || result_error(&piece, d, &result));
                    assert!(
                        bound >= error - slack(d),
                        "bound {bound:e} below error {error:e} for {piece:?}, d = {d}"
                    );
                    n_checked += 1;
                }
            }
        }
        assert!(n_checked > 200, "{n_checked}");
    }

    /// The number of segments, the number of pieces emitted at the maximum
    /// depth, and the worst error (a lower bound: the one-sided distance
    /// from the offset to the result), for families with sharp turns: random
    /// cubics, hairpins, and near-cusps whose minimum speed is set by a given
    /// gap (see `near_cusp`). These cases motivate the small-piece bound and
    /// the subdivision margin, and are rare in the random families and in the
    /// tiger. This takes a few minutes. Run with `--ignored --nocapture`.
    #[cfg(feature = "std")]
    #[test]
    #[ignore = "measurement, prints counts"]
    #[allow(clippy::print_stdout, reason = "this test exists to print counts")]
    fn near_cusp_depth_survey() {
        let mut rng = Rng(77);
        let mut families: Vec<(String, Vec<CubicBez>)> = Vec::new();
        families.push(("random".into(), (0..200).map(|_| rng.cubic()).collect()));
        let hairpins = (0..200)
            .map(|_| {
                let p0 = Point::new(rng.next() * 10.0, rng.next() * 10.0);
                let p3 = p0 + Vec2::new(rng.next() * 2.0 - 1.0, rng.next() * 2.0 - 1.0);
                let far = Vec2::new(40.0 + rng.next() * 40.0, rng.next() * 20.0);
                let spread = Vec2::new(rng.next() - 0.5, rng.next() - 0.5) * 0.2;
                CubicBez::new(p0, p3 + far + spread, p0 + far - spread, p3)
            })
            .collect();
        families.push(("hairpin".into(), hairpins));
        for gap in [1e-2, 1e-4, 1e-6, 0.0] {
            let cusps = (0..200).map(|_| near_cusp(&mut rng, gap)).collect();
            families.push((std::format!("cusp, gap {gap:e}"), cusps));
        }
        std::println!(
            "family, tolerance: segments, pieces at the maximum depth, and the worst distance \
             from the offset to the result over the tolerance"
        );
        for (name, curves) in &families {
            for tolerance in [0.25, 1e-2, 1e-4, 1e-6] {
                let (mut n_segments, mut n_out_of_depth) = (0, 0);
                let mut worst = 0.0_f64;
                for c in curves {
                    for d in [1.0, -1.0, 5.0, -5.0] {
                        OUT_OF_DEPTH.with(|count| count.set(0));
                        let mut result = BezPath::new();
                        offset_cubic(*c, d, tolerance, &mut result);
                        n_out_of_depth += OUT_OF_DEPTH.with(|count| count.get());
                        n_segments += result.segments().count();
                        // Measure against the offset of the regularized input,
                        // which is what is approximated: near a cusp,
                        // regularization alone moves the offset by up to
                        // `2 |d|`. The range matching of the oracle is
                        // unreliable here, so the one-sided measure is used.
                        let regularized = c.regularize_cusp(0.25 * tolerance);
                        let (error, _) = offset_to_result(regularized, d, &result, 4000);
                        worst = worst.max(error / tolerance);
                    }
                }
                std::println!(
                    "{name:>16}, {tolerance:e}: {n_segments:>7} {n_out_of_depth:>6} {worst:>9.2}"
                );
            }
        }
    }

    #[test]
    fn draw_arc_matches_offset() {
        // The vector form here should agree with the trigonometric form.
        let mut rng = Rng(12345);
        for _ in 0..100 {
            let utan0 = Vec2::from_angle(rng.next() * 6.0);
            let utan1 = Vec2::from_angle(rng.next() * 6.0);
            let th = utan1.cross(utan0).atan2(utan1.dot(utan0));
            let expected = (2. / 3.) / (1.0 + (0.5 * th).cos()) * 2.0 * (0.5 * th).sin();
            let (a, b) = draw_arc(utan0, utan1);
            assert!((a - expected).abs() < 1e-12, "{a} {expected}");
            assert_eq!(b, -a);
        }
    }

    /// The largest distance from a point of the result to the true offset
    /// of `c`.
    ///
    /// The offset is sampled densely, and the sample nearest to each point
    /// is refined by a ternary search between its neighbors.
    fn result_error(c: &CubicBez, d: f64, result: &BezPath) -> f64 {
        const N: usize = 2000;
        // Remain just inside the endpoints, where the derivative may vanish
        // and the normal is defined only as a limit.
        const END_EPSILON: f64 = 1e-9;
        let c1 = c.deriv();
        let offset_at = |s: f64| {
            let s = s.clamp(END_EPSILON, 1.0 - END_EPSILON);
            c.eval(s) + d * c1.eval(s).to_vec2().normalize().turn_90()
        };
        let samples: Vec<Point> = (0..=N).map(|j| offset_at(j as f64 / N as f64)).collect();
        let mut worst = 0.0_f64;
        for seg in result.segments() {
            for i in 0..=20 {
                let p = seg.eval(i as f64 / 20.0);
                // Refine around every local minimum of the sampled distance,
                // not only the smallest. Where the offset returns close to
                // itself, the nearest sample may lie on the wrong strand.
                let dist: Vec<f64> = samples.iter().map(|s| p.distance(*s)).collect();
                let mut nearest = f64::INFINITY;
                for j in 0..=N {
                    let left = j == 0 || dist[j] <= dist[j - 1];
                    let right = j == N || dist[j] <= dist[j + 1];
                    if !(left && right) {
                        continue;
                    }
                    let mut lo = j.saturating_sub(1) as f64 / N as f64;
                    let mut hi = (j + 1).min(N) as f64 / N as f64;
                    for _ in 0..60 {
                        let s1 = lo + (hi - lo) / 3.0;
                        let s2 = hi - (hi - lo) / 3.0;
                        if p.distance(offset_at(s1)) < p.distance(offset_at(s2)) {
                            hi = s2;
                        } else {
                            lo = s1;
                        }
                    }
                    nearest = nearest.min(p.distance(offset_at(0.5 * (lo + hi))));
                }
                worst = worst.max(nearest);
            }
        }
        worst
    }

    #[cfg(feature = "std")]
    #[test]
    fn default_result_with_hidden_cusp_pair() {
        // The cusp value is close to 1 at both endpoints of this piece of a
        // random cubic, but varies widely in its interior. With `d` of -10,
        // the offset has a pair of cusps that the signs at the endpoints do
        // not reveal. The osculating estimate, with guards that examined only
        // the endpoints, accepted a piece far outside the tolerance.
        let c = CubicBez::new(
            (79.057_384_592_444_27, 48.466_008_655_533_074),
            (12.503_273_981_653_885, 44.382_261_036_215_43),
            (52.908_321_475_974_894, 47.634_604_733_008_06),
            (28.539_084_055_419_305, 44.786_787_416_066_95),
        );
        let (co, _) = setup(c, -10.0);
        let (cusp_min, _) = cusp_range(&co);
        assert!(cusp_min < 0.0, "{cusp_min}");
        let tolerance = 0.25;
        let mut result = BezPath::new();
        offset_cubic(c, -10.0, tolerance, &mut result);
        let error = whole_result_error(c, -10.0, tolerance, &result);
        assert!(
            error <= 1.25 * tolerance,
            "error {error} over tolerance {tolerance}"
        );
    }

    #[test]
    #[ignore = "measurement, prints iteration counts"]
    #[allow(clippy::print_stdout, reason = "this test exists to print")]
    fn find_cusp_adversarial() {
        // Brackets of the scaled cusp value, from random pieces and from
        // near double roots: at an extremum of the curvature, `d` is chosen
        // so that the cusp value dips just below zero there. `find_cusp` is
        // compared with ITP on the same function and bracket.
        let mut rows: Vec<(&str, usize, usize, f64, f64)> = Vec::new();
        let mut run = |label: &'static str, c: CubicBez, d: f64, a: f64, b: f64| {
            let co = CubicOffset::new(c, d, 0.0);
            let f = |t: f64| co.scaled_cusp_and_deriv(t).0;
            let (ya, yb) = (f(a), f(b));
            if !(ya * yb < 0.0) {
                return;
            }
            CUSP_ITERATIONS.with(|count| count.set(0));
            let t = co.find_cusp(a, b, yb.signum());
            let newton = CUSP_ITERATIONS.with(|count| count.get());
            // ITP requires the function to be negative at `a`.
            let sign = -ya.signum();
            let mut itp = 0;
            let t_itp = crate::common::solve_itp(
                |t| {
                    itp += 1;
                    sign * f(t)
                },
                a,
                b,
                1e-12,
                1,
                0.2 / (b - a),
                sign * ya,
                sign * yb,
            );
            // The distance of each result from a sign change: the smallest
            // interval around it, of the form 2^-k, that brackets one.
            let miss = |t: f64| {
                let mut h = 1e-15_f64;
                while h < 1.0 {
                    let (l, r) = ((t - h).max(a), (t + h).min(b));
                    if f(l) * f(r) <= 0.0 {
                        return h;
                    }
                    h *= 2.0;
                }
                1.0
            };
            rows.push((label, newton, itp, miss(t), miss(t_itp)));
        };
        let mut rng = Rng(0x1234_5678_9abc_def1);
        for _ in 0..400 {
            let c = rng.cubic();
            for d in [0.5, -0.5, 3.0, -3.0, 30.0, -30.0] {
                for n in [1, 2, 4] {
                    for i in 0..n {
                        run(
                            "random",
                            c,
                            d,
                            i as f64 / n as f64,
                            (i + 1) as f64 / n as f64,
                        );
                    }
                }
            }
            // Local maxima of |kappa|, located on a grid and refined.
            let q = c.deriv();
            let qq = q.deriv();
            let kappa = |t: f64| {
                let v = q.eval(t).to_vec2();
                qq.eval(t).to_vec2().cross(v) / v.hypot().powi(3)
            };
            const N: usize = 200;
            for j in 1..N {
                let (t0, t1, t2) = (
                    (j - 1) as f64 / N as f64,
                    j as f64 / N as f64,
                    (j + 1) as f64 / N as f64,
                );
                let (k0, k1, k2) = (kappa(t0).abs(), kappa(t1).abs(), kappa(t2).abs());
                if !(k1 > k0 && k1 >= k2) {
                    continue;
                }
                // Refine the maximum of |kappa| by ternary search.
                let (mut lo, mut hi) = (t0, t2);
                for _ in 0..100 {
                    let m1 = lo + (hi - lo) / 3.0;
                    let m2 = hi - (hi - lo) / 3.0;
                    if kappa(m1).abs() < kappa(m2).abs() {
                        lo = m1;
                    } else {
                        hi = m2;
                    }
                }
                let t_star = 0.5 * (lo + hi);
                let k_star = kappa(t_star);
                for e in [1e-1, 1e-3, 1e-5, 1e-7, 1e-9, 1e-11, 1e-13, 1e-14] {
                    let d = -(1.0 + e) / k_star;
                    run("near double", c, d, 0.0, t_star);
                    run("near double", c, d, t_star, 1.0);
                    run("near double", c, d, (t_star - 0.1).max(0.0), t_star);
                    run("near double", c, d, t_star, (t_star + 0.1).min(1.0));
                }
            }
        }
        for label in ["random", "near double"] {
            let v: Vec<_> = rows.iter().filter(|r| r.0 == label).collect();
            let max_n = v.iter().map(|r| r.1).max().unwrap_or(0);
            let max_i = v.iter().map(|r| r.2).max().unwrap_or(0);
            let mean_n = v.iter().map(|r| r.1).sum::<usize>() as f64 / v.len() as f64;
            let mean_i = v.iter().map(|r| r.2).sum::<usize>() as f64 / v.len() as f64;
            let capped = v.iter().filter(|r| r.1 >= 100).count();
            let bad_n = v.iter().filter(|r| r.3 > 1e-9).count();
            let bad_i = v.iter().filter(|r| r.4 > 1e-9).count();
            let worst_n = v.iter().map(|r| r.3).fold(0.0, f64::max);
            let worst_i = v.iter().map(|r| r.4).fold(0.0, f64::max);
            std::println!(
                "{label}: {} brackets; evaluations Newton mean {:.1} max {max_n} (capped {capped}), ITP mean {:.1} max {max_i}; off a sign change by > 1e-9: Newton {bad_n} (worst {worst_n:.1e}), ITP {bad_i} (worst {worst_i:.1e})",
                v.len(),
                mean_n,
                mean_i,
            );
            let mut hist = std::collections::BTreeMap::new();
            for r in &v {
                *hist.entry(r.1.min(60) / 5 * 5).or_insert(0) += 1;
            }
            std::println!("  Newton evaluations, by 5: {hist:?}");
        }
    }

    /// The largest distance from the offset of `c` by `d`, sampled densely,
    /// to `result`, and the parameter at which it occurs.
    ///
    /// This is a lower bound on the Fréchet distance which, unlike the
    /// oracle, does not require matching pieces to intervals of the source,
    /// a matching that is ambiguous near a cusp. The distance to each
    /// segment of the result is that of its exact nearest point.
    fn offset_to_result(c: CubicBez, d: f64, result: &BezPath, n_off: usize) -> (f64, f64) {
        use crate::ParamCurveNearest;
        let q = c.deriv();
        let (limit0, limit1) = PathSeg::Cubic(c).tangents();
        let cubics: Vec<CubicBez> = result
            .segments()
            .filter_map(|seg| match seg {
                PathSeg::Cubic(cb) => Some(cb),
                _ => None,
            })
            .collect();
        // A segment lies in the bounding box of its control points.
        let boxes: Vec<Rect> = cubics
            .iter()
            .map(|cb| {
                Rect::from_points(cb.p0, cb.p1)
                    .union_pt(cb.p2)
                    .union_pt(cb.p3)
            })
            .collect();
        let mut worst = (0.0_f64, 0.0);
        let mut previous = 0;
        for i in 0..=n_off {
            let s = i as f64 / n_off as f64;
            let v = q.eval(s).to_vec2();
            let v = if v.hypot2() > 1e-300 {
                v
            } else if s < 0.5 {
                limit0
            } else {
                limit1
            };
            let p = c.eval(s) + d * v.normalize().turn_90();
            // Start from the segment nearest to the previous point, and
            // examine only the segments whose boxes are nearer than the best
            // distance so far.
            let mut best = f64::INFINITY;
            if let Some(cb) = cubics.get(previous) {
                best = cb.nearest(p, 1e-12).distance_sq.sqrt();
            }
            for (k, (cb, b)) in cubics.iter().zip(&boxes).enumerate() {
                let dx = (b.x0 - p.x).max(p.x - b.x1).max(0.0);
                let dy = (b.y0 - p.y).max(p.y - b.y1).max(0.0);
                if k == previous || dx * dx + dy * dy >= best * best {
                    continue;
                }
                let distance = cb.nearest(p, 1e-12).distance_sq.sqrt();
                if distance < best {
                    best = distance;
                    previous = k;
                }
            }
            if best > worst.0 {
                worst = (best, s);
            }
        }
        worst
    }

    #[cfg(feature = "std")]
    #[test]
    #[ignore = "measurement, prints errors"]
    #[allow(clippy::print_stdout, reason = "this test exists to print errors")]
    fn vanishing_end_derivative_survey() {
        // Random cubics with a control point coincident with an endpoint, at
        // stroke widths up to a substantial fraction of their size. On the
        // inner side the offset has a cusp near that endpoint. The errors are
        // measured by the oracle and by the one-sided distance, as multiples
        // of the tolerance.
        let mut rng = Rng(4243);
        let curves: Vec<CubicBez> = (0..100)
            .map(|i| {
                let c = rng.cubic();
                if i % 2 == 0 {
                    CubicBez::new(c.p0, c.p0, c.p2, c.p3)
                } else {
                    CubicBez::new(c.p0, c.p1, c.p3, c.p3)
                }
            })
            .collect();
        for d in [0.3, 3.0, 30.0] {
            for tolerance in [0.25, 0.01, 1e-3] {
                let mut segments = 0;
                let (mut over, mut worst) = ([0; 2], [0.0_f64; 2]);
                for c in &curves {
                    for d in [d, -d] {
                        let mut result = BezPath::new();
                        offset_cubic(*c, d, tolerance, &mut result);
                        segments += result.segments().count();
                        let regularized = c.regularize_cusp(tolerance * 0.25);
                        let errors = [
                            whole_result_error(*c, d, tolerance, &result),
                            offset_to_result(regularized, d, &result, 100_000).0,
                        ];
                        for i in 0..2 {
                            let ratio = errors[i] / tolerance;
                            over[i] += usize::from(ratio > 1.001);
                            worst[i] = worst[i].max(ratio);
                        }
                    }
                }
                std::println!(
                    "d={d:4} tol={tolerance:6}: segments {segments:5}; oracle over {:3} worst {:9.3}; one-sided over {:3} worst {:7.3}",
                    over[0],
                    worst[0],
                    over[1],
                    worst[1]
                );
            }
        }
    }

    /// A cubic with a near-cusp at a random parameter, from the hodograph
    /// `(t - t_cusp)(a + b t)`, perturbed by adding `3 gap` in a random
    /// direction to the derivative, so that the minimum speed is about
    /// `3 gap`. A cusp at `t = 1/2` would unduly favor midpoint subdivision.
    fn near_cusp(rng: &mut Rng, gap: f64) -> CubicBez {
        let t_cusp = 0.15 + 0.7 * rng.next();
        let mut random_vector =
            || Vec2::new(rng.next() * 300.0 - 150.0, rng.next() * 300.0 - 150.0);
        let a = random_vector();
        let b = random_vector();
        let dir = Vec2::new(rng.next() - 0.5, rng.next() - 0.5).normalize();
        // The hodograph in the power basis, then its control points, and
        // finally those of the curve.
        let (pow2, pow1, pow0) = (b, a - b * t_cusp, 3.0 * gap * dir - a * t_cusp);
        let q0 = pow0;
        let q1 = pow0 + 0.5 * pow1;
        let q2 = pow0 + pow1 + pow2;
        let p0 = Point::new(rng.next() * 100.0, rng.next() * 100.0);
        let p1 = p0 + q0 / 3.0;
        let p2 = p1 + q1 / 3.0;
        CubicBez::new(p0, p1, p2, p2 + q2 / 3.0)
    }

    #[cfg(feature = "std")]
    #[test]
    fn results_with_tips() {
        // A near-cusp of the source in the interior of a piece was invisible
        // to the error estimate, whose samples straddle it, and subdivision
        // repeatedly left it near an endpoint: before the root subdivision at
        // tips, the result missed the arc around the tip by a wide margin.
        // Measured by the one-sided distance against the regularized curve;
        // see `near_cusp_depth_survey`.
        let mut rng = Rng(2026);
        for gap in [1e-2, 1e-5, 0.0] {
            for _ in 0..6 {
                let c = near_cusp(&mut rng, gap);
                for d in [1.0, -1.0, 5.0] {
                    for tolerance in [0.25, 1e-2, 1e-4] {
                        let regularized = c.regularize_cusp(0.25 * tolerance);
                        OUT_OF_DEPTH.with(|count| count.set(0));
                        let mut result = BezPath::new();
                        offset_cubic(c, d, tolerance, &mut result);
                        assert_eq!(
                            OUT_OF_DEPTH.with(|count| count.get()),
                            0,
                            "out of depth: d = {d}, tolerance {tolerance}, {c:?}"
                        );
                        let (error, s) = offset_to_result(regularized, d, &result, 4000);
                        assert!(
                            error <= 1.1 * tolerance,
                            "error {error} at s = {s}: d = {d}, tolerance {tolerance}, {c:?}"
                        );
                    }
                }
            }
        }
    }

    /// The width `|C'| / |C''|` of the narrowest interior minimum of the
    /// speed, by sampling; infinite if there is none.
    #[cfg(feature = "std")]
    fn narrowest_turn(c: &CubicBez) -> f64 {
        const N: usize = 2000;
        let q = c.deriv();
        let speed: Vec<f64> = (0..=N)
            .map(|i| q.eval(i as f64 / N as f64).to_vec2().hypot())
            .collect();
        let mut narrowest = f64::INFINITY;
        for i in 1..N {
            if speed[i] <= speed[i - 1] && speed[i] <= speed[i + 1] {
                let c2 = q.deriv().eval(i as f64 / N as f64).to_vec2().hypot();
                narrowest = narrowest.min(speed[i] / c2);
            }
        }
        narrowest
    }

    /// Measures the results on random curves with a sharp turn too broad to
    /// be a tip, by the width of the turn: the number of segments, the
    /// number of calls over the tolerance, and the worst error, by the
    /// Fréchet oracle. The samples of the error estimate can straddle such a
    /// turn. This takes a few seconds. Run with `--ignored --nocapture`.
    #[cfg(feature = "std")]
    #[test]
    #[ignore = "measurement, prints counts"]
    #[allow(clippy::print_stdout, reason = "this test exists to print counts")]
    fn sharp_turn_survey() {
        const BANDS: [(f64, f64); 3] = [(0.01, 0.03), (0.03, 0.1), (0.1, 0.3)];
        const PER_BAND: usize = 400;
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let mut sets: [Vec<CubicBez>; 3] = Default::default();
        while sets.iter().any(|set| set.len() < PER_BAND) {
            let c = rng.cubic();
            let width = narrowest_turn(&c);
            for (set, (low, high)) in sets.iter_mut().zip(BANDS) {
                if width >= low && width < high && set.len() < PER_BAND {
                    set.push(c);
                }
            }
        }
        std::println!(
            "width, tolerance: segments, calls of {} over 1.05 and 1.5 times \
             the tolerance, worst",
            6 * PER_BAND
        );
        for (set, (low, high)) in sets.iter().zip(BANDS) {
            for tolerance in [0.25, 1e-2, 1e-4] {
                let (mut segments, mut over, mut far_over) = (0, 0, 0);
                let mut worst = 0.0_f64;
                for c in set {
                    for d in [0.5, -0.5, 2.0, -2.0, 10.0, -10.0] {
                        let mut result = BezPath::new();
                        offset_cubic(*c, d, tolerance, &mut result);
                        segments += result.segments().count();
                        let report = crate::offset_oracle::result_error(*c, d, tolerance, &result);
                        let ratio = report.error.max(report.unmeasured_lower) / tolerance;
                        over += usize::from(ratio > 1.05);
                        far_over += usize::from(ratio > 1.5);
                        worst = worst.max(ratio);
                    }
                }
                std::println!(
                    "{low} to {high}, {tolerance}: {segments:>6} {over:>3} {far_over:>3} {worst:>6.2}"
                );
            }
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn results_with_sharp_turns() {
        // Curves with a turn too broad to be a tip, at a tolerance of half the
        // offset distance, which earlier versions accepted whole, or nearly
        // so, well outside the tolerance. In order: an earlier sampled
        // estimate measured the distance to the source, and accepted an
        // approximation on the wrong side of it between two turns; the
        // osculating estimate found a sample over the limit after one with
        // excessive slip, and deferred to that sampled estimate, which fell
        // short; and the samples of the osculating estimate straddled the
        // turn.
        let cases = [
            (
                [
                    (70.48094071239159, 69.67513157564085),
                    (60.395361225140846, 21.14518938682526),
                    (82.70372826510773, 73.15404192150442),
                    (80.36747186021297, 63.619310802492066),
                ],
                -0.5,
            ),
            (
                [
                    (14.940246361937215, 24.323404923761505),
                    (16.969975647298806, 57.145266278455665),
                    (53.36354627583025, 30.428145078542368),
                    (51.880625747832134, 33.42931795389501),
                ],
                0.5,
            ),
            (
                [
                    (0.9055224166763853, 85.56012487359405),
                    (64.81963576741242, 27.466766809783483),
                    (62.88663400875335, 2.0243310497624845),
                    (18.286893883228384, 65.35842253157391),
                ],
                -0.5,
            ),
        ];
        for (points, d) in cases {
            let c = CubicBez::new(points[0], points[1], points[2], points[3]);
            let tolerance = 0.25;
            let mut result = BezPath::new();
            offset_cubic(c, d, tolerance, &mut result);
            let (error, s) = offset_to_result(c, d, &result, 4000);
            assert!(
                error <= 1.1 * tolerance,
                "error {error} at s = {s}: d = {d}, {c:?}"
            );
        }
    }

    #[test]
    fn tips_do_not_exhaust_the_depth() {
        // Pieces adjacent to the tip of a regularized cusp are only a few ulps
        // across, with very small speeds. Tests on their control points, or
        // absolute thresholds on the speed, send thousands of them to the
        // maximum depth; see `near_cusp_depth_survey`.
        let mut rng = Rng(7);
        for gap in [1e-6, 0.0] {
            let mut n_out_of_depth = 0;
            let mut n_segments = 0;
            for _ in 0..40 {
                let c = near_cusp(&mut rng, gap);
                for d in [1.0, -1.0, 5.0, -5.0] {
                    OUT_OF_DEPTH.with(|count| count.set(0));
                    let mut result = BezPath::new();
                    offset_cubic(c, d, 1e-4, &mut result);
                    n_out_of_depth += OUT_OF_DEPTH.with(|count| count.get());
                    n_segments += result.segments().count();
                }
            }
            // A bound on the total number of segments, and none, or nearly
            // none, at the maximum depth.
            assert!(
                n_out_of_depth <= 5,
                "gap {gap}: {n_out_of_depth} out of depth"
            );
            assert!(n_segments < 3500, "gap {gap}: {n_segments} segments");
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn default_result_with_vanishing_end_derivative() {
        // Where the derivative vanishes at an endpoint, the cusp value there
        // was once taken as 1, and the cusp near that endpoint, on the inside,
        // was never found. The fit then spanned it, and the osculating
        // estimate, whose samples do not reach the endpoint, accepted fits
        // several times over the tolerance. Measured by the one-sided
        // distance, since the oracle cannot follow a piece that reverses near
        // a cusp.
        let cases = [
            (
                CubicBez::new(
                    (71.988_963_612_376_5, 58.039_513_772_261_74),
                    (28.277_259_840_239_01, 75.409_677_717_394_56),
                    (76.641_865_945_822_9, 47.690_545_505_945_88),
                    (76.641_865_945_822_9, 47.690_545_505_945_88),
                ),
                0.3,
                0.25,
            ),
            (
                CubicBez::new(
                    (20.284_875_574_871_297, 73.667_977_655_445_3),
                    (20.284_875_574_871_297, 73.667_977_655_445_3),
                    (60.096_863_426_480_81, 22.650_499_256_665_98),
                    (45.303_963_035_391_234, 80.413_901_500_895_16),
                ),
                30.0,
                0.01,
            ),
        ];
        for (c, d, tolerance) in cases {
            let mut result = BezPath::new();
            offset_cubic(c, d, tolerance, &mut result);
            let (error, s) = offset_to_result(c, d, &result, 20_000);
            assert!(
                error <= 1.25 * tolerance,
                "error {error} at s = {s}, tolerance {tolerance}, for {c:?}, d = {d}"
            );
        }
    }

    #[test]
    fn offsets_from_open_issues() {
        // Reproductions of open kurbo issues against the previous version of
        // this module. The first (#617) has a vanishing endpoint derivative,
        // for which the previous version gave non-finite points on the inner
        // side; #344 is an older report of the same kind. The last (#615) has
        // an exact cusp at its midpoint, which `detect_cusp` missed, so it was
        // not regularized; midpoint subdivision then took the rounding error
        // there for the tangent, and the piece after the cusp started on the
        // wrong side, well outside the tolerance.
        let cases = [
            (
                CubicBez::new((0.5, -3.0), (-0.25, -0.5), (0.0, 0.0), (0.0, 0.0)),
                [-0.6, 0.6],
                [5e-5, 0.02],
            ),
            (
                CubicBez::new(
                    (51.0, 0.0),
                    (-0.085_937_5, 161.640_625),
                    (0.0, 164.0),
                    (0.0, 164.0),
                ),
                [-8.0, 8.0],
                [0.01, 1e-4],
            ),
            (
                CubicBez::new(
                    (1.600_000_000_000_000_8, 310.787_665_198_237_9),
                    (5.406_167_400_881_057, 306.981_497_797_356_84),
                    (5.406_167_400_881_058, 310.787_665_198_237_9),
                    (1.6, 306.981_497_797_356_84),
                ),
                [-0.6, 0.6],
                [0.02, 1e-4],
            ),
        ];
        for (c, ds, tolerances) in cases {
            for d in ds {
                for tolerance in tolerances {
                    let mut result = BezPath::new();
                    offset_cubic(c, d, tolerance, &mut result);
                    assert!(
                        result.elements().iter().all(|el| el
                            .end_point()
                            .is_none_or(|p| p.is_finite())
                            && match *el {
                                PathEl::CurveTo(p1, p2, _) => p1.is_finite() && p2.is_finite(),
                                _ => true,
                            }),
                        "non-finite point for {c:?}, d = {d}"
                    );
                    let (error, s) = offset_to_result(c, d, &result, 20_000);
                    assert!(
                        error <= tolerance,
                        "error {error} at s = {s}, tolerance {tolerance}, for {c:?}, d = {d}"
                    );
                }
            }
        }
    }

    /// The osculating estimate at the given samples.
    fn osculating_at(seg: &CubicBez, delta: &CubicBez, d: f64, samples: &[f64]) -> f64 {
        let c1 = seg.deriv();
        let c2 = c1.deriv();
        let mut worst = 0.0_f64;
        for &t in samples {
            let w = delta.eval(t).to_vec2();
            let q = c1.eval(t).to_vec2();
            let qq = c2.eval(t).to_vec2();
            let s = q.hypot();
            let n = q.turn_90() / s;
            let dot = n.dot(w);
            let cross = (q / s).dot(w);
            let kappa = qq.cross(q) / (s * s * s);
            let kappa_off = kappa / (1.0 + kappa * d);
            worst = worst.max(((dot - 1.0) + 0.5 * kappa_off * d * cross * cross).abs());
        }
        worst * d.abs()
    }

    #[cfg(feature = "std")]
    #[test]
    #[ignore = "measurement, prints ratios"]
    #[allow(clippy::print_stdout, reason = "this test exists to print ratios")]
    fn osculating_estimate_at_cusp_ends() {
        // Pieces that end exactly at a cusp of the offset, fitted as in
        // `approximate`, compared with the Fréchet error: whether the
        // osculating estimate remains reliable there, and whether samples near
        // the endpoints would help.
        use crate::offset_oracle::piece_error;
        let base: Vec<f64> = OSCULATING_SAMPLES.to_vec();
        let mut dense = base.clone();
        dense.extend([0.0078125, 0.015625, 0.03125, 0.96875, 0.984375, 0.9921875]);
        let mut rows: Vec<(f64, f64, f64)> = Vec::new();
        let mut rng = Rng(77);
        for _ in 0..400 {
            let c = rng.cubic();
            for d in [0.3, -0.3, 3.0, -3.0, 30.0, -30.0] {
                let co = CubicOffset::new(c, d, 0.0);
                // Sign changes of the cusp value on a grid.
                let n = 400;
                for j in 0..n {
                    let (a, b) = (j as f64 / n as f64, (j + 1) as f64 / n as f64);
                    let (ya, yb) = (co.scaled_cusp_and_deriv(a).0, co.scaled_cusp_and_deriv(b).0);
                    if !(ya * yb < 0.0) || a == 0.0 || b == 1.0 {
                        continue;
                    }
                    let t_star = co.find_cusp(a, b, yb.signum());
                    for h in [0.4, 0.2, 0.1, 0.05, 0.025] {
                        for side in [false, true] {
                            let (t0, t1) = if side {
                                (t_star, (t_star + h).min(1.0))
                            } else {
                                ((t_star - h).max(0.0), t_star)
                            };
                            if t1 - t0 < 1e-3 {
                                continue;
                            }
                            let piece = c.subsegment(t0..t1);
                            let q = c.deriv();
                            let (u0, u1) = (
                                q.eval(t0).to_vec2().normalize(),
                                q.eval(t1).to_vec2().normalize(),
                            );
                            let cusp0 = if side {
                                CUSP_EPSILON.copysign(co.cusp_sign(t1))
                            } else {
                                co.cusp_sign(t0)
                            };
                            let cusp1 = if side {
                                co.cusp_sign(t1)
                            } else {
                                CUSP_EPSILON.copysign(co.cusp_sign(t0))
                            };
                            if !(cusp0 > 0.0 && cusp1 > 0.0) {
                                continue;
                            }
                            let rec = OffsetRec {
                                t0: 0.0,
                                t1: 1.0,
                                utan0: u0,
                                utan1: u1,
                                cusp0,
                                cusp1,
                                depth: 1,
                            };
                            let pco = CubicOffset::new(piece, d, 0.0);
                            let has_inflection = (cusp0 - 1.0) * (cusp1 - 1.0) < 0.0;
                            let lin = if has_inflection {
                                two_point(&piece, &rec)
                            } else {
                                one_point(&piece, &rec)
                            };
                            let fit = match lin {
                                Some((a, b)) => {
                                    let f = pco.apply(&piece, &rec, a, b);
                                    if f.clamped {
                                        pco.draw_arc(&piece, &rec)
                                    } else {
                                        f
                                    }
                                }
                                None => pco.draw_arc(&piece, &rec),
                            };
                            let Some(err) = piece_error(piece, d, fit.approx) else {
                                continue;
                            };
                            if !(err > 1e-4 * d.abs() && err < 0.3 * d.abs()) {
                                continue;
                            }
                            rows.push((
                                err,
                                osculating_at(&piece, &fit.delta, d, &base),
                                osculating_at(&piece, &fit.delta, d, &dense),
                            ));
                        }
                    }
                }
            }
        }
        for (name, i) in [("8 samples", 1), ("8 + 6 near the ends", 2)] {
            let mut r: Vec<f64> = rows
                .iter()
                .map(|x| x.0 / if i == 1 { x.1 } else { x.2 })
                .collect();
            r.sort_by(f64::total_cmp);
            let q = |p: f64| r[((r.len() - 1) as f64 * p) as usize];
            std::println!(
                "{name}: {} pieces; error / estimate median {:.3} p90 {:.3} p99 {:.3} max {:.3}",
                r.len(),
                q(0.5),
                q(0.9),
                q(0.99),
                q(1.0)
            );
        }
    }

    #[test]
    fn pair_matches_one_side_at_a_time() {
        // The two-sided recursion shares work but makes the same decisions,
        // so its output should be bit-for-bit identical.
        let mut rng = Rng(98765);
        let mut n_segments = 0;
        for _ in 0..80 {
            let c = rng.cubic();
            for d in [0.3, 3.0, 30.0] {
                for tolerance in [0.25, 1e-2, 1e-4] {
                    let mut expect_pos = BezPath::new();
                    let mut expect_neg = BezPath::new();
                    offset_cubic(c, d, tolerance, &mut expect_pos);
                    offset_cubic(c, -d, tolerance, &mut expect_neg);
                    let mut pos = BezPath::new();
                    let mut neg = BezPath::new();
                    pos.move_to(expect_pos.elements()[0].end_point().unwrap());
                    neg.move_to(expect_neg.elements()[0].end_point().unwrap());
                    offset_cubic_pair(c, d, tolerance, &mut pos, &mut neg);
                    assert_eq!(pos.elements(), expect_pos.elements(), "{c:?}, d = {d}");
                    assert_eq!(neg.elements(), expect_neg.elements(), "{c:?}, d = -{d}");
                    n_segments += pos.elements().len() + neg.elements().len();
                }
            }
        }
        assert!(n_segments > 5000, "{n_segments}");
    }

    /// A comparison of the osculating parabola estimate with the Fréchet
    /// error, on fitted pieces. Run with `--ignored --nocapture`.
    #[cfg(feature = "std")]
    #[test]
    #[ignore = "measurement, prints ratios"]
    #[allow(clippy::print_stdout, reason = "this test exists to print ratios")]
    fn osculating_estimate_ratios() {
        use crate::offset_oracle::piece_error;
        for n in [4, 8] {
            for (label, curvy) in [("ordinary", false), ("near cusp", true)] {
                let mut ratios: Vec<f64> = Vec::new();
                for_each_piece(|piece, d| {
                    let (co, rec) = setup(piece, d);
                    let (cusp_min, _) = cusp_range(&co);
                    if (cusp_min < 0.2) != curvy || cusp_min < 0.0 {
                        return;
                    }
                    let has_inflection = (rec.cusp0 - 1.0) * (rec.cusp1 - 1.0) < 0.0;
                    let fit = if has_inflection {
                        two_point(&piece, &rec)
                    } else {
                        one_point(&piece, &rec)
                    };
                    let Some((a, b)) = fit else { return };
                    let fit = co.apply(&piece, &rec, a, b);
                    if fit.clamped {
                        return;
                    }
                    let Some(error) = piece_error(piece, d, fit.approx) else {
                        return;
                    };
                    // Only pieces near the acceptance threshold matter: an
                    // error between 1e-3 and 1 times |d| / 10.
                    if !(error > 1e-4 * d.abs() && error < 0.1 * d.abs()) {
                        return;
                    }
                    let (estimate, _) = osculating_estimate(&piece, &fit.delta, d, n);
                    ratios.push(error / estimate);
                });
                ratios.sort_by(f64::total_cmp);
                let q = |v: &Vec<f64>, p: f64| v[((v.len() - 1) as f64 * p) as usize];
                std::println!(
                    "n={n} {label}: {} pieces; error / osculating: median {:.3}, p90 {:.3}, p99 {:.3}, max {:.3}",
                    ratios.len(),
                    q(&ratios, 0.5),
                    q(&ratios, 0.9),
                    q(&ratios, 0.99),
                    q(&ratios, 1.0),
                );
            }
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn rounding_allowance_covers_large_coordinates() {
        // Far from the origin, the control points of an approximation are
        // rounded by about an ulp of their coordinates, which the bounds,
        // computed for the exact `D`, do not account for. The arc is drawn
        // and bounded there, then translated back to the origin, where the
        // subtraction is exact, and measured.
        use crate::offset_oracle::piece_error;
        let mut rng = Rng(31337);
        let mut n_checked = 0;
        for _ in 0..20 {
            let c = rng.cubic();
            for i in 0..32 {
                let piece = c.subsegment(i as f64 / 32.0..(i + 1) as f64 / 32.0);
                for shift in [1e6, 1e9] {
                    let shift = Vec2::new(shift, shift);
                    let far = CubicBez::new(
                        piece.p0 + shift,
                        piece.p1 + shift,
                        piece.p2 + shift,
                        piece.p3 + shift,
                    );
                    for d in [1.0, -1.0] {
                        let (co, rec) = setup(far, d);
                        let fit = co.draw_arc(&far, &rec);
                        let Some(bound) = hull_bound(&far, &rec, &fit.delta, d) else {
                            continue;
                        };
                        let back = |c: CubicBez| {
                            CubicBez::new(c.p0 - shift, c.p1 - shift, c.p2 - shift, c.p3 - shift)
                        };
                        let Some(error) = piece_error(back(far), d, back(fit.approx)) else {
                            continue;
                        };
                        let mut largest = 0.0_f64;
                        for p in [far.p0, far.p1, far.p2, far.p3] {
                            largest = largest.max(p.x.abs()).max(p.y.abs());
                        }
                        for p in [fit.approx.p0, fit.approx.p1, fit.approx.p2, fit.approx.p3] {
                            largest = largest.max(p.x.abs()).max(p.y.abs());
                        }
                        let allowance = ROUND_ULPS * f64::EPSILON * largest;
                        assert!(
                            bound + allowance >= error,
                            "bound {bound:e} plus allowance {allowance:e} below error {error:e}"
                        );
                        n_checked += 1;
                    }
                }
            }
        }
        assert!(n_checked > 500, "{n_checked}");
    }

    #[test]
    fn straight_line() {
        let cubic = CubicBez::new((0., 0.), (10., 0.), (20., 0.), (30., 0.));
        let mut result = BezPath::new();
        offset_cubic(cubic, 5., 1e-6, &mut result);
        // The offset of a straight line is exact, and should not be
        // subdivided.
        assert_eq!(result.elements().len(), 2);
        let PathEl::CurveTo(p1, p2, p3) = result.elements()[1] else {
            panic!("expected a curve");
        };
        for p in [p1, p2, p3] {
            assert!((p.y - 5.0).abs() < 1e-12, "{p:?}");
        }
    }

    #[test]
    fn degenerate_input_is_survived() {
        // Repeated and collinear control points, very small and very large
        // curves, and offset distances far larger than the curve. Accuracy is
        // not checked here, only that the output is finite and of reasonable
        // length.
        let mut rng = Rng(99);
        let mut result = BezPath::new();
        for i in 0..2000 {
            let mut c = rng.cubic();
            match i % 8 {
                0 => c.p1 = c.p0,
                1 => c.p2 = c.p3,
                2 => {
                    c.p1 = c.p0;
                    c.p2 = c.p3;
                }
                3 => {
                    c.p1 = c.p0.lerp(c.p3, rng.next() * 2.0 - 0.5);
                    c.p2 = c.p0.lerp(c.p3, rng.next() * 2.0 - 0.5);
                }
                4 => c.p2 = c.p1,
                5 => c.p3 = c.p0,
                6 => {
                    let scale = 1e-6;
                    c = CubicBez::new(
                        (c.p0.x * scale, c.p0.y * scale),
                        (c.p1.x * scale, c.p1.y * scale),
                        (c.p2.x * scale, c.p2.y * scale),
                        (c.p3.x * scale, c.p3.y * scale),
                    );
                }
                _ => {}
            }
            for d in [1e-3, -0.5, 7.0, -300.0, 1e5] {
                for tolerance in [1.0, 1e-2, 1e-6] {
                    offset_cubic(c, d, tolerance, &mut result);
                    // Subdivision at a cusp does not count toward the depth
                    // limit, so allow for a few such subdivisions.
                    assert!(result.elements().len() <= 4 << MAX_DEPTH);
                    for el in result.elements() {
                        if let PathEl::CurveTo(p1, p2, p3) = el {
                            assert!(
                                p1.is_finite() && p2.is_finite() && p3.is_finite(),
                                "not finite for {c:?}, d = {d}, tolerance = {tolerance}"
                            );
                        }
                    }
                }
            }
        }
    }

    // The remaining tests are carried over from the previous version of this
    // module, and exercise inputs that have caused problems in the past.

    #[test]
    fn pathological_curves() {
        let curve = CubicBez {
            p0: (-1236.3746269978635, 152.17981429574826).into(),
            p1: (-1175.18662093517, 108.04721798590596).into(),
            p2: (-1152.142883879584, 105.76260301083356).into(),
            p3: (-1151.842639804639, 105.73040758939104).into(),
        };
        let offset = 3603.7267536453924;
        let accuracy = 0.1;
        let mut result = BezPath::new();
        offset_cubic(curve, offset, accuracy, &mut result);
        assert!(matches!(result.iter().next(), Some(PathEl::MoveTo(_))));
    }

    #[test]
    fn infinite_recursion() {
        const TOLERANCE: f64 = 0.1;
        const OFFSET: f64 = -0.5;
        let c = CubicBez::new(
            (1096.2962962962963, 593.90243902439033),
            (1043.6213991769548, 593.90243902439033),
            (1030.4526748971193, 593.90243902439033),
            (1056.7901234567901, 593.90243902439033),
        );
        let mut result = BezPath::new();
        offset_cubic(c, OFFSET, TOLERANCE, &mut result);
    }

    /// The offsets of the curves of the strokes of the Ghostscript tiger, on
    /// both sides, are within tolerance by the Fréchet oracle at 1/32. The
    /// results at the tighter tolerances are printed but not checked, as one
    /// call slightly exceeds the tolerance at 1/256.
    ///
    /// This needs the 200 by 200 tiger from Vello's assets, which is usually
    /// rendered at 8x, so that a quarter pixel is 1/32. Run with
    /// `KURBO_TIGER=path/to/tiger.svg cargo test --release -- --ignored
    /// --nocapture tiger`.
    #[cfg(feature = "std")]
    #[test]
    #[ignore = "needs the tiger, from KURBO_TIGER"]
    #[allow(clippy::print_stdout, reason = "this test prints segment counts")]
    fn tiger_is_within_tolerance() {
        use crate::offset_oracle::result_error;
        let attribute = |element: &str, name: &str| -> Option<String> {
            let pattern = std::format!(" {name}=\"");
            let start = element.find(&pattern)? + pattern.len();
            let end = start + element[start..].find('"')?;
            Some(element[start..end].into())
        };
        let file = std::env::var("KURBO_TIGER").expect("KURBO_TIGER should name the tiger");
        let svg = std::fs::read_to_string(file).expect("reading the tiger");
        let mut cubics = Vec::new();
        for element in svg.split("<path").skip(1) {
            let element = &element[..element.find('>').unwrap_or(element.len())];
            let stroked = attribute(element, "stroke").is_some_and(|stroke| stroke != "none");
            let Some(d) = attribute(element, "d").filter(|_| stroked) else {
                continue;
            };
            let width: f64 = attribute(element, "stroke-width")
                .map(|width| width.parse().expect("stroke width"))
                .unwrap_or(1.0);
            // Lines are not offset as curves, and quadratics are raised to
            // cubics.
            for seg in BezPath::from_svg(&d).expect("path data").segments() {
                match seg {
                    PathSeg::Cubic(c) => cubics.push((c, 0.5 * width)),
                    PathSeg::Quad(q) => cubics.push((q.raise(), 0.5 * width)),
                    PathSeg::Line(_) => {}
                }
            }
        }
        assert!(cubics.len() > 300, "{} cubics", cubics.len());
        for tolerance in [1.0 / 32.0, 1.0 / 256.0, 1.0 / 2048.0] {
            let mut n_segments = 0;
            let mut n_over = 0;
            let mut worst = 0.0_f64;
            let mut result = BezPath::new();
            for &(c, half_width) in &cubics {
                for d in [half_width, -half_width] {
                    offset_cubic(c, d, tolerance, &mut result);
                    n_segments += result.segments().count();
                    let report = result_error(c, d, tolerance, &result);
                    let ratio = report.error.max(report.unmeasured_lower) / tolerance;
                    n_over += usize::from(ratio > 1.001);
                    worst = worst.max(ratio);
                }
            }
            std::println!(
                "tolerance 1/{}: {n_segments} segments, {n_over} calls over, worst {worst:.3}",
                (1.0 / tolerance).round()
            );
            if tolerance == 1.0 / 32.0 {
                assert_eq!(n_over, 0, "worst {worst}");
            }
        }
    }
}
