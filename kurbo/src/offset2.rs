// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Computation of offset curves of cubic Béziers, with strict error bounds.
//!
//! This is an experimental replacement for [`crate::offset`]. The recursion,
//! the regularization and the handling of cusps are taken from that module.
//! What is new is how a single piece is approximated and accepted:
//!
//! * The approximation is fitted with a linear solve (the "one-point" fit,
//!   or the "two-point" fit when the piece has an inflection), in place of
//!   arc drawing followed by least-squares refinement.
//! * It is accepted on the strength of a strict upper bound on the Fréchet
//!   distance to the true offset, in place of an error sampled at a few
//!   points.
//!
//! # The approximation
//!
//! Every approximation here has the form `A(t) = C(t) + D(t)`, where `C` is
//! the source piece and `D` is a cubic that is roughly `d` times the unit
//! normal of `C`. The end control points of `D` are exactly `d` times the
//! end normals, and the inner ones are displaced from those along the end
//! tangents, which makes `A` meet the offset with G1 continuity. The fits
//! differ only in the two displacements, called `a` and `b` as in
//! [`crate::offset`].
//!
//! # The bounds
//!
//! For a fixed `t`, let `F(s) = |A(t) - C(s)|^2 - d^2`. Its Taylor
//! coefficients about `s = t` are polynomials in `t`:
//!
//! ```text
//! F = P = |D|^2 - d^2,    F_s = -2 Q = -2 D.C',    F_ss = beta = 2 |C'|^2 - 2 D.C''
//! ```
//!
//! One Newton step on the foot point gives `H = P - 2 Q^2 / beta` as an
//! approximation to `V = rho^2 - d^2`, where `rho` is the distance from
//! `A(t)` to its foot on `C`. The strict bounds certify that the foot is
//! unique, local and monotone in `t`, bound the Taylor remainder, and then
//! bound `H` over the whole piece. G1 continuity gives `P` a double zero and
//! `Q` a simple zero at each end, which are divided out first.
//!
//! When the certificate fails there is a cruder bound that needs none: the
//! distance between `A(t)` and the offset at the same parameter. It is
//! first order in the tangential error, where the others are second order,
//! but it holds across cusps and for pieces of any shape.
//!
//! # Strictness
//!
//! On long or sharply turning pieces no strict bound may apply, or the
//! only one that does may be far too loose. [`offset_cubic_strict`] then
//! subdivides until one does. [`offset_cubic`] instead falls back on an
//! estimate of the same error, sampled at a few parameters, as
//! [`crate::offset`] does for every piece. That gives fewer segments, at
//! the cost of the guarantee on those pieces.
//!
//! # Status
//!
//! This is a first cut. See the notes on [`offset_cubic`] for what is and is
//! not handled yet.

#![allow(
    clippy::neg_cmp_op_on_partial_ord,
    reason = "The tests on which a bound depends are written so that a NaN fails them."
)]

use arrayvec::ArrayVec;

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

use crate::{
    BezPath, CubicBez, ParamCurve, ParamCurveDeriv, PathSeg, QuadBez, Vec2, common::solve_quadratic,
};

/// The smallest distance from a subdivision point to an end of the piece,
/// as a fraction of its parameter length.
const SPLIT_MARGIN: f64 = 1e-6;

/// The rounding allowance added to every strict bound, in units in the
/// last place of the largest coordinate; see [`CubicOffset::accepts`].
const ROUND_ULPS: f64 = 8.0;

/// The smallest share of a piece's parameter range on either side of a
/// subdivision point chosen by angle or curvature; see
/// [`CubicOffset::keep_inside`]. Measured on near cusps and hairpins
/// (`near_cusp_depth_survey`), 5 to 15% all do far better than none, and 10%
/// is the best balance; the speed elsewhere does not change.
const SUBDIVISION_MIN_SHARE: f64 = 0.1;

/// Maximum recursion depth.
///
/// This is a robustness guard. A piece at this depth is emitted whether or
/// not a bound has accepted it, so it should be reached only by input that
/// cannot be approximated. `crate::offset` uses 8, which is too few at tight
/// tolerances: near a sharp turn of the source, the offset sweeps an arc of
/// radius `|d|` over a sliver of parameter, and halving the parameter takes
/// many levels to reach it. At depth 8, every miss on the wild test family
/// was a piece emitted at this depth; at 12, none are.
const MAX_DEPTH: usize = 12;

/// We never let cusp values have an absolute value smaller than this.
const CUSP_EPSILON: f64 = 1e-12;

/// The largest parameter distance from an end, to the cusp next to it where
/// the derivative vanishes, for which the leading-order estimate of the stub
/// is used. See [`CubicOffset::end_cusp`].
const STUB_MAX_PARAM: f64 = 0.05;

/// The largest estimated extent of a stub, as a fraction of the tolerance,
/// that the default mode smooths over. The estimate is to leading order, so
/// this leaves room for the terms after it.
const STUB_MAX_EXTENT: f64 = 1.0 / 8.0;

/// The smallest determinant for which a linear fit is attempted.
///
/// The one-point fit has determinant `sin(theta)`, where `theta` is the
/// angle between the end tangents, so it degenerates both for a straight
/// piece and for a half turn.
const DET_EPSILON: f64 = 1e-9;

/// Compute an approximate offset curve.
///
/// The parallel curve of `c` offset by `d` is written to the `result` path.
/// This has the same signature and the same intent as
/// [`crate::offset::offset_cubic`].
///
/// Where the curvature changes little over a piece and the fit does not slip
/// far along the offset, a cheap estimate of its error (the osculating
/// parabola, with a safety factor) decides whether to accept it. Elsewhere a
/// piece is accepted when a strict upper bound on its Fréchet distance to
/// the true offset is within `tolerance`, and where no strict bound applies,
/// when a sampled estimate of its error, with a safety factor, is. Use
/// [`offset_cubic_strict`] to accept on strict bounds only, and
/// [`offset_cubic_pair`] for both sides of the source at once.
pub fn offset_cubic(c: CubicBez, d: f64, tolerance: f64, result: &mut BezPath) {
    offset_cubic_impl(c, d, tolerance, result, false);
}

/// Compute an approximate offset curve, accepting on strict bounds only.
///
/// This is [`offset_cubic`] without the estimates. Every piece of the result
/// is within `tolerance` of the offset of the regularized input, except a
/// piece at the maximum recursion depth, which is emitted regardless. The
/// bounds include an allowance for the rounding of the emitted control
/// points.
///
/// It can produce many more segments than [`offset_cubic`] on curves with
/// loops, tight turns or cusps.
pub fn offset_cubic_strict(c: CubicBez, d: f64, tolerance: f64, result: &mut BezPath) {
    offset_cubic_impl(c, d, tolerance, result, true);
}

/// Compute approximate offset curves on both sides of a cubic at once.
///
/// This appends an approximation of the offset of `c` by `d` to `positive`,
/// and one of the offset by `-d` to `negative`, as `curve_to` segments. Each
/// path must already have a current point, which should be where its offset
/// starts: `c.p0` plus `d` or `-d` times the unit normal there. This is what
/// a stroker needs, as it builds the two sides of a stroke in two paths.
///
/// The result is the same as [`offset_cubic`] at `d` and at `-d`, without the
/// initial `move_to`, but the two sides share the work that does not depend
/// on the side: regularizing and splitting the source, choosing where to
/// subdivide, and fitting.
pub fn offset_cubic_pair(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
) {
    offset_cubic_pair_impl(c, d, tolerance, positive, negative, false);
}

/// [`offset_cubic_pair`], accepting on strict bounds only.
///
/// The result is the same as [`offset_cubic_strict`] at `d` and at `-d`,
/// without the initial `move_to`.
pub fn offset_cubic_pair_strict(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
) {
    offset_cubic_pair_impl(c, d, tolerance, positive, negative, true);
}

fn offset_cubic_pair_impl(
    c: CubicBez,
    d: f64,
    tolerance: f64,
    positive: &mut BezPath,
    negative: &mut BezPath,
    strict: bool,
) {
    // As in `offset_cubic_impl`.
    const DIM_TUNE: f64 = 0.25;
    let c_regularized = c.regularize_cusp(tolerance * DIM_TUNE);
    let mut pos = CubicOffset::new(c_regularized, d, tolerance, strict);
    let mut neg = CubicOffset::new(c_regularized, -d, tolerance, strict);
    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
    let utan0 = tan0.normalize();
    let utan1 = tan1.normalize();
    let root = |co: &mut CubicOffset| OffsetRec {
        t0: 0.0,
        t1: 1.0,
        utan0,
        utan1,
        cusp0: co.end_cusp(false),
        cusp1: co.end_cusp(true),
        depth: 0,
    };
    let rec_pos = root(&mut pos);
    let rec_neg = root(&mut neg);
    offset_rec_pair(&pos, &neg, &rec_pos, &rec_neg, positive, negative);
}

/// The recursion of [`CubicOffset::offset_rec`], for both sides at once.
///
/// While both sides reject a piece, they are split at the same point, which
/// depends only on the source. Where one side accepts, the other carries on
/// alone. Where either side has a cusp in the piece, both carry on alone.
fn offset_rec_pair(
    pos: &CubicOffset,
    neg: &CubicOffset,
    rec_pos: &OffsetRec,
    rec_neg: &OffsetRec,
    out_pos: &mut BezPath,
    out_neg: &mut BezPath,
) {
    if rec_pos.has_cusp() || rec_neg.has_cusp() {
        pos.offset_rec(rec_pos, out_pos);
        neg.offset_rec(rec_neg, out_neg);
        return;
    }
    let seg = pos.piece(rec_pos);
    let mut shared = Shared::default();
    let (accepted_pos, best_pos, split_pos) = pos.approximate(&seg, rec_pos, &mut shared);
    let (accepted_neg, best_neg, split_neg) = neg.approximate(&seg, rec_neg, &mut shared);
    // A side that found a pair of cusps splits where it found them, alone.
    if accepted_pos.is_none()
        && accepted_neg.is_none()
        && split_pos.is_none()
        && split_neg.is_none()
        && rec_pos.depth < MAX_DEPTH
    {
        let SubdivisionPoint { t, utan } = pos.find_subdivision_point(rec_pos);
        let cusp_pos = pos.cusp_sign(t);
        let cusp_neg = neg.cusp_sign(t);
        let (pos0, pos1) = rec_pos.split(t, utan, cusp_pos, cusp_pos);
        let (neg0, neg1) = rec_neg.split(t, utan, cusp_neg, cusp_neg);
        offset_rec_pair(pos, neg, &pos0, &neg0, out_pos, out_neg);
        offset_rec_pair(pos, neg, &pos1, &neg1, out_pos, out_neg);
    } else {
        pos.emit_or_subdivide(rec_pos, accepted_pos, best_pos, split_pos, out_pos);
        neg.emit_or_subdivide(rec_neg, accepted_neg, best_neg, split_neg, out_neg);
    }
}

fn offset_cubic_impl(c: CubicBez, d: f64, tolerance: f64, result: &mut BezPath, strict: bool) {
    result.truncate(0);
    // See `crate::offset` for the reasoning behind regularization.
    const DIM_TUNE: f64 = 0.25;
    let c_regularized = c.regularize_cusp(tolerance * DIM_TUNE);
    let mut co = CubicOffset::new(c_regularized, d, tolerance, strict);
    let (tan0, tan1) = PathSeg::Cubic(c).tangents();
    let utan0 = tan0.normalize();
    let utan1 = tan1.normalize();
    let cusp0 = co.end_cusp(false);
    let cusp1 = co.end_cusp(true);
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
    co.offset_rec(&rec, result);
}

/// State used for computing an offset curve of a single cubic.
struct CubicOffset {
    /// The cubic being offset. This has been regularized.
    c: CubicBez,
    /// The derivative of `c`.
    q: QuadBez,
    /// The offset distance.
    d: f64,
    /// `c0 + c1 t + c2 t^2` is the cross product of second and first
    /// derivatives of the underlying cubic, multiplied by the offset.
    c0: f64,
    c1: f64,
    c2: f64,
    /// The tolerance.
    tolerance: f64,
    /// Whether to accept pieces on strict bounds only.
    strict: bool,
    /// Whether the offset has a stub at the start and at the end of the
    /// curve that is smoothed over rather than fitted. See
    /// [`CubicOffset::end_cusp`].
    smoothed_stub: [bool; 2],
}

/// State local to a subdivision.
struct OffsetRec {
    t0: f64,
    t1: f64,
    /// Unit tangent at `t0`.
    utan0: Vec2,
    /// Unit tangent at `t1`.
    utan1: Vec2,
    /// Cusp value (curvature times offset, plus one) at `t0`.
    cusp0: f64,
    /// Cusp value at `t1`.
    cusp1: f64,
    /// Recursion depth.
    depth: usize,
}

impl OffsetRec {
    /// The two halves of a piece split at `t`.
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

/// Result of choosing where to subdivide.
struct SubdivisionPoint {
    /// Source curve t value at subdivision point.
    t: f64,
    /// Unit tangent at subdivision point.
    utan: Vec2,
}

/// An approximation to the offset of one piece.
struct Fit {
    /// The approximation itself.
    approx: CubicBez,
    /// The approximation minus the source piece, `D = A - C`, as the
    /// control points of a cubic.
    delta: [Vec2; 4],
    /// Whether a handle was clamped to zero length, in which case the
    /// approximation is not the one the parameters asked for.
    clamped: bool,
}

impl CubicOffset {
    fn new(c: CubicBez, d: f64, tolerance: f64, strict: bool) -> Self {
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
            strict,
            smoothed_stub: [false; 2],
        }
    }

    /// Compute curvature of the source curve times offset plus 1.
    ///
    /// The offset has a cusp where this crosses zero, and an inflection of
    /// the source is where it equals one. See [`crate::offset`].
    fn cusp_sign(&self, t: f64) -> f64 {
        let ds2 = self.q.eval(t).to_vec2().hypot2();
        ((self.c2 * t + self.c1) * t + self.c0) / (ds2 * ds2.sqrt()) + 1.0
    }

    /// The cusp value times `|c'|^3`, and its derivative with respect to `t`.
    ///
    /// This has the sign of the cusp value, and unlike it has no pole where
    /// the derivative of the source vanishes, which suits Newton's method.
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

    /// Find a zero of the cusp value in `[a, b]`, where `s` times it is
    /// negative at `a` and positive at `b`.
    ///
    /// This is Newton's method on [`Self::scaled_cusp_and_deriv`],
    /// safeguarded by bisection, to an accuracy of about `1e-12` in `t`. It
    /// takes 6 to 8 evaluations where the ITP solver in `crate::offset`
    /// takes about 22.
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
            let newton = t - y / dy;
            // Bisect if Newton leaves the bracket, or is not converging
            // faster than bisection would.
            if newton > a && newton < b && (2.0 * y).abs() <= (step_old * dy).abs() {
                step_old = (newton - t).abs();
                t = newton;
                if step_old < EPS {
                    return t;
                }
            } else {
                step_old = 0.5 * (b - a);
                t = 0.5 * (a + b);
            }
        }
        0.5 * (a + b)
    }

    /// The cusp value at the start of the curve (`t = 0`) or at its end.
    ///
    /// Where the derivative vanishes at that end, the curvature there is
    /// infinite, and so is the cusp value. Its numerator `c0 + c1 t + c2 t^2`
    /// is then exactly `c2 (t - t_end)^2`, so the cusp value tends to
    /// infinity with the sign of `c2`. Where that is negative, the offset has
    /// a cusp next to the end, on the inside of the curve, and between the
    /// cusp and the end it runs backward: a stub. Only the sign matters to
    /// the recursion; the magnitude is taken as 1.
    ///
    /// The default mode reports the stub (a negative value), so that the
    /// cusp is found and the stub fitted, unless it is too small to matter:
    /// its estimates do not see it otherwise, and missed by up to ten times
    /// the tolerance. Strict mode does not: no certified bound accepts a
    /// piece containing the stub, and the bounds that do accept one measure
    /// against the offset with the stub in it. Resolving the cusp there only
    /// costs segments.
    ///
    /// This also records whether the stub at that end is smoothed over, for
    /// [`Self::apply`], so it is called once for each end, at the root.
    fn end_cusp(&mut self, at_end: bool) -> f64 {
        let (tan, y) = if at_end {
            (self.q.p2.to_vec2(), self.c0 + self.c1 + self.c2)
        } else {
            (self.q.p0.to_vec2(), self.c0)
        };
        const TAN_DIST_EPSILON: f64 = 1e-12;
        let tan_dist = tan.hypot();
        if tan_dist >= TAN_DIST_EPSILON {
            let rsqrt = 1.0 / tan_dist;
            return y * (rsqrt * rsqrt * rsqrt) + 1.0;
        }
        let smoothed = self.smooths_stub(at_end);
        self.smoothed_stub[usize::from(at_end)] = smoothed;
        if self.strict || !(self.c2 < 0.0) || smoothed {
            1.0
        } else {
            -1.0
        }
    }

    /// Whether the derivative vanishes at the start (or the end), the offset
    /// has a stub there, and the stub is small enough to smooth over.
    ///
    /// To leading order, with the derivative's control points `q` and `q1`
    /// the middle one, the offset near the end moves along the end tangent
    /// by `|q1| u^2 - d k u`, with `u` the parameter distance from the end and
    /// `k = (q1 x q_other) / (2 |q1|^2)` the rate at which the tangent turns.
    /// It turns back at the cusp, `u = d k / (2 |q1|)`, having gone
    /// `(d k)^2 / (4 |q1|)`.
    ///
    /// A smoothed stub keeps the cusp value 1 at that end, but the fit there
    /// keeps a handle pointing back along the stub rather than clamping it:
    /// clamped, it fell back to arc drawing, which missed by four times the
    /// tolerance on a stub far below it.
    ///
    /// The derivative is known to vanish at that end.
    fn smooths_stub(&self, at_end: bool) -> bool {
        if self.strict || !(self.c2 < 0.0) {
            return false;
        }
        let q1 = self.q.p1.to_vec2();
        let other = if at_end { self.q.p0 } else { self.q.p2 }.to_vec2();
        // With `m = |d (q1 x q_other)|`, the cusp is at `m / (4 |q1|^3)` and
        // the extent `m^2 / (16 |q1|^5)`; compared without dividing.
        let len2 = q1.hypot2();
        let len = len2.sqrt();
        let m = (self.d * q1.cross(other)).abs();
        len2 > 0.0
            && m <= 4.0 * STUB_MAX_PARAM * len2 * len
            && m * m <= 16.0 * STUB_MAX_EXTENT * self.tolerance * len2 * len2 * len
    }

    /// Primary entry point for recursive subdivision.
    fn offset_rec(&self, rec: &OffsetRec, result: &mut BezPath) {
        // If the cusp value changes sign, there is a cusp inside. Find it
        // and subdivide there. This is as in `crate::offset`, except for
        // the solver.
        if rec.has_cusp() {
            let a = rec.t0;
            let b = rec.t1;
            let s = rec.cusp1.signum();
            let t = self.find_cusp(a, b, s);
            // A root at an end of the piece is that end's sign being wrong,
            // as at an end where the derivative vanishes whose stub was
            // smoothed over: the region between is in fact past the cusp.
            // Give that end the other's sign instead of splitting off a
            // piece of zero length.
            let margin = SPLIT_MARGIN * (b - a);
            if t <= a + margin || t >= b - margin {
                self.offset_rec_without_cusp(rec, t <= a + margin, result);
                return;
            }
            // The derivative can vanish at an end of the piece, and the
            // solver can land there. Use the end tangent in that case.
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
            self.subdivide(rec, result, t, utan_t, cusp_t_minus, cusp_t_plus);
            return;
        }

        let seg = self.piece(rec);
        let (accepted, best_effort, split) = self.approximate(&seg, rec, &mut Unshared);
        self.emit_or_subdivide(rec, accepted, best_effort, split, result);
    }

    /// [`Self::offset_rec`] for a piece whose cusp lies at one of its ends,
    /// with that end given the other's sign.
    #[cold]
    #[inline(never)]
    fn offset_rec_without_cusp(&self, rec: &OffsetRec, at_start: bool, result: &mut BezPath) {
        let mut rec = OffsetRec { ..*rec };
        if at_start {
            rec.cusp0 = CUSP_EPSILON.copysign(rec.cusp1);
        } else {
            rec.cusp1 = CUSP_EPSILON.copysign(rec.cusp0);
        }
        self.offset_rec(&rec, result);
    }

    /// Approximate the offset of one piece.
    ///
    /// The first return value is an approximation that a strict bound has
    /// shown to be within tolerance, if there is one. The second is an
    /// approximation to fall back on if there is not and the piece cannot
    /// be subdivided further.
    fn approximate(
        &self,
        seg: &CubicBez,
        rec: &OffsetRec,
        sharing: &mut impl Sharing,
    ) -> (Option<CubicBez>, CubicBez, Option<f64>) {
        let d = self.d;
        let tolerance = self.tolerance;

        // A thin stroke can be accepted whatever the shape of the piece,
        // with arc drawing and a bound that needs no more than the fit. It
        // is at least `2 |d|`, so only try it when that is small enough.
        if 2.0 * d.abs() <= tolerance {
            let arc = self.draw_arc(seg, rec);
            if self.accepts(hairline_bound(&arc.delta, d), seg, &arc.approx) {
                return (Some(arc.approx), arc.approx, None);
            }
        }

        // At the root, a piece that turns very little can be accepted with
        // arc drawing and a bound that costs almost nothing. This also
        // catches straight and nearly straight pieces, for which the linear
        // fits below are ill conditioned.
        // The hull bound is at least `|d|` times the sine of the angle
        // between the end tangents, so skip it when that alone is too much.
        if rec.depth == 0 && d.abs() * rec.utan0.cross(rec.utan1).abs() <= tolerance {
            let arc = self.draw_arc(seg, rec);
            if let Some(bound) = hull_bound(seg, rec, &arc.delta, d) {
                if self.accepts(bound, seg, &arc.approx) {
                    return (Some(arc.approx), arc.approx, None);
                }
            }
        }

        // A piece that is itself smaller than the tolerance is the tip of a
        // near cusp of the source, or close to one. Its offset is nearly an
        // arc of a circle about the tip, however badly parametrized, and
        // arc drawing with a purely geometric bound deals with it.
        if (seg.p3 - seg.p0).hypot2() <= tolerance * tolerance {
            let arc = self.draw_arc(seg, rec);
            if let Some(bound) = small_piece_bound(seg, rec, &arc, d) {
                if self.accepts(bound, seg, &arc.approx) {
                    return (Some(arc.approx), arc.approx, None);
                }
            }
        }

        // An inflection is a sign change of the curvature, and the cusp
        // value is the curvature times `d`, plus one. With an inflection the
        // end tangents can be parallel, which defeats the one-point fit, so
        // use the two-point fit. Otherwise use one-point: it is cheaper to
        // bound, and it controls the tangential error better.
        let has_inflection = (rec.cusp0 - 1.0) * (rec.cusp1 - 1.0) < 0.0;
        // The linear fits do not depend on the side.
        let linear_fit = sharing.linear_fit(has_inflection, || {
            if has_inflection {
                two_point(seg, rec)
            } else {
                one_point(seg, rec)
            }
        });
        // In the default mode, try the osculating parabola estimate first. It
        // is a few multiply-adds per sample, against the Taylor polynomials,
        // certificate and bound, which cost more even when they succeed.
        // It is trusted only where its leading-order model holds: the
        // curvature times `d` does not change much over the piece, it has no
        // cusp at the samples, and a two-point fit does not slip far along
        // the offset.
        let cusp_min = rec.cusp0.min(rec.cusp1);
        let cusp_max = rec.cusp0.max(rec.cusp1);
        // A piece ending at a cusp, split there, has the value
        // `CUSP_EPSILON` at that end; the ratio does not apply to it.
        let cusp_ratio_ok =
            cusp_max <= OSCULATING_MAX_CUSP_RATIO * cusp_min || cusp_min == CUSP_EPSILON;
        if !self.strict && cusp_min > 0.0 && cusp_ratio_ok {
            let fit = match linear_fit {
                Some((a, b)) => {
                    let fit = self.apply(seg, rec, a, b);
                    if fit.clamped {
                        self.draw_arc(seg, rec)
                    } else {
                        fit
                    }
                }
                None => self.draw_arc(seg, rec),
            };
            let limit = tolerance / (OSCULATING_SAFETY * d.abs());
            // Only the two-point fit leaves the slip free. With one-point,
            // and with arc drawing, a large slip did not make the estimate
            // fall short, and limiting it sent most pieces with a vanishing
            // end derivative to the slower path.
            let two_point_fit = has_inflection && linear_fit.is_some() && !fit.clamped;
            let max_slip = if two_point_fit {
                OSCULATING_MAX_SLIP
            } else {
                f64::INFINITY
            };
            match osculating_decision(seg, &fit.delta, d, limit, max_slip) {
                Osculating::Decided(accepted) => {
                    return (accepted.then_some(fit.approx), fit.approx, None);
                }
                // The cusp value is negative at a sample, and positive at
                // both ends: there are at least two cusps in the piece.
                // Split there, so that both halves see a cusp.
                Osculating::Cusp(t) => {
                    return (None, fit.approx, Some(rec.t0 + t * (rec.t1 - rec.t0)));
                }
                Osculating::Untrusted => {}
            }
        }
        if let Some((a, b)) = linear_fit {
            // The linear fits assume their handles are used as computed.
            let fit = self.apply(seg, rec, a, b);
            if !fit.clamped {
                let taylor = Taylor::new(seg, &fit.delta, d);
                let certified = match Certificate::new(&taylor) {
                    // The two-point fit has no bound of its own. Where the
                    // general one does not accept it, try the bounds that
                    // need no certificate, and the estimate.
                    Some(certificate) if has_inflection => {
                        crude_certified_bound(&taylor, &certificate, d)
                            .filter(|&bound| self.accepts(bound, seg, &fit.approx))
                    }
                    Some(certificate) => one_point_bound(&taylor, &certificate, d),
                    None => None,
                };
                if let Some(bound) = certified {
                    // The certified bounds are tight, so take their word.
                    if self.accepts(bound, seg, &fit.approx) {
                        return (Some(fit.approx), fit.approx, None);
                    }
                    return (None, fit.approx, None);
                }
                // The certificate failed. Fall back to the bound that does
                // not need one, and then to the estimate.
                if self.accept_uncertified(seg, rec, &fit, &taylor) {
                    return (Some(fit.approx), fit.approx, None);
                }
                return (None, fit.approx, None);
            }
        }

        // No linear fit: the end tangents are parallel, the derivative
        // vanishes in the middle, or a handle would point the wrong way.
        // Arc drawing is always defined. It has no bound of its own, so use
        // the ones that need no certificate, and the estimate.
        let arc = self.draw_arc(seg, rec);
        let taylor = Taylor::new(seg, &arc.delta, d);
        if self.accept_uncertified(seg, rec, &arc, &taylor) {
            return (Some(arc.approx), arc.approx, None);
        }
        (None, arc.approx, None)
    }

    /// Whether a strict `bound` on the error of `approx` shows it to be
    /// within tolerance, allowing for rounding in its control points.
    ///
    /// The bounds are computed for the exact `D`, but the control points of
    /// `approx` are rounded in absolute coordinates, which no bound on `D`
    /// can see. As in the original (`ROUND_ULPS` in bezfit's
    /// `bound/cheap.rs`), allow a few ulps of the largest coordinate of the
    /// source and the approximation. It matters only when the tolerance
    /// approaches that floor.
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

    /// Decide whether to accept a piece without the foot-point certificate.
    ///
    /// The strict bounds that need no certificate are tried first. Unless
    /// `self.strict` is set, the sampled estimate is tried after them.
    fn accept_uncertified(
        &self,
        seg: &CubicBez,
        rec: &OffsetRec,
        fit: &Fit,
        taylor: &Taylor,
    ) -> bool {
        let d = self.d;
        let mut bound = equal_parameter_bound(taylor, rec, d);
        if bound.is_none() {
            bound = hull_bound(seg, rec, &fit.delta, d);
        }
        if let Some(bound) = bound {
            if self.accepts(bound, seg, &fit.approx) {
                return true;
            }
        }
        if self.strict {
            return false;
        }
        // With the curvature times `d` concentrated at one end, the error is
        // too, between the samples. A piece split at a cusp of the offset has
        // the value `CUSP_EPSILON` at that end; the ratio does not apply.
        let cusp_min = rec.cusp0.min(rec.cusp1);
        let cusp_max = rec.cusp0.max(rec.cusp1);
        if cusp_min > 0.0
            && cusp_min != CUSP_EPSILON
            && cusp_max > ESTIMATE_MAX_CUSP_RATIO * cusp_min
        {
            return false;
        }
        // Where the foot point is ill defined at a sample, which happens
        // if the piece hides a pair of cusps, estimate the distance at
        // equal parameter instead. That overestimates, but it is defined.
        let estimate = sampled_estimate(taylor, rec, d, self.tolerance / ESTIMATE_SAFETY)
            .unwrap_or_else(|| sampled_equal_parameter_distance(taylor, d));
        ESTIMATE_SAFETY * estimate <= self.tolerance
    }

    /// Recursively subdivide.
    ///
    /// See `crate::offset` for the treatment of the cusp values.
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

    /// The piece of the source that `rec` covers.
    #[inline(always)]
    fn piece(&self, rec: &OffsetRec) -> CubicBez {
        if rec.depth == 0 {
            self.c
        } else {
            self.c.subsegment(rec.t0..rec.t1)
        }
    }

    /// Emit an accepted piece, or subdivide a rejected one, or at the
    /// maximum depth emit the best effort anyway.
    #[inline(always)]
    fn emit_or_subdivide(
        &self,
        rec: &OffsetRec,
        accepted: Option<CubicBez>,
        best_effort: CubicBez,
        split: Option<f64>,
        result: &mut BezPath,
    ) {
        const UTAN_EPSILON: f64 = 1e-12;
        if let Some(approx) = accepted {
            result.curve_to(approx.p1, approx.p2, approx.p3);
        } else if let Some((t, q_t)) = split
            .filter(|_| rec.depth < MAX_DEPTH)
            .map(|t| (t, self.q.eval(t).to_vec2()))
            .filter(|(_, q_t)| q_t.length_squared() >= UTAN_EPSILON)
        {
            // At a sample with a negative cusp value, inside the piece.
            let cusp = self.cusp_sign(t);
            self.subdivide(rec, result, t, q_t.normalize(), cusp, cusp);
        } else if rec.depth < MAX_DEPTH {
            let SubdivisionPoint { t, utan } = self.find_subdivision_point(rec);
            let cusp = self.cusp_sign(t);
            self.subdivide(rec, result, t, utan, cusp, cusp);
        } else {
            // Out of depth. Emit the best approximation we have, though no
            // bound has accepted it.
            #[cfg(test)]
            tests::OUT_OF_DEPTH.with(|count| count.set(count.get() + 1));
            result.curve_to(best_effort.p1, best_effort.p2, best_effort.p3);
        }
    }

    /// Convert from (a, b) parameter space to an approximation.
    ///
    /// The control points of `D` are `d n0`, `d (n0 + a utan0)`,
    /// `d (n1 + b utan1)` and `d n1`, where `n0` and `n1` are the end
    /// normals.
    ///
    /// As in `crate::offset`, a handle of the approximation that would point
    /// against the direction the offset travels is clamped to zero length.
    #[inline(always)]
    fn apply(&self, seg: &CubicBez, rec: &OffsetRec, a: f64, b: f64) -> Fit {
        let d = self.d;
        let n0 = rec.utan0.turn_90();
        let n1 = rec.utan1.turn_90();
        let p0 = seg.p0 + d * n0;
        let p3 = seg.p3 + d * n1;
        // The handle lengths of the source piece, and of the approximation.
        let source_l0 = (seg.p1 - seg.p0).dot(rec.utan0);
        let source_l1 = (seg.p3 - seg.p2).dot(rec.utan1);
        let l0 = source_l0 + a * d;
        let l1 = source_l1 - b * d;

        // Form D directly, without subtracting control points of A and C,
        // whose coordinates may be much larger than the offset.
        let mut clamped = false;
        let mut p1 = p0 + l0 * rec.utan0;
        let mut delta1 = d * (n0 + a * rec.utan0);
        // At a smoothed stub the offset arrives moving backward, whatever
        // the cusp value says.
        let flip0 = rec.t0 == 0.0 && self.smoothed_stub[0];
        let flip1 = rec.t1 == 1.0 && self.smoothed_stub[1];
        if l0 * rec.cusp0 * if flip0 { -1.0 } else { 1.0 } <= 0.0 {
            p1 = p0;
            delta1 = p1 - seg.p1;
            clamped = true;
        }
        let mut p2 = p3 - l1 * rec.utan1;
        let mut delta2 = d * (n1 + b * rec.utan1);
        if l1 * rec.cusp1 * if flip1 { -1.0 } else { 1.0 } <= 0.0 {
            p2 = p3;
            delta2 = p2 - seg.p2;
            clamped = true;
        }
        Fit {
            approx: CubicBez::new(p0, p1, p2, p3),
            delta: [d * n0, delta1, delta2, d * n1],
            clamped,
        }
    }

    /// The arc drawing approximation of a piece.
    #[inline(always)]
    fn draw_arc(&self, seg: &CubicBez, rec: &OffsetRec) -> Fit {
        let (a, b) = draw_arc(rec.utan0, rec.utan1);
        self.apply(seg, rec, a, b)
    }

    /// Decide where to subdivide when a piece is not accepted.
    ///
    /// This is as in `crate::offset`: bisect the angle between the end
    /// tangents when there is no inflection, and otherwise subdivide by the
    /// integral of absolute curvature. Unlike there, a point at an end of
    /// the piece is never chosen; see [`Self::keep_inside`].
    fn find_subdivision_point(&self, rec: &OffsetRec) -> SubdivisionPoint {
        let t = 0.5 * (rec.t0 + rec.t1);
        let q_t = self.q.eval(t).to_vec2();
        let x0 = rec.utan0.cross(q_t).abs();
        let x1 = rec.utan1.cross(q_t).abs();
        const SUBDIVIDE_THRESH: f64 = 0.1;
        if x0 > SUBDIVIDE_THRESH * x1 && x1 > SUBDIVIDE_THRESH * x0 {
            let utan = q_t.normalize();
            return SubdivisionPoint { t, utan };
        }

        let chord = self.c.eval(rec.t1) - self.c.eval(rec.t0);
        if chord.cross(rec.utan0) * chord.cross(rec.utan1) < 0.0 {
            let tan = rec.utan0 + rec.utan1;
            if let Some(subdivision) =
                self.subdivide_for_tangent(rec.utan0, rec.t0, rec.t1, tan, false)
            {
                return self.keep_inside(rec, subdivision);
            }
        }
        // Curve definitely has an inflection point. Try to subdivide based
        // on integral of absolute curvature.
        let mut tangents: ArrayVec<Vec2, 4> = ArrayVec::new();
        let mut ts: ArrayVec<f64, 4> = ArrayVec::new();
        tangents.push(rec.utan0);
        ts.push(rec.t0);
        for t in self.c.inflections() {
            if t > rec.t0 && t < rec.t1 {
                tangents.push(self.q.eval(t).to_vec2());
                ts.push(t);
            }
        }
        tangents.push(rec.utan1);
        ts.push(rec.t1);
        let mut arc_angles: ArrayVec<f64, 3> = ArrayVec::new();
        let mut sum = 0.0;
        for i in 0..tangents.len() - 1 {
            let tan0 = tangents[i];
            let tan1 = tangents[i + 1];
            let th = tan0.cross(tan1).atan2(tan0.dot(tan1));
            sum += th.abs();
            arc_angles.push(th);
        }
        let mut target = sum * 0.5;
        let mut i = 0;
        while arc_angles[i].abs() < target {
            target -= arc_angles[i].abs();
            i += 1;
        }
        let rotation = Vec2::from_angle(target.copysign(arc_angles[i]));
        let base = tangents[i];
        let tan = base.rotate_scale(rotation);
        let utan0 = if i == 0 { rec.utan0 } else { base.normalize() };
        let subdivision = self
            .subdivide_for_tangent(utan0, ts[i], ts[i + 1], tan, true)
            .unwrap();
        self.keep_inside(rec, subdivision)
    }

    /// Move a subdivision point that is within [`SUBDIVISION_MIN_SHARE`] of
    /// an end of the piece to that distance from it.
    ///
    /// Splitting at an end leaves a piece of zero length, whose end tangents
    /// are arbitrary and whose cusp value can be `0 / 0`. Its approximation
    /// is then an arc fanned about the end point, which the bounds, looking
    /// at one piece at a time, cannot see is wrong. This happens where the
    /// derivative of the source vanishes at an end: `t = 0` then solves
    /// `tan x c'(t) = 0` for every direction `tan`.
    ///
    /// Splitting very near an end is nearly as bad. Next to a near cusp of
    /// the source almost all the turning is at the tip, and the angle
    /// bisector lands within a millionth of the piece from it, so each split
    /// leaves a piece that turns less but is as long as before.
    fn keep_inside(&self, rec: &OffsetRec, subdivision: SubdivisionPoint) -> SubdivisionPoint {
        let margin = SUBDIVISION_MIN_SHARE * (rec.t1 - rec.t0);
        if subdivision.t > rec.t0 + margin && subdivision.t < rec.t1 - margin {
            return subdivision;
        }
        let t = subdivision.t.clamp(rec.t0 + margin, rec.t1 - margin);
        const UTAN_EPSILON: f64 = 1e-12;
        let q_t = self.q.eval(t).to_vec2();
        let utan = if q_t.length_squared() >= UTAN_EPSILON {
            q_t.normalize()
        } else {
            subdivision.utan
        };
        SubdivisionPoint { t, utan }
    }

    /// Find a subdivision point, given a tangent vector.
    ///
    /// This is as in `crate::offset`, except that roots at the ends of
    /// `[t0, t1]` are ignored.
    fn subdivide_for_tangent(
        &self,
        utan0: Vec2,
        t0: f64,
        t1: f64,
        tan: Vec2,
        force: bool,
    ) -> Option<SubdivisionPoint> {
        let mut t = 0.0;
        let mut n_soln = 0;
        let z0 = tan.cross(self.q.p0.to_vec2());
        let z1 = tan.cross(self.q.p1.to_vec2());
        let z2 = tan.cross(self.q.p2.to_vec2());
        let c0 = z0;
        let c1 = 2.0 * (z1 - z0);
        let c2 = (z2 - z1) - (z1 - z0);
        // Roots at the ends are not useful, and where the derivative
        // vanishes at an end, that end is a root for every `tan`.
        let margin = SPLIT_MARGIN * (t1 - t0);
        for root in solve_quadratic(c0, c1, c2) {
            if root > t0 + margin && root < t1 - margin {
                t = root;
                n_soln += 1;
            }
        }
        if n_soln != 1 {
            if !force {
                return None;
            }
            if self.q.eval(t0).to_vec2().length_squared()
                > self.q.eval(t1).to_vec2().length_squared()
            {
                t = t1;
            } else {
                t = t0;
            }
        }
        let q = self.q.eval(t).to_vec2();
        const UTAN_EPSILON: f64 = 1e-12;
        let utan = if n_soln == 1 && q.length_squared() >= UTAN_EPSILON {
            q.normalize()
        } else if tan.length_squared() >= UTAN_EPSILON {
            tan.normalize()
        } else {
            utan0.turn_90()
        };
        Some(SubdivisionPoint { t, utan })
    }
}

// ---------------------------------------------------------------------------
// Fits
//
// Each of these returns the parameters (a, b) of an approximation, as taken
// by `CubicOffset::apply`. None of them depends on the offset distance.

/// Arc drawing.
///
/// `D / d` is the standard cubic approximation of the arc of the unit
/// circle from one end normal to the other. This is always defined, and it
/// is exact for a straight piece. Its error is fourth order in the length
/// of the piece. See `crate::offset`.
#[inline(always)]
fn draw_arc(utan0: Vec2, utan1: Vec2) -> (f64, f64) {
    // With theta the angle from utan1 to utan0, the handle length is
    // 4/3 tan(theta / 4) = 4/3 sin(theta / 2) / (1 + cos(theta / 2)).
    let cos_th = utan1.dot(utan0);
    let sin_th = utan1.cross(utan0);
    let cos_half = (0.5 * (1.0 + cos_th)).max(0.0).sqrt();
    let sin_half = (0.5 * (1.0 - cos_th)).max(0.0).sqrt().copysign(sin_th);
    let a = (4.0 / 3.0) * sin_half / (1.0 + cos_half);
    (a, -a)
}

/// The one-point fit.
///
/// At `t = 1/2` the approximation passes through the offset of the source
/// at the same parameter. Its error is fifth order in the length of the
/// piece. The tangential error also vanishes there, which keeps it small
/// throughout and is what makes this fit cheap to bound.
///
/// Returns `None` if the end tangents are parallel or the derivative of the
/// source vanishes at the midpoint.
#[inline(always)]
fn one_point(seg: &CubicBez, rec: &OffsetRec) -> Option<(f64, f64)> {
    let det = rec.utan0.cross(rec.utan1);
    if det.abs() < DET_EPSILON {
        return None;
    }
    let n0 = rec.utan0.turn_90();
    let n1 = rec.utan1.turn_90();
    let n_mid = unit_normal(seg, 0.5)?;
    // D(1/2) / d = (n0 + n1) / 2 + 3/8 (a utan0 + b utan1), which is to
    // equal n_mid.
    let rhs = (8.0 / 3.0) * (n_mid - 0.5 * (n0 + n1));
    let a = rhs.cross(rec.utan1) / det;
    let b = rec.utan0.cross(rhs) / det;
    Some((a, b))
}

/// The linear two-point fit.
///
/// At `t = 1/3` and `t = 2/3` the component of `D` along the source normal
/// at the same parameter equals `d`. Unlike one-point this works when the
/// end tangents are parallel, as they can be when the piece has an
/// inflection. It does not control the tangential error, and its error is
/// fourth order in the length of the piece.
///
/// Returns `None` if the system is singular.
#[inline(always)]
fn two_point(seg: &CubicBez, rec: &OffsetRec) -> Option<(f64, f64)> {
    let n0 = rec.utan0.turn_90();
    let n1 = rec.utan1.turn_90();
    let mut matrix = [[0.0; 2]; 2];
    let mut rhs = [0.0; 2];
    for (i, t) in [1.0 / 3.0, 2.0 / 3.0].into_iter().enumerate() {
        let n = unit_normal(seg, t)?;
        let mt = 1.0 - t;
        // The Bernstein weights of the four control points at t.
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

// ---------------------------------------------------------------------------
// Polynomial helpers
//
// Polynomials are in Bernstein form on [0, 1]. An array of length `n` holds
// a polynomial of degree `n - 1`.

/// Binomial coefficients, `BINOM[n][k]` for `n` up to 6.
const BINOM: [[f64; 7]; 7] = [
    [1., 0., 0., 0., 0., 0., 0.],
    [1., 1., 0., 0., 0., 0., 0.],
    [1., 2., 1., 0., 0., 0., 0.],
    [1., 3., 3., 1., 0., 0., 0.],
    [1., 4., 6., 4., 1., 0., 0.],
    [1., 5., 10., 10., 5., 1., 0.],
    [1., 6., 15., 20., 15., 6., 1.],
];

/// The reciprocals of the binomial coefficients.
const BINOM_RECIP: [[f64; 7]; 7] = binom_recip();

const fn binom_recip() -> [[f64; 7]; 7] {
    let mut result = [[0.0; 7]; 7];
    let mut n = 0;
    while n < 7 {
        let mut k = 0;
        while k <= n {
            result[n][k] = 1.0 / BINOM[n][k];
            k += 1;
        }
        n += 1;
    }
    result
}

/// Take the dot product of two vector polynomials in Bernstein form.
///
/// The output length `K` must be `M + N - 1`.
fn dot<const M: usize, const N: usize, const K: usize>(a: &[Vec2; M], b: &[Vec2; N]) -> [f64; K] {
    debug_assert_eq!(K, M + N - 1);
    let mut result = [0.0; K];
    for i in 0..M {
        for j in 0..N {
            result[i + j] += BINOM[M - 1][i] * BINOM[N - 1][j] * a[i].dot(b[j]);
        }
    }
    for k in 0..K {
        result[k] *= BINOM_RECIP[K - 1][k];
    }
    result
}

/// Take the dot product of a vector polynomial in Bernstein form with
/// itself, using the symmetry to halve the work.
///
/// The output length `K` must be `2 M - 1`.
fn dot_self<const M: usize, const K: usize>(a: &[Vec2; M]) -> [f64; K] {
    debug_assert_eq!(K, 2 * M - 1);
    let mut result = [0.0; K];
    for i in 0..M {
        result[2 * i] += BINOM[M - 1][i] * BINOM[M - 1][i] * a[i].hypot2();
        for j in 0..i {
            result[i + j] += 2.0 * BINOM[M - 1][i] * BINOM[M - 1][j] * a[i].dot(a[j]);
        }
    }
    for k in 0..K {
        result[k] *= BINOM_RECIP[K - 1][k];
    }
    result
}

/// An upper bound on the norm of a vector polynomial in Bernstein form.
fn max_norm(coeffs: &[Vec2]) -> f64 {
    let mut result = 0.0_f64;
    for v in coeffs {
        result = result.max(v.hypot2());
    }
    result.sqrt()
}

/// A lower bound on a polynomial in Bernstein form.
fn min_coeff(coeffs: &[f64]) -> f64 {
    let mut result = f64::INFINITY;
    for &x in coeffs {
        result = result.min(x);
    }
    result
}

/// An upper bound on a polynomial in Bernstein form.
fn max_coeff(coeffs: &[f64]) -> f64 {
    let mut result = f64::NEG_INFINITY;
    for &x in coeffs {
        result = result.max(x);
    }
    result
}

/// An upper bound on the absolute value of a polynomial in Bernstein form.
fn max_abs_coeff(coeffs: &[f64]) -> f64 {
    let mut result = 0.0_f64;
    for &x in coeffs {
        result = result.max(x.abs());
    }
    result
}

// ---------------------------------------------------------------------------
// The Taylor coefficients and the foot-point certificate

/// The Taylor coefficients of `F(s) = |A(t) - C(s)|^2 - d^2` about `s = t`,
/// and the vector polynomials they are built from.
#[derive(Clone)]
struct Taylor {
    /// `D = A - C`.
    delta: [Vec2; 4],
    /// `C'`.
    c1: [Vec2; 3],
    /// `C''`.
    c2: [Vec2; 2],
    /// `C'''`, which is constant.
    c3: Vec2,
    /// `A'`.
    a1: [Vec2; 3],
    /// `P = |D|^2 - d^2`, of degree 6. This is `F`.
    p: [f64; 7],
    /// `Q = D.C'`, of degree 5. `F_s` is `-2 Q`.
    q: [f64; 6],
    /// `beta = 2 |C'|^2 - 2 D.C''`, of degree 4. This is `F_ss`.
    beta: [f64; 5],
    /// `A'.C'`, of degree 4.
    a1_dot_c1: [f64; 5],
    /// `|C'|^2`, of degree 4.
    c1_dot_c1: [f64; 5],
}

impl Taylor {
    fn new(seg: &CubicBez, delta: &[Vec2; 4], d: f64) -> Self {
        let c1 = [
            3.0 * (seg.p1 - seg.p0),
            3.0 * (seg.p2 - seg.p1),
            3.0 * (seg.p3 - seg.p2),
        ];
        let c2 = [2.0 * (c1[1] - c1[0]), 2.0 * (c1[2] - c1[1])];
        let c3 = c2[1] - c2[0];
        // A' = C' + D'.
        let a1 = [
            c1[0] + 3.0 * (delta[1] - delta[0]),
            c1[1] + 3.0 * (delta[2] - delta[1]),
            c1[2] + 3.0 * (delta[3] - delta[2]),
        ];

        let delta_dot_delta: [f64; 7] = dot_self(delta);
        let mut p = delta_dot_delta;
        for coeff in &mut p {
            *coeff -= d * d;
        }
        let q: [f64; 6] = dot(delta, &c1);
        let c1_dot_c1: [f64; 5] = dot_self(&c1);
        let delta_dot_c2: [f64; 5] = dot(delta, &c2);
        let mut beta = [0.0; 5];
        for i in 0..5 {
            beta[i] = 2.0 * (c1_dot_c1[i] - delta_dot_c2[i]);
        }
        // A'.C' = beta / 2 + Q', and the derivative of a degree 5
        // polynomial has coefficients 5 (q[i + 1] - q[i]).
        let mut a1_dot_c1 = [0.0; 5];
        for i in 0..5 {
            a1_dot_c1[i] = 0.5 * beta[i] + 5.0 * (q[i + 1] - q[i]);
        }
        Taylor {
            delta: *delta,
            c1,
            c2,
            c3,
            a1,
            p,
            q,
            beta,
            a1_dot_c1,
            c1_dot_c1,
        }
    }
}

/// How [`CubicOffset::approximate`] gets the linear fit, which does not
/// depend on the side of the source. For one side, [`Unshared`] computes it.
/// For both sides, [`Shared`] keeps what the first side computed for the
/// second.
trait Sharing {
    fn linear_fit(
        &mut self,
        two_point_chosen: bool,
        fit: impl FnOnce() -> Option<(f64, f64)>,
    ) -> Option<(f64, f64)>;
}

/// Compute the linear fit directly, for one side.
struct Unshared;

impl Sharing for Unshared {
    #[inline(always)]
    fn linear_fit(
        &mut self,
        _two_point_chosen: bool,
        fit: impl FnOnce() -> Option<(f64, f64)>,
    ) -> Option<(f64, f64)> {
        fit()
    }
}

/// Keep the linear fit on a piece for the other side, in
/// [`offset_cubic_pair`]. Sharing the Taylor coefficients and the parts of
/// the certificate that do not depend on the side as well saved nothing
/// measurable.
#[derive(Default)]
struct Shared {
    /// Whether the two-point fit was chosen, and the linear fit's result.
    linear_fit: Option<(bool, Option<(f64, f64)>)>,
}

impl Sharing for Shared {
    fn linear_fit(
        &mut self,
        two_point_chosen: bool,
        fit: impl FnOnce() -> Option<(f64, f64)>,
    ) -> Option<(f64, f64)> {
        match self.linear_fit {
            Some((chosen, result)) if chosen == two_point_chosen => result,
            _ => {
                let result = fit();
                self.linear_fit = Some((two_point_chosen, result));
                result
            }
        }
    }
}

/// The result of a successful foot-point certificate.
///
/// The certificate establishes that for each `t`, the function `F` has a
/// unique stationary point `s(t)` near `t`, and that `s(t)` is increasing.
/// Then `s` is a valid reparametrization, and the distance from `A(t)` to
/// the offset at `s(t)` is `|rho - |d||`, with `rho^2 - d^2 = F(s(t))`.
///
/// On an ordinary piece `beta` is positive and the stationary point is a
/// minimum of `F`. Past a cusp of the offset, where the offset runs
/// backwards, `beta` is negative and it is a maximum. Everything else is
/// the same with the sign of `beta` reversed.
struct Certificate {
    /// The sign of `beta` over the piece, `1.0` or `-1.0`.
    sign: f64,
    /// A positive lower bound on `|beta|`.
    beta_min: f64,
    /// An upper bound on `|Q|`.
    q_max: f64,
    /// A bound on `|V - H|`, where `V = F(s(t))` and `H = P - 2 Q^2 / beta`
    /// is the stationary value of the quadratic model of `F`, attained at
    /// `sigma_0 = 2 Q / beta`.
    newton_remainder: f64,
}

impl Certificate {
    fn new(taylor: &Taylor) -> Option<Self> {
        // beta must keep one sign over the piece.
        let sign = if min_coeff(&taylor.beta) > 0.0 {
            1.0
        } else if max_coeff(&taylor.beta) < 0.0 {
            -1.0
        } else {
            return None;
        };
        let mut beta_min = f64::INFINITY;
        let mut a1_dot_c1_min = f64::INFINITY;
        for i in 0..5 {
            beta_min = beta_min.min(sign * taylor.beta[i]);
            a1_dot_c1_min = a1_dot_c1_min.min(sign * taylor.a1_dot_c1[i]);
        }
        let q_max = q_bound(&taylor.q);
        // C'.C'' is half the derivative of |C'|^2. Bounding it by its own
        // coefficients, rather than by |C'| |C''|, matters where the speed
        // is nearly uniform and C'.C'' is small.
        let k = &taylor.c1_dot_c1;
        let c1_dot_c2 = [
            2.0 * (k[1] - k[0]),
            2.0 * (k[2] - k[1]),
            2.0 * (k[3] - k[2]),
            2.0 * (k[4] - k[3]),
        ];
        let c1_dot_c2_max = max_abs_coeff(&c1_dot_c2);
        let delta_norm = max_norm(&taylor.delta);
        let c1_norm = max_norm(&taylor.c1);
        let c2_norm = max_norm(&taylor.c2);
        let c3_norm = taylor.c3.hypot();
        let a1_norm = max_norm(&taylor.a1);

        // The derivative bounds must hold for s within `margin` of [0, 1],
        // and the foot must lie within that margin. Grow the margin until
        // it does. The foot is never nearer than 2 Q / beta, so start a
        // little beyond that.
        let mut margin = 1.25 * 2.0 * q_max / beta_min;
        for _ in 0..8 {
            let c2_wide = c2_norm + margin * c3_norm;
            let c1_wide = c1_norm + margin * c2_wide;
            let delta_wide = delta_norm + margin * c1_wide;

            // F_sss = 6 C'.C'' - 2 (A(t) - C(s)).C'''
            // (C'.C'')' = |C''|^2 + C'.C'''.
            let c1_dot_c2_wide = c1_dot_c2_max + (c2_wide * c2_wide + c1_wide * c3_norm) * margin;
            let k3 = 6.0 * c1_dot_c2_wide + 2.0 * delta_wide * c3_norm;

            // delta is the smaller root of k3 delta^2 - beta_min delta + 2 q = 0.
            let discriminant = beta_min * beta_min - 8.0 * k3 * q_max;
            // Written this way so that a NaN also fails.
            if !(discriminant > 0.0) {
                return None;
            }
            let delta = 4.0 * q_max / (beta_min + discriminant.sqrt());

            if delta <= margin {
                // ds/dt = 2 A'(t).C'(s) / F_ss must be positive. Compare
                // A'(t).C'(t) with the most that C' can change between t
                // and s.
                if !(a1_dot_c1_min > a1_norm * c2_wide * delta) {
                    return None;
                }
                let newton_remainder = k3 * delta.powi(3) / 6.0;
                return Some(Certificate {
                    sign,
                    beta_min,
                    q_max,
                    newton_remainder,
                });
            }
            margin = 1.25 * delta;
        }
        None
    }
}

/// An upper bound on `|Q|` over [0, 1], from its Bernstein coefficients.
///
/// The fits make `Q` vanish at both ends, and the one-point fit also at
/// `t = 1/2`. Factoring those zeros out first gives a much better bound than
/// the coefficients of `Q` do. The factorizations are exact for any `Q`,
/// with what does not vanish carried as a residual, so the smallest of the
/// three bounds is valid whatever the fit.
///
/// `Q = Q_end + u S` exactly, with `u = t (1 - t)`, `Q_end` the end terms and
/// `S` a cubic. Then `S = (t - 1/2) S2 + s_misfit u` with `S2` quadratic,
/// as in [`one_point_bound`].
fn q_bound(q: &[f64; 6]) -> f64 {
    /// The maximum of `|t (1 - t) (t - 1/2)|` on [0, 1], `1 / (12 sqrt 3)`,
    /// rounded up.
    const V_MAX: f64 = 0.048_112_522_432_469;
    let q_residual = q[0].abs().max(q[5].abs());
    let s = [5.0 * q[1], 10.0 / 3.0 * q[2], 10.0 / 3.0 * q[3], 5.0 * q[4]];
    let s2 = [
        -2.0 * s[0],
        0.5 * ((-s[0] - 3.0 * s[1]) + (3.0 * s[2] + s[3])),
        2.0 * s[3],
    ];
    let s_misfit = 0.5 * (s[0] + 3.0 * s[1] + 3.0 * s[2] + s[3]);
    let plain = max_abs_coeff(q);
    let factored = 0.25 * max_abs_coeff(&s) + q_residual;
    let one_point = V_MAX * max_abs_coeff(&s2) + s_misfit.abs() / 16.0 + q_residual;
    plain.min(factored).min(one_point)
}

/// Convert bounds on `V = rho^2 - d^2` into a bound on `|rho - |d||`.
///
/// The arguments are such that `V <= v_above` and `-V <= v_below`. Returns
/// `None` if `rho` could reach zero, in which case the approximation could
/// cross to the other side of the source curve.
#[inline(always)]
fn distance_bound(v_above: f64, v_below: f64, d: f64) -> Option<f64> {
    let d_squared = d * d;
    if !(v_below < d_squared) {
        return None;
    }
    let d_abs = d.abs();
    let too_far = v_above / ((d_squared + v_above).sqrt() + d_abs);
    let too_near = v_below / (d_abs + (d_squared - v_below).sqrt());
    Some(too_far.max(too_near))
}

/// Bound the error of any approximation for which the certificate holds.
///
/// This uses only the ranges of `P`, `Q` and `beta`. On an ordinary piece
/// the foot is a minimum of `F`, so `V <= F(t) = P`, and
/// `V >= H - remainder` with `H = P - 2 Q^2 / beta`. Past a cusp the two
/// are exchanged.
///
/// It makes no use of the zeros of `P` and `Q`, so it is looser than the
/// bounds for particular fits, but it is second order in the tangential
/// error and it does not need the approximation to be G1.
#[inline(always)]
fn crude_certified_bound(taylor: &Taylor, certificate: &Certificate, d: f64) -> Option<f64> {
    let p_above = max_coeff(&taylor.p).max(0.0);
    let p_below = (-min_coeff(&taylor.p)).max(0.0);
    let slip = 2.0 * certificate.q_max * certificate.q_max / certificate.beta_min;
    let remainder = certificate.newton_remainder;
    if certificate.sign > 0.0 {
        distance_bound(p_above + remainder, p_below + slip + remainder, d)
    } else {
        distance_bound(p_above + slip + remainder, p_below + remainder, d)
    }
}

// ---------------------------------------------------------------------------
// The one-point bound

/// The maximum of `|w|` on [0, 1], where `w = u^2 (t - 1/2)` and
/// `u = t (1 - t)`.
///
/// The value is `1 / (25 sqrt(20))`, attained at `t = 1/2 ± 1/sqrt(20)`.
const W_MAX: f64 = 0.008_944_271_909_999_159;

/// The maximum of `w2 = u^2 (t - 1/2)^2` on [0, 1].
///
/// Attained at `t = 1/2 ± 1/sqrt(12)`.
const W2_MAX: f64 = 1.0 / 432.0;

/// Bound the error of the one-point fit.
///
/// The one-point fit gives `P` and `Q` a further zero at `t = 1/2`. With
/// those and the end zeros divided out, the Newton value is
///
/// ```text
/// H = w r_mid + w2 (r_slope - 2 g),    g = S2^2 / beta
/// ```
///
/// where `w` and `w2` are the fixed weights above, `r_mid` and `r_slope`
/// are numbers, and `g` varies slowly. The two terms are bounded
/// separately.
///
/// This stays valid for an approximation that is not a one-point fit, since
/// what is left of `P` and `Q` at the midpoint is carried along as an
/// error. It is then very loose.
#[inline(always)]
fn one_point_bound(taylor: &Taylor, certificate: &Certificate, d: f64) -> Option<f64> {
    let p = &taylor.p;
    let q = &taylor.q;
    let beta_min = certificate.beta_min;

    // G1 continuity gives P a double zero and Q a simple zero at each end,
    // so P = u^2 R and Q = u S. In floating point the end coefficients are
    // not exactly zero; what is left of them is carried as a residual.
    let p_residual = max_abs_coeff(&[p[0], p[1], p[5], p[6]]);
    let q_residual = max_abs_coeff(&[q[0], q[5]]);
    // Dividing out u^2 or u rescales the remaining coefficients by a ratio
    // of binomial coefficients.
    let r = [15.0 * p[2], 10.0 * p[3], 15.0 * p[4]];
    let s = [5.0 * q[1], 10.0 / 3.0 * q[2], 10.0 / 3.0 * q[3], 5.0 * q[4]];

    // R = (t - 1/2) R1 + 2 r_misfit u, with R1 = r_mid + (t - 1/2) r_slope.
    // The misfit is zero when R vanishes at the midpoint.
    let r_mid = r[2] - r[0];
    let r_slope = 2.0 * (r[0] + r[2]);
    let r_misfit = r[1] + 0.5 * (r[0] + r[2]);

    // S = (t - 1/2) S2 + s_misfit u, with S2 quadratic. The middle
    // coefficient of S2 can be read off from either side; take the mean.
    let s2 = [
        -2.0 * s[0],
        0.5 * ((-s[0] - 3.0 * s[1]) + (3.0 * s[2] + s[3])),
        2.0 * s[3],
    ];
    let s_misfit = 0.5 * (s[0] + 3.0 * s[1] + 3.0 * s[2] + s[3]);

    // The errors of the factored forms, using u <= 1/4:
    //
    // P - w R1            = 2 r_misfit u^3 + (end residual)
    // Q - u (t - 1/2) S2  = s_misfit u^2   + (end residual)
    let p_error = r_misfit.abs() / 32.0 + p_residual;
    let q_error = s_misfit.abs() / 16.0 + q_residual;
    let q_squared_error = q_error * (2.0 * certificate.q_max + q_error);
    let remainder = certificate.newton_remainder + p_error + 2.0 * q_squared_error / beta_min;

    // Since beta keeps one sign, g lies within the range of the ratios of
    // the coefficients of its numerator and denominator.
    let s2_squared = [
        s2[0] * s2[0],
        s2[0] * s2[1],
        (s2[0] * s2[2] + 2.0 * s2[1] * s2[1]) / 3.0,
        s2[1] * s2[2],
        s2[2] * s2[2],
    ];
    let mut ratios = [0.0; 5];
    for i in 0..5 {
        ratios[i] = s2_squared[i] / taylor.beta[i];
    }
    let mut g_min = min_coeff(&ratios);
    let mut g_max = max_coeff(&ratios);
    // g is a square over beta, so it also has the sign of beta.
    if certificate.sign > 0.0 {
        g_min = g_min.max(0.0);
    } else {
        g_max = g_max.min(0.0);
    }

    // The odd term, w r_mid, takes both signs. The even term has the sign
    // of (r_slope - 2 g), since w2 is nonnegative.
    let odd_term = W_MAX * r_mid.abs();
    let v_above = odd_term + W2_MAX * (r_slope - 2.0 * g_min).max(0.0) + remainder;
    let v_below = odd_term + W2_MAX * (2.0 * g_max - r_slope).max(0.0) + remainder;
    distance_bound(v_above, v_below, d)
}

// ---------------------------------------------------------------------------
// Bounds that need no certificate

/// Bound the distance between the approximation and the offset at equal
/// parameter.
///
/// That distance is `|D(t) - d n(t)|`, where `n` is the unit normal of the
/// source, and its maximum is an upper bound on the Fréchet distance
/// whatever the shape of the piece. No foot point is involved, so this holds
/// across cusps of the offset.
///
/// Writing `D / d` as `r` times a unit vector at angle `alpha` to `n`, the
/// squared distance is `d^2 (r^2 + 1 - 2 r cos(alpha))`. The range of `r^2`
/// comes from the coefficients of `P`, and `r sin(alpha) = D.C' / (d |C'|)`
/// is bounded from the coefficients of `Q` and a lower bound on the speed.
///
/// Returns `None` if the speed of the source cannot be bounded away from
/// zero, or if the tangential error is too large for the argument.
fn equal_parameter_bound(taylor: &Taylor, rec: &OffsetRec, d: f64) -> Option<f64> {
    let speed_min = speed_lower_bound(taylor, rec);
    if !(speed_min > 0.0) {
        return None;
    }

    // p = |D / d|^2 - 1 and b = r sin(alpha).
    let d_squared = d * d;
    let p_lo = min_coeff(&taylor.p) / d_squared;
    let p_hi = max_coeff(&taylor.p) / d_squared;
    let b = max_abs_coeff(&taylor.q) / (d.abs() * speed_min);
    let b_squared = b * b;
    // r cos(alpha) is one at the start, and its square is r^2 - b^2. If
    // that stays positive then so does r cos(alpha).
    if !(1.0 + p_lo - b_squared > 0.0) {
        return None;
    }
    // r^2 + 1 - 2 sqrt(r^2 - b^2), written so that nothing cancels: with
    // x = p - b^2 and s = sqrt(1 + x) it is (p x / (1 + s) + 2 b^2) / (1 + s).
    // It is largest at an end of the range of p, for the largest b.
    let squared_distance = |p: f64| {
        let x = p - b_squared;
        let one_plus_s = 1.0 + (1.0 + x).sqrt();
        (p * x / one_plus_s + 2.0 * b_squared) / one_plus_s
    };
    let worst = squared_distance(p_lo).max(squared_distance(p_hi)).max(0.0);
    Some(d.abs() * worst.sqrt())
}

/// A lower bound on the speed of the source, or zero if none is found.
///
/// This is the better of two: the component of `C'` along the bisector of
/// the end tangents, which is good for a piece that turns little, and the
/// square root of the smallest coefficient of `|C'|^2`, which does not care
/// how far the piece turns.
fn speed_lower_bound(taylor: &Taylor, rec: &OffsetRec) -> f64 {
    let bisector = rec.utan0 + rec.utan1;
    let bisector_len = bisector.hypot();
    let mut along_bisector = f64::INFINITY;
    for c1 in &taylor.c1 {
        along_bisector = along_bisector.min(c1.dot(bisector));
    }
    let from_bisector = if bisector_len > 0.0 {
        along_bisector / bisector_len
    } else {
        0.0
    };
    let from_square = min_coeff(&taylor.c1_dot_c1).max(0.0).sqrt();
    from_bisector.max(from_square)
}

/// Bound the distance between the approximation and the offset at equal
/// parameter, using only convex hulls.
///
/// `D(t)` lies in the hull of its control points. The direction of `C'(t)`
/// lies in the cone of the control points of the hodograph, so `n(t)` lies
/// on an arc of the unit circle between the normals to those. The distance
/// between a point of the hull and a point of the arc is largest at a
/// vertex of the one and an end of the other.
///
/// This is first order in the turning of the piece, so it is only useful
/// for a piece that is nearly straight relative to the tolerance. It uses
/// directions only, so it works when the derivative vanishes at an end,
/// given robust end tangents.
///
/// Returns `None` if the cone is a quarter turn or more.
fn hull_bound(seg: &CubicBez, rec: &OffsetRec, delta: &[Vec2; 4], d: f64) -> Option<f64> {
    const TAN_EPSILON: f64 = 1e-12;
    // The directions of the hodograph control points. The end ones are the
    // end tangents; the middle one is used if it is nonzero.
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
        let offset_point = d * direction.turn_90();
        for vertex in delta {
            // The argument needs each vertex within a quarter turn of each
            // normal.
            if !(vertex.dot(offset_point) > 0.0) {
                return None;
            }
            worst = worst.max((*vertex - offset_point).hypot2());
        }
    }
    Some(worst.sqrt())
}

/// Bound the distance between any approximation and the offset at equal
/// parameter, by the triangle inequality alone.
///
/// `D(t)` lies in the hull of its control points, and `d n(t)` has length
/// `|d|`, so `|D(t) - d n(t)|` is at most the largest control point norm
/// plus `|d|`. This holds for any piece and any fit, across cusps and
/// whatever the turning, but it is never below `2 |d|`, so it is only
/// useful where the stroke is thin relative to the tolerance. There it
/// accepts at the root pieces that turn too far for [`hull_bound`].
#[inline(always)]
fn hairline_bound(delta: &[Vec2; 4], d: f64) -> f64 {
    max_norm(delta) + d.abs()
}

/// Bound the error of arc drawing on a piece that is small.
///
/// Suppose the direction of the source tangent turns one way only over the
/// piece. Then the source normal sweeps the arc of the unit circle from one
/// end normal to the other, once. Arc drawing makes `D / d` the standard
/// cubic approximation of that arc, whose direction sweeps the same arc
/// once and whose length is within `epsilon` of one. Pair each point of the
/// approximation with the point of the offset whose normal has the same
/// direction. The two differ by the distance between two points of the
/// source piece, plus `|d| epsilon`.
///
/// So the Fréchet distance is at most the diameter of the source piece plus
/// `|d| epsilon`. Parametrization plays no part in this, which is the point:
/// near a cusp of the source the tangent turns very unevenly, and every
/// bound that compares the curves at related parameters fails.
///
/// Returns `None` if the tangent might not turn one way only, if it turns
/// through more than about a right angle, or if the approximation is not
/// plain arc drawing.
#[inline(always)]
fn small_piece_bound(seg: &CubicBez, rec: &OffsetRec, arc: &Fit, d: f64) -> Option<f64> {
    if arc.clamped {
        return None;
    }
    // The tangent turns one way only if the control points of the
    // hodograph are in angular order. The end ones may vanish, so use the
    // end tangents for their directions.
    let middle = seg.p2 - seg.p1;
    let turn_0 = rec.utan0.cross(middle);
    let turn_1 = middle.cross(rec.utan1);
    let turn = rec.utan0.cross(rec.utan1);
    let all_one_way = (turn_0 >= 0.0 && turn_1 >= 0.0 && turn >= 0.0)
        || (turn_0 <= 0.0 && turn_1 <= 0.0 && turn <= 0.0);
    // Keep to turns for which the angular order is unambiguous.
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
    // angle theta is 4/27 sin^6(theta / 4) / cos^2(theta / 4).
    let cos_half = (0.5 * (1.0 + cos_th)).sqrt();
    let cos_quarter_squared = 0.5 * (1.0 + cos_half);
    let sin_quarter_squared = 0.5 * (1.0 - cos_half);
    let epsilon = (4.0 / 27.0) * sin_quarter_squared.powi(3) / cos_quarter_squared;
    Some(diameter_squared.sqrt() + d.abs() * epsilon)
}

// ---------------------------------------------------------------------------
// The sampled estimate

/// The samples of the osculating parabola estimate, at the midpoints of 8
/// equal intervals, in the order [`osculating_decision`] takes them: where
/// the error of the one-point fit tends to peak, near 0.28 and 0.72, first.
///
/// With fewer samples, its misses went up sharply in tests.
const OSCULATING_SAMPLES: [f64; 8] = [
    0.3125, 0.6875, 0.1875, 0.8125, 0.4375, 0.5625, 0.0625, 0.9375,
];

/// The factor applied to the osculating parabola estimate before comparing
/// it with the tolerance.
///
/// Measured against the Fréchet error on fitted pieces, error over estimate
/// had median 1.05, p90 1.11 and p99 1.29. With the guards below, this
/// left no more misses than the default mode had without it.
const OSCULATING_SAFETY: f64 = 1.2;

/// The largest slip, the tangential part of `D / d`, for which the
/// osculating parabola estimate is trusted on a two-point fit.
///
/// The two-point fit does not control the slip, and with a large one the
/// estimate fell short by up to 4 times. The other fits need no such limit.
const OSCULATING_MAX_SLIP: f64 = 0.5;

/// The largest ratio between the cusp values at the ends of a piece for
/// which the osculating parabola estimate is trusted.
///
/// Inside the piece only the sign is checked, at the samples. A ratio there
/// sent most pieces ending where the derivative vanishes, whose cusp value
/// at that end is not its limit, to the slower path.
///
/// Where the curvature changes quickly, as at the tip of a sharp turn, the
/// error gathers near one end, between the samples. Up to a ratio of about 4
/// the estimate is as reliable as on ordinary pieces (p99 1.25 times the
/// error); 3 leaves some margin.
const OSCULATING_MAX_CUSP_RATIO: f64 = 3.0;

/// The factor applied to the sampled estimate before comparing it with the
/// tolerance.
///
/// The estimate is unbiased on short pieces, but it is used on the long and
/// sharply turning ones where the certificate fails, and there it can fall
/// short. In tests it was rarely below 0.8 of the true error.
const ESTIMATE_SAFETY: f64 = 1.25;

/// The largest ratio between the cusp values at the ends of a piece for
/// which the sampled estimate is trusted.
///
/// Above it the curvature is concentrated at one end. A piece next to a
/// regularized near cusp with a ratio of 160 was accepted at 0.31 of its
/// true error. The osculating estimate has the much tighter
/// [`OSCULATING_MAX_CUSP_RATIO`].
const ESTIMATE_MAX_CUSP_RATIO: f64 = 64.0;

/// The number of samples the estimate takes.
const N_SAMPLES: usize = 7;

/// The order in which the samples are taken: the quarter points, the
/// midpoint, then the rest.
const SAMPLE_ORDER: [usize; N_SAMPLES] = [1, 5, 3, 0, 6, 2, 4];

/// The number of interior points of the grid on which the estimate
/// searches for the extrema of its model.
const N_GRID: usize = 15;

/// Maps samples of `V` to values of the model on the grid.
///
/// The samples are at `t = i / (N_SAMPLES + 1)` and the grid points at
/// `t = j / (N_GRID + 1)`. The error has double zeros at both ends, so it
/// is modelled as `u^2 X(t)`, with `u = t (1 - t)` and `X` the polynomial
/// through the sampled values of `V / u^2`.
const SAMPLE_TABLE: [[f64; N_SAMPLES]; N_GRID] = sample_table();

const fn sample_table() -> [[f64; N_SAMPLES]; N_GRID] {
    let mut table = [[0.0; N_SAMPLES]; N_GRID];
    let mut j = 0;
    while j < N_GRID {
        let t = (j + 1) as f64 / (N_GRID + 1) as f64;
        let u = t * (1.0 - t);
        let mut i = 0;
        while i < N_SAMPLES {
            let node_i = (i + 1) as f64 / (N_SAMPLES + 1) as f64;
            // The Lagrange basis polynomial for sample i, at t.
            let mut basis = 1.0;
            let mut k = 0;
            while k < N_SAMPLES {
                if k != i {
                    let node_k = (k + 1) as f64 / (N_SAMPLES + 1) as f64;
                    basis *= (t - node_k) / (node_i - node_k);
                }
                k += 1;
            }
            let u_i = node_i * (1.0 - node_i);
            table[j][i] = basis * (u * u) / (u_i * u_i);
            i += 1;
        }
        j += 1;
    }
    table
}

/// Estimate the largest value of `|rho - |d||` over a piece.
///
/// This is not a bound. At each sample it takes one Newton step on the
/// foot point, corrected by the cubic Taylor term and the leading part of a
/// second step. It then fits the model described at [`SAMPLE_TABLE`] and
/// takes its extrema.
///
/// As soon as one sample shows that the estimate exceeds `limit`, this
/// returns what that sample gives, which is over `limit`. Pass infinity to
/// get the full estimate.
///
/// Returns `None` if the foot point is ill defined at a sample, as it is
/// at a cusp of the offset.
fn sampled_estimate(taylor: &Taylor, rec: &OffsetRec, d: f64, limit: f64) -> Option<f64> {
    // The sign `beta` should have: negative past a cusp.
    let sign = if rec.cusp0 > 0.0 { 1.0 } else { -1.0 };
    // The values of V beyond which the distance is over `limit`.
    let d_abs = d.abs();
    let v_too_far = limit * (2.0 * d_abs + limit);
    let v_too_near = if limit < d_abs {
        limit * (limit - 2.0 * d_abs)
    } else {
        f64::NEG_INFINITY
    };
    let mut v = [0.0; N_SAMPLES];
    // Take the samples where the error tends to be largest first, so that
    // a piece that is clearly over the limit is rejected sooner.
    for i in SAMPLE_ORDER {
        let t = (i + 1) as f64 / (N_SAMPLES + 1) as f64;
        let mt = 1.0 - t;
        // Evaluate D, C' and C'' at t.
        let delta = taylor.delta[0] * (mt * mt * mt)
            + taylor.delta[1] * (3.0 * mt * mt * t)
            + taylor.delta[2] * (3.0 * mt * t * t)
            + taylor.delta[3] * (t * t * t);
        let c1 = taylor.c1[0] * (mt * mt) + taylor.c1[1] * (2.0 * mt * t) + taylor.c1[2] * (t * t);
        let c2 = taylor.c2[0] * mt + taylor.c2[1] * t;

        // The Taylor coefficients of F about s = t.
        let p = delta.hypot2() - d * d;
        let q = delta.dot(c1);
        let beta = 2.0 * c1.hypot2() - 2.0 * delta.dot(c2);
        if !(sign * beta > 0.0) {
            return None;
        }
        // The first Newton step, and F there to third order.
        let sigma = 2.0 * q / beta;
        let f_sss = 6.0 * c1.dot(c2) - 2.0 * delta.dot(taylor.c3);
        let f = p - q * sigma + f_sss * sigma.powi(3) / 6.0;
        // F_s and F_ss there to leading order, and one more step.
        let f_s = 0.5 * f_sss * sigma * sigma;
        let f_ss = beta + f_sss * sigma;
        if !(sign * f_ss > 0.0) {
            return None;
        }
        let value = f - f_s * f_s / (2.0 * f_ss);
        // The grid below contains the sample points, and the model takes
        // the sampled values there exactly, so the estimate is at least
        // what any one sample gives. If that is over `limit`, stop.
        if value > v_too_far || value < v_too_near {
            return distance_bound(value.max(0.0), (-value).max(0.0), d);
        }
        v[i] = value;
    }

    // Evaluate the model on the grid. It is zero at both ends.
    let mut y = [0.0; N_GRID + 2];
    for j in 0..N_GRID {
        let mut sum = 0.0;
        for i in 0..N_SAMPLES {
            sum += SAMPLE_TABLE[j][i] * v[i];
        }
        y[j + 1] = sum;
    }
    // Find the largest and smallest values, refining each local extremum
    // with the parabola through it and its two neighbors.
    let mut v_max = 0.0_f64;
    let mut v_min = 0.0_f64;
    for j in 1..=N_GRID {
        let prev = y[j - 1];
        let here = y[j];
        let next = y[j + 1];
        let second_diff = prev - 2.0 * here + next;
        if here >= prev && here >= next && here > 0.0 {
            let refined = if second_diff < 0.0 {
                here - 0.125 * (prev - next) * (prev - next) / second_diff
            } else {
                here
            };
            v_max = v_max.max(refined);
        }
        if here <= prev && here <= next && here < 0.0 {
            let refined = if second_diff > 0.0 {
                here - 0.125 * (prev - next) * (prev - next) / second_diff
            } else {
                here
            };
            v_min = v_min.min(refined);
        }
    }
    distance_bound(v_max, -v_min, d)
}

/// Decide a piece by the osculating parabola estimate.
///
/// This is the estimate of `tests::osculating_estimate` at
/// [`OSCULATING_SAMPLES`], rearranged to
/// need one square root per sample and no division, and to stop at the
/// first sample that decides. With `A = c'^ . D`, `B = c' . D`,
/// `K = c'' x c'`, `s = |c'|` and `E = s^3 + K d` (`s^3` times the cusp
/// value), the error in units of `d` is
///
/// ```text
/// A / (s d) - 1 + K B^2 / (2 d s^2 E)
/// ```
///
/// and it is within `limit` when `|A s E - d s^2 E + K B^2 / 2|` is within
/// `limit |d s^2 E|`. The slip is within `max_slip` when
/// `B^2 <= max_slip^2 d^2 s^2`, and the cusp value is positive when `E` is.
///
/// The first sample that settles anything decides: over the limit, too much
/// slip, or a cusp value that is not positive. The caller has checked that
/// it is positive at both ends, so in the last case there are at least two
/// cusps in the piece, which the signs at the ends do not show.
fn osculating_decision(
    seg: &CubicBez,
    delta: &[Vec2; 4],
    d: f64,
    limit: f64,
    max_slip: f64,
) -> Osculating {
    let c1 = [
        3.0 * (seg.p1 - seg.p0),
        3.0 * (seg.p2 - seg.p1),
        3.0 * (seg.p3 - seg.p2),
    ];
    let c2 = [2.0 * (c1[1] - c1[0]), 2.0 * (c1[2] - c1[1])];
    let slip_scale = max_slip * max_slip * d * d;
    for t in OSCULATING_SAMPLES {
        let mt = 1.0 - t;
        let d_t = delta[0] * (mt * mt * mt)
            + delta[1] * (3.0 * mt * mt * t)
            + delta[2] * (3.0 * mt * t * t)
            + delta[3] * (t * t * t);
        let q = c1[0] * (mt * mt) + c1[1] * (2.0 * mt * t) + c1[2] * (t * t);
        let qq = c2[0] * mt + c2[1] * t;
        let s2 = q.hypot2();
        let b = q.dot(d_t);
        if b * b > slip_scale * s2 {
            return Osculating::Untrusted;
        }
        let s = s2.sqrt();
        let a = q.turn_90().dot(d_t);
        let k = qq.cross(q);
        let s3 = s2 * s;
        let e = s3 + k * d;
        // The cusp value here is `e / s3`, and `s3` is not negative.
        if !(e > 0.0) {
            return if e <= 0.0 {
                Osculating::Cusp(t)
            } else {
                Osculating::Untrusted
            };
        }
        let numerator = a * s * e - d * s2 * e + 0.5 * k * b * b;
        let denominator = d * s2 * e;
        // Written so that a NaN rejects.
        if !(numerator.abs() <= limit * denominator.abs()) {
            return Osculating::Decided(false);
        }
    }
    Osculating::Decided(true)
}

/// The outcome of [`osculating_decision`].
enum Osculating {
    /// Whether every sample is within the limit.
    Decided(bool),
    /// The slip is too large, or a value is not finite: the estimate is not
    /// to be trusted.
    Untrusted,
    /// The cusp value is not positive at this sample.
    Cusp(f64),
}

/// Estimate the largest distance between the approximation and the offset
/// at equal parameter.
///
/// This samples the quantity that [`equal_parameter_bound`] bounds. It is
/// not a bound, and it overestimates the error by the tangential mismatch,
/// but it is defined wherever the derivative of the source is nonzero.
fn sampled_equal_parameter_distance(taylor: &Taylor, d: f64) -> f64 {
    let mut worst = 0.0_f64;
    for i in 0..N_SAMPLES {
        let t = (i + 1) as f64 / (N_SAMPLES + 1) as f64;
        let mt = 1.0 - t;
        let delta = taylor.delta[0] * (mt * mt * mt)
            + taylor.delta[1] * (3.0 * mt * mt * t)
            + taylor.delta[2] * (3.0 * mt * t * t)
            + taylor.delta[3] * (t * t * t);
        let c1 = taylor.c1[0] * (mt * mt) + taylor.c1[1] * (2.0 * mt * t) + taylor.c1[2] * (t * t);
        let speed = c1.hypot();
        if !(speed > 0.0) {
            return f64::INFINITY;
        }
        let offset_vector = c1.turn_90() * (d / speed);
        worst = worst.max((delta - offset_vector).hypot2());
    }
    worst.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PathEl, Point};

    /// Estimate the error by the osculating parabola, at `n` samples.
    ///
    /// From the `failed_k4` branch of fast-stroke (`est_arc_error_dot`). At each
    /// sample, with `W = D / d` split into `dot` and `cross` along the normal and
    /// tangent of the source, the error in units of `d` is
    /// `(dot - 1) + kappa_off d cross^2 / 2`: the radial error, and how far the
    /// offset, of curvature `kappa_off = kappa / (1 + kappa d)`, bends away from
    /// its tangent over the slip `d cross`. This is the leading term of the
    /// Newton value `H / d^2`. It is not a bound: there is no remainder, and
    /// nothing between the samples, which are at the midpoints of `n` equal
    /// intervals.
    ///
    /// Returns the estimate, and the largest `|cross|`, the slip in units of
    /// `d`, beyond which the estimate is not to be trusted. Only the tests use
    /// this form; [`osculating_decision`] is the one in use.
    fn osculating_estimate(seg: &CubicBez, delta: &[Vec2; 4], d: f64, n: usize) -> (f64, f64) {
        let c1 = [
            3.0 * (seg.p1 - seg.p0),
            3.0 * (seg.p2 - seg.p1),
            3.0 * (seg.p3 - seg.p2),
        ];
        let c2 = [2.0 * (c1[1] - c1[0]), 2.0 * (c1[2] - c1[1])];
        let inv_d = 1.0 / d;
        let mut worst = 0.0_f64;
        let mut slip = 0.0_f64;
        for i in 0..n {
            let t = (i as f64 + 0.5) / n as f64;
            let mt = 1.0 - t;
            let d_t = delta[0] * (mt * mt * mt)
                + delta[1] * (3.0 * mt * mt * t)
                + delta[2] * (3.0 * mt * t * t)
                + delta[3] * (t * t * t);
            let q = c1[0] * (mt * mt) + c1[1] * (2.0 * mt * t) + c1[2] * (t * t);
            let qq = c2[0] * mt + c2[1] * t;
            let speed2 = q.hypot2();
            let speed = speed2.sqrt();
            let normal = q.turn_90() / speed;
            let w = d_t * inv_d;
            let dot = normal.dot(w);
            let cross = normal.cross(w);
            // kurbo's curvature, c'' x c' / |c'|^3, so that 1 + kappa d is the
            // cusp value.
            let kappa = qq.cross(q) / (speed2 * speed);
            let kappa_off = kappa / (1.0 + kappa * d);
            let error = (dot - 1.0) + 0.5 * kappa_off * d * cross * cross;
            worst = worst.max(error.abs());
            slip = slip.max(cross.abs());
        }
        (worst * d.abs(), slip)
    }

    thread_local! {
        /// The number of pieces emitted at the maximum depth without a bound.
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

    /// The setup for one piece, as `offset_rec` would see it.
    fn setup(c: CubicBez, d: f64) -> (CubicOffset, OffsetRec) {
        let mut co = CubicOffset::new(c, d, 0.0, true);
        let (tan0, tan1) = PathSeg::Cubic(c).tangents();
        let rec = OffsetRec {
            t0: 0.0,
            t1: 1.0,
            utan0: tan0.normalize(),
            utan1: tan1.normalize(),
            cusp0: co.end_cusp(false),
            cusp1: co.end_cusp(true),
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

    /// Measure the error of an approximation to the offset of `c`.
    ///
    /// For each of many points on the approximation, find the stationary
    /// point of its distance to `c` near the same parameter, and compare
    /// that distance with `|d|`. This is the quantity the certified bounds
    /// bound.
    fn measured_error(c: &CubicBez, approx: &CubicBez, d: f64) -> f64 {
        let c1 = c.deriv();
        let c2 = c1.deriv();
        let mut worst = 0.0_f64;
        const N: usize = 400;
        for i in 0..=N {
            let t = i as f64 / N as f64;
            let p = approx.eval(t);
            let mut s = t;
            for _ in 0..40 {
                let delta = p - c.eval(s);
                let tan = c1.eval(s).to_vec2();
                let f_ss = tan.hypot2() - delta.dot(c2.eval(s).to_vec2());
                let step = delta.dot(tan) / f_ss;
                s += step;
                if step.abs() < 1e-15 {
                    break;
                }
            }
            worst = worst.max((p.distance(c.eval(s)) - d.abs()).abs());
        }
        worst
    }

    /// The error of one piece: the Fréchet distance from the oracle where it
    /// can measure it, and otherwise `fallback`, a one-sided distance.
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

    /// The error of a whole result from `offset_cubic` or
    /// `offset_cubic_strict`, against the offset of the regularized curve.
    ///
    /// With std, this is the Fréchet oracle, taking the lower bound for any
    /// piece it cannot measure. Without, the sampled one-sided distance.
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

    /// Slack for rounding in the bounds and in the measurement.
    fn slack(d: f64) -> f64 {
        1e-9 * d.abs() + 1e-11
    }

    /// Visit pieces of random cubics, at several offsets.
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
    fn one_point_bound_is_strict() {
        let mut n_ordinary = 0;
        let mut n_reversed = 0;
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
            let (cusp_min, cusp_max) = cusp_range(&co);
            // Stay clear of cusps, where the measurement is unreliable.
            if cusp_min * cusp_max <= 0.0 || cusp_min.abs().min(cusp_max.abs()) < 0.2 {
                return;
            }
            let Some((a, b)) = one_point(&piece, &rec) else {
                return;
            };
            let fit = co.apply(&piece, &rec, a, b);
            if fit.clamped {
                return;
            }
            let taylor = Taylor::new(&piece, &fit.delta, d);
            let Some(certificate) = Certificate::new(&taylor) else {
                return;
            };
            let Some(bound) = one_point_bound(&taylor, &certificate, d) else {
                return;
            };
            let error = piece_frechet(&piece, &fit.approx, d, || {
                measured_error(&piece, &fit.approx, d)
            });
            assert!(
                bound >= error - slack(d),
                "bound {bound:e} below error {error:e} for {piece:?}, d = {d}"
            );
            if certificate.sign > 0.0 {
                n_ordinary += 1;
            } else {
                n_reversed += 1;
            }
        });
        // Make sure both kinds of piece were exercised.
        assert!(n_ordinary > 1000, "{n_ordinary}");
        assert!(n_reversed > 100, "{n_reversed}");
    }

    #[test]
    fn crude_certified_bound_is_strict() {
        let mut n_checked = 0;
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
            let (cusp_min, cusp_max) = cusp_range(&co);
            if cusp_min * cusp_max <= 0.0 || cusp_min.abs().min(cusp_max.abs()) < 0.2 {
                return;
            }
            let (a, b) = draw_arc(rec.utan0, rec.utan1);
            let fit = co.apply(&piece, &rec, a, b);
            let taylor = Taylor::new(&piece, &fit.delta, d);
            let Some(certificate) = Certificate::new(&taylor) else {
                return;
            };
            let Some(bound) = crude_certified_bound(&taylor, &certificate, d) else {
                return;
            };
            let error = piece_frechet(&piece, &fit.approx, d, || {
                measured_error(&piece, &fit.approx, d)
            });
            assert!(
                bound >= error - slack(d),
                "bound {bound:e} below error {error:e} for {piece:?}, d = {d}"
            );
            n_checked += 1;
        });
        assert!(n_checked > 1000, "{n_checked}");
    }

    #[test]
    fn uncertified_bounds_are_strict() {
        let mut n_equal_parameter = 0;
        let mut n_hull = 0;
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
            // These bounds hold whatever the cusp values, and for any fit.
            let (arc_a, arc_b) = draw_arc(rec.utan0, rec.utan1);
            let mut fits = vec![co.apply(&piece, &rec, arc_a, arc_b)];
            if let Some((a, b)) = one_point(&piece, &rec) {
                fits.push(co.apply(&piece, &rec, a, b));
            }
            for fit in &fits {
                let distance = equal_parameter_distance(&piece, &fit.approx, d);
                let taylor = Taylor::new(&piece, &fit.delta, d);
                if let Some(bound) = equal_parameter_bound(&taylor, &rec, d) {
                    assert!(
                        bound >= distance - slack(d),
                        "bound {bound:e} below distance {distance:e} for {piece:?}, d = {d}"
                    );
                    n_equal_parameter += 1;
                }
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
        assert!(n_equal_parameter > 1000, "{n_equal_parameter}");
        assert!(n_hull > 1000, "{n_hull}");
    }

    #[test]
    fn small_piece_bound_is_strict() {
        // Pieces around the slowest point of curves with a near cusp.
        let mut rng = Rng(4242);
        let mut n_checked = 0;
        for _ in 0..200 {
            // A hairpin: the middle control points lie beyond the ends.
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
                    let Some(bound) = small_piece_bound(&piece, &rec, &fit, d) else {
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

    /// Segments, pieces emitted at the maximum depth, and the worst error
    /// (a lower bound: the one-sided distance from the offset to the
    /// result), for families with sharp turns: random cubics, hairpins, and
    /// cusps made inexact by a given gap between the middle control points.
    /// These are what the small-piece bound and the subdivision margin are
    /// for, and what the random families and the tiger hardly contain. Takes
    /// a few minutes. Run with `--ignored --nocapture`.
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
            // The middle control points crossed over the chord give an
            // exact cusp at t = 1/2; moving them apart makes it a near one.
            let cusps = (0..200)
                .map(|_| {
                    let p0 = Point::new(rng.next() * 100.0, rng.next() * 100.0);
                    let p3 = Point::new(rng.next() * 100.0, rng.next() * 100.0);
                    let m = p0.midpoint(p3)
                        + Vec2::new(rng.next() * 60.0 - 30.0, rng.next() * 60.0 - 30.0);
                    let half_chord = (p3 - p0) * 0.5;
                    let dir = Vec2::new(rng.next() - 0.5, rng.next() - 0.5).normalize();
                    CubicBez::new(
                        p0,
                        m + half_chord + gap * dir,
                        m - half_chord - gap * dir,
                        p3,
                    )
                })
                .collect();
            families.push((std::format!("cusp, gap {gap:e}"), cusps));
        }
        std::println!(
            "family, tolerance: segments, pieces at the maximum depth, and the worst distance \
             from the offset to the result over the tolerance; default | strict"
        );
        for (name, curves) in &families {
            for tolerance in [0.25, 1e-2, 1e-4, 1e-6] {
                let mut cells = Vec::new();
                for strict in [false, true] {
                    let (mut n_segments, mut n_out_of_depth) = (0, 0);
                    let mut worst = 0.0_f64;
                    for c in curves {
                        for d in [1.0, -1.0, 5.0, -5.0] {
                            OUT_OF_DEPTH.with(|count| count.set(0));
                            let mut result = BezPath::new();
                            if strict {
                                offset_cubic_strict(*c, d, tolerance, &mut result);
                            } else {
                                offset_cubic(*c, d, tolerance, &mut result);
                            }
                            n_out_of_depth += OUT_OF_DEPTH.with(|count| count.get());
                            n_segments += result.segments().count();
                            // Against the offset of the regularized input,
                            // which is what is approximated: near a cusp,
                            // regularizing alone moves the offset by up to
                            // `2 |d|`. The oracle's range matching is
                            // unreliable here, so use the one-sided measure.
                            let regularized = c.regularize_cusp(0.25 * tolerance);
                            let (error, _) = offset_to_result(regularized, d, &result, 4000);
                            worst = worst.max(error / tolerance);
                        }
                    }
                    cells.push(std::format!(
                        "{n_segments:>7} {n_out_of_depth:>6} {worst:>9.2}"
                    ));
                }
                std::println!("{name:>16}, {tolerance:e}: {} | {}", cells[0], cells[1]);
            }
        }
    }

    #[test]
    fn draw_arc_matches_offset() {
        // The vector form here should agree with the trigonometric form in
        // `crate::offset`.
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
    /// The offset is sampled densely, and the nearest sample to each point
    /// is refined by a ternary search between its neighbors.
    fn result_error(c: &CubicBez, d: f64, result: &BezPath) -> f64 {
        const N: usize = 2000;
        // Keep just inside the ends, where the derivative may vanish and
        // the normal is defined only as a limit.
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
                // not only the smallest. Where the offset comes back near
                // itself, the nearest sample can be on the wrong strand.
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

    #[test]
    fn strict_result_is_within_tolerance() {
        let mut rng = Rng(777);
        let mut n_checked = 0;
        for _ in 0..60 {
            let c = rng.cubic();
            for d in [1.0, -1.0, 4.0, -4.0] {
                // Only check offsets that stay well clear of cusps.
                let (co, _) = setup(c, d);
                let (cusp_min, _) = cusp_range(&co);
                if cusp_min < 0.5 {
                    continue;
                }
                for tolerance in [1e-1, 1e-3] {
                    let mut result = BezPath::new();
                    OUT_OF_DEPTH.with(|count| count.set(0));
                    offset_cubic_strict(c, d, tolerance, &mut result);
                    let out_of_depth = OUT_OF_DEPTH.with(|count| count.get());
                    // The result approximates the offset of the regularized
                    // curve, so that is what to measure against.
                    let error = whole_result_error(c, d, tolerance, &result);
                    if out_of_depth > 0 {
                        // No bound accepted some piece, so nothing is
                        // promised.
                        continue;
                    }
                    assert!(
                        error <= tolerance * (1.0 + 1e-6) + 1e-9,
                        "error {error:e} over tolerance {tolerance:e} for {c:?}, d = {d}"
                    );
                    n_checked += 1;
                }
            }
        }
        assert!(n_checked > 50, "{n_checked}");
    }

    #[cfg(feature = "std")]
    #[test]
    fn default_result_with_hidden_cusp_pair() {
        // The cusp value is close to 1 at both ends of this piece, of a
        // random cubic, but runs from about -500 to 1500 inside it. With `d`
        // of -10, the offset has a pair of cusps that the signs at the ends
        // do not show. The osculating estimate, with its guards looking only
        // at the ends, accepted a piece 9.5 times over the tolerance.
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
        // near double roots: where the curvature has an extremum, set `d`
        // so that the cusp value just dips below zero there. Compare
        // `find_cusp` with ITP on the same function and bracket.
        let mut rows: Vec<(&str, usize, usize, f64, f64)> = Vec::new();
        let mut run = |label: &'static str, c: CubicBez, d: f64, a: f64, b: f64| {
            let co = CubicOffset::new(c, d, 0.0, true);
            let f = |t: f64| co.scaled_cusp_and_deriv(t).0;
            let (ya, yb) = (f(a), f(b));
            if !(ya * yb < 0.0) {
                return;
            }
            CUSP_ITERATIONS.with(|count| count.set(0));
            let t = co.find_cusp(a, b, yb.signum());
            let newton = CUSP_ITERATIONS.with(|count| count.get());
            // ITP wants the function negative at `a`.
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
            // How far each result is from a sign change: the smallest
            // interval about it, of the form 2^-k, that brackets one.
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
            // Local maxima of |kappa|, from a grid, refined.
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
    /// to `result`, and the parameter where it is.
    ///
    /// This is a lower bound on the Fréchet distance that, unlike the oracle,
    /// does not need to match pieces to stretches of the source, which is
    /// ambiguous next to a cusp. Each candidate is refined to the exact
    /// nearest point on a segment of the result.
    fn offset_to_result(c: CubicBez, d: f64, result: &BezPath, n_off: usize) -> (f64, f64) {
        use crate::ParamCurveNearest;
        let q = c.deriv();
        let (limit0, limit1) = PathSeg::Cubic(c).tangents();
        let segs: Vec<PathSeg> = result.segments().collect();
        let res: Vec<(Point, usize)> = segs
            .iter()
            .enumerate()
            .flat_map(|(k, seg)| (0..=400).map(move |j| (seg.eval(j as f64 / 400.0), k)))
            .collect();
        let cell = 0.05;
        let mut grid = std::collections::HashMap::<(i64, i64), Vec<(Point, usize)>>::new();
        for (p, k) in &res {
            grid.entry(((p.x / cell).floor() as i64, (p.y / cell).floor() as i64))
                .or_default()
                .push((*p, *k));
        }
        let mut worst = (0.0_f64, 0.0);
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
            let (gx, gy) = ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
            // Candidate segments: those with a sample within the first rings
            // that hold anything, plus one more ring.
            let mut cands: Vec<usize> = Vec::new();
            let mut found_ring = None;
            for r in 0_i64..4000 {
                for dx in -r..=r {
                    for dy in -r..=r {
                        if dx.abs() != r && dy.abs() != r {
                            continue;
                        }
                        if let Some(v) = grid.get(&(gx + dx, gy + dy)) {
                            cands.extend(v.iter().map(|x| x.1));
                        }
                    }
                }
                if !cands.is_empty() && found_ring.is_none() {
                    found_ring = Some(r);
                }
                if let Some(f) = found_ring {
                    if r >= f + 2 {
                        break;
                    }
                }
            }
            cands.sort_unstable();
            cands.dedup();
            let mut best = f64::INFINITY;
            for k in cands {
                let near = k.saturating_sub(1)..=(k + 1).min(segs.len() - 1);
                for seg in &segs[near] {
                    let PathSeg::Cubic(cb) = *seg else {
                        continue;
                    };
                    best = best.min(cb.nearest(p, 1e-12).distance_sq.sqrt());
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
        // Random cubics with a control point on an end point, at widths up
        // to a good fraction of their size. On the inner side the offset has
        // a cusp next to that end. Errors by the oracle, and by the one-sided
        // distance, as multiples of the tolerance.
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
        for strict in [false, true] {
            for d in [0.3, 3.0, 30.0] {
                for tolerance in [0.25, 0.01, 1e-3] {
                    let mut segments = 0;
                    let (mut over, mut worst) = ([0; 2], [0.0_f64; 2]);
                    for c in &curves {
                        for d in [d, -d] {
                            let mut result = BezPath::new();
                            if strict {
                                offset_cubic_strict(*c, d, tolerance, &mut result);
                            } else {
                                offset_cubic(*c, d, tolerance, &mut result);
                            }
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
                        "strict={strict:5} d={d:4} tol={tolerance:6}: segments {segments:5}; oracle over {:3} worst {:9.3}; one-sided over {:3} worst {:7.3}",
                        over[0],
                        worst[0],
                        over[1],
                        worst[1]
                    );
                }
            }
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn default_result_with_vanishing_end_derivative() {
        // Where the derivative vanishes at an end, the cusp value there was
        // taken as 1, and the cusp next to that end on the inside was never
        // found. The default mode then fitted across it, and the osculating
        // estimate, whose samples miss the end, accepted fits several times
        // over the tolerance (4.5 and 4.2 times for these). Measured by the
        // one-sided distance, since the oracle cannot follow a piece that
        // turns back next to a cusp.
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
        // Repros from open kurbo issues against `offset`. The first (#617)
        // has a vanishing end derivative, and `offset` gives non-finite
        // points on its inner side; #344 is an older report of the same
        // kind. The last (#615) has an exact cusp at its midpoint, which
        // `detect_cusp` missed, so it was not regularized; the midpoint
        // split then took the rounding noise there for the tangent, and the
        // piece after the cusp started on the wrong side (2.9 times the
        // tolerance at 1e-4, in both modes).
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
                    let mut default = BezPath::new();
                    offset_cubic(c, d, tolerance, &mut default);
                    let mut strict = BezPath::new();
                    offset_cubic_strict(c, d, tolerance, &mut strict);
                    for (result, mode) in [(default, "default"), (strict, "strict")] {
                        assert!(
                            result.elements().iter().all(|el| el
                                .end_point()
                                .is_none_or(|p| p.is_finite())
                                && match *el {
                                    PathEl::CurveTo(p1, p2, _) => p1.is_finite() && p2.is_finite(),
                                    _ => true,
                                }),
                            "{mode}: non-finite point for {c:?}, d = {d}"
                        );
                        let (error, s) = offset_to_result(c, d, &result, 20_000);
                        assert!(
                            error <= tolerance,
                            "{mode}: error {error} at s = {s}, tolerance {tolerance}, \
                             for {c:?}, d = {d}"
                        );
                    }
                }
            }
        }
    }

    /// The osculating estimate at the given samples.
    fn osculating_at(seg: &CubicBez, delta: &[Vec2; 4], d: f64, samples: &[f64]) -> f64 {
        let c1 = seg.deriv();
        let c2 = c1.deriv();
        let mut worst = 0.0_f64;
        for &t in samples {
            let mt = 1.0 - t;
            let d_t = delta[0] * (mt * mt * mt)
                + delta[1] * (3.0 * mt * mt * t)
                + delta[2] * (3.0 * mt * t * t)
                + delta[3] * (t * t * t);
            let q = c1.eval(t).to_vec2();
            let qq = c2.eval(t).to_vec2();
            let s = q.hypot();
            let n = q.turn_90() / s;
            let w = d_t / d;
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
        // Pieces that end exactly at a cusp of the offset, fitted as
        // `approximate` fits them, against the Fréchet error: does the
        // osculating estimate hold there, and do samples near the ends help?
        use crate::offset_oracle::piece_error;
        let base: Vec<f64> = OSCULATING_SAMPLES.to_vec();
        let mut dense = base.clone();
        dense.extend([0.0078125, 0.015625, 0.03125, 0.96875, 0.984375, 0.9921875]);
        let mut rows: Vec<(f64, f64, f64)> = Vec::new();
        let mut rng = Rng(77);
        for _ in 0..400 {
            let c = rng.cubic();
            for d in [0.3, -0.3, 3.0, -3.0, 30.0, -30.0] {
                let co = CubicOffset::new(c, d, 0.0, false);
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
                            let pco = CubicOffset::new(piece, d, 0.0, false);
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
    fn strict_result_with_vanishing_end_derivative() {
        // A control point on an end point makes the derivative vanish
        // there, and then that end solves `tan x c'(t) = 0` for every `tan`.
        // Subdividing there used to leave pieces of zero length, whose arcs
        // fanned about the end point and missed the offset. This is common
        // in hand-drawn art; the Ghostscript tiger is full of it.
        let mut curves = vec![
            // From the tiger, where the error had reached 2.5 times the
            // tolerance.
            CubicBez::new(
                (169.56, 167.519),
                (169.56, 167.519),
                (170.188, 164.537),
                (169.874, 164.067),
            ),
            CubicBez::new(
                (121.692, 181.331),
                (121.692, 181.331),
                (117.768, 169.874),
                (116.042, 168.775),
            ),
        ];
        let mut rng = Rng(4242);
        for i in 0..20 {
            let c = rng.cubic();
            curves.push(if i % 2 == 0 {
                CubicBez::new(c.p0, c.p0, c.p2, c.p3)
            } else {
                CubicBez::new(c.p0, c.p1, c.p3, c.p3)
            });
        }
        let mut n_checked = 0;
        for c in curves {
            for d in [0.2, -0.2, 1.0, -1.0] {
                for tolerance in [1.0 / 32.0, 1.0 / 256.0] {
                    let mut result = BezPath::new();
                    OUT_OF_DEPTH.with(|count| count.set(0));
                    offset_cubic_strict(c, d, tolerance, &mut result);
                    if OUT_OF_DEPTH.with(|count| count.get()) > 0 {
                        continue;
                    }
                    let error = whole_result_error(c, d, tolerance, &result);
                    assert!(
                        error <= tolerance * (1.0 + 1e-6) + 1e-9,
                        "error {error:e} over tolerance {tolerance:e} for {c:?}, d = {d}"
                    );
                    n_checked += 1;
                }
            }
        }
        assert!(n_checked > 100, "{n_checked}");
    }

    /// The Bernstein coefficients of the two halves of a polynomial.
    fn halves<const N: usize>(c: &[f64; N]) -> ([f64; N], [f64; N]) {
        let mut left = [0.0; N];
        let mut right = [0.0; N];
        let mut work = *c;
        for i in 0..N {
            left[i] = work[0];
            right[N - 1 - i] = work[N - 1 - i];
            for j in 0..N - 1 - i {
                work[j] = 0.5 * (work[j] + work[j + 1]);
            }
        }
        (left, right)
    }

    /// The original `certify` from bezfit's `bound/cheap.rs`, ported as is.
    /// It does not handle negative beta.
    fn original_certify(
        taylor: &Taylor,
        beta: &[f64; 5],
        ac: &[f64; 5],
        cc: &[f64; 4],
        width: f64,
        q: f64,
    ) -> bool {
        let bmin = min_coeff(beta);
        if bmin.is_nan() || bmin <= 0.0 {
            return false;
        }
        let cc = max_abs_coeff(cc);
        let ac = min_coeff(ac);
        let c1 = max_norm(&taylor.c1);
        let c2 = max_norm(&taylor.c2);
        let c3 = taylor.c3.hypot();
        let dmax = max_norm(&taylor.delta);
        let a1 = max_norm(&taylor.a1);
        let mut mu = 0.125 * width;
        for _ in 0..4 {
            let c2m = c2 + c3 * mu;
            let c1m = c1 + c2m * mu;
            let ccm = cc + (c2m * c2m + c1m * c3) * mu;
            let k = 6.0 * ccm + 2.0 * (dmax + c1m * mu) * c3;
            let disc = bmin * bmin - 8.0 * k * q;
            if disc.is_nan() || disc <= 0.0 {
                return false;
            }
            let delta = 4.0 * q / (bmin + disc.sqrt());
            if delta > mu {
                mu = 1.5 * delta;
                continue;
            }
            return ac > a1 * c2m * delta;
        }
        false
    }

    /// Compare the certificate with the original from bezfit's
    /// `bound/cheap.rs`, on the same pieces. Run with `--ignored
    /// --nocapture`. See the notes for the results.
    #[test]
    #[ignore = "measurement, prints rates"]
    #[allow(clippy::print_stdout, reason = "this test exists to print rates")]
    fn certificate_rates() {
        // [ours, original whole, original split, beta positive], for
        // [one-point, two-point].
        let mut counts = [[0usize; 4]; 2];
        let mut totals = [0usize; 2];
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
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
            let taylor = Taylor::new(&piece, &fit.delta, d);
            let k = usize::from(has_inflection);
            totals[k] += 1;
            if Certificate::new(&taylor).is_some() {
                counts[k][0] += 1;
            }
            let kk = &taylor.c1_dot_c1;
            let cc = [
                2.0 * (kk[1] - kk[0]),
                2.0 * (kk[2] - kk[1]),
                2.0 * (kk[3] - kk[2]),
                2.0 * (kk[4] - kk[3]),
            ];
            let q = &taylor.q;
            let q_residual = q[0].abs().max(q[5].abs());
            if original_certify(
                &taylor,
                &taylor.beta,
                &taylor.a1_dot_c1,
                &cc,
                1.0,
                q_bound(q),
            ) {
                counts[k][1] += 1;
            }
            // Split into halves, each with its own bound on Q.
            let sg = [5.0 * q[1], 10.0 / 3.0 * q[2], 10.0 / 3.0 * q[3], 5.0 * q[4]];
            let (bl, br) = halves(&taylor.beta);
            let (al, ar) = halves(&taylor.a1_dot_c1);
            let (cl, cr) = halves(&cc);
            let (ql, qr) = if has_inflection {
                let (sl, sr) = halves(&sg);
                (
                    0.25 * max_abs_coeff(&sl) + q_residual,
                    0.25 * max_abs_coeff(&sr) + q_residual,
                )
            } else {
                // max |t (1 - t) (t - 1/2)| is the same on each half.
                let s2 = [
                    -2.0 * sg[0],
                    0.5 * ((-sg[0] - 3.0 * sg[1]) + (3.0 * sg[2] + sg[3])),
                    2.0 * sg[3],
                ];
                let misfit = 0.5 * (sg[0] + 3.0 * sg[1] + 3.0 * sg[2] + sg[3]).abs() / 16.0;
                let (sl, sr) = halves(&s2);
                (
                    0.048_112_522_432_469 * max_abs_coeff(&sl) + misfit + q_residual,
                    0.048_112_522_432_469 * max_abs_coeff(&sr) + misfit + q_residual,
                )
            };
            if original_certify(&taylor, &bl, &al, &cl, 0.5, ql)
                && original_certify(&taylor, &br, &ar, &cr, 0.5, qr)
            {
                counts[k][2] += 1;
            }
            // Positive beta, on the piece or on both halves, is the most
            // that any of these can certify.
            if min_coeff(&taylor.beta) > 0.0 || (min_coeff(&bl) > 0.0 && min_coeff(&br) > 0.0) {
                counts[k][3] += 1;
            }
        });
        for (k, name) in ["one-point", "two-point"].into_iter().enumerate() {
            let pct = |n: usize| 100.0 * n as f64 / totals[k] as f64;
            std::println!(
                "{name}: {} pieces; ours {:.1}%, original {:.1}%, original split {:.1}%, beta positive {:.1}%",
                totals[k],
                pct(counts[k][0]),
                pct(counts[k][1]),
                pct(counts[k][2]),
                pct(counts[k][3])
            );
        }
    }

    #[test]
    fn pair_matches_one_side_at_a_time() {
        // The two-sided recursion shares work but makes the same decisions,
        // so its output should be the same, bit for bit.
        let mut rng = Rng(98765);
        let mut n_segments = 0;
        for _ in 0..80 {
            let c = rng.cubic();
            for d in [0.3, 3.0, 30.0] {
                for tolerance in [0.25, 1e-2, 1e-4] {
                    for strict in [false, true] {
                        let one_side: fn(CubicBez, f64, f64, &mut BezPath) = if strict {
                            offset_cubic_strict
                        } else {
                            offset_cubic
                        };
                        let mut expect_pos = BezPath::new();
                        let mut expect_neg = BezPath::new();
                        one_side(c, d, tolerance, &mut expect_pos);
                        one_side(c, -d, tolerance, &mut expect_neg);
                        let mut pos = BezPath::new();
                        let mut neg = BezPath::new();
                        pos.move_to(expect_pos.elements()[0].end_point().unwrap());
                        neg.move_to(expect_neg.elements()[0].end_point().unwrap());
                        if strict {
                            offset_cubic_pair_strict(c, d, tolerance, &mut pos, &mut neg);
                        } else {
                            offset_cubic_pair(c, d, tolerance, &mut pos, &mut neg);
                        }
                        assert_eq!(pos.elements(), expect_pos.elements(), "{c:?}, d = {d}");
                        assert_eq!(neg.elements(), expect_neg.elements(), "{c:?}, d = -{d}");
                        n_segments += pos.elements().len() + neg.elements().len();
                    }
                }
            }
        }
        assert!(n_segments > 5000, "{n_segments}");
    }

    /// How the osculating parabola estimate compares with the Fréchet
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
                let mut sampled: Vec<f64> = Vec::new();
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
                    // Only pieces near acceptance matter: error within
                    // 1e-3 to 1 of |d| / 10.
                    if !(error > 1e-4 * d.abs() && error < 0.1 * d.abs()) {
                        return;
                    }
                    let (estimate, _) = osculating_estimate(&piece, &fit.delta, d, n);
                    ratios.push(error / estimate);
                    let taylor = Taylor::new(&piece, &fit.delta, d);
                    if let Some(e) = sampled_estimate(&taylor, &rec, d, f64::INFINITY) {
                        sampled.push(error / e);
                    }
                });
                for v in [&mut ratios, &mut sampled] {
                    v.sort_by(f64::total_cmp);
                }
                let q = |v: &Vec<f64>, p: f64| v[((v.len() - 1) as f64 * p) as usize];
                std::println!(
                    "n={n} {label}: {} pieces; error / osculating: median {:.3}, p90 {:.3}, p99 {:.3}, max {:.3}; error / sampled: median {:.3}, p99 {:.3}, max {:.3}",
                    ratios.len(),
                    q(&ratios, 0.5),
                    q(&ratios, 0.9),
                    q(&ratios, 0.99),
                    q(&ratios, 1.0),
                    q(&sampled, 0.5),
                    q(&sampled, 0.99),
                    q(&sampled, 1.0),
                );
            }
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn rounding_allowance_covers_large_coordinates() {
        // Far from the origin, the control points of an approximation are
        // rounded by about an ulp of their coordinates, which the bounds,
        // computed for the exact D, cannot see. Fit and bound there, move
        // back to the origin, where the subtraction is exact, and measure.
        use crate::offset_oracle::piece_error;
        let mut rng = Rng(31337);
        let mut n_checked = 0;
        let mut n_needed = 0;
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
                        let Some((a, b)) = one_point(&far, &rec) else {
                            continue;
                        };
                        let fit = co.apply(&far, &rec, a, b);
                        if fit.clamped {
                            continue;
                        }
                        let taylor = Taylor::new(&far, &fit.delta, d);
                        let Some(certificate) = Certificate::new(&taylor) else {
                            continue;
                        };
                        let Some(bound) = one_point_bound(&taylor, &certificate, d) else {
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
                        if bound < error {
                            n_needed += 1;
                        }
                        n_checked += 1;
                    }
                }
            }
        }
        assert!(n_checked > 500, "{n_checked}");
        // At 1e9 the rounding exceeds the fit's own error on small pieces,
        // so without the allowance the bound would fall short.
        assert!(n_needed > 50, "{n_needed}");
    }

    #[test]
    fn sampled_estimate_is_close() {
        // The estimate is not a bound, but on pieces where the certificate
        // holds it should be close to the truth.
        let mut n_checked = 0;
        let mut n_far = 0;
        for_each_piece(|piece, d| {
            let (co, rec) = setup(piece, d);
            let (cusp_min, cusp_max) = cusp_range(&co);
            if cusp_min * cusp_max <= 0.0 || cusp_min.abs().min(cusp_max.abs()) < 0.2 {
                return;
            }
            let Some((a, b)) = one_point(&piece, &rec) else {
                return;
            };
            let fit = co.apply(&piece, &rec, a, b);
            if fit.clamped {
                return;
            }
            let taylor = Taylor::new(&piece, &fit.delta, d);
            if Certificate::new(&taylor).is_none() {
                return;
            }
            let Some(estimate) = sampled_estimate(&taylor, &rec, d, f64::INFINITY) else {
                return;
            };
            let error = measured_error(&piece, &fit.approx, d);
            if error < 1e-9 * d.abs() {
                return;
            }
            n_checked += 1;
            if !(0.8..=1.25).contains(&(estimate / error)) {
                n_far += 1;
            }
        });
        assert!(n_checked > 1000, "{n_checked}");
        assert!(n_far * 100 < n_checked, "{n_far} of {n_checked}");
    }

    #[test]
    fn straight_line() {
        let cubic = CubicBez::new((0., 0.), (10., 0.), (20., 0.), (30., 0.));
        let mut result = BezPath::new();
        offset_cubic(cubic, 5., 1e-6, &mut result);
        // A straight line is exact, and should not be subdivided.
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
        // Repeated and collinear control points, tiny and huge curves, and
        // offsets far larger than the curve. Nothing here is checked for
        // accuracy, only that the output is finite and not absurdly long.
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
                    for strict in [false, true] {
                        offset_cubic_impl(c, d, tolerance, &mut result, strict);
                        // Splitting at a cusp does not count against the depth
                        // limit, so allow for a few of those.
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
    }

    // The remaining tests are the ones from `crate::offset`, which exercise
    // inputs that have caused trouble in the past.

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
}
