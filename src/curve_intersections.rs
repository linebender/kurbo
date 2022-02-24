use crate::ParamCurveBezierClipping;
use crate::{ParamCurve, ParamCurveExtrema};
use crate::{Point, Rect, Affine};
use crate::real::*;
use arrayvec::ArrayVec;
use std::ops::Range;

bitflags::bitflags! {
    pub struct CurveIntersectionFlags: u32 {
        const NONE                           = 0b00000000;
        const KEEP_ENDPOINT_INTERSECTIONS    = 0b00000001;
        const KEEP_DUPLICATE_INTERSECTIONS   = 0b00000010;
    }
}

/// Compute the intersections between two Bézier curves
pub fn curve_curve_intersections<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
    curve1: &T,
    curve2: &U,
    flags: CurveIntersectionFlags,
) -> ArrayVec<[(f64, f64); 9]> {
    let mut av = ArrayVec::new();

    let overlaps = check_for_overlap(curve1, curve2, flags, &mut av);
    if overlaps {
        return av;
    }

    add_curve_intersections(
        curve1,
        curve2,
        &(0.0..1.0),
        &(0.0..1.0),
        &mut av,
        false,
        0,
        0,
        curve1,
        curve2,
        flags,
    );
    av
}

/// This function tests if one curve lies along another curve for some range
fn check_for_overlap<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
    curve1: &T,
    curve2: &U,
    flags: CurveIntersectionFlags,
    intersections: &mut ArrayVec<[(f64, f64); 9]>,
) -> bool {
    // If the two curves overlap, there are two main cases we need to account for:
    //   1) One curve is completely along some portion of the other curve, thus
    //      both endpoints of the first curve lie along the other curve.
    //      ex. curve 1: O------------------O
    //          curve 2:    O---O
    //   2) The two curves overlap, thus one endpoint from each curve lies
    //      along the other.
    //      ex. curve 1: O------------------O
    //          curve 2:               O-----------O

    // Use this function to get the t-value on another curve that corresponds to the t-value on this curve
    #[inline]
    fn t_value_on_other<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
        t_this: f64,
        curve_this: &T,
        curve_other: &U,
    ) -> (bool, f64) {
        let t_values_other = point_is_on_curve(curve_this.eval(t_this), curve_other);
        if !t_values_other.0 {
            return (false, 0.0);
        }

        assert!(t_values_other.1.len() >= 1); // This has to be true or we really messed up!
        if t_values_other.1.len() == 1 {
            return (true, t_values_other.1[0])
        }

        // Things get kind of complicated if we have more than one t value
        // here. The best solution I can think of is to compare the derivatives
        // at each point.
        let curve_deriv_at_t_this = curve_this.deriv().eval(t_this);
        let curve_deriv_other = curve_other.deriv();

        // Find the t-value with the closest derivative
        let mut deriv = Point::new(f64::MAX, f64::MAX);
        let mut index = 0;
        for (i, t) in t_values_other.1.iter().enumerate()  {
            let curve_deriv_at_t_other = curve_deriv_other.eval(*t);
            let d = (curve_deriv_at_t_other - deriv).hypot2();
            let d_compare = (curve_deriv_at_t_this - deriv).hypot2();
            if d < d_compare {
                deriv = curve_deriv_at_t_other;
                index = i;
            }
        }
        (true, t_values_other.1[index])
    }

    // First, test which endpoints, if any lie along the other curve
    let start1_on_2 = t_value_on_other(0., curve1, curve2);
    let end1_on_2 = t_value_on_other(1., curve1, curve2);
    let start2_on_1 = t_value_on_other(0., curve2, curve1);
    let end2_on_1 = t_value_on_other(1., curve2, curve1);

    // At least 2 endpoints must lie on the other curve for us to proceed
    let count = if start1_on_2.0 { 1 } else { 0 } + 
                if end1_on_2.  0 { 1 } else { 0 } + 
                if start2_on_1.0 { 1 } else { 0 } + 
                if end2_on_1.0   { 1 } else { 0 };
    if count < 2 {
        return false; // No overlap
    }

    // Check a few intermediate points to make sure that it's not just the
    // endpoints that lie on top of the other curve. I'll make the claim that
    // two curves can intersect at 4 points, but not be co-linear (or co-curved?).
    // This could happen if two bezier with the same start and end point, one
    // curved and one looped, intersected where the second curve intersects itself.
    // Therefore, it's safe if we check 5 positions in total. Since we've already
    // checked the endpoints, we just need to check 3 intermediate positions.

    fn get_projection(t_value: (bool, f64), default_value: f64) -> f64 {
        if t_value.0 { t_value.1 } else { default_value }
    }

    // Determine the range of curve1 that lies along curve2
    let overlap_along_curve1 = (
        if start1_on_2.0 { 0. } else { (get_projection(start2_on_1, 1.)).min(get_projection(end2_on_1, 1.)) },
        if end1_on_2.0   { 1. } else { (get_projection(start2_on_1, 0.)).max(get_projection(end2_on_1, 0.)) },
    );
    assert!(overlap_along_curve1.0 <= overlap_along_curve1.1);

    // Determine the range of curve2 that lies along curve1 such that
    // overlap_along_curve2.0 corresponds to the same position in space as
    // overlap_along_curve1.0.
    // Note: It's not guaranteed that overlap_along_curve2.0 < overlap_along_curve2.1
    let overlap_along_curve2 = (
        if start1_on_2.0 { start1_on_2.1 }
        else { let p1 = get_projection(start2_on_1, 1.); let p2 = get_projection(end2_on_1, 1.); if p1 < p2 { 0. } else { 1. } },
        if end1_on_2.0 { end1_on_2.1 }
        else { let p1 = get_projection(start2_on_1, 0.); let p2 = get_projection(end2_on_1, 0.); if p1 < p2 { 1. } else { 0. } },
    );

    // Check the 3 intermediate locations
    let interval1 = (overlap_along_curve1.1 - overlap_along_curve1.0) / 4.;
    let interval2 = (overlap_along_curve2.1 - overlap_along_curve2.0) / 4.;
    for n in 1..3 {
        if !point_is_equal(curve1.eval(overlap_along_curve1.0 + (n as f64) * interval1), curve2.eval(overlap_along_curve2.0 + (n as f64) * interval2)) {
            return false;
        }
    }

    if flags.contains(CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS) {
        // Add the endpoints as intersections
        intersections.push((overlap_along_curve1.0, overlap_along_curve2.0));
        intersections.push((overlap_along_curve1.1, overlap_along_curve2.1));
    }

    true // The two curves overlap!
}

// This function implements the main bézier clipping algorithm by recursively subdividing curve1 and
// curve2 in to smaller and smaller portions of the original curves with the property that one of
// the curves intersects the fat line of the other curve at each stage.
//
// curve1 and curve2 at each stage are sub-bézier curves of the original curves; flip tells us
// whether curve1 at a given stage is a subcurve of the original curve1 or the original curve2;
// similarly for curve2.  domain1 and domain2 shrink (or stay the same) at each stage and describe
// which subdomain of an original curve the current curve1 and curve2 correspond to. (The domains of
// curve1 and curve2 are 0..1 at every stage.)
#[allow(clippy::too_many_arguments)]
fn add_curve_intersections<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
    curve1: &T,
    curve2: &U,
    domain1: &Range<f64>,
    domain2: &Range<f64>,
    intersections: &mut ArrayVec<[(f64, f64); 9]>,
    flip: bool,
    mut recursion_count: u32,
    mut call_count: u32,
    orig_curve1: &T,
    orig_curve2: &U,
    flags: CurveIntersectionFlags,
) -> u32 {
    call_count += 1;
    recursion_count += 1;
    if call_count >= 4096 || recursion_count >= 128 {
        return call_count;
    }

    let epsilon = 1e-9;

    if real_is_equal(domain2.start, domain2.end) || curve2.arclen(epsilon) == 0.0 {
        add_point_curve_intersection(
            curve2,
            /* point is curve1 */ false,
            curve1,
            domain2,
            domain1,
            intersections,
            flip,
            flags,
        );
        return call_count;
    } else if curve2.start() == curve2.end() {
        // There's no curve2 baseline to fat-line against (and we'll (debug) crash if we try with
        // the current implementation), so split curve2 and try again.
        let new_2_curves = orig_curve2.subsegment(domain2.clone()).subdivide();
        let domain2_mid = (domain2.start + domain2.end) * 0.5;
        call_count = add_curve_intersections(
            curve1,
            &new_2_curves.0,
            domain1,
            &(domain2.start..domain2_mid),
            intersections,
            flip,
            recursion_count,
            call_count,
            orig_curve1,
            orig_curve2,
            flags,
        );
        call_count = add_curve_intersections(
            curve1,
            &new_2_curves.1,
            domain1,
            &(domain2_mid..domain2.end),
            intersections,
            flip,
            recursion_count,
            call_count,
            orig_curve1,
            orig_curve2,
            flags,
        );
        return call_count;
    }

    // (Don't call this before checking for point curves: points are inexact and can lead to false
    // negatives here.)
    if !rectangles_overlap(&curve1.bounding_box(), &curve2.bounding_box()) {
        return call_count;
    }

    // Clip to the normal fatline and the perpendicular fatline and limit our search to the
    // intersection of the two intervals.
    let clip_norm = restrict_curve_to_fat_line(curve1, curve2);
    let clip_perp = restrict_curve_to_perpendicular_fat_line(curve1, curve2);
    let (t_min_clip, t_max_clip) = match clip_norm {
        Some((min_norm, max_norm)) => {
            match clip_perp {
                Some((min_perp, max_perp)) => (min_norm.max(min_perp), max_norm.min(max_perp)),
                None => (min_norm, max_norm),
            }
        }
        None => {
            match clip_perp {
                Some((min_perp, max_perp)) => (min_perp, max_perp),
                None => return call_count,
            }
        },
    };

    // t_min_clip and t_max_clip are (0, 1)-based, so project them back to get the new restricted
    // range:
    let new_domain1 = &(domain_value_at_t(domain1, t_min_clip)..domain_value_at_t(domain1, t_max_clip));

    if (domain2.end - domain2.start).max(new_domain1.end - new_domain1.start) < epsilon {
        let t1 = (new_domain1.start + new_domain1.end) * 0.5;
        let t2 = (domain2.start + domain2.end) * 0.5;
        // There's an unfortunate tendency for curve2 endpoints that end near (but not all
        // that near) to the interior of curve1 to register as intersections, so try to avoid
        // that. (We could be discarding a legitimate intersection here.)
        if !flags.contains(CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS)
            && (real_lte(t2, 0.0) || real_gte(t2, 1.0))
            && point_is_equal(orig_curve1.eval(t1), orig_curve2.eval(t2))
        {
            return call_count;
        }
        add_intersection(t1, orig_curve1, t2, orig_curve2, flip, intersections, flags);
        return call_count;
    }

    // Reduce curve1 to the part that might intersect curve2.
    let curve1 = &orig_curve1.subsegment(new_domain1.clone());

    // (Note: it's possible for new_domain1 to have become a point, even if
    // t_min_clip < t_max_clip. It's also possible for curve1 to not be a point even if new_domain1
    // is a point (but then curve1 will be very small).)
    if real_is_equal(new_domain1.start, new_domain1.end) || curve1.arclen(epsilon) == 0.0 {
        add_point_curve_intersection(
            curve1,
            /* point is curve1 */ true,
            curve2,
            new_domain1,
            domain2,
            intersections,
            flip,
            flags,
        );
        return call_count;
    }

    // If the new range is still 80% or more of the old range, subdivide and try again.
    if t_max_clip - t_min_clip > 0.8 {
        // Subdivide the curve which has converged the least.
        if new_domain1.end - new_domain1.start > domain2.end - domain2.start {
            let new_1_curves = curve1.subdivide();
            let new_domain1_mid = (new_domain1.start + new_domain1.end) * 0.5;
            call_count = add_curve_intersections(
                curve2,
                &new_1_curves.0,
                domain2,
                &(new_domain1.start..new_domain1_mid),
                intersections,
                !flip,
                recursion_count,
                call_count,
                orig_curve2,
                orig_curve1,
                flags,
            );
            call_count = add_curve_intersections(
                curve2,
                &new_1_curves.1,
                domain2,
                &(new_domain1_mid..new_domain1.end),
                intersections,
                !flip,
                recursion_count,
                call_count,
                orig_curve2,
                orig_curve1,
                flags,
            );
        } else {
            let new_2_curves = orig_curve2.subsegment(domain2.clone()).subdivide();
            let domain2_mid = (domain2.start + domain2.end) * 0.5;
            call_count = add_curve_intersections(
                &new_2_curves.0,
                curve1,
                &(domain2.start..domain2_mid),
                new_domain1,
                intersections,
                !flip,
                recursion_count,
                call_count,
                orig_curve2,
                orig_curve1,
                flags,
            );
            call_count = add_curve_intersections(
                &new_2_curves.1,
                curve1,
                &(domain2_mid..domain2.end),
                new_domain1,
                intersections,
                !flip,
                recursion_count,
                call_count,
                orig_curve2,
                orig_curve1,
                flags,
            );
        }
    } else {
        // Iterate.
        if !real_is_equal(domain2.end, domain2.start) {
            call_count = add_curve_intersections(
                curve2,
                curve1,
                domain2,
                new_domain1,
                intersections,
                !flip,
                recursion_count,
                call_count,
                orig_curve2,
                orig_curve1,
                flags,
            );
        } else {
            // The interval on curve2 is already tight enough, so just continue iterating on curve1.
            call_count = add_curve_intersections(
                curve1,
                curve2,
                new_domain1,
                domain2,
                intersections,
                flip,
                recursion_count,
                call_count,
                orig_curve1,
                orig_curve2,
                flags,
            );
        }
    }

    call_count
}

fn add_point_curve_intersection<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
    pt_curve: &T,
    pt_curve_is_curve1: bool,
    curve: &U,
    pt_domain: &Range<f64>,
    curve_domain: &Range<f64>,
    intersections: &mut ArrayVec<[(f64, f64); 9]>,
    flip: bool,
    flags: CurveIntersectionFlags,
) {
    let pt = pt_curve.eval(0.5);
    // We assume pt is curve1 when we add intersections below.
    let flip = if pt_curve_is_curve1 { flip } else { !flip };

    // Generally speaking |curve| will be quite small at this point, so see if we can get away with
    // just sampling here.

    let epsilon = epsilon_for_point(pt);
    let pt_t = (pt_domain.start + pt_domain.end) * 0.5;

    let curve_t = {
        let mut t_for_min = 0.0;
        let mut min_dist_sq = epsilon;
        let tenths = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        for &t in tenths.iter() {
            let d = (pt - curve.eval(t)).hypot2();
            if d < min_dist_sq {
                t_for_min = t;
                min_dist_sq = d;
            }
        }

        if real_is_zero(min_dist_sq) {
            -1.0
        } else {
            domain_value_at_t(curve_domain, t_for_min)
        }
    };

    if !real_is_equal(curve_t, -1.0) {
        add_intersection(pt_t, pt_curve, curve_t, curve, flip, intersections, flags);
        return;
    }

    // If sampling didn't work, try a different approach.
    let results = t_along_curve_for_point(pt, curve);
    for t in results {
        let curve_t = domain_value_at_t(curve_domain, t);
        add_intersection(pt_t, pt_curve, curve_t, curve, flip, intersections, flags);
    }
}

pub fn t_along_curve_for_point<T: ParamCurveBezierClipping>(
    pt: Point,
    curve: &T,
) -> ArrayVec<[f64; 2]> {
    let mut result = ArrayVec::new();

    // If both endpoints are approximately close, we only return 0.0.
    if point_is_equal(pt, curve.start()) {
        result.push(0.0);
        return result;
    }
    if point_is_equal(pt, curve.end()) {
        result.push(1.0);
        return result;
    }

    let curve_x_t_params = curve.solve_t_for_x(pt.x);
    let curve_y_t_params = curve.solve_t_for_y(pt.y);
    // We want to coalesce parameters representing the same intersection from the x and y
    // directions, but the parameter calculations aren't very accurate, so give a little more
    // leeway there (TODO: this isn't perfect, as you might expect - the dupes that pass here are
    // currently being detected in add_intersection).
    for params in [curve_x_t_params, curve_y_t_params].iter() {
        for t in params {
            let t = *t;

            // Note: use point_is_approx below for a little more wiggle room
            // since our root finding algorithm is perfect
            if !point_is_approx(pt, curve.eval(t), 2.) { 
                continue;
            }
            let mut already_found_t = false;
            for u in &result {
                if real_is_equal(t, *u) {
                    already_found_t = true;
                    break;
                }
            }
            if !already_found_t {
                result.push(t);
            }
        }
    }

    if !result.is_empty() {
        return result;
    }

    // The remaining case is if pt is approximately equal to an interior point
    // of curve, but not within the x-range or y-range of the curve (which we
    // already checked) due to floating point errors - for example if curve is
    // a horizontal line that extends beyond its endpoints, and pt is on the
    // extrema, but just barely outside the x-y limits; or if the curve has a
    // cusp in one of the corners of its convex hull and pt is diagonally just
    // outside the hull.
    #[inline]
    fn maybe_add<T: ParamCurve + ParamCurveExtrema>(
        t: f64,
        pt: Point,
        curve: &T,
        result: &mut ArrayVec<[f64; 2]>,
    ) -> bool {
        if point_is_equal(curve.eval(t), pt) {
            result.push(t);
            return true;
        }
        false
    }

    for ex in curve.extrema() {
        maybe_add(ex, pt, curve, &mut result);
    }

    result
}

fn point_is_on_curve<T: ParamCurveBezierClipping>(
    pt: Point,
    curve: &T,
) -> (bool, ArrayVec<[f64; 2]>) {
    let t_values = t_along_curve_for_point(pt, curve);
    for t in &t_values {
        if point_is_equal(pt, curve.eval(*t)) {
            return (true, t_values.clone()); // Point lies along curve
        }
    }
    (false, t_values.clone()) // Point is not along curve
}

fn add_intersection<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
    t1: f64,
    orig_curve1: &T,
    t2: f64,
    orig_curve2: &U,
    flip: bool,
    intersections: &mut ArrayVec<[(f64, f64); 9]>,
    flags: CurveIntersectionFlags,
) {
    let (t1, t2) = if flip { (t2, t1) } else { (t1, t2) };
    
    // Discard endpoint/endpoint intersections.
    if !flags.contains(CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS)
    {
        let t1_is_an_endpoint = real_lte(t1, 0.0) || real_gte(t1, 1.0);
        let t2_is_an_endpoint = real_lte(t2, 0.0) || real_gte(t2, 1.0);
        if t1_is_an_endpoint && t2_is_an_endpoint {
            return;
        }
    }

    if !flags.contains(CurveIntersectionFlags::KEEP_DUPLICATE_INTERSECTIONS)
    {
        // (This should probably depend in some way on how large our input coefficients are.)
        let epsilon = 1e-3;

        // We can get repeated intersections when we split a curve at an intersection point, or when
        // two curves intersect at a point where the curves are very close together, or when the fat
        // line process breaks down.
        for i in 0..intersections.len() {
            let (old_t1, old_t2) = intersections[i];
            // f64 errors can be particularly bad (over a hundred) if we wind up keeping the "wrong"
            // duplicate intersection, so always keep the one that minimizes sample distance.
            if (t1 - old_t1).abs() < epsilon && (t2 - old_t2).abs() < epsilon {
                let cur_dist = (orig_curve1.eval(old_t1) - orig_curve2.eval(old_t2)).hypot2();
                let new_dist = (orig_curve1.eval(t1) - orig_curve2.eval(t2)).hypot2();
                if new_dist < cur_dist {
                    intersections[i] = (t1, t2);
                }
                return;
            }
        }
    }

    if intersections.len() < 9 {
        intersections.push((t1, t2));
    }
}

// Returns an interval (t_min, t_max) with the property that for parameter values outside that
// interval, curve1 is guaranteed to not intersect curve2; uses the fat line of curve2 as its basis
// for the guarantee. (See the Sederberg document for what's going on here.)
fn restrict_curve_to_fat_line<
    T: ParamCurveBezierClipping + ParamCurve + ParamCurveExtrema,
    U: ParamCurveBezierClipping + ParamCurve + ParamCurveExtrema,
>(
    curve1: &T,
    curve2: &U,
) -> Option<(f64, f64)> {
    // TODO: Consider clipping against the perpendicular fat line as well (recommended by
    // Sederberg).
    // TODO: The current algorithm doesn't handle the (rare) case where curve1 and curve2 are
    // overlapping lines.

    let baseline2 = curve2.baseline();
    let (mut top, mut bottom) = curve1.convex_hull_from_line(&baseline2);
    let (d_min, d_max) = curve2.fat_line_min_max(&baseline2);

    clip_convex_hull_to_fat_line(&mut top, &mut bottom, d_min, d_max)
}

// Returns an interval (t_min, t_max) with the property that for parameter values outside that
// interval, curve1 is guaranteed to not intersect curve2; uses the perpendicular fatline of
// curve2 as its basis for the guarantee. We use check the perpendicular fatline as well as the
// regular fatline because sometimes it converges to the result faster.
fn restrict_curve_to_perpendicular_fat_line<
    T: ParamCurveBezierClipping + ParamCurve + ParamCurveExtrema,
    U: ParamCurveBezierClipping + ParamCurve + ParamCurveExtrema,
>(
    curve1: &T,
    curve2: &U,
) -> Option<(f64, f64)> {
    use std::f64::consts::PI;

    // TODO: Consider clipping against the perpendicular fat line as well (recommended by
    // Sederberg).
    // TODO: The current algorithm doesn't handle the (rare) case where curve1 and curve2 are
    // overlapping lines.

    let baseline2 = curve2.baseline();
    let center_baseline2 = baseline2.bounding_box().center().to_vec2();
    let affine_baseline2 = Affine::translate(center_baseline2) * Affine::rotate(PI / 2.) * Affine::translate(-center_baseline2);
    let baseline2 = affine_baseline2 * baseline2;
    let (mut top, mut bottom) = curve1.convex_hull_from_line(&baseline2);
    let (d_min, d_max) = curve2.fat_line_min_max(&baseline2);

    clip_convex_hull_to_fat_line(&mut top, &mut bottom, d_min, d_max)
}

// Returns the min and max values at which the convex hull enters the fat line min/max offset lines.
fn clip_convex_hull_to_fat_line(
    hull_top: &mut Vec<Point>,
    hull_bottom: &mut Vec<Point>,
    d_min: f64,
    d_max: f64,
) -> Option<(f64, f64)> {
    // Walk from the left corner of the convex hull until we enter the fat line limits:
    let t_clip_min = walk_convex_hull_start_to_fat_line(hull_top, hull_bottom, d_min, d_max)?;

    // Now walk from the right corner of the convex hull until we enter the fat line limits - to
    // walk right to left we just reverse the order of the hull vertices, so that hull_top and
    // hull_bottom start at the right corner now:
    hull_top.reverse();
    hull_bottom.reverse();
    let t_clip_max = walk_convex_hull_start_to_fat_line(hull_top, hull_bottom, d_min, d_max)?;

    Some((t_clip_min, t_clip_max))
}

// Walk the edges of the convex hull until you hit a fat line offset value, starting from the
// (first vertex in hull_top_vertices == first vertex in hull_bottom_vertices).
fn walk_convex_hull_start_to_fat_line(
    hull_top_vertices: &[Point],
    hull_bottom_vertices: &[Point],
    d_min: f64,
    d_max: f64,
) -> Option<f64> {
    let start_corner = hull_top_vertices[0];

    if start_corner.y < d_min {
        walk_convex_hull_edges_to_fat_line(hull_top_vertices, true, d_min)
    } else if start_corner.y > d_max {
        walk_convex_hull_edges_to_fat_line(hull_bottom_vertices, false, d_max)
    } else {
        Some(start_corner.x)
    }
}

// Do the actual walking, starting from the first vertex of hull_vertices.
fn walk_convex_hull_edges_to_fat_line(
    hull_vertices: &[Point],
    vertices_are_for_top: bool,
    threshold: f64,
) -> Option<f64> {
    for i in 0..hull_vertices.len() - 1 {
        let p = hull_vertices[i];
        let q = hull_vertices[i + 1];
        if (vertices_are_for_top && q.y >= threshold) || (!vertices_are_for_top && q.y <= threshold)
        {
            return if (q.y - threshold).abs() < f64::EPSILON {
                Some(q.x)
            } else {
                Some(p.x + (threshold - p.y) * (q.x - p.x) / (q.y - p.y))
            };
        }
    }
    // All points of the hull are outside the threshold:
    None
}

#[inline]
// Return the point of domain corresponding to the point t, 0 <= t <= 1.
fn domain_value_at_t(domain: &Range<f64>, t: f64) -> f64 {
    domain.start + (domain.end - domain.start) * t
}

#[inline]
// Rect.intersects doesn't count edge/corner intersections, this version does.
fn rectangles_overlap(r1: &Rect, r2: &Rect) -> bool {
    r1.origin().x <= r2.origin().x + r2.size().width
        && r2.origin().x <= r1.origin().x + r1.size().width
        && r1.origin().y <= r2.origin().y + r2.size().height
        && r2.origin().y <= r1.origin().y + r1.size().height
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CubicBez, QuadBez, Line};

    fn do_test<T: ParamCurveBezierClipping, U: ParamCurveBezierClipping>(
        curve1: &T,
        curve2: &U,
        count: usize,
        flags: CurveIntersectionFlags,
    ) {
        let arr1 = curve_curve_intersections(curve1, curve2, flags);
        let arr2 = curve_curve_intersections(curve2, curve1, flags);
        assert_eq!(arr1.len(), count);
        assert_eq!(arr2.len(), count);
    }

    fn test_t_along_curve<T: ParamCurveBezierClipping>(
        curve: &T,
        t_test: f64,
    ) {
        let pt_test = curve.eval(t_test);
        let t_values = t_along_curve_for_point(pt_test, curve);
        assert!(!t_values.is_empty());

        let mut found_t = false;
        for t in &t_values {
            let pt = curve.eval(*t);

            if real_is_equal(*t, t_test)
            {
                found_t = true;
            }

            println!("t for pt on curve: {} {}, expected {} {}", t, pt, t_test, pt_test);
            assert!(point_is_equal(pt, pt_test))
        }
        assert!(found_t)
    }

    fn test_t<T: ParamCurveBezierClipping>(
        curve: &T,
    ) {
        let tenths = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        for &t in tenths.iter() {
            test_t_along_curve(curve, t);
        }
    }

    fn test_overlapping<T: ParamCurveBezierClipping>(
        curve: &T,
        t1: f64,
        t2: f64,
    ) {
        assert!(t1 < t2);

        do_test(
            &curve.subsegment(0.0..t2),
            &curve.subsegment(t1..1.0),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
        do_test(
            &curve.subsegment(0.0..t2),
            &curve.subsegment(1.0..t1),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
        do_test(
            &curve.subsegment(0.0..1.0),
            &curve.subsegment(t1..t2),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
        do_test(
            &curve.subsegment(0.0..1.0),
            &curve.subsegment(t2..t1),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
    }

    #[test]
    fn test_cubic_cubic_intersections() {
        test_t(&CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)));
        test_t(&CubicBez::new((0.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0)));
        
        do_test(
            &CubicBez::new((0.0, 0.0), (0.0, 1.0), (0.0, 1.0), (1.0, 1.0)),
            &CubicBez::new((0.0, 1.0), (1.0, 1.0), (1.0, 1.0), (1.0, 0.0)),
            1,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &CubicBez::new((48.0, 84.0), (104.0, 176.0), (190.0, 37.0), (121.0, 75.0)),
            &CubicBez::new((68.0, 145.0), (74.0, 6.0), (143.0, 197.0), (138.0, 55.0)),
            4,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &CubicBez::new((0.0, 0.0), (0.5, 1.0), (0.5, 1.0), (1.0, 0.0)),
            &CubicBez::new((0.0, 1.0), (0.5, 0.0), (0.5, 0.0), (1.0, 1.0)),
            2,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &CubicBez::new((0.2, 0.0), (0.5, 3.0), (0.5, -2.0), (0.8, 1.0)),
            &CubicBez::new((0.0, 0.0), (2.5, 0.5), (-1.5, 0.5), (1.0, 0.0)),
            9,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &QuadBez::new((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
            &CubicBez::new((0.2, 0.0), (0.5, 3.0), (0.5, -2.0), (0.8, 1.0)),
            3,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &QuadBez::new((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
            &QuadBez::new((0.0, 0.25), (0.5, 0.75), (1.0, 0.25)),
            1,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &QuadBez::new((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
            &QuadBez::new((0.0, 0.25), (0.5, -0.25), (1.0, 0.25)),
            2,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &QuadBez::new((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
            &Line::new((0.0, 0.5), (1.0, 0.25)),
            2,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &QuadBez::new((0.0, 0.0), (0.5, 1.0), (1.0, 0.0)),
            &Line::new((0.0, 0.5), (1.0, 0.5)),
            1,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        do_test(
            &CubicBez::new((0.2, 0.0), (0.5, 3.0), (0.5, -2.0), (0.8, 1.0)),
            &Line::new((0.0, 0.5), (1.0, 0.25)),
            3,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
        
        // These tests intersect at the endpoints which is only sometimes useful to know about.
        do_test(
            &CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)),
            &CubicBez::new((1.0, 0.0), (0.7, -1.0), (0.3, -1.0), (0.0, 0.0)),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
        do_test(
            &CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)),
            &CubicBez::new((1.0, 0.0), (0.7, -1.0), (0.3, -1.0), (0.0, 0.0)),
            0,
            CurveIntersectionFlags::NONE, // Discard endpoint intersections
        );
        do_test(
            &CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)),
            &CubicBez::new((0.0, 0.0), (0.3, -0.5), (0.7, -0.5), (1.0, 0.0)),
            2,
            CurveIntersectionFlags::KEEP_ENDPOINT_INTERSECTIONS, // Include endpoint intersections
        );
        do_test(
            &CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)),
            &CubicBez::new((0.0, 0.0), (0.3, -0.5), (0.7, -0.5), (1.0, 0.0)),
            0,
            CurveIntersectionFlags::NONE, // Discard endpoint intersections
        );

        // Test curves that lie exactly along one-another
        test_overlapping(&CubicBez::new((0.0, 0.0), (0.3, -1.0), (0.7, -1.0), (1.0, 0.0)), 0.2, 0.8);

        // (A previous version of the code was returning two practically identical
        // intersection points here.)
        do_test(
            &CubicBez::new(
                (718133.1363092018, 673674.987999388),
                (-53014.13135835016, 286988.87959900266),
                (-900630.1880107201, -7527.6889376943),
                (417822.48349384824, -149039.14932848653),
            ),
            &CubicBez::new(
                (924715.3309247112, 719414.5221912428),
                (965365.9679664494, -563421.3040676294),
                (273552.85484064696, 643090.0890117711),
                (-113963.134524995, 732017.9466050486),
            ),
            1,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );

        // On these curves the algorithm runs to a state at which the new clipped domain1 becomes a
        // point even though t_min_clip < t_max_clip (because domain1 was small enough to begin with
        // relative to the small distance between t_min_clip and t_max_clip), and the new curve1 is not
        // a point (it's split off the old curve1 using t_min_clip < t_max_clip).
        do_test(
            &CubicBez::new(
                (423394.5967598548, -91342.7434613118),
                (333212.450870987, 225564.45711810607),
                (668108.668469816, -626100.8367380127),
                (-481885.0610437216, 893767.5320803947),
            ),
            &CubicBez::new(
                (-484505.2601961801, -222621.44229855016),
                (22432.829984141514, -944727.7102144773),
                (-433294.66549074976, -168018.60431004688),
                (567688.5977972192, 13975.09633399453),
            ),
            3,
            CurveIntersectionFlags::NONE, // Standard algorithm
        );
    }
}
