//! Bézier paths (up to cubic).

use std::ops::{Mul, Range};

use arrayvec::ArrayVec;

use crate::common::{solve_cubic, solve_quadratic};
use crate::MAX_EXTREMA;
use crate::{
    Affine, CubicBez, Line, ParamCurve, ParamCurveArclen, ParamCurveArea, ParamCurveExtrema,
    ParamCurveNearest, Point, QuadBez, Rect, Shape, TranslateScale,
};

/// A path that can Bézier segments up to cubic, possibly with multiple subpaths.
#[derive(Clone, Default, Debug)]
pub struct BezPath(Vec<PathEl>);

/// The element of a Bézier path.
///
/// A valid path has `Moveto` at the beginning of each subpath.
#[derive(Clone, Copy, Debug)]
pub enum PathEl {
    MoveTo(Point),
    LineTo(Point),
    QuadTo(Point, Point),
    CurveTo(Point, Point, Point),
    ClosePath,
}

/// A segment of a Bézier path.
#[derive(Clone, Copy, Debug)]
pub enum PathSeg {
    Line(Line),
    Quad(QuadBez),
    Cubic(CubicBez),
}

impl BezPath {
    /// Create a new path.
    pub fn new() -> BezPath {
        Default::default()
    }

    /// Create a path from a vector of path elements.
    pub fn from_vec(v: Vec<PathEl>) -> BezPath {
        BezPath(v)
    }

    /// Push a generic path element onto the path.
    pub fn push(&mut self, el: PathEl) {
        self.0.push(el)
    }

    /// Push a "move to" element onto the path.
    pub fn move_to<P: Into<Point>>(&mut self, p: P) {
        self.push(PathEl::MoveTo(p.into()));
    }

    /// Push a "line to" element onto the path.
    pub fn line_to<P: Into<Point>>(&mut self, p: P) {
        self.push(PathEl::LineTo(p.into()));
    }

    /// Push a "quad to" element onto the path.
    pub fn quad_to<P: Into<Point>>(&mut self, p1: P, p2: P) {
        self.push(PathEl::QuadTo(p1.into(), p2.into()));
    }

    /// Push a "curPe to" element onto the path.
    pub fn curve_to<P: Into<Point>>(&mut self, p1: P, p2: P, p3: P) {
        self.push(PathEl::CurveTo(p1.into(), p2.into(), p3.into()));
    }

    /// Push a "close path" element onto the path.
    pub fn close_path(&mut self) {
        self.push(PathEl::ClosePath);
    }

    /// Get the path elements.
    pub fn elements(&self) -> &[PathEl] {
        &self.0
    }

    /// Iterate over the path segments.
    pub fn segments<'a>(&'a self) -> impl Iterator<Item = PathSeg> + 'a {
        BezPath::segments_of_slice(&self.0)
    }

    // TODO: expose as pub method? Maybe should be a trait so slice.segments() works?
    fn segments_of_slice<'a>(slice: &'a [PathEl]) -> BezPathSegs<'a> {
        let first = match slice.get(0) {
            Some(PathEl::MoveTo(ref p)) => *p,
            Some(_) => panic!("First element has to be a PathEl::Moveto!"),
            None => Default::default(),
        };

        BezPathSegs {
            c: slice.iter(),
            start: first,
            last: first,
        }
    }

    /// Get the segment at the given element index.
    ///
    /// The element index counts [`PathEl`](enum.PathEl.html) elements, so
    /// for example includes an initial `Moveto`.
    pub fn get_seg(&self, ix: usize) -> Option<PathSeg> {
        if ix == 0 || ix >= self.0.len() {
            return None;
        }
        let last = match self.0[ix - 1] {
            PathEl::MoveTo(p) => p,
            PathEl::LineTo(p) => p,
            PathEl::QuadTo(_, p2) => p2,
            PathEl::CurveTo(_, _, p3) => p3,
            _ => return None,
        };
        match self.0[ix] {
            PathEl::LineTo(p) => Some(PathSeg::Line(Line::new(last, p))),
            PathEl::QuadTo(p1, p2) => Some(PathSeg::Quad(QuadBez::new(last, p1, p2))),
            PathEl::CurveTo(p1, p2, p3) => Some(PathSeg::Cubic(CubicBez::new(last, p1, p2, p3))),
            PathEl::ClosePath => self.0[..ix].iter().rev().find_map(|el| match *el {
                PathEl::MoveTo(start) => Some(PathSeg::Line(Line::new(last, start))),
                _ => None,
            }),
            _ => None,
        }
    }

    /// Returns `true` if the path contains no segments.
    pub fn is_empty(&self) -> bool {
        !self.0.iter().any(|el| match *el {
            PathEl::LineTo(..) | PathEl::QuadTo(..) | PathEl::CurveTo(..) => true,
            _ => false,
        })
    }

    /// Apply an affine transform to the path.
    pub fn apply_affine(&mut self, affine: Affine) {
        for el in self.0.iter_mut() {
            *el = affine * (*el);
        }
    }

    /// Find the nearest point.
    ///
    /// Panics if path is empty or invalid.
    ///
    /// Note that the returned index counts segments, not elements. Thus, the
    /// initial `Moveto` is not counted. For a simple path consisting of a `Moveto`
    /// followed by `Lineto/Quadto/Cubicto` elements, the element index is the
    /// segment index + 1.
    ///
    /// Returns the index of the segment, the parameter within that segment, and
    /// the square of the distance to the point.
    pub fn nearest(&self, p: Point, accuracy: f64) -> (usize, f64, f64) {
        let mut best = None;
        for (ix, seg) in self.segments().enumerate() {
            let (t, r) = seg.nearest(p, accuracy);
            if best.map(|(_, _, r_best)| r < r_best).unwrap_or(true) {
                best = Some((ix, t, r));
            }
        }
        best.unwrap()
    }
}

impl<'a> IntoIterator for &'a BezPath {
    type Item = PathEl;
    type IntoIter = std::iter::Cloned<std::slice::Iter<'a, PathEl>>;

    fn into_iter(self) -> Self::IntoIter {
        self.elements().iter().cloned()
    }
}

impl Mul<PathEl> for Affine {
    type Output = PathEl;

    fn mul(self, other: PathEl) -> PathEl {
        match other {
            PathEl::MoveTo(p) => PathEl::MoveTo(self * p),
            PathEl::LineTo(p) => PathEl::LineTo(self * p),
            PathEl::QuadTo(p1, p2) => PathEl::QuadTo(self * p1, self * p2),
            PathEl::CurveTo(p1, p2, p3) => PathEl::CurveTo(self * p1, self * p2, self * p3),
            PathEl::ClosePath => PathEl::ClosePath,
        }
    }
}

impl Mul<BezPath> for Affine {
    type Output = BezPath;

    fn mul(self, other: BezPath) -> BezPath {
        BezPath(other.0.iter().map(|&el| self * el).collect())
    }
}

impl<'a> Mul<&'a BezPath> for Affine {
    type Output = BezPath;

    fn mul(self, other: &BezPath) -> BezPath {
        BezPath(other.0.iter().map(|&el| self * el).collect())
    }
}

impl Mul<PathEl> for TranslateScale {
    type Output = PathEl;

    fn mul(self, other: PathEl) -> PathEl {
        match other {
            PathEl::MoveTo(p) => PathEl::MoveTo(self * p),
            PathEl::LineTo(p) => PathEl::LineTo(self * p),
            PathEl::QuadTo(p1, p2) => PathEl::QuadTo(self * p1, self * p2),
            PathEl::CurveTo(p1, p2, p3) => PathEl::CurveTo(self * p1, self * p2, self * p3),
            PathEl::ClosePath => PathEl::ClosePath,
        }
    }
}

impl Mul<BezPath> for TranslateScale {
    type Output = BezPath;

    fn mul(self, other: BezPath) -> BezPath {
        BezPath(other.0.iter().map(|&el| self * el).collect())
    }
}

impl<'a> Mul<&'a BezPath> for TranslateScale {
    type Output = BezPath;

    fn mul(self, other: &BezPath) -> BezPath {
        BezPath(other.0.iter().map(|&el| self * el).collect())
    }
}

struct BezPathSegs<'a> {
    c: std::slice::Iter<'a, PathEl>,
    start: Point,
    last: Point,
}

impl<'a> Iterator for BezPathSegs<'a> {
    type Item = PathSeg;

    fn next(&mut self) -> Option<PathSeg> {
        for el in &mut self.c {
            let (ret, last) = match *el {
                PathEl::MoveTo(p) => {
                    self.start = p;
                    self.last = p;
                    continue;
                }
                PathEl::LineTo(p) => (PathSeg::Line(Line::new(self.last, p)), p),
                PathEl::QuadTo(p1, p2) => (PathSeg::Quad(QuadBez::new(self.last, p1, p2)), p2),
                PathEl::CurveTo(p1, p2, p3) => {
                    (PathSeg::Cubic(CubicBez::new(self.last, p1, p2, p3)), p3)
                }
                PathEl::ClosePath => {
                    if self.last != self.start {
                        (PathSeg::Line(Line::new(self.last, self.start)), self.start)
                    } else {
                        continue;
                    }
                }
            };

            self.last = last;
            return Some(ret);
        }
        None
    }
}

impl<'a> BezPathSegs<'a> {
    /// Here, `accuracy` specifies the accuracy for each Bézier segment. At worst,
    /// the total error is `accuracy` times the number of Bézier segments.

    // TODO: pub? Or is this subsumed by method of &[PathEl]?
    fn arclen(self, accuracy: f64) -> f64 {
        self.map(|seg| seg.arclen(accuracy)).sum()
    }

    // Same
    fn area(self) -> f64 {
        self.map(|seg| seg.signed_area()).sum()
    }

    // Same
    fn winding(self, p: Point) -> i32 {
        self.map(|seg| seg.winding(p)).sum()
    }

    // Same
    fn bounding_box(self) -> Rect {
        let mut bbox: Option<Rect> = None;
        for seg in self {
            let seg_bb = seg.bounding_box();
            if let Some(bb) = bbox {
                bbox = Some(bb.union(seg_bb));
            } else {
                bbox = Some(seg_bb)
            }
        }
        bbox.unwrap_or_default()
    }
}

impl ParamCurve for PathSeg {
    fn eval(&self, t: f64) -> Point {
        match *self {
            PathSeg::Line(line) => line.eval(t),
            PathSeg::Quad(quad) => quad.eval(t),
            PathSeg::Cubic(cubic) => cubic.eval(t),
        }
    }

    fn subsegment(&self, range: Range<f64>) -> PathSeg {
        match *self {
            PathSeg::Line(line) => PathSeg::Line(line.subsegment(range)),
            PathSeg::Quad(quad) => PathSeg::Quad(quad.subsegment(range)),
            PathSeg::Cubic(cubic) => PathSeg::Cubic(cubic.subsegment(range)),
        }
    }
}

impl ParamCurveArclen for PathSeg {
    fn arclen(&self, accuracy: f64) -> f64 {
        match *self {
            PathSeg::Line(line) => line.arclen(accuracy),
            PathSeg::Quad(quad) => quad.arclen(accuracy),
            PathSeg::Cubic(cubic) => cubic.arclen(accuracy),
        }
    }
}

impl ParamCurveArea for PathSeg {
    fn signed_area(&self) -> f64 {
        match *self {
            PathSeg::Line(line) => line.signed_area(),
            PathSeg::Quad(quad) => quad.signed_area(),
            PathSeg::Cubic(cubic) => cubic.signed_area(),
        }
    }
}

impl ParamCurveNearest for PathSeg {
    fn nearest(&self, p: Point, accuracy: f64) -> (f64, f64) {
        match *self {
            PathSeg::Line(line) => line.nearest(p, accuracy),
            PathSeg::Quad(quad) => quad.nearest(p, accuracy),
            PathSeg::Cubic(cubic) => cubic.nearest(p, accuracy),
        }
    }
}

impl ParamCurveExtrema for PathSeg {
    fn extrema(&self) -> ArrayVec<[f64; MAX_EXTREMA]> {
        match *self {
            PathSeg::Line(line) => line.extrema(),
            PathSeg::Quad(quad) => quad.extrema(),
            PathSeg::Cubic(cubic) => cubic.extrema(),
        }
    }
}

impl PathSeg {
    /// Returns a new `PathSeg` describing the same path as `self`, but with
    /// the points reversed.
    pub fn reverse(&self) -> PathSeg {
        match self {
            PathSeg::Line(Line { p0, p1 }) => PathSeg::Line(Line::new(*p1, *p0)),
            PathSeg::Quad(q) => PathSeg::Quad(QuadBez::new(q.p2, q.p1, q.p0)),
            PathSeg::Cubic(c) => PathSeg::Cubic(CubicBez::new(c.p3, c.p2, c.p1, c.p0)),
        }
    }

    /// Convert this segment to a cubic bezier.
    pub fn to_cubic(&self) -> CubicBez {
        match *self {
            PathSeg::Line(Line { p0, p1 }) => CubicBez::new(p0, p0, p1, p1),
            PathSeg::Cubic(c) => c,
            PathSeg::Quad(q) => q.raise(),
        }
    }

    // Assumes split at extrema.
    fn winding_inner(&self, p: Point) -> i32 {
        let start = self.start();
        let end = self.end();
        let sign = if end.y > start.y {
            if p.y < start.y || p.y >= end.y {
                return 0;
            }
            -1
        } else if end.y < start.y {
            if p.y < end.y || p.y >= start.y {
                return 0;
            }
            1
        } else {
            return 0;
        };
        match *self {
            PathSeg::Line(_line) => {
                if p.x < start.x.min(end.x) {
                    return 0;
                }
                if p.x >= start.x.max(end.x) {
                    return sign;
                }
                // line equation ax + by = c
                let a = end.y - start.y;
                let b = start.x - end.x;
                let c = a * start.x + b * start.y;
                if (a * p.x + b * p.y - c) * (sign as f64) >= 0.0 {
                    sign
                } else {
                    0
                }
            }
            PathSeg::Quad(quad) => {
                let p1 = quad.p1;
                if p.x < start.x.min(end.x).min(p1.x) {
                    return 0;
                }
                if p.x >= start.x.max(end.x).max(p1.x) {
                    return sign;
                }
                let a = end.y - 2.0 * p1.y + start.y;
                let b = 2.0 * (p1.y - start.y);
                let c = start.y - p.y;
                for t in solve_quadratic(c, b, a) {
                    if t >= 0.0 && t <= 1.0 {
                        let x = quad.eval(t).x;
                        if p.x >= x {
                            return sign;
                        } else {
                            return 0;
                        }
                    }
                }
                0
            }
            PathSeg::Cubic(cubic) => {
                let p1 = cubic.p1;
                let p2 = cubic.p2;
                if p.x < start.x.min(end.x).min(p1.x).min(p2.x) {
                    return 0;
                }
                if p.x >= start.x.max(end.x).max(p1.x).max(p2.x) {
                    return sign;
                }
                let a = end.y - 3.0 * p2.y + 3.0 * p1.y - start.y;
                let b = 3.0 * (p2.y - 2.0 * p1.y + start.y);
                let c = 3.0 * (p1.y - start.y);
                let d = start.y - p.y;
                for t in solve_cubic(d, c, b, a) {
                    if t >= 0.0 && t <= 1.0 {
                        let x = cubic.eval(t).x;
                        if p.x >= x {
                            return sign;
                        } else {
                            return 0;
                        }
                    }
                }
                0
            }
        }
    }

    /// Compute the winding number contribution of a single segment.
    ///
    /// Cast a ray to the left and count intersections.
    fn winding(&self, p: Point) -> i32 {
        self.extrema_ranges()
            .into_iter()
            .map(|range| self.subsegment(range).winding_inner(p))
            .sum()
    }
}

impl Shape for BezPath {
    type BezPathIter = std::vec::IntoIter<PathEl>;

    fn to_bez_path(&self, _tolerance: f64) -> Self::BezPathIter {
        self.clone().0.into_iter()
    }

    /// Signed area.
    fn area(&self) -> f64 {
        self.elements().area()
    }

    fn perimeter(&self, accuracy: f64) -> f64 {
        self.elements().perimeter(accuracy)
    }

    /// Winding number of point.
    fn winding(&self, pt: Point) -> i32 {
        self.elements().winding(pt)
    }

    fn bounding_box(&self) -> Rect {
        self.elements().bounding_box()
    }

    fn as_path_slice(&self) -> Option<&[PathEl]> {
        Some(&self.0)
    }
}

impl<'a> Shape for &'a [PathEl] {
    type BezPathIter = std::iter::Cloned<std::slice::Iter<'a, PathEl>>;

    #[inline]
    fn to_bez_path(&self, _tolerance: f64) -> Self::BezPathIter {
        self.iter().cloned()
    }

    /// Signed area.
    fn area(&self) -> f64 {
        BezPath::segments_of_slice(self).area()
    }

    fn perimeter(&self, accuracy: f64) -> f64 {
        BezPath::segments_of_slice(self).arclen(accuracy)
    }

    /// Winding number of point.
    fn winding(&self, pt: Point) -> i32 {
        BezPath::segments_of_slice(self).winding(pt)
    }

    fn bounding_box(&self) -> Rect {
        BezPath::segments_of_slice(self).bounding_box()
    }

    #[inline]
    fn as_path_slice(&self) -> Option<&[PathEl]> {
        Some(self)
    }
}
