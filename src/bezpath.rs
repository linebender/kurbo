//! Bézier paths (up to cubic).

use std::iter::{Extend, FromIterator};
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BezPath(Vec<PathEl>);

/// The element of a Bézier path.
///
/// A valid path has `Moveto` at the beginning of each subpath.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathEl {
    /// Move directly to the point without drawing anything, starting a new
    /// subpath.
    MoveTo(Point),
    /// Draw a line from the current location to the point.
    LineTo(Point),
    /// Draw a quadratic bezier using the current location and the two points.
    QuadTo(Point, Point),
    /// Draw a cubic bezier using the current location and the three points.
    CurveTo(Point, Point, Point),
    /// Close off the path.
    ClosePath,
}

/// A segment of a Bézier path.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PathSeg {
    /// A line segment.
    Line(Line),
    /// A quadratic bezier segment.
    Quad(QuadBez),
    /// A cubic bezier segment.
    Cubic(CubicBez),
}

impl BezPath {
    /// Create a new path.
    pub fn new() -> BezPath {
        Default::default()
    }

    /// Create a path from a vector of path elements.
    ///
    /// `BezPath` also implements `FromIterator<PathEl>`, so it works with `collect`:
    ///
    /// ```
    /// // a very contrived example:
    /// use kurbo::{BezPath, PathEl};
    ///
    /// let path = BezPath::new();
    /// let as_vec: Vec<PathEl> = path.into_iter().collect();
    /// let back_to_path: BezPath = as_vec.into_iter().collect();
    /// ```
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

    /// Push a "curve to" element onto the path.
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

    /// Returns an iterator over the path's elements.
    pub fn iter(&self) -> impl Iterator<Item = PathEl> + '_ {
        self.0.iter().copied()
    }

    /// Iterate over the path segments.
    pub fn segments(&self) -> impl Iterator<Item = PathSeg> + '_ {
        BezPath::segments_of_slice(&self.0)
    }

    /// Flatten the path, invoking the callback repeatedly.
    ///
    /// Flattening is the action of approximating a curve with a succession of line segments.
    ///
    /// <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 120 30" height="30mm" width="120mm">
    ///   <path d="M26.7 24.94l.82-11.15M44.46 5.1L33.8 7.34" fill="none" stroke="#55d400" stroke-width=".5"/>
    ///   <path d="M26.7 24.94c.97-11.13 7.17-17.6 17.76-19.84M75.27 24.94l1.13-5.5 2.67-5.48 4-4.42L88 6.7l5.02-1.6" fill="none" stroke="#000"/>
    ///   <path d="M77.57 19.37a1.1 1.1 0 0 1-1.08 1.08 1.1 1.1 0 0 1-1.1-1.08 1.1 1.1 0 0 1 1.08-1.1 1.1 1.1 0 0 1 1.1 1.1" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M77.57 19.37a1.1 1.1 0 0 1-1.08 1.08 1.1 1.1 0 0 1-1.1-1.08 1.1 1.1 0 0 1 1.08-1.1 1.1 1.1 0 0 1 1.1 1.1" color="#000" fill="#fff"/>
    ///   <path d="M80.22 13.93a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.08 1.1 1.1 0 0 1 1.08 1.08" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M80.22 13.93a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.08 1.1 1.1 0 0 1 1.08 1.08" color="#000" fill="#fff"/>
    ///   <path d="M84.08 9.55a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.1-1.1 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M84.08 9.55a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.1-1.1 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="#fff"/>
    ///   <path d="M89.1 6.66a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.08-1.08 1.1 1.1 0 0 1 1.1 1.08" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M89.1 6.66a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.08-1.08 1.1 1.1 0 0 1 1.1 1.08" color="#000" fill="#fff"/>
    ///   <path d="M94.4 5a1.1 1.1 0 0 1-1.1 1.1A1.1 1.1 0 0 1 92.23 5a1.1 1.1 0 0 1 1.08-1.08A1.1 1.1 0 0 1 94.4 5" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M94.4 5a1.1 1.1 0 0 1-1.1 1.1A1.1 1.1 0 0 1 92.23 5a1.1 1.1 0 0 1 1.08-1.08A1.1 1.1 0 0 1 94.4 5" color="#000" fill="#fff"/>
    ///   <path d="M76.44 25.13a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M76.44 25.13a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="#fff"/>
    ///   <path d="M27.78 24.9a1.1 1.1 0 0 1-1.08 1.08 1.1 1.1 0 0 1-1.1-1.08 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M27.78 24.9a1.1 1.1 0 0 1-1.08 1.08 1.1 1.1 0 0 1-1.1-1.08 1.1 1.1 0 0 1 1.1-1.1 1.1 1.1 0 0 1 1.08 1.1" color="#000" fill="#fff"/>
    ///   <path d="M45.4 5.14a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.1-1.1 1.1 1.1 0 0 1 1.1-1.08 1.1 1.1 0 0 1 1.1 1.08" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M45.4 5.14a1.1 1.1 0 0 1-1.08 1.1 1.1 1.1 0 0 1-1.1-1.1 1.1 1.1 0 0 1 1.1-1.08 1.1 1.1 0 0 1 1.1 1.08" color="#000" fill="#fff"/>
    ///   <path d="M28.67 13.8a1.1 1.1 0 0 1-1.1 1.08 1.1 1.1 0 0 1-1.08-1.08 1.1 1.1 0 0 1 1.08-1.1 1.1 1.1 0 0 1 1.1 1.1" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M28.67 13.8a1.1 1.1 0 0 1-1.1 1.08 1.1 1.1 0 0 1-1.08-1.08 1.1 1.1 0 0 1 1.08-1.1 1.1 1.1 0 0 1 1.1 1.1" color="#000" fill="#fff"/>
    ///   <path d="M35 7.32a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.1A1.1 1.1 0 0 1 35 7.33" color="#000" fill="none" stroke="#030303" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M35 7.32a1.1 1.1 0 0 1-1.1 1.1 1.1 1.1 0 0 1-1.08-1.1 1.1 1.1 0 0 1 1.1-1.1A1.1 1.1 0 0 1 35 7.33" color="#000" fill="#fff"/>
    ///   <text style="line-height:6.61458302px" x="35.74" y="284.49" font-size="5.29" font-family="Sans" letter-spacing="0" word-spacing="0" fill="#b3b3b3" stroke-width=".26" transform="translate(19.595 -267)">
    ///     <tspan x="35.74" y="284.49" font-size="10.58">→</tspan>
    ///   </text>
    /// </svg>
    ///
    /// The tolerance value controls the maximum distance between the curved input
    /// segments and their polyline approximations. (In technical terms, this is the
    /// Hausdorff distance). The algorithm attempts to bound this distance between
    /// by `tolerance` but this is not absolutely guaranteed. The appropriate value
    /// depends on the use, but for antialiasted rendering, a value of 0.25 has been
    /// determined to give good results. The number of segments tends to scale as the
    /// inverse square root of tolerance.
    ///
    /// <svg viewBox="0 0 47.5 13.2" height="100" width="350" xmlns="http://www.w3.org/2000/svg">
    ///   <path d="M-2.44 9.53c16.27-8.5 39.68-7.93 52.13 1.9" fill="none" stroke="#dde9af" stroke-width="4.6"/>
    ///   <path d="M-1.97 9.3C14.28 1.03 37.36 1.7 49.7 11.4" fill="none" stroke="#00d400" stroke-width=".57" stroke-linecap="round" stroke-dasharray="4.6, 2.291434"/>
    ///   <path d="M-1.94 10.46L6.2 6.08l28.32-1.4 15.17 6.74" fill="none" stroke="#000" stroke-width=".6"/>
    ///   <path d="M6.83 6.57a.9.9 0 0 1-1.25.15.9.9 0 0 1-.15-1.25.9.9 0 0 1 1.25-.15.9.9 0 0 1 .15 1.25" color="#000" stroke="#000" stroke-width=".57" stroke-linecap="round" stroke-opacity=".5"/>
    ///   <path d="M35.35 5.3a.9.9 0 0 1-1.25.15.9.9 0 0 1-.15-1.25.9.9 0 0 1 1.25-.15.9.9 0 0 1 .15 1.24" color="#000" stroke="#000" stroke-width=".6" stroke-opacity=".5"/>
    ///   <g fill="none" stroke="#ff7f2a" stroke-width=".26">
    ///     <path d="M20.4 3.8l.1 1.83M19.9 4.28l.48-.56.57.52M21.02 5.18l-.5.56-.6-.53" stroke-width=".2978872"/>
    ///   </g>
    /// </svg>
    ///
    /// The callback will be called in order with each element of the generated
    /// path. Because the result is made of polylines, these will be straight-line
    /// path elements only, no curves.
    ///
    /// This algorithm is based on the blog post [Flattening quadratic Béziers]
    /// but with some refinements. For one, there is a more careful approximation
    /// at cusps. For two, the algorithm is extended to work with cubic Béziers
    /// as well, by first subdividing into quadratics and then computing the
    /// subdivision of each quadratic. However, as a clever trick, these quadratics
    /// are subdivided fractionally, and their endpoints are not included.
    ///
    /// TODO: write a paper explaining this in more detail.
    ///
    /// Note: the [`flatten`](fn.flatten.html) function provides the same
    /// functionality but works with slices and other [`PathEl`] iterators.
    ///
    /// [Flattening quadratic Béziers]: https://raphlinus.github.io/graphics/curves/2019/12/23/flatten-quadbez.html
    /// [`PathEl`]: enum.PathEl.html
    pub fn flatten(&self, tolerance: f64, callback: impl FnMut(PathEl)) {
        flatten(self, tolerance, callback);
    }

    // TODO: expose as pub method? Maybe should be a trait so slice.segments() works?
    fn segments_of_slice(slice: &[PathEl]) -> BezPathSegs {
        let first = match slice.get(0) {
            Some(PathEl::MoveTo(p)) => *p,
            Some(PathEl::LineTo(p)) => *p,
            Some(PathEl::QuadTo(_, p2)) => *p2,
            Some(PathEl::CurveTo(_, _, p3)) => *p3,
            Some(PathEl::ClosePath) => panic!("Can't start a segment on a ClosePath"),
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

impl FromIterator<PathEl> for BezPath {
    fn from_iter<T: IntoIterator<Item = PathEl>>(iter: T) -> Self {
        let el_vec: Vec<_> = iter.into_iter().collect();
        BezPath::from_vec(el_vec)
    }
}

/// Allow iteration over references to `BezPath`.
///
/// Note: the semantics are slightly different than simply iterating over the
/// slice, as it returns `PathEl` items, rather than references.
impl<'a> IntoIterator for &'a BezPath {
    type Item = PathEl;
    type IntoIter = std::iter::Cloned<std::slice::Iter<'a, PathEl>>;

    fn into_iter(self) -> Self::IntoIter {
        self.elements().iter().cloned()
    }
}

impl IntoIterator for BezPath {
    type Item = PathEl;
    type IntoIter = std::vec::IntoIter<PathEl>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl Extend<PathEl> for BezPath {
    fn extend<I: IntoIterator<Item = PathEl>>(&mut self, iter: I) {
        self.0.extend(iter);
    }
}

/// Proportion of tolerance budget that goes to cubic to quadratic conversion.
const TO_QUAD_TOL: f64 = 0.1;

/// Flatten the path, invoking the callback repeatedly.
///
/// See [`BezPath::flatten`](struct.BezPath.html#method.flatten) for more discussion.
/// This signature is a bit more general, allowing flattening of `&[PathEl]` slices
/// and other iterators yielding `PathEl`.
pub fn flatten(
    path: impl IntoIterator<Item = PathEl>,
    tolerance: f64,
    mut callback: impl FnMut(PathEl),
) {
    let sqrt_tol = tolerance.sqrt();
    let mut last_pt = None;
    let mut quad_buf = Vec::new();
    for el in path {
        match el {
            PathEl::MoveTo(p) => {
                last_pt = Some(p);
                callback(PathEl::MoveTo(p));
            }
            PathEl::LineTo(p) => {
                last_pt = Some(p);
                callback(PathEl::LineTo(p));
            }
            PathEl::QuadTo(p1, p2) => {
                if let Some(p0) = last_pt {
                    let q = QuadBez::new(p0, p1, p2);
                    let params = q.estimate_subdiv(sqrt_tol);
                    let n = ((0.5 * params.val / sqrt_tol).ceil() as usize).max(1);
                    let step = 1.0 / (n as f64);
                    for i in 1..(n - 1) {
                        let u = (i as f64) * step;
                        let t = q.determine_subdiv_t(&params, u);
                        let p = q.eval(t);
                        callback(PathEl::LineTo(p));
                    }
                    callback(PathEl::LineTo(p2));
                }
                last_pt = Some(p2);
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if let Some(p0) = last_pt {
                    let c = CubicBez::new(p0, p1, p2, p3);

                    // Subdivide into quadratics, and estimate the number of
                    // subdivisions required for each, summing to arrive at an
                    // estimate for the number of subdivisions for the cubic.
                    // Also retain these parameters for later.
                    let iter = c.to_quads(tolerance * TO_QUAD_TOL);
                    quad_buf.clear();
                    quad_buf.reserve(iter.size_hint().0);
                    let sqrt_remain_tol = sqrt_tol * (1.0 - TO_QUAD_TOL).sqrt();
                    let mut sum = 0.0;
                    for (_, _, q) in iter {
                        let params = q.estimate_subdiv(sqrt_remain_tol);
                        sum += params.val;
                        quad_buf.push((q, params));
                    }
                    let n = ((0.5 * sum / sqrt_remain_tol).ceil() as usize).max(1);

                    // Iterate through the quadratics, outputting the points of
                    // subdivisions that fall within that quadratic.
                    let step = sum / (n as f64);
                    let mut i = 1;
                    let mut val_sum = 0.0;
                    for (q, params) in &quad_buf {
                        let mut target = (i as f64) * step;
                        let recip_val = params.val.recip();
                        while target < val_sum + params.val {
                            let u = (target - val_sum) * recip_val;
                            let t = q.determine_subdiv_t(&params, u);
                            let p = q.eval(t);
                            callback(PathEl::LineTo(p));
                            i += 1;
                            if i == n + 1 {
                                break;
                            }
                            target = (i as f64) * step;
                        }
                        val_sum += params.val;
                    }
                    callback(PathEl::LineTo(p3));
                }
                last_pt = Some(p3);
            }
            PathEl::ClosePath => {
                last_pt = None;
                callback(PathEl::ClosePath);
            }
        }
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

    /// Compute intersections against a line.
    ///
    /// Returns a vector of the intersections. For each intersection, the `t` value of the
    /// segment and line are given.
    ///
    /// Note: This test is designed to be inclusive of points near the endpoints of
    /// the segment. This is so that testing a line against multiple contiguous segments
    /// of a path will be guaranteed to catch at least one of them. In such cases, use
    /// higher level logic to coalesce the hits (the `t` value may be slightly outside
    /// the range of 0..1).
    pub fn intersect_line(&self, line: Line) -> ArrayVec<[(f64, f64); 3]> {
        const EPSILON: f64 = 1e-9;
        let p0 = line.p0;
        let p1 = line.p1;
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let mut result = ArrayVec::new();
        match self {
            PathSeg::Line(l) => {
                let det = dx * (l.p1.y - l.p0.y) - dy * (l.p1.x - l.p0.x);
                if det.abs() < EPSILON {
                    // Lines are coincident (or nearly so).
                    return result;
                }
                let t = dx * (p0.y - l.p0.y) - dy * (p0.x - l.p0.x);
                // t = position on self
                let t = t / det;
                if t >= -EPSILON && t <= 1.0 + EPSILON {
                // u = position on probe line
                    let u = (l.p0.x - p0.x) * (l.p1.y - l.p0.y) - (l.p0.y - p0.y) * (l.p1.x - l.p0.x);
                    let u = u / det;
                    if u >= 0.0 && u <= 1.0 {
                        result.push((t, u));
                    }
                }
            }
            PathSeg::Quad(q) => {
                // The basic technique here is to determine x and y as a quadratic polynomial
                // as a function of t. Then plug those values into the line equation for the
                // probe line (giving a sort of signed distance from the probe line) and solve
                // that for t.
                let (px0, px1, px2) = quadratic_bez_coefs(q.p0.x, q.p1.x, q.p2.x);
                let (py0, py1, py2) = quadratic_bez_coefs(q.p0.y, q.p1.y, q.p2.y);
                let c0 = dy * (px0 - p0.x) - dx * (py0 - p0.y);
                let c1 = dy * px1 - dx * py1;
                let c2 = dy * px2 - dx * py2;
                let invlen2 = (dx * dx + dy * dy).recip();
                for t in crate::common::solve_quadratic(c0, c1, c2) {
                    if t >= -EPSILON && t <= 1.0 + EPSILON {
                        let x = px0 + t * px1 + t * t * px2;
                        let y = py0 + t * py1 + t * t * py2;
                        let u = ((x - p0.x) * dx + (y - p0.y) * dy) * invlen2;
                        if u >= 0.0 && u <= 1.0 {
                            result.push((t, u));
                        }
                    }
                }
            }
            PathSeg::Cubic(c) => {
                // Same technique as above, but cubic polynomial.
                let (px0, px1, px2, px3) = cubic_bez_coefs(c.p0.x, c.p1.x, c.p2.x, c.p3.x);
                let (py0, py1, py2, py3) = cubic_bez_coefs(c.p0.y, c.p1.y, c.p2.y, c.p3.y);
                let c0 = dy * (px0 - p0.x) - dx * (py0 - p0.y);
                let c1 = dy * px1 - dx * py1;
                let c2 = dy * px2 - dx * py2;
                let c3 = dy * px3 - dx * py3;
                let invlen2 = (dx * dx + dy * dy).recip();
                for t in crate::common::solve_cubic(c0, c1, c2, c3) {
                    if t >= -EPSILON && t <= 1.0 + EPSILON {
                        let x = px0 + t * px1 + t * t * px2 + t * t * t * px3;
                        let y = py0 + t * py1 + t * t * py2 + t * t * t * py3;
                        let u = ((x - p0.x) * dx + (y - p0.y) * dy) * invlen2;
                        if u >= 0.0 && u <= 1.0 {
                            result.push((t, u));
                        }
                    }
                }
            }
        }
        result
    }
}

// Return polynomial coefficients given cubic bezier coordinates.
fn quadratic_bez_coefs(x0: f64, x1: f64, x2: f64) -> (f64, f64, f64) {
    let p0 = x0;
    let p1 = 2.0 * x1 - 2.0 * x0;
    let p2 = x2 - 2.0 * x1 + x0;
    (p0, p1, p2)
}

// Return polynomial coefficients given cubic bezier coordinates.
fn cubic_bez_coefs(x0: f64, x1: f64, x2: f64, x3: f64) -> (f64, f64, f64, f64) {
    let p0 = x0;
    let p1 = 3.0 * x1 - 3.0 * x0;
    let p2 = 3.0 * x2 - 6.0 * x1 + 3.0 * x0;
    let p3 = x3 - 3.0 * x2 + 3.0 * x1 - x0;
    (p0, p1, p2, p3)
}

impl From<CubicBez> for PathSeg {
    fn from(cubic_bez: CubicBez) -> PathSeg {
        PathSeg::Cubic(cubic_bez)
    }
}

impl From<Line> for PathSeg {
    fn from(line: Line) -> PathSeg {
        PathSeg::Line(line)
    }
}

impl From<QuadBez> for PathSeg {
    fn from(quad_bez: QuadBez) -> PathSeg {
        PathSeg::Quad(quad_bez)
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

/// Implements [`Shape`] for a slice of [`PathEl`], provided that the first element of the slice is
/// not a `PathEl::ClosePath`. If it is, several of these functions will panic.
///
/// If the slice starts with `LineTo`, `QuadTo`, or `CurveTo`, it will be treated as a `MoveTo`.
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

#[cfg(test)]
mod tests {
    use super::{CubicBez, Line, PathSeg, QuadBez};
    
    fn assert_approx_eq(x: f64, y: f64) {
        assert!((x - y).abs() < 1e-8, "{} != {}", x, y);
    }

    #[test]
    fn test_intersect_line() {
        let h_line = Line::new((0.0, 0.0), (100.0, 0.0));
        let v_line = Line::new((10.0, -10.0), (10.0, 10.0));
        let (u, t) = PathSeg::Line(h_line).intersect_line(v_line)[0];
        assert_approx_eq(u, 0.1);
        assert_approx_eq(t, 0.5);

        let v_line = Line::new((-10.0, -10.0), (-10.0, 10.0));
        assert!(PathSeg::Line(h_line).intersect_line(v_line).is_empty());

        let v_line = Line::new((10.0, 10.0), (10.0, 20.0));
        assert!(PathSeg::Line(h_line).intersect_line(v_line).is_empty());
    }

    #[test]
    fn test_intersect_qad() {
        let q = QuadBez::new((0.0, -10.0), (10.0, 20.0), (20.0, -10.0));
        let v_line = Line::new((10.0, -10.0), (10.0, 10.0));
        assert_eq!(PathSeg::Quad(q).intersect_line(v_line).len(), 1);
        let (u, t) = PathSeg::Quad(q).intersect_line(v_line)[0];
        assert_approx_eq(u, 0.5);
        assert_approx_eq(t, 0.75);

        let h_line = Line::new((0.0, 0.0), (100.0, 0.0));
        assert_eq!(PathSeg::Quad(q).intersect_line(h_line).len(), 2);
    }

    #[test]
    fn test_intersect_cubic() {
        let c = CubicBez::new((0.0, -10.0), (10.0, 20.0), (20.0, -20.0), (30.0, 10.0));
        let v_line = Line::new((10.0, -10.0), (10.0, 10.0));
        assert_eq!(PathSeg::Cubic(c).intersect_line(v_line).len(), 1);
        let (u, t) = PathSeg::Cubic(c).intersect_line(v_line)[0];
        assert_approx_eq(u, 0.333333333);
        assert_approx_eq(t, 0.592592592);

        let h_line = Line::new((0.0, 0.0), (100.0, 0.0));
        assert_eq!(PathSeg::Cubic(c).intersect_line(h_line).len(), 3);
    }
}
