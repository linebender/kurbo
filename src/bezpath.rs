//! Bézier paths (up to cubic).

use std::ops::{Mul, Range};
use std::io::Write;

use crate::{Affine, CubicBez, Line, ParamCurve, ParamCurveArea, ParamCurveArclen,
    ParamCurveNearest, QuadBez, Vec2};

/// A path that can Bézier segments up to cubic, possibly with multiple subpaths.
#[derive(Clone, Default)]
pub struct BezPath(Vec<PathEl>);

/// The element of a Bézier path.
///
/// A valid path has `Moveto` at the beginning of each subpath.
#[derive(Clone, Copy)]
pub enum PathEl {
    Moveto(Vec2),
    Lineto(Vec2),
    Quadto(Vec2, Vec2),
    Curveto(Vec2, Vec2, Vec2),
    Closepath,
}

/// A segment of a Bézier path.
#[derive(Clone, Copy)]
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

    /// Push a generic path element onto the path.
    pub fn push(&mut self, el: PathEl) {
        self.0.push(el)
    }

    /// Push a moveto element onto the path.
    pub fn moveto<V: Into<Vec2>>(&mut self, p: V) {
        self.push(PathEl::Moveto(p.into()));
    }

    /// Push a lineto element onto the path.
    pub fn lineto<V: Into<Vec2>>(&mut self, p: V) {
        self.push(PathEl::Lineto(p.into()));
    }

    /// Push a quadto element onto the path.
    pub fn quadto<V: Into<Vec2>>(&mut self, p1: V, p2: V) {
        self.push(PathEl::Quadto(p1.into(), p2.into()));
    }

    /// Push a curveto element onto the path.
    pub fn curveto<V: Into<Vec2>>(&mut self, p1: V, p2: V, p3: V) {
        self.push(PathEl::Curveto(p1.into(), p2.into(), p3.into()));
    }

    /// Push a closepath element onto the path.
    pub fn closepath(&mut self) {
        self.push(PathEl::Closepath);
    }

    /// Get the path elements.
    pub fn elements(&self) -> &[PathEl] {
        &self.0
    }

    /// Iterate over the path segments.
    ///
    /// The iterator returns the index within the path and the segment.
    pub fn segments<'a>(&'a self) -> impl Iterator<Item = (usize, PathSeg)> + 'a {
        BezPathSegs { c: &self.0, ix: 0, start: None, last: None }
    }

    /// Get the segment at the given index.
    pub fn get_seg(&self, ix: usize) -> Option<PathSeg> {
        if ix == 0 || ix >= self.0.len() {
            return None;
        }
        let last = match self.0[ix - 1] {
            PathEl::Moveto(p) => p,
            PathEl::Lineto(p) => p,
            PathEl::Quadto(_, p2) => p2,
            PathEl::Curveto(_, _ , p3) => p3,
            _ => return None,
        };
        match self.0[ix] {
            PathEl::Lineto(p) => Some(PathSeg::Line(Line::new(last, p))),
            PathEl::Quadto(p1, p2) => Some(PathSeg::Quad(QuadBez::new(last, p1, p2))),
            PathEl::Curveto(p1, p2, p3) => Some(PathSeg::Cubic(CubicBez::new(last, p1, p2, p3))),
            PathEl::Closepath => {
                self.0[..ix].iter().rev().find_map(|el| match *el {
                    PathEl::Moveto(start) => Some(PathSeg::Line(Line::new(last, start))),
                    _ => None,
                })
            }
            _ => None
        }
    }

    /// Returns `true` if the path contains no segments.
    pub fn is_empty(&self) -> bool {
        !self.0.iter().any(|el|
            match *el {
                PathEl::Lineto(..) | PathEl::Quadto(..) | PathEl::Curveto(..) => true,
                _ => false,
            }
        )
    }

    /// Convert the path to an SVG path string representation.
    ///
    /// The current implementation doesn't take any special care to produce a
    /// short string (reducing precision, using relative movement).
    pub fn to_svg(&self) -> String {
        let mut result = Vec::new();
        for el in &self.0 {
            match *el {
                PathEl::Moveto(p) => write!(result, "M{} {}", p.x, p.y).unwrap(),
                PathEl::Lineto(p) => write!(result, "L{} {}", p.x, p.y).unwrap(),
                PathEl::Quadto(p1, p2) => write!(result, "Q{} {} {} {}",
                    p1.x, p1.y, p2.x, p2.y).unwrap(),
                PathEl::Curveto(p1, p2, p3) => write!(result, "C{} {} {} {} {} {}",
                    p1.x, p1.y, p2.x, p2.y, p3.x, p3.y).unwrap(),
                PathEl::Closepath => write!(result, "Z").unwrap(),
            }
        }
        String::from_utf8(result).unwrap()
    }

    /// Apply an affine transform to the path.
    pub fn apply_affine(&mut self, affine: Affine) {
        for el in self.0.iter_mut() {
            *el = affine * (*el);
        }
    }

    /// Compute the total arclength.
    ///
    /// Here, `accuracy` specifies the accuracy for each Bézier segment. At worst,
    /// the total error is `accuracy` times the number of Bézier segments.
    ///
    /// Note: this is one of the methods that could be implemented on an iterator
    /// of `PathEl`, to save allocation.
    pub fn arclen(&self, accuracy: f64) -> f64 {
        self.segments().map(|(_, seg)| seg.arclen(accuracy)).sum()
    }

    /// Compute the total signed area.
    pub fn area(&self) -> f64 {
        self.segments().map(|(_, seg)| seg.signed_area()).sum()
    }

    /// Find the nearest point.
    ///
    /// Panics if path is empty or invalid.
    ///
    /// Returns the index of the element, the parameter within that segment, and
    /// the square of the distance to the point.
    pub fn nearest(&self, p: Vec2, accuracy: f64) -> (usize, f64, f64) {
        let mut best = None;
        for (ix, seg) in self.segments() {
            let (t, r) = seg.nearest(p, accuracy);
            if best.map(|(_, _, r_best)| r < r_best).unwrap_or(true) {
                best = Some((ix, t, r));
            }
        }
        best.unwrap()
    }
}

impl Mul<PathEl> for Affine {
    type Output = PathEl;

    fn mul(self, other: PathEl) -> PathEl {
        match other {
            PathEl::Moveto(p) => PathEl::Moveto(self * p),
            PathEl::Lineto(p) => PathEl::Lineto(self * p),
            PathEl::Quadto(p1, p2) => PathEl::Quadto(self * p1, self * p2),
            PathEl::Curveto(p1, p2, p3) => PathEl::Curveto(self * p1, self * p2, self * p3),
            PathEl::Closepath => PathEl::Closepath,
        }
    }
}

impl Mul<BezPath> for Affine {
    type Output = BezPath;

    fn mul(self, other: BezPath) -> BezPath {
        BezPath(other.0.iter().map(|&el| self * el).collect())
    }
}

struct BezPathSegs<'a> {
    c: &'a [PathEl],
    ix: usize,
    start: Option<Vec2>,
    last: Option<Vec2>,
}

impl<'a> Iterator for BezPathSegs<'a> {
    type Item = (usize, PathSeg);

    fn next(&mut self) -> Option<(usize, PathSeg)> {
        while self.ix < self.c.len() {
            let ix = self.ix;
            let el = self.c[ix];
            self.ix += 1;
            match el {
                PathEl::Moveto(p) => {
                    self.start = Some(p);
                    self.last = Some(p);
                }
                PathEl::Lineto(p) => {
                    let seg = PathSeg::Line(Line::new(self.last.unwrap(), p));
                    self.last = Some(p);
                    return Some((ix, seg));
                }
                PathEl::Quadto(p1, p2) => {
                    let seg = PathSeg::Quad(QuadBez::new(self.last.unwrap(), p1, p2));
                    self.last = Some(p2);
                    return Some((ix, seg));
                }
                PathEl::Curveto(p1, p2, p3) => {
                    let seg = PathSeg::Cubic(CubicBez::new(self.last.unwrap(), p1, p2, p3));
                    self.last = Some(p3);
                    return Some((ix, seg));
                }
                PathEl::Closepath => {
                    let last = self.last.take();
                    let start = self.start.take();
                    if last != start {
                        let seg = PathSeg::Line(Line::new(last.unwrap(), start.unwrap()));
                        return Some((ix, seg));
                    }
                }
            }
        }
        None
    }
}

impl ParamCurve for PathSeg {
    fn eval(&self, t: f64) -> Vec2 {
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
    fn nearest(&self, p: Vec2, accuracy: f64) -> (f64, f64) {
        match *self {
            PathSeg::Line(line) => line.nearest(p, accuracy),
            PathSeg::Quad(quad) => quad.nearest(p, accuracy),
            PathSeg::Cubic(cubic) => cubic.nearest(p, accuracy),
        }        
    }
}
