//! Experimental path operations module.

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BinaryHeap},
};

use crate::{BezPath, Line, ParamCurve, PathSeg, Point};

#[derive(Clone, Copy)]
struct LineSeg {
    orig: Line,
    line: Line,
    // +1 is increasing y
    winding: i32,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Orientation {
    Left,
    Atop,
    Right,
    Ambiguous,
}

#[derive(Default)]
struct ActiveList {
    segs: Vec<LineSeg>,
    horiz: HorizLines,
    queue: BinaryHeap<PriEl>,
}

struct TotalF64(f64);

struct PriEl {
    y: f64,
    seg: Option<LineSeg>,
}

#[derive(Default)]
struct HorizLines(BTreeMap<TotalF64, i32>);

// rename?
#[derive(Default)]
struct Runner {
    active: ActiveList,
}

impl Orientation {
    fn ordered(self) -> bool {
        self == Orientation::Left || self == Orientation::Right
    }
}

fn orient_float(a: f64, b: f64) -> Orientation {
    if a < b {
        Orientation::Left
    } else if a > b {
        Orientation::Right
    } else if a == b {
        Orientation::Atop
    } else {
        Orientation::Ambiguous
    }
}

// This should probably be a parameter.
const EPSILON: f64 = 1e-9;

// epsilon should be param?
fn orient_point_line(p: Point, l: Line) -> Orientation {
    if p.y == l.p0.y {
        return orient_float(p.x, l.p0.x);
    }
    if p.y == l.p1.y {
        return orient_float(p.x, l.p1.x);
    }
    orient_point_line_eqn(p, l)
}

// Orient point wrt infinite line
fn orient_point_line_eqn(p: Point, l: Line) -> Orientation {
    // line equation ax + by = c
    let a = l.p1.y - l.p0.y;
    let b = l.p0.x - l.p1.x;
    let c = a * l.p0.x + b * l.p0.y;
    // This is signed distance from line * length of line
    let orient = a * p.x + b * p.y - c;
    if orient.powi(2) < (a * a + b * b) * EPSILON.powi(2) {
        return Orientation::Ambiguous;
    }
    if orient < 0.0 {
        Orientation::Left
    } else {
        Orientation::Right
    }
}

// Orientation wrt line, so that if point is within epsilon, always ambiguous.
fn orient_point_line_eps(p: Point, l: Line, epsilon: f64) -> Orientation {
    let d = l.p1 - l.p0;
    let dotp = d.dot(p - l.p0);
    let d_squared = d.dot(d);
    let line_pt = if dotp <= 0.0 {
        l.p0
    } else if dotp >= d_squared {
        l.p1
    } else {
        l.eval(dotp / d_squared)
    };
    if p.distance_squared(line_pt) < epsilon * epsilon {
        Orientation::Ambiguous
    } else if p.x < line_pt.x {
        Orientation::Left
    } else {
        Orientation::Right
    }
}

fn solve_line_for_y(l: Line, y: f64) -> f64 {
    l.p0.x + (y - l.p0.y) * (l.p1.x - l.p0.x) / (l.p1.y - l.p0.y)
}

impl LineSeg {
    fn is_horiz(&self) -> bool {
        self.line.p0.y == self.line.p1.y
    }

    fn from_line(l: Line) -> Option<Self> {
        if l.p1.y > l.p0.y || (l.p1.y == l.p0.y && l.p1.x > l.p0.x) {
            Some(LineSeg {
                orig: l,
                line: l,
                winding: 1,
            })
        } else if l.p1.y < l.p0.y || (l.p1.y == l.p0.y && l.p1.x < l.p0.x) {
            let line = Line::new(l.p1, l.p0);
            Some(LineSeg {
                orig: line,
                line,
                winding: -1,
            })
        } else {
            None
        }
    }

    fn order_by_top(&self, other: &LineSeg) -> Ordering {
        self.line.p0.y.total_cmp(&other.line.p0.y)
    }

    fn x_for_y(&self, y: f64) -> f64 {
        if self.line.p0.y == y {
            self.line.p0.x
        } else if self.line.p1.y == y {
            self.line.p1.x
        } else {
            // Discussion: is this right?
            solve_line_for_y(self.orig, y)
        }
    }

    fn y_at_endpoint(&self, y: f64) -> bool {
        self.line.p0.y == y || self.line.p1.y == y
    }
}

// Return line segments sorted by top, horizontal lines removed.
fn path_to_line_segs(p: &BezPath) -> Vec<LineSeg> {
    let mut segs = p
        .segments()
        .flat_map(|seg| {
            if let PathSeg::Line(l) = seg {
                LineSeg::from_line(l)
            } else {
                panic!("only works on polygons");
            }
        })
        .collect::<Vec<_>>();
    segs.sort_by(LineSeg::order_by_top);
    segs
}

#[derive(Clone, Copy)]
struct El {
    x0: f64,
    x1: f64,
    i: usize,
}

impl El {
    fn from_line(l: Line, y0: f64, y1: f64, i: usize) -> Self {
        let x0 = solve_line_for_y(l, y0);
        let x1 = solve_line_for_y(l, y1);
        El { x0, x1, i }
    }

    fn line(&self, y0: f64, y1: f64) -> Line {
        Line::new((self.x0, y0), (self.x1, y1))
    }
}

fn do_slice(active: &[LineSeg], y0: f64, y1: f64) {
    let mut els = active
        .iter()
        .enumerate()
        .map(|(i, s)| El::from_line(s.line, y0, y1, i))
        .collect::<Vec<_>>();
    els.sort_by(|a, b| a.x0.total_cmp(&b.x0));
    let mut z: Vec<El> = Vec::with_capacity(els.len());
    for el in &els {
        let El { mut x0, mut x1, i } = *el;
        let mut j = z.len();
        while j > 0 && z[j].x1 > x1 {
            let prev = &mut z[j - 1];
            // TODO: this should be original
            let prev_l = prev.line(y0, y1);
            let l = Line::new((x0, y0), (x1, y1));
            // test if top is oriented
            let top_prev_l = orient_point_line(Point::new(prev.x0, y0), l);
            let top_l_prev = orient_point_line(Point::new(x0, y0), prev_l);
            let bot_prev_l = orient_point_line(Point::new(prev.x1, y1), l);
            let bot_l_prev = orient_point_line(Point::new(x1, y1), prev_l);
            if !top_prev_l.ordered() {
                x0 = prev.x0;
            } else if !top_l_prev.ordered() {
                prev.x0 = x0;
            }
            if !bot_prev_l.ordered() {
                x1 = prev.x1;
                break;
            } else if !bot_l_prev.ordered() {
                prev.x1 = x1;
            }
            // Note: if they're all ordered, that's an intersection
            j -= 1;
        }
        z.insert(j, El { x0, x1, i });
    }
}

impl ActiveList {
    fn insert_seg(&mut self, new_seg: LineSeg) {
        if new_seg.line.p0.y == new_seg.line.p1.y {
            self.horiz.add_line(new_seg.line.p0.x, new_seg.line.p1.x);
            return;
        }
        let y = new_seg.line.p0.y;
        // find insertion point (should be binary search)
        let mut i = 0;
        while i < self.segs.len() {
            let seg = &self.segs[i];

            let x1 = seg.x_for_y(y);
            let o0 = orient_point_line_eqn(Point::new(x1, y), new_seg.line);
            let o1 = orient_point_line_eqn(new_seg.line.p0, seg.orig);
            if o0 == Orientation::Right && o1 == Orientation::Left {
                break;
            }
            if o0 == Orientation::Ambiguous && o1 == Orientation::Ambiguous {
                let y1 = new_seg.line.p1.y.min(seg.line.p1.y);
                let x0 = new_seg.x_for_y(y1);
                let x1 = seg.x_for_y(y1);
                let o0 = orient_point_line_eqn(Point::new(x1, y1), new_seg.line);
                let o1 = orient_point_line_eqn(Point::new(x0, y1), seg.orig);
                if o0 == Orientation::Right && o1 == Orientation::Left {
                    break;
                }
            }
            i += 1;
        }
        self.segs.insert(i, new_seg);
        if i > 0 {
            self.try_intersect_pair(i - 1, i);
        }
        if i + 1 < self.segs.len() {
            self.try_intersect_pair(i, i + 1);
        }
    }

    // Another try at inserting a segment.
    //
    // This one tries to maintain the fully-ordered invariant.
    fn insert_seg2(&mut self, seg: LineSeg) {
        let Point { x, y } = seg.line.p0;
        let mut i = 0;
        while i < self.segs.len() {
            let seg = &self.segs[i];
            if x < seg.x_for_y(y) {
                break;
            }
            i += 1;
        }
    }

    // postcondition: segments are >= top-oriented
    fn sort_at(&mut self, y: f64) {
        let mut i = 0;
        let mut j: usize = 0;
        // invariant 0..i-1 top-oriented and i..j top-oriented
        loop {
            let k = if i == 0 { j } else { i - 1 };
            if k + 1 >= self.segs.len() {
                break;
            }
            let s = &mut self.segs[k..k + 2];
            let y_end0 = s[0].y_at_endpoint(y);
            let y_end1 = s[1].y_at_endpoint(y);
            if y_end0 || y_end1 {
                let mut at0 = y_end0;
                let mut at1 = y_end1;
                let x0 = s[0].x_for_y(y);
                let x1 = s[1].x_for_y(y);
                // Question: should be line or line.p0 to orig.p1, or just orig?
                let o0 = orient_point_line_eqn(Point::new(x1, y), s[0].line);
                let o1 = orient_point_line_eqn(Point::new(x0, y), s[1].line);
                if o0 != Orientation::Right && !y_end0 {
                    s[0].line.p0 = Point::new(x0, y);
                    at0 = true;
                    i = k;
                } else {
                    i = 0;
                }
                if o1 != Orientation::Left && !y_end1 {
                    s[1].line.p0 = Point::new(x1, y);
                    at1 = true;
                }
                if at0 && at1 && x0 > x1 {
                    // maybe choose which on basis of more vertical?
                    if o0 == Orientation::Ambiguous && !y_end0 {
                        s[0].line.p0.x = x1;
                    } else if o1 == Orientation::Ambiguous && !y_end1 {
                        s[1].line.p0.x = x0;
                    } else {
                        // need to add horizontal whisker
                        let i = if o0 == Orientation::Ambiguous { 0 } else { 1 };
                        let sm = &s[1 - i];
                        let xm = if sm.line.p0.y == y {
                            sm.line.p0.x
                        } else {
                            sm.line.p1.x
                        };
                        let si = &mut s[i];
                        let xi = if si.line.p0.y == y {
                            si.line.p0.x
                        } else {
                            si.line.p1.x
                        };
                        self.horiz.add_delta(xi, si.winding);
                        self.horiz.add_delta(xm, -si.winding);
                        si.line.p0.x = xm;
                    }
                }
            }
            j = j.max(k + 1);
        }
    }

    fn delete_bottom(&mut self, y: f64) {
        let mut last = None;
        for i in 0..self.segs.len() {
            if self.segs[i].line.p1.y != y {
                if let Some(last) = last {
                    if last + 1 != i {
                        self.try_intersect_pair(last, i);
                    }
                }
                last = Some(i);
            }
        }
        self.segs.retain(|seg| seg.line.p1.y != y);
    }

    fn try_intersect_pair(&mut self, i: usize, j: usize) {
        let l0 = self.segs[i].orig;
        let l1 = self.segs[j].orig;
        let ymin = self.segs[i].line.p1.y.min(self.segs[j].line.p1.y);
        let x0 = self.segs[i].x_for_y(ymin);
        let x1 = self.segs[j].x_for_y(ymin);
        let o0 = orient_point_line_eqn(Point::new(x1, ymin), l0);
        let o1 = orient_point_line_eqn(Point::new(x0, ymin), l1);
        if o0 == Orientation::Left && o1 == Orientation::Right {
            if let Some(p) = l0.crossing_point(l1) {
                let mut seg0 = self.segs[i];
                let mut seg1 = self.segs[j];
                // Proof obligation: seg0.p0.y < p.y < seg0.p1.y, also seg1
                self.segs[i].line.p1 = p;
                self.segs[j].line.p1 = p;
                seg0.line.p0 = p;
                seg1.line.p0 = p;
                self.queue_seg(seg0);
                self.queue_seg(seg1);
            }
        }
    }

    fn queue_seg(&mut self, seg: LineSeg) {
        self.queue.push(PriEl {
            y: seg.line.p0.y,
            seg: Some(seg),
        });
    }
}

impl PartialEq for TotalF64 {
    fn eq(&self, other: &Self) -> bool {
        self.0.total_cmp(&other.0) == Ordering::Equal
    }
}

impl Eq for TotalF64 {}

impl PartialOrd for TotalF64 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.0.total_cmp(&other.0))
    }
}

impl Ord for TotalF64 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl PartialEq for PriEl {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for PriEl {}

impl PartialOrd for PriEl {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PriEl {
    fn cmp(&self, other: &Self) -> Ordering {
        other.y.total_cmp(&self.y)
    }
}

impl HorizLines {
    fn add_delta(&mut self, x: f64, winding: i32) {
        // Note: could delete when winding goes to 0, probably easier to fix later.
        *self.0.entry(TotalF64(x)).or_insert(0) += winding;
    }

    fn add_line(&mut self, x0: f64, x1: f64) {
        if x0 != x1 {
            self.add_delta(x0, 1);
            self.add_delta(x1, -1);
        }
    }
}

impl Runner {
    fn new(p: &BezPath) -> Runner {
        let mut runner = Runner::default();
        for seg in p.segments() {
            if let PathSeg::Line(l) = seg {
                if let Some(seg) = LineSeg::from_line(l) {
                    let y1 = seg.line.p1.y;
                    runner.active.queue_seg(seg);
                    runner.active.queue.push(PriEl { y: y1, seg: None });
                }
            }
        }
        runner
    }

    fn step(&mut self) -> bool {
        if let Some(mut el) = self.active.queue.pop() {
            let y = el.y;
            loop {
                if let Some(seg) = el.seg {
                    self.active.insert_seg(seg);
                }
                if self
                    .active
                    .queue
                    .peek()
                    .map(|seg| seg.y != y)
                    .unwrap_or(true)
                {
                    break;
                }
                el = self.active.queue.pop().unwrap();
            }
            self.active.sort_at(y);
            self.active.delete_bottom(y);
            true
        } else {
            false
        }
    }
}

#[test]
fn test_orientation() {
    let l = Line::new((0., 0.), (1., 2.));
    assert_eq!(orient_point_line(Point::new(0., 0.), l), Orientation::Atop);
    assert_eq!(
        orient_point_line(Point::new(-1e-12, 0.), l),
        Orientation::Left
    );
    assert_eq!(
        orient_point_line(Point::new(1e-12, 0.), l),
        Orientation::Right
    );
    assert_eq!(orient_point_line(Point::new(1., 2.), l), Orientation::Atop);
    assert_eq!(
        orient_point_line(Point::new(1. - 1e-12, 2.), l),
        Orientation::Left
    );
    assert_eq!(
        orient_point_line(Point::new(1. + 1e-12, 2.), l),
        Orientation::Right
    );
    assert_eq!(
        orient_point_line(Point::new(0.5 - 1.2e-9, 1.), l),
        Orientation::Left
    );
    assert_eq!(
        orient_point_line(Point::new(0.5 - 1.1e-9, 1.), l),
        Orientation::Ambiguous
    );
    assert_eq!(
        orient_point_line(Point::new(0.5 + 1.1e-9, 1.), l),
        Orientation::Ambiguous
    );
    assert_eq!(
        orient_point_line(Point::new(0.5 + 1.2e-9, 1.), l),
        Orientation::Right
    );
}
