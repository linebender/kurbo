//! Experimental path operations module.

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BinaryHeap},
};

use crate::{BezPath, Line, PathSeg, Point};

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
    queue: PriQueue,
}

struct TotalF64(f64);

#[derive(Default)]
struct PriQueue(BinaryHeap<TotalF64>);

#[derive(Default)]
struct HorizLines(BTreeMap<TotalF64, i32>);

// rename?
struct Runner {
    active: ActiveList,
    lines: Vec<Line>,
    line_ix: usize,
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
        self.orig.p0.y == y || self.orig.p1.y == y
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
    fn insert_line(&mut self, l: Line) {
        if l.p0.y == l.p1.y {
            self.horiz.add_line(l.p0.x, l.p1.x);
            return;
        }
        if let Some(new_seg) = LineSeg::from_line(l) {
            let y = new_seg.line.p0.y;
            // find insertion point (should be binary search)
            let mut i = 0;
            while i < self.segs.len() {
                let seg = &self.segs[i];
                if y == seg.line.p0.y && l.p0.x < seg.line.p0.x {
                    break;
                }
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
            // note: can skip if s0.y0 != y && s1.y0 != y
            let x0 = s[0].x_for_y(y);
            let x1 = s[1].x_for_y(y);
            // Question: should be line or line.p0 to orig.p1?
            let o0 = orient_point_line_eqn(Point::new(x1, y), s[0].line);
            let o1 = orient_point_line_eqn(Point::new(x0, y), s[1].line);
            if (s[0].line.p0.y == y) != (s[1].line.p0.y == y) {
                if o0 != Orientation::Right && !s[0].y_at_endpoint(y) {
                    s[0].line.p0 = Point::new(x0, y);
                    i = k;
                } else {
                    i = 0;
                }
                if o1 != Orientation::Left && !s[1].y_at_endpoint(y) {
                    s[1].line.p0 = Point::new(x1, y);
                }
            }
            if s[0].line.p0.y == y && s[1].line.p0.y == y && s[0].line.p0.x > s[1].line.p1.x {
                // maybe choose which on basis of more vertical?
                if o0 == Orientation::Ambiguous && !s[0].y_at_endpoint(y) {
                    s[0].line.p0.x = s[1].line.p0.x;
                } else if o1 == Orientation::Ambiguous && !s[1].y_at_endpoint(y) {
                    s[1].line.p0.x = s[0].line.p0.x;
                } else {
                    // need to add horizontal whisker
                    let i = if o0 == Orientation::Ambiguous { 0 } else { 1 };
                    let x = s[1 - i].line.p0.x;
                    let mut si = &mut s[i];
                    self.horiz.add_delta(si.line.p0.x, si.winding);
                    self.horiz.add_delta(x, -si.winding);
                    si.line.p0.x = x;
                }
                // Discussion: do we ever need to actually swap?
                // We do if it's an intersection point
            }
            j = j.max(k + 1);
        }
    }

    fn delete_bottom(&mut self, y: f64) {
        self.segs.retain(|seg| seg.orig.p1.y != y);
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
                self.segs[i].line.p1 = p;
                self.segs[j].line.p1 = p;
            }
        }
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
        // Note order is reversed.
        Some(self.0.total_cmp(&other.0))
    }
}

impl Ord for TotalF64 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl PriQueue {
    fn push(&mut self, y: f64) {
        self.0.push(TotalF64(-y));
    }

    /// Return the least value in the priority queue.
    fn pop(&mut self) -> Option<f64> {
        self.0.pop().map(|t| -t.0)
    }

    fn pop_dedup(&mut self) -> Option<f64> {
        if let Some(x) = self.0.pop() {
            while let Some(y) = self.0.peek() {
                if x != *y {
                    break;
                }
                self.0.pop();
            }
            Some(-x.0)
        } else {
            None
        }
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
        let mut lines = p
            .segments()
            .map(|seg| {
                if let PathSeg::Line(l) = seg {
                    l
                } else {
                    panic!("only works on polygons");
                }
            })
            .collect::<Vec<_>>();
        lines.sort_by(|l1, l2| l1.p0.y.min(l1.p1.y).total_cmp(&l2.p0.y.min(l2.p1.y)));
        let mut queue = PriQueue::default();
        for l in &lines {
            queue.push(l.p0.y);
            queue.push(l.p1.y);
        }
        let active = ActiveList {
            horiz: HorizLines::default(),
            segs: Vec::new(),
            queue,
        };
        Runner {
            active,
            lines,
            line_ix: 0,
        }
    }

    fn step(&mut self) -> bool {
        if let Some(y) = self.active.queue.pop_dedup() {
            while self.line_ix < self.lines.len() {
                let line = &self.lines[self.line_ix];
                if line.p0.y.min(line.p1.y) != y {
                    break;
                }
                self.active.insert_line(*line);
                self.line_ix += 1;
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
