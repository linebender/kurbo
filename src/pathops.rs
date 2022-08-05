//! Experimental path operations module.

use std::cmp::Ordering;

use crate::{BezPath, Line, PathSeg, Point};

struct LineSeg {
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

// TODO move to LineSeg method
fn orient_point_line(p: Point, l: Line) -> Orientation {
    if p.y == l.p0.y {
        return orient_float(p.x, l.p0.x);
    }
    if p.y == l.p1.y {
        return orient_float(p.x, l.p1.x);
    }
    if p.x < l.p0.x.min(l.p1.x) {
        return Orientation::Left;
    }
    if p.x > l.p0.x.max(l.p1.x) {
        return Orientation::Right;
    }
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
    fn from_line(l: Line) -> Option<Self> {
        if l.p1.y > l.p0.y {
            Some(LineSeg {
                line: l,
                winding: 1,
            })
        } else if l.p1.y < l.p0.y {
            Some(LineSeg {
                line: Line::new(l.p1, l.p0),
                winding: -1,
            })
        } else {
            None
        }
    }

    fn order_by_top(&self, other: &LineSeg) -> Ordering {
        self.line.p0.y.total_cmp(&other.line.p0.y)
    }

    fn solve_for_y(&self, y: f64) -> f64 {
        solve_line_for_y(self.line, y)
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
        let El { mut x0, mut x1, i} = *el;
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
        z.insert(j, El { x0, x1, i});
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
