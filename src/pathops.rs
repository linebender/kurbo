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
}

// Return line segments sorted by top, horizontal lines removed.
fn path_to_line_segs(p: &BezPath) -> Vec<LineSeg> {
    let mut segs = p.segments()
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
