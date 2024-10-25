//! Alternate prototype of pathops.

use std::{cmp::Ordering, collections::{BinaryHeap, BTreeMap}};

use crate::{BezPath, Line, PathEl, PathSeg};

struct LineInput {
    line: Line,
    winding: i32,
}

#[derive(Default)]
struct PathOps {
    queue: BinaryHeap<PriEl>,
}

struct PriEl {
    y: f64,
    seg: Option<LineInput>,
}

struct TotalF64(f64);

#[derive(Default)]
struct HorizLines(BTreeMap<TotalF64, i32>);

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

impl LineInput {
    fn from_line(line: Line) -> LineInput {
        if (line.p1.y, line.p1.x) >= (line.p0.y, line.p0.x) {
            LineInput { line, winding: 1 }
        } else {
            LineInput {
                line: Line::new(line.p1, line.p0),
                winding: 0,
            }
        }
    }
}

impl PathOps {
    fn new(p: &BezPath) -> PathOps {
        let mut pathops = PathOps::default();
        for seg in p.segments() {
            if let PathSeg::Line(l) = seg {
                let line_inp = LineInput::from_line(l);
                let y0 = line_inp.line.p0.y;
                let y1 = line_inp.line.p1.y;
                pathops.queue.push(PriEl {
                    y: y0,
                    seg: Some(line_inp),
                });
                pathops.queue.push(PriEl { y: y1, seg: None });
            }
        }
        pathops
    }
}
