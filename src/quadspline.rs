// Copyright 2021 The kurbo Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Quadratic Bézier splines.
use crate::Point;
use crate::QuadBez;

/// A quadratic Bézier spline in [B-spline](https://en.wikipedia.org/wiki/B-spline) format.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadSpline(Vec<Point>);

impl QuadSpline {
    /// Construct a new `QuadSpline` from an array of [`Point`]s.
    #[inline]
    pub fn new(points: Vec<Point>) -> Self {
        Self(points)
    }

    /// Return the spline's control [`Point`]s.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.0
    }

    /// Return an iterator over the implied [`QuadBez`] sequence.
    ///
    /// The returns quads are guarranteed to be G1 continuous because
    /// this is a B-spline.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = QuadBez> + '_ {
        ToQuadBez {
            idx: 0,
            points: &self.0,
        }
    }
}

struct ToQuadBez<'a> {
    idx: usize,
    points: &'a Vec<Point>,
}

impl<'a> Iterator for ToQuadBez<'a> {
    type Item = QuadBez;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx + 2 >= self.points.len() {
            return None; // we're done here
        }
        // If we're at an off-curve, next is an off-curve, start halfway between
        let p0 = if self.idx == 0 {
            self.points[self.idx]
        } else {
            self.points[self.idx].midpoint(self.points[self.idx + 1])
        };
        let p1 = self.points[self.idx + 1];
        let p2 = if self.idx + 2 == self.points.len() - 1 {
            self.points[self.idx + 2]
        } else {
            self.points[self.idx + 1].midpoint(self.points[self.idx + 2])
        };

        self.idx += 1;

        Some(QuadBez { p0, p1, p2 })
    }
}

#[cfg(test)]
mod tests {
    use crate::{Point, QuadBez, QuadSpline};

    #[test]
    pub fn no_points_no_quads() {
        assert!(QuadSpline::new(Vec::new()).iter().next().is_none());
    }

    #[test]
    pub fn one_point_no_quads() {
        assert!(QuadSpline::new(vec![Point::new(1.0, 1.0)])
            .iter()
            .next()
            .is_none());
    }

    #[test]
    pub fn two_points_no_quads() {
        assert!(
            QuadSpline::new(vec![Point::new(1.0, 1.0), Point::new(1.0, 1.0)])
                .iter()
                .next()
                .is_none()
        );
    }

    #[test]
    pub fn three_points_same_quad() {
        let p0 = Point::new(1.0, 1.0);
        let p1 = Point::new(2.0, 2.0);
        let p2 = Point::new(3.0, 3.0);
        assert_eq!(
            vec![QuadBez { p0, p1, p2 }],
            QuadSpline::new(vec![p0, p1, p2]).iter().collect::<Vec<_>>()
        );
    }

    #[test]
    pub fn four_points_implicit_on_curve() {
        let p0 = Point::new(1.0, 1.0);
        let p1 = Point::new(3.0, 3.0);
        let p2 = Point::new(5.0, 5.0);
        let p3 = Point::new(8.0, 8.0);
        assert_eq!(
            vec![
                QuadBez {
                    p0,
                    p1,
                    p2: p1.midpoint(p2)
                },
                QuadBez {
                    p0: p1.midpoint(p2),
                    p1: p2,
                    p2: p3
                }
            ],
            QuadSpline::new(vec![p0, p1, p2, p3])
                .iter()
                .collect::<Vec<_>>()
        );
    }
}
