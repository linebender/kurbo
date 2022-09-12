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

/// A quadratic Bézier spline.
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
}
