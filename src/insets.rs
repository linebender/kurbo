// Copyright 2019 The kurbo Authors.
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

//! A description of the distances between the edges of two rectangles.

use std::ops::Add;

use crate::Rect;

/// Insets from the edges of a rectangle.
///
///
/// The inset value for each edge can be thought of as a delta computed from
/// the center of the rect to that edge. For instance, with an inset of `2.0` on
/// the x-axis, a rectange at `(0.0, 0.0)` with that inset added will be at
/// `(-2.0, 0.0)`
///
/// Put alternatively, a positive inset represents increased distance from center,
/// and a negative inset represents decreased distance from center.
///
/// # Examples
///
/// Positive insets produce larger rectangles:
/// ```
/// use kurbo::{Insets, Rect};
///
/// let rect = Rect::from_origin_size((0., 0.,), (10., 10.,));
/// let insets = Insets::uniform_xy(3., 0.,);
///
/// let inset_rect = rect + insets;
/// assert_eq!(inset_rect.width(), 16.0, "10.0 + 3.0 * 2");
/// assert_eq!(inset_rect.x0, -3.0);
/// ```
///
/// Negative insets produce smaller rectangles:
///
/// ```
/// use kurbo::{Insets, Rect};
///
/// let rect = Rect::from_origin_size((0., 0.,), (10., 10.,));
/// let insets = Insets::uniform_xy(-3., 0.,);
///
/// let inset_rect = rect + insets;
/// assert_eq!(inset_rect.width(), 4.0, "10.0 - 3.0 * 2");
/// assert_eq!(inset_rect.x0, 3.0);
/// ```
///
/// Insets ignore negative width & height when computing new rectangles.
///
/// ```
/// use kurbo::{Insets, Rect};
///
/// let rect = Rect::new(7., 11., 0., 0.,);
/// let insets = Insets::uniform_xy(0., 1.,);
///
/// assert_eq!(rect.width(), -7.0);
///
/// let inset_rect = rect + insets;
/// assert_eq!(inset_rect.width(), 7.0);
/// assert_eq!(inset_rect.x0, 0.0);
/// assert_eq!(inset_rect.height(), 13.0);
/// ```
#[derive(Clone, Copy, Default, Debug)]
pub struct Insets {
    /// The minimum x coordinate (left edge).
    pub x0: f64,
    /// The minimum y coordinate (top edge in y-down spaces).
    pub y0: f64,
    /// The maximum x coordinate (right edge).
    pub x1: f64,
    /// The maximum y coordinate (bottom edge in y-down spaces).
    pub y1: f64,
}

impl Insets {
    /// Zero'd insets.
    pub const ZERO: Insets = Insets::uniform(0.);

    /// New uniform insets.
    #[inline]
    pub const fn uniform(d: f64) -> Insets {
        Insets {
            x0: d,
            y0: d,
            x1: d,
            y1: d,
        }
    }

    /// New insets with uniform values along each axis.
    #[inline]
    pub const fn uniform_xy(x: f64, y: f64) -> Insets {
        Insets {
            x0: x,
            y0: y,
            x1: x,
            y1: y,
        }
    }

    /// New insets. The ordering of the arguments is "left, top, right, bottom",
    /// assuming a y-down coordinate space.
    #[inline]
    pub const fn new(x0: f64, y0: f64, x1: f64, y1: f64) -> Insets {
        Insets { x0, y0, x1, y1 }
    }

    /// The total delta on the x-axis represented by these insets.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Insets;
    ///
    /// let insets = Insets::uniform_xy(3., 8.);
    /// assert_eq!(insets.x_value(), 6.);
    ///
    /// let insets = Insets::new(5., 0., -12., 0.,);
    /// assert_eq!(insets.x_value(), -7.);
    /// ```
    #[inline]
    pub fn x_value(self) -> f64 {
        self.x0 + self.x1
    }

    /// The total delta on the y-axis represented by these insets.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Insets;
    ///
    /// let insets = Insets::uniform_xy(3., 7.);
    /// assert_eq!(insets.y_value(), 14.);
    ///
    /// let insets = Insets::new(5., 10., -12., 4.,);
    /// assert_eq!(insets.y_value(), 14.);
    /// ```
    #[inline]
    pub fn y_value(self) -> f64 {
        self.y0 + self.y1
    }
}

impl Add<Rect> for Insets {
    type Output = Rect;

    fn add(self, other: Rect) -> Rect {
        let other = other.abs();
        Rect::new(
            other.x0 - self.x0,
            other.y0 - self.y0,
            other.x1 + self.x1,
            other.y1 + self.y1,
        )
    }
}

impl Add<Insets> for Rect {
    type Output = Rect;

    fn add(self, other: Insets) -> Rect {
        other + self
    }
}

impl From<f64> for Insets {
    fn from(src: f64) -> Insets {
        Insets::uniform(src)
    }
}

impl From<(f64, f64)> for Insets {
    fn from(src: (f64, f64)) -> Insets {
        Insets::uniform_xy(src.0, src.1)
    }
}

impl From<(f64, f64, f64, f64)> for Insets {
    fn from(src: (f64, f64, f64, f64)) -> Insets {
        Insets::new(src.0, src.1, src.2, src.3)
    }
}
