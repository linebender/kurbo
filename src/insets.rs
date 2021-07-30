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

use std::ops::{Add, Neg, Sub};

use crate::{Rect, Size};

/// Insets from the edges of a rectangle.
///
///
/// The inset value for each edge can be thought of as a delta computed from
/// the center of the rect to that edge. For instance, with an inset of `2.0` on
/// the x-axis, a rectangle with the origin `(0.0, 0.0)` with that inset added
/// will have the new origin at `(-2.0, 0.0)`.
///
/// Put alternatively, a positive inset represents increased distance from center,
/// and a negative inset represents decreased distance from center.
///
/// # Examples
///
/// Positive insets added to a [`Rect`] produce a larger [`Rect`]:
/// ```
/// # use kurbo::{Insets, Rect};
/// let rect = Rect::from_origin_size((0., 0.,), (10., 10.,));
/// let insets = Insets::uniform_xy(3., 0.,);
///
/// let inset_rect = rect + insets;
/// assert_eq!(inset_rect.width(), 16.0, "10.0 + 3.0 × 2");
/// assert_eq!(inset_rect.x0, -3.0);
/// ```
///
/// Negative insets added to a [`Rect`] produce a smaller [`Rect`]:
///
/// ```
/// # use kurbo::{Insets, Rect};
/// let rect = Rect::from_origin_size((0., 0.,), (10., 10.,));
/// let insets = Insets::uniform_xy(-3., 0.,);
///
/// let inset_rect = rect + insets;
/// assert_eq!(inset_rect.width(), 4.0, "10.0 - 3.0 × 2");
/// assert_eq!(inset_rect.x0, 3.0);
/// ```
///
/// [`Insets`] operate on the absolute rectangle [`Rect::abs`], and so ignore
/// existing negative widths and heights.
///
/// ```
/// # use kurbo::{Insets, Rect};
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
///
/// The width and height of an inset operation can still be negative if the
/// [`Insets`]' dimensions are greater than the dimensions of the original [`Rect`].
///
/// ```
/// # use kurbo::{Insets, Rect};
/// let rect = Rect::new(0., 0., 3., 5.);
/// let insets = Insets::uniform_xy(0., 7.,);
///
/// let inset_rect = rect - insets;
/// assert_eq!(inset_rect.height(), -9., "5 - 7 × 2")
/// ```
///
/// `Rect - Rect = Insets`:
///
///
/// ```
/// # use kurbo::{Insets, Rect};
/// let rect = Rect::new(0., 0., 5., 11.);
/// let insets = Insets::uniform_xy(1., 7.,);
///
/// let inset_rect = rect + insets;
/// let insets2 = inset_rect - rect;
///
/// assert_eq!(insets2.x0, insets.x0);
/// assert_eq!(insets2.y1, insets.y1);
/// assert_eq!(insets2.x_value(), insets.x_value());
/// assert_eq!(insets2.y_value(), insets.y_value());
/// ```
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// Zeroed insets.
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

    /// Returns the total delta represented by these insets as a [`Size`].
    ///
    /// This is equivalent to creating a [`Size`] from the values returned by
    /// [`x_value`] and [`y_value`].
    ///
    /// This function may return a size with negative values.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::{Insets, Size};
    ///
    /// let insets = Insets::new(11.1, -43.3, 3.333, -0.0);
    /// assert_eq!(insets.size(), Size::new(insets.x_value(), insets.y_value()));
    /// ```
    ///
    /// [`x_value`]: Insets::x_value
    /// [`y_value`]: Insets::y_value
    pub fn size(self) -> Size {
        Size::new(self.x_value(), self.y_value())
    }

    /// Return `true` iff all values are nonnegative.
    pub fn are_nonnegative(self) -> bool {
        let Insets { x0, y0, x1, y1 } = self;
        x0 >= 0.0 && y0 >= 0.0 && x1 >= 0.0 && y1 >= 0.0
    }

    /// Return new `Insets` with all negative values replaced with `0.0`.
    ///
    /// This is provided as a convenience for applications where negative insets
    /// are not meaningful.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Insets;
    ///
    /// let insets = Insets::new(-10., 3., -0.2, 4.);
    /// let nonnegative = insets.nonnegative();
    /// assert_eq!(nonnegative.x_value(), 0.0);
    /// assert_eq!(nonnegative.y_value(), 7.0);
    /// ```
    pub fn nonnegative(self) -> Insets {
        let Insets { x0, y0, x1, y1 } = self;
        Insets {
            x0: x0.max(0.0),
            y0: y0.max(0.0),
            x1: x1.max(0.0),
            y1: y1.max(0.0),
        }
    }

    /// Are these insets finite?
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.x0.is_finite() && self.y0.is_finite() && self.x1.is_finite() && self.y1.is_finite()
    }

    /// Are these insets NaN?
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.x0.is_nan() || self.y0.is_nan() || self.x1.is_nan() || self.y1.is_nan()
    }
}

impl Neg for Insets {
    type Output = Insets;

    #[inline]
    fn neg(self) -> Insets {
        Insets::new(-self.x0, -self.y0, -self.x1, -self.y1)
    }
}

impl Add<Rect> for Insets {
    type Output = Rect;

    #[inline]
    #[allow(clippy::suspicious_arithmetic_impl)]
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

    #[inline]
    fn add(self, other: Insets) -> Rect {
        other + self
    }
}

impl Sub<Rect> for Insets {
    type Output = Rect;

    #[inline]
    fn sub(self, other: Rect) -> Rect {
        other + -self
    }
}

impl Sub<Insets> for Rect {
    type Output = Rect;

    #[inline]
    fn sub(self, other: Insets) -> Rect {
        other - self
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
