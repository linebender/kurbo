//! A 2D size.

use std::fmt;
use std::ops::{Mul, MulAssign};

use crate::common::FloatExt;
use crate::{Rect, Vec2};

/// A 2D size.
#[derive(Clone, Copy, Default, PartialEq)]
pub struct Size {
    /// The width.
    pub width: f64,
    /// The height.
    pub height: f64,
}

impl Size {
    /// A size with zero width or height.
    pub const ZERO: Size = Size::new(0., 0.);

    /// Create a new `Size` with the provided `width` and `height`.
    #[inline]
    pub const fn new(width: f64, height: f64) -> Self {
        Size { width, height }
    }

    /// Returns a new size bounded by `min` and `max.`
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    ///
    /// let this = Size::new(0., 100.);
    /// let min = Size::new(10., 10.,);
    /// let max = Size::new(50., 50.);
    /// assert_eq!(this.clamp(min, max), Size::new(10., 50.))
    /// ```
    pub fn clamp(self, min: Size, max: Size) -> Self {
        let width = self.width.max(min.width).min(max.width);
        let height = self.height.max(min.height).min(max.height);
        Size { width, height }
    }

    /// Convert this size into a [`Vec2`], with `width` mapped to `x` and `height`
    /// mapped to `y`.
    ///
    /// [`Vec2`]: struct.Vec2.html
    #[inline]
    pub const fn to_vec2(self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    /// Returns a new `Size`,
    /// with `width` and `height` rounded to the nearest integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    /// let size_pos = Size::new(3.3, 3.6).round();
    /// assert_eq!(size_pos.width, 3.0);
    /// assert_eq!(size_pos.height, 4.0);
    /// let size_neg = Size::new(-3.3, -3.6).round();
    /// assert_eq!(size_neg.width, -3.0);
    /// assert_eq!(size_neg.height, -4.0);
    /// ```
    #[inline]
    pub fn round(self) -> Size {
        Size::new(self.width.round(), self.height.round())
    }

    /// Returns a new `Size`,
    /// with `width` and `height` rounded up to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    /// let size_pos = Size::new(3.3, 3.6).ceil();
    /// assert_eq!(size_pos.width, 4.0);
    /// assert_eq!(size_pos.height, 4.0);
    /// let size_neg = Size::new(-3.3, -3.6).ceil();
    /// assert_eq!(size_neg.width, -3.0);
    /// assert_eq!(size_neg.height, -3.0);
    /// ```
    #[inline]
    pub fn ceil(self) -> Size {
        Size::new(self.width.ceil(), self.height.ceil())
    }

    /// Returns a new `Size`,
    /// with `width` and `height` rounded down to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    /// let size_pos = Size::new(3.3, 3.6).floor();
    /// assert_eq!(size_pos.width, 3.0);
    /// assert_eq!(size_pos.height, 3.0);
    /// let size_neg = Size::new(-3.3, -3.6).floor();
    /// assert_eq!(size_neg.width, -4.0);
    /// assert_eq!(size_neg.height, -4.0);
    /// ```
    #[inline]
    pub fn floor(self) -> Size {
        Size::new(self.width.floor(), self.height.floor())
    }

    /// Returns a new `Size`,
    /// with `width` and `height` rounded away from zero to the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    /// let size_pos = Size::new(3.3, 3.6).expand();
    /// assert_eq!(size_pos.width, 4.0);
    /// assert_eq!(size_pos.height, 4.0);
    /// let size_neg = Size::new(-3.3, -3.6).expand();
    /// assert_eq!(size_neg.width, -4.0);
    /// assert_eq!(size_neg.height, -4.0);
    /// ```
    #[inline]
    pub fn expand(self) -> Size {
        Size::new(self.width.expand(), self.height.expand())
    }

    /// Returns a new `Size`,
    /// with `width` and `height` rounded down towards zero the nearest integer,
    /// unless they are already an integer.
    ///
    /// # Examples
    ///
    /// ```
    /// use kurbo::Size;
    /// let size_pos = Size::new(3.3, 3.6).trunc();
    /// assert_eq!(size_pos.width, 3.0);
    /// assert_eq!(size_pos.height, 3.0);
    /// let size_neg = Size::new(-3.3, -3.6).trunc();
    /// assert_eq!(size_neg.width, -3.0);
    /// assert_eq!(size_neg.height, -3.0);
    /// ```
    #[inline]
    pub fn trunc(self) -> Size {
        Size::new(self.width.trunc(), self.height.trunc())
    }

    /// Convert this `Size` into a [`Rect`] with origin `(0.0, 0.0)`.
    ///
    /// [`Rect`]: struct.Rect.html
    #[inline]
    pub const fn to_rect(self) -> Rect {
        Rect::new(0., 0., self.width, self.height)
    }
}

impl fmt::Debug for Size {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}W×{:?}H", self.width, self.height)
    }
}

impl fmt::Display for Size {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "(")?;
        fmt::Display::fmt(&self.width, formatter)?;
        write!(formatter, "×")?;
        fmt::Display::fmt(&self.height, formatter)?;
        write!(formatter, ")")
    }
}

impl MulAssign<f64> for Size {
    #[inline]
    fn mul_assign(&mut self, other: f64) {
        *self = Size {
            width: self.width * other,
            height: self.height * other,
        };
    }
}

impl Mul<Size> for f64 {
    type Output = Size;

    #[inline]
    fn mul(self, other: Size) -> Size {
        other * self
    }
}

impl Mul<f64> for Size {
    type Output = Size;

    #[inline]
    fn mul(self, other: f64) -> Size {
        Size {
            width: self.width * other,
            height: self.height * other,
        }
    }
}

impl From<(f64, f64)> for Size {
    #[inline]
    fn from(v: (f64, f64)) -> Size {
        Size {
            width: v.0,
            height: v.1,
        }
    }
}

impl From<Size> for (f64, f64) {
    #[inline]
    fn from(v: Size) -> (f64, f64) {
        (v.width, v.height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display() {
        let s = Size::new(-0.12345, 9.87654);
        assert_eq!(format!("{}", s), "(-0.12345×9.87654)");

        let s = Size::new(-0.12345, 9.87654);
        assert_eq!(format!("{:+6.2}", s), "( -0.12× +9.88)");
    }
}
