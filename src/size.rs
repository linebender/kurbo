//! A 2d size.

use crate::Vec2;
use std::fmt;

/// A 2d size.
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

    /// Convert this size into a `Vec2`, with `width` mapped to `x` and `height`
    /// mapped to `y`.
    #[inline]
    pub fn to_vec2(self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    /// A new `Size`, with each of width and height rounded to the nearest
    /// integer value.
    #[inline]
    pub fn round(self) -> Size {
        Size::new(self.width.round(), self.height.round())
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
