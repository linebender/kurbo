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

    /// Convert this size into a `Vec2`, with `width` mapped to `x` and `height`
    /// mapped to `y`.
    #[inline]
    pub fn to_vec2(self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }
}

impl fmt::Debug for Size {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}W×{:?}H", self.width, self.height)
    }
}

impl fmt::Display for Size {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "({}×{})", self.width, self.height)
    }
}
