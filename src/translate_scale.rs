//! A transformation that includes both scale and translation.

use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

use crate::{Affine, Circle, Point, Rect, Vec2};

/// A transformation including scaling and translation.
#[derive(Clone, Copy, Debug)]
pub struct TranslateScale {
    translation: Vec2,
    scale: f64,
}

impl TranslateScale {
    /// Create a new transformation from translation and scale.
    #[inline]
    pub fn new(translation: Vec2, scale: f64) -> TranslateScale {
        TranslateScale { translation, scale }
    }

    /// Create a new transformation with scale only.
    #[inline]
    pub fn scale(s: f64) -> TranslateScale {
        TranslateScale::new(Vec2::ZERO, s)
    }

    /// Create a new transformation with translation only.
    #[inline]
    pub fn translate(t: Vec2) -> TranslateScale {
        TranslateScale::new(t, 1.0)
    }

    /// Decompose transformation into translation and scale.
    pub fn as_tuple(self) -> (Vec2, f64) {
        (self.translation, self.scale)
    }
}

impl Default for TranslateScale {
    #[inline]
    fn default() -> TranslateScale {
        TranslateScale::scale(1.0)
    }
}

impl From<TranslateScale> for Affine {
    fn from(ts: TranslateScale) -> Affine {
        let TranslateScale { translation, scale } = ts;
        Affine::new([scale, 0.0, 0.0, scale, translation.x, translation.y])
    }
}

impl Mul<Point> for TranslateScale {
    type Output = Point;

    #[inline]
    fn mul(self, other: Point) -> Point {
        (self.scale * other.to_vec2()).to_point() + self.translation
    }
}

impl Mul for TranslateScale {
    type Output = TranslateScale;

    #[inline]
    fn mul(self, other: TranslateScale) -> TranslateScale {
        TranslateScale {
            translation: self.translation + self.scale * other.translation,
            scale: self.scale * other.scale,
        }
    }
}

impl MulAssign for TranslateScale {
    #[inline]
    fn mul_assign(&mut self, other: TranslateScale) {
        *self = self.mul(other);
    }
}

impl Mul<TranslateScale> for f64 {
    type Output = TranslateScale;

    #[inline]
    fn mul(self, other: TranslateScale) -> TranslateScale {
        TranslateScale {
            translation: other.translation * self,
            scale: other.scale * self,
        }
    }
}

impl Add<Vec2> for TranslateScale {
    type Output = TranslateScale;

    #[inline]
    fn add(self, other: Vec2) -> TranslateScale {
        TranslateScale {
            translation: self.translation + other,
            scale: self.scale,
        }
    }
}

impl Add<TranslateScale> for Vec2 {
    type Output = TranslateScale;

    #[inline]
    fn add(self, other: TranslateScale) -> TranslateScale {
        other + self
    }
}

impl AddAssign<Vec2> for TranslateScale {
    #[inline]
    fn add_assign(&mut self, other: Vec2) {
        *self = self.add(other);
    }
}

impl Sub<Vec2> for TranslateScale {
    type Output = TranslateScale;

    #[inline]
    fn sub(self, other: Vec2) -> TranslateScale {
        TranslateScale {
            translation: self.translation - other,
            scale: self.scale,
        }
    }
}

impl SubAssign<Vec2> for TranslateScale {
    #[inline]
    fn sub_assign(&mut self, other: Vec2) {
        *self = self.sub(other);
    }
}

impl Mul<Circle> for TranslateScale {
    type Output = Circle;

    #[inline]
    fn mul(self, other: Circle) -> Circle {
        Circle::new(self * other.center, self.scale * other.radius)
    }
}

impl Mul<Rect> for TranslateScale {
    type Output = Rect;

    #[inline]
    fn mul(self, other: Rect) -> Rect {
        let pt0 = self * Point::new(other.x0, other.y0);
        let pt1 = self * Point::new(other.x1, other.y1);
        (pt0, pt1).into()
    }
}
