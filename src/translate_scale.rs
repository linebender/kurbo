//! A transformation that includes both scale and translation.

use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

use crate::{Affine, Circle, CubicBez, Line, Point, QuadBez, Rect, RoundedRect, Vec2};

/// A transformation including scaling and translation.
///
/// If the translation is `(x, y)` and the scale is `s`, then this
/// transformation represents this augmented matrix:
///
/// ```text
/// | s 0 x |
/// | 0 s y |
/// | 0 0 1 |
/// ```
///
/// See [`Affine`](struct.Affine.html) for more details about the
/// equivalence with augmented matrices.
///
/// Various multiplication ops are defined, and these are all defined
/// to be consistent with matrix multiplication. Therefore,
/// `TranslateScale * Point` is defined but not the other way around.
///
/// Also note that multiplication is not commutative. Thus,
/// `TranslateScale::scale(2.0) * TranslateScale::translate(Vec2::new(1.0, 0.0))`
/// has a translation of (2, 0), while
/// `TranslateScale::translate(Vec2::new(1.0, 0.0)) * TranslateScale::scale(2.0)`
/// has a translation of (1, 0). (Both have a scale of 2; also note that
/// the first case can be written
/// `2.0 * TranslateScale::translate(Vec2::new(1.0, 0.0))` as this case
/// has an implicit conversion).
///
/// This transformation is less powerful than `Affine`, but can be applied
/// to more primitives, especially including [`Rect`](struct.Rect.html).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TranslateScale {
    translation: Vec2,
    scale: f64,
}

impl TranslateScale {
    /// Create a new transformation from translation and scale.
    #[inline]
    pub const fn new(translation: Vec2, scale: f64) -> TranslateScale {
        TranslateScale { translation, scale }
    }

    /// Create a new transformation with scale only.
    #[inline]
    pub const fn scale(s: f64) -> TranslateScale {
        TranslateScale::new(Vec2::ZERO, s)
    }

    /// Create a new transformation with translation only.
    #[inline]
    pub const fn translate(t: Vec2) -> TranslateScale {
        TranslateScale::new(t, 1.0)
    }

    /// Decompose transformation into translation and scale.
    pub fn as_tuple(self) -> (Vec2, f64) {
        (self.translation, self.scale)
    }

    /// Compute the inverse transform.
    ///
    /// Multiplying a transform with its inverse (either on the
    /// left or right) results in the identity transform
    /// (modulo floating point rounding errors).
    ///
    /// Produces NaN values when scale is zero.
    pub fn inverse(self) -> TranslateScale {
        let scale_recip = self.scale.recip();
        TranslateScale {
            translation: self.translation * -scale_recip,
            scale: scale_recip,
        }
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

impl Mul<Line> for TranslateScale {
    type Output = Line;

    #[inline]
    fn mul(self, other: Line) -> Line {
        Line::new(self * other.p0, self * other.p1)
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

impl Mul<RoundedRect> for TranslateScale {
    type Output = RoundedRect;

    #[inline]
    fn mul(self, other: RoundedRect) -> RoundedRect {
        RoundedRect::from_rect(self * other.rect(), self.scale * other.radius())
    }
}

impl Mul<QuadBez> for TranslateScale {
    type Output = QuadBez;

    #[inline]
    fn mul(self, other: QuadBez) -> QuadBez {
        QuadBez::new(self * other.p0, self * other.p1, self * other.p2)
    }
}

impl Mul<CubicBez> for TranslateScale {
    type Output = CubicBez;

    #[inline]
    fn mul(self, other: CubicBez) -> CubicBez {
        CubicBez::new(
            self * other.p0,
            self * other.p1,
            self * other.p2,
            self * other.p3,
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::{Affine, Point, TranslateScale, Vec2};

    fn assert_near(p0: Point, p1: Point) {
        assert!((p1 - p0).hypot() < 1e-9, "{:?} != {:?}", p0, p1);
    }

    #[test]
    fn translate_scale() {
        let p = Point::new(3.0, 4.0);
        let ts = TranslateScale::new(Vec2::new(5.0, 6.0), 2.0);

        assert_near(ts * p, Point::new(11.0, 14.0));
    }

    #[test]
    fn conversions() {
        let p = Point::new(3.0, 4.0);
        let s = 2.0;
        let t = Vec2::new(5.0, 6.0);
        let ts = TranslateScale::new(t, s);

        // Test that conversion to affine is consistent.
        let a: Affine = ts.into();
        assert_near(ts * p, a * p);

        assert_near((s * p.to_vec2()).to_point(), TranslateScale::scale(s) * p);
        assert_near(p + t, TranslateScale::translate(t) * p);
    }

    #[test]
    fn inverse() {
        let p = Point::new(3.0, 4.0);
        let ts = TranslateScale::new(Vec2::new(5.0, 6.0), 2.0);

        assert_near(p, (ts * ts.inverse()) * p);
        assert_near(p, (ts.inverse() * ts) * p);
    }
}
