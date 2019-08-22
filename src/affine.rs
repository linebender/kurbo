//! Affine transforms.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::ops::{Mul, MulAssign};

use crate::{Point, Vec2};

/// A 2D affine transform.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Affine([f64; 6]);

impl Affine {
    /// Construct an affine transform from coefficients.
    ///
    /// If the coefficients are `(a, b, c, d, e, f)`, then the resulting
    /// transformation represents this augmented matrix:
    ///
    /// ```text
    /// | a c e |
    /// | b d f |
    /// | 0 0 1 |
    /// ```
    ///
    /// Note that this convention is transposed from PostScript and
    /// Direct2D, but is consistent with the
    /// [Wikipedia](https://en.wikipedia.org/wiki/Affine_transformation)
    /// formulation of affine transformation as augmented matrix. The
    /// idea is that `(A * B) * v == A * (B * v)`, where `*` is the
    /// [`Mul`](https://doc.rust-lang.org/std/ops/trait.Mul.html) trait.
    #[inline]
    pub fn new(c: [f64; 6]) -> Affine {
        Affine(c)
    }

    /// An affine transform representing uniform scaling.
    #[inline]
    pub fn scale(s: f64) -> Affine {
        Affine([s, 0.0, 0.0, s, 0.0, 0.0])
    }

    /// An affine transform representing rotation.
    ///
    /// The convention for rotation is that a positive angle rotates a
    /// positive X direction into positive Y. Thus, in a Y-down coordinate
    /// system (as is common for graphics), it is a clockwise rotation, and
    /// in Y-up (traditional for math), it is anti-clockwise.
    ///
    /// The angle, `th`, is expressed in radians.
    #[inline]
    pub fn rotate(th: f64) -> Affine {
        let s = th.sin();
        let c = th.cos();
        Affine([c, s, -s, c, 0.0, 0.0])
    }

    /// An affine transform representing translation.
    #[inline]
    pub fn translate<V: Into<Vec2>>(p: V) -> Affine {
        let p = p.into();
        Affine([1.0, 0.0, 0.0, 1.0, p.x, p.y])
    }

    /// Get the coefficients of the transform.
    #[inline]
    pub fn as_coeffs(self) -> [f64; 6] {
        self.0
    }
}

impl Default for Affine {
    #[inline]
    fn default() -> Affine {
        Affine::scale(1.0)
    }
}

impl Mul<Point> for Affine {
    type Output = Point;

    #[inline]
    fn mul(self, other: Point) -> Point {
        Point::new(
            self.0[0] * other.x + self.0[2] * other.y + self.0[4],
            self.0[1] * other.x + self.0[3] * other.y + self.0[5],
        )
    }
}

impl Mul for Affine {
    type Output = Affine;

    #[inline]
    fn mul(self, other: Affine) -> Affine {
        Affine([
            self.0[0] * other.0[0] + self.0[2] * other.0[1],
            self.0[1] * other.0[0] + self.0[3] * other.0[1],
            self.0[0] * other.0[2] + self.0[2] * other.0[3],
            self.0[1] * other.0[2] + self.0[3] * other.0[3],
            self.0[0] * other.0[4] + self.0[2] * other.0[5] + self.0[4],
            self.0[1] * other.0[4] + self.0[3] * other.0[5] + self.0[5],
        ])
    }
}

impl MulAssign for Affine {
    #[inline]
    fn mul_assign(&mut self, other: Affine) {
        *self = self.mul(other);
    }
}

impl Mul<Affine> for f64 {
    type Output = Affine;

    #[inline]
    fn mul(self, other: Affine) -> Affine {
        Affine([
            self * other.0[0],
            self * other.0[1],
            self * other.0[2],
            self * other.0[3],
            self * other.0[4],
            self * other.0[5],
        ])
    }
}

// Conversions to and from mint

#[cfg(feature = "mint")]
impl From<Affine> for mint::ColumnMatrix2x3<f64> {
    #[inline]
    fn from(a: Affine) -> mint::ColumnMatrix2x3<f64> {
        mint::ColumnMatrix2x3 {
            x: mint::Vector2 {
                x: a.0[0],
                y: a.0[1],
            },
            y: mint::Vector2 {
                x: a.0[2],
                y: a.0[3],
            },
            z: mint::Vector2 {
                x: a.0[4],
                y: a.0[5],
            },
        }
    }
}

#[cfg(feature = "mint")]
impl From<mint::ColumnMatrix2x3<f64>> for Affine {
    #[inline]
    fn from(m: mint::ColumnMatrix2x3<f64>) -> Affine {
        Affine([m.x.x, m.x.y, m.y.x, m.y.y, m.z.x, m.z.y])
    }
}

#[cfg(test)]
mod tests {
    use crate::{Affine, Point};
    use std::f64::consts::PI;

    fn assert_near(p0: Point, p1: Point) {
        assert!((p1 - p0).hypot() < 1e-9, "{:?} != {:?}", p0, p1);
    }

    #[test]
    fn affine_basic() {
        let p = Point::new(3.0, 4.0);

        assert_near(Affine::default() * p, p);
        assert_near(Affine::scale(2.0) * p, Point::new(6.0, 8.0));
        assert_near(Affine::rotate(0.0) * p, p);
        assert_near(Affine::rotate(PI / 2.0) * p, Point::new(-4.0, 3.0));
        assert_near(Affine::translate((5.0, 6.0)) * p, Point::new(8.0, 10.0));
    }

    #[test]
    fn affine_mul() {
        let a1 = Affine::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let a2 = Affine::new([0.1, 1.2, 2.3, 3.4, 4.5, 5.6]);

        let px = Point::new(1.0, 0.0);
        let py = Point::new(0.0, 1.0);
        let pxy = Point::new(1.0, 1.0);
        assert_near(a1 * (a2 * px), (a1 * a2) * px);
        assert_near(a1 * (a2 * py), (a1 * a2) * py);
        assert_near(a1 * (a2 * pxy), (a1 * a2) * pxy);
    }
}
