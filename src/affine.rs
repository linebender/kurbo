//! Affine transforms.

use std::ops::{Mul, MulAssign};

use crate::{Point, Rect, Vec2};

/// A 2D affine transform.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Affine([f64; 6]);

impl Affine {
    /// A transform that is flipped on the y-axis. Useful for converting between
    /// y-up and y-down spaces.
    pub const FLIP_Y: Affine = Affine::new([1.0, 0., 0., -1.0, 0., 0.]);

    /// A transform that is flipped on the x-axis.
    pub const FLIP_X: Affine = Affine::new([-1.0, 0., 0., 1.0, 0., 0.]);

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
    pub const fn new(c: [f64; 6]) -> Affine {
        Affine(c)
    }

    /// An affine transform representing uniform scaling.
    #[inline]
    pub const fn scale(s: f64) -> Affine {
        Affine([s, 0.0, 0.0, s, 0.0, 0.0])
    }

    /// An affine transform representing non-uniform scaling
    /// with different scale values for x and y
    #[inline]
    pub const fn scale_non_uniform(s_x: f64, s_y: f64) -> Affine {
        Affine([s_x, 0.0, 0.0, s_y, 0.0, 0.0])
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

    /// Compute the determinant of this transform.
    pub fn determinant(self) -> f64 {
        self.0[0] * self.0[3] - self.0[1] * self.0[2]
    }

    /// Compute the inverse transform.
    ///
    /// Produces NaN values when the determinant is zero.
    pub fn inverse(self) -> Affine {
        let inv_det = self.determinant().recip();
        Affine([
            inv_det * self.0[3],
            -inv_det * self.0[1],
            -inv_det * self.0[2],
            inv_det * self.0[0],
            inv_det * (self.0[2] * self.0[5] - self.0[3] * self.0[4]),
            inv_det * (self.0[1] * self.0[4] - self.0[0] * self.0[5]),
        ])
    }

    /// Compute the bounding box of a transformed rectangle.
    ///
    /// Returns the minimal `Rect` that encloses the given `Rect` after affine transformation.
    /// If the transform is axis-aligned, then this bounding box is "tight", in other words the
    /// returned `Rect` is the transformed rectangle.
    ///
    /// The returned rectangle always has non-negative width and height.
    pub fn transform_rect_bbox(self, rect: Rect) -> Rect {
        let p00 = self * Point::new(rect.x0, rect.y0);
        let p01 = self * Point::new(rect.x0, rect.y1);
        let p10 = self * Point::new(rect.x1, rect.y0);
        let p11 = self * Point::new(rect.x1, rect.y1);
        Rect::from_points(p00, p01).union(Rect::from_points(p10, p11))
    }

    /// Compute the singular value decomposition of the linear transformation (ignoring the
    /// translation).
    ///
    /// All non-degenerate linear transformations can be represented as
    ///
    ///  1. a rotation about the origin.
    ///  2. a scaling along the x and y axes
    ///  3. another rotation about the origin
    ///
    /// composed together. Decomposing a 2x2 matrix in this way is called a "singular value
    /// decomposition" and is written `U Σ V^T`, where U and V^T are orthogonal (rotations) and Σ
    /// is a diagonal matrix (a scaling).
    ///
    /// Since currently this function is used to calculate ellipse radii and rotation from an
    /// affine map on the unit circle, we don't calculate V^T, since a rotation of the unit (or
    /// any) circle about its center always results in the same circle. This is the reason that an
    /// ellipse mapped using an affine map is always an ellipse.
    ///
    /// Will return NaNs if the matrix (or equivalently the linear map) is singular.
    ///
    /// First part of the return tuple is the scaling, second part is the angle of rotation (in
    /// radians)
    #[inline]
    pub(crate) fn svd(self) -> (Vec2, f64) {
        let a = self.0[0];
        let a2 = a * a;
        let b = self.0[1];
        let b2 = b * b;
        let c = self.0[2];
        let c2 = c * c;
        let d = self.0[3];
        let d2 = d * d;
        let ab = a * b;
        let cd = c * d;
        let angle = 0.5 * (2.0 * (ab + cd)).atan2(a2 - b2 + c2 - d2);
        let s1 = a2 + b2 + c2 + d2;
        let s2 = ((a2 - b2 + c2 - d2).powi(2) + 4.0 * (ab + cd).powi(2)).sqrt();
        (
            Vec2 {
                x: (0.5 * (s1 + s2)).sqrt(),
                y: (0.5 * (s1 - s2)).sqrt(),
            },
            angle,
        )
    }

    /// Returns the translation part of this affine map (`(self.0[4], self.0[5])`).
    #[inline]
    pub(crate) fn get_translation(self) -> Vec2 {
        Vec2 {
            x: self.0[4],
            y: self.0[5],
        }
    }

    /// Replaces the translation portion of this affine map
    #[must_use]
    pub(crate) fn set_translation(mut self, trans: Vec2) -> Affine {
        self.0[4] = trans.x;
        self.0[5] = trans.y;
        self
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

    #[test]
    fn affine_inv() {
        let a = Affine::new([0.1, 1.2, 2.3, 3.4, 4.5, 5.6]);
        let a_inv = a.inverse();

        let px = Point::new(1.0, 0.0);
        let py = Point::new(0.0, 1.0);
        let pxy = Point::new(1.0, 1.0);
        assert_near(a * (a_inv * px), px);
        assert_near(a * (a_inv * py), py);
        assert_near(a * (a_inv * pxy), pxy);
        assert_near(a_inv * (a * px), px);
        assert_near(a_inv * (a * py), py);
        assert_near(a_inv * (a * pxy), pxy);
    }
}
