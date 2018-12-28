//! Affine transforms.

use std::ops::{Mul, MulAssign};

use crate::Vec2;

/// A 2D affine transform.
#[derive(Clone, Copy)]
pub struct Affine([f64; 6]);

impl Affine {
    /// Construct an affine transform from coefficients.
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
}

impl Default for Affine {
    #[inline]
    fn default() -> Affine {
        Affine::scale(1.0)
    }
}

impl Mul<Vec2> for Affine {
    type Output = Vec2;

    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        Vec2::new(self.0[0] * other.x + self.0[2] * other.y + self.0[4],
            self.0[1] * other.x + self.0[3] * other.y + self.0[5])
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

#[cfg(test)]
mod tests {
    use crate::{Affine, Vec2};
    use std::f64::consts::PI;

    fn assert_near(p0: Vec2, p1: Vec2) {
        assert!((p1 - p0).hypot() < 1e-9, "{:?} != {:?}", p0, p1);
    }

    #[test]
    fn affine_basic() {
        let p = Vec2::new(3.0, 4.0);

        assert_near(Affine::default() * p, p);
        assert_near(Affine::scale(2.0) * p, Vec2::new(6.0, 8.0));
        assert_near(Affine::rotate(0.0) * p, p);
        assert_near(Affine::rotate(PI / 2.0) * p, Vec2::new(-4.0, 3.0));
        assert_near(Affine::translate((5.0, 6.0)) * p, Vec2::new(8.0, 10.0));
    }

    #[test]
    fn affine_mul() {
        let a1 = Affine::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let a2 = Affine::new([0.1, 1.2, 2.3, 3.4, 4.5, 5.6]);

        let px = Vec2::new(1.0, 0.0);
        let py = Vec2::new(0.0, 1.0);
        assert_near(a1 * (a2 * px), (a1 * a2) * px);
        assert_near(a1 * (a2 * py), (a1 * a2) * py);
        assert_near(a1 * (a2 * (px + py)), (a1 * a2) * (px + py));
    }
}
