//! Implementation of circle shape.

use std::f64::consts::{FRAC_PI_2, PI};
use std::ops::{Add, Sub};

use crate::{PathEl, Rect, Shape, Vec2};

/// A circle.
#[derive(Clone, Copy, Default, Debug)]
pub struct Circle {
    /// The center.
    pub center: Vec2,
    /// The radius.
    pub radius: f64,
}

impl Circle {
    /// A new circle from center and radius.
    #[inline]
    pub fn new(center: impl Into<Vec2>, radius: f64) -> Circle {
        Circle {
            center: center.into(),
            radius,
        }
    }
}

impl Add<Vec2> for Circle {
    type Output = Circle;

    #[inline]
    fn add(self, v: Vec2) -> Circle {
        Circle {
            center: self.center + v,
            radius: self.radius,
        }
    }
}

impl Sub<Vec2> for Circle {
    type Output = Circle;

    #[inline]
    fn sub(self, v: Vec2) -> Circle {
        Circle {
            center: self.center - v,
            radius: self.radius,
        }
    }
}

#[doc(hidden)]
pub struct CirclePathIter {
    circle: Circle,
    delta_th: f64,
    arm_len: f64,
    ix: usize,
    n: usize,
}

impl Shape for Circle {
    type BezPathIter = CirclePathIter;

    fn to_bez_path(&self, tolerance: f64) -> CirclePathIter {
        let scaled_err = self.radius.abs() / tolerance;
        let (n, arm_len) = if scaled_err < 1.0 / 1.9608e-4 {
            // Solution from http://spencermortensen.com/articles/bezier-circle/
            (4, 0.551915024494)
        } else {
            // This is empirically determined to fall within error tolerance.
            let n = (1.1163 * scaled_err).powf(1.0 / 6.0).ceil() as usize;
            // Note: this isn't minimum error, but it is simple and we can easily
            // estimate the error.
            let arm_len = (4.0 / 3.0) * (FRAC_PI_2 / (n as f64)).tan();
            (n, arm_len)
        };
        CirclePathIter {
            circle: *self,
            delta_th: 2.0 * PI / (n as f64),
            arm_len,
            ix: 0,
            n,
        }
    }

    #[inline]
    fn area(&self) -> f64 {
        PI * self.radius.powi(2)
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        (2.0 * PI * self.radius).abs()
    }

    fn winding(&self, pt: Vec2) -> i32 {
        if (pt - self.center).hypot2() < self.radius.powi(2) {
            self.radius.signum() as i32
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        let r = self.radius.abs();
        let (x, y) = self.center.into();
        Rect::new(x - r, y - r, x + r, y + r)
    }

    fn as_circle(&self) -> Option<Circle> {
        Some(*self)
    }
}

impl Iterator for CirclePathIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<PathEl> {
        let a = self.arm_len;
        let r = self.circle.radius;
        let (x, y) = self.circle.center.into();
        let ix = self.ix;
        self.ix += 1;
        if ix == 0 {
            Some(PathEl::Moveto(Vec2::new(x + r, y)))
        } else if ix <= self.n {
            let th1 = self.delta_th * (ix as f64);
            let th0 = th1 - self.delta_th;
            let (c0, s0) = (th0.cos(), th0.sin());
            let (c1, s1) = if ix == self.n {
                (1.0, 0.0)
            } else {
                (th1.cos(), th1.sin())
            };
            Some(PathEl::Curveto(
                Vec2::new(x + r * (c0 - a * s0), y + r * (s0 + a * c0)),
                Vec2::new(x + r * (c1 + a * s1), y + r * (s1 - a * c1)),
                Vec2::new(x + r * c1, y + r * s1),
            ))
        } else if ix == self.n + 1 {
            Some(PathEl::Closepath)
        } else {
            None
        }
    }
}
