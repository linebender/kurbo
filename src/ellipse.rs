//! Implementation of ellipse shape.

use std::f64::consts::PI;
use std::{
    iter,
    ops::{Add, Sub},
};

use crate::{Arc, ArcAppendIter, PathEl, Point, Rect, Shape, Vec2};

/// A ellipse.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    /// The center.
    pub center: Point,
    /// The radii.
    pub radii: Vec2,
    /// How much to rotate the major/minor axes.
    pub x_rotation: f64,
}

impl Ellipse {
    /// A new ellipse from center and radii.
    #[inline]
    pub fn new(center: impl Into<Point>, radii: impl Into<Vec2>, x_rotation: f64) -> Ellipse {
        Ellipse {
            center: center.into(),
            radii: radii.into(),
            x_rotation,
        }
    }
}

impl Add<Vec2> for Ellipse {
    type Output = Ellipse;

    #[inline]
    fn add(self, v: Vec2) -> Ellipse {
        Ellipse {
            center: self.center + v,
            radii: self.radii,
            x_rotation: self.x_rotation,
        }
    }
}

impl Sub<Vec2> for Ellipse {
    type Output = Ellipse;

    #[inline]
    fn sub(self, v: Vec2) -> Ellipse {
        Ellipse {
            center: self.center - v,
            radii: self.radii,
            x_rotation: self.x_rotation,
        }
    }
}

impl Shape for Ellipse {
    type BezPathIter = iter::Chain<iter::Once<PathEl>, ArcAppendIter>;

    fn to_bez_path(&self, tolerance: f64) -> Self::BezPathIter {
        Arc {
            center: self.center,
            radii: self.radii,
            start_angle: 0.0,
            sweep_angle: 2.0 * PI,
            x_rotation: self.x_rotation,
        }
        .to_bez_path(tolerance)
    }

    #[inline]
    fn area(&self) -> f64 {
        PI * self.radii.x * self.radii.y
    }

    #[inline]
    fn perimeter(&self, accuracy: f64) -> f64 {
        self.clone()
            .into_bez_path(0.1)
            .elements()
            .perimeter(accuracy)
    }

    fn winding(&self, pt: Point) -> i32 {
        self.clone().into_bez_path(0.1).elements().winding(pt)
    }

    #[inline]
    fn bounding_box(&self) -> Rect {
        self.clone().into_bez_path(0.1).elements().bounding_box()
    }
}

/*
#[cfg(test)]
mod tests {
    use crate::{Ellipse, Point, Shape};
    use std::f64::consts::PI;

    fn assert_approx_eq(x: f64, y: f64) {
        // Note: we might want to be more rigorous in testing the accuracy
        // of the conversion into Béziers. But this seems good enough.
        assert!((x - y).abs() < 1e-7, "{} != {}", x, y);
    }

    #[test]
    fn area_sign() {
        let center = Point::new(5.0, 5.0);
        let c = Ellipse::new(center, 5.0);
        assert_approx_eq(c.area(), 25.0 * PI);

        assert_eq!(c.winding(center), 1);

        let p = c.into_bez_path(1e-9);
        assert_approx_eq(c.area(), p.area());
        assert_eq!(c.winding(center), p.winding(center));

        let c_neg_radius = Ellipse::new(center, -5.0);
        assert_approx_eq(c_neg_radius.area(), 25.0 * PI);

        assert_eq!(c_neg_radius.winding(center), 1);

        let p_neg_radius = c_neg_radius.into_bez_path(1e-9);
        assert_approx_eq(c_neg_radius.area(), p_neg_radius.area());
        assert_eq!(c_neg_radius.winding(center), p_neg_radius.winding(center));
    }
}
*/
