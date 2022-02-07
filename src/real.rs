//! A utility for comparing real values

use crate::{Point};

/// If we're comparing numbers, our epsilon should depend on how big the number
/// is. This function returns an epsilon appropriate for the size the largest
/// number.
pub fn epsilon_for_value(num: f64) -> f64 {
    // These values have been obtained experimentally and can be changed if
    // necessary.
    (num.abs() * 0.0000000001).max(f64::EPSILON)
}

/// If we're comparing distances between samples of curves, our epsilon should
/// depend on how big the points we're comparing are. This function returns an
/// epsilon appropriate for the size of pt.
pub fn epsilon_for_point(pt: Point) -> f64 {
    let max = f64::max(f64::abs(pt.x), f64::abs(pt.y));
    epsilon_for_value(max)
}

/// Compare if two real values are approximately equal
pub fn real_is_equal(num1: f64, num2: f64) -> bool
{
    real_is_approx(num1, num2, 1.)
}

/// Compare if two real values are approximately equal to a smaller degree than
/// real_is_equal
pub fn real_is_approx(num1: f64, num2: f64, scale: f64) -> bool
{
    if num1 == f64::INFINITY || num2 == f64::INFINITY
    {
        return num1 == num2; // Do exact comparison
    }

    // Do approximate comparison
    (num1 - num2).abs() <= (scale * epsilon_for_value(num1.max(num2)))
}

/// Compare if a  real value is approximately zero
pub fn real_is_zero(num1: f64) -> bool
{
    real_is_equal(num1, 0.)
}

/// Compare if num1 < num2 and not approximately equal
pub fn real_lt(num1: f64, num2: f64) -> bool
{
    (num1 < num2) && !real_is_equal(num1, num2)
}

/// Compare if num1 < num2 or approximately equal
pub fn real_lte(num1: f64, num2: f64) -> bool
{
    (num1 < num2) || real_is_equal(num1, num2)
}

/// Compare if num1 > num2 and not approximately equal
pub fn real_gt(num1: f64, num2: f64) -> bool
{
    (num1 > num2) && !real_is_equal(num1, num2)
}

/// Compare if num1 > num2 or approximately equal
pub fn real_gte(num1: f64, num2: f64) -> bool
{
    (num1 > num2) || real_is_equal(num1, num2)
}

/// Compare if two points are approximately equal
pub fn point_is_equal(pt1: Point, pt2: Point) -> bool
{
    point_is_approx(pt1, pt2, 1.)
}

/// Compare if two points are approximately equal to a smaller degree than
/// point_is_equal
pub fn point_is_approx(pt1: Point, pt2: Point, scale: f64) -> bool
{
    real_is_approx(pt1.x, pt2.x, scale) && real_is_approx(pt1.y, pt2.y, scale)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_real_comparisons() {
        // Equals
        assert!(real_is_equal(1., 1.));
        assert!(real_is_equal(1000000., 1000000.));
        assert!(real_is_equal(f64::INFINITY, f64::INFINITY));
        assert!(!real_is_equal(f64::INFINITY, f64::NEG_INFINITY));
        assert!(real_is_equal(f64::EPSILON, f64::EPSILON));
        assert!(!real_is_equal(f64::EPSILON, -f64::EPSILON));
        assert!(real_is_equal(f64::EPSILON, 0.));
        assert!(real_is_equal(0., f64::EPSILON));

        // Less than
        assert!(real_lte(1., 1.));
        assert!(real_lte(1000000., 1000000.));
        assert!(real_lte(f64::INFINITY, f64::INFINITY));
        assert!(!real_lt(1., 1.));
        assert!(!real_lt(1000000., 1000000.));
        assert!(!real_lt(f64::INFINITY, f64::INFINITY));
        assert!(real_lt(0., f64::INFINITY));
        assert!(real_lt(f64::NEG_INFINITY, 0.));
        assert!(real_lt(-1., 1.));
        assert!(real_lt(f64::NEG_INFINITY, f64::INFINITY));

        // Greater than
        assert!(real_gte(1., 1.));
        assert!(real_gte(1000000., 1000000.));
        assert!(real_gte(f64::INFINITY, f64::INFINITY));
        assert!(!real_gt(1., 1.));
        assert!(!real_gt(1000000., 1000000.));
        assert!(!real_gt(f64::INFINITY, f64::INFINITY));
        assert!(real_gt(f64::INFINITY, 0.));
        assert!(real_gt(0., f64::NEG_INFINITY));
        assert!(real_gt(1., -1.));
        assert!(real_gt(f64::INFINITY, f64::NEG_INFINITY));
    }

    #[test]
    fn test_point_comparisons() {
        assert!(point_is_equal(Point::new(0., 0.), Point::new(0., 0.)));
        assert!(point_is_equal(Point::new(1000000., 1000000.), Point::new(1000000., 1000000.)));
        assert!(point_is_equal(Point::new(f64::INFINITY, f64::INFINITY), Point::new(f64::INFINITY, f64::INFINITY)));
        assert!(!point_is_equal(Point::new(0., 0.), Point::new(1., 1.)));
        assert!(!point_is_equal(Point::new(f64::INFINITY, f64::INFINITY), Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY)));
        assert!(point_is_equal(Point::new(0., 0.), Point::new(f64::EPSILON, f64::EPSILON)));
    }
}