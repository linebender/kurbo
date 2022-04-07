//! A trait for clipping parametrized curves.

use crate::common::{solve_cubic, solve_quadratic, solve_linear};
use crate::{CubicBez, QuadBez, Line, Point, ParamCurve, ParamCurveBezierClipping};
use arrayvec::ArrayVec;

// Note that the line is unbounded here!
fn signed_distance_from_ray_to_point(l: &Line, p: Point) -> f64 {
    let vec2 = l.p1 - l.p0;
    let a = -vec2.y;
    let b = vec2.x;
    let c = -(a * l.start().x + b * l.start().y);

    let len = (a * a + b * b).sqrt();
    let a = a / len;
    let b = b / len;
    let c = c / len;

    if a.is_infinite() || b.is_infinite() || c.is_infinite()
    {
        // Can't compute distance from zero-length line, so return distance
        // from p0 instead
        return (p - l.p0).hypot();
    }

    a * p.x + b * p.y + c
}

impl ParamCurveBezierClipping for Line {
    fn solve_t_for_x(&self, x: f64) -> ArrayVec<f64, 3> {
        if (self.p0.x - self.p1.x).abs() < f64::EPSILON {
            return ArrayVec::new();
        }
        let (a, b) = self.parameters();
        solve_linear(b.x - x, a.x)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }
    fn solve_t_for_y(&self, y: f64) -> ArrayVec<f64, 3> {
        if (self.p0.y - self.p1.y).abs() < f64::EPSILON {
            return ArrayVec::new();
        }

        let (a, b) = self.parameters();
        solve_linear(b.y - y, a.y)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }

    fn fat_line_min_max(&self, baseline: &Line) -> (f64, f64) {
        // Get the signed distance to each point
        let d0 = signed_distance_from_ray_to_point(&baseline, self.p0);
        let d1 = signed_distance_from_ray_to_point(&baseline, self.p1);

        // Calculate min and max distances
        let d_min = f64::min(d0.min(d1), 0.0);
        let d_max = f64::max(d0.max(d1), 0.0);

        (d_min, d_max)
    }

    fn convex_hull_from_line(&self, l: &Line) -> (Vec<Point>, Vec<Point>) {
        let d0 = signed_distance_from_ray_to_point(l, self.start());
        let d1 = signed_distance_from_ray_to_point(l, self.end());
        
        let p0 = Point::new(0.0, d0);
        let p1 = Point::new(1.0, d1);

        // The hull is simply the line itself
        (vec![p0, p1], vec![p0, p1])
    }
}

impl ParamCurveBezierClipping for QuadBez {
    fn solve_t_for_x(&self, x: f64) -> ArrayVec<f64, 3> {
        if self.is_linear(f64::EPSILON) && (self.p0.x - self.p2.x).abs() < f64::EPSILON {
            return ArrayVec::new();
        }
        let (a, b, c) = self.parameters();
        solve_quadratic(c.x - x, b.x, a.x)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }
    fn solve_t_for_y(&self, y: f64) -> ArrayVec<f64, 3> {
        if self.is_linear(f64::EPSILON) && (self.p0.y - self.p2.y).abs() < f64::EPSILON {
            return ArrayVec::new();
        }

        let (a, b, c) = self.parameters();
        solve_quadratic(c.y - y, b.y, a.y)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }

    fn fat_line_min_max(&self, baseline: &Line) -> (f64, f64) {
        // Get the signed distance to each point
        let d0 = signed_distance_from_ray_to_point(&baseline, self.p0);
        let d1 = signed_distance_from_ray_to_point(&baseline, self.p1);
        let d2 = signed_distance_from_ray_to_point(&baseline, self.p2);

        // Shrink d1 to a more accurate number, but test the distance compared
        // to d0 and d3 in case we're looking at a perpendicular fat line
        let factor = 1.0 / 2.0;
        //let factor = 1.0;
        let d1_1 = (d1 - d0) * factor + d0;
        let d1_2 = (d1 - d2) * factor + d2;
        // let d1_1 = d1;
        // let d1_2 = d1;

        // Calculate min and max distances
        let d_min = f64::min(d0.min(d1_1).min(d1_2).min(d2), 0.0);
        let d_max = f64::max(d0.max(d1_1).max(d1_2).max(d2), 0.0);

        (d_min, d_max)
    }

    fn convex_hull_from_line(&self, l: &Line) -> (Vec<Point>, Vec<Point>) {
        let d0 = signed_distance_from_ray_to_point(l, self.start());
        let d1 = signed_distance_from_ray_to_point(l, self.p1);
        let d2 = signed_distance_from_ray_to_point(l, self.end());

        let p0 = Point::new(0.0, d0);
        let p1 = Point::new(1.0 / 2.0, d1);
        let p2 = Point::new(1.0, d2);
        // Compute the vertical signed distance of p1 and p2 from [p0, p3].
        let dist1 = d1 - (d0 + d2) / 2.0;

        // Compute the hull assuming p1 is on top - we'll switch later if needed.
        let mut hull = (vec![p0, p1, p2], vec![p0, p2]);

        // Flip the hull if needed:
        if dist1 < 0.0 {
            hull = (hull.1, hull.0);
        }

        hull
    }
}

impl ParamCurveBezierClipping for CubicBez {
    fn solve_t_for_x(&self, x: f64) -> ArrayVec<f64, 3> {
        if self.is_linear(f64::EPSILON) && (self.p0.x - self.p3.x).abs() < f64::EPSILON {
            return ArrayVec::new();
        }
        let (a, b, c, d) = self.parameters();
        solve_cubic(d.x - x, c.x, b.x, a.x)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }
    fn solve_t_for_y(&self, y: f64) -> ArrayVec<f64, 3> {
        if self.is_linear(f64::EPSILON) && (self.p0.y - self.p3.y).abs() < f64::EPSILON {
            return ArrayVec::new();
        }

        let (a, b, c, d) = self.parameters();
        solve_cubic(d.y - y, c.y, b.y, a.y)
            .iter()
            .copied()
            .filter(|&t| (0.0..=1.0).contains(&t))
            .collect()
    }

    fn fat_line_min_max(&self, baseline: &Line) -> (f64, f64) {
        // Get the signed distance to each point
        let d0 = signed_distance_from_ray_to_point(&baseline, self.p0);
        let d1 = signed_distance_from_ray_to_point(&baseline, self.p1);
        let d2 = signed_distance_from_ray_to_point(&baseline, self.p2);
        let d3 = signed_distance_from_ray_to_point(&baseline, self.p3);

        // Shrink d1 and d2 to more accurate numbers
        let factor: f64 = if ((d1 - d0) * (d2 - d3)) > 0.0 {
            3.0 / 4.0
        } else {
            4.0 / 9.0
        };
        let d1 = (d1 - d0) * factor + d0;
        let d2 = (d2 - d3) * factor + d3;

        // Calculate min and max distances
        let d_min = f64::min(d0.min(d1).min(d2).min(d3), 0.0);
        let d_max = f64::max(d0.max(d1).max(d2).max(d3), 0.0);

        (d_min, d_max)
    }

    fn convex_hull_from_line(&self, l: &Line) -> (Vec<Point>, Vec<Point>) {
        let d0 = signed_distance_from_ray_to_point(l, self.start());
        let d1 = signed_distance_from_ray_to_point(l, self.p1);
        let d2 = signed_distance_from_ray_to_point(l, self.p2);
        let d3 = signed_distance_from_ray_to_point(l, self.end());

        let p0 = Point::new(0.0, d0);
        let p1 = Point::new(1.0 / 3.0, d1);
        let p2 = Point::new(2.0 / 3.0, d2);
        let p3 = Point::new(1.0, d3);
        // Compute the vertical signed distance of p1 and p2 from [p0, p3].
        let dist1 = d1 - (2.0 * d0 + d3) / 3.0;
        let dist2 = d2 - (d0 + 2.0 * d3) / 3.0;

        // Compute the hull assuming p1 is on top - we'll switch later if needed.
        let mut hull = if dist1 * dist2 < 0.0 {
            // p1 and p2 lie on opposite sides of [p0, p3], so the hull is a quadrilateral:
            (vec![p0, p1, p3], vec![p0, p2, p3])
        } else {
            // p1 and p2 lie on the same side of [p0, p3]. The hull can be a triangle or a
            // quadrilateral, and [p0, p3] is part of the hull. The hull is a triangle if the vertical
            // distance of one of the middle points p1, p2 is <= half the vertical distance of the
            // other middle point.
            let dist1 = dist1.abs();
            let dist2 = dist2.abs();
            if dist1 >= 2.0 * dist2 {
                (vec![p0, p1, p3], vec![p0, p3])
            } else if dist2 >= 2.0 * dist1 {
                (vec![p0, p2, p3], vec![p0, p3])
            } else {
                (vec![p0, p1, p2, p3], vec![p0, p3])
            }
        };

        // Flip the hull if needed:
        if dist1 < 0.0 || (dist1 == 0.0 && dist2 < 0.0) {
            hull = (hull.1, hull.0);
        }

        hull
    }
}
