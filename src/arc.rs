use crate::{BezPath, Vec2, PathEl};
use std::f64::consts::{FRAC_PI_2, PI};

/// A single arc segment.
#[derive(Clone, Copy, Debug)]
pub struct Arc {
    pub center: Point,
    pub radii: Vec2,
    pub start_angle: f64,
    pub sweep_angle: f64,
    pub x_rotation: f64,
}

impl Arc {
    /// Create an iterator generating Bezier path elements.
    ///
    /// The generated elemets can be append to an existing bezier path.
    pub fn append_iter(&self, tolerance: f64) -> ArcAppendIter {
        let scaled_err = self.radii.x.max(self.radii.y) / tolerance;
        // Number of subdivisions per circle based on error tolerance.
        // Note: this may slightly underestimate the error for quadrants.
        let n_err = (1.1163 * scaled_err).powf(1.0 / 6.0).max(3.999_999);
        let n = (n_err * self.sweep_angle.abs() * (1.0 / (2.0 * PI))).ceil();
        let angle_step = self.sweep_angle / n;
        let n = n as usize;
        let arm_len = (4.0 / 3.0) * (0.25 * angle_step).abs().tan();
        let mut angle0 = self.start_angle;
        let mut p0 = sample_ellipse(self.radii, self.x_rotation, angle0);

        ArcAppendIter {
            idx: 0,

            center: self.center,
            radii: self.radii,
            x_rotation: self.x_rotation,
            n,
            arm_len,
            angle_step,

            p0,
            angle0,
        }
    }

    /// Converts an Arc into a series of cubic bezier segments.
    ///
    /// Closure will be invoked for each segment.
    pub fn to_cubic_beziers<P>(self, tolerance: f64, mut p: P)
    where
        P: FnMut(Point, Point, Point),
    {
        let path = self.append_iter(tolerance);
        while let Some(PathEl::CurveTo(p1, p2, p3)) = path.next() {
            p(p1, p2, p3);
        }
    }
}

#[doc(hidden)]
pub struct ArcAppendIter {
    idx: usize,

    center: Vec2,
    radii: Vec2,
    x_rotation: f64,
    n: usize,
    arm_len: f64,
    angle_step: f64,

    p0: Vec2,
    angle0: f64,
}

impl Iterator for ArcAppendIter {
    type Item = PathEl;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.n {
            return None;
        }

        let angle1 = self.angle0 + self.angle_step;
        let p0 = self.p0;
        let p1 = p0 + self.arm_len * sample_ellipse(self.radii, self.x_rotation, self.angle0 + FRAC_PI_2);
        let p3 = sample_ellipse(self.radii, self.x_rotation, angle1);
        let p2 = p3 - self.arm_len * sample_ellipse(self.radii, self.x_rotation, angle1 + FRAC_PI_2);

        self.angle0 = angle1;
        self.p0 = p3;
        self.idx += 1;

        Some(PathEl::Curveto(self.center + p1, self.center + p2, self.center + p3))
    }
}

fn sample_ellipse(radii: Vec2, x_rotation: f64, angle: f64) -> Vec2 {
    let u = radii.x * angle.cos();
    let v = radii.y * angle.sin();
    rotate_pt(Vec2::new(u, v), x_rotation)
}

fn rotate_pt(pt: Vec2, angle: f64) -> Vec2 {
    Vec2::new(
        pt.x * angle.cos() - pt.y * angle.sin(),
        pt.x * angle.sin() + pt.y * angle.cos(),
    )
}
