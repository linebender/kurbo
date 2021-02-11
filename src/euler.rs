use crate::{
    CubicBez, Line, ParamCurve, ParamCurveArclen, ParamCurveCurvature, ParamCurveDeriv, Point, Vec2,
};

/// An Euler spiral segment.
#[derive(Clone, Copy, Debug)]
pub struct EulerSeg {
    p0: Point,
    p1: Point,
    es: FitEulerResult,
}

/// The derivative of an Euler spiral segment.
#[derive(Clone, Copy)]
pub struct EulerSegDeriv {
    c0: f64,
    c1: f64,
    c2: f64,
    scale: f64,
}

/// The second derivative of an Euler spiral segment.
pub struct EulerSegDeriv2(EulerSegDeriv);

/// Parameters for an Euler spiral segment. Does not include endpoint geometry.
///
/// This is something of an internal detail for `EulerSeg` and might not make
/// it to the public interface. It's public here for experimentation.
///
/// It's entirely possible the disposition of this is to be inlined into `EulerSeg`.
/// I'm not sure it's useful by itself.
#[derive(Clone, Copy, Debug)]
pub struct FitEulerResult {
    k0: f64,
    k1: f64,
    chord: f64,
    chth: f64,
}

/// A path consisting of piecewise Euler spiral segments.
pub struct EulerPath(Vec<EulerPathEl>);

/// An element of a piecewise Euler spiral path.
#[derive(Clone, Copy, Debug)]
pub enum EulerPathEl {
    /// Start a new subpath at the given point.
    MoveTo(Point),
    /// A line segment to the given point.
    LineTo(Point),
    /// An Euler spiral segment to the given point.
    EulerTo(FitEulerResult, Point),
    /// Close the subpath.
    ClosePath,
}

/// An iterator producing euler segments from a cubic bezier.
///
/// Discussion: should this be an anonymous (`from_fn`) type?
pub struct CubicToEulerIter {
    c: CubicBez,
    tolerance: f64,
    // [t0 * dt .. (t0 + 1) * dt] is the range we're
    // currently considering.
    t0: u64,
    dt: f64,
}

fn integ_euler_12(k0: f64, k1: f64) -> (f64, f64) {
    let t1_1 = k0;
    let t1_2 = 0.5 * k1;
    let t2_2 = t1_1 * t1_1;
    let t2_3 = 2. * (t1_1 * t1_2);
    let t2_4 = t1_2 * t1_2;
    let t3_4 = t2_2 * t1_2 + t2_3 * t1_1;
    let t3_6 = t2_4 * t1_2;
    let t4_4 = t2_2 * t2_2;
    let t4_5 = 2. * (t2_2 * t2_3);
    let t4_6 = 2. * (t2_2 * t2_4) + t2_3 * t2_3;
    let t4_7 = 2. * (t2_3 * t2_4);
    let t4_8 = t2_4 * t2_4;
    let t5_6 = t4_4 * t1_2 + t4_5 * t1_1;
    let t5_8 = t4_6 * t1_2 + t4_7 * t1_1;
    let t5_10 = t4_8 * t1_2;
    let t6_6 = t4_4 * t2_2;
    let t6_7 = t4_4 * t2_3 + t4_5 * t2_2;
    let t6_8 = t4_4 * t2_4 + t4_5 * t2_3 + t4_6 * t2_2;
    let t6_9 = t4_5 * t2_4 + t4_6 * t2_3 + t4_7 * t2_2;
    let t6_10 = t4_6 * t2_4 + t4_7 * t2_3 + t4_8 * t2_2;
    let t7_8 = t6_6 * t1_2 + t6_7 * t1_1;
    let t7_10 = t6_8 * t1_2 + t6_9 * t1_1;
    let t8_8 = t6_6 * t2_2;
    let t8_9 = t6_6 * t2_3 + t6_7 * t2_2;
    let t8_10 = t6_6 * t2_4 + t6_7 * t2_3 + t6_8 * t2_2;
    let t9_10 = t8_8 * t1_2 + t8_9 * t1_1;
    let t10_10 = t8_8 * t2_2;
    let mut u = 1.;
    u -= (1. / 24.) * t2_2 + (1. / 160.) * t2_4;
    u += (1. / 1920.) * t4_4 + (1. / 10752.) * t4_6 + (1. / 55296.) * t4_8;
    u -= (1. / 322560.) * t6_6 + (1. / 1658880.) * t6_8 + (1. / 8110080.) * t6_10;
    u += (1. / 92897280.) * t8_8 + (1. / 454164480.) * t8_10;
    u -= 2.4464949595157930e-11 * t10_10;
    let mut v = (1. / 12.) * t1_2;
    v -= (1. / 480.) * t3_4 + (1. / 2688.) * t3_6;
    v += (1. / 53760.) * t5_6 + (1. / 276480.) * t5_8 + (1. / 1351680.) * t5_10;
    v -= (1. / 11612160.) * t7_8 + (1. / 56770560.) * t7_10;
    v += 2.4464949595157932e-10 * t9_10;
    (u, v)
}

/// A cheaper approximation to the Euler spiral integral. This might be useful
/// if we have lots of low-deflection segments, but it's not clear it'll be a
/// whole lot faster.
/*
pub fn integ_euler_8(k0: f64, k1: f64) -> (f64, f64) {
    let t1_1 = k0;
    let t1_2 = 0.5 * k1;
    let t2_2 = t1_1 * t1_1;
    let t2_3 = 2. * (t1_1 * t1_2);
    let t2_4 = t1_2 * t1_2;
    let t3_4 = t2_2 * t1_2 + t2_3 * t1_1;
    let t3_6 = t2_4 * t1_2;
    let t4_4 = t2_2 * t2_2;
    let t4_5 = 2. * (t2_2 * t2_3);
    let t4_6 = 2. * (t2_2 * t2_4) + t2_3 * t2_3;
    let t5_6 = t4_4 * t1_2 + t4_5 * t1_1;
    let t6_6 = t4_4 * t2_2;
    let mut u = 1.;
    u -= (1./24.) * t2_2 + (1./160.) * t2_4;
    u += (1./1920.) * t4_4 + (1./10752.) * t4_6;
    u -= (1./322560.) * t6_6;
    let mut v = (1./12.) * t1_2;
    v -= (1./480.) * t3_4 + (1./2688.) * t3_6;
    v += (1./53760.) * t5_6;
    (u, v)
}
*/

#[doc(hidden)]
/// Computation of the Euler spiral integral using subdivision.
pub fn integ_euler_12n(mut k0: f64, mut k1: f64, n: usize) -> (f64, f64) {
    let th1 = k0;
    let th2 = 0.5 * k1;
    let ds = (n as f64).recip();

    k0 *= ds;
    k1 *= ds;

    let mut x = 0.0;
    let mut y = 0.0;
    let s0 = 0.5 * ds - 0.5;

    for i in 0..n {
        let s = s0 + ds * (i as f64);
        let km0 = k1 * s + k0;
        let km1 = k1 * ds;

        let (u, v) = integ_euler_12(km0, km1);

        let th = (th2 * s + th1) * s;
        let cth = th.cos();
        let sth = th.sin();

        x += cth * u - sth * v;
        y += cth * v + sth * u;
    }
    (x * ds, y * ds)
}

/// The Euler spiral integral.
///
/// This approximates Equation 8.1 of the thesis.
///
/// Discussion question: should this take an accuracy parameter?
/// This version basically hardcodes 1e-9. The cost of accuracy is
/// expected to be modest; the polynomial approximation converges
/// very rapidly and we expect small deflections, especially for
/// piecewise Euler spiral representations of other curves.
fn integ_euler(k0: f64, k1: f64) -> (f64, f64) {
    let est_err_raw = 0.2 * k0 * k0 + k1.abs();
    if est_err_raw < 1.01 {
        integ_euler_12(k0, k1)
    } else {
        let n = if est_err_raw < 3.96 {
            2
        } else if est_err_raw < 8.23 {
            3
        } else if est_err_raw < 14.2 {
            4
        } else {
            // Maybe determine the formula?
            // Also, for these huge deflections, cephes-style computation
            // of the Fresnel integrals is probably the winning strategy.
            8
        };
        integ_euler_12n(k0, k1, n)
    }
}

impl FitEulerResult {
    /// Find the Euler spiral parameters for the given deflection.
    ///
    /// TODO: use research for direct solution.
    ///
    /// Discussion question: should this take an accuracy parameter?
    /// This version basically hardcodes 1e-9.
    pub fn fit_euler(th0: f64, th1: f64) -> FitEulerResult {
        // Note: we could skip the solving for very small deflection
        let mut k1_old = 0.0;
        let dth = th1 - th0;
        let k0 = th0 + th1;
        let mut k1 = (6.0 - (1. / 70.) * dth * dth - 0.1 * k0 * k0) * dth;
        let mut error_old = dth;
        for _ in 0..10 {
            let (u, v) = integ_euler(k0, k1);
            let chth = v.atan2(u);
            let error = dth - (0.25 * k1 - 2.0 * chth);
            if error.abs() < 1e-9 {
                let chord = u.hypot(v);
                return FitEulerResult {
                    k0,
                    k1,
                    chord,
                    chth,
                };
            }
            let new_k1 = k1 + (k1_old - k1) * error / (error - error_old);
            k1_old = k1;
            error_old = error;
            k1 = new_k1;
        }
        panic!("fit_euler diverged on {}, {}", th0, th1);
    }

    /// Create a `FitEulerResult` from k0 and k1 parameters.
    pub fn from_k0_k1(k0: f64, k1: f64) -> FitEulerResult {
        let (u, v) = integ_euler(k0, k1);
        let chth = v.atan2(u);
        let chord = u.hypot(v);
        FitEulerResult {
            k0,
            k1,
            chord,
            chth,
        }
    }

    /// Determine tangent angle at the given parameter.
    ///
    /// The sign may be confusing, but it matches the spiro code. When `t = 0`,
    /// the result is `-th0`, and when `t = 1`, the result is `th1`.
    fn th(&self, t: f64) -> f64 {
        let u = t - 0.5;
        (0.5 * self.k1 * u + self.k0) * u - self.chth
    }

    /// Evaluate the curve at the given parameter.
    ///
    /// The parameter is in the range 0..1, and the result goes from (0, 0) to (1, 0).
    pub fn eval(&self, t: f64) -> Point {
        let thm = self.th(t * 0.5);
        let k0 = self.k0;
        let k1 = self.k1;
        let (u, v) = integ_euler((k0 + k1 * 0.5 * (t - 1.0)) * t, k1 * t * t);
        let s = t / self.chord * thm.sin();
        let c = t / self.chord * thm.cos();
        let x = u * c - v * s;
        let y = -v * c - u * s;
        Point::new(x, y)
    }
}

impl EulerSeg {
    /// Create a new Euler segment.
    ///
    /// TODO: document the conventions. An SVG would be especially nice.
    pub fn new(p0: Point, p1: Point, th0: f64, th1: f64) -> EulerSeg {
        let es = FitEulerResult::fit_euler(th0, th1);
        EulerSeg { p0, p1, es }
    }

    /// Create an Euler segment from a cubic Bézier.
    ///
    /// The curve is fit according to G1 geometric Hermite interpolation, in
    /// other words the endpoints and tangents match the given curve.
    pub fn from_cubic(c: CubicBez) -> EulerSeg {
        let d01 = c.p1 - c.p0;
        let d23 = c.p3 - c.p2;
        let d03 = c.p3 - c.p0;
        let th0 = d03.cross(d01).atan2(d03.dot(d01));
        let th1 = d23.cross(d03).atan2(d23.dot(d03));
        let es = FitEulerResult::fit_euler(th0, th1);
        EulerSeg {
            p0: c.p0,
            p1: c.p3,
            es,
        }
    }

    /// Calculate error from cubic bezier.
    ///
    /// This is a fairly brute-force technique, sampling the bezier and
    /// reporting RMS distance error.
    ///
    /// Note: experimentation suggests that n = 4 is enough to estimate the
    /// error fairly accurately. A future evolution may get rid of that
    /// parameter, and possibly also a tolerance parameter.
    pub fn cubic_euler_err(&self, cubic: CubicBez, n: usize) -> f64 {
        // One way to improve this would be compute arclengths for each segment
        // and cumulative sum them, rather than from 0 each time.
        //
        // We can also consider Legendre-Gauss quadrature.
        //
        // It's likely a rough approximation to arclength will be effective
        // here, for example LGQ with a low order.
        let cubic_len = cubic.arclen(1e-9);
        let mut err = 0.0;
        for i in 0..n {
            let t = (i + 1) as f64 / ((n + 1) as f64);
            let norm_len = cubic.subsegment(0.0..t).arclen(1e-9) / cubic_len;
            let cubic_xy = cubic.eval(t);
            let euler_xy = self.eval(norm_len);
            err += (cubic_xy - euler_xy).hypot2();
        }
        (err / (n as f64)).sqrt()
    }

    /// Report whether the segment is a straight line.
    pub fn is_line(&self) -> bool {
        self.es.k0 == 0.0 && self.es.k1 == 0.0
    }
}

impl ParamCurve for EulerSeg {
    fn eval(&self, t: f64) -> Point {
        let Point { x, y } = self.es.eval(t);
        let chord = self.p1 - self.p0;
        Point::new(
            self.p0.x + chord.x * x - chord.y * y,
            self.p0.y + chord.x * y + chord.y * x,
        )
    }

    fn subsegment(&self, range: std::ops::Range<f64>) -> Self {
        let p0 = self.eval(range.start);
        let p1 = self.eval(range.end);
        let dt = range.end - range.start;
        let k0 = dt * (self.es.k0 + 0.5 * (range.start + range.end - 1.0) * self.es.k1);
        let k1 = dt * dt * self.es.k1;
        let es = FitEulerResult::from_k0_k1(k0, k1);
        EulerSeg { p0, p1, es }
    }

    fn start(&self) -> Point {
        self.p0
    }

    fn end(&self) -> Point {
        self.p1
    }
}

impl ParamCurveArclen for EulerSeg {
    /// The arc length of the curve.
    ///
    /// Note that this implementation is fast and accurate.
    fn arclen(&self, _accuracy: f64) -> f64 {
        (self.p1 - self.p0).hypot() / self.es.chord
    }

    /// The parameter that results in the given arc length.
    ///
    /// This implementation is also fast and accurate.
    fn inv_arclen(&self, arclen: f64, _accuracy: f64) -> f64 {
        arclen * self.es.chord / (self.p1 - self.p0).hypot()
    }
}

impl ParamCurveDeriv for EulerSeg {
    type DerivResult = EulerSegDeriv;

    fn deriv(&self) -> Self::DerivResult {
        let FitEulerResult { k0, k1, chth, .. } = self.es;
        EulerSegDeriv {
            c0: 0.5 * k0 - 0.125 * k1 + chth + (self.p1 - self.p0).atan2(),
            c1: -k0 + 0.5 * k1,
            c2: -0.5 * k1,
            scale: self.arclen(0.0),
        }
    }
}

impl ParamCurveCurvature for EulerSeg {
    fn curvature(&self, t: f64) -> f64 {
        (self.es.k0 + (t - 0.5) * self.es.k1) * self.es.chord / (self.p1 - self.p0).hypot()
    }
}

impl ParamCurve for EulerSegDeriv {
    fn eval(&self, t: f64) -> Point {
        let theta = self.c0 + t * self.c1 + t * t * self.c2;
        (self.scale * Vec2::from_angle(theta)).to_point()
    }

    fn subsegment(&self, range: std::ops::Range<f64>) -> Self {
        let t0 = range.start;
        let t1 = range.end;
        let dt = t1 - t0;
        EulerSegDeriv {
            c0: self.c0 + t0 * self.c1 + t0 * t0 * self.c2,
            c1: dt * (self.c1 + t0 * self.c2),
            c2: dt * dt * self.c2,
            scale: dt * self.scale,
        }
    }
}

impl ParamCurveDeriv for EulerSegDeriv {
    type DerivResult = EulerSegDeriv2;

    fn deriv(&self) -> Self::DerivResult {
        EulerSegDeriv2(self.clone())
    }
}

impl ParamCurve for EulerSegDeriv2 {
    fn eval(&self, t: f64) -> Point {
        let p = self.0.eval(t);
        let scale = self.0.c1 + 2.0 * t * self.0.c2;
        Point::new(-p.y * scale, p.x * scale)
    }

    fn subsegment(&self, range: std::ops::Range<f64>) -> Self {
        EulerSegDeriv2(self.0.subsegment(range))
    }
}

// TODO: other ParamCurve traits.

impl From<Line> for EulerSeg {
    fn from(l: Line) -> EulerSeg {
        EulerSeg {
            p0: l.p0,
            p1: l.p1,
            es: FitEulerResult {
                k0: 0.,
                k1: 0.,
                chord: 1.,
                chth: 0.,
            },
        }
    }
}

impl CubicToEulerIter {
    /// Subdivide a cubic Bézier into piecewise Euler spirals.
    pub fn new(c: CubicBez, tolerance: f64) -> CubicToEulerIter {
        CubicToEulerIter {
            c,
            tolerance,
            t0: 0,
            dt: 1.0,
        }
    }
}

impl Iterator for CubicToEulerIter {
    type Item = EulerSeg;

    fn next(&mut self) -> Option<Self::Item> {
        let t0 = (self.t0 as f64) * self.dt;
        if t0 == 1.0 {
            return None;
        }
        loop {
            let t1 = t0 + self.dt;
            let cubic = self.c.subsegment(t0..t1);
            let es = EulerSeg::from_cubic(cubic);
            let err: f64 = es.cubic_euler_err(cubic, 4);
            if err <= self.tolerance {
                self.t0 += 1;
                let shift = self.t0.trailing_zeros();
                self.t0 >>= shift;
                self.dt *= (1 << shift) as f64;
                return Some(es);
            }
            self.t0 *= 2;
            self.dt *= 0.5;
            // TODO: should probably limit recursion here.
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        CubicBez, CubicToEulerIter, EulerSeg, ParamCurve, ParamCurveCurvature, ParamCurveDeriv,
        ParamCurveNearest, Point,
    };

    #[test]
    fn euler_deriv() {
        let es = EulerSeg::new(Point::ORIGIN, Point::new(1., 1.), 0.2, 0.3);
        let esd = es.deriv();
        const DELTA: f64 = 0.001;
        for i in 0..11 {
            let t = (i as f64) * 0.1;
            let est_deriv = (es.eval(t + 0.5 * DELTA) - es.eval(t - 0.5 * DELTA)) * DELTA.recip();
            let computed_deriv = esd.eval(t);
            assert!((est_deriv.to_point() - computed_deriv).hypot() < 1e-7);
        }
    }

    #[test]
    fn euler_deriv_2() {
        let es = EulerSeg::new(Point::ORIGIN, Point::new(1., 1.), 0.2, 0.3);
        let esd = es.deriv();
        let esd2 = esd.deriv();
        const DELTA: f64 = 0.001;
        for i in 0..11 {
            let t = (i as f64) * 0.1;
            let est_deriv = (esd.eval(t + 0.5 * DELTA) - esd.eval(t - 0.5 * DELTA)) * DELTA.recip();
            let computed_deriv = esd2.eval(t);
            assert!((est_deriv.to_point() - computed_deriv).hypot() < 1e-7);
        }
    }

    #[test]
    fn euler_curvature() {
        let th = std::f64::consts::FRAC_PI_2;
        let es = EulerSeg::new(Point::ORIGIN, Point::new(0., 1.), th, th);
        for i in 0..11 {
            let t = (i as f64) * 0.1;
            assert!((es.curvature(t) - 2.0).abs() < 1e-9);
        }
    }

    #[test]
    fn euler_from_cubic() {
        let c = CubicBez::new((0., 0.), (1., 1.1), (2., 2.2), (3., 3.));
        let es = EulerSeg::from_cubic(c);
        // Check endpoints match. These should actually be bit-identical.
        assert!((es.p0 - c.p0).hypot() < 1e-15);
        assert!((es.p1 - c.p3).hypot() < 1e-15);
        // Check tangents match. This can have some error but should be small.
        let des = es.deriv();
        let dc = c.deriv();
        assert!(dc.eval(0.0).to_vec2().atan2() - des.eval(0.0).to_vec2().atan2() < 1e-9);
        assert!(dc.eval(1.0).to_vec2().atan2() - des.eval(1.0).to_vec2().atan2() < 1e-9);
    }

    #[test]
    fn cubic_to_euler() {
        const TOLERANCE: f64 = 1e-4;
        let c = CubicBez::new((0., 0.), (1., 1.1), (2., 2.2), (3., 3.));
        for es in CubicToEulerIter::new(c, TOLERANCE) {
            for i in 0..11 {
                let t = (i as f64) * 0.1;
                assert!(c.nearest(es.eval(t), 1e-9).distance_sq < TOLERANCE.powi(2));
            }
        }
    }
}
