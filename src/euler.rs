use crate::{CubicBez, ParamCurve, ParamCurveArclen, Point};

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
    u -= (1./24.) * t2_2 + (1./160.) * t2_4;
    u += (1./1920.) * t4_4 + (1./10752.) * t4_6 + (1./55296.) * t4_8;
    u -= (1./322560.) * t6_6 + (1./1658880.) * t6_8 + (1./8110080.) * t6_10;
    u += (1./92897280.) * t8_8 + (1./454164480.) * t8_10;
    u -= 2.4464949595157930e-11 * t10_10;
    let mut v = (1./12.) * t1_2;
    v -= (1./480.) * t3_4 + (1./2688.) * t3_6;
    v += (1./53760.) * t5_6 + (1./276480.) * t5_8 + (1./1351680.) * t5_10;
    v -= (1./11612160.) * t7_8 + (1./56770560.) * t7_10;
    v += 2.4464949595157932e-10 * t9_10;
    (u, v)
}

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
pub fn integ_euler_12n(mut k0: f64, mut k1: f64, n: usize) -> (f64, f64) {
    let th1 = k0;
    let th2 = 0.5 * k1;
    let ds =  (n as f64).recip();

    k0 *= ds;
    k1 *= ds;

    let mut x = 0.0;
    let mut y = 0.0;
    let mut s = 0.5 * ds - 0.5;

    for i in 0..n {
        let km0 = k1 * s + k0;
        let km1 = k1 * ds;

        let (u, v) = integ_euler_12(km0, km1);

        let th = (th2 * s + th1) * s;
        let cth = th.cos();
        let sth = th.sin();

        x += cth * u - sth * v;
        y += cth * v + sth * u;
        s += ds;
    }
    (x * ds, y * ds)
}

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
            // of the fresnel integrals is probably the winning strategy.
            8
        };
        integ_euler_12n(k0, k1, n)
    }
}

/// Parameters for an Euler spiral segment. Does not include endpoint geometry.
pub struct FitEulerResult {
    // These can be recovered from k0, k1, and chth, but it's annoying.
    th0: f64,
    th1: f64,
    k0: f64,
    k1: f64,
    chord: f64,
    chth: f64,
}

/// Find the Euler spiral parameters for the given deflection.
pub fn fit_euler(th0: f64, th1: f64) -> FitEulerResult {
    // Note: we could skip the solving for very small deflection
    let mut k1_old = 0.0;
    let dth = th1 - th0;
    let k0 = th0 + th1;
    let mut k1 = (6.0 - (1./70.) * dth * dth - 0.1 * k0 * k0) * dth;
    let mut error_old = dth;
    for _ in 0..10 {
        let (u, v) = integ_euler(k0, k1);
        let chth = v.atan2(u);
        let error = dth - (0.25 * k1 - 2.0 * chth);
        if error.abs() < 1e-9 {
            let chord = u.hypot(v);
            return FitEulerResult { th0, th1, k0, k1, chord, chth };
        }
        let new_k1 = k1 + (k1_old - k1) * error / (error - error_old);
        k1_old = k1;
        error_old = error;
        k1 = new_k1;
    }
    panic!("fit_euler diverged on {}, {}", th0, th1);
}

impl FitEulerResult {
    fn th(&self, t: f64) -> f64 {
        let u = t - 0.5;
        // Maybe sign of following is wrong; this is confusing. But it matches spiro code.
        (0.5 * self.k1 * u + self.k0) * u - self.chth
    }

    /// TODO
    // Param t in [0, 1]. Return value assumes chord (0, 0) - (1, 0)
    pub fn xy(&self, t: f64) -> Point {
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

    /// Calculate error from cubic bezier.
    ///
    /// This is a fairly brute-force technique, finely sampling the bezier and
    /// reporting RMS distance error.
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
            let euler_xy = self.xy(norm_len);
            err += (cubic_xy - euler_xy).hypot2();
        }
        (err / (n as f64)).sqrt()
    }

    /// Estimate the error wrt the cubic.
    ///
    /// It's an experiment to see how useful arclength is. It correlates fairly well,
    /// but can both underestimate and overestimate, so it's better to just make
    /// cubic_euler_err better.
    pub fn est_cubic_err(&self, cubic: CubicBez) -> f64 {
        let cubic_len = cubic.arclen(1e-9);
        let err_arclen = cubic_len * self.chord - 1.0;

        let err = self.cubic_euler_err(cubic, 3);
        err.max(err_arclen)
    }
}
