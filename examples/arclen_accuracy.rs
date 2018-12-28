//! A test program to plot the error of arclength approximation.

// TODO: make more functionality accessible from command line rather than uncommenting.

use std::env;
use std::time::Instant;

use kurbo::{ParamCurve, ParamCurveArclen, ParamCurveDeriv, QuadBez};

/// Calculate arclength using Gauss-Legendre quadrature using formula from Behdad
/// in https://github.com/Pomax/BezierInfo-2/issues/77
fn gauss_arclen_3(q: QuadBez) -> f64 {
    let v0 = (-0.492943519233745*q.p0 + 0.430331482911935*q.p1 + 0.0626120363218102*q.p2).hypot();
    let v1 = ((q.p2-q.p0)*0.4444444444444444).hypot();
    let v2 = (-0.0626120363218102*q.p0 - 0.430331482911935*q.p1 + 0.492943519233745*q.p2).hypot();
    v0 + v1 + v2
}

fn awesome_quad_arclen(q: QuadBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    let pm = (q.p0 + q.p2) / 2.0;
    let d1 = q.p1 - pm;
    let d = q.p2 - q.p0;
    let dhypot2 = d.hypot2();
    let x = 2.0 * d.dot(d1) / dhypot2;
    let y = 2.0 * d.cross(d1) / dhypot2;
    let lc = (q.p2 - q.p0).hypot();
    let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
    let est_err = 0.06 * (lp - lc) * (x * x + y * y).powf(2.0);
    if est_err < accuracy || depth == 16 {
        *count += 1;
        gauss_arclen_3(q)
    } else {
        let (q0, q1) = q.subdivide();
        awesome_quad_arclen(q0, accuracy * 0.5, depth + 1, count)
            + awesome_quad_arclen(q1, accuracy * 0.5, depth + 1, count)
    }
}

const MAX_DEPTH: usize = 16;
fn gravesen_rec(q: &QuadBez, l0: f64, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    let (q0, q1) = q.subdivide();
    let l0_q0 = calc_l0(q0);
    let l0_q1 = calc_l0(q1);
    let l1 = l0_q0 + l0_q1;
    let error = (l0 - l1) * (1.0 / 15.0);
    if error.abs() < accuracy || depth == MAX_DEPTH {
        *count += 1;
        l1 - error
    } else {
        gravesen_rec(&q0, l0_q0, accuracy * 0.5, depth + 1, count)
            + gravesen_rec(&q1, l0_q1, accuracy * 0.5, depth + 1, count)
    }
}

/// Another implementation, using weights from
/// https://pomax.github.io/bezierinfo/legendre-gauss.html
fn gauss_arclen_n<C: ParamCurveDeriv>(c: C, coeffs: &[(f64, f64)]) -> f64 {
    let d = c.deriv();
    coeffs.iter().map(|(wi, xi)|
        wi * d.eval(0.5 * (xi + 1.0)).hypot()
    ).sum::<f64>() * 0.5
}

fn gauss_arclen_5(q: QuadBez) -> f64 {
    gauss_arclen_n(q, &[
        (0.5688888888888889,  0.0000000000000000),
        (0.4786286704993665,  -0.5384693101056831),
        (0.4786286704993665,  0.5384693101056831),
        (0.2369268850561891,  -0.9061798459386640),
        (0.2369268850561891,  0.9061798459386640),
    ])
}

fn gauss_arclen_7(q: QuadBez) -> f64 {
    gauss_arclen_n(q, &[
        (0.4179591836734694,  0.0000000000000000),
        (0.3818300505051189,  0.4058451513773972),
        (0.3818300505051189,  -0.4058451513773972),
        (0.2797053914892766,  -0.7415311855993945),
        (0.2797053914892766,  0.7415311855993945),
        (0.1294849661688697,  -0.9491079123427585),
        (0.1294849661688697,  0.9491079123427585),
    ])
}

fn gauss_arclen_24(q: QuadBez) -> f64 {
    gauss_arclen_n(q, &[
        (0.1279381953467522,  -0.0640568928626056),
        (0.1279381953467522,  0.0640568928626056),
        (0.1258374563468283,  -0.1911188674736163),
        (0.1258374563468283,  0.1911188674736163),
        (0.1216704729278034,  -0.3150426796961634),
        (0.1216704729278034,  0.3150426796961634),
        (0.1155056680537256,  -0.4337935076260451),
        (0.1155056680537256,  0.4337935076260451),
        (0.1074442701159656,  -0.5454214713888396),
        (0.1074442701159656,  0.5454214713888396),
        (0.0976186521041139,  -0.6480936519369755),
        (0.0976186521041139,  0.6480936519369755),
        (0.0861901615319533,  -0.7401241915785544),
        (0.0861901615319533,  0.7401241915785544),
        (0.0733464814110803,  -0.8200019859739029),
        (0.0733464814110803,  0.8200019859739029),
        (0.0592985849154368,  -0.8864155270044011),
        (0.0592985849154368,  0.8864155270044011),
        (0.0442774388174198,  -0.9382745520027328),
        (0.0442774388174198,  0.9382745520027328),
        (0.0285313886289337,  -0.9747285559713095),
        (0.0285313886289337,  0.9747285559713095),
        (0.0123412297999872,  -0.9951872199970213),
        (0.0123412297999872,  0.9951872199970213),
    ])
}

fn awesome_quad_arclen7(q: QuadBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    let pm = (q.p0 + q.p2) / 2.0;
    let d1 = q.p1 - pm;
    let d = q.p2 - q.p0;
    let dhypot2 = d.hypot2();
    let x = 2.0 * d.dot(d1) / dhypot2;
    let y = 2.0 * d.cross(d1) / dhypot2;
    let lc = (q.p2 - q.p0).hypot();
    let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
    let est_err = 2.5e-2 * (lp - lc) * (x * x + y * y).powf(8.0).tanh();
    if est_err < accuracy || depth == 16 {
        *count += 1;
        gauss_arclen_7(q)
    } else {
        let (q0, q1) = q.subdivide();
        awesome_quad_arclen7(q0, accuracy * 0.5, depth + 1, count)
            + awesome_quad_arclen7(q1, accuracy * 0.5, depth + 1, count)
    }
}

fn awesome_quad_arclen24(q: QuadBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    let pm = (q.p0 + q.p2) / 2.0;
    let d1 = q.p1 - pm;
    let d = q.p2 - q.p0;
    let dhypot2 = d.hypot2();
    let x = 2.0 * d.dot(d1) / dhypot2;
    let y = 2.0 * d.cross(d1) / dhypot2;
    let lc = (q.p2 - q.p0).hypot();
    let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
    // This isn't quite right near (1.1, 0)
    let est_err = 1.0 * (lp - lc) * (0.5 * x * x + 0.1 * y * y).powf(16.0).tanh();
    if est_err < accuracy || depth == 16 {
        *count += 1;
        gauss_arclen_24(q)
    } else {
        let (q0, q1) = q.subdivide();
        awesome_quad_arclen24(q0, accuracy * 0.5, depth + 1, count)
            + awesome_quad_arclen24(q1, accuracy * 0.5, depth + 1, count)
    }
}

// Based on http://www.malczak.linuxpl.com/blog/quadratic-bezier-curve-length/
fn quad_arclen_analytical(q: QuadBez) -> f64 {
    let d2 = q.p0 - 2.0 * q.p1 + q.p2;
    let a = d2.hypot2();
    let d1 = q.p1 - q.p0;
    let c = d1.hypot2();
    if a < 5e-4 * c {
        return gauss_arclen_3(q);
    }
    let b = 2.0 * d2.dot(d1);

    let sabc = (a + b + c).sqrt();
    let a2 = a.powf(-0.5);
    let a32 = a2.powi(3);
    let c2 = 2.0 * c.sqrt();
    let ba = b * a2;

    sabc
        + 0.25 * (a2 * a2 * b * (2.0 * sabc-c2)
            + a32 * (4.0 * c * a - b * b) * (((2.0 * a + b) * a2 + 2.0 * sabc) / (ba + c2)).ln()
        )
}

/// Calculate the L0 metric from "Adaptive subdivision and the length and
/// energy of Bézier curves" by Jens Gravesen.
fn calc_l0(q: QuadBez) -> f64 {
    let lc = (q.p2 - q.p0).hypot();
    let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
    (2.0 / 3.0) * lc + (1.0 / 3.0) * lp
}

fn with_subdiv(q: QuadBez, f: &Fn(QuadBez) -> f64, depth: usize) -> f64 {
    if depth == 0 {
        f(q)
    } else {
        let (q0, q1) = q.subdivide();
        with_subdiv(q0, f, depth - 1) + with_subdiv(q1, f, depth - 1)
    }
}

/// Generate map data suitable for plotting in Gnuplot.
fn main() {
    let mut n_subdiv = 0;
    let mut func: &Fn(QuadBez) -> f64 = &gauss_arclen_3;
    for arg in env::args().skip(1) {
        if arg == "gauss3" {
            func = &gauss_arclen_3;
        } else if arg == "gauss5" {
            func = &gauss_arclen_5;
        } else if arg == "gauss7" {
            func = &gauss_arclen_7;
        } else if arg == "gauss24" {
            func = &gauss_arclen_24;
        } else if arg == "l0" {
            func = &calc_l0;
        } else if let Ok(n) = arg.parse() {
            n_subdiv = n;
        } else {
            println!("usage: arclen_accuracy [func] n_subdiv");
            std::process::exit(1);
        }
    }
    let n = 400;
    let accuracy = 1e-6;
    for i in 0..=n {
        let x = 0.1 * (i as f64) * (n as f64).recip();
        for j in 0..=n {
            let y = 0.1 * (j as f64) * (n as f64).recip();
            let q = QuadBez::new((-1.0, 0.0), (x, y), (1.0, 0.0));
            let mut count = 0;
            let accurate_arclen = awesome_quad_arclen7(q, 1e-15, 0, &mut count);
            //count = 0;
            let start_time = Instant::now();
            //let est = awesome_quad_arclen7(q, accuracy, 0, &mut count);
            let est = quad_arclen_analytical(q);
            let elapsed = start_time.elapsed();
            //let est = gravesen_rec(&q, calc_l0(q), accuracy, 0, &mut count);
            //let est = with_subdiv(q, func, n_subdiv);
            let error = est - accurate_arclen;
            println!("{} {} {}", x, y, (error.abs() + 1e-18).log10());
            //println!("{} {} {}", x, y, count);
            //println!("{} {} {}", x, y, elapsed.subsec_nanos());
            //let accurate_arclen = with_subdiv(q, &gauss_arclen_5, 8);
            //let error = est - accurate_arclen;
            /*
            let lc = (q.p2 - q.p0).hypot();
            let lp = (q.p1 - q.p0).hypot() + (q.p2 - q.p1).hypot();
            let est_err = 1.0 * (lp - lc) * (0.5 * x * x + 0.1 * y * y).powf(16.0).tanh();
            println!("{} {} {}", x, y, (est_err/error.abs() + 1e-15).log10());
            */
        }
        println!("");
    }
}
