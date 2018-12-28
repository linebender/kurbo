//! Research testbed for arclengths of cubic Bézier segments.

use kurbo::{ParamCurve, ParamCurveArclen, ParamCurveDeriv, CubicBez, Vec2};

/// Calculate arclength using Gauss-Legendre quadrature using formula from Behdad
/// in https://github.com/Pomax/BezierInfo-2/issues/77
fn gauss_arclen_5(c: CubicBez) -> f64 {
    let v0 = (c.p1-c.p0).hypot() *0.15;
    let v1 = (-0.558983582205757*c.p0 + 0.325650248872424*c.p1
        + 0.208983582205757*c.p2 + 0.024349751127576*c.p3).hypot();
    let v2 = (c.p3-c.p0+c.p2-c.p1).hypot()*0.26666666666666666;
    let v3 = (-0.024349751127576*c.p0 - 0.208983582205757*c.p1
        - 0.325650248872424*c.p2 + 0.558983582205757*c.p3).hypot();
    let v4 = (c.p3-c.p2).hypot()*0.15;

    v0 + v1 + v2 + v3 + v4
}

fn gauss_arclen_n<C: ParamCurveDeriv>(c: C, coeffs: &[(f64, f64)]) -> f64 {
    let d = c.deriv();
    coeffs.iter().map(|(wi, xi)|
        wi * d.eval(0.5 * (xi + 1.0)).hypot()
    ).sum::<f64>() * 0.5
}

fn gauss_arclen_7<C: ParamCurveDeriv>(c: C) -> f64 {
    gauss_arclen_n(c, &[
        (0.4179591836734694,  0.0000000000000000),
        (0.3818300505051189,  0.4058451513773972),
        (0.3818300505051189,  -0.4058451513773972),
        (0.2797053914892766,  -0.7415311855993945),
        (0.2797053914892766,  0.7415311855993945),
        (0.1294849661688697,  -0.9491079123427585),
        (0.1294849661688697,  0.9491079123427585),
    ])
}

fn est_gauss5_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    let d2 = c.deriv().deriv();
    let d3 = d2.deriv();
    let lmi = 2.0 / (lp + lc);
    7e-8 * (d3.eval(0.5).hypot() * lmi + 5.0 * d2.eval(0.5).hypot() * lmi).powi(5) * lp
}

fn gauss_errnorm_n<C: ParamCurveDeriv>(c: C, coeffs: &[(f64, f64)]) -> f64
    where C::DerivResult: ParamCurveDeriv
{
    let d = c.deriv().deriv();
    coeffs.iter().map(|(wi, xi)|
        wi * d.eval(0.5 * (xi + 1.0)).hypot2()
    ).sum::<f64>()
}

// Squared L2 norm of the second derivative of the cubic.
fn cubic_errnorm(c: CubicBez) -> f64 {
    let d = c.deriv().deriv();
    let dd = d.end() - d.start();
    d.start().hypot2() + d.start().dot(dd) + dd.hypot2() * (1.0 / 3.0)
}

// These are from https://pomax.github.io/bezierinfo/legendre-gauss.html
const GAUSS_LEGENDRE_COEFFS_3: &[(f64, f64)] = &[
    (0.8888888888888888,  0.0000000000000000),
    (0.5555555555555556,  -0.7745966692414834),
    (0.5555555555555556,  0.7745966692414834),
];

const GAUSS_LEGENDRE_COEFFS_9: &[(f64, f64)] = &[
    (0.3302393550012598,  0.0000000000000000),
    (0.1806481606948574,  -0.8360311073266358),
    (0.1806481606948574,  0.8360311073266358),
    (0.0812743883615744,  -0.9681602395076261),
    (0.0812743883615744,  0.9681602395076261),
    (0.3123470770400029,  -0.3242534234038089),
    (0.3123470770400029,  0.3242534234038089),
    (0.2606106964029354,  -0.6133714327005904),
    (0.2606106964029354,  0.6133714327005904),
];

const GAUSS_LEGENDRE_COEFFS_11: &[(f64, f64)] = &[
    (0.2729250867779006,  0.0000000000000000),
    (0.2628045445102467,  -0.2695431559523450),
    (0.2628045445102467,  0.2695431559523450),
    (0.2331937645919905,  -0.5190961292068118),
    (0.2331937645919905,  0.5190961292068118),
    (0.1862902109277343,  -0.7301520055740494),
    (0.1862902109277343,  0.7301520055740494),
    (0.1255803694649046,  -0.8870625997680953),
    (0.1255803694649046,  0.8870625997680953),
    (0.0556685671161737,  -0.9782286581460570),
    (0.0556685671161737,  0.9782286581460570),
];

fn est_gauss7_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    8e-9 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(6) * lp
}

fn gauss_arclen_9<C: ParamCurveDeriv>(c: C) -> f64 {
    gauss_arclen_n(c, GAUSS_LEGENDRE_COEFFS_9)
}

fn gauss_arclen_11<C: ParamCurveDeriv>(c: C) -> f64 {
    gauss_arclen_n(c, GAUSS_LEGENDRE_COEFFS_11)
}

fn est_gauss9_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    1e-10 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(8) * lp
}

fn est_gauss11_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    1e-12 * (2.0 * cubic_errnorm(c) / lc.powi(2)).powi(11) * lp
}

fn my_arclen(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss5_error(c) < accuracy {
        *count += 1;
        gauss_arclen_5(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen(c1, accuracy * 0.5, depth + 1, count)
    }
}

fn my_arclen7(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss7_error(c) < accuracy {
        *count += 1;
        gauss_arclen_7(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen7(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen7(c1, accuracy * 0.5, depth + 1, count)
    }
}

// Should make this generic instead of copy+paste, but we need only one when we're done.
fn my_arclen9(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss9_error(c) < accuracy {
        *count += 1;
        gauss_arclen_9(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen9(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen9(c1, accuracy * 0.5, depth + 1, count)
    }
}

// This doesn't help; we can't really get a more accurate error bound, so all this
// does is overkill the accuracy.
fn my_arclen11(c: CubicBez, accuracy: f64, depth: usize, count: &mut usize) -> f64 {
    if depth == 16 || est_gauss9_error(c) < accuracy {
        *count += 1;
        gauss_arclen_11(c)
    } else {
        let (c0, c1) = c.subdivide();
        my_arclen11(c0, accuracy * 0.5, depth + 1, count)
            + my_arclen11(c1, accuracy * 0.5, depth + 1, count)
    }
}

fn randpt() -> Vec2 {
    Vec2::new(rand::random(), rand::random())
}

fn randbez() -> CubicBez {
    CubicBez::new(randpt(), randpt(), randpt(), randpt())
}

fn main() {
    let accuracy = 1e-6;
    for _ in 0..10_000 {
        let c = randbez();
        let t: f64 = rand::random();
        let c = c.subsegment(0.0 .. t);
        //let accurate_arclen = c.arclen(1e-12);
        let mut count = 0;
        let accurate_arclen = my_arclen9(c, 1e-15, 0, &mut count);

        /*
        let est = gauss_arclen_11(c);
        let est_err = est_gauss11_error(c);
        let err = (accurate_arclen - est).abs();
        println!("{} {}", est_err, err);
        */

        let mut count = 0;
        let est = my_arclen9(c, accuracy, 0, &mut count);
        let err = (accurate_arclen - est).abs();
        println!("{} {}", err, count);
    }
}
