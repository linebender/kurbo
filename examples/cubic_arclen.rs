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

fn est_gauss5_error(c: CubicBez) -> f64 {
    let lc = (c.p3 - c.p0).hypot();
    let lp = (c.p1 - c.p0).hypot() + (c.p2 - c.p1).hypot() + (c.p3 - c.p2).hypot();

    let d2 = c.deriv().deriv();
    let d3 = d2.deriv();
    //lp - lc
    let lm = 0.5 * (lp + lc);
    7e-8 * (d3.eval(0.5).hypot() / lm + 5.0 * d2.eval(0.5).hypot() / lm).powi(5) * lp
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

fn randpt() -> Vec2 {
    Vec2::new(rand::random(), rand::random())
}

fn randbez() -> CubicBez {
    CubicBez::new(randpt(), randpt(), randpt(), randpt())
}

fn main() {
    let accuracy = 1e-6;
    for _ in 0..100_000 {
        let c = randbez();
        let t: f64 = rand::random();
        let c = c.subsegment(0.0 .. t);
        //let accurate_arclen = c.arclen(1e-12);
        let mut count = 0;
        let accurate_arclen = my_arclen(c, 1e-12, 0, &mut count);
        /*
        let est = gauss_arclen_5(c);
        let est_err = est_gauss5_error(c);
        println!("{} {}", est_err, err);
        */
        let mut count = 0;
        let est = my_arclen(c, accuracy, 0, &mut count);
        let err = (accurate_arclen - est).abs();
        println!("{} {}", err, count);
    }
}
