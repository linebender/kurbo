use kurbo::{CubicBez, EulerSeg, ParamCurve, ParamCurveArea, ParamCurveDeriv, common::GAUSS_LEGENDRE_COEFFS_32, Point};
use rand::{thread_rng, Rng};

fn calc_euler_area(e: &EulerSeg) -> f64 {
    let d = e.deriv();
    GAUSS_LEGENDRE_COEFFS_32.iter().map(|(wi, xi)| {
        let t = 0.5 + 0.5 * xi;
        (0.5 * wi) * e.eval(t).y * d.eval(t).x
    }).sum::<f64>()
}

fn calc_mx<T: ParamCurve + ParamCurveDeriv>(e: &T) -> f64 {
    let d = e.deriv();
    GAUSS_LEGENDRE_COEFFS_32.iter().map(|(wi, xi)| {
        let t = 0.5 + 0.5 * xi;
        let p = e.eval(t);
        (0.5 * wi) * (p.x - 0.5) * p.y * d.eval(t).x
    }).sum::<f64>()
}

fn rand_shallow_cubic() -> CubicBez {
    let mut rng = thread_rng();
    let p1 = Point::new(rng.gen_range(0.233..0.433), rng.gen_range(-0.1..0.1));
    let p2 = Point::new(rng.gen_range(0.567..0.767), rng.gen_range(-0.1..0.1));
    CubicBez::new(Point::ZERO, p1, p2, Point::new(1., 0.))
}

fn main() {
    //let c=  CubicBez::new((0., 0.), (0.3, 0.1), (0.7, 0.1), (1., 0.));
    for _ in 0..100_000 {
        let c = rand_shallow_cubic();
        let e = EulerSeg::from_cubic(c);
        let err  = e.cubic_euler_err(c, 10);
        let area_err = calc_euler_area(&e) + c.signed_area();
        let mx_err = calc_mx(&e) - calc_mx(&c);
        let est_err = (area_err / 0.82).hypot((mx_err * 5.0).abs());
        let crudest_err = (c.p1 - Point::new(1./3., 0.)).hypot2() + (c.p2 - Point::new(2./3., 0.)).hypot2();
        let est_err = est_err + 0.03 * crudest_err.powf(1.5);
        //let est_err = est_err + 0.6 * crudest_err.powf(2.0);
        if est_err < 0.001 {
            println!("{} {}", err, est_err);
        }
    }
}

