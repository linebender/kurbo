use kurbo::{
    common::{solve_cubic, solve_itp}, Affine, CubicBez, ParamCurve, ParamCurveArclen, ParamCurveNearest, Point,
    Shape, ParamCurveDeriv, PathSeg, Line, QuadBez, Vec2,
};
use rand::Rng;

fn rand_mono_cubic() -> CubicBez {
    let mut rnd = rand::thread_rng();
    let p0 = Point::new(0., 0.);
    let p1 = Point::new(rnd.gen_range(-1.0..1.), rnd.gen_range(0.0..1.));
    let p2 = Point::new(rnd.gen_range(-1.0..1.), rnd.gen_range(0.0..1.));
    let p3 = Point::new(0., 1.);
    CubicBez::new(p0, p1, p2, p3)
}

fn solve_t_for_y(c: CubicBez, y: f64) -> f64 {
    if y == c.p0.y {
        return 0.0;
    }
    if y == c.p3.y {
        return 1.0;
    }
    let c3 = c.p3.y - 3.0 * c.p2.y + 3.0 * c.p1.y - c.p0.y;
    let c2 = 3.0 * (c.p2.y - 2.0 * c.p1.y + c.p0.y);
    let c1 = 3.0 * (c.p1.y - c.p0.y);
    let c0 = c.p0.y - y;
    for t in solve_cubic(c0, c1, c2, c3) {
        if (0.0..=1.0).contains(&t) {
            return t;
        }
    }
    print_svg(c);
    println!("{:?}", c);
    println!("{} {} {} {}", c0, c1, c2, c3);
    panic!("no solution found, y = {}", y);
}

fn solve_x_for_y(c: CubicBez, y: f64) -> f64 {
    c.eval(solve_t_for_y(c, y)).x
}

fn max_x_dist(c0: CubicBez, c1: CubicBez, n: usize) -> f64 {
    let mut max = 0.0f64;
    for i in 1..n + 1 {
        let t = i as f64 / (n + 1) as f64;
        let p0 = c0.eval(t);
        let p0_c1_x = solve_x_for_y(c1, p0.y);
        max = max.max((p0.x - p0_c1_x).abs());
        let p1 = c1.eval(t);
        let p1_c0_x = solve_x_for_y(c0, p1.y);
        max = max.max((p1.x - p1_c0_x).abs());
    }
    max
}

fn print_svg(c: CubicBez) {
    let a = Affine::new([500., 0., 0., 500., 550., 10.]);
    let p = (a * c).to_path(1e-9).to_svg();
    println!("  <path d=\"{}\" fill=\"none\" stroke=\"#000\" />", p);
}

const ACCURACY: f64 = 1e-12;
fn half_hausdorff(c0: CubicBez, c1: CubicBez, n: usize) -> f64 {
    let mut max_dist_sq = 0.0f64;
    for i in 0..(n + 1) {
        let t = i as f64 / n as f64;
        let p = c0.eval(t);
        let dist_sq = c1.nearest(p, ACCURACY).distance_sq;
        max_dist_sq = max_dist_sq.max(dist_sq);
    }
    max_dist_sq.sqrt()
}

fn actual_hausdorff(c0: CubicBez, c1: CubicBez) -> f64 {
    const N: usize = 100;
    let d0 = half_hausdorff(c0, c1, N);
    let d1 = half_hausdorff(c1, c0, N);
    d0.max(d1)
}

fn run_one() -> f64 {
    let c0 = rand_mono_cubic();
    let c1 = rand_mono_cubic();
    let m3 = max_x_dist(c0, c1, 4);
    let m100 = max_x_dist(c0, c1, 100);
    let ratio = m3 / m100;
    if ratio < 0.25 {
        println!("c0 = {:?}, c1 = {:?}", c0, c1);
        print_svg(c0);
        print_svg(c1);
    }
    //println!("{}", ratio);
    ratio
}

fn frechet_est(c0: CubicBez, c1: CubicBez, n: usize) -> f64 {
    let arclen0 = c0.arclen(ACCURACY);
    let arclen1 = c1.arclen(ACCURACY);
    let mut max_dist_sq = 0.0f64;
    for i in 1..n {
        let t = i as f64 / n as f64;
        let t0 = c0.inv_arclen(arclen0 * t, ACCURACY);
        let t1 = c1.inv_arclen(arclen1 * t, ACCURACY);
        let dist_sq = c0.eval(t0).distance_squared(c1.eval(t1));
        max_dist_sq = max_dist_sq.max(dist_sq);
    }
    max_dist_sq.sqrt()
}

fn one_hausdorff() -> f64 {
    let c0 = rand_mono_cubic();
    let c1 = rand_mono_cubic();
    let fre = frechet_est(c0, c1, 100);
    let fre4 = frechet_est(c0, c1, 30);
    // 3: 0.16
    // 4: 0.34
    // 5: 0.48
    // 6: 0.56
    // 8: 0.604
    // 10: 0.63
    // 20: 0.74
    // 30: 0.83
    if fre4 < 0.85 * fre {
        print_svg(c0);
        print_svg(c1);
        let dist = actual_hausdorff(c0, c1);
        println!("dist = {}, fre = {}, fre4 = {}", dist, fre, fre4);
    }
    fre4 / fre
}

fn cbc_fun() {
    // Following is a difficult case for the cubic solver:
    //let (c0, c1, c2, c3) = (-0.9899157720006341, 1.5149655321812139, -0.5149655191281295, -0.00000001305308439114583);
    let (c0, c1, c2, c3) = (0., -1., 0.1, 1.);
    for t in solve_cubic(c0, c1, c2, c3) {
        let y = c0 + c1 * t + c2 * t.powi(2) + c3 * t.powi(3);
        println!("{} {}", t, y);
    }
    /*
       // Alternate solution for -d in cubic solver:
           let sqmq = (-q).sqrt();
           let th3 = (r / sqmq.powi(3)).acos() * (1. / 3.);
           result.push(2. * sqmq * th3.cos() - x0);
           result.push(2. * sqmq * (th3 + 2. * FRAC_PI_3).cos() - x0);
           result.push(2. * sqmq * (th3 + 4. * FRAC_PI_3).cos() - x0);
    */
}

fn single_itp(c0: CubicBez, c1: CubicBez) -> (f64, f64) {
    // Note: only works with one sign
    let f = |y| solve_x_for_y(c1, y) - solve_x_for_y(c0, y);
    let ya = f(0.0);
    let yb = f(1.0);
    let y = solve_itp(f, 0.0, 1.0, 1e-6, 1, 0.2, ya, yb);
    (solve_t_for_y(c0, y), solve_t_for_y(c1, y))
}

fn cubic_newton(c0: CubicBez, c1: CubicBez, init_t0: f64, init_t1: f64) -> (f64, f64) {
    // Note: this isn't really newton, we need to find the actual intersection point.
    // Will probably converge slowly when crossing tangents are close.
    const TOLERANCE: f64 = 1e-12;
    let d0 = c0.deriv();
    let d1 = c1.deriv();
    let mut t0 = init_t0;
    let mut t1 = init_t1;
    for _ in 0..10 {
        let p0 = c0.eval(t0);
        let p1 = c1.eval(t1);
        let d = p1 - p0;
        if d.hypot2() < TOLERANCE.powi(2) {
            break;
        }
        let pd0 = d0.eval(t0).to_vec2();
        let pd1 = d1.eval(t1).to_vec2();
        t0 += d.dot(pd0) / pd0.hypot2();
        t1 -= d.dot(pd1) / pd1.hypot2();
    }
    (t0, t1)
}

// c0 + c1 * x + c2 * x^2 is approx equal to cubic bez
fn est_parabola(c: CubicBez) -> (f64, f64, f64) {
    let c = c.subsegment(0.0..0.3);
    let seg = PathSeg::Cubic(c);
    let close_seg = PathSeg::Line(Line::new(c.p3, c.p0));
    let area = seg.area() + close_seg.area();
    let dx = c.p3.x - c.p0.x;
    // Note: this solution gives 0 error at endpoints. Arguably
    // a better solution would be based on mean / moments.
    let c2 = 6. * area / dx.powi(3);
    let c1 = (c.p3.y - c.p0.y - (c.p3.x.powi(2) - c.p0.x.powi(2)) * c2) / dx;
    let c0 = c.p0.y - c1 * c.p0.x - c2 * c.p0.x.powi(2);
    println!("{} {} {}", c0, c1, c2);
    let a = Affine::new([500., 0., 0., 500., 550., 10.]);
    print_svg(c);
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        let x = c.p0.lerp(c.p3, t).x;
        let y = c0 + c1 * x + c2 * x * x;
        let p = a * Point::new(x, y);
        println!("  <circle cx=\"{}\" cy=\"{}\" r=\"3\" />", p.x, p.y);
    }
    // Hybrid bezier concept from North Masters thesis
    let q0 = QuadBez::new(c.p0, c.p0.lerp(c.p1, 1.5) ,c.p3);
    let q1 = QuadBez::new(c.p0, c.p3.lerp(c.p2, 1.5) ,c.p3);
    print_svg(q0.raise());
    print_svg(q1.raise());
    let mut dmin = 0.0f64;
    let mut dmax = 0.0f64;
    for q in [&q0, &q1] {
        // Solve tangency with estimated parabola
        // Maybe this should be a separate function?
        let params = quad_parameters(*q);
        let dparams = (params.1, 2. * params.2);
        // d.qx/dt * dpara.y/dx - dq.y/dt = 0
        // para.y = c0 + c1 * x + c2 * x^2
        // dpara.y/dx = c1 + 2 * c2 * x = d0 + d1 * x
        let d0 = c1;
        let d1 = 2. * c2;
        let dydt0 = d0 + d1 * params.0.x;
        let dydt1 = d1 * params.1.x;
        let dydt2 = d1 * params.2.x;
        let f0 = dparams.0.x * dydt0 - dparams.0.y;
        let f1 = dparams.0.x * dydt1 + dparams.1.x * dydt0 - dparams.1.y;
        let f2 = dparams.0.x * dydt2 + dparams.1.x * dydt1;
        let f3 = dparams.1.x * dydt2;
        for t in solve_cubic(f0, f1, f2, f3) {
            if (0.0..=1.0).contains(&t) {
                let p = q.eval(t);
                let y = p.y - (c0 + c1 * p.x + c2 * p.x.powi(2));
                dmin = dmin.min(y);
                dmax = dmax.max(y);
            }
            println!("t = {}, pt = {:?}", t, q.eval(t));
        }
    }
    println!("dmin = {}, dmax = {}", dmin, dmax);
    // TODO: either want to return dmin/dmax or have separate function
    (c0, c1, c2)
}

/// Get the parameters such that the curve can be represented by the following formula:
///     B(t) = c0 + c1 * t + c2 * t^2
pub fn quad_parameters(q: QuadBez) -> (Vec2, Vec2, Vec2) {
    let c0 = q.p0.to_vec2();
    let c1 = (q.p1 - q.p0) * 2.0;
    let c2 = c0 - q.p1.to_vec2() * 2.0 + q.p2.to_vec2();
    (c0, c1, c2)
}

fn main() {
    /* */
    let c0 = CubicBez::new((0., 0.), (0.4, 0.2), (0.2, 0.8), (1., 1.));
    let c1 = CubicBez::new((1., 0.), (0.7, 0.2), (0.8, 0.7), (0., 1.));
    est_parabola(c0);
    let start = std::time::Instant::now();
    for _ in 0..1_000_000 {
        let _ = single_itp(c1, c0);
    }
    println!("{:?}", single_itp(c1, c0));
    println!("elapsed: {:?}", start.elapsed());
    /*
    let start = std::time::Instant::now();
    let mut s = 0.0;
    for i in 0..1_000_000 {
        let (t0, t1) = cubic_newton(c0, c1, 0.5, 0.5 + i as f64 * 1e-9);
        s += t0 + t1;
    }
    println!("elapsed: {:?}, s = {}", start.elapsed(), s);
    println!("{:?}", single_itp(c1, c0));
    */
}
