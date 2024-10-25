use std::cmp::Ordering;

use arrayvec::ArrayVec;
use kurbo::{
    common::{solve_cubic, solve_itp},
    Affine, CubicBez, Line, ParamCurve, ParamCurveArclen, ParamCurveDeriv, ParamCurveNearest,
    PathSeg, Point, QuadBez, Shape, Vec2,
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

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum QuadraticSigns {
    /// Signs are [-sign(b), sign(b)]
    Linear,
    /// Polynomial is of sign sign(c) everywhere (may be 0)
    Degenerate,
    /// No real roots, sign sign(a) everywhere
    QuadraticNone,
    /// One double root, sign [sign(a), sign(a)]
    QuadraticDouble,
    /// Two single roots, Sign [sign(a), -sign(a), sign(a)]
    QuadraticSingles,
}

/// Solve quadratic equation and characterize signs.
///
/// Third return value is coefficient that controls sign as x -> inf.
fn solve_quadratic_signs(c0: f64, c1: f64, c2: f64) -> (ArrayVec<f64, 2>, QuadraticSigns, f64) {
    let mut result = ArrayVec::new();
    let sc0 = c0 * c2.recip();
    let sc1 = c1 * c2.recip();
    if !sc0.is_finite() || !sc1.is_finite() {
        // c2 is zero or very small, treat as linear eqn
        let root = -c0 / c1;
        if root.is_finite() {
            result.push(root);
            return (result, QuadraticSigns::Linear, c1);
        } else if c0 == 0.0 && c1 == 0.0 {
            // Degenerate case
            result.push(0.0);
        }
        return (result, QuadraticSigns::Degenerate, c0);
    }
    let arg = sc1 * sc1 - 4.0 * sc0;
    let root1 = if !arg.is_finite() {
        // Likely, calculation of sc1 * sc1 overflowed. Find one root
        // using sc1 x + x² = 0, other root as sc0 / root1.
        -sc1
    } else {
        if arg < 0.0 {
            return (result, QuadraticSigns::QuadraticNone, c2);
        } else if arg == 0.0 {
            result.push(sc1);
            return (result, QuadraticSigns::QuadraticDouble, c2);
        }
        // See https://math.stackexchange.com/questions/866331
        -0.5 * (sc1 + arg.sqrt().copysign(sc1))
    };
    // Note: in this version we assume the result will be valid, as root1
    // is chosen to be the root with larger magnitude.
    let root2 = sc0 / root1;
    // Sort just to be friendly and make results deterministic.
    if root2 > root1 {
        result.push(root1);
        result.push(root2);
    } else {
        result.push(root2);
        result.push(root1);
    }
    (result, QuadraticSigns::QuadraticSingles, c2)
}

/// Represents an interval so that for y0 < y < y1, f(y) has sign x
struct SignInterval {
    y0: f64,
    y1: f64,
    x: f64,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Ternary {
    Left,
    In,
    Right,
}

impl QuadraticSigns {
    fn to_intervals(self, y0: f64, y1: f64, x: f64, roots: &ArrayVec<f64, 2>) -> Vec<SignInterval> {
        // This is an annoying case analysis, but not clear the clever approach would be better.
        match self {
            QuadraticSigns::Degenerate
            | QuadraticSigns::QuadraticNone
            | QuadraticSigns::QuadraticDouble => vec![SignInterval { y0, y1, x }],
            QuadraticSigns::Linear => {
                let root = roots[0];
                if root <= y0 {
                    vec![SignInterval { y0, y1, x }]
                } else if root < y1 {
                    vec![
                        SignInterval {
                            y0,
                            y1: root,
                            x: -x,
                        },
                        SignInterval { y0: root, y1, x },
                    ]
                } else {
                    vec![SignInterval { y0, y1, x: -x }]
                }
            }
            QuadraticSigns::QuadraticSingles => {
                let root0 = roots[0];
                let root1 = roots[1];
                if root1 <= y0 || root0 >= y1 {
                    vec![SignInterval { y0, y1, x }]
                } else if root0 <= y0 && root1 >= y1 {
                    vec![SignInterval { y0, y1, x: -x }]
                } else if root0 > y0 && root1 < y1 {
                    vec![
                        SignInterval { y0, y1: root0, x },
                        SignInterval {
                            y0: root0,
                            y1: root1,
                            x: -x,
                        },
                        SignInterval { y0: root1, y1, x },
                    ]
                } else if root0 > y0 {
                    vec![
                        SignInterval { y0, y1: root0, x },
                        SignInterval {
                            y0: root0,
                            y1,
                            x: -x,
                        },
                    ]
                } else {
                    // root1 < y1
                    vec![
                        SignInterval {
                            y0,
                            y1: root1,
                            x: -x,
                        },
                        SignInterval { y0: root1, y1, x },
                    ]
                }
            }
        }
    }
}

impl Ternary {
    fn from_signs(a: f64, b: f64) -> Self {
        if a < 0.0 && b < 0.0 {
            Ternary::Left
        } else if a > 0.0 && b > 0.0 {
            Ternary::Right
        } else {
            Ternary::In
        }
    }
}

fn merge_intervals(a: &[SignInterval], b: &[SignInterval]) -> Vec<(f64, f64, Ternary)> {
    let mut result = vec![];
    let mut y = a[0].y0;
    let mut i = 0;
    let mut j = 0;
    while i < a.len() {
        let t = Ternary::from_signs(a[i].x, b[j].x);
        let ya = a[i].y1;
        let yb = b[j].y1;
        let y1 = ya.min(yb);
        if ya == y1 {
            i += 1;
        }
        if yb == y1 {
            j += 1;
        }
        result.push((y, y1, t));
        y = y1;
    }
    result
}

#[derive(Clone, Copy, Debug)]
struct EstParab {
    c0: f64,
    c1: f64,
    c2: f64,
    dmin: f64,
    dmax: f64,
}

impl EstParab {
    // c0 + c1 * x + c2 * x^2 is approx equal to cubic bez
    fn from_cubic(c: CubicBez) -> Self {
        //let c = c.subsegment(0.0..0.3);
        let seg = PathSeg::Cubic(c);
        let close_seg = PathSeg::Line(Line::new(c.p3, c.p0));
        let area = seg.area() + close_seg.area();
        let dy = c.p3.y - c.p0.y;
        // Note: this solution gives 0 error at endpoints. Arguably
        // a better solution would be based on mean / moments.
        let c2 = -6. * area / dy.powi(3);
        let c1 = (c.p3.x - c.p0.x - (c.p3.y.powi(2) - c.p0.y.powi(2)) * c2) / dy;
        let c0 = c.p0.x - c1 * c.p0.y - c2 * c.p0.y.powi(2);
        /*
        println!("{} {} {}", c0, c1, c2);
        let a = Affine::new([500., 0., 0., 500., 550., 10.]);
        print_svg(c);
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let y = c.p0.lerp(c.p3, t).y;
            let x = c0 + c1 * y + c2 * y * y;
            let p = a * Point::new(x, y);
            println!("  <circle cx=\"{}\" cy=\"{}\" r=\"3\" />", p.x, p.y);
        }
        */
        // Hybrid bezier concept from North Masters thesis
        let q0 = QuadBez::new(c.p0, c.p0.lerp(c.p1, 1.5), c.p3);
        let q1 = QuadBez::new(c.p0, c.p3.lerp(c.p2, 1.5), c.p3);
        //print_svg(q0.raise());
        //print_svg(q1.raise());
        let mut dmin = 0.0f64;
        let mut dmax = 0.0f64;
        for q in [&q0, &q1] {
            // Solve tangency with estimated parabola
            // Maybe this should be a separate function?
            let params = quad_parameters(*q);
            let dparams = (params.1, 2. * params.2);
            // d.qy/dt * dpara.x/dy - dq.x/dt = 0
            // para.x = c0 + c1 * x + c2 * x^2
            // dpara.x/dy = c1 + 2 * c2 * x = d0 + d1 * x
            let d0 = c1;
            let d1 = 2. * c2;
            let dxdt0 = d0 + d1 * params.0.y;
            let dxdt1 = d1 * params.1.y;
            let dxdt2 = d1 * params.2.y;
            let f0 = dparams.0.y * dxdt0 - dparams.0.x;
            let f1 = dparams.0.y * dxdt1 + dparams.1.y * dxdt0 - dparams.1.x;
            let f2 = dparams.0.y * dxdt2 + dparams.1.y * dxdt1;
            let f3 = dparams.1.y * dxdt2;
            for t in solve_cubic(f0, f1, f2, f3) {
                if (0.0..=1.0).contains(&t) {
                    let p = q.eval(t);
                    let x = p.x - (c0 + c1 * p.y + c2 * p.y.powi(2));
                    dmin = dmin.min(x);
                    dmax = dmax.max(x);
                }
                //println!("t = {}, pt = {:?}", t, q.eval(t));
            }
        }
        //println!("dmin = {}, dmax = {}", dmin, dmax);
        EstParab {
            c0,
            c1,
            c2,
            dmin,
            dmax,
        }
    }

    fn eval(&self, y: f64) -> f64 {
        self.c0 + self.c1 * y + self.c2 * y * y
    }

    // Classify regions where x + dmax < 0, x + dmin > 0, or in between
    fn intervals(&self, y0: f64, y1: f64) -> Vec<(f64, f64, Ternary)> {
        let (roots0, signs0, x0) = solve_quadratic_signs(self.c0 + self.dmin, self.c1, self.c2);
        let (roots1, signs1, x1) = solve_quadratic_signs(self.c0 + self.dmax, self.c1, self.c2);
        let iv0 = signs0.to_intervals(y0, y1, x0, &roots0);
        let iv1 = signs1.to_intervals(y0, y1, x1, &roots1);
        merge_intervals(&iv0, &iv1)
    }

    /// Determine minimum and maximum possible values over the range.
    ///
    /// This computes just the polynomial; dmin and dmax can easily be added after.
    fn min_max(&self, y0: f64, y1: f64) -> (f64, f64) {
        let x0 = self.eval(y0);
        let x1 = self.eval(y1);
        let mut xmin = x0.min(x1);
        let mut xmax = x0.max(x1);
        let vertex = -0.5 * self.c1 / self.c2;
        // Note: may not be finite!
        if vertex > y0 && vertex < y1 {
            let x = self.eval(vertex);
            xmin = xmin.min(x);
            xmax = xmax.max(x);
        }
        (xmin, xmax)
    }
}

impl std::ops::Sub for EstParab {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        EstParab {
            c0: self.c0 - other.c0,
            c1: self.c1 - other.c1,
            c2: self.c2 - other.c2,
            dmin: self.dmin - other.dmax,
            dmax: self.dmax - other.dmin,
        }
    }
}

/// Get the parameters such that the curve can be represented by the following formula:
///     B(t) = c0 + c1 * t + c2 * t^2
pub fn quad_parameters(q: QuadBez) -> (Vec2, Vec2, Vec2) {
    let c0 = q.p0.to_vec2();
    let c1 = (q.p1 - q.p0) * 2.0;
    let c2 = c0 - q.p1.to_vec2() * 2.0 + q.p2.to_vec2();
    (c0, c1, c2)
}

fn intersect_cubics_rec(orig_c0: CubicBez, orig_c1: CubicBez, y0: f64, y1: f64) {
    println!("running recursion {}..{}", y0, y1);
    let c0 = orig_c0.subsegment(solve_t_for_y(orig_c0, y0)..solve_t_for_y(orig_c0, y1));
    let c1 = orig_c1.subsegment(solve_t_for_y(orig_c1, y0)..solve_t_for_y(orig_c1, y1));
    let ep0 = EstParab::from_cubic(c0);
    let ep1 = EstParab::from_cubic(c1);
    //println!("ep0 = {:?}", ep0);
    //println!("ep0 = {:?}", ep1);
    let dep = ep1 - ep0;
    //println!("ep1 - ep0 = {:?}", dep);
    let ivs = dep.intervals(y0, y1);
    for (new_y0, new_y1, t) in ivs {
        if t == Ternary::In {
            println!("delta y = {}", new_y1 - new_y0);
            let mid = 0.5 * (new_y0 + new_y1);
            if new_y1 - new_y0 > 1e-6 {
                if new_y1 - new_y0 < 0.5 * (y1 - y0) {
                    intersect_cubics_rec(orig_c0, orig_c1, new_y0, new_y1);
                } else {
                    intersect_cubics_rec(orig_c0, orig_c1, new_y0, mid);
                    intersect_cubics_rec(orig_c0, orig_c1, mid, new_y1);
                }
            } else {
                println!("intersection found at y = {}", mid);
            }
        }
    }
}

fn intersect_cubics(c0: CubicBez, c1: CubicBez) {
    intersect_cubics_rec(c0, c1, c0.p0.y, c0.p3.y);
}

fn main() {
    /* */
    let c0 = CubicBez::new((0., 0.), (1.4, 0.2), (1.2, 0.8), (0., 1.));
    let c1 = CubicBez::new((1., 0.), (0.7, 0.2), (0.8, 0.7), (0., 1.));
    print_svg(c0);
    print_svg(c1);
    intersect_cubics(c0, c1);
    /*
    let start = std::time::Instant::now();
    for _ in 0..1_000_000 {
        let _ = single_itp(c1, c0);
    }
    println!("{:?}", single_itp(c1, c0));
    println!("elapsed: {:?}", start.elapsed());
    */
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
