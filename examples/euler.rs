// Copyright 2021 The kurbo Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! A few test programs and experiments to exercise Euler segments.

use kurbo::{Affine, BezPath, CubicBez, EulerParams, EulerSeg, ParamCurve, Point, Shape, Vec2};

#[allow(unused)]
fn check_euler_int_accuracy() {
    for _ in 0..1000000 {
        let k0 = 8.0 * rand::random::<f64>();
        let k1 = 8.0 * rand::random::<f64>();
        let accurate = kurbo::integ_euler_12n(k0, k1, 8);
        let est = kurbo::integ_euler_12n(k0, k1, 4);
        let err = (est.0 - accurate.0).abs().max(est.1 - accurate.1).abs();
        let est_err_raw = 0.2 * k0 * k0 + k1.abs();
        if est_err_raw < 14.2 && err > 1e-9 {
            println!("est_err_raw = {:?}, err = {:e}", est_err_raw, err);
        }
    }
}

fn rand_cubic() -> CubicBez {
    let p0 = Point::new(0., 0.);
    let p1 = Point::new(
        rand::random::<f64>() * 0.5,
        rand::random::<f64>() * 2.0 - 1.0,
    );
    let p2 = Point::new(
        rand::random::<f64>() * 0.5 + 0.5,
        rand::random::<f64>() * 2.0 - 1.0,
    );
    let p3 = Point::new(1., 0.);
    CubicBez::new(p0, p1, p2, p3)
}

fn cubic_err_scatter() {
    println!(
        r##"<!DOCTYPE html>
<html>
    <body>
    <svg width="1000" height="800"
    <line x1="0" y1="0" x2="800" y2="800" stroke="blue" />"##
    );
    for _ in 0..100000 {
        let c = rand_cubic();
        let es = EulerSeg::from_cubic(c);
        let err = es.cubic_euler_err(c, 20);
        //println!("{:?} {}", c, es.cubic_euler_err(c));
        const ERR_SCALE: f64 = 100.0;
        let x = 950. * (err * ERR_SCALE).min(1.0);
        let est_err = es.cubic_euler_err(c, 4);
        let y = 750. * (est_err * ERR_SCALE).min(1.0);
        if y < 750. {
            let a = Affine::new([20., 0., 0., 20., x, y]);
            let path = (a * c).into_path(1e-6);
            println!("<path d='{}' fill='none' stroke='#000' />", path.to_svg());
        }
    }
    println!(
        r#"    </svg>
    </body>
</html>"#
    );
}

fn fit_cubic_plot() {
    const N: usize = 513;
    println!("P3");
    println!("{} {}", N, N);
    println!("255");
    const KMAX: f64 = 0.1;
    for y in 0..N {
        let k1 = KMAX * ((y as f64) * (2.0 / ((N - 1) as f64)) - 1.0);
        let k1 = 4. * k1;
        for x in 0..N {
            let k0 = KMAX * ((x as f64) * (2.0 / ((N - 1) as f64)) - 1.0);
            let params = EulerParams::from_k0_k1(k0, k1);
            let es = EulerSeg::from_params(Point::new(0., 0.), Point::new(1., 0.), params);
            let th0 = -params.th(0.0);
            let th1 = params.th(1.0);
            let v0 = Vec2::from_angle(th0);
            let p0 = Point::new(0., 0.);
            let p1 = p0 + 2. / 3. / (1. + v0.x) * v0;
            let p3 = Point::new(1., 0.);
            let v1 = Vec2::from_angle(-th1);
            let p2 = p3 - 2. / 3. / (1. + v1.x) * v1;
            let c = CubicBez::new(p0, p1, p2, p3);
            let err = es.cubic_euler_err(c, 10);
            let mut est = 0.0;
            est += 1.5e-5 * k0.powi(5).abs();
            est += 3e-6 * k1.powi(3).abs();
            est += 6e-4 * (k0 * k0 * k1).abs();
            est += 10e-5 * (k0 * k1 * k1).abs();
            //est += 5e-3 * (k0 * k0 * k1 * k1).abs();
            let err = err.powf(0.33);
            let est = est.powf(0.33);
            let escale = 1.5e4;
            let r = escale * err;
            let g = escale * est;
            let b = g;
            println!(
                "{} {} {}",
                (r.round() as u32).min(255),
                (g.round() as u32).min(255),
                (b.round() as u32).min(255)
            );
            //println!("{} {}: {} {}", k0, k1, err, est);
        }
    }
}

fn arc_toy() {
    let k0 = 0.2;
    let k1 = 0.0;
    let params = EulerParams::from_k0_k1(k0, k1);
    let es = EulerSeg::from_params(Point::new(0., 0.), Point::new(1., 0.), params);
    let th0 = -params.th(0.0);
    let th1 = params.th(1.0);
    let v0 = Vec2::from_angle(th0);
    let p0 = Point::new(0., 0.);
    let p1 = p0 + 2. / 3. / (1. + v0.x) * v0;
    let p3 = Point::new(1., 0.);
    let v1 = Vec2::from_angle(-th1);
    let p2 = p3 - 2. / 3. / (1. + v1.x) * v1;
    let c = CubicBez::new(p0, p1, p2, p3);
    println!("{:?}", c);
    let err = es.cubic_euler_err(c, 10);
    println!("err = {:e}", err);
}

fn print_svg_path(bp: &BezPath, color: &str) {
    println!(
        r"    <path d='{}' stroke='{}' fill='none' />",
        bp.to_svg(),
        color
    );
}

fn print_euler_seg(es: &EulerSeg, color: &str, accuracy: f64) {
    let mut bp = BezPath::new();
    bp.move_to(es.start());
    bp.extend(es.to_cubics(accuracy));
    print_svg_path(&bp, color);
}

// Note: this is the code that procedurally draws the figures in the
// "Cleaner parallel curves with Euler spirals" blog post.

fn parallel_1() {
    println!(
        "<svg width='450' height='250' viewBox='0 0 450 250' xmlns='http://www.w3.org/2000/svg'>"
    );
    let es = EulerSeg::new(Point::new(25., 140.), Point::new(425., 140.), 0.7, 1.1);
    print_euler_seg(&es, "black", 0.1);

    let offset = -150.0;
    let es_par = es.parallel_approx(offset, 1.);
    print_euler_seg(&es_par, "#c00", 0.1);

    let mut bp = BezPath::new();
    for seg in es.parallel_curve(offset, 1e-1) {
        if bp.is_empty() {
            bp.move_to(seg.start());
        }
        bp.extend(seg.to_cubics(0.1));
    }
    print_svg_path(&bp, "#00c");

    println!("</svg>");
}

fn parallel_2() {
    println!(
        "<svg width='450' height='400' viewBox='0 0 450 400' xmlns='http://www.w3.org/2000/svg'>"
    );
    let es = EulerSeg::from_params(
        Point::new(180., 270.),
        Point::new(330., 70.),
        EulerParams::from_k0_k1(0.0, 40.0),
    );
    print_euler_seg(&es, "black", 0.1);

    let offset = -60.0;

    let mut bp = BezPath::new();
    for seg in es.parallel_curve(offset, 1e-1) {
        if bp.is_empty() {
            bp.move_to(seg.start());
        }
        bp.extend(seg.to_cubics(0.1));
    }
    print_svg_path(&bp, "#00c");

    println!("</svg>");
}

const COLORS: &[&str] = &["#00f", "#aaf"];

fn parallel_multi_1() {
    println!(
        "<svg width='620' height='620' viewBox='0 0 620 620' xmlns='http://www.w3.org/2000/svg'>"
    );
    let es = EulerSeg::new(Point::new(200., 515.), Point::new(600., 515.), 0.7, 1.1);
    //let es = EulerSeg::from_params(Point::new(300., 400.), Point::new(700., 400.), EulerParams::from_k0_k1(5.0, 10.0));
    print_euler_seg(&es, "black", 0.1);

    for i in 1..25 {
        let offset = -25.0 * i as f64;

        for (j, seg) in es.parallel_curve(offset, 1e-1).enumerate() {
            print_euler_seg(&seg, COLORS[j % COLORS.len()], 0.1);
        }
    }

    println!("</svg>");
}

fn parallel_multi_2() {
    println!(
        "<svg width='900' height='770' viewBox='0 0 900 770' xmlns='http://www.w3.org/2000/svg'>"
    );
    let es = EulerSeg::from_params(
        Point::new(10., 370.),
        Point::new(410., 370.),
        EulerParams::from_k0_k1(5.0, 10.0),
    );
    let accuracy = 1e-5;
    print_euler_seg(&es, "black", accuracy);

    for i in 1..25 {
        let offset = -25.0 * i as f64;

        for (j, seg) in es.parallel_curve(offset, accuracy).enumerate() {
            print_euler_seg(&seg, COLORS[j % COLORS.len()], accuracy);
        }
    }

    println!("</svg>");
}

fn main() {
    if let Some(cmd) = std::env::args().nth(1) {
        match cmd.as_str() {
            "cubic_err_scatter" => cubic_err_scatter(),
            "fit_cubic_plot" => fit_cubic_plot(),
            "arc_toy" => arc_toy(),
            "parallel_1" => parallel_1(),
            "parallel_2" => parallel_2(),
            "parallel_multi_1" => parallel_multi_1(),
            "parallel_multi_2" => parallel_multi_2(),
            _ => println!("unknown cmd"),
        }
    } else {
        println!("usage: euler <cmd>");
        println!("  cubic_err_scatter: scatter plot of cubic->ES error estimates");
        println!("  fit_cubic_plot: 2d image of ES->cubic error");
        println!("  arc_toy: something that prints single ES->cubic fit");
        println!("  parallel_[12]: experiment with parallel curves");
        println!("  parallel_multi_[12]: multiple offsets from one ES");
    }
}
