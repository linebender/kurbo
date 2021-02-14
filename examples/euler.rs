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

use kurbo::{Affine, CubicBez, EulerSeg, Point, Shape};

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

// errors for 1e-9 threshold
// n == 1: 1.01
// n == 2: 3.96
// n == 3: 8.23
// n == 4: 14.2

// integ_euler_8
// est = 0.112: 1e-9
// 0.62: 1e-6

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
    <svg height="800" width="1000">
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
            println!(
                r##"<path d="{}" fill="none" stroke="#000" />"##,
                path.to_svg()
            );
        }
    }
    println!(
        r#"    </svg>
    </body>
</html>"#
    );
}

fn main() {
    cubic_err_scatter()
}
