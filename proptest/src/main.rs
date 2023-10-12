use std::{
    io::{stdout, Write},
    panic::catch_unwind,
};

use kurbo::{fit_to_bezpath, offset::CubicOffset, CubicBez, ParamCurveNearest, Point, QuadBez};
use rand::{thread_rng, Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

fn main() {
    let seed: u64 = thread_rng().gen();
    println!("Using seed {}", seed);
    let mut runner = Runner {
        rng: ChaCha8Rng::seed_from_u64(seed),
    };
    //let mut stdout = stdout().lock();
    for _ in 0..100_000_000 {
        let bez = QuadBez::new(runner.point(), runner.point(), runner.point());
        let test = runner.point();
        match catch_unwind(|| {
            let nearest_ = bez.nearest(test, 1e-3);
        }) {
            Ok(_) => (),
            Err(_) => {
                println!("error running offset test is quad_bezier={bez:?}, test point={test:?}")
            }
        }
    }
}

struct Runner<R> {
    rng: R,
}

impl<R: Rng> Runner<R> {
    fn point(&mut self) -> Point {
        Point::new(self.number(), self.number())
    }

    /// Get an f64
    fn number(&mut self) -> f64 {
        match self.rng.gen_range(0..30) {
            0u8 => 0.,
            1 => 1.,
            2 => f64::MAX,
            3 => f64::MIN,
            4 => f64::MIN_POSITIVE,
            5 => f64::MIN_POSITIVE * 0.1,
            6 => -f64::MIN_POSITIVE,
            7 => f64::INFINITY,
            8 => f64::NEG_INFINITY,
            9 => f64::NAN,
            10..=20 => self.rational(),
            21..=29 => {
                // truely random f64
                f64::from_bits(self.rng.gen())
            }
            _ => unreachable!(),
        }
    }

    fn rational(&mut self) -> f64 {
        let num = self.rng.gen_range(-100..=100);
        let mut denom = self.rng.gen_range(-100..=100);
        while denom == 0 {
            denom = self.rng.gen_range(-100..=100);
        }
        num as f64 / denom as f64
    }
}
