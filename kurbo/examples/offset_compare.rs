// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Compare `kurbo::offset::offset_cubic` with `kurbo::offset2::offset_cubic`.
//!
//! For several families of cubics, offset distances and tolerances, this
//! prints the number of output segments, the time per call, and how often
//! the measured error exceeds the tolerance. In each group of three columns
//! the first is `offset`, the second `offset2::offset_cubic` and the third
//! `offset2::offset_cubic_strict`.
//!
//! The error is measured by `kurbo::offset_oracle`: the Fréchet distance of
//! each piece of the result from the offset of the regularized input, which
//! is what both functions approximate. A piece the oracle cannot measure
//! counts with a lower bound on its error.

#![allow(clippy::print_stdout, reason = "this example exists to print a table")]

use std::hint::black_box;
use std::time::Instant;

use kurbo::{BezPath, CubicBez, Point};

/// A small deterministic random number generator.
struct Rng(u64);

impl Rng {
    /// A uniform value in [0, 1).
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1_u64 << 53) as f64
    }

    /// A uniform value in [-1, 1).
    fn signed(&mut self) -> f64 {
        2.0 * self.next() - 1.0
    }
}

/// Four random control points in a 100 unit box. Many of these have loops,
/// inflections and tight turns.
fn wild(rng: &mut Rng) -> CubicBez {
    let mut point = || Point::new(rng.next() * 100.0, rng.next() * 100.0);
    CubicBez::new(point(), point(), point(), point())
}

/// A smooth arc about 100 units long, turning through at most a right angle
/// or so, with control points near the thirds of the chord.
fn gentle(rng: &mut Rng) -> CubicBez {
    let bulge = 30.0 * rng.signed();
    CubicBez::new(
        (0.0, 0.0),
        (33.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
        (67.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
        (100.0, 10.0 * rng.signed()),
    )
}

/// A nearly straight cubic.
fn nearly_straight(rng: &mut Rng) -> CubicBez {
    CubicBez::new(
        (0.0, 0.0),
        (33.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
        (67.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
        (100.0, 0.0),
    )
}

type OffsetFn = fn(CubicBez, f64, f64, &mut BezPath);

/// A name and a generator for a family of test curves.
type Family = (&'static str, fn(&mut Rng) -> CubicBez);

/// Segments per call, nanoseconds per call, and the fraction of calls whose
/// measured error is over the tolerance.
fn measure(f: OffsetFn, curves: &[CubicBez], d: f64, tolerance: f64) -> (f64, f64, f64) {
    let mut path = BezPath::new();
    let mut n_segments = 0;
    let mut n_over = 0;
    let mut n_calls = 0;
    for c in curves {
        for d in [d, -d] {
            f(*c, d, tolerance, &mut path);
            n_segments += path.segments().count();
            let report = kurbo::offset_oracle::result_error(*c, d, tolerance, &path);
            if report.error.max(report.unmeasured_lower) > tolerance * 1.001 {
                n_over += 1;
            }
            n_calls += 1;
        }
    }
    const REPS: usize = 200;
    let start = Instant::now();
    for _ in 0..REPS {
        for c in curves {
            for d in [d, -d] {
                f(black_box(*c), d, tolerance, &mut path);
                black_box(&path);
            }
        }
    }
    let ns = start.elapsed().as_nanos() as f64 / (REPS * n_calls) as f64;
    (
        n_segments as f64 / n_calls as f64,
        ns,
        n_over as f64 / n_calls as f64,
    )
}

fn main() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let families: [Family; 3] = [
        ("nearly straight", nearly_straight),
        ("gentle", gentle),
        ("wild", wild),
    ];
    println!(
        "{:<16}{:>5}{:>8} |{:>6}{:>6}{:>7} |{:>7}{:>7}{:>7} |{:>5}{:>5}{:>7}",
        "family",
        "d",
        "tol",
        "segs:",
        "new",
        "strict",
        "ns:",
        "new",
        "strict",
        "over:",
        "new",
        "strict"
    );
    for (name, make) in families {
        let curves: Vec<CubicBez> = (0..100).map(|_| make(&mut rng)).collect();
        for d in [0.5, 2.0, 10.0] {
            for tolerance in [0.25, 0.01, 0.0001] {
                let old = measure(kurbo::offset::offset_cubic, &curves, d, tolerance);
                let new = measure(kurbo::offset2::offset_cubic, &curves, d, tolerance);
                let strict = measure(kurbo::offset2::offset_cubic_strict, &curves, d, tolerance);
                println!(
                    "{:<16}{:>5}{:>8} |{:>6.2}{:>6.2}{:>7.2} |{:>7.0}{:>7.0}{:>7.0} |{:>4.0}%{:>4.0}%{:>6.0}%",
                    name,
                    d,
                    tolerance,
                    old.0,
                    new.0,
                    strict.0,
                    old.1,
                    new.1,
                    strict.1,
                    100.0 * old.2,
                    100.0 * new.2,
                    100.0 * strict.2
                );
            }
        }
    }
}
