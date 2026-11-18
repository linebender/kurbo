// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Compare `offset` with `offset2` on the strokes of the Ghostscript tiger.
//!
//! Usage: `cargo run --release --example offset_tiger -- path/to/tiger.svg`
//!
//! This expects the 200 by 200 version of the tiger found in Vello's assets,
//! which is usually rendered at 8x. A quarter pixel there is a tolerance of
//! 1/32 in source coordinates, so that is the main setting; two tighter ones
//! are included to exercise more than the fast paths.
//!
//! Every curve of every stroked path is offset by half the stroke width, on
//! both sides. Lines are skipped, as the stroker does not offset them as
//! curves, and quadratics are raised to cubics. The parsing of the SVG is
//! minimal and only meant for this file.
//!
//! In each group of three columns the first is `offset`, the second
//! `offset2::offset_cubic` and the third `offset2::offset_cubic_strict`.

#![allow(clippy::print_stdout, reason = "this example exists to print a table")]

use std::hint::black_box;
use std::time::Instant;

use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, PathSeg, Point};

/// The value of an attribute in the text of one element.
fn attribute<'a>(element: &'a str, name: &str) -> Option<&'a str> {
    let pattern = format!(" {name}=\"");
    let start = element.find(&pattern)? + pattern.len();
    let end = start + element[start..].find('"')?;
    Some(&element[start..end])
}

/// The cubics of the stroked paths, each with its half stroke width.
fn stroked_cubics(svg: &str) -> Vec<(CubicBez, f64)> {
    let mut cubics = Vec::new();
    for element in svg.split("<path").skip(1) {
        let element = &element[..element.find('>').unwrap_or(element.len())];
        let stroked = attribute(element, "stroke").is_some_and(|stroke| stroke != "none");
        let Some(d) = attribute(element, "d") else {
            continue;
        };
        if !stroked {
            continue;
        }
        let width: f64 = attribute(element, "stroke-width")
            .map(|width| width.parse().expect("stroke width"))
            .unwrap_or(1.0);
        let path = BezPath::from_svg(d).expect("path data");
        for seg in path.segments() {
            match seg {
                PathSeg::Cubic(c) => cubics.push((c, 0.5 * width)),
                PathSeg::Quad(q) => cubics.push((q.raise(), 0.5 * width)),
                PathSeg::Line(_) => {}
            }
        }
    }
    cubics
}

/// The largest distance from a point of `result` to the true offset of `c`.
///
/// The same measure as in `offset_compare`, except that the parameter is
/// kept just inside the ends. Many of the tiger's cubics have a control
/// point on an end point, where the derivative vanishes and the normal is
/// only defined as a limit.
fn result_error(c: &CubicBez, d: f64, result: &BezPath) -> f64 {
    const N: usize = 2000;
    const END_EPSILON: f64 = 1e-9;
    let c1 = c.deriv();
    let offset_at = |s: f64| {
        let s = s.clamp(END_EPSILON, 1.0 - END_EPSILON);
        c.eval(s) + d * c1.eval(s).to_vec2().normalize().turn_90()
    };
    let samples: Vec<Point> = (0..=N).map(|j| offset_at(j as f64 / N as f64)).collect();
    let mut worst = 0.0_f64;
    for seg in result.segments() {
        for i in 0..=10 {
            let p = seg.eval(i as f64 / 10.0);
            // Refine around every local minimum of the sampled distance, not
            // only the smallest. Where the offset comes back near itself,
            // the nearest sample can be on the wrong strand.
            let dist: Vec<f64> = samples.iter().map(|s| p.distance_squared(*s)).collect();
            let mut nearest = f64::INFINITY;
            for j in 0..=N {
                let left = j == 0 || dist[j] <= dist[j - 1];
                let right = j == N || dist[j] <= dist[j + 1];
                if !(left && right) {
                    continue;
                }
                let mut lo = j.saturating_sub(1) as f64 / N as f64;
                let mut hi = (j + 1).min(N) as f64 / N as f64;
                for _ in 0..50 {
                    let s1 = lo + (hi - lo) / 3.0;
                    let s2 = hi - (hi - lo) / 3.0;
                    if p.distance_squared(offset_at(s1)) < p.distance_squared(offset_at(s2)) {
                        hi = s2;
                    } else {
                        lo = s1;
                    }
                }
                nearest = nearest.min(p.distance(offset_at(0.5 * (lo + hi))));
            }
            worst = worst.max(nearest);
        }
    }
    worst
}

type OffsetFn = fn(CubicBez, f64, f64, &mut BezPath);

/// Total segments, nanoseconds per call (best of five), and the number of
/// calls whose measured error is over the tolerance.
fn measure(f: OffsetFn, cubics: &[(CubicBez, f64)], tolerance: f64) -> (usize, f64, usize) {
    let mut path = BezPath::new();
    let mut n_segments = 0;
    let mut n_over = 0;
    for (c, half_width) in cubics {
        for d in [*half_width, -half_width] {
            f(*c, d, tolerance, &mut path);
            n_segments += path.segments().count();
            if result_error(c, d, &path) > tolerance * 1.001 {
                n_over += 1;
            }
        }
    }
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        const REPS: usize = 200;
        let start = Instant::now();
        for _ in 0..REPS {
            for (c, half_width) in cubics {
                for d in [*half_width, -half_width] {
                    f(black_box(*c), d, tolerance, &mut path);
                    black_box(&path);
                }
            }
        }
        let ns = start.elapsed().as_nanos() as f64 / (REPS * 2 * cubics.len()) as f64;
        best = best.min(ns);
    }
    (n_segments, best, n_over)
}

fn main() {
    let file = std::env::args()
        .nth(1)
        .expect("usage: offset_tiger path/to/tiger.svg");
    let svg = std::fs::read_to_string(&file).expect("reading the SVG");
    let cubics = stroked_cubics(&svg);
    println!(
        "{} cubics in stroked paths, {} offset calls per setting",
        cubics.len(),
        2 * cubics.len()
    );
    println!(
        "{:>10} |{:>7}{:>7}{:>7} |{:>6}{:>6}{:>7} |{:>5}{:>5}{:>7}",
        "tol", "segs:", "new", "strict", "ns:", "new", "strict", "over:", "new", "strict"
    );
    for tolerance in [1.0 / 32.0, 1.0 / 256.0, 1.0 / 2048.0] {
        let old = measure(kurbo::offset::offset_cubic, &cubics, tolerance);
        let new = measure(kurbo::offset2::offset_cubic, &cubics, tolerance);
        let strict = measure(kurbo::offset2::offset_cubic_strict, &cubics, tolerance);
        println!(
            "{:>10} |{:>7}{:>7}{:>7} |{:>6.0}{:>6.0}{:>7.0} |{:>5}{:>5}{:>7}",
            format!("1/{}", (1.0 / tolerance).round()),
            old.0,
            new.0,
            strict.0,
            old.1,
            new.1,
            strict.1,
            old.2,
            new.2,
            strict.2
        );
    }
}
