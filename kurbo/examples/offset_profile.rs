// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

// Scratch harness used while optimizing offset2; kept for the record.
//
// - `offset_profile bench`: time per call for offset, offset2 and strict,
//   over the same families and settings as `offset_compare`, best of five.
// - `offset_profile fingerprint`: segment counts and a hash of the output of
//   offset2 and strict, to check whether a change alters the output.
// - `offset_profile oracle`: compare `kurbo::offset_oracle` with the sampled
//   one-sided distance it replaced, on the same results.
// - `offset_profile FAMILY D TOL WHICH REPS`: a loop to run under a
//   profiler, where FAMILY is wild, gentle or nearly, and WHICH is old, new
//   or strict.
#![allow(missing_docs, clippy::print_stdout, reason = "scratch")]
use kurbo::{BezPath, CubicBez, Point};
use std::hint::black_box;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1_u64 << 53) as f64
    }
    fn signed(&mut self) -> f64 {
        2.0 * self.next() - 1.0
    }
}

fn fingerprint() {
    use kurbo::PathEl;
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    for (fi, family) in ["nearly", "gentle", "wild"].iter().enumerate() {
        let curves: Vec<CubicBez> = (0..300).map(|_| make(family, &mut rng)).collect();
        for which in [0, 1] {
            let f: fn(CubicBez, f64, f64, &mut BezPath) = if which == 0 {
                kurbo::offset2::offset_cubic
            } else {
                kurbo::offset2::offset_cubic_strict
            };
            let mut h: u64 = 0xcbf29ce484222325;
            let mut nseg = 0;
            let mut path = BezPath::new();
            for c in &curves {
                for d in [0.5, 2.0, 10.0, -0.5, -2.0, -10.0] {
                    for tol in [0.25, 0.01, 0.0001] {
                        f(*c, d, tol, &mut path);
                        for el in path.elements() {
                            let pts: Vec<Point> = match *el {
                                PathEl::MoveTo(p) => vec![p],
                                PathEl::CurveTo(a, b, c) => {
                                    nseg += 1;
                                    vec![a, b, c]
                                }
                                _ => vec![],
                            };
                            for p in pts {
                                for x in [p.x, p.y] {
                                    h = (h ^ x.to_bits()).wrapping_mul(0x100000001b3);
                                }
                            }
                        }
                    }
                }
            }
            println!("{} {} segs={} hash={:016x}", fi, which, nseg, h);
        }
    }
}

fn make(family: &str, rng: &mut Rng) -> CubicBez {
    match family {
        "wild" => {
            let mut p = || Point::new(rng.next() * 100.0, rng.next() * 100.0);
            CubicBez::new(p(), p(), p(), p())
        }
        "gentle" => {
            let bulge = 30.0 * rng.signed();
            CubicBez::new(
                (0.0, 0.0),
                (33.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
                (67.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
                (100.0, 10.0 * rng.signed()),
            )
        }
        _ => CubicBez::new(
            (0.0, 0.0),
            (33.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
            (67.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
            (100.0, 0.0),
        ),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(|s| s.as_str()) == Some("fingerprint") {
        fingerprint();
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("g1") {
        g1_sweep();
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("oracle") {
        oracle_check();
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("bench") {
        bench();
        return;
    }
    let family = args.get(1).map(|s| s.as_str()).unwrap_or("wild");
    let d: f64 = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(2.0);
    let tol: f64 = args.get(3).map(|s| s.parse().unwrap()).unwrap_or(0.01);
    let which = args.get(4).map(|s| s.as_str()).unwrap_or("new");
    let reps: usize = args.get(5).map(|s| s.parse().unwrap()).unwrap_or(20000);
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let curves: Vec<CubicBez> = (0..100)
        .map(|_| match family {
            "wild" => {
                let mut p = || Point::new(rng.next() * 100.0, rng.next() * 100.0);
                CubicBez::new(p(), p(), p(), p())
            }
            "gentle" => {
                let bulge = 30.0 * rng.signed();
                CubicBez::new(
                    (0.0, 0.0),
                    (33.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
                    (67.0 + 8.0 * rng.signed(), bulge + 5.0 * rng.signed()),
                    (100.0, 10.0 * rng.signed()),
                )
            }
            _ => CubicBez::new(
                (0.0, 0.0),
                (33.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
                (67.0 + 8.0 * rng.signed(), 0.5 * rng.signed()),
                (100.0, 0.0),
            ),
        })
        .collect();
    let f: fn(CubicBez, f64, f64, &mut BezPath) = match which {
        "old" => kurbo::offset::offset_cubic,
        "strict" => kurbo::offset2::offset_cubic_strict,
        _ => kurbo::offset2::offset_cubic,
    };
    let mut path = BezPath::new();
    let start = std::time::Instant::now();
    for _ in 0..reps {
        for c in &curves {
            for d in [d, -d] {
                f(black_box(*c), d, tol, &mut path);
                black_box(&path);
            }
        }
    }
    println!(
        "{:.1} ns/call",
        start.elapsed().as_nanos() as f64 / (reps * 200) as f64
    );
}

fn time_one(f: fn(CubicBez, f64, f64, &mut BezPath), curves: &[CubicBez], d: f64, tol: f64) -> f64 {
    let mut path = BezPath::new();
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        let reps = 100;
        let start = std::time::Instant::now();
        for _ in 0..reps {
            for c in curves {
                for d in [d, -d] {
                    f(black_box(*c), d, tol, &mut path);
                    black_box(&path);
                }
            }
        }
        best = best.min(start.elapsed().as_nanos() as f64 / (reps * curves.len() * 2) as f64);
    }
    best
}

fn bench() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let mut tot = [0.0; 3];
    println!(
        "{:<8}{:>5}{:>8} {:>7}{:>7}{:>7}",
        "family", "d", "tol", "old", "new", "strict"
    );
    for family in ["nearly", "gentle", "wild"] {
        let curves: Vec<CubicBez> = (0..100).map(|_| make(family, &mut rng)).collect();
        for d in [0.5, 2.0, 10.0] {
            for tol in [0.25, 0.01, 0.0001] {
                let o = time_one(kurbo::offset::offset_cubic, &curves, d, tol);
                let n = time_one(kurbo::offset2::offset_cubic, &curves, d, tol);
                let s = time_one(kurbo::offset2::offset_cubic_strict, &curves, d, tol);
                tot[0] += o.ln();
                tot[1] += n.ln();
                tot[2] += s.ln();
                println!(
                    "{:<8}{:>5}{:>8} {:>7.0}{:>7.0}{:>7.0}",
                    family, d, tol, o, n, s
                );
            }
        }
    }
    println!(
        "geomean          {:>7.0}{:>7.0}{:>7.0}",
        (tot[0] / 27.0).exp(),
        (tot[1] / 27.0).exp(),
        (tot[2] / 27.0).exp()
    );
}

/// The largest distance from a point of `result` to the offset of `c`, by
/// sampling: the oracle used before `kurbo::offset_oracle`.
fn sampled_error(c: &CubicBez, d: f64, result: &BezPath) -> f64 {
    use kurbo::{ParamCurve, ParamCurveDeriv};
    const N: usize = 2000;
    let c1 = c.deriv();
    let offset_at = |s: f64| {
        let s = s.clamp(1e-9, 1.0 - 1e-9);
        c.eval(s) + d * c1.eval(s).to_vec2().normalize().turn_90()
    };
    let samples: Vec<Point> = (0..=N).map(|j| offset_at(j as f64 / N as f64)).collect();
    let mut worst = 0.0_f64;
    for seg in result.segments() {
        for i in 0..=10 {
            let p = seg.eval(i as f64 / 10.0);
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

/// Compare the Fréchet oracle with the sampled one.
fn oracle_check() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    println!(
        "family   which  | calls unmeas | frechet<sampled-eps | frechet>1.1*sampled+eps | over tol: sampled frechet"
    );
    for family in ["nearly", "gentle", "wild"] {
        let curves: Vec<CubicBez> = (0..100).map(|_| make(family, &mut rng)).collect();
        for (name, f) in [
            (
                "old",
                kurbo::offset::offset_cubic as fn(CubicBez, f64, f64, &mut BezPath),
            ),
            ("new", kurbo::offset2::offset_cubic),
            ("strict", kurbo::offset2::offset_cubic_strict),
        ] {
            let (mut calls, mut unmeasured, mut lower, mut higher, mut over_s, mut over_f) =
                (0, 0, 0, 0, 0, 0);
            let mut reasons = [0_usize; 3];
            let mut worst_lower = (0.0, String::new());
            let mut path = BezPath::new();
            for c in &curves {
                for d in [0.5, 2.0, 10.0, -0.5, -2.0, -10.0] {
                    for tol in [0.25, 0.01, 0.0001] {
                        f(*c, d, tol, &mut path);
                        calls += 1;
                        let sampled =
                            sampled_error(&kurbo::offset_oracle::regularized(*c, tol), d, &path);
                        let r = kurbo::offset_oracle::result_error(*c, d, tol, &path);
                        let eps = 1e-6 * tol;
                        if r.unmeasured > 0 {
                            unmeasured += 1;
                        }
                        for k in 0..3 {
                            reasons[k] += r.unmeasured_reasons[k];
                        }
                        let frechet = r.error.max(r.unmeasured_lower);
                        if frechet < sampled - eps - 0.05 * tol {
                            lower += 1;
                            let gap = (sampled - frechet) / tol;
                            if gap > worst_lower.0 {
                                worst_lower = (
                                    gap,
                                    format!(
                                        "{c:?} d={d} tol={tol} sampled={sampled:e} frechet={frechet:e} unmeasured={}",
                                        r.unmeasured
                                    ),
                                );
                            }
                        }
                        if frechet > 1.1 * sampled + eps {
                            higher += 1;
                        }
                        if sampled > tol * 1.001 {
                            over_s += 1;
                        }
                        if frechet > tol * 1.001 {
                            over_f += 1;
                        }
                    }
                }
            }
            println!(
                "{family:<8} {name:<6} | {calls:>5} {unmeasured:>6} | {lower:>19} | {higher:>23} | {over_s:>5} {over_f:>5}"
            );
            println!(
                "    unmeasured pieces: no range {}, empty range {}, rays not advancing {}",
                reasons[0], reasons[1], reasons[2]
            );
            if !worst_lower.1.is_empty() {
                println!(
                    "    worst lower by {:.2} tol: {}",
                    worst_lower.0, worst_lower.1
                );
            }
        }
    }
}

/// Sweep the factor within which the midpoint G1 fit is tried: segments,
/// calls over tolerance by the Fréchet oracle, and time.
fn g1_sweep() {
    println!(
        "{:<8} | {:>16} | {:>16} | {:>16} | geomean ns",
        "factor", "nearly segs/over", "gentle segs/over", "wild segs/over"
    );
    for factor in [0.0, 2.0, 4.0, 8.0, 16.0, 64.0] {
        kurbo::offset2::g1_retry::set(factor);
        for (name, f) in [
            (
                "new",
                kurbo::offset2::offset_cubic as fn(CubicBez, f64, f64, &mut BezPath),
            ),
            ("strict", kurbo::offset2::offset_cubic_strict),
        ] {
            let mut rng = Rng(0x2545_f491_4f6c_dd1d);
            let mut cells = Vec::new();
            let mut log_time = 0.0;
            for family in ["nearly", "gentle", "wild"] {
                let curves: Vec<CubicBez> = (0..100).map(|_| make(family, &mut rng)).collect();
                let (mut segs, mut over) = (0, 0);
                let mut path = BezPath::new();
                for d in [0.5, 2.0, 10.0] {
                    for tol in [0.25, 0.01, 0.0001] {
                        for c in &curves {
                            for dd in [d, -d] {
                                f(*c, dd, tol, &mut path);
                                segs += path.segments().count();
                                let r = kurbo::offset_oracle::result_error(*c, dd, tol, &path);
                                if r.error.max(r.unmeasured_lower) > tol * 1.001 {
                                    over += 1;
                                }
                            }
                        }
                        log_time += time_one(f, &curves, d, tol).ln();
                    }
                }
                cells.push(format!("{segs:>6} / {over:>3}"));
            }
            println!(
                "{:<8} | {:>16} | {:>16} | {:>16} | {:.0} {name}",
                factor,
                cells[0],
                cells[1],
                cells[2],
                (log_time / 27.0).exp()
            );
        }
    }
}
