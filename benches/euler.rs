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

// Note: this `cfg` may not work, TODO either move to criterion or sort it out.
// I've been locally removing it to run benches.
#![cfg(nightly)]
#![feature(test)]
extern crate test;
use test::{Bencher, black_box};

use kurbo::*;

// About 15ns
#[bench]
fn bench_euler_integ(b: &mut Bencher) {
    b.iter(|| integ_euler(black_box(0.1), black_box(0.2), 1e-9))
}

// About 21ns. This is evidence that computing this root directly as part of the
// error estimation would be very expensive, unless it could be constant folded.
#[bench]
fn bench_sixth_root(b: &mut Bencher) {
    b.iter(|| black_box(0.1f64).powf(1.0 / 6.0))
}

// A lower order approximation to the main integral. The code is here
// if we want to use it, and might be a speedup if the workload consists
// of very low deflection segments (and/or the error threshold is high),
// but for general use any loss of branch prediction coherence would
// probably be a lose.
fn integ_euler_8(k0: f64, k1: f64) -> (f64, f64) {
    let t1_1 = k0;
    let t1_2 = 0.5 * k1;
    let t2_2 = t1_1 * t1_1;
    let t2_3 = 2. * (t1_1 * t1_2);
    let t2_4 = t1_2 * t1_2;
    let t3_4 = t2_2 * t1_2 + t2_3 * t1_1;
    let t3_6 = t2_4 * t1_2;
    let t4_4 = t2_2 * t2_2;
    let t4_5 = 2. * (t2_2 * t2_3);
    let t4_6 = 2. * (t2_2 * t2_4) + t2_3 * t2_3;
    let t5_6 = t4_4 * t1_2 + t4_5 * t1_1;
    let t6_6 = t4_4 * t2_2;
    let mut u = 1.;
    u -= (1./24.) * t2_2 + (1./160.) * t2_4;
    u += (1./1920.) * t4_4 + (1./10752.) * t4_6;
    u -= (1./322560.) * t6_6;
    let mut v = (1./12.) * t1_2;
    v -= (1./480.) * t3_4 + (1./2688.) * t3_6;
    v += (1./53760.) * t5_6;
    (u, v)
}

// About 4ns
#[bench]
fn bench_euler_integ_8(b: &mut Bencher) {
    b.iter(|| integ_euler_8(black_box(0.1), black_box(0.2)))
}

// About 240ns
#[bench]
fn bench_fit_euler(b: &mut Bencher) {
    b.iter(|| FitEulerResult::fit_euler(black_box(0.1), black_box(0.2)))
}

// Pretty much straight out of the notebook. Looking at the generated
// asm, the compiler does a good job 
fn fast_fit_euler(th0: f64, th1: f64) -> f64 {
    let k0 = th0 + th1;
    let dth = th1 - th0;
    let mut est = dth * 6.;
    est += dth.powi(3) * (1. / -70.);
    est += dth.powi(5) * (1. / -10780.);
    est += dth.powi(7) * 2.769178184818219e-07;
    est += dth * k0.powi(2) * (1. / -10.);
    est += dth.powi(3) * k0.powi(2) * (1. / 4200.);
    est += dth.powi(5) * k0.powi(2) * 1.6959677820260655e-05;
    est += dth * k0.powi(4) * (1. / -1400.);
    est += dth.powi(3) * k0.powi(4) * 6.84915970574303e-05;
    est += dth * k0.powi(6) * -7.936475029053326e-06;
    est
}

// Same as above but using mostly even powers. A small improvement.
fn fast_fit_euler2(th0: f64, th1: f64) -> f64 {
    let k0 = th0 + th1;
    let dth = th1 - th0;
    let mut est = 6.;
    est += dth.powi(2) * (1. / -70.);
    est += dth.powi(4) * (1. / -10780.);
    est += dth.powi(6) * 2.769178184818219e-07;
    est += k0.powi(2) * (1. / -10.);
    est += dth.powi(2) * k0.powi(2) * (1. / 4200.);
    est += dth.powi(4) * k0.powi(2) * 1.6959677820260655e-05;
    est += k0.powi(4) * (1. / -1400.);
    est += dth.powi(2) * k0.powi(4) * 6.84915970574303e-05;
    est += k0.powi(6) * -7.936475029053326e-06;
    est * dth
}

// Compute both k1 and chord at the same time, which is what an
// actual implementation would use.
fn fast_fit_euler3(th0: f64, th1: f64) -> (f64, f64) {
    let k0 = th0 + th1;
    let dth = th1 - th0;
    let mut k1 = 6.;
    k1 += dth.powi(2) * (1. / -70.);
    k1 += dth.powi(4) * (1. / -10780.);
    k1 += dth.powi(6) * 2.769178184818219e-07;
    k1 += k0.powi(2) * (1. / -10.);
    k1 += dth.powi(2) * k0.powi(2) * (1. / 4200.);
    k1 += dth.powi(4) * k0.powi(2) * 1.6959677820260655e-05;
    k1 += k0.powi(4) * (1. / -1400.);
    k1 += dth.powi(2) * k0.powi(4) * 6.84915970574303e-05;
    k1 += k0.powi(6) * -7.936475029053326e-06;
    let mut ch = 1.;
    ch += dth.powi(2) * (1. / -40.);
    ch += dth.powi(4) * 0.00034226190482569864;
    ch += dth.powi(6) * -1.9349474568904524e-06;
    ch += k0.powi(2) * (1. / -24.);
    ch += dth.powi(2) * k0.powi(2) * 0.0024702380951963226;
    ch += dth.powi(4) * k0.powi(2) * -3.7297408997537985e-05;
    ch += k0.powi(4) * (1. / 1920.);
    ch += dth.powi(2) * k0.powi(4) * -4.87350869747975e-05;
    ch += k0.powi(6) * -3.1001936068463107e-06;
    (k1 * dth, ch)
}

// About 3ns
#[bench]
fn bench_fast_fit_euler(b: &mut Bencher) {
    b.iter(|| fast_fit_euler(black_box(0.1), black_box(0.2)))
}

// About 2ns
#[bench]
fn bench_fast_fit_euler2(b: &mut Bencher) {
    b.iter(|| fast_fit_euler2(black_box(0.1), black_box(0.2)))
}

// About 7ns
#[bench]
fn bench_fast_fit_euler3(b: &mut Bencher) {
    b.iter(|| fast_fit_euler3(black_box(0.1), black_box(0.2)))
}
