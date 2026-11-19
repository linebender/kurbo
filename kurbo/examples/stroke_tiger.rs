// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Time whole-stroke expansion of the Ghostscript tiger, with the cubic
//! offsets computed by `offset`, by `offset2` one side at a time, and by
//! `offset2` for both sides at once (`offset_cubic_pair`). The last two give
//! the same outline.
//!
//! Usage: `cargo run --release --example stroke_tiger -- path/to/tiger.svg`
//!
//! This expects the 200 by 200 tiger from Vello's assets, usually rendered
//! at 8x, so a quarter pixel is 1/32 in source coordinates. Every stroked
//! path is expanded with the SVG defaults: miter joins with a limit of 4 and
//! butt caps. Unlike `offset_tiger`, this includes lines, joins, caps and
//! the stroker's own handling of nearly straight cubics.

#![allow(clippy::print_stdout, reason = "this example exists to print a table")]

use std::hint::black_box;
use std::time::Instant;

use kurbo::{BezPath, Cap, Join, OffsetMethod, PathEl, Stroke, StrokeCtx, StrokeOpts, stroke_with};

/// The value of an attribute in the text of one element.
fn attribute<'a>(element: &'a str, name: &str) -> Option<&'a str> {
    let pattern = format!(" {name}=\"");
    let start = element.find(&pattern)? + pattern.len();
    let end = start + element[start..].find('"')?;
    Some(&element[start..end])
}

/// The stroked paths, each with its stroke style.
fn stroked_paths(svg: &str) -> Vec<(BezPath, Stroke)> {
    let mut paths = Vec::new();
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
        let style = Stroke::new(width)
            .with_join(Join::Miter)
            .with_miter_limit(4.0)
            .with_caps(Cap::Butt);
        paths.push((BezPath::from_svg(d).expect("path data"), style));
    }
    paths
}

/// Output segments for the whole tiger, and microseconds per whole tiger
/// (best of five).
fn measure(paths: &[(BezPath, Stroke)], opts: &StrokeOpts, tolerance: f64) -> (usize, f64) {
    let mut ctx = StrokeCtx::default();
    let mut n_segments = 0;
    for (path, style) in paths {
        stroke_with(path.iter(), style, opts, tolerance, &mut ctx);
        n_segments += ctx
            .output()
            .elements()
            .iter()
            .filter(|el| !matches!(el, PathEl::MoveTo(_) | PathEl::ClosePath))
            .count();
    }
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        const REPS: usize = 200;
        let start = Instant::now();
        for _ in 0..REPS {
            for (path, style) in paths {
                stroke_with(black_box(path).iter(), style, opts, tolerance, &mut ctx);
                black_box(ctx.output());
            }
        }
        best = best.min(start.elapsed().as_secs_f64() * 1e6 / REPS as f64);
    }
    (n_segments, best)
}

fn main() {
    let file = std::env::args()
        .nth(1)
        .expect("usage: stroke_tiger path/to/tiger.svg");
    let svg = std::fs::read_to_string(&file).expect("reading the SVG");
    let paths = stroked_paths(&svg);
    let n_input: usize = paths.iter().map(|(path, _)| path.segments().count()).sum();
    println!(
        "{} stroked paths, {n_input} input segments; time is per whole tiger",
        paths.len()
    );
    println!(
        "{:>10} |{:>8}{:>8} |{:>9}{:>9}{:>9} |{:>8}{:>8}",
        "tol", "segs:", "offset2", "us:", "offset2", "pair", "ratio:", "pair"
    );
    let opts = |method| StrokeOpts::default().offset_method(method);
    for tolerance in [1.0 / 32.0, 1.0 / 256.0, 1.0 / 2048.0] {
        let old = measure(&paths, &opts(OffsetMethod::Offset), tolerance);
        let new = measure(&paths, &opts(OffsetMethod::Offset2), tolerance);
        let pair = measure(&paths, &opts(OffsetMethod::Offset2Pair), tolerance);
        assert_eq!(new.0, pair.0, "the pair should give the same outline");
        println!(
            "{:>10} |{:>8}{:>8} |{:>9.1}{:>9.1}{:>9.1} |{:>8.2}{:>8.2}",
            format!("1/{}", (1.0 / tolerance).round()),
            old.0,
            new.0,
            old.1,
            new.1,
            pair.1,
            new.1 / old.1,
            pair.1 / old.1
        );
    }
}
