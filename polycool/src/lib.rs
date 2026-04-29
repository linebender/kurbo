// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! This is a crate for numerical polynomial root-finding.
//!
//! Currently, we implement a single solver: Yuksel's iterative algorithm
//! for finding roots in a bounded interval. We aspire to have
//! more, with thorough tests and benchmarks.

// LINEBENDER LINT SET - lib.rs - v4
// See https://linebender.org/wiki/canonical-lints/
// These lints shouldn't apply to examples or tests.
#![cfg_attr(not(test), warn(unused_crate_dependencies))]
// These lints shouldn't apply to examples.
#![warn(clippy::print_stdout, clippy::print_stderr)]
// Targeting e.g. 32-bit means structs containing usize can give false positives for 64-bit.
#![cfg_attr(target_pointer_width = "64", warn(clippy::trivially_copy_pass_by_ref))]
// END LINEBENDER LINT SET
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(all(not(feature = "std"), not(test)), no_std)]
mod cubic;
#[cfg(feature = "libm")]
mod libm_polyfill;
mod poly;
#[cfg(feature = "std")]
mod poly_dyn;
mod quadratic;
mod yuksel;

#[cfg(any(test, feature = "arbitrary"))]
pub mod arbitrary;

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("kurbo requires either the `std` or `libm` feature");

// Suppress the unused_crate_dependencies lint when both std and libm are specified.
#[cfg(all(feature = "std", feature = "libm"))]
use libm as _;

pub use poly::{Cubic, Poly, Quadratic, Quartic, Quintic};
#[cfg(feature = "std")]
pub use poly_dyn::PolyDyn;

fn different_signs(x: f64, y: f64) -> bool {
    (x < 0.0) != (y < 0.0)
}
