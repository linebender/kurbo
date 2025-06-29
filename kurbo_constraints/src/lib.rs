// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Kurbo Constraints

// LINEBENDER LINT SET - lib.rs - v3
// See https://linebender.org/wiki/canonical-lints/
// These lints shouldn't apply to examples or tests.
#![cfg_attr(not(test), warn(unused_crate_dependencies))]
// These lints shouldn't apply to examples.
#![warn(clippy::print_stdout, clippy::print_stderr)]
// Targeting e.g. 32-bit means structs containing usize can give false positives for 64-bit.
#![cfg_attr(target_pointer_width = "64", warn(clippy::trivially_copy_pass_by_ref))]
// END LINEBENDER LINT SET
#![cfg_attr(docsrs, feature(doc_auto_cfg))]
#![allow(
    clippy::exhaustive_enums,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::new_without_default,
    missing_docs
)]
#![no_std]

extern crate alloc;

mod constraint;
mod debuginfo;
mod handle;
mod solver;

pub use constraint::Constraint;
pub use debuginfo::DebugInfo;
pub use handle::Handle;
pub use solver::{Solver, SolverError, SolverMeta, SolverState};
