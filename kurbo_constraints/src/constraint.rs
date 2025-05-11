// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::{Handle, SolverError, SolverMeta, SolverState};
use alloc::vec::Vec;

pub trait Constraint: core::fmt::Debug {
    fn apply(&self, state: &mut SolverState, meta: &SolverMeta) -> Result<(), SolverError>;
    fn targets(&self, meta: &SolverMeta) -> Vec<Handle>;
    fn type_name(&self) -> &'static str {
        core::any::type_name::<Self>()
    }
}
