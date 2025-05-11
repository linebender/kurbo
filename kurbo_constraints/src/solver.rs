// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

#![allow(dead_code)]

use crate::{Constraint, DebugInfo, Handle};
use alloc::boxed::Box;
use alloc::vec::Vec;
use hashbrown::HashMap;
use kurbo::Point;

#[derive(Debug, Default)]
pub struct Solver {
    meta: SolverMeta,
    state: SolverState,
}

#[derive(Debug)]
pub struct SolverMeta {
    constraints: Vec<Box<dyn Constraint>>,
    tolerance: f64,
    debug: DebugInfo,
}

#[derive(Debug, Default)]
pub struct SolverState {
    points: HashMap<Handle, Point>,
}

#[derive(Debug)]
pub enum SolverError {
    Diverged,
    Infeasible,
}

impl Default for SolverMeta {
    fn default() -> Self {
        Self {
            constraints: Vec::new(),
            tolerance: 1e-6,
            debug: DebugInfo::default(),
        }
    }
}

impl Solver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_point(&mut self, pos: Point) -> Handle {
        let handle = Handle::new();
        self.state.points.insert(handle, pos);
        handle
    }

    pub fn add_constraint(&mut self, constraint: impl Constraint + 'static) {
        self.meta.constraints.push(Box::new(constraint));
    }

    pub fn solve(&mut self, iterations: usize) -> Result<(), SolverError> {
        self.meta.debug.position_history.clear();

        // Record initial state
        for (handle, pos) in &self.state.points {
            self.meta
                .debug
                .position_history
                .entry(*handle)
                .or_default()
                .push(*pos);
        }

        for _ in 0..iterations {
            for constraint in &self.meta.constraints {
                constraint.apply(&mut self.state, &self.meta)?;
            }

            // Record positions
            for (handle, pos) in &self.state.points {
                self.meta
                    .debug
                    .position_history
                    .get_mut(handle)
                    .unwrap()
                    .push(*pos);
            }
        }
        Ok(())
    }
}
