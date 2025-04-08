// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::Handle;
use alloc::vec::Vec;
use hashbrown::HashMap;
use kurbo::Point;

#[derive(Debug, Default)]
pub struct DebugInfo {
    pub position_history: HashMap<Handle, Vec<Point>>,
}
