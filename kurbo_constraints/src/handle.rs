// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use core::sync::atomic::{AtomicU32, Ordering};

static HANDLE_COUNTER: AtomicU32 = AtomicU32::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Handle(u32);

impl Handle {
    pub fn new() -> Self {
        Self(HANDLE_COUNTER.fetch_add(1, Ordering::Relaxed))
    }
}
