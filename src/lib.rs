// Copyright 2018 The kurbo Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! A garden of data structures for manipulating 2D curves.

mod affine;
mod bezpath;
mod circle;
pub mod common;
mod cubicbez;
#[cfg(feature = "euclid")]
mod euclid;
mod line;
mod param_curve;
mod quadbez;
mod rect;
mod shape;
mod svg;
mod vec2;

pub use crate::affine::*;
pub use crate::bezpath::*;
pub use crate::circle::*;
pub use crate::cubicbez::*;
pub use crate::line::*;
pub use crate::param_curve::*;
pub use crate::quadbez::*;
pub use crate::rect::*;
pub use crate::shape::*;
pub use crate::svg::*;
pub use crate::vec2::*;
