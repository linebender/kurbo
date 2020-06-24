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

//! A garden of data structures for manipulating 2D shapes and curves.
//!
//! The kurbo library contains data structures and algorithms for curves and
//! vector paths. It is probably most appropriate for creative tools,
//! but is general enough it might be useful for other applications.

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![allow(clippy::unreadable_literal)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::excessive_precision)]
#![deny(clippy::trivially_copy_pass_by_ref)]

mod affine;
mod arc;
mod bezpath;
mod circle;
pub mod common;
mod cubicbez;
mod ellipse;
mod insets;
mod line;
mod param_curve;
mod point;
mod quadbez;
mod rect;
mod rounded_rect;
mod shape;
mod size;
mod svg;
mod translate_scale;
mod vec2;

pub use crate::affine::*;
pub use crate::arc::*;
pub use crate::bezpath::*;
pub use crate::circle::*;
pub use crate::cubicbez::*;
pub use crate::ellipse::*;
pub use crate::insets::*;
pub use crate::line::*;
pub use crate::param_curve::*;
pub use crate::point::*;
pub use crate::quadbez::*;
pub use crate::rect::*;
pub use crate::rounded_rect::*;
pub use crate::shape::*;
pub use crate::size::*;
pub use crate::svg::*;
pub use crate::translate_scale::*;
pub use crate::vec2::*;
