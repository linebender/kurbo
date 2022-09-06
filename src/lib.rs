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

//! 2D geometry, with a focus on curves.
//!
//! The kurbo library contains data structures and algorithms for curves and
//! vector paths. It was designed to serve the needs of 2D graphics applications,
//! but it is intended to be general enough to be useful for other applications.
//!
//! Kurbo is designed to be used by [`Piet`], a crate for drawing 2D graphics,
//! and is in turn used by [`Druid`], a cross-platform GUI toolkit.
//!
//! # Examples
//!
//! Basic UI-style geometry:
//! ```
//! use kurbo::{Insets, Point, Rect, Size, Vec2};
//!
//! let pt = Point::new(10.0, 10.0);
//! let vector = Vec2::new(5.0, -5.0);
//! let pt2 = pt + vector;
//! assert_eq!(pt2, Point::new(15.0, 5.0));
//!
//! let rect = Rect::from_points(pt, pt2);
//! assert_eq!(rect, Rect::from_origin_size((10.0, 5.0), (5.0, 5.0)));
//!
//! let insets = Insets::uniform(1.0);
//! let inset_rect = rect - insets;
//! assert_eq!(inset_rect.size(), Size::new(3.0, 3.0));
//! ```
//!
//! Finding the closest position on a [`Shape`]'s perimeter to a [`Point`]:
//!
//! ```
//! use kurbo::{Circle, ParamCurve, ParamCurveNearest, Point, Shape};
//!
//! const DESIRED_ACCURACY: f64 = 0.1;
//!
//! /// Given a shape and a point, returns the closest position on the shape's
//! /// perimeter, or `None` if the shape is malformed.
//! fn closest_perimeter_point(shape: impl Shape, pt: Point) -> Option<Point> {
//!     let mut best: Option<(Point, f64)> = None;
//!     for segment in shape.path_segments(DESIRED_ACCURACY) {
//!         let nearest = segment.nearest(pt, DESIRED_ACCURACY);
//!         if best.map(|(_, best_d)| nearest.distance_sq < best_d).unwrap_or(true) {
//!             best = Some((segment.eval(nearest.t), nearest.distance_sq))
//!         }
//!     }
//!     best.map(|(point, _)| point)
//! }
//!
//! let circle = Circle::new((5.0, 5.0), 5.0);
//! let hit_point = Point::new(5.0, -2.0);
//! let expectation = Point::new(5.0, 0.0);
//! let hit = closest_perimeter_point(circle, hit_point).unwrap();
//! assert!(hit.distance(expectation) <= DESIRED_ACCURACY);
//! ```
//! [`Piet`]: https://docs.rs/piet
//! [`Druid`]: https://docs.rs/druid

#![forbid(unsafe_code)]
#![deny(missing_docs, clippy::trivially_copy_pass_by_ref)]
#![warn(rustdoc::broken_intra_doc_links)]
#![allow(
    clippy::unreadable_literal,
    clippy::many_single_char_names,
    clippy::excessive_precision
)]

mod affine;
mod arc;
mod bezpath;
mod circle;
pub mod common;
mod cubicbez;
pub mod curve_intersections;
mod ellipse;
mod insets;
mod line;
mod mindist;
mod param_curve;
mod point;
mod quadbez;
mod quadspline;
mod rect;
mod rounded_rect;
mod rounded_rect_radii;
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
pub use crate::curve_intersections::curve_curve_intersections;
pub use crate::ellipse::*;
pub use crate::insets::*;
pub use crate::line::*;
pub use crate::param_curve::*;
pub use crate::point::*;
pub use crate::quadbez::*;
pub use crate::quadspline::*;
pub use crate::rect::*;
pub use crate::rounded_rect::*;
pub use crate::rounded_rect_radii::*;
pub use crate::shape::*;
pub use crate::size::*;
pub use crate::svg::*;
pub use crate::translate_scale::*;
pub use crate::vec2::*;
