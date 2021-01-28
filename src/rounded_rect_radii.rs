// Copyright 2021 The kurbo Authors.
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

//! A description of the radii for each corner of a rounded rectangle.

use std::convert::From;

/// Radii for each corner of a rounded rectangle.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoundedRectRadii {
    /// The radius of the top-left corner.
    pub top_left: f64,
    /// The radius of the top-right corner.
    pub top_right: f64,
    /// The radius of the bottom-right corner.
    pub bottom_right: f64,
    /// The radius of the bottom-left corner.
    pub bottom_left: f64,
}

impl RoundedRectRadii {
    /// Create a new RoundedRectRadii. This function takes radius values for
    /// the four corners. The argument order is "top_left, top_right,
    /// bottom_right, bottom_left", or clockwise starting from top_left.
    pub const fn new(top_left: f64, top_right: f64, bottom_right: f64, bottom_left: f64) -> Self {
        RoundedRectRadii {
            top_left,
            top_right,
            bottom_right,
            bottom_left,
        }
    }

    /// Create a new RoundedRectRadii from a single radius. The `radius`
    /// argument will be set as the radius for all four corners.
    pub const fn from_single_radius(radius: f64) -> Self {
        RoundedRectRadii {
            top_left: radius,
            top_right: radius,
            bottom_right: radius,
            bottom_left: radius,
        }
    }
}

impl From<f64> for RoundedRectRadii {
    fn from(radius: f64) -> Self {
        RoundedRectRadii::from_single_radius(radius)
    }
}

impl From<(f64, f64, f64, f64)> for RoundedRectRadii {
    fn from(radii: (f64, f64, f64, f64)) -> Self {
        RoundedRectRadii::new(radii.0, radii.1, radii.2, radii.3)
    }
}
