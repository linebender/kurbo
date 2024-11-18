// Copyright 2024 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::{
    Arc, BezPath, Circle, CircleSegment, CubicBez, Ellipse, Line, PathSeg, Point, QuadBez, Rect,
    RoundedRect, Shape, Triangle,
};

/// An enum type with variants matching the shape types this crate exports.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum ConcreteShape {
    /// An [`Arc`] instance.
    Arc(Arc),
    /// A [`BezPath`] instance.
    BezPath(BezPath),
    /// A [`Circle`] instance.
    Circle(Circle),
    /// A [`CircleSegment`] instance.
    CircleSegment(CircleSegment),
    /// A [`CubicBez`] instance.
    CubicBez(CubicBez),
    /// An [`Ellipse`] instance.
    Ellipse(Ellipse),
    /// A [`Line`] instance.
    Line(Line),
    /// A [`PathSeg`] instance.
    PathSeg(PathSeg),
    /// A [`QuadBez`] instance.
    QuadBez(QuadBez),
    /// A [`Rect`] instance.
    Rect(Rect),
    /// A [`RoundedRect`] instance.
    RoundedRect(RoundedRect),
    /// A [`Triangle`] instance.
    Triangle(Triangle),
}

macro_rules! from_impl {
    ($Shape: ident) => {
        impl From<$Shape> for ConcreteShape {
            fn from(value: $Shape) -> Self {
                Self::$Shape(value)
            }
        }
    };
}

from_impl!(Arc);
from_impl!(BezPath);
from_impl!(Circle);
from_impl!(CircleSegment);
from_impl!(CubicBez);
from_impl!(Ellipse);
from_impl!(Line);
from_impl!(PathSeg);
from_impl!(QuadBez);
from_impl!(Rect);
from_impl!(RoundedRect);
from_impl!(Triangle);

macro_rules! match_shape {
    ($x:ident, $i:ident, $e: expr) => {
        match $x {
            ConcreteShape::PathSeg($i) => $e,
            ConcreteShape::Arc($i) => $e,
            ConcreteShape::BezPath($i) => $e,
            ConcreteShape::Circle($i) => $e,
            ConcreteShape::CircleSegment($i) => $e,
            ConcreteShape::CubicBez($i) => $e,
            ConcreteShape::Ellipse($i) => $e,
            ConcreteShape::Line($i) => $e,
            ConcreteShape::QuadBez($i) => $e,
            ConcreteShape::Rect($i) => $e,
            ConcreteShape::RoundedRect($i) => $e,
            ConcreteShape::Triangle($i) => $e,
        }
    };
}

impl Shape for ConcreteShape {
    type PathElementsIter<'iter> = Box<dyn Iterator<Item = crate::PathEl> + 'iter>;

    fn path_elements(&self, tol: f64) -> Box<dyn Iterator<Item = crate::PathEl> + '_> {
        match_shape!(self, it, Box::new(it.path_elements(tol)))
    }

    fn perimeter(&self, acc: f64) -> f64 {
        match_shape!(self, it, it.perimeter(acc))
    }

    fn area(&self) -> f64 {
        match_shape!(self, it, it.area())
    }

    fn winding(&self, pt: Point) -> i32 {
        match_shape!(self, it, it.winding(pt))
    }

    fn bounding_box(&self) -> Rect {
        match_shape!(self, it, it.bounding_box())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_collection() {
        let r = Rect::from_origin_size((0.0, 0.0), (1.0, 1.0));
        let l = Line::new((0.0, 0.0), (0.5, 0.5));
        let shapes: Vec<ConcreteShape> = vec![r.into(), l.into()];

        assert_eq!(
            shapes,
            vec![ConcreteShape::Rect(r), ConcreteShape::Line(l),]
        );
    }
}
