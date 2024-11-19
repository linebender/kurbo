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
            Self::PathSeg($i) => $e,
            Self::Arc($i) => $e,
            Self::BezPath($i) => $e,
            Self::Circle($i) => $e,
            Self::CircleSegment($i) => $e,
            Self::CubicBez($i) => $e,
            Self::Ellipse($i) => $e,
            Self::Line($i) => $e,
            Self::QuadBez($i) => $e,
            Self::Rect($i) => $e,
            Self::RoundedRect($i) => $e,
            Self::Triangle($i) => $e,
        }
    };
}

impl Shape for ConcreteShape {
    type PathElementsIter<'iter> = PathElementsIter<'iter>;

    fn path_elements(&self, tol: f64) -> PathElementsIter<'_> {
        match self {
            Self::PathSeg(it) => PathElementsIter::PathSeg(it.path_elements(tol)),
            Self::Arc(it) => PathElementsIter::Arc(it.path_elements(tol)),
            Self::BezPath(it) => PathElementsIter::BezPath(it.path_elements(tol)),
            Self::Circle(it) => PathElementsIter::Circle(it.path_elements(tol)),
            Self::CircleSegment(it) => PathElementsIter::CircleSegment(it.path_elements(tol)),
            Self::CubicBez(it) => PathElementsIter::CubicBez(it.path_elements(tol)),
            Self::Ellipse(it) => PathElementsIter::Ellipse(it.path_elements(tol)),
            Self::Line(it) => PathElementsIter::Line(it.path_elements(tol)),
            Self::QuadBez(it) => PathElementsIter::QuadBez(it.path_elements(tol)),
            Self::Rect(it) => PathElementsIter::Rect(it.path_elements(tol)),
            Self::RoundedRect(it) => PathElementsIter::RoundedRect(it.path_elements(tol)),
            Self::Triangle(it) => PathElementsIter::Triangle(it.path_elements(tol)),
        }
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

pub enum PathElementsIter<'i> {
    PathSeg(<PathSeg as Shape>::PathElementsIter<'i>),
    Arc(<Arc as Shape>::PathElementsIter<'i>),
    BezPath(<BezPath as Shape>::PathElementsIter<'i>),
    Circle(<Circle as Shape>::PathElementsIter<'i>),
    CircleSegment(<CircleSegment as Shape>::PathElementsIter<'i>),
    CubicBez(<CubicBez as Shape>::PathElementsIter<'i>),
    Ellipse(<Ellipse as Shape>::PathElementsIter<'i>),
    Line(<Line as Shape>::PathElementsIter<'i>),
    QuadBez(<QuadBez as Shape>::PathElementsIter<'i>),
    Rect(<Rect as Shape>::PathElementsIter<'i>),
    RoundedRect(<RoundedRect as Shape>::PathElementsIter<'i>),
    Triangle(<Triangle as Shape>::PathElementsIter<'i>),
}

impl<'i> Iterator for PathElementsIter<'i> {
    type Item = crate::PathEl;

    fn next(&mut self) -> Option<Self::Item> {
        match_shape!(self, i, i.next())
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
