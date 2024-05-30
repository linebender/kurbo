// Copyright 2024 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

#![allow(unused_qualifications)]

use crate::{Point, Rect, Shape};
use alloc::boxed::Box;

mod _never_shape {
    use super::*;
    use crate::PathEl;
    /// An uninhabited type that implements shape.
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub enum NeverShape {}
    impl Shape for NeverShape {
        type PathElementsIter<'a> = core::iter::Empty<PathEl>;
        fn path_elements(&self, _: f64) -> Self::PathElementsIter<'_> {
            unreachable!()
        }
        fn area(&self) -> f64 {
            unreachable!()
        }
        fn perimeter(&self, _: f64) -> f64 {
            unreachable!()
        }
        fn winding(&self, _: Point) -> i32 {
            unreachable!()
        }
        fn bounding_box(&self) -> Rect {
            unreachable!()
        }
    }
}

/// Because the `Shape` trait is not dyn safe, it can be difficult to store
/// Collections of `Shape` items in hetereogenuous collections.
///
/// It allows an external `Shape` impl to be provided as an extension point
/// for shape impls provided by external crates.  This defaults to an
/// uninhabited type.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum PrimitiveShape<External = _never_shape::NeverShape>
where
    External: Shape,
{
    /// Corresponds to a `PathSeg`
    PathSeg(crate::PathSeg),
    /// Corresponds to an `Arc`
    Arc(crate::Arc),
    /// Corresponds to a `BezPath`
    BezPath(crate::BezPath),
    /// Corresponds to a `Circle`
    Circle(crate::Circle),
    /// Corresponds to a `CircleSegment`
    CircleSegment(crate::CircleSegment),
    /// Corresponds to a `CubicBez`
    CubicBez(crate::CubicBez),
    /// Corresponds to an `Ellipse`
    Ellipse(crate::Ellipse),
    /// Corresponds to a `Line`
    Line(crate::Line),
    /// Corresponds to a `QuadBez`
    QuadBez(crate::QuadBez),
    /// Corresponds to a `Rect`
    Rect(Rect),
    /// Corresponds to a `RoundedRect`
    RoundedRect(crate::RoundedRect),
    /// A type implementing shape that may be defined by an external crate.
    External(External),
}

macro_rules! from_shape {
    ($it: ident) => {
        impl From<crate::$it> for PrimitiveShape {
            fn from(it: crate::$it) -> Self {
                Self::$it(it)
            }
        }
    };
}

from_shape!(PathSeg);
from_shape!(Arc);
from_shape!(BezPath);
from_shape!(Circle);
from_shape!(CircleSegment);
from_shape!(CubicBez);
from_shape!(Ellipse);
from_shape!(Line);
from_shape!(QuadBez);
from_shape!(Rect);
from_shape!(RoundedRect);

impl PrimitiveShape {
    /// Builds a static shape from an external shape implementation.
    ///
    /// For a kurbo provided shape implementation, you would normally use the `from` impl instead.
    pub fn from_external_shape<External>(shape: External) -> PrimitiveShape<External>
    where
        External: Shape,
    {
        PrimitiveShape::External(shape)
    }
}

macro_rules! match_shape {
    ($x:ident, $it:ident, $e: expr) => {
        match $x {
            PrimitiveShape::PathSeg($it) => $e,
            PrimitiveShape::Arc($it) => $e,
            PrimitiveShape::BezPath($it) => $e,
            PrimitiveShape::Circle($it) => $e,
            PrimitiveShape::CircleSegment($it) => $e,
            PrimitiveShape::CubicBez($it) => $e,
            PrimitiveShape::Ellipse($it) => $e,
            PrimitiveShape::Line($it) => $e,
            PrimitiveShape::QuadBez($it) => $e,
            PrimitiveShape::Rect($it) => $e,
            PrimitiveShape::RoundedRect($it) => $e,
            PrimitiveShape::External($it) => $e,
        }
    };
}

impl<External> Shape for PrimitiveShape<External>
where
    External: Shape,
{
    type PathElementsIter<'iter> = Box<dyn Iterator<Item = crate::PathEl> + 'iter> where External: 'iter;
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
        let r = crate::Rect::from_origin_size((0.0, 0.0), (1.0, 1.0));
        let l = crate::Line::new((0.0, 0.0), (0.5, 0.5));
        let shapes: Vec<PrimitiveShape> = vec![r.into(), l.into()];
        assert_eq!(
            shapes,
            vec![PrimitiveShape::Rect(r), PrimitiveShape::Line(l),]
        );
    }
    #[test]
    fn test_external() {
        let l = crate::Line::new((0.0, 0.0), (0.5, 0.5));
        assert_eq!(
            PrimitiveShape::from_external_shape(l),
            PrimitiveShape::External(l)
        );
    }
}
