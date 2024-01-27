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
pub enum StaticShape<External = _never_shape::NeverShape>
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
    Rect(crate::Rect),
    /// Corresponds to a `RoundedRect`
    RoundedRect(crate::RoundedRect),
    /// A type implementing shape that may be defined by an external crate.
    External(External),
}

macro_rules! from_shape {
    ($it: ident) => {
        impl From<crate::$it> for StaticShape {
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

impl StaticShape {
    /// Builds a static shape from an external shape implementation.
    ///
    /// For a kurbo provided shape implementation, you would normally use the `from` impl instead.
    pub fn from_external_shape<External>(shape: External) -> StaticShape<External>
    where
        External: Shape,
    {
        StaticShape::External(shape)
    }
}

impl<External> Shape for StaticShape<External>
where
    External: Shape,
{
    type PathElementsIter<'iter> = Box<dyn Iterator<Item = crate::PathEl> + 'iter> where External: 'iter;
    fn path_elements(&self, tol: f64) -> Box<dyn Iterator<Item = crate::PathEl> + '_> {
        use StaticShape as S;
        match self {
            S::PathSeg(it) => Box::new(it.path_elements(tol)),
            S::Arc(it) => Box::new(it.path_elements(tol)),
            S::BezPath(it) => Box::new(it.path_elements(tol)),
            S::Circle(it) => Box::new(it.path_elements(tol)),
            S::CircleSegment(it) => Box::new(it.path_elements(tol)),
            S::CubicBez(it) => Box::new(it.path_elements(tol)),
            S::Ellipse(it) => Box::new(it.path_elements(tol)),
            S::Line(it) => Box::new(it.path_elements(tol)),
            S::QuadBez(it) => Box::new(it.path_elements(tol)),
            S::Rect(it) => Box::new(it.path_elements(tol)),
            S::RoundedRect(it) => Box::new(it.path_elements(tol)),
            S::External(it) => Box::new(it.path_elements(tol)),
        }
    }

    fn perimeter(&self, acc: f64) -> f64 {
        use StaticShape as S;
        match self {
            S::PathSeg(it) => it.perimeter(acc),
            S::Arc(it) => it.perimeter(acc),
            S::BezPath(it) => it.perimeter(acc),
            S::Circle(it) => it.perimeter(acc),
            S::CircleSegment(it) => it.perimeter(acc),
            S::CubicBez(it) => it.perimeter(acc),
            S::Ellipse(it) => it.perimeter(acc),
            S::Line(it) => it.perimeter(acc),
            S::QuadBez(it) => it.perimeter(acc),
            S::Rect(it) => it.perimeter(acc),
            S::RoundedRect(it) => it.perimeter(acc),
            S::External(it) => it.perimeter(acc),
        }
    }

    fn area(&self) -> f64 {
        use StaticShape as S;
        match self {
            S::PathSeg(it) => it.area(),
            S::Arc(it) => it.area(),
            S::BezPath(it) => it.area(),
            S::Circle(it) => it.area(),
            S::CircleSegment(it) => it.area(),
            S::CubicBez(it) => it.area(),
            S::Ellipse(it) => it.area(),
            S::Line(it) => it.area(),
            S::QuadBez(it) => it.area(),
            S::Rect(it) => it.area(),
            S::RoundedRect(it) => it.area(),
            S::External(it) => it.area(),
        }
    }

    fn winding(&self, pt: Point) -> i32 {
        use StaticShape as S;
        match self {
            S::PathSeg(it) => it.winding(pt),
            S::Arc(it) => it.winding(pt),
            S::BezPath(it) => it.winding(pt),
            S::Circle(it) => it.winding(pt),
            S::CircleSegment(it) => it.winding(pt),
            S::CubicBez(it) => it.winding(pt),
            S::Ellipse(it) => it.winding(pt),
            S::Line(it) => it.winding(pt),
            S::QuadBez(it) => it.winding(pt),
            S::Rect(it) => it.winding(pt),
            S::RoundedRect(it) => it.winding(pt),
            S::External(it) => it.winding(pt),
        }
    }

    fn bounding_box(&self) -> Rect {
        use StaticShape as S;
        match self {
            S::PathSeg(it) => it.bounding_box(),
            S::Arc(it) => it.bounding_box(),
            S::BezPath(it) => it.bounding_box(),
            S::Circle(it) => it.bounding_box(),

            S::CircleSegment(it) => it.bounding_box(),
            S::CubicBez(it) => it.bounding_box(),
            S::Ellipse(it) => it.bounding_box(),
            S::Line(it) => it.bounding_box(),
            S::QuadBez(it) => it.bounding_box(),
            S::Rect(it) => it.bounding_box(),
            S::RoundedRect(it) => it.bounding_box(),
            S::External(it) => it.bounding_box(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_collection() {
        let r = crate::Rect::from_origin_size((0.0, 0.0), (1.0, 1.0));
        let l = crate::Line::new((0.0, 0.0), (0.5, 0.5));
        let shapes: Vec<StaticShape> = vec![r.into(), l.into()];
        assert_eq!(shapes, vec![StaticShape::Rect(r), StaticShape::Line(l),])
    }
    #[test]
    fn test_external() {
        let l = crate::Line::new((0.0, 0.0), (0.5, 0.5));
        assert_eq!(
            StaticShape::from_external_shape(l),
            StaticShape::External(l)
        );
    }
}
