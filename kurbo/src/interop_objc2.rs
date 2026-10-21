// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Interoperability with Apple graphics types from `objc2` crates.
//!
//! `objc2-foundation` defines `NSPoint`, `NSSize`, and `NSRect` as aliases of
//! `CGPoint`, `CGSize`, and `CGRect`, so these conversions also cover the
//! AppKit geometry names without depending on AppKit.

use objc2_core_foundation::{CGAffineTransform, CGPoint, CGRect, CGSize, CGVector};
use objc2_core_graphics::{CGLineCap, CGLineJoin};

use crate::{Affine, Cap, Join, Point, Rect, Size, Vec2};

impl TryFrom<CGLineCap> for Cap {
    type Error = CGLineCap;

    fn try_from(cap: CGLineCap) -> Result<Self, Self::Error> {
        match cap {
            CGLineCap::Butt => Ok(Cap::Butt),
            CGLineCap::Square => Ok(Cap::Square),
            CGLineCap::Round => Ok(Cap::Round),
            _ => Err(cap),
        }
    }
}

impl From<Cap> for CGLineCap {
    fn from(cap: Cap) -> Self {
        match cap {
            Cap::Butt => CGLineCap::Butt,
            Cap::Round => CGLineCap::Round,
            Cap::Square => CGLineCap::Square,
        }
    }
}

impl TryFrom<CGLineJoin> for Join {
    type Error = CGLineJoin;

    fn try_from(join: CGLineJoin) -> Result<Self, Self::Error> {
        match join {
            CGLineJoin::Miter => Ok(Join::Miter),
            CGLineJoin::Round => Ok(Join::Round),
            CGLineJoin::Bevel => Ok(Join::Bevel),
            _ => Err(join),
        }
    }
}

impl From<Join> for CGLineJoin {
    fn from(join: Join) -> Self {
        match join {
            Join::Bevel => CGLineJoin::Bevel,
            Join::Miter => CGLineJoin::Miter,
            Join::Round => CGLineJoin::Round,
        }
    }
}

impl From<CGPoint> for Point {
    #[inline]
    fn from(point: CGPoint) -> Self {
        Point::new(point.x, point.y)
    }
}

impl From<Point> for CGPoint {
    #[inline]
    fn from(point: Point) -> Self {
        CGPoint::new(point.x, point.y)
    }
}

impl From<CGRect> for Rect {
    #[inline]
    fn from(rect: CGRect) -> Self {
        Rect::from_origin_size(rect.origin, rect.size)
    }
}

impl From<Rect> for CGRect {
    #[inline]
    fn from(rect: Rect) -> Self {
        CGRect::new(
            CGPoint::new(rect.x0, rect.y0),
            CGSize::new(rect.width(), rect.height()),
        )
    }
}

impl From<CGSize> for Size {
    #[inline]
    fn from(size: CGSize) -> Self {
        Size::new(size.width, size.height)
    }
}

impl From<Size> for CGSize {
    #[inline]
    fn from(size: Size) -> Self {
        CGSize::new(size.width, size.height)
    }
}

impl From<CGVector> for Vec2 {
    #[inline]
    fn from(vector: CGVector) -> Self {
        Vec2::new(vector.dx, vector.dy)
    }
}

impl From<Vec2> for CGVector {
    #[inline]
    fn from(vector: Vec2) -> Self {
        CGVector::new(vector.x, vector.y)
    }
}

impl From<CGAffineTransform> for Affine {
    #[inline]
    fn from(transform: CGAffineTransform) -> Self {
        Affine::new([
            transform.a,
            transform.b,
            transform.c,
            transform.d,
            transform.tx,
            transform.ty,
        ])
    }
}

impl From<Affine> for CGAffineTransform {
    #[inline]
    fn from(transform: Affine) -> Self {
        let [a, b, c, d, tx, ty] = transform.as_coeffs();
        CGAffineTransform { a, b, c, d, tx, ty }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_cap_conversion() {
        assert_eq!(Cap::try_from(CGLineCap::Butt), Ok(Cap::Butt));
        assert_eq!(Cap::try_from(CGLineCap::Round), Ok(Cap::Round));
        assert_eq!(Cap::try_from(CGLineCap::Square), Ok(Cap::Square));
        assert_eq!(Cap::try_from(CGLineCap(-1)), Err(CGLineCap(-1)));
        assert_eq!(Cap::try_from(CGLineCap(3)), Err(CGLineCap(3)));

        assert_eq!(CGLineCap::from(Cap::Butt), CGLineCap::Butt);
        assert_eq!(CGLineCap::from(Cap::Round), CGLineCap::Round);
        assert_eq!(CGLineCap::from(Cap::Square), CGLineCap::Square);
    }

    #[test]
    fn line_join_conversion() {
        assert_eq!(Join::try_from(CGLineJoin::Miter), Ok(Join::Miter));
        assert_eq!(Join::try_from(CGLineJoin::Round), Ok(Join::Round));
        assert_eq!(Join::try_from(CGLineJoin::Bevel), Ok(Join::Bevel));
        assert_eq!(Join::try_from(CGLineJoin(-1)), Err(CGLineJoin(-1)));
        assert_eq!(Join::try_from(CGLineJoin(3)), Err(CGLineJoin(3)));

        assert_eq!(CGLineJoin::from(Join::Miter), CGLineJoin::Miter);
        assert_eq!(CGLineJoin::from(Join::Round), CGLineJoin::Round);
        assert_eq!(CGLineJoin::from(Join::Bevel), CGLineJoin::Bevel);
    }

    #[test]
    fn point_conversion() {
        let point = Point::new(1.25, -2.5);
        let cg_point = CGPoint::new(1.25, -2.5);

        assert_eq!(CGPoint::from(point), cg_point);
        assert_eq!(Point::from(cg_point), point);
    }

    #[test]
    fn rect_conversion() {
        let rect = Rect::new(1.0, 2.0, 4.0, 6.0);
        let cg_rect = CGRect::new(CGPoint::new(1.0, 2.0), CGSize::new(3.0, 4.0));

        assert_eq!(CGRect::from(rect), cg_rect);
        assert_eq!(Rect::from(cg_rect), rect);
    }

    #[test]
    fn size_conversion() {
        let size = Size::new(7.0, 9.0);
        let cg_size = CGSize::new(7.0, 9.0);

        assert_eq!(CGSize::from(size), cg_size);
        assert_eq!(Size::from(cg_size), size);
    }

    #[test]
    fn vector_conversion() {
        let vector = Vec2::new(-3.0, 8.0);
        let cg_vector = CGVector::new(-3.0, 8.0);

        assert_eq!(CGVector::from(vector), cg_vector);
        assert_eq!(Vec2::from(cg_vector), vector);
    }

    #[test]
    fn affine_conversion() {
        let affine = Affine::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let cg_affine = CGAffineTransform {
            a: 1.0,
            b: 2.0,
            c: 3.0,
            d: 4.0,
            tx: 5.0,
            ty: 6.0,
        };

        assert_eq!(CGAffineTransform::from(affine), cg_affine);
        assert_eq!(Affine::from(cg_affine), affine);
    }
}
