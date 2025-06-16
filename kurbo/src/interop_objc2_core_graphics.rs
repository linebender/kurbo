// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use objc2_core_foundation::{CGPoint, CGRect, CGSize, CGVector};
use objc2_core_graphics::{CGLineCap, CGLineJoin};

use crate::{Cap, Join, Point, Rect, Size, Vec2};

impl From<CGLineCap> for Cap {
    fn from(cap: CGLineCap) -> Self {
        match cap {
            CGLineCap::Butt => Cap::Butt,
            CGLineCap::Square => Cap::Square,
            CGLineCap::Round => Cap::Round,
            CGLineCap(i32::MIN..=-1_i32) | CGLineCap(3_i32..=i32::MAX) => {
                panic!("Unsupported line cap {}.", cap.0)
            }
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

impl From<CGLineJoin> for Join {
    fn from(join: CGLineJoin) -> Self {
        match join {
            CGLineJoin::Miter => Join::Miter,
            CGLineJoin::Round => Join::Round,
            CGLineJoin::Bevel => Join::Bevel,
            CGLineJoin(i32::MIN..=-1_i32) | CGLineJoin(3_i32..=i32::MAX) => {
                panic!("Unsupported line join {}.", join.0)
            }
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
