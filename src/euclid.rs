//! Conversions and helpers for working with the [Euclid crate][].
//!
//!
//! - [Euclid crate]: https://docs.rs/euclid/

use euclid_crate as euclid;

// Rect

impl<T, U> From<euclid::TypedRect<T, U>> for crate::Rect
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: euclid::TypedRect<T, U>) -> crate::Rect {
        crate::Rect::new(
            src.min_x().as_(),
            src.min_y().as_(),
            src.max_x().as_(),
            src.max_y().as_(),
        )
    }
}

impl<T, U> From<&euclid::TypedRect<T, U>> for crate::Rect
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: &euclid::TypedRect<T, U>) -> crate::Rect {
        crate::Rect::new(
            src.min_x().as_(),
            src.min_y().as_(),
            src.max_x().as_(),
            src.max_y().as_(),
        )
    }
}

// Point2D

impl<T, U> From<euclid::TypedPoint2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: euclid::TypedPoint2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.x.as_(), src.y.as_())
    }
}

impl<T, U> From<&euclid::TypedPoint2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: &euclid::TypedPoint2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.x.as_(), src.y.as_())
    }
}

//Size2D

impl<T, U> From<euclid::TypedSize2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: euclid::TypedSize2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.width.as_(), src.height.as_())
    }
}

impl<T, U> From<&euclid::TypedSize2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: &euclid::TypedSize2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.width.as_(), src.height.as_())
    }
}

// Vector2D

impl<T, U> From<euclid::TypedVector2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: euclid::TypedVector2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.x.as_(), src.y.as_())
    }
}

impl<T, U> From<&euclid::TypedVector2D<T, U>> for crate::Vec2
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    fn from(src: &euclid::TypedVector2D<T, U>) -> crate::Vec2 {
        crate::Vec2::new(src.x.as_(), src.y.as_())
    }
}

impl<T, U> crate::Shape for euclid::TypedRect<T, U>
where
    T: num_traits::Float + num_traits::AsPrimitive<f64>,
{
    type BezPathIter = crate::rect::RectPathIter;

    fn to_bez_path(&self, _tolerance: f64) -> crate::rect::RectPathIter {
        let rect = *self;
        crate::rect::RectPathIter {
            rect: rect.into(),
            ix: 0,
        }
    }

    // It's a bit of duplication having both this and the impl method, but
    // removing that would require using the trait. We'll leave it for now.
    #[inline]
    fn area(&self) -> f64 {
        self.area().as_()
    }

    #[inline]
    fn perimeter(&self, _accuracy: f64) -> f64 {
        2.0 * (self.size.width.abs() + self.size.height.abs()).as_()
    }

    //FIXME: @raph, is this logic right? I suspect I'm missing a subtlety
    #[inline]
    fn winding(&self, pt: crate::Vec2) -> i32 {
        let xmin = self.min_x().as_();
        let xmax = self.max_x().as_();
        let ymin = self.min_y().as_();
        let ymax = self.max_y().as_();
        if pt.x >= xmin && pt.x < xmax && pt.y >= ymin && pt.y < ymax {
            if (self.size.width.as_() > 0.) ^ (self.size.height.as_() > 0.) {
                -1
            } else {
                1
            }
        } else {
            0
        }
    }

    #[inline]
    fn bounding_box(&self) -> crate::Rect {
        let rect: crate::Rect = self.into();
        rect.abs()
    }

    #[inline]
    fn as_rect(&self) -> Option<crate::Rect> {
        let rect: crate::Rect = self.into();
        Some(rect)
    }
}
