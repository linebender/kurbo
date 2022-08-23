// DO NOT COMMIT THIS FILE

use std::{fmt};
use std::any::Any;

use raqote::*;
use crate::{
    Affine, CubicBez, QuadBez, Shape, ParamCurve, Point, Vec2, Size, Rect,
    Line, FatLine
};
use arrayvec::ArrayVec;

/*****************************************************************************
 * Struct
 *****************************************************************************/

/// A struct that produces an image with the contained items drawn on it
pub struct Image
{
    /// The image scale
    scale: f64,
    /// The offset between the top-left corner of the item bounds and the image origin
    offset: Vec2,
    /// The white space around the edge of the image in pixels
    margins: i32,
    /// The fixed size of the image
    fixed_size: Size,
    /// Whether or not the image should use the fixed_size field (or the image scale)
    use_fixed_size: bool,
    /// The items to draw to the image
    items: Vec<Box<dyn Drawable>>,
}

/*****************************************************************************
 * Enums
 *****************************************************************************/

 #[derive(Debug)]
enum CurveType {
    CurveNone,
    CurveCubic,
    CurveQuad,
    CurveLinear,
}

fn get_curve_type_from_curve_param(c1: &dyn Any) -> CurveType
{
    if c1.is::<CubicBez>() { return CurveType::CurveCubic }
    if c1.is::<QuadBez>() { return CurveType::CurveQuad }
    if c1.is::<Line>() { return CurveType::CurveLinear }
    CurveType::CurveNone
}

/*****************************************************************************
 * Trait - Drawable Element
 *****************************************************************************/

trait Drawable
{
    fn bounds(&self) -> Rect;
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget);
}

struct DrawableCubicBez(pub CubicBez, pub u32, pub bool);
impl Drawable for DrawableCubicBez {
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        let bbox = if self.2 { Rect::ZERO } else { self.0.bounding_box() };
        // let bbox = bbox.union_pt(self.0.p1);
        // let bbox = bbox.union_pt(self.0.p2);
        bbox
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        if self.2
        {
            image.draw_cubic_bez_with_opacity(dt, &self.0, self.1, 0x40);
        }
        else
        {
            image.draw_cubic_bez(dt, &self.0, self.1);
            image.draw_point(dt, &self.0.p0, 5., self.1, true);
            image.draw_point_with_opacity(dt, &self.0.p1, 5., self.1, 0x80, true);
            // image.draw_dashed_line_with_opacity(dt, &Line::new(self.0.p0, self.0.p1), self.1, 0x40);
            image.draw_line_with_opacity(dt, &Line::new(self.0.p0, self.0.p1), self.1, 0x40);
            image.draw_point_with_opacity(dt, &self.0.p2, 5., self.1, 0x80, true);
            image.draw_point(dt, &self.0.p3, 5., self.1, true);
            // image.draw_dashed_line_with_opacity(dt, &Line::new(self.0.p3, self.0.p2), self.1, 0x40);
            image.draw_line_with_opacity(dt, &Line::new(self.0.p3, self.0.p2), self.1, 0x40);
        }
    }
}

struct DrawableQuadBezier(pub QuadBez, pub u32, pub bool);
impl Drawable for DrawableQuadBezier {
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        let bbox = if self.2 { Rect::ZERO } else { self.0.bounding_box() };
        // let bbox = bbox.union_pt(self.0.p1);
        bbox
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        if self.2
        {
            image.draw_quad_bez_with_opacity(dt, &self.0, self.1, 0x40);
        }
        else
        {
            image.draw_quad_bez(dt, &self.0, self.1);
            image.draw_point(dt, &self.0.p0, 5., self.1, true);
            image.draw_point_with_opacity(dt, &self.0.p1, 5., self.1, 0x80, true);
            image.draw_point(dt, &self.0.p2, 5., self.1, true);
            // image.draw_dashed_line_with_opacity(dt, &Line::new(self.0.p0, self.0.p1), self.1, 0x40);
            // image.draw_dashed_line_with_opacity(dt, &Line::new(self.0.p2, self.0.p1), self.1, 0x40);
            image.draw_line_with_opacity(dt, &Line::new(self.0.p0, self.0.p1), self.1, 0x40);
            image.draw_line_with_opacity(dt, &Line::new(self.0.p2, self.0.p1), self.1, 0x40);
        }
    }
}

struct DrawableLine(pub Line, pub u32, pub bool);
impl Drawable for DrawableLine {
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        let bbox = if self.2 { Rect::ZERO } else { self.0.bounding_box() };
        bbox
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        if self.2
        {
            image.draw_line_with_opacity(dt, &self.0, self.1, 0x40);
        }
        else
        {
            image.draw_line(dt, &self.0, self.1);
            image.draw_point(dt, &self.0.p0, 5., self.1, true);
            image.draw_point(dt, &self.0.p1, 5., self.1, true);
        }
    }
}

struct DrawableFatLine(pub FatLine, pub u32);
impl Drawable for DrawableFatLine {
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        // Use empty rect so that it's not used in calculating total bounds
        // Note: Fat lines are technically infinite
        Rect::ZERO
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        image.draw_fat_line(dt, &self.0, self.1);
    }
}

struct DrawablePoint(pub Point, pub u32, pub f32, pub bool);
impl Drawable for DrawablePoint
{
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        // Use empty rect so that it's not used in calculating total bounds
        Rect::ZERO
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        image.draw_point(dt, &self.0, self.2, self.1, self.3);
    }
}

struct DrawableIntersections<T: ParamCurve, U: ParamCurve>(
    pub ArrayVec<(f64, f64), 9>,
    pub T,
    pub U,
    pub u32,
    pub f32,
    pub bool
    );
impl <T: ParamCurve, U: ParamCurve> Drawable for DrawableIntersections<T, U> where
    T: ParamCurve,
    U: ParamCurve
{
    #[inline(never)]
    fn bounds(&self) -> Rect
    {
        // Use empty rect so that it's not used in calculating total bounds
        Rect::ZERO
    }
    
    #[inline(never)]
    fn draw_to_image(&self, image: &Image, dt: &mut DrawTarget)
    {
        for &i in &self.0
        {
            image.draw_point(dt, &self.1.eval(i.0), self.4, self.3, self.5);
            image.draw_point(dt, &self.2.eval(i.1), self.4 + 5., self.3, false);
        }
    }
}

/*****************************************************************************
 * Implementation
 *****************************************************************************/
 
impl Image
{
    /// Create an image that scales all items by a constant amount
    pub fn from_scale(scale_: f64, margins_: i32) -> Image
    {
        Image {
            scale: scale_,
            offset: Vec2::new(0., 0.),
            margins: margins_,
            fixed_size: Size::new(0., 0.),
            use_fixed_size: false,
            items: vec![],
            }
    }
    
    /// Create an image with a fixed size, all items are scaled to fit
    pub fn from_fixed_size(size_: Size, margins_: i32) -> Image
    {
        Image {
            scale: 1.0,
            offset: Vec2::new(0., 0.),
            margins: margins_,
            fixed_size: size_,
            use_fixed_size: true,
            items: vec![],
            }
    }
    
    /// Adds a cubic bezier to the list of items
    pub fn add_cubic_bez(&mut self, new_curve: &CubicBez, color: u32)
    {
        self.items.push(Box::new(DrawableCubicBez(new_curve.clone(), color, false)));
    }

    /// Adds a quadratic bezier to the list of items
    pub fn add_quad_bez(&mut self, new_curve: &QuadBez, color: u32)
    {
        self.items.push(Box::new(DrawableQuadBezier(new_curve.clone(), color, false)));
    }

    /// Adds a line to the list of items
    pub fn add_line(&mut self, new_curve: &Line, color: u32)
    {
        self.items.push(Box::new(DrawableLine(new_curve.clone(), color, false)));
    }

    /// Adds a path segment to the list of items if possible
    pub fn add_path_segment(&mut self, new_curve: &dyn Any, color: u32)
    {
        match get_curve_type_from_curve_param(new_curve) {
            CurveType::CurveCubic => { self.add_cubic_bez(new_curve.downcast_ref::<CubicBez>().unwrap(), color); },
            CurveType::CurveQuad => { self.add_quad_bez(new_curve.downcast_ref::<QuadBez>().unwrap(), color); },
            CurveType::CurveLinear => { self.add_line(new_curve.downcast_ref::<Line>().unwrap(), color); },
            _ => { println!("Cannot add invalid cubic bezier to image!"); return },
        }
    }

    /// Adds a phantom cubic bezier to the list of items
    pub fn add_phantom_cubic_bez(&mut self, new_curve: &CubicBez, color: u32)
    {
        self.items.push(Box::new(DrawableCubicBez(new_curve.clone(), color, true)));
    }

    /// Adds a phantom quadratic bezier to the list of items
    pub fn add_phantom_quad_bez(&mut self, new_curve: &QuadBez, color: u32)
    {
        self.items.push(Box::new(DrawableQuadBezier(new_curve.clone(), color, true)));
    }

    /// Adds a phantom line to the list of items
    pub fn add_phantom_line(&mut self, new_curve: &Line, color: u32)
    {
        self.items.push(Box::new(DrawableLine(new_curve.clone(), color, true)));
    }

    /// Adds a phantom path segment to the list of items if possible
    pub fn add_phantom_path_segment(&mut self, new_curve: &dyn Any, color: u32)
    {
        match get_curve_type_from_curve_param(new_curve) {
            CurveType::CurveCubic => { self.add_phantom_cubic_bez(new_curve.downcast_ref::<CubicBez>().unwrap(), color); },
            CurveType::CurveQuad => { self.add_phantom_quad_bez(new_curve.downcast_ref::<QuadBez>().unwrap(), color); },
            CurveType::CurveLinear => { self.add_phantom_line(new_curve.downcast_ref::<Line>().unwrap(), color); },
            _ => { println!("Cannot add invalid cubic bezier to image!"); return },
        }
    }
    
    /// Adds a fat line to the list of items
    pub fn add_fat_line(&mut self, fat_line: FatLine, color: u32)
    {
        self.items.push(Box::new(DrawableFatLine(fat_line, color)));
    }

    /// Adds a point to the list of items
    pub fn add_point(&mut self, pt: Point, color: u32, radius: f32, do_fill: bool)
    {
        self.items.push(Box::new(DrawablePoint(pt, color, radius, do_fill)));
    }

    /// Adds a set of interections to the list of items
    pub fn add_intersections<T: ParamCurve + 'static, U: ParamCurve + 'static>(
        &mut self,
        intersections: &ArrayVec<(f64, f64), 9>,
        c1: T,
        c2: U,
        color: u32,
        radius: f32,
        fill_first: bool)
    {
        self.items.push(Box::new(DrawableIntersections(intersections.clone(), c1, c2, color, radius, fill_first)));
    }

    /// Adds a set of interections to the list of items if possible
    pub fn add_intersections_cubic(
        &mut self,
        intersections: &ArrayVec<(f64, f64), 9>,
        c1: &dyn Any,
        c2: &dyn Any,
        color: u32,
        radius: f32,
        fill_first: bool)
    {
        let t1 = get_curve_type_from_curve_param(c1);
        let t2 = get_curve_type_from_curve_param(c2);

        match t1 {
            CurveType::CurveCubic => {
                match t2 {
                    CurveType::CurveCubic => { self.add_intersections(intersections, *c1.downcast_ref::<CubicBez>().unwrap(), *c2.downcast_ref::<CubicBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveQuad => { self.add_intersections(intersections, *c1.downcast_ref::<CubicBez>().unwrap(), *c2.downcast_ref::<QuadBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveLinear => { self.add_intersections(intersections, *c1.downcast_ref::<CubicBez>().unwrap(), *c2.downcast_ref::<Line>().unwrap(), color, radius, fill_first); },
                    _ => { println!("Cannot add invalid curve intersections to image!"); return },
                }
            }
            CurveType::CurveQuad => {
                match t2 {
                    CurveType::CurveCubic => { self.add_intersections(intersections, *c1.downcast_ref::<QuadBez>().unwrap(), *c2.downcast_ref::<CubicBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveQuad => { self.add_intersections(intersections, *c1.downcast_ref::<QuadBez>().unwrap(), *c2.downcast_ref::<QuadBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveLinear => { self.add_intersections(intersections, *c1.downcast_ref::<QuadBez>().unwrap(), *c2.downcast_ref::<Line>().unwrap(), color, radius, fill_first); },
                    _ => { println!("Cannot add invalid curve intersections to image!"); return },
                }
            }
            CurveType::CurveLinear => {
                match t2 {
                    CurveType::CurveCubic => { self.add_intersections(intersections, *c1.downcast_ref::<Line>().unwrap(), *c2.downcast_ref::<CubicBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveQuad => { self.add_intersections(intersections, *c1.downcast_ref::<Line>().unwrap(), *c2.downcast_ref::<QuadBez>().unwrap(), color, radius, fill_first); },
                    CurveType::CurveLinear => { self.add_intersections(intersections, *c1.downcast_ref::<Line>().unwrap(), *c2.downcast_ref::<Line>().unwrap(), color, radius, fill_first); },
                    _ => { println!("Cannot add invalid curve intersections to image!"); return },
                }
            }
            _ => { println!("Cannot add invalid curve intersections to image!"); return },
        }
    }
    
    /// Gets the desired image size from the bounding rect of the items
    fn image_size_from_bounds(&mut self, bounds: &Rect) -> Size
    {
        if self.use_fixed_size
        {
            let scale_x = (self.fixed_size.width - (self.margins as f64) * 2.) / bounds.width();
            let scale_y = (self.fixed_size.height - (self.margins as f64) * 2.) / bounds.height();
            self.scale = scale_x.min(scale_y);
            self.offset = Vec2::new(self.fixed_size.width, self.fixed_size.height) * 0.5 - bounds.center().to_vec2() * self.scale;
            return self.fixed_size;
        }
        else
        {
            self.offset = Vec2::new(self.margins as f64, self.margins as f64) - bounds.origin().to_vec2() * self.scale;
            return bounds.size() * self.scale + Size::new(self.margins as f64, self.margins as f64) * 2.;
        }
    }
    
    /// Creates a color that is lighter than the given color
    // fn lighter_color(color: u32) -> u32
    // {
    //     let r = (((color >> 16) as u8) as u32 + 0xFF) / 2;
    //     let g = (((color >> 8) as u8) as u32 + 0xFF) / 2;
    //     let b = (((color >> 0) as u8) as u32 + 0xFF) / 2;
    //     (r << 16) | (g << 8) | (b << 0)
    // }
    
    /// Creates a raqote source from the given color
    fn source_from_color(color: u32) -> Source<'static>
    {
        Source::Solid(
            SolidSource { 
                r: (color >> 16) as u8,
                g: (color >> 8) as u8,
                b: (color >> 0) as u8,
                a: 0xFF // full opacity
                }
            )
    }
    
    /// Creates a raqote source from the given color with a given opacity (0-255)
    fn source_from_color_with_opacity(color: u32, alpha: u8) -> Source<'static>
    {
        Source::Solid(
            SolidSource { 
                r: (color >> 16) as u8,
                g: (color >> 8) as u8,
                b: (color >> 0) as u8,
                a: alpha
                }
            )
    }
    
    /// Strokes the given path onto the image
    fn stroke_path_raw(dt: &mut DrawTarget, path: &Path, color: &Source<'static>)
    {
        dt.stroke(
            &path,
            &color,
            &StrokeStyle {
                cap: LineCap::Round,
                join: LineJoin::Round,
                width: 3.,
                miter_limit: 2.,
                dash_array: vec![],
                dash_offset: 0.,
                },
            &DrawOptions::new()
            );
    }

    /// Strokes the given path onto the image
    fn stroke_path(dt: &mut DrawTarget, path: &Path, color: u32)
    {
        Image::stroke_path_raw(dt, path, &Image::source_from_color(color));
    }

    /// Strokes the given path onto the image
    fn stroke_path_with_opacity(dt: &mut DrawTarget, path: &Path, color: u32, alpha: u8)
    {
        Image::stroke_path_raw(dt, path, &Image::source_from_color_with_opacity(color, alpha));
    }
    
    /// Strokes the given path onto the image with a dash pattern
    fn stroke_dashed_path_raw(dt: &mut DrawTarget, path: &Path, color: &Source<'static>)
    {
        dt.stroke(
            &path,
            &color,
            &StrokeStyle {
                cap: LineCap::Round,
                join: LineJoin::Round,
                width: 3.,
                miter_limit: 2.,
                dash_array: vec![10., 5.],
                dash_offset: 0.,
                },
            &DrawOptions::new()
            );
    }

    /// Strokes the given path onto the image with a dash pattern
    fn stroke_dashed_path(dt: &mut DrawTarget, path: &Path, color: u32)
    {
        Image::stroke_dashed_path_raw(dt, path, &Image::source_from_color(color));
    }

    /// Fills the given path onto the image
    fn fill_path_raw(dt: &mut DrawTarget, path: &Path, color: &Source<'static>)
    {
        dt.fill(
            &path,
            &color,
            &DrawOptions::new()
            );
    }
    
    /// Fills the given path onto the image
    // fn fill_path(dt: &mut DrawTarget, path: &Path, color: u32)
    // {
    //     Image::fill_path_raw(dt, path, &Image::source_from_color(color));
    // }
    
    /// Fills the given path onto the image with an opacity (0-255)
    fn fill_path_with_opacity(dt: &mut DrawTarget, path: &Path, color: u32, alpha: u8)
    {
        Image::fill_path_raw(dt, path, &Image::source_from_color_with_opacity(color, alpha));
    }
    
    /// Gets the affine transform necessary to draw an item onto the image
    fn get_affine_transform(&self) -> Affine
    {
        Affine::translate(self.offset) * Affine::scale(self.scale)
    }

    /// Gets the affine transform necessary to draw an item onto the image
    fn get_transformed_point(&self, pt: &Point) -> Point
    {
        self.get_affine_transform() * *pt
    }
    
    /// Draws a point onto the image
    fn draw_point_raw(&self, dt: &mut DrawTarget, pt: &Point, radius: f32, color: &Source<'static>, do_fill: bool)
    {
        let transformed_point = self.get_transformed_point(pt);
            
        let mut pb = PathBuilder::new();
        pb.arc(
            transformed_point.x as f32, transformed_point.y as f32,
            radius,
            0., (360.0_f32).to_radians()
            );
        let path = pb.finish();
        
        Image::stroke_path_raw(dt, &path, &color);
        if do_fill
        {
            Image::fill_path_raw(dt, &path, &color);
        }
    }

    /// Draws a point onto the image
    fn draw_point(&self, dt: &mut DrawTarget, pt: &Point, radius: f32, color: u32, do_fill: bool)
    {
        self.draw_point_raw(dt, pt, radius, &Image::source_from_color(color), do_fill);
    }

    /// Draws a point onto the image with an opacity (0-255)
    fn draw_point_with_opacity(&self, dt: &mut DrawTarget, pt: &Point, radius: f32, color: u32, alpha: u8, do_fill: bool)
    {
        self.draw_point_raw(dt, pt, radius, &Image::source_from_color_with_opacity(color, alpha), do_fill);
    }

    /// Draws a line onto the image
    fn draw_line_raw(&self, dt: &mut DrawTarget, line: &Line, color: &Source<'static>)
    {
        let transformed_line = Line::new(
            self.get_transformed_point(&line.p0),
            self.get_transformed_point(&line.p1)
        );
            
        let mut pb = PathBuilder::new();
        pb.move_to(transformed_line.p0.x as f32, transformed_line.p0.y as f32);
        pb.line_to(transformed_line.p1.x as f32, transformed_line.p1.y as f32);
        let path = pb.finish();
        
        Image::stroke_path_raw(dt, &path, &color);
    }

    /// Draws a line onto the image
    fn draw_line(&self, dt: &mut DrawTarget, line: &Line, color: u32)
    {
        self.draw_line_raw(dt, line, &Image::source_from_color(color));
    }

    /// Draws a line onto the image with an opacity (0-255)
    fn draw_line_with_opacity(&self, dt: &mut DrawTarget, line: &Line, color: u32, alpha: u8)
    {
        self.draw_line_raw(dt, line, &Image::source_from_color_with_opacity(color, alpha));
    }

    /// Draws a dashed line onto the image
    // fn draw_dashed_line_raw(&self, dt: &mut DrawTarget, line: &Line, color: &Source<'static>)
    // {
    //     let transformed_line = Line::new(
    //         self.get_transformed_point(&line.p0),
    //         self.get_transformed_point(&line.p1)
    //     );
            
    //     let mut pb = PathBuilder::new();
    //     pb.move_to(transformed_line.p0.x as f32, transformed_line.p0.y as f32);
    //     pb.line_to(transformed_line.p1.x as f32, transformed_line.p1.y as f32);
    //     let path = pb.finish();
        
    //     Image::stroke_dashed_path_raw(dt, &path, &color);
    // }

    /// Draws a dashed line onto the image
    // fn draw_dashed_line(&self, dt: &mut DrawTarget, line: &Line, color: u32)
    // {
    //     self.draw_dashed_line_raw(dt, line, &Image::source_from_color(color));
    // }

    /// Draws a dashed line onto the image with an opacity (0-255)
    // fn draw_dashed_line_with_opacity(&self, dt: &mut DrawTarget, line: &Line, color: u32, alpha: u8)
    // {
    //     self.draw_dashed_line_raw(dt, line, &Image::source_from_color_with_opacity(color, alpha));
    // }
    
    /// Draws a cubic bezier onto the image
    fn draw_cubic_bez(&self, dt: &mut DrawTarget, curve: &CubicBez, color: u32)
    {
        let transformed_curve = self.get_affine_transform() * *curve;
            
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_curve.p0.x as f32, transformed_curve.p0.y as f32
            );
        pb.cubic_to(
            transformed_curve.p1.x as f32, transformed_curve.p1.y as f32,
            transformed_curve.p2.x as f32, transformed_curve.p2.y as f32,
            transformed_curve.p3.x as f32, transformed_curve.p3.y as f32
            );
        let path = pb.finish();
        
        Image::stroke_path(dt, &path, color);
    }

    /// Draws a cubic bezier onto the image
    fn draw_cubic_bez_with_opacity(&self, dt: &mut DrawTarget, curve: &CubicBez, color: u32, alpha: u8)
    {
        let transformed_curve = self.get_affine_transform() * *curve;
            
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_curve.p0.x as f32, transformed_curve.p0.y as f32
            );
        pb.cubic_to(
            transformed_curve.p1.x as f32, transformed_curve.p1.y as f32,
            transformed_curve.p2.x as f32, transformed_curve.p2.y as f32,
            transformed_curve.p3.x as f32, transformed_curve.p3.y as f32
            );
        let path = pb.finish();
        
        Image::stroke_path_with_opacity(dt, &path, color, alpha);
    }
    
    /// Draws a quadratic bezier onto the image
    fn draw_quad_bez(&self, dt: &mut DrawTarget, curve: &QuadBez, color: u32)
    {
        let transformed_curve = self.get_affine_transform() * *curve;
        //let transformed_curve = (*curve * self.scale) + self.offset;
            
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_curve.p0.x as f32, transformed_curve.p0.y as f32
            );
        pb.quad_to(
            transformed_curve.p1.x as f32, transformed_curve.p1.y as f32,
            transformed_curve.p2.x as f32, transformed_curve.p2.y as f32
            );
        let path = pb.finish();
        Image::stroke_path(dt, &path, color);
    }

    /// Draws a quadratic bezier onto the image
    fn draw_quad_bez_with_opacity(&self, dt: &mut DrawTarget, curve: &QuadBez, color: u32, alpha: u8)
    {
        let transformed_curve = self.get_affine_transform() * *curve;
        //let transformed_curve = (*curve * self.scale) + self.offset;
            
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_curve.p0.x as f32, transformed_curve.p0.y as f32
            );
        pb.quad_to(
            transformed_curve.p1.x as f32, transformed_curve.p1.y as f32,
            transformed_curve.p2.x as f32, transformed_curve.p2.y as f32
            );
        let path = pb.finish();
        Image::stroke_path_with_opacity(dt, &path, color, alpha);
    }

    /// Offsets a line by a signed distance
    fn offset_line(line: &Line, distance: f64) -> Line
    {
        let vec = line.p1 - line.p0;
        let vec = Vec2::new(vec.y, -vec.x);
        let vec = (vec / vec.hypot()) * -distance;
        
		line.clone() + vec
    }

    /// Extends a line by a signed distance
    fn extend_line(line: &Line, distance: f64) -> Line
    {
        let t_extend = distance / (line.p1 - line.p0).hypot();
		Line::new(line.eval(-t_extend), line.eval(1. + t_extend))
    }
    
    /// Draws a fat line onto the image
    fn draw_fat_line(&self, dt: &mut DrawTarget, fl: &FatLine, color: u32)
    {
        let img_size = Vec2::new(dt.width() as f64, dt.height() as f64).hypot();
        
        let line = Line::new(fl.p0, fl.p1);
        let line_dmin = Image::offset_line(&line, fl.dmin);
        let line_dmax = Image::offset_line(&line, fl.dmax);
        
        let transform = self.get_affine_transform();
        let transformed_line = Image::extend_line(&(transform * line), img_size);
        let transformed_line_dmin = Image::extend_line(&(transform * line_dmin), img_size);
        let transformed_line_dmax = Image::extend_line(&(transform * line_dmax), img_size);
        // let transformed_line = transform * line;
        // let transformed_line_dmin = transform * line_dmin;
        // let transformed_line_dmax = transform * line_dmax;
            
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_line.p0.x as f32, transformed_line.p0.y as f32,
            );
        pb.line_to(
            transformed_line.p1.x as f32, transformed_line.p1.y as f32,
            );
        let path = pb.finish();
        Image::stroke_path(dt, &path, color);
        
        let mut pb = PathBuilder::new();
        pb.move_to(
            transformed_line_dmin.p0.x as f32, transformed_line_dmin.p0.y as f32,
            );
        pb.line_to(
            transformed_line_dmin.p1.x as f32, transformed_line_dmin.p1.y as f32,
            );
        pb.line_to(
            transformed_line_dmax.p1.x as f32, transformed_line_dmax.p1.y as f32,
            );
        pb.line_to(
            transformed_line_dmax.p0.x as f32, transformed_line_dmax.p0.y as f32,
            );
        pb.line_to(
            transformed_line_dmin.p0.x as f32, transformed_line_dmin.p0.y as f32,
            );
        let path = pb.finish();
        // Image::fill_path_with_opacity(dt, &path, Image::lighter_color(color), 0xB0);
        Image::fill_path_with_opacity(dt, &path, color, 0xB0);
        Image::stroke_dashed_path(dt, &path, color);
        // Image::stroke_path(dt, &path, color);
    }
    
    /// Saves the image to a file
    pub fn save(&mut self, file_name: &str)
    {
        let mut bounds = Rect::ZERO;
        for drawable in self.items.iter()
        {
            let dbounds = drawable.bounds();
            if bounds == Rect::ZERO
            {
                bounds = dbounds;
            }
            else if dbounds != Rect::ZERO
            {
                bounds = bounds.union(dbounds);
            }
        }

        if bounds.size() == Size::ZERO
        {
            println!("cannot create image from zero bounds: {}", bounds);
            return;
        }
        
        let image_size = self.image_size_from_bounds(&bounds);
        let mut dt = DrawTarget::new(image_size.width as i32, image_size.height as i32);
        
        let mut background = PathBuilder::new();
        background.rect(0.0, 0.0, dt.width() as f32, dt.height() as f32);
        background.close();
        let background = background.finish();
        dt.fill(
            &background,
            &Source::Solid(SolidSource { r: 0xFF, g: 0xFF, b: 0xFF, a: 0xFF }),
            &DrawOptions::new()
            );

        for drawable in self.items.iter()
        {
            drawable.draw_to_image(self, &mut dt);
        }

        let save_success = dt.write_png(file_name);
        match save_success
        {
            Ok(_) => println!("Image saved! {}", file_name),
            Err(e) => println!("Unable to save image: {:?}", e),
        }
    }
}

/*****************************************************************************
 * Format
 *****************************************************************************/

impl fmt::Display for Image
{
    /// Defines how Image is printed to the output
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result
    {
        write!(f, "Img(scale: {:?}, margins: {}, items: {})", self.scale, self.margins, self.items.len())
    }
}