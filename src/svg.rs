//! SVG path representation.

use std::f64::consts::PI;
use std::io::Write;

use crate::{Arc, BezPath, PathEl, PathSeg, Point, Vec2, ParamCurve};

// Note: the SVG arc logic is heavily adapted from https://github.com/nical/lyon

/// A single SVG arc segment.
#[derive(Clone, Copy, Debug)]
pub struct SvgArc {
    pub from: Point,
    pub to: Point,
    pub radii: Vec2,
    pub x_rotation: f64,
    pub large_arc: bool,
    pub sweep: bool,
}

impl BezPath {
	/// Create a BezPath with segments corresponding to the sequence of
    /// `PathSeg`s
    pub fn from_path_segments(segments: impl Iterator<Item=PathSeg>) -> BezPath {
        let mut path_elements = Vec::new();
        let mut current_pos = None;

        for segment in segments {
            let start = segment.start();
            if Some(start) != current_pos {
                path_elements.push(PathEl::MoveTo(start));
            };
            path_elements.push(match segment {
				    PathSeg::Line(l) => PathEl::LineTo(l.p1),
				    PathSeg::Quad(q) => PathEl::QuadTo(q.p1, q.p2),
				    PathSeg::Cubic(c) => PathEl::CurveTo(c.p1, c.p2, c.p3)
			});

            current_pos = Some(segment.end());
        }
        
        BezPath::from_vec(path_elements)
    }

    /// Convert the path to an SVG path string representation.
    ///
    /// The current implementation doesn't take any special care to produce a
    /// short string (reducing precision, using relative movement).
    pub fn to_svg(&self) -> String {
        let mut result = Vec::new();
        for el in self.elements() {
            match *el {
                PathEl::MoveTo(p) => write!(result, "M{} {}", p.x, p.y).unwrap(),
                PathEl::LineTo(p) => write!(result, "L{} {}", p.x, p.y).unwrap(),
                PathEl::QuadTo(p1, p2) => {
                    write!(result, "Q{} {} {} {}", p1.x, p1.y, p2.x, p2.y).unwrap()
                }
                PathEl::CurveTo(p1, p2, p3) => write!(
                    result,
                    "C{} {} {} {} {} {}",
                    p1.x, p1.y, p2.x, p2.y, p3.x, p3.y
                )
                .unwrap(),
                PathEl::ClosePath => write!(result, "Z").unwrap(),
            }
        }
        String::from_utf8(result).unwrap()
    }

    pub fn from_svg(data: &str) -> Result<BezPath, SvgParseError> {
        let mut lexer = SvgLexer::new(data);
        let mut path = BezPath::new();
        let mut last_cmd = 0;
        let mut last_ctrl = None;
        while let Some(c) = lexer.get_cmd(last_cmd) {
            if c == b'm' || c == b'M' {
                let pt = lexer.get_maybe_relative(c)?;
                path.move_to(pt);
                lexer.last_pt = pt;
                last_ctrl = Some(pt);
                last_cmd = c - (b'M' - b'L');
            } else if c == b'l' || c == b'L' {
                let pt = lexer.get_maybe_relative(c)?;
                path.line_to(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'h' || c == b'H' {
                let mut x = lexer.get_number()?;
                lexer.opt_comma();
                if c == b'h' {
                    x += lexer.last_pt.x;
                }
                let pt = Point::new(x, lexer.last_pt.y);
                path.line_to(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'v' || c == b'V' {
                let mut y = lexer.get_number()?;
                lexer.opt_comma();
                if c == b'v' {
                    y += lexer.last_pt.y;
                }
                let pt = Point::new(lexer.last_pt.x, y);
                path.line_to(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'q' || c == b'Q' {
                let p1 = lexer.get_maybe_relative(c)?;
                let p2 = lexer.get_maybe_relative(c)?;
                path.quad_to(p1, p2);
                last_ctrl = Some(p1);
                lexer.last_pt = p2;
                last_cmd = c;
            } else if c == b't' || c == b'T' {
                let p1 = match last_ctrl {
                    Some(ctrl) => (2.0 * lexer.last_pt.to_vec2() - ctrl.to_vec2()).to_point(),
                    None => lexer.last_pt,
                };
                let p2 = lexer.get_maybe_relative(c)?;
                path.quad_to(p1, p2);
                last_ctrl = Some(p1);
                lexer.last_pt = p2;
                last_cmd = c;
            } else if c == b'c' || c == b'C' {
                let p1 = lexer.get_maybe_relative(c)?;
                let p2 = lexer.get_maybe_relative(c)?;
                let p3 = lexer.get_maybe_relative(c)?;
                path.curve_to(p1, p2, p3);
                last_ctrl = Some(p2);
                lexer.last_pt = p3;
                last_cmd = c;
            } else if c == b's' || c == b'S' {
                let p1 = match last_ctrl {
                    Some(ctrl) => (2.0 * lexer.last_pt.to_vec2() - ctrl.to_vec2()).to_point(),
                    None => lexer.last_pt,
                };
                let p2 = lexer.get_maybe_relative(c)?;
                let p3 = lexer.get_maybe_relative(c)?;
                path.curve_to(p1, p2, p3);
                last_ctrl = Some(p2);
                lexer.last_pt = p3;
                last_cmd = c;
            } else if c == b'a' || c == b'A' {
                let radii = lexer.get_number_pair()?;
                let x_rotation = lexer.get_number()?.to_radians();
                lexer.opt_comma();
                let large_arc = lexer.get_number()?;
                lexer.opt_comma();
                let sweep = lexer.get_number()?;
                lexer.opt_comma();
                let p = lexer.get_maybe_relative(c)?;
                let svg_arc = SvgArc {
                    from: lexer.last_pt,
                    to: p,
                    radii: radii.to_vec2(),
                    x_rotation,
                    large_arc: large_arc != 0.0,
                    sweep: sweep != 0.0,
                };

                match Arc::from_svg_arc(&svg_arc) {
                    Some(arc) => {
                        // TODO: consider making tolerance configurable
                        arc.to_cubic_beziers(0.1, |p1, p2, p3| {
                            path.curve_to(p1, p2, p3);
                        });
                    }
                    None => {
                        path.line_to(p);
                    }
                }

                lexer.last_pt = p;
                last_cmd = c;
            } else if c == b'z' || c == b'Z' {
                path.close_path();
                // TODO: implicit moveto
            } else {
                return Err(SvgParseError::UnknownCommand(c as char));
            };
        }
        Ok(path)
    }
}

/// An error which can be returned when parsing an SVG.
#[derive(Debug)]
pub enum SvgParseError {
    Wrong,
    UnexpectedEof,
    UnknownCommand(char),
}

struct SvgLexer<'a> {
    data: &'a str,
    ix: usize,
    pub last_pt: Point,
}

impl<'a> SvgLexer<'a> {
    fn new(data: &str) -> SvgLexer {
        SvgLexer {
            data,
            ix: 0,
            last_pt: Point::ORIGIN,
        }
    }

    fn skip_ws(&mut self) {
        while let Some(&c) = self.data.as_bytes().get(self.ix) {
            if !(c == b' ' || c == 9 || c == 10 || c == 12 || c == 13) {
                break;
            }
            self.ix += 1;
        }
    }

    fn get_cmd(&mut self, last_cmd: u8) -> Option<u8> {
        self.skip_ws();
        if let Some(c) = self.get_byte() {
            if (c >= b'a' && c <= b'z') || (c >= b'A' && c <= b'Z') {
                return Some(c);
            } else if last_cmd != 0 && (c == b'-' || c == b'.' || (c >= b'0' && c <= b'9')) {
                // Plausible number start
                self.unget();
                return Some(last_cmd);
            } else {
                self.unget();
            }
        }
        None
    }

    fn get_byte(&mut self) -> Option<u8> {
        self.data.as_bytes().get(self.ix).map(|&c| {
            self.ix += 1;
            c
        })
    }

    fn unget(&mut self) {
        self.ix -= 1;
    }

    fn get_number(&mut self) -> Result<f64, SvgParseError> {
        self.skip_ws();
        let start = self.ix;
        let c = self.get_byte().ok_or(SvgParseError::UnexpectedEof)?;
        if !(c == b'-' || c == b'+') {
            self.unget();
        }
        let mut digit_count = 0;
        let mut seen_period = false;
        while let Some(c) = self.get_byte() {
            if c >= b'0' && c <= b'9' {
                digit_count += 1;
            } else if c == b'.' && !seen_period {
                seen_period = true;
            } else {
                self.unget();
                break;
            }
        }
        if digit_count > 0 {
            self.data[start..self.ix]
                .parse()
                .map_err(|_| SvgParseError::Wrong)
        } else {
            Err(SvgParseError::Wrong)
        }
    }

    fn get_number_pair(&mut self) -> Result<Point, SvgParseError> {
        let x = self.get_number()?;
        self.opt_comma();
        let y = self.get_number()?;
        self.opt_comma();
        Ok(Point::new(x, y))
    }

    fn get_maybe_relative(&mut self, cmd: u8) -> Result<Point, SvgParseError> {
        let pt = self.get_number_pair()?;
        if cmd >= b'a' && cmd <= b'z' {
            Ok(self.last_pt + pt.to_vec2())
        } else {
            Ok(pt)
        }
    }

    fn opt_comma(&mut self) {
        self.skip_ws();
        if let Some(c) = self.get_byte() {
            if c != b',' {
                self.unget();
            }
        }
    }
}

impl SvgArc {
    /// Checks that arc is actually a straight line.
    ///
    /// In this case, it can be replaced with a LineTo.
    pub fn is_straight_line(&self) -> bool {
        self.radii.x.abs() <= 1e-5 || self.radii.y.abs() <= 1e-5 || self.from == self.to
    }
}

impl Arc {
    /// Creates an `Arc` from a `SvgArc`.
    ///
    /// Returns `None` if `arc` is actually a straight line.
    pub fn from_svg_arc(arc: &SvgArc) -> Option<Arc> {
        // Have to check this first, otherwise `sum_of_sq` will be 0.
        if arc.is_straight_line() {
            return None;
        }

        let mut rx = arc.radii.x.abs();
        let mut ry = arc.radii.y.abs();

        let xr = arc.x_rotation % (2.0 * PI);
        let cos_phi = xr.cos();
        let sin_phi = xr.sin();
        let hd_x = (arc.from.x - arc.to.x) * 0.5;
        let hd_y = (arc.from.y - arc.to.y) * 0.5;
        let hs_x = (arc.from.x + arc.to.x) * 0.5;
        let hs_y = (arc.from.y + arc.to.y) * 0.5;

        // F6.5.1
        let p = Vec2::new(
            cos_phi * hd_x + sin_phi * hd_y,
            -sin_phi * hd_x + cos_phi * hd_y,
        );

        // Sanitize the radii.
        // If rf > 1 it means the radii are too small for the arc to
        // possibly connect the end points. In this situation we scale
        // them up according to the formula provided by the SVG spec.

        // F6.6.2
        let rf = p.x * p.x / (rx * rx) + p.y * p.y / (ry * ry);
        if rf > 1.0 {
            let scale = rf.sqrt();
            rx *= scale;
            ry *= scale;
        }

        let rxry = rx * ry;
        let rxpy = rx * p.y;
        let rypx = ry * p.x;
        let sum_of_sq = rxpy * rxpy + rypx * rypx;

        debug_assert_ne!(sum_of_sq, 0.0);

        // F6.5.2
        let sign_coe = if arc.large_arc == arc.sweep {
            -1.0
        } else {
            1.0
        };
        let coe = sign_coe * ((rxry * rxry - sum_of_sq) / sum_of_sq).abs().sqrt();
        let transformed_cx = coe * rxpy / ry;
        let transformed_cy = -coe * rypx / rx;

        // F6.5.3
        let center = Point::new(
            cos_phi * transformed_cx - sin_phi * transformed_cy + hs_x,
            sin_phi * transformed_cx + cos_phi * transformed_cy + hs_y,
        );

        let start_v = Vec2::new((p.x - transformed_cx) / rx, (p.y - transformed_cy) / ry);
        let end_v = Vec2::new((-p.x - transformed_cx) / rx, (-p.y - transformed_cy) / ry);

        let start_angle = start_v.atan2();

        let mut sweep_angle = (end_v.atan2() - start_angle) % (2.0 * PI);

        if arc.sweep && sweep_angle < 0.0 {
            sweep_angle += 2.0 * PI;
        } else if !arc.sweep && sweep_angle > 0.0 {
            sweep_angle -= 2.0 * PI;
        }

        Some(Arc {
            center,
            radii: Vec2::new(rx, ry),
            start_angle,
            sweep_angle,
            x_rotation: arc.x_rotation,
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::{BezPath, Shape, Point, PathSeg, Line, QuadBez, CubicBez};

    #[test]
    fn test_parse_svg() {
        let path = BezPath::from_svg("m10 10 100 0 0 100 -100 0z").unwrap();
        assert_eq!(path.segments().count(), 4);
    }

    #[test]
    fn test_parse_svg_arc() {
        let path = BezPath::from_svg("M 100 100 A 25 25 0 1 0 -25 25 z").unwrap();
        assert_eq!(path.segments().count(), 3);
    }

    // Regression test for #51
    #[test]
    fn test_parse_svg_arc_pie() {
        let path = BezPath::from_svg("M 100 100 h 25 a 25 25 0 1 0 -25 25 z").unwrap();
        // Approximate figures, but useful for regression testing
        assert_eq!(path.area().round(), -1473.0);
        assert_eq!(path.perimeter(1e-6).round(), 168.0);
    }

    #[test]
    fn test_write_svg_single() {
        let segments = [
            CubicBez::new(
                Point::new(10., 10.),
                Point::new(20., 20.),
                Point::new(30., 30.),
                Point::new(40., 40.),
            ).into()
        ];
        let path = BezPath::from_path_segments(segments.iter().cloned());

        assert_eq!(path.to_svg(), "M10 10C20 20 30 30 40 40");
    }

    #[test]
    fn test_write_svg_two_nomove() {
        let segments = [
            CubicBez::new(
                Point::new(10., 10.),
                Point::new(20., 20.),
                Point::new(30., 30.),
                Point::new(40., 40.),
            ).into(),

            CubicBez::new(
                Point::new(40., 40.),
                Point::new(30., 30.),
                Point::new(20., 20.),
                Point::new(10., 10.),
            ).into(),
        ];
        let path = BezPath::from_path_segments(segments.iter().cloned());

        assert_eq!(path.to_svg(), "M10 10C20 20 30 30 40 40C30 30 20 20 10 10");
    }

    #[test]
    fn test_write_svg_two_move() {
        let segments = [
            CubicBez::new(
                Point::new(10., 10.),
                Point::new(20., 20.),
                Point::new(30., 30.),
                Point::new(40., 40.),
            ).into(),

            CubicBez::new(
                Point::new(50., 50.),
                Point::new(30., 30.),
                Point::new(20., 20.),
                Point::new(10., 10.),
            ).into(),
        ];
        let path = BezPath::from_path_segments(segments.iter().cloned());

        assert_eq!(path.to_svg(), "M10 10C20 20 30 30 40 40M50 50C30 30 20 20 10 10");
    }

    use rand::prelude::*;

    fn gen_random_path_sequence(rng: &mut impl Rng) -> Vec<PathSeg> {
        const MAX_LENGTH: u32 = 10;

        let mut elements = vec![];

        let mut position = None;
        let length = rng.gen_range(0, MAX_LENGTH);
        for _ in 0..length {
            let should_follow: bool = rand::random();
            let kind = rng.gen_range(0, 3);

            let first = if should_follow && position.is_some() {
                position.unwrap()
            } else {
                Point::new(rng.gen(), rng.gen())
            };

            let element = match kind {
                0 => Line::new(
                    first, 
                    Point::new(rng.gen(), rng.gen()),
                ).into(),

                1 => QuadBez::new(
                    first,
                    Point::new(rng.gen(), rng.gen()),
                    Point::new(rng.gen(), rng.gen()),
                ).into(),

                2 => CubicBez::new(
                    first,
                    Point::new(rng.gen(), rng.gen()),
                    Point::new(rng.gen(), rng.gen()),
                    Point::new(rng.gen(), rng.gen()),
                ).into(),

                _ => unreachable!(),
            };

            elements.push(element);
        };

        elements
    }

    #[test]
    fn test_serialize_deserialize() {
        const N_TESTS: u32 = 100;
        let mut rng = rand::thread_rng();

        for _ in 0..N_TESTS {
            let vec = gen_random_path_sequence(&mut rng);
            let ser = BezPath::from_path_segments(vec.iter().cloned()).to_svg();
            let deser = BezPath::from_svg(&ser).expect("failed deserialization");

            let deser_vec = deser.segments().collect::<Vec<PathSeg>>();

            assert_eq!(vec, deser_vec);
        }
    }
}
