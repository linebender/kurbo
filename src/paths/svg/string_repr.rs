//! This module handles converting to/from the string representation of an SVG path.
use crate::{
    paths::svg::{ArcTo, CubicTo, OneVec, Path, PathEl, QuadTo, SmoothCubicTo},
    Point,
};
use std::{error::Error as StdError, fmt};

// parse

/// Try to parse the input as an SVG path.
pub fn parse(input: &str) -> Result<Path, SvgParseError> {
    match parser::path(input) {
        Ok((_, v)) => Ok(v),
        Err(_) => Err(SvgParseError::Wrong),
    }
}

/// An error which can be returned when parsing an SVG.
#[derive(Debug)]
pub enum SvgParseError {
    /// A number was expected.
    Wrong,
    /// The input string ended while still expecting input.
    UnexpectedEof,
    /// Encountered an unknown command letter.
    UnknownCommand(char),
}

impl fmt::Display for SvgParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SvgParseError::Wrong => write!(f, "Unable to parse a number"),
            SvgParseError::UnexpectedEof => write!(f, "Unexpected EOF"),
            SvgParseError::UnknownCommand(letter) => write!(f, "Unknown command, \"{letter}\""),
        }
    }
}

impl StdError for SvgParseError {}

mod parser {
    use super::{ArcTo, CubicTo, OneVec, Path, PathEl, QuadTo, SmoothCubicTo};
    use crate::{Point, Vec2};
    use nom::{
        branch::alt,
        bytes::complete::{tag, take_till},
        combinator::{map, opt, recognize, value},
        multi::{fold_many1, many0},
        number::complete::double,
        sequence::tuple,
        IResult,
    };
    use std::f64::consts::PI;

    /// top-level parser for svg path
    pub fn path(input: &str) -> IResult<&str, Path> {
        let mut path = Path::new();

        let (input, _) = whitespace(input)?;
        let (input, el) = moveto(input)?;
        path.push(el).unwrap(); // must be a moveto so panic unreachable
        let (mut input, _) = whitespace(input)?;

        while !input.is_empty() {
            let (input_, el) = command(input)?;
            input = input_;
            path.push(el).unwrap();
            let (input_, _) = whitespace(input)?;
            input = input_;
        }
        Ok((input, path))
    }

    fn command(input: &str) -> IResult<&str, PathEl> {
        alt((
            moveto,
            closepath,
            lineto,
            horizontal_lineto,
            vertical_lineto,
            curveto,
            smooth_curveto,
            quadto,
            smooth_quadto,
            arcto,
            bearing,
        ))(input)
    }

    fn moveto<'src>(input: &'src str) -> IResult<&'src str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("M")), value(true, tag("m"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, points) = coordinate_pair_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::MoveToRel(points)
            } else {
                PathEl::MoveTo(points)
            },
        ))
    }

    fn closepath(input: &str) -> IResult<&str, PathEl> {
        value(PathEl::ClosePath, alt((tag("z"), tag("Z"))))(input)
    }

    fn lineto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("L")), value(true, tag("l"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, points) = coordinate_pair_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::LineToRel(points)
            } else {
                PathEl::LineTo(points)
            },
        ))
    }

    fn horizontal_lineto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, _) = whitespace(input)?;
        let (input, rel) = alt((value(false, tag("H")), value(true, tag("h"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, points) = coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::HorizRel(points)
            } else {
                PathEl::Horiz(points)
            },
        ))
    }

    fn vertical_lineto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("V")), value(true, tag("v"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, points) = coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::VertRel(points)
            } else {
                PathEl::Vert(points)
            },
        ))
    }

    fn curveto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("C")), value(true, tag("c"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, curves) = curveto_coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::CubicToRel(curves)
            } else {
                PathEl::CubicTo(curves)
            },
        ))
    }

    fn smooth_curveto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("S")), value(true, tag("s"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, curves) = smooth_curveto_coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::SmoothCubicToRel(curves)
            } else {
                PathEl::SmoothCubicTo(curves)
            },
        ))
    }

    fn quadto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("Q")), value(true, tag("q"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, curves) = quadto_coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::QuadToRel(curves)
            } else {
                PathEl::QuadTo(curves)
            },
        ))
    }

    fn smooth_quadto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("T")), value(true, tag("t"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, points) = coordinate_pair_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::SmoothQuadToRel(points)
            } else {
                PathEl::SmoothQuadTo(points)
            },
        ))
    }

    fn arcto(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("A")), value(true, tag("a"))))(input)?;

        // one or more coordinate pairs
        let (input, _) = whitespace(input)?;
        let (input, arcs) = arcto_coordinate_sequence(input)?;
        Ok((
            input,
            if rel {
                PathEl::EllipticArcRel(arcs)
            } else {
                PathEl::EllipticArc(arcs)
            },
        ))
    }

    fn bearing(input: &str) -> IResult<&str, PathEl> {
        // command
        let (input, rel) = alt((value(false, tag("B")), value(true, tag("b"))))(input)?;
        let (input, _) = whitespace(input)?;
        let folder = if rel {
            |acc: f64, value: f64| (acc + value).rem_euclid(2. * PI)
        } else {
            |_, value| value
        };
        let (input, value) = fold_many1(double, || 0., folder)(input)?;
        Ok((
            input,
            if rel {
                PathEl::BearingRel(value)
            } else {
                PathEl::Bearing(value)
            },
        ))
    }

    fn arcto_coordinate_sequence(input: &str) -> IResult<&str, OneVec<ArcTo>> {
        sequence(arcto_coordinates)(input)
    }

    fn quadto_coordinate_sequence(input: &str) -> IResult<&str, OneVec<QuadTo>> {
        sequence(quadto_coordinates)(input)
    }

    fn smooth_curveto_coordinate_sequence(input: &str) -> IResult<&str, OneVec<SmoothCubicTo>> {
        sequence(smooth_curveto_coordinates)(input)
    }

    fn curveto_coordinate_sequence(input: &str) -> IResult<&str, OneVec<CubicTo>> {
        sequence(curveto_coordinates)(input)
    }

    fn coordinate_pair_sequence(input: &str) -> IResult<&str, OneVec<Point>> {
        sequence(coordinate_pair)(input)
    }

    fn coordinate_sequence(input: &str) -> IResult<&str, OneVec<f64>> {
        sequence(coordinate)(input)
    }

    fn sequence<T>(
        mut inner: impl FnMut(&str) -> IResult<&str, T>,
    ) -> impl FnMut(&str) -> IResult<&str, OneVec<T>> {
        move |input| {
            let (input, first) = inner(input)?;
            let (input, rest) = many0(map(tuple((comma_or_ws, &mut inner)), |(_, p)| p))(input)?;
            Ok((input, OneVec::from_single_rest(first, rest)))
        }
    }

    fn arcto_coordinates(input: &str) -> IResult<&str, ArcTo> {
        let (input, rx) = double(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, ry) = double(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, x_rotation) = double(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, large_arc) = flag(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, sweep) = flag(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, to) = coordinate_pair(input)?;
        Ok((
            input,
            ArcTo {
                to,
                radii: Vec2::new(rx, ry),
                x_rotation,
                large_arc,
                sweep,
            },
        ))
    }

    fn quadto_coordinates(input: &str) -> IResult<&str, QuadTo> {
        let (input, ctrl) = coordinate_pair(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, to) = coordinate_pair(input)?;
        Ok((input, QuadTo { ctrl, to }))
    }

    fn smooth_curveto_coordinates(input: &str) -> IResult<&str, SmoothCubicTo> {
        let (input, ctrl2) = coordinate_pair(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, to) = coordinate_pair(input)?;
        Ok((input, SmoothCubicTo { ctrl2, to }))
    }

    fn curveto_coordinates(input: &str) -> IResult<&str, CubicTo> {
        let (input, ctrl1) = coordinate_pair(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, ctrl2) = coordinate_pair(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, to) = coordinate_pair(input)?;
        Ok((input, CubicTo { ctrl1, ctrl2, to }))
    }

    fn coordinate_pair(input: &str) -> IResult<&str, Point> {
        let (input, x) = coordinate(input)?;
        let (input, _) = comma_or_ws(input)?;
        let (input, y) = coordinate(input)?;
        Ok((input, Point { x, y }))
    }

    fn coordinate(input: &str) -> IResult<&str, f64> {
        double(input)
    }

    fn flag(input: &str) -> IResult<&str, bool> {
        alt((value(true, tag("1")), value(false, tag("0"))))(input)
    }

    fn comma_or_ws(input: &str) -> IResult<&str, &str> {
        fn inner(input: &str) -> IResult<&str, ()> {
            let (input, _) = whitespace(input)?;
            let (input, _) = opt(tag(","))(input)?;
            let (input, _) = whitespace(input)?;
            Ok((input, ()))
        }
        recognize(inner)(input)
    }

    fn whitespace(input: &str) -> IResult<&str, &str> {
        take_till(|ch| matches!(ch, ' ' | '\u{9}' | '\u{a}' | '\u{c}' | '\u{d}'))(input)
    }
}

// Stringify

impl fmt::Display for Path {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut iter = self.elements.iter();
        if let Some(el) = iter.next() {
            write!(f, "{el}")?;
        }
        for el in iter {
            write!(f, " {el}")?;
        }
        Ok(())
    }
}

impl fmt::Display for PathEl {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            PathEl::MoveTo(points) => {
                write!(f, "M")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::MoveToRel(points) => {
                write!(f, "m")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::ClosePath => {
                write!(f, "Z")?;
            }
            PathEl::LineTo(points) => {
                write!(f, "L")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::LineToRel(points) => {
                write!(f, "l")?;
                points.write_spaced(write_point, f)?;
            }
            PathEl::Horiz(amts) => {
                write!(f, "H")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::HorizRel(amts) => {
                write!(f, "h")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::Vert(amts) => {
                write!(f, "V")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::VertRel(amts) => {
                write!(f, "v")?;
                amts.write_spaced(|v, f| write!(f, "{}", v), f)?;
            }
            PathEl::CubicTo(cubic_tos) => {
                write!(f, "C")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::CubicToRel(cubic_tos) => {
                write!(f, "c")?;
                cubic_tos.write_spaced(CubicTo::write_vals, f)?;
            }
            PathEl::SmoothCubicTo(cubic_tos) => {
                write!(f, "S")?;
                cubic_tos.write_spaced(SmoothCubicTo::write_vals, f)?;
            }
            PathEl::SmoothCubicToRel(cubic_tos) => {
                write!(f, "s")?;
                cubic_tos.write_spaced(SmoothCubicTo::write_vals, f)?;
            }
            PathEl::QuadTo(quad_tos) => {
                write!(f, "Q")?;
                quad_tos.write_spaced(QuadTo::write_vals, f)?;
            }
            PathEl::QuadToRel(quad_tos) => {
                write!(f, "q")?;
                quad_tos.write_spaced(QuadTo::write_vals, f)?;
            }
            PathEl::SmoothQuadTo(quad_tos) => {
                write!(f, "T")?;
                quad_tos.write_spaced(write_point, f)?;
            }
            PathEl::SmoothQuadToRel(quad_tos) => {
                write!(f, "t")?;
                quad_tos.write_spaced(write_point, f)?;
            }
            PathEl::EllipticArc(arcs) => {
                write!(f, "A")?;
                arcs.write_spaced(ArcTo::write_vals, f)?;
            }
            PathEl::EllipticArcRel(arcs) => {
                write!(f, "a")?;
                arcs.write_spaced(ArcTo::write_vals, f)?;
            }
            PathEl::Bearing(bearing) => {
                write!(f, "B{bearing}",)?;
            }
            PathEl::BearingRel(bearing) => {
                write!(f, "b{bearing}",)?;
            }
        }
        Ok(())
    }
}

impl CubicTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{} {},{}",
            self.ctrl1.x, self.ctrl1.y, self.ctrl2.x, self.ctrl2.y, self.to.x, self.to.y
        )
    }
}

impl SmoothCubicTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{}",
            self.ctrl2.x, self.ctrl2.y, self.to.x, self.to.y
        )
    }
}

impl QuadTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {},{}",
            self.ctrl.x, self.ctrl.y, self.to.x, self.to.y
        )
    }
}

impl ArcTo {
    fn write_vals(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{},{} {} {},{} {},{}",
            self.radii.x,
            self.radii.y,
            self.x_rotation,
            self.large_arc,
            self.sweep,
            self.to.x,
            self.to.y
        )
    }
}

fn write_point(Point { x, y }: &Point, f: &mut fmt::Formatter) -> fmt::Result {
    write!(f, "{x},{y}")
}
