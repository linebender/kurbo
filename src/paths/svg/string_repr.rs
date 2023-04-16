//! This module handles converting to/from the string representation of an SVG path.
use self::lexer::{Lexer, Token};
use crate::{
    paths::svg::{ArcTo, CubicTo, Path, PathEl, QuadTo, SmoothCubicTo},
    Point,
};
use std::{error::Error as StdError, fmt};

// parse

/// Try to parse the input as an SVG path.
pub fn parse(input: &str) -> Result<Path, SvgParseError> {
    todo!()
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
    use super::{Path, PathEl};
    use crate::Point;
    use nom::{
        branch::alt,
        bytes::complete::{tag, take_till},
        combinator::{map, opt, recognize, value},
        number::complete::double,
        sequence::tuple,
        IResult,
    };

    /// top-level parser for svg path
    pub fn path(input: &str) -> IResult<&str, Path> {
        let mut path = Path::new();

        let (input, _) = whitespace(input)?;
        let (input, _) = moveto(&mut path, input)?;
        todo!()
    }

    fn command(input: &str) -> IResult<&str, PathEl> {
        todo!()
    }

    fn moveto<'src>(path: &mut Path, input: &'src str) -> IResult<&'src str, ()> {
        // command
        let (input, _) = whitespace(input)?;
        let (input, rel) = alt((value(true, tag("M")), value(false, tag("m"))))(input)?;

        // one or more coordinate pairs

        // optional closepath
        let (input, _) = whitespace(input)?;
        let (input, close) = opt(closepath)(input)?;
        let close = close.is_some();

        // now we're at the end, add all this to the path
        todo!()
    }

    fn closepath(input: &str) -> IResult<&str, &str> {
        alt((tag("z"), tag("Z")))(input)
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

mod lexer {
    use super::SvgParseError;
    use crate::Point;

    type Result<T = (), E = SvgParseError> = std::result::Result<T, E>;

    /// An input token
    ///
    /// Here we are more flexible than the specification. Some parts of the specification require
    /// either a comma or space between numbers, but here we allow this to be omitted. I think the
    /// specification encourages the behavior of this parser, but I find it unclear on this point.
    pub enum Token<'a> {
        Number(TokenData<'a, f64>),
        Command(TokenData<'a, char>),
    }

    pub struct TokenData<'a, T> {
        value: T,
        src: &'a str,
    }

    pub struct Lexer<'a> {
        src: &'a str,
    }

    impl<'a> Lexer<'a> {
        fn new(src: &str) -> Lexer {
            Lexer { src }
        }

        fn get_command(&mut self) -> Result<Option<TokenData<char>>> {
            todo!()
        }

        fn get_number(&mut self) -> Result<TokenData<f64>> {
            todo!()
        }

        fn skip_ws(&self) {
            let mut input = self.src;
            while let Some(c) = input.chars().next() {
                if !(c == ' ' // '\u{20}'
                    || c == '\u{9}'
                    || c == '\u{a}'
                    || c == '\u{c}'
                    || c == '\u{d}'
                    || c == ',')
                {
                    break;
                }
                input = &input[1..];
            }
        }

        fn get_cmd(&mut self, last_cmd: u8) -> Option<u8> {
            self.skip_ws();
            if let Some(c) = self.get_byte() {
                if c.is_ascii_lowercase() || c.is_ascii_uppercase() {
                    return Some(c);
                } else if last_cmd != 0 && (c == b'-' || c == b'.' || c.is_ascii_digit()) {
                    // Plausible number start
                    self.unget();
                    return Some(last_cmd);
                } else {
                    self.unget();
                }
            }
            None
        }

        fn try_number(&self) -> Result<(&str, f64), SvgParseError> {
            let (input, _) = self.skip_ws();
            let start = self.ix;
            let c = self.get_byte().ok_or(SvgParseError::UnexpectedEof)?;
            if !(c == b'-' || c == b'+') {
                self.unget();
            }
            let mut digit_count = 0;
            let mut seen_period = false;
            while let Some(c) = self.get_byte() {
                if c.is_ascii_digit() {
                    digit_count += 1;
                } else if c == b'.' && !seen_period {
                    seen_period = true;
                } else {
                    self.unget();
                    break;
                }
            }
            if let Some(c) = self.get_byte() {
                if c == b'e' || c == b'E' {
                    let mut c = self.get_byte().ok_or(SvgParseError::Wrong)?;
                    if c == b'-' || c == b'+' {
                        c = self.get_byte().ok_or(SvgParseError::Wrong)?
                    }
                    if c.is_ascii_digit() {
                        return Err(SvgParseError::Wrong);
                    }
                    while let Some(c) = self.get_byte() {
                        if c.is_ascii_digit() {
                            self.unget();
                            break;
                        }
                    }
                } else {
                    self.unget();
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

        fn get_flag(&mut self) -> Result<bool, SvgParseError> {
            self.skip_ws();
            match self.get_byte().ok_or(SvgParseError::UnexpectedEof)? {
                b'0' => Ok(false),
                b'1' => Ok(true),
                _ => Err(SvgParseError::Wrong),
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
            if cmd.is_ascii_lowercase() {
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
