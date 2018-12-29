//! SVG path representation.

use std::io::Write;

use crate::{BezPath, PathEl, Vec2};

impl BezPath {
    /// Convert the path to an SVG path string representation.
    ///
    /// The current implementation doesn't take any special care to produce a
    /// short string (reducing precision, using relative movement).
    pub fn to_svg(&self) -> String {
        let mut result = Vec::new();
        for el in self.elements() {
            match *el {
                PathEl::Moveto(p) => write!(result, "M{} {}", p.x, p.y).unwrap(),
                PathEl::Lineto(p) => write!(result, "L{} {}", p.x, p.y).unwrap(),
                PathEl::Quadto(p1, p2) => {
                    write!(result, "Q{} {} {} {}", p1.x, p1.y, p2.x, p2.y).unwrap()
                }
                PathEl::Curveto(p1, p2, p3) => write!(
                    result,
                    "C{} {} {} {} {} {}",
                    p1.x, p1.y, p2.x, p2.y, p3.x, p3.y
                )
                .unwrap(),
                PathEl::Closepath => write!(result, "Z").unwrap(),
            }
        }
        String::from_utf8(result).unwrap()
    }

    pub fn from_svg(data: &str) -> Result<BezPath, SvgParseError> {
        let mut lexer = SvgLexer::new(data);
        let mut path = BezPath::new();
        let mut last_cmd = 0;
        while let Some(c) = lexer.get_cmd(last_cmd) {
            if c == b'm' || c == b'M' {
                let pt = lexer.get_number_pair()?;
                path.moveto(pt);
                lexer.last_pt = pt;
                last_cmd = c - (b'M' - b'L');
            } else if c == b'l' || c == b'L' {
                let pt = lexer.get_maybe_relative(c)?;
                path.lineto(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'c' || c == b'C' {
                let p1 = lexer.get_maybe_relative(c)?;
                let p2 = lexer.get_maybe_relative(c)?;
                let p3 = lexer.get_maybe_relative(c)?;
                path.curveto(p1, p2, p3);
                lexer.last_pt = p3;
                last_cmd = c;
            } else if c == b'z' || c == b'Z' {
                path.closepath();
                // TODO: implicit moveto
            }
        }
        Ok(path)
    }
}

#[derive(Debug)]
pub enum SvgParseError {
    Wrong,
    UnexpectedEof,
}

struct SvgLexer<'a> {
    data: &'a str,
    ix: usize,
    pub last_pt: Vec2,
}

impl<'a> SvgLexer<'a> {
    fn new(data: &str) -> SvgLexer {
        SvgLexer {
            data,
            ix: 0,
            last_pt: Vec2::new(0.0, 0.0),
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

    fn get_number_pair(&mut self) -> Result<Vec2, SvgParseError> {
        let x = self.get_number()?;
        self.opt_comma();
        let y = self.get_number()?;
        self.opt_comma();
        Ok(Vec2::new(x, y))
    }

    fn get_maybe_relative(&mut self, cmd: u8) -> Result<Vec2, SvgParseError> {
        let pt = self.get_number_pair()?;
        if cmd >= b'a' && cmd <= b'z' {
            Ok(pt + self.last_pt)
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

#[cfg(test)]
mod tests {
    use crate::BezPath;

    #[test]
    fn test_parse_svg() {
        let path = BezPath::from_svg("m10 10 100 0 0 100 -100 0z").unwrap();
        assert_eq!(path.segments().count(), 4);
    }
}
