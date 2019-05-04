//! SVG path representation.

use std::f64::consts::PI;
use std::io::Write;

use crate::{BezPath, PathEl, Vec2};

// Note: the SVG arc logic is heavily adapted from https://github.com/nical/lyon
struct SvgArc {
    pub from: Vec2,
    pub to: Vec2,
    pub radii: Vec2,
    pub x_rotation: f64,
    pub large_arc: bool,
    pub sweep: bool,
}

// TODO: consider adding this to the public API.
struct Arc {
    pub center: Vec2,
    pub radii: Vec2,
    pub start_angle: f64,
    pub sweep_angle: f64,
    pub x_rotation: f64,
}

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
        let mut last_ctrl = None;
        while let Some(c) = lexer.get_cmd(last_cmd) {
            if c == b'm' || c == b'M' {
                let pt = lexer.get_maybe_relative(c)?;
                path.moveto(pt);
                lexer.last_pt = pt;
                last_ctrl = Some(pt);
                last_cmd = c - (b'M' - b'L');
            } else if c == b'l' || c == b'L' {
                let pt = lexer.get_maybe_relative(c)?;
                path.lineto(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'h' || c == b'H' {
                let mut x = lexer.get_number()?;
                lexer.opt_comma();
                if c == b'h' {
                    x += lexer.last_pt.x;
                }
                let pt = Vec2::new(x, lexer.last_pt.y);
                path.lineto(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'v' || c == b'V' {
                let mut y = lexer.get_number()?;
                lexer.opt_comma();
                if c == b'v' {
                    y += lexer.last_pt.y;
                }
                let pt = Vec2::new(lexer.last_pt.x, y);
                path.lineto(pt);
                lexer.last_pt = pt;
                last_cmd = c;
            } else if c == b'c' || c == b'C' {
                let p1 = lexer.get_maybe_relative(c)?;
                let p2 = lexer.get_maybe_relative(c)?;
                let p3 = lexer.get_maybe_relative(c)?;
                path.curveto(p1, p2, p3);
                last_ctrl = Some(p2);
                lexer.last_pt = p3;
                last_cmd = c;
            } else if c == b's' || c == b'S' {
                let p1 = match last_ctrl {
                    Some(ctrl) => 2.0 * lexer.last_pt - ctrl,
                    None => lexer.last_pt,
                };
                let p2 = lexer.get_maybe_relative(c)?;
                let p3 = lexer.get_maybe_relative(c)?;
                path.curveto(p1, p2, p3);
                last_ctrl = Some(p2);
                lexer.last_pt = p3;
                last_cmd = c;
            } else if c == b'a' || c == b'A' {
                let radii = lexer.get_number_pair()?;
                let x_rotation = lexer.get_number()?;
                lexer.opt_comma();
                let large_arc = lexer.get_number()?;
                lexer.opt_comma();
                let sweep = lexer.get_number()?;
                lexer.opt_comma();
                let p = lexer.get_maybe_relative(c)?;
                let svg_arc = SvgArc {
                    from: lexer.last_pt,
                    to: p,
                    radii,
                    x_rotation,
                    large_arc: large_arc != 0.0,
                    sweep: sweep != 0.0,
                };
                let arc = Arc::from_svg_arc(&svg_arc);
                arc.append_to_path(&mut path, 0.1);
                lexer.last_pt = p;
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

impl Arc {
    fn from_svg_arc(arc: &SvgArc) -> Arc {
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
        let center = Vec2::new(
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

        Arc {
            center,
            radii: Vec2::new(rx, ry),
            start_angle,
            sweep_angle: sweep_angle,
            x_rotation: arc.x_rotation,
        }
    }

    fn get_angle(&self, t: f64) -> f64 {
        self.start_angle + t * self.sweep_angle
    }

    fn append_to_path(&self, path: &mut BezPath, _tolerance: f64) {
        // TODO: it's cheesy to do linear flattening, figure out curve math
        let n = 5;
        let recip_n = (n as f64).recip();
        for i in 0..n {
            let t = (i + 1) as f64 * recip_n;
            path.lineto(
                self.center + sample_ellipse(self.radii, self.x_rotation, self.get_angle(t)),
            );
        }
    }
}

fn sample_ellipse(radii: Vec2, x_rotation: f64, angle: f64) -> Vec2 {
    let u = radii.x * angle.cos();
    let v = radii.y * angle.sin();
    Vec2::new(
        u * x_rotation.cos() - v * x_rotation.sin(),
        u * x_rotation.sin() + v * x_rotation.cos(),
    )
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
