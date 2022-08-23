// DO NOT COMMIT THIS FILE

use std::{ops, fmt};

pub use crate::{Point, Vec2, Line};

/*****************************************************************************
 * Struct
 *****************************************************************************/

/// A struct that defines a fat line
#[derive(Debug, Copy, Clone)]
pub struct FatLine
{
    /// The line's start point.
    pub p0: Point,
    /// The line's end point.
    pub p1: Point,
    /// The min distance from the center-line (zero or less)
    pub dmin: f64,
    /// The max distance from the center-line (zero or more)
    pub dmax: f64,
}

/*****************************************************************************
 * Implementation
 *****************************************************************************/
 
impl FatLine
{
    /// Gives the distance from a point to the center line
    fn distance_from_line(line: &Line, pt: &Point) -> f64
    {
        // Calculate line parameters
        let mut a = line.p1.y - line.p0.y;
        let mut b = line.p0.x - line.p1.x;
        let mut c = line.p0.x * -line.p1.y + line.p0.y * line.p1.x;
        let length = (a * a + b * b).sqrt();
        
        // Normalize parameters
        a /= length;
        b /= length;
        c /= length;
        
        // Calculate distance
        a * pt.x + b * pt.y + c
    }
    
    /// Creates a fat line from a control polygon
    pub fn from_control_poly(control_poly: &Vec<Point>) -> FatLine
    {
        // Get start/end points of fat line
        let line = Line {
            p0: control_poly.first().unwrap().clone(),
            p1: control_poly.last().unwrap().clone(),
            };
            
        // Determine min/max distances
        let mut mind: f64 = 0.;
        let mut maxd: f64 = 0.;
        println!("test poly: {}", control_poly.len());
        for pt in control_poly
        {
            let d = FatLine::distance_from_line(&line, pt);
            mind = mind.min(d);
            maxd = maxd.max(d);
            
            println!("test dist: {} -> {} {}", d, mind, maxd);
        }
        
        FatLine {
            p0: line.p0,
            p1: line.p1,
            dmin: mind,
            dmax: maxd
            }
    }
}

/*****************************************************************************
 * Ops
 *****************************************************************************/

impl ops::Add<Vec2> for FatLine
{
    type Output = Self;

    /// Adds a Vec2 to a fat line
    fn add(self, other: Vec2) -> Self::Output
    {
        FatLine {
            p0: self.p0 + other,
            p1: self.p1 + other,
            dmin: self.dmin,
            dmax: self.dmax,
            }
    }
}

/*****************************************************************************
 * Format
 *****************************************************************************/

impl fmt::Display for FatLine
{
    /// Defines how fat lines are printed to the output
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result
    {
        write!(f, "Fattie({}, {}, [{}, {}])", self.p0, self.p1, self.dmin, self.dmax)
    }
}