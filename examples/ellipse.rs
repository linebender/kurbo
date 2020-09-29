//! Example of ellipse

use kurbo::{Ellipse, Shape};
use std::f64::consts::PI;

fn main() {
    let ellipse = Ellipse::new((400.0, 400.0), (200.0, 100.0), 0.25 * PI);
    println!("<!DOCTYPE html>");
    println!("<html>");
    println!("<body>");
    println!("<svg height=\"800\" width=\"800\" style=\"background-color: #999\">");
    let path = ellipse.to_path(1e-3).to_svg();
    println!("  <path d=\"{}\" stroke=\"black\" fill=\"none\" />", path);
    let path = ellipse.to_path(1.0).to_svg();
    println!("  <path d=\"{}\" stroke=\"red\" fill=\"none\" />", path);
    println!("</svg>");
    println!("</body>");
    println!("</html>");
}
