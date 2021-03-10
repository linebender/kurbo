//! Example of circle
use kurbo::{Circle, Shape};

fn main() {
    let circle = Circle::new((400.0, 400.0), 380.0);
    println!("<!DOCTYPE html>");
    println!("<html>");
    println!("<body>");
    println!("<svg height=\"800\" width=\"800\">");
    let path = circle.to_path(1e-3).to_svg();
    println!("  <path d=\"{}\" stroke=\"black\" fill=\"none\" />", path);
    let path = circle.to_path(1.0).to_svg();
    println!("  <path d=\"{}\" stroke=\"red\" fill=\"none\" />", path);
    println!("</svg>");
    println!("</body>");
    println!("</html>");
}
