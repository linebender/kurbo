// Copyright 2019 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Example of circle

use kurbo::{Circle, Shape};

fn main() {
    let circle = Circle::new((400.0, 400.0), 380.0);
    println!("<!DOCTYPE html>");
    println!("<html>");
    println!("<body>");
    println!("<svg height=\"800\" width=\"800\">");
    let path = circle.to_path(1e-3).to_svg();
    println!("  <path d=\"{path}\" stroke=\"black\" fill=\"none\" />");
    let path = circle.to_path(1.0).to_svg();
    println!("  <path d=\"{path}\" stroke=\"red\" fill=\"none\" />");
    println!("</svg>");
    println!("</body>");
    println!("</html>");
}
