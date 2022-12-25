use kurbo::{fit_to_bezpath, offset::CubicOffset, CubicBez, Shape};

fn main() {
    println!("<svg width='800' height='600' xmlns='http://www.w3.org/2000/svg'>");
    let c = CubicBez::new((100., 100.), (150., 75.), (300., 50.), (400., 200.));
    println!(
        "  <path d='{}' stroke='#000' fill='none' />",
        c.to_path(1e-9).to_svg()
    );
    for i in 1..=20 {
        let co = CubicOffset::new(c, i as f64 * 15.0);
        let path = fit_to_bezpath(&co, 1.0);
        println!("  <path d='{}' stroke='#008' fill='none' />", path.to_svg());
    }
    println!("</svg>");
}
