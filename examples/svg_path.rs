use kurbo::{
    paths::svg::{self, OneVec},
    Affine, Line, Point, Size,
};
use snog::{
    peniko::{Color, Stroke},
    App, RenderCtx,
};

fn main() {
    let data = Data::new();
    App::new_with_data(data).with_render(render).run()
}

// TODO both the lifetime of the RenderCtx and the ref to user data could be the same - nothing is
// gained by having one longer than the other.
fn render(data: &mut Data, mut ctx: RenderCtx<'_>) {
    let Size { width, height } = ctx.screen().size();

    let stroke = Stroke::new(0.005);
    let scale = Affine::scale_non_uniform(width, height);
    let brush = Color::WHITE;
    ctx.stroke(&stroke, scale, &brush, None, &data.path);
}

struct Data {
    path: svg::Path,
}

impl Data {
    fn new() -> Self {
        let path = svg::Path::try_from(vec![
            svg::PathEl::MoveTo(OneVec::single(Point::new(0.1, 0.1))),
            svg::PathEl::Bearing(std::f64::consts::FRAC_PI_2),
            svg::PathEl::LineToRel(
                OneVec::try_from(vec![Point::new(0.2, 0.0), Point::new(-0.1, -0.2)]).unwrap(),
            ),
            svg::PathEl::VertRel(OneVec::single(-0.2)),
            svg::PathEl::CubicToRel(OneVec::single(svg::CubicTo {
                to: Point::new(0.2, 0.0),
                ctrl1: Point::new(0.1, 0.1),
                ctrl2: Point::new(0.1, -0.1),
            })),
            svg::PathEl::SmoothCubicToRel(
                OneVec::try_from(vec![
                    svg::SmoothCubicTo {
                        to: Point::new(0.2, 0.0),
                        ctrl2: Point::new(0.1, -0.1),
                    },
                    svg::SmoothCubicTo {
                        to: Point::new(0.2, 0.0),
                        ctrl2: Point::new(0.1, -0.1),
                    },
                ])
                .unwrap(),
            ),
            svg::PathEl::QuadToRel(OneVec::single(svg::QuadTo {
                to: Point::new(0.0, 0.1),
                ctrl: Point::new(0.1, 0.1),
            })),
            svg::PathEl::SmoothQuadToRel(
                OneVec::try_from(vec![Point::new(0.0, 0.1), Point::new(0.0, 0.1)]).unwrap(),
            ),
        ])
        .unwrap();

        Data { path }
    }
}
