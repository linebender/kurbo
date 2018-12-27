# kurbo, a Rust 2D curves library

The kurbo library contains data structures and algorithms for curves and vector paths. It is probably most appropriate for creative tools, but is general enough it might be useful for other applications.

The name "kurbo" is Esperanto for "curve".

There is a focus on accuracy and good performance in high-accuracy conditions. Thus, the library might be useful in engineering and science contexts as well, as opposed to visual arts where rough approximations are often sufficient. Many approximate functions come with an accuracy parameter, and analytical solutions are used where they are practical. An example is area calculation, which is done using Green's theorem.

The library is still in fairly early development stages. There are traits intended to be useful for general curves (not just Béziers), but these will probably be reorganized.

## Similar crates

Here we mention a few other curves libraries and touch on some of the decisions made differently here.

* [lyon_geom] has a lot of very good vector algorithms. It's most focused on rendering.

* [flo_curves] has good Bézier primitives, and seems tuned for animation. It's generic on the coordinate type, while we use `f64` for everything.

* [vek] has both 2D and 3D Béziers among other things, and is tuned for game engines.

## More info

To learn more about Bézier curves, [A Primer on Bézier Curves] by Pomax is indispensable.

## Contributing

Contributions are welcome. The [Rust Code of Conduct] applies.

[Rust Code of Conduct]: https://www.rust-lang.org/policies/code-of-conduct
[lyon_geom]: https://crates.io/crates/lyon_geom
[flo_curves]: https://crates.io/crates/flo_curves
[vek]: https://crates.io/crates/vek
[A Primer on Bézier Curves]: https://pomax.github.io/bezierinfo/
