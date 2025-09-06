# poly-cool

**Polynomial root-finding in Rust**

[![dependency status](https://deps.rs/repo/github/linebender/poly-cool/status.svg)](https://deps.rs/repo/github/linebender/poly-cool)
[![Apache 2.0 or MIT license.](https://img.shields.io/badge/license-Apache--2.0_OR_MIT-blue.svg)](#license)
[![Build status](https://github.com/linebender/kurbo/workflows/CI/badge.svg)](https://github.com/linebender/kurbo/actions)
[![Crates.io](https://img.shields.io/crates/v/poly-cool.svg)](https://crates.io/crates/poly-cool)
[![Docs](https://docs.rs/poly-cool/badge.svg)](https://docs.rs/poly-cool)

A small rust crate for numerically finding roots of low-degree polynomials.

```rust
use poly_cool::Poly;

// The polynomial x^3 - 6x^2 + 11x - 6
let p = Poly::new([-6.0, 11.0, -6.0, 1.0]);

dbg!(p.roots_between(-10.0, 10.0, 1e-6));
// [0.9999999999999996, 2.0000000000000018, 2.9999999999999982]
```

Currently, we implement [Yuksel's] iterative solver for finding roots within a
given interval to a specified target accuracy.


[Yuksel's]: https://www.cemyuksel.com/research/polynomials/
