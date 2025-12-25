<!-- Instructions

This changelog follows the patterns described here: <https://keepachangelog.com/en/>.

Subheadings to categorize changes are `added, changed, deprecated, removed, fixed, security`.

-->

# Changelog

The latest published Kurbo release is [0.13.0](#0130-2025-11-27) which was released on 2025-11-27.
You can find its changes [documented below](#0130-2025-11-27).

## [Unreleased]

This release has an [MSRV][] of 1.85.

## Changed

- Improve performance of `RoundedRect::winding` and `RoundedRect::contains`. ([#534][] by [@tomcur][])

## [0.13.0] (2025-11-27)

This release has an [MSRV][] of 1.85.

## Added

- The newly added `BezPath::into_elements` method allows getting all path elements as a `Vec` without allocating, consuming the original path. This is the counterpart of `BezPath::from_vec`. ([#504][] by [@cmyr][])
- Add a derive `Hash` to the `Axis` type. ([#527][] by [@jrmoulton][])

## Changed

- Speed up methods like `Ellipse::radii` by reworking the singular value decomposition expression. ([#499][] by [@tomcur][])
- The `Line::nearest` method to calculate the projection of a point onto a line segment has been made more performant. ([#505][] by [@tomcur][])
- The `QuadBez::arclen` calculation has been made more numerically stable ([#503][] by [@jneem][]).
- `SvgParseError` now implements `core::error:Error`. It previously implemented `std::error::Error` in `std`-enabled builds. ([#517][] by [@Bombaninha][])
- The behavior of open versus closed shapes when using `BezPath::extend` (extending Bezier paths using an iterator over `PathEl`) has been clarified. [#523][] by [@DJMcNab][])
- More methods are marked `inline`. ([#506][] by [@LaurenzV][], [#509][] by [@tomcur][])

The following functions are now callable from `const` contexts. ([#510][], [#512][], [#521][], [#522][], [#524][], [#525][], [#526][] by [@tomcur][])

- `Affine::{skew, then_translate, map_unit_square, as_coeffs, determinant, inverse, translation, with_translation}`
- `Point::midpoint`
- `Rect::{width, height, origin, size, area, is_zero_area, center, aspect_ratio_width, overlap, contains_rect, union, intersect, inscribed_rect_with_aspect_ratio, inflate, scale_from_origin, abs}`
- `Size::{max_side, min_side, area, is_zero_area, min, max, clamp, aspect_ratio_width}`
- In addition, the `is_finite` and `is_nan` methods on the following types are now `const`: `Affine`, `Circle`, `ConstPoint`, `CubicBez`, `Ellipse`, `Insets`, `Line`, `Point`, `QuadBez`, `Rect`, `RoundedRect`, `RoundedRectRadii`, `Size`, `Triangle`, `Vec2`.

## [0.12.0] (2025-09-04)

This release has an [MSRV][] of 1.82.
It was increased to support floating point math in const functions.

### Added

- Newly public `StrokeCtx` allows for reusing allocations when stroking multiple paths. ([#475][] by [@LaurenzV][])
- New `Axis` type. ([#476][] by [@PoignardAzur][])
- Add `Ellipse::major_radius` and `Ellipse::minor_radius` methods. ([#497][] by [@tomcur][])

### Changed

- The implementation of stroking is much faster. ([#427][] by [@raphlinus][])
- More `Vec2` methods can now be called in `const` contexts. ([#479][] by [@tomcur][])
- `aspect_ratio` on `Rect` and `Size` has been deprecated and replaced with `aspect_ratio_width`.
  This is because the implementation of `aspect_ratio` used the ratio of height to width, whereas
  aspect rations are otherwise always ratios of width to height. ([#486][] by [@ErisianArchitect][] and [@DJMcNab][])
- Deprecated `contained_rect_with_aspect_ratio`, replaced with `inscribed_rect_with_aspect_ratio`, which
  incidentally also uses the usual definition of aspect ratio (where the old name didn't). ([#486][] by [@DJMcNab][])
- Breaking change: The deprecated `offset::CubicOffset` has been removed, and replaced by
  `offset::offset_cubic`. ([#489][] by [@jneem][])
- Several methods marked `#[inline]`. ([#472][], [#480][], [#496][] by [@tomcur][])

### Fixed

- Improved cubic to quadratic conversion handling for degenerate cubic curves with 3-4 consecutive equal control points. The approximation now correctly handles edge cases where cubics degenerate to lines or single points, matching fonttools' cu2qu behavior. ([#485][] by [@anthrotype][])
- Fix miter join in dashed strokes. ([#490][] by [@gemberg][])

### Removed

- Breaking change: `DashIterator` has been removed. Replace `DashIterator::new` with `dash`. ([#488][] by [@DJMcNab][])
- Breaking change: The previously deprecated `BezPath::flatten`, `Ellipse::[with_]x_rotation`, `{Rect, Size}::is_empty`, `Shape::[in]to_bez_path`,
  and `TranslateScale::as_tuple` have been removed. ([#487][] by [@DJMcNab][])

## [0.11.3][] (2025-07-21)

This release has an [MSRV][] of 1.65.

### Added

- Add `current_position` method to `BezPath`. ([#462][] by [@sagudev][])
- Add `From` conversions between `euclid` and `kurbo` types behind the `euclid` feature. ([#463][] by [@sagudev][])
- Add Green's theorem moments. ([#452][] by [@simoncozens][])
- Make `Vec2::splat` public. ([#469][] by [@xorgy][])

### Changed

- Let more methods take `Into<Point>` instead of `Point`. ([#466][] by [@sagudev][])

  **Note**: this can impact type inference.
- Inline the `Mul<PathEl>` implementation for `Affine`. ([#461][] by [@LaurenzV][])
- Inline the `next` method of the `ToQuads` iterator. ([#460][] by [@LaurenzV][])
- Avoid calling `f64::hypot`, as it calls a slow library function. ([#448][] by [@beholdnec][], [#451][] by [@raphlinus][])

### Fixed

- Documentation of `RoundedRect` no longer incorrectly specifies the corner radii are equal. ([#447][] by [@tomcur][])
- Fixed negative dash offset by normalization. ([#454][] by [@sagudev][])
- Use exact endpoints for `PathSeg`. ([#465][] by [@jneem])

## [0.11.2][] (2025-04-28)

This release has an [MSRV][] of 1.65.

### Added

- `Stroke` is now `PartialEq`, `StrokeOpts` is now `Clone`, `Copy`, `Debug`, `Eq`, `PartialEq`. ([#379][] by [@waywardmonkeys][])
- Implement `Sum` for `Vec2`. ([#399][] by [@Philipp-M][])
- Add triangle shape. ([#350][] by [@juliapaci][], [#387][] by [@tomcur][])
- Implement `Div<f64>` and `Mul<f64>` for `Insets`. ([#384][] by [@liferooter][])
- Add `Vec2::turn_90` and `Vec2::rotate_scale` methods ([#409][] by [@raphlinus][])
- Add `min` and `max` methods to `Size`. ([#412][] by [@nils-mathieu][])
- Add an `INFNITY` constant to `Size`. ([#413][] by [@nils-mathieu][])
- Add `BezPath::with_capacity` method ([#418][] by [@LaurenzV][])
- Add  `Affine::scale_about` and `Affine::then_scale_about`. ([#429][] by [@xorgy][])

### Changed

- Reduce number of operations in `Triangle::circumscribed_circle`. ([#390][] by [@tomcur][])
- Numerically approximate ellipse perimeter. ([#383][], [#407][] by [@tomcur][])
- Always inline trivial casts, splats, and swizzles. ([#428][] by [@xorgy][])

### Fixed

- Fix documentation of `Affine::svd`. ([#388][] by [@tomcur][])
- Fix documentation of cross product. ([#409][] by [@raphlinus][])

## [0.11.1][] (2024-09-12)

This release has an [MSRV][] of 1.65.

### Added

- Add `From (f32, f32)` for `Point`. ([#339][] by [@rsheeter][])
- Add `Rect::overlaps` and `Rect::contains_rect`. ([#347][] by [@nils-mathieu][])
- Add `CubicBez::tangents` ([#288][] by [@raphlinus][])
- Add `Arc::reversed`. ([#367][] by [@waywardmonkeys][])
- Add `CircleSegment::inner_arc` and `CircleSegment::outer_arc` ([#368][] by [@waywardmonkeys][])
- Add `Rect::is_zero_area` and `Size::is_zero_area` and deprecate their `is_empty` methods. ([#370][] by [@waywardmonkeys][])
- Add `Line::reversed` and `Line::midpoint`. ([#375][] by [@waywardmonkeys][])
- Allow construction of `Line` from `(Point, Point)` and `(Point, Vec2)`. ([#376][] by [@waywardmonkeys][])

### Changed

- Move `Self: Sized` bound from `Shape` to methods. ([#340][] by [@waywardmonkeys][])
- Enable partial SVG path support in `no_std` builds. ([#356][] by [@waywardmonkeys][])
- Deprecate `BezPath::flatten`, prefer `flatten`. ([#361][] by [@waywardmonkeys][])

### Fixed

- An edge case in `mindist` was fixed. ([#334][] by [@platlas][])
- Allow lines in simplify input. ([#343][] by [@raphlinus][])
- Don't skip first dash in dash pattern. ([#353][] by [@dominikh][])
- Documentation for `Arc.perimeter` was corrected. ([#354][] by [@simoncozens][])
- Parsing scientific notation in an SVG path was fixed. ([#365][] by [@GabrielDertoni][])

## [0.11.0][] (2024-02-14)

This release has an [MSRV][] of 1.65.

Note: A changelog was not kept for or before this release

[@anthrotype]: https://github.com/anthrotype
[@beholdnec]: https://github.com/beholdnec
[@Bombaninha]: https://github.com/Bombaninha
[@cmyr]: https://github.com/cmyr
[@DJMcNab]: https://github.com/DJMcNab
[@dominikh]: https://github.com/dominikh
[@ErisianArchitect]: https://github.com/ErisianArchitect
[@GabrielDertoni]: https://github.com/GabrielDertoni
[@gemberg]: https://github.com/gemberg
[@jneem]: https://github.com/jneem
[@jrmoulton]: https://github.com/jrmoulton
[@juliapaci]: https://github.com/juliapaci
[@LaurenzV]: https://github.com/LaurenzV
[@liferooter]: https://github.com/liferooter
[@nils-mathieu]: https://github.com/nils-mathieu
[@Philipp-M]: https://github.com/Philipp-M
[@platlas]: https://github.com/platlas
[@PoignardAzur]: https://github.com/PoignardAzur
[@raphlinus]: https://github.com/raphlinus
[@rsheeter]: https://github.com/rsheeter
[@sagudev]: https://github.com/sagudev
[@simoncozens]: https://github.com/simoncozens
[@tomcur]: https://github.com/tomcur
[@waywardmonkeys]: https://github.com/waywardmonkeys
[@xorgy]: https://github.com/xorgy

[#288]: https://github.com/linebender/kurbo/pull/288
[#334]: https://github.com/linebender/kurbo/pull/334
[#339]: https://github.com/linebender/kurbo/pull/339
[#340]: https://github.com/linebender/kurbo/pull/340
[#343]: https://github.com/linebender/kurbo/pull/343
[#347]: https://github.com/linebender/kurbo/pull/347
[#350]: https://github.com/linebender/kurbo/pull/350
[#353]: https://github.com/linebender/kurbo/pull/353
[#354]: https://github.com/linebender/kurbo/pull/354
[#356]: https://github.com/linebender/kurbo/pull/356
[#361]: https://github.com/linebender/kurbo/pull/361
[#365]: https://github.com/linebender/kurbo/pull/365
[#367]: https://github.com/linebender/kurbo/pull/367
[#368]: https://github.com/linebender/kurbo/pull/368
[#370]: https://github.com/linebender/kurbo/pull/370
[#375]: https://github.com/linebender/kurbo/pull/375
[#376]: https://github.com/linebender/kurbo/pull/376
[#379]: https://github.com/linebender/kurbo/pull/379
[#383]: https://github.com/linebender/kurbo/pull/383
[#384]: https://github.com/linebender/kurbo/pull/384
[#387]: https://github.com/linebender/kurbo/pull/387
[#388]: https://github.com/linebender/kurbo/pull/388
[#390]: https://github.com/linebender/kurbo/pull/390
[#399]: https://github.com/linebender/kurbo/pull/399
[#407]: https://github.com/linebender/kurbo/pull/407
[#409]: https://github.com/linebender/kurbo/pull/409
[#412]: https://github.com/linebender/kurbo/pull/412
[#413]: https://github.com/linebender/kurbo/pull/413
[#418]: https://github.com/linebender/kurbo/pull/418
[#427]: https://github.com/linebender/kurbo/pull/427
[#428]: https://github.com/linebender/kurbo/pull/428
[#429]: https://github.com/linebender/kurbo/pull/429
[#447]: https://github.com/linebender/kurbo/pull/447
[#448]: https://github.com/linebender/kurbo/pull/448
[#451]: https://github.com/linebender/kurbo/pull/451
[#452]: https://github.com/linebender/kurbo/pull/452
[#454]: https://github.com/linebender/kurbo/pull/454
[#460]: https://github.com/linebender/kurbo/pull/460
[#461]: https://github.com/linebender/kurbo/pull/461
[#462]: https://github.com/linebender/kurbo/pull/462
[#463]: https://github.com/linebender/kurbo/pull/463
[#465]: https://github.com/linebender/kurbo/pull/465
[#466]: https://github.com/linebender/kurbo/pull/466
[#469]: https://github.com/linebender/kurbo/pull/469
[#472]: https://github.com/linebender/kurbo/pull/472
[#475]: https://github.com/linebender/kurbo/pull/475
[#476]: https://github.com/linebender/kurbo/pull/476
[#479]: https://github.com/linebender/kurbo/pull/479
[#480]: https://github.com/linebender/kurbo/pull/480
[#485]: https://github.com/linebender/kurbo/pull/485
[#486]: https://github.com/linebender/kurbo/pull/486
[#487]: https://github.com/linebender/kurbo/pull/487
[#488]: https://github.com/linebender/kurbo/pull/488
[#489]: https://github.com/linebender/kurbo/pull/489
[#490]: https://github.com/linebender/kurbo/pull/490
[#496]: https://github.com/linebender/kurbo/pull/496
[#497]: https://github.com/linebender/kurbo/pull/497
[#499]: https://github.com/linebender/kurbo/pull/499
[#503]: https://github.com/linebender/kurbo/pull/503
[#504]: https://github.com/linebender/kurbo/pull/504
[#505]: https://github.com/linebender/kurbo/pull/505
[#506]: https://github.com/linebender/kurbo/pull/506
[#509]: https://github.com/linebender/kurbo/pull/509
[#510]: https://github.com/linebender/kurbo/pull/510
[#512]: https://github.com/linebender/kurbo/pull/512
[#517]: https://github.com/linebender/kurbo/pull/517
[#521]: https://github.com/linebender/kurbo/pull/521
[#522]: https://github.com/linebender/kurbo/pull/522
[#523]: https://github.com/linebender/kurbo/pull/523
[#524]: https://github.com/linebender/kurbo/pull/524
[#525]: https://github.com/linebender/kurbo/pull/525
[#526]: https://github.com/linebender/kurbo/pull/526
[#527]: https://github.com/linebender/kurbo/pull/527
[#534]: https://github.com/linebender/kurbo/pull/534

[Unreleased]: https://github.com/linebender/kurbo/compare/v0.13.0...HEAD
[0.13.0]: https://github.com/linebender/kurbo/releases/tag/v0.13.0
[0.12.0]: https://github.com/linebender/kurbo/releases/tag/v0.12.0
[0.11.3]: https://github.com/linebender/kurbo/releases/tag/v0.11.3
[0.11.2]: https://github.com/linebender/kurbo/releases/tag/v0.11.2
[0.11.1]: https://github.com/linebender/kurbo/releases/tag/v0.11.1
[0.11.0]: https://github.com/linebender/kurbo/releases/tag/v0.11.0

[MSRV]: README.md#minimum-supported-rust-version-msrv
