<!-- Instructions

This changelog follows the patterns described here: <https://keepachangelog.com/en/>.

Subheadings to categorize changes are `added, changed, deprecated, removed, fixed, security`.

-->

# Changelog

The latest published Kurbo release is [0.11.1](#0110-2024-09-12) which was released on 2024-09-12.
You can find its changes [documented below](#0111-2024-09-12).

## [Unreleased]

This release has an [MSRV][] of 1.74.

### Changed

- `Stroke` is now `PartialEq`, `StrokeOpts` is now `Clone`, `Copy`, `Debug`, `Eq`, `PartialEq`. ([#379] by [@waywardmonkeys])

## [0.11.1][] (2024-09-12)

This release has an [MSRV][] of 1.65.

### Added

- Add `From (f32, f32)` for `Point`. ([#339] by [@rsheeter])
- Add `Rect::overlaps` and `Rect::contains_rect`. ([#347] by [@nils-mathieu])
- Add `CubicBez::tangents` ([#288] by [@raphlinus])
- Add `Arc::reversed`. ([#367] by [@waywardmonkeys])
- Add `CircleSegment::inner_arc` and `CircleSegment::outer_arc` ([#368] by [@waywardmonkeys])
- Add `Rect::is_zero_area` and `Size::is_zero_area` and deprecate their `is_empty` methods. ([#370] by [@waywardmonkeys])
- Add `Line::reversed` and `Line::midpoint`. ([#375] by [@waywardmonkeys])
- Allow construction of `Line` from `(Point, Point)` and `(Point, Vec2)`. ([#376] by [@waywardmonkeys])

### Changed

- Move `Self: Sized` bound from `Shape` to methods. ([#340] by [@waywardmonkeys])
- Enable partial SVG path support in `no_std` builds. ([#356] by [@waywardmonkeys])
- Deprecate `BezPath::flatten`, prefer `flatten`. ([#361] by [@waywardmonkeys])

### Fixed

- An edge case in `mindist` was fixed. ([#334] by [@platlas])
- Allow lines in simplify input. ([#343] by [@raphlinus])
- Don't skip first dash in dash pattern. ([#353] by [@dominikh])
- Documentation for `Arc.perimeter` was corrected. ([#354] by [@simoncozens])
- Parsing scientific notation in an SVG path was fixed. ([#365] by [@GabrielDertoni])

## [0.11.0][] (2024-02-14)

This release has an [MSRV][] of 1.65.

Note: A changelog was not kept for or before this release

[@dominikh]: https://github.com/dominikh
[@GabrielDertoni]: https://github.com/GabrielDertoni
[@nils-mathieu]: https://github.com/nils-mathieu
[@platlas]: https://github.com/platlas
[@raphlinus]: https://github.com/raphlinus
[@rsheeter]: https://github.com/rsheeter
[@simoncozens]: https://github.com/simoncozens
[@waywardmonkeys]: https://github.com/waywardmonkeys

[#288]: https://github.com/linebender/kurbo/pull/288
[#334]: https://github.com/linebender/kurbo/pull/334
[#339]: https://github.com/linebender/kurbo/pull/339
[#340]: https://github.com/linebender/kurbo/pull/340
[#343]: https://github.com/linebender/kurbo/pull/343
[#347]: https://github.com/linebender/kurbo/pull/347
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

[Unreleased]: https://github.com/linebender/kurbo/compare/v0.11.1...HEAD
[0.11.0]: https://github.com/linebender/kurbo/releases/tag/v0.11.0
[0.11.1]: https://github.com/linebender/kurbo/releases/tag/v0.11.1

[MSRV]: README.md#minimum-supported-rust-version-msrv
