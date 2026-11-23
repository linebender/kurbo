# Notes on `offset2.rs`

These notes record the reasoning, measurements and open questions behind
`kurbo/src/offset2.rs`, so that work can continue without the conversation
that produced it. They were written in November 2026 against kurbo commit
`a5f97ff` (0.13.1).

The first sections, up to "Code that is not in the module", date from the
web session; timings there are from a 2.1 GHz Xeon sandbox. Later sections
are dated and use an M5 Max. Accuracy figures are from random cubics with
control points in a 100 unit box, unless stated.

## Open items

The one current list. Each points to its section below.

Small:

1. **Newton cusp solve and depth 12 in `offset.rs`**, on the
   `offset-split-at-end` branch, so that benchmarks compare bounds and
   fits, not solvers and depth limits.
2. (Done: midpoint subdivision everywhere; see "Trimming again".)

Medium:

3. **Certifying each half separately**: 2 to 5 more points of
   certification; port from the original's `split` option, with
   remainders per half ("The original certificate, compared").
4. (Decided: strict mode and the pair both stay; see "Trimming again".)
5. **Both sides in one pass.** `offset_cubic_pair` matches two one-sided
   calls exactly; whole tiger strokes are 28 to 31% slower without it
   (one side at a time against the pair). Since "The
   tiger at 1/32" one pass over the osculating samples decides both
   sides. Sharing the sampled estimate's evaluations was tried and made
   it slower, and sharing the Taylor coefficients and certificate saved
   nothing measurable ("Both sides at once", "Trimming").
6. **Another corpus**, with ordinary end handles, to see how much of the
   tiger's behaviour is the tiger's, and to check the guards on the
   osculating estimate ("The osculating parabola estimate").

Large, or open questions:

7. A certificate that allows `beta` to vanish at an end, for zero end
   derivatives. Possibly not worth a special case.
8. (Dropped with the G1 bound: the mirrored certificate for it.)
9. Strong correctness: locate every cusp, report reversals, and add
    evolute sections, which need their own fit and bound. Hidden pairs
    of cusps are now found where a sample of the osculating estimate
    lands between them ("Where the time goes").
10. **Vanishing end derivatives**: the cusp value and the clamped fit are
    fixed in the default mode ("Vanishing end derivatives"). Left: strict
    mode never certifies a piece touching such an end (`beta(0) = 0`; item
    7), and `PathSeg::tangents` uses an absolute threshold for a zero
    handle, kurbo-wide.
11. **Hidden pairs of cusps in strict mode**, which has no samples to find
    them; see the end of "Vanishing end derivatives".
12. **Tolerances of 1e-6 of the curve's size at near cusps**: a few
    hundred pieces still reach the maximum depth there, and the default
    mode is up to 1.24 times over; everything looser is clean ("Tips at the
    root").
13. **A sharp turn of width 0.03 to 0.1**: the samples of the estimates
    still straddle it. Of 2,400 offsets of random curves with such a turn,
    at a tolerance of half the distance, 8 are over, the worst at 1.44
    times (1.96 on one curve of the survey); at a tolerance of 0.01, 2,
    the worst at 1.43 ("Trimming again"). Splitting them at the root as
    the narrower ones are costs 3 to 5% on the tiger, which has 31 of them
    in 336 curves ("Sharp turns").
14. **The default one-sided call is 2 to 5% slower** than before the tips
    on the cheapest curves: code layout, not work ("Tips at the root").
    Since "The tiger at 1/32" it is another 5 to 7% slower on nearly
    straight and gentle random curves (42 to 45 ns, 72 to 77), 4% faster
    on wild ones, and 2% slower through the stroker at 1/256 and 1/2048.
    Not found: the original loop of the estimate, the estimate out of
    line, and the arc test at the root only each made no difference.
15. **Slivers at a pair of cusps.** A piece split at its middle between
    two cusps of a small swallowtail leaves a sliver each side of the
    split, and four pieces where three would do ("The tiger at 1/32").

Done or dropped: the midpoint G1 retry and the G1 bound (removed, see
"Trimming"), the slip guard on two-point fits only, and hidden
cusp pairs found at the samples (both adopted), the osculating circle
(dropped), deciding both sides in one pass (dropped, then done another
way; see "The tiger at 1/32"), Newton against ITP for the
cusp (measured, kept), the rounding allowance (added), a cheap estimate for the
default mode (the osculating
parabola, adopted with guards), misses at the maximum depth (depth raised from 8 to 12),
the near-line slowdown (did not reproduce), the end subdivision bug (fixed
here, and in `offset.rs` on `offset-split-at-end`), the error oracle (now
Fréchet), the `kappa d` chooser and skip-level subdivision (tried,
dropped), tuning the estimate's safety factor (not needed), midpoint
subdivision everywhere and the sampled estimate's removal (both done, see
"Trimming again").

## What the module is

A candidate replacement for `offset.rs`. The recursion, the regularization,
the cusp finding (sign change of the cusp value, then an ITP solve) and the
choice of subdivision point are copied from `offset.rs`. What is new is how
one piece is fitted and how it is accepted.

- `offset_cubic` has the same signature as the existing function. It
  accepts a piece on a strict bound where one applies, and otherwise on a
  sampled estimate with a safety factor.
- `offset_cubic_strict` accepts on strict bounds only, and subdivides where
  none applies.

The stroker still calls `offset::offset_cubic`. With it switched to
`offset2::offset_cubic`, kurbo's whole test suite passed.

## Notation

- `C` is the source piece, `d` the signed offset distance, `n = turn_90(T)`
  the unit normal, and `O = C + d n` the offset.
- Every approximation is `A = C + D`. `W = D / d` is roughly the unit
  normal. Its control points are `n0`, `n0 + a utan0`, `n1 + b utan1`, `n1`,
  with `(a, b)` as in `offset.rs`.
- For the linear fits and for arc drawing, `W` does not depend on `d`.
- For fixed `t`, `F(s) = |A(t) - C(s)|^2 - d^2`. Its Taylor coefficients
  about `s = t` are `P = |D|^2 - d^2`, `F_s = -2 Q` with `Q = D.C'`, and
  `F_ss = beta = 2 |C'|^2 - 2 D.C''`.
- `V = rho^2 - d^2`, where `rho` is the distance from `A(t)` to its foot on
  `C`. One Newton step gives `H = P - 2 Q^2 / beta`.
- In the frame of `C` at `t`, with `W = a n + b T`:
  `H / d^2 = (a^2 - 1) - b^2 kappa d a / (1 - kappa d a)`, up to a term in
  the change of speed. `b` is the "slip", the tangential mismatch.
- The cusp value is `1 + kappa d` in kurbo's sign convention
  (`kappa = c'' x c' / |c'|^3`). The offset has a cusp where it crosses
  zero. An inflection is where it equals one. `beta` is about
  `2 |C'|^2` times the cusp value.

## Fits

| Fit | Definition | Order | Notes |
|---|---|---|---|
| Arc drawing | `W` is the standard cubic arc from `n0` to `n1` | 4 | Always defined, exact for a line. Always errs on the near side. |
| One-point | `W(1/2) = n(1/2)` | 5 | Determinant is `sin(theta)`. Slip vanishes at the midpoint. |
| Linear two-point | `W.n = 1` at `t = 1/3, 2/3` | 4 | Works with parallel end tangents. Does not control slip. |
| True two-point, midpoint G1 | nonlinear | 6 | Not in the module. 50 to 170 times more accurate than one-point at 8 to 32 pieces per curve. |

Linear two-point is order 4, not 6: its slip is pinned only at the ends, and
the slip term dominates under subdivision.

## Bounds

| Bound | Applies to | Certificate | Order in slip | Cost | Tightness seen |
|---|---|---|---|---|---|
| One-point | one-point fit | yes | 2 | about 100 ns in all | median ratio 1.01 to 1.06, p90 1.1 to 1.5 |
| G1 (FootCubic) | any G1 fit, ordinary piece | yes | 2 | about 320 ns in all | two-point: median 1.01, p90 1.16 |
| General certified | any fit | yes | 2 | small | arc drawing: median 1.6 |
| Equal-parameter | any fit, any piece | no | 1 | about 30 ns | median ratio in the hundreds to thousands |
| Hull | any fit, turn under a right angle | no | 1 in the turning | few ns after a gate | essentially `|d| theta` |
| Small piece | arc drawing, tangent turns one way | no | none | small | diameter plus `|d| epsilon` |
| Hairline | any fit, any piece | no | none | trivial | `max |D_i| + |d|`, at least `2 |d|` |

"Ratio" is bound over measured error. Zero cases were found of any bound
falling below the measured error, once the bugs listed below were fixed.

The certificate establishes that for each `t` the foot is unique, within
`delta` of `t`, and increasing in `t`. It needs `beta` to keep one sign.
`delta` is the smaller root of `K delta^2 - beta_min delta + 2 q = 0`, with
`K` a bound on `|F_sss|` from norm bounds widened by a margin, and the
margin grown by a factor 1.25, up to 8 times, until it covers `delta`.

Past a cusp `beta` is negative and the foot is a maximum of `F`. The
certificate and the one-point bound handle that case (the "mirrored"
certificate). The G1 bound returns `None` there.

## How a piece is handled

0. If `2 |d|` is within the tolerance: arc drawing with the hairline bound.
1. Root only, behind a gate on `|d| sin(theta)`: arc drawing with the hull
   bound.
2. If the chord is under the tolerance: arc drawing with the small piece
   bound.
3. Fit: linear two-point if there is an inflection, else one-point.
4. If the certificate holds: the one-point or G1 bound decides, finally.
5. If it does not: the equal-parameter bound, then the hull bound, then
   (unless strict) the osculating parabola estimate under looser guards.
   (This was a separate sampled estimate times 1.25 until "Trimming
   again".)
6. If there is no usable linear fit: arc drawing, with the general certified
   bound and then step 5.
7. If nothing accepts: subdivide, or at the depth limit emit the piece
   unchecked.

## Decisions and the reasons for them

- **One-point is the default.** It is the cheapest fit with a strict bound,
  and its slip was about 16 times smaller than the other fits'.
- **Inflection selects two-point.** With an inflection the end tangents can
  be parallel, which makes one-point singular. A test on `kappa d` was
  better in measurements (see below) but is not in the module.
- **Arc drawing is not tried first.** It passed about 40% of the time on
  random cubics, below break-even. It is used at the root for near-lines,
  for tiny pieces, and where no linear fit exists.
- **The root hull test doubles as a conditioning guard.** One-point is only
  reached when `|d| theta` exceeds the tolerance, which bounds its
  determinant below.
- **A certified bound's verdict is final.** If it says the error is over
  tolerance, the estimate is not consulted.
- **Two entry points.** Strict acceptance costs two to three times the
  segments on curves with loops and cusps, and nothing on gentle curves.

## Measurements worth keeping

End to end, old against new, 100 curves per family (segments per call, then
time):

| Family | d | Tolerance | Old | New | Strict |
|---|---|---|---|---|---|
| Nearly straight | 2 | 0.25 | 1.00, 223 ns | 1.00, 80 ns | 1.00 |
| Nearly straight | 2 | 0.01 | 1.00, 224 ns | 1.00, 359 ns | 1.00 |
| Gentle | 2 | 0.0001 | 3.00, 2520 ns | 2.67, 953 ns | 2.67 |
| Gentle | 10 | 0.0001 | 3.94, 3675 ns | 3.63, 1381 ns | 3.63 |
| Wild | 0.5 | 0.25 | 1.65, 933 ns | 2.29, 1074 ns | 5.09 |
| Wild | 2 | 0.01 | 4.53, 4065 ns | 5.48, 3588 ns | 9.46 |
| Wild | 10 | 0.0001 | 17.2, 18610 ns | 17.6, 9814 ns | 22.3 |

On gentle curves the old code was over tolerance on 1 to 4% of calls at
tight tolerance, and the new code on none. Rerun with
`cargo run --release -p kurbo --example offset_compare`.

Cost of the stages for one accepted one-point piece: `Taylor::new` 30 ns,
`Certificate::new` 45 ns, `one_point_bound` 24 ns, fit 9 ns, `apply` 8 ns.
`g1_bound` 245 ns, `sampled_estimate` 157 ns.

Choosing between one-point and two-point, by regret (chosen error over the
better error, geometric mean): always one-point 3.43, inflection only 2.57,
always two-point 2.14, inflection or `|kappa d / (1 - kappa d)| < 0.025`
at the ends 1.43. Inflection only is poor for thin strokes.

Sampled estimate (7 samples, corrected one-step kernel): ratio to the true
error at least 0.98 for one-point and 0.87 for two-point where the error is
under 1% of `|d|`; at least 0.77 where the certificate fails.

The n^6 fits with the G1 bound: strict, but loose in the tail, because the
remainder for the cubic Taylor term is the same order as the error. A level
that keeps the cubic term as `u^3 G` ("FootWeighted") beat the one-point
bound's certified error on 92% of pieces, by a median factor of 10, at
about 340 ns.

## Bugs found, and what they teach

- **Split certificate.** Combining the two halves' `delta` and `K` and then
  computing a remainder from the combination gave bounds as low as half the
  true error. Remainders must be computed within each half.
- **Cancellation.** `r^2 + 1 - 2 sqrt(r^2 - b^2)` loses eight digits. The
  equal-parameter bound is written to avoid it.
- **NaN tangent.** The cusp solver can land on an endpoint where the
  derivative vanishes. The tangent there falls back to the end tangent.
- **Reference error.** "Distance to the source, less `|d|`" is wrong where
  the curve comes near itself and near cusps. The tests measure distance to
  the true offset, sampled and refined, against the regularized curve.

All four were found by tests against measured error, not by reading.

## Performance pass on an M5 Max (18 November 2026)

The table above was rerun on an M5 Max. The anomaly behind "Not done" item
1 does not reproduce there: offset2 is faster than `offset.rs` on nearly
straight curves at every tolerance. Before this pass, the one case where it
was slower was wild curves at tolerance 0.25.

Changes, all within the existing structure:

- **Cusp solve.** ITP took about 22 evaluations per cusp, roughly 16% of
  the time on wild curves. It is replaced by Newton's method, safeguarded
  by bisection, on the cusp value times `|c'|^3`, which has the same sign
  and no pole. It takes 6 to 8 evaluations. `offset.rs` has the same ITP
  call and would gain from the same change; it was left alone so it stays
  the reference.
- **Early rejection in the sampled estimate.** The estimate's grid contains
  its sample points exactly, so one sample over the limit settles the
  rejection. Samples are taken in the order 1/4, 3/4, 1/2, then the rest.
- **Certificate margin.** The loop started at margin 0 and always needed a
  second pass. It now starts at 1.25 times `2 q_max / beta_min`, a lower
  bound on `delta`, and usually takes one pass.
- **Symmetric products** for `|D|^2` and `|C'|^2` in `Taylor::new`.

The output changes slightly. On 300 wild curves times 18 settings, the
default mode gives 42,892 segments against 42,882 before, and strict gives
63,425 against 63,413. Gentle and nearly straight curves give identical
output, bit for bit. The over-tolerance rates in `offset_compare` are
unchanged.

Time per call in ns, best of five, 100 curves, both signs of `d`:

| Family | d | Tolerance | `offset.rs` | offset2 before | after | strict before | after |
|---|---|---|---|---|---|---|---|
| Nearly straight | 2 | 0.01 | 92 | 76 | 67 | 75 | 67 |
| Nearly straight | 10 | 0.0001 | 313 | 100 | 94 | 101 | 94 |
| Gentle | 2 | 0.0001 | 851 | 301 | 283 | 301 | 283 |
| Gentle | 10 | 0.0001 | 1228 | 436 | 403 | 424 | 401 |
| Wild | 0.5 | 0.25 | 347 | 401 | 291 | 626 | 533 |
| Wild | 2 | 0.01 | 1449 | 1128 | 778 | 1276 | 1086 |
| Wild | 10 | 0.0001 | 6483 | 3271 | 2357 | 3116 | 2688 |
| Geometric mean, 27 settings | | | 345 | 208 | 174 | 219 | 194 |

After this pass the profile is flat. On wild curves, `Taylor::new` is
about 23%, `sampled_estimate` 23%, `Certificate::new` 13%, and nothing else
is above 6%. On gentle curves, `Taylor::new` is 33%, `Certificate::new` 19%
and `one_point_bound` 7%. `Taylor::new` compiles to about 300 vectorized
instructions with the binomial constants folded, so there is little left
inside it.

The larger remaining cost is rejected pieces: about half of all calls to
`approximate` on wild curves, and a third on gentle ones, end in
subdivision, and each pays for a fit, `Taylor` and the certificate. Cutting
that needs a change of structure.

One such change was tried and dropped: when a piece is rejected with an
error `e` (from a certified bound, or from the sampled estimate; the loose
bounds were not used), skip fitting the next `k` generations, with `k` the
largest such that `e / tol > margin * factor^k`. Totals over 27 settings
(segments in the default mode, geometric mean time):

| factor | margin | Wild segments | Wild ns | All ns |
|---|---|---|---|---|
| off | | 13351 | 914 | 186 |
| 8 | 4 | 17184 | 904 | 185 |
| 16 | 2 | 15099 | 872 | 182 |
| 32 | 2 | 14059 | 872 | 182 |
| 32 | 16 | 13547 | 893 | 184 |
| 32 | 256 | 13382 | 911 | 186 |

At best it saves about 2% of the time, and only by adding segments. Even a
piece rejected at 500 times the tolerance often has a child that is
accepted, because the error tends to sit in one spot and a single
subdivision isolates it. Strict mode gains nothing, since it has no
estimate where the certificate fails.

## The Ghostscript tiger (18 November 2026)

`cargo run --release --example offset_tiger -- path/to/Ghostscript_Tiger.svg`
offsets every curve of every stroked path in the 200 by 200 tiger from
Vello's assets, by half the stroke width, on both sides. That is 336
cubics, 672 calls. The tiger is usually rendered at 8x, so a quarter pixel
is a tolerance of 1/32 in source coordinates. Half widths run from 0.0075
to 0.196, mostly below that tolerance, so two tighter tolerances are
included as well.

261 of the 336 cubics (78%) have a control point on an end point, so the
derivative vanishes there. `regularize_cusp` changes none of them: it only
deals with cusps in the interior.

That exposed a bug shared with `offset.rs`. When `c'(0) = 0`, `t = 0`
solves `tan x c'(t) = 0` for every direction `tan`, so
`subdivide_for_tangent` could choose the end of the piece as the
subdivision point. Its forced fallback also picks the end where `|c'|` is
smallest, which is the same end. The result is pieces of zero length, with
cusp values `0 / 0` and arbitrary end tangents. Arc drawing turns each one
into an arc fanned about the end point. The small-piece bound accepts each
arc correctly on its own terms, but together they miss the offset. Strict
mode was up to 2.5 times over tolerance.

The fix ignores roots within `SPLIT_MARGIN` (1e-6 of the parameter range)
of either end, and replaces a subdivision point at an end with the
midpoint. The output on the random families is unchanged, bit for bit.
`strict_result_with_vanishing_end_derivative` covers it. `offset.rs` still
has the bug.

Results with the hairline bound (segments, ns per call best of five, calls
over tolerance):

| Tolerance | `offset.rs` | offset2 | strict | `offset.rs` ns | offset2 ns | strict ns | over: `offset.rs` | offset2 | strict |
|---|---|---|---|---|---|---|---|---|---|
| 1/32 | 838 | 922 | 1727 | 175 | 133 | 228 | 3 | 0 | 0 |
| 1/256 | 1959 | 1197 | 3307 | 758 | 234 | 545 | 11 | 0 | 0 |
| 1/2048 | 5215 | 1925 | 4847 | 2401 | 389 | 826 | 41 | 0 | 0 |

Before the fix, offset2 was over tolerance on 0, 10 and 33 calls, and
strict on 6, 23 and 35.

At 1/32 the default mode is 20% faster than `offset.rs` with 10% more
segments. Strict mode makes about twice the segments: with `c'` vanishing
at an end, `beta = 2 |C'|^2` vanishes there too, so the certificate fails
on every piece that touches such an end, and the uncertified bounds
subdivide toward it. Handling that end better, for instance by fitting
the end piece differently or with a bound that allows `beta` to vanish at
the end, is the obvious next step for this corpus.

### Why offset2 is only 20% faster here

Measured by stroke width at 1/32 (calls; segments for `offset.rs`, offset2,
strict; ns per call):

| Half width | Calls | Segments | ns |
|---|---|---|---|
| 0.0075 | 72 | 72, 72, 120 | 91, 84, 128 |
| 0.0195 | 204 | 204, 240, 330 | 95, 85, 112 |
| 0.022 | 104 | 104, 104, 210 | 91, 80, 168 |
| 0.087 | 40 | 40, 40, 76 | 91, 96, 156 |
| 0.196 | 252 | 418, 466, 1039 | 314, 233, 419 |

- 420 of the 672 calls have `|d|` below the tolerance. Both modules make
  one segment, and both cost about 90 ns, mostly fixed cost. The curves
  turn more than a right angle, so the root hull bound fails, and offset2
  pays for a fit, `Taylor`, a certificate that fails, and the estimate,
  only to accept the root.
- On the widest strokes, 63% of the time, only 32 of 646 pieces are
  decided by a certified bound, and 494 by the sampled estimate. With
  `p1 = p0`, `C'(0) = 0` and `C''(0)` is parallel to the tangent, so
  `beta(0) = 0` exactly and the certificate fails on every piece touching
  that end. The source handle has zero length, so the fit's handle is
  often clamped, which sent 201 pieces to arc drawing.

Lengthening a handle shorter than `tol / 4` to that length, in the
regularizer, was tried and made things worse: 488 against 466 segments on
the widest strokes, and slower. It turns infinite curvature at the end
into a very large finite one, and that stretch still needs pieces.

The hairline bound (above) was added for the first point. It takes the
0.0075 strokes from 84 to 28 ns, and the whole tiger at 1/32 from 140 to
133 ns (strict: 238 to 228, 1775 to 1727 segments). It cannot reach
0.0195 or 0.022, where `2.7 |d|` is over the tolerance. No cheap bound
that pairs hull vertices with cone directions can, since such a bound is
first order in the turning.

A vanishing end derivative might not deserve a special case. The tiger
may be unusual in having so many, so another corpus would show whether
this matters in general.

### Whole strokes

`StrokeOpts::offset2(true)`, a hidden switch, made the stroker use
offset2 (now `offset_method`; see "Both sides at once"). `cargo run --release --example stroke_tiger -- path/to/tiger.svg`
expands every stroked path with the SVG defaults (miter joins, limit 4,
butt caps), so it includes lines, joins, caps and the stroker's own check
for nearly straight cubics. Time is per whole tiger, best of five:

| Tolerance | Segments, `offset.rs` | offset2 | µs, `offset.rs` | offset2 | Ratio |
|---|---|---|---|---|---|
| 1/32 | 1629 | 1713 | 127 | 98 | 0.77 |
| 1/256 | 3046 | 2284 | 523 | 168 | 0.32 |
| 1/2048 | 6454 | 3164 | 1634 | 277 | 0.17 |

The cubic offsets are almost all of the time: 672 calls at the per-call
figures above add up to about 118 of the 127 µs at 1/32. The whole-stroke
result therefore follows the per-call one. The full test suite passes with
the stroker defaulting to offset2.

The error measure in `offset_compare`, and in the tests, used the normal
of the source at exactly `t = 0`, which is NaN there. The nearest-sample
search then stuck at index 0. The tests and `offset_tiger` now keep the
parameter 1e-9 inside the ends.

## Not done

This list was superseded by "Open items" at the top.

## The original certificate, compared (18 November 2026)

The original code is now available in the `slop1` branch of
github.com/raphlinus/fast-stroke, in `bezfit/bezfit/src/bound/cheap.rs`.
Its `certify` was ported exactly into the ignored test `certificate_rates`
and run on the same pieces as offset2's (random cubics in 2 to 16 pieces,
`|d|` of 0.3, 3 and 30):

| Certificate succeeds | One-point (6208 pieces) | Two-point (607) |
|---|---|---|
| offset2 as committed | 80.7% | 60.1% |
| offset2 with the two bounds below | 83.9% | 61.4% |
| Original, whole piece | 80.8% | 60.1% |
| Original, split in halves | 82.9% | 64.7% |
| `beta` positive on the piece or both halves (upper limit) | 89.7% | 91.8% |

So the reconstruction was not weaker. The 89 to 94% in the original notes
counts pieces where the whole bound succeeds, on its own test set, mostly at
n = 2 and above. On whole curves (n = 1) it was 45 to 81% there.

Two things from the original are now in `Certificate::new`:

- `C'.C''` in `K` is bounded by its own Bernstein coefficients, as half the
  derivative of `|C'|^2`, not by `|C'| |C''|`.
- `|Q|` is bounded with its known zeros factored out (`q_bound`):
  `Q = Q_end + u S` exactly, and for the one-point fit also
  `S = (t - 1/2) S2 + s_misfit u`. Both are exact for any `Q`, so the
  smallest of the three bounds is valid for every fit. The original's
  residual for the one-point form, `|s_misfit| / 12`, is valid but looser
  than `/16`.

Effect: strict mode makes 2 to 8% fewer segments on wild curves and the
tiger; the default mode is unchanged in segments. Each call costs about 3%
more (geometric mean 176 to 181 ns; strict 197 to 201 ns).

Splitting into halves would add another 2 to 5 points of certification.
The remainders would then have to be computed per half (see "Bugs found").

### The error oracle was wrong where the offset nears itself

The tighter certificate made `strict_result_is_within_tolerance` fail, at
7.8 times the tolerance. The bound was right; the oracle was not.
`result_error` took the nearest of 2000 samples of the offset and refined
only between its neighbours. Where the offset comes back near itself, the
nearest sample can be on the other strand, and the refinement then finds a
distance to the wrong strand. It now refines around every local minimum of
the sampled distance, in the tests, `offset_compare` and `offset_tiger`.

That changes the over-tolerance rates reported on wild curves a lot. At
tolerance 1e-4, d = 2: `offset.rs` 30% to 13%, offset2 23% to 5%, strict
24% to 5%. At d = 10: 52% to 17%, 48% to 8%, 52% to 8%. Earlier tables in
these notes used the old oracle.

The oracle is still not a Fréchet distance: it is one sided (it does not
see parts of the offset that the result skips), ignores order, and samples.
It has since been replaced; see below.

### The Fréchet oracle

`kurbo::offset_oracle` (hidden, std only) is ported from bezfit's
`frechet.rs`. For one piece, it measures the error along the normal rays
of the approximation. If the feet of the rays advance along the offset,
they give a monotone correspondence, so the largest ray error bounds the
Fréchet distance; the critical points, where the tangents are parallel,
are found and refined, so that bound is attained. If the feet do not
advance, the piece is reported as unmeasured, with a sampled lower bound,
rather than underestimated. `result_error` splits a whole result into
pieces by recovering each piece's source range from its end points, which
both modules put exactly on the offset of the regularized curve.

Changes from bezfit, each found by comparing with the sampled oracle on
the same results (`offset_profile oracle`):

- The maximum includes the coarse samples, not only the refined critical
  points. bezfit used the extrema alone when there were any, and missed a
  maximum of 0.24 next to a near cusp, reporting 0.07.
- 128 coarse rays instead of 32, for the same case: with 32 the check that
  the feet advance passed by luck.
- The foot of a ray falls back to bisection when Newton fails. Newton fails
  near a cusp of the offset, whose speed vanishes there; without the
  fallback about 1200 pieces were unmeasured on the wild family, nearly all
  ending at a cusp. With it, 33.
- Robust end tangents, where the derivative vanishes at an end, and the
  distances of the end points are included.
- Recovered parameters are snapped to a cusp of the offset when its point
  matches, since distance pins `s` down poorly there.

Against the sampled oracle on the same results (1800 calls per family and
module), the Fréchet oracle is never lower, and is higher where the
sampled oracle was blind. Calls over tolerance, sampled then Fréchet:

| Family | `offset.rs` | offset2 | strict |
|---|---|---|---|
| Nearly straight | 2, 4 | 0, 0 | 0, 0 |
| Gentle | 17, 24 | 0, 0 | 0, 0 |
| Wild | 108, 150 | 40, 41 | 42, 42 |

All 42 of strict mode's misses are in calls with a piece at the maximum
depth, which is emitted unchecked. So by this measure strict mode holds
wherever it claims to. The tests now use the oracle: the strictness tests
for the one-point, G1, crude and small-piece bounds measured all of their
pieces with it (none fell back), and so do the tests on whole results.

On the tiger, `offset.rs` is over tolerance on 6, 23 and 47 of 672 calls at
1/32, 1/256 and 1/2048 (the sampled oracle said 3, 11 and 41); offset2 and
strict on none.

### Where the default mode misses

With each piece traced to the path that accepted it and measured with the
oracle (`piece_errors`), over the wild family (1800 calls):

- 41 calls are over tolerance, and every one has a piece emitted at the
  maximum depth. Of 95 such pieces, 92 are over tolerance, the worst by
  20,000 times (2.0 at tolerance 1e-4). The approximation emitted there is
  whatever fit the last attempt made, with no check at all.
- The sampled estimate accepted 3086 pieces, one of them over tolerance,
  by 0.1%. Where it accepted and the error was above 1% of the tolerance,
  error over estimate had median 1.000, p90 1.011, p99 1.22 and maximum
  2.31, so the safety factor of 1.25 is doing its job.
- No strict bound accepted a piece over tolerance.

Nearly straight and gentle curves had no misses. So the misses are a
question of what to emit at the maximum depth, not of the estimate.

### The `kappa d` test in the chooser, tried

Using the two-point fit, with the G1 bounds, also where
`|(1 - c) / c| < threshold` for the cusp value `c` at both ends (or at
either end), in addition to where there is an inflection. Segments and
calls over tolerance (Fréchet oracle) on the three random families, all
nine settings each, and geometric mean time per call:

| Setting | Nearly | Gentle | Wild | ns, default | ns, strict |
|---|---|---|---|---|---|
| Off | 1800 / 0 | 2971 / 0 | 13319 / 41 | 186 | 200 |
| Both, 0.0125 | 1809 / 0 | 2921 / 0 | 13330 / 43 | 188 | 210 |
| Both, 0.025 | 1809 / 0 | 2915 / 0 | 13333 / 43 | 191 | 215 |
| Either, 0.025 | 1809 / 0 | 2865 / 0 | 13513 / 46 | 197 | 225 |
| Both, 0.1 | 1809 / 0 | 2930 / 0 | 13662 / 43 | 202 | 230 |
| Either, 0.2 | 1809 / 0 | 3157 / 0 | 15244 / 48 | 218 | 253 |

On the tiger, "both, 0.0125" gives 897 against 922 segments at 1/32 and
130 against 135 ns, but 1% and 2% more segments at 1/256 and 1/2048.
"Either" is worse throughout.

So the regret result above (1.43 against 2.57) does not carry over. It
compared the errors of single fits. End to end, the two-point fit has to
be accepted through the G1 bound, which costs about ten times the
one-point bound, and its certificate succeeds less often (61% against 84%
of pieces). Not adopted. It might pay with a cheaper G1 path, such as the
original's `g1_cheap_bound`.

## The midpoint G1 fit (18 November 2026)

`midpoint_g1` makes the approximation pass through the offset at `t = 1/2`
with the offset's tangent there, at a source parameter `s` that is solved
for, besides G1 at the ends. The error then has double zeros at both ends
and the midpoint, and is sixth order. It is reconstructed from bezfit's
`min_frechet.rs` (the augmented determinant, which works with parallel end
tangents), solved by Newton's method from `s = 1/2` with a numerical
derivative. The earlier version in `failed_k4` (`perturb.rs`,
`midpoint.rs`) solved the same condition through the one-point position
solve, with an analytic derivative, and stopped once the tangent mismatch
was below 3 times the tolerance.

On ordinary pieces (no cusp, no inflection) of 200 random cubics, by the
Fréchet oracle (ignored test `midpoint_g1_rates`):

| Pieces per curve | Solved | One-point error / midpoint G1, median | Certified | G1 bound / error, median, p90 |
|---|---|---|---|---|
| 1 | 83% | 1.7 | 0.5% | 1.58, 1.58 |
| 4 | 91% | 3.7 | 52% | 1.30, 8.3 |
| 8 | 92% | 8.0 | 78% | 1.17, 34 |
| 16 | 93% | 17 | 89% | 1.09, 34 |
| 32 | 93% | 37 | 92% | 1.06, 25 |

The ratio doubles per halving, so the fit is one order higher. The notes
from the web session quote 50 to 170 times at 8 to 32 pieces, which is 3
to 6 times more than this; that was either a different fit or different
curves. Newton must stop at a step of 1e-9 in `s`: at 1e-12, rounding in
the determinant made it fail on 45% of pieces at 32 per curve.

Used as a retry: where the one-point bound rejects a piece but is within
a factor of the tolerance, fit with midpoint G1 and accept on the crude
or G1 bound before subdividing. Segments and calls over tolerance on the
random families, and geometric mean time:

| Factor | Nearly | Gentle | Wild | ns, default | ns, strict |
|---|---|---|---|---|---|
| Off | 1800 / 0 | 2971 / 0 | 13319 / 41 | 178 | 197 |
| 2 | 1800 / 0 | 2769 / 0 | 12943 / 39 | 185 | 206 |
| 4 | 1800 / 0 | 2657 / 0 | 12740 / 38 | 192 | 214 |
| 8 | 1800 / 0 | 2601 / 0 | 12650 / 37 | 196 | 218 |
| 16 | 1800 / 0 | 2574 / 0 | 12594 / 37 | 200 | 222 |
| 64 | 1800 / 0 | 2564 / 0 | 12572 / 37 | 205 | 227 |

At factor 8: 12% fewer segments on gentle curves and 5% on wild ones, for
10% more time. On the tiger it does nothing (1924 to 1920 segments at
1/2048), since there the one-point certificate rarely holds and the
retry is never reached. It is behind a runtime switch, off by default.

Possible improvements: the analytic derivative from `failed_k4`, fewer
iterations (stop at a mismatch relative to the tolerance, as there), and
a retry on the estimate path for the default mode.

## The maximum depth (18 November 2026)

Every miss in either mode was a piece emitted unchecked at depth 8. Traced,
the 518 such pieces over all three modules' runs were nearly all at
tolerance 1e-4, of parameter width close to 1/256, reached mostly by
parametric halving (3662 of about 4100 steps), with cusp values in the
hundreds to tens of thousands at their ends: a near cusp of the source,
where the offset sweeps an arc of radius `|d|` over a sliver of parameter.

The test that chooses the parametric midpoint compares the sines of the
angles turned on each side of it, and sines are not monotone past a right
angle: 160 and 10 degrees pass as balanced. Comparing the angles instead
halves the pieces out of depth at depth 8, but the depth limit matters
more. Wild family, segments / calls over tolerance / pieces out of depth,
and geometric mean time over all three families:

| Setting | Default | Strict | ns, default / strict |
|---|---|---|---|
| Sines, depth 8 | 13319 / 41 / 95 | 19258 / 42 / 423 | 180 / 201 |
| Angles, depth 8 | 13426 / 32 / 37 | 19715 / 30 / 360 | 183 / 207 |
| Sines, depth 12 | 13492 / 1 / 0 | 20024 / 0 / 6 | 180 / 202 |
| Angles, depth 12 | 13479 / 4 / 0 | 20323 / 0 / 2 | 183 / 209 |
| Angles, depth 16 | 13479 / 4 / 0 | 20325 / 0 / 0 | 185 / 209 |

Nearly straight and gentle curves are the same in every row. `MAX_DEPTH`
is now 12, with the sine test kept. In `offset_compare` offset2 and strict
are now within tolerance at every setting (they were up to 8% over on wild
curves at 1e-4), for up to 9% more segments in strict mode at the tightest
settings; `offset.rs` is still 2 to 20% over there. The tiger is unchanged
except strict at 1/2048 (4679 to 4710 segments). Timing is unchanged.

The remaining default-mode miss at depth 12 was not traced; the sampled
estimate's one known miss, by 0.1%, is the likely candidate. The earlier
midpoint G1 module in `failed_k4` split at the source parameter the fit
matched, which was not tried here.

## Both sides at once (18 November 2026)

`offset_cubic_pair(c, d, tolerance, positive, negative)` and its strict
form append the offsets by `+d` and `-d` to two paths, as `curve_to`
segments only: each path must already have its current point at the start
of its offset, as a stroker's forward and backward paths do. There is no
record of reversals; that is left for the strongly correct stroker.

Design:

- Both sides walk one subdivision tree while neither has a cusp in the
  piece. The subdivision point depends only on the source and the end
  tangents, so the split each side would choose alone is the same. Where
  one side accepts a piece, the other carries on alone; where either side
  has a cusp, both do (`offset_rec_pair`).
- `approximate` is generic over `Sharing`. For one side, `Unshared`
  computes everything. For both, `Shared` hands the second side the linear
  fit (which does not depend on the side), the Taylor coefficients
  mirrored (`Taylor::mirrored`: `P` is even in `D`, `Q` odd, `|C'|^2`
  shared, so only `A'`, `D.C''`, `beta` and `A'.C'` are recomputed), and
  the parts of the certificate that do not change when `D` is negated
  (`SharedBounds`: the bound on `|Q|`, and the norms).
- All of this is exact: the output is the same, bit for bit, as two calls
  of `offset_cubic` without the initial `move_to`, which
  `pair_matches_one_side_at_a_time` checks over 80 curves, three
  distances, three tolerances and both modes.

Time per pair against two calls of `offset_cubic` built from the
previous commit (separate binaries, best of five), geometric mean over
the 27 settings: 0.933 in the default mode, 0.937 strict. The saving is
largest on gentle curves at tight tolerance (0.85 to 0.88), and at d = 10
on wild curves it is slightly slower (1.01 to 1.04).

The estimate of 1.3 times one side assumed that the work which depends on
the side is small. It is not: the certificate alone is 25 to 30% of the
time on gentle curves, and it, the bounds and the sampled estimate all
depend on `beta`, which differs between the sides. What is shared (the
fit, most of the Taylor coefficients, the split of the source, the
recursion) is under half of one side, so this design cannot do much
better than about 0.8.

Adding the pair made the one-sided functions 2 to 5% slower (7% on gentle
curves at 1e-4), with no change to their source path. With the pair's
recursion compiled out the difference disappears. The second
instantiation of `approximate` gives helpers such as `one_point_bound`,
`two_point` and `small_piece_bound` a second caller, and the compiler
stopped inlining them; they are now `#[inline(always)]`, which recovers
part of it. Of the rest, some is code placement: with every function
aligned to 64 bytes (`-C llvm-args=-align-all-functions=6`) the gap
shrinks, but `Taylor::new` was still 15% slower with identical
instructions and alignment, which was not explained. Aligning `Taylor`
itself did not help.

In the stroker: `StrokeOpts::offset_method` (hidden) takes an
`OffsetMethod`, `Offset`, `Offset2` or `Offset2Pair`, replacing the
earlier `offset2(bool)`. A runtime option lets one binary compare all
three; a cargo feature would need two builds, and a generic parameter
would add public API for an experiment. `Offset2Pair` appends both sides
straight to the forward and backward paths, and gives the same outline as
`Offset2`, bit for bit (`offset2_pair_matches_offset2` in `stroke.rs`).
Whole tiger strokes, time per tiger:

| Tolerance | `offset.rs` | offset2 | pair | pair / `offset.rs` |
|---|---|---|---|---|
| 1/32 | 126 µs | 101 to 107 µs | 95 to 100 µs | 0.75 to 0.79 |
| 1/256 | 529 µs | 173 to 175 µs | 162 to 163 µs | 0.31 |
| 1/2048 | 1644 µs | 288 to 297 µs | 256 to 261 µs | 0.16 |

Sharing the sampled estimate's evaluations was tried and dropped. At each
sample the estimate needs six dot products, `|D|^2`, `D.C'`, `|C'|^2`,
`D.C''`, `C'.C''` and `D.C'''`; for the other side the three odd in `D`
change sign, exactly. Caching them per sample in `Shared`, filled in
whichever orientation computed them first since either side may stop
early, kept the output bit for bit, but made the pair 4% slower in the
default mode and 2% slower in strict, which never takes the estimate.
Each sample costs a few multiply-adds, no more than checking and storing
it, and the larger cache costs on every piece.

## An estimate-only default mode, tried (18 November 2026)

The question: does the default mode pay for the strict machinery (the
Taylor polynomials, the certificate and the bound) when it only promises
the sampled estimate? Tried by skipping straight from the linear fit (or
arc drawing, if it is clamped or missing) to the sampled estimate, which
needs only `D`, `C'`, `C''` and `C'''` along the piece. The cheap strict
paths (hairline, root hull, small piece) were kept.

Random families, default mode, segments / calls over tolerance by the
Fréchet oracle / geometric mean ns per call:

| Mode | Nearly | Gentle | Wild | All, ns |
|---|---|---|---|---|
| Current | 1800 / 0 / 55 | 2971 / 0 / 121 | 13492 / 1 / 904 | 182 |
| Estimate only | 1800 / 0 / 68 | 3031 / 0 / 148 | 13632 / 1 / 829 | 202 |

On the tiger (ns per call, segments):

| Tolerance | Current | Estimate only |
|---|---|---|
| 1/32 | 137, 922 | 121, 922 |
| 1/256 | 241, 1196 | 203, 1197 |
| 1/2048 | 407, 1924 | 412, 2421 |

So the strict machinery is not a tax. On a piece it accepts, the Taylor
polynomials, the certificate and the one-point bound cost less than the
sampled estimate does when it accepts, since then it takes all seven
samples and searches its grid. The certified bound is also tighter (within
a few percent of the error, against the estimate's factor of 1.25), so it
needs fewer segments. The estimate wins only where the certificate fails
anyway: on wild curves (8% faster) and on the tiger at 1/32 and 1/256 (12
and 16%), whose zero end derivatives defeat the certificate. Accuracy was
the same throughout.

Routing by the sign of `beta`, tried and dropped: `Taylor::new` was
staged to compute `beta` first (from `|C'|^2` and `D.C''`), and in the
default mode a piece whose `beta` coefficients change sign went straight
to the hull bound and the sampled estimate, skipping `P`, `Q`, the
certificate and the equal-parameter bound. Output was unchanged in
segments and accuracy, and the pair still matched exactly. But the tiger
gained only 1 to 3% (against 12 to 16% for estimate only), since most of
its certificate failures are not sign changes of `beta`, and everything
else lost about 5%, strict mode included, though its path barely changed:
the same kind of inlining and layout cost as with the pair.

What is left from this: a cheaper estimate. The osculating parabola check
from `failed_k4` turned out to be that; see the next section.

## The osculating parabola estimate (18 November 2026)

From `est_arc_error_dot` on the `failed_k4` branch of fast-stroke
(commits ca05460 and 4e9deb4). At each of 8 samples, at the midpoints of
8 equal intervals, `W = D / d` is split into `dot` and `cross` along the
normal and tangent of the source, and the error in units of `d` is

```text
(dot - 1) + kappa_off d cross^2 / 2,    kappa_off = kappa / (1 + kappa d)
```

the radial error, and how far the offset bends away from its tangent over
the slip `d cross`. It is the leading term of the Newton value `H / d^2`,
with no remainder and nothing between the samples: a few multiply-adds,
a square root and two divisions per sample, and no grid search.

Against the Fréchet error on fitted pieces (ignored test
`osculating_estimate_ratios`), error over estimate with 8 samples had
median 1.05, p90 1.11, p99 1.29 and maximum 1.81; with 4, p99 1.98 and
maximum 3.9. The sampled estimate is far tighter (p99 1.02), but costs
more.

Tried as the whole of the default mode's acceptance (after the cheap
strict paths), it was 32% faster on the random families, but missed on
wild curves, up to 3.55 times the tolerance. Tracing the misses gave two
guards:

- Slip. The misses were two-point fits, which do not control the slip;
  the estimate fell short by up to 4 times there. Trusting it only where
  the largest `|cross|` is under 0.5 removed them (at 1.0 they came back).
- Change of curvature. On the tiger, a piece running into the tip of a
  sharp turn was 1.19 times over even at a safety factor of 1.4: the
  error gathers near one end, between the samples. Trusting the estimate
  only where the cusp values at the ends are positive and within a factor
  of 2 removed it. A guard on the cusp values alone (above 0, 0.2 or 0.5)
  did not help with the wild misses.

Adopted, in the default mode only: where the cusp values at the ends are
positive and within a factor of 2, fit (one-point or two-point, or arc
drawing if that is clamped or missing) and take the osculating estimate;
if its slip is under 0.5, accept when 1.2 times it is within the
tolerance, and otherwise subdivide. Anywhere else, the path is as before.
A rejection here is final, even where the certified bound might have
accepted; the figures include that. Strict mode is unchanged.

Results against the previous commit:

| | Before | After |
|---|---|---|
| Random families, geometric mean ns, default | 182 | 134 |
| Calls over tolerance (Fréchet), default | 1 (wild, 1.00 x) | 1 (wild, 1.00 x) |
| Segments, gentle / wild | 2971 / 13492 | 2983 / 13454 |
| Tiger ns at 1/32, 1/256, 1/2048 | 137, 239, 404 | 100, 149, 257 |
| Tiger segments | 922, 1196, 1924 | 911, 1182, 1851 |
| Whole tiger strokes, pair, over `offset.rs` | 0.74, 0.31, 0.16 | 0.55, 0.19, 0.10 |

The pair's saving over two one-sided calls falls to about 5% in the
default mode, since the estimate is per side and cheap.

Safety factors of 1.3 and 1.4 gave no fewer misses and up to 3% more
segments. The guards were tuned on the three random families and the
tiger; another corpus should check them.

### A cheaper form of it

With 42% of the default mode's time on gentle curves (35% nearly
straight, 20% wild) in the estimate, `osculating_decision` computes the
same thing without divisions and stops early. With `A = c'^ . D`,
`B = c' . D`, `K = c'' x c'`, `s = |c'|` and `E = s^3 + K d`, the error in
units of `d` is `A / (s d) - 1 + K B^2 / (2 d s^2 E)`, so it is within the
limit when `|A s E - d s^2 E + K B^2 / 2| <= limit |d s^2 E|`, and the
slip is within bounds when `B^2 <= 0.25 d^2 s^2`: one square root per
sample, no division. The samples are taken where the one-point fit's
error tends to peak first (0.3125 and 0.6875), and the first one over the
limit rejects.

Output is the same, bit for bit, on nearly straight and gentle curves and
in strict mode. On wild curves the default mode makes 9 more segments in
42,850: a sample over the limit now rejects before a later sample's slip
would have sent the piece to the strict path. Calls over tolerance are
unchanged.

Time against the previous commit: the default mode's geometric mean 132
to 129 ns; gentle and wild curves at tight tolerance 5 to 10% faster; the
tiger the same at 1/32 and 4 and 7% faster at 1/256 and 1/2048; whole
tiger strokes with the pair 4 to 8% faster (0.53, 0.18 and 0.09 of
`offset.rs`). Strict mode, whose path did not change, measured 3 to 6%
slower, also with every function aligned to 64 bytes; profiled, the
difference is spread over `offset_rec`, the certificate and the
subdivision, and keeping the new function out of line did not help.

So a cheaper estimator gains only a little more: what is left per sample
is evaluating `D`, `C'` and `C''` and one square root, and the rest of the
default mode's time is fitting, splitting and recursion.

## The rounding allowance (18 November 2026)

The bounds are computed for the exact `D`, which offset2 forms directly
rather than by subtracting control points. But the approximation's
control points are emitted in absolute coordinates, rounded by about an
ulp of the largest coordinate, and no bound on `D` sees that. As in the
original (`ROUND_ULPS` in bezfit's `bound/cheap.rs`), every strict
acceptance now goes through `CubicOffset::accepts`, which adds 8 ulps of
the largest coordinate of the source and the approximation to the bound.
The estimates are not bounds and are unchanged.

`rounding_allowance_covers_large_coordinates` fits and bounds pieces of
random cubics moved to 1e6 and 1e9 from the origin, moves them back (the
subtraction is exact there) and measures the error with the oracle. At
1e9 the bound alone falls short in over 50 cases, since rounding (about
1e-7) dwarfs the fit's own error on small pieces; with the allowance none
do. One or two ulps would pass this test too; 8 leaves room for the
rounding when the curves are evaluated downstream.

At ordinary coordinates the output is unchanged, bit for bit, and the
time within noise.

The notes on rounding elsewhere ("not handled yet" in the module's
documentation, and the rounding note in the original list) are now
answered by this.

## Where the time goes, and the slip guard (18 November 2026)

A step back on speed, on whole tiger strokes at 1/32, where nearly every
call makes one or two segments.

**The floor.** With `osculating_decision` stubbed to accept everything,
the tiger costs 32 ns per call against 96 (`offset_tiger`, 688 segments
against 911). So two thirds of the time is deciding: the estimate, the
slower path it hands over to, and the subdivisions that follow. Fitting,
splitting, regularization, end tangents and emitting cost about 32 ns.

**Who takes the slow path.** Per side at 1/32, about 1,100 decisions: 72
hairline, 176 root hull, 18 small piece, 518 osculating, and about 320
through `Taylor`, the certificate (which almost never holds on the tiger:
1 acceptance) and the sampled estimate. That slow path was about 40% of
the time, `accept_uncertified` alone (the sampled estimate, inlined) 20%.
Of those 320, about 100 fail the guard on cusp values (77 on the ratio,
23 next to a cusp) and 224 come back untrusted for slip: 123 one-point
fits and 93 arcs (a one-point handle clamped at a zero-length source
handle), 142 at a vanishing end derivative, and 99 on the thin strokes,
where a slip of half of `d` is a hundredth of a unit.

**The osculating circle, tried.** The parabola is second order in the slip,
so the distance from `A(t)` to the offset's circle of curvature, which is
exact for any slip where the offset is locally circular, looked like the
fix. In units that need no division:
`N = K (a^2 + b^2 - d^2 s^2) - 2 s^4 (d s - a)` over
`s sqrt((s^4 + a K)^2 + (b K)^2) + s^2 |E|`. Against the Fréchet oracle it
is no better than the parabola in any slip bin (tiger, slip 0.5 to 1:
p99 1.65 to 1.89 against 1.54 to 1.72). The pieces it underestimates
have their error between the samples, not in the expansion.

**What the slip guard catches.** Every piece accepted by the osculating
estimate with the slip guard off, logged and measured with the oracle
(wild cubics and the tiger at three scales, three tolerances, 38,141
pieces): the misses with slip over 0.5 are two-point fits (up to 3.5
times). One-point and arc misses are all mild (at most 1.12 times),
except one at 9.5 times with slip 0.40, inside the guard. That one is a
pair of cusps hidden in the piece: the cusp values at the ends are 0.999
and 0.984, and inside they run from -523 to 1553. It is reachable at
`8b8ab63` too; `default_result_with_hidden_cusp_pair` fails there (error
2.36 at tolerance 0.25).

**Changes:**

- The slip guard applies to two-point fits only.
- `osculating_decision` checks the sign of the cusp value at every sample
  (`E > 0`; it already computes `E`). Where it is not positive with both
  ends positive, there are at least two cusps in the piece, and the piece
  is split at that sample, so that each half shows a sign change and
  `find_cusp` finds both. Not at an end where the derivative vanishes,
  whose cusp value is not its limit (see below); there the piece goes to
  the slower path as before.

Results against `8b8ab63`:

| | Before | After |
|---|---|---|
| Whole tiger strokes, pair, µs at 1/32, 1/256, 1/2048 | 66.6, 96.1, 150.9 | 56.6, 89.7, 154.9 |
| Pair over `offset.rs` | 0.53, 0.18, 0.09 | 0.44, 0.17, 0.09 |
| Tiger, ns per call, default | 96, 144, 240 | 80, 135, 243 |
| Tiger segments, default | 911, 1182, 1851 | 892, 1181, 1879 |
| Tiger calls over tolerance, default | 0, 0, 0 | 1 (1.013 times), 0, 0 |
| Random families, geometric mean ns, default | 133 | 131 |
| `offset_compare` calls over tolerance, default | 0% | 0% |

Strict mode is unchanged, bit for bit (fingerprint), and so is the
default mode on nearly straight and gentle curves; on wild curves it makes
42,672 segments against 42,859. Slip alone (without the sign check) gave
51.9, 85.1 and 143.5 µs, so finding hidden cusps costs 5 to 8% on the
tiger, mostly at fine tolerance.

Tried and dropped:

- A ratio check at the samples (each sample's cusp value within a factor
  of 2 of both ends) instead of the sign: 63.2 µs at 1/32. The cusp value
  computed at an end where the derivative vanishes is 1 (`0 * infinity +
  1` with the clamp in `endpoint_cusp`), while the true values next to it
  are unbounded, so the check sent those pieces back to the slow path.
- Rejecting at a negative sample, and subdividing at the usual point:
  56.5 µs against 57.6 for the slow path, but the usual point seldom
  falls between the cusps. Splitting at the sample is better (56.0).
- Deciding both sides in one pass. `W = D / d` is the same for both sides,
  and at each sample only the signs of `A`, `B` and `d` change, and `E`
  becomes `s^3 - K d`, so one loop can give both verdicts, bit for bit.
  It was slower: the pair went from 84 to 98 ns on gentle curves, where
  every piece fused, with the one-sided path unchanged, and on the tiger
  only 97 of about 550 decisions can fuse, since at a zero-length handle
  the linear fit is clamped on exactly one side.

### Newton against ITP for the cusp (18 November 2026)

`find_cusp` is Newton's method safeguarded by a bracket (as `rtsafe`): it
bisects when the step would leave the bracket or be more than half the
previous one, so it cannot diverge, and a non-finite step bisects. Unlike
ITP it has no proven bound in terms of bisection, only a cap of 100
evaluations, after which it returns the middle of the bracket.

The hard case is a cusp where `kappa' = 0`: the scaled cusp value
`s^3 (1 + kappa d)` has derivative `3 s^2 s' (1 + kappa d) + s^3 kappa' d`,
so a root there is double. The ignored test `find_cusp_adversarial`
builds such brackets from 400 random cubics: at each local maximum of
`|kappa|`, `d = -(1 + e) / kappa` for `e` from 1e-1 to 1e-14, bracketing
each root from the extremum.

| Brackets | Newton evaluations, mean / max | ITP, mean / max | Over 1e-9 from a sign change: Newton / ITP |
|---|---|---|---|
| Random pieces (1,885) | 10.4 / 44 | 17.9 / 41 | 0 / 0 |
| Near double roots (19,962) | 20.3 / 45 | 22.2 / 41 | 6 (worst 1e-9) / 9 (worst 4e-9) |

So the near double root takes Newton's advantage away, and its worst case
is slightly over ITP's, but it never came near the cap, and its accuracy
is the same as ITP's. Both are limited there by the conditioning: the
function is tiny over a wide interval, and rounding decides the sign. A
rule that bisects whenever two steps fail to halve the bracket would give
a proven bound of about twice bisection, at the cost of interrupting
one-sided Newton convergence; not done.

### What is left

- The floor, 32 ns per call, is now about 40% of the tiger's time at 1/32.
- At a vanishing end derivative, three things go wrong together: the cusp
  value there is computed as 1 when its limit is infinite (with the sign
  of `d (q2 x q1)`), so the tiny cusp next to that end on the inner side
  is not found by the signs; one side's linear fit is always clamped to
  arc drawing; and the certificate fails since `beta(0) = 0`. Computing
  the end's cusp value properly would expose the cusp, but the outer side
  would then fail the ratio guard, and the inner side would gain a segment
  for the cusp; it needs its own treatment.
- 100 decisions per side at 1/32 still fail the guard on the cusp values
  at the ends.

## Vanishing end derivatives (19 November 2026)

A control point on an end point (`p1 = p0` or `p2 = p3`) makes the
derivative vanish there. The tiger is full of them (261 of 336 cubics).

**How such pieces were decided.** In the default mode, after the slip
change, almost all go through the osculating estimate. Strict mode never
certifies a piece touching such an end (`beta(0) = 0`) and subdivides
toward it: at 1/2048 on the tiger, over 2,000 rejections at those ends
alone, which is why it makes 2.5 times the default mode's segments there.

**A survey with wide strokes** (ignored test
`vanishing_end_derivative_survey`: 100 random cubics with a zero-length
handle, `d` of 0.3, 3 and 30 both ways, three tolerances) showed the
default mode badly wrong on the inside of the curve: up to 9.5 times the
tolerance by an exact measure, and 62 times at `8b8ab63`.

**The cause.** With `c'(t_end) = 0`, the cusp value's numerator
`c0 + c1 t + c2 t^2` is exactly `c2 (t - t_end)^2`, so the cusp value
tends to infinity with the sign of `c2`. `endpoint_cusp` computed it as
`0 * (1 / 1e-12)^3 + 1 = 1`. Where `c2 < 0` (the inside), the offset has a
cusp next to the end and runs backward from it to the end: a stub. The
sign test never found that cusp. Worse, the end value 1 told `apply` the
offset leaves the end forward, so the fit's handle there, which points
back along the stub, was clamped, and the fallback to arc drawing, with
the slip guard now off for arcs, gave fits several times over that the
estimate, whose samples stop short of the end, accepted.

**Two measurement problems found on the way.**

- The oracle built end tangents with `PathSeg::tangents`, which treats a
  handle shorter than about `1e-6` (`hypot2 < 1e-12`, absolute) as zero
  and falls back to the chord. On a short piece next to such an end, that
  turned the end tangent by `7.5e-5` and reported a 2.2 times miss where
  the piece was within `7e-8`. The oracle now uses a threshold relative to
  the curve's size. `PathSeg::tangents` has the same absolute threshold
  everywhere in kurbo; the stroker and offset2's root use it.
- The oracle recovers each piece's stretch of the source from its end
  points, taking the first match. Next to a cusp the stub runs back along
  the forward branch, the match lands on the wrong one, and it reported
  misses of hundreds or thousands of times where there were none. The
  survey therefore also uses `offset_to_result`: the largest distance from
  the offset, sampled densely, to the result, each candidate refined to an
  exact nearest point. It is a lower bound on the Fréchet distance, needs
  no matching, and agrees with the oracle wherever the oracle is sound.
  (A first version sampled the result instead of refining, and its
  resolution, about 0.01 on long segments, looked like misses at tolerance
  0.01; a two-way version was also fooled at sharp turns of the source,
  where uniform samples skip most of the offset's arc of radius `d`.)

By that measure strict mode was correct at `46f031b`: with the end value 1
the pieces holding a stub are accepted only by bounds that measure against
the offset, stub included, or subdivided. Its oracle misses were the
matching problem.

**Changes, in the default mode only:**

- `end_cusp` (replacing `endpoint_cusp`) gives the true sign at a
  vanishing end: -1 on the inside, so that the cusp is found and the stub
  fitted.
- Unless the stub is small. To leading order, with `q1` the middle control
  point of the derivative, the offset near the end moves along the end
  tangent by `|q1| u^2 - d k u`, `k = (q1 x q_other) / (2 |q1|^2)` the
  rate the tangent turns: back to the cusp at `u = d k / (2 |q1|)`, a
  distance `(d k)^2 / (4 |q1|)`. Where that is within `tol / 8` and
  `u <= 0.05`, the end keeps the value 1 (the stub is smoothed over), but
  `apply` does not clamp the handle at that end, which is left pointing
  back along the stub. On the tiger at 1/32 the stubs are about `2e-4`
  long against a tolerance of 0.031, and nearly all are smoothed.
- A cusp that `find_cusp` puts at an end of a piece (within
  `SPLIT_MARGIN`) means that end's sign was wrong; the end takes the other
  end's sign instead of a split that leaves a piece of zero length. In
  strict mode this replaces such splits next to vanishing ends.
- The sample check for hidden pairs of cusps no longer excludes pieces
  touching a vanishing end, since the end's sign is now right.

Survey, worst error over tolerance by the exact one-sided measure, default
mode (strict was and is within tolerance throughout):

| d | Tolerance | `46f031b` | Now |
|---|---|---|---|
| 0.3 | 0.25 | 4.51 | 1.16 |
| 3 | 0.001 | 4.46 | 0.96 |
| 30 | 0.01 | 4.18 | 0.96 |
| 30 | 0.001 | 9.53 | 0.99 |

The few left over 1 (1.16 to 1.44, on 4 of 1,800 calls) are pieces away
from the ends, the estimate's own looseness. The tiger's one call over
tolerance (1.013 times, at 1/32) is gone. `default_result_with_vanishing_end_derivative`
keeps two of the cases; it fails at `46f031b`.

Cost, against `46f031b`: whole tiger strokes with the pair 4% slower at
1/32, 6% at 1/256 and 0.5% at 1/2048 (segments 1681, 2272, 3047 against
1683, 2268, 3118). Random families give identical output, within 1 to 2%
in time. About 2% of it is code layout (the extra state in `apply`); the
rest is the fitting of the inner sides with their real handles and the
stubs now fitted at fine tolerance.

Tried and dropped:

- The true sign everywhere, with no smoothing: correct, but whole tiger
  strokes took 131 µs at 1/32 against 56: every inner side became a cusp,
  a stub piece and a piece ending at the cusp, each decided by the slow
  path.
- Smoothing small stubs with the end value 1 and the handle clamped as
  before: the 4.5 times miss came back, on a stub `5e-7` long. The stub was
  never the problem there; the clamped fit was.
- Deciding the stubs in `CubicOffset::new`: 3% slower on the random
  families, which have none. It is done in `end_cusp` at the root.
- A table of the osculating samples' Bernstein weights instead of
  computing them in the loop: 3% slower.

Still open: strict mode, with no estimate to check samples, can miss a
pair of cusps hidden at a sharp turn of the source until subdivision lands
between them, and runs out of depth on one survey curve (by the oracle;
the exact measure does not reach its tolerance of 1e-4).

## Pieces ending at a cusp (19 November 2026)

A piece split at a cusp has the value `CUSP_EPSILON` at that end, so the
ratio guard on the osculating estimate always failed and the piece went to
the slow path, which on the tiger mostly accepted it anyway: about 100
decisions per side at 1/32, each paying for `Taylor`, a certificate that
cannot hold (`beta` vanishes at a cusp), the equal-parameter bound and the
sampled estimate.

The guard was added for pieces running into the tip of a near cusp, where
the cusp value falls toward zero without crossing it. Measured on pieces
that end exactly at a cusp (ignored test
`osculating_estimate_at_cusp_ends`: 4,575 pieces from 400 random cubics at
six widths, fitted as `approximate` fits them), error over the osculating
estimate has median 0.98, p90 1.08, p99 1.16 and maximum 1.59, as good as
on ordinary pieces. Six more samples near the ends changed nothing.

So an end that is a cusp split (`cusp_min == CUSP_EPSILON`) is exempt from
the ratio. Results against `ec0dff1`:

| | Before | After |
|---|---|---|
| Whole tiger strokes, pair, µs at 1/32, 1/256, 1/2048 | 58.3, 93.9, 153.8 | 55.7, 88.1, 143.8 |
| Tiger segments (strokes) | 1681, 2272, 3047 | the same |
| Tiger, ns per call, default | 85, 144, 244 | 79, 132, 228 |
| Random families, geometric mean ns | 132 to 134 | 129 to 132 |
| Wild segments, default | 42,672 | 42,636 |
| `offset_compare` over tolerance | 0% | 0% |

Nearly straight and gentle curves give identical output. In the vanishing
end survey the exact one-sided measure is unchanged; the oracle reports
two new cases at `d = 30`, 1.13 and 1.26 times, within the looseness
measured above.

`smooths_stub`, which runs at every root end where the derivative
vanishes (about 550 times per tiger), was 4.4% of the tiger's time at
1/32: it was marked cold, out of line, and divided twice. Comparing by
cross-multiplication, inline, takes whole tiger strokes to 54.8, 87.0 and
142.8 µs, with the same output.

Pieces on the reversed stretch between two cusps (both cusp values
negative) were also measured: the estimate errs high there (error over
estimate median 0.77, p99 1.11, maximum 1.30). Letting it decide them
saved 2% on the tiger at 1/2048 and nothing elsewhere, and cost 0.6% more
segments on wild curves, so it was not adopted.

Deciding both sides in one pass, tried again. With the stubs fixed, both
sides of a tiger stroke share their fit far more often: 296 of 719
osculating decisions at 1/32 are a second side whose `D` is exactly the
first side's negated (97 of about 550 before). Two versions:

- One loop over the samples computing the shared values and then each
  side's checks, with per-side state: in isolation 49.6 ns against 43.4
  for two separate calls. The per-side state checks seem to defeat the
  overlap between iterations that the plain loop gets from prediction.
- The shared values for all eight samples into an array first, then each
  side's checks as its own early-exit loop: in isolation 34.5 against
  43.6 ns when every sample is needed, and 29.5 against 31.5 with
  rejections. But in the stroker it was still slower (whole tiger strokes
  56.8, 90.8 and 150.3 µs against 55.0, 87.4 and 142.8): the cached
  verdict and `D` make `Shared` bigger on every piece, `approximate` for
  the pair compiles worse, and a pass is wasted when the other side never
  asks. This is the third time added state on the pair's path has cost
  more than it saved; the pair is best kept thin.

The ratio guard itself, measured (pieces from random cubics in 1 to 16
parts at six widths, both cusp values positive, ratio over 2; error over
the 8-sample estimate, and over a 64-sample one):

| Ratio of cusp values | Pieces | p99, 8 samples | max | p99, 64 samples | max |
|---|---|---|---|---|---|
| 2 to 4 | 1,398 | 1.25 | 1.60 | 1.17 | 1.51 |
| 4 to 16 | 989 | 1.46 | 2.64 | 1.39 | 1.89 |
| over 16 | 307 | 2.19 | 8.11 | 2.10 | 6.09 |

So up to a ratio of about 4 the estimate is as reliable as on ordinary
pieces, and above it the model fails, however densely sampled; three more
samples near the end with the smaller value changed nothing. Raising the
limit from 2 to 3 or 4 makes whole tiger strokes 2 to 3.5% faster with the
same segments and keeps `offset_compare` at 0%, but brings back one tiger
call over tolerance (1.19 times, at 1/256, a piece with a zero-length
handle). Adopted at 3 (19 November 2026), accepting that one miss; 4 would
leave no margin at the edge of the reliable range. Measured again on
adoption, interleaved with the guard at 2: whole tiger strokes with the pair
54.6 to 54.9, 85.7 to 88.7 and 141.2 to 142.4 µs (from 55.7 to 55.9, 88.0
to 88.5 and 145.6 to 145.8), with 1683, 2272 and 3046 segments (from 1681,
2272 and 3047); `offset_tiger` over tolerance 0, 1 and 0.

This more than pays back the vanishing end fix: whole tiger strokes are now
faster than at `46f031b` (56.3, 89.2 and 154.3 µs).

Also tried in this round, and dropped:

- Computing all eight osculating samples without branches, then checking
  them in order, so that the compiler might pair them in SIMD registers:
  it did not, and losing the early exit made it 12% slower.
- Horner's rule in the power basis for the samples: 1 to 3% slower. The
  loop is bound by latency, and the Bernstein form's independent products
  overlap better than Horner's chain.
- Retrying a rejected one-point fit with the midpoint G1 fit before
  splitting: 40% slower on the tiger, and 20% on the random families, for
  0.3 to 2% fewer segments. After a rejection it rarely succeeds.

## Where this departs from the original notes

(These "original notes" and "original code" are Claude's, not human work;
see "Trimming again", under where the ideas came from.)

The bounds were reconstructed from "Cheap Offset Error Bounds", not ported
from the original code. Choices made where the notes left room: the margin
growth factor, the form of `K`, the range of `kappa` as an interval product
widened to include zero, and division by the node polynomial in the power
basis. The reconstruction certifies a smaller share of pieces than the
notes report (67 to 78% against 89 to 94% on similar pieces), which may be
the test curves or a weaker certificate.

## Code that is not in the module

Written in the same session and tested there, but kept out to limit size:
the HalfFit, FootWeighted and split variants of the G1 bound
(`g1_cheap_bound.rs`), and a standalone sampler with three kernels
(`offset_error_estimate.rs`).

## Trimming (19 November 2026)

Asked how small the module can be for at most a 5% slowdown. Each part
was removed alone and measured: whole tiger strokes one side at a time
and with the pair, and the random families (best of five, geometric mean
over the 27 rows, worst error by the Fréchet oracle), in both modes.

| Removed | Default mode | Strict mode | Kept? |
|---|---|---|---|
| Midpoint G1 retry (off by default) | 0 | 0 | removed |
| Certified bound on arc drawing (never decided a piece) | 0 | 0 | removed |
| Sharing Taylor and certificate between the sides | 0 | 0 | removed |
| `small_piece_bound` | 0 | +0.8%, +0.6% segments | removed, then restored: see "Near cusps" |
| `g1_bound` (two-point fits take the crude bound) | 0 | +3.6%, +3% segments | removed |
| `one_point_bound` (crude bound instead) | +1% | +8%, +11% segments | kept |
| Sharing the linear fit as well (no `Sharing` trait) | +2 to 3% | +5 to 7% | kept |
| Hairline bound | +2.7% at 1/32 | | kept |
| Root hull bound | +1% | +6% | kept |
| `equal_parameter_bound`, or the uncertified hull bound | 0 | misses at the maximum depth | kept |
| Sampled estimate as plain maximum of samples (no grid) | -1% | 0 | not adopted: 1.16 times on one wild curve |
| Midpoint subdivision | -4% | -5% | not adopted: 1.5 times on one wild curve |
| No certified bounds at all in the default mode | 0 | | (strict mode needs them) |

The two misses are the same curve (wild, `d = 0.5`, tolerance 0.25). It
turns sharply near its end, and with a midpoint split a piece keeps the
turn inside. The cusp values at its ends are both about 1, so the ratio
guard passes, and the curvature peaks between the osculating samples. The
angle split happens to put an end near the peak. A guard that looks at
the cusp value at the samples, or the curvature inside the piece, would
be needed before midpoint subdivision is safe.

Adopted together (`1fd3fe4`): non-test code 1736 to 1291 lines, comments
847 to 685, the whole file 4347 to 3534 (3 tests and an ignored survey of
what was removed go with it). With `small_piece_bound` restored, non-test
code is 1327 lines. Measured interleaved with the previous version, four
runs: tiger strokes one side at a time 1.9 to 2.7% faster, with the pair
unchanged (within 0.4%); random families 3.8% faster in the default mode
and 1.3% slower in strict mode. Default segments are unchanged on the
tiger; strict ones rise from 3155 to 3185 and from 4677 to 4759 at 1/256
and 1/2048. No accuracy change in either mode.

Larger cuts, measured but left for a decision:

- Dropping strict mode. Without certified bounds the default mode makes
  the same segments at the same speed, so the certificate, the one-point,
  crude and equal-parameter bounds, the Bernstein products and most of
  `Taylor` exist only for strict mode: about 550 to 600 non-test lines,
  and about 500 lines of tests.
- Dropping the pair: about 160 lines of the module and a little of
  `stroke.rs`, for 9 to 13% on whole tiger strokes.

### Near cusps (19 November 2026)

The trim above was measured on the tiger and the random families, which
hardly contain near cusps, and removing `small_piece_bound` looked free.
On hairpins and on cusps made inexact by a gap between the middle control
points (ignored test `near_cusp_depth_survey`) it was not: strict mode at
a gap of 1e-6 and tolerance 1e-4 went from 31,329 segments and 2,946
pieces at the maximum depth to 127,666 and 93,103, and at 1e-6 from
199,488 to 628,646 segments. It is restored. The tip of a near cusp is
where its arc acceptance is needed, and no other bound takes that piece.

The survey also shows a problem from before the trim: near cusps at tight
tolerances run out of depth with the angle split. At a gap of 1e-6 and
tolerance 1e-6 every call does, emitting 86,232 pieces at the maximum
depth in the default mode (800 calls, 147,873 segments). Splits tried on
the survey, the tiger and the random families, with the bound restored:

| Split | Tiger and random, time | Random, worst | Hairpins | Near cusps, tight tolerance |
|---|---|---|---|---|
| Angle and curvature (current) | | 1.005 | | runs out of depth |
| Midpoint everywhere | 2 to 6% faster | 1.495 | 40 to 2,416 pieces at the maximum depth where the current split has 0 to 213 | 33,082 segments and 1,619 at the maximum depth at gap 1e-6, tolerance 1e-6 |
| Midpoint where there is an inflection | same | 1.005 | 16 to 20% more segments at 0.25 and 1e-2 | same as current |
| The inflection nearest the middle | same | 1.005 | slightly better | somewhat worse |

So the curvature integral in the inflection branch earns its keep on
hairpins, and neither split converges well everywhere. A split that
finds the tip of a near cusp directly (the minimum of `|C'|`) may be what
is missing.

### Lopsided splits (19 November 2026)

Splitting at the minimum of `|C'|` was suggested for near cusps. A trace
of a failing one showed that the angle split finds the tip at once; the
trouble is every split after that. Next to the tip nearly all the
turning is concentrated at one end of the piece, and the bisector of the
end tangents lands within 1e-6 to 1e-7 of the piece from it, so each split
leaves a piece that turns a little less but is as long as before (1e-4
across at tolerance 1e-6), until the maximum depth. Splitting at the
minimum of `|C'|` would only help the first split of a tip.

So `keep_inside` now moves a split point to at least
`SUBDIVISION_MIN_SHARE` (10%) of the piece from either end. Midpoint
fallback instead of moving was much worse on hairpins. On the survey,
clamping at 5, 10, 15 and 25% all beat none; 10% is the balance (25% is
best at the extremes and worse on moderate near cusps). Against the
previous version: pieces at the maximum depth fall 3 to 20 times on near
cusps, hairpins at tolerance 1e-6 go from 168 times the tolerance to 1.09
and from 213 such pieces to 15, segments fall, and the tiger and random
families are unchanged in time (within 1.4%).

The new split points exposed a default-mode miss of 2.2 times on one wild
curve (`d = 10`, tolerance 0.25): the sampled estimate accepted, at 0.31
of its true error, a piece next to a regularized near cusp whose end cusp
values differ 160 times. It now refuses pieces above
`ESTIMATE_MAX_CUSP_RATIO` (64), as the osculating estimate does above 3,
with the same exemption for a piece ending at a cusp of the offset. That
removes the miss with no measured cost; at 16 it costs segments. It adds
10 to 20% default segments on the near-cusp families at loose tolerances.

Measuring accuracy near cusps needs care. The functions approximate the
offset of the regularized input, and regularizing alone moves the offset
near a cusp by up to `2 |d|`; measured against the raw input, every near
cusp looked off by exactly that. And the oracle's range matching broke
down here (strict mode at a trillion times the tolerance). The survey now
reports the one-sided distance from the offset of the regularized input
to the result, which needs no matching.

With that measure, two problems remain, both from before:

- The default mode misses a near cusp inside a piece (open item 13). The
  ends of such a piece look ordinary, with cusp values near 1, and the
  samples straddle the tip. Trusting the estimates only where the tangent
  turns less than a half turn (the middle hodograph control point within
  a quarter turn of both end tangents) brings the default mode to strict
  mode's accuracy on every row, and fixes a random curve at 1.96 times at
  tolerance 0.25. It costs 42 to 48% on the tiger at 1/32 and 6 to 7% at
  1/2048 (with end tangents for zero handles; with the handles themselves,
  2 to 2.8 times), so a finer test is needed.
- Strict mode emits pieces far from the offset at the maximum depth on
  near cusps at tolerance 1e-4 and below (open item 12).

## Tips at the root (19 November 2026)

A tip is a sharp turn or near cusp of the source: a minimum of the speed
`|C'|` inside the curve at which the tangent turns fast. There `C'` is
perpendicular to `C''`, and near it the tangent turns through
`atan(s / w)` over a parameter distance `s`, with the width
`w = |C'| / |C''|`. A regularized cusp has a width of 1e-6 or less.

**What was wrong.** With the survey's cusps moved to random parameters
(they were all at `t = 1/2`, where the first midpoint split lands on the
tip by luck), `968bf57` was badly off in both modes: the default mode up
to 69,000 times the tolerance and strict mode up to 170,000 times. Two
causes:

- Nothing in the recursion sees a tip inside a piece. The estimates' samples
  straddle it, and the cusp values at the ends of such a piece can both be
  near 1, so every guard passes. The default mode then misses the whole
  sweep around the tip, an error of `2 |d|`.
- The bounds reject a piece that turns so far, but the angle split lands
  beside the tip, and the piece that keeps it is as hard as before, down to
  the maximum depth, where it is emitted unchecked.

**Per piece, tried first.** A test for a dip of `|C'|` inside the piece
(the middle control point of the hodograph within a quarter turn of both
end tangents clears most pieces; then, for a zero-length handle, a
point-to-segment distance; then the distance from the origin to the
hodograph's control triangle; then a nearest-point solve), used to distrust
the estimates and as the split point of a rejected piece. It fixed the
accuracy, but the guard cost 1 to 5% and the split another 3 to 8% on the
tiger, most of it from the extra code in `find_subdivision_point` and not
from extra work: even a threshold that almost never split cost 2.5 to 4%.
Comparing the tangents at neighbouring samples, which is cheaper, does not
work at all: regularizing turns a cusp into a tiny loop or a double
inflection, and the tangents either side of it agree.

**At the root.** A tip is a property of the curve, not of a piece: the
minima of the speed are the roots of the cubic `C'.C''` at which it rises,
at most two. So they are found once, in `regularize_and_split_tips`, and
the curve is split there before the recursion starts: at the tip, and
`TIP_REACH` (8) widths either side, so that the pieces next to it turn
through most of a quarter turn each. A minimum is a tip if its width is
under `TIP_MAX_WIDTH` (0.01). The recursion has no guard and no state for
this on its hot path.

Nearly every curve has no tip, and the test for that
(`CubicBez::may_have_slow_point`) has to cost less than offsetting a simple
curve:

- `|C'|^2` is no less than the smallest of its five Bernstein coefficients,
  which are dot products of the hodograph's control points. If that exceeds
  `(TIP_MAX_WIDTH |C''|)^2`, with `C''` at its larger end, there is no tip.
- With a zero-length handle that bound is zero, and 78% of the tiger's
  curves have one. Then `C' / 3 = t (a + t m)`, and `C'.C''` is `t` times
  the quadratic `|a|^2 + 3 a.m t + 2 |m|^2 t^2`. The speed has a minimum
  inside only at its larger root, which is inside (0, 1) under conditions on
  `a.a`, `a.m` and `m.m`, and is no less than the vertex
  `-3 a.m / (4 |m|^2)`; with the exact minimum of `|a + t m|` on [0, 1]
  that bounds the speed at the minimum from below. No square root.
- Regularizing looks for a cusp at the same minima, so the one test covers
  both (its limit is `|C'|^2 <= |C''| dimension`), and `regularize_cusp`
  uses it too.

A curve that passes goes to `CubicBez::slow_points`, out of line: the sign
pattern of the Bernstein coefficients of `C'.C''` (a rising root needs a
negative one before a positive one), then the minimum itself. Writing the
hodograph as `A t^2 + B t + C`, its cross product with `A` is linear in
`t`, zero at one parameter, and a point of the hodograph within `r` of the
origin is within `r |A| / |A x B|` of it; when that range is small, four
Newton steps from there find the minimum, and otherwise `solve_cubic` does.

What each step of that bought, on the tiger (336 curves): solving the cubic
on every curve with a possible minimum, 126 solves and 17 to 37% slower;
with the Bernstein bound, the zero-handle test without the root's position,
then without a bound on the speed there, 131 and 122 curves reached the
slow path, 3 to 9% slower; with both, 41 do, of which 31 have a minimum and
none is a tip. The Newton path in place of `solve_cubic` made no measurable
difference by then. What mattered as much as the arithmetic was the shape
of the root: returning the regularized curve and the splits from the test,
or passing them to a cold function by reference, cost 2 to 5% on the
cheapest calls. The root now computes one boolean, and a curve that passes
goes to a separate cold entry (`offset_cubic_at_slow_point`) that shares
the root's body.

The old check in `regularize_cusp` was not free either: with no
regularization at all the tiger is 1 to 4% faster and nearly straight
random curves 8%.

**Two bugs in `detect_cusp`,** both shared with `offset.rs` and the
stroker. It required the control polygon to be in a region (three
determinant signs) that does not contain every cusp: the survey's exact
cusps at random parameters were never regularized, the tip split then
landed on a true zero of `C'`, and the pieces either side collapsed to
zero width. The region test is gone; the classification into loop and
double inflection is kept. And a zero-length handle is now excluded
explicitly (the region test did so by accident, and the nudges divide by
the handle lengths).

**Three more changes this needed.**

- `small_piece_bound` judged whether the tangent turns one way from
  `seg.p2 - seg.p1`. Next to a tip the piece is a few ulps across and that
  difference is rounding noise, so the pieces were rejected down to the
  maximum depth: 469,000 segments on one survey row. The same direction is
  now taken from the curve's own derivative (`hodograph_middle`).
- Beyond a few widths from a tip the curvature falls off as the cube of the
  distance from it. A rejected piece whose ends are at distances in a ratio
  over `TIP_SPLIT_RATIO` (2) from a tip is split at their geometric mean
  (`split_toward_tip`), which halves the range in log scale; the angle
  split, clamped to 10%, took ten times as many levels. This is the only
  state the tips add to the recursion (`CubicOffset::tips`, checked for NaN
  in `find_subdivision_point`).
- A split at a cusp of the offset no longer counts toward the depth limit.
  Neither half has a cusp, so there are only as many as the offset has
  cusps, and with a wide offset they used up half the depth before any
  ordinary subdivision. Written into `offset_rec` this made curves with no
  cusp 3% slower; it is in `split_at_cusp`, out of line.

**Results.** The survey (`near_cusp_depth_survey`; 800 calls per row;
segments, pieces at the maximum depth, worst error over tolerance by the
one-sided measure, default | strict), `968bf57` and now:

| Family, tolerance | `968bf57` default | strict | Now default | strict |
|---|---|---|---|---|
| Random, 0.25 | 2844, 0, 1.96 | 5978, 0, 0.65 | 2862, 0, 1.96 | 5842, 0, 0.65 |
| Random, 1e-6 | 28447, 2, 5.35 | 32530, 284, 5.35 | 28452, 0, 0.99 | 32915, 116, 0.99 |
| Hairpin, 1e-2 | 5298, 3, 0.80 | 13321, 4, 0.64 | 5394, 0, 0.50 | 10830, 0, 0.12 |
| Hairpin, 1e-6 | 39227, 15, 1.09 | 51277, 110, 0.98 | 38518, 0, 0.99 | 47456, 0, 0.99 |
| Gap 1e-2, 0.25 | 5412, 0, 17.9 | 9010, 25, 0.49 | 4876, 0, 0.84 | 5823, 0, 0.06 |
| Gap 1e-2, 1e-2 | 8636, 34, 25.4 | 12672, 104, 9.68 | 5962, 0, 0.91 | 10564, 0, 0.81 |
| Gap 1e-2, 1e-4 | 12951, 220, 685 | 24499, 451, 1663 | 12200, 0, 0.99 | 23181, 0, 0.99 |
| Gap 1e-2, 1e-6 | 32901, 2280, 67900 | 43555, 3929, 169850 | 39034, 0, 0.99 | 51059, 68, 0.99 |
| Gap 1e-4, 0.25 | 5799, 0, 23.2 | 9352, 86, 0.46 | 5018, 0, 0.83 | 6147, 0, 0.23 |
| Gap 1e-4, 1e-4 | 13234, 786, 769 | 19873, 1156, 814 | 13297, 0, 0.95 | 22655, 0, 0.96 |
| Gap 1e-4, 1e-6 | 17577, 1335, 69239 | 29625, 3450, 69572 | 22938, 230, 1.24 | 42608, 280, 0.99 |
| Gap 1e-6, 0.25 | 5744, 0, 17.8 | 9478, 110, 0.52 | 4969, 0, 0.79 | 5959, 0, 0.47 |
| Gap 1e-6, 1e-4 | 12432, 950, 41.3 | 16013, 1197, 42.8 | 12093, 1, 0.94 | 18937, 0, 0.96 |
| Gap 1e-6, 1e-6 | 15017, 1231, 6871 | 21033, 2416, 6871 | 21162, 124, 1.12 | 34633, 374, 0.99 |
| Exact cusp, 0.25 | 5732, 0, 24.1 | 9377, 115, 0.56 | 4946, 0, 0.83 | 5927, 0, 0.11 |
| Exact cusp, 1e-4 | 12329, 930, 85.3 | 15598, 1153, 85.3 | 12240, 0, 0.98 | 18803, 1, 0.98 |
| Exact cusp, 1e-6 | 15530, 1686, 10877 | 20221, 2879, 6884 | 21227, 346, 0.99 | 32187, 1025, 0.99 |

Strict mode is within tolerance on every row. The default mode is within
1.04 times everywhere but three rows: 1.12 and 1.24 at a tolerance of 1e-6
of the curve's size, and the random curve at 1.96 (open item 13), which is
a sharp turn of width over 0.01 and untouched by this. At loose tolerances
the tips also save segments, up to 40% in strict mode.

Time against `4f0cadd`, interleaved, best of four runs: whole tiger strokes
with the pair +0.4 to +0.8% (noise), one side at a time +2 to +6%; random
families +2.8% in the default mode (nearly straight +1.2, gentle +3.3, wild
+3.4) and -0.2% in strict. Tiger segments are the same in the default mode
(892, 1184, 1801 by `offset_tiger`); strict at 1/256 and 1/2048 gains 4 and
1 (3087, 4674), from the cusp splits.

Tried on the way and dropped: a threshold of 0.1 for a tip (the tiger's
ordinary sharp turns, with widths of 0.015 to 0.09, were then split: 5%
more segments and a call over tolerance); a clamp of 1% or none on split
points once the tips were known (far worse); the root's body out of line,
called from both entries (no better).

`results_with_tips` keeps 18 near cusps at three distances and three
tolerances in both modes: no piece at the maximum depth, and within 1.1
times the tolerance (strict: within it). It fails at `4f0cadd`.
`slow_points_finds_every_slow_minimum` in `cubicbez.rs` checks the test
and the solve against sampled minima of the speed on 4,000 curves.

## After the tips (19 November 2026)

**Where the time is.** Whole tiger strokes with the pair, sampled
(`sample`, 8 s each): `osculating_decision` is 27% of the time at 1/32 and
36% at 1/2048; `approximate` for the pair 19 and 16%; `offset_rec` (with
the one-sided `approximate` inlined) 13%; the stroker itself (`do_join`,
`stroke_with`, `extend_reversed`, `do_cubic`, `finish_closed`) about 12%
at 1/32; `subsegment` 3 to 6%; `accept_uncertified` 4 to 7%; the root
4 to 5%; `hull_bound` 4% at 1/32.

**Fewer samples for the osculating estimate, measured and dropped.** Every
piece that reaches the estimate was logged on the tiger (three tolerances)
and on wild and gentle random curves, with its Fréchet error from the
oracle, and each candidate set of samples scored by what it would accept.
One-point fits, accepted / over tolerance / worst, at the safety factor
1.2:

| Samples | Tiger (4022 measured) | Wild (21715) | Gentle (5694) |
|---|---|---|---|
| 8 uniform (current) | 2904 / 1 / 1.19 | 13789 / 0 | 4027 / 0 |
| 6 uniform | 2905 / 1 / 1.01 | 13738 / 8 / 2.07 | 4006 / 0 |
| 6: .08 .2 .34 .66 .8 .92 | 2915 / 1 / 1.26 | 13876 / 3 / 2.05 | 4038 / 0 |
| 4: .14 .28 .72 .86 | 2928 / 5 / 1.51 | 13719 / 6 / 2.07 | 4005 / 0 |
| 4 uniform | 2953 / 25 / 1.37 | 14294 / 235 / 4.04 | 4157 / 71 / 1.33 |
| 2, at the peaks of `w` | 2934 / 9 / 1.51 | 13798 / 42 / 5.52 | 4006 / 0 |

For the one-point fit the Newton value is exactly
`w r_mid + w2 (r_slope - 2 g)`, so with `g` constant two samples at
`1/2 ± 1/sqrt(20)` would determine it, and on gentle curves they do. On
the tiger and on wild curves `g` varies, and every smaller set lets misses
through. Two-point fits need all eight (4 uniform: 10 of 271 accepted over
tolerance on the tiger, worst 1.78). The eight stay.

**Two samples first, then six, dropped.** The first two samples are where
the one-point fit's error peaks and about 37% of decisions are rejections,
so evaluating those two, rejecting if either is over, and only then the
other six, gives the same decisions with less work. It was 2 to 3% slower
(12 to 20% when the two sets were taken as slices of the sample array, so
that the weights were no longer constants). The eight samples in one
branch-free loop overlap in the pipeline; a data-dependent branch after
two costs more than the six it saves.

**The inflection branch of the subdivision is the midpoint.** With the tips
split off, subdividing a piece with an inflection by the integral of
absolute curvature made no difference to the survey (every row within a
few segments, same errors), and under half a percent to segments on the
tiger and the random families (tiger 1185 and 1805 against 1184 and 1801
at 1/256 and 1/2048; strict 3100 and 4670 against 3087 and 4674). It is
gone, with the forced fallback in `subdivide_for_tangent`: 45 lines less,
and 0.3 to 1% faster. Midpoint everywhere was measured again too: 2 to 8%
faster (12% for strict mode on wild curves) and the survey unchanged, but
1881 against 1801 segments on the tiger at 1/2048 and a wild curve at 1.35
times its tolerance.

**A regression in that commit, fixed in the next.** `99f534c` was measured
as a variant and then rewritten for the commit, and the rewrite gave the
midpoint's tangent the fallback used elsewhere: the start tangent if
`|C'|^2 < 1e-12`. Next to a regularized cusp the speed is around 1e-7 and
the tangent is still well defined, so pieces there got a wrong tangent and
ran to the maximum depth: on the survey's exact cusps at tolerance 1e-4,
16,064 such pieces in the default mode where the variant had none (and
283,450 at 1e-6). Only an exact zero is refused now. The survey on the
committed code, not on a variant of it, matches the variant row for row,
and `tips_do_not_exhaust_the_depth` counts pieces at the maximum depth on
80 near and exact cusps (941 with the bug, none without). The other
absolute thresholds on `|C'|^2` (`UTAN_EPSILON` in the cusp split,
`keep_inside` and `subdivide_for_tangent`) have the same flaw in principle
and were not seen to bite.

## Sharp turns (19 November 2026)

Open item 13 was one random curve at 1.96 times a tolerance of 0.25. A
survey built for it found more, and three separate causes, two of them
bugs in the estimates and not a matter of sharp turns at all.

**The survey** (`sharp_turn_survey`, an ignored test): random
cubics with control points in a 100 unit box, sorted by the width
`|C'| / |C''|` of their narrowest interior minimum of the speed into three
bands, 0.01 to 0.03, 0.03 to 0.1 and 0.1 to 0.3, 400 curves each; offsets
at ±0.5, ±2 and ±10, so 2,400 calls a band at each tolerance; error by the
Fréchet oracle, and every miss confirmed by the one-sided distance. Nearly
all the misses are at a tolerance of 0.25 with `|d| = 0.5`. Calls over 1.05
times the tolerance, and the worst:

| Width | Tolerance | `f2e3592` | Two fixes | With the split |
|---|---|---|---|---|
| 0.01 to 0.03 | 0.25 | 20, 3.92 | 14, 2.28 | 0, 1.01 |
| | 0.01 | 1, 1.09 | 1, 1.09 | 1, 1.14 |
| 0.03 to 0.1 | 0.25 | 11, 1.41 | 7, 1.41 | 7, 1.41 |
| | 0.01 | 0 | 0 | 0 |
| 0.1 to 0.3 | 0.25, 0.01 | 0 | 0 | 0 |

(At 1e-4 one call in the widest band is at 1.09 throughout, and nothing
else is over.)

**The sampled estimate did not check the side.** It estimates
`|rho - |d||`, with `rho` the distance from the approximation to the
source, and a distance is the same on both sides. The worst curve turns
through 160 degrees and back, with end tangents nearly parallel. The
two-point fit for the whole curve is close to a translation by `d n0`,
which between the turns runs along the *wrong* side of the source, at the
right distance. It crosses the source at each turn, where `rho` passes
through zero, but between samples. The osculating estimate had it at -1.9
at four samples, but the first sample slipped too far and that returned
"untrusted" (next item). Each sample of the sampled estimate now requires
`d (C' x D) > 0`, and hands the piece to the equal-parameter distance
otherwise. Strict mode was never affected: its bounds hold for every
parameter, so they see the crossing.

**"Untrusted" hid a sample over the limit.** `osculating_decision` took
its samples in order, and returned "untrusted" at the first whose slip was
over the limit for two-point fits, even if a later sample was far over the
error limit. The piece then went to the sampled estimate, which has seven
samples to the osculating eight, at different places, and accepted pieces
at up to 2.4 times the tolerance that a sample of the osculating estimate
had at 3.6 times `|d|`, against a limit of 0.42. Now a cusp at a sample,
or a sample over the limit, decides first; too much slip only stops the
estimate from accepting. This is also simpler, and no slower: the tiger
takes the same time at 1/32 and 1/256 and 2% less at 1/2048, with one
segment more.

These two are commit `674c6ab`. Random families: 1 call of 5,400 over
before, none after.

**Turns of width 0.01 to 0.03 are split at the root**, at the turn and 4
widths either side (`TURN_MAX_WIDTH`, `TURN_REACH`). They are not recorded
as tips, so `split_toward_tip` and the midpoint tangent rule do not apply
to them. What was tried, on the band 0.01 to 0.03, over 1.05 / worst at
0.25, then at 0.01, and segments on the tiger at 1/32, 1/256 and 1/2048
(892, 1185 and 1805 before):

| Variant | 0.25 | 0.01 | Tiger segments |
|---|---|---|---|
| No split | 14, 2.28 | 1, 1.09 | 893, 1185, 1805 |
| At the turn only | 1, 1.35 | 1, 1.15 | 893, 1184, 1801 |
| 2 widths either side | 2, 1.31 | 3, 1.38 | 912, 1193, 1811 |
| 4 widths | 0, 1.01 | 1, 1.14 | 912, 1192, 1800 |
| 8 widths | 2, 3.26 | 0, 1.01 | 912, 1195, 1807 |
| As tips (`TIP_MAX_WIDTH` 0.03) | 4, 3.26 | 0, 1.01 | 912, 1195, 1807 |

Split at the turn only, each piece has nearly all of its curvature at one
end, the ratio of the cusp values at its ends is over 3, and it goes to
the sampled estimate, which reads 0.58 of the true error on such a piece
(the first sample is an eighth of the way in, past the turn). That put a
new miss in the random families (1.185) and one in the near-cusp survey
(1.33 on "random" at 1e-2). With 4 widths the first is gone and the second
is 1.12: the piece from 4 widths off the turn to the end of the curve,
whose cusp values are 1.97 and 0.92, inside the osculating estimate's
guard, with the error a tenth of the way in. The 3.26 with 8 widths was
not traced.

The near-cusp survey is otherwise as it was, row for row, in both modes.
Pieces at the maximum depth: 230 to none and 340 to 131 in the default
mode at 1e-6 (gap 1e-4 and exact cusps), 1026 to 754 in strict mode on
exact cusps, and 4 new ones in the default mode at gap 1e-4 and 1e-4.

**Why not wider.** The samples are an eighth of a piece apart, so the
honest limit is about 0.06, and the band 0.03 to 0.1 still has misses. But
the tiger has no curve with a turn under 0.01, 6 from 0.01 to 0.03, 31
from 0.03 to 0.1 and 79 from 0.1 to 0.3 (of 336; 210 have no interior
minimum of the speed), and those 37 take a third of its time already
(26.5 of 80 microseconds at 1/256). Split at the turn only, a limit of
0.06 cost 3% on its strokes and 0.1 cost 5%, with 1% more segments: every
piece next to a turn fails the ratio guard and takes the slower path. At
0.03 the cost is the root filter passing 76 curves instead of 41, which
does not show.

**Dropped.**

- *One more osculating sample at the turn, on accept* (the "soft tip"):
  20 to 12 misses in the first band, 2 to 5% slower on the tiger from a
  filter width of 0.1 and a check in the hot path.
- *`ESTIMATE_MAX_CUSP_RATIO` of 8* in place of 64 cures every miss next to
  a turn split alone, but costs 2.5% in segments and up to 9% in time on
  the tiger at 1/256. 16 changes nothing.
- *Two more samples for the sampled estimate*, at 1/16 and 1/32 from the
  end with the larger curvature: no change to any miss. Its value itself
  falls short there, not only its sampling.
- *The sign pattern of `C'.C''` in the root filter*, to keep curves whose
  slow point is at an end (a short handle) off the cold path: no gain in
  time, at any width.

**Where it stands** (`a1c8150`, M5 Max). Whole tiger strokes, baseline and
pair: 121 and 52 microseconds at 1/32 (1629 and 1703 segments), 500 and 80
at 1/256 (3046 and 2279), 1562 and 126 at 1/2048 (6454 and 3039): 2.3,
6.3 and 12.4 times. Against `f2e3592` in the same runs the pair takes 1%
longer at 1/32, 1% less at 1/256 and 4% less at 1/2048.

| `offset_tiger` (baseline / default / strict) | 1/32 | 1/256 | 1/2048 |
|---|---|---|---|
| Segments | 838 / 912 / 1673 | 1959 / 1192 / 3091 | 5215 / 1800 / 4651 |
| ns per call | 168 / 78 / 239 | 725 / 122 / 548 | 2296 / 200 / 865 |
| Calls over tolerance | 6 / 0 / 0 | 25 / 1 / 0 | 50 / 0 / 0 |

| Random families | Baseline | Default | Strict |
|---|---|---|---|
| Geometric mean, ns | 322 | 120 | 210 |
| Nearly straight, gentle, wild | 104, 208, 1548 | 43, 71, 565 | 60, 125, 1220 |
| Segments | 17232 | 18103 | 25376 |
| Calls over tolerance (of 5,400) | 178 | 0 | 0 |

The 135 ns for the default mode on the random families in earlier tables,
and 59 for its nearly straight curves, were artefacts: they were timed in
the run that also measures errors, where the first function timed came out
slow on its first family. Timed alone the default mode was 120 to 122 ns
at `f2e3592`, and the nearly straight family 42 to 44.

**Splitting twice on a bad estimate, measured and dropped.** Rejections
are 39% of the evaluations on the tiger at 1/2048 (1,015 to 1,800 accepted
pieces), and a rejected piece's osculating samples say how far over each
half is, so a half that is far over could be split again without being
evaluated. Every piece was logged with the largest sample ratio (estimate
over limit) its parent had on its side of the split, one side at a time:

| Parent's ratio on this side | Tiger 1/32 | 1/256 | 1/2048 | Wild 1e-4 |
|---|---|---|---|---|
| under 1 (accepted / rejected) | 57 / 1 | 92 / 0 | 208 / 0 | 1635 / 2 |
| 1 to 4 | 122 / 14 | 380 / 11 | 626 / 24 | 8453 / 188 |
| 4 to 16 | 37 / 38 | 105 / 47 | 378 / 112 | 5342 / 1077 |
| 16 to 64 | 0 / 7 | 65 / 83 | 126 / 149 | 2190 / 2624 |
| 64 to 256 | 0 / 0 | 0 / 7 | 23 / 109 | 557 / 2697 |
| 256 to 1024 | 0 / 0 | 0 / 4 | 26 / 50 | 112 / 2215 |
| over 1024 | 0 / 0 | 0 / 0 | 0 / 7 | 39 / 3337 |
| no such parent | 696 / 98 | 550 / 276 | 413 / 564 | 6739 / 9108 |

The ratio predicts poorly: a side at 16 to 64 times the limit is accepted
almost as often as not once it has its own fit. A threshold of 256 would
skip 3% of the evaluations on the tiger at 1/2048 for 1.4% more segments,
and none at the looser tolerances; on wild curves at 1e-4, 12% for 0.6%.
Most of the tiger's rejections are of whole curves, which have no parent.
Not worth a flag through the recursion. (The other end of the table, not
evaluating a side whose parent had it within the limit, would save 5 to 7%
of the evaluations and accept a few pieces that their own samples reject.)

## The tiger at 1/32 (20 November 2026)

Whole strokes of the Ghostscript tiger with the pair, at a tolerance of
1/32, is the number other people look at. This is where its time was at
`bba4e43`, what came off, and what did not.

**What the tiger is.** 336 stroked cubics. 250 have a zero-length first
handle (`p0 = p1`, from the PostScript `v` operator) and 11 a zero-length
last one, so three quarters of the offsets have a vanishing end
derivative. The half widths are 0.008 to 0.2 against a tolerance of
0.031.

| Curves, at `bba4e43` | Number | ns a pair | Share of the time |
|---|---|---|---|
| One segment a side | 277 | 75 | 46% |
| More | 59 | 420 | 54% |

The one-segment pairs cost 55 to 75 ns whichever test accepts them
(hairline 36 pairs, hull 89, osculating 152): the root, the fit and the
emitting are most of it, not the decision. The others are hooks: a
zero-length first handle, a turn of 130 to 170 degrees concentrated in a
tenth of the parameter, a half width of 0.2. Their inner side has a cusp
next to the start (the stub) and often two more at the turn, so four
pieces before anything is fitted.

**Both sides from one pass** (`757aab5`). For each piece that both sides
reach together, the sides nearly always take the same route: at the root,
36 pairs by the hairline bound, 89 by the hull bound, 36 by the one-point
fit, 140 by arc drawing after the one-point fit clamped a handle on both
sides at once (with a zero-length handle the clamps are mirror images),
19 by the two-point fit. With the same fit and no clamp, `D` on one side
is minus `D` on the other, and the osculating estimate differs in the
sign of `A`, `B` and `d` only: `approximate_pair` takes the eight samples
once and decides twice, to the last bit of what `approximate` gives. 306
of 323 such pieces go that way. The earlier attempt at this ("Where the
time goes") gave up because one side clamped; both do.

**The cheap bounds before the cusps** (`a1e9287`). The hairline and hull
bounds are on the distance at equal parameter, which holds across cusps.
They were tried after a piece had been split at its cusps, and the hull
bound at the root only. Now `accept_arc` comes first on every piece whose
end normals times `d` are within the tolerance of each other, which no
such bound can beat (the old test, on the sine of the turn, let half turns
through: 144 of its 319 attempts failed). Four thin strokes with a sharp
turn (half width 0.019, under the tolerance but over half of it) went
from 10 segments and 790 ns a pair to 8 and 200.

**The estimate past a cusp** (same commit). A piece with a negative cusp
value at both ends runs backward along a curve that is as smooth, and the
estimate is the same with the sign reversed. Those were half the pieces
on the slower path (32 of 63 at 1/32). Against the sampled estimate on
11,955 such pieces of random curves it accepts 21 that the other rejects.

**The cusp search** (`a793839`). It took 13 evaluations a cusp, not the
6 to 8 its comment said. Newton's method arrives from one side, the far
end of the bracket never moves, and a last step that rounded to nothing
was taken for leaving the bracket, which sent the search back to
bisecting the whole of it: 40 evaluations for 5 of the tiger's 39 cusps.
A step under the accuracy asked for now ends it: 7 a cusp.

**ITP in its place, measured.** With `solve_itp` on the cusp value, as in
`offset.rs`: 21 evaluations a cusp, and whole tiger strokes at 1/32 took
59.7 microseconds against 46.9. Each evaluation is a square root and a
division, and each ITP step waits on the last through another division,
so a cusp costs about 320 ns against 50. Bisection steps overlap in the
pipeline; these do not.

**Smaller things.** A linear fit that will be clamped is no longer built
first (`clamps`). The cusp value at a shared split and the hull bound of
an unclamped arc are computed once for both sides. `Sharing`, `Shared`
and `Unshared` are gone: what was left for them to share was the fit on
17 pieces of 323, and strict mode, which is 1.5% slower with the pair.

**Where it stands**, `bba4e43` against `d7b0e94` in the same run, M5 Max:

| Whole tiger strokes | 1/32 | 1/256 | 1/2048 |
|---|---|---|---|
| Pair before, µs | 52.5 | 80.2 | 126.5 |
| Pair now, µs | 44.0 | 71.6 | 115.1 |
| Change | -16% | -11% | -9% |
| Baseline (`offset.rs`), µs | 121.4 | 501.3 | 1565 |
| Baseline over pair | 2.8 | 7.0 | 13.6 |
| One side at a time, before and now, µs | 59.8, 57.6 | 90.4, 91.7 | 144.3, 147.6 |
| Segments, before and now | 1703, 1694 | 2279, 2280 | 3039, 3041 |

| `offset_tiger`, default and strict | 1/32 | 1/256 | 1/2048 |
|---|---|---|---|
| ns a call, before | 77, 238 | 122, 549 | 201, 864 |
| ns a call, now | 75, 205 | 126, 498 | 208, 829 |
| Segments, now | 903, 1648 | 1193, 3053 | 1802, 4645 |
| Calls over tolerance, now | 0, 0 | 1, 0 | 0, 0 |

Random families, one side at a time: default 120 to 124 ns (nearly
straight 42 to 45, gentle 72 to 77, wild 569 to 547), strict 211 to 214;
0 of 5,400 over in both modes, as before. The near-cusp survey has no
row with a larger worst error, fewer segments at the loose tolerance
(hairpins 4588 to 4044), and fewer pieces at the depth limit at 1e-6
(122 to 6, 131 to 55, strict 754 to 377). The sharp-turn survey is
unchanged.

**A metric that rejects a piece but accepts its halves, measured.** The
idea: skip the estimate on a half when its parent's result already says
it will pass. Every piece with a parent was logged with the parent's
largest sample ratio (estimate over limit) on its side of the split, and
its own:

| Parent's ratio on this side | Tiger 1/32: pieces, rejected | Wild: pieces, rejected | Hooks: pieces, rejected |
|---|---|---|---|
| under 0.25 | 14, 0 | 411, 0 | 123, 0 |
| 0.25 to 0.5 | 13, 0 | 820, 7 | 397, 2 |
| 0.5 to 1 | 31, 1 | 2146, 23 | 1519, 13 |
| 1 to 2 | 63, 3 | 7133, 158 | 4940, 75 |
| 2 to 4 | 71, 9 | 5960, 368 | 4090, 175 |
| 4 to 8 | 67, 30 | 4963, 708 | 3521, 333 |

There is no ratio above the limit that guarantees the halves: the error
of a hook sits in a tenth of the piece, and halving does not divide it
until the split lands there (a parent at 2.4 times the limit had a half at
1.1). Below a quarter of the limit no half was rejected, but that is 5%
of the halves on the tiger at 1/32, a quarter of a microsecond.

**Also tried and dropped.**

- Splitting at the tangent bisector more often (`SUBDIVIDE_THRESH` 0.3,
  0.6, 1): 1697 to 1690 segments against 1703, and 2 to 6% slower.
- The cheap bounds at the root before the slow-point filter, with the
  two sides' state built together (`pair_at_root`, shared end curvature):
  no gain; the curves that pass the filter built their state twice.
- A start for the cusp search (the line through the cusp values at the
  ends; the stub's cusp to leading order with the double root at the end
  divided out): 6.4 evaluations a cusp at 1/32 and 3.7 for stubs at
  1/2048, 1% of the tiger's time for 45 lines.
- An equal-parameter check at the eight samples before the sampled
  estimate: passes 41 of the 63 slow-path pieces at 1/32 but 45 of 197 at
  1/2048, and those past a cusp no longer get there.

**What is left of the time**, sampled at `0c4fff4`: the recursion with
the pair's estimate 44%, the one-sided `approximate` 13%, the stroker
12%, `hull_bound` 6% (before it was shared), the root 5%, `subsegment`
3%.
The eight-sample loop is 227 vector instructions for both sides and near
what the arithmetic needs without fused multiply-adds. What costs is the
number of pieces: the hooks take 1.45 fits a segment, and their inner
side is cut by cusps before anything is fitted. A hidden pair of cusps
found at a midpoint split leaves two slivers (item 15).

## Trimming again (23 November 2026)

Strict mode and the pair both stay: strict mode by consensus, and the
pair because whole tiger strokes at 1/32 are 30% slower without it (45.4
against 58.9 µs; it was 8% at `0403eaf`, before both sides were decided
from one pass). So this pass looked at the default mode's own code, with
the tiger at 1/32 as the measure that matters and tighter tolerances
secondary. Each change was measured alone, interleaved with `9fc9f20`, three
runs, M5 Max; accuracy by the near-cusp, sharp-turn and vanishing end
derivative surveys and the random families.

| Change | Non-test lines | Tiger strokes, pair, 1/32 | Elsewhere |
|---|---|---|---|
| Midpoint subdivision everywhere (tips still split toward) | -57 | same 1694 segments, -2.5% | strict +1.2% segments at 1/32, default +4% at 1/2048; one more sharp-turn call over 1.05 (1.44 against 1.41) |
| Osculating estimate in place of the sampled one | -117 net | 1701 segments, -1.5% | wild curves at loose tolerances up to +19% segments; no worst case worse; one near-cusp row 1.24 to 0.98 |
| No estimate at all after the cheap bounds | -127 | 1718 segments, +1 to 2% | wild curves +18 to 27% segments, +12 to 19% time |
| One-sided calls through the pair | about -90 | unchanged | one-sided calls 73% slower (78 to 135 ns), strict 81% |

The first two are adopted. Non-test code goes from 1756 to 1577 lines.
Together, interleaved with `9fc9f20`:

| Whole tiger strokes, pair | 1/32 | 1/256 | 1/2048 |
|---|---|---|---|
| Before, µs | 45.5 | 74.3 | 119.4 |
| After, µs | 44.5 | 77.6 | 114.3 |
| Segments, before and after | 1694, 1701 | 2280, 2365 | 3041, 3113 |

`offset_tiger` (default, strict): 910 and 1668 segments at 1/32 against
903 and 1648; strict mode 5 to 7% faster at every tolerance, from the
cheaper subdivision. No call over tolerance at 1/32 in either mode. The
surveys are unchanged except the sharp-turn band of width 0.03 to 0.1,
which neither change makes worse alone: at a tolerance of 0.01, 2 calls
over 1.05 (worst 1.43, against 0 and 0.98), and at 0.25, 8 (worst 1.44,
against 7 and 1.41). That band is item 13.

**The slow-point solve** (`CubicBez::slow_points`). Replacing both of its
shortcuts with a plain `solve_cubic` (41 lines) made whole tiger strokes
with the pair 2.5% slower at 1/32. Measured separately, four runs each:
without the quadratic for a zero-length handle (14 lines), 45.0 µs
against 43.9; without the Newton step from the closed-form start (27
lines), 43.7. Three quarters of the tiger's offsets have a zero-length
handle, and the quadratic takes one square root where the cubic solve
takes several. The Newton step is removed; the surveys are unchanged but
for segment counts within 0.4% and pieces at the depth limit at 1e-6
(65 to 54, 377 to 344, 234 to 236).

**The fallback estimate.** The osculating estimate replaces the sampled one
in `accept_uncertified` with its slip limit on every fit (on the first
try it applies to two-point fits only) and the sampled estimate's cusp
ratio limit of 64 (`FALLBACK_MAX_CUSP_RATIO`) in place of 3. A cusp at a
sample rejects, and the piece is subdivided. The sampled estimate fitted
a polynomial through 7 Newton steps on the foot point and searched a grid
for its extrema; it was tighter (p99 1.02 against 1.29) but cost more,
and it was reached by few pieces on the tiger.

**Midpoint subdivision.** Open item 2 since "After the tips". The angle
split's uneven splits were what the tips were the worst case of; with
the tips split first and pieces running toward one split at the
geometric mean, the midpoint does as well. This removes the tangent
solve, `keep_inside` and `SUBDIVISION_MIN_SHARE`.

**Against `0403eaf`** (`offset2-vectorize`, the last commit before the
tips), at `9fc9f20`: whole tiger strokes with the pair 14% faster at 1/32
(52.8 to 45.3 µs), one side at a time 3% slower; random families in the
default mode 5% slower (124 to 130 ns geometric mean), strict unchanged.
Non-test code 1346 to 1756 lines: the tips +430 with `cubicbez.rs`, the
one-pass pair +161. With the near-cusp survey's cusps at random
parameters (its generator at `9fc9f20`), `0403eaf` was up to 67,894 times
over in the default mode and 169,850 in strict, with about 13,500 pieces at
the maximum depth; `9fc9f20` is within 1.24 and 0.99, with 61.

**The baseline, with the end subdivision fixed.** `offset.rs` with the
fix on `offset-split-at-end` (5693b35), applied to `9fc9f20`: whole tiger
strokes at 1/32, 1/256 and 1/2048 take 125, 216 and 438 µs, against 126,
520 and 1615 unfixed, with 1629, 2146 and 2931 segments against 1629,
3046 and 6454. So the pair's lead over the fixed baseline is 2.8, 2.9
and 3.7 times, not 2.8, 7.0 and 13.6. Calls over tolerance in
`offset_tiger`: 6, 17 and 24 of 672, against 6, 25 and 50.

**The order of the fits**, measured with a scratch test: 60 random cubics
in a 100 unit box for each placement, `d` of 1 and 10, pieces of 0.4 down
to 0.4 / 256 in the parameter, error by the Fréchet oracle, the exponent
from each halving:

| Fit | Around an inflection | No inflection |
|---|---|---|
| Linear two-point | 6.3, 6.1, 5.7, 5.5, 5.2, then 5.05 | 5.8, 4.9, 4.3, then 4.0 |
| One-point | 3.0 centred on it, 4.0 off centre | 5.7, 5.4, 5.2, then 5.0 |
| Arc drawing | 4.0 | 4.0 |

Linear two-point is fifth order at an inflection, not sixth, though it
looks sixth on large pieces. Presumably its normal error, with zeros at
both ends (double) and at the two fitted points, is sixth order, and the
slip term `kappa d slip^2 / 2` is fourth order in general but fifth where
`kappa` vanishes in the piece. One-point is worse than arc drawing at an
inflection, which supports choosing two-point there.

**Where the ideas came from.** Checked against `fast-stroke`, Raph's
research repository, counting a commit as human unless it has a Claude
co-author trailer. Raph's: the one-point fit (`d7bd193`) and the
derivative of its error at 1/2 as a rough measure (`fe88c4c`; it
survives only as `r_slope` in the strict `one_point_bound`); linear
two-point (`774ca2d`) and choosing it by inflection (`bece728`, the
`hybrid` branch); arc drawing (`6be3e0b`); the osculating parabola
estimate, with its 8 samples at interval midpoints and the 1.2 factor
(`ca05460`, `4e9deb4`); subdivision by angle (`1de3286`); the error being
linear in `(a, b)`, which makes the fits linear solves; the slip term;
the method of the Fréchet oracle (`5859687`); clamping a handle by the
cusp sign (`03c9a48`); and finding cusps from minima of the speed
(`6333074`). Also from Raph, in `offset.rs`: `A = C + D` with `a` and `b`,
the cusp value and finding cusps by its sign, and `regularize_cusp`.
Laurenz Stampfl's: the branch-free osculating samples (`0403eaf`). Not
human: `bezfit`'s `bound/cheap.rs` and "Cheap Offset Error Bounds"
(`c0b0d72`, `6091b82`, `6d517d8`), which these notes call the original
code and notes, so the certificate, the strict bounds, `q_bound` and
`ROUND_ULPS`; and `bezfit`'s `min_frechet.rs`, the source of the removed
midpoint G1 fit, whose human precursor is on `failed_k4`.
