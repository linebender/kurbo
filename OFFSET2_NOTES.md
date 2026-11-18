# Notes on `offset2.rs`

These notes record the reasoning, measurements and open questions behind
`kurbo/src/offset2.rs`, so that work can continue without the conversation
that produced it. They were written in November 2026 against kurbo commit
`a5f97ff` (0.13.1).

All timings below are from a 2.1 GHz Xeon sandbox, not from the machine the
code is meant for. All accuracy figures are from random cubics with control
points in a 100 unit box, unless stated. Treat both as indicative.

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
   (unless strict) the sampled estimate times 1.25.
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

The error measure in `offset_compare`, and in the tests, used the normal
of the source at exactly `t = 0`, which is NaN there. The nearest-sample
search then stuck at index 0. The tests and `offset_tiger` now keep the
parameter 1e-9 inside the ends.

## Not done

Roughly in the order I would take them:

1. ~~The reason near-lines at tight tolerance cost more than gentle
   curves.~~ Does not reproduce on an M5 Max; see the performance pass.
2. Two-sided output. With `D = d W`, `P` is even in `d`, `Q` is odd, and
   only `beta` differs between the sides. Estimated at 1.3 times one side.
3. The `kappa d` test in the chooser.
4. The split certificate, which certified about 8% more pieces.
5. The mirrored certificate for the G1 bound.
6. A rounding allowance. The bounds are strict down to a small multiple of
   machine epsilon times `d`.
7. The n^6 fits, switched to deep in the recursion.
8. Strong correctness: locate every cusp, report reversals, and add evolute
   sections, which need their own fit and bound.
9. Performance. Nothing here is tuned.

## Where this departs from the original notes

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
