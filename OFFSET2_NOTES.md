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

1. **Rounding allowance.** The original adds 8 ulps of the largest
   coordinate to every bound (`ROUND_ULPS` in bezfit's `bound/cheap.rs`).
2. **Newton cusp solve and depth 12 in `offset.rs`**, on the
   `offset-split-at-end` branch, so that benchmarks compare bounds and
   fits, not solvers and depth limits.
3. **The midpoint G1 retry**: decide whether to adopt it, and speed up its
   solve ("The midpoint G1 fit").

Medium:

4. **Certifying each half separately**: 2 to 5 more points of
   certification; port from the original's `split` option, with
   remainders per half ("The original certificate, compared").
5. **The original's tighter G1 bound levels** (`FootCubic`, `HalfFit`,
   and FootWeighted). The G1 bound is loose in the tail on the midpoint G1
   fit (p90 of bound over error 25 to 34 times).
6. **Both sides in one pass.** Estimated at 1.3 times one side. Deferred.
7. **Another corpus**, with ordinary end handles, to see how much of the
   tiger's behaviour is the tiger's.

Large, or open questions:

8. A certificate that allows `beta` to vanish at an end, for zero end
   derivatives. Possibly not worth a special case.
9. The mirrored certificate for the G1 bound.
10. Strong correctness: locate every cusp, report reversals, and add
    evolute sections, which need their own fit and bound.

Done or dropped: misses at the maximum depth (depth raised from 8 to 12),
the near-line slowdown (did not reproduce), the end subdivision bug (fixed
here, and in `offset.rs` on `offset-split-at-end`), the error oracle (now
Fréchet), the `kappa d` chooser and skip-level subdivision (tried,
dropped), tuning the estimate's safety factor (not needed).

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

### Whole strokes

`StrokeOpts::offset2(true)`, a hidden switch, makes the stroker use
offset2. `cargo run --release --example stroke_tiger -- path/to/tiger.svg`
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
