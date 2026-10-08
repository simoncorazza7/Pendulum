# Swing fits for 0.7 (upright reference) and 0.10 (pendulum identification)

Written in advance (2026-10-08) and validated on synthetic swings. Real recordings come from
`pendctl --mode encoder-log` (needs a go: it reads the real encoder inputs only).

## Key point: c_bottom must be re-established after every program start
The encoder is incremental with no index pulse. Every decoder (old `Encoder` and new) starts counting
at 0 wherever the pendulum is when the program starts. So **c_bottom is only valid within one run of
the program**. The 0.7 "re-runnable command" must run on the Pi at start-up, without numpy/scipy:
1. Identification (0.10, once, on the PC) gives the hanging damped frequency w and decay σ.
2. At start-up the pendulum hangs. Simon gives it a small swing (2–5°), and the runtime records the edge
   crossings for ~3–5 s.
3. `core::fitOffset(t, u, w, σ)` (linear least squares: u = c + e^{−σt}(a cos wt + b sin wt))
   returns c_bottom with its standard error. On synthetic data, 3 s give ±0.01 counts.

"Hanging still" is not a substitute: with Coulomb friction the pendulum stops anywhere in a stiction
band (synthetic example: ±0.2 counts; the real width comes from 0.10). The fit averages over that band.

## Method: fit edge crossings, not sampled counts
Each encoder edge is an exact crossing of a count boundary: at the edge that raises the count to n, the
continuous position is u = n (lowering to n: u = n + 1). Fitting (t_edge, u) avoids the
staircase bias of sampled counts and gives sub-count precision. Runtime convention:
count = floor(u), so θ = wrap((count + 0.5 − c_bottom − CPR/2)·2π/CPR)
(`core::thetaFromCounts`, the centre of the count interval).

## Tools
| Tool | Purpose |
|---|---|
| `py/pendel/decoders.py` | decode raw encoder-log events (kernel edge type or toggle) → counts → edge positions u |
| `py/pendel/swingfit.py` | `fit_damped` (c, A, σ, w, φ with standard errors); `fit_offset_linear` (c with w, σ known); `windowed` (amplitude, frequency, offset per 2-period window); `envelope_models` (exponential = viscous, linear = Coulomb, combined dA/dt = −σA − k); `omega0_from_windows` (w(A) = w0(1 − A²/16) → w0) |
| `py/fit_swing.py RUN [--t0 --t1]` | all of the above on a recording; prints c_bottom, w0, T0, L_eq, b/J, Coulomb γ, stiction band; flags a biased single fit; saves `swingfit.png` |
| `src/core/swingfit.{h,cpp}` | `fitOffset()` for the Pi (3×3 normal equations), golden-tested against Python |
| `py/test_swingfit.py` | validation on simulated nonlinear swings (10 checks) |

Derived quantities: b/J = 2σ (viscous), Coulomb γ = π·w0·k/2 (rad/s², torque/J), stiction band
±γ/w0², L_eq = g/w0², ζ_hang = σ/w0.

## Validation (synthetic: w0 = 2π·0.98 rad/s, b/J = 0.08 1/s, γ = 0.02 rad/s², c_bottom = 123.37)
| Check | Result |
|---|---|
| 4° swing, 20 s: c_bottom (damped fit) | 123.374 ± 0.014 (true 123.37) |
| same, linear fit with known w, σ | 123.374; with only 3 s: 123.365 |
| reversed count direction | passes |
| 30° swing: w0 extrapolated to A → 0 | within 0.08 %; A² coefficient 0.068 (theory 1/16) |
| 30° swing: combined envelope | σ = 0.0405 (0.040), k = 2.03e-3 (2.07e-3) rad/s |
| viscous only / Coulomb only | envelope shape correctly identified (rms ratio ≈ 100) |
| end-to-end, synthetic encoder-log run (raw A/B events, 25° swing, 40 s) | w0 6.1581 (6.1575), b/J 0.0812 (0.08), γ 0.0192 (0.02), "both" |
| same: c_bottom | **single damped fit biased by 0.19 counts** (large swing); windowed median −166.618 (true −166.63) |

Lesson: for c_bottom use a **small** swing (brief 0.7), or the windowed median.

## Rig procedures (later, each recording needs a go for encoder-log)
- **0.7:** pendulum hanging and still, start recording, small swing (2–5°), let it decay for ~20 s, stop.
  `py/fit_swing.py` → c_bottom ± se. Repeat 3× to check repeatability (target < 0.2 counts).
- **0.10:** swings from ~5°, ~10° and ~30°, at least 3 each, each recorded until nearly at rest.
  `py/fit_swing.py --t0 --t1` per swing → w0, b/J, γ with spread across repeats.
