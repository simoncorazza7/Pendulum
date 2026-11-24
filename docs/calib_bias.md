# Start-up c_bottom calibration on the 2026-11-02 pivot: is it biased? (log analysis, 2026-11-20)

Status: log analysis and simulation only (`py/calib_bias.py`). Nothing in the runtime, the config, `py/pendel` or the
launchers changed, and nothing ran on the Pi. Question from `docs/offset_estimator.md` §6: in 3 of 4 new-pivot
swing-up sessions the start-up fit differed from the hanging equilibrium fitted after the runs by 0.6–0.8 count.

**Result.**
- **The start-up fit is not biased; it is noisy.** All 18 new-pivot start-ups were replayed exactly and compared with
  the turning-point centre of the same swing (§4):
  - error mean −0.00, sd 0.19, max 0.40 count;
  - on synthetic swings with a known centre it is unbiased too (mean within ±0.01).
- **The error depends on where the 3 s window falls in the swing cycle.** The fixed exponential envelope
  (`calib.swing_sigma` 0.12) cannot follow a Coulomb-dominated decay:
  - sd 0.07–0.16 count on synthetic swings, 0.12–0.17 on the 11-02 decay;
  - single start-ups reach ±0.4 (11-03 10-02-18: −0.40; 09-52-36: +0.38).
- **The acceptance test is a coin toss.** It needs two fits 0.5 s apart within 0.2 count. The two windows share 2.5 s,
  so it passes common errors (10-02-18: −0.86 and −0.97, both 0.3–0.4 off). It also rejects good swings by chance
  (09-52-36: accepted only on the third swing, at 61 s).
- **Most of the §6 disagreement is a convention artefact.** `offset_check.py centres` places the edges at count ∓ 0.5.
  The calibrator and `core::thetaFromCounts` use count = floor(u), so the after-run values in §6 are 0.5 count too low.
  Corrected, start-up minus after-run is −0.51, +0.32, +0.11 and +0.09 count (§3).
- **Two real effects remain.**
  - The start-up fit's own error (10-02-18: −0.40 on its own swing).
  - **A hanging reference that moves.** In 09-52-36 it moved by −0.7 count between two start-up swings, after the
    pendulum was handled, with the cart not yet moved. Within one session the after-run centres span 0.5–0.6 count
    (§7). Neither is a whole count, so neither is lost edges.
- **None of this explains the large offsets.** The balances felt +1.2…+1.4 count in three of four hand-catch
  sessions, with start-up errors of +0.01…+0.19. Swing-up balances scatter by up to 3.8 count within one session (§8).
- **Proposed fix (not built): a turning-point calibration** (§9). It takes the centre from consecutive turning points
  and needs no ω, σ or friction model.
  - Synthetic swings: ±0.022 count (sd), against ±0.070 today.
  - The 11-02 decay: ±0.021, against ±0.123.
  - It accepts every logged start-up swing, a median 0.4 s sooner.
  - The cheaper alternative, a 5-column basis in `core::fitOffset`, roughly halves today's error.
- **Side findings.**
  - 14 of 18 start-up swings show a **stall just after the turning points**: the encoder edges stop for tens of
    milliseconds (35 ms in the example of §7), then catch up faster than a free swing can. That points to an elastic
    element with friction between the encoder shaft and the swinging rod.
  - With the reference jumps, this makes the shaft–coupling–rod joint worth a mechanical look.

```
.venv/Scripts/python py/calib_bias.py sessions    replay, turning points, after-run centres, felt offsets   ~10 s
.venv/Scripts/python py/calib_bias.py bench       the 11-02 free decays: every 3 s window, fit variants      ~5 s
.venv/Scripts/python py/calib_bias.py fix         turning-point calibration vs today's (logs, decays, sim)   ~40 s
.venv/Scripts/python py/calib_bias.py synth       synthetic swings with a known centre                       ~30 min
```

All need the run directories under `logs/` (not in git). Units: counts of the 2400-count encoder. 1 count = 0.15°,
and a reference error of 1 count drives the cart at g·δ = 0.026 m/s².

## 1. The start-up calibration

`rt::SwingCalibrator` (`src/runtime/swing_calib.cpp`) runs with the driver off. Every 0.5 s it fits
u = c + e^{−σt}(a cos ωt + b sin ωt) to the edges of the last 3 s (`core::fitOffset`). Here ω = ω₀(1 − A²/16) for
the window's half-range A, ω₀ = `plant.w0` 4.97 and σ = `calib.swing_sigma` 0.12. It accepts when:
- the amplitude is 2–20°,
- the residual is below 2 counts and the standard error below 0.05,
- two consecutive fits agree within 0.2 count.

Edge convention (calibrator, `core::thetaFromCounts`, `pendel.decoders`): count = floor(u). An edge that raised the
count to n is at u = n, and one that lowered it to n is at u = n + 1.

## 2. Replay

`replay_startup` feeds the calibrator sample by sample with the `data.csv` ticks (t_ns, counts) and the `edges.csv`
edges (ts_ns, count_after, tick), using each run's `config_used.conf` values. **All 18 new-pivot start-ups reproduce
`meta.json` `c_bottom_fit_counts` to ≤ 5·10⁻⁵ count.** They were accepted at 12–20°, 6–61 s after program start.

## 3. The §6 comparison mixes two edge conventions

`cmd_centres` in `py/offset_check.py` set `u = cc − 0.5` for a rising count and `cc + 0.5` for a falling one (fixed
2026-11-20, §10)
(count = round(u)). The calibrator uses `cc` and `cc + 1`, so the fitted after-run centres come out exactly 0.5 count
lower. Same fits (`fit_damped`, first 3 s of every free swing after a run), both conventions (`sessions`; the round
column reproduces the printed `centres` values to 0.02):

| session | start-up fit | after the runs, as §6 (round) | after the runs, calibrator's convention | start-up − after (mean) |
|---|---|---|---|---|
| 11-03 10-02-18 | −0.97 | −0.87 … −1.08 | −0.37 … −0.58 | **−0.51** |
| 11-03 15-32-17 | +0.40 | −0.26 … −0.73 | +0.24 … −0.23 | **+0.32** |
| 11-04 09-15-27 | +1.86 | +1.13 … +1.46 | +1.63 … +1.96 | **+0.11** |
| 11-04 10-16-08 | −0.63 | −1.06 … −1.45 | −0.57 … −0.95 | **+0.09** |

So two sessions agree within the fits' own scatter and two do not, one in each direction. That is not the pattern of
a bias.

## 4. A reference: the turning-point centre

Neither fit can be the judge of the other, because both use a 3 s window and a model of the waveform. The reference
here uses only the turning points:
- **The extremum.** Around a turn, the boundaries n, n−1, … are crossed once on the way in and once on the way out.
  n_k = x_ext − κ·Δt_k² holds to parabolic order whatever the accelerations on the two sides are (Coulomb friction
  makes them differ). Δt_k is the time between the two crossings. The intercept over ≤ 8 pairs is the extremum, with
  no clock offset, common latency or frequency involved.
- **The centre**, from three consecutive extrema: c = (x₁ + 2x₂ + x₃)/4.
  - With Coulomb friction every half swing is a cosine about c + Δ or c − Δ (Δ = γ/ω₀² ≈ 2.6 counts here), and the
    formula is exact.
  - Viscous friction adds (1 − r)·[X(1 − r) + Δ(1 + r)]/4 ≈ 0.03 count, with opposite signs for max-first and
    min-first triples. Neighbouring estimates are averaged.
  - It needs no ω, σ or friction model. The same holds for any friction law that is symmetric in the direction of
    motion.

Validation (`synth`): nonlinear pendulum ω₀ 4.97, b/J 0.054, Coulomb 0.168. Encoder boundaries are displaced by a
0.1-count quadrature error plus 0.02-count line noise, with 2 µs timestamp jitter. Three swings from 24° each.

| case | 15–25° | 10–15° | 5–10° | 2–5° |
|---|---|---|---|---|
| 11-02 pivot: error of one estimate, mean ± sd | +0.001 ± 0.046 | +0.009 ± 0.043 | +0.001 ± 0.031 | +0.031 ± 0.024 |
| Coulomb 0.25 (50 % more) | +0.012 ± 0.053 | +0.005 ± 0.049 | −0.004 ± 0.047 | −0.018 ± 0.036 |
| viscous only | −0.003 ± 0.023 | −0.003 ± 0.022 | – | – |

Averaged over a 3 s window the error is ≤ 0.06 count. On the real 11-02 decay (`15-43-11`) the estimates give
+1.94 ± 0.06 over 15 estimates at 5–20° (+2.05 above 20°).

## 5. Is the start-up fit biased?

### Sessions (`sessions`): today's accepted fit against the turning-point centre of the same 3 s window

| session | kind | amp ° | start-up fit | turning points (n) | **error** |
|---|---|---|---|---|---|
| 11-02 15-46-08 | hand | 16.2 | +0.180 | +0.352 (5) | −0.17 |
| 11-02 15-52-23 | hand | 12.2 | +1.186 | +1.123 (4) | +0.06 |
| 11-03 09-44-23 | swing-up | 18.4 | −0.573 | −0.330 (5) | −0.24 |
| 11-03 09-52-36 | swing-up | 18.0 | −2.688 | −3.072 (5) | +0.38 |
| 11-03 10-02-18 | swing-up | 18.1 | −0.965 | −0.562 (5) | **−0.40** |
| 11-03 14-32-03 | hand (remote) | 17.8 | −0.272 | −0.183 (4) | −0.09 |
| 11-03 15-26-06 | swing-up | 18.2 | −513.232 | −513.307 (5) | +0.07 |
| 11-03 15-32-17 | swing-up (remote) | 15.2 | +0.398 | +0.123 (5) | **+0.28** |
| 11-04 08-58-08 | hand | 19.3 | +1.867 | +1.857 (5) | +0.01 |
| 11-04 09-09-03 | hand | 17.8 | −0.757 | −0.942 (5) | +0.19 |
| 11-04 09-15-27 | swing-up | 17.8 | +1.862 | +1.721 (5) | +0.14 |
| 11-04 09-20-36 | swing-up (remote) | 18.7 | +0.852 | +0.902 (4) | −0.05 |
| 11-04 09-34-02 | swing-up (remote) | 18.4 | +0.432 | +0.484 (5) | −0.05 |
| 11-04 09-36-34 | swing-up (remote) | 19.7 | +1.838 | +1.661 (4) | +0.18 |
| 11-04 10-06-40 | swing-up (remote) | 14.7 | −0.731 | −0.628 (5) | −0.10 |
| 11-04 10-07-34 | swing-up (remote) | 19.3 | −2.489 | −2.495 (4) | +0.01 |
| 11-04 10-08-36 | swing-up (remote) | 19.9 | −2.665 | −2.490 (4) | −0.17 |
| 11-04 10-16-08 | swing-up | 16.5 | −0.633 | −0.548 (5) | −0.09 |

The error has mean −0.003, sd 0.187 and max 0.40 count. Against the median of the whole free swing instead (the
operator stops it a few seconds later): −0.02 / 0.19 / 0.44. `fit_damped` on the same windows (free ω and σ) is no
better: +0.01 / 0.18 / 0.35.

### The 11-02 free decay (`bench`): every 3 s window, every 20 ms

`15-43-11` (after the bearing fix; driver off). Error against the turning-point centre at the window's middle.
Windows the calibrator would reject on rms or se are included.

| fit | 15–20°: mean / sd / max \|e\| | 10–15° | 5–10° |
|---|---|---|---|
| today: ω(A), σ 0.12 | +0.03 / 0.15 / 0.37 | −0.08 / 0.12 / 0.29 | −0.19 / 0.17 / 0.50 |
| σ from a free fit | +0.03 / 0.14 / 0.29 | −0.09 / 0.09 / 0.24 | −0.14 / 0.07 / 0.25 |
| `fit_damped` (all free) | +0.02 / 0.11 / 0.22 | −0.09 / 0.11 / 0.31 | −0.14 / 0.10 / 0.33 |
| 1, cos ωt, sin ωt, t cos ωt, t sin ωt | +0.01 / 0.11 / 0.21 | −0.08 / 0.07 / 0.18 | −0.16 / 0.04 / 0.24 |
| today + a Coulomb column (edge direction) | +0.03 / 0.14 / 0.35 | −0.08 / 0.11 / 0.30 | −0.18 / 0.16 / 0.49 |
| window = 2 periods | −0.00 / 0.19 / 0.45 | −0.13 / 0.25 / 0.71 | −0.11 / 0.45 / 0.93 |
| window = 3 periods | +0.06 / 0.32 / 0.74 | −0.09 / 0.20 / 0.56 | −0.11 / 0.38 / 0.82 |

`15-27-20` (before the bearing fix, Coulomb ≈ 0.31): today's fit has sd 0.33 / 0.43 / 0.73. Its acceptance test never
passes on that decay, as `CLAUDE.md` records.

### Synthetic swings (`synth`): 11-02 pivot, error against the true centre

| fit | 15–20° | 10–15° | 5–10° |
|---|---|---|---|
| today | +0.01 / 0.07 / 0.17 | −0.00 / 0.08 / 0.17 | −0.01 / 0.16 / 0.37 |
| `fit_damped` | +0.01 / 0.07 / 0.11 | −0.01 / 0.06 / 0.11 | +0.02 / 0.09 / 0.19 |
| 5-column basis | −0.00 / 0.05 / 0.10 | +0.00 / 0.03 / 0.05 | −0.00 / 0.02 / 0.03 |
| turning points in the window | +0.01 / 0.04 / 0.06 | +0.01 / 0.02 / 0.06 | +0.01 / 0.01 / 0.05 |

The same plant with viscous friction only: today's fit has sd 0.38 at 15–20°, and any free-σ fit has 0.02. With
Coulomb 0.25, today's sd is 0.15 / 0.23 / 0.36.

### Which fit was off in the four §6 sessions

| session | start-up fit − its own swing | after-run centres (turning points) | reading |
|---|---|---|---|
| 10-02-18 | **−0.40** | −0.25 … −0.45; start-up swing −0.56 | the start-up fit was off; the reference then rose ~0.1–0.3 |
| 15-32-17 | +0.28 against its window (+0.10 against the settled swing; the window holds the release) | +0.06 … +0.27; swing +0.30 | mostly the start-up fit; the reference also dropped ~0.1–0.2 |
| 09-15-27 | +0.14 | +2.13, +1.53, +2.00 | agree; the reference moves by 0.6 between runs |
| 10-16-08 | −0.09 | −1.07 after run 2, −0.53 at the end | agree at start; the reference moved −0.5 and back |

## 6. The suspects

- **Swing amplitude (12–20° at start-up, ~10° after the runs): not the cause.** At 10–20° the mean error shows no
  trend: −0.01 / −0.04 in the sessions, +0.03 / −0.08 on the decay, +0.01 / −0.00 synthetic. The scatter grows below
  10° (decay sd 0.17, synthetic 0.16), but no start-up was accepted there.
- **Decay model (σ 0.12 against Coulomb friction 0.168): the mechanism of the scatter, not a bias.**
  - Coulomb decay is linear in time. Its effective exponential rate is 2γ/(πω₀A) + β/2: 0.10 1/s at 17° and 0.15 at
    10°. A fixed σ is therefore wrong for most of every window.
  - The misfit leaks into c, with a sign set by the window's phase in the swing. On the decay at 15–20° it swings
    between −0.20 and +0.37 as the window slides, with the period of the swing.
  - Synthetic, viscous-only proof: with the right σ the fit is exact (sd 0.02); with σ = 0.12 the sd is 0.38.
  - Letting amplitude and phase drift linearly in the window (the 5-column basis) removes most of it on synthetic
    swings (0.05 / 0.03 / 0.02).
- **Window placement.**
  - Phase: see the previous point.
  - An integer number of periods makes things worse (sd up to 0.42 on synthetic and 0.45 on the decay, errors up to
    0.93). The window is then 2.5 s or 3.8 s, and the Coulomb square wave it would cancel is not the dominant error.
  - A Coulomb column (the edge direction) changes nothing: the sinusoid already absorbs the square wave's fundamental.
- **The acceptance test.** Two fits 0.5 s apart share 2.5 of 3 s, so their errors are correlated.
  - 10-02-18: −0.857, then −0.965 → accepted, 0.40 off.
  - 09-52-36: on two swings consecutive fits differed by +0.38 and +0.24, on a swing whose turning points agree within
    ±0.1. It accepted only the third swing, at 61 s.
  - Window timing a few ticks different decides between accept and reject there.
- **What the model does not cover.** On the session swings, `fit_damped` and the 5-column basis do not beat today's fit
  against the turning points (sd 0.18 for `fit_damped`). Each swing has a part common to all whole-waveform fits,
  absent from the turning points.

## 7. The hanging reference moves

- **09-52-36, start-up, driver off, cart not yet moved.**
  - The operator gave three swings. Swings 1 and 2 centre at −2.30 and −2.40, swing 1 stable from 47° down to 2°.
  - The pendulum was handled at 48–51 s. Swing 3 then centres at −3.05, stable from 32° down to 5°.
  - The 3 s fits see the same jump (medians −2.29 / −2.19 → −2.88), so it is not an artefact of the turning points.
  - After the swing-up runs the centre is −2.15 … −2.20.
  - `fix`: the turning-point calibration accepts swing 1 at −2.36; today's calibration accepted swing 3 at −2.69.
- **Between runs** (after-run free swings, turning points): 09-15-27 +2.13 / +1.53 / +2.00; 10-16-08 −1.07 / −0.53;
  09-34-02 +0.05 … +0.56; 09-44-23 −0.67 / −1.01.
- These moves are not whole counts, so they are not lost edges. `offset_estimator.md` §3 ("the count reference does
  not move between runs … within ±0.2 count") holds for the scatter of the fits, not for the turning points.
- **Turn stall.** The extremum regression leaves 0.02–0.04 count on clean turns. In 14 of 18 start-up swings it
  leaves 0.16–0.41 (`fix`, last column). It can change from one session to the next (11-04 08-58-08: 0.02;
  09-09-03, 11 minutes later: 0.21).
  - Example, 14-32-03, the maximum at 9.88 s, against the approach mirrored about the turn: boundaries 98 and 97 are
    re-crossed within ~7 ms, boundary 96 about 35 ms late. Over the next eight boundaries the delay shrinks to 23 ms:
    the encoder catches up faster than a free swing can.
  - A free pendulum cannot stall mid-swing and then outrun its own trajectory. Something elastic with friction sits
    between the encoder shaft and the swinging mass: a flexible coupling, a clamp, or the rod.
  - Symmetric stalls do not move a swing's centre. A joint that slips when the pendulum is handled or swung up hard
    would move it, and the upright with it.

## 8. What the balances felt

Drift-identity offsets per balance (the `offset_check.py replay` ground truth; + = c_bottom used above the true
upright − 1200), next to each start-up's own error:

| session | kind | start-up error | felt offset per balance (count) |
|---|---|---|---|
| 11-02 15-52-23 | hand | +0.06 | +1.32 |
| 11-03 14-32-03 | hand (remote) | −0.09 | +0.08, −0.10 |
| 11-04 08-58-08 | hand | +0.01 | +1.22, +1.21, +1.26 |
| 11-04 09-09-03 | hand | +0.19 | +1.38 |
| 11-03 10-02-18 | swing-up | −0.40 | −0.72, +0.35, +0.59 |
| 11-03 15-32-17 | swing-up (remote) | +0.28 | +0.10, +1.63 |
| 11-04 09-15-27 | swing-up | +0.14 | −1.02, −1.60 |
| 11-04 09-36-34 | swing-up (remote) | +0.18 | −1.26, −0.60 |
| 11-04 10-16-08 | swing-up | −0.09 | +2.34, −1.42, +0.22 |

- The +1.2…+1.4 of three hand-catch sessions is not a calibration error: their start-ups were within +0.01…+0.19 of
  their swings. Nor is 14-32-03's +0.0 explained by its start-up error of −0.09. All four ran with `encoder.trim` 0.
- The swing-up scatter (up to 3.8 count within a session) is 10 times the largest start-up error and 5 times the
  hanging-centre moves of §7.
- So a better start-up calibration removes up to ±0.4 count (0.010 m/s²). It does not touch the dominant offsets. The
  online estimate of `docs/offset_estimator.md` stays the tool for those.

## 9. Proposed fix: the turning-point calibration (not built; each rig step needs a go)

`tp_calibrator` in `py/calib_bias.py` runs causally, on the same edges the Pi already collects:
- Every reversal gives an extremum: ≥ 3 boundary pairs, regression rms ≤ 0.6 count. A clean turn leaves 0.02–0.04;
  the turn stall leaves up to 0.45.
- Extrema must alternate, at most 0.9 s apart (half period 0.63 s). Each consecutive triple gives a centre estimate.
- Accept when the last 4 estimates (6 extrema, ~2.5 periods) agree within 0.25 count (max − min), lie at 2–20°, and do
  not grow by more than 1 count (a push or a hand does). Then c_bottom = their mean.
- Pi cost: one 2-parameter regression per reversal over ≤ 8 points, and a ring of 6 extrema. No exp or cos, no
  `plant.w0`, no `calib.swing_sigma`.

Quantified (`fix`). Each calibration is started at every release time, every 0.1 s while the swing is below 24°:

| swing | calibration | accepted | error mean / sd / max \|e\| (count) | median time to accept |
|---|---|---|---|---|
| synthetic, 11-02 pivot | today | 216 | −0.005 / 0.070 / 0.15 | 3.0 s |
| | turning points | 330 | **+0.008 / 0.022 / 0.05** | 3.7 s |
| synthetic, Coulomb 0.25 | today | 30 | +0.069 / 0.138 / 0.19 | 3.0 s |
| | turning points | 210 | +0.003 / 0.020 / 0.04 | 3.7 s |
| 11-02 decay `15-43-11` | today | 72 | −0.028 / 0.123 / 0.28 | 3.0 s |
| | turning points | 113 | +0.004 / 0.021 / 0.05 | 3.7 s |
| 11-02 decay `15-27-20` (before the bearing fix) | today | 0 | – | – |
| | turning points | 49 | −0.009 / 0.029 / 0.05 | 3.7 s |

Notes on the table and the logs:
- On the decays the reference is a quadratic in time through all the decay's turning-point estimates (they scatter
  0.04 about it), so it partly shares data with the turning-point calibration. The synthetic rows are the accuracy
  test.
- On the 18 logged start-ups it accepts every swing, a median 0.4 s sooner than today's (between 42.7 s sooner and
  1.2 s later).
- Where both accepted the same swing (15 sessions), the two values differ by mean −0.03, sd 0.15, max 0.39 count.
  That is the change in c_bottom the fix would have made.
- In three sessions it accepted an earlier swing: 09-44-23 (−0.46), 09-52-36 (−0.32) and 10-16-08 (−0.17).

**Cheaper alternative.** Keep the calibrator and change the basis of `core::fitOffset` to
1, cos ωt, sin ωt, t·cos ωt, t·sin ωt: 5×5 normal equations, and no σ. That gives synthetic sd 0.05 / 0.03 / 0.02
(today 0.07 / 0.08 / 0.16) and decay sd 0.11 / 0.07 / 0.04 (today 0.15 / 0.12 / 0.17). It does not remove the per-swing
part of §6 or the correlated acceptance test.

**On the rig** (not built; each step needs a go):
1. Port `tp_calibrator` to C++ as a second calibrator next to `rt::SwingCalibrator`. Golden-test it against the Python
   on synthetic and logged edges.
2. Make it opt-in with a new key (e.g. `calib.method = fit | turning`, default `fit`). `demo.sh`, `swingup.sh` and
   `remote.sh` then behave as today until it is switched.
3. Watch first: run both in every session and write both values to `meta.json`. Switch only after they agree on the rig.
4. The reference moves of §7 limit any start-up calibration. The coupling, clamp or rod joint is worth a mechanical
   check; that is Simon's call.

## 10. Open

- **Why the reference moves** (0.5–0.7 count with handling or between runs), and the turn stall behind it. A slipping
  joint would also move the upright, which no start-up calibration can follow.
- **The +1.2…+1.4 hand-catch offsets and the swing-up scatter** are not calibration errors (§8).
- **Corrected 2026-11-20** after this study: `py/offset_check.py` `cmd_centres` now uses `cc` / `cc + 1` (the
  calibrator's convention; its output moved by exactly +0.5). `docs/offset_estimator.md` §6 now has the corrected
  table, and §3 carries §7's caveat.
