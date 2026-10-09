# Stage 0.11: velocity estimator

**Decision (2026-10-09, real data + identified plant): backward difference + 2nd-order Butterworth
low-pass at 20 Hz (`ctrl.estimator = bdlp2`, `ctrl.est_fc = 20`).** The synthetic study below chose LP2
30 Hz provisionally; the real-data run changed the corner.

## Why it matters: the D-term spikes (shadow run 13-36-33)
With BD+LP1 30 Hz every single count step (2.6 mrad) kicked ω by ~0.36 rad/s, i.e. Kd·ω ≈ ±2.4 m/s²
command spikes while the pendulum was held still near upright. The real encoder does not chatter at rest
(direction reversals only at swing turnarounds, 2 per period; at rest no edges at all), so the spikes come
from isolated count steps: hand tremor here, the ±1–2-count quantisation limit cycle when balancing.
Whatever the filter, one count step adds the same velocity Kd·q = 17 mm/s to the command; the filter only
shapes the pulse. What the shape costs is clipping at a_max, step-loss risk (peak acceleration) and, for
a smoother estimate, phase lag (delay margin).

## Real-data comparison (`py/estimator_real.py`, plot `docs/img/estimators_real.png`)
- Open loop: exact-edge recordings replayed through the runtime's 2 ms sampling, an edge visible once it
  reached user space (`read_ns`; latency p50 33 µs, p99 50 µs). *hold*: 10-31-06, hand holding the bob;
  *swing*: 09-32-26 free decay 14→5°, truth = non-causal smoothing-spline derivative.
- Closed loop: identified plant (ω₀ 4.931, b/J 0.024, Coulomb 0.0117), config gains (Kp 62.8, Ki 96.8,
  Kd 6.56, a_max 3, v_max 0.5), Axis cap 15 m/s²; catch from 3°, 8 s; tolerated extra delay by bisection.

| estimator | hold: Kd·ω RMS / peak (m/s²) | swing: err RMS (rad/s), lag (ms) | closed loop: RMS a / p99 a (m/s²), RMS θ (°), x(8 s) (m) | tolerated extra delay (ms) |
|---|---|---|---|---|
| BD+LP1 30 Hz (shadow run) | 0.48 / 2.39 | 0.112, 5.7 | 0.93 / 2.49, 0.053, +0.20 | 56 |
| BD+LP1 15 Hz | 0.36 / 1.51 | 0.067, 11.3 | 0.55 / 1.49, 0.053, +0.19 | 50 |
| BD+LP2 30 Hz | 0.44 / 1.47 | 0.048, 8.3 | 0.66 / 1.62, 0.048, +0.24 | 54 |
| **BD+LP2 20 Hz** | **0.36 / 0.99** | **0.037, 12.1** | **0.46 / 1.12, 0.046, +0.24** | **50** |
| BD+LP2 15 Hz | 0.31 / 0.84 | 0.039, 15.8 | 0.36 / 0.89, 0.044, +0.25 | 46 |
| α-β 25 Hz | 0.35 / 1.09 | 0.042, 11.5 | 0.46 / 1.15, 0.050, +0.21 | 50 |
| edge-ts 10 ms/3 | 0.23 / 0.65 | 0.037, 12.4 | 0.45 / 1.23, 0.158, +0.50 | 48 |
| edge-ts 20 ms/4 | 0.23 / 0.65 | 0.041, 15.9 | 0.40 / 1.06, 0.153, +0.47 | 46 |
| edge-ts 10 ms/3 + LP1 40 Hz | 0.23 / 0.65 | 0.043, 16.7 | 0.24 / 0.57, 0.077, +0.37 | 44 |

Checks: the replay of the shadow log through BD+LP1 30 Hz reproduces the logged Kd·ω peak exactly (2.55
m/s² in the 26–33 s upright hold); BD+LP2 20 Hz would have given 1.26 (RMS 0.47 → 0.35). C++ closed loop
with the rig config (`test_balance.cpp: balance_identified_plant_bdlp2_20hz_cuts_command_noise`): RMS a
0.98 → 0.50, p99 2.48 → 1.12 m/s², RMS θ 0.057° both.

## Reading
- **BD+LP2 20 Hz**: spikes 2.4× lower, closed-loop command noise halved, best swing tracking (3× better
  than today's filter), RMS θ slightly better; costs 6 ms of delay tolerance (56 → 50 ms). With the real,
  slower plant there is more margin than the synthetic study suggested (38–40 ms there).
- **15 Hz** is quieter still but gives up another 4 ms; 20 Hz is the balance. Revisit at Gate B together
  with the actuator data (0.9) and the step-loss limits.
- **Edge timestamps** are quietest open loop, but in closed loop the angle wanders 3× more inside the
  ±1-count limit cycle and the cart drifts twice as far: rejected.
- **All linear candidates drift ~0.2 m in 8 s** after a 3° catch (residual velocity from the transient;
  angle-only control). With ±15 cm soft limits that bounds a balance run to ~4–5 s: a Gate B topic
  (catch transient, i_delay, trim), not an estimator one.

## Still open
- `linear.py` models only the 1st-order filter: add LP2 for exact margins/Nyquist at Gate B.
- `pid_step`'s built-in filter stays BD+LP1 (`ctrl.est_fc` also feeds its `vel_fc`); the runtime always
  passes the external estimate, so the built-in one is unused on the rig.

---

# Earlier synthetic study (2026-10-08, PLACEHOLDER plant): kept for reference

Done while the Pi was unreachable (2026-10-08). The brief asks for the comparison on **logged** data
(free swings, slow hand motion, rest). This run uses a synthetic encoder instead, so the choice below
is **provisional**. Re-run `py/estimator_compare.py` with real logs after 0.6/0.10, and check the
event-timestamp clock first (0.6: `read_ns − ts_ns` must be small and positive if both are
CLOCK_MONOTONIC).

## Candidates (`py/pendel/estimators.py`, common `update(t, count, events)` interface)
| | Method | Parameter |
|---|---|---|
| (a) | backward difference at Ts + 1st-order low-pass (current `pid_step`) | fc = 20 / 30 / 40 Hz |
| (a) | backward difference + 2nd-order Butterworth low-pass (bilinear, prewarped) | fc = 30 / 50 Hz |
| (b) | α-β tracker on the quantised angle, error poles at exp(−2π f_n Ts) (double) | f_n = 15 / 25 Hz |
| (c) | edge timestamps: Δcounts/Δt over the newest same-direction edges (window, max edges), bounded by 1 count / (time since last edge) so it decays to 0 | 30 ms/8, 10 ms/3, 10 ms/3 + LP1 40 Hz |

## Method
- **Synthetic encoder** (`py/pendel/encsim.py`): exact edge times of a motion θ(t) at 2400 counts/rev,
  count phase 0.37, 2 µs timestamp jitter, an edge usable 60 µs after it happens; samples every 2 ms.
- **Rest:** the mean sits exactly on a count edge with 0.15 + 0.05 count micro-vibration at 25 + 71 Hz.
  The output RMS is the noise the D term sees.
- **Slow:** constant 0.02 / 0.1 / 0.3 rad/s (one count every 131 / 26 / 8.7 ms); RMS error.
- **Sine:** 3° at 1, 2, 3, 5 Hz; gain and phase lag of the estimate against the true velocity.
- **Swing:** decaying 1 Hz swing from 10°; RMS error.
- **Closed loop:** the sim with the placeholder plant (1 s hanging period) and the preview PID gains
  (Kp 62.8, Ki 123, Kd 5.13); catch from 3°, 6 s. Measures RMS a_cmd after 1 s (chatter) and RMS θ.
- **Tolerated extra delay:** largest added command delay (whole 2 ms samples) for which the 3° catch
  still holds below 1° RMS in the last second of 4 s. The same measure works for the nonlinear (c).
  Cross-check: for BD+LP1 30 Hz it gives 40 ms, against 41 ms from the independent linear analysis
  (`linear.py`).

## Results
| estimator | rest RMS (rad/s) | slow RMS err 0.02/0.1/0.3 (rad/s) | lag 1/2/3/5 Hz (°) | lag @2 Hz (ms) | swing RMS err (rad/s) | closed loop RMS a (m/s²) | RMS θ (°) | tolerated extra delay (ms) |
|---|---|---|---|---|---|---|---|---|
| BD+LP1 20 Hz | 0.124 | 0.050 / 0.079 / 0.083 | 3.2 / 6.5 / 9.6 / 15.8 | 9.0 | 0.089 | 0.55 | 0.052 | 38 |
| **BD+LP1 30 Hz** (current) | 0.158 | 0.061 / 0.107 / 0.117 | 2.3 / 4.6 / 6.8 / 11.3 | 6.3 | 0.116 | 0.75 | 0.052 | 40 |
| BD+LP1 40 Hz | 0.182 | 0.070 / 0.128 / 0.147 | 1.8 / 3.6 / 5.4 / 9.0 | 5.0 | 0.143 | 0.91 | 0.053 | 42 |
| **BD+LP2 30 Hz** | 0.154 | 0.055 / 0.076 / 0.030 | 3.0 / 6.1 / 9.1 / 15.4 | 8.5 | **0.048** | **0.54** | **0.046** | 38 |
| BD+LP2 50 Hz | 0.194 | 0.072 / 0.135 / 0.070 | 1.9 / 3.9 / 5.8 / 9.8 | 5.4 | 0.073 | 0.84 | 0.049 | 40 |
| α-β 15 Hz | 0.050 | 0.029 / 0.021 / 0.011 | 7.3 / 14.6 / 21.6 / 35.3 | 20.2 | 0.082 | 0.25 | 0.044 | 28 |
| α-β 25 Hz | 0.096 | 0.041 / 0.047 / 0.028 | 4.3 / 8.5 / 12.8 / 21.1 | 11.9 | 0.056 | 0.38 | 0.048 | 34 |
| edge-ts 30 ms/8 | 0.131 | 0 / 0 / 0 | 9.7 / 13.6 / 16.5 / 18.0 | 18.9 | 0.079 | 0.39 | 0.098 | 30 |
| edge-ts 10 ms/3 | 0.131 | 0 / 0 / 0 | 7.9 / 8.5 / 9.7 / 11.6 | 11.9 | 0.053 | 0.38 | 0.100 | 36 |
| edge-ts 10 ms/3 + LP1 40 Hz | 0.096 | 0 / 0 / 0 | 9.4 / 11.4 / 13.9 / 18.7 | 15.8 | 0.066 | 0.27 | 0.090 | 32 |

The true velocity RMS in the rest case is 0.060 rad/s (the micro-vibration itself).
Figure: `docs/img/estimators.png`.

## Reading the results
- **(a) BD+LP1 shows the classic trade-off.** Raising fc buys a little lag (6.3 → 5.0 ms at 2 Hz) and
  ~2 ms of delay tolerance, at the cost of more noise and chatter (0.75 → 0.91 m/s²).
- **(a) BD+LP2 30 Hz** keeps nearly the same delay tolerance as the current filter (38 vs 40 ms) but
  cuts closed-loop chatter by ~30 % (0.54 vs 0.75 m/s²). Its 2nd-order roll-off removes the
  quantisation steps (one count per sample = 1.3 rad/s spikes) much better. It also tracks slow motion
  and swings 2–4× better.
- **(b) α-β** is the quietest (0.25 m/s² at 15 Hz) but lags most. It gives up 6–12 ms of delay
  tolerance, which is margin Stage 2 needs for network latency.
- **(c) edge timestamps** are exact at constant speed (they measure time between edges instead of
  counting per sample). Short windows are needed: 30 ms/8 lags 19 ms; 10 ms/3 lags 12 ms. In the closed
  loop they give low chatter (0.38), but the angle wanders twice as far within the ±1-count limit
  cycle (RMS θ 0.10° vs 0.05°), and they tolerate 4 ms less delay. They also depend on what 0.6 finds
  about edge reliability, timestamp clock and latency.

## Provisional choice: (a) backward difference + 2nd-order Butterworth low-pass at ~30 Hz
1. It loses almost no delay margin against the current filter (38 vs 40 ms), and margin is what Stage 2
   (latency) spends.
2. It has the lowest chatter among the options that keep ≥ 38 ms, which matters for step loss (0.9) and
   for the acceleration noise the cart has to follow.
3. Best tracking of slow motion and swings among the linear options, and the best RMS θ in the
   closed loop.
4. It is linear: `linear.py` can analyse it exactly (margins, Nyquist), and it is trivial to mirror in
   C++ and the golden test.
5. It only uses counts, so it stays valid whatever 0.6 finds about edge types and timestamps.

Keep (c) as the alternative if 0.6 shows clean timestamps and the real limit cycle turns out slow
enough that 1-count steps per sample dominate the noise. Revisit the cutoff (20–50 Hz) once the real
ω₀ and the measured delays are known. The optimum scales with the closed-loop bandwidth.

## To do before this becomes the decision (brief 0.11)
- Re-run on logged data: rest, slow hand motion, free swings (0.6/0.10 recordings), with the real
  quantisation, the real decoder behaviour and the real timestamp clock.
- Add BD+LP2 to `pid_step`, `src/core/pid.cpp` and the golden vectors once confirmed (currently BD+LP1).
- Extend `linear.py` with the 2nd-order filter, for exact margins.
