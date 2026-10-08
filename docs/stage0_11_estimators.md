# Stage 0.11: velocity estimator candidates (SYNTHETIC data, PLACEHOLDER plant)

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
