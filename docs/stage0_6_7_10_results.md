# Hand-test session 2026-10-09: encoder integrity (0.6a), upright reference (0.7), identification (0.10)

All recordings: `pendctl --mode encoder-log` on the real encoder inputs (GPIO10/19, inputs only,
SCHED_FIFO 85 on core 2, `sudo`), driver off throughout (GPIO24 high before and after every run).
Simon moved the pendulum by hand. Analysis: `py/encoder_report.py`, `py/fit_swing.py`,
`py/pendel/swingfit.py`. Runs are under `logs/` on the PC and the Pi.

## 0.6a Encoder integrity
| Test (run) | Events | Decoders | Anomalies | Result |
|---|---|---|---|---|
| 5 + 5 turns (`08-51-53`) | 59 237 | type = toggle = level; orig −1 (held edge) | 0 | settled bottoms −1299.5 → −13 300.0 → −1300.5: **5 turns = 12 000.5 counts, return −1.0 count**. CPR = 2400 confirmed; N·CPR ± 1 passes |
| 4 hard swings from ~90° (`09-08-34`) | 349 297 | all agree (final 1) | 0; every read 1 event | rest centre before swings 2/3/4 and at the end: 1.5 / 1.5 / 1.0 counts: **no drift** |
| fast spins, up to 1.67 rev/s (`09-17-21`) | 96 939 | all agree | 0 | after 3 net turns the swing centre is 7201.0 = **3·2400 + 1** |
| (identification run `09-32-26`) | 188 714 | all agree (orig −1) | 0; every read 1 event | window offsets over 145 s: −0.16…+0.30 counts, no trend |

**Findings:**
- **No edge loss and no wrong edge types in normal reading.** The rising/falling-type hypothesis from
  Gate A (§2) never showed up (0 same-type repeats in 0.7 M events). The old decoder's only defect seen
  here is the one-edge hold (orig = type − 1), as predicted.
- **Edge loss at reader startup:** in `09-20-36` the pendulum was still swinging when the recording
  started. `encoder-log` claimed the lines ~0.15 s before its first read (buffer allocation +
  mlockall), both 16-event kernel queues filled, and edges were lost (2 same-type repeats on B; the
  toggle decoder diverges, the type decoder recovers). That's issue 2's drop mechanism, demonstrated.
  Fixed: the lines are now claimed after setup. **Lesson for the runtime:** start reading before the
  pendulum moves, and keep reader stalls well under 16 edges' worth.
- **One 7.3 ms reader stall during a fast spin** (`09-17-21`, t = 51 s; 16 events read > 1 ms late,
  queue depth ≤ 4). There was no loss, but at 2000 edges/s per line the queue covers only 8 ms. Cause not
  yet known (candidate: the kernel GPIO IRQ thread's priority/core vs the SCHED_FIFO 85 reader).
  To investigate before the runtime relies on edge reading at high speed. Balancing speeds are far lower.
- Read latency (kernel timestamp → user space): p50 29–38 µs, p99 45–68 µs. Event timestamps and
  CLOCK_MONOTONIC agree (latency small and positive), so they share the clock (prerequisite for the
  edge-timestamp estimator, 0.11 (c)).
- Not yet done: the same tests under CPU load and with the stepper running (EMI) (0.6b, needs motion).

## 0.7 Upright reference (`09-20-36`, 4 swings of 5–14°)
Damped-sine fit on edge crossings per swing (windows trimmed to exclude hand contact):

| Swing | A0 | c_bottom (counts) | ω_d (rad/s) | σ (1/s) | residual |
|---|---|---|---|---|---|
| 1 | 13.8° | 61.959 ± 0.006 | 4.9065 | 0.019 | 0.42 |
| 2 | ~5° | 62.122 ± 0.011 | 4.9123 | 0.055 | 0.36 |
| 3 | 8.8° | 61.859 ± 0.012 | 4.9138 | 0.030 | 0.76 |
| 4 | 8.2° | 62.045 ± 0.003 | 4.9129 | 0.024 | 0.14 |

**c_bottom = 62.0 counts, spread ±0.13 (SD 0.11)** within one program run. That meets the ~0.2-count
target. It corresponds to ±0.003 m/s² of drift acceleration, versus 0.026 m/s² per count. c_bottom is
relative to the count at program start, so it is re-measured at every start (`core::fitOffset` with
`calib.swing_w` = 4.913 rad/s and `calib.swing_sigma` = 0.03 1/s). σ depends on amplitude, so for
the linear start-up fit, keep the start-up swing in the same 5–15° range.

## 0.10 Identification (`09-32-26`: 3 swings each from ~5°, 10°, 30°, then a free decay 40° → 0.5°)
| Parameter | Value | Source |
|---|---|---|
| ω₀ (A → 0) | **4.931 rad/s** (± 0.01: segment spread 4.914–4.940) | free decay, windowed; 8 clean segments agree |
| T₀ | 1.274 s | |
| L_eq = g/ω₀² | **0.4035 m** | |
| amplitude coefficient | 0.071 (rigid pendulum 1/16 = 0.0625) | |
| b/J (viscous) | 0.024 1/s (ζ_hang = 0.0024) | combined envelope, free decay |
| Coulomb γ | 0.0117 rad/s² (stiction band ±0.18 counts) | combined envelope, free decay |
| envelope | "both" fits 3× better than exponential, 6× better than linear | |

Caveats: viscous vs Coulomb can only be separated over a wide amplitude range (the free decay).
Segments that stay at 25–46° show ~40 % more apparent viscous damping, which suggests a speed-squared
(air drag) component. All of this is irrelevant for balancing: b/J ≈ 0.02 1/s against
2ζω_c ≈ 14 1/s in the design.

Still to do in 0.10: validate the input gain ω₀²/g with a short known cart acceleration pulse while the
pendulum hangs (needs motion and a go).

## Implications
- Pole placement now has a measured plant: ω₀ = 4.93 rad/s (slower than the 2π placeholder). With
  ω_c = 2ω₀: Kp = 5g = 49 m/s²/rad, Kd = (2·0.7·9.86 − 0.024)·9.81/24.3 = 5.6 m/s²/(rad/s).
  To be finalised at Gate B with the actuator data (0.9).
- The encoder path is sound for Stage 1 speeds once the startup order is right. The 7.3 ms stall is
  the remaining open point for high-speed edge reading.
