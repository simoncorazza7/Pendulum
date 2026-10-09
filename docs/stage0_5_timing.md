# Stage 0.5: timing (runs 2026-10-08, analysed 2026-10-09)

## Setup
`pendctl --mode timing-probe` (commit 3cb07cb, -O2) under `sudo`, on the **gpio-mockup chip**: the real
Axis step thread runs and toggles a simulated STEP line, so the real driver is never touched.
- Control loop: SCHED_FIFO 80, core 1, 500 Hz, `clock_nanosleep(TIMER_ABSTIME)`, `mlockall`.
- Axis step thread: SCHED_FIFO 90, core 3, instrumented: scheduled vs actual time of every STEP edge.
- Profiles:
  - **accel-stream**: a 500 Hz `setAcceleration` stream, a = 160000 cos(2π·1 Hz·t) toggles/s², peak ≈ 1 m/s
    nominal. This is exactly the Stage 1 usage, including the clear()+push() of issue 6.
  - **speed**: ±26667 toggles/s (≈ 1 m/s nominal), reversing every 2 s.
- Load: SCHED_OTHER busy loop pinned to each of the 4 cores, SD-card writes (`dd oflag=direct`) and a
  compile loop. 10 min per run. Script: `tests/timing/run_timing.sh`; analysis `py/timing_report.py`.

## Results
| run | loop wake-up late p50/p99/p99.9/max (µs) | STEP edge late p50/p99/p99.9/max (µs) | STEP edges | resyncs (>500 µs) | loop exec max (µs) | overruns | step-thread CPU |
|---|---|---|---|---|---|---|---|
| accel-stream, idle | 5 / 6 / 10 / 45 | 7 / 8 / 12 / 102 | 9.8 M | 0 | 29 | 0 | 69 % |
| accel-stream, load | 11 / 35 / 40 / 329 | 9 / 12 / 21 / 318 | 9.8 M | 0 | 37 | 0 | 75 % |
| speed, idle | 6 / 6 / 19 / 131 | 7 / 8 / 12 / 189 | 15.6 M | 0 | 21 | 0 | 65 % |
| speed, load | 10 / 35 / 40 / 314 | 9 / 11 / 25 / 120 | 15.6 M | 0 | 22 | 0 | 80 % |

Rare tails under load: loop lateness > 200 µs 7–11 times in 10 min; STEP edges > 200 µs late 6 times
(accel-stream). Control period min/max 1.68 / 2.32 ms. No kernel RT throttling (dmesg) in any run.
Figure: `docs/img/timing_0_5.png` (histograms on a log scale, and the per-tick max STEP lateness over time).

Compared with the SCHED_OTHER baseline from 0.4 (wake-up ≈ 56 µs late every time, from timer slack),
SCHED_FIFO removes the slack: p50 is 5–6 µs.

## Thermal (important)
Under the 4-core load the SoC went from 54.5 °C to **82.7 °C in 10 min** and hit the **soft temperature
limit** (`get_throttled = 0x80008`: soft limit active, and it has occurred). Above ~80 °C the firmware
lowers the ARM clock. The timing above still held, but sustained experiments in the closed enclosure
would run at a reduced, unpredictable clock. Idle runs stayed at 50–55 °C. **Recommendation: heatsink
and fan on the Pi**, since the enclosure also holds the motor driver.

## Interpretation
1. **The control loop is fine on the Pi.** Worst-case wake-up lateness was 0.33 ms (16 % of Ts) under
   full load, the tick body took ≤ 37 µs, and there were no overruns. Ts = 2 ms is feasible. The ~35 µs
   p99 under load is the delay budget to assume in the sim (0.12).
2. **Step generation is good at p99 (≤ 12 µs) but has rare bursts.** A 0.1–0.3 ms delay at ~27k
   toggles/s (37.5 µs per toggle) postpones 3–8 toggles. The loop then emits the backlog back to back
   (the absolute schedule catches up without sleeping), which is a momentary step-rate spike. That is a
   plausible step-loss mechanism at high speed. Check in 0.9 (step-loss audit under load) before
   relying on aggressive moves. The 500 µs resync never fired, so no integrated acceleration was lost.
3. **The step thread busy-polls**: 65–80 % of core 3 at 8 µs wake-ups. That's below the 95 % RT
   throttling budget, but it's a whole core.
4. **Microcontroller: not required by timing.** Stage 1 runs on the Pi as is. Keep the cheap PCB
   footprint as insurance; the final call comes after 0.6 (encoder reliability) and 0.9 (step loss
   under load).

## Proposed, not applied (each needs Simon's OK)
- Cooling: heatsink + fan (hardware).
- `isolcpus=1,3 nohz_full=1,3 irqaffinity=0,2` on the kernel command line, to keep IRQs and other tasks
  off the RT cores and trim the load tail. PREEMPT_RT is not justified by these numbers.
- CPU governor `performance` during experiments (it is `ondemand`; frequency switches add jitter).

## Still open in 0.5
- **Scope check** (needs a go): the probe on the real chip with STEP routed to the spare GPIO18 (pin 12),
  to confirm the pin edges against the logged timestamps.
- The control loop and step thread with the real Axis on STEP (27) are measured implicitly in 0.9 (motion).
