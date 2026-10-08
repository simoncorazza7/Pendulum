# Gate A report (Stage 0.1–0.2), 2026-10-08

Evidence tags: **[code]** = from reading the source, **[env]** = observed on the Pi (read-only),
**[hyp]** = hypothesis to be tested. Nothing has been run on the rig.

## 1. Findings summary
- Environment: see `stage0_discovery.md`. Pi 4 (1 GB), bookworm armhf, PREEMPT kernel, libgpiod 1.6.3, gnuplot absent.
- Upstream doesn't compile. The Pi copy (upstream + 2 fixes by Simon) builds with **0 warnings** at
  `-Wall`, and 3 benign ones at `-Wextra -Wshadow` (two shadowed names, one unused parameter).
  Kompilierer.sh builds **without optimisation (-O0)**. The prebuilt `main` (54 KB) differs from an
  as-is rebuild (`build/main_asis`, 203 KB), so it was built with other flags or other code.
- **[env]** `pinctrl_bcm2835.persist_gpio_outputs = Y`: GPIO outputs **keep their last level when a
  process exits or dies**. If `main` dies, EN stays 0, the driver stays energised, and the motor holds.
- **[env]** Timer slack of normal (SCHED_OTHER) threads is 50 µs. That matters for the step loop (§2, 9).
- **[env]** Right now every rig pin reads its SoC default pull (GPIO2 hi via the board's SDA pull-up,
  GPIO9/10/19 lo). With the thesis's 8 kΩ pull-ups connected, GPIO9 (left switch, NO) should read hi.
  So **the rig appears to be disconnected or unpowered**. Question for Simon.

## 2. Verdicts on the suspected issues (brief §2)

| # | Verdict | Evidence / detail |
|---|---|---|
| 1 | **Confirmed [code]** | `angleVelocity` is only recomputed in `processEvent()` (encoder.cpp:90), so it freezes at its last value when edges stop. It also pairs the count of the last *processed* edge with the timestamp of the last *read* event (`eL`), and it's an int in counts/s. Unusable for D. |
| 2 | **Confirmed [code]**, drops **[hyp]** | `eventLoop` only decodes when *both* queues are non-empty (encoder.cpp:33), so the newest edge always waits for the other channel. At rest or at a reversal (one channel dithering), edges pile up undecoded and the count is stale indefinitely. In the steady state counts stay correct, but late. Drops: one event per line is read per `poll`, and the v1 kernel FIFO holds 16 per line, so a starved thread (≈ 6.7 ms at 2 rev/s) loses edges. **New [hyp]**: for both-edge requests the kernel decides rising/falling by *re-reading the pin level* in the IRQ thread, so fast or bouncy edges can get the wrong type. The decoder trusts `event_type`, so that becomes a miscount. This plausibly explains the "duplicate edge" workaround in `getEvent()` and the drifting swing centre. To be tested in 0.6. |
| 3 | **Confirmed [code]** | STEP toggles once per `1e6/|curSpeed|` µs (axis.cpp:98–111) and `pos` counts rising edges only, so **µsteps/s = curSpeed/2**. Every *rate* in the protocol (speed, accel, maxSpeed, maxAccel) is in "toggles" = half-microsteps; positions are true microsteps. `decelerateForPos` uses v²/(4d), which is exactly right *given* the ×2, so the author compensated empirically. Physical confirmation is planned in 0.8. |
| 4 | **Clarified [code]** | At the *protocol* level the signs are consistent: +accel → +speed → +pos, and DIR pin = 1 ⇔ pos increasing. Internally `curSpeed = −(protocol speed)`, and the two `×(−1)` in `Pendel` undo that. pos ≈ 0 at the "right" switch (GPIO2) and increases toward the "left" switch (GPIO9). The physical direction of +pos, which physical switch is on GPIO2, and the encoder sign are unknown; settle them in 0.8. Homing is only correct if DIR polarity and switch wiring agree (see N4). |
| 5 | **Confirmed, and worse [code]** | `inverseMaxSpeed` and `minUsDelay` are only set at thread start (axis.cpp:9–10). If `maxSpeed` is lowered below the current speed, the caps `(maxSpeed−|v|)·inv` go **negative**: BRAKESPEED then *accelerates*, and ACCELERATE can run away for curSpeed < 0. Raising maxSpeed can't exceed the initial 125 000 toggles/s (`minUsDelay` = 8 µs). |
| 6 | **Confirmed [code]** | `clear()` then `push()`. If the step thread sees the empty queue it `continue`s *before* the step block, so a due toggle slips by one period (pos stays consistent; velocity hiccup). `setAcceleration` also writes `direction` and the DIR pin **from the TCP thread** when curSpeed == 0, which races the step thread. |
| 7 | **Confirmed [code]** | `stopOnLimit()` is not a safety net. (a) It assumes constant decel v²/(2A), but the real decel is A·(1−|v|/V). The ×2 unit mismatch happens to make it 2× conservative, so it holds up to ~0.69·maxSpeed and then fails. (b) **int overflow**: `curSpeed*curSpeed` overflows above 46 341 toggles/s (≈ 1.7 m/s nominal), and above that it never brakes. (c) It's only active in SPEED/ACCELERATE, not POS. (d) It uses the open-loop `pos` against [0, endPos], and those bounds lie **250 µsteps (≈ 19 mm nominal) beyond each switch**. (e) At 500 Hz each `setAcceleration` cancels the brake task, though it re-arms next loop. (f) **The limit switches never stop motion** outside homing; they only overwrite `pos`. |
| 8 | **Confirmed [code]** | Plain `int`/`bool` shared across threads: Axis `pos, curSpeed, direction, endPos, isHoming, maxSpeed, maxAcceleration, running`; Encoder `winkel, angleVelocity, running`; Pendel `plotRunning`. Worse: **`running` (Axis, Encoder) and `plotRunning` are never initialised**, so reading them is undefined behaviour. In `startThread()` a garbage `true` skips claiming the lines *and* starting the thread. |
| 9 | **Confirmed [code+env]** | SCHED_OTHER with nice −20; the kernel is PREEMPT, not RT. The step thread wakes every `minUsDelay` = 8 µs, but normal-thread timer slack is 50 µs, so it oversleeps and then catches up in bursts. Step edges jitter by tens of µs, and lateness over 500 µs triggers a silent resync that **drops integrated acceleration** (the realised speed then lags the command). There's no CPU pinning, and the desktop is running. |
| 10 | **Not an issue [env]** | `/dev/gpiochip4 → gpiochip0` comes from a udev rule, and libgpiod 1.6.3 (v1) is installed. No port needed. |
| 11 | **Confirmed [env]** | Upstream fails: `axis.h` has an unterminated `#ifndef`, and `winkelSchritte` is undeclared. The Pi copy has both fixed. |
| 12 | **Confirmed [code+env]** | gnuplot is not installed, so `popen` succeeds via `sh`, and the first `fflush` at t = 99·50 ms ≈ **4.95 s raises SIGPIPE and kills `main` mid-homing**. With `persist_gpio_outputs=Y` the driver stays enabled and holding. Even with gnuplot installed, `set terminal x11` under `sudo` on a Wayland desktop would likely fail. |
| 13 | **Confirmed [code]** | An empty line leaves `newlinePos` stale (main.cpp:203–206). The loop then eats the following bytes and **spins forever**: the server stops, but motion keeps going (an ACCELERATE stays active until `stopOnLimit`). `isCalibrating` sends a reply *and* returns it, so the reply is duplicated. Also: a whitespace-only line calls `front()` on an empty queue (UB), and `send()` to a vanished client raises SIGPIPE and kills the process. |

## 3. New findings (not in the brief)
| ID | Finding | Impact |
|---|---|---|
| N1 | Uninitialised `running` / `plotRunning` (above). | Startup can silently skip the motor or encoder thread. |
| N2 | `getDecelDistance()` int overflow (above). | Soft stop and POS braking fail above ≈ 1.7 m/s. |
| N3 | `pos` is silently clamped to ≥ 250 / ≤ endPos−250 unless the switch is pressed (axis.cpp:135–142). | Hides step loss; `pos` can *freeze while the cart moves*. The controller must treat that as a fault. |
| N4 | Homing has no timeout, and the initial `endPos` = 10000 ends homing early if the switch spacing exceeds 9750 µsteps. If DIR polarity or switch wiring is swapped, homing drives away from the "right" switch and finishes with a garbage `endPos`. | Verify direction with a tiny attended move before homing is ever used. |
| N5 | Code defaults are `maxSpeed` = `maxAcceleration` = 120 000 (toggles), not the protocol doc's 16 000 / 80 000. Nominally that's ≈ 4.5 m/s and 4.5 m/s². | Defaults are high; the thesis document is stale. |
| N6 | Process exit/crash leaves EN low (driver on, holding) due to `persist_gpio_outputs=Y`. A deliberate EN = 1 *at speed* lets the cart **coast** on a low-friction rail. | Drives the 0.3 handler design (§5.1). A hardware EN pull-up only helps if outputs don't persist. |
| N7 | The Pi is powered from the same 24 V supply via the buck converter, so the rocker switch, the only cut, also hard-powers-off the Pi. | E-stop question (§6). |
| N8 | The motor branch has a 1 A fuse, and the 300 µF bulk cap is on a separate fused branch, not at VMOT. | 0.9 tests may blow the fuse. A VMOT E-stop must keep a bulk cap on the driver side. |
| N9 | `BRAKEPOS` completes only on exact equality, so after an overshoot it never pops (POS only). | Irrelevant to Stage 1; noted. |

## 4. Early microcontroller flag
The evidence so far: a software step loop on a non-RT kernel (#9), event-type and FIFO risks in the
v1 edge path (#2), the thesis's own reports of encoder miscounts and cumulative position drift, and
the open-loop cart. A small MCU on the new PCB (e.g. an RP2040: PIO quadrature counter plus step
generator from an acceleration/velocity setpoint, talking to the Pi over SPI/UART) would remove the
two riskiest software paths. **Recommendation:** since the PCB is still changeable, reserve a footprint
or header for it now, routed to STEP/DIR/EN and encoder A/B. Decide after 0.5/0.6 data. On-Pi
fall-backs to evaluate first: pigpio (DMA sampling and DMA step waves; `pigpiod` is installed but
disabled), and the kernel `rotary-encoder` overlay.

## 5. Proposed architecture
Goals: a pure controller step function shared by sim, Pi and PC; hardware behind a thin interface
with a simulated implementation; the original `main` and its protocol keep working.

```
existing: main.cpp pendel.* axis.* encoder.* SafeQueue.h Kompilierer.sh start.sh   (main + TCP protocol)
Makefile            builds main (same flags as Kompilierer.sh, which stays) + new targets + tests
src/core/           PURE, no I/O, no clock, no threads (compiles on PC too)
  units.h           SI constants, counts<->rad, steps<->m, protocol "toggle" units (the ×2) at the boundary
  estimator.*       velocity estimator(s) chosen in 0.11
  pid.*             step(params, state, meas, dt) -> (a_cmd, state'); anti-windup, clamps, enable delay
  supervisor.*      IDLE->ARMED->ENGAGED->STOPPING; pure inputs -> (mode, abort reason)
  limits.*          saturation, v_max taper, cap-aware braking distance, soft limits
  delay_line.h      N-sample command delay (default 0, Stage 2a)
src/hw/
  hw.h              interface: read() -> {counts, t_last_edge, pos_steps, v_cmd, switches, axis_mode, faults};
                    command_accel(a); brake(a_brake); enable(bool)
  hw_pi.*           wraps Axis + decoder (existing classes, minimally extended)
  hw_sim.*          nonlinear pendulum + kinematic cart + quantisation + delays + Axis caps
src/rt/             SCHED_FIFO, affinity, mlockall, clock_nanosleep(ABS) loop; preallocated ring log
src/app/pendctl.cpp new executable: modes bringup-move | shadow | damp | balance | timing-probe;
                    loads config/pendctl.conf (key = value  # unit); writes run dir with
                    data.csv + config + git hash; CLI first (optional ctrl* TCP later)
tests/              C++ unit tests (tiny in-repo assert harness, no external deps) + golden vectors
py/                 model, sim, controller mirror, analysis/plots (venv on the PC)
```

Planned changes to *existing* classes. Each one is small and additive, gets its own commit, and is
made only with evidence:
- 0.3 (mandated by the brief):
  - Constructor options `homeOnStart`, `plot`, plus `main` CLI flags (protocol unchanged).
  - Initialise `running`/`plotRunning`.
  - Atomics for shared state.
  - Signal handling (§5.1).
  - `MSG_NOSIGNAL` on `send`.
- Later, with evidence:
  - An atomic acceleration-setpoint mode in Axis for 500 Hz streaming (#6).
  - An Axis state/fault getter for the supervisor.
  - Recompute caps in the setters (#5); overflow-safe, cap-aware braking (#7, N2).
  - Decoder changes after 0.6.
- **The ×2 stays in the protocol** (other tools depend on it); the new runtime converts at its boundary.

### 5.1 Signal/abort behaviour
The brief asks for "stop stepping and disable (EN = 1)". The nuance: disabling *at speed* lets the cart
coast, while a stopped stepper that stays enabled holds the cart.

I propose:
- **SIGINT/SIGTERM**: a controlled ramp to zero at a_brake (bounded to ≤ 0.5 s), then EN = 1.
- **Fatal signals** (SEGV/ABRT/BUS/FPE): EN = 1 immediately, using an async-signal-safe ioctl.
- **SIGPIPE**: ignored.

### 5.2 Threads and timing (to be measured in 0.5)
| Thread | Policy | Core |
|---|---|---|
| Step | FIFO 90 | 3 |
| Encoder | FIFO 85 | 2, with the GPIO IRQ thread pinned there too |
| Control loop | FIFO 80, Ts = 2 ms | 1 |
| Everything else | normal | 0 |

This needs RT privileges: either `sudo` per run, or an `/etc/security/limits.d` entry for user `pendel`
(rtprio, memlock). That's a system change, so ask first.

## 6. Safety prerequisites before any motion
1. **E-stop.** Today the only cut is the rocker switch, which also hard-powers-off the Pi (N7).
   Acceptable for emergencies (risk: SD-card corruption). Better: a VMOT-only E-stop upstream of a
   bulk cap placed at VMOT (N8).
2. Rig connected and powered: confirm (§1, the pin readings suggest it isn't).
3. Every first motion uses `pendctl bringup-move` (no homing, no TCP, explicit distance/speed), cart
   placed mid-track by hand, driver enabled only for the move, then EN = 1.

## 7. Test plan (Stage 0, after Gate A)
"Go" = needs Simon's explicit go and presence at the rig. All motion tests start with 24 V on, cart mid-track.

| Step | What | Motion? | Pass criterion / output |
|---|---|---|---|
| 0.3a | bring-up flags, atomics, init fixes, signal handlers; unit tests in sim | no | tests pass; `main` protocol unchanged (scripted TCP check with homing off) |
| 0.3b | handler check: start bring-up with **driver disabled**, send SIGINT/SIGTERM, read EN with `pinctrl get 24` | no (EN = 1 throughout, STEP not issued) | EN reads 1 after every exit path |
| 0.4 | ring-buffer logger, CSV + config + git hash, Python venv + plotting | no | sim run → plots |
| 0.5 | timing probe: control loop at 500 Hz + the step thread toggling a **spare, unconnected GPIO** (pin to be confirmed by Simon) instead of STEP. Fallback is STEP with EN = 1, but the A4988 translator still counts while disabled, so re-enabling snaps the rotor by up to ±2 full steps (≈ ±1.2 mm). Run ≥ 10 min idle and under `stress`-type load (a package; ask); `get_throttled` before/after | no | histograms of wake-up latency, period jitter, step lateness; RT-throttle events |
| 0.6a | encoder by hand (motor off): N turns each way, 20 hard swings return-to-bottom, fast spins; old decoder vs logged raw events | no | N·CPR ± 1; resting count drift = 0 |
| 0.6b | same with the stepper running (EMI) | **go** | as 0.6a |
| 0.7 | small free swing → damped-sinusoid fit → c_bottom (sub-count); `pendctl calib-upright` | no | repeatable to < 0.2 count |
| 0.8a | first motion: 400 µsteps at ≈ 2 cm/s, observe direction; then known-step moves, Simon measures travel | **go** | steps/m ± uncertainty; ×2 settled; +x, +θ sketch |
| 0.8b | slow approach to each switch (no homing code) → switch spacing vs endPos; soft limits ≥ 5 cm inside | **go** | track length; soft limits agreed |
| 0.9 | moves at rising a, v + step-loss audit at the right switch; 500 Hz ACCELERATE streams (steps, chirp) | **go** each | reliable a_max, v_max; delay; cap; glitches; fuse margin |
| 0.10 | free swings 5°/10°/30° ×3; hanging accel-pulse test | hand; pulse = **go** | ω₀, b/J (viscous vs Coulomb), ω₀²/g validated |
| 0.11 | estimator comparison on logs | no | choice + justification |
| 0.12 | Python sim, validated against 0.9/0.10 | no | overlay plots |
| Gate B | parameter table, plots, Stage 1 design | | |
