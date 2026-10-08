# Inverted pendulum (cart-pole): staged PID controller development

You have no prior context; this brief is it. I'm Simon, a mechatronics student continuing a lab rig built for a 2025 bachelor thesis. Scope of this session: Stage 0 (bring-up, audit, identification) and Stage 1 (on-Pi balance PID). Nothing beyond Stage 1. I want to understand and own the design, so explain the reasoning behind your choices. Put derivations in `docs/`, not in long chat messages.

## 1. Ground rules

1. This brief lives in the repo as `CONTROL_BRIEF.md` and is the full spec. Before anything else, create `CLAUDE.md` in the repo you work in.
   - It condenses this brief: system facts, conventions, safety rules, stage plan, status, and build/run commands.
   - It points to `CONTROL_BRIEF.md` for detail.
   - Keep it under ~150 lines; other details go in `docs/`.
   - Update it whenever a value or convention is established, and at the end of every session. Future sessions start with zero context.
2. You cannot see the rig. Never run anything that energises the motor, moves the cart, or arms a controller without my explicit "go" for that specific run.
   - Before each run, state what moves, which direction, max distance/speed/acceleration, duration, how it ends, and how I abort it.
   - Before the first motion test, confirm I have a way to cut motor power within reach.
   - The existing `main` starts homing the moment it launches: the `Axis` constructor calls `home()`, the driver is enabled, and the cart drives toward the right limit switch. Starting it counts as a hardware action.
3. Ask before any of these:
   - `sudo` or system changes (boot config, kernel parameters, CPU governor, services)
   - installing packages
   - changing pin assignments or driver settings
   - deleting or restructuring existing code
   - changing the existing TCP protocol, which other tools depend on

   No drive-by refactors.
4. Git: work on a new branch (e.g. `control-dev`) with small, descriptive commits. Never push anywhere without asking; upstream isn't mine.
5. Stop at every gate (A, B, C), report, and wait for approval.
6. If a hardware fact is uncertain, ask me. If it's measurable, design a measurement. Never "fix" existing code on a hunch; show evidence (a test or a log) first.
7. Use SI units internally (m, s, rad, m/s²). Convert to counts or steps only at the hardware boundary. Put a unit comment on every constant. Log raw integers (counts, steps) next to the SI values.

## 2. The system

### Hardware (verify anything marked ✱)

- Cart-pole: a pendulum on a belt-driven cart, on a linear track mounted to an 85 × 35 cm base board.
- Raspberry Pi 4B ✱ (`cat /proc/device-tree/model`). Hostname `pendel`, user `pendel`, SSH/scp at `pendel@pendel.local`.
- Actuator: NEMA 23 stepper (23HS2430, 3 A rated) on a Pololu A4988 carrier.
  - The A4988 is limited to ≤ 2 A. Record its current-limit (Vref) setting; don't change it.
  - 24 V supply; 5 V logic from a buck converter.
  - 1/8 microstepping: the code drives MS1 = MS2 = 1. MS3 is undriven and presumably pulled low on the carrier ✱.
  - Belt/pulley pitch is unknown, so steps per metre must be measured.
- There is no cart encoder. Cart position is the step count, open loop, so lost steps are silent position errors.
- Pendulum angle: incremental quadrature encoder.
  - NPN open collector with pull-ups to 3.3 V.
  - Nominally 600 PPR, which gives 2400 counts/rev with ×4 decoding ✱. The fitted unit may differ from the thesis one.
- Two normally-open limit switches at the track ends, pulled up (1 = free, 0 = pressed).
- Work in progress: the hand wiring is moving onto a custom PCB, and the cart/encoder mount is being redesigned because the old one bent the encoder shaft. Pins and wiring may change, so confirm with me before trusting the map below.
- GPIO (BCM) in the current code:
  - encoder A = 10, B = 19
  - STEP = 27, DIR = 22, EN = 24 (0 = enabled)
  - right limit = 2, left limit = 9
  - MS1 = 20, MS2 = 21

  GPIO 2 is I²C SDA (with a 1.8 kΩ pull-up on the Pi), and GPIO 9/10 are SPI0. Check with `gpioinfo` that no I²C or SPI driver claims them.

### Existing software

Upstream is https://github.com/lzprograms/Pendel, but the copy on the Pi is the source of truth. Diff it against upstream and tell me what differs.

- C++17, libgpiod v1 C API (`gpiod_line_*`), `std::thread`.
  - Build: `Kompilierer.sh` (`g++ -Wall -std=c++17 … -lgpiod -lgpiodcxx -pthread`).
  - Run: `start.sh` (`sudo nice -n -20 ./main`).
  - Some identifiers and comments are German: Winkel = angle, Flanke = edge, Drehgeber = encoder, Endschalter = limit switch.
- `main.cpp`: TCP server on port 8080, one client at a time, line-based text commands, one reply line per command. Commands: `setPos`, `setRelPos`, `setSpeed`, `setAcceleration`, `setMaxSpeed`, `setMaxAcceleration`, `getAngle`, `getAngleVelocity`, `getPos`, `getSpeed`, `getEndPos`, `calibratePos`.
- `Pendel`: owns `Encoder` and `Axis`, opens chip `"gpiochip4"`, and starts a gnuplot/X11 plot thread at 20 Hz.
- `Encoder`: a thread polls edge events on A and B, merges the two event queues by kernel timestamp, and runs a ×4 transition table into `winkel`. `winkel` is in counts and is zeroed when the thread starts, i.e. wherever the pendulum hangs at program start.
- `Axis`: a thread generates STEP pulses in software from a task queue (POS / SPEED / ACCELERATE / BRAKEPOS / BRAKESPEED / CHANGEDIRECTION).
  - Timing uses `sleep_until` with a base tick of 1e6/maxSpeed µs (≈ 8 µs), and silently resyncs if it falls > 500 µs behind.
  - ACCELERATE integrates a commanded acceleration into `curSpeed`, capped at `maxAcceleration·(1 − |v|/maxSpeed)`.
  - `stopOnLimit()` brakes when the computed braking distance would leave [0, endPos].
  - Homing: right switch sets `pos = 250`, then the left switch sets `endPos = pos + 250`.

### Suspected issues

These come from reading upstream. Verify each one with a test before relying on it or changing it.

1. `Encoder::getAngleVelocity()` only updates when edges arrive and never decays to zero, so it is stale exactly when the pendulum is near equilibrium. It's unusable for a D-term; estimate velocity in the control loop instead.
2. The decoder only processes when both the A and B queues are non-empty, so the newest edge can be held until the other channel toggles (latency, ±1 count). The kernel's per-line event buffer is small (~16 events in the v1 uAPI), so a starved thread drops edges and the count drifts. I've seen the centre of a free swing drift monotonically before; the cause was never confirmed.
3. Possible ×2 unit mismatch: STEP is toggled once per 1/|curSpeed| s, the A4988 steps on rising edges only, and `pos` counts rising edges. That would make microsteps/s ≈ curSpeed/2.
4. Signs flip between layers:
   - `Pendel::getSpeed` and `Pendel::setAcceleration` multiply by −1.
   - `Axis` increments `pos` when `curSpeed < 0`.
   - Homing heads for the right switch at low `pos`.

   Establish ground truth physically and document one convention.
5. Calling `setMaxSpeed()` at runtime doesn't update `inverseMaxSpeed`/`minUsDelay`, which are computed once at thread start. After that, the ACCELERATE cap scales wrongly.
6. `setAcceleration()` does `clear()` then `push()`, so the step thread can find the queue empty in between and skip a tick. For 500 Hz updates, use an atomic acceleration setpoint instead.
7. Don't treat `stopOnLimit()` as the safety net: its braking-distance estimate ignores the speed-dependent cap. Verify.
8. Shared state (`winkel`, `pos`, `curSpeed`, `direction`) is plain `int` accessed across threads. Make it atomic.
9. There is no real-time scheduling (only nice -20), and the step thread spins on a core.
10. `"gpiochip4"` is what a Pi 5 exposed on older kernels; on a Pi 4 the header is normally `gpiochip0`. A newer OS may ship libgpiod 2.x, which removed the v1 API; check with `pkg-config --modversion libgpiod`. Tell me before porting.
11. Upstream `encoder.cpp` assigns `winkelSchritte`, which `encoder.h` doesn't declare. Check whether it builds.
12. The plot thread pipes to gnuplot via `popen`. If gnuplot isn't installed or exits, the first flush (~5 s after start) writes to a dead pipe, and SIGPIPE kills the process.
13. In `main.cpp`, an empty line appears to spin the line-parsing loop forever, and `isCalibrating` sends a duplicate reply. This matters for Stage 2.

## 3. Model (re-derive it and check my signs)

A stepper imposes cart motion kinematically, as long as it doesn't lose steps. So the input is cart acceleration a = ẍ, and cart mass and motor constants drop out.

Conventions: +x is the direction of increasing step count ✱. θ is the angle from upright, positive when the top leans toward +x.

For a rigid pendulum with mass m, pivot-to-COM distance l_c, inertia J about the pivot, and viscous pivot friction b:

    J·θ̈ = m·g·l_c·sin θ − m·l_c·a·cos θ − b·θ̇     (plus possible Coulomb friction)

ω₀² = m·g·l_c / J is exactly the squared small-angle frequency of the hanging pendulum. So m·l_c/J = ω₀²/g, and near upright:

    θ̈ = ω₀²·θ − (b/J)·θ̇ − (ω₀²/g)·a        G(s) = θ/a = −(ω₀²/g) / (s² + (b/J)·s − ω₀²)

One free swing identifies the plant: the period gives ω₀, and the decay gives b/J = 2·ζ_hang·ω₀. The effective length is L_eq = g/ω₀²; for example, a 1.0 s hanging period gives ω₀ ≈ 6.3 rad/s and L_eq ≈ 0.25 m. Near the hanging position, the input gain flips sign.

With a = Kp·θ + Ki·∫θ + Kd·θ̇ (positive gains move the cart under the pendulum), the characteristic polynomial is:

    s³ + (b/J + ω₀²·Kd/g)·s² + ω₀²·(Kp/g − 1)·s + ω₀²·Ki/g

Placing the poles at (s² + 2ζω_c·s + ω_c²)(s + p) gives:

    Kp = g·(1 + (ω_c² + 2ζω_c·p)/ω₀²)
    Kd = (2ζω_c + p − b/J)·g/ω₀²
    Ki = g·ω_c²·p/ω₀²                      (p = 0 gives PD)

Kp must exceed g (in m/s² per rad). This loop has a lower gain margin as well as an upper one.

Two consequences to check in simulation and keep in mind when reading results:
- **The I-term acts as hidden cart-velocity feedback.** In the frictionless linear model, C = (g/ω₀²)·θ̇ + ẋ − g·∫θ is conserved whatever the controller does.
  - With PID, θ, θ̇ and ∫θ all go to zero, so the cart returns to its velocity at engagement (≈ rest after a gentle release).
  - With PD, the cart keeps g·∫θ dt from the catch transient as residual velocity.
- **No angle-loop term fixes a reference offset.** If the upright reference is off by δ, the loop settles at a constant cart acceleration ≈ g·δ.
  - 1 count (0.15°) ≈ 0.026 m/s², which is ≈ 0.3 m of travel in 5 s.
  - Sub-count calibration of the upright reference is what buys Stage 1 run time.
  - Removing the offset properly is the Stage 3 cart loop's job.

## 4. Roadmap

This section is context only. Build Stage 1 so the later stages slot in.

- **Stage 0 (now):** bring-up, audit, calibration, identification, timing, simulation.
- **Stage 1 (now):** angle-only PID with the loop closed entirely on the Pi.
  - It engages only near upright; I lift the pendulum there by hand.
  - No swing-up.
  - The controller ignores cart position and track length, but every safety limit stays.
- **Stage 2:** latency.
  - 2a: on-Pi artificial delay injection to measure the delay margin.
  - 2b: controller on my PC with the Pi as I/O server; measure the latency distribution and compare.
- **Stage 3:** cascaded PID. An outer cart position/velocity loop sets the angle reference, for sustained balancing within the finite track.
- **Stage 4:** energy-based swing-up and hand-over to the balance controller, within track limits.

Architecture implications:
- Make the controller a pure, deterministic step function: params + state + measurement + dt → command + new state, with no I/O and no clock reads. The same code then runs in simulation, on the Pi, and later on a PC.
- Put the hardware behind a thin interface with a simulated implementation, so the whole runtime (threads, timing, safety logic, logging) can be tested without moving anything.
- Add a configurable N-sample delay line on the command path (default 0) for Stage 2a.
- Keep the original `main` and its protocol working. Prefer a new executable for the controller runtime that reuses the existing classes. Propose the structure at Gate A.

## 5. Stage 0

**0.1 Discovery (read-only).** Find out whether you are on the Pi or on my PC with SSH.
- If SSH wants a password, ask me to set up key auth; I type passwords myself.
- If you're on my PC, propose how code stays in sync with the Pi.

Then collect: board model, OS release, kernel, libgpiod version, `gpiodetect`/`gpioinfo` output, CPU governor, `vcgencmd get_throttled`, compiler, git status and remotes, and the diff against upstream. If the thesis PDF or my notes are in the workspace, read them.

**0.2 Build** the code as-is. Report warnings and errors.

**→ Gate A:** report your findings, a verdict on each suspected issue (from code reading), the proposed architecture, and the test plan. Wait.

**0.3 Make experimenting safe.**
- Add a bring-up mode with no auto-homing and no gnuplot unless asked.
- Add SIGINT/SIGTERM/fatal-error handlers that stop stepping and disable the driver (EN = 1).
- Make shared state atomic.
- Make other fixes only after you have evidence.

**0.4 Instrumentation.**
- A preallocated ring buffer in the real-time path, flushed to CSV after the run. No file I/O in the loop.
- Every run saves its full config and the git commit hash.
- Python scripts (in a venv) for plots and metrics.

**0.5 Timing.**
- Build a prototype fixed-rate control thread: `SCHED_FIFO`, CPU pinning, `mlockall`, `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`. Give the step and encoder threads RT priority and pinning too.
- Measure histograms of wake-up latency, period jitter, and step-thread lateness over ≥ 10 min, both idle and under load.
- Watch for RT throttling if an RT thread spins. Check `get_throttled` (undervoltage/thermal) before and after.
- Propose, but don't apply, `isolcpus`/`nohz_full`/PREEMPT_RT if needed.
- If results are bad enough that a microcontroller should take over STEP generation or quadrature decoding, say so early; the PCB may still be changeable.

**0.6 Encoder integrity.**
- CPR: N full turns each way should give N·CPR ± 1.
- Return to bottom: 20 hard swings, letting it settle each time. The resting count must not drift. Repeat under CPU load and with the stepper running (EMI).
- Fast spins.
- If counts drift, propose decoder fixes, e.g. immediate per-event state update, a bulk line request, priority/pinning, the kernel `rotary-encoder` overlay, or pigpio.

**0.7 Upright reference.**
- Find the bottom equilibrium from a small free swing by fitting a damped sinusoid with offset, which gives sub-count precision. "Hanging still" is biased by stiction.
- Compute θ = wrap((counts − c_bottom − CPR/2)·2π/CPR) into (−π, π].
- Make this a re-runnable command, and keep a trim offset in config for Stage 1 to refine.

**0.8 Cart calibration and conventions.**
- Measure microsteps per metre with slow moves of known step counts; I measure the travel. Cross-check `endPos` against the measured switch spacing.
- Settle suspected issue 3 (the ×2 question).
- Establish +x and +θ physically and document them with a sketch in `CLAUDE.md`.
- Determine the usable track length and set soft limits ≥ 5 cm inside each switch (confirm with me).

**0.9 Actuator capability, with the pendulum mounted.**
- Run moves at increasing acceleration and speed. After each one, do a step-loss audit: drive slowly onto the right switch and compare the trigger `pos` with the expected value.
- Then stream acceleration commands at 500 Hz through ACCELERATE (steps, chirp). Compare the realised cart motion (from `pos`) with the integrated command, and measure the effective delay, the speed-dependent cap, and any glitches.
- Set the controller limits a_max and v_max to 50–70 % of the reliable maxima. Set the Axis `maxAcceleration`/`maxSpeed` so its own caps never bite inside the controller's range.

**0.10 Pendulum identification.**
- Free swings from ~5°, ~10°, and ~30°, at least 3 each.
- Get ω₀ from the period (note any amplitude dependence) and damping from the decay envelope: exponential means viscous, linear means Coulomb.
- Validate the input gain ω₀²/g: with the pendulum hanging, apply a short known acceleration pulse and compare the swing with the model.

**0.11 Velocity estimator.** On logged data (free swings, slow hand motion, rest), compare:
- (a) backward difference at Ts plus a 1st/2nd-order low-pass, starting at 20–40 Hz (equivalent to the PID derivative filter);
- (b) an α-β tracking filter;
- (c) an edge-timestamp estimate: Δcounts/Δt over recent edges, bounded by 1 count / time since the last edge so it decays to 0. Check which clock the event timestamps use first.

Compare noise at rest and in slow motion against phase lag at 1–5 Hz. Pick one and justify it.

**0.12 Simulation (Python).** Include:
- the nonlinear model
- sampling at Ts and compute/actuation delay
- 2400-count quantization
- the chosen estimator
- saturation and the Axis caps
- PID with anti-windup
- the engage/abort logic

Validate it against the 0.9 and 0.10 data before designing with it.

**→ Gate B:** report the following, then wait:
- a table of measured parameters with uncertainty
- plots
- issues found and fixed
- the proposed Stage 1 design (gains, Ts, filter, limits) with predicted margins
- proposed acceptance thresholds

## 6. Stage 1: on-Pi angle PID

### Objective

Catch and hold the pendulum upright after I release it near upright, with the loop closed entirely on the Pi. My PC is only for SSH, starting runs, editing config, and fetching logs. No swing-up and no cart-position feedback. The cart drifting until a soft limit is expected: analyse it, don't add cart feedback to "fix" it.

### Controller

- Fixed rate, default Ts = 2 ms; justify it or change it with data.
- Error e = θ − θ_ref, with θ_ref = 0 + trim.
- a_cmd = Kp·e + Ki·∫e + Kd·θ̇_f, where θ̇_f is the filtered derivative of the measurement. Compute it from the unwrapped cumulative count, not from e. Sign-check against §3 once the conventions are established.
- Integrator:
  - zeroed at engage
  - anti-windup (conditional integration or back-calculation against the realised, saturated command)
  - clamped
  - optional enable delay after engage, since my hand may still be on the pendulum for 0.1–0.3 s
- Output: saturate at a_max, and taper authority near v_max so the cart never exceeds the reliable speed. Convert to Axis units only at the boundary.
- Expect a ±1–2-count limit cycle from quantization. Don't add deadbands; in an unstable loop they cause drift.

### Supervisor

States: IDLE → ARMED → ENGAGED → STOPPING → IDLE.

- **Engage** when |θ| < θ_engage (3–5°) and |θ̇| < ω_engage for N consecutive samples.
- **Abort** on any of:
  - |θ| > θ_abort (~20–25°)
  - cart within braking distance of a soft limit
  - any limit switch pressed
  - Axis leaving ACCELERATE (e.g. its own `stopOnLimit()` fired)
  - a control-loop overrun streak
  - an encoder fault
  - max run time
  - manual stop
- **On abort:** controlled stop at a_brake, log the reason, disarm. Re-arming requires the pendulum hanging and the cart re-centred, and re-centring is a move that needs my go.
- **Interface:** a CLI on the Pi, optionally plus new TCP commands (`ctrlArm`, `ctrlStop`, `ctrlSet`, `ctrlStatus`) that leave the existing commands untouched. Parameters live in a config file with units.

### Gain design

Use pole placement (§3) with the measured ω₀ and b/J. Start around ω_c ≈ 1.5–3·ω₀, ζ ≈ 0.7, and a small p. Justify the trade-off: disturbance rejection and smaller catch excursions on one side; noise amplification, saturation, step loss, and delay tolerance on the other.

Check in the discrete simulation:
- Nyquist (one open-loop RHP pole means one counter-clockwise encirclement of −1)
- lower and upper gain margins, phase margin, delay margin
- RMS acceleration from encoder noise
- catch region over (θ₀, θ̇₀)
- predicted drift versus reference error

Show PD and PID side by side.

### Hardware sequence

Each step needs my go. My hands stay on the pendulum rod, never near the belt or pulleys.

1. **Shadow mode.** The controller computes, but the motor is not commanded. I tilt the pendulum by hand around upright. Check the signs and magnitudes of a_cmd (lean toward +x → command toward +x), the estimator, and the engage/abort logic.
2. **Hanging active-damping test.** Run a damping-only law around the bottom (the input gain sign is flipped there). The swing should die out much faster than open loop and match the simulation. This is a low-risk check of signs, delay, and input gain.
3. **First closed-loop catches** with reduced a_max and v_max, released from ~2–3°.
4. **Raise the limits** toward the Gate B values, then run the experiments below.

### Experiments and metrics

- **Catch success:** at least 10 releases each from ~2°, ~5°, and ~8°, on both sides. Success means engaged, |θ| < 3° within 1 s, and balanced until the soft limit or timeout.
- **Balance quality:** duration and termination causes, RMS θ after the transient, saturation fraction, peak a and v.
- **Drift:**
  - Fit x(t) = x₀ + v₀·t + ½·a_d·t² over the balanced segment.
  - Estimate the reference offset as |δ| ≈ |a_d|/g, deriving the sign from the conventions.
  - Update the trim and repeat. Show run time versus trim iteration.
- **PD vs PID:** confirm or refute the residual-velocity prediction from §3.
- **Tap test:** a small impulse on the pendulum; measure recovery.
- **Sensitivity:** ±25 % on Kp and on Kd, one at a time.
- **Step-loss audit** after aggressive runs.
- **Simulation vs hardware:** overlay runs with matching initial conditions.

### Deliverables

- code on the branch
- config
- unit tests for the controller step function, including a golden test that the C++ and Python implementations agree within floating-point tolerance on the same input sequence
- logs and plots
- `docs/stage1_report.md`: setup, parameters with uncertainty, methods, results, issues, and recommendations for Stage 2
- an updated `CLAUDE.md`

**→ Gate C:** report, then stop.

## 7. Stage 2 notes

Don't build any of this yet; just don't block it.

- The Pi streams [seq, t_sample, counts, θ, θ̇, pos, v] every Ts. The PC replies [seq, a_cmd]. The Pi applies the newest command (zero-order hold) and logs the command's age. A watchdog aborts if commands stop arriving.
- Prefer UDP. With TCP, set TCP_NODELAY on both ends; otherwise Nagle plus delayed ACKs can add stalls of ~40 ms (Linux) to ~200 ms (Windows).
- The PC side should react to incoming packets rather than pace itself with `Sleep()`; Windows' default timer resolution is ~15.6 ms.
- Use wired Ethernet. On Wi-Fi, disable power saving.
- Compare the measured latency distribution (median and tail) against the Stage 2a delay margin.

## 8. Reporting

Keep status updates short and use tables for measured values. End every report with an explicit list of questions for me. When you need me at the rig, give numbered step-by-step instructions.
