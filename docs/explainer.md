# The Pendel project explained: theory, implementation and every step

A self-contained explainer of what we are building, why it works, how the software is organised, and why
each step of the plan exists. Numbers are the measured ones (status 2026-10-12). Details and evidence live
in the step documents linked throughout; this file ties them together.

Contents
1. The rig and the task
2. Background theory
3. Implementation
4. The steps, one by one, and why each matters
5. What surprised us
6. Numbers and symbols at a glance
7. Where things are

---

## 1. The rig and the task

### 1.1 Hardware
- **Cart** on an MGN12 linear rail, driven by a GT2 belt over a 60-tooth pulley (120 mm of belt per motor
  turn) by a **NEMA 23 stepper** (23HS2430, 200 full steps/rev) through an **A4988 driver** at 24 V.
  The driver's microstep pins MS1/MS2 select the step size: 1/8 step gives 1600 steps per motor turn,
  i.e. **13 333 steps per metre** (75 µm per step).
- **Pendulum**: a threaded rod with a weight, screwed onto the shaft of a **quadrature encoder**
  (600 pulses/rev). Counting both edges of both channels gives **2400 counts/rev = 0.15° per count**.
- **Two limit switches** near the ends of the ~0.5 m track (normally open, pulled up: 1 = free, 0 = pressed).
- **Raspberry Pi 4** runs everything: it reads the encoder and switches on GPIO pins and drives the
  driver's STEP, DIR and EN pins. EN high = driver off (motor unpowered), low = driver on.
- Power: 24 V supply with 1 A fuses; the Pi gets 5 V from a buck converter on the same supply, so the
  rocker switch (the current emergency stop) cuts the motor and the Pi together.

### 1.2 What we can measure and what we can command
| | Signal | Notes |
|---|---|---|
| Measure | pendulum angle (encoder counts) | **relative**: no index pulse, so the count is 0 wherever the pendulum is at program start |
| Measure | limit switches | only at the track ends |
| Know (not measure) | cart position | the number of steps we sent; there is **no cart encoder** |
| Command | cart acceleration | the stepper follows a commanded acceleration profile |

The stepper is a *kinematic* source: as long as it doesn't lose steps, the cart moves exactly as commanded,
whatever its mass or the belt friction. So the input to our model is the **cart acceleration a**, and the
main actuator failure mode is **step loss** (the cart silently falls behind the step count). It can't be
seen directly; it's found by driving to a switch and comparing where it triggers with where it should.

### 1.3 The task (the brief)
- **Stage 0**: bring the rig up safely, audit the inherited code, measure everything the controller
  depends on, and build a validated simulation. Gates A and B are review points where Simon decides.
- **Stage 1**: an **angle-only PID** running entirely on the Pi catches the pendulum when Simon releases
  it near upright and holds it there. No swing-up and **no cart-position feedback**: the cart is allowed to
  drift until a soft limit stops it. Gate C closes Stage 1.

---

## 2. Background theory

Full derivations: `docs/model_derivation.md`. Here the ideas and the numbers.

### 2.1 Equation of motion
Angle θ from upright, positive when the top leans toward +x; pivot on the cart. For a rigid pendulum (mass
m, pivot-to-centre-of-mass distance l, inertia J about the pivot, viscous pivot friction b) whose pivot is
accelerated at a:

    J θ̈ = m g l sin θ − m l a cos θ − b θ̇            (plus a small Coulomb friction term)

Dividing by J and writing ω₀² = m g l / J:

    θ̈ = ω₀² sin θ − (ω₀²/g) a cos θ − (b/J) θ̇

Two numbers describe the whole pendulum: **ω₀** (how fast gravity acts) and **b/J** (friction). The mass
and shape only enter through them. L_eq = g/ω₀² is the length of the equivalent point-mass pendulum:
measured **ω₀ = 4.931 rad/s, L_eq = 0.40 m**.

Intuition, the broom on your palm: if the top leans right (θ > 0), gravity makes it fall further right,
unless you accelerate your hand to the right (a > 0), which pushes the bottom under it. That's why the
controller's gains are positive: **lean toward +x → accelerate toward +x**.

### 2.2 Hanging versus upright
Near **hanging** (θ = π + φ) the equation becomes φ̈ = −ω₀²φ − (b/J)φ̇ + (ω₀²/g)a: a stable oscillator
with period 2π/ω₀ = **1.27 s**. A free swing therefore *measures* ω₀ (from the period) and b/J (from the
decay), which is how the plant is identified (step 0.10). Note the input gain's sign flips at the bottom:
accelerating toward +x makes a hanging bob lag toward −x.

Near **upright** it becomes θ̈ = ω₀²θ − (b/J)θ̇ − (ω₀²/g)a. Without control the solution grows like
e^{ω₀ t}: **a tilt grows by a factor e every 1/ω₀ ≈ 0.2 s**. That is the speed the controller must beat,
and it's why the loop runs at 500 Hz (100 samples per e-folding time).

### 2.3 The controller: PID on the angle
    a = Kp·θ + Ki·∫θ dt + Kd·θ̇

- **P** pushes the cart under the pendulum in proportion to the tilt. It must beat gravity: **Kp > g**
  (otherwise the cart can't accelerate fast enough to get under it; this is the lower stability limit).
- **D** reacts to the angular velocity and damps the motion (without it the loop would oscillate).
- **I** integrates the tilt. It removes a steady angle error, and (§2.7) it is also the controller's
  hidden memory of the cart's velocity.

Closing the loop on the linear model gives a third-order characteristic polynomial. **Pole placement**
picks the gains so the closed loop has chosen poles, a damped pair at ω_c with damping ζ and a real pole at p:

    (s² + 2ζω_c s + ω_c²)(s + p)  ⇒  Kp = g(1 + (ω_c² + 2ζω_c p)/ω₀²),  Kd = (2ζω_c + p − b/J)·g/ω₀²,  Ki = g ω_c² p/ω₀²

- ω_c is the speed of the closed loop (faster than the instability ω₀), ζ = 0.7 gives a well-damped
  response, p sets how quickly the integral acts (p = 0 is PD).
- Proposed: **ω_c = 2ω₀, ζ = 0.7, p = 0.5ω₀ → Kp = 62.8, Ki = 96.7, Kd = 6.56** (SI units). Kp means
  1.1 m/s² of cart acceleration per degree of tilt, or 0.16 m/s² per encoder count.

### 2.4 From the ideal model to the real loop
The continuous model has no upper limit on the gains. The real loop does, because of:
- **Sampling** every 2 ms and **delays** (computation, the step thread, the driver): the controller always
  acts on slightly old information.
- **Quantisation**: the angle is known only to 0.15°.
- **Saturation**: the cart can only accelerate and move so fast (a_max, v_max, measured in 0.9).

The standard tool for "how close to unstable" is the **Nyquist criterion**. For a plant with one unstable
pole the loop is stable only if the open-loop curve L(e^{jωT}) encircles −1 exactly **once
counter-clockwise**. From it come the **margins**: by how much all gains can be scaled down or up
(gain margin), how much phase is to spare at the crossover (phase margin), and how much extra delay the
loop tolerates (delay margin). Our design, computed exactly on the sampled model with the real filter
(`py/pendel/linear.py`): **gain ×0.25 to ×6.9, phase margin 50°, delay margin 48 ms**
(`docs/gateB_report.md`, `docs/img/gateb_nyquist.png`). The delay margin matters most for later stages,
where the loop may run over a network.

### 2.5 Estimating the angular velocity
The D term needs θ̇, but the encoder only gives counts. The simplest estimate, the backward difference
(change in angle per 2 ms), jumps by 2.6 mrad / 2 ms = **1.3 rad/s** whenever one count changes.
Low-pass filtering spreads that spike out, but every filter adds lag, and lag eats delay margin.

Each count step adds the same velocity Kd·q ≈ 17 mm/s to the cart command; the filter only decides whether
that comes as a tall short spike (risking clipping and step loss) or a low wide bump. On real recordings,
the 2nd-order Butterworth low-pass at 20 Hz (**BD+LP2 20 Hz**) halves the closed-loop command noise against
the first filter (BD+LP1 30 Hz) and cuts the spikes 2.4×, for 6 ms of delay margin
(`docs/stage0_11_estimators.md`, `docs/img/estimators_real.png`).

### 2.6 Where "upright" is: calibration without an index
The encoder counts from 0 at program start, so the angle reference must be found again at **every start**.
Two physical facts make this possible:
1. The hanging equilibrium has the centre of mass exactly below the pivot; the upright one exactly above.
   They are **exactly 180° apart**, whatever the pendulum's shape.
2. A free swing oscillates symmetrically about the hanging equilibrium, so fitting
   u(t) = c + e^{−σt}(a cos ωt + b sin ωt) to the encoder edge times finds the centre c to a fraction of
   a count.

Upright is then c + 1200 counts. Two refinements matter in practice: fitting the *edge crossings* (exact
positions at exact times) instead of sampled counts, and the pendulum's **amplitude-dependent period**
ω(A) = ω₀(1 − A²/16), which otherwise biases large swings (`docs/stage0_7_0_10_fits.md`,
`docs/stage1_damp.md`). Why it matters so much: see §2.7.

### 2.7 Why the cart drifts: a conserved quantity
Integrating the upright equation once gives a quantity that **no controller can change** (friction
neglected):

    v(t) − v(0) = g·∫θ dt − L_eq·(θ̇(t) − θ̇(0))

After a catch the pendulum is upright and still, so the final cart velocity is

    v_∞ = v(engage) + L_eq·θ̇(engage) + g·∫θ dt

- With **PID**, the closed loop forces the integrator back to zero, i.e. ∫θ dt → 0. So the cart returns to
  its engage velocity plus the pendulum's momentum term L_eq·θ̇. **The integrator is hidden cart-velocity
  feedback.**
- With **PD**, ∫θ of the catch stays as cart velocity (simulated: 68 mm/s after a 3° catch).
- If the true upright differs from the controller's zero by δ, the pendulum can only stay balanced while
  the cart accelerates at **a = g·δ**: about 26 mm/s² per count of error, i.e. 0.15 m in 3.4 s. Hence the
  effort on sub-count calibration and the brief's trim experiment.
- **A hand on the rod after engage** breaks the identity: the controller accelerates the cart while the
  hand stops the pendulum from reacting, and that momentum stays (~90 mm/s per 50 ms of holding).
- **Quantisation**: the ±1-count limit cycle settles at the count boundary nearest upright, an offset of up
  to half a count (up to ±6 mm/s² of drift), which the trim can only partly remove.

Angle-only control cannot stop all drift; it only keeps the cart's velocity memory. Holding the cart in
place needs cart-position feedback, which is a later stage. Stage 1 accepts the drift, measures it, and
minimises it (`docs/stage1_drift.md`).

### 2.8 Identifying the plant
- **Free decay** of the hanging pendulum (0.10): the period gives ω₀; the decay envelope separates viscous
  friction (exponential decay) from Coulomb friction (linear decay). Measured b/J = 0.024 1/s and Coulomb
  0.0117 rad/s²: tiny next to the controller's damping (2ζω_c ≈ 14 1/s).
- **Input gain**: the model says a cart acceleration a changes θ̈ by ω₀²/g·a. A short known acceleration
  pulse with the pendulum hanging, fitted with the model, measured this at **93 %** of ω₀²/g, and its sign
  confirmed both sign conventions together.
- **Closed-loop check**: the hanging active-damping test (a = −gain·φ̇) should make the swing decay at
  the designed rate. Measured **1.019 1/s vs 0.986** designed, i.e. input gain 103 %.

---

## 3. Implementation

### 3.1 Two programs
- **`main`** (the thesis code): a TCP server that drives the stepper and reports the angle. It's kept
  working (other tools use its protocol), with safety fixes: it no longer homes or enables the driver at
  start, and signal handlers switch the driver off.
- **`pendctl`** (new): the controller runtime, one program with modes: `log-selftest`, `timing-probe`,
  `encoder-log`, `move`, `shadow`, `damp` (and `balance` for Stage 1 catches). Every mode that can touch
  the real pins refuses to run unless explicitly allowed (`*.allow_real_chip=true`), and by default
  talks to a **simulated GPIO chip** (`gpio-mockup`) instead.

### 3.2 Layers
| Layer | Files | What it does | Tested how |
|---|---|---|---|
| Core | `src/core/` (pid, supervisor, units, estimator, swingfit, profile) | pure functions: inputs → outputs, no I/O, no clock | unit tests; **golden files**: the Python reference (`py/pendel/`) computes the same cases, C++ must match (bit-identical on the PC, within 1e-15 on the Pi) |
| Runtime | `src/runtime/` (BalanceRuntime, DampRuntime, SwingCalibrator, params) | one control period: read → convert → estimate → supervise → control → command | closed-loop tests on the simulated rig |
| Hardware interface | `src/hw/hardware.h` | raw units only (counts, edge timestamps, steps, toggles) | |
| Implementations | `PiHw` (motor + encoder), `SensorHw` (encoder + switches only), `SimHw` (simulated rig) | the same runtime runs on the rig or in simulation | SimHw golden vs Python; validated against the rig (0.12) |
| Apps | `src/app/` (modes) | real-time loop, console, logging | mock-chip tests (`tests/mock/`), then the rig |
| Config | `config/pendctl.conf` | every number with its unit and where it came from | `test_params` loads the real file |
| Logs | `logs/<time>_<mode>/` | every tick as CSV + meta (git commit, config used) | `py/` analysis scripts |

Why the layering: the same controller code runs in a fast simulation on the PC and on the rig, so it is
tested thoroughly before it ever moves anything, and a second implementation in Python catches errors
in either.

### 3.3 Real time on the Pi
The Pi runs normal Linux with a preemptible kernel. Three threads get **SCHED_FIFO** priorities and their
own cores, and memory is locked so nothing pages out mid-loop:

| Thread | Priority | Core | Job |
|---|---|---|---|
| Step thread (Axis) | 90 | 3 | toggles STEP at the right moments |
| Encoder reader | 85 | 2 | reads edge events from the kernel |
| Control loop | 80 | 1 | the 500 Hz tick |
| everything else | normal | 0 | |

Measured (0.5): the loop wakes p99 **6 µs** late idle, 35 µs under full load, worst 0.33 ms; step edges
p99 ≤ 12 µs. The 2 ms period has plenty of margin, so no extra microcontroller is needed.

### 3.4 The encoder path
The kernel timestamps every edge on the A and B lines. A dedicated thread (`EdgeReader`) reads them, merges
the two lines in time order and decodes each edge with the quadrature table into ±1 counts. It keeps the
cumulative count and a short history of edges with their exact timestamps. Each control tick takes the
new edges. Faults (an impossible transition, or the kernel queue overflowing) are flagged and the
supervisor stops. The old decoder from the thesis waited for both channels and held back the newest edge;
it was replaced after the audit and the 0.6 measurements.

### 3.5 The actuator path
The thesis's `Axis` class runs a step thread that integrates a commanded **acceleration** into a speed
and emits STEP pulses. Quirks handled at the boundary (`src/core/units`): protocol rates count STEP
*toggles* (2 per step), the internal speed sign is inverted, and the Axis's own position rewriting and
switch logic are bypassed (`rawPosition`) so the new supervisor owns all limits.

### 3.6 The supervisor
A small state machine around the controller (`src/core/supervisor`):

    IDLE --arm (hanging, still, cart centred)--> ARMED --near upright, still 50 ms--> ENGAGED
    ENGAGED --abort or stop--> STOPPING (brake at a_brake) --stopped--> IDLE

Aborts: |θ| > 22°, a limit switch, an encoder fault, the Axis not following, 3 late ticks in a row, the
run timer, a stop request, or the **soft limit**: the cart's position plus the distance it needs to brake
(including the Axis's own acceleration cap and a reaction time) reaches ±0.15 m. That last one is what
ends a normal Stage 1 run: the cart drifts, the supervisor brakes it before the switches.

### 3.7 Safety engineering
Nothing moves without a reason and a go:
- **Driver off by default**: EN is held high from boot; every program switches it off on exit, on a
  signal, and on a crash.
- **No motion without Simon's explicit go for that specific run**, with the motion envelope stated first
  (direction, distance, speed, acceleration, duration, how it ends, how to abort).
- **Mock chip first**: every new mode is tested on the simulated GPIO chip, with the kernel's trace of
  every pin write checked (e.g. "EN never went low").
- **Shadow mode** runs the full controller on the real sensors with the motor pins *never requested*:
  the safest possible real test of signs, logic and timing.
- After the stale-binary incident (2026-10-08): delete binaries before building, gate runs on a
  successful build, never run anything that can open the real chip without a go.

### 3.8 The testing pyramid
1. Unit tests (63 on the PC and the Pi), including golden comparisons with Python.
2. Closed-loop tests on the simulated rig: catches, aborts, sign errors, delays, the hand on the rod.
3. Mock-chip tests of the real program (21 checks: calibration, enabling, braking, refusals).
4. Shadow runs on the real sensors (no motion possible).
5. Motion runs with a stated envelope and a go, smallest first.

---

## 4. The steps, one by one, and why each matters

Each entry: **what** we do, **result** so far, **why it matters**.

### Stage 0: bring-up, audit, identification

**0.1 Discovery.** Inventory of the Pi (OS, kernel, libraries), the code, the wiring against the thesis
schematic. *Result:* 32-bit userland, libgpiod v1, all pins free, the published code doesn't compile as
is. *Why:* every later decision (threads, atomics, pins) depends on the actual platform, not the thesis.

**0.2 Build as-is.** Compile the inherited code unchanged. *Result:* builds with two fixes. *Why:* a
known baseline (`pi-baseline` tag) to compare against.

**Gate A: code audit.** A line-by-line review of the inherited code before running anything.
*Result:* 13 issues plus N1–N9 (`docs/gateA_report.md`), among them: uninitialised flags, non-atomic data
shared between threads, **limit switches that never stop motion**, a ×2 unit mix-up, a decoder that holds
back the newest edge, a normal-priority step thread. *Why:* these are exactly the bugs that would turn a
balancing attempt into a crash into the end stop; finding them on paper is cheaper than on the rig.

**0.3 Safe bring-up.** Bring-up mode (no homing, driver off until the first command), signal handlers,
atomics, EN high at boot, the mock-chip test harness (35 checks). *Why:* every experiment after this one
can fail safely.

**0.4 Instrumentation.** Ring-buffer logging at 500 Hz, CSV plus metadata (git commit, the exact config),
Python analysis environment. *Why:* every claim in this project is backed by a log that can be re-analysed.

**0.5 Timing.** Measure loop and step-pulse timing under idle and full CPU load with SCHED_FIFO and
pinning; scope check of the STEP pin. *Result:* p99 35 µs under load, no overruns; the Pi reached 82.7 °C
(heatsink since fitted). *Why:* the delay budget for the design, and proof that the Pi alone can run the
loop.

**0.6 Encoder integrity.** Record every raw edge while Simon turns and swings the pendulum; compare
decoders offline. *Result:* exactly 2400 counts/rev, no lost or wrong edges in 0.7 million, no drift
over many turns; one startup loss mechanism found and fixed. *Why:* the angle is the controller's only
measurement; a miscounting encoder means a wrong upright and a cart that runs off.

**0.7 Upright reference.** Fit free swings to find the hanging equilibrium to sub-count accuracy.
*Result:* ±0.13 counts within a run. *Why:* each count of reference error is 26 mm/s² of drift (§2.7).

**0.8 Conventions and cart calibration.** First motion: which way is +x, which switch is which, steps per
metre, the ×2. *Result:* +x is toward the left switch; θ sign +1 (hand test); the ×2 confirmed; **the
microstep mode turned out to be intermittent** (half-step one morning, 1/8 the next), fixed by the new PCB.
*Why:* a sign error makes the controller push the pendulum over instead of catching it; a wrong
steps/m makes every distance, speed and acceleration wrong by the same factor.

**0.9 Actuator capability** (pending, needs the new PCB). Moves at increasing acceleration and speed with
a step-loss audit after each, then 500 Hz acceleration streams. *Why:* the catch region (§4, Gate B) is set
entirely by how hard and how fast the cart can reliably move; a_max and v_max are set to 50–70 % of the
reliable maxima so the controller never asks for more than the motor delivers.

**0.10 Identification.** Free swings and a free decay (ω₀, friction), then the input-gain pulse.
*Result:* ω₀ = 4.931 ± 0.01 rad/s; input gain 93 % (pulse) and 103 % (damping). *Why:* the gains are
computed from these numbers (§2.3); a wrong ω₀ or input gain shifts every pole.

**0.11 Velocity estimator.** Compare candidates on real recordings and in closed loop. *Result:* BD+LP2
20 Hz. *Why:* the D term is the biggest consumer of measurement noise (§2.5).

**0.12 Simulation and validation.** A nonlinear simulation with sampling, delay, quantisation, the
estimator, saturation and the supervisor; the linear analysis for margins; then **validation against the
rig**: replaying the real pulse and damping runs. *Result:* closed-loop damping reproduced to 0.12° and
1 mm of cart travel (`docs/img/sim_validation.png`). *Why:* the design (and everything Gate B claims)
rests on the simulation; it's only trustworthy once it reproduces real data.

**Gate B: design review** (`docs/gateB_report.md`). Measured parameters with uncertainties, issues fixed,
the proposed design with predicted margins, the catch region, drift predictions, acceptance thresholds.
*Key finding:* every candidate tuning catches the same region, θ₀ + θ̇₀/ω₀ ≲ 10° with the placeholder
limits, so **the catch region is set by the actuator, not the gains**. *Why:* the gate is where Simon
decides, with the evidence in one place, whether the cart may try to catch the pendulum.

### Stage 1: catching and balancing

**1. Shadow mode** (done twice). The full runtime on the real sensors, motor pins never claimed.
*Result:* calibration, arm/engage/abort logic and the angle sign at upright confirmed; found the D-term
spikes, a reader race, and that "engage on release" can't yet tell a release from a hand adjustment.
*Why:* checks everything except the motor, with zero risk.

**2. Hanging active damping** (done). The first closed-loop motion: around the bottom, a = −gain·φ̇ makes
the cart follow the bob. *Result:* decay 1.019 vs 0.986 1/s designed. *Why:* checks signs, delay and
input gain in closed loop where a mistake only makes a swing grow (and the abort catches it), instead of
dropping an upright pendulum.

**3. First catches** (after Gate B and 0.9). Reduced a_max and v_max, releases from ~2–3°. *Why:* the
first real test of the upright loop, with the smallest possible energy if something is wrong.

**4. Raise the limits and run the experiments.** Catch success from 2°, 5°, 8° on both sides; balance
quality (RMS angle, saturation, duration); drift fits and trim iterations; PD vs PID. *Why:* turns "it
balances" into measured performance against the acceptance thresholds, and checks the theory (drift,
PD residual velocity) on the real rig.

**Gate C: Stage 1 report.** Results, comparison with predictions, what limits performance, what the next
stage needs (most likely cart-position feedback and a cart encoder).

---

## 5. What surprised us (and why the process exists)
- **A failed build ran a stale program** that homed the cart unasked (day 1). Hence the build/run rules.
- **The step mode changes on its own**: the same commanded move went 4.3× further one day than the next
  (a loose microstep wire). Every motion run now starts with a measured distance check until the new PCB.
- **The first velocity filter spiked ±2.4 m/s² per count** while the pendulum was held still: found in
  the first shadow run, fixed with data in 0.11.
- **The biggest drift source is the operator's hand**, not the controller: 50 ms of holding after engage
  costs ~90 mm/s of cart velocity.
- **A release can't be told from a hand adjustment** in the first ~0.15 s from the encoder alone, so the
  elegant "engage on release" logic stays off until it's tuned on labelled data.
- **Two real bugs in our own new code** (a race in the edge reader, a clock wrap) were found only by
  running on the Pi, after all simulated tests passed: the reason for the mock and shadow stages.

## 6. Numbers and symbols at a glance
| Symbol | Meaning | Value |
|---|---|---|
| θ | angle from upright, + top toward +x | rad (1 count = 2.618 mrad = 0.15°) |
| φ | angle from hanging | θ − π |
| x, v, a | cart position, velocity, acceleration; +x toward the left switch | m, m/s, m/s² |
| ω₀ | hanging small-angle frequency | 4.931 rad/s (T₀ = 1.27 s; 1/ω₀ = 0.20 s) |
| L_eq | equivalent pendulum length g/ω₀² | 0.40 m |
| b/J, γ | viscous and Coulomb pivot friction | 0.024 1/s, 0.0117 rad/s² |
| ω₀²/g | input gain (rad/s² per m/s²) | 2.48 1/m |
| Kp, Ki, Kd | PID gains (ω_c = 2ω₀, ζ 0.7, p 0.5ω₀) | 62.8, 96.7, 6.56 |
| Ts | control period | 2 ms |
| c_bottom | hanging position in counts (fitted each start) | upright = c_bottom + 1200 |
| a_max, v_max, a_brake | actuator limits | PENDING 0.9 (placeholders 3, 0.5, 1) |
| steps/m | cart scale at 1/8 step | 13 333 |

## 7. Where things are
| Topic | Document | Code |
|---|---|---|
| The brief | `CONTROL_BRIEF.md` | |
| Model and derivations | `docs/model_derivation.md` | `py/model_check.py` |
| Gate A audit | `docs/gateA_report.md` | |
| Safe bring-up, logging, timing | `docs/stage0_3_*`, `stage0_4_*`, `stage0_5_*` | `src/app/`, `src/rt/` |
| Encoder, upright, identification | `docs/stage0_6_7_10_results.md`, `docs/stage0_7_0_10_fits.md` | `src/hw/edge_reader.*`, `src/core/swingfit.*` |
| Estimator | `docs/stage0_11_estimators.md` | `src/core/estimator.*`, `py/estimator_real.py` |
| Simulation, validation | `docs/stage0_12_sim.md` | `py/pendel/sim.py`, `simhw.py`, `linear.py`, `py/sim_validate.py` |
| Stage 1 runtime | `docs/stage1_runtime.md`, `docs/stage1_damp.md` | `src/runtime/`, `src/app/shadow_mode.cpp`, `damp_mode.cpp` |
| Drift | `docs/stage1_drift.md` | `py/drift_analysis.py` |
| Gate B | `docs/gateB_report.md` | `py/gateb_design.py` |
| Session history | `docs/session_log.md` | |
