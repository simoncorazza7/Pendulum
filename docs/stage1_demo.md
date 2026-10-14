# Stage 1 demo: the balance session (`./demo.sh`, `pendctl --mode balance`)

Written 2026-10-12 for Simon's supervisor demonstration on Wednesday 2026-10-14, on the current hardware
(before the new PCB). This is brief §6 hardware step 3, "first closed-loop catches with reduced a_max and
v_max", packaged as one interactive program run with many catches. It is not Gate C, and it runs ahead of
Gate B (0.9 is still pending): Simon's call, like hardware steps 1 and 2. Every run on the rig needs his go.

## Running it

On the Pi, from a terminal you can type into (`ssh -t pendel@pendel.local`, then `cd ~/Pendel-main`):

```
./demo.sh
```

The script shows a checklist and waits for `yes`. It then builds `build/pendctl` and stops if the build fails, so a stale binary never runs. Finally it starts `sudo build/pendctl --mode balance` on the real chip. Afterwards it prints the state of the driver-enable pin (`op dh` = driver off).

| Key | Effect |
|---|---|
| Enter | next step: start homing, stop the current run, or re-centre after a run |
| q + Enter, Ctrl-C | controlled stop, driver off, logs written, program ends |
| rocker switch | hard stop: cuts the motor and the Pi (that run's logs are lost) |

What you do and what you see:

1. **Calibration** (driver off). Give the hanging pendulum a 3–8° swing and take your hands off. After about 5 s it prints `calib: accepted`.
2. **Stop the swing by hand**, let the pendulum hang still, press Enter. **Homing** takes about 25 s:
   - The cart drives at 40 mm/s to the right switch.
   - It then drives to the left switch and prints the measured span. It must say `step mode OK`.
   - Finally it moves to the centre between the switches.
3. **ARMED** appears by itself once the pendulum hangs still. Lift the pendulum to upright and hold the rod lightly near the top with your fingertips, hand still in space.
   - At the beep (`ENGAGED`) the cart slides 1–2 cm under your hand and stops.
   - About 1 s later, let go cleanly: open your fingers, no push.
4. **Balancing.** The motor hums faintly: the ±1-count quantisation limit cycle at about 15 Hz, which the brief expects. The cart drifts slowly to one side. The run ends at the soft limit (±150 mm from the centre) with a controlled stop, typically after 3–8 s in simulation. Catch the pendulum when it falls.
5. Lower the pendulum to hanging and let it settle, then press Enter. **Re-centring:**
   - The cart touches the nearer switch and prints how far it was from where the step count expected it (the step-loss audit).
   - It then drives to the centre and arms again. Repeat from 3.
6. **q + Enter** ends the session. The logs go to `logs/<time>_balance/`: `data.csv` (every tick), `edges.csv`, and `meta.json` (span, per-run summary, touch errors).
   - On the PC, `py/fetch_runs.py` fetches them.
   - `py/stage1_report.py balance` then prints a table per run and the drift fit with the suggested trim change, and saves `balance_runs.png`.

### Messages

| Message | Meaning | What to do |
|---|---|---|
| `waiting to arm: ... not hanging / moving` | the supervisor's arm check (within 10° of hanging, below 0.3 rad/s, cart still) | let the pendulum settle, or stop it by hand |
| `waiting to arm: ... not centred` | cart more than 20 mm from the centre | cannot happen after homing or re-centring; report it |
| `STOPPING: soft limit within braking distance` | the normal end: drift reached ±150 mm | catch the pendulum, Enter to re-centre |
| `STOPPING: angle beyond theta_abort` | the pendulum fell (\|θ\| > 22°): failed catch, or let go with a push | as above |
| `FAILED: track span ...` | the switches were not about 6104 µsteps apart: the step mode is not 1/8 (the loose MS1/MS2 wire) | re-seat or secure the MS wires, restart `./demo.sh` |
| `FAILED: ... switch found ... mm from where the step count puts it` / `not within 20 mm` | step loss, or the step mode changed during the session | restart (it homes again); look at the run that came before |
| `FAILED: calibration ...` | no accepted swing fit within 120 s; nothing moved | restart and swing 3–8° with hands off |

## Design and why

**Reduced limits.**
- a_max = 2.0 m/s² and v_max = 0.3 m/s (`balance.a_max`, `balance.v_max`); they replace `ctrl.a_max` and `ctrl.v_max` in this mode only.
- The gains are the Gate B draft design: pole placement ω_c = 2ω₀, ζ = 0.7, p = 0.5ω₀, giving Kp 62.8, Ki 96.7, Kd 6.56. BD+LP2 at 20 Hz, Ts 2 ms.
- a_max is 2/3 of the PRIOR value, inside what the 0.10 pulse and damp runs exercised more gently (1 m/s²).
- v_max 0.3 m/s is 4000 µsteps/s, about 150 rpm.

**Homing and the span check.** The step mode on this wiring is intermittent: half-step on the morning of 10-09, 1/8 that afternoon.
- The config assumes 1/8 (13333 µsteps/m). A half-step flip makes every physical distance, speed and acceleration 4× larger.
- The soft limits would then lie beyond the switches, and the loop gain would be 4× (inside the upper gain margin of 6.9, but with less phase margin).
- So before anything balances, the session measures the distance between the two switches:
  - at 1/8 step it is 4 × 1526 = 6104 µsteps (1526 was measured at half step in 0.8c);
  - at half step it is 1526, at quarter step 3052, at 1/16 step 12208;
  - the tolerance is ±5 %.
- The seek speed of 40 mm/s keeps the half-step case safe. The cart then really moves at 160 mm/s and brakes about 3 mm past the switch (test below: 3.4 mm).

**Driver on from homing to the end; re-centre on a switch.** With the driver off the cart can be pushed, and the step count no longer equals its position. The session therefore keeps the driver on from homing until the end.
- Every re-centre touches the nearer switch and compares where the switch closed with where the step count expected it. That is the brief's step-loss audit after each run.
- Within 15 mm, the session re-zeroes on the switch and continues. Beyond that it stops.

**Moves leave the pendulum still.**
- Every session move accelerates and decelerates for exactly one pendulum period, T0 = 2π/ω₀ = 1.274 s.
- For the linear pendulum, a constant acceleration held for a whole period returns it to rest. The move therefore leaves no swing, and the next arm check passes at once.
- Simulation, 0.2 m at 0.1 m/s: 0.16° of residual swing, against 4.7° for a plain 0.3 m/s² trapezoid.

**Holding the cart between runs (`BalanceConfig::keep_driver_on`).** This is a latent issue found while writing the session.
- The Axis keeps executing its last ACCELERATE command until it gets a new one. STOPPING ends when |v| < 5 mm/s, and the last command it sent is a full brake.
- In IDLE nothing is sent, so the Axis keeps decelerating, through zero and into reverse, up to its speed cap.
- In every earlier mode the program ended right there with the driver off, so it never mattered. A multi-run session would see a cart "moving" on the next arm check, or worse.
- With `keep_driver_on`, the runtime holds a = −v/Ts (at most a_brake) in IDLE and keeps the driver on.

**Engage while the hand still holds the rod.** `sup.engage_on_release` stays off: hand adjustments looked like releases in shadow 14-38-54. The supervisor therefore engages after 50 ms still within 4°, with the hand still on the rod.
- `docs/stage1_drift.md` modelled that hand as rigid in angle: θ pinned, so the cart gains (Kp − g)θ per second and the run is lost within about 1 s.
- `py/hand_hold.py` compares a hand fixed in space. The rod pivots on the cart, and the grip point stays where the hand is.
- Then the loop a = Kp(X_hand − x)/l_h − Kd·v/l_h + … is a stable spring and damper. The cart slides under the hand and stops.
- Results with a_max 2, v_max 0.3, held at 2°, grip 0.40 m up the rod, soft limit ±0.15 m:

| Hand model | Let go after ENGAGED | Residual v after release | Balanced after release |
|---|---|---|---|
| rigid in angle | 0.05 s | +109 mm/s | 1.1 s |
| rigid in angle | ≥ 0.3 s | — | soft limit before release |
| fixed in space, 2–10 Hz grip | 0.05 s | −9 … +2 mm/s | 7.5 s to the end of the 8 s simulation |
| fixed in space, 2–10 Hz grip | 0.3 s | −27 … −46 mm/s | 3.5–5 s |
| fixed in space, 2–10 Hz grip | 1–2 s | −10 mm/s | 6–7 s |

This gives the operator procedure: a light fingertip hold with the hand still in space, not following the cart, and a clean release about 1 s after the beep, once the cart has settled. A reaction-time release (about 0.3 s) falls in the worst window, mid-transient. Which hand model the rig is closest to is the first thing the first trial shows: x during the hold, in `data.csv`.

**Pre-load only with engage on release.** `ctrl.preload` now needs `sup.engage_on_release`, which is what the config comment already said. Without it, a chance release detection at a still-hold engage would have pre-loaded the integrator with a wrong momentum.

## Evidence (PC, `bash tests/unit/run_pc.sh`: 71/71)

`tests/unit/test_session.cpp` runs on the simulated rig: identified plant, switches 6104 µsteps apart, hard stops 15 mm beyond them. The operator is scripted.

- **Full flow, 2 catches:**
  - Calibration from an 8° swing.
  - Homing in 23.5 s with at most 1.6° of swing; switches found within 2 µsteps.
  - Catch 1 with a 1-count reference error: 2.6 s balanced, RMS θ 0.07°, ends at the soft limit. The cart then stays put while held (< 0.5 mm in 2 s).
  - Re-centre on the nearer switch, with a touch error within 2 µsteps.
  - Catch 2, stopped by Enter. Then quit: driver off, no limit switch hit, no crash.
- **Wrong step mode** (the simulated driver runs half step): `FAILED: track span 1526 usteps ...`. The cart stops 3.4 mm past the left switch, the driver goes off, no crash.
- **Step loss at re-centring:**
  - 100 µsteps (7.5 mm): measured, re-zeroed, the cart physically centred again.
  - 300 µsteps (22 mm): `FAILED ... step loss`, driver off.
- **Other cases:** a quit during homing stops the cart and turns the driver off. A calibration timeout ends the program and nothing is ever enabled.
- `py/stage1_report.py balance` was checked on a simulated session written in the same format. The injected reference error was +0.5 counts, and the suggested trim change was −0.58 counts, so the sign is confirmed.
- `tests/mock/run_balance_mock.sh` (Pi, gpio-mockup) covers the plumbing: stdin through a FIFO, switches pressed on a schedule, EN and STEP traced, SIGINT, logs. It has not run yet; the Pi was off.

## Risks on the current hardware

- **Step mode flips during a run.** This is not detectable until the next re-centre.
  - Distances are 4× too large physically: the soft limit lies beyond the switches. The limit-switch abort then brakes at a_brake, which is also 4× physically.
  - Mitigation: secure the MS1/MS2 wires (tape or hot glue) before the demo. Homing at every start and the switch touch after every run catch the case where the mode has stayed flipped. Keep the rocker switch within reach.
- **1 A fuse on VMOT:** peak demand is 2 m/s². Keep a spare fuse at hand.
- **0.9 not done:** the motor at 0.3 m/s and 2 m/s² with this load is unmeasured, though modest.
- **Hand-hold behaviour** is unverified on the rig; the first trial shows it.
- **E-stop:** the rocker switch cuts the Pi too (accepted for the slow tests; a motor-only switch is still to come).

## First rig trial (before the demo; each step needs Simon's go)

1. Pi on. Push, `make`, `make test`, then `tests/mock/run_balance_mock.sh` (gpio-mockup; nothing moves).
2. `./demo.sh` with calibration and homing only, then q at ARMED. The only motion is homing at 40 mm/s plus the centring move. Check the span, the residual swing, and that the cart sits visually in the middle.
3. Catches: one at a time at first, q in between, then a few in a row. Run `py/stage1_report.py balance`.
4. From the report:
   - If the drift is consistent across runs, apply the average trim change (`encoder.trim`, or `./demo.sh --set encoder.trim=...`).
   - If runs end within about 1 s, the hand is closer to the rigid model. In that case, practise the release, or let go right at the beep.

## First rig session, 2026-10-12 (`logs/2026-10-12T15-30-52_balance`, Simon ran `./demo.sh`)

From `meta.json` (`data.csv` not yet fetched: the Pi was switched off right after):
- Homing span 6099 µsteps (expected 6104): 1/8 step, the MS wire held. Seven re-centres, touch errors −11 … +2
  µsteps (< 1 mm): no step loss.
- 8 runs, all caught, all ended at the soft limit (none fell).

| Runs | Balanced | RMS θ after 1 s | Peak \|v\| | Saturated | Ended at |
|---|---|---|---|---|---|
| 1 | 1.1 s | 1.6° | 0.30 m/s | 46 % | +139 mm (left) |
| 2–5 | 1.3–3.3 s | 0.075–0.10° (simulation: 0.07°) | 0.12–0.17 m/s | 0–1.5 % | +139 mm (left) |
| 6–8 | 11, 34, 36 s | 1.1–1.6° | 0.27–0.30 m/s (= v_max) | 4–9 % | −138, −138, +139 mm |

- **Runs 2–5:** balanced exactly like the simulation, but they left in 1.3–3.3 s, always toward +x (the left switch). That is either release momentum or a reference offset (trim). The report's drift fit (v0 vs a_d) and its release Δv separate the two.
- **Runs 6–8:** Simon confirms the cart visibly swayed. `py/sway_study.py` adds the missing rig effects one at a time.
  - Extra delay up to 40 ms, input gain ×0.7–2.5, a belt resonance ≥ 12 Hz, and ×10 pivot friction leave the balance as quiet as the model.
  - A belt resonance of 5–8 Hz gives a 5–7 Hz wobble that saturates.
  - Drifting into v_max reproduces the runs 6–8 signature: RMS θ 1.4–3°, \|v\| at 0.30 m/s, about 10 % saturated. A 0.5–2 count reference offset or an input gain of ×0.5 gets it there.
  - In simulation, though, the cart then runs away in one direction and the pendulum falls after 7–29 s. It never keeps reversing for 36 s within ±139 mm.
  - Something on the rig reversed the drift. A hand nudging the pendulum would do exactly that.
- `py/stage1_report.py balance` now tests this directly. The drift identity v − v₀ = g∫θ − L_eq Δθ̇ − L_eq (b/J) Δθ holds only without external torque, so a push shows as a step in its residual.
  - The report gives the release Δv (the hold-and-release momentum) and later steps more than 1.5 s after ENGAGED (nudges).
  - On simulated sessions: clean runs 4–5 mm/s, a 1.7° nudge 28–42 mm/s. A run above 20 mm/s is flagged and its trim advice ignored.
  - It also prints the dominant θ frequency and the time spent in the v_max taper.

### Analysis of 15-30-52 (data fetched 2026-10-13)
- **Reference offset:** 16 of 17 quiet stretches (runs 3, 6, 7, 8; no outside step; from 1.5 s after ENGAGED) drift toward +x at 49–61 mm/s². That is δ = +1.9 … +2.3 counts (median 2.09 counts, 0.31°).
  - Runs 2–3 give the same from the cart position alone (a_d ≈ +60 mm/s²). This, not release momentum, ended runs 2–5 in 1.3–3.3 s.
  - Proposed: `encoder.trim = −2.1` counts, to be checked first with `./demo.sh --set encoder.trim=-2.1`.
  - One 9 s stretch (run 8, from 10.4 s) had zero drift, with the cart still at +45 mm. This is the quantisation branch effect from `docs/stage1_drift.md`.
- **Runs 6–8:** between the stretches, the identity residual has sharp steps of 150–200 mm/s, always reversing the drift. These are outside pushes, Simon's hand acting as the position loop; that is why those runs lasted 11–36 s.
- **Catch transients of runs 1 and 6:** a growing oscillation of about 2 Hz with a_cmd hitting ±2 m/s² for up to 4 s. The residual oscillates by ±400 mm/s, so the hand was still on the rod and fighting the controller. The real hand is stiffer than the "fixed in space" model. Procedure: let go cleanly soon after the beep; if the cart starts buzzing back and forth while you hold, let go at once.
- **08-58-26:** `homing: switch not found before the give-up position`. The motor power was off (Simon), so the step count ran 0.55 m while the cart stood still, and the session stopped as designed.

## Session 2026-10-14 (`logs/2026-10-14T10-20-52_balance`, `--set encoder.trim=-2.1`)

- **Trim validated.** 10 quiet stretches give a median residual drift of −3.1 mm/s² (range −11.7 … +2.3), down from
  +54 mm/s², i.e. δ = −0.12 counts: inside the quantisation band. Run 11 balanced 14.9 s untouched (before: 1–3 s).
  **`encoder.trim = −2.1` is now the config default.** Caveat: the untrimmed session `2026-10-13T10-15-19` gave +0.7
  counts, but over a wide range (−3.6 … +56.6 mm/s², hand pushes); `15-30-52` gave 2.09 and today implies 1.98.
  Re-check the offset whenever the report shows a consistent drift.
- **Release momentum** is now the limit of untouched runs: v0 of +50 … +95 mm/s in runs 2, 4, 6 and 7 ran out of
  track in 2–3 s. Release more cleanly (open the fingers, no push), or later, Stage 2 cart feedback.
- **A ~28 Hz oscillation, open.** Every run's θ spectrum peaks at 27–32 Hz, in both sessions (the rigid-model
  simulation puts its quantisation limit cycle at 10–16 Hz).
  - In runs 2–6 today it grew to several counts, with a_cmd saturated at ±2 m/s² 16–73 % of the time, and the runs
    ended in 1.6–3.9 s.
  - The edges are clean: no invalid transitions, a minimum gap of 0.16–0.4 ms, no chatter. The shaft really moves up
    to 5 counts per 2 ms tick, back and forth.
  - A rigid pendulum cannot do that at 28 Hz with 2 m/s² (that needs ~1000 rad/s²). Hypothesis: a structural or
    actuator resonance near 28 Hz inside the loop (rod bending, the shaft coupling, the cart or belt) that the
    controller sometimes pumps up.
  - The closed-loop cross-spectrum a_cmd → counts cannot identify it: in closed loop it returns about 1/|C|.
  - Next, with a go: a sensors-only tap test on the hanging and the held-upright rod (does it ring at 28 Hz with the
    motor off?), then an open-loop chirp of small cart accelerations while hanging. Fit the mode, then decide between
    a notch, a lower velocity-filter cut-off or less Kd.
  - Demo advice meanwhile: if a run starts rattling loudly, stop it (Enter) and catch again.

### Tap test and the velocity-filter trade (2026-10-14, `py/tap_test.py`, `py/fc_study.py`)

- **Tap test** (`logs/2026-10-14T10-58-02_encoder-log`): encoder-log only, motor off, taps on the bob, on the rod
  near the pivot, on the cart, and front-to-back.
  - On the swings the residual after a 6 Hz low-pass is pure quantisation (0.29 counts RMS = 1/√12).
  - With the pendulum still, each tap gave only a 0.06–0.18 s flicker across one count boundary, at scattered
    25–76 Hz. No sustained ring, no multi-count motion.
  - So the pendulum and its coupling do not ring noticeably with the motor off. The encoder cannot see a mode
    smaller than about 1 count, though.
- **Where the 28 Hz then lives.** If several counts of encoder motion at 28 Hz were cart translation, it would take
  X = θ·L_eq ≈ 5 mm at 28 Hz, i.e. > 150 m/s²; the command never exceeds 2 m/s². Likeliest: a mode that exists
  only while the driver holds and drives the cart. Candidates are cart rocking (pitch) from the belt pulling off
  the bearing axis with play in the rail bearings, or the encoder body on a compliant bracket.
  - The test that discriminates: an open-loop chirp of small cart accelerations while hanging. Any visible
    response near 28 Hz is the resonance, since the rigid response there is about 0.03 counts per m/s².
    It needs a small new mode and a go.
- **Mitigation independent of the cause:** cut the loop gain at 28 Hz, which is mostly Kd × the velocity estimate.

| Velocity filter (`ctrl.est_fc`) | a_cmd per 1-count wobble at 28 Hz | \|L(28 Hz)\| | PM | DM | GM [lo, hi] | RMS a, quantisation | 2° catch: overshoot / settle |
|---|---|---|---|---|---|---|---|
| 20 Hz (now) | 1.49 m/s² | 0.045 | 50° | 48 ms | [0.25, 6.9] | 0.19 | 18 % / 0.71 s |
| 16 Hz | 1.03 | 0.031 | 48° | 45 ms | [0.25, 5.7] | 0.15 | 15 % / 0.77 s |
| **14 Hz** | **0.82** | 0.025 | 46° | 43 ms | [0.25, 5.1] | 0.13 | 13 % / 0.69 s |
| 12 Hz | 0.63 | 0.019 | 44° | 40 ms | [0.25, 4.4] | 0.11 | 13 % / 0.77 s |
| 10 Hz | 0.46 | 0.014 | 40° | 36 ms | [0.24, 3.6] | 0.10 | 12 % / 0.74 s |

  - At 20 Hz a ±1.3-count wobble at 28 Hz already saturates the ±2 m/s² command, which is how readily it ran away in
    runs 2–6.
  - **14 Hz** cuts that by 45 % for 4° of phase margin and 5 ms of delay margin, and also lowers the buzz.
  - Proposed A/B on the rig: `./demo.sh --set ctrl.est_fc=14`. If the growth stops, the loop was pumping a
    drive-side resonance. A 28 Hz notch (needs code) would cost even less phase at crossover, once the mode is
    pinned down.

### Filter A/B result and trim iterations (2026-10-14, `py/osc_compare.py`)

| Velocity filter | Sessions | Runs where the 20–40 Hz band exceeded 1 count RMS | Quiet runs: counts / a_cmd band RMS |
|---|---|---|---|
| 20 Hz | 15-30-52, 10-20-52 | **3 of 17** (10-20-52 runs 3–5: 2.5–5.5 counts) | 0.12–0.62 / 0.17–0.89 m/s² |
| 14 Hz | 11-02-59, 11-08-04 | **0 of 7** | 0.15–0.31 / 0.12–0.25 m/s² |

- Not statistically decisive: the mode was rare at 20 Hz too, so 0 of 7 would have a ~25 % chance at the old rate.
  But it agrees with the model (45 % less loop gain at 28 Hz) and with the lower buzz, and the margins stay
  comfortable. **`ctrl.est_fc = 14` is now the default.**
- Catch quality at 14 Hz: RMS θ 0.056–0.07° in clean runs, the same as at 20 Hz.

Trim iterations (offset from the quiet stretches, `stage1_report.py balance`; untrimmed = residual minus the trim):

| Session | Trim | Residual offset | Untrimmed offset | Longest untouched run |
|---|---|---|---|---|
| 2026-10-12 15-30-52 | 0 | +2.09 counts (+54 mm/s²) | +2.09 | 3.3 s |
| 2026-10-13 10-15-19 | 0 | +0.70 (wide range, pushes) | +0.70 | — |
| 2026-10-14 10-20-52 | −2.1 | −0.12 (−3 mm/s²) | +1.98 | 14.9 s |
| 2026-10-14 11-02-59 | −2.1 | −0.78 (2 stretches) | +1.32 | — |
| 2026-10-14 11-08-04 | −2.1 | −0.70 (12 stretches, −18 mm/s²) | +1.40 | 5.2 s |

- The offset scatters by about ±0.5 counts from one program start to the next. That is the start-up calibration,
  not the trim, and it limits what a fixed trim can do to about ±13 mm/s².
- **`encoder.trim = −1.6`** (today's mean) is now the default.
- Next for longer untouched runs: a cleaner release, since v0 was +80 … +100 mm/s in runs 1, 4 and 5 (always +x).
  Engage on release is the systematic fix, and Stage 2 cart feedback the real one.
