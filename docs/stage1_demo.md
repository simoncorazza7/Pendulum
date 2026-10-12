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
