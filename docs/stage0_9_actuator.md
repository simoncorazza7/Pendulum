# Stage 0.9: actuator capability

**Status (2026-11-20): tooling built and tested on the PC and the Pi; nothing has moved on the rig.**
- Done: commit `94de94c` was pushed to the Pi. `make` gave 0 warnings and `make test` passed 106/106 there.
- Mock test (13:46): A1–A5 pass, 20 of 20 checks.
- A6 failed homing with "tracking error", because the control loop stalled.
  - The loop ran up to 377 ms late, with 190 late ticks in all.
  - The Pi was busy at the time: `pendctl` had just been rebuilt with another session's offset code, and a real
    swing-up session started at 13:55.
  - A6 must be rerun on an idle Pi.
- **Stall guard added (2026-11-20):** if the control loop sends no command for `actuator.stall_timeout` (20 ms), the
  Axis step thread brakes the cart to standstill by itself, at the ladder's hardest acceleration. Tested on the PC
  (simulated rig) and built on the Pi (11-23, `2cc0116`: `make` 0 warnings, `make test` 114/114). See Risks.
- **The stalls came from printing in the control loop.** A write to stdout (a file on the SD card in the mock; an ssh
  terminal over Wi-Fi on the rig) blocked the loop for 0.23–1.56 s, each time at a print. The actuator mode now prints
  from a separate, non-real-time thread through a lock-free queue (`273dfb5`).
- **Mock on the Pi, 11-23 (`3668a12`): 28/28 checks**, worst loop lateness 0.0 ms in every run, 0 stall-guard trips,
  A6 homing passes.
- **The balance mode** (`demo.sh`, `swingup.sh`, `remote.sh`) prints the same way since `f280ff9`. Mocks on 11-23
  (`c647334`): balance 28/28, actuator 28/28, worst loop lateness 0.0 ms.
- Then: the prerequisites below, then the blocks.

Every rig block is a hardware action: it needs Simon's go for that block, after the statement in its row has been
read out (CLAUDE.md safety rules). The results will go into this file.

## What 0.9 decides

The balance modes run with `balance.a_max` = 2.0 m/s² and `balance.v_max` = 0.3 m/s. These are the reduced limits
chosen for the first catches: two thirds of the PRIOR 3.0 m/s² (`docs/stage1_demo.md`, "Reduced limits"). No
measurement backs them. The brief sets the controller limits at 50–70 % of the reliable maxima, measured with the
pendulum mounted (`CONTROL_BRIEF.md`, 0.9). Gate B still lists those maxima as PENDING 0.9 (`docs/gateB_report.md`).

0.9 measures:
- the reliable acceleration at each speed, a_rel(v), and the reliable speed v_rel;
- the deceleration the cart can brake with from speed (this sets `a_brake`);
- the step generator's delay, and whether the Axis cap ever bites;
- if a scope is available, VMOT during the hardest moves (the margin on the 1 A fuse).

From these it sets `balance.a_max`, `balance.v_max`, the `a_brake` values and the `move.max_*` typo guards. Then
the Gate B catch region is re-run at the new limits.

Why it matters, in simulation (`docs/neuro_sim.md` §6, `py/headroom_study.py`, 2026-11-19): at 2 m/s² and 0.3 m/s
most of the catches the controllers lose are limited by the actuator, not the controller, and 3 m/s² helps more than
0.5 m/s. `sup.a_brake` enters the same soft-limit check and was not varied there.

## What is known already

- **2 m/s² at up to 0.3 m/s is proven on the new carriage.** In four sessions (11-03 `10-02-18` and `15-32-17`,
  11-04 `09-15-27` and `10-16-08`):
  - the command sat at ±2 m/s² for 1.2–8.2 s in total, 0.8–2.6 s of it at |v| ≥ 0.2 m/s;
  - the peak speed was 0.29–0.30 m/s;
  - the 22 switch-touch audits read 0 or ±1 µstep, and 2 µsteps once.

  The ladder starts there.
- **The audit can see a single lost step.**
  - A stepper that falls out of sync slips by whole electrical cycles of 4 full steps. That is 32 µsteps
    (2.4 mm) at 1/8 step and 13 333 µsteps/m.
  - The touch scatter stays within ±2 µsteps, so one slip is 16 times the largest error seen so far.
- **The hanging pendulum feels the cart's real acceleration.** Its input gain ω₀²/g was confirmed at 93–103 % in
  0.10 (`docs/stage1_damp.md`). Apart from the switches, it is the only sensor of the cart's real motion.
  - One 2.4 mm slip moves it by only ~2.3 counts (`py/actuator_sizing.py`, with the pivot friction). That is close
    to the noise of a model fit.
  - So the pendulum catches stalls and repeated slips, and the touch audit catches single ones.
- **Holding torque is unlikely to be the first limit.**
  - Per kg of moving mass, 1 m/s² needs 1 N, which is 0.019 N·m at the 19.1 mm pulley (GT2, 60 teeth).
  - The rotor adds the equivalent of 0.22 kg (800 g·cm² / r²).
  - The motor holds 2.8 N·m at its rated 3 A. The A4988 runs it at ≤ 2 A, set by Vref, which is not recorded on the PC.
  - Torque falls with speed, and the cart mass and the motor's inductance (its torque-speed curve) are unknown.
  - So the limit may come from speed, the ~28 Hz drive-side mode, the fuse or the driver as easily as from torque.
    The ladder is there to find out which.
- **The Axis cap stays clear of the ladder.** `axis.max_accel` is 400 000 toggles/s² = 15 m/s², tapering as
  15·(1 − v / 4.5 m/s): 12.3 m/s² at 0.8 m/s. The ladder stops at 10 m/s².
- **Driver heating is not new.** The A4988 holds full current at standstill. Those sessions kept it on for
  6–7 min at a time without a trip.

## Before the first 0.9 motion

| # | Item | Why |
|---|---|---|
| 1 | **Vref recorded.** It should be in the pcb-test stage C log of 10-30 on the Pi (stage C item "C5 Vref"). If that item was skipped, measure it the way stage C says: black lead on J2's GND, red probe on the potentiometer's metal top only. Don't turn the potentiometer. | The results hold only for this current setting. |
| 2 | **A motor-only cut.** A switch in the 24 V feed to F2 (VMOT), upstream of C5 so the driver keeps its 100 µF. On this board F2 shares J2's +24 V with F1, so this is a wire mod at F2. **Never put it in the motor leads (J9):** opening a coil while current flows destroys the A4988. Without such a switch, the rocker (motor and Pi) is the cut; back up the SD card first. | Gate A N7/N8. The brief wants it before the aggressive 0.9 runs. |
| 3 | **F2 is a slow-blow T1A**, with a spare at hand. | PCB review #6. If F2 opens mid-move, the cart coasts (Gate A N6). |
| 4 | **Padding at both hard stops**, and the gap from each switch to its stop measured. | A motor that loses sync while braking cannot stop the cart, and the MGN15 carriage runs freely. |
| 5 | **Belt tension, carriage screws and encoder-bracket screws checked.** | The ladder jolts the cart at up to 10 m/s², and the upright trim depends on the encoder mount. |
| 6 | Optional: **scope on VMOT at C5's + leg**, black lead on J2's GND. Never on the 40-pin header. | Shows sag (current draw) and pump-up while braking. The A4988's absolute maximum is 35 V. |

## Tooling (built 2026-11-19)

`pendctl --mode move` cannot do this job:
- every run starts its own step count, so no switch reference carries over between runs;
- it cannot stream acceleration commands;
- an abort brakes at `move.a_brake` = 0.5 m/s², which from 0.8 m/s takes 0.64 m.

So 0.9 has its own mode and launcher. They sit beside the existing ones; the balance session, its modes and their
scripts are unchanged.

| Piece | Where | What it does |
|---|---|---|
| Runtime | `src/runtime/actuator.{h,cpp}`, `rt::ActuatorTest` | Homing, the ladder, the reference trajectories, the audit and its verdicts, the aborts. Portable: it also runs on `hw::SimHw`. |
| Mode | `src/app/actuator_mode.cpp`, `pendctl --mode actuator` | Lists the block's rungs before homing and states each rung before its Enter. Writes `data.csv`, `edges.csv`, `reps.csv`, `rungs.csv` and `meta.json`. |
| Settings | `actuator.*` in `config/pendctl.conf`, read by `rt::actuatorFrom` | Refuses speeds above `actuator.v_top` (0.8 m/s), accelerations above `actuator.a_top` (10 m/s²) or out of order, legs closer than 50 mm to a switch, and any ladder in which the Axis cap would bite. |
| Launcher | `./actuator.sh` | The checklist, the Vref (optional), `make` under `set -e`, then the mode on gpiochip4. |
| Report | `py/actuator_report.py [RUN ...]` | Per rep: touch error, step-generator delay and mismatch, pendulum fit, 28 Hz RMS. Then a_rel per speed, the limits by the 50–70 % rule, and `actuator.png`. Several runs merge into one map. |
| Replay | `py/actuator_replay.py RUN --t0 T0 --t1 T1 --amax A` | Turns a rig run's `a_cmd` into a bounded `t,x,v,a` table for the profile `table`. |
| Tests | `tests/unit/test_actuator.cpp` (11 tests); `tests/mock/run_actuator_mock.sh` (A1–A6, Pi only) | See "Checked so far". |

**How a block runs:**
- **One program run per block.** The driver stays on from homing to the end, so the step count stays the cart
  position, as in the balance session.
- **Homing** as in the balance session: right switch, left switch, span check, centre (`rt::Mover`).
- **Rep (`actuator.profile = pair`):** an out-and-back pair of legs centred on the track, from −d/2 to +d/2 and back.
  - Each leg accelerates at a to v, cruises 0.1 s and brakes at a.
  - The return leg starts one pendulum period T0 = 1.264 s after the outbound one. The two legs' kicks then cancel
    in the linear model, whatever the profile, so the pendulum is left nearly at rest (table below).
- **Other profiles:**
  - `oneway`: one leg per rep, alternating direction, each followed by a touch of the switch ahead.
  - `bang`: ±a, with the speed swinging between ±v; it ends at rest where it started.
  - `chirp`: acceleration amplitude a, exponential sweep up to 40 Hz, tapered ends. It starts high enough that
    |v| ≤ v and |x| ≤ 0.10 m.
  - `table`: a file from `py/actuator_replay.py`.
  - None of the streams needs a centring loop: each is bounded by construction.
- **Before each rep:** a shaped move to its start, then the cart is held until the pendulum swings less than 3°
  (at most 40 s, then the rep runs anyway and is flagged).
- **Audit after every rep:** touch the switch on the side where the rep ended, at `balance.v_seek` (0.04 m/s).
  Record the touch error in µsteps, then re-zero on the switch.
  - Within ±3 µsteps the rep passes.
  - At 4–7 µsteps the rep runs once more; a second marginal touch fails it.
  - At 8 µsteps (one full step) or more it fails, and the rest of that speed's ladder is skipped.
  - Beyond `balance.touch_tol` (15 mm) the block ends FAILED and the driver goes off.
- **Operator keys:** Enter runs the stated rung, `s` + Enter skips the rest of its speed, `q` + Enter stops.
- **Aborts during a run:**
  - The triggers: an unexpected switch, an overrun streak, the Axis leaving ACCELERATE, an encoder fault, a
    tracking error over 3 mm on the step count, `q`, Ctrl-C.
  - Each brakes at the rung's own acceleration, so by step count the cart never passes the planned end.
  - `q` and Ctrl-C end the block FINISHED; the others end it FAILED.
- **Command limit:** the command is limited to 1.1 × the rung's acceleration. The tracking feedback adds about 2.5 %
  in the simulation.

**Checked so far (PC, 2026-11-19):**
- `bash tests/unit/run_pc.sh`: 106/106, of which 11 are new, on the simulated rig:
  - **The ladder:** 0.45 m/s at 2, 6 and 10 m/s², 2 reps each. Every touch was within 1 µstep and every rung
    passed. Tracking stayed under 1 mm, and at most 0.3° of swing was left at a touch.
  - **One slip:** 32 µsteps injected into a rep. That rung fails (touch +32 ±1), the rest of its speed is skipped,
    and the next speed runs clean after the re-zero.
  - **Small and large losses:** 5 µsteps repeats the rep once, and the rung then passes. 300 µsteps (22 mm) ends
    the block FAILED with the driver off.
  - **q mid-leg** at 0.6 m/s and 8 m/s²: it brakes at 8 m/s², never passes the leg end, and ends FINISHED.
  - **The rest:**
    - `s` skips a speed;
    - one-way legs alternate switches and settle below 3°;
    - bang and chirp stay within ±0.10 m and 0.45 m/s;
    - the settings refuse bad values;
    - the table loader refuses malformed files.
- **WSL compile check:** g++ 15 compiles the Pi-only sources with a stub `gpiod.h` (`-Wall -Wextra`, no warnings).
  Nothing was linked or run.
- **Report and replay:** both were run on a simulated block written in the mode's file format.
  - The injected slip shows as FAIL at +33 µsteps, and the step generator's 0.5 ms delay is recovered exactly.
  - The pendulum fit (with the pivot's Coulomb term) gives input gains of 90–99 % and residuals of 1.1–1.9 counts.
    It does not see the single slip, as expected: the touch audit decides.
  - A replay table from 11-03 `10-02-18` run 6 ran twice on the simulated rig and passed both touches.
- **On the Pi (2026-11-20, commit `94de94c`):** `make` with 0 warnings (441 s) and `make test` 106/106.
- **Mock on the Pi (2026-11-20 13:46):** A1–A5 pass (20 checks). A6 failed homing on a loop stall of up to 377 ms
  (see Status). It still needs a rerun on an idle Pi.

When committing, keep `actuator.sh` and `tests/mock/run_actuator_mock.sh` executable:
`git add --chmod=+x actuator.sh tests/mock/run_actuator_mock.sh`.

## Rig blocks

Each block is one run of `./actuator.sh` and one go.
- **Abort:** Ctrl-C or `q` brakes at the rep's acceleration; the motor-only switch (or the rocker) cuts the drive.
- **Start:** every block homes first: switch to switch at ≤ 0.10 m/s, seeks at 0.04 m/s, ~25 s.
- **Audit travel:** between reps, the cart travels at ≤ 0.10 m/s to the right switch and back.
- **End:** each block ends with the cart centred and the driver off.

| Block | What moves | Max v / a | Legs | Motion |
|---|---|---|---|---|
| **B0** tool check | 3 reps at 0.3 m/s and 2 m/s² (proven values) | 0.3 m/s / 2 m/s² | ±35 mm | ~1 min |
| **B1** | ladder at 0.3 m/s: a = 2, 3, 4, 5, 6, 8, 10 m/s², 2 reps each | 0.3 / 10 | ±35 mm | ~2 min |
| **B2** | the same at 0.45 m/s | 0.45 / 10 | ±75 mm | ~2 min |
| **B3** | the same at 0.6 m/s | 0.6 / 10 | ±120 mm | ~2 min |
| **B5a** | confirmation: each speed's top passing rung 5 more times (one run per speed) | as found | as above | ~1 min per speed |
| **B5b** | one-way legs at the same rung: out → touch the left switch, back → touch the right | as found | as above | ~1 min per speed |
| **B6** | streams at A = a_rel(0.45) (see list below) | 0.45 / A | ±50 mm | ~3 min |
| **B4** optional, last | ladder at 0.8 m/s: a = 3 … 10 m/s² | 0.8 / 10 | ±145 mm (78 mm from the switches) | ~2 min |

The B6 streams:
- bang-bang, 10 s;
- a chirp, 10 s, from A/(2π·0.45 m/s) up to 40 Hz;
- replays of rig `a_cmd` records: 11-03 `10-02-18` run 6 (22 % saturated), and a 10-14 run with the 28 Hz mode.
  - Their speed bounds them: run 6 would need 0.87 m/s at 6 m/s².
  - At |v| ≤ 0.45 m/s, `py/actuator_replay.py` scales it to 3.3 m/s².
  - So a replay checks realistic waveforms at about half the ladder's peak; the bang-bang stream covers the full A.

B4 is needed only to take `balance.v_max` above 0.42 m/s (70 % of 0.6 m/s). By the 50–70 % rule, v_max 0.5 needs a
reliable speed of 0.7–1.0 m/s.

The commands, on the Pi after `git push pi control-dev`. V and A are the speed and the rung found in B1–B3:

| Block | Command |
|---|---|
| B0 | `./actuator.sh --set actuator.accels=2 --set actuator.reps=3` |
| B1 | `./actuator.sh` (the defaults: 0.3 m/s, a = 2 … 10 m/s², 2 reps) |
| B2 | `./actuator.sh --set actuator.speeds=0.45` |
| B3 | `./actuator.sh --set actuator.speeds=0.6` |
| B5a | `./actuator.sh --set actuator.speeds=V --set actuator.accels=A --set actuator.reps=5` |
| B5b | `./actuator.sh --set actuator.profile=oneway --set actuator.speeds=V --set actuator.accels=A` |
| B6 | `./actuator.sh --set actuator.profile=bang --set actuator.speeds=0.45 --set actuator.accels=A --set actuator.reps=1`, then the same with `chirp` |
| B6 replay | see the steps below |
| B4 | `./actuator.sh --set actuator.speeds=0.8 --set actuator.accels=3,4,5,6,8,10` |

The B6 replay takes three steps:
1. On the PC, build the table:
   `python py/actuator_replay.py logs/2026-11-03T10-02-18_balance --t0 314.4 --t1 334.4 --amax A --out build/replay_run6.csv`
2. Copy it to the Pi's `build/`.
3. On the Pi, run it: `./actuator.sh --set actuator.profile=table --set actuator.table=build/replay_run6.csv`

After each block, on the PC: `py/fetch_runs.py`, then `py/actuator_report.py logs/<run> ...`.

### Rep sizes

From `py/actuator_sizing.py`: the identified plant with its Coulomb term, the pendulum hanging at rest at the start,
0.1 s of cruise per leg, the return leg starting at T0. Each leg takes 0.16–0.70 s, so the return always starts at
1·T0. A rep with its touch takes about 8 s.

| v (m/s) | leg d (m), a = 2 … 10 m/s² | peak swing during the rep | left after the pair | left after one leg alone |
|---|---|---|---|---|
| 0.3 | 0.07 … 0.04 | 8.9° … 5.0° | 1.3° … 0.9° | 8.9° … 4.7° |
| 0.45 | 0.15 … 0.07 | 16.6° … 8.3° | 2.1° … 1.1° | 16.6° … 8.3° |
| 0.6 | 0.24 … 0.10 | 24.9° … 12.3° | 3.1° … 1.5° | 24.9° … 12.3° |
| 0.8 (a ≥ 3) | 0.29 … 0.14 | 32.1° … 18.5° | 4.4° … 2.5° | 32.1° … 18.5° |

The pair leaves 5–8 times less swing than a single leg; the rest comes from the pivot friction and the nonlinearity.
The one-way legs of B5b leave up to 32°. The program waits for that to decay below 3° before the next leg: the
Coulomb friction takes ~1.2°/s off.

Streams at |v| ≤ 0.45 m/s:
- bang-bang ±A has a period of 1.8/A s (0.18–0.45 s for A = 10…4 m/s²) and moves x by ±10–25 mm;
- the chirp starts at 1.4–3.5 Hz with x ±20–51 mm.

## Pass criteria

- **Rep:** all of these:
  - touch error within ±3 µsteps;
  - no abort;
  - peak |a_cmd| ≤ 1.05·a;
  - the Axis speed equals the integrated command to within one tick;
  - no encoder fault;
  - a pendulum-fit residual like that of the B0 reps.
- **Rung:** all its reps pass.
- **a_rel(v):** the highest rung at that speed that passed its 2 reps, the 5 confirmation reps of B5a, and the
  one-way legs of B5b. If even 10 m/s² passes, a_rel ≥ 10 m/s² is only a lower bound.
- **Streams:** every touch within ±3 µsteps at A = a_rel(0.45). Otherwise step A down one rung and repeat.

## From the results to the config

- **`balance.a_max`** = 0.5–0.7 × the smallest a_rel(v) for v up to the chosen v_max. `ctrl.a_max` (shadow mode)
  gets the same value.
- **The brake values** `sup.a_brake`, `balance.a_brake`, `move.a_brake` and `damp.a_brake`: at most 0.7 × a_rel(v_max).
  - This is what can unlock v_max. The session refuses limits for which the soft limit, plus the braking distance
    from v_max, plus touch_tol reaches the switches (`src/runtime/params.cpp:182-188`).
  - Today 0.3 m/s just fits with a_brake 1.0. With a_brake 3 m/s², 0.5 m/s fits inside the ±0.15 m soft limits.
- **`balance.v_max`** = 0.5–0.7 × v_rel, and it must pass that reach check.
- **`balance.span_usteps`** 6104 → 5939 for the new carriage. It passes the ±5 % span check now, but the reach
  check uses this value.
- **`move.max_accel`, `move.max_speed`:** raise them to the new limits; these typo guards cap every balance mode.
- **`tests/unit/test_params.cpp:27`** asserts that the Axis cap exceeds 4 × `ctrl.a_max`. Above 3.75 m/s², raise
  `axis.max_accel` or revisit that check. The Axis cap must stay above a_max at v_max in any case.
- **`swingup.a_pump`** stays ≤ `balance.a_max`; this is checked at start.
- **Then:**
  - re-run `py/gateb_design.py`, `py/catch_study.py` and the swing-up study at the new limits;
  - fill in Gate B's PENDING rows;
  - stop at Gate B.
- **After Gate B:** the brief's step "raise the limits toward the Gate B values" happens in rig catches, with the
  step-loss audit. More authority also lets the Kd·ω spikes from single count steps (±2.4 m/s² each) through
  unclipped. In those first sessions, watch the saturation and the 28 Hz content (`py/osc_compare.py`).

## Risks

| Risk | Mitigation |
|---|---|
| A slip while braking: the cart coasts past the leg end on the free-running carriage. | Legs centred, ≥ 78 mm from the switches even at the top speed; padded stops; 0.8 m/s last and optional. |
| F2 opens or the A4988 shuts down: the outputs go off and the cart coasts. | The same; a spare T1A at hand; VMOT on the scope. |
| Jolts shift the encoder mount or the belt. | Check them before. The first balance session afterwards reports the trim residual (`py/stage1_report.py balance`). |
| The chirp excites the ~28 Hz mode. | Watch the encoder; `q` ends the stream. A rigid pendulum answers 28 Hz with only 0.3 counts at 10 m/s², so a large 28 Hz signal in the counts comes from the structure. |
| The step mode flips (the old board's fault). | Homing's span check at every start; the touch audit (a flip makes the error huge). |
| **The control loop stalls during a run.** The Axis keeps executing the last ACCELERATE command, so a 0.4 s stall at 10 m/s² adds 4 m/s. A 377 ms stall happened in the mock on 11-20 while the Pi was busy. The overrun check only acts after the stall. | **Stall guard** (`Axis::setCommandWatchdog`, on only in `--mode actuator`): with no new command for 20 ms the step thread, which keeps running during a stall, brakes the cart to standstill at the ladder's hardest acceleration. In the simulated rig a 0.4 s stall at 10 m/s² peaks at 0.40 m/s with the guard and 2.3 m/s without (`actuator_stall_guard_brakes_the_cart`). The late RUN then aborts, and `reps.csv` records each rep's worst loop lateness (`worst_late_ms`). A cap on the Axis maxSpeed was rejected: its acceleration taper would cut into the rungs. Also keep the Pi idle during blocks (no builds, no other SSH sessions). |
| Slips on the way out and back cancel in the net touch error. | The one-way legs of B5b, and the pendulum fit of each leg. |

## Open questions for Simon

1. Vref: is it in the 10-30 stage C log?
2. Is there a motor-only switch now? If not, build one in the F2 feed, or use the rocker with the SD card backed up?
3. The gap from each switch to its hard stop, and padding?
4. A scope (and a DC clamp meter) for VMOT and the supply current?
5. Top rung 10 m/s² and top speed 0.8 m/s: OK?
6. Optional: the moving mass (cart plus pendulum), for the torque margin.
7. When the Pi is on: the go to build there and run `make test` and `tests/mock/run_actuator_mock.sh` (mock chip,
   nothing moves).
