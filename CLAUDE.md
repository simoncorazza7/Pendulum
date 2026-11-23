# CLAUDE.md: Pendel cart-pole controller

Full spec: `CONTROL_BRIEF.md`. This file condenses it, so read it first every session. Overview: `docs/explainer.md`.
Derivations and long notes go in `docs/`. Keep this file under ~150 lines and update it whenever
a value or convention is established, and at the end of every session.

## Scope
Simon (mechatronics student) is continuing a 2025 bachelor-thesis rig. Current scope: **Stage 0**
(bring-up, audit, identification) and **Stage 1** (on-Pi angle-only balance PID). Exceptions (Simon): swing-up
(Stage 4, 10-14; `./swingup.sh`, `docs/swingup_notes.md`); remote controller on the PC (Stage 2b, 10-19;
`./remote.sh`, `docs/remote.md`); cascaded cart loop (Stage 3, 11-03; `docs/stage3_cascade.md`); evolved neurocontrollers
(Pasemann 1998 / ENS³, 11-19), **simulation only** (`./neuro_sim.sh`, `py/neuro/`, `docs/neuro_sim.md`): a separate method
with its own launcher, never inside `swingup.sh` or the swing-up code (Simon). Nothing else beyond.
Explain the reasoning behind design choices.

## Safety rules (non-negotiable)
- Claude cannot see the rig. **Never** run anything that energises the motor, moves the cart, or
  arms a controller without Simon's explicit "go" for *that specific run*.
- Before each run, state what moves, its direction, max distance/speed/accel, duration, how it ends,
  and how to abort. Before the first motion test, confirm a motor-power cut is within reach.
- **Starting the existing `main` is a hardware action**: the `Axis` ctor calls `home()`, enables
  the driver, and drives toward the right limit switch.
- **Incident 2026-10-08**: a failed build plus `;` ran a stale `main` twice, and the cart moved. Rules:
  `rm -f` the binary before building; gate every run with `&&` on the build; never run any binary
  that can open the real chip (`gpiochip0`/`gpiochip4`) without a go. Use the mock chip for checks.
  After anything unexpected, run `pinctrl get 24` and restore `pinctrl set 24 op dh` (driver off).
- Ask first for: `sudo`/system changes, package installs, pin/driver changes, deleting or
  restructuring code, and changes to the existing TCP protocol. No drive-by refactors.
- Fix nothing on a hunch; show evidence (a test or a log) first. If a hardware fact is uncertain, ask;
  if it's measurable, design a measurement.
- Simon types passwords himself. Never push without asking (upstream isn't his).
- Stop at Gates A, B and C, report, and wait.
- **Never ask Simon to probe the 40-pin header while it is powered** (2026-10-22: a probe across pins 1-2 put 5 V on
  the Pi's 3V3 and killed the rig Pi). Powered test points: 5 V = C4 +, 3V3 = R3's 3V3 end, VMOT = C5 +.

## Conventions
- SI internally (m, s, rad, m/s²). Counts/steps only at the hardware boundary. Unit comment on every
  constant. Log raw integers (counts, steps) next to the SI values.
- Git: branch `control-dev`, small descriptive commits.
- **+x = increasing Axis step count = toward the LEFT switch** (as Simon stands at the rig front),
  established physically on 2026-10-09 (`calib.x_sign = 1`). The right switch (GPIO2) is at the low-pos end,
  as the code assumes. θ = angle from upright, + when the top leans toward +x. **`theta_sign = +1`**
  (hand test 2026-10-09: bob pushed toward +x lowered the counts, i.e. θ fell from π). The 0.10 pulse
  test must confirm the combined sign before any balancing.
  ```
     LEFT switch (GPIO9)                                   RIGHT switch (GPIO2)
     pos high  |<========== +x (pos increasing) ==========|  pos low
               |<------------ ~0.5 m, 1536 steps --------->|
  ```
- **Step mode is intermittent** (2026-10-09): morning half-step (1526 steps ≈ 0.5 m track, 3072/m), afternoon
  1/8 step (307 steps → ~23 mm, ≈ 13 300/m = thesis value; config 13333). Suspect a loose MS1/MS2 wire. A flip to
  half-step makes every move 4.3× longer/faster: check a short move's distance before motion runs. New PCB fixes it.
- German identifiers: Winkel = angle, Flanke = edge, Drehgeber = encoder, Endschalter = limit switch.

## System facts (✱ = unverified; see "Status" for what has been verified)
- Pi 4 Model B Rev 1.5, 4 GB (replacement from 2026-10-23; the original 1 GB Pi died 10-22). Host `pendel`, user `pendel`, `pendel@pendel.local` (DHCP: .100 on 10-09, .102 on 10-12; use the name) (wired
  eth0). Raspberry Pi OS bookworm, **32-bit armhf userland**, kernel 6.12 PREEMPT (not RT), RT throttling
  95 %, governor ondemand, desktop (labwc) running. libgpiod 1.6.3 (v1 API). g++ 12.2, Python 3.11.
  No gnuplot, no cmake. Full table in `docs/stage0_discovery.md`.
- Upstream: https://github.com/lzprograms/Pendel (push disabled). It does NOT compile as published.
- NEMA 23 (23HS2430) on A4988 (≤ 2 A; record Vref, don't change it), 24 V, 1/8 µstep (MS1=MS2=1,
  MS3 not connected per thesis schematic ✱). Steps/m must be measured; the thesis prior (GT2, 60T
  pulley, 1600 µsteps/rev) gives ≈ 13 333 µsteps/m. No cart encoder (position is open-loop).
- 24 V is fused at 1 A on VMOT (confirmed by Simon) and could blow in accel tests. Pull-ups (new PCB): encoder 4.7 kΩ, switches 10 kΩ.
  Details in `docs/thesis_notes.md`. Thesis PDF is in the folder above the repo.
- Pendulum encoder: quadrature, NPN OC, pull-ups to 3.3 V, 600 PPR → 2400 counts/rev ✱.
- Limit switches NO, pulled up (1 = free, 0 = pressed).
- GPIO (BCM), may change with the new PCB, so confirm with Simon: enc A=10, B=19; STEP=27, DIR=22,
  EN=24 (0=enabled); limit R=2, L=9; MS1=20, MS2=21. Matches the thesis schematic. `gpioinfo` shows no
  I²C/SPI claims (all BCM 2–27 unused). `/dev/gpiochip4` → `gpiochip0` symlink (udev), so the code's
  `"gpiochip4"` works. User `pendel` is in group `gpio` (no sudo needed for GPIO).
- Code: C++17, libgpiod v1 C API, std::thread. `main` = TCP server :8080 (protocol must not change).
  `Pendel` owns `Encoder` + `Axis`, opens `"gpiochip4"`, runs a gnuplot thread.
- Code facts (Gate A, `docs/gateA_report.md`):
  - **Units ×2**: protocol *rates* (speed, accel, max*) are STEP toggles = half-µsteps; positions are
    µsteps. Keep this in the protocol; convert in the new runtime.
  - Protocol sign: +accel → +speed → +pos (DIR = 1). pos ≈ 0 at the "right" switch (GPIO2). Internally
    `curSpeed` = −protocol speed. The physical direction is still unknown.
  - **Limit switches never stop motion** (outside homing they only rewrite `pos`). `stopOnLimit` isn't a safety net.
  - `main` dies ~5 s after start via SIGPIPE (gnuplot missing). `persist_gpio_outputs=Y`, so after any exit
    EN stays 0 (driver on, holding).
  - The Pi's 5 V comes from the 24 V via the buck, so the rocker switch also cuts the Pi.

## Model (§3 of the brief)
Input = cart accel a. Near upright: θ̈ = ω₀²θ − (b/J)θ̇ − (ω₀²/g)a. ω₀ and b/J come from a hanging free swing.
PID a = Kpθ + Ki∫θ + Kdθ̇. Pole placement (s²+2ζω_c s+ω_c²)(s+p):
Kp = g(1+(ω_c²+2ζω_c p)/ω₀²), Kd = (2ζω_c+p−b/J)g/ω₀², Ki = gω_c²p/ω₀². Kp > g is required.
A reference offset δ gives a steady cart accel ≈ gδ, so sub-count upright calibration matters.

## Stage plan
- **Stage 0**: 0.1 discovery → 0.2 build as-is → **Gate A** → 0.3 safe bring-up mode, signal
  handlers, atomics → 0.4 ring-buffer logging + CSV + Python venv → 0.5 RT timing → 0.6 encoder
  integrity → 0.7 upright reference → 0.8 cart calibration/conventions → 0.9 actuator capability →
  0.10 pendulum ID → 0.11 velocity estimator → 0.12 Python sim → **Gate B**.
- **Stage 1**: pure step-function controller + supervisor (IDLE→ARMED→ENGAGED→STOPPING), new
  executable, sim-backed HW interface, N-sample delay line (default 0). Shadow → hanging damping
  → catches → experiments → `docs/stage1_report.md` → **Gate C**.

## Code sync (PC ↔ Pi)
- Edit and commit on the PC in `C:\Users\simon\Downloads\Pendel-main\Pendel-main` (branch `control-dev`).
- `git push pi control-dev` updates the Pi's `~/Pendel-main` working tree (`updateInstead`); the Pi tree
  must be clean. Never edit on the Pi directly. Never push to `upstream`.
- `pi-baseline` = the Pi code as found (upstream `eb760fe` + 2 build fixes), commit `345ab78`.
- SSH from the PC: `ssh -o BatchMode=yes pendel@pendel.local '<cmd>'` (key `~/.ssh/id_ed25519`).

## Build / run
- Build (on Pi): `./Kompilierer.sh` (`g++ -Wall -std=c++17 -o main main.cpp encoder.cpp axis.cpp
  pendel.cpp -lgpiod -lgpiodcxx -pthread`). Untracked prebuilt `main` from 2026-10-07 exists on the Pi.
  Kompilierer.sh now also compiles `safety.cpp`.
- Run (on Pi, HARDWARE ACTION): `./start.sh [--home] [--plot] [--chip NAME]`. The default is bring-up
  mode: no homing, no plot, EN = 1 until the first motion command. `--home` = original start-up
  (homes at once). SIGINT/SIGTERM → brake ≤ 0.5 s → EN = 1; crash → EN = 1.
- `make` (Pi) → `build/main`, `build/pendctl`, `build/unit_tests`; `make test` runs the unit tests.
  `pendctl --mode log-selftest` has no GPIO code. Runs go to `logs/<time>_<mode>/` (data.csv, meta.json
  with the git commit, config_used.conf). PC: `.venv/Scripts/python py/fetch_runs.py`, then `py/plot_run.py`.
  See `docs/stage0_4_instrumentation.md`.
- Mock tests (no hardware): `sudo /sbin/modprobe gpio-mockup gpio_mockup_ranges=-1,28
  gpio_mockup_named_lines=1` (per boot), then `tests/mock/run_mock_tests.sh` (35), `run_stage1_mock.sh`,
  `run_balance_mock.sh`. See `docs/stage0_3_safe_bringup.md`.
- **Demo / Stage 1 step 3: `./demo.sh`** (checklist, build, `pendctl --mode balance` on the real chip; interactive,
  Simon runs it in `ssh -t`). Procedure, messages, design: `docs/stage1_demo.md`. Claude runs it only with a go
  (stdin through a FIFO, as in the balance mock). **`./remote.sh`** = the same with the PID on the PC
  (`py/remote_ctrl.py` first); `pendctl --mode remote-sim` = no GPIO, simulated pendulum (latency measurement).

## Status (details: `docs/session_log.md`)
- Claude runs on Simon's Windows PC, drives the Pi over SSH (host key ED25519
  SHA256:+/uYSPx4f31bInTJd3Lgdp3Fgh76LSaEVURRtlaBABE, new Pi 10-23). Tests: `bash tests/unit/run_pc.sh` / Pi `make test` (106, both, 11-20).
  Pi = 192.168.0.102 on the lab LAN; pendel.local resolves only from PowerShell/Windows ssh, not Git Bash.
- Done: 0.1–0.5 (scope check passed 2026-10-09), Gate A. GPIO24 held high from boot (`gpio=24=op,dh`).
- E-stop: the rocker switch (cuts motor AND Pi); a motor-only switch (+ bulk cap at VMOT) before the aggressive 0.9 runs.
- **New PCB soldered 2026-10-21** (KiCad in `pcb/`; same pin map; TSR 3-2450 5 V, MS1/MS2 on board). Bring-up A-E:
  `docs/pcb_bringup.md`, `./pcbtest.sh C|D|E` (virtual board: `tests/pcb/run_pcb_fake.sh`). Not yet run on the rig.
  2026-10-22: stage B had no 5 V (Simon fixed it; readings not reported); then the **rig Pi 4 died** (probe spark across header pins 1-2, Pi on the
  board): red on, green dark/solid, no HDMI even on USB-C. Earlier, 2 A in bench-supply CC mode burned L1 (U1 may have
  seen >30 V). 10-23: new Pi set up (`setup_pi.sh --rig`, GPIO24 op hi at boot, make + 88 tests OK), NOT on the PCB until
  the board re-check passes (Ω at J2+/C6+/C4+, 5 V at C4+ with no Pi).
- 0.5 (`docs/stage0_5_timing.md`): loop p99 6/35 µs idle/load; step edges p99 ≤ 12 µs. SoC heatsink fitted 10-09.
- C++ core mirrors Python with golden tests (pid, supervisor, units, swingfit, estimator, SimHw); `docs/model_derivation.md`.
- **c_bottom must be re-found after every program start** (no index): small swing + `core::fitOffset`.
- 2026-10-09 (`docs/stage0_6_7_10_results.md`): CPR = 2400 exact, no lost edges, c_bottom repeatable ±0.13 counts.
  **Plant: ω₀ = 4.931 rad/s (L_eq 0.4035 m), b/J 0.024 1/s, Coulomb 0.0117 rad/s²** (config `plant.*`). Open: 7.3 ms reader stall.
- 2026-10-09 first motion (0.8): right switch at low pos; θ sign by hand (theta_sign = +1); EN verified (3.3 V at ENABLE).
  Afternoon (`docs/stage1_damp.md`): start-up swing calibration, `--mode shadow` / `--mode damp`; on the rig shadow,
  0.10 pulse (gain 93 %, signs confirmed), hanging damping (decay 1.02 vs 0.99 1/s). Stage 1 steps 1-2 done.
- Drift (`docs/stage1_drift.md`): v_inf = v + L_eq*theta' + g*int(theta) at engage; i_delay 0; a hand holding the rod after
  ENGAGED ~90 mm/s per 50 ms (`py/hand_hold.py`). Engage on release + pre-load built but OFF. Reference offset -> trim.
- 0.11: BD+LP2 20 Hz (`docs/stage0_11_estimators.md`). Open for Gate B: 0.9 with the new PCB; 0.6b.
- 2026-10-12: 0.12 validated on the rig data (`py/sim_validate.py`); **Gate B draft `docs/gateB_report.md`**
  (PID wc=2w0, LP2 20 Hz: GM [0.25, 6.9], PM 50 deg, DM 48 ms; catch region set by actuator limits). PENDING 0.9.
  2026-10-14 **zeta 0.7 kept (Simon)**: Ms minimal at 0.6-0.7; zeta 1 doesn't remove the catch overshoot (integrator);
  Ts 2 ms: w_gc*Ts = 0.036 (Gate B appendix; py/zeta_study.py, zeta_sweep.py, ts_study.py).
- 2026-10-12 **balance session** (`rt::Session` + `rt::Mover`, `./demo.sh`, `docs/stage1_demo.md`): homing with span check
  (6104 usteps = 1/8 step), shaped moves (one period T0), switch-touch re-centre (step-loss audit), a_max 2 / v_max 0.3,
  `keep_driver_on` hold. First rig catches 15-30-52: RMS theta 0.08 deg (= sim); drift from a +2.1-count offset; long runs = hand.
- 2026-10-14 `10-20-52_balance`: **trim -2.1 validated** (residual -3 mm/s^2, run 11 14.9 s untouched) -> config default.
  Open: **~28 Hz oscillation** (all runs peak 27-32 Hz; runs 2-6 grew, a_cmd saturated): suspected structural/actuator
  resonance. Tap test (motor off): no ring -> likely drive side (cart rocking/belt while driven). 10-26 `py/osc_speed.py`:
  a **fixed 28.1-28.6 Hz mode** at 15-300 mm/s, also in open-loop moves; no belt-tooth line (500/m), so a toothed idler
  won't fix it; the full-step ripple (1667/m) is visible. Old carriage only: the rig is disassembled (10-26), no new-carriage
  runs yet, so rerun it on the first ones. **est_fc 20 -> 14 Hz
  default** (0 of 7 runs grown vs 3 of 17; PM 46, DM 43 ms). **Trim -1.6** (5 sessions, start-to-start scatter ~0.5
  counts). Limit now: release momentum (+80..100 mm/s).
- 2026-10-14 **swing-up built** (`rt::SwingUpRuntime`, `./swingup.sh`; energy from LP2(theta)). Rig 11-36-43: **7/7 swung
  up and handed over** (18-19.5 s), energy out 7/7; balance 0.25-11.6 s: cart at 70-110 mm/s at hand-over -> soft limit.
  Tuned e_ref 0.007, u_sw 3. Real fix: Stage 2 cart feedback.
- 2026-10-19 **remote controller** (Stage 2b, `docs/remote.md`): constant delay remote.delay_ticks, shadow PID, `RemoteFault`.
  PC 82/82 (bit-exact = delay line under jitter); WSL e2e OK; Windows PC program worst answer 0.99 ms (default priority).
  Next: wired Windows PC measurement, Pi build. 10-20: **new carriage** being fitted: re-commission first
  (`docs/stage1_demo.md` "After a mechanical rebuild": theta sign, span, trim, plant).
- 2026-11-02 rig rebuilt on the new PCB + carriage (Pi now 192.168.0.101); PCB stages C/D/E and re-commissioning not
  logged. Simon added an external support bearing on the encoder shaft: demo calibration never accepts. Free swing
  `11-02T15-27-20_encoder-log`: 43 deg -> rest in 16 s, **Coulomb 0.31 rad/s^2 (27x), stiction +-4.4 counts**, linear
  decay; w0 ~5.0 at small amplitude. Replayed calibration: no `calib.swing_sigma`/`plant.w0` passes (rms 5-10 counts).
  Decision (Simon): fix the bearing friction first (alignment / seals), then re-record and update `plant.*`.
  After Simon's bearing fix, `15-43-11` (fit --t0 2 --t1 29): 43 deg -> rest in ~27 s, **Coulomb 0.168 (14x old), b/J
  0.054, w0 4.97** (A^2 coeff 0.071, ~rigid). Calibration replay: accepts at 12-18 deg with w0 4.97, swing_sigma 0.12
  (c 1.8-2.0 consistent); current defaults still reject. Sim (catch_study plant): no falls, runs reach the soft limit
  sooner (3-4.6 s vs 4-10 s). Not yet in config: pass via `--set` for the demo.
  Simon then ran `./demo.sh` with those `--set`s: `11-02T15-52-23_balance` calib accepted (se 0.03), **span 5938 usteps**
  (new carriage, -2.7 %, inside tol), touch errors 0/1; 3 catches: 2.8 s, **60 s (max run time)**, 16.4 s -> theta sign and
  combined loop OK on the new PCB/carriage. pcb-test: only stage C logged (10-30). Next (Simon): swing-up. Estimated
  extra loss on the last approach with the new friction ~+2 % of 2w0^2 (bJ*4w0 + coulomb*pi), so e_ref 0.007 may turn
  back ~15 deg short: first run at 0.007, then raise e_ref from the measured arrival energy (`py/swingup_report.py`).
- 2026-11-03 swing-up `09-44-23_balance` (defaults + plant --sets): 2 runs, **no hand-over in 40 s, peak energy 0.92 /
  0.84**, x -53..+63 mm, |v| <= 220 mm/s. Energy balance from data.csv (input = -a theta' cos theta / 2g, a from v):
  input ~0.08-0.10/s vs loss 0.07-0.09/s at E 0.8-0.9 (model bJ*<theta'^2> + coulomb*<|theta'|> gives 0.064; old
  plant ~0.02). Next try (Simon OK'd looser limits): a_pump 1.1, k_e 2, e_ref 0.02 (old arrival loss 0.7 % x ~3.5),
  pump_max 60; v_max kept 0.3 (0.4 fails the soft-limit + braking reach check and the move.max_speed guard).
  Result `09-52-36_balance`: pumping now reaches the top in 17-18 s (4/4), but **0/4 hand-overs**: arrivals E 0.99-1.015
  (over the top at 30-70 deg/s, or turned back at 10-11 deg), min |u| 2.9-5.6 deg vs u_sw 3; cart at x +-50..100 mm,
  |v| 150-250 mm/s at the top; all 4 ended "soft limit while pumping". Arrival ~0.7-1 % below e_ref at k_e 2 (not 2.4 %).
  **`10-02-18_balance` (e_ref 0.01, u_sw 5): 6/6 handed over** (pump 16.6-21 s, x -81..+95 mm), arrivals E 1.0008-1.0048,
  hand-over at |theta| 9.1 deg, u 1.3-3.8 deg. Balanced 2.8, 0.35, 0.36, 0.96, **30, 30 s** (balance_max). Short runs: cart
  +52 mm at +190 mm/s at hand-over (soft limit in 0.35 s). Run 5: RMS theta 0.10 deg, 0 % sat, x -65..+91; run 6: RMS
  1.19 deg, 22 % sat, 1.6 Hz, many identity-residual steps (hand? note the identity ignores the new Coulomb torque).
  **Config defaults now = these values** (Simon, 11-03): plant 4.97/0.054/0.168, calib.swing_sigma 0.12 (swing ~20 deg),
  encoder.trim 0, swingup a_pump 1.1, k_e 2, e_ref 0.01, u_sw 5, pump_max 60. Session/swing-up tests simulate the new plant.
- 2026-11-03 remote (wired PC 192.168.0.103): `14-14-50_remote-sim` 42 085 replies, 0 lost, RTT p99.9 2.39 / max 4.17 ms ->
  **remote.delay_ticks 4**. remote-sim with the new plant.coulomb: the sim rod sticks at upright and the cart runs away
  (fell at 6.04 s, 3 runs); use `--set plant.coulomb=0` (gains don't use it). Next: 10-min confirm run, then `./remote.sh`.
  **First rig remote session `14-32-03_balance`** (delay 4): 3 catches 47.9 s (RMS 0.70 deg, 1.1 Hz, pushes flagged),
  5.1 s, **60 s at RMS 0.099 deg** (= local quality). Trim residual +0.08 counts (trim 0 holds). Link 5.8 min, 172 907
  replies, 0 lost; RTT p99.9 2.5 ms but clustered stalls to **13.2 ms** (at 5.9 s, 56 s): 8 late, 4 held ticks engaged,
  longest run 3 (abort at 5). Report: N=7 covers all, 6 leaves 2 single holds. Confirm sim 14-25-13 (no taps): max 9.3 ms,
  1 late; ended at the 9.5 m sim soft limit after 68.6 s (angle-only drift), not the link. -> **remote.delay_ticks 6** (Simon).
- 2026-11-03 **remote swing-up built** (`./swingup.sh --remote`; docs/remote.md "Remote swing-up"): pumping on the Pi,
  BALANCE on the PC with the constant delay; protocol v2 sends the pre-load as `i0` (old PC program: link never up).
  Sim (`py/remote_catch_study.py`, 10-02-18 hand-overs): 6 ticks no worse than local, no pre-load worse. PC 91/91.
  Pi 91/91 + e2e 4/4. **First rig run `15-32-17_balance`: 5/7 handed over, 30 s + 30 s (balance_max) remotely**, also 1.8,
  0.36, 0.18 s (cart at -41..-51 mm moving -200..-253 mm/s at hand-over). Link 180 793 replies, 0 late, 0 missed, max
  10.0 ms; |PC - shadow| = 0 (pre-load i0 arrived). Runs 4-5 never pumped up: sudden energy losses of 0.1-0.25/s (10x
  friction; run 4 at E 0.26 = 61 deg, run 5 every ~1.5 s at ~30 deg) = Simon interfering on purpose. No step loss
  (touch errors 0/-1). `15-26-06`: encoder_same_type 2, both runs "encoder fault": pendulum taken off, program started,
  pendulum put back on (Simon). Not a wiring fault.
- 2026-11-03 **Stage 3 cascade built** (Simon: "go there"): theta_ref = clamp(-(cart_kx x + cart_kv v)/g, +-4 deg) into the
  balance PID (`core::cascadeStep`, `control.cascade_step`, golden). Default ON: cart_kx 0.25, cart_kv 1.0 (w_x 0.5, zeta 1;
  `py/cascade_study.py`: 12/13 sim cases kept 30 s vs 0/13 angle only). Hand catches, swing-up BALANCE, remote (protocol
  v3). P outer loop parks ~0.1 m per count of reference error. PC 94/94. Not on the rig yet; 0/0 = Stage 1.
- 2026-11-04 first Stage 3 rig sessions (7853407): hand catches 08-58-08 / 09-09-03 all 5 runs ended at the soft limit
  at +139 mm after 4.7-8.7 s; swing-up local 09-15-27 0.2-18.5 s, remote 09-20-36 0.2-1.9 s. Cause 1: reference offset
  +1.2/+1.4 counts today (stage1_report; yesterday +0.08): the inner loop tracks theta_ref within 0.3 counts, but the
  P outer loop parks at g delta / kx = 0.13 m -> soft limit. Needs an outer integral (or the trim). Cause 2: hand-overs at
  +228..+242 mm/s (v_max 0.3) end in 0.2-0.5 s -> hand-over speed gate. remote.delay_ticks back to 4 (Simon, d7b9710).
- 2026-11-04 **Stage 3b built**: cart integral (kx 0.41, kv 1.0, ki 0.05, i_max 2 deg; reset per catch) + swing-up gate
  swingup.v_sw 0.07 on |v + (g/w0) u| (rig: all >= 8 s balances had <= 62 mm/s). Sim at 1.3 counts: 20/29 vs P 2/29.
  Protocol v4. PC 95/95. docs/stage3_cascade.md "Stage 3b". Not on the rig yet.
  Pi (fad2469): 95/95 after fixing a 128-byte test buffer (PARAMS 140 B overflowed it: UB, wrong value on ARM only);
  e2e 4/4 (one earlier abort = a 464 ms stall of the non-RT Python on the Pi). Rig `10-16-08` (local swing-up, Stage 3b):
  6/7 handed over, **3 x 30 s**, 3.9 / 3.5 / 0.7 s; gate refused run 5 (v_pred 106 mm/s) -> came round -> soft limit
  while pumping. Open: cart wanders +-90..136 mm while balancing; the gate ignores x (run 2 accepted at x +68 mm).
- 2026-11-19 **ENS³ neurocontrollers, simulation only** (`docs/neuro_sim.md`, `./neuro_sim.sh test` 14/14): paper reproduced only
  with output-at-once + semi-implicit Euler (explicit Euler: 2-input nets 0 %). `py/neuro/rig.py` = pendel.plant/pid_step/BdLp2
  exactly. Unseen plants, 300 x 30 s: PID + cart loop 81 %, evolved 4-input net (x, theta, v, omega; one neuron, no recurrence)
  85 % at 1.7x effort; 3-input recurrent nets fragile, not sane on replayed rig logs. No PC program, no launcher, no rig run.
  Re-evolved at the PID's effort (`--budget`, delays 4-7, late replies): 81-85 % at 0.15-0.18 m/s^2; the gains converge on
  the PID's own (a ~ 43-64 theta + 5.5 omega + 6-9 x + 7-8 v, no integral: ends 15-31 mm off centre vs 6 mm). No gain over the PID.
  Headroom (`py/headroom_study.py`, LP feasibility + oracle MPC): of 45 starts both lose, 22 uncatchable by any controller at
  a_max 2 / v_max 0.3, 20 only with zero slack, 3 robust: more PC compute buys <= 1 point. a_max 3 / v_max 0.5 (the ctrl.*
  priors) makes 28 of 45 robustly catchable (~94 %): Stage 0.9's actuator limits are the lever.
- 2026-11-19 **0.9 actuator test built** (`docs/stage0_9_actuator.md`: plan, blocks B0-B6 with commands; sizing
  `py/actuator_sizing.py`). `pendctl --mode actuator` (`rt::ActuatorTest`, `src/app/actuator_mode.cpp`, `actuator.*` keys)
  + `./actuator.sh`: homes, then ladders of out-and-back reps (return at T0 leaves the pendulum ~at rest), oneway/bang/
  chirp/table profiles, switch-touch audit after each rep (pass +-3 usteps, 4-7 repeat, >= 8 fail + skip that speed; one
  slip = 32); aborts brake at the rung's a. `py/actuator_report.py` (audit, step-generator delay, pendulum fit with
  Coulomb, 28 Hz, a_rel, 50-70 % limits), `py/actuator_replay.py` (rig a_cmd -> table). Commit 94de94c, pushed to the Pi
  11-20: make (0 warnings, 441 s) + make test 106/106 there (PC the same). Mock 13:46: A1-A5 pass (20/20 checks);
  A6 FAILED homing "tracking error": the loop stalled up to 377 ms (190 late ticks) while the Pi was busy (pendctl
  rebuilt with the offset code, then a real swing-up session at 13:55). Rerun A6 on an idle Pi. During a stall the Axis
  keeps the last ACCELERATE command, so the actuator mode now turns on a STALL GUARD (Axis::setCommandWatchdog, opt-in,
  off for main and the other modes): no command for actuator.stall_timeout (20 ms) -> the step thread brakes the cart
  at the ladder's hardest a. SimHw model + test: 0.4 s stall at 10 m/s^2 peaks at 0.40 m/s with it, 2.3 m/s without.
  reps.csv has worst_late_ms per rep. Pi 11-23 (2cc0116; 2ad3299 did not link on armhf: 64-bit atomic): make 0 warnings,
  make test 114/114. NOT yet: the mock (A6 rerun) with the guard. Keep the Pi idle during actuator blocks.
  Baseline: 2 m/s^2 at <= 0.3 m/s proven (22 audits <= 2 usteps). Open: Vref, motor-only cut, end-stop padding.

