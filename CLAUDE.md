# CLAUDE.md: Pendel cart-pole controller

Full spec: `CONTROL_BRIEF.md`. This file condenses it, so read it first every session. Overview: `docs/explainer.md`.
Derivations and long notes go in `docs/`. Keep this file under ~150 lines and update it whenever
a value or convention is established, and at the end of every session.

## Scope
Simon (mechatronics student) is continuing a 2025 bachelor-thesis rig. Current scope: **Stage 0**
(bring-up, audit, identification) and **Stage 1** (on-Pi angle-only balance PID). Exceptions (Simon): swing-up
(Stage 4, 10-14; `./swingup.sh`, `docs/swingup_notes.md`); remote controller on the PC (Stage 2b, 10-19;
`./remote.sh`, `docs/remote.md`). Nothing else beyond.
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
  SHA256:+/uYSPx4f31bInTJd3Lgdp3Fgh76LSaEVURRtlaBABE, new Pi 10-23). Tests: `bash tests/unit/run_pc.sh` / Pi `make test` (88).
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
