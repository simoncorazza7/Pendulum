# CLAUDE.md: Pendel cart-pole controller

Full spec: `CONTROL_BRIEF.md`. This file condenses it, so read it first every session.
Derivations and long notes go in `docs/`. Keep this file under ~150 lines and update it whenever
a value or convention is established, and at the end of every session.

## Scope
Simon (mechatronics student) is continuing a 2025 bachelor-thesis rig. Current scope: **Stage 0**
(bring-up, audit, identification) and **Stage 1** (on-Pi angle-only balance PID). Nothing beyond that.
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

## Conventions
- SI internally (m, s, rad, m/s²). Counts/steps only at the hardware boundary. Unit comment on every
  constant. Log raw integers (counts, steps) next to the SI values.
- Git: branch `control-dev`, small descriptive commits.
- +x = direction of increasing step count ✱(unverified). θ = angle from upright, + when the top leans
  toward +x. Sketch to be added once established physically (0.8).
- German identifiers: Winkel = angle, Flanke = edge, Drehgeber = encoder, Endschalter = limit switch.

## System facts (✱ = unverified; see "Status" for what has been verified)
- Pi 4 Model B Rev 1.5, 1 GB. Host `pendel`, user `pendel`, `pendel@pendel.local` = 192.168.0.100 (wired
  eth0). Raspberry Pi OS bookworm, **32-bit armhf userland**, kernel 6.12 PREEMPT (not RT), RT throttling
  95 %, governor ondemand, desktop (labwc) running. libgpiod 1.6.3 (v1 API). g++ 12.2, Python 3.11.
  No gnuplot, no cmake. Full table in `docs/stage0_discovery.md`.
- Upstream: https://github.com/lzprograms/Pendel (push disabled). It does NOT compile as published.
- NEMA 23 (23HS2430) on A4988 (≤ 2 A; record Vref, don't change it), 24 V, 1/8 µstep (MS1=MS2=1,
  MS3 not connected per thesis schematic ✱). Steps/m must be measured; the thesis prior (GT2, 60T
  pulley, 1600 µsteps/rev) gives ≈ 13 333 µsteps/m. No cart encoder (position is open-loop).
- 24 V is fused at 1 A on VMOT (confirmed by Simon) and could blow in accel tests. Pull-ups are 8 kΩ (thesis).
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
  gpio_mockup_named_lines=1` (per boot), then `tests/mock/run_mock_tests.sh` (35 checks). See
  `docs/stage0_3_safe_bringup.md`.

## Status (details: `docs/session_log.md`)
- Claude runs on Simon's Windows PC, drives the Pi over SSH (host key ED25519
  SHA256:Wnp+aM+kwJnd9hLdTFa+kKDR0QoMufwu4zVaZaG2wlM). PC tests: `bash tests/unit/run_pc.sh` (46/46).
- Done: 0.1–0.4, Gate A. GPIO24 held high from boot (`gpio=24=op,dh`). Simon will add a motor-only E-stop.
- **0.5 in progress on the Pi** (started ~14:44 under nohup, mock chip): fetch `logs/*timing-probe*` +
  `build/timing_all.out`, run `py/timing_report.py`, write `docs/stage0_5_timing.md`.
- **Unpushed local commits** (PC was on another network): push to the Pi, then `make && make test` there.
- Built offline, validated on synthetic data only: §3 derivation (`docs/model_derivation.md`), 0.12
  sim/analysis (`docs/stage0_12_sim.md`), 0.11 estimators (provisional BD + 2nd-order LP ~30 Hz,
  `docs/stage0_11_estimators.md`), C++ core (pid, supervisor, units, swingfit; golden vs Python),
  0.7/0.10 fit scripts (`docs/stage0_7_0_10_fits.md`), hw::Hardware + SimHw (golden vs Python).
- **c_bottom must be re-found after every program start** (no index): small swing + `core::fitOffset`.
- Next on the rig (each needs a go): scope check of GPIO18 (timing-probe, `probe.allow_real_chip`),
  0.6 encoder hand tests (`pendctl --mode encoder-log`, `enc.allow_real_chip`), 0.7, 0.8 (first motion).
- Runtime tick `rt::BalanceRuntime` on hw::Hardware + closed-loop tests on SimHw done
  (`docs/stage1_runtime.md`); PC tests 46/46. Next in software (needs the Pi): PiHw adapter, pendctl
  shadow/balance modes with start-up swing calibration.
