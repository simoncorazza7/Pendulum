# Session log

Detailed per-session notes moved out of CLAUDE.md to keep it short.

## Session 1 (2026-10-08), full log
- Session 1 (2026-10-08): Claude runs on Simon's Windows PC (Wi-Fi "birdwatcher", 192.168.0.103)
  and drives the Pi over SSH. The Pi host key is ED25519 SHA256:Wnp+aM+kwJnd9hLdTFa+kKDR0QoMufwu4zVaZaG2wlM.
  - 0.1 discovery done: `docs/stage0_discovery.md`. Thesis read: `docs/thesis_notes.md`.
  - Git set up on the Pi and the PC (see Code sync). 1 A fuse on VMOT confirmed by Simon, so it limits 0.9.
  - 0.2: the Pi copy builds with 0 warnings (`build/main_asis`); upstream fails (2 errors). Simon made
    the fixes and built/ran the old `main`.
  - Gate A answers (2026-10-08):
    - Handlers: SIGINT/SIGTERM → ramp ≤ 0.5 s then EN = 1; fatal → EN = 1 at once; SIGPIPE ignored. OK'd.
    - `main` default = safe (no homing, no plot unless `--home`/`--plot`).
    - E-stop: the rocker is the only cut today; Simon will add a motor-only switch before motion tests.
    - Rig is connected and the mount is usable.
    - RT tests: run with `sudo` per run, no limits file. The Pi has NOPASSWD sudo, but still ask per brief.
    - Python venv on the PC is approved. Spare GPIO for timing probes: BCM18 (pin 12), unconnected per
      the schematic.
  - The SIGPIPE path was reproduced standalone: killed at 4.96 s. Simon is unsure whether his `main` survived homing.
  - Unclaimed GPIO24 boots as input pull-down → EN low → **driver enabled whenever no program owns
    it**. Added `gpio=24=op,dh` to `/boot/firmware/config.txt` (backup `config.txt.bak-2026-10-08`);
    took effect after a reboot (verified `op hi`).
  - Mechanics: the belt's supporting structure reaches the left switch before the cart does
    (shorter usable travel at that end). Measure in 0.8.
  - 0.3 done: atomics, bring-up mode, signal handlers; mock suite 35/35; 0.3b on the real chip passed
    (EN stays high on SIGINT/TERM/SEGV; the real encoder counts).
  - 0.4 done: ring log, run dirs + meta, config, pendctl log-selftest, Makefile, unit tests 9/9, PC venv
    (Python 3.14, numpy 2.5.3, scipy 1.18.1, matplotlib 3.11.2). The SCHED_OTHER 500 Hz loop wakes ~56 µs
    late (timer slack). Next: 0.5 timing.
  - 0.5 in progress: four 10-min timing-probe runs started 2026-10-08 ~14:44 Pi time under nohup
    (accel-stream/speed × idle/load, mock chip). Output: Pi `~/Pendel-main/build/timing_all.out` and
    `logs/*timing-probe*`. Then: fetch, `py/timing_report.py`, write `docs/stage0_5_timing.md`.
  - 0.6 tooling committed locally (e1d75cc, not yet pushed/built): `pendctl --mode encoder-log`,
    `py/encoder_report.py`. Next: scope check of GPIO18 (needs a go) and the 0.6 hand tests.
  - Offline while the PC was on another network: `docs/model_derivation.md` (§3 confirmed + 3
    refinements, `py/model_check.py` 9/9) and the 0.12 framework `py/pendel/{control,plant,sim,linear}.py`
    with a placeholder-parameter preview (`docs/stage0_12_sim.md`). Not validated yet.
  - C++ controller core `src/core/pid.{h,cpp}` (pure pidStep, mirrors py/pendel/control.py) with golden
    test vs Python (`py/make_golden.py`, tests/golden/): bit-identical on the PC. PC tests:
    `bash tests/unit/run_pc.sh` (20/20). Pi: `make test` (not yet run there).
  - 0.11 on synthetic data (`docs/stage0_11_estimators.md`): provisional pick = backward difference +
    2nd-order Butterworth LP ~30 Hz (same delay tolerance as BD+LP1 30 Hz, ~30 % less chatter). Redo on
    real logs before deciding; then mirror in pid_step/pid.cpp/golden.
  - Supervisor + units core (`src/core/supervisor`, `units`, mirrors in py/pendel) and swing fits for
    0.7/0.10 (`docs/stage0_7_0_10_fits.md`). **c_bottom must be re-found after every program start** (no
    index): small swing + `core::fitOffset` with w, sigma from 0.10. PC unit tests 30/30.
