# Pendel: handover

For whoever picks up the rig next: where the project stands, what is open, and where everything is.
Status as of **2026-11-20**. Items marked **TODO (handover)** still need input from the outgoing maintainer.
List them with `grep -n "TODO (handover)" HANDOVER.md`, and delete each marker once it is filled in.

- Every other document, and whether it is current or a dated record: [`docs/README.md`](docs/README.md).
- The full history, run by run: the "Status" section of [`CLAUDE.md`](CLAUDE.md).

## 1. The project in short

- **The rig.** A stepper motor drives a cart along a ~0.5 m track between two limit switches. The pendulum hangs on a
  600-pulse quadrature encoder (2400 counts per revolution). A Raspberry Pi 4 on a custom carrier PCB runs a 500 Hz
  C++ controller (`pendctl`). The controller runs either on the Pi (local mode) or on a PC over UDP with a constant
  delay (remote mode).
- **History.** The rig was built for a 2025/26 bachelor thesis. The thesis code is upstream at
  https://github.com/lzprograms/Pendel and does not compile as published. From October 2026 the software was
  redeveloped in stages, following [`CONTROL_BRIEF.md`](CONTROL_BRIEF.md):
  - Stage 0: bring-up and identification.
  - Stage 1: a balance PID on the Pi.
  - Later: swing-up, remote mode, a cascaded cart loop, and a simulation study of evolved neural controllers.

  The hardware also changed: a new carrier PCB, a new carriage and a reworked pivot.
- **Today.** The rig catches the pendulum by hand and balances it for the full 60 s run limit. It swings the
  pendulum up and hands over to the balance controller. Both work with the controller on a PC. The next lever is
  the actuator limits (Stage 0.9).

## 2. Start here

**Reading order.** Nothing in it moves anything.
1. This file.
2. "Safety first" in [`docs/02_setup_and_operation_guide.md`](docs/02_setup_and_operation_guide.md) (`docs/02`
   from here on), before going near the rig.
3. [`docs/01_theory_and_technical_reference.md`](docs/01_theory_and_technical_reference.md) (`docs/01`):
   - §1: the rig.
   - §2.4–2.6: the model.
   - §3.1–3.5: the controllers and the supervisor.
   - §4.6: safety.
   - §5: the software.
4. `docs/02` §0–6 to set up a PC and reach the Pi. Then §7 (local runs) and §8 (remote runs).
5. [`config/pendctl.conf`](config/pendctl.conf): every setting, with its unit and the reason for its value.
6. Any topic note as needed, via [`docs/README.md`](docs/README.md).

**First steps.** Steps 1–4 never move the cart.
1. Get access (§6). Clone the `control-dev` branch (`docs/02` §4) and set up the Python environment (`docs/02` §5).
2. Run the PC tests: `bash tests/unit/run_pc.sh`. Every test must pass.
3. Put the code on the Pi (`docs/02` §6.6), then run `make && make test` there. For tests on a simulated GPIO
   chip, see [`docs/stage0_3_safe_bringup.md`](docs/stage0_3_safe_bringup.md).
4. Plot a recorded run with `py/plot_run.py` (`docs/02` §7.6), for example `logs/2026-11-03T10-02-18_balance`.
   This needs the log archive (§5).
5. First motion: `./demo.sh`, after the pre-flight checklist (`docs/02` §7.2). If possible, have someone with you
   who has run it before.
6. If the rig was rebuilt, moved or rewired, work through "After a mechanical rebuild" in
   [`docs/stage1_demo.md`](docs/stage1_demo.md) before the first catch.

## 3. Where things stand

### 3.1 Shown on the rig

| What | Result | Run (`logs/`) | Details |
|---|---|---|---|
| Hand catches, angle-only PID (Stage 1) | RMS θ 0.08° on the first catches; a full 60 s run (the limit) on the new PCB and carriage | `2026-10-12T15-30-52_balance`, `2026-11-02T15-52-23_balance` | [`docs/stage1_demo.md`](docs/stage1_demo.md); the 11-02 run: `CLAUDE.md`, Status 2026-11-02 |
| Swing-up by energy pumping, local | 6 of 6 handed over; 2 balanced the full 30 s (the limit) | `2026-11-03T10-02-18_balance` | `CLAUDE.md`, Status 2026-11-03; design: [`docs/swingup_notes.md`](docs/swingup_notes.md) |
| Remote balance: PID on a wired PC, 4 ticks (8 ms) of constant delay | 60 s at RMS θ 0.099°, as good as local; 172 907 replies, none lost | `2026-11-03T14-32-03_balance` | [`docs/remote.md`](docs/remote.md) |
| Remote swing-up: pumping on the Pi, balance on the PC | 5 of 7 handed over; 2 balanced 30 s | `2026-11-03T15-32-17_balance` | [`docs/remote.md`](docs/remote.md) |
| Stage 3b (cart loop with integral, hand-over speed gate), local swing-up | 6 of 7 handed over; 3 balanced 30 s | `2026-11-04T10-16-08_balance` | `CLAUDE.md`, Status 2026-11-04; design: [`docs/stage3_cascade.md`](docs/stage3_cascade.md) |
| Plant after the pivot rework | ω₀ 4.97 rad/s, b/J 0.054 1/s, Coulomb 0.168 rad/s² (now the config defaults) | `2026-11-02T15-43-11_encoder-log` | `CLAUDE.md`, Status 2026-11-02 |

The rig results from 2026-11-02 to 2026-11-04 are written up only in the `CLAUDE.md` Status log so far. The topic
notes stop before them (§7).

### 3.2 Built, but not run on the rig

- **The current defaults with hand catches or in remote mode.** Stage 3b is on by default, but it has run on the rig
  in only one local swing-up session (`2026-11-04T10-16-08`). All 5 hand catches with the earlier P-only cart loop
  ended at the soft limit. Stage 3b is the fix, but it is untested in hand catches.
- **Stage 0.9 actuator test** (`pendctl --mode actuator`, `./actuator.sh`, `py/actuator_report.py`). It was built
  and tested on the PC on 2026-11-19 (commit `94de94c`), but not yet built on the Pi. Plan, prerequisites and rig
  blocks: [`docs/stage0_9_actuator.md`](docs/stage0_9_actuator.md).
- **Upright-offset estimate carried between catches** (`./offset_sim.sh`,
  [`docs/offset_estimator.md`](docs/offset_estimator.md)). In progress since 2026-11-19: simulation and log replay,
  plus a port into the Pi runtime. The port is off by default and not yet built on the Pi.
- **Engage on release, with the integrator pre-load** (`sup.engage_on_release`, `ctrl.preload`). Built, but off
  by default (`docs/stage1_drift.md`).
- **Evolved neural controllers (ENS³).** Simulation only (`./neuro_sim.sh`, [`docs/neuro_sim.md`](docs/neuro_sim.md)).
  They converge on the PID without its integral and do not beat it. A rig run would need its own PC program and
  launcher, and neither exists.
- **PCB bring-up stages D and E** (`./pcbtest.sh D|E`, [`docs/pcb_bringup.md`](docs/pcb_bringup.md)). Never logged,
  although the board has run the rig since 2026-11-02.

### 3.3 Open problems, most important first

1. **Actuator limits (Stage 0.9).** Balancing runs at a_max 2 m/s² and v_max 0.3 m/s, the only limits proven so
   far. These limits, not the controller, are what limits balancing:
   - In simulation, 45 starts are lost by both the PID and the evolved controllers. At today's limits, 22 of them
     cannot be caught by any controller.
   - At 3 m/s² and 0.5 m/s, 28 of the 45 become robustly catchable: about 94 % success, against 86 % for the best
     possible controller at today's limits (`docs/neuro_sim.md` §6).
   - Better control buys at most about 1 point.

   Before the first 0.9 run, see "Before the first 0.9 motion" in `docs/stage0_9_actuator.md`:
   - record the A4988 Vref;
   - fit a motor-only power cut, with a bulk capacitor at VMOT;
   - pad the end stops.

   The 1 A VMOT fuse may blow during the tests.
2. **The upright reference shifts from day to day.** The fitted offset was +0.08 counts on 2026-11-03 and +1.2 to
   +1.4 counts on 2026-11-04. An offset δ makes the cart accelerate steadily (about g·δ). The Stage 3b cart integral
   copes with up to about 1.5 counts; at 2.5 counts only 5–10 of 29 simulated hand-overs survive
   (`docs/stage3_cascade.md`). A possible fix is to estimate the offset online (`docs/01` §3.8, "State estimation").
   A first version is in progress in simulation (`docs/offset_estimator.md`).
3. **The cart wanders while balancing after a swing-up.** It moved ±90–136 mm in `2026-11-04T10-16-08`, against
   soft limits at ±0.15 m. The hand-over gate also ignores the cart position: one run was accepted at x = +68 mm
   (`docs/01` §3.8).
4. **The ~28 Hz vibration.** A mode at 28.1–28.6 Hz, cause unknown:
   - It stays at that frequency whatever the cart speed, so it is not the belt teeth or the step rate.
   - It does not ring when tapped with the motor off, so it is probably on the drive side
     (`docs/stage1_demo.md`, analysis of 2026-10-26).
   - It is still there on the new carriage. On 2026-11-19, `py/osc_speed.py` was run on `2026-11-02T15-52-23`,
     `2026-11-03T10-02-18` and `2026-11-04T10-16-08`. It found peaks at 27.8–28.6 Hz in open-loop moves and during
     swing-up.
   - It found none while balancing, where the velocity filter's cut-off (`ctrl.est_fc`) is 14 Hz. It was lowered
     from 20 Hz because the mode grew in some balance runs.
5. **Gates B and C are open.**
   - The Gate B report ([`docs/gateB_report.md`](docs/gateB_report.md)) is a draft waiting for 0.9. Its numbers
     use the original pivot and the 20 Hz filter.
   - Gate C needs `docs/stage1_report.md`, which was never written. Its data tool, `py/stage1_report.py`, exists.
6. **Smaller loose ends.**
   - GitHub's `main` branch is old and inconsistent: its Pi code speaks remote protocol v1, its `py/remote_ctrl.py`
     speaks v4. Use `control-dev` (`docs/02` §4.2).
   - The encoder reader once stalled for 7.3 ms during a fast spin. It was never explained
     (`docs/stage0_6_7_10_results.md`).
   - `pendctl --mode remote-sim` with the reworked pivot's friction sticks the simulated rod: pass
     `--set plant.coulomb=0`. A 10-minute confirming run at `remote.delay_ticks` 4 is also still open
     (`docs/remote.md`, "Open").
   - 16 places in `docs/01` and `docs/02` are marked "TODO / unverified".

### 3.4 What was tried, and what it taught

- **Angle-only control cannot keep the cart on the track.** Any upright offset or release momentum carries it to a
  soft limit (`docs/stage1_drift.md`). Hence the cart loop (Stage 3).
- **A P-only cart loop parks the cart at g·δ/kx.** For a 1.3-count offset that is about 0.13 m: the soft limit.
  Hence the integral (Stage 3b).
- **Hand-overs with the cart near v_max end within 0.2–0.5 s.** Hence the speed gate `swingup.v_sw` (Stage 3b).
- **The swing-up arrival needs about 1 % energy precision.** `swingup.e_ref` 0.02 with `swingup.u_sw_deg` 3 gave
  0 of 4 hand-overs. `swingup.e_ref` 0.01 with `swingup.u_sw_deg` 5 gave 6 of 6 (`CLAUDE.md` Status 2026-11-03).
- **A support bearing added on the encoder shaft raised the pivot friction 27-fold.** The start-up calibration then
  never accepted. Realigning the bearing brought the friction down to 14× the original. Re-identify the plant after
  any change at the pivot (`CLAUDE.md` Status 2026-11-02).
- **A toothed idler would not cure the 28 Hz mode.** The spectrum has no belt-tooth line (`docs/stage1_demo.md`).
- **v_max 0.4 m/s fails the soft-limit braking check** on this track (`CLAUDE.md` Status 2026-11-03).
- **Evolved neural controllers rediscover the PID** and do not beat it (`docs/neuro_sim.md`).
- **Three incidents set the safety rules in §4:**
  - 2026-10-08: a stale binary moved the cart.
  - 2026-10-22: a probe on the powered 40-pin header killed the rig's Pi.
  - A bench supply in constant-current mode at 2 A burned inductor L1 on the PCB (`docs/pcb_bringup.md`).

### 3.5 Next steps (draft)

**TODO (handover):** confirm, reorder or replace this list. Add what you would do with another month, and why.

The draft follows the evidence above and a review of controller options on 2026-11-19:
1. **Finish Stage 0.9.** Prerequisites first, then the Pi build and mock test, then the rig blocks. Set a_max and
   v_max from the results, and close Gate B with the current plant.
2. **Keep the upright offset across catches and sessions.** First, carry the Stage 3b integral's settled lean over
   as the trim. Then add a Kalman filter on θ, θ̇ and the offset, with the commanded acceleration as a known input.
   Validate it offline on logged runs first. A first version is in progress: it estimates the offset during each
   balance and applies it as the trim at the next catch (`docs/offset_estimator.md`).
3. **Swing-up: control the arrival.** Add the predicted cart position to the hand-over gate, and adapt
   `swingup.e_ref` from run to run.
4. **Remote mode: a state predictor on the PC** over the 4–6 commands in flight.
5. **LQR/LQI as the building block.** MPC only after 0.9 has measured the real limits; re-run
   `py/headroom_study.py` first.
6. **Write `docs/stage1_report.md`** for Gate C.

Each new controller gets its own mode and its own launcher script, next to `demo.sh`, `remote.sh` and `swingup.sh`.
It is built and tested in simulation first; `docs/01` §3.8 describes the pattern.

## 4. Rules that are not optional

The full lists are "Safety first" in `docs/02`, §4.6 of `docs/01`, and the working rules in `CLAUDE.md`.
- **Nothing that can move the cart runs** without the pre-flight checklist (`docs/02` §7.2), with the rocker switch
  within reach.
- **Before building, `rm -f` the old binary.** Chain the build and the run with `&&`, never `;`.
- **Never put a probe on the Pi's 40-pin header while anything is powered.** Use these test points instead, with
  the black lead on J2's GND:
  - 5 V: C4 +.
  - 3.3 V: R3's 3.3 V end.
  - VMOT: C5 +.
- **Run the bench supply in constant-voltage mode only.**
- **Edit on the PC, commit, then `git push pi control-dev`.** Never edit on the Pi. Never push to `upstream`.
- **After any mechanical change, re-commission before the first catch.** See "After a mechanical rebuild" in
  `docs/stage1_demo.md`.
- **After anything unexpected, check that the motor driver is off.** `pinctrl get 24` must read high
  (`docs/02` §7.5).

## 5. Where everything is

| What | Where | Notes |
|---|---|---|
| Code, docs, config | GitHub `simoncorazza7/Pendulum`, branch `control-dev` | Private, on a personal account (§6). `main` is out of date. |
| The Pi's copy | `pendel@pendel.local:~/Pendel-main` (git remote `pi`) | Updated only by `git push pi control-dev`. Its tree must stay clean. |
| Settings | `config/pendctl.conf` | `--set key=value` overrides a key for one run (`docs/02` §9). |
| Run logs | `logs/`, **not in git**: about 1 GB of runs and studies | Rig runs are written on the Pi and copied to the PC with `py/fetch_runs.py`. The October runs were recorded on the Pi that died on 2026-10-22, so the PC copy may be the only one. **TODO (handover):** archive location. |
| Bachelor thesis (German) | PDF in the folder above the repo, not in git | Summary: `docs/thesis_notes.md`. **TODO (handover):** where a successor gets it. |
| CAD and datasheets | `cad/` in the folder above the repo, not in git | `cad/README.md`, `docs/parts_reference.md`. **TODO (handover):** move into the repo or onto a shared drive. |
| PCB design | `pcb/` (KiCad) | No schematic PDF, Gerbers or BOM exported yet. **TODO (handover).** |
| Browser simulator | `pendulum-lab.html` in the folder above the repo, not in git | One-file cart-pole sim with a force or stepper actuator; it can load `config/pendctl.conf`. **TODO (handover):** move into the repo. |
| ML course built around the rig | `ml-first-principles/` in the folder above the repo | Its own git repo, with no remote. **TODO (handover):** decide whether it is part of the handover. |
| Explainer of the ENS³ paper (Pasemann 1998) | A Claude Docs document on the outgoing maintainer's account | Nobody else can open it, and it has suggested edits pending. **TODO (handover):** export it into `docs/`. |
| Original thesis code | https://github.com/lzprograms/Pendel (git remote `upstream`) | Read only; pushing there is disabled on purpose. |
| Image of the rig Pi's SD card | None yet | **TODO (handover).** A fresh Pi can be set up with `setup_pi.sh --rig` (`docs/02` §6.6). |

## 6. Access and people

| What | Now | Before the handover |
|---|---|---|
| GitHub repository | Private, owned by a personal account. Only the owner can add people. | Transfer it to the lab's organisation, or give the supervisor admin rights. |
| Pi login | Host `pendel`, user `pendel`. The password is deliberately not in the repo. | **TODO (handover):** record who holds it. |
| SSH keys | The outgoing maintainer's PC key is in the Pi's `~/.ssh/authorized_keys`. | Newcomers add their own key (`docs/02` §6.3). Remove the outgoing key. |
| Network | Lab LAN with DHCP, so the address changes (seen at .100–.102). Use `pendel.local`, which resolves from PowerShell but not from Git Bash. | — |
| Supervisor, owner of the rig | **TODO (handover)** | |
| Lab: room, keys, hours, who to ask | **TODO (handover)** | |

## 7. Handover checklist

For the outgoing maintainer. Delete this section when every box is ticked.

- [ ] Commit and push `control-dev`. As of 2026-11-20, several commits are not on GitHub
      (`git log origin/control-dev..` lists them), and work in progress is not committed (`git status`).
- [ ] Archive `logs/` where the lab keeps data, and fill in §5.
- [ ] Move or hand over `cad/`, `pendulum-lab.html`, the thesis PDF and `ml-first-principles/`; fill in §5.
- [ ] Export the ENS³ explainer into `docs/`. Accept or reject its pending suggestions first.
- [ ] Transfer the GitHub repository or add an admin. Record who holds the Pi password. Remove your SSH key from
      the Pi.
- [ ] Image the rig Pi's SD card, and record where the image is.
- [ ] Add photos to `docs/img/`:
  - the rig from the front, with +x, LEFT and RIGHT marked;
  - the PCB, with the powered test points circled;
  - the power path, with the rocker switch.
- [ ] Record a short video of one full session: power on, SSH, `./demo.sh`, a catch, stop, fetch the logs.
- [ ] Export the PCB: schematic PDF, Gerbers, BOM.
- [ ] Move the rig results of 2026-11-02 to 2026-11-04 out of the `CLAUDE.md` Status log and into the topic notes:
      `docs/stage1_demo.md` (the 60 s catch on the new hardware), `docs/swingup_notes.md` (the 11-03 tuning) and
      `docs/stage3_cascade.md` (the Stage 3b rig run).
- [ ] Make `CLAUDE.md` work for the next person. Today it names one person as the one who gives the go for every
      run and who sets the scope.
- [ ] Fill in every **TODO (handover)** in this file, and confirm §3.5.
- [ ] Run one supervised `./demo.sh` session together with the successor.
