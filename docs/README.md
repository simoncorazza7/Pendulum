# Pendel documents

Every document in the project, what it covers, and how current it is, as of 2026-11-20. New here? Start with
[`../HANDOVER.md`](../HANDOVER.md).

**Current values** (gains, limits, plant) are in [`../config/pendctl.conf`](../config/pendctl.conf) and
[`01_theory_and_technical_reference.md`](01_theory_and_technical_reference.md). Dated records keep the numbers of
their day.

Many files were last edited on 2026-11-19 in a wording pass. A recent edit date does not mean the numbers in a file
are current.

Status words:
- **Current:** kept up to date.
- **Record:** what was measured or decided on that date, not updated since.
- **Draft:** unfinished.
- **Plan:** not run yet.

## Start here

| Document | What it covers | Status |
|---|---|---|
| [`../HANDOVER.md`](../HANDOVER.md) | Where the project stands, open problems, where everything is, access | Current |
| [`02_setup_and_operation_guide.md`](02_setup_and_operation_guide.md) | From an empty Windows PC to running the rig in local and remote mode: git, Python, SSH, safety, the pre-flight checklist, changing settings, troubleshooting, a cheat sheet | Current |
| [`01_theory_and_technical_reference.md`](01_theory_and_technical_reference.md) | The rig, physics and model, PID and cascade, latency, hardware and pinout, safety, software architecture, the remote protocol. §3.8 is the roadmap. | Current |
| [`../config/pendctl.conf`](../config/pendctl.conf) | Every setting, with its unit and the reason for its value | Current |
| [`working_with_the_pi.md`](working_with_the_pi.md) | A short version of `02`: get the code, where to start reading, working on the Pi | Current. `02` is the full version. |

## Rules, plan and history (repository root)

| Document | What it covers | Status |
|---|---|---|
| [`../README.md`](../README.md) | The front page | Current |
| [`../CLAUDE.md`](../CLAUDE.md) | Working rules (safety, conventions), hardware facts, the model, the stage plan. Its run-by-run Status log is the most complete history of the project. | Current, but written for Claude Code and around one operator. See "Known stale statements" below. |
| [`../CONTROL_BRIEF.md`](../CONTROL_BRIEF.md) | The original brief: ground rules, model, staged roadmap, Gates A–C, deliverables | The plan of record. Stages 2b, 3 and 4 were started early as exceptions (`CLAUDE.md`, Scope). |
| [`session_log.md`](session_log.md) | Detailed notes of the sessions on 2026-10-08 and 2026-10-09 | Record. Stops on 2026-10-09; later sessions are only in the `CLAUDE.md` Status log. |

## Controllers and modes: design and rig results

| Document | What it covers | Status |
|---|---|---|
| [`stage1_demo.md`](stage1_demo.md) | The balance session (`./demo.sh`, `pendctl --mode balance`): running it, its messages, the design, the rig sessions of 2026-10-12 and 10-14, the ~28 Hz analysis, "After a mechanical rebuild" | Current up to 2026-10-26. The 60 s catch on the new hardware (2026-11-02) is in the `CLAUDE.md` Status log, not here. |
| [`swingup_notes.md`](swingup_notes.md) | Swing-up by energy pumping: theory, feasibility study, implementation, the first rig test (2026-10-14) | The design is current. The 2026-11-03 tuning and its rig results are in the `CLAUDE.md` Status log, not here. |
| [`remote.md`](remote.md) | Remote mode (controller on the PC): how it works, remote swing-up, choosing the delay, procedure, files, open items | Current |
| [`stage3_cascade.md`](stage3_cascade.md) | Stage 3 cascade (outer cart loop) and Stage 3b (cart integral, hand-over speed gate) | The design is current. The Stage 3b rig run (`2026-11-04T10-16-08`) is in the `CLAUDE.md` Status log, not here. |
| [`stage0_9_actuator.md`](stage0_9_actuator.md) | Stage 0.9, the actuator test: what it decides, prerequisites, tooling, rig blocks, pass criteria | Plan. The tooling was built on the PC on 2026-11-19 (commit `94de94c`). Nothing has run on the Pi or the rig. |
| [`neuro_sim.md`](neuro_sim.md) | Evolved neural controllers (Pasemann 1998, ENS³) in simulation, and the headroom study (§6) | Current. Simulation only. |
| [`offset_estimator.md`](offset_estimator.md) | The upright-offset estimate carried between catches: method, log replay, simulation, and a port into the Pi runtime (off by default) | In progress since 2026-11-19. Simulation and PC tests only. |
| [`stage1_drift.md`](stage1_drift.md) | Why the cart drifts under angle-only control; engage on release with pre-load (built, off) | Record, 2026-10-09. Old plant and 20 Hz filter, but the reasoning still holds. |

## Hardware

| Document | What it covers | Status |
|---|---|---|
| [`pcb_bringup.md`](pcb_bringup.md) | Carrier PCB review findings, bring-up stages A–E, the powered test points | Current. Stages D and E were never logged. |
| [`parts_reference.md`](parts_reference.md) | Every part in the thesis, with datasheets and CAD (the CAD is in `cad/`, outside the repo) | Current (2026-11-11) |
| [`order_links.md`](order_links.md) | Where to buy the parts | Current (links checked 2026-11-11) |
| [`thesis_notes.md`](thesis_notes.md) | What the 2025/26 bachelor thesis says about the rig: priors, not measurements | Record |

## Derivations

| Document | What it covers | Status |
|---|---|---|
| [`eom_derivation.md`](eom_derivation.md) | The pendulum's equation of motion, derived three ways | Current |
| [`model_derivation.md`](model_derivation.md) | A check of the brief's model and gain formulas, with three refinements | Current |

## Stage 0 and early Stage 1 records

What was measured or decided on the day. The Status column notes values that were replaced later.

| Document | What it covers | Status |
|---|---|---|
| [`stage0_discovery.md`](stage0_discovery.md) | 0.1: inventory of the Pi and the software as found | Record, 2026-10-08. It describes the original Pi, replaced on 2026-10-23. |
| [`gateA_report.md`](gateA_report.md) | Gate A: audit of the thesis code (units, signs, limit switches, start-up behaviour) | Record, 2026-10-08. Still the reference for the original `main`. |
| [`stage0_3_safe_bringup.md`](stage0_3_safe_bringup.md) | 0.3: safe bring-up mode, signal handlers, the tests on a simulated GPIO chip | Record, 2026-10-08. Still how to run those tests. |
| [`stage0_4_instrumentation.md`](stage0_4_instrumentation.md) | 0.4: ring-buffer logging, run folders, config | Record, 2026-10-08 |
| [`stage0_5_timing.md`](stage0_5_timing.md) | 0.5: real-time timing of the loop and the step edges | Record, 2026-10-09 |
| [`stage0_6_7_10_results.md`](stage0_6_7_10_results.md) | 0.6, 0.7, 0.10: encoder integrity, upright reference, pendulum identification | Record, 2026-10-09. The plant numbers were replaced after the pivot rework (2026-11-02). |
| [`stage0_7_0_10_fits.md`](stage0_7_0_10_fits.md) | The swing fits behind 0.7 and 0.10, and why the upright reference is re-found at every start | Record, 2026-10-08. The method is still in use. |
| [`stage0_11_estimators.md`](stage0_11_estimators.md) | 0.11: the velocity estimator (backward difference and a 2nd-order low-pass) | Record, 2026-10-09. The cut-off is now 14 Hz, not 20 (`stage1_demo.md`). |
| [`stage0_12_sim.md`](stage0_12_sim.md) | 0.12: the Python simulation framework, and its validation on rig data | Record, 2026-10-12. The header ("nothing validated yet") predates the validation section at the end. |
| [`stage1_runtime.md`](stage1_runtime.md) | Stage 1 runtime layers, simulated hardware, closed-loop tests | Record, 2026-10-09. Placeholder plant. |
| [`stage1_damp.md`](stage1_damp.md) | Stage 1 steps 1–2 (shadow mode, hanging damping) and the 0.10 pulse test | Record, 2026-10-09 |
| [`gateB_report.md`](gateB_report.md) | Gate B: measured parameters, proposed design, margins, catch region, the ζ appendix | Draft, 2026-10-12. It waits for 0.9, and uses the original pivot and the 20 Hz filter. |

## Overviews (snapshots)

| Document | What it covers | Status |
|---|---|---|
| [`explainer.md`](explainer.md) | The whole project explained: theory, implementation, every step | Snapshot, 2026-10-12. A box at the top lists the later changes. |
| [`pendel_guide.html`](pendel_guide.html) | An HTML version of `explainer.md` with the figures embedded; open it in a browser | Snapshot, 2026-10-12, without the box of later changes. No script builds it, so it does not follow edits to `explainer.md`. |
| [`img/`](img/) | Plots used by the notes above | — |

## Known stale statements

These read as current, but are not:
- **`CLAUDE.md`, System facts:** the GPIO pin list "may change with the new PCB, so confirm with Simon". The new
  PCB kept the same pin map (`CLAUDE.md` Status: PCB soldered 2026-10-21).
- **`CLAUDE.md`, Conventions:** "Step mode is intermittent". That was a loose MS1/MS2 wire on the old board. The new
  PCB has MS1/MS2 on the board, and the track span measured on 2026-11-02 (5938 µsteps) matches 1/8 step.
- **`CLAUDE.md`, the Status heading:** it points to `session_log.md` for details, but that log stops on 2026-10-09.
- **`session_log.md`:** the Pi host key belongs to the original Pi.
- **`gateB_report.md`:** it says the new PCB "arrives Wed 2026-10-14". The PCB has been in the rig since 2026-11-02.
- **`explainer.md` and `pendel_guide.html`:** plant, filter and gains as of 2026-10-12.
- **`stage0_6_7_10_results.md`, `stage0_11_estimators.md`, `stage1_drift.md`, `gateB_report.md`:** the plant
  ω₀ 4.931 rad/s, b/J 0.024 1/s, Coulomb 0.0117 rad/s², and the 20 Hz filter. Today's values are 4.97 / 0.054 /
  0.168 and 14 Hz.
- **`01` and `02`:** 16 notes are marked "TODO / unverified". Each one says what is not known.

## Not in this repository

The run logs (`logs/`, ignored by git), the thesis PDF, `cad/`, `pendulum-lab.html`, the ML course and the ENS³
explainer. Where each one is: `../HANDOVER.md` §5.
