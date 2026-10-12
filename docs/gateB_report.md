# Gate B report (DRAFT, 2026-10-12)

**Status:** everything the brief asks for at Gate B, except the actuator capability (0.9), which needs
the new PCB (arrives Wed 2026-10-14; it also fixes the intermittent microstep mode). Values that depend
on 0.9 are marked **PENDING 0.9**; the analysis is set up so that they only have to be filled in.
Simon decides at the gate whether Stage 1 catches may start.

## 1. Measured parameters

| Parameter | Value | Uncertainty / evidence | Source |
|---|---|---|---|
| Encoder resolution | 2400 counts/rev | exact: 5 turns = 12 000.5 counts, return −1 count | 0.6 (`docs/stage0_6_7_10_results.md`) |
| Encoder integrity | no lost or wrong-type edges in 0.7 M events | 2 startup/race bugs found and fixed (§3) | 0.6 |
| θ sign (`calib.theta_sign`) | +1 (θ > 0: top toward +x) | confirmed three ways: hand test, shadow upright tilt, pulse/damping | 0.6, Stage 1 |
| x sign (`calib.x_sign`) | +1 (+x = increasing steps = toward the LEFT switch) | switch runs; combined sign confirmed by the pulse fit (k > 0) | 0.8 |
| ω₀ (hanging, small angle) | 4.931 rad/s | ± 0.01 (8 decay segments 4.914–4.940) | 0.10 |
| L_eq = g/ω₀² | 0.4035 m | ± 0.002 | 0.10 |
| Viscous b/J | 0.024 1/s | ~± 50 %, irrelevant (design 2ζω_c ≈ 14 1/s) | 0.10 |
| Coulomb γ | 0.0117 rad/s² (stiction band ± 0.18 counts) | ~± 30 % | 0.10 |
| Input gain ω₀²/g | 2.48 1/m | pulse fit 93 %, damping decay 103 % → ± 7 % | 0.10 (`docs/stage1_damp.md`) |
| Upright reference c_bottom | fitted at every start (swing 2–20°) | ± 0.13 counts within a run (5–14° swings); two starts with 15–20° swings differed by 0.28 counts | 0.7, Stage 1 |
| Cart scale | 13 333 steps/m (1/8 step) | ruler ± ~5 %; **step mode intermittent** (half-step seen: 3072/m) | 0.8; PCB fixes |
| STEP toggles per step | 2 | 1.9985 measured | 0.8 |
| Track | switches ~0.5 m apart; soft limits ± 0.15 m about the centre | rough (tape) | 0.8 |
| Control loop timing (SCHED_FIFO 80, core 1) | lateness p99 6 µs idle / 35 µs load, max 0.33 ms | 10 min runs, no overruns | 0.5 |
| Step-edge timing (FIFO 90, core 3) | p99 ≤ 12 µs, rare 0.1–0.3 ms bursts | scope check passed | 0.5 |
| Encoder read latency | p50 33 µs, p99 50 µs | kernel timestamps share CLOCK_MONOTONIC | 0.6/0.11 |
| Velocity estimator | BD + 2nd-order Butterworth 20 Hz | chosen on real data (`docs/stage0_11_estimators.md`) | 0.11 |
| Simulation | validated: pulse peak 100 %, damping decay 1.022 vs 1.019 1/s, φ RMS 0.12° | `docs/stage0_12_sim.md` | 0.12 |
| a_max, v_max (reliable), a_brake, effective actuator delay, step-loss limits | **PENDING 0.9** | | 0.9 |

## 2. Plots
- Identification and integrity: `docs/img/` from `py/fit_swing.py`, `py/encoder_report.py` (0.6/0.7/0.10).
- Pulse and damping on the rig: `logs/2026-10-09T13-48-40_move/pulse.png`, `logs/2026-10-09T13-50-13_damp/damp.png`.
- Simulation vs rig: `docs/img/sim_validation.png`.
- Estimators on real data: `docs/img/estimators_real.png`.
- Design: `docs/img/gateb_catch_region.png` (catch regions), `docs/img/gateb_nyquist.png` (Nyquist, proposed design).

## 3. Issues found and fixed
| Issue | Evidence | Fix |
|---|---|---|
| Original code (Gate A items 1–13, N1–N9): uninitialised flags, non-atomic shared state, limit switches never stop motion, ×2 unit mix, old decoder holds the newest edge, normal-priority step thread | `docs/gateA_report.md` | Bring-up mode and signal handling in `main`; new `pendctl` runtime: atomics, SCHED_FIFO + pinning, own supervisor and soft limits (Axis raw position), units at the boundary, new edge reader |
| A failed build ran a stale binary that homed the cart (2026-10-08) | incident | rm before build, `&&`-gated runs, no real-chip run without a go |
| Encoder reader lost edges at startup | recording 09-20-36 | claim lines after memory/RT setup |
| False encoder fault at reader start | runs A/B | ignore same-type events older than the start-level read |
| **"Encoder fault" while lifting fast** (edge arriving during the drain counted as overflow) | shadow 14-38-54 | overflow only when edges are lost or the buffer is full |
| Calibration never timed out (clock wrap before the first tick) | mock run on the Pi | fixed, mock timeout added |
| Shadow armed on the estimator's first (zero) sample | mock run | first arm after 0.2 s |
| Start-up calibration rejected natural 10–20° swings | shadow 13-24-19 | amplitude-corrected frequency (validated on the free decay), accept 2–20° |
| Steps/m 4× off and intermittent (half-step vs 1/8) | 307 steps → 23 mm vs 1526 steps ≈ 0.5 m | config 13 333; distance check before motion; **new PCB** |
| D-term spikes ± 2.4 m/s² per count step | shadow 13-36-33 | BD+LP2 20 Hz: spikes 2.4× lower, closed-loop noise halved |
| Drift: integrator enable delay cost 42 mm/s | `docs/stage1_drift.md` | `ctrl.i_delay = 0` |
| Engage on release: hand adjustments looked like releases | shadow 14-38-54 (both engages false) | **open**: feature off, edge log for labelled tuning |

## 4. Proposed Stage 1 design
| Item | Proposal | Reason |
|---|---|---|
| Ts | 2 ms (500 Hz) | timing margin large (p99 35 µs under load); ω_c·Ts = 0.02 |
| Controller | PID, pole placement (s² + 2ζω_c s + ω_c²)(s + p), **ω_c = 2ω₀ = 9.9 rad/s, ζ = 0.7, p = 0.5ω₀** | table below |
| Gains | **Kp 62.8 (m/s²)/rad, Ki 96.7 (m/s²)/(rad·s), Kd 6.56 (m/s²)/(rad/s)** | Kp = 6.4 g > g |
| Velocity | BD + LP2 20 Hz on counts (`ctrl.estimator = bdlp2`) | 0.11 |
| Integrator | runs from engage (`i_delay = 0`), conditional integration in saturation, clamp \|Ki·I\| ≤ 1 m/s² | drift budget |
| a_max, v_max | **PENDING 0.9**: 50–70 % of the reliable maxima (placeholders 3 m/s², 0.5 m/s) | brief |
| Supervisor | engage \|θ\| < 4°, \|ω\| < 0.5 rad/s for 50 ms; abort \|θ\| > 22°, soft limits ± 0.15 m with braking distance at a_brake (**PENDING 0.9**, placeholder 1 m/s²), switch, encoder fault, Axis not in ACCELERATE, 3 overruns, 60 s | brief §6 |
| Engage on release + momentum pre-load | built, sim-tested, **off** | false engages from hand adjustments |
| Start-up calibration | free swing 2–20° (5–10° recommended), fit at every start | no encoder index |

### Predicted margins (linear analysis with the LP2 filter, exact ZOH, 0.5 ms compute delay; `py/gateb_design.py`)
| Design | Kp / Ki / Kd | Gain margin [k_lo, k_hi] | Phase margin | Delay margin | RMS a from quantisation | 3° catch (nonlinear sim): RMS a / p99 / RMS θ |
|---|---|---|---|---|---|---|
| PID ω_c = 1.5ω₀ | 42.2 / 54.4 / 5.16 | [0.33, 9.0] | 49° @ 13.6 rad/s | 63 ms | 0.15 m/s² | 0.36 / 0.87 / 0.069° |
| **PID ω_c = 2ω₀** | **62.8 / 96.7 / 6.56** | **[0.25, 6.9]** | **50° @ 18.0 rad/s** | **48 ms** | **0.19 m/s²** | **0.47 / 1.12 / 0.057°** |
| PID ω_c = 3ω₀ | 118.7 / 217.7 / 9.34 | [0.16, 4.7] | 47° @ 27.1 rad/s | 30 ms | 0.30 m/s² | 0.70 / 1.66 / 0.042° |
| PD ω_c = 2ω₀ | 49.1 / 0 / 5.56 | [0.20, 8.3] | 51° @ 15.5 rad/s | 57 ms | 0.16 m/s² | 0.43 / 0.94 / 0.048° |

All: Nyquist one counter-clockwise encirclement of −1 for one unstable open-loop pole (stable). The linear
LP2 model matches a direct time simulation to 1e-16.

**Trade-off:** ω_c = 3ω₀ catches no more (below) but halves the delay margin and raises the command noise
by 50 %; 1.5ω₀ is quieter with more delay margin but holds the angle less tightly. 2ω₀ keeps 48 ms of delay
margin for Stage 2 latency, a ×4 gain margin downward and ×7 upward (covers the ± 7 % input-gain uncertainty
and the ± 5 % steps/m uncertainty many times over). If 0.9 shows step loss at the noise level, fall back to
1.5ω₀.

### Catch region (`docs/img/gateb_catch_region.png`)
With the placeholder limits (3 m/s², 0.5 m/s) every design catches the same region: **θ₀ + θ̇₀/ω₀ ≲ 10°**
(9.5° at rest, 6.5° moving away at 0.25 rad/s, 3.5° at 0.5 rad/s). The boundary slopes at exactly 1/ω₀:
the pendulum's unstable mode z = θ + θ̇/ω₀ must stay within what the actuator can still turn around. **The
catch region is set by the actuator limits, not the gains**, so it is recomputed with the 0.9 values.

### Drift (`docs/stage1_drift.md`)
- Residual cart velocity after a catch: ≈ 0 for PID with a clean release (sim), 68 mm/s for PD (brief §3
  confirmed); each 50 ms of hand on the rod after engage adds ~90 mm/s.
- Reference offset δ: PID a_d = gδ; PD a_d = gδ·Kp/(Kp − g):

| δ | PID: a_d, time to 0.15 m | PD: a_d, time to 0.15 m |
|---|---|---|
| 0.25 count (0.65 mrad) | 6.4 mm/s², 6.8 s | 8.0 mm/s², 6.1 s |
| 0.5 count (1.31 mrad) | 12.8 mm/s², 4.8 s | 16.1 mm/s², 4.3 s |
| 1 count (2.62 mrad) | 25.7 mm/s², 3.4 s | 32.1 mm/s², 3.1 s |

The quantisation limit cycle alone gives up to ± 6 mm/s² (where upright falls in the count grid), and the
trim reduces it only within a branch (discontinuous). Expected balanced runs: ~4–7 s before a soft limit.

## 5. Proposed acceptance thresholds (Stage 1 experiments)
| Metric | Threshold | Prediction |
|---|---|---|
| Catch success, ~2° release, each side | ≥ 9/10 | inside the region with large margin |
| Catch success, ~5° release, each side | ≥ 8/10 | inside (z < 10°) |
| Catch success, ~8° release, each side | ≥ 5/10, or recomputed after 0.9 | near the edge of the region |
| Settling | \|θ\| < 3° within 1 s of release (brief) | 3° catch: \|θ\| < 1° after ~0.5 s |
| RMS θ after 1 s | ≤ 0.3° | 0.06° (sim) |
| Saturated fraction after 1 s | ≤ 5 % | 0 % (sim) |
| Balanced time before a soft limit or timeout | median ≥ 3 s with the untrimmed reference; improves with trim | 4–7 s |
| Faults | 0 encoder faults, 0 overrun aborts, no step loss (post-run switch audit within ± 10 steps) | |
| Trim | \|a_d\| ≤ 6 mm/s² (≈ 0.25 count) within 3 iterations | discontinuous near a branch jump |
| PD vs PID | PD residual velocity matches g∫θ of the catch within 50 % | 68 mm/s for a 3° catch |

## 6. Before the first catch
1. **New PCB (Wed):** pin map check (`pins.*`), microstep mode check with a measured move (expect 1/8, 13 333/m).
2. **Motor-only E-stop with a bulk capacitor at VMOT** (Gate A N8: the 1 A fuse and the remote bulk cap) before
   the aggressive 0.9 runs, unless the PCB provides it.
3. **0.9 actuator capability** with the pendulum mounted: moves at increasing accel/speed with a step-loss
   audit after each, then 500 Hz acceleration streams (steps, chirp): reliable maxima, effective delay,
   Axis cap. Set a_max, v_max to 50–70 % of the reliable maxima, a_brake, and re-run `py/gateb_design.py`.
4. Fill in the PENDING values here, re-run the catch region, finalise the thresholds.
5. Operator procedure: calibration swing 5–10°; lift to within ~2° of upright, hold still, let go the moment
   it engages (hand-hold drift), hands ready to catch.

## 7. Decisions for Simon at the gate
- Approve the design (PID ω_c = 2ω₀, LP2 20 Hz, Ts 2 ms) or pick another row.
- Approve the acceptance thresholds.
- Engage logic: keep "still near upright" (release promptly), or a labelled shadow session to tune engage-on-release.
- After 0.9: approve a_max, v_max, a_brake and the first-catch limits (brief: reduced a_max and v_max first).
