# Stage 0.12: simulation and loop analysis (framework; PLACEHOLDER parameters)

Written while the Pi was unreachable (2026-10-08). **Nothing here is validated yet.** The brief
requires validating against the 0.9 (actuator) and 0.10 (pendulum) data before designing with it.
The numbers below only show that the tools work and agree with the derivation
(`model_derivation.md`).

## Modules (`py/pendel/`)
| Module | Contents |
|---|---|
| `control.py` | **Reference controller step function** (pure: params + state + measurement + dt → a_cmd + state). Derivative of the measurement (backward difference + 1st-order low-pass `vel_fc`), conditional-integration anti-windup, integrator clamp `i_max`, enable delay `i_delay`, saturation at `a_max` with authority taper toward ±`v_max`, `theta_ref` trim. `DelayLine` (N samples, Stage 2a). `design_gains()` = pole placement. The C++ core will mirror this (golden test, Stage 1). |
| `plant.py` | Nonlinear pendulum (viscous + optional Coulomb friction), Axis ACCELERATE cap A·(1 − \|v\|/V), encoder quantisation with phase and reference error δ. |
| `sim.py` | Closed loop: controller at Ts, RK4 plant at Ts/20, compute delay inside the period, N-sample delay line, abort on \|θ\| or \|x\|. |
| `linear.py` | Exact ZOH discretisation with fractional + whole-sample delay, the same controller as state space. Gain range [k_lo, k_hi] and delay margin from closed-loop eigenvalues; phase margins at \|L\| = 1; Nyquist encirclement count vs unstable open-loop poles; RMS acceleration from white quantisation noise (discrete Lyapunov). |
| `py/design_preview.py` | PD vs PID table, catch simulations, `docs/img/design_preview.png`. |

## Preview with placeholder parameters
Plant: ω₀ = 2π rad/s (1.0 s hanging period, L_eq = 0.25 m), b/J = 0.1 1/s. Ts = 2 ms, delay 0.5 ms.
Design: ω_c = 2ω₀, ζ = 0.7, PID real pole p = 0.5ω₀. Limits a_max = 5 m/s², v_max = 0.8 m/s,
derivative low-pass 30 Hz.

| | Kp (m/s²/rad) | Ki | Kd (m/s²/(rad/s)) | stable gain range | delay margin | phase margin | Nyquist CCW / OL unstable | RMS a from quantisation |
|---|---|---|---|---|---|---|---|---|
| PD | 49.1 | 0 | 4.35 | 0.20 – 33.7 × | 48 ms | 53° @ 3.0 Hz | 1 / 1 | 0.52 m/s² |
| PID | 62.8 | 123.3 | 5.13 | 0.25 – 28.2 × | 41 ms | 52° @ 3.5 Hz | 1 / 1 | 0.62 m/s² |

Consistency checks: PD's lower gain limit 0.20 = g/Kp, i.e. Kp > g (§5 of the derivation); one
counter-clockwise encirclement for one unstable open-loop pole.

Catches (cart at rest, run ends at |x| = 0.3 m or 6 s):

| | from 3° | from 8° |
|---|---|---|
| PD | held 6 s, x = 0.23 m | x-limit at 1.6 s, residual v = 0.17 m/s |
| PID | held 6 s, x = −0.08 m | x-limit at 3.3 s |

## Observations to carry into the design (to confirm with measured parameters)
1. **Quantisation limit cycle.** After the catch, θ dithers by one count and a_cmd chatters by
   ±1–2 m/s² (≈ 0.5–0.6 m/s² RMS from the noise model). Kd and the derivative low-pass set this
   trade-off. The 0.11 estimator choice matters here, and so does step loss under chatter (0.9).
2. **PD keeps the catch's residual velocity** (§7 of the derivation): 0.17 m/s after an 8° catch,
   which uses up the track in ~1.6 s.
3. **Quantisation acts as a reference offset.** With an integrator, the limit cycle settles around a
   count *edge*, not the count centre, so the effective reference is off by a fraction of a count.
   That gives constant acceleration g·δ and parabolic drift (PID 3° curve). This is the §3 point that
   sub-count reference calibration (0.7) and trim buy run time, and that only the Stage 3 cart loop
   removes the drift.

## Still to do in 0.12
- Validate against 0.9 (realised vs commanded cart motion, effective delay, Axis cap) and 0.10 (free
  swings, the hanging pulse response).
- Add the chosen velocity estimator (0.11), and the original decoder's one-edge lag if it stays.
- Model the Axis step-rate quantisation and lost steps if 0.9 shows them.
- Supervisor (engage/abort) for the catch-region map over (θ₀, θ̇₀).
