# Stage 1: cart drift under angle-only control (2026-10-09)

Scope (brief §6): no cart-position feedback in Stage 1; the drift is analysed and minimised within
angle-only control, not "fixed" with a cart loop. Tools: `py/drift_analysis.py` (Python sim, identified
plant ω₀ 4.931, b/J 0.024, Coulomb 0.0117, config gains Kp 62.8 / Ki 96.8 / Kd 6.56, BD+LP2 20 Hz),
`tests/unit/test_balance.cpp` (C++ runtime on SimHw, rig config).

## The identity behind all of it
Integrating the pendulum equation θ̈ = ω₀²θ − (b/J)θ̇ − (ω₀²/g)a (no external torque) gives

  v(t) − v(0) = g∫θ dt − L_eq(θ̇(t) − θ̇(0)) − L_eq(b/J)(θ(t) − θ(0)),  L_eq = g/ω₀² = 0.40 m.

After the transient: **v_∞ = v(engage) + L_eq·θ̇(engage) + g·∫θ dt**. The PID equilibrium needs θ = 0
and Ki·I = 0, so the integrator state I = ∫θ_meas dt must return to zero. Consequences:
- Every part of ∫θ the integrator does **not** see stays as cart velocity (enable delay, holds).
- Momentum the pendulum already has at engage stays (L_eq·θ̇), unless the integrator starts pre-loaded.
- A constant offset δ of the true balance angle gives a constant acceleration g·δ.
- PD (Ki = 0) keeps g∫θ of the transient: brief §3 confirmed (68 mm/s vs ~0 for PID, 3° catch).
- An external torque after engage (a hand on the rod) breaks the identity: its impulse stays as cart momentum.

## Budget (simulation, 3° catch unless noted; v0 = residual velocity, a_d = drift acceleration)
| Source | Effect | Fix within angle-only control |
|---|---|---|
| **Hand still on the rod after ENGAGED** (rigid hand, C++) | 50 ms hold at 2.3°: **~90 mm/s**; ≥ 100 ms: soft limit within 2 s. ≈ (Kp − g)·θ·t_hold | **Engage on release** (below): the cart never moves while held |
| Integrator enable delay `i_delay` 0.1 s | **+42 mm/s** (0.37 m in 8 s) | `ctrl.i_delay = 0` → ~0 mm/s |
| Integrator hold in saturation, clamp | ~0 for ≤ 2° releases; 27 mm/s at 3° (a_max 3 saturates) | release within ~2° of upright |
| Pendulum moving at engage | **v0 = L_eq·θ̇** exactly (40 mm/s per 0.1 rad/s; the old 0.5 rad/s gate allowed 200) | **pre-load** I(0) = θ̇/ω₀² |
| Quantisation limit cycle | a_d up to **±6 mm/s²**, set by where upright falls in the count grid (encoder phase) | partly by trim; see below |
| Calibration error of c_bottom | g·δ: ±0.3 counts (today's fits at 15–20°) → ±7.7 mm/s² | smaller calibration swings; fixed fractional part (open) |

## Engage on release + momentum pre-load (implemented, `sup.engage_on_release`, `ctrl.preload`)
- **Release detection** (`rt::BalanceRuntime`): the two newest encoder edges step the same way **away
  from upright**, the first after ≥ 0.15 s without edges (the still hold), the newest ≤ 0.1 s old. A
  released pendulum near upright always falls away from it; a held one produces no such pair.
  The supervisor (ARMED) then engages if |θ| < θ_engage, instead of after 25 still samples.
- **Two-edge free-motion velocity:** between the two edges the pendulum is free, θ(t) = A e^{ω₀t} +
  B e^{−ω₀t}; the two edges give exact positions (count boundaries) at exact kernel timestamps, so A, B
  and θ̇(engage) follow without filtering, lag or any assumption about how the hand let go
  (simulated error 0.4 %; with a 0.05 rad/s push at release also exact).
- **Pre-load:** I(0) = θ̇(engage)/ω₀² (cart at rest while ARMED), which makes v_∞ = 0 in the identity.
- Python, releases from rest at 1/2/3° (+ with a push): residual velocity −5, −5, +27 (saturation), +4,
  −3 mm/s, against 19, 31, 70, 37, 33 without the pre-load. Engage 92–144 ms after release.
- C++ runtime, rig config, hand holds 1 s at 1°/2° then lets go: no engage and no cart motion while
  held; engage 156/92 ms after release; two-edge θ̇ within 0.01 rad/s; residual **−3.6 / +7.3 mm/s**.
- Operator procedure changes: lift to within ~2° of upright, **hold still**, **let go cleanly**, keep the
  hand ready to catch. (Shadow mode logs `released` and `omega_release` to check this on the rig first.)

## Quantisation offset and the trim
The ±1-count limit cycle dithers across the count boundary nearest to upright, so the true mean angle
sits near that boundary, not at upright. Phase sweep (where upright falls in the count grid): a_d from
−6.2 to +5.3 mm/s², ≈ 50–65 % of g × (distance to the nearest boundary). Trim sweep at phase 0.37:
a_d −11.6 → −3.4 mm/s² for trims −1.5 … +0.3 mrad, then a **jump to +10.8** at +0.6 mrad (the limit cycle
moves to the other boundary). The brief's iterative trim therefore works within a branch but is
discontinuous near the jump: expect convergence to a few mm/s², not zero. Getting below that needs
sub-count information (dither or a model-based observer): a later topic.

## Expected run length after the fix
v0 ≈ ±5 mm/s and a_d ≈ ±(quantisation + calibration) ≈ 5–10 mm/s² → |x| = 0.15 m after ~5–7 s, against
~1–2 s with engage-while-held. The remaining drift is the reference offset, which is what the brief's
trim experiment addresses (fit x = x0 + v0 t + ½ a_d t², δ ≈ a_d/g).

## Open
- Calibration accuracy: today's two start-up fits (15–20° swings) differ by 0.28 counts in the fractional
  part, which is physical and should be constant. Options: accept only ≤ 10° swings (validated ≤ 0.07
  counts), or measure the fractional part once precisely and let the start-up fit pick only the integer.
- Saturation hold at larger catches (3°): revisit with a_max from 0.9.
- Release detection on the real rig: hand tremor while holding; check with shadow mode first.
