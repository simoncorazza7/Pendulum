# Model derivation and sign check (brief §3)

Result first: **every equation in §3 checks out**. Three refinements:
1. In the ideal model the loop has only the lower gain margin (Kp > g). The upper margin comes from
   sampling, delay and actuator dynamics.
2. The final cart velocity under PID also contains (g/ω₀²)·θ̇₀ from the release.
3. Under **PD** a reference offset δ gives a steady acceleration g·δ·Kp/(Kp − g), not g·δ. Exactly g·δ
   holds only with an integrator.

Numerical checks of all of this: `py/model_check.py`.

## 1. Coordinates and conventions
- x: cart position (m), **+x = direction of increasing step count** (physical direction still to be
  established in 0.8).
- θ: pendulum angle from upright (rad), **positive when the top leans toward +x**.
- Pivot on the cart at (x, 0), y up. Centre of mass at r = (x + l_c sin θ, l_c cos θ).
- m mass, l_c pivot→COM distance, J inertia about the pivot (J = J_c + m l_c²), b viscous pivot friction.
- Input: cart acceleration a = ẍ. The stepper imposes x(t) kinematically (while it doesn't lose
  steps), so cart mass, belt and motor torque drop out of the pendulum dynamics.

## 2. Equation of motion
Step by step, three ways, with the force-input comparison: `docs/eom_derivation.md`.

Lagrangian with x(t) prescribed. The COM velocity is (ẋ + l_c θ̇ cos θ, −l_c θ̇ sin θ), so

    T = ½ m ẋ² + m l_c ẋ θ̇ cos θ + ½ J θ̇²        V = m g l_c cos θ

    d/dt ∂L/∂θ̇ = J θ̈ + m l_c ẍ cos θ − m l_c ẋ θ̇ sin θ
    ∂L/∂θ      = −m l_c ẋ θ̇ sin θ + m g l_c sin θ

The Euler–Lagrange equation with the friction torque −b θ̇ gives

    J θ̈ = m g l_c sin θ − m l_c a cos θ − b θ̇                      (✓ matches §3)

Physical check: leaning toward +x (θ > 0), gravity increases θ. Accelerating the cart toward +x (a > 0)
decreases θ̈, because the pivot moves under the COM. So a = +K·θ with K > 0 is the restoring direction:
**positive gains move the cart under the pendulum** ✓.

## 3. Hanging pendulum (identification, 0.10)
Put θ = π + φ, so sin θ ≈ −φ and cos θ ≈ −1:

    φ̈ + (b/J) φ̇ + ω₀² φ = +(ω₀²/g) a,      ω₀² = m g l_c / J

- ω₀ is the small-angle frequency of the hanging pendulum. The period gives ω₀ and the decay gives
  b/J = 2 ζ_hang ω₀ (valid while ζ_hang ≪ 1).
- **The input gain has the opposite sign at the bottom** (+ω₀²/g vs −ω₀²/g at the top). The hanging
  active-damping test (Stage 1, step 2) must use a = −K_d,hang·φ̇ in these φ coordinates, with the
  same sign convention for a.
- Equivalent length L_eq = g/ω₀² = J/(m l_c). Example: a 1.0 s hanging period gives ω₀ = 6.28 rad/s and
  L_eq = 0.248 m. A 0.5 m point-mass pendulum (the thesis's reference) gives ω₀ = 4.43 rad/s.

## 4. Upright linearisation and plant
With m l_c / J = ω₀²/g:

    θ̈ = ω₀² θ − (b/J) θ̇ − (ω₀²/g) a                                 (✓)
    G(s) = Θ/A = −(ω₀²/g) / (s² + (b/J) s − ω₀²)                     (✓)

Poles: s = −b/(2J) ± √(ω₀² + (b/2J)²), i.e. one in the right half-plane at ≈ +ω₀. That pole sets
the time scale of the instability: an uncorrected tilt grows by e every 1/ω₀ (≈ 0.16 s for a 1 s
hanging period).

## 5. PID loop and characteristic polynomial
Controller a = Kp θ + Ki ∫θ + Kd θ̇ (on the measurement, sign as in §2). With I = ∫θ, k = ω₀²/g,
β = b/J:

    s³ + (β + k Kd) s² + (k Kp − ω₀²) s + k Ki = 0
    ⇔  s³ + (b/J + ω₀² Kd/g) s² + ω₀² (Kp/g − 1) s + ω₀² Ki/g = 0     (✓)

Routh–Hurwitz (a₂, a₁, a₀ > 0 and a₂ a₁ > a₀):
- a₁ > 0 ⇔ **Kp > g** (m/s² per rad). Below it the cart can't accelerate fast enough to get under the
  pendulum. This is the lower gain margin.
- a₀ > 0 ⇔ Ki > 0 (if used). a₂ > 0 ⇔ Kd > −β g/ω₀² (the D term may even be slightly negative in theory).
- a₂ a₁ > a₀ bounds Ki from above for given Kp, Kd.
- **There is no upper bound on Kp in this ideal model.** The upper gain margin and the delay margin
  come from the 2 ms sampling, the computation/actuation delay, the velocity filter and the Axis's
  own dynamics. They have to be evaluated on the discrete model with delay (0.12).

## 6. Pole placement
Target (s² + 2ζω_c s + ω_c²)(s + p) = s³ + (2ζω_c + p) s² + (ω_c² + 2ζω_c p) s + ω_c² p. Matching
coefficients:

    Kp = g (1 + (ω_c² + 2ζ ω_c p)/ω₀²)                                (✓)
    Kd = (2ζ ω_c + p − b/J) g/ω₀²                                     (✓)
    Ki = g ω_c² p / ω₀²                                               (✓, p = 0 → PD)

Units: Kp in (m/s²)/rad, Kd in (m/s²)/(rad/s), Ki in (m/s²)/(rad·s).
Scale: with ω_c = 2ω₀, ζ = 0.7, p = 0, Kp = 5g ≈ 49 m/s² per rad, i.e. 0.86 m/s² per degree, or
0.13 m/s² per encoder count. One count of quantisation therefore already commands ~0.13 m/s².

## 7. The integrator as hidden cart-velocity feedback
Define C = (g/ω₀²) θ̇ + ẋ − g ∫θ dt. In the frictionless linear model

    dC/dt = (g/ω₀²) θ̈ + ẍ − g θ = (g/ω₀²)(ω₀² θ − (ω₀²/g) a) + a − g θ = 0

for **any** input a, so C is conserved whatever the controller does (✓). Integrating from engagement
(t = 0, ∫θ = 0) to the settled state (θ = θ̇ = 0):

    ẋ∞ = ẋ₀ + (g/ω₀²) θ̇₀ + g · ∫₀^∞ θ dt

- **PID**: the integral state is part of the closed-loop state and goes to zero (Ki ∫θ → 0, hence
  ∫θ → 0). So ẋ∞ = ẋ₀ + (g/ω₀²) θ̇₀: the cart returns to its engagement velocity **plus** a term from
  the angular velocity at release. With θ̇₀ = 0.1 rad/s and L_eq = 0.25 m that's 2.5 cm/s. "Gentle
  release" must mean small θ̇₀ as well as small θ₀ (refinement 2).
- **PD**: ∫θ settles at a nonzero value, so the cart keeps the residual velocity g·∫θ dt (✓).
- With pivot friction, dC/dt = −(g/ω₀²)(b/J) θ̇, so C changes by (g/ω₀²)(b/J)·θ₀ over the catch. That
  is a small correction, proportional to the release angle.

## 8. Reference offset → steady acceleration
Let the controller's upright reference be off by δ, so it regulates e = θ − δ. A steady state with
constant cart acceleration needs θ̈ = 0, i.e. **a = g θ** (the pendulum leans so that gravity balances
the pseudo-force).
- **PID**: the integrator forces e → 0, so θ = δ and a = g δ (✓).
- **PD**: a = Kp(θ − δ) and a = g θ give θ = Kp δ/(Kp − g) and **a = g δ · Kp/(Kp − g)**. With Kp = 5g
  that's 1.25 g δ; near the lower margin it blows up (refinement 3).

Numbers (2400 counts/rev): 1 count = 2.618 mrad = 0.15°, so g δ = 0.0257 m/s² per count, and from
rest x = ½ a t² = 0.32 m after 5 s (✓ §3: "≈ 0.3 m in 5 s"). A 0.6 m track gives ≈ 6.8 s of
balancing for a one-count offset, and ≈ 9.7 s for half a count, so sub-count reference calibration
(0.7) pays off directly. Track tilt doesn't enter: the cart motion is kinematic, and c_bottom
(hanging equilibrium) defines true vertical.

## 9. Hardware-boundary units (for the runtime)
- θ = wrap((counts − c_bottom − CPR/2)·2π/CPR) into (−π, π] (§5.7). The sign of counts vs +θ is to be
  established physically (0.8).
- x = pos_usteps / usteps_per_m. Protocol/Axis *rates* are STEP toggles (issue 3), so
  a_toggles = a·usteps_per_m·toggles_per_ustep, with toggles_per_ustep = 2 until 0.8 confirms it.
- Axis internal sign: `curSpeed` = −(protocol speed), and `Axis::setAcceleration(arg)` adds arg·dt to
  `curSpeed`. So in Axis units, arg = −a_toggles for +x acceleration. (`Pendel::setAcceleration` does
  this negation; pendctl will do it at its boundary.)
