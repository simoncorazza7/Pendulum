# The pendulum's equation of motion, derived step by step

The short version is in `docs/model_derivation.md` §2. This document derives the same equation three
ways: in the cart's frame, from Newton's laws, and with Lagrange. It then shows why the cart's mass,
the belt and the motor do not appear, and how ω₀, L_eq, the hanging form and the upright linear model follow.

Result:

    J θ̈ = m g l sin θ − m l a cos θ − b θ̇        ⇔        θ̈ = ω₀² sin θ − (ω₀²/g) a cos θ − (b/J) θ̇

## 0. Setup

```
          y
          ↑        G (centre of mass)
          |       ●
          |      /
          |  l  /  θ   (from vertical, + when the top leans toward +x)
          |    /
          |   /
   ───────┼──P────────────→ x      P = pivot on the cart, at (x, 0)
        cart, moving with acceleration a = ẍ
```

| Symbol | Meaning |
|---|---|
| x, ẋ, a = ẍ | cart (pivot) position, velocity, acceleration along the track; +x = toward the LEFT switch |
| θ | pendulum angle from upright, + when the top leans toward +x (clockwise in this picture) |
| m | pendulum mass (rod, bob, everything that rotates) |
| l | distance from the pivot to the centre of mass G |
| J_c, J | moment of inertia about G, and about the pivot: J = J_c + m l² |
| b | viscous friction in the pivot bearing (torque −b θ̇) |
| g | 9.81 m/s² |

Position of G, and its derivatives (chain rule, with x and θ both functions of time):

    x_G = x + l sin θ             y_G = l cos θ
    ẋ_G = ẋ + l θ̇ cos θ           ẏ_G = −l θ̇ sin θ
    ẍ_G = a + l θ̈ cos θ − l θ̇² sin θ
    ÿ_G = −l θ̈ sin θ − l θ̇² cos θ

The θ̈ terms are G's tangential acceleration, and the θ̇² terms are its centripetal acceleration (pointing from G to P).

**Torque sign.** θ is measured clockwise, so a torque counts as positive when it turns the pendulum toward +θ. For a force F = (F_x, F_y) acting at a point r = (r_x, r_y) relative to the reference point, that torque is

    τ_θ = r_y F_x − r_x F_y          (= −(r × F)_z, the clockwise part)

## 1. In the cart's frame (the quickest route, and the intuition)

Ride along with the cart. The pivot is at rest in that frame, so the pendulum simply rotates about a fixed axis:

    J θ̈ = Σ torques about P

The frame accelerates with a, so every mass element feels an extra inertial force −(mass)·a along x. For a rigid body the total of these forces acts at G, as gravity's does:

With the lever from P to G, r = (l sin θ, l cos θ):

| Load | Value | Torque about P, τ_θ = r_y F_x − r_x F_y |
|---|---|---|
| gravity at G | F = (0, −m g) | −l sin θ · (−m g) = **+m g l sin θ** |
| inertial force at G | F = (−m a, 0) | l cos θ · (−m a) = **−m l a cos θ** |
| bearing friction | a torque | **−b θ̇** |

The pivot force acts at P itself, so it has no lever and no torque about P. Summing:

    J θ̈ = m g l sin θ − m l a cos θ − b θ̇

In the cart's frame, accelerating the cart to the right is like a wind blowing to the left that pushes on the centre of mass. If the top leans right, gravity tips it further right; a rightward acceleration "blows" it back left. That is the broom on your palm, and why the controller's gains are positive.

## 2. Newton–Euler in the lab frame (no pseudo-forces; shows the pivot force)

Free-body diagram of the pendulum:
- gravity (0, −m g) at G;
- the force the cart exerts on the pendulum through the pivot, (H, V) at P;
- the bearing friction torque −b θ̇.

**Translation of G** (Newton's second law for the centre of mass):

    H = m ẍ_G = m (a + l θ̈ cos θ − l θ̇² sin θ)
    V − m g = m ÿ_G   ⇒   V = m g − m l θ̈ sin θ − m l θ̇² cos θ

**Rotation about G** (Euler: J_c θ̈ = torques about the centre of mass). Gravity acts at G, so it has no torque here. The pivot force acts at r = P − G = (−l sin θ, −l cos θ):

    τ_θ(H, V) = r_y H − r_x V = −l H cos θ + l V sin θ

    J_c θ̈ = −l H cos θ + l V sin θ − b θ̇

**Substitute H and V:**

    −l H cos θ = −m l a cos θ − m l² θ̈ cos² θ + m l² θ̇² sin θ cos θ
    +l V sin θ = +m g l sin θ − m l² θ̈ sin² θ − m l² θ̇² sin θ cos θ
    ─────────────────────────────────────────────────────────────────
    sum        = m g l sin θ − m l a cos θ − m l² θ̈ (cos² θ + sin² θ) + 0

The centripetal terms cancel exactly, and cos² + sin² = 1:

    J_c θ̈ = m g l sin θ − m l a cos θ − m l² θ̈ − b θ̇
    (J_c + m l²) θ̈ = m g l sin θ − m l a cos θ − b θ̇
    J θ̈ = m g l sin θ − m l a cos θ − b θ̇                     ✓ same as §1

The parallel-axis theorem J = J_c + m l² appears by itself. This route also gives the force the cart has to
exert on the pendulum, H = m(a + l θ̈ cos θ − l θ̇² sin θ). The motor supplies H plus the force that
accelerates the cart itself.

## 3. Lagrange (the systematic route)

With the cart motion x(t) prescribed, θ is the only generalised coordinate.

**Kinetic energy:** the translation of G plus the rotation about G.

    T = ½ m (ẋ_G² + ẏ_G²) + ½ J_c θ̇²
      = ½ m (ẋ² + 2 l ẋ θ̇ cos θ + l² θ̇² cos² θ + l² θ̇² sin² θ) + ½ J_c θ̇²
      = ½ m ẋ² + m l ẋ θ̇ cos θ + ½ (J_c + m l²) θ̇²
      = ½ m ẋ² + m l ẋ θ̇ cos θ + ½ J θ̇²

**Potential energy** (height of G): V = m g l cos θ. **Lagrangian:** L = T − V.

    ∂L/∂θ̇ = m l ẋ cos θ + J θ̇
    d/dt ∂L/∂θ̇ = m l ẍ cos θ − m l ẋ θ̇ sin θ + J θ̈
    ∂L/∂θ = −m l ẋ θ̇ sin θ + m g l sin θ

**Euler–Lagrange,** with the non-conservative friction torque as the generalised force Q_θ = −b θ̇:

    d/dt ∂L/∂θ̇ − ∂L/∂θ = Q_θ
    J θ̈ + m l a cos θ − m l ẋ θ̇ sin θ + m l ẋ θ̇ sin θ − m g l sin θ = −b θ̇
    J θ̈ = m g l sin θ − m l a cos θ − b θ̇                     ✓

The ẋ θ̇ terms cancel. A constant cart **velocity** has no effect at all (Galilean invariance); only the
acceleration a enters.

## 4. Why the cart's mass, the belt and the motor are not in it

The textbook cart-pole is driven by a **force** F on a cart of mass M. Then x is a second degree of freedom:

    (M + m) ẍ + m l θ̈ cos θ − m l θ̇² sin θ = F                  (cart and pendulum together)
    J θ̈ + m l ẍ cos θ − m g l sin θ = −b θ̇                       (pendulum, as above)

The two equations are coupled. M, the motor's torque curve and the belt's compliance all shape how the cart responds to F.

Our stepper is different: it imposes the **position** x(t), step by step. As long as it doesn't lose steps,
the cart's motion is whatever we command, and the force it takes is supplied automatically, up to the motor's
torque limit. So a = ẍ is the **input**, and only the pendulum equation remains. That is why the model needs
just two numbers, and why it worked on the rig:
- the 0.10 pulse test measured the input gain at 93 % of the prediction;
- the hanging damping test measured 103 %.

Where the assumption breaks:
- **Step loss** (too much torque demanded): the cart lags the command. The re-centre touch audits this; 0 to 1 mm so far.
- **A soft belt:** the pivot lags the motor. `py/sway_study.py` finds this harmless above about 12 Hz.
- **Supply limits:** the 1 A VMOT fuse, and 0.9 to come.

## 5. Two numbers describe the whole pendulum

Divide by J:

    θ̈ = (m g l / J) sin θ − (m l / J) a cos θ − (b/J) θ̇

and name the groups:

    ω₀² = m g l / J          m l / J = ω₀² / g          b/J
    θ̈ = ω₀² sin θ − (ω₀²/g) a cos θ − (b/J) θ̇

m, l and J only ever appear in these combinations, so the mass, the shape and where the bob sits don't need to be
known separately. Two numbers suffice, both measured from a hanging swing: **ω₀ = 4.931 rad/s** from the period, and
**b/J = 0.024 s⁻¹** from the decay.

**Equivalent length.**

    L_eq = g / ω₀² = J / (m l)

A point mass m at distance L has J = m L², so ω₀² = g/L: L_eq is the length of the point-mass pendulum that behaves
identically. Two examples:
- a uniform rod of length L, pivoted at its end, has l = L/2 and J = m L²/3, so L_eq = 2L/3;
- ours: L_eq = 9.81 / 4.931² = **0.4035 m**.

**Input gain.** ω₀²/g = 2.48 (rad/s²) per (m/s²). One m/s² of cart acceleration gives the pendulum 2.48 rad/s²,
about 142°/s².

## 6. Hanging and upright, linearised

**Hanging.** Put θ = π + φ, where φ is the small angle from straight down. Then sin θ = −sin φ ≈ −φ and
cos θ = −cos φ ≈ −1:

    φ̈ = −ω₀² φ + (ω₀²/g) a − (b/J) φ̇

This is a damped oscillator with natural frequency ω₀: period 2π/ω₀ = 1.274 s, the swing used for identification
and for the start-up calibration. The input enters with the **opposite sign** to the upright case. That is why the
hanging damping test uses a = −gain·φ̇: the cart follows the bob.

**Upright.** sin θ ≈ θ and cos θ ≈ 1:

    θ̈ = ω₀² θ − (b/J) θ̇ − (ω₀²/g) a
    G(s) = Θ(s)/A(s) = −(ω₀²/g) / (s² + (b/J) s − ω₀²)

The characteristic roots are s = −b/(2J) ± √(ω₀² + (b/2J)²) ≈ ±4.93 s⁻¹:
- one root is in the right half-plane, so an uncorrected tilt grows by a factor e every 1/ω₀ = 0.20 s,
  and doubles every 0.14 s;
- this is the time scale the controller has to beat, and why Ts = 2 ms and a 48 ms delay margin are comfortable.

The linearisation error is small where we work. sin θ = θ to 0.03 % at 2.4° and 0.8 % at 13°. The supervisor
aborts at 22°, where the error is 2.5 %; the simulation uses the full nonlinear equation anyway.

## 7. A first consequence: leaning into a steady acceleration

For the pendulum to stay at a constant angle while the cart accelerates steadily, set θ̈ = θ̇ = 0:

    0 = ω₀² sin θ − (ω₀²/g) a cos θ   ⇒   a = g tan θ ≈ g θ

The pendulum has to lean into the "wind" of §1. Turned around, a controller whose idea of upright is off by δ
balances the pendulum at θ = δ, which only holds while the cart accelerates at a = g δ.

That is the drift of the 2026-10-12 runs: a 2.1-count offset (5.5 mrad) gives 9.81 × 0.0055 = 54 mm/s², exactly the
measured slope. The PID, the conserved quantity behind the cart velocity, and the PD-vs-PID drift difference continue
in `docs/model_derivation.md` §5–8 and `docs/stage1_drift.md`.

## 8. What the model leaves out (and how much it matters)

| Left out | Effect | Status |
|---|---|---|
| Coulomb (dry) friction in the bearing | extra torque −c·sign(θ̇) | measured: 0.0117 rad/s², in the simulation; worth 0.18 counts of static angle |
| step loss, belt compliance | pivot ≠ commanded motion | touch audit 0–1 mm; belt ≥ 12 Hz harmless |
| air drag | ∝ θ̇² | far below the bearing friction at these speeds |
| a flexible rod | extra modes | rod is stiff; no sign in the data |
| track tilt | gravity along the track | does not enter: x(t) is imposed, and the hanging calibration defines true vertical |

Numerical check of the whole chain: `py/model_check.py`. Validation against the rig: `docs/stage0_12_sim.md`, where the
pulse peak matches to 100 % and the damping decay to 1.022 vs 1.019 1/s.
