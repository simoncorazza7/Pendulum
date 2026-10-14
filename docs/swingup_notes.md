# Swing-up by energy pumping: notes and feasibility (2026-10-14)

Brief §7 puts this in **Stage 4** ("energy-based swing-up and hand-over to the balance controller, within track
limits"). It is not in the current scope (Stages 0–1), so nothing here is built into pendctl. This is the theory
and a feasibility simulation for the discussion with Simon (`py/swingup_study.py`).

## 1. Energy and how the cart changes it

Pendulum energy per unit inertia, with the zero at the upright rest state (θ from upright, `docs/eom_derivation.md`):

    E = ½ θ̇² + ω₀² (cos θ − 1)          hanging at rest: E = −2ω₀²;   upright at rest: E = 0

Its rate follows from the equation of motion θ̈ = ω₀² sin θ − (ω₀²/g) a cos θ − (b/J) θ̇:

    dE/dt = θ̇ θ̈ − ω₀² sin θ θ̇ = −(ω₀²/g) a θ̇ cos θ − (b/J) θ̇²

The cart's acceleration is the only handle on the pendulum's energy. It works through the product θ̇ cos θ, and
friction only removes energy.

- **To add energy fastest within |a| ≤ a_p:** a = −a_p · sign(θ̇ cos θ).
- **Near the bottom** (θ = π + φ): a = +a_p · sign(φ̇). The cart accelerates **against** the bob's motion, like
  pumping a swing. Hanging damping (damp mode, `docs/stage1_damp.md`) is the same structure with the sign flipped:
  a = −gain·φ̇, where the cart follows the bob and removes energy.

## 2. The Åström–Furuta law

    a_pump = sat_{a_p}( k (E − E_ref) · sign(θ̇ cos θ) )

With a large k this is bang-bang at ±a_p while E is well below E_ref, and it fades out as E reaches E_ref. If E
overshoots, the same law removes energy. Two parameters dominate:

- **a_p, the pumping cap.** A bang-bang ±a_p at the pendulum frequency moves the cart by about ±4a_p/(π ω₀²) per
  swing, ±52 mm per m/s². With the small-angle estimate, the swing amplitude grows by about 4a_p/g rad per period.
  a_p therefore trades swing-up time against track.
- **E_ref, the energy target.** It must be almost exactly the upright energy.
  - Arriving at upright with E > 0 means a velocity θ̇_top = √(2E). That is unstable-mode content θ̇_top/ω₀,
    which the catch then has to handle; 2 % too much energy is already 16°, not catchable.
  - With E < 0 the pendulum turns back below the top. To come within 10° it needs E within about 0.8 % of 0.
  - Friction near the top makes the approach slow.

A slow cart PD (a −= kx·x + kv·ẋ, about 2 rad/s, well below ω₀) keeps the cart on the track while it pumps. That is
cart-position feedback, the Stage 2 element.

## 3. Hand-over to the balance controller

Switch to the PID when |θ| < θ_sw **and** the unstable-mode content |θ + θ̇/ω₀| < u_sw.

- An arrival with E ≈ 0 runs along the stable manifold of the upright (θ̇ ≈ −ω₀θ), where that content is about 0.
  This is the same quantity that bounds the catch region in the Gate B design.
- **Not too early.** At 20° the pendulum is coasting up by itself, but the PID sees a large angle and pushes it over
  the top: about 10 m/s² unsaturated, against the needed ~0. In the simulation every hand-over at 20° failed; at
  10° and 6° they succeed.
- At hand-over the pendulum moves freely, which is exactly the case the momentum **pre-load** I(0) = θ̇/ω₀² was built
  for (`docs/stage1_drift.md`). The cart velocity it has then stays, by the drift identity, so after a swing-up the
  balance phase needs cart feedback (Stage 2).

## 4. Feasibility on this rig (`py/swingup_study.py`)

Identified plant with Coulomb friction, hand-over at |θ| < 10° and |θ + θ̇/ω₀| < 6°, track ±0.20 m, cart PD 2 rad/s
during pumping, then the angle-only PID (config gains, pre-load) for 2 s:

| a_max / v_max | a_p | E target | Result | Hand-over at | Max \|x\| | Cart v at hand-over |
|---|---|---|---|---|---|---|
| 2 / 0.3 | 0.75 | 0–0.1 % | **caught, balanced** | 17.4 s (~14 swings) | 61–67 mm | −44 mm/s |
| 2 / 0.3 | 0.75 | 1 % | catch, then track limit | 17.4 s | 200 | −31 |
| 2 / 0.3 | 1.0 | 0–1 % | catch, then track limit within ~1.5 s | 12.4 s | 200 | −14 |
| 3 / 0.5 | 0.75 | 0–0.1 % | **caught, balanced** | 17.4 s | 64–98 | −44 |
| 3 / 0.5 | 1.0 | 0–0.1 % | **caught, balanced** | 12.9 s (~10 swings) | 82 | −53 |

First attempts, kept as lessons:
- An uncapped law slammed into a_max at once and hit the track end within 1 s.
- An energy target 2 % high overshot the top at 1.4 rad/s.
- A 20° hand-over failed every time.

## 5. What it would take (in order)

1. **Gate B and 0.9** with the new PCB: the real a_max and v_max, and a motor-only E-stop. Swing-up drives the
   cart harder and longer than balancing.
2. **The ~28 Hz oscillation** seen in balance runs on 2026-10-14 (`docs/stage1_demo.md`) has to be understood first.
   Bang-bang pumping excites structure more than balancing does; smooth the pumping, e.g. a sinusoid locked to the
   swing phase.
3. **Stage 2 cart feedback**: keep the cart on the track during pumping, and stop the post-catch drift.
4. **Accurate energy**: E depends on ω₀ (identified to ±0.2 %), on c_bottom (start-up fit), and on θ̇ (the
   estimator; at swing speeds use edge timing). The target needs to hold to about 0.1 %.
5. **Supervisor:** a SWINGUP state between ARMED and ENGAGED.
   - Aborts: energy far above E_ref (it would rotate over), soft limits with the braking distance at swing speeds,
     v_max, a timeout.
   - Engage through the free-motion path, with pre-load.
   - If the catch fails, fall back to damping (the damp law) and a controlled stop, not more pumping.
