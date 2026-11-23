# Swing-up by energy pumping: notes and feasibility (2026-10-14)

Brief §4 puts this in **Stage 4** ("energy-based swing-up and hand-over to the balance controller, within track
limits"). §1–5 are the theory and the feasibility simulation written for the discussion with Simon
(`py/swingup_study.py`), before anything was built. Simon then pulled swing-up forward: §6 describes its
implementation in pendctl (`rt::SwingUpRuntime`, `./swingup.sh`, 2026-10-14) and §7 the first rig test. Since the
Stage 3 cascade (2026-11-03), the balance phase after the hand-over also runs the cart loop (`docs/stage3_cascade.md`).

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

## 6. Implementation (2026-10-14; Simon pulled swing-up forward for testing)

- `rt::SwingUpRuntime` (`src/runtime/swingup.{h,cpp}`) has phases KICK → PUMP → BALANCE → BRAKE → DAMP_OUT → DONE,
  plus STOPPING. Like the damp runtime, it carries its own aborts; the hand-catch supervisor and its golden tests are
  untouched.
  - **Pumping:** Åström–Furuta with a cap (`swingup.a_pump` 0.75), k_e 1.0, target `swingup.e_ref` +0.3 %, plus cart
    centring 2 rad/s. The v_max taper and a_max come from the balance PID.
  - **Hand-over:** |θ_f| < 10° and |θ_f + θ̇/ω₀| < 6°, with the integrator pre-load. Then the balance PID, angle only.
  - **Normal ends:** PUMP ends on energy above +10 %, 40 s, or the operator's Enter, going to DAMP_OUT. BALANCE ends on
    the soft limit, θ_abort, 30 s, or Enter, going to BRAKE → DAMP_OUT. DAMP_OUT runs the same law toward the hanging
    energy (capped at 0.5 m/s²) until the swing is below 10°, then a final brake.
  - **Hard ends:** limit switch, encoder fault, Axis not in ACCELERATE, overrun streak, quit, or a soft limit while
    kicking or removing energy go to STOPPING, a brake only.
- **The energy estimate needs a matched filter.** θ̇ from BD+LP2 lags about 16 ms at 14 Hz. ½θ̇² + ω₀²(cos θ − 1)
  with the raw θ wobbles by up to about 8 % within a swing. In SimHw no gain ever handed over: the energy hovered
  1–7 % high, or friction-limited below the target.
  - The estimate is exactly d/dt LP2(θ). So the runtime filters θ with the same LP2 (θ_f) and uses it for E, for the
    switching sign and for the hand-over test. That gives the energy of the trajectory 16 ms ago, which is constant
    along a swing.
  - With it, k_e 1.0 hands over in 16.7 s at a_pump 0.75 (11.7 s at 1.0). Lower k_e stalls below the target, because
    friction wins.
- **Session:** `balance.swingup = true` (`./swingup.sh`). After homing or re-centring the session holds the cart and
  prints READY once the pendulum hangs still and the cart is centred. Enter starts a run. Afterwards AFTER_RUN, and
  Enter re-centres. `data.csv` gains `swing_phase` and `energy`; meta `runs` gains `swingup` entries.
- **Tests:** PC 76/76 (`tests/unit/test_swingup.cpp`), on SimHw with the identified plant and the switches:
  - full run: hand-over 16.7 s, balanced 8.1 s until the soft limit, brought down to a 10° swing, done at 43 s, max
    |x| 140 mm (the balance drift), within the soft limits;
  - too much energy: caught at +10 % and brought down;
  - Enter while pumping (87° swing): down in 10 s; quit: a hard stop;
  - the whole session in swing-up mode (calibrate, home, READY, run, quit).
- **What it does not do yet:** cart feedback after the hand-over (Stage 2), so the balance lasts only until the drift
  reaches the soft limit, a few seconds. The rig's real friction and energy losses may need e_ref or a_pump retuned;
  start conservative.

### Running it (each run needs Simon's go)

`./swingup.sh` gives a checklist; then calibration and homing as in the demo.
- **The new hazard:** the pendulum sweeps the whole circle around the pivot, with the tip passing the bottom at up to
  ~4 m/s. Stay outside that circle; never reach in to catch it while it pumps.
- **Enter during a run** = bring it down: the energy is pumped out and the cart stops with the pendulum hanging.
  **q or Ctrl-C** = hard stop: the cart stops, but the pendulum keeps its energy and keeps swinging; let it die down
  before reaching in.
- First run: watch the pumping phase. The energy line should climb to about 1.00 in 15–25 s and the cart should swing
  ±60–100 mm. If the hand-over or the catch fails, the runtime removes the energy by itself.

### Remote (2026-11-03)

`./swingup.sh --remote` runs the balance after the hand-over on the PC (`py/remote_ctrl.py`); the pumping and all
aborts stay on the Pi. Design, tests and the delay study: `docs/remote.md`, "Remote swing-up".

## 7. First rig test (2026-10-14, `logs/2026-10-14T11-36-43_balance`, `py/swingup_report.py`)

- **Swing-up works on the rig.** 7 of 7 runs pumped smoothly to the top in 18.1–19.5 s (SimHw: 16.7 s), with the
  cart within −54 … +86 mm while pumping. 7 of 7 handed over: θ ±9.3–9.6°, u = θ + θ̇/ω₀ 1.4–4.0°, energy
  0.995–0.997, cart within ±30 mm moving at 70–110 mm/s, always toward the lean side.
- **Energy removal works.** Every run was brought down in 15–16 s, with the cart swinging about ±30 mm.
- **The balance after the hand-over was short:** 0.25–0.47 s in 5 runs; 2.2 s and **11.6 s** in the two with u = 1.4°.
  In the short runs the PID drove the cart to v_max and the soft limit.
- **Why** (`py/catch_study.py`, which reproduces exactly which runs survive):
  - Catching from the stable manifold needs no extra cart speed. Each degree of u costs about 35 mm/s, by the drift
    identity (v∞ = v₀ + L_eq θ̇₀ + g∫θ). That comes on top of the 70–110 mm/s the cart already has from pumping.
  - u ≥ 3° ran into the soft limit within 0.3–1.2 s.
  - Neither a gentle first catch (a = 2g·u, the poles mirrored at −ω₀) nor a pre-load that also cancels v₀
    (I(0) = v₀/g + θ̇₀/ω₀²) helped.
  - With angle-only control the cart keeps about its hand-over speed, so even a perfect catch lasts only 2–5 s.
- **Tuned for the next test:** `swingup.e_ref` 0.003 → **0.007**, because the rig lost about 0.6 % more energy on the
  last approach than SimHw does. `swingup.u_sw_deg` 6 → **3**: hand over only on a good arrival, otherwise it comes
  round again. Expect fewer instant soft-limit stops and balances of a few seconds.
- **The real fix is Stage 2 cart feedback** (position and velocity in the balance law after the hand-over). Without
  it, no catch from a moving cart lasts.

## 8. Rig sessions after the pivot rework (2026-11-03)

The reworked pivot (`docs/stage1_demo.md`, "Rebuild on the new PCB and carriage") loses several times more energy
per swing, so the tuning of §7 no longer reached the top.

- **`logs/2026-11-03T09-44-23_balance`** (old swing-up defaults, new plant via `--set`): 2 runs, no hand-over in 40 s,
  peak energy 0.92 and 0.84, cart −53 … +63 mm, |v| ≤ 220 mm/s. Energy balance from `data.csv` (input
  −a θ̇ cos θ / 2g, with a from v): input 0.08–0.10/s against a loss of 0.07–0.09/s at E 0.8–0.9. The friction model
  (b/J ⟨θ̇²⟩ + Coulomb ⟨|θ̇|⟩) gives 0.064/s; the old pivot lost about 0.02/s.
- **Looser limits** (agreed for the next try): `swingup.a_pump` 1.1, `swingup.k_e` 2, `swingup.e_ref` 0.02,
  `swingup.pump_max` 60. v_max stayed 0.3 m/s: 0.4 fails the soft-limit braking-distance check and the
  `move.max_speed` guard.
- **`logs/2026-11-03T09-52-36_balance`:** pumping now reached the top in 17–18 s (4 of 4), but **0 of 4 hand-overs**.
  The arrivals were at E 0.99–1.015: over the top at 30–70°/s, or turned back at 10–11°. The minimum |u| was 2.9–5.6°
  against `swingup.u_sw_deg` 3. The cart was at ±50–100 mm, moving at 150–250 mm/s at the top, and all 4 runs ended
  "soft limit while pumping". At k_e 2 the arrival lands about 0.7–1 % below e_ref.
- **`logs/2026-11-03T10-02-18_balance`** (`swingup.e_ref` 0.01, `swingup.u_sw_deg` 5): **6 of 6 handed over** after
  16.6–21 s of pumping (cart −81 … +95 mm). Arrivals at E 1.0008–1.0048; hand-over at |θ| 9.1°, u 1.3–3.8°. Balances
  2.8, 0.35, 0.36, 0.96, **30 and 30 s** (`swingup.balance_max`). The short ones handed over with the cart at about
  +52 mm moving at +190 mm/s, and reached the soft limit within 0.35 s. Run 5: RMS θ 0.10°, 0 % saturation. Run 6:
  RMS θ 1.19°, 22 % saturation and a 1.6 Hz sway, probably a hand on the rod.
- **These became the defaults** on 2026-11-03: `swingup.a_pump` 1.1, `swingup.k_e` 2, `swingup.e_ref` 0.01,
  `swingup.u_sw_deg` 5, `swingup.pump_max` 60.
- **Lesson:** the hand-over needs the arrival energy within about 1 %, and the cart speed at the hand-over decides how
  long the balance lasts. What came next: the remote swing-up the same afternoon (`docs/remote.md`), then the cart
  loop and the hand-over speed gate (`docs/stage3_cascade.md`, Stage 3 and 3b).
