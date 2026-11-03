# Stage 3: cascaded PID (outer cart loop), 2026-11-03

Asked for by Simon after the first remote swing-up (2026-11-03): with angle-only control the cart keeps whatever
speed it has when the balance starts, so a catch lasts until the soft limit. Brief §4, Stage 3: "An outer cart
position/velocity loop sets the angle reference, for sustained balancing within the finite track."

## The law

```
a_d       = -cart_kx x - cart_kv v                       (the cart acceleration the outer loop wants)
theta_ref = theta_ref_trim + clamp(a_d / g, +-cart_theta_max)
a         = PID(theta - theta_ref)                       (the Stage 1 balance PID, unchanged)
```

- **Why a lean.** Holding the pendulum at a lean θ takes a cart acceleration gθ (θ̈ = 0 in the model). The angle loop
  drives θ to θ_ref, so at frequencies well below its own bandwidth the cart obeys x'' ≈ a_d: a mass on a spring and
  damper with ω_x = √cart_kx, ζ_x = cart_kv / (2ω_x).
- **Why slow.** To accelerate toward −x the cart must first step toward +x, to lean the pendulum the other way (the
  non-minimum-phase zero at ω₀ ≈ 5 rad/s). The outer loop has to stay far below that and below the angle loop's
  crossover (~10 rad/s).
- **Code.** `core::cartThetaRef` / `core::cascadeStep` (`src/core/pid.*`) and the mirror `control.cart_theta_ref` /
  `cascade_step` (`py/pendel/control.py`), golden-tested (`tests/golden/cascade_*.csv`). With cart_kx = cart_kv = 0 it
  is exactly the angle-only `pidStep` (`cascade_off_is_exactly_the_angle_pid`). Hand catches (`rt::BalanceRuntime`), the
  swing-up's BALANCE phase and the remote PC (`py/remote_ctrl.py`; the gains travel in PARAMS, protocol version 3) all
  use it. Config: `ctrl.cart_kx`, `ctrl.cart_kv`, `ctrl.cart_theta_max_deg`.

## Choosing the gains (`py/cascade_study.py`)

Simulated with the 11-02 plant (catch-study model: estimator, quantisation, actuator, soft limits with the braking
distance), 30 s per case. Cases: 7 real swing-up hand-overs (10-02-18, 15-32-17; cart up to ±55 mm moving up to
253 mm/s), 3 hand releases (v0 ±90 mm/s, x 30 mm), a ±1-count reference error, and the 6-tick remote delay.

| outer loop (ω_x rad/s, ζ_x, lean clamp) | kept 30 s | worst max \|x\| | worst RMS θ |
|---|---|---|---|
| angle only (Stage 1) | 0/13 | — | — |
| **0.5, 1.0, ±4° (default)** | **12/13** | 114 mm | **0.15°** |
| 0.7, 0.7, ±4° | 12/13 | 111 mm | 0.16° |
| 0.7, 1.0, ±4° | 11/13 | 126 mm | 0.19° |
| 0.85, 1.0, ±4° | 10/13 | 131 mm | 0.31° |
| 1.0–2.0, 1.0 | ≤ 5/13 | | sways 1.5–3° |
| 0.7, 1.4, ±4° | 7/13 | 119 mm | 0.55° |

- **Default: ω_x 0.5 rad/s, ζ_x 1 → cart_kx 0.25 1/s², cart_kv 1.0 1/s, clamp ±4°.** Critically damped: the cart
  returns to the centre in ~8 s without overshoot. The velocity term does most of the work: it bleeds off the
  hand-over speed. The clamp never binds in normal balancing (θ_ref = 0.6° at 0.1 m/s).
- **The one loss** is hand-over 15-32-17 #6: −41 mm at −253 mm/s, close to v_max; the soft limit comes in 0.2 s
  whatever the controller does. Remedy (not built): hand over only below a cart speed, else come round again.
- **Reference offset.** A proportional outer loop parks the cart at x ≈ g·δ / cart_kx off-centre for a reference
  error δ: ~0.10 m per count at kx 0.25 (the simulated pivot's stiction band moved it to 70 mm). With the trim within
  ±0.5 count that is ≤ 50 mm. An outer integral would remove it; not added (slower, less margin).

## Tests (PC 94/94)

- `cascade_golden_matches_python`: C++ = Python on 9000 samples (rig gains, a clamping case, off).
- `session_cascade_holds_the_cart_on_the_track`: the 1-count reference error that drives the angle-only session into
  the soft limit (`session_full_flow_two_catches_recentre_quit`, now explicitly angle-only) balances 20 s, parked at
  70 mm, at rest.
- The swing-up tests (local, session, remote) now balance the full 30 s (`balance_max`) instead of 1.5 s.
- `remote_*`: the fake PC runs `cascadeStep` on x, v; every BALANCE tick bit-identical to the shadow controller.

## On the rig (first runs need Simon's go)

1. `./demo.sh` (hand catches): after the release the cart should slow down and come back toward the centre over
   ~5–10 s instead of drifting into the soft limit. Runs now end by Enter or `sup.max_run_time` (60 s).
2. `./swingup.sh` and `./swingup.sh --remote`: balances should reach `swingup.balance_max` (30 s) unless the hand-over
   is already at the speed limit.
3. To compare with Stage 1: `--set ctrl.cart_kx=0 --set ctrl.cart_kv=0`.
4. Watch for a slow sway (0.1–0.3 Hz) that grows: that would be the outer loop too fast for the real rig; halve
   cart_kx and cart_kv.
