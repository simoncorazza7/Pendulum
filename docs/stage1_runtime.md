# Stage 1 runtime: balance tick, simulated hardware, closed-loop tests (2026-10-08)

Built and tested on the PC while the Pi was unreachable. All behaviour below is in **simulation with
PLACEHOLDER plant parameters**. Gains and limits are designed at Gate B from measured values.

## Layers
| Layer | Files | Notes |
|---|---|---|
| Hardware interface | `src/hw/hardware.h` | Raw units only: counts + edge timestamps, Axis microsteps, STEP toggles (internal sign), switches, axis_accelerating, encoder fault, driver |
| Simulated hardware | `src/hw/sim_hw.*`, `py/pendel/simhw.py` | Nonlinear pendulum (viscous + Coulomb), Axis cap A(1−\|v\|/V) after the compute delay, HALT, coasting with the driver off, exact edges, switches and hard stops, separate "true" calibration. C++ bit-identical to Python (golden). Test hook `holdAt(θ, duration)` / `release()` = the operator's hand |
| Units | `src/core/units.*` | counts→θ (centred), pos→x, curSpeed→v, a→Axis argument (×2, sign) |
| Estimator | `src/core/estimator.*` | counts-based BD+LP1 / BD+LP2, golden vs Python |
| Controller | `src/core/pid.*` | pure pidStep, optional external velocity; golden vs Python |
| Supervisor | `src/core/supervisor.*` | IDLE/ARMED/ENGAGED/STOPPING, reasons, braking distance with the Axis cap; golden vs Python |
| Runtime tick | `src/runtime/balance.*` | one control period on any hw::Hardware (below) |

The runtime is portable (no threads, clocks or I/O). On the Pi, pendctl will call `tick()` from the
SCHED_FIFO 500 Hz loop and push the records into the ring log. On the PC it runs in a plain loop with
`SimHw`.

## BalanceRuntime::tick(hw, overrun)
1. `hw.read()`, then θ, ω (estimator × omegaScale), x, v via `core::units` (runtime calibration, c_bottom from
   the start-up fit).
2. Supervisor with operator requests (`requestArm`/`requestStop`, thread-safe) and safety inputs:
   limit switches, encoder fault or edge overflow, loop overrun, Axis in ACCELERATE. The Axis-mode check
   only applies once the runtime has sent a command.
3. Driver follows the mode (on from ARMED, off in IDLE).
4. Command: ENGAGED → controller output through the Stage 2a delay line (reset on engage, with the
   controller); STOPPING → brake command (not delayed, stays local); ARMED → 0 (hold); IDLE → nothing.
5. `axisAccelArg()` → `hw.commandAccel()` (skipped in shadow mode, `actuate = false`).
6. Returns a `TickRecord`: raw counts/pos/speed next to θ, ω, x, v, mode, event, a_ctrl, a_cmd, axis
   argument, flags, integrator.

## Closed-loop tests (`tests/unit/test_balance.cpp`, all pass)
| Test | Result |
|---|---|
| Arm while hanging → lift to 2.3° over 0.4 s → let go on engage → balance → stop request | engages 66 ms after reaching upright; \|θ\| < 0.13° from 1 s on; stop within 0.5 s; driver off |
| Kp = 0.5 g (below the Kp > g limit) | slow divergence, the cart chases the pendulum; **soft-limit abort first**, controlled stop at x = 0.189 m, no crash |
| Upright reference off by 2 counts (PD) | drifts toward +x as derived; soft-limit abort after 1.86 s balanced; stops at x = 0.189 m, inside the 0.2 m soft limit and before the switch |
| Shadow mode | engage/abort logic runs and a_cmd is computed and logged; driver never enabled, no command, cart never moves |
| Delay line 5 vs 30 samples (10 vs 60 ms) | 10 ms holds; 60 ms (beyond the ~40 ms margin) aborts |
| 3 consecutive loop overruns | OVERRUN abort on the third |
| Arm with the cart on the right switch | refused (limit switch), stays IDLE, driver off |
| Wrong θ sign in the configuration | engages, then aborts within 0.5 s (positive feedback), no crash |
| BD+LP2 estimator | balances (\|θ\| < 1° from 1 s on) |

The first version of the hand moved the pendulum to upright instantly: 1185 edges in one sample,
which overflowed the 64-edge buffer. The supervisor correctly treated it as an encoder fault and
disarmed. A real lift spreads the edges, so the hand now moves over 0.4 s.

## Observations for Stage 1
- **Engaging while the hand still holds the pendulum:** the controller already moves the cart, so the
  cart can have velocity at release, which PID then keeps (§7 of the derivation). The tests let go on
  engage. At the rig: release promptly; consider ramping the output over the first 0.1–0.2 s (Gate B
  decision).
- A configuration sign error is caught by θ_abort within 0.5 s, and shadow mode (Stage 1 step 1) must
  catch it before any actuation.

## Next (Pi)
- `PiHw`: wraps `Axis` (atomic acceleration setpoint path, state getters; issue 6), the encoder decoder
  (0.6 decides which), switch reads, encoder fault.
- `pendctl --mode shadow|balance`: SCHED_FIFO loop calling `tick()`, start-up swing calibration
  (`core::fitOffset`), CLI arm/stop, ring log, run directory.
