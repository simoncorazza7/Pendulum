# Stage 1 hardware steps 1–2 (shadow, hanging damping) and the 0.10 pulse test (2026-10-09)

Built on the PC while the Pi was off. Tested in simulation (`tests/unit/test_damp.cpp`, identified
plant) and syntax-checked against stub headers; the Pi build and the mock-chip run
(`tests/mock/run_stage1_mock.sh`) come first in the next rig session. **Order:** these are Stage 1
steps 1–2 ahead of Gate B, which Simon OK'd on 2026-10-09; catches stay behind Gate B (0.9 after the new PCB).

## Start-up calibration (every program start)
`rt::SwingCalibrator` collects encoder edge crossings while Simon's small swing (3–8°) decays freely, and
every 0.5 s fits u = c + e^{−σt}(a cos wt + b sin wt) over the last 3 s (`core::fitOffset`, w and σ
from 0.10). It accepts c_bottom when the amplitude is 2–9°, the residual RMS is < 1 count (a hand on the
rod breaks the free-swing model), se < 0.05 counts, and two consecutive fits agree within 0.2 counts.
In simulation: accepted after ~4 s, error 0.003 counts. It waits while a hand holds the rod and rejects
large swings (the fixed-w model is only valid for small swings). Config `calib.*`.

## Shadow mode (`pendctl --mode shadow`): no motion possible
`hw::SensorHw` requests only the encoder lines (EdgeReader) and the two limit switches as inputs. **The
STEP, DIR, EN and MS lines are never requested**, so EN stays high from the boot config; motor
commands would only be counted (`motor_commands_ignored`, must be 0). The full `rt::BalanceRuntime`
runs with `actuate = false`: it arms automatically whenever IDLE (refused unless the pendulum hangs
still), engages near upright, computes the would-be command, and aborts beyond 22°.

What it checks: θ sign at upright (top toward the LEFT switch = +x must read θ > 0), a_cmd sign
(same as θ: the cart moves under the pendulum), magnitudes (Kp = 6.4 g = 62.8 m/s² per rad ≈ 1.1 m/s²
per degree), estimator, engage/abort logic. Analysis: `py/stage1_report.py shadow` (per-tilt sign
agreement, exact chain check a = Kp θ + Kd ω + Ki I on unsaturated samples).

## Hanging active damping (`pendctl --mode damp`): moves the cart
Near hanging, φ = θ − π: φ'' = −ω₀²φ − (b/J)φ' + (ω₀²/g)a (the input gain flips sign vs upright:
accelerating toward +x makes the bob lag toward −x, i.e. φ grows). The law **a = −gain·φ'** gives
φ'' = −ω₀²φ − (b/J + gain·ω₀²/g)φ', a damping ratio ζ_h with

  gain = (2ζ_hω₀ − b/J)·g/ω₀² = 0.786 (m/s²)/(rad/s) for ζ_h = 0.2,

so the amplitude decays at ζ_hω₀ = 0.99 1/s (open loop 0.012 1/s): it halves every 0.7 s. Physically
the cart follows the bob. **Self-checking:** any sign error (θ, x, Axis direction) makes it
anti-damping; the swing grows and the |φ| > 25°, |v| > 0.3 m/s or soft-limit aborts stop it
(simulated: aborts, no crash).

Cart motion for a swing of amplitude A engaged at a zero crossing: x = −gain∫φ dt runs between 0 and
2·gain·A/ω₀ (2.8 cm at 5°), peak v = gain·A (0.07 m/s), peak a = gain·ω₀·A (0.34 m/s²), and it settles
near gain·A/ω₀ plus a quantisation drift (1 count of φ at engage/end ≈ 2 mm/s). Simulated at 6°: max
|x| 44 mm, decay over 2 s = 0.125 (design 0.139).

Sequence: calibration (driver OFF) → driver on, cart holds → engage at the next zero crossing with
|φ'| > 0.15 rad/s → 5 s damping → brake → driver off. Aborts: switch, encoder fault, Axis not in
ACCELERATE, 3 overruns, |φ| > 25°, |v| > 0.3 m/s, soft limit ±10 cm with braking distance, SIGINT.
Analysis: `py/stage1_report.py damp`: damped-sine fit of the decay rate σ → implied input gain
k = (2σ − b/J)/gain vs ω₀²/g = 2.48 1/m (synthetic check: 101 %).

## 0.10 input-gain pulse (existing `--mode move`)
A triangle move (accelerate a for T, decelerate a for T; distance aT²) with the pendulum hanging still,
dwell 6 s, the same back, dwell 6 s. `py/stage1_report.py pulse` fits the encoder counts with the
linear model, driven by the commanded (and, separately, realised) acceleration; unknowns: offset, φ₀,
φ'₀ and the input gain k (linear least squares). **k ≈ +2.48 1/m confirms ω₀²/g and the combined sign
θ_sign·x_sign; k < 0 means a sign error: do not balance.** Synthetic check (a = 1 m/s², T = 0.25 s,
12° swing): +2.556 (103 %; the 3 % is the 12° nonlinearity), and −2.553 with a flipped sign.

| Pulse | a (m/s²) | T (s) | distance | peak v | predicted swing |
|---|---|---|---|---|---|
| gentle | 0.5 | 0.2 | 20 mm | 0.10 m/s | 2.7° |
| larger | 1.0 | 0.25 | 62 mm | 0.25 m/s | 8° |

Swing amplitude ≈ (ω₀/g)·aT·2 sin(ω₀T/2). Today's moves reached ~0.43 m/s² and ~85 mm/s physically.

## Rig results 2026-10-09 afternoon
- Mock chip (`run_stage1_mock.sh`): 21/21; Pi unit tests 58/58. Found and fixed on the Pi: `TickClock::seconds()`
  before the first tick wrapped (calibration never timed out); shadow armed on the estimator's first (zero) sample.
- First real shadow run timed out: natural hand-started swings are 10–20°, the 9° cap rejected them. Fixed with the
  amplitude-corrected frequency (validated on the 09-32-26 free decay), accept 2–20°.
- **Shadow (`13-36-33_shadow`)**: c_bottom −0.50 counts (se 0.04, 19.6° swing). Armed only when hanging still,
  engaged at +3.9°, aborted at +22.1°, re-armed. θ > 0 with the top toward the LEFT switch and θ < 0 toward the
  right (Simon confirmed): **θ sign at upright confirmed**; a_cmd had θ's sign 100 %, chain check 0.02 m/s².
  0 motor commands. **Gate B item:** one count step kicks ω by ~0.36 rad/s → Kd·ω ≈ ±2.4 m/s² command spikes.
- **Pulse (`13-40-12_move`)**: 61 steps out/back in 0.4 s; response 3–4 counts, counts rose first (sign OK).
  The 100 mm check (`13-44-03_move`, 307 steps) moved ~23 mm: the driver was in **1/8 step** (≈13 300/m), not
  the morning's half-step, so the cart moved ~4.6 mm and the predicted swing is 4.0 counts (measured ~3.5–4).
  The whole-record fit (k = 0.25 → 1.08 1/m after rescaling) is biased low at this amplitude by Coulomb friction
  (−0.73 counts per period) and quantisation: repeat with a larger swing.
- **Pulse repeated at the right scale (`13-48-40_move`, 266 steps = 20 mm measured by Simon, 1/8 step held):**
  first swing predicted ±17.2 counts, measured ±16; whole-record fit **k = +2.32 1/m = 93 % of ω₀²/g**, positive.
  The remaining few % are consistent with Coulomb friction (linear decay ~1 count/period, faster than the model's
  viscous term). **Input gain and the combined sign θ_sign·x_sign confirmed end to end** (0.10 done).
