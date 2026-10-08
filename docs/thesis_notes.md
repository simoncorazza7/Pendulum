# Notes from the 2025/26 bachelor thesis

Source: `../Bachelorarbeit_Leonard_Zensen_1211472_WS2526 (1).pdf` (L. Zensen, submitted 2026-04-01,
in German). It sits in the folder above the code. Everything here is a **thesis claim**, i.e. a prior,
not a measurement. Values the brief asks us to measure are still to be measured.

## Mechanics
| Item | Thesis value | Implication / prior |
|---|---|---|
| Belt | GT2, 2 mm pitch | |
| Motor pulley | 60 teeth, 8 mm bore | 120 mm of belt per motor rev |
| Motor | 23HS2430, 1.8° (200 full steps/rev), 3 A | |
| Microstepping | 1/8 → 1600 µsteps/rev (protocol table also says 1600/rev) | **≈ 13 333 µsteps/m, 75 µm/µstep** (to be checked in 0.8) |
| Rail | MGN12, 700 mm (BOM: "ca. 60–70 cm") | usable travel < 0.7 m |
| Motor mount | rubber isolators (fixed resonance-induced step loss) | |
| Base | 85 × 35 cm wooden board | |
| Pendulum | M3 threaded rod + weight, screwed into a pulley on the encoder shaft (replacing a grub screw) | no length/mass given |

## Electronics (Appendix B schematic, matches the brief's pin map)
- Encoder B38S6G5-600B-G24N: NPN OC, VCC from Pi 5 V, Phase 1 → GPIO10, Phase 2 → GPIO19.
- Pull-ups: 4 × 8 kΩ to Pi 3V3 (encoder A/B, both limit switches); ≈ 0.4 mA per input.
- Limit switches NO to GND: right → GPIO2, left → GPIO9.
- A4988: STEP ← GPIO27, DIR ← GPIO22, EN ← GPIO24, MS1 ← GPIO20, MS2 ← GPIO21, **MS3 not
  connected** (relies on the carrier's pull-down), RESET–SLEEP bridged, VDD = 3.3 V.
  DIR LOW = CCW, HIGH = CW (motor-side; cart direction not stated).
- 24 V supply with **three 1 A blade fuses**: one to the 300 µF bulk cap, one to the 5 V buck
  (which feeds the Pi via header pin 2), and one to the A4988 VMOT. ⚠ The 1 A fuse on VMOT could open
  during aggressive accel tests (0.9). Check how it is wired now.
- A 7" touchscreen runs the desktop and gnuplot (CPU/X11 load on the Pi).

## Software / protocol (Appendix C)
- Positions are in motor steps from the start (right) switch. Defaults: `maxSpeed` 16 000 steps/s,
  `maxAcceleration` 80 000 steps/s². At 13 333 µsteps/m that is ≈ 1.2 m/s and ≈ 6 m/s² (halve the
  speed if suspected issue 3, the ×2 question, is real).
- `getAngle`/`getAngleVelocity` use 2400 counts/rev.
- Accel ramp: accel ∝ (vmax − v), factor `maxAcceleration`; brake ramp linear at `maxAcceleration`.
- Limit switches were moved slightly inward and re-zero the position on every contact at runtime
  (pos = 250 from the end).

## Problems the author reported (relevant to Stage 0)
- Encoder miscounts every 10–20 s on a Pi 3B+. RC filtering (10 kHz), Schmitt triggers, shielding
  and damping helped only partly. The author says moving to the Pi 4 fixed it ("absolutely reliable"),
  but gives no measurement. A "process only on redundant/duplicate entries" logic was also added
  (likely related to suspected issue 2).
- Step loss: fixed by motor rubber mounts, soldered connectors, and full step → 1/8 µstep.
- Cumulative cart-position error was attributed to OS jitter and compensated by re-zeroing at the
  switches. Not proven; open-loop pos counts emitted pulses, so a growing error means lost steps (or
  a counting bug). It's a candidate for the 0.9 step-loss audit.
- The thesis expected 1–10 ms of LAN latency to be "uncritical", based on the 0.7 Hz hanging frequency
  of a 0.5 m pendulum. That is the wrong frequency for an unstable loop (the relevant scale is
  ω₀ and the closed-loop bandwidth), so Stage 2 should re-check it.
