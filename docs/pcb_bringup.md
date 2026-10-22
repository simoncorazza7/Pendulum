# Carrier PCB bring-up (KiCad 10 design in `pcb/`, DRC of 2026-09-25)

Simon soldered the new carrier PCB on 2026-10-21. This is the test procedure, from no power to the motor. Each stage
starts only once the one before has passed.

| Stage | Setup | How | What it proves |
|---|---|---|---|
| **A** | no power, no Pi, driver carrier out (if socketed) | multimeter, this page | no shorts; no path from 5 V or 24 V to any Pi pin; resistor values; orientation |
| **B** | 24 V on, **no Pi** on the board | multimeter, this page | 5 V rail correct before the Pi sees it; nothing feeds the 3.3 V rail or a GPIO pin; encoder is open-collector |
| **C** | Pi on the board; J3, J4, J9 empty; F2 out | `./pcbtest.sh C` | every header GPIO: shorts to the rails, solder bridges between lines, drive readback; the 5 V supply under full CPU load; Vref recorded |
| **D** | switches in J3, encoder in J4; F2 out | `./pcbtest.sh D` | each switch on its own line; encoder quiet at rest, ±2400 counts per turn in the right direction, both channels, no glitches |
| **E** | motor in J9, F2 in, **motor moves** | `./pcbtest.sh E` | driver holds and releases; direction; all four step modes (MS1/MS2) against the limit switch; switch-to-switch span |

Stages A and B are by hand because the worst faults on this board destroy the Pi on first contact, and the Pi
cannot measure them itself. In particular, a GPIO line that the board pulls up to 3.3 V reads the same whether it
sits at 3.3 V or at a fatal 5 V.

> **Probing rule: never put a probe on the 40-pin header while it is powered.** Header pin 1 (3V3) sits right next
> to pin 2 (5 V). A probe touching both with the Pi on puts 5 V on the Pi's 3.3 V rail, which feeds the SoC's I/O and
> the SD card. That destroyed the rig's Pi 4 on 2026-10-22: a spark, then red LED on, green dark or solid, no HDMI
> output, no Ethernet link, with or without its board. Clip the black lead to J2's GND terminal so one hand holds
> one probe, and measure at these points instead:
> - 5 V: C4's + leg (the leg without the stripe).
> - 3.3 V: R3's 3V3 end (find it in stage A, item 9, and mark it).
> - VMOT: C5's + leg.
>
> Header pins are measured only in stages A and B, with no Pi on the board.

## The board (from `pcb/PCB.kicad_pcb`)

- **Power:**
  - 24 V in on J2.
  - **F1** (1 A) → LC filter (C9 220 µF, L1 8.2 µH, C6 150 µF) → **U1** TRACO TSR 3-2450 → **+5 V** → Pi header pins 2/4,
    plus the encoder supply (J4.1). R5 = 5.6 kΩ from TRIM to GND sets 5.0 V: R = 11.2 / (Vout − 3) kΩ, per the
    TSR-3 datasheet. The filter matches the datasheet's recommended input filter.
  - **F2** (1 A) → VMOT on the A4988 carrier (C5 100 µF).
  - D1 (blue) shows 24 V through R7 1.2 kΩ; D2 (green) shows 5 V through R6 160 Ω.
- **A4988 carrier (A1):**
  - STEP ← GPIO27 (pin 13), DIR ← GPIO22 (15), ~ENABLE ← GPIO24 (18), MS1 ← GPIO20 (38), MS2 ← GPIO21 (40).
  - MS3 is not connected (the A4988's own pull-down holds it low, so 1/8 step is the finest).
  - ~RESET is bridged to ~SLEEP. VDD = the Pi's 3.3 V (pins 1/17), so the driver's logic is only powered with the Pi
    on.
  - Coils on J9: 1B, 1A, 2A, 2B.
- **Inputs:**
  - Right switch GPIO2 (pin 3, J3.2) with R4 10 kΩ, plus the Pi's own 1.8 kΩ.
  - Left switch GPIO9 (pin 21, J3.3) with R3 10 kΩ.
  - Encoder A GPIO10 (pin 19, J4.2) with R1 4.7 kΩ; encoder B GPIO19 (pin 35, J4.3) with R2 4.7 kΩ.
  - All pull-ups go to 3.3 V. J3.1 and J4.4 are GND.
- **Same pin map as the old board,** so `config/pendctl.conf pins.*` stay as they are. `pcb-test` refuses to run if
  they differ from the board.

### Review findings

| # | Finding | What to do |
|---|---|---|
| 1 | **No pull-up on ~ENABLE.** GPIO24 sits at its power-on pull-down until the firmware applies `gpio=24=op,dh` (about 1 s), so the driver is **enabled for the first second or so of every boot**. It holds, with no steps; the old board was the same. | Expected. Stage C checks GPIO24 reads "output high" after boot. If you redo the board: a 10 kΩ pull-up from ~ENABLE to 3V3 closes this window. |
| 2 | **R7 dissipates (24 − ~3.1 V) × 17.4 mA ≈ 0.36 W.** | Check its rating (a ¼ W part runs hot). 4.7 kΩ would give 4.4 mA and 0.09 W, still clearly visible. |
| 3 | **U1 or A1 inserted wrongly kills the Pi.** U1 reversed puts 24 V on its output, onto the 5 V rail. A1 rotated 180° puts VMOT (24 V) on ~ENABLE, i.e. GPIO24. | Stage A visual check; stage B measures with the Pi off the board. |
| 4 | **5 V header pins 2/4 sit next to GPIO2 (pin 3) and 3V3 (pin 1).** A bridge there puts 5 V on a GPIO or the 3.3 V rail. | Stage A, items 5–7. |
| 5 | **The encoder is powered from 5 V.** Its outputs are safe only if it is open-collector (NPN), which the thesis says it is. | Stage B, item 9. |
| 6 | **Inrush:** 220 + 150 µF behind F1 and 100 µF behind F2 charge from a hot-plugged 24 V. | Use slow-blow fuses (T1A). A fast 1 A fuse may blow at plug-in with nothing wrong. |
| 7 | **Capacitors on 24 V (C5, C6, C9)** need ≥ 35 V ratings. | Check the markings. |
| 8 | Design-rule report: 0 errors, 0 unconnected pads, 3 silkscreen-at-edge warnings (cosmetic). | Nothing to do. |

## Stage A: before any power

No Pi on the board, A4988 carrier out if socketed, nothing plugged into J2, J3, J4 or J9. Multimeter in Ω or
continuity mode.

**Look:**
1. **U1 orientation** matches the silkscreen (pin 1 = ON/OFF).
2. **A1 orientation:** VMOT/GND end and potentiometer as the silkscreen shows. **Rotated by 180° it kills the Pi.**
3. Electrolytics C4, C5, C6, C9: the stripe (−) on the GND pad. Ratings ≥ 35 V on C5, C6 and C9.
4. LEDs: the flat side / short leg (K) on pad 1 (GND).
5. F1 and F2: 1 A, preferably slow-blow.
6. No solder bridges on the 40-pin header, especially pins 1–6, nor on U1's and A1's pads. No solder balls or
   whiskers.

**Measure** (pin numbers are the Pi header J1, with GND = pin 6):

| # | Between | Expected | If not |
|---|---|---|---|
| 1 | J2.1 (+24 V) – J2.2 (GND) | > 10 kΩ (it may climb while the capacitors charge) | < 100 Ω: short on the 24 V side |
| 2 | VMOT (C5 +) – GND | > 10 kΩ | short at the A4988 / C5 |
| 3 | pin 2 (5 V) – pin 6 (GND) | > 1 kΩ (the green LED path can read a few kΩ one way round) | < 50 Ω: short on the 5 V rail |
| 4 | pin 1 (3V3) – pin 6 | > 10 kΩ | short on the 3.3 V rail |
| 5 | **pin 2 (5 V) – pin 1 (3V3)** | **open** | **would destroy the Pi's 3.3 V supply** |
| 6 | **pin 2 (5 V) – pins 3, 5, 7, 13, 15, 18, 19, 21, 35, 38, 40** | **open** | **5 V on a GPIO destroys it** |
| 7 | **J2.1 (+24 V) and VMOT – pins 1, 2, 3, 13, 15, 18, 19, 21, 35, 38, 40** | **open** | **24 V path to the Pi** |
| 8 | pin 1 (3V3) – pin 19 / pin 35 | 4.7 kΩ each (R1, R2) | missing or wrong resistor |
| 9 | pin 1 (3V3) – pin 21 / pin 3 | 10 kΩ each (R3, R4). Mark the end of R3 that reads 0 Ω to pin 1: the safe 3V3 test point later | missing or wrong resistor |
| 10 | U1 pin 5 (TRIM) – GND | 5.6 kΩ (R5) | **wrong 5 V output voltage**: fix before stage B |
| 11 | pins 13, 15, 18, 38, 40 → A1 pads 15, 16, 9, 10, 11 | 0 Ω each (STEP, DIR, ~ENABLE, MS1, MS2) | open trace or joint |
| 12 | pin 3 → J3.2, pin 21 → J3.3, pin 19 → J4.2, pin 35 → J4.3, pin 2 → J4.1, GND → J3.1 and J4.4 | 0 Ω | open trace or joint |
| 13 | A1 pad 13 (~RESET) – pad 14 (~SLEEP) | 0 Ω; pad 12 (MS3) open | |
| 14 | F1, F2 across | ~0 Ω | blown or loose fuse |
| 15 | Motor plugged into J9 (A4988 out): J9.1–J9.2 and J9.3–J9.4 | a few Ω (one coil each); J9.1–J9.3 open | coils paired wrongly. Unplug the motor again afterwards. |

## Stage B: first power, without the Pi

The Pi is **not** on the board; A4988 out if socketed; nothing in J3, J4 or J9. A bench supply with a current limit
(24 V, 0.2 A) is ideal.

1. 24 V on J2: the blue and green LEDs light.
2. Supply current (bench supply): below ~50 mA. The blue LED takes ~17 mA. U1 idling with the green LED behind it
   takes 5–15 mA (the LED's 18 mA at 5 V is ~4 mA at 24 V). With D1 not fitted yet (2026-10-22), a 0.01 A display
   shows 0.00–0.01. That is the point of this step: no short.
3. **5 V rail, pin 2 to pin 6: 4.90–5.10 V.**
   - Above 5.25 V: **stop** (R5 wrong or its joint open).
   - About 24 V: **stop** (U1 reversed).
4. **3V3 pins 1 and 17: about 0 V.** Anything above 0.3 V means something feeds the 3.3 V rail: **stop**.
5. **Every GPIO pin (3, 13, 15, 18, 19, 21, 35, 38, 40): below 0.3 V.**
6. VMOT at C5 or A1 pad 8: about 24 V with F2 in.
7. After 2 minutes nothing is warm except R7 (warm is expected; too hot to touch = its rating is too small).
8. Power off, insert the A4988 carrier, power on, and repeat items 4 and 5: still about 0 V. A rotated carrier would
   show 24 V here.
9. Power off, plug the encoder into J4, power on. J4.2 and J4.3 to GND must stay **below 0.5 V while you turn the
   encoder**.
   - An open-collector encoder can only pull low, and without the Pi there is no pull-up voltage.
   - Near 5 V means a push-pull 5 V encoder that would drive 5 V into the Pi: **stop**.
   - Power off and unplug the encoder again.
10. Power off. **Raise the bench supply's current limit to 1 A**: with the Pi on, the board draws ~0.13 A at 24 V
    idle and ~0.3 A in stage C's load test, and a 0.2 A limit would sag the supply into a false under-voltage
    verdict. **Remove F2** for stages C and D. Mount the Pi with the power off. Never connect the Pi's USB-C while the
    board powers it: two 5 V supplies would fight.

## Stages C, D, E: `./pcbtest.sh`

On the Pi, from a terminal you can type into (`ssh -t pendel@pendel.local`, then `cd ~/Pendel-main`), one stage per
run. Power off between stages to plug things in.

```
./pcbtest.sh C      # J3/J4/J9 empty, F2 out
./pcbtest.sh D      # switches in J3, encoder in J4, F2 out
./pcbtest.sh E      # motor in J9, F2 in: the motor moves (max 20 mm/s)
```

Each run prints a checklist, builds `pendctl`, runs `pendctl --mode pcb-test` and ends with `pinctrl get 24`. It writes
`logs/<time>_pcb-test-<stage>/report.txt` (every item: PASS / WARN / FAIL / SKIP with the reason) and `meta.json`.

**What stage C does:**
- **C1: which lines.** A header GPIO in an alternate function (UART, SPI) or claimed by a driver is skipped, not
  touched.
- **C2: shorts.** Each line is read with the internal pull-up and then the pull-down. Expected per line:
  - unconnected and A4988 inputs follow the pull;
  - the switch and encoder lines and GPIO3 stay high (pull-ups);
  - MS1/MS2 read low with the pull-down (the A4988's pull-downs).

  "Held HIGH/LOW" names the line and its header neighbours. A line held by something is never driven afterwards.
- **C3: bridges.** Each drivable line is driven high for ~0.3 ms while all others are pulled down, then low while all
  others are pulled up. Every pair of lines can show a bridge this way, and each line's own readback shows that it
  reaches both levels. With nothing in J3/J4 and F2 out, nothing on the board drives these lines, so driving them
  is safe. ENABLE toggling on the A4988 does nothing without VMOT.
- **C4: supply.** `vcgencmd get_throttled` since boot, then 30 s with all four cores at full load, then again. Any
  under-voltage flag fails. Your 5 V (at C4 +) and 3V3 (at R3's 3V3 end) readings under load are entered and judged
  (4.90–5.25 V, 3.20–3.40 V). Never on the header pins: see the probing rule above.
- **C5: Vref.** The A4988's current-limit voltage, measured and recorded; it was never recorded before. The program
  prints the resulting current for the common sense resistors. Don't turn the potentiometer.

**What stage D does:**
- **Switches:**
  - D1: both read 1 at rest.
  - D2: press each switch: only its line goes to 0 and back. A reaction on the other line means swapped wires.
- **Encoder (D3):**
  - 10 s at rest: at most a couple of flickers.
  - One slow turn pushing the bob toward the LEFT switch: −2400 ± 6 counts. Back the other way: +2400.
  - Both channels toggle equally, there are no impossible transitions, and the quadrature quarters are near 0.25.
  - **If the count runs the other way, A and B are swapped.** With `theta_sign = +1` the controller would then push
    the pendulum over, so it fails loudly.

**What stage E does:** it refuses unless the newest stage D log passed, and a limit switch closing unexpectedly ends
any move.
- **E1, hold:** the driver is on for 5 s with no steps; you push the cart and it should resist. Then off: free.
- **E2, direction:** 10 mm toward the LEFT switch and back. If it goes right first, a coil is reversed on J9 and the
  stage stops.
- **E3, step modes:**
  - Seek the right switch at 10 mm/s and back off.
  - Five 15 mm trips: 1/8, 1/4, 1/2 and full step, then 1/8 again. Each goes out the same distance in its own mode
    and comes back in 1/8 counting steps to the switch.
  - All must agree within 10 %. "Stepped like 1/2" and similar names the fault (a swap, an open joint or a bridge on
    MS1/MS2). This was the old board's intermittent fault.
- **E4, span:** switch to switch at 20 mm/s, compared with `balance.span_usteps` (6104). With the new carriage the
  travel may differ: then the step modes have already passed, and the span is the new value for the config. Ends in
  the middle, driver off.

On every exit, including q, Ctrl-C and errors, the lines are left safe:
- ~ENABLE high (driver off);
- MS1/MS2 high (1/8 step);
- STEP/DIR low;
- every other touched line an input with its power-on pull.

A crash triggers `pendctl`'s fatal-signal handler, which sets ~ENABLE high too (`activeEnLine`).

## Tested without hardware

- `tests/unit/test_pcb_check.cpp`: every verdict (pull, bridges, encoder turns including a dead channel and swapped
  A/B, step modes including swapped MS lines, supply flags).
- `tests/pcb/run_pcb_fake.sh`: the real `pcb_test.cpp` against a virtual board (`tests/pcb/fake_gpiod.cpp`, a fake
  libgpiod). It simulates:
  - this board's resistors and the Pi's pulls, with injected solder bridges, shorts and missing resistors;
  - pressed switches and pendulum turns;
  - a cart that steps by MS1/MS2 and DIR.

  It runs stages C, D and E on a good board and with each fault, and checks the verdicts and the safe end state.
