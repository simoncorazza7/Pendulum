# Stage 0.3: making experiments safe (2026-10-08)

## Changes (branch `control-dev`)
| Commit | Change |
|---|---|
| `8205aa2` | Cross-thread state → `std::atomic` (Axis: pos, curSpeed, direction, endPos, isHoming, maxSpeed, maxAcceleration, running, last*LimitValue; Encoder: winkel, angleVelocity, running; Pendel: plotRunning). The never-initialised `running`/`plotRunning` now start `false`, and `startPlot()` sets its flag before starting the thread. |
| `ebfed6c` | `main [--home] [--plot] [--chip NAME]`. **Default = bring-up mode**: no homing, no gnuplot, EN = 1 until the first motion command (setPos/setRelPos/setSpeed/setAcceleration/calibratePos). `--home` = original start-up. `start.sh` forwards arguments. Protocol unchanged. |
| `1f70ccf` | `safety.{h,cpp}`. SIGINT/SIGTERM → watcher thread → Axis `HALT` (ignores the queue, constant decel at `maxAcceleration`, no overshoot) for ≤ 0.5 s → EN = 1 → exit 128+sig. SEGV/BUS/ILL/FPE/ABRT → EN = 1 in the handler → default action. SIGPIPE ignored + `MSG_NOSIGNAL`. |
| `6281b5f`… | `tests/mock/run_mock_tests.sh` + `tcpq.py`: test suite on the gpio-mockup chip. |

Not changed (no evidence yet, or not 0.3 scope): decoder (#1, #2), Axis caps/braking (#5, #7),
setAcceleration race (#6), protocol parsing (#13), ×2 units (#3, kept on purpose).

## Test method
`gpio-mockup` provides a fake 28-line chip (`gpiochip2`, "gpio-mockup-A"), loaded with
`sudo /sbin/modprobe gpio-mockup gpio_mockup_ranges=-1,28 gpio_mockup_named_lines=1` (gone after a reboot).
`main --chip gpiochip2` uses the same line offsets as on the real chip.
- Inputs (encoder A/B, limit switches) are driven via `/sys/kernel/debug/gpio-mockup/gpiochip2/<line>`,
  which generates real edge events.
- Outputs are read the same way while the process lives. The mock resets lines to 0 on release, so
  **EN after exit is taken from the kernel `gpio_value` trace event**: every value the program wrote
  to EN, in order.
- The script refuses to run unless it finds the mock chip. The real EN read `op hi` before and after.

## Results: 35/35 pass (2026-10-08, commit after `6281b5f`)
| Test | Result | Bears on |
|---|---|---|
| T1 bring-up idle | EN only ever written 1; no STEP toggles; MS1 = MS2 = 1 | 0.3 |
| T2 replies | `getPos getEndPos getSpeed getAngle getAngleVelocity bogus` → `10000 10000 0 0 0 ERR unknown` | protocol unchanged |
| T3 encoder | 12 quadrature edges → count −11 (newest edge held); next A edge → −12; then B dithering with A static → **stale at −13 while the true count is −14** | **#2 confirmed on simulated edges** |
| T4 SIGINT while stepping | EN writes `1 1 0 1`, "Stopped." printed, exit 130 | handler |
| T5 SIGTERM idle | EN writes `1 1 1`, exit 143 | handler |
| T6 SIGSEGV while stepping | EN writes `1 1 0 1` (handler), exit 139 | fatal path |
| T7 `--plot`, no gnuplot | alive after 7 s, survives explicit SIGPIPE | **#12 fixed** |
| T8 empty line | no reply, main thread spins; SIGTERM still stops it (exit 143) | **#13 confirmed**; the watcher is independent of the stuck thread |
| T9 `--home` with simulated switches | duplicated `ERR CALIBRATING`; DIR = 0 toward the "right" switch, DIR = 1 after it; endPos ≈ 1330 after 0.8 s at 2000 toggles/s; EN writes `0 0 1`, exit 130 | **#13 duplicate confirmed**; #4 DIR mapping |

Note on T9: from the right switch (pos = 250) to the left-switch press took ~0.8 s and endPos − 250 − 250 ≈ 830.
That is consistent with ≈ 1000 pos/s at 2000 toggles/s, i.e. **pos counts one per two toggles (#3)**,
seen in the logic. The physical step rate is still to be measured in 0.8.

## Incident during 0.3
A failed build plus `;` ran a stale `main` twice on the real chip, so the cart moved (homing) and EN was
left low. The driver was switched off within ~2 min. Rules are now in CLAUDE.md. No damage reported.

## Still open for 0.3
- **0.3b** (needs a go): run the new `main` in bring-up mode on the real chip, send SIGINT/SIGTERM/SEGV,
  and read EN with `pinctrl get 24` after each. The driver stays off throughout (EN = 1) and no motion
  command is sent.
