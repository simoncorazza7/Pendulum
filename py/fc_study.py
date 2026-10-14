"""Velocity-filter cut-off vs the ~28 Hz oscillation: how much loop gain is left at 28 Hz, and what it costs.

    .venv/Scripts/python py/fc_study.py

The balance runs of 2026-10-14 grew an oscillation at 27-32 Hz that the rigid model does not have
(docs/stage1_demo.md). Whatever resonates there (most likely the driven cart: rocking or belt), the loop can
only pump it if it still has gain at that frequency. Most of the loop's gain at 28 Hz is Kd times the velocity
estimate, so the BD+LP2 cut-off is the knob. Per cut-off: |C(28 Hz)| (controller gain from counts to command,
the amount of command a 1-count wobble at 28 Hz produces), |L(28 Hz)|, phase and delay margins, the upper gain
margin, quantisation buzz, and a 2 deg catch in the nonlinear simulation.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.linear import build_loop, margins, noise_rms
from zeta_study import DELAY, PLANT_LIN, Q, TS, W0, catch, pid_for

F_OSC = 28.0


def ctrl_gain(pid, fc, f):
    """|a| per count of a sine at f through Kp + Kd * BD+LP2(fc) (+ Ki/s, negligible here)."""
    w = 2 * math.pi * f
    z = np.exp(1j * w * TS)
    bd = (1 - 1 / z) / TS                                    # backward difference
    wc = 2 * math.tan(math.pi * fc * TS) / TS               # prewarped bilinear Butterworth
    s = 2 / TS * (1 - 1 / z) / (1 + 1 / z)
    lp = wc ** 2 / (s ** 2 + math.sqrt(2) * wc * s + wc ** 2)
    c = pid.kp + pid.kd * bd * lp + pid.ki * TS / (1 - 1 / z)
    return abs(c) * Q


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    pid0 = pid_for(2 * W0, 0.7, 0.5 * W0)
    wosc = np.array([2 * math.pi * F_OSC])
    print("| velocity filter | command per 1-count wobble at 28 Hz (m/s²) | abs L(28 Hz) | phase margin | "
          "delay margin (ms) | gain margin [lo, hi] | RMS a, quantisation | 2° catch: overshoot / settle |")
    print("|---|---|---|---|---|---|---|---|")
    for fc in (20, 16, 14, 12, 10, 8):
        pid = pid0.__class__(**{**pid0.__dict__, "vel_fc": float(fc)})
        m = margins(PLANT_LIN, pid, TS, DELAY, est="lp2", est_fc=fc)
        if not m["stable"]:
            print(f"| {fc} Hz | — | — | unstable | — | — | — | — |")
            continue
        pm = min((abs(ph) for _, ph in m["phase_crossings"]), default=float("nan"))
        L = build_loop(PLANT_LIN, pid, TS, DELAY, eps_int=1e-9, est="lp2", est_fc=fc).L(wosc)[0]
        nz = noise_rms(PLANT_LIN, pid, TS, DELAY, Q / math.sqrt(12), est="lp2", est_fc=fc)
        c = catch(pid, 2.0)
        print(f"| {fc} Hz | {ctrl_gain(pid, fc, F_OSC):.2f} | {abs(L):.3f} | {pm:.0f}° | {m['delay_margin'] * 1e3:.0f} | "
              f"[{m['k_lo']:.2f}, {m['k_hi']:.1f}] | {nz:.2f} | {c['over']:.0%} / {c['settle']:.2f} |", flush=True)


if __name__ == "__main__":
    main()
