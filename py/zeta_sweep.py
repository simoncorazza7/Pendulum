"""Why zeta = 0.7? Sweep zeta at the design bandwidth (wc = 2 w0, p = 0.5 w0) on the real loop.

    .venv/Scripts/python py/zeta_sweep.py

Same loop as py/gateb_design.py / py/zeta_study.py: identified plant, BD+LP2 20 Hz, Ts 2 ms, 0.5 ms delay,
demo limits a_max 2, v_max 0.3. Per zeta:
  Kd (multiplies the noisy velocity estimate), RMS command from quantisation noise (the buzz),
  margins (gain [lo, hi], phase, delay), Ms = max |S| (worst-case amplification of a disturbance; 1/Ms is the
  closest approach of the Nyquist curve to -1), Mt = max |T| (resonant peak of the closed loop),
  and a 2 deg catch in the nonlinear simulation (overshoot, settling).
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.linear import build_loop, margins, noise_rms
from zeta_study import DELAY, PLANT_LIN, Q, TS, W0, catch, pid_for


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    w = np.logspace(-1, math.log10(math.pi / TS * 0.999), 4000)
    print("| zeta | Kp | Kd | dominant pair | RMS a, quantisation (m/s²) | gain margin [lo, hi] | phase margin | "
          "delay margin (ms) | Ms = max abs S | Mt = max abs T | 2° catch: overshoot / settle to 0.2° (s) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for zeta in (0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2):
        wc, p = 2 * W0, 0.5 * W0
        pid = pid_for(wc, zeta, p)
        m = margins(PLANT_LIN, pid, TS, DELAY, est="lp2", est_fc=20.0)
        pm = min((abs(ph) for _, ph in m["phase_crossings"]), default=float("nan"))
        nz = noise_rms(PLANT_LIN, pid, TS, DELAY, Q / math.sqrt(12), est="lp2", est_fc=20.0)
        L = build_loop(PLANT_LIN, pid, TS, DELAY, eps_int=1e-9, est="lp2", est_fc=20.0).L(w)
        S = 1 / (1 + L)
        T = L / (1 + L)
        pair = "real" if zeta >= 1 else f"−{zeta * wc:.1f}±{wc * math.sqrt(1 - zeta ** 2):.1f}j"
        c = catch(pid, 2.0)
        print(f"| {zeta:.1f} | {pid.kp:.1f} | {pid.kd:.2f} | {pair} | {nz:.2f} | [{m['k_lo']:.2f}, {m['k_hi']:.1f}] | "
              f"{pm:.0f}° | {m['delay_margin'] * 1e3:.0f} | {np.abs(S).max():.2f} | {np.abs(T).max():.2f} | "
              f"{c['over']:.0%} / {c['settle']:.2f} |", flush=True)


if __name__ == "__main__":
    main()
