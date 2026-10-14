"""When is a continuous-time design valid? The pole-placement gains (designed in s) run at sampling period Ts.

    .venv/Scripts/python py/ts_study.py

Emulation design: the gains come from the continuous model, the controller runs every Ts with a zero-order
hold (ZOH) and a compute delay. Sampling then adds a lag of about Ts/2 (the ZOH holds each command for a whole
period, on average half a period late) plus the compute delay tau, i.e. a phase loss of (Ts/2 + tau) * w at
frequency w. The design stays valid while that loss at the gain-crossover frequency w_gc is small next to the
phase margin, i.e. w_gc * Ts << 1 (rules of thumb: sampling 20-30x the closed-loop bandwidth).

For each Ts the exact discrete loop (pendel.linear: exact ZOH of the identified plant, delay, BD+LP2 velocity
filter at min(20 Hz, f_s/10)) gives the margins, the RMS command from quantisation noise, and the dominant
closed-loop poles mapped back to s = ln(z)/Ts. As Ts -> 0 they converge to the continuous loop (with the filter).
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.control import PidParams, design_gains
from pendel.linear import build_loop, margins, noise_rms
from pendel.plant import PlantParams

W0, BJ = 4.931, 0.024
PLANT = PlantParams(w0=W0, bJ=BJ)
TAU = 0.0005                     # s, compute delay (sample -> command reaches the step thread)
Q = 2 * math.pi / 2400


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    kp, ki, kd = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
    print(f"continuous design: Kp {kp:.2f}, Ki {ki:.2f}, Kd {kd:.3f}; designed poles -6.9±7.0j, -2.5 (no filter, "
          f"no delay)\n")
    print("| Ts (ms) | f_s (Hz) | filter (Hz) | w_gc (rad/s) | w_gc·Ts | ZOH + delay lag at w_gc | phase margin | "
          "gain margin [lo, hi] | delay margin (ms) | RMS a, quantisation (m/s²) | dominant closed-loop poles s = ln(z)/Ts |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for ts in (0.0005, 0.001, 0.002, 0.005, 0.010, 0.020, 0.030, 0.040):
        fc = min(20.0, 0.1 / ts)
        pid = PidParams(kp=kp, ki=ki, kd=kd, a_max=1e9, v_max=1e9, i_max=0.0, vel_fc=fc)
        m = margins(PLANT, pid, ts, TAU, est="lp2", est_fc=fc)
        if not m["stable"]:
            print(f"| {ts * 1e3:g} | {1 / ts:.0f} | {fc:.0f} | — | — | — | **unstable** | — | — | — | — |")
            continue
        cross = m["phase_crossings"]
        wgc, pm = (cross[-1][0], abs(cross[-1][1])) if cross else (float("nan"), float("nan"))
        lag = math.degrees((ts / 2 + TAU) * wgc)
        nz = noise_rms(PLANT, pid, ts, TAU, Q / math.sqrt(12), est="lp2", est_fc=fc)
        z = np.linalg.eigvals(build_loop(PLANT, pid, ts, TAU, est="lp2", est_fc=fc).closed())
        z = z[np.abs(z) > 1e-9]
        s = np.log(z.astype(complex)) / ts
        dom = sorted((p for p in s if abs(p) < 40 and p.imag >= -1e-9), key=lambda p: -p.real)
        ptxt = ", ".join(f"{p.real:.1f}" + (f"±{abs(p.imag):.1f}j" if abs(p.imag) > 0.05 else "") for p in dom)
        print(f"| {ts * 1e3:g} | {1 / ts:.0f} | {fc:.0f} | {wgc:.1f} | {wgc * ts:.3f} | {lag:.1f}° | {pm:.0f}° | "
              f"[{m['k_lo']:.2f}, {m['k_hi']:.1f}] | {m['delay_margin'] * 1e3:.0f} | {nz:.2f} | {ptxt} |", flush=True)


if __name__ == "__main__":
    main()
