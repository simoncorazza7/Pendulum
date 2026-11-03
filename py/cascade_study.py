"""Stage 3 design study: a cascaded PID. An outer cart loop sets the angle reference of the balance PID:

    a_d = -kx x - kv v          (the cart acceleration the outer loop wants; kx = w_x^2, kv = 2 zeta_x w_x)
    theta_ref = sat(a_d / g, +-theta_ref_max)

Holding a lean theta_ref takes a cart acceleration g theta_ref, so at frequencies well below the inner loop the
cart obeys x'' = a_d. The outer loop must stay well below the inner crossover (~2 w0 = 10 rad/s) and the
non-minimum-phase zero at w0 (~5 rad/s): to accelerate toward -x the cart first has to step toward +x to lean
the pendulum. Plant, estimator, quantisation and actuator as py/catch_study.py, with the 2026-11-02 pivot.

    .venv/Scripts/python py/cascade_study.py

Cases: the 7 swing-up hand-overs of 2026-11-03T10-02-18 / 15-32-17 that ended at the soft limit or lasted,
hand-catch releases (cart still, release momentum v0), and a reference offset (trim error) of +-1 count.
Scored over 30 s: stays inside the soft limits (with the braking distance), max |x|, max |v|, RMS theta after 3 s,
cart position at the end.
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

import catch_study as cs
from pendel.control import PidState, pid_step
from pendel.estimators import BdLp2
from pendel.plant import pendulum_accel, realised_accel

G, W0 = 9.81, 4.97
cs.W0 = W0
cs.PLANT = replace(cs.PLANT, w0=W0, bJ=0.054, coulomb=0.168)
TS, H = cs.TS, cs.H_SUB
COUNT = 2 * math.pi / 2400            # rad per encoder count

# theta (deg), theta' (deg/s), x (mm), v (mm/s), label
HANDOVERS = [(9.1, -61, 22, 174, "su 10-02 #1"), (9.2, -56, 55, 191, "su 10-02 #2"), (9.1, -52, 52, 192, "su 10-02 #3"),
             (9.1, -62, -3, 69, "su 10-02 #4"), (-8.8, 50, -51, -201, "su 15-32 #2"),
             (-9.4, 25, -41, -253, "su 15-32 #6"), (-9.0, 68, -8, -150, "su 15-32 #7")]
RELEASES = [(1.0, 0, 0, 90, "hand, v0 +90"), (-1.0, 0, 0, -90, "hand, v0 -90"), (0.5, 10, 30, 50, "hand, x 30 v0 50")]


def run(th0, thd0, x0, v0, kx, kv, th_ref_max, delay=0, offset_counts=0.0, t_end=30.0):
    p = cs.pid_params(1.0)
    th, thd, x, v = math.radians(th0), math.radians(thd0), x0 * 1e-3, v0 * 1e-3
    est = BdLp2(TS, 14.0)
    c_prev = int(np.floor(th / COUNT + cs.SENS.phase))
    events, a_applied, h = [], 0.0, TS / H
    st = PidState(integral=thd / W0 ** 2)          # the swing-up pre-load (0 for a still release)
    line = [0.0] * delay
    vmax = xmax = 0.0
    sq, n = 0.0, 0
    for k in range(int(t_end / TS)):
        t = k * TS
        counts, th_m = cs.measure(th, cs.SENS)
        th_m += offset_counts * COUNT              # reference error: the controller's upright is off by this
        om = est.update(t, counts, events) if k > 0 else thd
        events = []
        a_d = -kx * x - kv * v
        th_ref = max(-th_ref_max, min(th_ref_max, a_d / G))
        a_pid, st, _ = pid_step(replace(p, theta_ref=th_ref), st, th_m, v, TS, om)
        line.append(a_pid)
        a = line.pop(0)
        stop = v * v / (2 * cs.A_BRAKE) + abs(v) * 0.006 + 0.01
        if abs(x) >= cs.X_MAX or (v > 0 and x + stop >= cs.X_MAX) or (v < 0 and x - stop <= -cs.X_MAX):
            return dict(ok=False, why="soft limit", t=t, xmax=xmax, vmax=vmax, rms=None, x_end=x)
        if abs(th) > math.radians(22):
            return dict(ok=False, why="FELL", t=t, xmax=xmax, vmax=vmax, rms=None, x_end=x)
        for j in range(H):
            if j == 2:
                a_applied = a
            a_r = realised_accel(a_applied, v, cs.ACT)
            acc = pendulum_accel(th, thd, a_r, cs.PLANT)
            th_old = th
            th += thd * h + 0.5 * acc * h * h
            thd += acc * h
            x += v * h + 0.5 * a_r * h * h
            v += a_r * h
            u0, u1 = th_old / COUNT + cs.SENS.phase, th / COUNT + cs.SENS.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t, b if c_new > c_prev else b - 1))
            c_prev = c_new
        vmax, xmax = max(vmax, abs(v)), max(xmax, abs(x))
        if t > 3.0:
            sq += th * th
            n += 1
    return dict(ok=True, why="30 s", t=t_end, xmax=xmax, vmax=vmax, rms=math.degrees(math.sqrt(sq / n)), x_end=x)


def cell(r):
    if not r["ok"]:
        return f"{r['why']} {r['t']:.1f} s"
    return f"ok, max x {r['xmax'] * 1e3:.0f} mm, RMS {r['rms']:.2f} deg"


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    designs = [("angle only (now)", 0.0, 0.0, 0.0)]
    for wx, zx in [(0.7, 1.0), (1.0, 1.0), (1.5, 1.0), (2.0, 1.0)]:
        designs.append((f"w_x {wx}, zeta {zx}", wx * wx, 2 * zx * wx, math.radians(4)))
    cases = HANDOVERS + RELEASES
    print("| case | " + " | ".join(d[0] for d in designs) + " |")
    print("|---" * (len(designs) + 1) + "|")
    for th0, thd0, x0, v0, label in cases:
        cells = [cell(run(th0, thd0, x0, v0, kx, kv, tm)) for _, kx, kv, tm in designs]
        print(f"| {label} | " + " | ".join(cells) + " |")
    print()
    print("Reference offset (trim error) and remote delay, hand release v0 +90 mm/s:")
    print("| design | offset +1 count | offset -1 count | remote 6 ticks | remote 6 + offset +1 |")
    print("|---|---|---|---|---|")
    for name, kx, kv, tm in designs[1:]:
        cells = [cell(run(1.0, 0, 0, 90, kx, kv, tm, offset_counts=+1)), cell(run(1.0, 0, 0, 90, kx, kv, tm, offset_counts=-1)),
                 cell(run(1.0, 0, 0, 90, kx, kv, tm, delay=6)), cell(run(1.0, 0, 0, 90, kx, kv, tm, delay=6, offset_counts=1))]
        print(f"| {name} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
