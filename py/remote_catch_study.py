"""Swing-up hand-over catch with an extra command delay (remote, docs/remote.md "Remote swing-up"): the six rig
hand-overs of 2026-11-03T10-02-18_balance, the 11-02 plant, local vs 6 and 10 ticks and without the pre-load.

    .venv/Scripts/python py/remote_catch_study.py
"""
import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

import catch_study as cs
from pendel.control import PidState, pid_step
from pendel.estimators import BdLp2
from pendel.plant import measure, pendulum_accel, realised_accel

W0 = 4.97
cs.W0 = W0
cs.PLANT = replace(cs.PLANT, w0=W0, bJ=0.054, coulomb=0.168)
TS, H = cs.TS, cs.H_SUB
# rig 10-02-18 hand-overs: theta deg, theta' deg/s, x mm, v mm/s; balanced on the rig (s)
STATES = [(9.1, -61, 22, 174, 2.82), (9.2, -56, 55, 191, 0.35), (9.1, -52, 52, 192, 0.36),
          (9.1, -62, -3, 69, 0.96), (8.9, -63, 8, 69, 30.0), (-8.9, 55, -10, -134, 30.0)]

def run(th0, thd0, x0, v0, n_delay, preload=True, t_end=5.0):
    p = cs.pid_params(1.0)
    th, thd, x, v = math.radians(th0), math.radians(thd0), x0 * 1e-3, v0 * 1e-3
    est = BdLp2(TS, 14.0); q = 2 * math.pi / cs.SENS.cpr
    c_prev = int(np.floor(th / q + cs.SENS.phase))
    events, a_applied, h = [], 0.0, TS / H
    st = PidState(integral=(thd / W0**2 if preload else 0.0))
    line = [0.0] * n_delay
    for k in range(int(t_end / TS)):
        t = k * TS
        counts, th_m = cs.measure(th, cs.SENS)
        om = est.update(t, counts, events) if k > 0 else thd
        events = []
        a_pid, st, _ = pid_step(p, st, th_m, v, TS, om)
        line.append(a_pid); a = line.pop(0)
        stop = v * v / (2 * cs.A_BRAKE) + abs(v) * 0.006 + 0.01
        if abs(x) >= cs.X_MAX or (v > 0 and x + stop >= cs.X_MAX) or (v < 0 and x - stop <= -cs.X_MAX):
            return t, 'soft limit'
        if abs(th) > math.radians(22):
            return t, 'FELL'
        for j in range(H):
            if j == 2: a_applied = a
            a_r = realised_accel(a_applied, v, cs.ACT)
            acc = pendulum_accel(th, thd, a_r, cs.PLANT)
            th_old = th
            th += thd * h + 0.5 * acc * h * h; thd += acc * h
            x += v * h + 0.5 * a_r * h * h; v += a_r * h
            u0, u1 = th_old / q + cs.SENS.phase, th / q + cs.SENS.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t, b if c_new > c_prev else b - 1))
            c_prev = c_new
    return t_end, 'ok'

print("| hand-over (theta, theta', x, v) | rig local | sim local | sim remote 6 ticks | sim remote 6, no pre-load | sim remote 10 ticks |")
print("|---|---|---|---|---|---|")
for th0, thd0, x0, v0, rig in STATES:
    cells = []
    for nd, pl in [(0, True), (6, True), (6, False), (10, True)]:
        t, why = run(th0, thd0, x0, v0, nd, pl)
        cells.append(f"{why} {t:.2f} s" if why != 'ok' else "ok 5 s")
    print(f"| {th0:+.1f} deg, {thd0:+d} deg/s, {x0:+d} mm, {v0:+d} mm/s | {rig} s | " + " | ".join(cells) + " |")
