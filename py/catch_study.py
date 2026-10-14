"""Swing-up hand-over: how to catch without running the cart into the soft limit (rig session 2026-10-14T11-36-43).

    .venv/Scripts/python py/catch_study.py

At the 7 rig hand-overs the pendulum was at +-9.5 deg with theta + theta'/w0 = 1.4-4 deg and the cart within
+-30 mm moving at 70-110 mm/s toward the lean side. The PID (config gains) saturated at once and drove the cart to
v_max and the soft limit within 0.25-0.5 s in 5 of 7 runs. Variants, from those exact states:
  A  now: PID at once, integrator pre-load theta'/w0^2 (clamped by i_max 1 m/s^2 = Ki * I)
  B  PID at once, pre-load v/g + theta'/w0^2 (drift identity v_inf = v + L_eq theta' + g int theta, with
     int theta forced to -I(0): cancels the cart's velocity as well), i_max 3
  C  gentle catch first: a = 2 g (theta + theta'/w0) (poles at -w0, -w0: mirrors the unstable pole, i.e. the least
     effort that stabilises; no action along the stable manifold) until |theta| < 3 deg, then the PID with the
     pre-load of B computed at that moment
  D  C, but pre-load theta'/w0^2 only (as A)
Plant, estimator (BD+LP2 14 Hz), quantisation, Axis cap and the v_max taper as in py/hand_hold.py; soft limit with the
supervisor's braking distance. Scored: survives 5 s inside the soft limits, peak |v|, max |x|, cart v after 3 s.
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.control import PidParams, PidState, design_gains, pid_step
from pendel.estimators import BdLp2
from pendel.plant import ActuatorParams, PlantParams, SensorParams, measure, pendulum_accel, realised_accel

TS, H_SUB, G = 0.002, 10, 9.81
W0, BJ, COUL = 4.931, 0.024, 0.0117
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=0.0005)
SENS = SensorParams()
X_MAX, A_BRAKE = 0.15, 1.0
# rig hand-overs: theta (deg), theta' (deg/s), x (mm), v (mm/s)
STATES = [(+9.6, -28, +30, +100), (+9.3, -39, +10, +76), (+9.6, -28, +17, +110), (-9.4, +31, -19, -105),
          (-9.4, +33, -30, -79), (-9.4, +33, -30, -77), (-9.3, +39, -14, -71)]


def pid_params(i_max):
    kp, ki, kd = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
    return PidParams(kp=kp, ki=ki, kd=kd, a_max=2.0, v_max=0.3, v_taper=0.25, i_max=i_max, i_delay=0.0, vel_fc=14.0)


def taper(a, v, a_max=2.0, v_max=0.3, w=0.075):
    a = max(-a_max, min(a_max, a))
    if a > 0:
        a *= max(0.0, min(1.0, (v_max - v) / w))
    elif a < 0:
        a *= max(0.0, min(1.0, (v_max + v) / w))
    return a


def run(variant, th0, thd0, x0, v0, t_end=5.0):
    p = pid_params(1.0 if variant in ("A", "D") else 3.0)
    th, thd, x, v = math.radians(th0), math.radians(thd0), x0 * 1e-3, v0 * 1e-3
    est = BdLp2(TS, 14.0)
    q = 2 * math.pi / SENS.cpr
    c_prev = int(np.floor(th / q + SENS.phase))
    events, a_applied, h = [], 0.0, TS / H_SUB
    gentle = variant in ("C", "D")
    st = None
    vmax = xmax = 0.0
    for k in range(int(t_end / TS)):
        t = k * TS
        counts, th_m = measure(th, SENS)
        om = est.update(t, counts, events) if k > 0 else thd   # estimator warm at the hand-over in the runtime
        events = []
        if gentle and abs(th_m) < math.radians(3):
            gentle = False
        if gentle:
            a = taper(2 * G * (th_m + om / W0), v)
        else:
            if st is None:
                i0 = om / W0 ** 2 + (v / G if variant in ("B", "C") else 0.0)
                lim = p.i_max / p.ki
                st = PidState(integral=max(-lim, min(lim, i0)))
            a, st, _ = pid_step(p, st, th_m, v, TS, om)
        stop = v * v / (2 * A_BRAKE) + abs(v) * 0.006 + 0.01
        if abs(x) >= X_MAX or (v > 0 and x + stop >= X_MAX) or (v < 0 and x - stop <= -X_MAX):
            return dict(ok=False, t=t, vmax=vmax, xmax=xmax, v_end=v)
        if abs(th) > math.radians(22):
            return dict(ok=False, t=t, vmax=vmax, xmax=xmax, v_end=v, fell=True)
        for j in range(H_SUB):
            if j == 2:
                a_applied = a
            a_r = realised_accel(a_applied, v, ACT)
            acc = pendulum_accel(th, thd, a_r, PLANT)
            th_old = th
            th += thd * h + 0.5 * acc * h * h
            thd += acc * h
            x += v * h + 0.5 * a_r * h * h
            v += a_r * h
            u0, u1 = th_old / q + SENS.phase, th / q + SENS.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t, b if c_new > c_prev else b - 1))
            c_prev = c_new
        vmax, xmax = max(vmax, abs(v)), max(xmax, abs(x))
    return dict(ok=True, t=t_end, vmax=vmax, xmax=xmax, v_end=v)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    print("| variant | survived 5 s | time to soft limit per run (s) | peak abs v (mm/s, median) | max abs x (mm, median) | "
          "cart v at the end (mm/s, median of survivors) |")
    print("|---|---|---|---|---|---|")
    for var in ("A", "B", "C", "D"):
        rs = [run(var, *s) for s in STATES]
        ok = [r for r in rs if r["ok"]]
        times = " ".join("5+" if r["ok"] else f"{r['t']:.2f}{'F' if r.get('fell') else ''}" for r in rs)
        vend = f"{np.median([r['v_end'] for r in ok]) * 1e3:+.0f}" if ok else "-"
        print(f"| {var} | {len(ok)} of {len(rs)} | {times} | {np.median([r['vmax'] for r in rs]) * 1e3:.0f} | "
              f"{np.median([r['xmax'] for r in rs]) * 1e3:.0f} | {vend} |", flush=True)


if __name__ == "__main__":
    main()
