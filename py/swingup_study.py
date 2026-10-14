"""Energy-based swing-up on this rig: does it fit the track? (Feasibility only; Stage 4 in the brief, not built.)

    .venv/Scripts/python py/swingup_study.py

Energy per unit inertia, zero at the upright rest state (theta from upright):
    E = theta'^2/2 + w0^2 (cos theta - 1)        hanging at rest: E = -2 w0^2
    dE/dt = -(w0^2/g) a theta' cos theta - (b/J) theta'^2          (from the equation of motion)
Astrom-Furuta pumping law: a_pump = k E sign(theta' cos theta), so with E < 0, dE/dt = (w0^2/g) k |E| |theta' cos theta| > 0
(near the bottom: the cart accelerates with the bob's motion, the damp test's law with the sign flipped).
A slow cart PD keeps the cart on the track: a = sat(a_pump - kx x - kv v), with the v_max taper.
Hand-over to the balance PID (config gains, BD+LP2 20 Hz) when |theta| < theta_sw and the unstable-mode content
|theta + theta'/w0| < u_sw (the pendulum arrives along the stable manifold, so this is small when E ~ 0).
The PID then runs angle-only (Stage 1): the cart keeps its hand-over velocity (drift identity), so the run is
scored on reaching upright and the cart velocity at hand-over.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.control import PidParams, PidState, design_gains, pid_step
from pendel.plant import PlantParams, pendulum_accel

TS, H_SUB, G = 0.002, 10, 9.81
W0, BJ, COUL = 4.931, 0.024, 0.0117
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)


def run(a_max, v_max, a_p, k_e=1.0, e_ref=0.02, kx=4.0, kv=2.8, x_lim=0.20, theta_sw=math.radians(10),
        u_sw=math.radians(4), t_end=40.0, kick=0.05):
    """a_p: cap on the pumping acceleration (sets the cart's oscillation, ~4 a_p/(pi w0^2) per side); k_e: high
    gain, so the law is bang-bang at a_p until E is within a few % of the target; e_ref: target energy as a
    fraction of 2 w0^2 above upright (a little overshoot, else friction stalls it just short of the top)."""
    e_target = e_ref * 2 * W0 ** 2
    best = dict(e=-1.0, th=180.0)
    kp, ki, kd = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
    pid = PidParams(kp=kp, ki=ki, kd=kd, a_max=a_max, v_max=v_max, i_max=1.0, i_delay=0.0, vel_fc=20.0)
    th, thd, x, v = math.pi, kick, 0.0, 0.0          # hanging, a small push to start
    h = TS / H_SUB
    mode, st, t_sw, v_sw, xmax, vmax = "pump", PidState(), None, 0.0, 0.0, 0.0
    for k in range(int(t_end / TS)):
        t = k * TS
        thw = math.atan2(math.sin(th), math.cos(th))
        E = 0.5 * thd ** 2 + W0 ** 2 * (math.cos(th) - 1)
        if mode == "pump":
            best["e"] = max(best["e"], E / (2 * W0 ** 2))
            best["th"] = min(best["th"], abs(math.degrees(math.atan2(math.sin(th), math.cos(th)))))
        if mode == "pump":
            if abs(thw) < theta_sw and abs(thw + thd / W0) < u_sw:
                mode, t_sw, v_sw = "balance", t, v
                th = thw
                st = PidState(integral=thd / W0 ** 2)      # momentum pre-load (docs/stage1_drift.md)
            else:
                s = math.copysign(1.0, thd * math.cos(th)) if thd * math.cos(th) != 0 else 0.0
                a = max(-a_p, min(a_p, k_e * (E - e_target) * s)) - kx * x - kv * v
        if mode == "balance":
            a, st, _ = pid_step(pid, st, th, v, TS, thd)
            if abs(th) > math.radians(30):
                return dict(ok=False, why=f"catch failed at {t:.1f} s", t_sw=t_sw, v_sw=v_sw, xmax=xmax, vmax=vmax, best=best)
            if t - t_sw > 2.0:
                return dict(ok=True, why="balanced 2 s", t_sw=t_sw, v_sw=v_sw, xmax=xmax, vmax=vmax, best=best)
        a = max(-a_max, min(a_max, a))
        w = 0.25 * v_max                                # authority taper toward v_max
        if a > 0:
            a *= max(0.0, min(1.0, (v_max - v) / w))
        elif a < 0:
            a *= max(0.0, min(1.0, (v_max + v) / w))
        for _ in range(H_SUB):
            acc = pendulum_accel(th, thd, a, PLANT)
            th += thd * h + 0.5 * acc * h * h
            thd += acc * h
            x += v * h + 0.5 * a * h * h
            v += a * h
        xmax, vmax = max(xmax, abs(x)), max(vmax, abs(v))
        if abs(x) > x_lim:
            return dict(ok=False, why=f"track limit at {t:.1f} s", t_sw=t_sw, v_sw=v_sw, xmax=xmax, vmax=vmax,
                        best=best)
    return dict(ok=False, why="timeout", t_sw=t_sw, v_sw=v_sw, xmax=xmax, vmax=vmax, best=best)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    print("| a_max / v_max | pumping cap a_p (m/s²) | energy target above upright | result | hand-over at (s) | "
          "max abs x (mm) | max abs v (m/s) | cart v at hand-over (mm/s) |")
    print("|---|---|---|---|---|---|---|---|")
    for a_max, v_max in ((2.0, 0.3), (3.0, 0.5)):
        for a_p in (0.75, 1.0):
            for e_ref in (0.0, 0.001, 0.003, 0.01):
                r = run(a_max, v_max, a_p, e_ref=e_ref, u_sw=math.radians(6))
                tsw = f"{r['t_sw']:.1f}" if r["t_sw"] is not None else "—"
                print(f"| {a_max} / {v_max} | {a_p} | {e_ref:.1%} | {r['why']} | {tsw} | {r['xmax'] * 1e3:.0f} | "
                      f"{r['vmax']:.2f} | {r['v_sw'] * 1e3:+.0f} |", flush=True)


if __name__ == "__main__":
    main()
