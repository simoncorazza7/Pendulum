"""Stage 3b design study: the cart loop with an integral, and a hand-over gate (docs/stage3_cascade.md).

    a_d       = -cart_kx x - cart_kv v - cart_ki X,      X = integral of x dt
    theta_ref = clamp(a_d / g, +-cart_theta_max)

The integral removes the parking offset x = g delta / cart_kx that a reference error delta leaves (rig 2026-11-04:
delta ~ 1.3 counts -> 0.13 m, every run ended at the +x soft limit). Anti-windup: X only integrates while theta_ref is
inside its clamp, and |cart_ki X| <= g cart_i_max. Gains from (s^2 + 2 zeta w s + w^2)(s + p): kx = w^2 + 2 zeta w p,
kv = 2 zeta w + p, ki = w^2 p.

Cases: the 29 swing-up hand-overs logged 2026-11-03/04 (theta, theta', x, v at the hand-over), at the measured
reference error (sim offset -1.3 counts = drift toward +x, as on the rig) and at -2.5. Plant etc. as
py/cascade_study.py. Also prints, per case, the gate quantities: v and the predicted cart speed after the catch.

    .venv/Scripts/python py/cascade_pi_study.py
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

import cascade_study as cz
from pendel.control import PidState, pid_step
from pendel.estimators import BdLp2
from pendel.plant import pendulum_accel, realised_accel

cs = cz.cs
G, W0, COUNT = cz.G, cz.W0, cz.COUNT
TS, H = cs.TS, cs.H_SUB

# theta (deg), theta' (deg/s), x (mm), v (mm/s), rig balance (s), label  [py: logs, swing_phase == 2 start]
HANDOVERS = [
    (-8.9, 71.3, -19.4, -83.1, 1.24, "11-04 09-15#1"), (-8.9, 61.6, -30.1, -138.5, 18.55, "09-15#2"),
    (-9.05, 63.9, -18.4, -112.1, 9.73, "09-15#3"), (9.1, -51.7, 23.0, 227.6, 0.4, "09-15#4"),
    (9.25, -44.5, 27.4, 241.8, 0.23, "09-15#5"), (-8.9, 59.8, -31.0, -95.1, 8.28, "09-15#6"),
    (-9.05, 63.5, -10.6, -89.7, 0.65, "09-20#1"), (-8.9, 62.9, -16.5, -105.4, 1.12, "09-20#2"),
    (8.95, -57.0, 17.4, 194.6, 1.91, "09-20#3"), (9.25, -48.4, 26.5, 237.8, 0.23, "09-20#4"),
    (9.1, -53.1, 23.6, 228.8, 0.47, "09-20#6"), (-8.84, 69.2, 3.2, -77.0, 1.1, "09-34#1"),
    (-8.99, 67.6, -2.6, -111.1, 30.0, "09-34#2"), (-8.84, 64.9, -3.8, -54.1, 0.8, "09-34#3"),
    (9.01, -60.2, 7.7, 149.3, 3.45, "09-34#4"), (8.86, -68.0, -16.1, 41.5, 0.44, "09-34#5"),
    (-9.05, 63.9, -31.2, -116.1, 8.56, "09-36#1"), (-9.05, 62.5, -18.5, -131.8, 4.69, "09-36#2"),
    (9.07, -61.1, 22.4, 174.0, 2.82, "11-03 10-02#1"), (9.22, -55.6, 54.8, 191.4, 0.35, "10-02#2"),
    (9.07, -51.6, 52.1, 191.7, 0.36, "10-02#3"), (9.07, -62.3, -2.9, 69.5, 0.96, "10-02#4"),
    (8.92, -63.3, 8.4, 69.0, 30.0, "10-02#5"), (-8.93, 55.1, -9.8, -133.8, 30.0, "10-02#6"),
    (-8.98, 68.1, -5.9, -151.8, 30.0, "15-32#1"), (-8.83, 49.7, -51.5, -201.5, 0.36, "15-32#2"),
    (-8.83, 68.2, -5.0, -143.2, 30.0, "15-32#3"), (-9.43, 24.5, -41.5, -253.2, 0.18, "15-32#6"),
    (-8.98, 67.5, -8.5, -150.3, 1.81, "15-32#7"),
]


def gains(w, zeta, p):
    return w * w + 2 * zeta * w * p, 2 * zeta * w + p, w * w * p


def run(th0, thd0, x0, v0, kx, kv, ki, th_ref_max=math.radians(4), i_max=math.radians(2), offset_counts=0.0,
        t_end=30.0, trace=False):
    p = cs.pid_params(1.0)
    th, thd, x, v = math.radians(th0), math.radians(thd0), x0 * 1e-3, v0 * 1e-3
    est = BdLp2(TS, 14.0)
    c_prev = int(np.floor(th / COUNT + cs.SENS.phase))
    events, a_applied, h = [], 0.0, TS / H
    st = PidState(integral=thd / W0 ** 2)
    xi = 0.0
    xmax = 0.0
    sq = n = 0
    rows = []
    for k in range(int(t_end / TS)):
        t = k * TS
        counts, th_m = cs.measure(th, cs.SENS)
        th_m += offset_counts * COUNT
        om = est.update(t, counts, events) if k > 0 else thd
        events = []
        a_d = -kx * x - kv * v - ki * xi
        raw = a_d / G
        th_ref = max(-th_ref_max, min(th_ref_max, raw))
        if ki > 0 and raw == th_ref:                    # anti-windup: integrate only inside the clamp
            xi += x * TS
            lim = G * i_max / ki
            xi = max(-lim, min(lim, xi))
        a, st, _ = pid_step(replace(p, theta_ref=th_ref), st, th_m, v, TS, om)
        stop = v * v / (2 * cs.A_BRAKE) + abs(v) * 0.006 + 0.01
        if abs(x) >= cs.X_MAX or (v > 0 and x + stop >= cs.X_MAX) or (v < 0 and x - stop <= -cs.X_MAX):
            return dict(ok=False, why="soft limit", t=t, xmax=xmax, x_end=x, rows=rows)
        if abs(th) > math.radians(22):
            return dict(ok=False, why="FELL", t=t, xmax=xmax, x_end=x, rows=rows)
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
        xmax = max(xmax, abs(x))
        if trace and k % 250 == 0:
            rows.append((t, x, v, math.degrees(th_ref)))
        if t > 3.0:
            sq += th * th
            n += 1
    return dict(ok=True, why="30 s", t=t_end, xmax=xmax, x_end=x, rms=math.degrees(math.sqrt(sq / n)), rows=rows)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    designs = [("P (now): kx .25 kv 1.0", 0.25, 1.0, 0.0)]
    for w, z, pp in [(0.5, 1.0, 0.15), (0.5, 1.0, 0.25), (0.5, 0.8, 0.2), (0.6, 1.0, 0.2)]:
        kx, kv, ki = gains(w, z, pp)
        designs.append((f"PI w {w} z {z} p {pp}: kx {kx:.2f} kv {kv:.2f} ki {ki:.3f}", kx, kv, ki))
    for off in (-1.3, -2.5):
        print(f"\nReference error {off:+.1f} counts (sim; the rig's +1.3 drift toward +x):")
        print("| design | kept 30 s | of those: worst max x (mm) | ended (sim) |")
        print("|---|---|---|---|")
        for name, kx, kv, ki in designs:
            ok, xs, ends = 0, [], []
            for th0, thd0, x0, v0, rig, lab in HANDOVERS:
                r = run(th0, thd0, x0, v0, kx, kv, ki, offset_counts=off)
                if r["ok"]:
                    ok += 1
                    xs.append(r["xmax"] * 1e3)
                else:
                    ends.append(f"{r['t']:.1f}")
            print(f"| {name} | {ok}/{len(HANDOVERS)} | {max(xs) if xs else 0:.0f} | {', '.join(ends)} s |")


if __name__ == "__main__":
    main()
