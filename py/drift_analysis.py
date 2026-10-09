"""Stage 1 drift budget for angle-only PID (identified plant, config gains, BD+LP2 20 Hz).

    .venv/Scripts/python py/drift_analysis.py

Physics identity (integrate the pendulum equation, no external torque, small angles):
    v(t) - v(0) = g * int_0^t theta dt - L_eq (theta'(t) - theta'(0)) - L_eq (b/J) (theta(t) - theta(0))
so after the transient  v_inf = v(0) + L_eq theta'(engage) + g * int theta dt.  With an ideal PID the
equilibrium needs theta = 0 and Ki*I = 0, i.e. I = int theta_meas dt = 0: every part of int theta that the
integrator does NOT see (enable delay, conditional hold in saturation, clamp) becomes cart velocity.
A constant offset delta of the true balance angle gives a steady acceleration g*delta (PID).

Each experiment: catch, 8 s, fit x = x1 + v0 (t - 1) + a_d (t - 1)^2 / 2 over 1..8 s.
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.control import PidParams, design_gains
from pendel.estimators import BdLp2
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.sim import SimParams, simulate

TS = 0.002
G = 9.81
Q = 2 * math.pi / 2400
W0, BJ, COUL = 4.931, 0.024, 0.0117
L_EQ = G / W0 ** 2
KP, KI, KD = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
BASE = PidParams(kp=KP, ki=KI, kd=KD, a_max=3.0, v_max=0.5, i_max=1.0, i_delay=0.1)
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=0.0005)


def run(pid=BASE, th0=math.radians(3), thd0=0.0, phase=0.37, t_end=8.0, plant=PLANT):
    r = simulate(pid, th0, thd0, t_end, plant, ACT, SensorParams(phase=phase), SimParams(ts=TS, x_limit=50.0),
                 estimator=BdLp2(TS, 20.0))
    t, x = r["t"], r["x"]
    ok = ~np.isnan(x)
    t, x = t[ok], x[ok]
    m = t >= 1.0
    tt = t[m] - 1.0
    c = np.polyfit(tt, x[m], 2)
    a_d, v0 = 2 * c[0], c[1]
    th = r["theta"][ok][m]
    return dict(reason=r["reason"], v0=v0, a_d=a_d, x8=x[-1], th_mean=th.mean(), r=r)


def nearest_boundary_offset(phase):
    """True mean angle if the limit cycle straddles the count boundary nearest to upright (rad)."""
    # boundaries at theta/q = k - phase; nearest to 0
    k = round(phase)
    return (k - phase) * Q


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    print(f"Kp {KP:.2f} Ki {KI:.2f} Kd {KD:.3f}; L_eq {L_EQ:.4f} m; one count {Q * 1e3:.3f} mrad -> g*q = {G * Q:.4f} m/s^2\n")

    print("1) Catch-transient terms (phase 0: upright sits exactly on a count boundary, no quantisation offset)")
    print("| variant | v0 (mm/s) | a_d (mm/s^2) | x(8 s) (mm) | end |")
    print("|---|---|---|---|---|")
    variants = [
        ("current: i_delay 0.1, hold in saturation, clamp 1 m/s^2", BASE),
        ("i_delay 0", replace(BASE, i_delay=0.0)),
        ("i_delay 0, no clamp", replace(BASE, i_delay=0.0, i_max=0.0)),
        ("i_delay 0, no clamp, a_max 30 (no saturation)", replace(BASE, i_delay=0.0, i_max=0.0, a_max=30.0)),
        ("PD (Ki = 0)", replace(BASE, ki=0.0)),
    ]
    for name, pid in variants:
        o = run(pid, phase=0.0)
        print(f"| {name} | {o['v0'] * 1e3:+.1f} | {o['a_d'] * 1e3:+.2f} | {o['x8'] * 1e3:+.0f} | {o['reason']} |")

    print("\n2) Release with the pendulum moving (ideal integrator: i_delay 0, no clamp, a_max 30; phase 0)")
    print("| theta'(engage) (rad/s) | v0 (mm/s) | L_eq theta' (mm/s) |")
    print("|---|---|---|")
    ideal = replace(BASE, i_delay=0.0, i_max=0.0, a_max=30.0)
    for thd0 in (0.0, 0.1, 0.3, 0.5, -0.3):
        o = run(ideal, th0=math.radians(2), thd0=thd0, phase=0.0)
        print(f"| {thd0:+.1f} | {o['v0'] * 1e3:+.1f} | {L_EQ * thd0 * 1e3:+.1f} |")

    print("\n3) Quantisation: where upright falls relative to the count boundaries (ideal integrator)")
    print("| encoder phase | nearest-boundary offset (mrad) | mean true theta (mrad) | a_d (mm/s^2) | g*offset (mm/s^2) | x(8 s) (mm) |")
    print("|---|---|---|---|---|---|")
    for ph in (0.0, 0.1, 0.25, 0.37, 0.5, 0.63, 0.75, 0.9):
        o = run(ideal, phase=ph)
        off = nearest_boundary_offset(ph)
        print(f"| {ph:.2f} | {off * 1e3:+.3f} | {o['th_mean'] * 1e3:+.3f} | {o['a_d'] * 1e3:+.2f} | {G * off * 1e3:+.2f} | "
              f"{o['x8'] * 1e3:+.0f} |")


def trim_sweep() -> None:
    print("\n4) Can the reference trim cancel the quantisation offset? (phase 0.37, ideal integrator)")
    print("| theta_ref trim (mrad) | trim (counts) | a_d (mm/s^2) | mean true theta (mrad) |")
    print("|---|---|---|---|")
    ideal = replace(BASE, i_delay=0.0, i_max=0.0, a_max=30.0)
    for tr in (-1.5, -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0, 1.5):
        o = run(replace(ideal, theta_ref=tr * 1e-3), phase=0.37)
        print(f"| {tr:+.1f} | {tr * 1e-3 / Q:+.2f} | {o['a_d'] * 1e3:+.2f} | {o['th_mean'] * 1e3:+.3f} |")


def release_engage() -> None:
    """Engage on release: the hand lets go at theta0 (at rest), the pendulum falls freely, the runtime
    engages once the count has moved 2 counts away from upright. Integrator pre-load I(0) = omega/w0^2
    cancels the momentum term L_eq*theta'(engage) (identity above, cart at rest at engage)."""
    print("\n5) Engage on release (free fall, engage after 2 counts away from upright), ideal integrator, phase 0")
    print("| release angle | detect after (ms) | theta, theta' at engage | v0 no pre-load (mm/s) | "
          "v0 pre-load from estimate (mm/s) | max |theta| (deg), max |a| (m/s^2) |")
    print("|---|---|---|---|---|---|")
    ideal = replace(BASE, i_delay=0.0, i_max=0.0)
    for deg in (1.0, 2.0, 3.0):
        th, thd = math.radians(deg), 0.0
        sens = SensorParams(phase=0.0)
        c0 = math.floor(th / Q + sens.phase)
        est = BdLp2(TS, 20.0)
        h = TS / 20
        k = 0
        while True:
            c = math.floor(th / Q + sens.phase)
            w_est = est.update(k * TS, c, [])
            if (c - c0) * math.copysign(1, th) >= 2:
                break
            for _ in range(20):                      # free fall (a = 0), viscous + Coulomb
                def f(s):
                    acc = W0 ** 2 * math.sin(s[0]) - BJ * s[1] - (math.copysign(COUL, s[1]) if s[1] else 0.0)
                    return np.array([s[1], acc])
                s0 = np.array([th, thd])
                k1 = f(s0); k2 = f(s0 + h / 2 * k1); k3 = f(s0 + h / 2 * k2); k4 = f(s0 + h * k3)
                th, thd = s0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            k += 1
        res = []
        for pre in (0.0, w_est / W0 ** 2):
            r = simulate(ideal, th, thd, 8.0, PLANT, ACT, sens, SimParams(ts=TS, x_limit=50.0),
                         estimator=BdLp2(TS, 20.0), integral0=pre)
            t, x = r["t"], r["x"]
            ok = ~np.isnan(x)
            m = t[ok] >= 1.0
            c2 = np.polyfit(t[ok][m] - 1.0, x[ok][m], 2)
            res.append((c2[1], np.degrees(np.nanmax(np.abs(r["theta"]))), np.nanmax(np.abs(r["a_cmd"]))))
        print(f"| {deg:.0f}° | {k * TS * 1e3:.0f} | {math.degrees(th):.2f}°, {thd:.3f} rad/s (est {w_est:.3f}) | "
              f"{res[0][0] * 1e3:+.1f} | {res[1][0] * 1e3:+.1f} | {res[1][1]:.2f}, {res[1][2]:.2f} |")


if __name__ == "__main__":
    main()
    trim_sweep()
    release_engage()
