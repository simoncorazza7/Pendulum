"""Hand on the rod at engage: two hand models, residual cart velocity after the release (demo procedure).

    .venv/Scripts/python py/hand_hold.py

The supervisor engages while the operator still holds the pendulum near upright (sup.engage_on_release
is off). docs/stage1_drift.md modelled that hand as RIGID IN ANGLE (theta pinned): then the controller
pushes against a pendulum that cannot move and the cart gains (Kp - g) theta per second of hold.
A light grip on the rod is closer to a hand FIXED IN SPACE: the rod pivots on the cart and the grip
point stays where the hand is, so theta = asin((X_hand - x)/l_h). The cart then moves under the hand
(the loop a = Kp (X_hand - x)/l_h - Kd v/l_h + ... is a stable spring-damper) and settles at rest.

Hand model (space-fixed): angular acceleration added to the pendulum
    alpha_h = -k_h (theta - theta_h(x)) - c_h (theta' - theta_h'(x, v)),  theta_h = asin((X_h - x)/l_h)
k_h = (2 pi f_h)^2 sets how stiff the grip is relative to the rod's inertia (f_h = natural frequency of
the rod against the hand). Plant, controller, estimator, quantisation and Axis cap as in py/pendel/sim.py
(identified plant, config gains, BD+LP2 20 Hz, Ts 2 ms, 0.5 ms compute delay).
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel.control import PidParams, PidState, design_gains, pid_step
from pendel.estimators import BdLp2
from pendel.plant import ActuatorParams, PlantParams, SensorParams, measure, pendulum_accel, realised_accel

TS, H_SUB, G = 0.002, 20, 9.81
W0, BJ, COUL = 4.931, 0.024, 0.0117
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=0.0005)
SENS = SensorParams()
L_H = 0.40          # m, grip height on the rod (the operator holds the rod near the top; L_eq = 0.40 m)


def pid(a_max, v_max):
    kp, ki, kd = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
    return PidParams(kp=kp, ki=ki, kd=kd, a_max=a_max, v_max=v_max, i_max=1.0, i_delay=0.0, vel_fc=20.0)


def run(p: PidParams, hand: str, theta0: float, hold: float, f_h: float = 5.0, zeta_h: float = 0.3,
        t_end: float = 8.0, x_lim: float = 0.15):
    """Engaged at t = 0 with the hand on the rod at theta0 (cart at rest), released at t = hold."""
    q = 2 * math.pi / SENS.cpr
    k_h, c_h = (2 * math.pi * f_h) ** 2, 2 * zeta_h * 2 * math.pi * f_h
    X_h = L_H * math.sin(theta0)
    th, thd, x, v = theta0, 0.0, 0.0, 0.0
    st, est = PidState(), BdLp2(TS, 20.0)
    a_applied, h = 0.0, TS / H_SUB
    c_prev = int(np.floor(th / q + SENS.phase))
    events, out = [], {"t": [], "x": [], "v": [], "theta": [], "a": []}
    n = int(round(t_end / TS))
    end = "time"
    for k in range(n):
        t = k * TS
        counts, th_meas = measure(th, SENS)
        vel = est.update(t, counts, events)
        events = []
        a_new, st, _ = pid_step(p, st, th_meas, v, TS, vel)
        for key, val in (("t", t), ("x", x), ("v", v), ("theta", th), ("a", a_new)):
            out[key].append(val)
        if abs(x) > x_lim:
            end = "soft limit"
            break
        if abs(th) > math.radians(22):
            end = "theta abort"
            break
        held = t < hold
        for j in range(H_SUB):
            if j == 1:
                a_applied = a_new
            a_r = realised_accel(a_applied, v, ACT)

            def f(s):
                acc = pendulum_accel(s[0], s[1], a_r, PLANT)
                if held and hand == "space":
                    sh = max(-0.99, min(0.99, (X_h - s[2]) / L_H))
                    th_h = math.asin(sh)
                    thd_h = -s[3] / (L_H * math.cos(th_h))
                    acc += -k_h * (s[0] - th_h) - c_h * (s[1] - thd_h)
                return np.array([s[1], acc, s[3], a_r])

            s0 = np.array([th, thd, x, v])
            k1 = f(s0); k2 = f(s0 + 0.5 * h * k1); k3 = f(s0 + 0.5 * h * k2); k4 = f(s0 + h * k3)
            th_old = th
            th, thd, x, v = s0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            if held and hand == "rigid":
                th, thd = theta0, 0.0
            u0, u1 = th_old / q + SENS.phase, th / q + SENS.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t, b if c_new > c_prev else b - 1))
            c_prev = c_new
    r = {key: np.array(val) for key, val in out.items()}
    r["end"] = end
    return r


def summary(r, hold):
    t = r["t"]
    after = (t > hold + 1.0) & (t < hold + 2.0)
    v_res = float(np.mean(r["v"][after])) if after.any() else float("nan")
    i_rel = int(np.searchsorted(t, hold))
    x_rel = float(r["x"][min(i_rel, len(t) - 1)])
    t_bal = float(t[-1] - hold)
    peak_a = float(np.max(np.abs(r["a"])))
    return v_res, x_rel, t_bal, peak_a


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    for a_max, v_max in ((2.0, 0.3), (3.0, 0.5)):
        p = pid(a_max, v_max)
        print(f"\n## a_max {a_max} m/s², v_max {v_max} m/s (Kp {p.kp:.1f}, Ki {p.ki:.1f}, Kd {p.kd:.2f}); held at 2°, "
              f"grip {L_H:.2f} m up the rod, soft limit ±0.15 m")
        print("| hand model | hold after engage (s) | x at release (mm) | residual v 1-2 s after release (mm/s) | "
              "time balanced after release (s), end | peak |a| (m/s²) |")
        print("|---|---|---|---|---|---|")
        for hand, f_h in (("rigid", 0.0), ("space", 2.0), ("space", 5.0), ("space", 10.0)):
            label = "rigid in angle" if hand == "rigid" else f"fixed in space, f_h {f_h:.0f} Hz"
            for hold in (0.05, 0.3, 1.0, 2.0):
                r = run(p, hand, math.radians(2.0), hold, f_h=f_h if f_h else 5.0)
                v_res, x_rel, t_bal, peak_a = summary(r, hold)
                print(f"| {label} | {hold:.2f} | {x_rel * 1e3:+.0f} | {v_res * 1e3:+.0f} | {t_bal:.1f}, {r['end']} | "
                      f"{peak_a:.2f} |")


if __name__ == "__main__":
    main()
