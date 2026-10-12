"""Which rig effects make the balance loop sway? Candidates for runs 6-8 of 2026-10-12T15-30-52_balance.

    .venv/Scripts/python py/sway_study.py

Rig (Simon): runs 2-5 sat still (RMS theta 0.08 deg, as simulated) but drifted off in 1.3-3.3 s; runs 6-8
balanced 11-36 s while the cart visibly swayed (RMS theta 1.1-1.6 deg, |v| up to v_max 0.3 m/s, 4-9 %
of samples at the 2 m/s^2 limit). The validated model (docs/stage0_12_sim.md) does not sway, so something it
leaves out must. Each candidate below is added alone to the identified plant with the demo controller
(config gains, a_max 2, v_max 0.3, taper 25 %, BD+LP2 20 Hz, Ts 2 ms, 0.5 ms compute delay), caught from
2 deg at rest and run for 30 s without soft limits. Reported: RMS theta after 2 s, dominant theta frequency,
peak |v|, saturated fraction, x range, end. The sway frequency measured in the rig data then points to the
cause (a fast 2-4 Hz wobble = phase margin at crossover; a slow 0.2-1 Hz sway = velocity/saturation loop).

Candidates:
  delay      extra command delay (N samples): eats phase margin (delay margin 48 ms in the linear model)
  gain       input gain k x w0^2/g (the 0.10 pulse measured 0.93, damping 1.03)
  belt       cart compliance: the pivot follows the commanded motion through a 2nd-order resonance f_b, zeta 0.1
  coulomb    pivot Coulomb friction x10
  offset     upright reference error (counts): steady drift acceleration g delta into the v_max taper
  push       release with theta' = 0.3 rad/s (a push or an early engage while the hand still moves)
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

TS, H_SUB = 0.002, 20
W0, BJ, COUL = 4.931, 0.024, 0.0117
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=0.0005)


def pid(a_max=2.0, v_max=0.3):
    kp, ki, kd = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
    return PidParams(kp=kp, ki=ki, kd=kd, a_max=a_max, v_max=v_max, v_taper=0.25, i_max=1.0, i_delay=0.0,
                     vel_fc=20.0)


def run(p: PidParams, t_end=30.0, theta0=math.radians(2), thetad0=0.0, delay=0, gain=1.0, belt_hz=0.0,
        coul=COUL, ref_counts=0.0):
    plant = PlantParams(w0=W0, bJ=BJ, coulomb=coul, g=9.81 / gain)      # input gain x gain
    sens = SensorParams(ref_error=ref_counts * 2 * math.pi / 2400)
    q = 2 * math.pi / sens.cpr
    th, thd = theta0, thetad0
    x, v = 0.0, 0.0                    # Axis (commanded) cart
    xp, vp = 0.0, 0.0                  # pivot (through the belt) when belt_hz > 0
    wb = 2 * math.pi * belt_hz
    st, est = PidState(), BdLp2(TS, 20.0)
    buf = [0.0] * delay
    a_applied, h = 0.0, TS / H_SUB
    c_prev = int(np.floor(th / q + sens.phase))
    events = []
    n = int(round(t_end / TS))
    rec = {"t": np.zeros(n), "theta": np.zeros(n), "x": np.zeros(n), "v": np.zeros(n), "a": np.zeros(n),
           "sat": np.zeros(n, bool)}
    end, k_end = "time", n
    for k in range(n):
        t = k * TS
        counts, th_meas = measure(th, sens)
        vel = est.update(t, counts, events)
        events = []
        a_ctrl, st, dbg = pid_step(p, st, th_meas, v, TS, vel)
        if delay:
            buf.append(a_ctrl)
            a_new = buf.pop(0)
        else:
            a_new = a_ctrl
        rec["t"][k], rec["theta"][k], rec["x"][k], rec["v"][k], rec["a"][k] = t, th, x, v, a_ctrl
        rec["sat"][k] = dbg["saturated"]
        if abs(th) > math.radians(22):
            end, k_end = "fell", k + 1
            break
        for j in range(H_SUB):
            if j == 1:
                a_applied = a_new
            a_r = realised_accel(a_applied, v, ACT)
            if belt_hz > 0:            # pivot driven by the belt spring toward the Axis position
                ap = wb * wb * (x - xp) + 2 * 0.1 * wb * (v - vp)
            else:
                ap = a_r

            def f(s, ap=ap, a_r=a_r):
                return np.array([s[1], pendulum_accel(s[0], s[1], ap, plant)])

            s0 = np.array([th, thd])
            k1 = f(s0); k2 = f(s0 + 0.5 * h * k1); k3 = f(s0 + 0.5 * h * k2); k4 = f(s0 + h * k3)
            th_old = th
            th, thd = s0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            x += v * h + 0.5 * a_r * h * h
            v += a_r * h
            if belt_hz > 0:
                xp += vp * h + 0.5 * ap * h * h
                vp += ap * h
            else:
                xp, vp = x, v
            u0, u1 = th_old / q + sens.phase, th / q + sens.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t, b if c_new > c_prev else b - 1))
            c_prev = c_new
    r = {key: val[:k_end] for key, val in rec.items()}
    r["end"] = end
    return r


def stats(r):
    t, th = r["t"], r["theta"]
    m = t > 2.0
    if m.sum() < 500:
        return dict(rms=float("nan"), f=float("nan"), vmax=np.abs(r["v"]).max(), sat=r["sat"].mean(),
                    xr=np.ptp(r["x"]), end=r["end"], dur=t[-1])
    y = th[m] - th[m].mean()
    spec = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), TS)
    sel = fr > 0.1
    return dict(rms=math.degrees(np.sqrt(np.mean(th[m] ** 2))), f=fr[sel][np.argmax(spec[sel])],
                vmax=np.abs(r["v"]).max(), sat=r["sat"][m].mean(), xr=np.ptp(r["x"]), end=r["end"], dur=t[-1])


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    p = pid()
    cases = [("baseline (validated model)", {})]
    cases += [(f"delay +{d * 2} ms", {"delay": d}) for d in (5, 10, 15, 20)]
    cases += [(f"input gain x{g}", {"gain": g}) for g in (0.5, 0.7, 1.5, 2.5)]
    cases += [(f"belt resonance {f} Hz", {"belt_hz": f}) for f in (5, 8, 12, 20)]
    cases += [("pivot Coulomb x10", {"coul": COUL * 10})]
    cases += [(f"reference offset {c} count", {"ref_counts": c}) for c in (0.5, 1.0, 2.0)]
    cases += [("release with theta' 0.3 rad/s", {"thetad0": 0.3})]
    print("| candidate | end | RMS theta after 2 s (deg) | dominant theta freq (Hz) | peak abs v (m/s) | "
          "saturated after 2 s | x range (mm) |")
    print("|---|---|---|---|---|---|---|")
    for name, kw in cases:
        s = stats(run(p, **kw))
        end = s["end"] if s["end"] == "time" else f"{s['end']} at {s['dur']:.1f} s"
        print(f"| {name} | {end} | {s['rms']:.2f} | {s['f']:.2f} | {s['vmax']:.2f} | {s['sat']:.0%} | "
              f"{s['xr'] * 1e3:.0f} |", flush=True)


if __name__ == "__main__":
    main()
