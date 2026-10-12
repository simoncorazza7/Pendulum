"""Stage 0.12: validate the simulation (py/pendel/simhw.py, bit-identical to the C++ SimHw) against the
rig recordings of 2026-10-09, before designing with it.

    .venv/Scripts/python py/sim_validate.py [--out docs/img/sim_validation.png]

1. Pulse (0.10, logs/2026-10-09T13-48-40_move): the logged Axis commands (axis_arg per tick) drive the
   simulated rig open loop from the hanging rest state; compare encoder counts and Axis position.
2. Damping, open loop (logs/2026-10-09T13-50-13_damp): the logged commands from the start of WAIT, the
   pendulum's state there taken from a free-swing fit over the preceding 1.5 s; compare phi and x.
3. Damping, closed loop: the damping law with the estimator that ran (BD+LP1 30 Hz) drives the simulated
   rig from the same state; compare phi, x and the engage time.
Plant: identified (w0 4.931, b/J 0.024, Coulomb 0.0117); Axis cap from config (15 m/s^2, 4.5 m/s);
1/8 step (13333 steps/m, the mode of these runs).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel.estimators import BdLp1
from stage1_report import decay_fit
from pendel.runs import load_run
from pendel.simhw import SimHw, SimHwParams
from pendel.units import Calib

TS = 0.002
CPR = 2400
Q = 2 * math.pi / CPR
W0, BJ, COUL = 4.931, 0.024, 0.0117
UPM = 13333.0


def params(c_bottom: float) -> SimHwParams:
    truth = Calib(c_bottom=c_bottom, usteps_per_m=UPM, pos_center=4000.0)
    return SimHwParams(w0=W0, bJ=BJ, coulomb=COUL, axis_a=400000 / (2 * UPM), axis_v=120000 / (2 * UPM),
                       substeps=20, pos_switch_right=-1e9, pos_switch_left=1e9, pos_end_right=-1e9,
                       pos_end_left=1e9, truth=truth)


def swing_state(t, counts, t_end, w0=W0, sigma=0.03, win=1.5):
    """phi, phi' (rad, rad/s) at t_end from a damped-sine fit of the tick counts over [t_end - win, t_end]."""
    m = (t >= t_end - win) & (t <= t_end)
    tt, u = t[m] - t_end, counts[m] + 0.5
    amp = 0.5 * (u.max() - u.min()) * Q
    w = w0 * (1 - amp * amp / 16)
    e = np.exp(-sigma * tt)
    M = np.column_stack([np.ones_like(tt), e * np.cos(w * tt), e * np.sin(w * tt)])
    (c, a, b), *_ = np.linalg.lstsq(M, u, rcond=None)
    return (a) * Q, (b * w - sigma * a) * Q, c        # phi(0) = a*q, phi'(0) = (b w - sigma a) q, offset


def pulse(ax):
    r = load_run("logs/2026-10-09T13-48-40_move")
    d = r.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    hw = SimHw(params(c_bottom=-CPR + 0.5), theta0=math.pi)   # hanging at rest in the middle of count 0
    hw.enable_driver(True)
    sc, sp = [], []
    for k in range(len(t)):
        s = hw.read()
        sc.append(s["counts"])
        sp.append(s["pos_usteps"] - 4000)
        hw.command_accel(int(d["axis_arg"][k]))
        hw.advance(TS)
    sc, sp = np.array(sc), np.array(sp)
    meas = d["counts"] - d["counts"][0]
    first = t < 6.0
    pk_m, pk_s = np.abs(meas[first]).max(), np.abs(sc[first]).max()
    rms = np.sqrt(np.mean((sc[first] - meas[first]) ** 2))
    pos_err = np.abs(sp - (d["pos_usteps"] - d["pos_usteps"][0])).max()
    print(f"1) pulse: first-swing peak measured {pk_m} counts, simulated {pk_s} counts ({pk_s / pk_m:.0%}); "
          f"RMS difference over 6 s {rms:.2f} counts; max Axis position difference {pos_err} steps")
    ax.plot(t, meas, lw=0.8, label="rig")
    ax.plot(t, sc, "--", lw=0.8, label="simulation (logged commands)")
    ax.set_ylabel("pulse: counts")
    ax.legend(fontsize=8)


def damp_runs(ax_phi, ax_x):
    r = load_run("logs/2026-10-09T13-50-13_damp")
    d = r.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    ph = d["phase"].astype(int)
    k0 = int(np.flatnonzero(ph == 0)[0])                   # WAIT starts: driver on
    cb = float(r.meta["c_bottom_fit_counts"])
    phi0, phid0, _ = swing_state(t, d["counts"].astype(float) - cb, t[k0])
    gain = float(r.meta["gain"])
    amax = float(r.config.get("damp.a_max", 1.0))
    n = len(t) - k0
    meas_phi = d["phi"][k0:]
    meas_x = d["x"][k0:] * 1e3
    tt = t[k0:] - t[k0]
    calib = Calib(c_bottom=cb, usteps_per_m=UPM)

    # open loop: the logged commands
    hw = SimHw(params(c_bottom=cb - CPR), theta0=math.pi + phi0, thetadot0=phid0)
    hw.enable_driver(True)
    ol_phi, ol_x = [], []
    for k in range(n):
        s = hw.read()
        ol_phi.append(hw.th - math.pi)
        ol_x.append(hw.x * 1e3)
        hw.command_accel(int(d["axis_arg"][k0 + k]))
        hw.advance(TS)
    ol_phi, ol_x = np.array(ol_phi), np.array(ol_x)

    # closed loop: the damping law (engage at the first zero crossing with |omega| >= 0.15), BD+LP1 30 Hz
    hw = SimHw(params(c_bottom=cb - CPR), theta0=math.pi + phi0, thetadot0=phid0)
    hw.enable_driver(True)
    est = BdLp1(TS, 30.0)
    phase, prev, k_eng = 0, None, None
    cl_phi, cl_x = [], []
    for k in range(n):
        s = hw.read()
        # measured angle from hanging, as the runtime computes it (count centre minus the fitted c_bottom)
        phi_m = math.remainder((s["counts"] + hw.u0_floor + 0.5 - cb) * Q, 2 * math.pi)
        om = est.update(k * TS, s["counts"], [])
        if phase == 0 and prev is not None and (prev < 0) != (phi_m < 0) and abs(phi_m) < 0.02 and abs(om) >= 0.15:
            phase, k_eng = 1, k
        a = max(-amax, min(amax, -gain * om)) if phase == 1 else 0.0
        if phase == 1 and k - k_eng > int(float(r.config.get("damp.duration", 5)) / TS):
            phase, a = 2, 0.0
        prev = phi_m
        cl_phi.append(hw.th - math.pi)
        cl_x.append(hw.x * 1e3)
        hw.command_accel(int(round(-a * UPM * 2)))
        hw.advance(TS)
    cl_phi, cl_x = np.array(cl_phi), np.array(cl_x)

    eng_meas = (int(np.flatnonzero(ph == 1)[0]) - k0) * TS
    span = (tt > 0) & (tt < 5.5)
    print(f"2) damp, open loop (logged commands from WAIT, start phi {math.degrees(phi0):+.2f} deg, "
          f"phi' {phid0:+.3f} rad/s): RMS phi difference {np.degrees(np.sqrt(np.mean((ol_phi[span] - meas_phi[span]) ** 2))):.2f} deg, "
          f"x end {ol_x[span][-1]:+.1f} vs {meas_x[span][-1]:+.1f} mm")
    print(f"3) damp, closed loop (law + BD+LP1 30 Hz): engage after {k_eng * TS if k_eng else float('nan'):.3f} s "
          f"(rig {eng_meas:.3f} s); RMS phi difference {np.degrees(np.sqrt(np.mean((cl_phi[span] - meas_phi[span]) ** 2))):.2f} deg; "
          f"x max {np.abs(cl_x[span]).max():.1f} vs {np.abs(meas_x[span]).max():.1f} mm, end {cl_x[span][-1]:+.1f} vs {meas_x[span][-1]:+.1f} mm")
    for name, y, k_e in (("rig", meas_phi, int(round(eng_meas / TS))), ("open loop", ol_phi, int(round(eng_meas / TS))),
                         ("closed loop", cl_phi, k_eng or 0)):
        m = slice(k_e, k_e + int(5.0 / TS))
        sg, sg_se, _ = decay_fit(tt[m], y[m], 0.99, W0)
        print(f"   decay rate after engage ({name}): {sg:.3f} +- {sg_se:.3f} 1/s")
    ax_phi.plot(tt, np.degrees(meas_phi), lw=0.9, label="rig")
    ax_phi.plot(tt, np.degrees(ol_phi), "--", lw=0.8, label="sim, logged commands")
    ax_phi.plot(tt, np.degrees(cl_phi), ":", lw=1.0, label="sim, closed loop")
    ax_phi.set_ylabel("damp: phi (deg)")
    ax_phi.legend(fontsize=8)
    ax_x.plot(tt, meas_x, lw=0.9, label="rig")
    ax_x.plot(tt, ol_x, "--", lw=0.8, label="sim, logged commands")
    ax_x.plot(tt, cl_x, ":", lw=1.0, label="sim, closed loop")
    ax_x.set_ylabel("damp: x (mm)")
    ax_x.set_xlabel("t (s)")


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/img/sim_validation.png")
    a = ap.parse_args()
    fig, ax = plt.subplots(3, 1, figsize=(12, 9))
    pulse(ax[0])
    damp_runs(ax[1], ax[2])
    for x in ax:
        x.grid(True)
    fig.savefig(a.out, dpi=110, bbox_inches="tight")
    print(f"plot: {a.out}")


if __name__ == "__main__":
    main()
