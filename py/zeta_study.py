"""Critically damped balance controller? Pole placement with zeta = 1, compared with the current design.

    .venv/Scripts/python py/zeta_study.py

Closed loop (linear, continuous): (s^2 + 2 zeta wc s + wc^2)(s + p). zeta = 1 makes the dominant pair a real
double pole; p = wc as well makes all three poles coincide (the "fully critically damped" PID). With an
integrator the angle still crosses zero once after a catch: the PID only rests when int(theta) = 0
(docs/stage1_drift.md), so theta must go to the other side. Only PD (p = 0) can settle monotonically, and it
keeps the cart drifting at g int(theta).

Every candidate on the identified plant with the demo limits (a_max 2, v_max 0.3), BD+LP2 20 Hz, Ts 2 ms,
0.5 ms compute delay. Linear (pendel.linear, exact ZOH): margins and RMS command from quantisation noise.
Nonlinear (pendel.sim, validated): a catch from 2 deg at rest (the hand-release case) and from 4 deg (the engage
threshold): overshoot to the other side, zero crossings, settling, saturation, residual cart velocity.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel.control import PidParams, design_gains
from pendel.estimators import BdLp2
from pendel.linear import margins, noise_rms
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.sim import SimParams, simulate

TS, DELAY = 0.002, 0.0005
W0, BJ, COUL = 4.931, 0.024, 0.0117
Q = 2 * math.pi / 2400
PLANT_LIN = PlantParams(w0=W0, bJ=BJ)
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=DELAY)
A_MAX, V_MAX = 2.0, 0.3


def kd_of(wc, zeta, p):
    return (2 * zeta * wc + p - BJ) * 9.81 / W0 ** 2


def wc_for_kd(kd, zeta, p_of_wc):
    """Bandwidth wc giving the same Kd (same noise gain) for a given zeta and p(wc)."""
    lo, hi = 0.5, 50.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if kd_of(mid, zeta, p_of_wc(mid)) < kd else (lo, mid)
    return 0.5 * (lo + hi)


KD_NOW = kd_of(2 * W0, 0.7, 0.5 * W0)
WC_SAME_KD = wc_for_kd(KD_NOW, 1.0, lambda wc: 0.5 * W0)
WC_TRIPLE_SAME_KD = wc_for_kd(KD_NOW, 1.0, lambda wc: wc)
CANDIDATES = [  # name, wc, zeta, p
    ("now: wc 2w0, zeta 0.7, p 0.5w0", 2 * W0, 0.7, 0.5 * W0),
    ("zeta 1, same wc and p", 2 * W0, 1.0, 0.5 * W0),
    (f"zeta 1, wc {WC_SAME_KD / W0:.2f}w0 (same Kd)", WC_SAME_KD, 1.0, 0.5 * W0),
    (f"triple pole at -{WC_TRIPLE_SAME_KD / W0:.2f}w0 (same Kd)", WC_TRIPLE_SAME_KD, 1.0, WC_TRIPLE_SAME_KD),
    ("triple pole at -2w0", 2 * W0, 1.0, 2 * W0),
    ("PD zeta 1, wc 2w0", 2 * W0, 1.0, 0.0),
    ("zeta 0.7, slower integrator p 0.25w0", 2 * W0, 0.7, 0.25 * W0),
    ("zeta 1, slower integrator p 0.25w0", 2 * W0, 1.0, 0.25 * W0),
]


def pid_for(wc, zeta, p):
    kp, ki, kd = design_gains(W0, BJ, wc, zeta, p)
    return PidParams(kp=kp, ki=ki, kd=kd, a_max=A_MAX, v_max=V_MAX, i_max=1.0, i_delay=0.0, vel_fc=20.0)


def catch(pid, th0_deg, t_end=4.0):
    r = simulate(pid, math.radians(th0_deg), 0.0, t_end, PLANT, ACT, SensorParams(), SimParams(ts=TS, x_limit=5.0),
                 estimator=BdLp2(TS, pid.vel_fc))
    t, th = r["t"], np.degrees(r["theta"])
    sgn = np.sign(th0_deg)
    over = max(0.0, -sgn * th.min() if sgn > 0 else th.max()) / abs(th0_deg)
    # zero crossings of the true angle beyond a 0.15 deg band (ignores the +-1 count limit cycle)
    band = np.where(np.abs(th) > 0.15, np.sign(th), 0)
    nz = band[band != 0]
    crossings = int(np.sum(nz[1:] != nz[:-1]))
    outside = np.flatnonzero(np.abs(th) > 0.1 * abs(th0_deg))
    settle = t[outside[-1]] if len(outside) else 0.0
    late = t > t_end - 1.0
    return dict(r=r, over=over, cross=crossings, settle=settle, sat=float(np.mean(r["sat"][t < 1.0])),
                peak_a=float(np.abs(r["a_cmd"]).max()), v_end=float(np.mean(r["v"][late])), end=r["reason"])


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    print("| design | Kp | Ki | Kd | closed-loop poles (1/s) | gain margin [lo, hi] | phase margin (deg) | delay margin (ms) "
          "| RMS a from quantisation (m/s²) | 2° catch: overshoot / crossings / settle to 0.2° (s) / saturated 1st s "
          "| 4° catch: overshoot / settle / peak a | cart v after 3 s, 2° catch (mm/s) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    for i, (name, wc, zeta, p) in enumerate(CANDIDATES):
        pid = pid_for(wc, zeta, p)
        m = margins(PLANT_LIN, pid, TS, DELAY, est="lp2", est_fc=20.0)
        pm = min((abs(ph) for _, ph in m["phase_crossings"]), default=float("nan"))
        nz = noise_rms(PLANT_LIN, pid, TS, DELAY, Q / math.sqrt(12), est="lp2", est_fc=20.0)
        poles = np.roots(np.polymul([1, 2 * zeta * wc, wc * wc], [1, p]))
        ptxt = ", ".join(f"{z.real:.1f}" + (f"±{abs(z.imag):.1f}j" if abs(z.imag) > 1e-6 else "")
                         for z in poles if z.imag >= -1e-9)
        c2, c4 = catch(pid, 2.0), catch(pid, 4.0)
        print(f"| {name} | {pid.kp:.1f} | {pid.ki:.1f} | {pid.kd:.2f} | {ptxt} | [{m['k_lo']:.2f}, {m['k_hi']:.1f}] | "
              f"{pm:.0f} | {m['delay_margin'] * 1e3:.0f} | {nz:.2f} | {c2['over']:.0%} / {c2['cross']} / "
              f"{c2['settle']:.2f} / {c2['sat']:.0%} | {c4['over']:.0%} / {c4['settle']:.2f} / {c4['peak_a']:.1f} | "
              f"{c2['v_end'] * 1e3:+.0f} |", flush=True)
        r = c2["r"]
        ax[0].plot(r["t"], np.degrees(r["theta"]), lw=1, label=name)
        ax[1].plot(r["t"], r["v"] * 1e3, lw=1)
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("theta (deg), catch from 2°")
    ax[0].set_ylim(-1.0, 2.2)
    ax[0].legend(fontsize=7)
    ax[1].set_ylabel("cart v (mm/s)")
    ax[1].set_xlabel("time since ENGAGED (s)")
    for a in ax:
        a.grid(True, lw=0.3)
    ax[0].set_xlim(0, 3)
    fig.savefig("docs/img/zeta_study.png", dpi=110, bbox_inches="tight")
    print("\nplot: docs/img/zeta_study.png")


if __name__ == "__main__":
    main()
