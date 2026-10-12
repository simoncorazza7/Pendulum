"""Gate B design comparison (brief §6 "Gain design"): candidate tunings on the identified plant.

    .venv/Scripts/python py/gateb_design.py

Every candidate: Ts 2 ms, compute delay 0.5 ms, BD+LP2 20 Hz velocity (0.11), pole placement
(s^2 + 2 zeta wc s + wc^2)(s + p) with zeta 0.7. Linear (pendel.linear, exact ZOH, LP2 state model):
gains, gain margins [k_lo, k_hi], phase margin, delay margin, Nyquist check, RMS command from
quantisation noise (q/sqrt(12)). Nonlinear (pendel.sim, validated in docs/stage0_12_sim.md): 3 deg
catch (RMS / p99 command after 1 s, saturated fraction, RMS theta) and the catch region over
(theta0, theta'0) with a_max 3 m/s^2, v_max 0.5 m/s (PRIOR until 0.9).
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel.control import PidParams, design_gains
from pendel.estimators import BdLp2
from pendel.linear import build_loop, margins, noise_rms
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.sim import SimParams, simulate

TS, DELAY, G = 0.002, 0.0005, 9.81
W0, BJ, COUL = 4.931, 0.024, 0.0117
Q = 2 * math.pi / 2400
PLANT_LIN = PlantParams(w0=W0, bJ=BJ)
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=DELAY)
A_MAX, V_MAX = 3.0, 0.5

CANDIDATES = [("PID wc=1.5 w0", 1.5, 0.5), ("PID wc=2 w0 (config)", 2.0, 0.5), ("PID wc=3 w0", 3.0, 0.5),
              ("PD wc=2 w0", 2.0, 0.0)]


def pid_for(wc_f, p_f):
    kp, ki, kd = design_gains(W0, BJ, wc_f * W0, 0.7, p_f * W0)
    return PidParams(kp=kp, ki=ki, kd=kd, a_max=A_MAX, v_max=V_MAX, i_max=1.0, i_delay=0.0, vel_fc=20.0)


def catch(pid, th0, thd0, t_end=3.0):
    r = simulate(pid, th0, thd0, t_end, PLANT, ACT, SensorParams(), SimParams(ts=TS, x_limit=5.0),
                 estimator=BdLp2(TS, 20.0))
    ok = r["reason"] == "time"
    tail = r["t"] > t_end - 0.5
    return ok and np.degrees(np.nanmax(np.abs(r["theta"][tail]))) < 1.0, r


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    rows, regions = [], {}
    th_grid = np.radians(np.arange(0.5, 10.01, 0.5))
    thd_grid = np.array([-0.5, -0.25, 0.0, 0.25, 0.5])           # rad/s, + = away from upright (same sign as theta)
    for name, wc_f, p_f in CANDIDATES:
        pid = pid_for(wc_f, p_f)
        m = margins(PLANT_LIN, pid, TS, DELAY, est="lp2", est_fc=20.0)
        pm = min((abs(p) for _, p in m["phase_crossings"]), default=float("nan"))
        wcross = m["phase_crossings"][0][0] if m["phase_crossings"] else float("nan")
        nz = noise_rms(PLANT_LIN, pid, TS, DELAY, Q / math.sqrt(12), est="lp2", est_fc=20.0)
        ok3, r = catch(pid, math.radians(3), 0.0, 6.0)
        late = r["t"] > 1.0
        a = r["a_cmd"][late]
        reg = np.array([[catch(pid, th, thd)[0] for th in th_grid] for thd in thd_grid])
        regions[name] = reg
        max_static = th_grid[reg[2]].max() if reg[2].any() else 0.0
        rows.append((name, pid, m, pm, wcross, nz, ok3, a, r, late, max_static, reg))
        print(f"{name}: done", flush=True)

    print("\n| design | Kp | Ki | Kd | gain margin [k_lo, k_hi] | phase margin (deg) @ rad/s | delay margin (ms) | "
          "Nyquist CCW / OL unstable | RMS a from quantisation (linear) | 3° catch: RMS a / p99 / sat / RMS θ | "
          "largest caught θ0 at rest | catch-region cells |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name, pid, m, pm, wcross, nz, ok3, a, r, late, max_static, reg in rows:
        print(f"| {name} | {pid.kp:.1f} | {pid.ki:.1f} | {pid.kd:.2f} | [{m['k_lo']:.2f}, {m['k_hi']:.1f}] | "
              f"{pm:.0f} @ {wcross:.1f} | {m['delay_margin'] * 1e3:.0f} | {m['nyquist_ccw']} / {m['ol_unstable_poles']} | "
              f"{nz:.2f} | {np.sqrt(np.mean(a ** 2)):.2f} / {np.percentile(np.abs(a), 99):.2f} / "
              f"{np.mean(r['sat'][late]):.0%} / {np.degrees(np.sqrt(np.mean(r['theta'][late] ** 2))):.3f} | "
              f"{math.degrees(max_static):.1f}° | {reg.sum()}/{reg.size} |")

    # drift vs reference error (constant offset delta of the true balance angle)
    print("\n| reference error | PID: a_d = g δ (mm/s²), t to 0.15 m | PD: a_d = g δ Kp/(Kp - g) (mm/s²), t to 0.15 m |")
    print("|---|---|---|")
    kp_pd = pid_for(2.0, 0.0).kp
    for counts in (0.25, 0.5, 1.0):
        d = counts * Q
        a_pid, a_pd = G * d, G * d * kp_pd / (kp_pd - G)
        print(f"| {counts} count ({d * 1e3:.2f} mrad) | {a_pid * 1e3:.1f}, {math.sqrt(0.3 / a_pid):.1f} s | "
              f"{a_pd * 1e3:.1f}, {math.sqrt(0.3 / a_pd):.1f} s |")

    # figures: catch region (config design) and Nyquist (config design)
    fig, ax = plt.subplots(1, len(CANDIDATES), figsize=(14, 3.2), sharey=True)
    for i, (name, *_rest) in enumerate(rows):
        reg = regions[name]
        ax[i].imshow(reg, origin="lower", aspect="auto", cmap="Greens", vmin=0, vmax=1.3,
                     extent=[0.25, 10.25, -0.625, 0.625])
        ax[i].set_title(name, fontsize=9)
        ax[i].set_xlabel("θ0 (deg)")
    ax[0].set_ylabel("θ'0 (rad/s), + away from upright")
    fig.savefig("docs/img/gateb_catch_region.png", dpi=110, bbox_inches="tight")
    loop = build_loop(PLANT_LIN, pid_for(2.0, 0.5), TS, DELAY, eps_int=1e-9, est="lp2", est_fc=20.0)
    w = np.concatenate([-np.logspace(math.log10(math.pi / TS * 0.999), -2, 3000), np.logspace(-2, math.log10(math.pi / TS * 0.999), 3000)])
    L = loop.L(w)
    fig2, ax2 = plt.subplots(figsize=(5.5, 5))
    ax2.plot(L.real, L.imag, lw=0.8)
    ax2.plot([-1], [0], "r+", ms=12)
    ax2.set_xlim(-3, 1.5)
    ax2.set_ylim(-2.5, 2.5)
    ax2.set_title("Nyquist L(e^{jωT}), PID ωc = 2ω0, LP2 20 Hz", fontsize=9)
    ax2.grid(True)
    fig2.savefig("docs/img/gateb_nyquist.png", dpi=110, bbox_inches="tight")
    print("\nplots: docs/img/gateb_catch_region.png, docs/img/gateb_nyquist.png")


if __name__ == "__main__":
    main()
