"""Swing-up runs of a balance session (balance.swingup = true): pumping, hand-over state, how the balance ended.

    .venv/Scripts/python py/swingup_report.py [RUN]

Per run: pumping time and the cart's range while pumping; the state at the hand-over (theta, theta', the
unstable-mode content theta + theta'/w0, energy, cart x and v, and the soft-limit margin x_max - |x| - stopping
distance); the balance phase (duration, how it ended, x and v at the end); the energy removal. Plot: energy,
theta, x and v against time for every run, saved as swingup_runs.png in the run directory.
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

from pendel.runs import latest_run, load_run

PHASES = {0: "KICK", 1: "PUMP", 2: "BALANCE", 3: "BRAKE", 4: "DAMP_OUT", 5: "STOPPING", 6: "DONE"}
DEG = 180 / math.pi


def stop_dist(v, a_brake=1.0, t_react=0.006, margin=0.01):
    return v * v / (2 * a_brake) + abs(v) * t_react + margin


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    run = load_run(sys.argv[1] if len(sys.argv) > 1 else latest_run("logs", "balance"))
    d = run.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    w0 = float(run.config["plant.w0"])
    x_max = float(run.config["sup.x_max"])
    sp = d["swing_phase"]
    print(f"{run.path.name}: {run.meta['result']}; runs: {run.meta['attempts']}")
    print("\n| run | pumping (s) | x range pumping (mm) | at hand-over: theta / theta' / u (deg, deg/s, deg) | energy | "
          "x (mm) / v (mm/s) | soft-limit margin (mm) | balanced (s) | end: x (mm) / v (mm/s) | energy out (s) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    for n in sorted(set(d["attempt"][sp >= 0])):
        m = (d["attempt"] == n) & (sp >= 0)
        idx = np.flatnonzero(m)
        if len(idx) < 10:
            continue
        i0 = idx[0]
        tt = t[idx] - t[i0]
        ph = sp[idx]
        pump = np.isin(ph, (0, 1))
        bal = ph == 2
        out = ph == 4
        xs, vs = d["x"][idx], d["v"][idx]
        row = f"| {n} | {tt[pump][-1]:.1f} | {xs[pump].min() * 1e3:+.0f} .. {xs[pump].max() * 1e3:+.0f} | "
        if bal.any():
            h = np.flatnonzero(bal)[0]
            e = np.flatnonzero(bal)[-1]
            th, om = d["theta"][idx][h], d["omega"][idx][h]
            x, v = xs[h], vs[h]
            margin = x_max - abs(x) - (stop_dist(v) if x * v > 0 else 0.0)
            row += (f"{th * DEG:+.1f} / {om * DEG:+.0f} / {(th + om / w0) * DEG:+.1f} | {d['energy'][idx][h]:.4f} | "
                    f"{x * 1e3:+.0f} / {v * 1e3:+.0f} | {margin * 1e3:+.0f} | {tt[e] - tt[h]:.2f} | "
                    f"{xs[e] * 1e3:+.0f} / {vs[e] * 1e3:+.0f} | ")
        else:
            row += "no hand-over | | | | | | "
        row += f"{(tt[out][-1] - tt[out][0]) if out.any() else 0:.1f} |"
        print(row)
        ax[0].plot(tt, d["energy"][idx], lw=0.8, label=f"run {n}")
        ax[1].plot(tt, d["theta"][idx] * DEG, lw=0.6)
        ax[2].plot(tt, xs * 1e3, lw=0.8)
        ax[3].plot(tt, vs * 1e3, lw=0.8)
    ax[0].set_ylabel("energy (1 = upright)")
    ax[0].axhline(1.0, color="k", lw=0.5)
    ax[0].legend(fontsize=7, ncol=7)
    ax[1].set_ylabel("theta (deg from upright)")
    ax[2].set_ylabel("cart x (mm)")
    ax[2].axhline(x_max * 1e3, color="r", lw=0.6)
    ax[2].axhline(-x_max * 1e3, color="r", lw=0.6)
    ax[3].set_ylabel("cart v (mm/s)")
    ax[3].set_xlabel("time since the swing-up started (s)")
    for a in ax:
        a.grid(True, lw=0.3)
    outp = run.path / "swingup_runs.png"
    fig.savefig(outp, dpi=110, bbox_inches="tight")
    print(f"\nplot: {outp}")


if __name__ == "__main__":
    main()
