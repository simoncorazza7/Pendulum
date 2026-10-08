"""Stage 1 design preview on PLACEHOLDER plant parameters (rerun with measured values at Gate B).

    .venv/Scripts/python py/design_preview.py [--w0 6.28] [--bJ 0.1] [--ts 0.002] [--delay 0.0005]
Prints PD vs PID gains, margins (lower/upper gain, delay, phase), Nyquist check, RMS
acceleration from quantisation noise, and simulated catches; saves docs/img/design_preview.png.
"""

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel.control import PidParams, design_gains
from pendel.linear import build_loop, margins, noise_rms
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.sim import SimParams, simulate


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--w0", type=float, default=2 * math.pi, help="rad/s (PLACEHOLDER: 1 s hanging period)")
    ap.add_argument("--bJ", type=float, default=0.1, help="1/s (PLACEHOLDER)")
    ap.add_argument("--ts", type=float, default=0.002)
    ap.add_argument("--delay", type=float, default=0.0005, help="s, sample -> accel takes effect")
    ap.add_argument("--wc", type=float, default=2.0, help="omega_c / omega_0")
    ap.add_argument("--zeta", type=float, default=0.7)
    ap.add_argument("--p", type=float, default=0.5, help="PID real pole, in units of omega_0")
    ap.add_argument("--out", default="docs/img/design_preview.png")
    a = ap.parse_args()

    plant = PlantParams(w0=a.w0, bJ=a.bJ)
    act = ActuatorParams(compute_delay=a.delay)
    q = 2 * math.pi / 2400
    base = dict(a_max=5.0, v_max=0.8, vel_fc=30.0)
    designs = {}
    for name, p in (("PD", 0.0), ("PID", a.p * a.w0)):
        kp, ki, kd = design_gains(a.w0, a.bJ, a.wc * a.w0, a.zeta, p)
        designs[name] = PidParams(kp=kp, ki=ki, kd=kd, **base)

    print(f"PLACEHOLDER plant: w0 = {a.w0:.3f} rad/s (T_hang = {2 * math.pi / a.w0:.3f} s, "
          f"L_eq = {9.81 / a.w0**2:.3f} m), b/J = {a.bJ} 1/s; Ts = {a.ts * 1e3:.1f} ms, delay = {a.delay * 1e3:.2f} ms")
    print(f"target: omega_c = {a.wc:.2f} w0, zeta = {a.zeta}, p = {a.p:.2f} w0 (PID)\n")
    print("| | Kp (m/s²/rad) | Ki (m/s²/(rad·s)) | Kd (m/s²/(rad/s)) | gain range k | delay margin (ms) "
          "| phase margin(s) | Nyquist CCW / OL unstable | RMS a from quantisation (m/s²) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for name, pid in designs.items():
        m = margins(plant, pid, a.ts, a.delay)
        pm = ", ".join(f"{pm_:.0f}° @ {w_ / (2 * math.pi):.1f} Hz" for w_, pm_ in m.get("phase_crossings", []))
        rms = noise_rms(plant, pid, a.ts, a.delay, q / math.sqrt(12))
        print(f"| {name} | {pid.kp:.1f} | {pid.ki:.1f} | {pid.kd:.2f} | {m['k_lo']:.2f} – {m['k_hi']:.2f} | "
              f"{m['delay_margin'] * 1e3:.1f} | {pm} | {m['nyquist_ccw']} / {m['ol_unstable_poles']} | {rms:.2f} |")

    # Catch simulations from 3 deg and 8 deg, PD vs PID.
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    sp = SimParams(ts=a.ts, x_limit=0.3)
    print("\ncatches (cart starts at rest; ends at |x| = 0.3 m or 6 s):")
    for name, pid in designs.items():
        for th0_deg in (3.0, 8.0):
            r = simulate(pid, math.radians(th0_deg), 0.0, 6.0, plant, act, SensorParams(), sp)
            t = r["t"]
            ok = r["reason"] != "theta_abort"
            settle = t[np.argmax(np.abs(np.degrees(r["theta"])) < 3.0)] if ok else float("nan")
            print(f"  {name} {th0_deg:.0f}°: end={r['reason']} at {t[-1]:.2f} s, |theta|<3° after {settle:.3f} s, "
                  f"peak |a| {np.max(np.abs(r['a_cmd'])):.2f} m/s², sat {100 * np.mean(r['sat']):.0f} %, "
                  f"v_end {r['v'][-1]:+.3f} m/s")
            lbl = f"{name} {th0_deg:.0f}°"
            axes[0].plot(t, np.degrees(r["theta"]), lw=0.8, label=lbl)
            axes[1].plot(t, r["a_cmd"], lw=0.6, label=lbl)
            axes[2].plot(t, r["v"], lw=0.8, label=lbl)
            axes[3].plot(t, r["x"], lw=0.8, label=lbl)
    for ax, yl in zip(axes, ("theta (deg)", "a_cmd (m/s²)", "cart v (m/s)", "cart x (m)")):
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8, ncol=4)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("PLACEHOLDER plant parameters (not measured yet)", fontsize=10)
    fig.tight_layout()
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"\nfigure -> {out}")


if __name__ == "__main__":
    main()
