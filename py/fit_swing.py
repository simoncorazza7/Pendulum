"""Fit free swings recorded with `pendctl --mode encoder-log` (Stage 0.7 and 0.10).

    .venv/Scripts/python py/fit_swing.py [RUN_DIR] [--t0 S] [--t1 S] [--decoder type|toggle]

Uses the exact edge crossings (py/pendel/decoders.py) and prints:
  - c_bottom (counts, relative to the recording's start count) from a damped-sine fit with offset,
    with its standard error (0.7 upright reference: theta = wrap((count + 0.5 - c_bottom - CPR/2) 2pi/CPR)),
  - damped frequency and decay; windowed amplitude/frequency; w0 extrapolated to zero amplitude,
  - decay envelope: exponential (viscous) vs linear (Coulomb) vs both, with b/J, Coulomb gamma, L_eq.
Saves swingfit.png in the run directory. Select one swing with --t0/--t1 (s from the first edge).
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

from pendel import latest_run, load_run
from pendel.decoders import edges_from_run
from pendel.swingfit import envelope_models, fit_damped, fit_offset_linear, omega0_from_windows, windowed

G = 9.81


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?")
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=math.inf)
    ap.add_argument("--decoder", choices=("type", "toggle"), default="type")
    ap.add_argument("--cpr", type=int, default=2400)
    a = ap.parse_args()

    run = load_run(a.run or latest_run(mode="encoder-log"))
    t, u = edges_from_run(run, use_type=a.decoder == "type")
    m = (t >= a.t0) & (t <= a.t1)
    t, u = t[m] - t[m][0], u[m]
    q = 2 * math.pi / a.cpr
    print(f"run {run.path.name}: {len(t)} edges over {t[-1]:.1f} s ({a.decoder} decoder)")

    g = fit_damped(t, u)
    c_lin, se_lin = fit_offset_linear(t, u, g.w, g.sigma)
    print(f"\ndamped fit  u = c + A e^(-sigma t) cos(w t + phi):")
    print(f"  c_bottom = {g.c:.3f} ± {g.se['c']:.3f} counts   (linear re-fit {c_lin:.3f} ± {se_lin:.3f})")
    print(f"  w = {g.w:.4f} ± {g.se['w']:.4f} rad/s  (T = {2 * math.pi / g.w:.4f} s), sigma = {g.sigma:.4f} ± "
          f"{g.se['sigma']:.4f} 1/s, A0 = {g.A * q * 180 / math.pi:.2f}°, residual {g.rms:.3f} counts")

    win = windowed(t, u)
    if len(win["t"]) >= 4:
        w0, slope = omega0_from_windows(win["A"], win["w"], a.cpr)
        env = envelope_models(win["t"], win["A"] * q)
        best = min(("exp", "lin", "both"), key=lambda k: env[k]["rms"])
        print(f"\nwindowed: {len(win['t'])} windows, amplitude {win['A'].max() * q * 180 / math.pi:.1f}° → "
              f"{win['A'].min() * q * 180 / math.pi:.1f}°")
        print(f"  w0 (A → 0) = {w0:.4f} rad/s, T0 = {2 * math.pi / w0:.4f} s, L_eq = g/w0² = {G / w0**2 * 1e3:.1f} mm; "
              f"A² coefficient {slope:.4f} (rigid pendulum: 0.0625)")
        print(f"  envelope rms (rad): exp {env['exp']['rms']:.2e}, lin {env['lin']['rms']:.2e}, both "
              f"{env['both']['rms']:.2e} → best: {best}")
        s, k = env["both"]["sigma"], env["both"]["k"]
        print(f"  combined: b/J = 2 sigma = {2 * s:.4f} 1/s (zeta_hang = {s / w0:.4f}); Coulomb gamma = "
              f"pi w0 k / 2 = {math.pi * w0 * k / 2:.4f} rad/s² (stiction band ±{math.pi * k / (2 * w0) / q:.2f} counts)")
        drift = win["c"].max() - win["c"].min()
        c_win = float(np.median(win["c"]))
        print(f"  window offsets c: {win['c'].min():.2f} … {win['c'].max():.2f} counts (spread {drift:.2f}; "
              f"a trend means lost counts); median {c_win:.3f}")
    if g.rms > 1.0:
        print(f"\nNOTE: the single damped fit leaves {g.rms:.1f} counts RMS (large swing: amplitude-dependent "
              f"frequency, non-exponential decay), so its c_bottom is biased. Use a small swing (2-5°) for 0.7, or the "
              f"windowed median above.")
    else:
        win, env = None, None
        print("\n(too short for windowed analysis)")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    tt = np.linspace(0, t[-1], 4000)
    axes[0].plot(t, u, ".", ms=1.5, label="edges")
    axes[0].plot(tt, g.c + g.A * np.exp(-g.sigma * tt) * np.cos(g.w * tt + g.phi), lw=0.7, label="damped fit")
    axes[0].axhline(g.c, color="k", lw=0.5)
    axes[0].set(ylabel="u (counts)", title=run.path.name)
    axes[0].legend(fontsize=8)
    resid = u - (g.c + g.A * np.exp(-g.sigma * t) * np.cos(g.w * t + g.phi))
    axes[1].plot(t, resid, ".", ms=1.5)
    axes[1].set(ylabel="residual (counts)")
    if win is not None:
        axes[2].plot(win["t"], win["A"] * q * 180 / math.pi, "o", ms=3, label="window amplitude")
        for key, style in (("exp", "--"), ("lin", ":")):
            e = env[key]
            fit = e["A0"] * np.exp(-e["sigma"] * win["t"]) if key == "exp" else e["A0"] - e["k"] * win["t"]
            axes[2].plot(win["t"], fit * 180 / math.pi, style, label=key)
        axes[2].set(xlabel="t (s)", ylabel="amplitude (°)")
        axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run.path / "swingfit.png", dpi=110)
    print(f"\nfigure -> {run.path / 'swingfit.png'}")


if __name__ == "__main__":
    main()
