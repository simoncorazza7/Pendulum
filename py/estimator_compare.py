"""Stage 0.11: compare velocity estimator candidates on SYNTHETIC encoder data (until real logs exist).

    .venv/Scripts/python py/estimator_compare.py [--out docs/img/estimators.png]

Open loop (synthetic encoder, 2400 counts/rev, 2 us timestamp jitter, 60 us read latency):
  rest      micro-vibration across one count edge (true velocity tiny): output RMS = noise at rest
  slow      constant 0.02 / 0.1 / 0.3 rad/s: RMS error
  sine      3 deg at 1, 2, 3, 5 Hz: gain and phase lag of the estimate vs the true velocity
  swing     decaying 1 Hz swing from 10 deg: RMS error
Closed loop (sim with the placeholder plant and the preview PID gains, catch from 3 deg, 6 s):
  RMS command acceleration after the catch (chatter), RMS angle, drift.
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
from pendel.encsim import encode, run
from pendel.estimators import candidates
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.sim import SimParams, simulate

TS = 0.002
Q = 2 * math.pi / 2400
FREQS = (1.0, 2.0, 3.0, 5.0)


def fine(t_end, fn):
    t = np.arange(0, t_end, 5e-6)
    return t, fn(t)


def fit(t, y, f):
    """Complex amplitude of y at frequency f (least squares on sin/cos)."""
    M = np.column_stack([np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(M, y, rcond=None)
    return complex(c[0], -c[1]), y - M @ c


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/img/estimators.png")
    args = ap.parse_args()
    names = list(candidates(TS))
    res = {n: {} for n in names}

    # rest: mean exactly on a count edge (phase 0.37), vibration 0.15 + 0.05 count at 25 + 71 Hz
    edge = (5 - 0.37) * Q
    enc = encode(*fine(4.0, lambda t: edge + 0.15 * Q * np.sin(2 * np.pi * 25 * t) + 0.05 * Q * np.sin(2 * np.pi * 71 * t)))
    true_rest = np.sqrt(np.mean(enc["vel"] ** 2))
    for n in names:
        v = run(candidates(TS)[n], enc)
        res[n]["rest"] = np.sqrt(np.mean(v[250:] ** 2))

    # slow constant velocity
    for w in (0.02, 0.1, 0.3):
        enc = encode(*fine(3.0, lambda t, w=w: w * t))
        for n in names:
            v = run(candidates(TS)[n], enc)
            res[n][f"slow{w}"] = np.sqrt(np.mean((v[250:] - w) ** 2))

    # sine: gain and phase lag
    for f in FREQS:
        A = math.radians(3)
        enc = encode(*fine(4.0, lambda t, f=f: A * np.sin(2 * np.pi * f * t)))
        sl = slice(250, None)
        tv, _ = fit(enc["t"][sl], enc["vel"][sl], f)
        for n in names:
            v = run(candidates(TS)[n], enc)
            ev, resid = fit(enc["t"][sl], v[sl], f)
            h = ev / tv
            lag = -math.degrees(math.atan2(h.imag, h.real))
            res[n][f"gain{f}"] = abs(h)
            res[n][f"lag{f}"] = lag
            res[n][f"lagms{f}"] = lag / 360 / f * 1e3
            res[n][f"resid{f}"] = np.sqrt(np.mean(resid ** 2))

    # free swing
    enc = encode(*fine(8.0, lambda t: math.radians(10) * np.exp(-0.05 * t) * np.cos(2 * np.pi * t)))
    for n in names:
        v = run(candidates(TS)[n], enc)
        res[n]["swing"] = np.sqrt(np.mean((v[250:] - enc["vel"][250:]) ** 2))

    # closed loop with the preview PID (placeholder plant)
    w0 = 2 * math.pi
    kp, ki, kd = design_gains(w0, 0.1, 2 * w0, 0.7, 0.5 * w0)
    pid = PidParams(kp=kp, ki=ki, kd=kd, a_max=5.0, v_max=0.8)
    for n in names + ["built-in (BD+LP1 30 Hz in pid_step)"]:
        est = candidates(TS).get(n)
        r = simulate(pid, math.radians(3), 0.0, 6.0, PlantParams(w0=w0, bJ=0.1), ActuatorParams(), SensorParams(),
                     SimParams(ts=TS, x_limit=0.5), estimator=est)
        late = r["t"] > 1.0
        d = res.setdefault(n, {})
        d["cl_ok"] = r["reason"]
        d["cl_arms"] = np.sqrt(np.mean(r["a_cmd"][late] ** 2)) if late.any() else float("nan")
        d["cl_threms"] = np.degrees(np.sqrt(np.mean(r["theta"][late] ** 2))) if late.any() else float("nan")
        d["cl_x"] = r["x"][-1]
        # extra delay (whole samples) the loop tolerates: catch from 3 deg must stay below 1 deg RMS
        # over the last second of 4 s
        def holds(nd, n=n):
            rr = simulate(pid, math.radians(3), 0.0, 4.0, PlantParams(w0=w0, bJ=0.1), ActuatorParams(),
                          SensorParams(), SimParams(ts=TS, x_limit=5.0, delay_samples=nd),
                          estimator=candidates(TS).get(n))
            tail = rr["t"] > 3.0
            return rr["reason"] == "time" and np.degrees(np.sqrt(np.mean(rr["theta"][tail] ** 2))) < 1.0
        lo, hi = 0, 40
        if not holds(lo):
            d["dm"] = 0.0
        else:
            while hi - lo > 1:
                mid = (lo + hi) // 2
                lo, hi = (mid, hi) if holds(mid) else (lo, mid)
            d["dm"] = lo * TS * 1e3

    print(f"true velocity RMS in the rest case: {true_rest:.4f} rad/s (micro-vibration across one edge)\n")
    print("| estimator | rest RMS (rad/s) | slow 0.02/0.1/0.3 RMS err (rad/s) | lag 1/2/3/5 Hz (deg) | "
          "lag (ms) @2 Hz | gain @5 Hz | swing RMS err | closed loop: end, RMS a (m/s²), RMS θ (°) | "
          "tolerated extra delay (ms) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for n in names:
        d = res[n]
        print(f"| {n} | {d['rest']:.3f} | {d['slow0.02']:.3f} / {d['slow0.1']:.3f} / {d['slow0.3']:.3f} | "
              + " / ".join(f"{d[f'lag{f}']:.1f}" for f in FREQS)
              + f" | {d['lagms2.0']:.1f} | {d['gain5.0']:.2f} | {d['swing']:.3f} | "
              f"{d['cl_ok']}, {d['cl_arms']:.2f}, {d['cl_threms']:.3f} | {d['dm']:.0f} |")
    b = res["built-in (BD+LP1 30 Hz in pid_step)"]
    print(f"| (built-in, closed loop only) | | | | | | | {b['cl_ok']}, {b['cl_arms']:.2f}, {b['cl_threms']:.3f} "
          f"| {b['dm']:.0f} |")

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    for n in names:
        axes[0].plot(FREQS, [res[n][f"lag{f}"] for f in FREQS], "o-", label=n)
    axes[0].set(xlabel="f (Hz)", ylabel="phase lag (deg)", title="phase lag vs true velocity")
    axes[1].barh(names, [res[n]["rest"] for n in names])
    axes[1].set(xlabel="rad/s", title="output RMS at rest (dither across one edge)")
    axes[2].barh(names, [res[n]["cl_arms"] for n in names])
    axes[2].set(xlabel="m/s²", title="closed loop: RMS a_cmd after catch")
    axes[0].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("SYNTHETIC encoder data and PLACEHOLDER plant (re-run on real logs)", fontsize=10)
    fig.tight_layout()
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"\nfigure -> {out}")


if __name__ == "__main__":
    main()
