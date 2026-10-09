"""Stage 0.11 on REAL data and the IDENTIFIED plant: velocity estimator candidates vs the D-term spikes.

    .venv/Scripts/python py/estimator_real.py [--out docs/img/estimators_real.png]

Background (shadow run 2026-10-09T13-36-33): with BD+LP1 30 Hz every single count step kicks the
estimate by ~0.36 rad/s, i.e. Kd*omega ~ +-2.4 m/s^2 command spikes. Whatever the filter, one count step
adds the same velocity Kd*q (~17 mm/s) to the cart command; the filter only shapes the pulse (tall and
short or low and wide). What the shape costs is saturation (clipping at a_max), step-loss risk (peak
acceleration) and, for a smoother estimate, phase lag (delay margin).

A. Open loop on exact-edge recordings (encoder-log), replayed through the runtime's 2 ms sampling: an
   edge is visible at a tick once it reached user space (read_ns <= tick).
     hold   10-31-06, hand holding the bob at ~34 deg (tremor, isolated count steps): Kd*omega RMS/peak
     swing  09-32-26, free decay 14 -> 5 deg: error vs a non-causal smoothing-spline derivative, lag
B. Closed loop: sim with the identified plant (w0 4.931, b/J 0.024, Coulomb 0.0117), the config gains
   (Kp 62.8, Ki 96.8, Kd 6.56, a_max 3, v_max 0.5) and the Axis cap; catch from 3 deg, 8 s:
   RMS / p99 of a_cmd after 1 s, saturated fraction, RMS theta, cart drift, tolerated extra delay.
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
from scipy.interpolate import make_smoothing_spline

from pendel.control import PidParams, design_gains
from pendel.decoders import decode
from pendel.estimators import AlphaBeta, BdLp1, BdLp2, EdgeTimestamp
from pendel.plant import ActuatorParams, PlantParams, SensorParams
from pendel.runs import load_run
from pendel.sim import SimParams, simulate

TS = 0.002
Q = 2 * math.pi / 2400
W0, BJ, COUL = 4.931, 0.024, 0.0117
KP, KI, KD = design_gains(W0, BJ, 2 * W0, 0.7, 0.5 * W0)
PID = PidParams(kp=KP, ki=KI, kd=KD, a_max=3.0, v_max=0.5, i_max=1.0, i_delay=0.1)
PLANT = PlantParams(w0=W0, bJ=BJ, coulomb=COUL)
ACT = ActuatorParams(axis_a=400000 / (2 * 13333), axis_v=120000 / (2 * 13333), compute_delay=0.0005)


def candidates() -> dict:
    return {
        "BD+LP1 30 Hz (today)": lambda: BdLp1(TS, 30.0),
        "BD+LP1 15 Hz": lambda: BdLp1(TS, 15.0),
        "BD+LP2 30 Hz": lambda: BdLp2(TS, 30.0),
        "BD+LP2 20 Hz": lambda: BdLp2(TS, 20.0),
        "BD+LP2 15 Hz": lambda: BdLp2(TS, 15.0),
        "alpha-beta 25 Hz": lambda: AlphaBeta(TS, 25.0),
        "edge-ts 10 ms/3": lambda: EdgeTimestamp(TS, window=0.01, max_edges=3),
        "edge-ts 20 ms/4": lambda: EdgeTimestamp(TS, window=0.02, max_edges=4),
        "edge-ts 10 ms/3 + LP1 40 Hz": lambda: EdgeTimestamp(TS, window=0.01, max_edges=3, fc=40.0),
    }


# ------------------------------------------------------------------------------------------- A. real data
def replay(run_dir: str):
    """Ticks every TS through an encoder-log run: (t_tick, counts, events per tick), plus exact edges."""
    r = load_run(run_dir)
    d = r.data
    o = np.argsort(d["ts_ns"], kind="stable")
    counts, steps, _ = decode(d["line"][o].astype(int), d["type"][o].astype(int),
                              int(r.meta["start_level_a"]), int(r.meta["start_level_b"]))
    real = steps != 0
    t0 = d["ts_ns"][o][0]
    ts = (d["ts_ns"][o][real] - t0) * 1e-9
    rd = (d["read_ns"][o][real] - t0) * 1e-9
    c = counts[real]
    u = np.where(steps[real] > 0, c, c + 1).astype(float)       # continuous position at the edge
    ticks = np.arange(0.0, rd[-1] + 0.1, TS)
    idx = np.searchsorted(rd, ticks, side="right")             # edges visible at each tick
    cnt = np.where(idx > 0, c[np.maximum(idx - 1, 0)], 0)
    return ticks, cnt, idx, ts, c, u


def run_est(est, ticks, cnt, idx, ts, c):
    out = np.zeros(len(ticks))
    prev = 0
    for k, t in enumerate(ticks):
        ev = list(zip(ts[prev:idx[k]], c[prev:idx[k]]))
        prev = idx[k]
        out[k] = est.update(t, int(cnt[k]), ev)
    return out


def open_loop(names):
    res = {n: {} for n in names}
    # hold: hand on the bob, 10-31-06 t = 2.4 .. 4.4 s after the first edge
    ticks, cnt, idx, ts, c, u = replay("logs/2026-10-09T10-31-06_encoder-log")
    m = (ticks > 2.4) & (ticks < 4.4)
    for n in names:
        w = run_est(candidates()[n](), ticks, cnt, idx, ts, c)
        res[n]["hold_rms"] = KD * np.sqrt(np.mean(w[m] ** 2))
        res[n]["hold_peak"] = KD * np.abs(w[m]).max()
    # swing: free decay 09-32-26, 500..560 s (14 -> 5 deg)
    ticks, cnt, idx, ts, c, u = replay("logs/2026-10-09T09-32-26_encoder-log")
    sel = (ts > 495) & (ts < 565)
    spl = make_smoothing_spline(ts[sel], u[sel] * Q, lam=1e-7)
    truth_fn = spl.derivative()
    m = (ticks > 500) & (ticks < 560)
    tr = truth_fn(ticks[m])
    res["_swing_truth"] = (ticks[m], tr)
    for n in names:
        w = run_est(candidates()[n](), ticks, cnt, idx, ts, c)[m]
        res[n]["swing_err"] = np.sqrt(np.mean((w - tr) ** 2))
        # lag: shift (in 0.1 ms) that best aligns the estimate with the truth
        shifts = np.arange(0, 0.03, 1e-4)
        errs = [np.mean((np.interp(ticks[m] - s, ticks[m], tr) - w) ** 2) for s in shifts]
        res[n]["lag_ms"] = shifts[int(np.argmin(errs))] * 1e3
        res[n]["_swing"] = w
    return res


# ------------------------------------------------------------------------------------------ B. closed loop
def closed_loop(names, res):
    for n in names:
        r = simulate(PID, math.radians(3), 0.0, 8.0, PLANT, ACT, SensorParams(),
                     SimParams(ts=TS, x_limit=5.0), estimator=candidates()[n]())
        late = r["t"] > 1.0
        a = r["a_cmd"][late]
        d = res[n]
        d["cl_end"] = r["reason"]
        d["cl_arms"] = np.sqrt(np.mean(a ** 2))
        d["cl_a99"] = np.percentile(np.abs(a), 99)
        d["cl_sat"] = np.mean(r["sat"][late])
        d["cl_thrms"] = np.degrees(np.sqrt(np.mean(r["theta"][late] ** 2)))
        d["cl_x"] = r["x"][~np.isnan(r["x"])][-1]
        d["_cl"] = r

        def holds(nd, n=n):
            rr = simulate(PID, math.radians(3), 0.0, 4.0, PLANT, ACT, SensorParams(),
                          SimParams(ts=TS, x_limit=5.0, delay_samples=nd), estimator=candidates()[n]())
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


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/img/estimators_real.png")
    args = ap.parse_args()
    names = list(candidates())
    print(f"gains: Kp {KP:.2f}  Ki {KI:.2f}  Kd {KD:.3f}  (one count step through Kd: {KD * Q * 1e3:.1f} mm/s)")
    res = open_loop(names)
    closed_loop(names, res)
    print("| estimator | hold: Kd·ω RMS / peak (m/s²) | swing: err RMS (rad/s), lag (ms) | closed loop: "
          "RMS a / p99 a (m/s²), sat, RMS θ (°), x(8 s) (m) | tolerated extra delay (ms) |")
    print("|---|---|---|---|---|")
    for n in names:
        d = res[n]
        print(f"| {n} | {d['hold_rms']:.2f} / {d['hold_peak']:.2f} | {d['swing_err']:.3f}, {d['lag_ms']:.1f} | "
              f"{d['cl_end']}: {d['cl_arms']:.2f} / {d['cl_a99']:.2f}, {d['cl_sat']:.1%}, {d['cl_thrms']:.3f}, "
              f"{d['cl_x']:+.3f} | {d['dm']:.0f} |")

    fig, ax = plt.subplots(3, 1, figsize=(12, 9))
    show = ["BD+LP1 30 Hz (today)", "BD+LP2 20 Hz", "edge-ts 10 ms/3"]
    tt, tr = res["_swing_truth"]
    m = (tt > 540) & (tt < 542)
    ax[0].plot(tt[m], tr[m], "k", lw=2, label="truth (spline)")
    for n in show:
        ax[0].plot(tt[m], res[n]["_swing"][m], lw=0.8, label=n)
    ax[0].set_ylabel("omega (rad/s), free swing")
    ax[0].legend(fontsize=8)
    for n in show:
        r = res[n]["_cl"]
        m = (r["t"] > 3) & (r["t"] < 5)
        ax[1].plot(r["t"][m], r["a_cmd"][m], lw=0.7, label=n)
        ax[2].plot(r["t"][m], np.degrees(r["theta"][m]), lw=0.7, label=n)
    ax[1].set_ylabel("closed loop a_cmd (m/s^2)")
    ax[2].set_ylabel("closed loop theta (deg)")
    ax[2].set_xlabel("t (s)")
    ax[1].legend(fontsize=8)
    for x in ax:
        x.grid(True)
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"plot: {args.out}")


if __name__ == "__main__":
    main()
