"""Replay table for the Stage 0.9 actuator test (pendctl --mode actuator, profile table): a rig run's commanded
acceleration as a bounded reference trajectory for the cart.

    python py/actuator_replay.py RUN --t0 T0 --t1 T1 --amax A [--vmax V] [--xmax X] [--out FILE]

Takes a_cmd of RUN (a balance-session data.csv: hand catches or swing-ups) from T0 to T1 s after its first record,
scales it so its peak becomes A, and removes its slow part (below ~0.3 Hz, the cart drift a balancing loop would
correct with its own cart feedback) so the cart stays near the centre. Both ends are tapered to rest. If the result
needs more than V (m/s) or X (m), everything is scaled down until it fits, and the peak acceleration printed is the
one the table really has. Writes t,x,v,a every loop.ts (actuator.table); run it with
    ./actuator.sh --set actuator.profile=table --set actuator.table=FILE
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

sys.path.insert(0, str(Path(__file__).parent))
from pendel.runs import load_run  # noqa: E402


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--t0", type=float, required=True, help="s from the run's first record")
    ap.add_argument("--t1", type=float, required=True)
    ap.add_argument("--amax", type=float, required=True, help="m/s^2, peak acceleration of the table")
    ap.add_argument("--vmax", type=float, default=0.45, help="m/s, speed bound (default 0.45)")
    ap.add_argument("--xmax", type=float, default=0.10, help="m, position bound (default 0.10)")
    ap.add_argument("--fc", type=float, default=0.3, help="Hz, slower content is removed (default 0.3)")
    ap.add_argument("--taper", type=float, default=1.0, help="s, cosine ramp at each end (default 1.0)")
    ap.add_argument("--out", default="build/actuator_replay.csv")
    a = ap.parse_args()

    run = load_run(a.run)
    d = run.data
    ts = run.cfg("loop.ts")
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    sel = (t >= a.t0) & (t <= a.t1)
    acc = d["a_cmd"][sel].astype(float)
    n = len(acc)
    if n < int(4 * a.taper / ts):
        sys.exit("window too short for the tapers")
    peak = np.max(np.abs(acc))
    if peak <= 0:
        sys.exit("a_cmd is zero in that window")
    acc *= a.amax / peak

    # Exact double integration of the piecewise-constant command (as the Axis integrates it).
    v = np.concatenate(([0.0], np.cumsum(acc[:-1]) * ts))
    x = np.concatenate(([0.0], np.cumsum(v[:-1] * ts + 0.5 * acc[:-1] * ts * ts)))
    # Remove the slow part: x - lowpass(x), with the trend's own derivatives taken off v and a as well.
    trend = sosfiltfilt(butter(2, a.fc, fs=1 / ts, output="sos"), x)
    dtrend = np.gradient(trend, ts)
    x, v, acc = x - trend, v - dtrend, acc - np.gradient(dtrend, ts)
    # Cosine tapers to rest at both ends, by the product rule (x w)'' = x'' w + 2 x' w' + x w''.
    tt = np.arange(n) * ts
    r, T = a.taper, tt[-1]
    w = np.ones(n); dw = np.zeros(n); ddw = np.zeros(n)
    for m, s in ((tt < r, tt), (tt > T - r, T - tt)):
        sgn = 1.0 if s is tt else -1.0
        w[m] = 0.5 * (1 - np.cos(math.pi * s[m] / r))
        dw[m] = sgn * 0.5 * math.pi / r * np.sin(math.pi * s[m] / r)
        ddw[m] = 0.5 * (math.pi / r) ** 2 * np.cos(math.pi * s[m] / r)
    x, v, acc = x * w, v * w + x * dw, acc * w + 2 * v * dw + x * ddw   # uses the untapered x, v on the right
    k = min(1.0, a.vmax / np.max(np.abs(v)), a.xmax / np.max(np.abs(x)))
    x, v, acc = k * x, k * v, k * acc
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"# py/actuator_replay.py {a.run} --t0 {a.t0} --t1 {a.t1} --amax {a.amax} --vmax {a.vmax} "
                f"--xmax {a.xmax} --fc {a.fc} --taper {a.taper}\n")
        f.write(f"# scaled by {k:.3f} to fit: peak |a| {np.max(np.abs(acc)):.3f} m/s^2, |v| {np.max(np.abs(v)):.3f} m/s, "
                f"|x| {np.max(np.abs(x)) * 1e3:.1f} mm\n")
        f.write("t,x,v,a\n")
        for i in range(n):
            f.write(f"{i * ts:.6f},{x[i]:.7f},{v[i]:.7f},{acc[i]:.5f}\n")
    print(f"{out}: {n} rows ({n * ts:.1f} s) from {Path(a.run).name} {a.t0}-{a.t1} s; scaled by {k:.3f}: peak |a| "
          f"{np.max(np.abs(acc)):.2f} m/s^2, |v| {np.max(np.abs(v)):.3f} m/s, |x| {np.max(np.abs(x)) * 1e3:.1f} mm")


if __name__ == "__main__":
    main()
