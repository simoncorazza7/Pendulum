"""Stage 0.6: decode an encoder-log run with several decoders and report anomalies.

Usage (repo root):
    .venv/Scripts/python py/encoder_report.py [RUN_DIR] [--rest 1.0]

Decoders, all on the same raw events:
  orig    replay of the original Encoder class: one queue per line, decode only while both
          queues hold an event (earlier front first), state from the kernel edge type
  type    every event in timestamp order, state from the kernel edge type
  toggle  every event in timestamp order, the event's line flips (edge type ignored)
  level   line levels read after each batch; Gray-code steps between consecutive snapshots
Counts use the original sign convention (A leading B = negative).
Rest periods (no event for --rest s) are listed with each decoder's count, which is the
return-to-bottom drift test.
"""

import argparse
import pathlib
import sys
from collections import deque

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel import latest_run, load_run

# Quadrature table of the original code, indexed [prev][cur] with state = 2*A + B.
TABLE = np.array([[0, +1, -1, 0], [-1, 0, 0, +1], [+1, 0, 0, -1], [0, -1, +1, 0]])
RISING = 1


def decode_orig(line, typ, ts, a0, b0):
    """Count visible after each event, in read order."""
    state = [a0, b0]
    queues = (deque(), deque())
    count, out = 0, np.empty(len(line), dtype=np.int64)
    for i in range(len(line)):
        queues[line[i]].append((ts[i], typ[i]))
        while queues[0] and queues[1]:
            ln = 0 if queues[0][0][0] < queues[1][0][0] else 1   # ties go to B, as in the original
            _, t = queues[ln].popleft()
            new = list(state)
            new[ln] = 1 if t == RISING else 0
            count += TABLE[2 * state[0] + state[1], 2 * new[0] + new[1]]
            state = new
        out[i] = count
    return out


def decode_ts(line, typ, a0, b0, use_type):
    """Count after each event, events already in timestamp order. Returns counts, same-type hits."""
    state = [a0, b0]
    count, out = 0, np.empty(len(line), dtype=np.int64)
    same = [0, 0]
    for i in range(len(line)):
        new = list(state)
        if use_type:
            new[line[i]] = 1 if typ[i] == RISING else 0
            if new == state:
                same[line[i]] += 1
        else:
            new[line[i]] ^= 1
        count += TABLE[2 * state[0] + state[1], 2 * new[0] + new[1]]
        state = new
        out[i] = count
    return out, same


def decode_level(la, lb, a0, b0):
    """Count after each event from the level snapshots; returns counts, invalid (2-bit) steps."""
    prev = 2 * a0 + b0
    count, invalid, out = 0, 0, np.empty(len(la), dtype=np.int64)
    for i in range(len(la)):
        cur = 2 * int(la[i]) + int(lb[i])
        if cur != prev and (prev ^ cur) == 3:
            invalid += 1
        else:
            count += TABLE[prev, cur]
        prev = cur
        out[i] = count
    return out, invalid


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?")
    ap.add_argument("--rest", type=float, default=1.0, help="gap (s) that counts as a rest period")
    args = ap.parse_args()
    run = load_run(args.run or latest_run(mode="encoder-log"))
    m = run.meta
    a0, b0 = int(m["start_level_a"]), int(m["start_level_b"])
    d = run.data
    if len(d) == 0:
        print("no events")
        return

    line, typ, ts, rd = d["line"].astype(int), d["type"].astype(int), d["ts_ns"], d["read_ns"]
    orig = decode_orig(line, typ, ts, a0, b0)                      # read order
    o = np.argsort(ts, kind="stable")                              # timestamp order
    typed, same = decode_ts(line[o], typ[o], a0, b0, use_type=True)
    toggled, _ = decode_ts(line[o], typ[o], a0, b0, use_type=False)
    level, invalid = decode_level(d["lvl_a"], d["lvl_b"], a0, b0)  # read order
    t0 = ts.min()

    # Edge rate per line in 10 ms windows.
    rate = 0.0
    for ln in (0, 1):
        tl = np.sort(ts[line == ln])
        if len(tl) > 1:
            n = np.searchsorted(tl, tl + 10_000_000) - np.arange(len(tl))
            rate = max(rate, n.max() / 0.01)
    lat = (rd - ts) / 1e3

    print(f"run {run.path.name}: {len(d)} events over {(ts.max() - t0) / 1e9:.1f} s, start A={a0} B={b0}")
    print(f"  final counts: orig {orig[-1]}, type {typed[-1]}, toggle {toggled[-1]}, level {level[-1]}")
    print(f"  same-type repeats (type decoder): A {same[0]}, B {same[1]}; level 2-bit jumps: {invalid}")
    print(f"  read latency us: p50 {np.median(lat):.0f}, p99 {np.percentile(lat, 99):.0f}, max {lat.max():.0f}; "
          f"max batch {m.get('max_batch')}, full (16) batches {m.get('full_batches')}")
    print(f"  max edge rate per line: {rate:.0f} edges/s ({rate / 1200:.2f} rev/s at 1200 edges/line/rev)")

    # Rest periods: gaps in the timestamp-ordered events.
    tso = ts[o]
    gaps = np.where(np.diff(tso) > args.rest * 1e9)[0]
    rest_idx = list(gaps) + [len(tso) - 1]
    print(f"\n  rest periods (gap > {args.rest} s): index, t (s), counts orig/type/toggle")
    for k, i in enumerate(rest_idx):
        j = np.searchsorted(ts, tso[i], side="right") - 1    # read-order index of that event (approx.)
        print(f"   {k:3d}  t={(tso[i] - t0) / 1e9:8.2f}  {orig[min(j, len(orig) - 1)]:7d} {typed[i]:7d} {toggled[i]:7d}")

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot((ts - t0) / 1e9, orig, lw=0.8, label="orig (read order)")
    axes[0].plot((tso - t0) / 1e9, typed, lw=0.8, label="type")
    axes[0].plot((tso - t0) / 1e9, toggled, lw=0.8, ls="--", label="toggle")
    axes[0].plot((ts - t0) / 1e9, level, lw=0.8, ls=":", label="level")
    axes[0].set(xlabel="t (s)", ylabel="counts", title=run.path.name)
    axes[0].legend(fontsize=8)
    axes[1].hist(lat, bins=200, log=True)
    axes[1].set(xlabel="read latency (us) = read time - kernel timestamp", ylabel="events")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run.path / "encoder.png", dpi=110)
    print(f"\n  figure -> {run.path / 'encoder.png'}")


if __name__ == "__main__":
    main()
