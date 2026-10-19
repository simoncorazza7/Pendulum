"""Latency of the remote controller link and the constant delay it needs (Stage 2b, docs/remote.md).

    .venv/Scripts/python py/remote_report.py RUN        a remote-sim or balance.remote run dir (remote.csv)
    .venv/Scripts/python py/remote_report.py rtt.csv    a py/net_probe_pi.py recording (seq,rtt_us)

Round trip = sample time on the Pi -> the PC's reply arriving at the Pi (kernel timestamp; the probe: the Pi's
clock), so it is exactly what the constant delay has to cover. A reply for the sample of tick k is used at tick
k + N if it arrives before that tick starts: rtt < N * Ts - margin (margin 0.1 ms: the sample leaves ~50-100 us
into its tick). Prints the distribution, the late replies for each N and their longest run (each late reply
is a tick that holds the previous command; remote.max_miss in a row abort), and the smallest N that covered
every reply in the recording. For a run dir also: missed ticks, |PC - local PID|. Plot: latency.png.
"""

from __future__ import annotations

import csv
import json
import math
import pathlib
import sys

import numpy as np

TS = 0.002
MARGIN = 0.0001
PID_DELAY_MARGIN = 0.043      # s, extra constant delay the PID tolerates (Gate B appendix; 37 ms if it varies)
W_GC = 18.6                   # rad/s, crossover: each tick of delay costs W_GC * TS = 2.1 deg of phase margin


def load(path: pathlib.Path):
    """(seq, rtt_s) of the replies received, the samples inside the recording that got no reply (lost packets:
    not a latency; each one holds the previous command for one tick), the run's meta.json."""
    if path.is_dir():
        rows = list(csv.DictReader(open(path / "remote.csv")))
        seq = np.array([int(r["seq"]) for r in rows])
        rtt = np.array([int(r["rtt_ns"]) for r in rows]) * 1e-9
        meta = json.loads((path / "meta.json").read_text()) if (path / "meta.json").exists() else {}
    else:
        seq, rtt = [], []
        for line in open(path):
            if line.startswith("#") or line.startswith("seq"):
                continue
            s, r = line.strip().split(",")
            if int(r) >= 0:
                seq.append(int(s))
                rtt.append(int(r) * 1e-6)
        seq, rtt, meta = np.array(seq), np.array(rtt), {}
    lost = int(seq.max() - seq.min() + 1 - len(np.unique(seq))) if len(seq) else 0
    return seq, rtt, lost, meta


def longest_run(mask: np.ndarray) -> int:
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    path = pathlib.Path(sys.argv[1])
    seq, rtt, lost, meta = load(path)
    if not len(rtt):
        print("no replies")
        return
    o = np.argsort(seq)
    seq, rtt = seq[o], rtt[o]
    dur = (seq[-1] - seq[0] + 1) * TS
    q = lambda p: np.percentile(rtt, p) * 1e3   # noqa: E731
    print(f"{path}: {len(rtt)} replies over {dur / 60:.1f} min, {lost} samples inside it without a reply "
          f"({100 * lost / max(1, lost + len(rtt)):.3f} %: lost packets, each holds the previous command one tick)")
    print("\n| round trip (ms) | p50 | p90 | p99 | p99.9 | p99.99 | max |")
    print("|---|---|---|---|---|---|---|")
    print(f"| | {q(50):.3f} | {q(90):.3f} | {q(99):.3f} | {q(99.9):.3f} | {q(99.99):.3f} | {rtt.max() * 1e3:.3f} |")
    worst = np.argsort(rtt)[-8:][::-1]
    print("\nworst replies: " + ", ".join(f"{rtt[i] * 1e3:.2f} ms at {seq[i] * TS:.1f} s" for i in worst))

    print(f"\n| delay N (ticks) | delay (ms) | late replies | longest run of late | phase margin left (deg) |")
    print("|---|---|---|---|---|")
    n_cover = None
    for n in range(1, 31):
        late = rtt > n * TS - MARGIN
        if n_cover is None and not late.any():
            n_cover = n
        # 46 deg for the local loop, whose command acts ~0.5 ms after its sample; remote: at tick k + N (+~0.1 ms)
        pm = 46.0 - math.degrees(W_GC * (n * TS - 0.0004))
        if n <= 12 or late.any() or n == n_cover:
            print(f"| {n} | {n * TS * 1e3:.0f} | {int(late.sum())} | {longest_run(late)} | {pm:.0f} |")
        if n_cover is not None and n > n_cover + 1 and n > 12:
            break
    if n_cover is None:
        print("\nno N up to 30 ticks (60 ms) covered every reply: this link is too slow for the constant delay")
    else:
        print(f"\nsmallest N that covered every reply: {n_cover} ticks = {n_cover * TS * 1e3:.0f} ms. With one tick of margin "
              f"for a tail longer than this recording: remote.delay_ticks = {n_cover + 1} "
              f"({(n_cover + 1) * TS * 1e3:.0f} ms; the PID tolerates ~{PID_DELAY_MARGIN * 1e3:.0f} ms).")
    if meta:
        keys = ["result", "end_reason", "engaged_s", "missed_ticks", "remote_missed_ticks", "remote_delay_ticks",
                "remote_late_replies", "remote_max_abs_diff", "remote_rms_diff", "remote_peer", "overruns"]
        print("\nrun: " + ", ".join(f"{k} {meta[k]}" for k in keys if k in meta))
    if path.is_dir() and (path / "data.csv").exists():
        d = list(csv.DictReader(open(path / "data.csv")))
        if d and "r_miss" in d[0]:
            miss = np.array([int(r["r_miss"]) for r in d])
            ctrl = np.array([int(r["ctrl"]) for r in d])
            print(f"engaged ticks {int(ctrl.sum())}, ticks that held the previous command (late / missing reply) "
                  f"{int(miss.sum())}, longest run {longest_run(miss.astype(bool))}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(1, 2, figsize=(13, 4))
    ax[0].plot(seq * TS, rtt * 1e3, ".", ms=1)
    ax[0].set_xlabel("time (s)")
    ax[0].set_ylabel("round trip (ms)")
    if n_cover:
        ax[0].axhline(n_cover * TS * 1e3, color="r", lw=0.6, label=f"N = {n_cover} ticks")
        ax[0].legend()
    ax[1].hist(rtt * 1e3, bins=200, log=True)
    ax[1].set_xlabel("round trip (ms)")
    ax[1].set_ylabel("replies")
    for a in ax:
        a.grid(True, lw=0.3)
    out = (path if path.is_dir() else path.parent) / "latency.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"plot: {out}")


if __name__ == "__main__":
    main()
