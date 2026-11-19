"""Latency of the remote controller link and the constant delay it needs (Stage 2b, docs/remote.md).

    .venv/Scripts/python py/remote_report.py RUN [--pc PCRUN]   a remote-sim or balance.remote run dir (remote.csv)
    .venv/Scripts/python py/remote_report.py rtt.csv            a py/net_probe_pi.py recording (seq,rtt_us)

Round trip = sample time on the Pi -> the PC's reply arriving at the Pi (kernel timestamp; the probe: the Pi's
clock), so it is exactly what the constant delay has to cover. A reply for the sample of tick k is used at tick
k + N if it arrives before that tick starts: rtt < N * Ts - margin (margin 0.1 ms: the sample leaves ~50-100 us
into its tick). Prints the distribution, the late replies for each N and their longest run (each late reply
is a tick that holds the previous command; remote.max_miss in a row abort), and the smallest N that covered
every reply in the recording. For a run dir also: missed ticks, |PC - local PID|. Plot: latency.png.

Split by direction (run dir + the PC program's log, logs/<time>_remote-pc/samples.csv: --pc, or found next to the
run by its time): where a slow round trip lost its time.
  - Pi -> PC: the Pi sends sample k at k * 2 ms, so the PC's receive time minus k * 2 ms is constant up to the
    clock offset and drift. Its lower envelope (fastest sample of each second, interpolated) is taken as the floor;
    the excess over it is the delay of that sample: Pi send, network, PC network card, Windows waking the program.
  - PC processing: the PC's own receive -> send time (pc_proc_ns in the reply).
  - PC -> Pi: what is left of the round trip (PC send, network, Pi network card up to the kernel timestamp),
    as excess over its 0.1 % quantile. No clock sync is needed: only differences on each clock are used.
A stall shows as a jump in one leg. It tells the direction, not which machine: to tell the network from the PC's
or the Pi's network card, change one thing (direct cable, other port, settings) and compare.
"""

from __future__ import annotations

import argparse
import csv
import datetime
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
    not a latency; each one holds the previous command for one tick), the run's meta.json, the PC's processing
    time per reply (s; zeros for a probe recording)."""
    if path.is_dir():
        rows = list(csv.DictReader(open(path / "remote.csv")))
        seq = np.array([int(r["seq"]) for r in rows])
        rtt = np.array([int(r["rtt_ns"]) for r in rows]) * 1e-9
        proc = np.array([int(r["pc_proc_ns"]) for r in rows]) * 1e-9
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
        proc = np.zeros(len(rtt))
    lost = int(seq.max() - seq.min() + 1 - len(np.unique(seq))) if len(seq) else 0
    return seq, rtt, lost, meta, proc


def longest_run(mask: np.ndarray) -> int:
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best


def _dir_time(p: pathlib.Path):
    try:
        return datetime.datetime.strptime(p.name[:19], "%Y-%m-%dT%H-%M-%S")
    except ValueError:
        return None


def find_pc_log(run: pathlib.Path, duration_s: float):
    """The PC program's log of this run: the first <time>_remote-pc dir next to the run that was written (when the
    PC program ended) after the run started and within its duration + 15 min. Both names are local time."""
    t0 = _dir_time(run)
    if t0 is None:
        return None
    t1 = t0 + datetime.timedelta(seconds=duration_s + 900)
    cands = sorted((t, d) for d in run.parent.glob("*_remote-pc") if (t := _dir_time(d)) and t0 <= t <= t1)
    return cands[0][1] if cands else None


def load_pc(pc_dir: pathlib.Path, pi_seq: np.ndarray):
    """(seq, t_rx_ns) from the PC program's samples.csv. With --stay one log can hold several Pi sessions (seq
    restarts): the session whose seq overlap with the Pi's replies is largest."""
    seq, t = [], []
    with open(pc_dir / "samples.csv") as f:
        next(f)
        for line in f:
            p = line.split(",", 2)
            seq.append(int(p[0]))
            t.append(int(p[1]))
    seq, t = np.array(seq, dtype=np.int64), np.array(t, dtype=np.int64)
    cuts = np.flatnonzero(np.diff(seq) < 0) + 1
    best = None
    for s, ts in zip(np.split(seq, cuts), np.split(t, cuts)):
        n = len(np.intersect1d(s, pi_seq))
        if best is None or n > best[0]:
            best = (n, s, ts)
    return best[1], best[2]


def split_legs(seq: np.ndarray, rtt: np.ndarray, proc: np.ndarray, pc_seq: np.ndarray, pc_t: np.ndarray):
    """Per reply (matched on seq): Pi -> PC excess, PC processing, PC -> Pi excess (s); None if too few matches."""
    # One-way Pi -> PC delay + clock offset + drift, for every sample the PC received (also those whose reply was lost).
    _, i = np.unique(pc_seq, return_index=True)
    ps, pt = pc_seq[i], pc_t[i]
    f = (pt - ps * int(TS * 1e9)) * 1e-9
    f -= f[0]                                       # (keep the numbers small; the floor removes any offset anyway)
    # Floor: the fastest sample of each second; drop a second that was stalled as a whole (> 0.5 ms above its
    # neighbourhood's median), then interpolate between them: follows a drifting or slewed clock.
    bins = (ps - ps[0]) // int(1.0 / TS)
    edges = np.flatnonzero(np.diff(bins)) + 1
    xs, ys = [], []
    for a, b in zip(np.r_[0, edges], np.r_[edges, len(ps)]):
        j = a + int(np.argmin(f[a:b]))
        xs.append(ps[j])
        ys.append(f[j])
    xs, ys = np.array(xs), np.array(ys)
    if len(xs) < 3:
        return None
    med = np.array([np.median(ys[max(0, k - 2):k + 3]) for k in range(len(ys))])
    keep = ys <= med + 0.0005
    floor = np.interp(ps, xs[keep], ys[keep])
    fwd_all = f - floor
    # Match the Pi's replies.
    k = np.searchsorted(ps, seq)
    k = np.clip(k, 0, len(ps) - 1)
    ok = ps[k] == seq
    if ok.sum() < 0.5 * len(seq):
        return None
    fwd = np.full(len(seq), np.nan)
    fwd[ok] = fwd_all[k[ok]]
    back = rtt - proc - fwd
    back -= np.nanpercentile(back, 0.1)
    return ok, fwd, back, (ps, fwd_all)


def print_legs(path, pc_dir, seq, rtt, proc, legs, delay_ticks):
    ok, fwd, back, (ps, fwd_all) = legs
    s, r, fw, pr, bk = seq[ok], rtt[ok], fwd[ok], proc[ok], back[ok]
    print(f"\nSplit by direction (PC log {pc_dir}, {int(ok.sum())} of {len(seq)} replies matched; excess over each leg's "
          f"floor, PC processing absolute):")
    print("\n| ms | p50 | p99 | p99.9 | max |")
    print("|---|---|---|---|---|")
    for name, v in (("round trip", r), ("Pi -> PC excess", fw), ("PC processing", pr), ("PC -> Pi excess", bk)):
        print(f"| {name} | " + " | ".join(f"{np.percentile(v, p) * 1e3:.3f}" for p in (50, 99, 99.9)) +
              f" | {v.max() * 1e3:.3f} |")
    if delay_ticks:
        slow = r > delay_ticks * TS - MARGIN
        what = f"late at the run's delay of {delay_ticks} ticks"
    else:
        slow = np.zeros(len(r), bool)
    if not slow.any():
        slow = r >= np.sort(r)[-min(20, len(r))]
        what = f"the {int(slow.sum())} slowest"
    parts = np.stack([fw, pr, bk])
    blame = np.argmax(parts, axis=0)
    names = ("Pi -> PC", "PC processing", "PC -> Pi")
    counts = [int((blame[slow] == i).sum()) for i in range(3)]
    print(f"\nSlow replies ({what}): {int(slow.sum())}; most of the extra time in " +
          ", ".join(f"{n} {c}" for n, c in zip(names, counts)))
    print("\n| t (s) | round trip | Pi -> PC | PC proc | PC -> Pi |  (ms)")
    print("|---|---|---|---|---|")
    for i in np.argsort(r)[::-1][:12]:
        print(f"| {s[i] * TS:.2f} | {r[i] * 1e3:.2f} | {fw[i] * 1e3:.2f} | {pr[i] * 1e3:.2f} | {bk[i] * 1e3:.2f} |")
    gaps = np.diff(ps)
    j = int(np.argmax(fwd_all))
    print(f"\nLargest Pi -> PC excess over all samples the PC received: {fwd_all[j] * 1e3:.2f} ms at {ps[j] * TS:.2f} s"
          f" (samples lost on the way: {int((gaps - 1).clip(0).sum())})")
    print("Reading it: a stall in one leg for a run of consecutive samples that then drains in a burst = something held "
          "the packets (that direction's sender, the network or the receiver's network card / OS); a stall in PC "
          "processing = the PC program itself (priority, console, GC).")


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="run dir (remote.csv) or a net_probe_pi.py recording")
    ap.add_argument("--pc", help="the PC program's log dir (<time>_remote-pc); default: found next to the run")
    a = ap.parse_args()
    path = pathlib.Path(a.path)
    seq, rtt, lost, meta, proc = load(path)
    if not len(rtt):
        print("no replies")
        return
    o = np.argsort(seq)
    seq, rtt, proc = seq[o], rtt[o], proc[o]
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

    legs = None
    if path.is_dir():
        pc_dir = pathlib.Path(a.pc) if a.pc else find_pc_log(path, dur)
        if pc_dir is None:
            print("\nno PC log (<time>_remote-pc) found next to the run: pass --pc for the split by direction")
        elif not (pc_dir / "samples.csv").exists():
            print(f"\n{pc_dir}: no samples.csv; no split by direction")
        else:
            pc_seq, pc_t = load_pc(pc_dir, seq)
            legs = split_legs(seq, rtt, proc, pc_seq, pc_t)
            if legs is None:
                print(f"\n{pc_dir}: under half of the replies match its samples (another session?): pass --pc")
            else:
                dt = meta.get("remote_delay_ticks")
                print_legs(path, pc_dir, seq, rtt, proc, legs, int(dt) if dt else None)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, axs = plt.subplots(2 if legs else 1, 2, figsize=(13, 8 if legs else 4), squeeze=False)
    ax = axs[0]
    if legs:
        ok, fwd, back, _ = legs
        t = seq[ok] * TS
        axs[1][0].plot(t, fwd[ok] * 1e3, ".", ms=1, label="Pi -> PC excess")
        axs[1][0].plot(t, back[ok] * 1e3, ".", ms=1, label="PC -> Pi excess")
        axs[1][0].plot(t, proc[ok] * 1e3, ".", ms=1, label="PC processing")
        axs[1][0].set_xlabel("time (s)")
        axs[1][0].set_ylabel("ms")
        axs[1][0].legend(markerscale=8)
        for v, lab in ((fwd[ok], "Pi -> PC excess"), (back[ok], "PC -> Pi excess"), (proc[ok], "PC processing")):
            axs[1][1].hist(v * 1e3, bins=200, log=True, histtype="step", label=lab)
        axs[1][1].set_xlabel("ms")
        axs[1][1].legend()
        for b in axs[1]:
            b.grid(True, lw=0.3)
    ax[0].plot(seq * TS, rtt * 1e3, ".", ms=1)
    ax[0].set_xlabel("time (s)")
    ax[0].set_ylabel("round trip (ms)")
    if n_cover:
        ax[0].axhline(n_cover * TS * 1e3, color="r", lw=0.6, label=f"N = {n_cover} ticks")
        ax[0].legend()
    ax[1].hist(rtt * 1e3, bins=200, log=True)
    ax[1].set_xlabel("round trip (ms)")
    ax[1].set_ylabel("replies")
    for b in ax:
        b.grid(True, lw=0.3)
    out = (path if path.is_dir() else path.parent) / "latency.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"plot: {out}")


if __name__ == "__main__":
    main()
