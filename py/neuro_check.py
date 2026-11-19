"""Check evolved networks in simulation (PC only, no hardware).

    .venv/Scripts/python py/neuro_check.py published [--t 120] [--png docs/img/neuro_benchmark.png]
    .venv/Scripts/python py/neuro_check.py rig NET.json [--runs 300] [--t 30] [--seed 7] [--baseline]
    .venv/Scripts/python py/neuro_check.py replay NET.json logs/<time>_balance [...] [--min-s 2]

published  Pasemann's controllers w1-w9 (py/neuro/published.py) on his benchmark: the 40 x 40 grid of start states
           (x0, theta0; velocities 0), a square counts if the pole is still up after --t seconds (the paper: 120 s),
           out of the 38 x 38 = 1444 inside the limits; and the paper's extreme start (x0 -2.0 or -1.9 m,
           theta0 -0.18 rad) with how the run ends. Simulated as the paper's plots require (output acts at once,
           semi-implicit Euler); then the grid again for the three other readings of 'Euler discretization' and
           'updated simultaneously' (one-step-late output, explicit Euler), which the 2-input networks do not survive.
rig        A saved network on the rig model (py/neuro/rig.py) over --runs start states and plants drawn from
           rig.TEST: reference errors to +-2 counts, w0 +-3 %, friction ranges, delays 4-7 ticks. --baseline adds
           the rig's own controller (PID + Stage 3b cart loop, pendel.control.cascade_step) on the same draws.
replay     A saved network fed, open loop, the logged x, theta, v, omega of every ENGAGED stretch of real rig runs;
           its command is compared with the one the rig's controller sent (plausibility on real sensor data).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from neuro import bench, rig
from neuro.net import Batch, Genome
from neuro.published import TABLE, published

EXTREME = {"w2": -1.9, "w4": -1.9}       # x0 of the paper's figure (b) where it is not -2.0 m


VARIANTS = [("immediate", "semi"), ("immediate", "explicit"), ("delayed", "semi"), ("delayed", "explicit")]


def extreme_run(g: Genome, t: float) -> dict:
    x0 = EXTREME.get(g.meta.get("name"), -2.0)
    r = bench.run(Batch([g]), [x0], [-0.18], t_max=t, trace=(0, 0))
    tr = r["trace"]
    last = tr["t"] > tr["t"][-1] - 20.0
    ok = bool(r["alive"][0, 0])
    # settled: from some time on |x| < 0.1 m and |theta| < 1 deg until the end
    calm = (np.abs(tr["x"]) < 0.1) & (np.abs(tr["theta"]) < math.radians(1))
    settle = None
    if ok and calm[-1]:
        bad = np.flatnonzero(~calm)
        settle = float(tr["t"][bad[-1] + 1]) if len(bad) else 0.0
    return {"x0": x0, "ok": ok, "t_fail": float(r["t_fail"][0, 0]),
            "x_last": float(np.mean(np.abs(tr["x"][last]))) if ok else float("nan"),
            "th_last": float(np.degrees(np.max(np.abs(tr["theta"][last])))) if ok else float("nan"),
            "F_rms": float(np.sqrt(np.mean(tr["F"][last] ** 2))) if ok else float("nan"), "settle": settle}


def cmd_published(args) -> int:
    X0, TH0, shape = bench.grid(40)
    inside = (np.abs(X0) <= bench.X_LIM) & (np.abs(TH0) <= bench.TH_LIM)
    maps = {}
    n_in = int(inside.sum())
    print(f"Pasemann's benchmark, {args.t:.0f} s per start state; grid squares out of {n_in} inside the limits.")
    print("Output acts at once, semi-implicit Euler (what the paper's plots require).\n")
    print("| net | inputs | class | grid squares balanced | extreme start | end state (last 20 s) |")
    print("|---|---|---|---|---|---|")
    for name in TABLE:
        g = published(name)
        r = bench.run(Batch([g]), X0, TH0, t_max=args.t)
        ok = r["alive"][0] & inside
        maps[(name, VARIANTS[0])] = ok.reshape(shape)
        e = extreme_run(g, args.t)
        if e["ok"]:
            end = (f"mean |x| {e['x_last']:.2f} m, max |theta| {e['th_last']:.2f} deg, F rms {e['F_rms']:.2f} N"
                   + (f"; settled after {e['settle']:.0f} s" if e["settle"] is not None else "; not settled"))
            ext = f"x0 {e['x0']} m: balanced {args.t:.0f} s"
        else:
            end = "-"
            ext = f"x0 {e['x0']} m: failed at {e['t_fail']:.2f} s"
        print(f"| {name} | {g.n_in} | {g.cls} | {int(ok.sum())} ({100 * ok.sum() / n_in:.0f} %) | {ext} | {end} |",
              flush=True)
    print("\nThe same grid under the other readings of the paper's timing and integrator:\n")
    print("| net | " + " | ".join(f"{t}, {i}" for t, i in VARIANTS) + " |")
    print("|---|" + "---|" * len(VARIANTS))
    for name in TABLE:
        g = published(name)
        cells = []
        for timing, integ in VARIANTS:
            if (name, (timing, integ)) not in maps:
                r = bench.run(Batch([g]), X0, TH0, t_max=args.t, timing=timing, integrator=integ)
                maps[(name, (timing, integ))] = (r["alive"][0] & inside).reshape(shape)
            cells.append(f"{100 * maps[(name, (timing, integ))].sum() / n_in:.0f} %")
        print(f"| {name} | " + " | ".join(cells) + " |", flush=True)
    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        names = list(TABLE)
        rows = [VARIANTS[0], VARIANTS[1]]
        fig, axs = plt.subplots(len(rows), len(names), figsize=(1.9 * len(names), 2.3 * len(rows) + 0.4),
                                constrained_layout=True)
        for c, name in enumerate(names):
            for r_, var in enumerate(rows):
                ax = axs[r_, c]
                ax.imshow(~maps[(name, var)], cmap="gray", origin="lower", vmin=0, vmax=1,
                          extent=[-bench.X_LIM, bench.X_LIM, -12, 12], aspect="auto", interpolation="nearest")
                ax.set_title(f"{name}: {var[1]} Euler", fontsize=8)
                ax.tick_params(labelsize=6)
                if c == 0:
                    ax.set_ylabel("theta0 (deg)", fontsize=7)
                if r_ == len(rows) - 1:
                    ax.set_xlabel("x0 (m)", fontsize=7)
        fig.suptitle(f"Start states still balanced after {args.t:.0f} s (black). Top: semi-implicit Euler, as the "
                     f"paper's plots require; bottom: explicit Euler", fontsize=9)
        pathlib.Path(args.png).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.png, dpi=130)
        print(f"\nwrote {args.png}")
    return 0


def rig_report(policy, ics: dict, spec: rig.RigSpec, t: float) -> dict:
    r = rig.run(policy, ics, spec, t_max=t)
    alive = r["alive"][0]
    return {"survived": float(alive.mean()), "t_fail_med": float(np.median(r["t_fail"][0])),
            "end_x_med": float(np.median(np.abs(r["end_x"][0][alive]))) if alive.any() else float("nan"),
            "IA": float(np.mean(r["IA"][0])), "DU": float(np.mean(r["DU"][0]))}


def cmd_rig(args) -> int:
    g = Genome.load(args.net)
    spec = rig.RigSpec(n_in=g.n_in, net_every=int(g.meta.get("net_every", 5)))
    ics = rig.draw(np.random.default_rng(args.seed), args.runs, replace(rig.TEST, stall_rate=args.stall_rate), spec)
    rows = [("network", rig_report(Batch([g]), ics, spec, args.t))]
    if args.baseline:
        rows.append(("rig PID + cart loop (Stage 3b)",
                     rig_report(rig.PidPolicy(), ics, rig.RigSpec(n_in=g.n_in, net_every=1), args.t)))
    print(f"{args.net}: {g.n_in} inputs, network every {spec.net_every} ticks; {args.runs} runs of {args.t:.0f} s "
          f"on the rig model, start states and plants from rig.TEST (seed {args.seed})"
          + (f", late replies {args.stall_rate:g}/s" if args.stall_rate else "") + "\n")
    print(f"| controller | survived {args.t:.0f} s | median failure time | median end abs x (survivors) "
          f"| mean abs a | command changes |")
    print("|---|---|---|---|---|---|")
    for name, rep in rows:
        print(f"| {name} | {100 * rep['survived']:.0f} % | {rep['t_fail_med']:.1f} s | {1000 * rep['end_x_med']:.0f} mm "
              f"| {rep['IA']:.3f} m/s^2 | {rep['DU']:.2f} /s |")
    print()
    for line in g.rows():
        print("  " + line)
    return 0


def cmd_replay(args) -> int:
    """Open loop: the network sees the logged x, theta, v (and omega) of every ENGAGED stretch, reset at its start,
    updating every net_every ticks as in evolution; its command (after the same limiter) is compared with the one the
    rig's controller sent. The log's trajectory came from that controller, so this checks plausibility on real,
    quantised sensor data, not closed-loop behaviour."""
    from neuro.net import NetController
    g = Genome.load(args.net)
    spec = rig.RigSpec(n_in=g.n_in, net_every=int(g.meta.get("net_every", 5)))
    print(f"{args.net} ({g.n_in} inputs, every {spec.net_every} ticks): open-loop replay of ENGAGED stretches "
          f"of at least {args.min_s:.0f} s\n")
    print("| run | start (s) | length (s) | corr(net, logged) | rms logged | rms net | rms difference "
          "| same sign (logged > 0.1 m/s^2) | net at its limit | rms change per update: logged / net |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for run_dir in args.runs:
        p = pathlib.Path(run_dir)
        with open(p / "data.csv", encoding="utf-8") as f:
            header = f.readline().strip().split(",")
        cols = ["mode", "ctrl", "theta", "omega", "x", "v", "a_cmd"]
        d = np.loadtxt(p / "data.csv", delimiter=",", skiprows=1, usecols=[header.index(c) for c in cols])
        mode, ctrl, th, om, x, v, a_log = d.T
        eng = ((mode == 2) | (ctrl == 1)).astype(int)     # hand catches: mode ENGAGED; swing-up sessions: ctrl = 1
        edges = np.flatnonzero(np.diff(np.r_[0, eng, 0]))
        for s, e in zip(edges[::2], edges[1::2]):
            n = e - s
            if n * spec.ts < args.min_s:
                continue
            c = NetController(g)
            a_net = np.zeros(n)
            u = 0.0
            for k in range(n):
                i = s + k
                if k % spec.net_every == 0:
                    feats = [x[i] / spec.x_max, th[i] / spec.th_scale, v[i] / spec.v_max]
                    if g.n_in == 4:
                        feats.append(om[i] / spec.th_scale)
                    u = c.step(feats)
                a_net[k] = float(rig.limit(spec.a_max * u, v[i], spec))
            al = a_log[s:e]
            big = np.abs(al) > 0.1
            same = float(np.mean(np.sign(a_net[big]) == np.sign(al[big]))) if big.any() else float("nan")
            sub = slice(0, n, spec.net_every)
            print(f"| {p.name} | {s * spec.ts:.1f} | {n * spec.ts:.1f} | {np.corrcoef(a_net, al)[0, 1]:.2f} "
                  f"| {np.sqrt(np.mean(al ** 2)):.3f} | {np.sqrt(np.mean(a_net ** 2)):.3f} "
                  f"| {np.sqrt(np.mean((a_net - al) ** 2)):.3f} | {100 * same:.0f} % "
                  f"| {100 * np.mean(np.abs(a_net) > 0.98 * spec.a_max):.1f} % "
                  f"| {np.sqrt(np.mean(np.diff(al[sub]) ** 2)):.3f} / {np.sqrt(np.mean(np.diff(a_net[sub]) ** 2)):.3f} |",
                  flush=True)
    return 0


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("published")
    p.add_argument("--t", type=float, default=120.0, help="s per start state (the paper: 120)")
    p.add_argument("--png", default="", help="write the grid maps here")
    q = sub.add_parser("rig")
    q.add_argument("net")
    q.add_argument("--runs", type=int, default=300)
    q.add_argument("--t", type=float, default=30.0)
    q.add_argument("--seed", type=int, default=7)
    q.add_argument("--baseline", action="store_true", help="also run the rig's PID + cart loop on the same draws")
    q.add_argument("--stall-rate", type=float, default=0.0,
                   help="late replies per second on the remote path (the Pi holds the command 1-3 ticks)")
    r_ = sub.add_parser("replay")
    r_.add_argument("net")
    r_.add_argument("runs", nargs="+", help="run directories with data.csv (logs/<time>_balance)")
    r_.add_argument("--min-s", type=float, default=2.0, help="skip ENGAGED stretches shorter than this (s)")
    args = ap.parse_args()
    return {"published": cmd_published, "rig": cmd_rig, "replay": cmd_replay}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
