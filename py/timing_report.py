"""Stage 0.5: compare timing-probe runs (histograms, time series, summary table).

Usage (repo root):
    .venv/Scripts/python py/timing_report.py RUN_DIR [RUN_DIR ...] [--out docs/img/timing.png]
Prints a markdown table and saves one figure with
  - control-loop wake-up lateness histogram (log y),
  - STEP-edge lateness histogram (log y),
  - per-tick max STEP lateness over time.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pendel import load_run


def hist(run_dir: pathlib.Path, name: str) -> tuple[np.ndarray, np.ndarray]:
    d = np.genfromtxt(run_dir / name, delimiter=",", names=True, dtype=None, encoding="utf-8")
    d = np.atleast_1d(d)
    return d["bin_us"].astype(float), d["count"].astype(float)


def env(run_dir: pathlib.Path) -> dict:
    out = {}
    f = run_dir / "env.txt"
    if f.exists():
        for line in f.read_text().splitlines():
            if "=" in line and not line.startswith("#"):
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out


def label(run) -> str:
    return f"{run.meta.get('profile')}, load={env(run.path).get('load', '?')}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default="timing.png")
    args = ap.parse_args()
    runs = [load_run(r) for r in args.runs]

    print("| run | loop late p50/p99/p99.9/max (us) | STEP late p50/p99/p99.9/max (us) | STEP edges | resyncs | "
          "loop exec max (us) | throttled after |")
    print("|---|---|---|---|---|---|---|")
    for r in runs:
        m, e = r.meta, env(r.path)
        print(f"| {label(r)} | {m['loop_late_p50_us']}/{m['loop_late_p99_us']}/{m['loop_late_p999_us']}/"
              f"{m['loop_late_max_us']} | {m['step_late_p50_us']}/{m['step_late_p99_us']}/"
              f"{m['step_late_p999_us']}/{m['step_late_max_us']} | {m['step_edges']} | {m['axis_resyncs']} | "
              f"{r['exec_ns'].max() / 1e3:.0f} | {e.get('throttled_after', '?')} |")

    fig, axes = plt.subplots(3, 1, figsize=(10, 11))
    for r in runs:
        for ax, name in ((axes[0], "hist_loop_late.csv"), (axes[1], "hist_step_late.csv")):
            b, c = hist(r.path, name)
            if len(b):
                ax.step(b, c, where="post", label=label(r))
        t = (r["t_ns"] - r["t_ns"][0]) * 1e-9
        axes[2].plot(t, r["step_late_max_ns"] / 1e3, lw=0.5, label=label(r))
    axes[0].set(title="control loop wake-up lateness", xlabel="us", ylabel="count", yscale="log")
    axes[1].set(title="STEP edge lateness (actual - scheduled)", xlabel="us", ylabel="count", yscale="log")
    axes[2].set(title="max STEP lateness per 2 ms tick", xlabel="t (s)", ylabel="us", yscale="log")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"\nfigure -> {out}")


if __name__ == "__main__":
    main()
