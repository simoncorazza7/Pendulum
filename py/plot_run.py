"""Plot the columns of a run against time and save plot.png in the run directory.

Usage (from the repo root, with the venv):
    .venv/Scripts/python py/plot_run.py [RUN_DIR] [--cols a,b,c] [--x t] [--show]
RUN_DIR defaults to the newest run under logs/.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

from pendel import latest_run, load_run


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?", help="run directory (default: newest under logs/)")
    ap.add_argument("--cols", help="comma-separated columns (default: all except x and raw clocks)")
    ap.add_argument("--x", default="t", help="x-axis column (default t)")
    ap.add_argument("--show", action="store_true", help="open a window as well")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = load_run(args.run or latest_run())
    x = run[args.x]
    cols = args.cols.split(",") if args.cols else [c for c in run.columns if c not in (args.x, "t_ns", "tick")]

    fig, axes = plt.subplots(len(cols), 1, sharex=True, figsize=(10, 2.2 * len(cols) + 0.8), squeeze=False)
    for ax, col in zip(axes[:, 0], cols):
        ax.plot(x, run[col], lw=0.8)
        ax.set_ylabel(col)
        ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel(args.x)
    m = run.meta
    fig.suptitle(f"{run.path.name}   git {m.get('git_commit')}{' (dirty)' if m.get('git_dirty') == 'true' else ''}",
                 fontsize=10)
    fig.tight_layout()
    out = run.path / "plot.png"
    fig.savefig(out, dpi=110)
    print(f"{len(run.data)} records, columns {', '.join(run.columns)} -> {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
