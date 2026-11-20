"""The upright-offset estimate on logged rig sessions (open loop; simulation prototype, PC only).
docs/offset_estimator.md. Needs the run directories under logs/ (not in git).

    .venv/Scripts/python py/offset_check.py replay [RUN ...] [--cache DIR] [--w-use 3]
        Every balance of every session (hand catches: ENGAGED; swing-ups: BALANCE) through
        offset.identity.DriftIdentityEstimator, its pool carried from one balance to the next. Per balance: the
        offset carried from the earlier balances of the session (used once >= w-use s of quiet balance stood
        behind it) against that balance's own offset from the drift identity (py/stage1_report.py quiet_offsets,
        the rig-validated method). Summary: mean |offset| a catch starts with, without and with the carry.
    .venv/Scripts/python py/offset_check.py centres RUN ... [--cache DIR]
        Swing-up sessions: the hanging equilibrium after each run (free swing, cart at rest, fitted on the encoder
        edges like the start-up calibration) against the start-up c_bottom fit. A count reference that moved
        between runs (lost edges in fast rotation) would show here.
--cache DIR keeps the parsed columns as .npz (the CSVs are 10-70 MB each).
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from offset import identity as di
from offset import replay as rp
from pendel.swingfit import fit_damped

LOGS = pathlib.Path(__file__).resolve().parents[1] / "logs"
CNT = di.COUNT


def sessions(names: list[str]) -> list[pathlib.Path]:
    if names:
        return [pathlib.Path(n) if pathlib.Path(n).is_dir() else LOGS / n for n in names]
    return sorted(d for d in LOGS.iterdir() if d.is_dir() and d.name.endswith("_balance"))


def cmd_replay(args) -> int:
    cache = pathlib.Path(args.cache) if args.cache else None
    rows = []
    print("| session | kind | plant w0 / b/J | balances with an own offset: own / carried (counts, weight s) |")
    print("|---|---|---|---|")
    for path in sessions(args.runs):
        s = rp.load(path, cache)
        if not s.segs:
            continue
        w0, bJ = s.f("plant.w0", 4.931), s.f("plant.bJ", 0.024)
        _, own = rp.ground_truth(s, w0, bJ)
        rec = rp.replay_identity(s, di.IdentityParams(w0=w0, bJ=bJ))
        pi = [r for r in rec if "pi_end" in r]
        if pi:                                       # a run with offset.mode watch/apply: the Pi's estimate
            worst = max(abs(r["pi_end"] - r["end"]) for r in pi) / CNT
            print(f"| {path.name[:19]} | the Pi's own estimate after each balance | | "
                  + "; ".join(f"#{j + 1} {r['pi_end'] / CNT:+.2f} ({r['pi_w_end']:.0f} s), replay {r['end'] / CNT:+.2f}"
                              for j, r in enumerate(rec) if "pi_end" in r)
                  + f"; worst difference {worst:.3f} counts |")
        cells = []
        for j, (r, g) in enumerate(zip(rec, own)):
            if math.isnan(g):
                continue
            carried = r["start"] if r["w_start"] >= args.w_use else 0.0
            rows.append((s.kind, g / CNT, carried / CNT, r["w_start"] >= args.w_use))
            cells.append(f"#{j + 1} {g / CNT:+.2f} / {carried / CNT:+.2f} ({r['w_start']:.0f})")
        if cells:
            print(f"| {path.name[:19]} | {s.kind} | {w0:.3f} / {bJ:.3f} | {'; '.join(cells)} |")
    if not rows:
        print("no balances with a quiet stretch")
        return 1
    g = np.array([r[1] for r in rows])
    c = np.array([r[2] for r in rows])
    used = np.array([r[3] for r in rows])
    kind = np.array([r[0] for r in rows])
    print(f"\nA balance starts with the offset it has (without carry) or with own - carried (with carry). "
          f"Carried once >= {args.w_use:g} s of quiet balance stood behind it.\n")
    print("| balances | n | mean abs offset without / with carry (counts) | median without / with | worse with carry |")
    print("|---|---|---|---|---|")
    for lab, m in (("all with an own offset", np.ones(len(g), bool)), ("those with a carried estimate", used),
                   ("... hand catches", used & (kind == "hand")), ("... swing-up hand-overs", used & (kind == "swing-up"))):
        if not m.any():
            continue
        a, b = np.abs(g[m]), np.abs(g[m] - c[m])
        print(f"| {lab} | {m.sum()} | {a.mean():.2f} / {b.mean():.2f} | {np.median(a):.2f} / {np.median(b):.2f} | "
              f"{np.sum(b > a + 1e-9)} |")
    return 0


def cmd_centres(args) -> int:
    cache = pathlib.Path(args.cache) if args.cache else None
    for path in sessions(args.runs):
        s = rp.load(path, cache)
        if "swing_phase" not in s.cols:
            continue
        meta = json.loads((path / "meta.json").read_text(encoding="utf-8"))
        e = np.loadtxt(path / "edges.csv", delimiter=",", skiprows=1, ndmin=2)
        et, ec = e[:, 0] * 1e-9, e[:, 1]
        tns = s.cols["t_ns"] * 1e-9
        # after a run: no swing-up phase, session AFTER_RUN (4), cart standing
        still = (s.cols["swing_phase"] == -1) & (s.cols["phase"] == 4) & (np.abs(s.cols["v"]) < 1e-9)
        print(f"{path.name}: start-up c_bottom fit {meta.get('c_bottom_fit_counts')} counts. Hanging equilibrium "
              f"after each run (first 3 s of free swing):")
        for i0, i1 in rp.segments(still, 250):
            m = (et >= tns[i0] + 0.3) & (et <= min(tns[i1 - 1], tns[i0] + 3.3))
            if m.sum() < 30:
                continue
            cc = ec[m]
            prev = np.concatenate(([cc[0]], cc[:-1]))
            u = np.where(cc > prev, cc - 0.5, cc + 0.5)          # the boundary each edge crossed
            try:
                f = fit_damped(et[m] - et[m][0], u)
            except (RuntimeError, ValueError):
                continue
            ok = f.se["c"] < 0.1 and f.rms < 1.0
            amp = 0.5 * (cc.max() - cc.min()) * 360 / 2400
            print(f"  t {tns[i0] - tns[0]:6.1f} s  centre {((f.c + 1200) % 2400) - 1200:+7.2f}  se {f.se['c']:.2f}  "
                  f"amp {amp:4.1f} deg  rms {f.rms:.2f}{'' if ok else '  (poor fit, ignore)'}")
    return 0


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("replay")
    r.add_argument("runs", nargs="*")
    r.add_argument("--cache")
    r.add_argument("--w-use", type=float, default=3.0)
    c = sub.add_parser("centres")
    c.add_argument("runs", nargs="+")
    c.add_argument("--cache")
    args = ap.parse_args()
    return {"replay": cmd_replay, "centres": cmd_centres}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
