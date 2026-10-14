"""A/B of the ~28 Hz oscillation across balance sessions (e.g. ctrl.est_fc 20 vs 14 Hz).

    .venv/Scripts/python py/osc_compare.py RUN_DIR [RUN_DIR ...]

Per ENGAGED run, from 0.5 s after engage to 0.3 s before the end (the final fall):
  band     20-40 Hz content of the encoder counts (zero-phase band-pass), RMS per 0.5 s window, counts
  a band   the same band of a_cmd, m/s^2
  sat      fraction of samples at the a_max limit
A run counts as GROWN when a 0.5 s window has a counts band RMS above 1.0 (a quiet +-1-count limit cycle gives
0.1-0.5). Saturation is reported but not used: a hand still on the rod or a push saturates the command too, at
low frequency, which the 20-40 Hz band ignores.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np
from scipy.signal import butter, filtfilt

from pendel.runs import load_run

TS = 0.002
B, A = butter(2, [20 * 2 * TS, 40 * 2 * TS], btype="band")
WIN = 250                                                  # 0.5 s


def runs_of(path):
    run = load_run(path)
    d = run.data
    fc = run.config.get("ctrl.est_fc", "?")
    a_max = float(run.config.get("balance.a_max", 2.0))
    eng = d["mode"] == 2
    idx = np.flatnonzero(np.diff(np.concatenate(([0], eng.astype(int), [0]))))
    out = []
    for i0, i1 in zip(idx[::2], idx[1::2]):
        a0, a1 = i0 + int(0.5 / TS), i1 - int(0.3 / TS)
        if a1 - a0 < WIN + 60:
            continue
        c = filtfilt(B, A, d["counts"][a0:a1].astype(float))
        ab = filtfilt(B, A, d["a_cmd"][a0:a1])
        sat = np.abs(d["a_cmd"][a0:a1]) >= 0.99 * a_max
        w = [(c[k:k + WIN].std(), ab[k:k + WIN].std(), sat[k:k + WIN].mean()) for k in range(0, len(c) - WIN + 1, WIN)]
        cmax, amax, smax = (max(x[j] for x in w) for j in range(3))
        grown = cmax > 1.0
        out.append((int(d["attempt"][i0]), (a1 - a0) * TS, np.median([x[0] for x in w]), cmax,
                    np.median([x[1] for x in w]), amax, smax, grown))
    return run.path.name, fc, out


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    for path in sys.argv[1:]:
        name, fc, rows = runs_of(path)
        n_grown = sum(r[-1] for r in rows)
        print(f"\n{name} (velocity filter {fc} Hz): {n_grown} of {len(rows)} runs grown")
        print("| run | analysed (s) | counts band RMS median / max | a_cmd band RMS median / max (m/s²) | "
              "max saturated per 0.5 s | grown |")
        print("|---|---|---|---|---|---|")
        for n, dur, cmed, cmax, amed, amx, smax, grown in rows:
            print(f"| {n} | {dur:.1f} | {cmed:.2f} / {cmax:.2f} | {amed:.2f} / {amx:.2f} | {smax:.0%} | "
                  f"{'GROWN' if grown else ''} |")


if __name__ == "__main__":
    main()
