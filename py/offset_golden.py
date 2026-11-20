"""Golden data for the C++ offset estimator (src/core/offset_est.*, tests/unit/test_offset_est.cpp).

    .venv/Scripts/python py/offset_golden.py

Runs offset.identity.DriftIdentityEstimator on a fixed input sequence and writes, with repr() (shortest exact
round-trip), the parameters (tests/golden/offset_params.csv) and per tick the inputs and the outputs
(tests/golden/offset_golden.csv). The C++ estimator must reproduce the outputs to floating-point tolerance.

The sequence: a linear pendulum held by a PD on the quantised measured angle, true offset 1.3 counts; a push at
4 s; a measured excursion past theta_max at 6 s; the balance ends at 8 s (end_balance); 0.5 s not balancing; a
second balance of 4 s from t_engaged 0 (the carry). Case "prior" repeats it with a prior of 1.0 count, weight 5 s.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from offset.identity import COUNT, DriftIdentityEstimator, IdentityParams

ROOT = pathlib.Path(__file__).resolve().parents[1]
W0, BJ, G, TS = 4.97, 0.054, 9.81, 0.002


def inputs(seed: int = 3) -> list[tuple[float, float, float, float, int, int]]:
    """[(t_engaged, theta_m, omega, v, balancing, end)] per tick."""
    rng = np.random.default_rng(seed)
    delta, h = 1.3 * COUNT, TS / 4
    th, om, v = 0.0, 0.0, 0.0
    rows = []

    def balance(t_len: float, push_at: float | None, excursion_at: float | None):
        nonlocal th, om, v
        n = int(round(t_len / TS))
        for k in range(n):
            t = k * TS
            thm = (math.floor((th - delta) / COUNT + 0.37) - 0.37 + 0.5) * COUNT
            a = 60.0 * thm + 6.5 * om + float(rng.normal(0.0, 0.05))
            seen = thm
            if excursion_at is not None and excursion_at <= t < excursion_at + 0.04:
                seen = math.radians(3.5)                       # a measured excursion past theta_max
            rows.append((t, seen, om, v, 1, 1 if k == n - 1 else 0))
            tau = 2.0 if push_at is not None and push_at <= t < push_at + 0.1 else 0.0
            for _ in range(4):
                acc = W0 ** 2 * th - (W0 ** 2 / G) * a - BJ * om + tau
                om += acc * h
                th += om * h
                v += a * h

    balance(8.0, push_at=4.0, excursion_at=6.0)
    for k in range(int(round(0.5 / TS))):                      # not balancing: the estimator holds
        rows.append((0.0, 0.0, 0.0, v, 0, 0))
    th, om = 0.0, 0.0
    balance(4.0, push_at=None, excursion_at=None)
    return rows


def main() -> None:
    p = IdentityParams(w0=W0, bJ=BJ, ts=TS)
    names = ["w0", "bJ", "ts", "g", "t_settle", "theta_max", "lag", "smooth", "step", "min_stretch", "delta_max"]
    cases = [("plain", 0.0, 0.0), ("prior", 1.0 * COUNT, 5.0)]
    lines = ["case," + ",".join(names) + ",prior,prior_weight"]
    for name, prior, pw in cases:
        lines.append(name + "," + ",".join(repr(float(getattr(p, f))) for f in names) + f",{prior!r},{float(pw)!r}")
    (ROOT / "tests/golden/offset_params.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

    rows = inputs()
    out = ["case,k,t_engaged,theta,omega,v,balancing,end,slope,weight,delta_hat,stretches"]
    for name, prior, pw in cases:
        est = DriftIdentityEstimator(p, prior, pw)
        for k, (t, th, om, v, bal, end) in enumerate(rows):
            est.step(t, th, om, v, bool(bal))
            if end:
                est.end_balance()
            out.append(f"{name},{k},{t!r},{th!r},{om!r},{v!r},{bal},{end},{est.slope()!r},{est.weight()!r},"
                       f"{est.delta_hat()!r},{est.n_stretches}")
        print(f"{name}: {len(rows)} ticks, end: delta_hat {est.delta_hat() / COUNT:+.4f} counts, "
              f"weight {est.weight():.2f} s, {est.n_stretches} stretches")
    (ROOT / "tests/golden/offset_golden.csv").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("wrote tests/golden/offset_params.csv, tests/golden/offset_golden.csv")


if __name__ == "__main__":
    main()
