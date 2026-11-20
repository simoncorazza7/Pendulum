"""Replay the offset observer on logged balance sessions (open loop: the logged runs were balanced by the rig's
own controller; the observer only watches). Ground truth per balance stretch is the drift identity of
py/stage1_report.py (quiet_offsets): v - v0 - g int(theta) + L_eq (theta' - theta'0) + L_eq (b/J)(theta - theta0)
grows at g delta whatever the controller does; it is what predicted the 2026-11-04 parking offset and validated the
2026-10-14 trim (docs/stage1_demo.md). PC only, reads logs/, writes nothing.
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import dataclass

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pendel.runs import parse_conf  # noqa: E402

from . import identity as di  # noqa: E402
from . import observer as ob  # noqa: E402

G = 9.81
COLS = ("t_ns", "phase", "attempt", "mode", "theta", "omega", "x", "v", "a_cmd", "swing_phase")


@dataclass
class Session:
    name: str
    cfg: dict
    t: np.ndarray            # s from the first tick
    cols: dict               # column -> array
    segs: list               # [(i0, i1)] balance stretches (hand catch: ENGAGED; swing-up: BALANCE)
    kind: str                # "hand" | "swing-up"

    def f(self, key: str, default: float = float("nan")) -> float:
        try:
            return float(self.cfg[key])
        except (KeyError, ValueError):
            return default


def segments(mask: np.ndarray, min_len: int = 10) -> list[tuple[int, int]]:
    e = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(int), [0]))))
    return [(int(a), int(b)) for a, b in zip(e[::2], e[1::2]) if b - a > min_len]


def load(run_dir: str | pathlib.Path, cache: pathlib.Path | None = None) -> Session:
    p = pathlib.Path(run_dir)
    cfg = parse_conf((p / "config_used.conf").read_text(encoding="utf-8"))
    header = (p / "data.csv").open(encoding="utf-8").readline().strip().split(",")
    use = [c for c in COLS if c in header]
    arr = None
    if cache is not None:
        cf = cache / f"{p.name}.npz"
        if cf.exists():
            z = np.load(cf)
            if all(c in z.files for c in use):
                arr = {c: z[c] for c in z.files}
    if arr is None:
        raw = np.loadtxt(p / "data.csv", delimiter=",", skiprows=1, usecols=[header.index(c) for c in use], ndmin=2)
        arr = {c: raw[:, i] for i, c in enumerate(use)}
        if cache is not None:
            cache.mkdir(parents=True, exist_ok=True)
            np.savez(cache / f"{p.name}.npz", **arr)
    t = (arr["t_ns"] - arr["t_ns"][0]) * 1e-9
    swing = cfg.get("balance.swingup", "false") == "true"
    if swing and "swing_phase" in arr:
        segs = segments(arr["swing_phase"] == 2)
        kind = "swing-up"
    else:
        segs = segments(arr["mode"] == 2)
        kind = "hand"
    return Session(p.name, cfg, t, arr, segs, kind)


def ground_truth(s: Session, w0: float, bJ: float) -> tuple[list, list]:
    """Drift-identity offsets: per stretch [(seg index, start s, length s, delta rad)] and per segment the
    length-weighted mean (nan where no quiet stretch)."""
    import stage1_report as sr                      # quiet_offsets: the rig-validated method
    d = {"theta": s.cols["theta"], "omega": s.cols["omega"], "v": s.cols["v"], "attempt": np.arange(len(s.t))}
    per_seg = []
    stretches = []
    for j, (i0, i1) in enumerate(s.segs):
        q = sr.quiet_offsets(s.t, d, [(i0, i1)], w0, bJ)
        for _, t0, ln, slope in q:
            stretches.append((j, t0, ln, slope / G))
        if q:
            w = np.array([ln for _, _, ln, _ in q])
            per_seg.append(float(np.sum(w * np.array([sl for _, _, _, sl in q])) / w.sum() / G))
        else:
            per_seg.append(float("nan"))
    return stretches, per_seg


def cart_integral_offset(s: Session, i0: int, i1: int) -> float:
    """Stage 3b's own offset estimate at the end of a stretch: the lean its cart integral settled on, ki x_int / g
    (pendel.control.cascade_step's anti-windup rule recomputed from the log). nan without a cart integral."""
    kx, kv, ki = s.f("ctrl.cart_kx", 0.0), s.f("ctrl.cart_kv", 0.0), s.f("ctrl.cart_ki", 0.0)
    if ki <= 0:
        return float("nan")
    th_max = math.radians(s.f("ctrl.cart_theta_max_deg", 4.0))
    lim = G * math.radians(s.f("ctrl.cart_i_max_deg", 2.0)) / ki
    ts = s.f("loop.ts", 0.002)
    x, v = s.cols["x"][i0:i1], s.cols["v"][i0:i1]
    xi = 0.0
    for k in range(len(x)):
        raw = (-kx * x[k] - kv * v[k] - ki * xi) / G
        if abs(raw) <= th_max:
            xi = min(lim, max(-lim, xi + x[k] * ts))
    return ki * xi / G


def replay_identity(s: Session, p: di.IdentityParams, prior: float = 0.0, prior_weight: float = 0.0,
                    est: di.DriftIdentityEstimator | None = None) -> list[dict]:
    """The online drift-identity estimator through every balance stretch of a session, its pool carried from one
    balance to the next. Per balance: the carried estimate and its weight at the start, and both at the end."""
    est = est or di.DriftIdentityEstimator(p, prior, prior_weight)
    th, om, v = s.cols["theta"], s.cols["omega"], s.cols["v"]
    out = []
    for i0, i1 in s.segs:
        rec = {"start": est.delta_hat(), "w_start": est.weight(), "len": s.t[i1 - 1] - s.t[i0]}
        for k in range(i0, i1):
            est.step(s.t[k] - s.t[i0], float(th[k]), float(om[k]), float(v[k]), True)
        est.end_balance()
        rec["end"], rec["w_end"] = est.delta_hat(), est.weight()
        out.append(rec)
    return out


def replay(s: Session, p: ob.ObserverParams, prior: float = 0.0, marks=(0.5, 1, 2, 3, 5, 10)) -> list[dict]:
    """Run the observer through every balance stretch of a session, carrying delta_hat from one to the next
    (prior: delta_hat before the first). Per stretch: delta_hat at its start (the carried value), after `marks`
    seconds of learning, and at its end; plus the rate RMS difference to the logged BD+LP2 estimate."""
    K = ob.gains(p)
    st = ob.with_delta(ob.ObserverState(), p, prior)
    th, om, v = s.cols["theta"], s.cols["omega"], s.cols["v"]
    out = []
    for i0, i1 in s.segs:
        st = ob.restart(st, float(th[i0]), float(v[i0]), float(om[i0]))
        rec = {"start": ob.delta_hat(p, st), "len": s.t[i1 - 1] - s.t[i0], "at": {}, "learned": 0.0}
        t_learn = 0.0
        dom = []
        for k in range(i0 + 1, i1):
            te = s.t[k] - s.t[i0]
            learn = ob.learn_gate(p, te, st)
            st = ob.step(p, K, st, float(th[k]), float(v[k]), learn)
            if learn:
                t_learn += p.ts
                for m in marks:
                    if m not in rec["at"] and t_learn >= m:
                        rec["at"][m] = ob.delta_hat(p, st)
            if te > 0.5:
                dom.append(st.omega - om[k])
        rec["end"] = ob.delta_hat(p, st)
        rec["learned"] = t_learn
        rec["omega_rms_diff"] = float(np.sqrt(np.mean(np.square(dom)))) if dom else float("nan")
        out.append(rec)
    return out
