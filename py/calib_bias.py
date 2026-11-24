"""Start-up c_bottom calibration on the 2026-11-02 pivot: is the fit biased? (log analysis and simulation, PC only)

docs/calib_bias.md. Reads logs/ (not in git), writes nothing, talks to nothing.

    .venv/Scripts/python py/calib_bias.py sessions [RUN ...]
        The start-up calibration of every new-pivot balance session, replayed exactly (rt::SwingCalibrator,
        core::fitOffset; checked against meta.json), against the turning-point centre of the same swing.
        Swing-up sessions: the hanging centre after every run, as `offset_check.py centres` fits it (its edge
        convention and the calibrator's) and from the turning points.
    .venv/Scripts/python py/calib_bias.py bench
        The two 2026-11-02 free decays (encoder-log, driver off): every 3 s window, the start-up fit and its
        variants against the turning-point centre, by amplitude.
    .venv/Scripts/python py/calib_bias.py synth
        Synthetic swings with a known centre (nonlinear pendulum, viscous + Coulomb, encoder line errors): the
        turning-point centre and the fit variants. About 30 minutes (fit_damped on ~5000 windows).
    .venv/Scripts/python py/calib_bias.py fix [RUN ...]
        The proposed turning-point calibration, next to today's: on the sessions (what it would have accepted,
        when), on the two decays and on synthetic swings (released at many points of the swing).

Edge convention (the calibrator's, core::thetaFromCounts' and pendel.decoders'): count = floor(u); an edge that
raised the count to n is at u = n, one that lowered it to n at u = n + 1.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from pendel import load_run
from pendel.decoders import decode
from pendel.runs import parse_conf
from pendel.swingfit import fit_damped

REPO = pathlib.Path(__file__).resolve().parents[1]
LOGS = REPO / "logs"
CPR = 2400
CPD = CPR / 360.0                                   # counts per degree
BENCH = (("2026-11-02T15-43-11_encoder-log", 2.0, 29.0), ("2026-11-02T15-27-20_encoder-log", 0.3, 16.5))
BANDS = ((15, 20), (10, 15), (5, 10))


# ------------------------------------------------------------------------------------------------ logs
def edges(ts_ns: np.ndarray, count_after: np.ndarray, prev0: int = 0):
    """Edge times (ns), positions u (counts, floor convention) and directions d (+1 up, -1 down)."""
    prev = np.concatenate(([prev0], count_after[:-1]))
    step = count_after - prev
    ok = step != 0
    d = np.sign(step[ok]).astype(int)
    c = count_after[ok]
    return ts_ns[ok], np.where(d > 0, c, c + 1).astype(float), d


def load_session(path: pathlib.Path) -> dict:
    cfg = parse_conf((path / "config_used.conf").read_text(encoding="utf-8"))
    meta = json.loads((path / "meta.json").read_text(encoding="utf-8"))
    hdr = (path / "data.csv").open(encoding="utf-8").readline().strip().split(",")
    use = [c for c in ("tick", "t_ns", "phase", "sevent", "counts", "swing_phase", "x", "v", "mode", "theta", "omega")
           if c in hdr]
    idx = [hdr.index(c) for c in use]
    rows = []
    with (path / "data.csv").open(encoding="utf-8") as f:
        next(f)
        for line in f:
            q = line.split(",")
            rows.append([float(q[i]) for i in idx])
    a = np.array(rows)
    d = {c: a[:, i] for i, c in enumerate(use)}
    e = np.loadtxt(path / "edges.csv", delimiter=",", skiprows=1, dtype=np.int64, ndmin=2)
    return {"name": path.name, "cfg": cfg, "meta": meta, "d": d,
            "e": {"ts_ns": e[:, 0], "count_after": e[:, 1], "tick": e[:, 2]}}


def load_encoder_log(name: str):
    run = load_run(LOGS / name)
    r = run.data
    o = np.argsort(r["ts_ns"], kind="stable")
    counts, steps, _ = decode(r["line"][o].astype(int), r["type"][o].astype(int),
                              int(run.meta["start_level_a"]), int(run.meta["start_level_b"]))
    real = steps != 0
    ts = r["ts_ns"][o][real]
    return (ts - ts[0]) * 1e-9, np.where(steps > 0, counts, counts + 1)[real].astype(float), steps[real].astype(int)


def sessions(names: list[str]) -> list[pathlib.Path]:
    if names:
        return [pathlib.Path(n) if pathlib.Path(n).is_dir() else LOGS / n for n in names]
    return sorted(p for p in LOGS.iterdir() if p.is_dir() and p.name.endswith("_balance") and p.name >= "2026-11-02")


# ------------------------------------------------------------------------------------------------ today's start-up fit
@dataclass
class CalibParams:                                  # rt::SwingCalibParams
    w0: float = 4.97
    sigma: float = 0.12
    window: float = 3.0
    interval: float = 0.5
    amp_min: float = 2 * CPD
    amp_max: float = 20 * CPD
    rms_max: float = 2.0
    se_max: float = 0.05
    agree: float = 0.2

    @classmethod
    def from_cfg(cls, cfg: dict) -> "CalibParams":
        g = lambda k: float(cfg[k])  # noqa: E731
        return cls(g("plant.w0"), g("calib.swing_sigma"), g("calib.window"), 0.5, g("calib.amp_min_deg") * CPD,
                   g("calib.amp_max_deg") * CPD, g("calib.rms_max"), g("calib.se_max"), g("calib.agree"))


def fit_offset(t, u, w, sigma):
    """core::fitOffset: u = c + e^{-sigma t}(a cos wt + b sin wt) by linear least squares. Returns c, se, rms."""
    e = np.exp(-sigma * t)
    return fit_lin(u, [np.ones_like(t), e * np.cos(w * t), e * np.sin(w * t)])


def fit_lin(u, cols):
    P = np.stack(cols)
    N = P @ P.T
    try:
        p = np.linalg.solve(N, P @ u)
    except np.linalg.LinAlgError:
        return float("nan"), float("nan"), float("nan")
    r = u - p @ P
    s2 = float(r @ r) / max(1, len(u) - len(cols))
    return float(p[0]), math.sqrt(max(0.0, float(np.linalg.inv(N)[0, 0]) * s2)), math.sqrt(s2)


def w_of_amp(w0: float, amp_counts: float) -> float:
    a = amp_counts * 2 * math.pi / CPR
    return w0 * (1 - a * a / 16)


def _judge(tt, uu, p: CalibParams):
    """One rt::SwingCalibrator::fitNow on the window's edges: (fit or None, reason)."""
    n = len(tt)
    if n < 40:
        return None, "few edges"
    amp = 0.5 * (uu.max() - uu.min())
    if amp < p.amp_min:
        return None, "small"
    if amp > p.amp_max:
        return None, "large"
    if tt[-1] - tt[0] < 0.8 * p.window:
        return None, "short"
    w = w_of_amp(p.w0, amp)
    c, se, rms = fit_offset(tt - tt[0], uu, w, p.sigma)
    if not math.isfinite(c):
        return None, "singular"
    if rms > p.rms_max:
        return None, "rms"
    if se > p.se_max:
        return None, "se"
    return {"c": c, "se": se, "rms": rms, "amp": amp, "w": w}, "ok"


def replay_startup(s: dict, log: list | None = None) -> dict | None:
    """rt::SwingCalibrator fed sample by sample, exactly as the session fed it (until the Calibrated event).
    `log` collects (t s, outcome, c or nan, amp counts) of every fit."""
    d, e = s["d"], s["e"]
    p = CalibParams.from_cfg(s["cfg"])
    acc = np.flatnonzero(d["sevent"] == 1)
    if not len(acc):
        return None
    ticks, tns = d["tick"].astype(np.int64), d["t_ns"].astype(np.int64)
    ek, ets, eca = e["tick"], e["ts_ns"], e["count_after"]
    j, prev = 0, 0
    T: list[int] = []
    U: list[float] = []
    next_fit, prev_c = 0, None
    for k in range(int(acc[0]) + 1):
        while j < len(ek) and ek[j] <= ticks[k]:
            step = int(eca[j]) - prev
            prev = int(eca[j])
            if step:
                T.append(int(ets[j]))
                U.append(float(eca[j] if step > 0 else eca[j] + 1))
            j += 1
        prev = int(d["counts"][k])
        if next_fit == 0:
            next_fit = int(tns[k]) + int(p.interval * 1e9)
        if tns[k] < next_fit:
            continue
        next_fit = int(tns[k]) + int(p.interval * 1e9)
        i0 = int(np.searchsorted(np.asarray(T, dtype=np.int64), int(tns[k]) - int(p.window * 1e9))) if T else 0
        del T[:i0], U[:i0]
        tt, uu = (np.asarray(T, dtype=np.int64) - T[0]) * 1e-9 if T else np.zeros(0), np.asarray(U)
        f, why = _judge(tt, uu, p) if len(T) else (None, "few edges")
        if log is not None:
            log.append(((tns[k] - d["t_ns"][0]) * 1e-9, why if f is None else
                        ("agree" if prev_c is not None and abs(f["c"] - prev_c) < p.agree else
                         "first" if prev_c is None else "disagree"),
                        f["c"] if f else float("nan"), f["amp"] if f else float("nan")))
        if f is None:
            prev_c = None
            continue
        if prev_c is not None and abs(f["c"] - prev_c) < p.agree:
            t0 = d["t_ns"][0]
            return {**f, "t_acc": (tns[k] - t0) * 1e-9, "win": ((T[0] - t0) * 1e-9, (T[-1] - t0) * 1e-9),
                    "params": p}
        prev_c = f["c"]
    return None


def swing_calibrator(t, u, p: CalibParams, t_start: float):
    """The same logic on bare edges (no samples): fits every `interval` from t_start. (c, t_acc, amp) or None."""
    tf = t_start + p.interval
    prev_c = None
    while tf <= t[-1]:
        i0, i1 = int(np.searchsorted(t, max(t_start, tf - p.window))), int(np.searchsorted(t, tf, side="right"))
        f, _ = _judge(t[i0:i1], u[i0:i1], p) if i1 > i0 else (None, "")
        if f is None:
            prev_c = None
        elif prev_c is not None and abs(f["c"] - prev_c) < p.agree:
            return f["c"], tf, f["amp"]
        else:
            prev_c = f["c"]
        tf += p.interval
    return None


# ------------------------------------------------------------------------------------------------ turning points
def turning_points(t, u, d, k_max: int = 8, min_pairs: int = 3):
    """Extrema from symmetric boundary-crossing pairs. Around a maximum the boundaries n, n-1, ... are crossed
    up at t_up and down at t_dn. With a constant (but different) acceleration on each side,
    n_k = x_ext - kappa (t_dn - t_up)^2, so the intercept of n_k against dt^2 is the extremum (friction that
    differs before and after the turn only changes kappa). Returns t_ext, x_ext, sign (+1 maximum), known_at
    (time of the last edge used), rms of the regression."""
    out = []
    for i in np.flatnonzero(d[1:] != d[:-1]):
        s = d[i]
        if u[i] != u[i + 1]:
            continue
        nk, dt, last = [], [], t[i + 1]
        for k in range(k_max):
            a, b = i - k, i + 1 + k
            if a < 0 or b >= len(u) or d[a] != s or d[b] != -s or u[a] != u[i] - s * k or u[b] != u[i] - s * k:
                break
            nk.append(u[a])
            dt.append(t[b] - t[a])
            last = t[b]
        if len(nk) < min_pairs:
            continue
        nk, dt = np.asarray(nk), np.asarray(dt)
        A = np.column_stack([np.ones_like(dt), dt * dt])
        coef, *_ = np.linalg.lstsq(A, nk, rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - nk) ** 2)))
        out.append((0.5 * (t[i] + t[i + 1]), coef[0], s, last, rms))
    if not out:
        return tuple(np.zeros(0) for _ in range(5))
    o = np.asarray(out)
    return o[:, 0], o[:, 1], o[:, 2].astype(int), o[:, 3], o[:, 4]


def triples(te, xe, se, max_gap: float = 0.9):
    """Hanging centre from three consecutive extrema, (x1 + 2 x2 + x3)/4: exact for a linear oscillator with
    Coulomb friction (each half swing is a cosine about c + Delta or c - Delta), (1 - r)^2-small with viscous
    friction. Returns t of x2, centre, half-amplitude (counts), index of x3."""
    out = []
    for j in range(1, len(xe) - 1):
        if se[j - 1] == -se[j] == se[j + 1] and te[j + 1] - te[j - 1] <= 2 * max_gap:
            out.append((te[j], 0.25 * (xe[j - 1] + 2 * xe[j] + xe[j + 1]),
                        0.25 * abs(xe[j - 1] - 2 * xe[j] + xe[j + 1]), j + 1))
    if not out:
        return tuple(np.zeros(0) for _ in range(4))
    o = np.asarray(out)
    return o[:, 0], o[:, 1], o[:, 2], o[:, 3].astype(int)


@dataclass
class TPParams:                                     # the proposed start-up calibration
    n: int = 4                  # consecutive centre estimates (6 extrema, 2.5 periods)
    agree: float = 0.25         # counts, max - min of those n estimates
    amp_min: float = 2 * CPD    # counts, half-amplitude of each estimate
    amp_max: float = 20 * CPD
    amp_rise: float = 1.0       # counts; a free decay never grows (a hand or a push does)
    max_gap: float = 0.9        # s between consecutive extrema (half period 0.63 s)
    ext_rms: float = 0.6        # counts, residual of each extremum's pair regression (clean turns 0.02-0.03; the
    #                             turn stall of some sessions 0.2-0.45; the agreement and decay checks do the rest)


def tp_calibrator(t, u, d, p: TPParams = TPParams(), t_start: float | None = None):
    """Causal: every extremum becomes known when its last pair edge arrives; accept at the first extremum after
    which the last n centre estimates come from one unbroken chain of clean extrema, decay, and agree.
    Returns (c, t_acc, amp_counts, list of the n estimates) or None."""
    if t_start is not None:
        m = t >= t_start
        t, u, d = t[m], u[m], d[m]
    te, xe, se, known, rms = turning_points(t, u, d)
    chain: list[int] = []                         # indices of the current unbroken chain of extrema
    for j in np.argsort(known, kind="stable"):
        ok = rms[j] <= p.ext_rms
        if chain and ok and se[j] == -se[chain[-1]] and te[j] - te[chain[-1]] <= p.max_gap:
            chain.append(j)
        else:
            chain = [j] if ok else []
            continue
        if len(chain) < p.n + 2:
            continue
        x = xe[chain[-(p.n + 2):]]
        c = 0.25 * (x[:-2] + 2 * x[1:-1] + x[2:])
        a = 0.25 * np.abs(x[:-2] - 2 * x[1:-1] + x[2:])
        if a.min() < p.amp_min or a.max() > p.amp_max or np.any(np.diff(a) > p.amp_rise):
            continue
        if c.max() - c.min() <= p.agree:
            return float(c.mean()), float(known[j]), float(a[-1]), c
    return None


# ------------------------------------------------------------------------------------------------ fit variants
VARIANTS = ("now", "sigma_free", "damped", "taylor", "dir", "2T", "3T", "tp")
LABEL = {"now": "today: w(A), sigma 0.12", "sigma_free": "sigma from a free fit", "damped": "fit_damped (all free)",
         "taylor": "1, cos, sin, t cos, t sin", "dir": "today + Coulomb column", "2T": "window = 2 periods",
         "3T": "window = 3 periods", "tp": "turning points in the window"}


def window_variants(t, u, d, tend, w0=4.97, sigma=0.12, window=3.0, amp_band=(2 * CPD, 20 * CPD)):
    m = (t > tend - window) & (t <= tend)
    if m.sum() < 40:
        return None
    tt, uu, dd = t[m] - t[m][0], u[m], d[m]
    amp = 0.5 * (uu.max() - uu.min())
    if not amp_band[0] <= amp <= amp_band[1]:
        return None
    w = w_of_amp(w0, amp)
    e = np.exp(-sigma * tt)
    one, cs, sn = np.ones_like(tt), np.cos(w * tt), np.sin(w * tt)
    tm = tt - 0.5 * tt[-1]
    out = {"amp": amp / CPD, "now": fit_offset(tt, uu, w, sigma)[0],
           "taylor": fit_lin(uu, [one, cs, sn, tm * cs, tm * sn])[0],
           "dir": fit_lin(uu, [one, e * cs, e * sn, -dd.astype(float)])[0]}
    try:
        g = fit_damped(tt, uu, w_guess=w)
        out["damped"] = g.c
        out["sigma_free"] = fit_offset(tt, uu, w, max(g.sigma, 0.0))[0]
    except (RuntimeError, ValueError, np.linalg.LinAlgError):
        out["damped"] = out["sigma_free"] = float("nan")
    for n in (2, 3):
        m2 = (t > tend - n * 2 * math.pi / w) & (t <= tend)
        out[f"{n}T"] = fit_offset(t[m2] - t[m2][0], u[m2], w, sigma)[0]
    _, cc, _, _ = triples(*turning_points(t[m], u[m], d[m])[:3])
    out["tp"] = float(np.mean(cc)) if len(cc) else float("nan")
    return out


def reference(t, u, d, t0: float | None = None, near: float | None = None, tol: float = 1.5):
    """Turning-point centre along a swing, smoothed over neighbouring estimates (max-first and min-first triples
    carry opposite small viscous terms): a function of time. Estimates from t0 on, within tol of `near`."""
    tc, cc, _, _ = triples(*turning_points(t, u, d)[:3])
    keep = np.ones(len(tc), bool)
    if t0 is not None:
        keep &= tc >= t0
    if near is not None:
        keep &= np.abs(cc - near) < tol
    tc, cc = tc[keep], cc[keep]
    if len(cc) < 2:
        return None
    cs, ts = 0.5 * (cc[1:] + cc[:-1]), 0.5 * (tc[1:] + tc[:-1])
    return lambda x: np.interp(x, ts, cs), ts[0], ts[-1]


def band_table(rows: list[dict], keys=VARIANTS) -> None:
    print("| fit | " + " | ".join(f"{lo}-{hi}°: mean / sd / max \\|e\\|" for lo, hi in BANDS) + " |")
    print("|---|" + "---|" * len(BANDS))
    for v in keys:
        cells = []
        for lo, hi in BANDS:
            e = np.array([r[v] - r["truth"] for r in rows if lo <= r["amp"] < hi and v in r])
            e = e[np.isfinite(e)]
            cells.append(f"{e.mean():+.2f} / {e.std():.2f} / {np.abs(e).max():.2f}" if len(e) else "–")
        print(f"| {LABEL[v]} | " + " | ".join(cells) + " |")


# ------------------------------------------------------------------------------------------------ synthetic swings
def synth_swing(phi0_deg, t_end, w0, beta, gamma, c_true, quad=0.0, seed=0, jitter=2e-6):
    """Nonlinear pendulum phi'' = -w0^2 sin phi - beta phi' - gamma sgn(phi'), edges of an encoder whose boundary
    n sits at n + quad (-1)^n + N(0, 0.02) (quadrature error and line spacing). Returns t, u (= n), d."""
    from scipy.integrate import solve_ivp

    def f(_, s):
        return [s[1], -w0 ** 2 * math.sin(s[0]) - beta * s[1] - gamma * math.tanh(s[1] / 1e-4)]
    tg = np.arange(0, t_end, 2e-5)
    sol = solve_ivp(f, (0, t_end), [math.radians(phi0_deg), 0.0], t_eval=tg, rtol=1e-10, atol=1e-12, max_step=1e-3)
    x = c_true + sol.y[0] * CPR / (2 * math.pi)
    rng = np.random.default_rng(seed)
    te, ue, de = [], [], []
    for n in range(int(np.floor(x.min())) - 1, int(np.ceil(x.max())) + 2):
        s = x - (n + quad * (-1) ** n + rng.normal(0, 0.02))
        for i in np.flatnonzero(np.sign(s[1:]) != np.sign(s[:-1])):
            te.append(tg[i] + s[i] / (s[i] - s[i + 1]) * (tg[i + 1] - tg[i]))
            ue.append(float(n))
            de.append(1 if s[i + 1] > s[i] else -1)
    o = np.argsort(te)
    return np.asarray(te)[o] + rng.normal(0, jitter, len(te)), np.asarray(ue)[o], np.asarray(de)[o]


SYNTH = (("11-02 pivot: b/J 0.054, Coulomb 0.168 rad/s², quadrature error 0.1 count", 0.054, 0.168, 0.1),
         ("Coulomb 0.25 rad/s² (50 % more friction)", 0.054, 0.25, 0.1),
         ("viscous only, b/J 0.054 (sigma 0.12 is then the wrong decay)", 0.054, 0.0, 0.1))
C_TRUE = (0.37, -0.81, 1.53)


# ------------------------------------------------------------------------------------------------ commands
def startup_swing(s: dict):
    """Edges from program start to the start of homing (the calibration swing; the operator stops it by hand
    after the Calibrated event)."""
    d = s["d"]
    t0 = d["t_ns"][0]
    t_home = (d["t_ns"][int(np.flatnonzero(d["phase"] == 2)[0])] - t0) * 1e-9
    ts, u, dd = edges(s["e"]["ts_ns"], s["e"]["count_after"])
    t = (ts - t0) * 1e-9
    m = t < t_home
    return t[m], u[m], dd[m], (ts - t0) * 1e-9, u, dd


def cmd_sessions(args) -> int:
    rows, after, felt = [], [], []
    print("| session | kind | amp ° | today's fit (meta) | replay - meta | TP, same window (n) | TP, whole swing (n) | "
          "today - TP window | today - TP swing | fit_damped - TP window |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for path in sessions(args.runs):
        s = load_session(path)
        r = replay_startup(s)
        if r is None:
            continue
        t, u, d, t_all, u_all, d_all = startup_swing(s)
        tc, cc, _, _ = triples(*turning_points(t, u, d)[:3])
        inwin = (tc > r["win"][0]) & (tc <= r["win"][1])
        tp = float(np.mean(cc[inwin])) if inwin.any() else float("nan")
        # the whole free swing from the window on (the operator stops it before homing), outliers > 1.5 counts out
        sw = (tc > r["win"][0]) & (np.abs(cc - tp) < 1.5)
        tps = float(np.median(cc[sw])) if sw.any() else float("nan")
        m = (t >= r["win"][0]) & (t <= r["win"][1])
        try:
            dmp = fit_damped(t[m] - t[m][0], u[m], w_guess=r["w"]).c
        except (RuntimeError, ValueError):
            dmp = float("nan")
        meta_c = float(s["meta"]["c_bottom_fit_counts"])
        swing = s["cfg"].get("balance.swingup", "false") == "true"
        rows.append((r["c"] - tp, r["c"] - tps, dmp - tp))
        felt.append((path.name, swing, r["c"] - tp, felt_offsets(s)))
        print(f"| {path.name[:19]} | {'swing-up' if swing else 'hand'} | {r['amp'] / CPD:.1f} | {meta_c:+.3f} | "
              f"{r['c'] - meta_c:+.0e} | {tp:+.3f} ({inwin.sum()}) | {tps:+.3f} ({sw.sum()}) | **{r['c'] - tp:+.2f}** | "
              f"{r['c'] - tps:+.2f} | {dmp - tp:+.2f} |")
        if swing and "swing_phase" in s["d"]:
            after.append((path.name, meta_c, r, tp, after_runs(s, t_all, u_all, d_all)))
    e = np.array(rows)
    for j, lab in ((0, "today - TP (same window)"), (1, "today - TP (whole swing)"), (2, "fit_damped - TP (same window)")):
        print(f"\n{len(e)} start-ups, {lab}: mean {np.nanmean(e[:, j]):+.3f}, sd {np.nanstd(e[:, j]):.3f}, max |e| "
              f"{np.nanmax(np.abs(e[:, j])):.2f} counts", end="")
    print()
    print("\nAfter every swing-up run (cart standing, free swing): `centres` fits fit_damped to the first 3 s with the "
          "edges at count -+ 0.5 (round); the calibrator puts them at count or count + 1 (floor), 0.5 count higher. "
          "TP = median turning-point centre of the whole free swing (n estimates).")
    print("\n| session | start-up fit | its TP | after-run fit_damped, round (= centres) | same, floor | "
          "after-run TP (n) | cart x mm |")
    print("|---|---|---|---|---|---|---|")
    for name, meta_c, r, tp, runs in after:
        ok = [a for a in runs if a["ok"]]
        print(f"| {name[:19]} | {meta_c:+.2f} | {tp:+.2f} | " +
              ", ".join(f"{a['round']:+.2f}" for a in ok) + " | " + ", ".join(f"{a['floor']:+.2f}" for a in ok) +
              " | " + ", ".join(f"{a['tp']:+.2f} ({a['n_tp']})" if a["n_tp"] else "–" for a in ok) + " | " +
              ", ".join(f"{a['x']:+.0f}" for a in ok) + " |")
    print("\nWhat the balances felt (drift identity, counts; + = c_bottom used above the true upright - 1200) next "
          "to the start-up fit's own error (today - TP, same window). Sessions with at least one quiet stretch:\n")
    print("| session | kind | start-up error | felt offset per balance (start s: counts) |")
    print("|---|---|---|---|")
    for name, swing, err, fo in felt:
        cells = [f"{a:.0f}: {g:+.2f}" for a, _, g in fo if math.isfinite(g)]
        if cells:
            print(f"| {name[:19]} | {'swing-up' if swing else 'hand'} | {err:+.2f} | {'; '.join(cells)} |")
    return 0


def felt_offsets(s: dict) -> list[tuple[float, float, float]]:
    """The offset each balance felt: the drift identity of py/stage1_report.py (quiet_offsets), as
    `offset_check.py replay` takes it. Per balance (start s, end s, counts; nan without a quiet stretch)."""
    from offset import identity as di
    from offset import replay as rp
    d = s["d"]
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    swing = s["cfg"].get("balance.swingup", "false") == "true" and "swing_phase" in d
    segs = rp.segments(d["swing_phase"] == 2) if swing else rp.segments(d["mode"] == 2)
    if not segs:
        return []
    ses = rp.Session(s["name"], s["cfg"], t, d, segs, "swing-up" if swing else "hand")
    _, own = rp.ground_truth(ses, ses.f("plant.w0", 4.931), ses.f("plant.bJ", 0.024))
    return [(float(t[i0]), float(t[i1 - 1]), g / di.COUNT) for (i0, i1), g in zip(segs, own)]


def after_runs(s: dict, t, u, d) -> list[dict]:
    """Hanging centre after each run: AFTER_RUN, no swing-up phase, cart standing (offset_check.py centres)."""
    dd = s["d"]
    tt = (dd["t_ns"] - dd["t_ns"][0]) * 1e-9
    still = (dd["swing_phase"] == -1) & (dd["phase"] == 4) & (np.abs(dd["v"]) < 1e-9)
    e = np.flatnonzero(np.diff(np.concatenate(([0], still.astype(int), [0]))))
    out = []
    for i0, i1 in zip(e[::2], e[1::2]):
        if i1 - i0 <= 250:
            continue
        m = (t >= tt[i0] + 0.3) & (t <= tt[i1 - 1])
        m3 = m & (t <= tt[i0] + 3.3)
        if m3.sum() < 30:
            continue
        base = CPR * round(float(np.median(u[m])) / CPR)
        try:
            f = fit_damped(t[m3] - t[m3][0], u[m3] - base)
        except (RuntimeError, ValueError):
            continue
        _, cc, aa, _ = triples(*turning_points(t[m], u[m] - base, d[m])[:3])
        cc = cc[aa <= 20 * CPD]
        out.append({"t": tt[i0], "floor": f.c, "round": f.c - 0.5, "ok": f.se["c"] < 0.1 and f.rms < 1.0,
                    "tp": float(np.median(cc)) if len(cc) else float("nan"), "n_tp": len(cc), "x": dd["x"][i0] * 1e3})
    return out


def cmd_bench(args) -> int:
    for name, t0, t1 in BENCH:
        t, u, d = load_encoder_log(name)
        m = (t >= t0) & (t <= t1)
        t, u, d = t[m], u[m], d[m]
        ref = reference(t, u, d)
        tc, cc, aa, _ = triples(*turning_points(t, u, d)[:3])
        rows = []
        for tend in np.arange(t[0] + 3.0, t[-1], 0.02):
            r = window_variants(t, u, d, tend)
            if r is not None and ref[1] <= tend - 1.5 <= ref[2]:
                r["truth"] = float(ref[0](tend - 1.5))
                rows.append(r)
        k = (aa >= 5 * CPD) & (aa <= 20 * CPD)
        print(f"\n{name}: free decay, driver off. Turning-point centre {cc[k].mean():+.2f} (sd {cc[k].std():.2f}, "
              f"{k.sum()} estimates at 5-20°; {cc[aa > 20 * CPD].mean():+.2f} above 20°). {len(rows)} windows of 3 s "
              f"(every 20 ms), error against the turning-point centre at the window's middle (counts):\n")
        band_table(rows)
    return 0


def cmd_synth(args) -> int:
    for label, beta, gamma, quad in SYNTH:
        rows, tp = [], []
        for k, c0 in enumerate(C_TRUE):
            t, u, d = synth_swing(24.0, 24.0, 4.97, beta, gamma, c0, quad=quad, seed=k)
            _, cc, aa, _ = triples(*turning_points(t, u, d)[:3])
            tp += [(a / CPD, c - c0) for a, c in zip(aa, cc)]
            for tend in np.arange(3.0, t[-1], 0.02):
                r = window_variants(t, u, d, tend)
                if r is not None:
                    r["truth"] = c0
                    rows.append(r)
        tp = np.asarray(tp)
        print(f"\n{label}: 3 swings from 24°, true centres {C_TRUE}.")
        print("turning-point estimates, error by amplitude: " + "; ".join(
            f"{lo}-{hi}° {tp[(tp[:, 0] >= lo) & (tp[:, 0] < hi), 1].mean():+.3f} ± "
            f"{tp[(tp[:, 0] >= lo) & (tp[:, 0] < hi), 1].std():.3f} (n {((tp[:, 0] >= lo) & (tp[:, 0] < hi)).sum()})"
            for lo, hi in ((15, 25), (10, 15), (5, 10), (2, 5)) if ((tp[:, 0] >= lo) & (tp[:, 0] < hi)).any()))
        print(f"3 s windows every 20 ms ({len(rows)}), error against the true centre (counts):\n")
        band_table(rows)
    return 0


def _release_trials(t, u, d, truth, starts, cp: CalibParams):
    """Both calibrations started at every release time: today's fit and the turning-point one. The reference is
    taken at the middle of what each one used (today: the last 3 s; turning points: the last 2.5 periods)."""
    now, tp = [], []
    for ts in starts:
        a = swing_calibrator(t, u, cp, ts)
        if a is not None:
            now.append((a[0] - truth(a[1] - 1.5), a[1] - ts, a[2] / CPD))
        b = tp_calibrator(t, u, d, TPParams(), ts)
        if b is not None:
            tp.append((b[0] - truth(b[1] - 1.6), b[1] - ts, b[2] / CPD))
    return np.asarray(now), np.asarray(tp)


def release_starts(t, u, step=0.1, amp_max_deg=24.0):
    """Release times every `step` s while the swing (half the range over the next period) is below amp_max."""
    out = []
    for x in np.arange(t[0], t[-1] - 3.0, step):
        m = (t >= x) & (t < x + 1.3)
        if m.sum() > 2 and 0.5 * np.ptp(u[m]) <= amp_max_deg * CPD:
            out.append(float(x))
    return out


def slow_centre(t, u, d, lo_deg=3.0, hi_deg=24.0):
    """The decay's centre as a slow function of time: a quadratic through all turning-point estimates between
    lo and hi degrees (one decay: 20-40 estimates). Used to judge both calibrations on the decays."""
    tc, cc, aa, _ = triples(*turning_points(t, u, d)[:3])
    k = (aa >= lo_deg * CPD) & (aa <= hi_deg * CPD)
    p = np.polyfit(tc[k], cc[k], 2)
    return lambda x: float(np.polyval(p, x)), float(np.std(cc[k] - np.polyval(p, tc[k]))), int(k.sum())


def _trial_line(label, a) -> str:
    if not len(a):
        return f"| {label} | 0 | – | – | – |"
    return (f"| {label} | {len(a)} | {a[:, 0].mean():+.3f} / {a[:, 0].std():.3f} / {np.abs(a[:, 0]).max():.2f} | "
            f"{np.median(a[:, 1]):.1f} | {a[:, 2].min():.0f}-{a[:, 2].max():.0f} |")


def cmd_fix(args) -> int:
    print("Sessions: what each calibration accepts on the logged start-up swing (counts; t from program start). "
          "Turn residual = median rms of the extrema's pair regressions (clean turns 0.02-0.04; a stall just after "
          "the turn, then a catch-up faster than a free swing, gives 0.2-0.45).\n")
    print("| session | today: c at t (amp °) | turning points: c at t (amp °) | today - TP | the 4 estimates | "
          "turn residual |")
    print("|---|---|---|---|---|---|")
    diffs = []
    for path in sessions(args.runs):
        s = load_session(path)
        r = replay_startup(s)
        if r is None:
            continue
        t, u, d, *_ = startup_swing(s)
        res = float(np.median(turning_points(t, u, d)[4]))
        b = tp_calibrator(t, u, d)
        if b is None:
            print(f"| {path.name[:19]} | {r['c']:+.3f} at {r['t_acc']:.1f} s ({r['amp'] / CPD:.0f}) | none | – | – | "
                  f"{res:.2f} |")
            continue
        diffs.append((r["c"] - b[0], b[1] - r["t_acc"]))
        print(f"| {path.name[:19]} | {r['c']:+.3f} at {r['t_acc']:.1f} s ({r['amp'] / CPD:.0f}) | {b[0]:+.3f} at "
              f"{b[1]:.1f} s ({b[2] / CPD:.0f}) | **{r['c'] - b[0]:+.2f}** | {', '.join(f'{x:+.2f}' for x in b[3])} | "
              f"{res:.2f} |")
    e = np.asarray(diffs)
    print(f"\ntoday - turning points over {len(e)} sessions: mean {e[:, 0].mean():+.3f}, sd {e[:, 0].std():.3f}, "
          f"max |d| {np.abs(e[:, 0]).max():.2f}; accepted {np.median(e[:, 1]):+.1f} s later than today (median; "
          f"{e[:, 1].min():+.1f} … {e[:, 1].max():+.1f})")
    same = np.abs(e[:, 1]) < 3.0                  # both accepted the same swing (the operator gave several in some)
    print(f"  the same swing ({same.sum()}): mean {e[same, 0].mean():+.3f}, sd {e[same, 0].std():.3f}, max |d| "
          f"{np.abs(e[same, 0]).max():.2f}; different swings ({(~same).sum()}): {', '.join(f'{x:+.2f}' for x in e[~same, 0])}")

    print("\nDecays and synthetic swings: both calibrations started at every release time (every 0.1 s while the "
          "swing is below 24°), error of the accepted c (decays: against a quadratic in time through all the decay's "
          "turning-point estimates at 3-24°; synthetic: against the true centre).\n")
    print("| swing | calibration | accepted | error mean / sd / max \\|e\\| | median time to accept s | amp ° |")
    print("|---|---|---|---|---|---|")
    cp = CalibParams()
    for name, t0, t1 in BENCH:
        t, u, d = load_encoder_log(name)
        m = (t >= t0) & (t <= t1)
        t, u, d = t[m], u[m], d[m]
        ref, scatter, n = slow_centre(t, u, d)
        now, tp = _release_trials(t, u, d, ref, release_starts(t, u), cp)
        print(_trial_line(f"{name[:19]} (estimates about the quadratic: sd {scatter:.2f}, n {n}) | today", now))
        print(_trial_line(f"{name[:19]} | turning points", tp))
    for label, beta, gamma, quad in SYNTH[:2]:
        now_all, tp_all = [], []
        for k, c0 in enumerate(C_TRUE):
            t, u, d = synth_swing(24.0, 24.0, 4.97, beta, gamma, c0, quad=quad, seed=k)
            now, tp = _release_trials(t, u, d, lambda x, c0=c0: c0, release_starts(t, u), cp)
            now_all += list(now)
            tp_all += list(tp)
        short = label.split(":")[0].split("(")[0].strip()
        print(_trial_line(f"synthetic, {short} | today", np.asarray(now_all)))
        print(_trial_line(f"synthetic, {short} | turning points", np.asarray(tp_all)))
    return 0


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("sessions", "fix"):
        p = sub.add_parser(name)
        p.add_argument("runs", nargs="*")
    sub.add_parser("bench")
    sub.add_parser("synth")
    args = ap.parse_args()
    return {"sessions": cmd_sessions, "bench": cmd_bench, "synth": cmd_synth, "fix": cmd_fix}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
