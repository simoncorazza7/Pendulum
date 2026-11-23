"""Stage 0.9 actuator test report (pendctl --mode actuator, ./actuator.sh; docs/stage0_9_actuator.md).

    python py/actuator_report.py [RUN ...]

RUN defaults to the newest logs/*_actuator. Several runs (one block each) are merged into one ladder map.

Per rep (one RUN of the reference trajectory):
  touch     the switch-touch audit after the rep: switch found - expected, in usteps (one lost-sync slip = 32)
  axis      the step generator: the Axis speed against the integrated command, as the delay (ms) that fits best and
            the worst mismatch (m/s^2); a cap or a glitch in the Axis shows here, the motor does not
  pendulum  the hanging pendulum as an accelerometer: the linear hanging model driven by the Axis acceleration,
            fitted to the counts from 0.5 s before the rep to 2 s after it; input gain against w0^2/g and the
            residual (counts). A stall or repeated slips spoil the fit; one slip (~2.3 counts) may not
  28 Hz     RMS of the counts between 24 and 33 Hz during the rep (the drive-side mode of docs/stage1_demo.md)
Then a_rel(v) per speed: the highest rung reached with no failed or aborted rung below it, and the limits that
the 50-70 % rule of the brief gives. Plot: actuator.png in the last run directory.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from pendel.runs import latest_run, load_run  # noqa: E402
from stage1_report import pcfg, responses  # noqa: E402

G = 9.81
RUN_PHASE = 5                        # rt::ActPhase::Run
KC = 2 * math.pi / 2400              # rad per count


def read_rows(path: Path) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def run_windows(d) -> list[tuple[int, int]]:
    """[start, end) record indices of every RUN phase, in order (one per rep that ran)."""
    run = (d["phase"] == RUN_PHASE).astype(int)
    e = np.diff(np.concatenate(([0], run, [0])))
    return list(zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1)))


def axis_check(t, v, a, i0, i1) -> tuple[float, float]:
    """Delay (s) at which the shifted command best explains the Axis speed change, and the worst mismatch."""
    ts = float(np.median(np.diff(t[i0:i1])))
    dv = np.diff(v[i0:i1 + 1]) / ts                    # realised over [k, k+1]
    k = np.arange(i0, i0 + len(dv))
    best = (math.inf, 0.0, 0.0)
    for lag in np.arange(0.0, 3.01, 0.05):             # ticks
        n, f = int(lag), lag - int(lag)
        cmd = (1 - f) * a[np.clip(k - n, 0, None)] + f * a[np.clip(k - n - 1, 0, None)]
        err = dv - cmd
        rms = float(np.sqrt(np.mean(err ** 2)))
        if rms < best[0]:
            best = (rms, lag * ts, float(np.max(np.abs(err))))
    return best[1], best[2]


def pendulum_fit(t, u, a, w0, bJ, sgn, i0, i1) -> tuple[float, float]:
    """Input gain (fraction of w0^2/g) and residual RMS (counts) of the hanging model on [i0, i1):
    phi'' = -w0^2 phi - bJ phi' + k a - c sign(phi'), with the sign of phi' taken from the counts, so the Coulomb
    pivot friction (plant.coulomb) is one more known-shape input; unknowns offset, phi0, phi'0, k, c."""
    if i1 - i0 < 50:
        return math.nan, math.nan
    tt = t[i0:i1]
    uu = u[i0:i1]
    du = np.gradient(np.convolve(uu, np.ones(9) / 9, mode="same"), tt)      # counts/s, smoothed over 18 ms
    s = -np.sign(du) * sgn * (np.abs(du) > 3.0)                              # -sign(phi'), 0 near the turns
    R = responses(tt, a[i0:i1], w0, bJ) / KC
    C = responses(tt, s, w0, bJ)[:, 2] / KC
    X = np.column_stack([np.ones_like(tt), R, C])
    beta, *_ = np.linalg.lstsq(X, uu, rcond=None)
    res = uu - X @ beta
    return beta[3] * sgn / (w0 * w0 / G), float(res.std())


def band_rms(t, u, i0, i1, lo=24.0, hi=33.0) -> float:
    if i1 - i0 < 128:
        return math.nan
    y = u[i0:i1] - np.polyval(np.polyfit(t[i0:i1], u[i0:i1], 1), t[i0:i1])
    ts = float(np.median(np.diff(t[i0:i1])))
    Y = np.fft.rfft(y)
    f = np.fft.rfftfreq(len(y), ts)
    sel = (f >= lo) & (f <= hi)
    return float(np.sqrt(2 * np.sum(np.abs(Y[sel]) ** 2)) / len(y))


def analyse(path: Path) -> list[dict]:
    run = load_run(path)
    d = run.data
    reps = read_rows(path / "reps.csv")
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    v = d["v"].astype(float)
    acmd = d["a_cmd"].astype(float)
    u = d["counts"].astype(float) + 0.5
    w0, bJ = pcfg(run, "plant.w0"), pcfg(run, "plant.bJ")   # keys the mode never read: config_source.conf
    sgn = int(pcfg(run, "calib.theta_sign"))
    ts = pcfg(run, "loop.ts")
    a_real = np.gradient(v, t)
    wins = run_windows(d)
    out = []
    # reps.csv has one row per rep that ran (aborted ones included) in the order they ran, like the RUN windows;
    # a block that ended in RUN has a window without a row, and the ring log may have dropped the oldest windows.
    shift = max(0, len(reps) - len(wins))
    for j, row in enumerate(reps):
        w = j - shift
        rec = {"run": path.name, "v": float(row["v"]), "a": float(row["a"]), "rung": int(row["rung"]),
               "rep": int(row["rep"]), "repeat": row["repeat"] == "1", "verdict": row["verdict"],
               "touch": int(row["touch_err_usteps"]), "side": int(row["side"]), "peak_a": float(row["peak_a_cmd"]),
               "peak_v": float(row["peak_v"]), "track_mm": float(row["max_track_m"]) * 1e3,
               "swing0": float(row["swing_start_deg"]), "late_ms": float(row.get("worst_late_ms") or "nan"),
               "delay_ms": math.nan, "axis_err": math.nan,
               "gain": math.nan, "res": math.nan, "hz28": math.nan}
        if 0 <= w < len(wins):
            i0, i1 = wins[w]
            delay, worst = axis_check(t, v, acmd, i0, i1)
            rec["delay_ms"], rec["axis_err"] = delay * 1e3, worst
            j0 = max(0, i0 - int(0.5 / ts))
            j1 = min(len(t), i1 + int(2.0 / ts))
            rec["gain"], rec["res"] = pendulum_fit(t, u, a_real, w0, bJ, sgn, j0, j1)
            rec["hz28"] = band_rms(t, u, i0, i1)
        out.append(rec)
    print(f"{path.name}: {run.meta.get('result', '?')}; profile {run.meta.get('profile', '?')}, "
          f"Vref {run.meta.get('vref_v', '?')} V, span {run.meta.get('span_usteps', '?')} usteps, "
          f"{len(reps)} reps")
    return out


def a_rel(recs: list[dict]) -> dict[float, tuple[float, str]]:
    """Per speed: the highest acceleration whose rung passed with no failed or aborted rung below it."""
    out = {}
    for v in sorted({r["v"] for r in recs}):
        rungs: dict[float, list[str]] = {}
        for r in recs:
            if r["v"] == v:
                rungs.setdefault(r["a"], []).append(r["verdict"])
        best, why = 0.0, "no passing rung"
        for a in sorted(rungs):
            vs = rungs[a]
            bad = [x for x in vs if x in ("FAIL", "aborted")]
            if bad:
                why = f"{bad[0]} at {a:g} m/s^2"
                break
            if all(x in ("PASS", "marginal") for x in vs) and "PASS" in vs:
                best, why = a, "top of the ladder" if a == max(rungs) else ""
        out[v] = (best, why)
    return out


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    paths = [Path(p) for p in sys.argv[1:]] or [latest_run("logs", "actuator")]
    recs = []
    for p in paths:
        recs += analyse(p)
    if not recs:
        print("no reps")
        return
    print("\n v m/s  a m/s^2 rep  verdict    touch  peak a  peak v  track mm  delay ms  axis err  gain    res   28Hz")
    for r in recs:
        print(f" {r['v']:5.2f}  {r['a']:6.1f}  {r['rep']}{'r' if r['repeat'] else ' '}  {r['verdict']:9s} "
              f"{r['touch']:+5d}  {r['peak_a']:6.2f}  {r['peak_v']:6.3f}  {r['track_mm']:8.2f}  {r['delay_ms']:8.2f}  "
              f"{r['axis_err']:8.2f}  {r['gain']:6.0%}  {r['res']:5.2f}  {r['hz28']:5.2f}")
    rel = a_rel(recs)
    print("\nReliable acceleration per speed (confirm the top rung with 5 more reps and one-way legs, block B5):")
    for v, (a, why) in rel.items():
        print(f"  v {v:.2f} m/s: a_rel {a:g} m/s^2" + (f"  ({why})" if why else ""))
    speeds = sorted(rel)
    print("\nWhat the 50-70 % rule gives (CONTROL_BRIEF 0.9), for each speed tested taken as v_max:")
    for i, vm in enumerate(speeds):
        amin = min(rel[v][0] for v in speeds[:i + 1])
        if amin <= 0:
            continue
        print(f"  v_max {0.5 * vm:.2f}-{0.7 * vm:.2f} m/s (from v {vm:.2f}): balance.a_max {0.5 * amin:.1f}-{0.7 * amin:.1f} "
              f"m/s^2, a_brake <= {0.7 * rel[vm][0]:.1f} m/s^2")
    print("  then: move.max_accel / max_speed, the soft-limit reach check (params.cpp), py/gateb_design.py")

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    for k, v in enumerate(speeds):
        rr = [r for r in recs if r["v"] == v]
        col = f"C{k}"
        for r in rr:
            mk = {"PASS": "o", "marginal": "s", "FAIL": "x", "aborted": "^"}.get(r["verdict"], ".")
            ax[0].plot(r["a"], r["touch"], mk, color=col, ms=7, mfc="none" if mk in "os^" else col)
            ax[1].plot(r["a"], 100 * r["gain"], mk, color=col, ms=7, mfc="none" if mk in "os^" else col)
        ax[0].plot([], [], "o", color=col, label=f"v {v:.2f} m/s")
    ax[0].axhspan(-3, 3, color="0.9", zorder=0)
    ax[0].set_ylabel("touch error (usteps)")
    ax[0].legend(title="o pass, s marginal, x fail, ^ aborted")
    ax[1].axhline(100, color="k", lw=0.7)
    ax[1].set_ylabel("pendulum input gain (% of w0^2/g)")
    ax[1].set_xlabel("rung acceleration a (m/s^2)")
    for x in ax:
        x.grid(True)
    fig.savefig(paths[-1] / "actuator.png", dpi=110, bbox_inches="tight")
    print(f"\nplot: {paths[-1] / 'actuator.png'}")


if __name__ == "__main__":
    main()
