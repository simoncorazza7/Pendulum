"""Reports for the Stage 1 hardware steps and the 0.10 pulse test.

    python py/stage1_report.py shadow [RUN]   sign/magnitude check of the would-be command (shadow mode)
    python py/stage1_report.py damp [RUN]     decay rate of the actively damped hanging swing (damp mode)
    python py/stage1_report.py pulse [RUN]    input gain w0^2/g and combined sign from a move-mode pulse
    python py/stage1_report.py balance [RUN]  every run of a balance session: duration, end, RMS theta,
                                              saturation, drift fit x0 + v0 t + a_d t^2/2 -> trim correction

RUN defaults to the newest run of that mode under logs/. Plots are saved into the run directory.
Conventions: CLAUDE.md (theta from upright, + toward +x; phi = theta - pi near hanging).
Model near hanging (docs/stage1_damp.md): phi'' = -w0^2 phi - (b/J) phi' + (w0^2/g) a.
"""

from __future__ import annotations

import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm
from scipy.optimize import curve_fit

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from pendel.runs import latest_run, load_run, parse_conf  # noqa: E402

G = 9.81
DEG = 180 / math.pi
MODES = {0: "IDLE", 1: "ARMED", 2: "ENGAGED", 3: "STOPPING"}


def pcfg(run, key: str) -> float:
    """Config value used by the run; keys the mode never read come from its config_source.conf copy."""
    if key in run.config:
        return float(run.config[key])
    src = parse_conf((run.path / "config_source.conf").read_text(encoding="utf-8"))
    return float(src[key])


def segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index ranges where mask is True."""
    d = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


# ---------------------------------------------------------------------------------------------- shadow
def shadow(run) -> None:
    d = run.data
    live = d["calibrating"] == 0
    t = (d["t_ns"] - d["t_ns"][live][0]) * 1e-9
    kp, ki, kd = (float(run.meta[k]) for k in ("kp", "ki", "kd"))
    print(f"{run.path.name}: {run.meta['result']}; c_bottom {run.meta['c_bottom_fit_counts']} counts "
          f"(se {run.meta['c_bottom_se_counts']}); Kp {kp:.2f} Ki {ki:.2f} Kd {kd:.3f}; "
          f"motor commands ignored {run.meta['motor_commands_ignored']}")
    eng = live & (d["mode"] == 2)
    if not eng.any():
        print("never ENGAGED")
    for i0, i1 in segments(eng):
        th, a = d["theta"][i0:i1], d["a_cmd"][i0:i1]
        nxt = d["event"][i1] if i1 < len(d) else -1
        print(f"  ENGAGED {t[i0]:7.2f}-{t[i1 - 1]:7.2f} s: theta {th.min() * DEG:+6.2f}..{th.max() * DEG:+6.2f} deg, "
              f"a_cmd {a.min():+6.2f}..{a.max():+6.2f} m/s^2, ended by event {nxt}")
        for sgn, name in ((1, "+x (top toward the LEFT switch)"), (-1, "-x (top toward the RIGHT switch)")):
            m = sgn * th > math.radians(1.5)
            if m.any():
                frac = np.mean(np.sign(a[m]) == sgn)
                print(f"      tilt {name}: {m.sum() / 500:.2f} s, a_cmd has the same sign in {frac:.0%} "
                      f"(must be ~100 %: the cart moves under the pendulum)")
    # the controller chain itself: unsaturated ENGAGED samples must satisfy a = Kp th + Kd w + Ki I exactly
    m = eng & (d["saturated"] == 0)
    if m.sum() > 10:
        pred = kp * d["theta"][m] + kd * d["omega"][m] + ki * d["integral"][m]
        print(f"  chain check: max |a_cmd - (Kp th + Kd w + Ki I)| = {np.abs(d['a_cmd'][m] - pred).max():.2e} m/s^2 "
              f"over {m.sum()} unsaturated samples")
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    ax[0].plot(t[live], d["theta"][live] * DEG, lw=0.7)
    ax[0].set_ylabel("theta (deg from upright)")
    ax[0].set_ylim(-30, 30)
    ax[1].plot(t[live], d["a_cmd"][live], lw=0.7, color="C3")
    ax[1].set_ylabel("would-be a_cmd (m/s^2)")
    ax[1].set_xlabel("t (s)")
    for i0, i1 in segments(eng):
        for x in ax:
            x.axvspan(t[i0], t[i1 - 1], color="C2", alpha=0.15)
    for x in ax:
        x.grid(True)
    fig.savefig(run.path / "shadow.png", dpi=110, bbox_inches="tight")
    print(f"  plot: {run.path / 'shadow.png'}")


# ---------------------------------------------------------------------------------------------- damp
def peak_decay(t: np.ndarray, y: np.ndarray, floor: float) -> tuple[float, np.ndarray, np.ndarray]:
    """Decay rate (1/s) from the extrema of |y| per half period while above floor."""
    s = np.sign(y)
    edges = np.flatnonzero(np.diff(s) != 0)
    pt, pa = [], []
    for i0, i1 in zip(edges[:-1], edges[1:]):
        k = i0 + 1 + np.argmax(np.abs(y[i0 + 1:i1 + 1]))
        if abs(y[k]) > floor:
            pt.append(t[k])
            pa.append(abs(y[k]))
    pt, pa = np.array(pt), np.array(pa)
    if len(pt) < 3:
        return float("nan"), pt, pa
    slope = np.polyfit(pt, np.log(pa), 1)[0]
    return -slope, pt, pa


def decay_fit(t: np.ndarray, y: np.ndarray, sigma0: float, w0: float) -> tuple[float, float, np.ndarray]:
    """Least-squares fit y = c + e^{-sigma t}(a cos wt + b sin wt) over all samples.
    Returns (sigma, its standard error, fitted curve)."""
    t = t - t[0]
    f = lambda tt, c, a, b, sg, w: c + np.exp(-sg * tt) * (a * np.cos(w * tt) + b * np.sin(w * tt))  # noqa: E731
    p0 = [0.0, y[0], (y[min(5, len(y) - 1)] - y[0]) / (w0 * (t[min(5, len(t) - 1)] + 1e-9)), sigma0, w0]
    p, cov = curve_fit(f, t, y, p0=p0, maxfev=20000)
    return p[3], math.sqrt(cov[3, 3]), f(t, *p)


def damp(run) -> None:
    d = run.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    w0, bJ = run.cfg("plant.w0"), run.cfg("plant.bJ")
    gain, pred = float(run.meta["gain"]), float(run.meta["predicted_decay_rate"])
    print(f"{run.path.name}: {run.meta['result']}; c_bottom {run.meta['c_bottom_fit_counts']} counts; "
          f"gain {gain:.3f} (m/s^2)/(rad/s); engaged at {run.meta['engaged_at_s']} s")
    eng = d["phase"] == 1
    if not eng.any():
        print("never ENGAGED")
        return
    i0, i1 = segments(eng)[0]
    te, phi = t[i0:i1], d["phi"][i0:i1]
    floor = 3 * 2 * math.pi / 2400                                      # 3 counts
    sigma_pk, pt, pa = peak_decay(te, phi, floor)
    sigma, sigma_se, phi_fit = decay_fit(te, phi, pred, w0)
    k_in = (2 * sigma - bJ) / gain                                      # 2 sigma = b/J + k gain
    print(f"  decay rate {sigma:.3f} +- {sigma_se:.3f} 1/s (damped-sine fit; peaks: {sigma_pk:.3f} from {len(pt)}); "
          f"predicted zeta_h w0 = {pred:.3f} 1/s (open loop {bJ / 2:.3f})")
    print(f"  implied input gain k = (2 sigma - b/J)/gain = {k_in:.3f} 1/m vs w0^2/g = {w0 * w0 / G:.3f} "
          f"({k_in / (w0 * w0 / G):.0%}); a negative value would mean a sign error")
    print(f"  cart: max |x| {float(run.meta['max_abs_x_m']) * 1e3:.1f} mm, end x {d['x'][-1] * 1e3:+.1f} mm, "
          f"net {run.meta['net_usteps']} steps; saturated {d['saturated'][eng].mean():.0%} of engaged samples")
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 7))
    live = d["phase"] >= 0
    ax[0].plot(t[live], d["phi"][live] * DEG, lw=0.7)
    ax[0].plot(te, phi_fit * DEG, "k--", lw=0.8, label=f"fit: decay {sigma:.2f} 1/s (design {pred:.2f})")
    ax[0].legend()
    ax[0].set_ylabel("phi (deg from hanging)")
    ax[1].plot(t[live], d["x"][live] * 1e3, lw=0.7, color="C1")
    ax[1].set_ylabel("x (mm)")
    ax[2].plot(t[live], d["a_cmd"][live], lw=0.7, color="C3")
    ax[2].set_ylabel("a_cmd (m/s^2)")
    ax[2].set_xlabel("t (s)")
    for x in ax:
        x.axvspan(te[0], te[-1], color="C2", alpha=0.12)
        x.grid(True)
    fig.savefig(run.path / "damp.png", dpi=110, bbox_inches="tight")
    print(f"  plot: {run.path / 'damp.png'}")


# ---------------------------------------------------------------------------------------------- pulse
def responses(t: np.ndarray, a: np.ndarray, w0: float, bJ: float) -> np.ndarray:
    """Columns: free response to phi0 = 1, to phi'0 = 1, and the forced response to a(t) with input
    gain 1 (ZOH per sample), for phi'' = -w0^2 phi - bJ phi' + a."""
    M = np.zeros((3, 3))
    M[:2, :2] = [[0.0, 1.0], [-w0 * w0, -bJ]]
    M[1, 2] = 1.0
    E = expm(M * float(np.median(np.diff(t))))   # fixed loop period (jitter is microseconds)
    Ad, Bd = E[:2, :2], E[:2, 2]
    n = len(t)
    out = np.zeros((n, 3))
    for col, x0 in ((0, [1.0, 0.0]), (1, [0.0, 1.0]), (2, [0.0, 0.0])):
        x = np.array(x0)
        for i in range(n):
            out[i, col] = x[0]
            x = Ad @ x + (Bd * a[i] if col == 2 else 0.0)
    return out


def pulse(run) -> None:
    d = run.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    w0, bJ = pcfg(run, "plant.w0"), pcfg(run, "plant.bJ")
    sgn = int(pcfg(run, "calib.theta_sign"))
    kc = 2 * math.pi / 2400
    u = d["counts"] + 0.5
    a_real = np.gradient(d["v"], t)
    print(f"{run.path.name}: {run.meta['result']}; net {run.meta['net_usteps']} steps")
    expected = w0 * w0 / G
    for name, a in (("commanded a", d["a_cmd"]), ("realised a (from Axis speed)", a_real)):
        R = responses(t, a, w0, bJ) / kc
        X = np.column_stack([np.ones_like(t), R])
        beta, *_ = np.linalg.lstsq(X, u, rcond=None)
        res = u - X @ beta
        k = beta[3] * sgn                       # u = c + sgn*phi/kc  ->  coefficient of y is sgn*k/kc
        print(f"  {name}: input gain k = {k:+.3f} 1/m (expected +{expected:.3f}: {k / expected:.0%}), "
              f"c = {beta[0]:.2f} counts, residual rms {res.std():.2f} counts")
        if name.startswith("commanded"):
            fit = X @ beta
    if k < 0:
        print("  NEGATIVE input gain: the combined sign theta_sign * x_sign is wrong. Do not balance.")
    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    ax[0].plot(t, d["counts"], lw=0.7, label="counts")
    ax[0].plot(t, fit - 0.5, "k--", lw=0.8, label="model (commanded a)")
    ax[0].legend()
    ax[0].set_ylabel("encoder counts")
    ax[1].plot(t, d["a_cmd"], lw=0.7, color="C3", label="a_cmd")
    ax[1].plot(t, d["x"], lw=0.7, color="C1", label="x (m)")
    ax[1].legend()
    ax[1].set_xlabel("t (s)")
    for x in ax:
        x.grid(True)
    fig.savefig(run.path / "pulse.png", dpi=110, bbox_inches="tight")
    print(f"  plot: {run.path / 'pulse.png'}")


# --------------------------------------------------------------------------------------------- balance
def balance(run) -> None:
    """Brief §6 metrics per run (ENGAGED segment). Drift: x(t) = x0 + v0 t + a_d t^2/2 fitted from 1 s after
    engage; a reference offset delta gives a_d = g delta (PID), and theta_meas = theta_true - delta with
    delta = theta_sign (c_runtime - c_true) q, so encoder.trim should change by -theta_sign a_d / (g q)."""
    d = run.data
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    m = run.meta
    cpr, ts = pcfg(run, "encoder.counts_per_rev"), pcfg(run, "loop.ts")
    th_sign = pcfg(run, "calib.theta_sign")
    q = 2 * math.pi / cpr
    print(f"{run.path.name}: {m['result']}; {m['attempts']} runs")
    print(f"  c_bottom fit {m['c_bottom_fit_counts']} (se {m['c_bottom_se_counts']}), used {m['c_bottom_used_counts']} counts "
          f"(trim {pcfg(run, 'encoder.trim'):+.2f}); Kp {float(m['kp']):.2f} Ki {float(m['ki']):.2f} "
          f"Kd {float(m['kd']):.3f}; a_max {m['a_max']}, v_max {m['v_max']}")
    span = int(m["span_usteps"])
    print(f"  homing: switches at {m['pos_right_usteps']} / {m['pos_left_usteps']} usteps, span {span} "
          f"({span / pcfg(run, 'axis.usteps_per_m'):.3f} m); re-centre touch errors (usteps): "
          f"{m['touch_errors_usteps'] or 'none'}")
    eng = d["mode"] == 2
    segs = [(i0, i1) for i0, i1 in segments(eng) if i1 - i0 > 10]
    if not segs:
        print("  never ENGAGED")
        return
    print()
    print("| run | balanced (s) | ended by | RMS theta after 1 s (deg) | saturated | peak abs a (m/s²) | "
          "peak abs v (m/s) | v0 (mm/s) | a_d (mm/s²) | trim change (counts) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    for i0, i1 in segs:
        n = int(d["attempt"][i0])
        tt = t[i0:i1] - t[i0]
        th, x, a = d["theta"][i0:i1], d["x"][i0:i1], d["a_cmd"][i0:i1]
        end = MODES_END.get(int(d["event"][i1]) if i1 < len(d) else -1, "?")
        late = tt > 1.0
        rms = np.sqrt(np.mean(th[late] ** 2)) * DEG if late.any() else float("nan")
        v0 = a_d = dtrim = float("nan")
        if late.sum() > 250:                                   # >= 0.5 s of balance after the first second
            c = np.polyfit(tt[late], x[late], 2)
            a_d, v0 = 2 * c[0], c[1]
            dtrim = -th_sign * a_d / (G * q)
            ax[1].plot(tt[late], np.polyval(c, tt[late]) * 1e3, "k--", lw=0.8)
        print(f"| {n} | {tt[-1]:.2f} | {end} | {rms:.3f} | {np.mean(d['saturated'][i0:i1]):.0%} | "
              f"{np.abs(a).max():.2f} | {np.abs(d['v'][i0:i1]).max():.3f} | {v0 * 1e3:+.0f} | {a_d * 1e3:+.1f} | "
              f"{dtrim:+.2f} |")
        ax[0].plot(tt, th * DEG, lw=0.7, label=f"run {n}")
        ax[1].plot(tt, x * 1e3, lw=0.8)
        ax[2].plot(tt, a, lw=0.5)
    ax[0].set_ylabel("theta (deg from upright)")
    ax[0].legend(fontsize=7, ncol=6)
    ax[1].set_ylabel("cart x (mm from centre)")
    ax[1].axhline(pcfg(run, "sup.x_max") * 1e3, color="r", lw=0.6)
    ax[1].axhline(pcfg(run, "sup.x_min") * 1e3, color="r", lw=0.6)
    ax[2].set_ylabel("a_cmd (m/s²)")
    ax[2].set_xlabel("time since ENGAGED (s); dashed: drift fit from 1 s")
    for a_ in ax:
        a_.grid(True, lw=0.3)
    out = run.path / "balance_runs.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print()
    print(f"plot: {out}")
    print("trim: a_d > 0 = drift toward +x (LEFT switch). Average the trim changes of the longer runs before "
          "applying one (encoder.trim); expect a few mm/s² to remain (quantisation, docs/stage1_drift.md).")


MODES_END = {6: "theta abort", 7: "soft limit", 1: "manual stop", 2: "limit switch", 3: "encoder fault",
             4: "axis left ACCELERATE", 5: "overruns", 8: "max run time"}


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    if len(sys.argv) < 2 or sys.argv[1] not in ("shadow", "damp", "pulse", "balance"):
        print(__doc__)
        sys.exit(2)
    what = sys.argv[1]
    path = sys.argv[2] if len(sys.argv) > 2 else latest_run("logs", "move" if what == "pulse" else what)
    {"shadow": shadow, "damp": damp, "pulse": pulse, "balance": balance}[what](load_run(path))


if __name__ == "__main__":
    main()
