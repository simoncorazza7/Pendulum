"""Checks for py/offset (the upright-offset estimate; simulation only). PC only, no hardware, no logs needed.

    .venv/Scripts/python py/test_offset.py
Prints PASS/FAIL per check; exit code 1 on failure.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

import cascade_pi_study as cps
import offset_study as ost
from offset import identity as di
from offset import observer as ob

failures = 0
CNT = di.COUNT
W0, BJ, G = 4.97, 0.054, 9.81


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    failures += 0 if ok else 1


def linear_balance(delta: float, t_end: float, quantise: bool = True, push: float = 0.0, seed: int = 1):
    """A linear pendulum held by a PD on the measured angle (offset delta), the cart acceleration the PD's command
    plus a little noise. quantise: 2400-count encoder (phase 0.37), else the exact angle plus 0.3-count noise.
    push: a hand nudge at 10 s, rad/s^2 for 0.1 s. Returns arrays t, theta_m, omega (true rate), v."""
    rng = np.random.default_rng(seed)
    ts, h = 0.002, 0.0005
    th, om, v = 0.0, 0.0, 0.0
    T, TH, OM, V = [], [], [], []
    for k in range(int(t_end / ts)):
        t = k * ts
        if quantise:
            thm = (math.floor((th - delta) / CNT + 0.37) - 0.37 + 0.5) * CNT
        else:
            thm = th - delta + rng.normal(0.0, 0.3 * CNT)
        a = 60.0 * thm + 6.5 * om + rng.normal(0.0, 0.05)
        T.append(t)
        TH.append(thm)
        OM.append(om)
        V.append(v)
        tau = push if 10.0 <= t < 10.1 else 0.0
        for _ in range(4):
            acc = W0 ** 2 * th - (W0 ** 2 / G) * a - BJ * om + tau
            om += acc * h
            th += om * h
            v += a * h
    return np.array(T), np.array(TH), np.array(OM), np.array(V)


def estimate(t, thm, om, v, p) -> di.DriftIdentityEstimator:
    est = di.DriftIdentityEstimator(p)
    for k in range(len(t)):
        est.step(t[k], thm[k], om[k], v[k], True)
    return est


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    p = di.IdentityParams(w0=W0, bJ=BJ)

    # 1. observability of the offset from the angle alone, in both coordinate forms (they differ by det T = w0^2)
    Cm = np.array([1.0, 0.0, 0.0])
    Am = np.array([[0.0, 1.0, 0.0], [W0 ** 2, -BJ, 1.0], [0.0, 0.0, 0.0]])      # (theta_m, omega, d)
    At = np.array([[0.0, 1.0, 0.0], [W0 ** 2, -BJ, 0.0], [0.0, 0.0, 0.0]])      # (theta_true, omega, delta)
    Ct = np.array([1.0, 0.0, -1.0])                                            # y = theta_true - delta
    dm = np.linalg.det(np.vstack([Cm, Cm @ Am, Cm @ Am @ Am]))
    dt_ = np.linalg.det(np.vstack([Ct, Ct @ At, Ct @ At @ At]))
    check("offset observable from the angle alone (det 1 measured frame, w0^2 true frame)",
          abs(abs(dm) - 1) < 1e-9 and abs(abs(dt_) - W0 ** 2) < 1e-9, f"{dm:.4f}, {dt_:.4f}")

    # 2. the fixed-gain observer places its error poles; with learning off the offset is held (eigenvalue 1)
    po = ob.ObserverParams(w0=W0, bJ=BJ, f_v=6.0, tau_d=1.0)
    K = ob.gains(po)
    eL = np.sort(np.abs(np.linalg.eigvals(ob.error_matrix(po, K, True))))
    eF = np.sort(np.abs(np.linalg.eigvals(ob.error_matrix(po, K, False))))
    check("observer poles placed; offset held when not learning",
          np.allclose(eL, np.sort(np.abs(ob.observer_poles(po))), atol=1e-9) and abs(eF[-1] - 1) < 1e-12
          and eF[1] < 1, f"learn {eL.round(4)}, held {eF.round(4)}")

    # 3. a prior alone is the estimate; with no prior and no data the estimate is 0
    e = di.DriftIdentityEstimator(p, prior=1.3 * CNT, prior_weight=20.0)
    e0 = di.DriftIdentityEstimator(p)
    check("prior alone = the estimate; nothing = 0", abs(e.delta_hat() - 1.3 * CNT) < 1e-12 and e.weight() == 20.0
          and e0.delta_hat() == 0.0 and e0.weight() == 0.0)

    # 4. without quantisation the drift identity recovers a known offset within 0.1 count in 20 s
    for delta_c in (1.3, -2.0):
        est = estimate(*linear_balance(delta_c * CNT, 20.0, quantise=False), p)
        err = est.delta_hat() / CNT - delta_c
        check(f"exact angle, offset {delta_c:+.1f} counts: estimate within 0.1 count", abs(err) < 0.1
              and est.weight() > 15, f"{est.delta_hat() / CNT:+.2f} counts, weight {est.weight():.1f} s")

    # 5. with the 2400-count encoder the online estimate equals the batch line fit of the same residual (the cart
    #    really feels the quantisation bias, up to half a count: docs/stage1_drift.md), and stays in that band
    for delta_c in (1.3, -2.0):
        t, thm, om, v = linear_balance(delta_c * CNT, 20.0)
        est = estimate(t, thm, om, v, p)
        L = G / W0 ** 2
        m = (t >= p.t_settle) & (t <= t[-1] - p.lag)
        i0 = int(np.argmax(m))
        integ = np.concatenate(([0.0], np.cumsum(0.5 * (thm[i0 + 1:] + thm[i0:-1]) * 0.002)))
        r = (v[i0:] - v[i0]) - G * integ + L * (om[i0:] - om[i0]) + L * BJ * (thm[i0:] - thm[i0])
        tt = t[i0:] - t[i0]
        keep = tt <= tt[-1] - p.lag
        batch = np.polyfit(tt[keep], r[keep], 1)[0] / G / CNT
        check(f"quantised, offset {delta_c:+.1f}: online = batch fit (0.05 count), within the 0.5-count band",
              abs(est.delta_hat() / CNT - batch) < 0.05 and abs(batch - delta_c) < 0.55,
              f"online {est.delta_hat() / CNT:+.3f}, batch {batch:+.3f}")

    # 6. nothing is learnt before t_settle or beyond theta_max
    est = di.DriftIdentityEstimator(p)
    for k in range(int(1.4 / 0.002)):
        est.step(k * 0.002, 0.0, 0.0, 0.05 * k * 0.002, True)        # accelerating cart, before t_settle
    for k in range(1000):
        est.step(2.0 + k * 0.002, math.radians(5), 0.0, 0.0, True)    # beyond theta_max
    check("no learning before t_settle or beyond theta_max", est.weight() == 0.0)

    # 7. a hand nudge mid-balance (2 rad/s^2 for 0.1 s: an 80 mm/s residual step) starts a new stretch, and the
    #    estimate matches the same balance without the nudge
    clean = estimate(*linear_balance(1.0 * CNT, 20.0, quantise=False), p)
    nudged = estimate(*linear_balance(1.0 * CNT, 20.0, quantise=False, push=2.0), p)
    nudged.end_balance()
    check("a nudge splits the stretch; the estimate stays within 0.15 count of the clean run",
          nudged.n_stretches >= 2 and abs(nudged.delta_hat() - clean.delta_hat()) / CNT < 0.15,
          f"{nudged.n_stretches} stretches, {nudged.delta_hat() / CNT:+.2f} vs {clean.delta_hat() / CNT:+.2f} counts")

    # 8. carry-over: the pool survives end_balance(), and the next balance starts from it
    w_before, d_before = nudged.weight(), nudged.delta_hat()
    nudged.step(0.0, 0.0, 0.0, 0.0, True)                             # a new catch begins (t_engaged 0)
    check("estimate and weight carried into the next catch",
          nudged.delta_hat() == d_before and nudged.weight() == w_before and w_before > 15)

    # 9. the study's loop with the trim off is py/cascade_pi_study.run exactly (Stage 3b on the 11-02 plant)
    kx, kv, ki = cps.gains(0.5, 0.8, 0.2)
    c = cps.HANDOVERS[1]
    a = cps.run(*c[:4], kx, kv, ki, offset_counts=-1.3)
    tr, bl = ost.make_trim("off", 0.0, cps.W0, cps.cs.PLANT.bJ)
    b = ost.run_handover(*c[:4], kx, kv, ki, tr, -1.3, bl)
    check("offset_study trim 'off' = cascade_pi_study.run", a["ok"] == b["ok"] and a["t"] == b["t"]
          and a["x_end"] == b["x_end"], f"x_end {a['x_end'] * 1e3:.3f} / {b['x_end'] * 1e3:.3f} mm")

    # 10. in closed loop on the rig model the estimator finds the simulated offset (+1.3 counts) during a balance
    tr, bl = ost.make_trim("carry", 1.3 * CNT, cps.W0, cps.cs.PLANT.bJ)
    r = ost.run_handover(*c[:4], kx, kv, ki, tr, -1.3, bl)
    d1 = tr.est.delta_hat() / CNT
    check("closed loop: after a 30 s balance the carried estimate is near the simulated +1.3 counts",
          r["ok"] and abs(d1 - 1.3) < 0.4 and r["used"] == 0.0, f"estimate {d1:+.2f} counts; first catch untrimmed")

    # 11. ... and the next catch starts with it as its trim (apply at engage)
    r2 = ost.run_handover(*c[:4], kx, kv, ki, tr, -1.3, bl)
    check("the next catch starts trimmed by the carried estimate", abs(r2["used"] / CNT - d1) < 1e-12,
          f"trim {r2['used'] / CNT:+.2f} counts")

    print(f"\n{'all passed' if failures == 0 else f'{failures} FAILED'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
