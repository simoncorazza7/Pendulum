"""Closed-loop study of the upright-offset estimate (simulation only, PC only; docs/offset_estimator.md).

    .venv/Scripts/python py/offset_study.py handovers [--offsets -1.3 -2.5] [--kinds ...]
    .venv/Scripts/python py/offset_study.py draws [--runs 300] [--seed 7] [--prior-sd 0.5] [--kinds ...]

handovers: the 29 rig swing-up hand-overs of py/cascade_pi_study.py, in their logged order as one session, 30 s each.
draws:     the rig.TEST draws of py/neuro/rig.py (offsets to +-2 counts, w0 +-3 %, friction ranges, delays 4-7
           ticks), each run a separate first catch; the baseline is the 81 % of docs/neuro_sim.md.

The balance controller is the rig's Stage 3b cascade (pendel.control.cascade_step's law and gains, cart loop with
its integral). The estimate enters only as a reference trim, theta_ref = -delta_used. Variants (--kinds):
  off           no trim (the rig today)
  online        offset.identity.DriftIdentityEstimator from nothing, applied within the catch it learns from
                once it has W_USE s of quiet balance (rate-limited to RATE counts/s)
  online-bl     the same, but each trim change is handed to the cart integral bumplessly (x_int -= g dtrim / ki),
                so the command does not jump and the integral stops holding what the trim now holds
  carry         [handovers] the estimator carried from one catch to the next; the trim is set at engage from what
                the earlier catches learnt and held for the catch (learning goes on, for the next one)
  carry-always  [handovers] carried, and also updated within the catch (as online)
  integral      [handovers] the quick version: a catch that lasted >= 10 s hands its settled cart-integral lean
                (ki x_int / g) to the next catch as its trim
  prior         [draws] the trim at engage from earlier balances: the true offset plus an error (--prior-sd
                counts, normal), held for the catch
  prior-always  [draws] the same prior (weight 20 s), also updated within the catch
  ideal         the true offset from the start, held (upper bound)
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

import cascade_pi_study as cps
from neuro import rig
from offset.identity import COUNT, DriftIdentityEstimator, IdentityParams
from pendel.control import PidState, pid_step
from pendel.estimators import BdLp2
from pendel.plant import pendulum_accel, realised_accel

W_USE = 3.0        # s of quiet balance before an estimate is used
RATE = 1.0         # counts/s, rate limit of trim changes within a catch
G = 9.81


class Trim:
    """delta_used for a sequence of catches. start() at engage, tick() every sample, end() when the catch ends.

    source: "none" | "estimator" | "integral" | "ideal";  apply: "engage" (set at engage, held) | "always"."""

    def __init__(self, source: str, apply: str = "engage", true_delta: float = 0.0,
                 est: DriftIdentityEstimator | None = None):
        self.source, self.apply, self.true_delta, self.est = source, apply, true_delta, est
        self.carry = 0.0                      # integral source: trim for the next catch
        self.used = 0.0

    def start(self) -> float:
        if self.source == "ideal":
            self.used = self.true_delta
        elif self.source == "integral":
            self.used = self.carry
        elif self.source == "estimator" and self.est.weight() >= W_USE:
            self.used = self.est.delta_hat()
        else:
            self.used = 0.0
        return self.used

    def tick(self, t: float, theta: float, omega: float, v: float, ts: float) -> float:
        if self.est is not None:
            self.est.step(t, theta, omega, v, True)
            if self.apply == "always" and self.est.weight() >= W_USE:
                lim = RATE * COUNT * ts
                self.used += max(-lim, min(lim, self.est.delta_hat() - self.used))
        return self.used

    def end(self, lasted: float, x_int: float, ki: float) -> None:
        if self.est is not None:
            self.est.end_balance()
        if self.source == "integral" and lasted >= 10.0:
            self.carry = self.used + ki * x_int / G


def make_trim(kind: str, true_delta: float, w0: float, bJ: float, prior: float = 0.0,
              prior_weight: float = 0.0) -> tuple[Trim, bool]:
    """(Trim, bumpless) for a variant name."""
    def est():
        return DriftIdentityEstimator(IdentityParams(w0=w0, bJ=bJ), prior, prior_weight)
    table = {
        "off": lambda: (Trim("none"), False),
        "online": lambda: (Trim("estimator", "always", true_delta, est()), False),
        "online-bl": lambda: (Trim("estimator", "always", true_delta, est()), True),
        "carry": lambda: (Trim("estimator", "engage", true_delta, est()), False),
        "carry-always": lambda: (Trim("estimator", "always", true_delta, est()), False),
        "integral": lambda: (Trim("integral", "engage", true_delta), False),
        "prior": lambda: (Trim("estimator", "engage", true_delta, est()), False),
        "prior-always": lambda: (Trim("estimator", "always", true_delta, est()), False),
        "ideal": lambda: (Trim("ideal", "engage", true_delta), False),
    }
    return table[kind]()


def run_handover(th0, thd0, x0, v0, kx, kv, ki, trim: Trim, offset_counts: float, bumpless: bool = False,
                 th_ref_max=math.radians(4), i_max=math.radians(2), t_end=30.0):
    """py/cascade_pi_study.run with the trim hook (identical when the trim stays 0). offset_counts as there: the
    controller sees theta + offset_counts * COUNT, so the true offset is delta = -offset_counts counts."""
    cs = cps.cs
    p = cs.pid_params(1.0)
    th, thd, x, v = math.radians(th0), math.radians(thd0), x0 * 1e-3, v0 * 1e-3
    est = BdLp2(cps.TS, 14.0)
    c_prev = int(np.floor(th / cps.COUNT + cs.SENS.phase))
    events, a_applied, h = [], 0.0, cps.TS / cps.H
    st = PidState(integral=thd / cps.W0 ** 2)
    xi = 0.0
    lim_xi = G * i_max / ki if ki > 0 else 0.0
    used = used_prev = trim.start()
    t = 0.0
    for k in range(int(t_end / cps.TS)):
        t = k * cps.TS
        counts, th_m = cs.measure(th, cs.SENS)
        th_m += offset_counts * cps.COUNT
        om = est.update(t, counts, events) if k > 0 else thd
        events = []
        used = trim.tick(t, th_m, om, v, cps.TS)
        if bumpless and ki > 0 and used != used_prev:
            xi = max(-lim_xi, min(lim_xi, xi - G * (used - used_prev) / ki))
        used_prev = used
        a_d = -kx * x - kv * v - ki * xi
        raw = a_d / G
        th_ref = max(-th_ref_max, min(th_ref_max, raw))
        if ki > 0 and raw == th_ref:
            xi += x * cps.TS
            xi = max(-lim_xi, min(lim_xi, xi))
        a, st, _ = pid_step(replace(p, theta_ref=th_ref - used), st, th_m, v, cps.TS, om)
        stop = v * v / (2 * cs.A_BRAKE) + abs(v) * 0.006 + 0.01
        if abs(x) >= cs.X_MAX or (v > 0 and x + stop >= cs.X_MAX) or (v < 0 and x - stop <= -cs.X_MAX):
            trim.end(t, xi, ki)
            return dict(ok=False, why="soft limit", t=t, x_end=x, used=used)
        if abs(th) > math.radians(22):
            trim.end(t, xi, ki)
            return dict(ok=False, why="fell", t=t, x_end=x, used=used)
        for j in range(cps.H):
            if j == 2:
                a_applied = a
            a_r = realised_accel(a_applied, v, cs.ACT)
            acc = pendulum_accel(th, thd, a_r, cs.PLANT)
            th_old = th
            th += thd * h + 0.5 * acc * h * h
            thd += acc * h
            x += v * h + 0.5 * a_r * h * h
            v += a_r * h
            u0, u1 = th_old / cps.COUNT + cs.SENS.phase, th / cps.COUNT + cs.SENS.phase
            c_new = int(np.floor(u1))
            for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                events.append((t + j * h + h * (b - u0) / (u1 - u0) if u1 != u0 else t,
                               b if c_new > c_prev else b - 1))
            c_prev = c_new
    trim.end(t_end, xi, ki)
    return dict(ok=True, why="30 s", t=t_end, x_end=x, used=used)


def cmd_handovers(args) -> None:
    kx, kv, ki = cps.gains(0.5, 0.8, 0.2)                       # the config's Stage 3b loop (kx .41 kv 1.0 ki .05)
    w0, bJ = cps.W0, cps.cs.PLANT.bJ
    print(f"Stage 3b cascade (kx {kx:.2f}, kv {kv:.2f}, ki {ki:.3f}); estimates used after {W_USE:.0f} s of quiet "
          f"balance, in-catch changes rate-limited to {RATE:g} counts/s. The 29 hand-overs of "
          f"py/cascade_pi_study.py in their logged order as one session, 30 s each.\n")
    print("| true offset (counts) | trim | kept 30 s | lost at (s) | trim at the end, median (counts) |")
    print("|---|---|---|---|---|")
    for off in args.offsets:
        delta = -off * COUNT
        for kind in args.kinds:
            session = kind in ("carry", "carry-always", "integral")
            tr, bl = make_trim(kind, delta, w0, bJ)
            ok, lost, used = 0, [], []
            for th0, thd0, x0, v0, _rig, _lab in cps.HANDOVERS:
                if not session:
                    tr, bl = make_trim(kind, delta, w0, bJ)
                r = run_handover(th0, thd0, x0, v0, kx, kv, ki, tr, off, bl)
                used.append(r["used"] / COUNT)
                if r["ok"]:
                    ok += 1
                else:
                    lost.append(f"{r['t']:.1f}")
            print(f"| {-off:+.1f} | {kind} | {ok}/{len(cps.HANDOVERS)} | {', '.join(lost) or '-'} | "
                  f"{np.median(used):+.2f} |", flush=True)


class OffsetPolicy(rig.PidPolicy):
    """rig.PidPolicy (Stage 3b cascade_step) plus a trim; one Trim per run."""

    def __init__(self, kind: str, ics: dict, prior_sd: float = 0.0, seed: int = 0, **kw):
        super().__init__(**kw)
        self.kind, self.ics, self.prior_sd = kind, ics, prior_sd
        self.rng = np.random.default_rng(seed)

    def reset(self, K: int) -> None:
        super().reset(K)
        self.k = 0
        self.trims, self.bl, self.prev = [], [], []
        prior = self.kind in ("prior", "prior-always")
        for i in range(K):
            delta = float(self.ics["ref"][i])                      # rig.measure: theta_m = theta_true - ref
            pr = delta + self.rng.normal(0.0, self.prior_sd * COUNT) if prior else 0.0
            tr, bl = make_trim(self.kind, delta, 4.97, 0.054, pr, 20.0 if prior else 0.0)
            self.prev.append(tr.start())
            self.trims.append(tr)
            self.bl.append(bl)
        self.lim_xi = G * self.p.cart_i_max / self.p.cart_ki if self.p.cart_ki > 0 else 0.0

    def act(self, x, thm, v, om, dt):
        from pendel.control import cascade_step
        out = np.zeros((1, len(self.s)))
        t = self.k * dt
        for i in range(len(self.s)):
            used = self.trims[i].tick(t, float(thm[0, i]), float(om[0, i]), float(v[0, i]), dt)
            if self.bl[i] and used != self.prev[i] and self.p.cart_ki > 0:
                xi = self.s[i].x_int - G * (used - self.prev[i]) / self.p.cart_ki
                self.s[i] = replace(self.s[i], x_int=max(-self.lim_xi, min(self.lim_xi, xi)))
            self.prev[i] = used
            q = replace(self.p, theta_ref=self.p.theta_ref - used) if used else self.p
            out[0, i], self.s[i], _ = cascade_step(q, self.s[i], float(thm[0, i]), float(x[0, i]),
                                                   float(v[0, i]), dt, float(om[0, i]))
        self.k += 1
        return out


def cmd_draws(args) -> None:
    spec = rig.RigSpec(n_in=4, net_every=1)
    ics = rig.draw(np.random.default_rng(args.seed), args.runs, rig.TEST, spec)
    print(f"rig.TEST draws (seed {args.seed}, {args.runs} runs of {args.t:.0f} s): offsets to +-2 counts, w0 +-3 %, "
          f"b/J 0.03-0.08, Coulomb 0.10-0.25, delays 4-7 ticks. Estimates used after {W_USE:.0f} s, in-catch "
          f"changes rate-limited to {RATE:g} counts/s; prior error sd {args.prior_sd} counts.\n")
    print("| trim | survived 30 s | median end abs x (survivors) | lost, abs offset > 1 count | lost, abs offset <= 1 |")
    print("|---|---|---|---|---|")
    big = np.abs(np.asarray(ics["ref"])) > COUNT
    for kind in args.kinds:
        pol = OffsetPolicy(kind, ics, args.prior_sd, args.seed) if kind != "off" else rig.PidPolicy()
        r = rig.run(pol, ics, spec, t_max=args.t)
        alive = r["alive"][0]
        endx = np.median(np.abs(r["end_x"][0][alive])) * 1e3 if alive.any() else float("nan")
        print(f"| {kind} | {100 * alive.mean():.0f} % ({alive.sum()}/{len(alive)}) | {endx:.0f} mm | "
              f"{np.sum(~alive & big)} of {big.sum()} | {np.sum(~alive & ~big)} of {np.sum(~big)} |", flush=True)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("handovers")
    h.add_argument("--offsets", type=float, nargs="+", default=[-1.3, -2.5],
                   help="cascade_pi_study convention: the controller sees theta + offset counts")
    h.add_argument("--kinds", nargs="+", default=["off", "online", "carry", "carry-always", "integral", "ideal"])
    d = sub.add_parser("draws")
    d.add_argument("--runs", type=int, default=300)
    d.add_argument("--seed", type=int, default=7)
    d.add_argument("--t", type=float, default=30.0)
    d.add_argument("--prior-sd", type=float, default=0.5)
    d.add_argument("--kinds", nargs="+", default=["off", "online", "online-bl", "prior", "prior-always", "ideal"])
    args = ap.parse_args()
    {"handovers": cmd_handovers, "draws": cmd_draws}[args.cmd](args)


if __name__ == "__main__":
    main()
