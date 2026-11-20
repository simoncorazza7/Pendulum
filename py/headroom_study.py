"""Headroom: which starts that the rig's controller loses could ANY controller catch? (PC only, simulation)

    .venv/Scripts/python py/headroom_study.py [--runs 300] [--seed 7] [--only N] [--survivors 40]

Same start states and plants as `py/neuro_check.py rig` (rig.TEST, seed 7). The PID + cart loop loses about 19 % of them,
almost all within 2 s, during the catch (and some more drift away later). For every start it loses:
  1. Feasibility. Linear programs for 4 s of commanded acceleration on a 40 ms grid, from the state the command delay
     leaves (no command during the first delay ticks, as at engage), under the rig's limits: |a| <= a_max, pid_step's
     v_max taper, |theta| bounded, and the supervisor's soft limit with braking distance (convex: tangent lines); each
     must end at rest (|v| <= 2 cm/s, |theta| <= 1 deg, |theta'| <= 0.1 rad/s).
       'catchable at all': no margin, 22 deg, Coulomb friction chosen in the controller's favour. This is a relaxation
       (the linear model is also optimistic for a catch), so 'no' means no acceleration profile catches the start.
       'catchable with 1 cm slack': the rig's 1 cm margin plus 1 cm, within 18 deg. A plan with no slack can touch the
       braking boundary exactly, and friction or the nonlinearity then tips it over.
  2. An oracle MPC. The same constraints and an L1 cost toward rest, re-solved every 40 ms over 2 s from the true
     state (true plant, no quantisation, the command delay predicted exactly), run for 10 s in the nonlinear rig model
     with friction. What it holds for 10 s is catchable for certain. It knows the true state and plant, so it is an
     upper bound for a real MPC on the PC.
"""

from __future__ import annotations

import argparse
import collections
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np
from scipy.linalg import expm
from scipy.optimize import linprog

from neuro import rig
from neuro.net import Batch, Genome

G = 9.81
SPEC = rig.RigSpec(n_in=4)
V_TANGENTS = np.linspace(0.0, 0.45, 10)          # m/s, tangent points of the braking distance v^2 / (2 a_brake)
NET = pathlib.Path(__file__).resolve().parent / "neuro" / "nets" / "rig4_budget_seeded.json"   # py/neuro/nets/README.md


def discretize(w0: float, bJ: float, h: float, B=None):
    """Zero-order-hold model of the linearised rig (state theta, theta', x, v; input a, or the input vector B)."""
    A = np.array([[0, 1, 0, 0], [w0 ** 2, -bJ, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]], float)
    B = np.array([0, -w0 ** 2 / G, 0, 1], float) if B is None else np.asarray(B, float)
    M = np.zeros((5, 5))
    M[:4, :4], M[:4, 4] = A, B
    E = expm(M * h)
    return E[:4, :4], E[:4, 4]


def prediction(Ad, Bd, N: int):
    """s_k = S0[k] s0 + Su[k] a for k = 0..N."""
    P = [np.eye(4)]
    for _ in range(N):
        P.append(Ad @ P[-1])
    S0 = np.array(P)
    Su = np.zeros((N + 1, 4, N))
    for k in range(1, N + 1):
        for j in range(k):
            Su[k, :, j] = P[k - 1 - j] @ Bd
    return S0, Su


class Planner:
    """The linear program over N steps of h: fixed matrices per plant, only the right-hand side follows the state.

    fric_help > 0 adds a free disturbance f_k in [-c, c] on theta'' that the program may choose: a superset of what
    Coulomb friction can do, so with it (and margin 0) the program is a relaxation and 'infeasible' is a hard verdict.
    """

    def __init__(self, w0: float, bJ: float, h: float, N: int, margin: float, theta_lim: float, terminal: bool,
                 cost: bool, fric_help: float = 0.0):
        self.N, self.cost = N, cost
        Ad, Bd = discretize(w0, bJ, h)
        S0, Su = prediction(Ad, Bd, N)
        nf = N if fric_help > 0 else 0
        if nf:
            _, Bf = discretize(w0, bJ, h, B=[0, -1.0, 0, 0])
            _, Sf = prediction(Ad, Bf, N)
        self.S0 = S0
        s = SPEC
        w = s.v_taper * s.v_max
        n_in = N + nf                                   # inputs: a (N), then f (nf)
        rows, rhs = [], []        # rhs: (constant, coefficient row on s0) so b = const - coef @ s0

        def U(k, idx):
            return np.concatenate([Su[k, idx], Sf[k, idx]]) if nf else Su[k, idx]

        def add(Arow, const, coef):
            rows.append(Arow)
            rhs.append((const, coef))

        for k in range(1, N + 1):                       # path constraints on the states after each step
            th_a, th_s = U(k, 0), S0[k, 0]
            x_a, x_s = U(k, 2), S0[k, 2]
            v_a, v_s = U(k, 3), S0[k, 3]
            add(th_a, theta_lim, th_s)
            add(-th_a, theta_lim, -th_s)
            for vj in V_TANGENTS:                       # x + v^2/(2 ab) + t_react v + margin <= x_max, both sides
                kv = vj / s.a_brake + s.t_react
                c0 = s.x_max - margin + vj * vj / (2 * s.a_brake)
                add(x_a + kv * v_a, c0, x_s + kv * v_s)
                add(-x_a - kv * v_a, c0, -x_s - kv * v_s)
        for k in range(N):                              # v_max taper on each command, at the speed it is given
            e = np.zeros(n_in)
            e[k] = 1.0
            v_a, v_s = U(k, 3), S0[k, 3]
            add(e + s.a_max / w * v_a, s.a_max * s.v_max / w, s.a_max / w * v_s)
            add(-e - s.a_max / w * v_a, s.a_max * s.v_max / w, -s.a_max / w * v_s)
        if terminal:                                    # end at rest
            for idx, lim in ((3, 0.02), (0, math.radians(1)), (1, 0.1)):
                add(U(N, idx), lim, S0[N, idx])
                add(-U(N, idx), lim, -S0[N, idx])
        A = np.array(rows)
        self.b_const = np.array([r[0] for r in rhs])
        self.b_coef = np.array([r[1] for r in rhs])
        nA = A.shape[0]
        bounds = [(-s.a_max, s.a_max)] * N + [(-fric_help, fric_help)] * nf
        if cost:                                        # L1 toward rest: aux t >= |theta|, |x|, |v| per step
            aux_rows = []
            aux_const = []
            aux_coef = []
            for i, idx in enumerate((0, 2, 3)):
                for k in range(1, N + 1):
                    for sign in (1.0, -1.0):
                        r = np.zeros(n_in + 3 * N)
                        r[:n_in] = sign * U(k, idx)
                        r[n_in + N * i + k - 1] = -1.0
                        aux_rows.append(r)
                        aux_const.append(0.0)
                        aux_coef.append(sign * S0[k, idx])
            A = np.hstack([A, np.zeros((nA, 3 * N))])
            A = np.vstack([A, np.array(aux_rows)])
            self.b_const = np.concatenate([self.b_const, aux_const])
            self.b_coef = np.vstack([self.b_coef, np.array(aux_coef)])
            wts = np.concatenate([np.full(N, 1 / math.radians(5)), np.full(N, 1 / 0.05), np.full(N, 1 / 0.1)])
            self.c = np.concatenate([np.zeros(n_in), wts / N])
            bounds += [(0, None)] * (3 * N)
        else:
            self.c = np.zeros(n_in)
        self.bounds = bounds
        self.A = A

    def solve(self, s0):
        b = self.b_const - self.b_coef @ np.asarray(s0, float)
        r = linprog(self.c, A_ub=self.A, b_ub=b, bounds=self.bounds, method="highs")
        return r.x[:self.N] if r.status == 0 else None


def step_tick(st, a_app, p):
    """One 2 ms tick of the nonlinear rig model (rig.run's integration, 4 substeps)."""
    th, thd, x, v = st
    h = SPEC.ts / 4
    for _ in range(4):
        a_r = float(rig.axis_cap(a_app, v, SPEC))
        acc = float(rig.pend_accel(th, thd, a_r, p["w0"], p["bJ"], p["coulomb"]))
        th, thd = th + thd * h + 0.5 * acc * h * h, thd + acc * h
        x, v = x + v * h + 0.5 * a_r * h * h, v + a_r * h
    return th, thd, x, v


def failed(st) -> bool:
    th, _, x, v = st
    s = SPEC
    stop = v * v / (2 * s.a_brake) + abs(v) * s.t_react + s.brake_margin
    return (abs(th) > s.theta_abort or abs(x) >= s.x_max or (v > 0 and x + stop >= s.x_max)
            or (v < 0 and x - stop <= -s.x_max))


def after_delay(p):
    st = (p["th0"], p["thd0"], p["x0"], p["v0"])
    for _ in range(p["delay"]):
        st = step_tick(st, 0.0, p)
    return st


def comfortable(p) -> bool:
    """Catchable with 1 cm of slack beyond the rig's 1 cm margin, staying within 18 deg where the linear model is
    still honest (holding a lean theta takes g tan(theta), not g theta: at 20 deg the linear model is 4 % optimistic).
    Without slack a plan can touch the braking boundary exactly, and friction or the nonlinearity then tips it over."""
    pl = Planner(p["w0"], p["bJ"], 0.04, 100, margin=SPEC.brake_margin + 0.01, theta_lim=math.radians(18),
                 terminal=True, cost=False)
    st = after_delay(p)
    return (not failed(st)) and pl.solve(st) is not None


def possible(p) -> bool:
    """The relaxation: no margin, the full 22 deg, the linear model (optimistic for a catch) and friction chosen in the
    controller's favour. False = no acceleration profile catches it."""
    pl = Planner(p["w0"], p["bJ"], 0.04, 100, margin=0.0, theta_lim=SPEC.theta_abort, terminal=True, cost=False,
                 fric_help=p["coulomb"])
    st = after_delay(p)
    return (not failed(st)) and pl.solve(st) is not None


def oracle(p, t_end: float = 10.0) -> tuple[bool, float]:
    """MPC from the true state: plan 2 s on a 40 ms grid, apply the first move, re-plan every 40 ms. Plans with a
    5 mm safety margin and 20 deg if it can, else on the rig's exact limits, else on the relaxed ones."""
    levels = [(SPEC.brake_margin + 0.01, math.radians(18)), (SPEC.brake_margin + 0.005, math.radians(18)),
              (SPEC.brake_margin, math.radians(18)), (SPEC.brake_margin, math.radians(20)), (0.0, math.radians(21))]
    planners: dict = {}
    per = int(round(0.04 / SPEC.ts))
    buf = collections.deque([0.0] * p["delay"])
    st = (p["th0"], p["thd0"], p["x0"], p["v0"])
    a_new = 0.0
    for k in range(int(round(t_end / SPEC.ts))):
        if k % per == 0:
            pred = st
            for a_b in buf:
                pred = step_tick(pred, a_b, p)
            for lvl in levels:
                if lvl not in planners:
                    planners[lvl] = Planner(p["w0"], p["bJ"], 0.04, 50, margin=lvl[0], theta_lim=lvl[1],
                                            terminal=False, cost=True)
                plan = planners[lvl].solve(pred)
                if plan is not None:
                    a_new = float(plan[0])
                    break
        a_cmd = float(rig.limit(a_new, st[3], SPEC))
        buf.append(a_cmd)
        st = step_tick(st, buf.popleft(), p)
        if failed(st):
            return False, (k + 1) * SPEC.ts
    return True, t_end


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=300)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--only", type=int, default=0, help="check only the first N lost starts (debugging)")
    ap.add_argument("--survivors", type=int, default=40, help="also run the oracle on this many PID survivors")
    a = ap.parse_args()
    t0 = time.time()
    ics = rig.draw(np.random.default_rng(a.seed), a.runs, rig.TEST, SPEC)
    pid = rig.run(rig.PidPolicy(), ics, rig.RigSpec(n_in=4, net_every=1), t_max=30.0)
    net = rig.run(Batch([Genome.load(NET)]), ics, SPEC, t_max=30.0)
    ok_pid, ok_net = pid["alive"][0], net["alive"][0]
    tf_pid = pid["t_fail"][0]
    xi0 = np.degrees(ics["th0"] + ics["thd0"] / ics["w0"])
    lost = np.flatnonzero(~ok_pid | ~ok_net)
    if a.only:
        lost = lost[:a.only]
    print(f"{a.runs} starts (rig.TEST, seed {a.seed}): PID + cart loop keeps {ok_pid.sum()}, budget network keeps "
          f"{ok_net.sum()}; {len(lost)} lost by either.  ({time.time() - t0:.0f} s)\n", flush=True)
    print("| start | xi0 (deg) | v0 (mm/s) | delay | PID | network | catchable with 1 cm slack | catchable at all "
          "| oracle MPC |")
    print("|---|---|---|---|---|---|---|---|---|")
    res = {}
    for i in lost:
        p = {k: (float(ics[k][i]) if k != "delay" else int(ics[k][i]))
             for k in ("th0", "thd0", "x0", "v0", "w0", "bJ", "coulomb", "delay")}
        f_rig, f_rel = comfortable(p), possible(p)
        o, t_o = oracle(p)
        res[i] = (f_rig, f_rel, o)
        pid_s = "keeps" if ok_pid[i] else (f"lost at {tf_pid[i]:.1f} s")
        net_s = "keeps" if ok_net[i] else f"lost at {net['t_fail'][0][i]:.1f} s"
        print(f"| {i} | {xi0[i]:+.1f} | {1000 * p['v0']:+.0f} | {p['delay']} | {pid_s} | {net_s} | "
              f"{'yes' if f_rig else 'no'} | {'yes' if f_rel else 'no'} | "
              f"{'holds 10 s' if o else f'lost at {t_o:.1f} s'} |", flush=True)
    sv = np.flatnonzero(ok_pid & ok_net)[:a.survivors]
    sv_ok = 0
    for i in sv:
        p = {k: (float(ics[k][i]) if k != "delay" else int(ics[k][i]))
             for k in ("th0", "thd0", "x0", "v0", "w0", "bJ", "coulomb", "delay")}
        sv_ok += oracle(p)[0]
    print(f"\noracle on {len(sv)} starts both controllers keep: holds {sv_ok}")
    catch = [i for i in lost if not ok_pid[i] and tf_pid[i] < 2.0]
    drift = [i for i in lost if not ok_pid[i] and tf_pid[i] >= 2.0]
    for name, grp in (("PID catch failures (< 2 s)", catch), ("PID drift failures (>= 2 s)", drift),
                      ("network failures", [i for i in lost if not ok_net[i]])):
        if not grp:
            continue
        f_rig = sum(res[i][0] for i in grp)
        f_rel = sum(res[i][1] for i in grp)
        o_n = sum(res[i][2] for i in grp)
        print(f"{name}: {len(grp)}; catchable with 1 cm slack {f_rig}, catchable at all {f_rel}; oracle holds {o_n}")
    best = ok_pid | ok_net | np.array([res.get(i, (False, False, False))[2] for i in range(a.runs)])
    print(f"\nkept by PID {100 * ok_pid.mean():.0f} %, network {100 * ok_net.mean():.0f} %, "
          f"PID or network or oracle {100 * best.mean():.0f} %   ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
