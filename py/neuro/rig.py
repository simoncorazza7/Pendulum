"""The Pendel rig as an environment for evolved networks: acceleration input, identified pendulum, rig limits.

A vectorised mirror of py/pendel/plant.py (pendulum_accel, realised_accel, measure; test_neuro.py checks the two
against each other) with the balance session's limits and supervisor checks from config/pendctl.conf:
    theta'' = w0^2 sin th - (w0^2/g) a cos th - (b/J) th' - c sgn(th')        x'' = a (the stepper imposes it)
Command path: u in [-1, 1] from the network, a = a_max u, then pendel.control.pid_step's v_max taper, then a
constant delay of delay_ticks (remote.delay_ticks on the PC path), then the Axis cap (realised_accel).
Failure: |theta| > theta_abort, or the soft limit with the supervisor's braking distance (as py/catch_study.py).
Inputs: in1 = x/x_max, in2 = theta_meas/12 deg (the paper's angle scaling), in3 = v/v_max. On a stepper rig the
cart's x and v are integrals of the commanded acceleration (exact while no steps are lost), so the network gets
them for free and only has to work out theta' itself. theta_meas is the quantised encoder angle minus the upright
reference error. With n_in = 4 it also gets in4 = omega/12 deg, the Pi's own velocity estimate (backward difference
+ 2nd-order Butterworth at ctrl.est_fc, pendel.estimators.BdLp2, run every tick), which the remote path's SAMPLE
packet already carries; that is the realistic deployment, the 3-input case is Pasemann's harder problem.
Output performance, as the paper's but with the rig's limits: dt (0.5 (1 - |theta|/theta_abort) + 0.5 (1 - |x|/x_max))
summed while balancing. IA = sum |a| dt / t_max (m/s^2), DU = sum |u_k - u_k-1| / t_max over network updates (1/s).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .net import Batch

G = 9.81  # m/s^2


@dataclass(frozen=True)
class RigSpec:
    ts: float = 0.002                       # s, control period (loop.ts)
    substeps: int = 4                       # plant integration steps per tick
    net_every: int = 5                      # ticks per network update (5: 100 Hz, the paper's 0.01 s)
    timing: str = "immediate"               # "immediate": output used right after its update; "delayed": one update late
    a_max: float = 2.0                      # m/s^2 (balance.a_max)
    v_max: float = 0.3                      # m/s (balance.v_max)
    v_taper: float = 0.25                   # fraction of v_max over which authority tapers (ctrl.v_taper)
    axis_a: float = 400000 / (2 * 13333)    # m/s^2, Axis acceleration cap (axis.max_accel, 15 m/s^2)
    axis_v: float = 120000 / (2 * 13333)    # m/s, Axis speed cap (axis.max_speed, 4.5 m/s)
    x_max: float = 0.15                     # m, soft limits about the start (sup.x_max)
    a_brake: float = 1.0                    # m/s^2 (sup.a_brake)
    t_react: float = 0.006                  # s (sup.t_react)
    brake_margin: float = 0.01              # m (sup.brake_margin)
    theta_abort: float = math.radians(22)   # rad (sup.theta_abort_deg)
    cpr: int = 2400                         # counts/rev (encoder.counts_per_rev)
    phase: float = 0.37                     # counts, where the count edges fall (pendel.plant.SensorParams)
    th_scale: float = math.pi / 15          # rad, angle input scaling (12 deg, as the paper)
    est_fc: float = 14.0                    # Hz, the Pi's velocity estimator cut-off (ctrl.est_fc)
    n_in: int = 3                           # 3: x, theta, v; 4: + the Pi's velocity estimate


@dataclass(frozen=True)
class Draw:
    """Ranges for start states and plants; every run draws its own (uniform)."""
    theta_deg: float = 3.0                  # +- deg at engage (the supervisor engages below 4 deg)
    omega: float = 0.3                      # +- rad/s (the supervisor engages below 0.5 rad/s)
    x: float = 0.02                         # +- m
    v: float = 0.0                          # +- m/s
    ref_counts: float = 1.5                 # +- counts of upright reference error (0 to 2.1 seen between sessions)
    w0: tuple = (4.97, 4.97)                # rad/s (plant.w0, 2026-11-02)
    bJ: tuple = (0.054, 0.054)              # 1/s (plant.bJ)
    coulomb: tuple = (0.168, 0.168)         # rad/s^2 (plant.coulomb)
    delay_ticks: tuple = (4, 4)             # inclusive (remote.delay_ticks 4; stalls to 7 seen)


TRAIN = Draw()
TEST = Draw(theta_deg=3.5, omega=0.4, x=0.03, v=0.03, ref_counts=2.0, w0=(4.82, 5.12), bJ=(0.03, 0.08),
            coulomb=(0.10, 0.25), delay_ticks=(4, 7))


def draw(rng: np.random.Generator, K: int, d: Draw, spec: RigSpec = RigSpec()) -> dict:
    q = 2 * math.pi / spec.cpr
    return {"th0": rng.uniform(-1, 1, K) * math.radians(d.theta_deg),
            "thd0": rng.uniform(-1, 1, K) * d.omega,
            "x0": rng.uniform(-1, 1, K) * d.x,
            "v0": rng.uniform(-1, 1, K) * d.v,
            "ref": rng.uniform(-1, 1, K) * d.ref_counts * q,
            "w0": rng.uniform(*d.w0, K), "bJ": rng.uniform(*d.bJ, K), "coulomb": rng.uniform(*d.coulomb, K),
            "delay": rng.integers(d.delay_ticks[0], d.delay_ticks[1] + 1, K)}


def measure(th, spec: RigSpec, ref):
    """pendel.plant.measure, vectorised: centred quantisation minus the reference error."""
    q = 2 * math.pi / spec.cpr
    counts = np.floor(th / q + spec.phase)
    return (counts - spec.phase + 0.5) * q - ref


def limit(a, v, spec: RigSpec):
    """pendel.control.pid_step's saturation: +-a_max, no authority toward a v_max that is near."""
    w = spec.v_taper * spec.v_max
    a_hi = spec.a_max * np.clip((spec.v_max - v) / w, 0.0, 1.0)
    a_lo = -spec.a_max * np.clip((spec.v_max + v) / w, 0.0, 1.0)
    return np.minimum(a_hi, np.maximum(a_lo, a))


def axis_cap(a, v, spec: RigSpec):
    """pendel.plant.realised_accel, vectorised."""
    cap = spec.axis_a * np.maximum(0.0, 1.0 - np.abs(v) / spec.axis_v)
    return np.minimum(cap, np.maximum(-cap, a))


def pend_accel(th, thd, a, w0, bJ, coulomb):
    """pendel.plant.pendulum_accel, vectorised (theta from upright)."""
    return w0 ** 2 * np.sin(th) - (w0 ** 2 / G) * a * np.cos(th) - bJ * thd - coulomb * np.sign(thd)


def bdlp2(ts: float, fc: float) -> tuple:
    """Coefficients (b0, b1, b2, a1, a2) of pendel.estimators.BdLp2's low-pass (bilinear, prewarped Butterworth)."""
    k = math.tan(math.pi * fc * ts)
    n = 1 / (1 + math.sqrt(2) * k + k * k)
    b0 = k * k * n
    return b0, 2 * b0, b0, 2 * (k * k - 1) * n, (1 - math.sqrt(2) * k + k * k) * n


class PidPolicy:
    """The rig's own balance controller as the yardstick: pendel.control.cascade_step with the config's gains
    (pole placement wc = 2 w0, zeta 0.7, p = 0.5 w0 from the nominal plant; Stage 3b cart loop with its integral),
    fed the same measured angle, the Pi's velocity estimate and the cart's x and v. One state per run, scalar
    loop: slow, for evaluation only. Run it with RigSpec(net_every=1): the PID updates every tick."""

    def __init__(self, w0: float = 4.97, bJ: float = 0.054, a_max: float = 2.0, v_max: float = 0.3,
                 cascade: bool = True):
        from pendel.control import PidParams, design_gains
        kp, ki, kd = design_gains(w0, bJ, 2.0 * w0, 0.7, 0.5 * w0)
        self.p = PidParams(kp=kp, ki=ki, kd=kd, a_max=a_max, v_max=v_max, v_taper=0.25, i_max=1.0,
                           cart_kx=0.41 if cascade else 0.0, cart_kv=1.0 if cascade else 0.0,
                           cart_theta_max=math.radians(4), cart_ki=0.05 if cascade else 0.0,
                           cart_i_max=math.radians(2))
        self.P = 1

    def reset(self, K: int) -> None:
        from pendel.control import PidState
        self.s = [PidState() for _ in range(K)]

    def act(self, x, thm, v, om, dt):
        from pendel.control import cascade_step
        out = np.zeros((1, len(self.s)))
        for k in range(len(self.s)):
            out[0, k], self.s[k], _ = cascade_step(self.p, self.s[k], float(thm[0, k]), float(x[0, k]),
                                                   float(v[0, k]), dt, float(om[0, k]))
        return out


def run(policy, ics: dict, spec: RigSpec = RigSpec(), t_max: float = 10.0, trace: tuple | None = None) -> dict:
    """Run a Batch of networks (or another policy, e.g. PidPolicy) from the K start states and plants in ics.

    Returns (P, K) arrays: perf (s), IA (m/s^2), DU (1/s), t_fail (s; t_max if it never failed), alive, and
    end_x (m) where each run ended. trace (p, k): also t, x, v, theta, a_cmd of that run.
    """
    net = isinstance(policy, Batch)
    if net and policy.n_in != spec.n_in:
        raise ValueError(f"the rig model feeds {spec.n_in} inputs")
    batch = policy
    K = len(ics["th0"])
    shape = (policy.P, K)

    def init(v):
        return np.broadcast_to(np.asarray(v, float), shape).copy()

    th, thd, x, v = init(ics["th0"]), init(ics["thd0"]), init(ics["x0"]), init(ics["v0"])
    ref, w0, bJ, coul = (np.asarray(ics[k], float)[None, :] for k in ("ref", "w0", "bJ", "coulomb"))
    delay = np.asarray(ics["delay"], int)
    D = int(delay.max())
    buf = np.zeros((D + 1, batch.P, K))          # command of the last D+1 ticks (0 before engage)
    cols = np.arange(K)
    if net:
        a_net = batch.reset(K)
    else:
        policy.reset(K)
    a_cmd = np.zeros(shape)
    u_prev = np.zeros(shape)
    alive = np.ones(shape, bool)
    perf, IA, DU = np.zeros(shape), np.zeros(shape), np.zeros(shape)
    t_fail = np.full(shape, t_max)
    n = int(round(t_max / spec.ts))
    h = spec.ts / spec.substeps
    four = spec.n_in == 4
    # the Pi's estimator runs every tick, warm at engage (it runs all the time); used by 4-input nets and the PID
    b0, b1, b2, a1, a2 = bdlp2(spec.ts, spec.est_fc)
    q = 2 * math.pi / spec.cpr
    cnt_prev = np.floor((th - thd * spec.ts) / q + spec.phase)
    ex1, ex2, ey1, ey2 = thd.copy(), thd.copy(), thd.copy(), thd.copy()
    rec = {"t": [], "x": [], "v": [], "theta": [], "a_cmd": []} if trace is not None else None
    for k in range(n):
        cnt = np.floor(th / q + spec.phase)
        xin = (cnt - cnt_prev) * q / spec.ts
        cnt_prev = cnt
        om = b0 * xin + b1 * ex1 + b2 * ex2 - a1 * ey1 - a2 * ey2
        ex2, ex1, ey2, ey1 = ex1, xin, ey1, om
        if k % spec.net_every == 0:
            thm = measure(th, spec, ref)
            if net:
                feats = [x / spec.x_max, thm / spec.th_scale, v / spec.v_max]
                if four:
                    feats.append(om / spec.th_scale)
                inp = np.stack(feats, axis=-1)
                if spec.timing == "delayed":
                    u = batch.command(a_net)
                    a_net = batch.step(a_net, inp)
                else:
                    a_net = batch.step(a_net, inp)
                    u = batch.command(a_net)
                a_want = spec.a_max * u
            else:
                a_want = policy.act(x, thm, v, om, spec.ts * spec.net_every)
                u = a_want / spec.a_max
            a_cmd = limit(a_want, v, spec)
            DU += np.where(alive, np.abs(u - u_prev) / t_max, 0.0)
            u_prev = u
        buf[k % (D + 1)] = a_cmd
        a_app = buf[(k - delay) % (D + 1), :, cols].T        # each run's own delay
        th1, thd1, x1, v1 = th, thd, x, v
        for _ in range(spec.substeps):
            a_r = axis_cap(a_app, v1, spec)
            acc = pend_accel(th1, thd1, a_r, w0, bJ, coul)
            th1 = th1 + thd1 * h + 0.5 * acc * h * h
            thd1 = thd1 + acc * h
            x1 = x1 + v1 * h + 0.5 * a_r * h * h
            v1 = v1 + a_r * h
        th, thd = np.where(alive, th1, th), np.where(alive, thd1, thd)
        x, v = np.where(alive, x1, x), np.where(alive, v1, v)
        stop = v * v / (2 * spec.a_brake) + np.abs(v) * spec.t_react + spec.brake_margin
        failed = alive & ((np.abs(th) > spec.theta_abort) | (np.abs(x) >= spec.x_max)
                          | ((v > 0) & (x + stop >= spec.x_max)) | ((v < 0) & (x - stop <= -spec.x_max)))
        t_fail[failed] = (k + 1) * spec.ts
        alive &= ~failed
        perf += np.where(alive, spec.ts * (0.5 * (1 - np.abs(th) / spec.theta_abort)
                                           + 0.5 * (1 - np.abs(x) / spec.x_max)), 0.0)
        IA += np.where(alive, np.abs(a_app) * spec.ts / t_max, 0.0)
        if rec is not None:
            p, kk = trace
            rec["t"].append((k + 1) * spec.ts)
            rec["x"].append(x[p, kk])
            rec["v"].append(v[p, kk])
            rec["theta"].append(th[p, kk])
            rec["a_cmd"].append(a_cmd[p, kk])
        if not alive.any():
            break
    out = {"perf": perf, "IA": IA, "DU": DU, "t_fail": t_fail, "alive": alive, "end_x": x}
    if rec is not None:
        out["trace"] = {key: np.array(val) for key, val in rec.items()}
    return out
