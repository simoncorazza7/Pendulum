"""Pasemann's benchmark cart-pole (section 3 of the paper): force input, explicit Euler at 0.01 s, no friction.

    theta'' = (m g sin th - cos th (F + mp l th'^2 sin th)) / (l (4/3 m - mp cos^2 th))
    x''     = (F + mp l (th'^2 sin th - th'' cos th)) / m
mc = 1.0 kg, mp = 0.1 kg, m = mc + mp, l = 0.5 m (half the pole), g = 9.81 m/s^2. A positive force makes theta''
more negative, so the controller pushes the way the pole leans. Failure: |x| > 2.4 m or |theta| > 12 deg.
Inputs: in1 = x/2.4 and in2 = 15 theta/pi (= theta/12 deg); with four inputs also in3 = x'/2.4, in4 = 15 theta'/pi.
Output performance P = sum over the steps survived of dt (0.5 (1 - 15|theta|/pi) + 0.5 (1 - |x|/2.4)): seconds
balanced, discounted for leaning and for distance from the centre. I_F = sum |F| dt / t_max.
"""

from __future__ import annotations

import math

import numpy as np

from .net import Batch

G, MC, MP, L, DT = 9.81, 1.0, 0.1, 0.5, 0.01   # m/s^2, kg, kg, m (half the pole), s
M = MC + MP                                     # kg
X_LIM = 2.4                                     # m
TH_LIM = math.pi / 15                           # rad (12 deg)
F_MAX = 10.0                                    # N


def accel(th, thd, F):
    """(x'', theta'') of the paper's cart-pole."""
    s, c = np.sin(th), np.cos(th)
    thdd = (M * G * s - c * (F + MP * L * thd ** 2 * s)) / (L * (4.0 / 3.0 * M - MP * c ** 2))
    xdd = (F + MP * L * (thd ** 2 * s - thdd * c)) / M
    return xdd, thdd


def scaled_inputs(n_in: int, x, xd, th, thd):
    if n_in == 2:
        return np.stack([x / X_LIM, th / TH_LIM], axis=-1)
    if n_in == 4:
        return np.stack([x / X_LIM, th / TH_LIM, xd / X_LIM, thd / TH_LIM], axis=-1)
    raise ValueError("the benchmark has 2 or 4 inputs")


def grid(n: int = 40):
    """The paper's 40 x 40 benchmark start states (x0, theta0), velocities 0. Square centres are spaced so that the
    outer ring lies just outside the limits (the paper: outer squares 'already produce a failure signal')."""
    dx, dth = 2 * X_LIM / (n - 2), 2 * TH_LIM / (n - 2)
    xs = -X_LIM + (np.arange(n) - 0.5) * dx
    ths = -TH_LIM + (np.arange(n) - 0.5) * dth
    X0, TH0 = np.meshgrid(xs, ths, indexing="xy")      # rows: theta, columns: x
    return X0.ravel(), TH0.ravel(), (n, n)


def run(batch: Batch, x0, th0, xd0=0.0, thd0=0.0, t_max: float = 20.0, timing: str = "immediate",
        integrator: str = "semi", trace: tuple | None = None) -> dict:
    """Run every network in batch from K start states (x0, th0: shape (K,); velocities scalars or (K,)).

    timing 'immediate': the network steps on the current inputs and its new output acts at once. 'delayed': network
    and cart-pole step together and F(t) = 10 u(a(t)), so inputs reach the force one step late.
    integrator 'semi': semi-implicit Euler (velocities first, positions with the new velocities: adds no energy to
    an oscillation). 'explicit': positions with the old velocities (slowly pumps energy into oscillations).
    The paper says only 'Euler discretization'; its published plots (w1 settles with a smooth force, w4's pole
    oscillation dies out) match 'immediate' + 'semi' and not the others (docs/neuro_sim.md).
    trace (p, k): also record t, x, theta and F of that run.
    Returns (P, K) arrays: perf (P, s), IF (N), t_fail (s; t_max when it never failed), alive (at t_max).
    Failed runs are frozen, so their numbers stay finite.
    """
    x0 = np.atleast_1d(np.asarray(x0, float))
    K = len(x0)
    shape = (batch.P, K)

    def init(v):
        return np.broadcast_to(np.asarray(v, float) * np.ones(K), shape).copy()

    x, th, xd, thd = init(x0), init(th0), init(xd0), init(thd0)
    a = batch.reset(K)
    n = int(round(t_max / DT))
    alive = (np.abs(x) <= X_LIM) & (np.abs(th) <= TH_LIM)
    perf, IF = np.zeros(shape), np.zeros(shape)
    t_fail = np.where(alive, t_max, 0.0)
    rec = {"t": [], "x": [], "theta": [], "F": []} if trace is not None else None
    for i in range(n):
        inp = scaled_inputs(batch.n_in, x, xd, th, thd)
        if timing == "delayed":
            u = batch.command(a)
            a = batch.step(a, inp)
        else:
            a = batch.step(a, inp)
            u = batch.command(a)
        F = F_MAX * u
        xdd, thdd = accel(th, thd, F)
        xd1, thd1 = xd + DT * xdd, thd + DT * thdd
        if integrator == "semi":
            x1, th1 = x + DT * xd1, th + DT * thd1
        else:
            x1, th1 = x + DT * xd, th + DT * thd
        x, xd = np.where(alive, x1, x), np.where(alive, xd1, xd)
        th, thd = np.where(alive, th1, th), np.where(alive, thd1, thd)
        IF += np.where(alive, np.abs(F) * DT / t_max, 0.0)
        failed = alive & ((np.abs(x) > X_LIM) | (np.abs(th) > TH_LIM))
        t_fail[failed] = (i + 1) * DT
        alive &= ~failed
        perf += np.where(alive, DT * (0.5 * (1 - np.abs(th) / TH_LIM) + 0.5 * (1 - np.abs(x) / X_LIM)), 0.0)
        if rec is not None:
            p, k = trace
            rec["t"].append((i + 1) * DT)
            rec["x"].append(x[p, k])
            rec["theta"].append(th[p, k])
            rec["F"].append(F[p, k])
        if not alive.any():
            break
    out = {"perf": perf, "IF": IF, "t_fail": t_fail, "alive": alive}
    if rec is not None:
        out["trace"] = {k: np.array(v) for k, v in rec.items()}
    return out
