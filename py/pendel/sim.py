"""Closed-loop simulation: nonlinear plant, sampled controller, delay, quantisation, Axis cap."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .control import DelayLine, PidParams, PidState, pid_step
from .plant import ActuatorParams, PlantParams, SensorParams, measure, pendulum_accel, realised_accel


@dataclass(frozen=True)
class SimParams:
    ts: float = 0.002             # s, controller period
    substeps: int = 20            # plant RK4 steps per controller period
    delay_samples: int = 0        # extra N-sample delay line (Stage 2a)
    theta_abort: float = 0.4      # rad
    x_limit: float = 0.25         # m, |x| at which the run ends (soft limit)


def simulate(pid: PidParams, theta0: float, thetadot0: float, t_end: float,
             plant: PlantParams = PlantParams(), act: ActuatorParams = ActuatorParams(),
             sens: SensorParams = SensorParams(), sp: SimParams = SimParams(), estimator=None) -> dict:
    """Engaged at t = 0 with the cart at rest at x = 0. Returns per-sample arrays and an end reason.

    estimator: optional object with update(t, count, events) -> rad/s (py/pendel/estimators.py); it
    then replaces the controller's built-in velocity filter. Edge times come from the RK4 substeps.
    """
    n = int(round(t_end / sp.ts))
    h = sp.ts / sp.substeps
    d_sub = int(round(act.compute_delay / h))            # compute delay in substeps (< one period)
    th, thd, x, v = theta0, thetadot0, 0.0, 0.0
    a_applied = 0.0
    state = PidState()
    delay = DelayLine(sp.delay_samples)
    rec = {k: np.full(n, np.nan) for k in
           ("t", "theta", "thetadot", "x", "v", "a_cmd", "a_real", "theta_meas", "vel_f", "counts", "sat")}
    reason = "time"
    q = 2 * np.pi / sens.cpr
    c_prev = int(np.floor(th / q + sens.phase))
    events = []
    for k in range(n):
        counts, th_meas = measure(th, sens)
        vel_est = None if estimator is None else estimator.update(k * sp.ts, counts, events)
        events = []
        a_ctrl, state, dbg = pid_step(pid, state, th_meas, v, sp.ts, vel_est)
        a_new = delay.push(a_ctrl)
        a_real = realised_accel(a_applied, v, act)
        rec_vals = dict(t=k * sp.ts, theta=th, thetadot=thd, x=x, v=v, a_cmd=a_ctrl, a_real=a_real,
                        theta_meas=th_meas, vel_f=dbg["vel_f"], counts=counts, sat=float(dbg["saturated"]))
        for key, val in rec_vals.items():
            rec[key][k] = val
        if abs(th) > sp.theta_abort:
            reason = "theta_abort"
            break
        if abs(x) > sp.x_limit:
            reason = "x_limit"
            break
        # Integrate one period; the new command takes effect after the compute delay.
        for j in range(sp.substeps):
            if j == d_sub:
                a_applied = a_new
            a_r = realised_accel(a_applied, v, act)

            def f(s):
                return np.array([s[1], pendulum_accel(s[0], s[1], a_r, plant), s[3], a_r])

            s0 = np.array([th, thd, x, v])
            k1 = f(s0)
            k2 = f(s0 + 0.5 * h * k1)
            k3 = f(s0 + 0.5 * h * k2)
            k4 = f(s0 + h * k3)
            th_old = th
            th, thd, x, v = s0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            if estimator is not None:   # edge events within the substep, linear interpolation
                u0, u1 = th_old / q + sens.phase, th / q + sens.phase
                c_new = int(np.floor(u1))
                t0 = k * sp.ts + j * h
                for b in (range(c_prev + 1, c_new + 1) if c_new > c_prev else range(c_prev, c_new, -1)):
                    events.append((t0 + h * (b - u0) / (u1 - u0), b if c_new > c_prev else b - 1))
                c_prev = c_new
    last = int(np.nanargmax(rec["t"])) + 1 if np.any(~np.isnan(rec["t"])) else 0
    out = {key: arr[:last] for key, arr in rec.items()}
    out["reason"] = reason
    return out
