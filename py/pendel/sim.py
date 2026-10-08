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
             sens: SensorParams = SensorParams(), sp: SimParams = SimParams()) -> dict:
    """Engaged at t = 0 with the cart at rest at x = 0. Returns per-sample arrays and an end reason."""
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
    for k in range(n):
        counts, th_meas = measure(th, sens)
        a_ctrl, state, dbg = pid_step(pid, state, th_meas, v, sp.ts)
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
            th, thd, x, v = s0 + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    last = int(np.nanargmax(rec["t"])) + 1 if np.any(~np.isnan(rec["t"])) else 0
    out = {key: arr[:last] for key, arr in rec.items()}
    out["reason"] = reason
    return out
