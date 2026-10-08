"""Simulated hardware in raw units (Python reference of src/hw/sim_hw.*).

Same interface as hw::Hardware: read() -> sample dict, command_accel(axis_arg), enable_driver(on),
halt(), is_stopped(), plus advance(dt) to move simulated time. The "true" calibration can differ
from the runtime's, to test calibration errors.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .units import Calib


def lround(x: float) -> int:
    """Round half away from zero, like C++ std::llround."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


@dataclass(frozen=True)
class SimHwParams:
    w0: float = 2 * math.pi          # rad/s, hanging small-angle frequency
    bJ: float = 0.1                  # 1/s, viscous pivot friction b/J
    coulomb: float = 0.0             # rad/s^2, Coulomb friction acceleration
    g: float = 9.81
    axis_a: float = 4.5              # m/s^2, Axis cap A(1 - |v|/V) and HALT deceleration
    axis_v: float = 4.5              # m/s
    compute_delay: float = 0.0005    # s, command takes effect this long after commandAccel
    coast_decel: float = 0.5         # m/s^2, cart deceleration while the driver is off (approximation)
    substeps: int = 20               # RK4 steps per advance(dt)
    pos_switch_right: float = 250.0  # usteps: right switch closed for pos <= this
    pos_switch_left: float = 7750.0  # usteps: left switch closed for pos >= this
    pos_end_right: float = 0.0       # usteps: hard stop
    pos_end_left: float = 8000.0     # usteps: hard stop
    truth: Calib = field(default_factory=lambda: Calib(c_bottom=12.37, pos_center=4000.0))


class SimHw:
    def __init__(self, p: SimHwParams, theta0: float = math.pi, thetadot0: float = 0.0, x0: float = 0.0):
        self.p = p
        self.th, self.thd, self.x, self.v = theta0, thetadot0, x0, 0.0
        self.t = 0.0
        self.a_applied = 0.0
        self.a_pending = None
        self.driver = False
        self.halted = False
        self.commanded = False
        self.crashed = False
        self.u0_floor = math.floor(self._u(theta0))   # decoder zero = count at "program start"
        self.c_prev = self.u0_floor
        self.edges = []

    # -- raw-unit helpers --
    def _u(self, th: float) -> float:
        c = self.p.truth
        return c.c_bottom + c.cpr / 2 + c.theta_sign * th * c.cpr / (2 * math.pi)

    def _pos(self, x: float) -> float:
        c = self.p.truth
        return c.pos_center + c.x_sign * x * c.usteps_per_m

    def true_c_bottom_relative(self) -> float:
        """c_bottom in the decoder's counts (what the start-up fit should find)."""
        return self.p.truth.c_bottom - self.u0_floor

    # -- hardware interface --
    def read(self) -> dict:
        c = self.p.truth
        pos = self._pos(self.x)
        speed_protocol = c.x_sign * self.v * c.usteps_per_m * c.toggles_per_ustep
        s = {
            "t_ns": lround(self.t * 1e9),
            "counts": self.c_prev - self.u0_floor,
            "edges": self.edges[:64],
            "edges_overflow": len(self.edges) > 64,
            "pos_usteps": lround(pos),
            "speed_toggles": -lround(speed_protocol),
            "limit_right": pos <= self.p.pos_switch_right,
            "limit_left": pos >= self.p.pos_switch_left,
            "axis_accelerating": self.driver and self.commanded and not self.halted,
            "encoder_fault": False,
            "driver_enabled": self.driver,
        }
        self.edges = []
        return s

    def command_accel(self, axis_arg: int) -> None:
        c = self.p.truth
        if self.halted:
            return
        self.a_pending = -c.x_sign * axis_arg / (c.usteps_per_m * c.toggles_per_ustep)
        self.commanded = True

    def enable_driver(self, on: bool) -> None:
        self.driver = on

    def halt(self) -> None:
        self.halted = True

    def is_stopped(self) -> bool:
        return self.v == 0.0

    # -- simulation --
    def _realised(self, v: float, h: float) -> float:
        p = self.p
        if not self.driver:
            return -math.copysign(min(p.coast_decel, abs(v) / h), v) if v != 0.0 else 0.0
        if self.halted:
            return -math.copysign(min(p.axis_a, abs(v) / h), v) if v != 0.0 else 0.0
        cap = p.axis_a * max(0.0, 1.0 - abs(v) / p.axis_v)
        return min(cap, max(-cap, self.a_applied))

    def _f(self, s, a):
        th, thd, x, v = s
        p = self.p
        w2 = p.w0 * p.w0
        acc = w2 * math.sin(th) - (w2 / p.g) * a * math.cos(th) - p.bJ * thd
        if p.coulomb and thd != 0.0:
            acc -= math.copysign(p.coulomb, thd)
        return (thd, acc, v, a)

    def advance(self, dt: float) -> None:
        p = self.p
        h = dt / p.substeps
        d_sub = lround(p.compute_delay / h)
        for j in range(p.substeps):
            if j == d_sub and self.a_pending is not None:
                self.a_applied = self.a_pending
                self.a_pending = None
            a = self._realised(self.v, h)
            s0 = (self.th, self.thd, self.x, self.v)
            k1 = self._f(s0, a)
            s1 = tuple(s0[i] + 0.5 * h * k1[i] for i in range(4))
            k2 = self._f(s1, a)
            s2 = tuple(s0[i] + 0.5 * h * k2[i] for i in range(4))
            k3 = self._f(s2, a)
            s3 = tuple(s0[i] + h * k3[i] for i in range(4))
            k4 = self._f(s3, a)
            new = tuple(s0[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4))
            th_old = self.th
            self.th, self.thd, self.x, self.v = new
            if (self.halted or not self.driver) and self.v * s0[3] < 0:   # braking/coasting stops at 0
                self.v = 0.0
            pos = self._pos(self.x)
            if pos <= p.pos_end_right or pos >= p.pos_end_left:           # hard stop: crash
                end = p.pos_end_right if pos <= p.pos_end_right else p.pos_end_left
                self.x = (end - p.truth.pos_center) / (p.truth.x_sign * p.truth.usteps_per_m)
                self.v = 0.0
                self.crashed = True
            u0, u1 = self._u(th_old), self._u(self.th)
            c_new = math.floor(u1)
            if c_new != self.c_prev:
                t0 = self.t + j * h
                bounds = range(self.c_prev + 1, c_new + 1) if c_new > self.c_prev else range(self.c_prev, c_new, -1)
                for b in bounds:
                    ts = t0 + h * (b - u0) / (u1 - u0)
                    after = (b if c_new > self.c_prev else b - 1) - self.u0_floor
                    self.edges.append((lround(ts * 1e9), after))
                self.c_prev = c_new
        self.t += dt
