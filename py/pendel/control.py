"""Reference controller step function (Stage 1 design, Python mirror of the future C++ core).

Pure and deterministic: params + state + measurement + dt -> command + new state. No I/O, no
clock. The C++ version must reproduce this to floating-point tolerance (golden test).

Sign convention (docs/model_derivation.md): theta > 0 when the top leans toward +x; a > 0
accelerates the cart toward +x; positive gains move the cart under the pendulum.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class PidParams:
    kp: float                 # (m/s^2)/rad
    ki: float                 # (m/s^2)/(rad*s)
    kd: float                 # (m/s^2)/(rad/s)
    a_max: float              # m/s^2, command saturation
    v_max: float              # m/s, cart speed limit (authority tapers before it)
    v_taper: float = 0.25     # fraction of v_max over which authority toward the limit ramps to 0
    i_max: float = 0.0        # m/s^2, clamp of |ki * integral| (0: no clamp)
    i_delay: float = 0.0      # s after engage before the integrator runs (hand still on the rod)
    theta_ref: float = 0.0    # rad, upright reference trim
    vel_fc: float = 30.0      # Hz, 1st-order low-pass on the backward-difference velocity
    cart_kx: float = 0.0      # 1/s^2, Stage 3 outer loop: wanted cart accel -cart_kx x - cart_kv v (0, 0: angle only)
    cart_kv: float = 0.0      # 1/s
    cart_theta_max: float = 0.0   # rad, clamp of the outer loop's angle reference


@dataclass(frozen=True)
class PidState:
    integral: float = 0.0     # rad*s, integral of the error
    t_engaged: float = 0.0    # s since engage
    vel_f: float = 0.0        # rad/s, filtered velocity estimate
    prev_theta: float | None = None   # rad, previous measured angle (unwrapped)


def taper(margin: float, width: float) -> float:
    """1 when margin >= width, linear to 0 at margin <= 0."""
    if width <= 0:
        return 1.0 if margin > 0 else 0.0
    return min(1.0, max(0.0, margin / width))


def pid_step(p: PidParams, s: PidState, theta: float, v_cart: float, dt: float,
             vel_est: float | None = None) -> tuple[float, PidState, dict]:
    """One controller sample.

    theta  measured angle from upright, unwrapped (rad), e.g. from cumulative encoder counts
    v_cart cart velocity (m/s), as realised by the Axis (used only for the v_max taper)
    dt     sample period (s)
    vel_est optional external velocity estimate (rad/s) replacing the built-in filtered backward
           difference (estimator comparison, Stage 0.11; not mirrored in C++ yet)
    Returns (a_cmd in m/s^2, new state, debug dict).
    """
    # Derivative of the measurement (not of the error), filtered.
    if s.prev_theta is None:
        vel_raw = 0.0
    else:
        vel_raw = (theta - s.prev_theta) / dt
    alpha = dt / (dt + 1.0 / (2.0 * math.pi * p.vel_fc))
    vel_f = s.vel_f + alpha * (vel_raw - s.vel_f) if vel_est is None else vel_est

    e = theta - p.theta_ref
    u = p.kp * e + p.ki * s.integral + p.kd * vel_f

    # Saturation with authority taper near +-v_max: no more acceleration toward the limit.
    w = p.v_taper * p.v_max
    a_hi = p.a_max * taper(p.v_max - v_cart, w)
    a_lo = -p.a_max * taper(p.v_max + v_cart, w)
    a_cmd = min(a_hi, max(a_lo, u))
    saturated = a_cmd != u

    # Conditional integration: hold the integrator while it would push further into saturation,
    # and during the enable delay. Clamp its contribution.
    integral = s.integral
    if s.t_engaged >= p.i_delay and p.ki != 0.0:
        pushes_out = saturated and (e * (u - a_cmd) > 0)
        if not pushes_out:
            integral += e * dt
        if p.i_max > 0:
            lim = p.i_max / abs(p.ki)
            integral = min(lim, max(-lim, integral))

    new = PidState(integral=integral, t_engaged=s.t_engaged + dt, vel_f=vel_f, prev_theta=theta)
    return a_cmd, new, {"u": u, "e": e, "vel_raw": vel_raw, "vel_f": vel_f, "saturated": saturated}


G_CASCADE = 9.81             # m/s^2, the outer loop's lean per acceleration (C++: core::kGCascade)


def cart_theta_ref(p: PidParams, x_cart: float, v_cart: float) -> float:
    """Stage 3 outer loop (docs/stage3_cascade.md): the lean that makes the cart accelerate by
    a_d = -cart_kx x - cart_kv v. Holding a lean theta takes a cart acceleration g theta, so the angle loop
    turns this reference into that acceleration at frequencies well below its own bandwidth."""
    if p.cart_kx == 0.0 and p.cart_kv == 0.0:
        return 0.0
    a_d = -p.cart_kx * x_cart - p.cart_kv * v_cart
    return min(p.cart_theta_max, max(-p.cart_theta_max, a_d / G_CASCADE))


def cascade_step(p: PidParams, s: PidState, theta: float, x_cart: float, v_cart: float, dt: float,
                 vel_est: float | None = None) -> tuple[float, PidState, dict]:
    """Cascaded PID (brief Stage 3): pid_step with theta_ref + cart_theta_ref(x, v). With cart_kx = cart_kv = 0
    it is exactly pid_step. x_cart: cart position from the track centre (m)."""
    q = replace(p, theta_ref=p.theta_ref + cart_theta_ref(p, x_cart, v_cart))
    a, s2, dbg = pid_step(q, s, theta, v_cart, dt, vel_est)
    dbg["theta_ref"] = q.theta_ref
    return a, s2, dbg


@dataclass
class DelayLine:
    """N-sample delay on the command path (Stage 2a). N = 0: pass-through."""
    n: int = 0
    buf: deque = field(default_factory=deque)

    def push(self, a: float) -> float:
        if self.n == 0:
            return a
        self.buf.append(a)
        if len(self.buf) <= self.n:
            return 0.0
        return self.buf.popleft()


def design_gains(w0: float, bJ: float, wc: float, zeta: float, p: float, g: float = 9.81) -> tuple[float, float, float]:
    """Pole placement (docs/model_derivation.md §6): returns (kp, ki, kd)."""
    kp = g * (1 + (wc**2 + 2 * zeta * wc * p) / w0**2)
    kd = (2 * zeta * wc + p - bJ) * g / w0**2
    ki = g * wc**2 * p / w0**2
    return kp, ki, kd


def with_gains(p: PidParams, kp: float, ki: float, kd: float) -> PidParams:
    return replace(p, kp=kp, ki=ki, kd=kd)
