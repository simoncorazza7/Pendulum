"""Hardware-boundary unit conversions (Python mirror of src/core/units.*).

SI inside, counts/microsteps/STEP toggles only here. Signs are configuration until 0.8 has
established +x and +theta physically. See docs/model_derivation.md §9.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Calib:
    cpr: int = 2400                  # counts/rev
    c_bottom: float = 0.0            # counts, continuous encoder position of the hanging equilibrium (0.7 fit)
    theta_sign: int = 1              # +1 if increasing counts means increasing theta
    usteps_per_m: float = 13333.3    # microsteps/m (PRIOR, 0.8)
    toggles_per_ustep: float = 2.0   # STEP toggles per microstep (issue 3, 0.8)
    x_sign: int = 1                  # +1 if increasing Axis pos means increasing x
    pos_center: float = 0.0          # microsteps, Axis pos at x = 0


def wrap(a: float) -> float:
    """Angle into (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def theta_from_counts(counts: int, c: Calib) -> float:
    """theta from upright (rad). A count n means the position lies in [n, n+1): use its centre."""
    u = counts + 0.5
    return wrap(c.theta_sign * (u - c.c_bottom - c.cpr / 2) * 2 * math.pi / c.cpr)


def omega_scale(c: Calib) -> float:
    """rad per count with sign: multiply a count-rate estimate (counts/s) by this."""
    return c.theta_sign * 2 * math.pi / c.cpr


def x_from_pos(pos_usteps: int, c: Calib) -> float:
    return c.x_sign * (pos_usteps - c.pos_center) / c.usteps_per_m


def v_from_axis_speed(cur_speed_toggles: int, c: Calib) -> float:
    """Axis::getSpeed() returns curSpeed (STEP toggles/s, internal sign = minus protocol speed)."""
    return c.x_sign * (-cur_speed_toggles) / (c.toggles_per_ustep * c.usteps_per_m)


def axis_accel_arg(a: float, c: Calib) -> int:
    """Argument for Axis::setAcceleration (toggles/s^2, internal sign) for cart acceleration a (m/s^2)."""
    v = -c.x_sign * a * c.usteps_per_m * c.toggles_per_ustep
    return int(math.copysign(math.floor(abs(v) + 0.5), v))   # half away from zero, like C++ lround
