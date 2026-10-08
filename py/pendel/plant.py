"""Plant, actuator and sensor models for simulation (Stage 0.12).

PLACEHOLDER defaults: replace with measured values from 0.8-0.10 before designing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81  # m/s^2


@dataclass(frozen=True)
class PlantParams:
    w0: float = 2 * math.pi       # rad/s, hanging small-angle frequency (PLACEHOLDER: 1.0 s period)
    bJ: float = 0.1               # 1/s, viscous pivot friction b/J (PLACEHOLDER)
    coulomb: float = 0.0          # rad/s^2, Coulomb friction acceleration (PLACEHOLDER)
    g: float = G


@dataclass(frozen=True)
class ActuatorParams:
    """The Axis ACCELERATE path: realised accel = command capped at A*(1 - |v|/V), applied after a delay."""
    axis_a: float = 4.5           # m/s^2, Axis maxAcceleration in SI (PLACEHOLDER: 120000 toggles/s^2)
    axis_v: float = 4.5           # m/s, Axis maxSpeed in SI (PLACEHOLDER: 120000 toggles/s)
    compute_delay: float = 0.0005 # s, sample -> command reaches the step thread (PLACEHOLDER)


@dataclass(frozen=True)
class SensorParams:
    cpr: int = 2400               # counts/rev
    phase: float = 0.37           # counts, encoder phase (where the count edges fall)
    ref_error: float = 0.0        # rad, delta: the controller's zero is at true theta = delta


def pendulum_accel(theta: float, thetadot: float, a: float, p: PlantParams) -> float:
    """theta'' of the nonlinear pendulum (theta from upright)."""
    k = p.w0**2 / p.g
    acc = p.w0**2 * math.sin(theta) - k * a * math.cos(theta) - p.bJ * thetadot
    if p.coulomb and thetadot != 0.0:
        acc -= math.copysign(p.coulomb, thetadot)
    return acc


def realised_accel(a_cmd: float, v: float, ap: ActuatorParams) -> float:
    cap = ap.axis_a * max(0.0, 1.0 - abs(v) / ap.axis_v)
    return min(cap, max(-cap, a_cmd))


def measure(theta: float, sp: SensorParams) -> tuple[int, float]:
    """Encoder counts (relative to upright) and the measured angle the controller sees."""
    q = 2 * math.pi / sp.cpr
    counts = math.floor(theta / q + sp.phase)
    theta_meas = (counts - sp.phase + 0.5) * q - sp.ref_error   # centred quantisation, minus offset
    return counts, theta_meas
