"""Fixed-gain observer for angle, rate and upright offset (simulation prototype, PC only; docs/offset_estimator.md).

Result: with a fixed memory (tau_d 1 s) its offset estimate swings by about +-2 counts within one balance on the
2026-11-02 pivot (offset.replay.replay on the logs), so the estimate that is carried between catches comes from
offset.identity (long memory, quiet stretches only). Kept for the open question whether a model-based rate estimate
(the known cart acceleration as input) beats BD+LP2's lag without passing more of the ~28 Hz drive-side mode.

The balance loop regulates the measured angle theta_m = theta_true - delta, where delta is the error of the
upright reference (start-up c_bottom fit, encoder eccentricity, mount). Holding theta_m at 0 leans the real
pendulum by delta, and holding that lean takes a steady cart acceleration g delta: the drift that ran the Stage 3
balances of 2026-11-04 into the soft limit. In the measured frame the offset acts exactly like a constant
disturbance acceleration d = w0^2 delta:

    theta_m'' = w0^2 sin(theta_m) - (w0^2/g) a cos(theta_m) - (b/J) theta_m' + d,        d' = 0

(a real constant torque on the rod, e.g. cable drag, looks the same and needs the same correction). On a stepper rig
the cart acceleration is known: a = (v_k - v_{k-1}) / Ts from the step generator's speed, exact while no steps are
lost. With only theta_m measured, (theta_m, theta_m', d) is observable: in these coordinates the observability matrix
of the linearised model has determinant 1 (d is the part of the angle's acceleration that the model and the known
cart acceleration leave unexplained); in the true-angle form (theta_true, theta', delta) with y = theta_true - delta
it is w0^2, because there gravity is what ties a steady lean to a steady cart acceleration.

The observer predicts one tick with that model and the measured acceleration, then corrects with the encoder angle
(fixed gains, "current estimator" form). Its gains come from pole placement on the linearised, exactly discretised
model: a pair at f_v Hz (damping zeta_v) for angle and rate, and one real pole at 1/tau_d for the offset.
The offset part learns only while the pendulum balances near upright (learn_gate); otherwise d_hat is held, which
is what carries the estimate from one catch to the next. delta_hat = d_hat / w0^2, clamped to +-delta_max.

The controller uses it as a reference trim: theta_ref = theta_ref_config - delta_hat (+ the cart loop's lean), so
that theta_true = theta_m + delta settles at 0 and the cart needs no steady acceleration.

Pure functions (params + state + measurement -> state), no I/O, like pendel.control: a C++ port would take the three
gains from here and be golden-tested against this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.linalg import expm
from scipy.signal import place_poles

COUNT = 2 * math.pi / 2400       # rad per encoder count (encoder.counts_per_rev 2400)


@dataclass(frozen=True)
class ObserverParams:
    w0: float                                  # rad/s, hanging small-angle frequency (plant.w0)
    bJ: float                                  # 1/s, viscous pivot friction b/J (plant.bJ)
    ts: float = 0.002                          # s, sample period (loop.ts)
    g: float = 9.81                            # m/s^2
    f_v: float = 6.0                           # Hz, natural frequency of the angle/rate error poles
    zeta_v: float = 0.7                        # damping of that pole pair
    tau_d: float = 1.0                         # s, time constant of the offset estimate while it learns
    t_settle: float = 1.0                      # s after engage before learning (hand on the rod, catch transient)
    theta_learn: float = math.radians(3.0)     # rad, learns only while |theta_hat| is below this
    delta_max: float = 5 * COUNT               # rad, clamp of delta_hat (5 counts)
    substeps: int = 2                          # semi-implicit Euler steps per tick in the prediction


@dataclass(frozen=True)
class ObserverState:
    theta: float = 0.0          # rad, estimated measured-frame angle theta_m
    omega: float = 0.0          # rad/s, its rate
    d: float = 0.0              # rad/s^2, disturbance acceleration w0^2 delta (kept across restarts)
    v_prev: float = 0.0         # m/s, cart speed at the previous tick
    running: bool = False       # False: the next step (re)starts from the measurement, keeping d


def linear_model(p: ObserverParams) -> tuple[np.ndarray, np.ndarray]:
    """Exact ZOH discretisation (Phi, Gamma) of the linearised model, state (theta, omega, d), input a."""
    A = np.array([[0.0, 1.0, 0.0], [p.w0 ** 2, -p.bJ, 1.0], [0.0, 0.0, 0.0]])
    B = np.array([[0.0], [-p.w0 ** 2 / p.g], [0.0]])
    M = np.zeros((4, 4))
    M[:3, :3], M[:3, 3:] = A, B
    E = expm(M * p.ts)
    return E[:3, :3], E[:3, 3]


def observer_poles(p: ObserverParams) -> np.ndarray:
    """Desired z-plane poles of the estimation error (learning)."""
    wn = 2 * math.pi * p.f_v
    s = np.array([complex(-p.zeta_v * wn, wn * math.sqrt(max(0.0, 1 - p.zeta_v ** 2))),
                  complex(-p.zeta_v * wn, -wn * math.sqrt(max(0.0, 1 - p.zeta_v ** 2))),
                  complex(-1.0 / p.tau_d, 0.0)])
    if p.zeta_v >= 1.0:                       # real pair: wn*(-zeta +- sqrt(zeta^2 - 1))
        r = wn * math.sqrt(p.zeta_v ** 2 - 1)
        s[0], s[1] = complex(-p.zeta_v * wn + r, 0), complex(-p.zeta_v * wn - r, 0)
    return np.exp(s * p.ts)


def gains(p: ObserverParams) -> np.ndarray:
    """K (3,) for x+ = x- + K (y - theta-): eig((I - K C) Phi) = observer_poles. Dual pole placement."""
    Phi, _ = linear_model(p)
    C = np.array([[1.0, 0.0, 0.0]])
    poles = observer_poles(p)
    if np.all(np.abs(poles.imag) < 1e-12):
        poles = poles.real.copy()
        if abs(poles[0] - poles[1]) < 1e-9:          # one output: place_poles needs distinct real poles
            poles[1] *= 1.0 - 1e-6
    res = place_poles(Phi.T, (C @ Phi).T, poles)
    return res.gain_matrix.T[:, 0].copy()


def error_matrix(p: ObserverParams, K: np.ndarray, learning: bool = True) -> np.ndarray:
    """Estimation-error transition (I - K C) Phi; with learning False the offset gain is 0 (d held)."""
    Phi, _ = linear_model(p)
    k = K.copy()
    if not learning:
        k[2] = 0.0
    return (np.eye(3) - np.outer(k, [1.0, 0.0, 0.0])) @ Phi


def restart(s: ObserverState, theta_m: float, v: float, omega: float = 0.0) -> ObserverState:
    """Start tracking from a measurement (e.g. at engage), keeping the offset estimate d."""
    return ObserverState(theta=theta_m, omega=omega, d=s.d, v_prev=v, running=True)


def step(p: ObserverParams, K: np.ndarray, s: ObserverState, theta_m: float, v: float,
         learn: bool) -> ObserverState:
    """One tick: predict with the cart acceleration of the last interval, correct with the measured angle.

    theta_m  measured angle from upright (rad, the controller's theta)
    v        cart speed now (m/s), from the step generator
    learn    update the offset estimate this tick (see learn_gate)
    """
    if not s.running:
        return restart(s, theta_m, v)
    a = (v - s.v_prev) / p.ts
    th, om = s.theta, s.omega
    h = p.ts / p.substeps
    k_in = p.w0 ** 2 / p.g
    for _ in range(p.substeps):
        acc = p.w0 ** 2 * math.sin(th) - k_in * a * math.cos(th) - p.bJ * om + s.d
        om += acc * h
        th += om * h
    r = theta_m - th
    th += K[0] * r
    om += K[1] * r
    d = s.d
    if learn:
        dmax = p.w0 ** 2 * p.delta_max
        d = min(dmax, max(-dmax, d + K[2] * r))
    return ObserverState(theta=th, omega=om, d=d, v_prev=v, running=True)


def delta_hat(p: ObserverParams, s: ObserverState) -> float:
    """Offset estimate (rad): theta_true ~ theta_m + delta_hat."""
    return s.d / p.w0 ** 2


def learn_gate(p: ObserverParams, t_engaged: float, s: ObserverState) -> bool:
    """Learn only while balancing quietly: t_settle after engage, and near upright."""
    return t_engaged >= p.t_settle and abs(s.theta) < p.theta_learn


def with_delta(s: ObserverState, p: ObserverParams, delta: float) -> ObserverState:
    """A state carrying the offset delta (rad), e.g. a prior from the previous catch or session."""
    return replace(s, d=p.w0 ** 2 * delta)
