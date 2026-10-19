"""Supervisor state machine (brief §6), Python mirror of src/core/supervisor.*.

IDLE -> ARMED -> ENGAGED -> STOPPING -> IDLE. Pure: params + state + inputs + dt -> output + state.
The runtime enables the driver while the output says so, runs the controller while
`run_controller`, resets the controller when `reset_controller`, and commands `a_brake_cmd` in
STOPPING. `event` names the reason of a transition or refusal this sample (NONE otherwise).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

from .units import wrap


class Mode(IntEnum):
    IDLE = 0
    ARMED = 1
    ENGAGED = 2
    STOPPING = 3


class Reason(IntEnum):
    NONE = 0
    MANUAL_STOP = 1
    LIMIT_SWITCH = 2
    ENCODER_FAULT = 3
    AXIS_NOT_ACCELERATING = 4
    OVERRUN = 5
    THETA_ABORT = 6
    SOFT_LIMIT = 7
    MAX_RUN_TIME = 8
    STOPPED = 9                  # STOPPING finished normally
    STOP_TIMEOUT = 10            # STOPPING gave up waiting for standstill
    ARM_NOT_HANGING = 11
    ARM_NOT_CENTRED = 12
    ARM_MOVING = 13
    ENGAGED_EVENT = 14           # informational: ARMED -> ENGAGED
    ARMED_EVENT = 15             # informational: IDLE -> ARMED
    REMOTE_FAULT = 16            # Stage 2b: the PC's commands stopped arriving (or the link is down)


@dataclass(frozen=True)
class SupervisorParams:
    theta_engage: float = math.radians(4)    # rad
    omega_engage: float = 0.5                # rad/s
    engage_samples: int = 25                 # consecutive samples (50 ms at 2 ms)
    theta_abort: float = math.radians(22)    # rad
    x_min: float = -0.20                     # m, soft limit (PLACEHOLDER until 0.8)
    x_max: float = 0.20                      # m
    a_brake: float = 3.0                     # m/s^2, controlled-stop deceleration
    t_react: float = 0.006                   # s, reaction time added to the braking distance
    brake_margin: float = 0.01               # m
    axis_a: float = 4.5                      # m/s^2, Axis maxAcceleration in SI (its cap A(1-|v|/V))
    axis_v: float = 4.5                      # m/s, Axis maxSpeed in SI
    overrun_streak: int = 3                  # consecutive loop overruns -> abort
    max_run_time: float = 60.0               # s engaged
    v_stopped: float = 0.005                 # m/s, standstill threshold
    stop_timeout: float = 2.0                # s in STOPPING before giving up
    hang_tol: float = math.radians(10)       # rad from hanging, to arm
    omega_hang: float = 0.3                  # rad/s, to arm
    x_center: float = 0.0                    # m
    center_tol: float = 0.02                 # m, to arm
    engage_on_release: bool = False          # engage on the runtime's release detection instead of
                                             # engage_samples still samples (docs/stage1_drift.md)


@dataclass(frozen=True)
class SupervisorInputs:
    theta: float                   # rad from upright, wrapped
    omega: float                   # rad/s
    x: float                       # m
    v: float                       # m/s
    limit_pressed: bool = False
    axis_accelerating: bool = True # Axis is executing ACCELERATE (not its own brake/stop)
    overrun: bool = False          # this loop iteration overran
    encoder_fault: bool = False
    arm: bool = False              # operator request (one sample)
    stop: bool = False             # operator request (one sample)
    released: bool = False         # runtime: free motion away from upright after a still hold
    remote_fault: bool = False     # runtime (Stage 2b): remote commands missing / link down


@dataclass(frozen=True)
class SupervisorState:
    mode: Mode = Mode.IDLE
    engage_count: int = 0
    overrun_count: int = 0
    t_mode: float = 0.0            # s in the current mode


@dataclass(frozen=True)
class SupervisorOutput:
    mode: Mode
    driver_enabled: bool
    run_controller: bool
    reset_controller: bool
    a_brake_cmd: float             # m/s^2, valid in STOPPING (else 0)
    event: Reason


def brake_distance(v: float, a_brake: float, axis_a: float, axis_v: float) -> float:
    """Distance (m) to stop from |v| decelerating at a_brake, limited by the Axis cap A(1-|v|/V)."""
    v = abs(v)
    if v == 0.0:
        return 0.0
    if v >= axis_v:
        return math.inf
    v_star = axis_v * (1 - a_brake / axis_a) if a_brake < axis_a else 0.0   # above it the cap binds
    if v <= v_star:
        return v * v / (2 * a_brake)

    def F(u):  # integral of u / (A (1 - u/V)) du
        return (axis_v / axis_a) * (-u - axis_v * math.log(1 - u / axis_v))

    return v_star * v_star / (2 * a_brake) + F(v) - F(v_star)


def stopping_distance(p: SupervisorParams, v: float) -> float:
    return brake_distance(v, p.a_brake, p.axis_a, p.axis_v) + abs(v) * p.t_react + p.brake_margin


def _common_abort(p: SupervisorParams, i: SupervisorInputs, overruns: int) -> Reason:
    if i.stop:
        return Reason.MANUAL_STOP
    if i.limit_pressed:
        return Reason.LIMIT_SWITCH
    if i.encoder_fault:
        return Reason.ENCODER_FAULT
    if i.remote_fault:
        return Reason.REMOTE_FAULT
    if not i.axis_accelerating:
        return Reason.AXIS_NOT_ACCELERATING
    if overruns >= p.overrun_streak:
        return Reason.OVERRUN
    return Reason.NONE


def _soft_limit(p: SupervisorParams, i: SupervisorInputs) -> bool:
    if i.x >= p.x_max or i.x <= p.x_min:
        return True
    d = stopping_distance(p, i.v)
    return (i.v > 0 and i.x + d >= p.x_max) or (i.v < 0 and i.x - d <= p.x_min)


def _arm_refusal(p: SupervisorParams, i: SupervisorInputs) -> Reason:
    if i.limit_pressed:
        return Reason.LIMIT_SWITCH
    if i.encoder_fault:
        return Reason.ENCODER_FAULT
    if i.remote_fault:
        return Reason.REMOTE_FAULT
    if abs(wrap(i.theta)) < math.pi - p.hang_tol:
        return Reason.ARM_NOT_HANGING
    if abs(i.omega) > p.omega_hang or abs(i.v) > p.v_stopped:
        return Reason.ARM_MOVING
    if abs(i.x - p.x_center) > p.center_tol:
        return Reason.ARM_NOT_CENTRED
    return Reason.NONE


def supervisor_step(p: SupervisorParams, s: SupervisorState, i: SupervisorInputs, dt: float
                    ) -> tuple[SupervisorOutput, SupervisorState]:
    mode, ec, t_mode = s.mode, s.engage_count, s.t_mode + dt
    oc = s.overrun_count + 1 if i.overrun else 0
    event, reset = Reason.NONE, False

    if mode == Mode.IDLE:
        if i.arm:
            event = _arm_refusal(p, i)
            if event == Reason.NONE:
                mode, t_mode, ec, event = Mode.ARMED, 0.0, 0, Reason.ARMED_EVENT
    elif mode == Mode.ARMED:
        event = _common_abort(p, i, oc)
        if event == Reason.NONE and _soft_limit(p, i):
            event = Reason.SOFT_LIMIT
        if event != Reason.NONE:
            mode, t_mode = Mode.STOPPING, 0.0
        elif p.engage_on_release:
            if i.released and abs(i.theta) < p.theta_engage:
                mode, t_mode, reset, event = Mode.ENGAGED, 0.0, True, Reason.ENGAGED_EVENT
        else:
            near = abs(i.theta) < p.theta_engage and abs(i.omega) < p.omega_engage
            ec = ec + 1 if near else 0
            if ec >= p.engage_samples:
                mode, t_mode, reset, event = Mode.ENGAGED, 0.0, True, Reason.ENGAGED_EVENT
    elif mode == Mode.ENGAGED:
        event = _common_abort(p, i, oc)
        if event == Reason.NONE and abs(i.theta) > p.theta_abort:
            event = Reason.THETA_ABORT
        if event == Reason.NONE and _soft_limit(p, i):
            event = Reason.SOFT_LIMIT
        if event == Reason.NONE and s.t_mode + dt > p.max_run_time:
            event = Reason.MAX_RUN_TIME
        if event != Reason.NONE:
            mode, t_mode = Mode.STOPPING, 0.0
    elif mode == Mode.STOPPING:
        if abs(i.v) < p.v_stopped:
            mode, t_mode, event = Mode.IDLE, 0.0, Reason.STOPPED
        elif t_mode > p.stop_timeout:
            mode, t_mode, event = Mode.IDLE, 0.0, Reason.STOP_TIMEOUT

    a_brake = 0.0
    if mode == Mode.STOPPING and i.v != 0.0:
        a_brake = -math.copysign(min(p.a_brake, abs(i.v) / dt), i.v)
    if mode != Mode.ARMED:
        ec = 0
    out = SupervisorOutput(mode=mode, driver_enabled=mode != Mode.IDLE, run_controller=mode == Mode.ENGAGED,
                           reset_controller=reset, a_brake_cmd=a_brake, event=event)
    return out, SupervisorState(mode=mode, engage_count=ec, overrun_count=oc, t_mode=t_mode)
