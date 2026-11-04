"""Golden vectors for the controller core: the Python reference produces them, the C++ unit test
(tests/unit/test_pid.cpp) must reproduce them to floating-point tolerance.

    .venv/Scripts/python py/make_golden.py          write tests/golden/*.csv
    .venv/Scripts/python py/make_golden.py --check  regenerate in memory, compare with the files

Numbers are written with repr() (shortest exact round-trip), so the C++ side reads exactly the
doubles Python used. Regenerate deliberately whenever control.py changes, and say so in the commit.
"""

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from dataclasses import fields, replace

from pendel import units
from pendel.encsim import edge_crossings, encode
from pendel.estimators import BdLp1, BdLp2
from pendel.simhw import SimHw, SimHwParams
from pendel.swingfit import fit_offset_linear
from pendel.control import PidParams, PidState, cascade_step, design_gains, pid_step
from pendel.supervisor import (Mode, SupervisorInputs, SupervisorParams, SupervisorState, brake_distance,
                               supervisor_step)

GOLDEN = pathlib.Path(__file__).resolve().parent.parent / "tests" / "golden"
PARAM_FIELDS = ("kp", "ki", "kd", "a_max", "v_max", "v_taper", "i_max", "i_delay", "theta_ref", "vel_fc")
Q = 2 * math.pi / 2400   # rad per count


def cases():
    """(name, params, theta[], v_cart[], dt[]) - deterministic."""
    rng = np.random.default_rng(20261008)
    n = 2000
    t = np.arange(n) * 0.002

    # 1) PID catch-like transient: quantised decaying oscillation + noise; cart speed sweeps past v_max
    theta = 0.14 * np.exp(-3 * t) * np.cos(9 * t) + rng.normal(0, 0.3 * Q, n)
    theta = np.floor(theta / Q + 0.37) * Q
    v = 0.95 * np.sin(1.3 * t)
    dt = np.full(n, 0.002)
    dt[700:720] = 0.0021                       # a few long periods (overruns)
    kp, ki, kd = design_gains(2 * math.pi, 0.1, 4 * math.pi, 0.7, math.pi)
    yield "pid_catch", PidParams(kp=kp, ki=ki, kd=kd, a_max=5.0, v_max=0.8, i_max=1.0, i_delay=0.15,
                                 theta_ref=0.001, vel_fc=30.0), theta, v, dt

    # 2) PD with a small a_max: frequent saturation, cart speed ramps through +-v_max
    kp, ki, kd = design_gains(2 * math.pi, 0.1, 4 * math.pi, 0.7, 0.0)
    theta2 = 0.05 * np.sin(4 * t) + rng.normal(0, 0.5 * Q, n)
    v2 = np.linspace(-1.0, 1.0, n)
    yield "pd_saturating", PidParams(kp=kp, ki=0.0, kd=kd, a_max=1.0, v_max=0.8, v_taper=0.5, vel_fc=20.0), \
        theta2, v2, np.full(n, 0.002)

    # 3) Windup: a long constant error with a small a_max, then back to 0 (integrator hold + clamp)
    theta3 = np.where(t < 2.0, 0.3, 0.0) + rng.normal(0, 0.2 * Q, n)
    yield "pid_windup", PidParams(kp=60.0, ki=150.0, kd=5.0, a_max=2.0, v_max=0.8, i_max=1.5, vel_fc=40.0), \
        theta3, np.zeros(n), np.full(n, 0.002)


SUP_PARAM_FIELDS = [f.name for f in fields(SupervisorParams)]
SUP_IN_FIELDS = ["theta", "omega", "x", "v", "limit_pressed", "axis_accelerating", "overrun", "encoder_fault",
                 "arm", "stop", "released"]


def supervisor_cases():
    """(name, params, scenario); scenario(k, previous output) -> SupervisorInputs, or None to end.
    Scenarios react to the supervisor's mode, as the real runtime and plant would."""
    base = SupervisorParams()
    hang = math.pi - 0.01

    def lift_then(engaged_fn, stopping_v_decay=True, release_k=None):
        st = {"v": 0.0, "x": 0.0, "t_eng": None}

        def scen(k, out):
            t = k * 0.002
            mode = out.mode if out else Mode.IDLE
            if k < 10:
                return SupervisorInputs(theta=hang, omega=0.0, x=0.0, v=0.0)
            if k == 10:
                return SupervisorInputs(theta=1.0, omega=0.0, x=0.0, v=0.0, arm=True)    # refused: not hanging
            if k == 15:
                return SupervisorInputs(theta=hang, omega=0.0, x=0.05, v=0.0, arm=True)  # refused: not centred
            if k == 20:
                return SupervisorInputs(theta=hang, omega=0.0, x=0.0, v=0.0, arm=True)   # armed
            if mode == Mode.IDLE:
                return None if k > 20 else SupervisorInputs(theta=hang, omega=0.0, x=0.0, v=0.0)
            if mode == Mode.ARMED:
                if k < 220:                                                     # lifting by hand, 0.4 s
                    th = hang - (k - 20) / 200 * (hang - 0.02)
                    return SupervisorInputs(theta=th, omega=-7.8, x=0.0, v=0.0)
                rel = release_k is not None and k >= release_k                  # held still until release_k
                return SupervisorInputs(theta=0.02 + 0.005 * math.sin(40 * t), omega=0.1, x=0.0, v=0.0, released=rel)
            if mode == Mode.ENGAGED:
                if st["t_eng"] is None:
                    st["t_eng"] = t
                return engaged_fn(t - st["t_eng"], st)
            if mode == Mode.STOPPING:
                if stopping_v_decay:
                    st["v"] = math.copysign(max(0.0, abs(st["v"]) - 3.0 * 0.002), st["v"])
                st["x"] += st["v"] * 0.002
                return SupervisorInputs(theta=0.05, omega=0.0, x=st["x"], v=st["v"])
            return None
        return scen

    def drift(te, st):                       # cart accelerates at 0.3 m/s^2 toward +x -> soft limit
        st["v"] = 0.3 * te
        st["x"] = 0.15 * te * te
        return SupervisorInputs(theta=0.01 * math.sin(30 * te), omega=0.3 * math.cos(30 * te), x=st["x"], v=st["v"])

    def fall(te, st):                        # pendulum falls -> theta abort
        return SupervisorInputs(theta=0.02 * math.exp(6 * te), omega=0.12 * math.exp(6 * te), x=0.0, v=0.0)

    def overrun(te, st):
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=0.0, overrun=(te > 0.1 and int(te / 0.002) % 7 != 0))

    def limit(te, st):
        st["v"] = -0.05
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=-0.05, limit_pressed=te > 0.2)

    def axis(te, st):
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=0.0, axis_accelerating=te < 0.3)

    def long_run(te, st):
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=0.0)

    def manual(te, st):
        st["v"] = 0.02
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=0.02, stop=abs(te - 0.25) < 1e-9)

    def stuck(te, st):                       # STOPPING never reaches standstill -> timeout
        st["v"] = 0.1
        return SupervisorInputs(theta=0.0, omega=0.0, x=0.0, v=0.1, stop=te > 0.1)

    yield "soft_limit", base, lift_then(drift)
    yield "theta_abort", base, lift_then(fall)
    yield "overrun", base, lift_then(overrun)
    yield "limit_switch", base, lift_then(limit)
    yield "axis_left_accel", base, lift_then(axis)
    yield "max_run_time", replace(base, max_run_time=0.5), lift_then(long_run)
    yield "manual_stop", base, lift_then(manual)
    yield "stop_timeout", replace(base, stop_timeout=0.3), lift_then(stuck, stopping_v_decay=False)
    yield "engage_on_release", replace(base, engage_on_release=True), lift_then(fall, release_k=600)


def _fmt(x):
    return repr(float(x)) if isinstance(x, float) else str(int(x))


def render_supervisor() -> dict[str, str]:
    NL = chr(10)
    params_lines = ["case," + ",".join(SUP_PARAM_FIELDS)]
    lines = ["case,k,dt," + ",".join(SUP_IN_FIELDS) + ",mode,driver,run,reset,a_brake,event"]
    for name, p, scen in supervisor_cases():
        params_lines.append(name + "," + ",".join(repr(float(getattr(p, f))) for f in SUP_PARAM_FIELDS))
        s, out = SupervisorState(), None
        for k in range(20000):
            i = scen(k, out)
            if i is None:
                break
            out, s = supervisor_step(p, s, i, 0.002)
            vals = [_fmt(getattr(i, f)) for f in SUP_IN_FIELDS]
            lines.append(f"{name},{k},0.002," + ",".join(vals) +
                         f",{int(out.mode)},{int(out.driver_enabled)},{int(out.run_controller)},"
                         f"{int(out.reset_controller)},{out.a_brake_cmd!r},{int(out.event)}")
    bd = ["v,a_brake,axis_a,axis_v,distance"]
    for v, ab, A, V in ((0.5, 3.0, 4.5, 4.5), (2.0, 3.0, 4.5, 4.5), (1.0, 6.0, 4.5, 4.5), (0.3, 1.0, 10.0, 1.5),
                        (-1.2, 2.0, 3.0, 2.0)):
        bd.append(",".join(repr(float(x)) for x in (v, ab, A, V, brake_distance(v, ab, A, V))))
    un = ["counts,theta_sign,c_bottom,pos,x_sign,cur_speed,a,theta,x,v,axis_arg"]
    for counts, ts_, cb, pos, xs, cs, a in ((1200, 1, 0.0, 6000, 1, -26667, 2.5), (-1201, 1, 0.37, -100, -1, 500, -0.7),
                                          (37, -1, 1236.6, 0, 1, 0, 1e-5), (3600, -1, -0.5, 13333, -1, 13333, -4.2)):
        c = units.Calib(c_bottom=cb, theta_sign=ts_, x_sign=xs)
        un.append(",".join([str(counts), str(ts_), repr(cb), str(pos), str(xs), str(cs), repr(a),
                            repr(units.theta_from_counts(counts, c)), repr(units.x_from_pos(pos, c)),
                            repr(units.v_from_axis_speed(cs, c)), str(units.axis_accel_arg(a, c))]))
    return {"supervisor_params.csv": NL.join(params_lines) + NL,
            "supervisor_golden.csv": NL.join(lines) + NL,
            "brake_distance.csv": NL.join(bd) + NL,
            "units.csv": NL.join(un) + NL}


def render_swing() -> dict[str, str]:
    """Analytic decaying swing about c = 123.37 counts with 20 us timestamp jitter; edges and the
    linear offset fit (w, sigma known) that the Pi's start-up calibration (core::fitOffset) must match."""
    NL = chr(10)
    w, sig, c_true = 6.15, 0.04, 123.37
    t = np.arange(0, 5.0, 2e-5)
    u = c_true + 25.0 * np.exp(-sig * t) * np.cos(w * t + 0.3)
    te, ue = edge_crossings(t, u)
    te = te + np.random.default_rng(7).normal(0, 20e-6, len(te))
    c, se = fit_offset_linear(te, ue, w, sig)
    edges = ["t,u"] + [f"{a!r},{b!r}" for a, b in zip(te.tolist(), ue.tolist())]
    fit = ["w,sigma,c,se", f"{w!r},{sig!r},{c!r},{se!r}"]
    return {"swing_edges.csv": NL.join(edges) + NL, "swing_fit.csv": NL.join(fit) + NL}


def simhw_cases():
    """(name, params, theta0, steps, action(k) -> (enable or None, halt, axis_arg or None))."""
    dt = 0.002
    base = SimHwParams(coulomb=0.01)

    def drive_halt_coast(k):
        t = k * dt
        enable = True if k == 50 else (False if k == 710 else None)
        arg = None
        if 50 <= k < 600:
            arg = units.axis_accel_arg(1.5 * math.sin(2 * math.pi * 1.3 * t), base.truth)
        elif 600 <= k < 700:
            arg = units.axis_accel_arg(-3.0, base.truth)
        return enable, k == 700, arg

    def crash_left(k):
        return (True if k == 0 else None), False, units.axis_accel_arg(4.0, base.truth)

    rev = replace(base, truth=replace(base.truth, theta_sign=-1, x_sign=-1, c_bottom=-7.6))

    def reversed_signs(k):
        t = k * dt
        return (True if k == 0 else None), False, units.axis_accel_arg(2.0 * math.sin(2 * math.pi * 2.0 * t), rev.truth)

    yield "drive_halt_coast", base, math.pi + 0.08, 1500, drive_halt_coast
    yield "crash_left", base, math.pi - 0.03, 600, crash_left
    yield "reversed_signs", rev, -math.pi + 0.05, 800, reversed_signs


def render_simhw() -> dict[str, str]:
    NL = chr(10)
    rows = ["case,k,enable,halt,cmd_valid,cmd_arg,t_ns,counts,n_edges,overflow,pos,speed,lim_r,lim_l,axis_acc,"
            "driver,theta,thetadot,x,v"]
    edges = ["case,k,i,ts_ns,count_after"]
    params = ["case,w0,bJ,coulomb,c_bottom,theta_sign,x_sign,theta0"]
    for name, p, th0, steps, action in simhw_cases():
        params.append(f"{name},{p.w0!r},{p.bJ!r},{p.coulomb!r},{p.truth.c_bottom!r},{p.truth.theta_sign},"
                      f"{p.truth.x_sign},{th0!r}")
        hw = SimHw(p, theta0=th0)
        for k in range(steps):
            smp = hw.read()
            enable, halt, arg = action(k)
            rows.append(f"{name},{k},{-1 if enable is None else int(enable)},{int(halt)},{int(arg is not None)},"
                        f"{0 if arg is None else arg},{smp['t_ns']},{smp['counts']},{len(smp['edges'])},"
                        f"{int(smp['edges_overflow'])},{smp['pos_usteps']},{smp['speed_toggles']},"
                        f"{int(smp['limit_right'])},{int(smp['limit_left'])},{int(smp['axis_accelerating'])},"
                        f"{int(smp['driver_enabled'])},{hw.th!r},{hw.thd!r},{hw.x!r},{hw.v!r}")
            for i, (ts, ca) in enumerate(smp["edges"]):
                edges.append(f"{name},{k},{i},{ts},{ca}")
            if enable is not None:
                hw.enable_driver(enable)
            if halt:
                hw.halt()
            if arg is not None:
                hw.command_accel(arg)
            hw.advance(0.002)
    return {"simhw_params.csv": NL.join(params) + NL, "simhw_golden.csv": NL.join(rows) + NL,
            "simhw_edges.csv": NL.join(edges) + NL}


def render_estimators() -> dict[str, str]:
    """Counts-based velocity estimators and the controller's external-velocity path."""
    NL = chr(10)
    ts = 0.002
    t = np.arange(0, 4.0, 5e-6)
    q = 2 * math.pi / 2400
    theta = np.where(t < 2.0, math.radians(10) * np.sin(2 * math.pi * 3 * t),
                     (5 - 0.37) * q + 0.2 * q * np.sin(2 * math.pi * 31 * t))
    enc = encode(t, theta, ts=ts)
    e1, e2, e3 = BdLp1(ts, 30.0), BdLp2(ts, 30.0), BdLp2(ts, 50.0)
    est = ["k,counts,bdlp1_30,bdlp2_30,bdlp2_50"]
    pid_lines = ["k,theta,v_cart,vel_est,a_cmd,u,integral,saturated"]
    kp, ki, kd = design_gains(2 * math.pi, 0.1, 4 * math.pi, 0.7, math.pi)
    pp = PidParams(kp=kp, ki=ki, kd=kd, a_max=5.0, v_max=0.8, i_max=1.0, i_delay=0.15, theta_ref=0.001)
    st = PidState()
    for k, c in enumerate(enc["count"].tolist()):
        v1, v2, v3 = e1.update(0, c, []), e2.update(0, c, []), e3.update(0, c, [])
        est.append(f"{k},{c},{v1!r},{v2!r},{v3!r}")
        th = c * q
        vc = 0.9 * math.sin(1.7 * k * ts)
        a, st, dbg = pid_step(pp, st, th, vc, ts, vel_est=v2)
        pid_lines.append(f"{k},{th!r},{vc!r},{v2!r},{a!r},{dbg['u']!r},{st.integral!r},{int(dbg['saturated'])}")
    return {"estimator_golden.csv": NL.join(est) + NL, "pid_ext_golden.csv": NL.join(pid_lines) + NL}


CASCADE_FIELDS = PARAM_FIELDS + ("cart_kx", "cart_kv", "cart_theta_max", "cart_ki", "cart_i_max")


def render_cascade() -> dict[str, str]:
    """Stage 3 cascade (control.cascade_step): the outer loop's reference, inside and beyond its clamp."""
    NL = chr(10)
    kp, ki, kd = design_gains(4.97, 0.054, 2 * 4.97, 0.7, 0.5 * 4.97)
    base = PidParams(kp=kp, ki=ki, kd=kd, a_max=2.0, v_max=0.3, i_max=1.0, i_delay=0.0, theta_ref=-0.0007, vel_fc=14.0)
    defs = [("rig", replace(base, cart_kx=0.41, cart_kv=1.0, cart_theta_max=math.radians(4), cart_ki=0.05,
                            cart_i_max=math.radians(2))),
            ("clamped", replace(base, cart_kx=40.0, cart_kv=8.0, cart_theta_max=math.radians(2), cart_ki=30.0,
                                cart_i_max=math.radians(1))),
            ("off", base)]
    cases_lines = ["case," + ",".join(CASCADE_FIELDS)]
    rows = ["case,k,theta,x,v,dt,theta_ref,a_cmd,u,integral,vel_f,saturated,x_int"]
    ts = 0.002
    for name, p in defs:
        cases_lines.append(name + "," + ",".join(repr(float(getattr(p, f))) for f in CASCADE_FIELDS))
        s = PidState()
        for k in range(3000):
            t = k * ts
            th = math.floor((0.02 * math.sin(2 * math.pi * 0.9 * t) + 0.004 * math.sin(2 * math.pi * 7 * t)) / Q) * Q
            x = 0.12 * math.sin(2 * math.pi * 0.15 * t)
            v = 0.12 * 2 * math.pi * 0.15 * math.cos(2 * math.pi * 0.15 * t) + 0.25 * math.sin(2 * math.pi * 0.5 * t)
            a, s, dbg = cascade_step(p, s, th, x, v, ts)
            rows.append(f"{name},{k},{th!r},{x!r},{v!r},{ts!r},{dbg['theta_ref']!r},{a!r},{dbg['u']!r},"
                        f"{s.integral!r},{s.vel_f!r},{int(dbg['saturated'])},{s.x_int!r}")
    return {"cascade_cases.csv": NL.join(cases_lines) + NL, "cascade_golden.csv": NL.join(rows) + NL}


def render() -> dict[str, str]:
    params_lines = ["case," + ",".join(PARAM_FIELDS)]
    golden_lines = ["case,k,theta,v_cart,dt,a_cmd,u,integral,vel_f,saturated"]
    for name, p, theta, v, dt in cases():
        params_lines.append(name + "," + ",".join(repr(float(getattr(p, f))) for f in PARAM_FIELDS))
        s = PidState()
        for k in range(len(theta)):
            th, vc, d = float(theta[k]), float(v[k]), float(dt[k])
            a, s, dbg = pid_step(p, s, th, vc, d)
            golden_lines.append(f"{name},{k},{th!r},{vc!r},{d!r},{a!r},{dbg['u']!r},{s.integral!r},"
                                f"{s.vel_f!r},{int(dbg['saturated'])}")
    gains_lines = ["w0,bJ,wc,zeta,p,kp,ki,kd"]
    for w0, bJ, wc, zeta, p in ((2 * math.pi, 0.1, 4 * math.pi, 0.7, 0.0), (6.0, 0.05, 15.0, 0.8, 3.0),
                                (4.43, 0.2, 9.0, 0.6, 1.5)):
        kp, ki, kd = design_gains(w0, bJ, wc, zeta, p)
        gains_lines.append(",".join(repr(float(x)) for x in (w0, bJ, wc, zeta, p, kp, ki, kd)))
    return {"pid_cases.csv": "\n".join(params_lines) + "\n",
            "pid_golden.csv": "\n".join(golden_lines) + "\n",
            "design_gains.csv": "\n".join(gains_lines) + "\n",
            **render_supervisor(),
            **render_swing(),
            **render_simhw(),
            **render_estimators(),
            **render_cascade()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    files = render()
    if args.check:
        bad = [n for n, txt in files.items() if (GOLDEN / n).read_text(encoding="utf-8") != txt]
        print("golden files up to date" if not bad else f"MISMATCH: {', '.join(bad)} (rerun without --check)")
        raise SystemExit(1 if bad else 0)
    GOLDEN.mkdir(parents=True, exist_ok=True)
    for n, txt in files.items():
        (GOLDEN / n).write_text(txt, encoding="utf-8", newline="\n")
        print(f"wrote {GOLDEN / n} ({txt.count(chr(10)) - 1} rows)")


if __name__ == "__main__":
    main()
