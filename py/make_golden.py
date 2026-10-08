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

from pendel.control import PidParams, PidState, design_gains, pid_step

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
            "design_gains.csv": "\n".join(gains_lines) + "\n"}


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
