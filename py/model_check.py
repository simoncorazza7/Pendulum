"""Numerical checks of docs/model_derivation.md (no hardware, no symbolic algebra).

    .venv/Scripts/python py/model_check.py
Each check prints PASS/FAIL; exit code 1 on any failure.
"""

import numpy as np
from scipy.integrate import solve_ivp

G = 9.81
rng = np.random.default_rng(1)
failures = 0


def check(name, ok, detail=""):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    failures += 0 if ok else 1


def gains(w0, bJ, wc, zeta, p):
    kp = G * (1 + (wc**2 + 2 * zeta * wc * p) / w0**2)
    kd = (2 * zeta * wc + p - bJ) * G / w0**2
    ki = G * wc**2 * p / w0**2
    return kp, ki, kd


# 1) Pole placement: closed-loop roots equal the designed poles (random parameters).
worst = 0.0
for _ in range(200):
    w0, bJ = rng.uniform(3, 9), rng.uniform(0, 0.5)
    wc, zeta, p = rng.uniform(1, 4) * w0, rng.uniform(0.4, 1.0), rng.uniform(0, 3)
    kp, ki, kd = gains(w0, bJ, wc, zeta, p)
    k = w0**2 / G
    roots = np.sort_complex(np.roots([1, bJ + k * kd, k * kp - w0**2, k * ki]))
    target = np.sort_complex(np.roots(np.polymul([1, 2 * zeta * wc, wc**2], [1, p])))
    worst = max(worst, np.max(np.abs(roots - target)) / wc)
check("pole placement reproduces designed poles", worst < 1e-6, f"(max rel. error {worst:.1e})")

# 2) Linear model: C = (g/w0^2) th' + x' - g int(th) is conserved for an arbitrary input.
w0 = 6.0


def lin(t, s, a_fun):
    th, thd, xd, I = s
    a = a_fun(t, s)
    return [thd, w0**2 * th - (w0**2 / G) * a, a, th]


a_rand = lambda t, s: 3 * np.sin(7 * t) + 2 * np.cos(2.3 * t) + 40 * s[0]
sol = solve_ivp(lin, (0, 3), [0.05, -0.2, 0.1, 0.0], args=(a_rand,), rtol=1e-10, atol=1e-12, max_step=1e-3)
C = (G / w0**2) * sol.y[1] + sol.y[2] - G * sol.y[3]
check("C conserved under arbitrary input (linear, frictionless)", np.ptp(C) < 1e-6, f"(spread {np.ptp(C):.1e})")

# 3) PID vs PD final cart velocity after a catch.
wc, zeta = 2 * w0, 0.7
th0, thd0, xd0 = 0.04, 0.1, 0.0
for name, p in (("PID", 3.0), ("PD", 0.0)):
    kp, ki, kd = gains(w0, 0.0, wc, zeta, p)
    ctrl = lambda t, s: kp * s[0] + ki * s[3] + kd * s[1]
    sol = solve_ivp(lin, (0, 15), [th0, thd0, xd0, 0.0], args=(ctrl,), rtol=1e-10, atol=1e-12, max_step=1e-3)
    xd_end, I_end = sol.y[2, -1], sol.y[3, -1]
    pred = xd0 + (G / w0**2) * thd0 + G * I_end
    check(f"{name}: final cart velocity = x'0 + (g/w0^2) th'0 + g int(th)", abs(xd_end - pred) < 1e-6,
          f"(v_end {xd_end:+.4f} m/s, int(th) {I_end:+.2e} rad*s)")
    if name == "PID":
        check("PID: int(th) -> 0, so v_end = x'0 + (g/w0^2) th'0", abs(I_end) < 1e-6 and
              abs(xd_end - (xd0 + (G / w0**2) * thd0)) < 1e-6)

# 4) Reference offset delta: steady acceleration g*delta (PID) vs g*delta*Kp/(Kp-g) (PD).
delta = 2 * np.pi / 2400  # one encoder count
for name, p in (("PID", 3.0), ("PD", 0.0)):
    kp, ki, kd = gains(w0, 0.0, wc, zeta, p)
    def rhs(t, s):
        th, thd, xd, I = s
        a = kp * (th - delta) + ki * I + kd * thd
        return [thd, w0**2 * th - (w0**2 / G) * a, a, th - delta]
    sol = solve_ivp(rhs, (0, 20), [0.0, 0.0, 0.0, 0.0], rtol=1e-10, atol=1e-13, max_step=1e-3)
    th_end = sol.y[0, -1]
    a_end = kp * (th_end - delta) + ki * sol.y[3, -1] + kd * sol.y[1, -1]
    pred = G * delta * (1 if p > 0 else kp / (kp - G))
    check(f"{name}: steady accel from 1-count offset", abs(a_end - pred) < 1e-6 * max(1, pred) + 1e-9,
          f"(a = {a_end:.5f} m/s^2, predicted {pred:.5f}; g*delta = {G * delta:.5f})")

# 5) Nonlinear model reduces to the linear one for small angles, and the hanging input gain flips sign.
def nonlin(t, s, a):
    th, thd = s
    return [thd, w0**2 * np.sin(th) - (w0**2 / G) * a * np.cos(th)]
for th_eq, label, sign in ((0.0, "upright", -1), (np.pi, "hanging", +1)):
    a = 0.5
    d = nonlin(0, [th_eq, 0], a)[1] - nonlin(0, [th_eq, 0], 0)[1]
    check(f"{label}: input gain sign {'+' if sign > 0 else '-'}w0^2/g", abs(d - sign * (w0**2 / G) * a) < 1e-9,
          f"(d(theta'')/da = {d / a:+.4f})")

raise SystemExit(1 if failures else 0)
