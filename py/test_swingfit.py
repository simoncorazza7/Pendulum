"""Validate the swing fits (py/pendel/swingfit.py) on synthetic hanging-pendulum swings.

    .venv/Scripts/python py/test_swingfit.py
Nonlinear pendulum with viscous and Coulomb friction, known c_bottom; edges from the synthetic
encoder. Prints PASS/FAIL per check; exit code 1 on failure.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np
from scipy.integrate import solve_ivp

from pendel.encsim import edge_crossings
from pendel.swingfit import envelope_models, fit_damped, fit_offset_linear, omega0_from_windows, windowed

CPR = 2400
failures = 0


def check(name, ok, detail=""):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    failures += 0 if ok else 1


def swing(phi0_deg, t_end, w0, beta, gamma, c_bottom, sign=1):
    """Hanging pendulum phi'' = -w0^2 sin phi - beta phi' - gamma sgn(phi'). Returns edges (t, u)."""
    def f(t, s):
        return [s[1], -w0**2 * math.sin(s[0]) - beta * s[1] - gamma * math.tanh(s[1] / 1e-4)]
    t = np.arange(0, t_end, 2e-5)
    sol = solve_ivp(f, (0, t_end), [math.radians(phi0_deg), 0.0], t_eval=t, rtol=1e-10, atol=1e-12,
                    max_step=1e-3)
    u = c_bottom + sign * sol.y[0] * CPR / (2 * math.pi)
    return edge_crossings(t, u)


W0, BETA, GAMMA, CB = 2 * math.pi * 0.98, 0.08, 0.02, 123.37

# 0.7: small swing -> c_bottom to a fraction of a count
t, u = swing(4.0, 20.0, W0, BETA, GAMMA, CB)
g = fit_damped(t, u)
check("small swing: c_bottom from the full damped fit", abs(g.c - CB) < 0.05,
      f"(c = {g.c:.3f} +- {g.se['c']:.3f}, true {CB}; {g.n} edges, residual {g.rms:.3f} counts)")
check("small swing: damped frequency", abs(g.w - math.sqrt(W0**2 - (BETA / 2) ** 2)) / W0 < 2e-3,
      f"(w = {g.w:.4f} rad/s, w0 = {W0:.4f})")
c_lin, se_lin = fit_offset_linear(t, u, g.w, g.sigma)
check("small swing: linear offset fit with known w, sigma (Pi start-up method)", abs(c_lin - CB) < 0.05,
      f"(c = {c_lin:.3f} +- {se_lin:.3f})")
c_short, _ = fit_offset_linear(t[t < 3.0], u[t < 3.0], g.w, g.sigma)
check("small swing: 3 s are enough for the linear offset fit", abs(c_short - CB) < 0.1, f"(c = {c_short:.3f})")
t2, u2 = swing(4.0, 20.0, W0, BETA, GAMMA, CB, sign=-1)
check("small swing: works with reversed count direction", abs(fit_damped(t2, u2).c - CB) < 0.05)
# "hanging still" is biased by stiction: where does the swing end?
t3, u3 = swing(4.0, 120.0, W0, BETA, GAMMA, CB)
print(f"      (for comparison: after 120 s the last edge is at u = {u3[-1]:.0f}, stiction band "
      f"+-{GAMMA / W0**2 * CPR / (2 * math.pi):.2f} counts)")

# 0.10: large swing -> w0 (amplitude dependence) and damping type
t, u = swing(30.0, 60.0, W0, BETA, GAMMA, CB)
win = windowed(t, u)
w0_est, slope = omega0_from_windows(win["A"], win["w"])
check("large swing: w0 extrapolated to zero amplitude", abs(w0_est - math.sqrt(W0**2 - (BETA / 2) ** 2)) / W0 < 2e-3,
      f"(w0 = {w0_est:.4f}, A^2 coefficient {slope:.4f}, rigid pendulum 1/16 = 0.0625)")
env = envelope_models(win["t"], win["A"] * 2 * math.pi / CPR)
k_true = 2 * GAMMA / (math.pi * W0)
print(f"      envelope rms: exp {env['exp']['rms']:.2e}, lin {env['lin']['rms']:.2e}, both {env['both']['rms']:.2e} rad")
check("large swing: combined envelope recovers viscous sigma = beta/2",
      abs(env["both"]["sigma"] - BETA / 2) < 0.006, f"(sigma = {env['both']['sigma']:.4f}, true {BETA / 2})")
check("large swing: combined envelope recovers Coulomb k = 2 gamma/(pi w0)",
      abs(env["both"]["k"] - k_true) / k_true < 0.15, f"(k = {env['both']['k']:.2e} rad/s, true {k_true:.2e})")

for label, beta, gamma, best in (("viscous only", 0.1, 0.0, "exp"), ("Coulomb only", 0.0, 0.05, "lin")):
    t, u = swing(20.0, 40.0, W0, beta, gamma, CB)
    win = windowed(t, u)
    env = envelope_models(win["t"], win["A"] * 2 * math.pi / CPR)
    pick = "exp" if env["exp"]["rms"] < env["lin"]["rms"] else "lin"
    check(f"{label}: envelope shape identified as {best}", pick == best,
          f"(rms exp {env['exp']['rms']:.2e}, lin {env['lin']['rms']:.2e})")

raise SystemExit(1 if failures else 0)
