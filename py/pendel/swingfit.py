"""Free-swing fits on encoder edge crossings (Stage 0.7 upright reference, 0.10 identification).

Data: (t_i, u_i) = time and continuous encoder position (counts) of every edge. Each edge is an
exact crossing of a count boundary, so fitting edges gives sub-count precision without the
staircase bias of sampled counts.

  fit_damped(t, u)        u = c + A e^{-sigma t} cos(w t + phi): offset c (= c_bottom for a swing
                          about the hanging equilibrium), w, sigma, with standard errors
  fit_offset_linear(...)  c with w and sigma known: linear least squares (the Pi's start-up method)
  windowed(t, u, ...)     per-window amplitude, frequency and offset (amplitude dependence, envelope)
  envelope_models(...)    peak-amplitude decay: exponential (viscous) vs linear (Coulomb) vs both
  omega0_from_windows()   w0 by extrapolating w(A) = w0 (1 - A^2/16) to A = 0 (pendulum nonlinearity)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares


@dataclass
class DampedFit:
    c: float              # counts, offset (equilibrium position)
    A: float              # counts, amplitude at t = 0
    sigma: float          # 1/s, exponential decay rate (b/J = 2 sigma for viscous friction)
    w: float              # rad/s, damped frequency
    phi: float            # rad
    se: dict              # standard errors of c, A, sigma, w, phi
    rms: float            # counts, residual RMS
    n: int


def _guess(t, u):
    c0 = 0.5 * (u.max() + u.min())
    A0 = 0.5 * (u.max() - u.min())
    # frequency: FFT peak of u resampled on a uniform grid (robust to edges sitting exactly on c0)
    tg = np.arange(t[0], t[-1], 1e-3)
    if len(tg) < 16:
        return c0, A0, 2 * math.pi
    ug = np.interp(tg, t, u)
    ug = (ug - ug.mean()) * np.hanning(len(ug))
    n = 1 << int(math.ceil(math.log2(len(ug)) + 2))           # zero-pad for a finer grid
    spec = np.abs(np.fft.rfft(ug, n))
    f = np.fft.rfftfreq(n, 1e-3)
    k = int(np.argmax(spec[1:])) + 1
    if 1 <= k < len(spec) - 1:                                # parabolic peak interpolation
        a, b, c = np.log(spec[k - 1:k + 2] + 1e-300)
        k = k + 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else k
    return c0, A0, 2 * math.pi * float(k) * (f[1] - f[0])


def fit_damped(t: np.ndarray, u: np.ndarray, w_guess: float | None = None) -> DampedFit:
    c0, A0, wg = _guess(t, u)
    w0 = w_guess or wg
    phi0 = math.acos(max(-1.0, min(1.0, (u[0] - c0) / A0))) if A0 > 0 else 0.0

    def res(p):
        c, A, sig, w, ph = p
        return c + A * np.exp(-sig * t) * np.cos(w * t + ph) - u

    best = None
    for ph in (phi0, -phi0):
        r = least_squares(res, [c0, A0, 0.05, w0, ph], x_scale="jac", max_nfev=2000)
        if best is None or r.cost < best.cost:
            best = r
    J = best.jac
    dof = max(1, len(t) - 5)
    s2 = 2 * best.cost / dof
    try:
        cov = np.linalg.inv(J.T @ J) * s2
        se = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        se = np.full(5, np.nan)
    c, A, sig, w, ph = best.x
    if A < 0:
        A, ph = -A, ph + math.pi
    return DampedFit(c, A, sig, w, ph, dict(zip(("c", "A", "sigma", "w", "phi"), se)),
                     math.sqrt(s2), len(t))


def fit_offset_linear(t: np.ndarray, u: np.ndarray, w: float, sigma: float) -> tuple[float, float]:
    """c and its standard error with w, sigma fixed: u = c + e^{-sigma t}(a cos wt + b sin wt)."""
    e = np.exp(-sigma * t)
    M = np.column_stack([np.ones_like(t), e * np.cos(w * t), e * np.sin(w * t)])
    coef, *_ = np.linalg.lstsq(M, u, rcond=None)
    r = u - M @ coef
    s2 = float(r @ r) / max(1, len(t) - 3)
    cov = np.linalg.inv(M.T @ M) * s2
    return float(coef[0]), float(math.sqrt(cov[0, 0]))


def windowed(t: np.ndarray, u: np.ndarray, periods: float = 2.0, step_periods: float = 1.0,
             min_edges: int = 20) -> dict:
    """Fits c + A cos(w t + phi) (no decay) in sliding windows. Returns arrays t_mid, A, w, c."""
    g = fit_damped(t, u)
    T = 2 * math.pi / g.w
    out = {"t": [], "A": [], "w": [], "c": []}
    t0 = t[0]
    while t0 + periods * T <= t[-1]:
        m = (t >= t0) & (t < t0 + periods * T)
        if m.sum() >= min_edges:
            tt, uu = t[m] - t0, u[m]

            def res(p):
                return p[0] + p[1] * np.cos(p[2] * tt + p[3]) - uu
            c0, A0, _ = _guess(tt, uu)
            best = None
            for ph in np.linspace(0, 2 * math.pi, 8, endpoint=False):
                r = least_squares(res, [c0, A0, g.w, ph], x_scale="jac", max_nfev=500)
                if best is None or r.cost < best.cost:
                    best = r
            out["t"].append(t0 + 0.5 * periods * T)
            out["A"].append(abs(best.x[1]))
            out["w"].append(best.x[2])
            out["c"].append(best.x[0])
        t0 += step_periods * T
    return {k: np.asarray(v) for k, v in out.items()}


def envelope_models(t_mid: np.ndarray, A: np.ndarray) -> dict:
    """Fit the amplitude decay: exponential A0 e^{-sigma t} (viscous), linear A0 - k t (Coulomb),
    and the combination A(t) for dA/dt = -sigma A - k. Returns parameters and residual RMS."""
    out = {}
    r = least_squares(lambda p: p[0] * np.exp(-p[1] * t_mid) - A, [A[0], 0.05])
    out["exp"] = {"A0": r.x[0], "sigma": r.x[1], "rms": math.sqrt(2 * r.cost / len(A))}
    k_lin = np.polyfit(t_mid, A, 1)
    out["lin"] = {"A0": k_lin[1], "k": -k_lin[0], "rms": float(np.sqrt(np.mean((np.polyval(k_lin, t_mid) - A) ** 2)))}

    def comb(p):   # dA/dt = -s A - k  ->  A = (A0 + k/s) e^{-s t} - k/s
        A0, s, k = p
        s = max(s, 1e-9)
        return (A0 + k / s) * np.exp(-s * t_mid) - k / s - A
    r = least_squares(comb, [A[0], 0.02, 0.1 * max(1e-6, (A[0] - A[-1]) / max(1e-6, t_mid[-1] - t_mid[0]))],
                      bounds=([0, 0, 0], [np.inf, np.inf, np.inf]))
    out["both"] = {"A0": r.x[0], "sigma": r.x[1], "k": r.x[2], "rms": math.sqrt(2 * r.cost / len(A))}
    return out


def omega0_from_windows(A_counts: np.ndarray, w: np.ndarray, cpr: int = 2400) -> tuple[float, float]:
    """w0 from w(A) = w0 (1 - A^2/16) (A in rad), linear fit in A^2. Returns (w0, slope check)."""
    A = A_counts * 2 * math.pi / cpr
    p = np.polyfit(A * A, w, 1)
    w0 = p[1]
    return float(w0), float(-p[0] / w0)   # second value should be ~1/16 = 0.0625 for a rigid pendulum
