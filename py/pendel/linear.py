"""Discrete-time small-signal analysis of the balance loop (Stage 0.12 / Stage 1 design).

Exact ZOH discretisation of the linearised upright plant with a total input delay
tau = fractional compute delay + N whole samples, closed with the same controller as
control.pid_step (backward difference + 1st-order low-pass derivative, forward-Euler integral,
no saturation). Margins are found directly from closed-loop eigenvalues:
  gain margins  : the interval [k_lo, k_hi] of gain multipliers k (all gains * k) that stay stable
  delay margin  : extra input delay (s) before instability
  phase margin  : from L(e^{jwT}) at |L| = 1 (all crossings)
  Nyquist check : counter-clockwise encirclements of -1 must equal the unstable open-loop poles
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm, solve_discrete_lyapunov

from .control import PidParams
from .plant import PlantParams


def _zoh_integral(A: np.ndarray, B: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray]:
    """(e^{At}, int_0^t e^{As} ds B)."""
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * t)
    return E[:n, :n], E[:n, n:]


@dataclass
class Loop:
    A_ol: np.ndarray
    B_ol: np.ndarray      # from controller output r
    C_ol: np.ndarray      # controller output u
    Bn: np.ndarray        # from measurement noise n (rad)
    Dn: float
    ts: float

    def closed(self, k: float = 1.0) -> np.ndarray:
        return self.A_ol + k * np.outer(self.B_ol, self.C_ol)

    def stable(self, k: float = 1.0) -> bool:
        return bool(np.max(np.abs(np.linalg.eigvals(self.closed(k)))) < 1.0)

    def L(self, w: np.ndarray) -> np.ndarray:
        """Loop gain L = -G_ol at frequencies w (rad/s); 1 + L = 0 on the stability boundary."""
        z = np.exp(1j * w * self.ts)
        n = self.A_ol.shape[0]
        out = np.empty(len(w), dtype=complex)
        for i, zi in enumerate(z):
            out[i] = -self.C_ol @ np.linalg.solve(zi * np.eye(n) - self.A_ol, self.B_ol)
        return out


def build_loop(plant: PlantParams, pid: PidParams, ts: float, delay: float, eps_int: float = 0.0) -> Loop:
    """delay: total input delay (s) from sampling to the acceleration taking effect."""
    w0, beta, g = plant.w0, plant.bJ, plant.g
    A = np.array([[0.0, 1.0], [w0**2, -beta]])
    B = np.array([[0.0], [-w0**2 / g]])
    N = int(math.floor(delay / ts + 1e-12))
    d = delay - N * ts
    Phi, G0 = _zoh_integral(A, B, ts - d)          # input u'_k acts on [kT + d, (k+1)T)
    E_d, Gd = _zoh_integral(A, B, d)
    Phi_full = Phi @ E_d
    G1 = Phi @ Gd                                   # input u'_{k-1} acts on [kT, kT + d)
    G0, G1 = G0[:, 0], G1[:, 0]

    alpha = ts / (ts + 1.0 / (2 * math.pi * pid.vel_fc))
    # state: x1, x2, u_prev, d_1..d_N, I, vf, th_prev
    iu, idl = 2, 3
    iI, ivf, itp = 3 + N, 4 + N, 5 + N
    dim = 6 + N
    A_ol = np.zeros((dim, dim))
    B_ol = np.zeros(dim)
    Bn = np.zeros(dim)
    A_ol[0:2, 0:2] = Phi_full
    A_ol[0:2, iu] = G1
    if N == 0:
        B_ol[0:2] = G0          # u' = r
        B_ol[iu] = 1.0
    else:
        A_ol[0:2, idl + N - 1] = G0
        A_ol[iu, idl + N - 1] = 1.0
        B_ol[idl] = 1.0
        for i in range(1, N):
            A_ol[idl + i, idl + i - 1] = 1.0
    # controller
    if pid.ki != 0.0:            # with ki = 0 the state is unused; leave it at z = 0, not z = 1
        A_ol[iI, iI] = 1.0 - eps_int
        A_ol[iI, 0] = ts
        Bn[iI] = ts
    A_ol[ivf, ivf] = 1.0 - alpha
    A_ol[ivf, 0] = alpha / ts
    A_ol[ivf, itp] = -alpha / ts
    Bn[ivf] = alpha / ts
    A_ol[itp, 0] = 1.0
    Bn[itp] = 1.0
    C_ol = np.zeros(dim)
    C_ol[0] = pid.kp + pid.kd * alpha / ts
    C_ol[iI] = pid.ki
    C_ol[ivf] = pid.kd * (1.0 - alpha)
    C_ol[itp] = -pid.kd * alpha / ts
    Dn = pid.kp + pid.kd * alpha / ts
    return Loop(A_ol, B_ol, C_ol, Bn, Dn, ts)


def _bisect(f_stable, lo: float, hi: float, iters: int = 60) -> float:
    """lo stable, hi unstable -> boundary."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if f_stable(mid):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def margins(plant: PlantParams, pid: PidParams, ts: float, delay: float) -> dict:
    loop = build_loop(plant, pid, ts, delay)
    res = {"stable": loop.stable(1.0)}
    if not res["stable"]:
        return res
    # gain margins: scan multipliers, then refine the boundaries around k = 1
    ks = np.logspace(-2, 2, 400)
    st = np.array([loop.stable(k) for k in ks])
    below = ks[(ks < 1) & ~st]
    above = ks[(ks > 1) & ~st]
    res["k_lo"] = _bisect(loop.stable, 1.0, below.max()) if len(below) else 0.0
    res["k_hi"] = _bisect(loop.stable, 1.0, above.min()) if len(above) else math.inf
    # delay margin (extra seconds)
    stable_with = lambda extra: build_loop(plant, pid, ts, delay + extra).stable()
    hi = 0.001
    while stable_with(hi) and hi < 1.0:
        hi *= 2
    res["delay_margin"] = _bisect(stable_with, 0.0, hi, 40) if hi < 1.0 else math.inf
    # phase margins at |L| = 1 crossings
    loop_e = build_loop(plant, pid, ts, delay, eps_int=1e-9)
    w = np.logspace(-2, math.log10(math.pi / ts * 0.999), 4000)
    Lw = loop_e.L(w)
    mag = np.abs(Lw)
    idx = np.where(np.diff(np.sign(mag - 1.0)) != 0)[0]
    pms = [(w[i], math.degrees(math.atan2(Lw[i].imag, Lw[i].real)) + 180.0) for i in idx]
    res["phase_crossings"] = [(wi, ((pm + 180) % 360) - 180) for wi, pm in pms]
    # Nyquist: CCW encirclements of -1 by L over the full unit circle vs unstable OL poles
    wf = np.linspace(-math.pi / ts, math.pi / ts, 20001)
    Lf = loop_e.L(wf)
    winding = np.round(np.sum(np.diff(np.unwrap(np.angle(1 + Lf)))) / (2 * math.pi))
    p_unstable = int(np.sum(np.abs(np.linalg.eigvals(loop_e.A_ol)) > 1.0 + 1e-9))
    res["nyquist_ccw"] = int(winding)
    res["ol_unstable_poles"] = p_unstable
    return res


def noise_rms(plant: PlantParams, pid: PidParams, ts: float, delay: float, sigma: float) -> float:
    """RMS of the commanded acceleration (m/s^2) for white measurement noise of std sigma (rad)."""
    loop = build_loop(plant, pid, ts, delay)
    Acl = loop.closed()
    Bcl = loop.Bn + loop.B_ol * loop.Dn
    P = solve_discrete_lyapunov(Acl, np.outer(Bcl, Bcl) * sigma**2)
    return float(math.sqrt(loop.C_ol @ P @ loop.C_ol + (loop.Dn * sigma) ** 2))
