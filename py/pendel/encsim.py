"""Synthetic quadrature encoder: exact edge times for a given motion (Stage 0.11, before real logs).

encode(t, theta) on a fine time grid -> per-controller-sample counts and edge events, plus the true
angle and velocity at the samples. Models the count phase, timestamp jitter and the read latency
(an edge is usable only once its timestamp is at least `latency` before the sample).
"""

from __future__ import annotations

import math

import numpy as np


def encode(t: np.ndarray, theta: np.ndarray, ts: float = 0.002, cpr: int = 2400, phase: float = 0.37,
           jitter: float = 2e-6, latency: float = 60e-6, seed: int = 1) -> dict:
    q = 2 * math.pi / cpr
    u = theta / q + phase
    c = np.floor(u).astype(np.int64)
    edges_t, edges_c = [], []
    for i in np.nonzero(np.diff(c))[0]:          # count changes between t[i] and t[i+1]
        step = 1 if c[i + 1] > c[i] else -1
        for b in range(c[i] + (1 if step > 0 else 0), c[i + 1] + (1 if step > 0 else 0), step):
            frac = (b - u[i]) / (u[i + 1] - u[i])  # boundary crossing by linear interpolation
            edges_t.append(t[i] + frac * (t[i + 1] - t[i]))
            edges_c.append(b if step > 0 else b - 1)
    rng = np.random.default_rng(seed)
    edges_t = np.asarray(edges_t) + rng.normal(0, jitter, len(edges_t))
    edges_c = np.asarray(edges_c, dtype=np.int64)
    order = np.argsort(edges_t, kind="stable")
    edges_t, edges_c = edges_t[order], edges_c[order]

    tk = np.arange(t[0], t[-1], ts)
    avail = np.searchsorted(edges_t, tk - latency, side="right")   # edges usable at each sample
    counts = np.where(avail > 0, edges_c[np.maximum(avail - 1, 0)], c[0])
    events = [list(zip(edges_t[a:b], edges_c[a:b])) for a, b in zip(np.r_[0, avail[:-1]], avail)]
    theta_k = np.interp(tk, t, theta)
    vel_k = np.interp(tk, t, np.gradient(theta, t))
    return {"t": tk, "count": counts, "events": events, "theta": theta_k, "vel": vel_k,
            "edges_t": edges_t, "q": q}


def run(est, enc: dict) -> np.ndarray:
    out = np.empty(len(enc["t"]))
    for k in range(len(out)):
        out[k] = est.update(enc["t"][k], int(enc["count"][k]), enc["events"][k])
    return out


def edge_crossings(t: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Exact count-boundary crossings of a continuous encoder position u(t) (counts, count = floor(u)).
    Returns (crossing times, boundary values u = n), sorted in time."""
    c = np.floor(u).astype(np.int64)
    te, ue = [], []
    for i in np.nonzero(np.diff(c))[0]:
        lo, hi = sorted((c[i], c[i + 1]))
        for b in range(lo + 1, hi + 1):
            te.append(t[i] + (b - u[i]) / (u[i + 1] - u[i]) * (t[i + 1] - t[i]))
            ue.append(float(b))
    o = np.argsort(te)
    return np.asarray(te)[o], np.asarray(ue)[o]
