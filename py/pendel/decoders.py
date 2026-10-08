"""Quadrature decoders on raw edge events (encoder-log runs or the synthetic encoder).

Quadrature table of the original code, indexed [prev][cur] with state = 2*A + B; counts use the
original sign convention (A leading B = negative).
"""

from __future__ import annotations

import numpy as np

TABLE = np.array([[0, +1, -1, 0], [-1, 0, 0, +1], [+1, 0, 0, -1], [0, -1, +1, 0]])
RISING = 1


def decode(line, typ, a0: int, b0: int, use_type: bool = True):
    """Events already in timestamp order. Returns (count after each event, step of each event,
    same-type repeats per line). step is +1/-1, or 0 when the event changed nothing."""
    state = [a0, b0]
    count = 0
    counts = np.empty(len(line), dtype=np.int64)
    steps = np.zeros(len(line), dtype=np.int64)
    same = [0, 0]
    for i in range(len(line)):
        new = list(state)
        if use_type:
            new[line[i]] = 1 if typ[i] == RISING else 0
            if new == state:
                same[line[i]] += 1
        else:
            new[line[i]] ^= 1
        d = TABLE[2 * state[0] + state[1], 2 * new[0] + new[1]]
        count += d
        steps[i] = d
        state = new
        counts[i] = count
    return counts, steps, same


def edge_positions(counts: np.ndarray, steps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Continuous encoder position u (in counts) at each edge, with count = floor(u): an edge that
    raised the count to n happened at u = n; one that lowered it to n happened at u = n + 1.
    Returns (mask of real edges, u at those edges)."""
    real = steps != 0
    u = np.where(steps > 0, counts, counts + 1)
    return real, u[real].astype(float)


def edges_from_run(run, use_type: bool = True):
    """(t in s from the first event, u in counts) for an encoder-log run (pendel.load_run)."""
    d = run.data
    o = np.argsort(d["ts_ns"], kind="stable")
    counts, steps, _ = decode(d["line"][o].astype(int), d["type"][o].astype(int),
                              int(run.meta["start_level_a"]), int(run.meta["start_level_b"]), use_type)
    real, u = edge_positions(counts, steps)
    ts = d["ts_ns"][o][real]
    return (ts - ts[0]) * 1e-9, u
