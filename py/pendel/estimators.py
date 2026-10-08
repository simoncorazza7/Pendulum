"""Velocity estimator candidates for the balance loop (Stage 0.11).

Common interface, called once per controller sample:
    est.update(t, count, events) -> angular velocity (rad/s)
      t       sample time (s)
      count   cumulative encoder count at the sample (int)
      events  edges that became available since the previous sample: [(timestamp_s, count_after), ...]
Linear candidates use `count` only; the edge-timestamp candidate uses `events`.
"""

from __future__ import annotations

import math
from collections import deque


class BdLp1:
    """(a) Backward difference at Ts + 1st-order low-pass (same as control.pid_step)."""

    def __init__(self, ts: float, fc: float, cpr: int = 2400):
        self.q, self.ts = 2 * math.pi / cpr, ts
        self.alpha = ts / (ts + 1.0 / (2 * math.pi * fc))
        self.prev, self.v = None, 0.0

    def update(self, t, count, events):
        vr = 0.0 if self.prev is None else (count - self.prev) * self.q / self.ts
        self.prev = count
        self.v += self.alpha * (vr - self.v)
        return self.v


class BdLp2:
    """(a) Backward difference + 2nd-order Butterworth low-pass (bilinear, prewarped)."""

    def __init__(self, ts: float, fc: float, cpr: int = 2400):
        self.q, self.ts = 2 * math.pi / cpr, ts
        k = math.tan(math.pi * fc * ts)
        n = 1 / (1 + math.sqrt(2) * k + k * k)
        self.b0 = k * k * n
        self.b1, self.b2 = 2 * self.b0, self.b0
        self.a1 = 2 * (k * k - 1) * n
        self.a2 = (1 - math.sqrt(2) * k + k * k) * n
        self.prev = None
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def update(self, t, count, events):
        x = 0.0 if self.prev is None else (count - self.prev) * self.q / self.ts
        self.prev = count
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2, self.x1, self.y2, self.y1 = self.x1, x, self.y1, y
        return y


class AlphaBeta:
    """(b) alpha-beta tracker on the quantised angle; error poles placed at exp(-2 pi fn Ts) (double)."""

    def __init__(self, ts: float, fn: float, cpr: int = 2400):
        self.q, self.ts = 2 * math.pi / cpr, ts
        p = math.exp(-2 * math.pi * fn * ts)
        self.a, self.b = 1 - p * p, (1 - p) ** 2
        self.x, self.v = None, 0.0

    def update(self, t, count, events):
        z = count * self.q
        if self.x is None:
            self.x = z
            return 0.0
        xp = self.x + self.ts * self.v
        r = z - xp
        self.x = xp + self.a * r
        self.v += self.b / self.ts * r
        return self.v


class EdgeTimestamp:
    """(c) Counts over time between recent edges of one direction, bounded by 1 count / (time since
    the last edge) so it decays to 0 when edges stop. Optional 1st-order low-pass on the result."""

    def __init__(self, ts: float, window: float = 0.03, max_edges: int = 8, fc: float | None = None,
                 cpr: int = 2400):
        self.q, self.ts, self.window, self.max_edges = 2 * math.pi / cpr, ts, window, max_edges
        self.edges = deque()            # (timestamp, count_after), newest last
        self.alpha = None if fc is None else ts / (ts + 1.0 / (2 * math.pi * fc))
        self.v = 0.0

    def update(self, t, count, events):
        self.edges.extend(events)
        while len(self.edges) > 2 and (t - self.edges[1][0] > self.window or len(self.edges) > self.max_edges + 1):
            self.edges.popleft()
        v = 0.0
        if len(self.edges) >= 2:
            e = list(self.edges)
            step = e[-1][1] - e[-2][1]                     # direction of the newest edge
            i = len(e) - 2
            while i > 0 and (e[i][1] - e[i - 1][1]) == step:
                i -= 1                                    # extend back over edges of the same direction
            dc, dt = e[-1][1] - e[i][1], e[-1][0] - e[i][0]
            if dt > 0:
                v = dc * self.q / dt
            since = t - e[-1][0]
            if since > 0:
                bound = self.q / since                    # at most one count since the last edge
                v = math.copysign(min(abs(v), bound), v)
        if self.alpha is not None:
            self.v += self.alpha * (v - self.v)
            return self.v
        return v


def candidates(ts: float) -> dict:
    """The set compared in py/estimator_compare.py (fresh instances)."""
    return {
        "BD+LP1 20 Hz": BdLp1(ts, 20.0),
        "BD+LP1 30 Hz": BdLp1(ts, 30.0),
        "BD+LP1 40 Hz": BdLp1(ts, 40.0),
        "BD+LP2 30 Hz": BdLp2(ts, 30.0),
        "BD+LP2 50 Hz": BdLp2(ts, 50.0),
        "alpha-beta 15 Hz": AlphaBeta(ts, 15.0),
        "alpha-beta 25 Hz": AlphaBeta(ts, 25.0),
        "edge-ts 30 ms/8": EdgeTimestamp(ts, window=0.03, max_edges=8),
        "edge-ts 10 ms/3": EdgeTimestamp(ts, window=0.01, max_edges=3),
        "edge-ts 10 ms/3 + LP1 40 Hz": EdgeTimestamp(ts, window=0.01, max_edges=3, fc=40.0),
    }
