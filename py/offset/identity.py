"""Online upright-offset estimate from the drift identity (simulation prototype, PC only; docs/offset_estimator.md).

The identity (docs/stage1_drift.md, py/stage1_report.py quiet_offsets): along any balance, whatever the controller,

    r(t) = (v - v_s) - g int_s^t theta dt + L_eq (theta' - theta'_s) + L_eq (b/J)(theta - theta_s) = g delta (t - t_s) + noise

where theta is the measured angle, delta = theta_true - theta the upright reference error, and _s marks the start of
the stretch. It is the linear model integrated once, so it needs no acceleration and no model of the controller.
stage1_report fits its slope offline over "quiet" stretches; it predicted the 2026-11-04 parking offset and set the
2026-10-14 trim. This class does the same online, one tick at a time:

- A stretch starts t_settle after engage (hand on the rod, catch transient) and ends when |theta| leaves theta_max,
  when the balance ends, or when the residual steps (a push: its 0.25 s mean moves by more than `step` m/s within
  `lag` s, beyond the fitted slope). Samples are committed `lag` s late, so a push, a fall or an abort discards the
  samples that saw it instead of fitting them.
- Each stretch with at least min_stretch s committed gives a slope; the estimate is the length-weighted mean of all
  stretch slopes so far (finished ones and the running one), plus an optional prior with its own weight in seconds.
  That is what carries the offset from one catch to the next: the pool persists between balances.
- Weights are lengths, not the T^3 of a white-noise line fit: the residual's noise is correlated (the 14 Hz rate
  estimate, pivot friction), and on the 11-02 pivot quiet stretches of one balance scatter by ~0.5-0.7 counts.

delta_hat() is the estimate (rad, theta_true ~ theta + delta_hat); weight() the seconds of quiet balance behind it.
Use it at engage: theta_ref = theta_ref_config - delta_hat once weight() >= 3 s, held for the whole catch, while
step() keeps learning for the next one. Changing the trim within a catch fights the Stage 3b cart integral, which is
already correcting the same offset there (docs/offset_estimator.md, py/offset_study.py).
Deterministic, no I/O: a C++ port would keep the running sums and a ring buffer of lag/Ts samples.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

COUNT = 2 * math.pi / 2400       # rad per encoder count


@dataclass(frozen=True)
class IdentityParams:
    w0: float                                  # rad/s (plant.w0)
    bJ: float                                  # 1/s (plant.bJ)
    ts: float = 0.002                          # s (loop.ts)
    g: float = 9.81                            # m/s^2
    t_settle: float = 1.5                      # s after engage before a stretch may start (stage1_report: 1.5)
    theta_max: float = math.radians(3.0)       # rad, |theta| above this ends the stretch
    lag: float = 0.3                           # s, commit delay = step-detection window (stage1_report: 0.3)
    smooth: float = 0.25                       # s, moving mean of r for the step detector (stage1_report: 0.25)
    step: float = 0.03                         # m/s, residual step that marks a push (stage1_report: 0.03)
    min_stretch: float = 1.0                   # s committed before a stretch's slope counts (stage1_report: 1.0)
    delta_max: float = 5 * COUNT               # rad, clamp of the estimate


class DriftIdentityEstimator:
    def __init__(self, p: IdentityParams, prior: float = 0.0, prior_weight: float = 0.0):
        self.p = p
        self.L = p.g / p.w0 ** 2
        self.nlag = max(1, int(round(p.lag / p.ts)))
        self.nsm = max(1, int(round(p.smooth / p.ts)))
        # pool of finished stretches: sum of T_j * slope_j and sum of T_j (slopes in m/s^2), prior included
        self.pool_ws = prior_weight * p.g * prior
        self.pool_w = prior_weight
        self.n_stretches = 0
        self._reset_stretch(None)

    # --- stretch bookkeeping -------------------------------------------------------------------------------------
    def _reset_stretch(self, start):
        """start: (theta, omega, v) at the stretch start, or None (no stretch open)."""
        self.open = start is not None
        if start is not None:
            self.th_s, self.om_s, self.v_s = start
        self.k = 0                                   # ticks since the stretch start
        self.integ = 0.0                             # int theta dt since the start (trapezoid)
        self.th_prev = start[0] if start is not None else 0.0
        self.buf = deque()                           # (t, r) not yet committed
        self.sm = deque()                            # last nsm residuals (moving mean)
        self.sm_sum = 0.0
        self.sm_hist = deque()                       # moving means of the last nlag + 1 ticks
        self.n = 0                                   # committed samples: n, sum t, sum r, sum t^2, sum t r
        self.st = self.sr = self.stt = self.str = 0.0

    def _stretch_len(self) -> float:
        return self.n * self.p.ts

    def _stretch_slope(self) -> float | None:
        if self._stretch_len() < self.p.min_stretch or self.n < 3:
            return None
        den = self.stt - self.st * self.st / self.n
        if den <= 0:
            return None
        return (self.str - self.st * self.sr / self.n) / den

    def _close(self):
        """End the open stretch: discard uncommitted samples, pool its slope if it is long enough."""
        if self.open:
            sl = self._stretch_slope()
            if sl is not None:
                T = self._stretch_len()
                self.pool_ws += T * sl
                self.pool_w += T
                self.n_stretches += 1
        self._reset_stretch(None)

    # --- public ----------------------------------------------------------------------------------------------------
    def end_balance(self):
        """The balance ended (abort, soft limit, hand-over failed, operator): close the stretch."""
        self._close()

    def step(self, t_engaged: float, theta: float, omega: float, v: float, balancing: bool = True) -> None:
        """One control tick. t_engaged: s since engage (or hand-over); theta: measured angle (rad); omega: the
        loop's rate estimate (rad/s); v: cart speed (m/s); balancing: the balance controller is in charge."""
        p = self.p
        ok = balancing and t_engaged >= p.t_settle and abs(theta) < p.theta_max
        if not ok:
            if self.open:
                self._close()
            return
        if not self.open:
            self._reset_stretch((theta, omega, v))
            return
        self.k += 1
        self.integ += 0.5 * (theta + self.th_prev) * p.ts
        self.th_prev = theta
        t = self.k * p.ts
        r = (v - self.v_s) - p.g * self.integ + self.L * (omega - self.om_s) + self.L * p.bJ * (theta - self.th_s)
        # step detector on the 0.25 s moving mean, against the current slope estimate
        self.sm.append(r)
        self.sm_sum += r
        if len(self.sm) > self.nsm:
            self.sm_sum -= self.sm.popleft()
        m = self.sm_sum / len(self.sm)
        self.sm_hist.append(m)
        if len(self.sm_hist) > self.nlag + 1:
            self.sm_hist.popleft()
        if len(self.sm_hist) == self.nlag + 1 and len(self.sm) == self.nsm:
            jump = m - self.sm_hist[0] - self.slope() * p.lag
            if abs(jump) > p.step:
                self._close()                                       # the push's samples are discarded
                self._reset_stretch((theta, omega, v))              # a new stretch, new intercept
                return
        # commit late
        self.buf.append((t, r))
        if len(self.buf) > self.nlag:
            tc, rc = self.buf.popleft()
            self.n += 1
            self.st += tc
            self.sr += rc
            self.stt += tc * tc
            self.str += tc * rc

    def slope(self) -> float:
        """Current estimate of g delta (m/s^2): length-weighted mean of the stretch slopes (+ prior)."""
        ws, w = self.pool_ws, self.pool_w
        sl = self._stretch_slope() if self.open else None
        if sl is not None:
            T = self._stretch_len()
            ws += T * sl
            w += T
        return ws / w if w > 0 else 0.0

    def weight(self) -> float:
        """Seconds of quiet balance (and prior) behind the estimate."""
        w = self.pool_w
        if self.open and self._stretch_slope() is not None:
            w += self._stretch_len()
        return w

    def delta_hat(self) -> float:
        d = self.slope() / self.p.g
        return max(-self.p.delta_max, min(self.p.delta_max, d))
