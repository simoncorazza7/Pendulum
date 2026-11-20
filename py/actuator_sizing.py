"""Stage 0.9 sizing (docs/stage0_9_actuator.md): what the planned actuator-test reps do to the hanging pendulum.

    python py/actuator_sizing.py

A rep is an out-and-back pair of trapezoid legs: accelerate at a to v, cruise CRUISE, brake at a; the return leg
starts n*T0 after the outbound one (n = 1 while a leg is shorter than T0), so the two legs' kicks cancel in the
linear model. For each (v, a) rung this prints the leg length, the peak swing during the rep, the swing left after
the pair, and the swing a single leg would leave. Then the swing one lost-sync slip causes (4 full steps =
32 usteps), and the amplitudes of the planned acceleration streams. Plant: config/pendctl.conf plant.* (2026-11-02),
nonlinear, with the Coulomb term; the pendulum starts hanging at rest.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from pendel.plant import PlantParams, pendulum_accel  # noqa: E402

PLANT = PlantParams(w0=4.97, bJ=0.054, coulomb=0.168)
T0 = 2 * math.pi / PLANT.w0          # s
K = PLANT.w0**2 / PLANT.g            # 1/m, input gain near hanging
COUNT = 2 * math.pi / 2400           # rad per encoder count
USTEPS_PER_M = 13333
CRUISE = 0.10                        # s of cruise per leg
LEG_MAX = 0.30                       # m; legs are centred, so |x| <= 0.15 m (switches at +-0.223 m, span 5939 usteps)
SPEEDS = (0.3, 0.45, 0.6, 0.8)       # m/s
ACCELS = (2, 3, 4, 5, 6, 8, 10)      # m/s^2
DT = 1e-4                            # s, semi-implicit Euler (the profiles have steps in a)


def trapezoid(t: float, a: float, v: float) -> float:
    """Cart acceleration of one leg starting at t = 0."""
    ta = v / a
    if 0 <= t < ta:
        return a
    if ta + CRUISE <= t < 2 * ta + CRUISE:
        return -a
    return 0.0


def simulate(accel, t_end: float) -> list[float]:
    """phi = theta - pi (rad) of the hanging pendulum driven by the cart acceleration accel(t)."""
    th, w, out = math.pi, 0.0, []
    for i in range(int(t_end / DT)):
        w += pendulum_accel(th, w, accel(i * DT), PLANT) * DT
        th += w * DT
        out.append(th - math.pi)
    return out


def peak_after(phi: list[float], t: float) -> float:
    i = int(t / DT)
    return max((abs(p) for p in phi[i:]), default=0.0)


def main() -> None:
    print(f"T0 {T0:.3f} s, input gain w0^2/g {K:.3f} 1/m, 1 count = {math.degrees(COUNT):.3f} deg\n")
    print("v m/s | a m/s^2 | leg m | leg s | return at | peak swing deg | left after pair deg | left after one leg deg")
    for v in SPEEDS:
        for a in ACCELS:
            leg = v * v / a + v * CRUISE
            if leg > LEG_MAX:
                print(f"{v:5} | {a:7} | {leg:5.2f} | skipped (leg > {LEG_MAX} m)")
                continue
            tau = 2 * v / a + CRUISE
            t_ret = math.ceil(tau / T0) * T0
            pair = simulate(lambda t: trapezoid(t, a, v) - trapezoid(t - t_ret, a, v), t_ret + tau + 4.0)
            one = simulate(lambda t: trapezoid(t, a, v), tau + 4.0)
            print(f"{v:5} | {a:7} | {leg:5.2f} | {tau:5.2f} | {t_ret:6.2f} s | {math.degrees(max(map(abs, pair))):14.1f} | "
                  f"{math.degrees(peak_after(pair, t_ret + tau + 0.2)):19.1f} | {math.degrees(peak_after(one, tau + 0.2)):.1f}")

    # One lost-sync slip: the cart falls 32 usteps behind over 5 ms (raised-cosine position step), compared with
    # the same pendulum without the slip: at rest, and mid-swing.
    dx, ts = 32 / USTEPS_PER_M, 0.005

    def slip(t: float) -> float:
        return -dx * (math.pi / ts) ** 2 / 2 * math.cos(math.pi * t / ts) if 0 <= t < ts else 0.0

    print(f"\nOne slip of 32 usteps ({dx * 1e3:.2f} mm):")
    for w_start in (0.0, 1.5):
        def run(with_slip: bool) -> list[float]:
            th, w, out = math.pi, w_start, []
            for i in range(int(2.0 / 1e-5)):
                a = slip(i * 1e-5 - 0.3) if with_slip else 0.0
                w += pendulum_accel(th, w, a, PLANT) * 1e-5
                th += w * 1e-5
                out.append(th)
            return out
        dev = max(abs(p - q) for p, q in zip(run(True), run(False)))
        print(f"  pendulum {'at rest' if w_start == 0 else f'swinging at {w_start} rad/s'}: deviation "
              f"{math.degrees(dev):.2f} deg = {dev / COUNT:.1f} counts (linear, instantaneous: {K * dx / COUNT:.1f})")

    print("\nStreams at |v| <= 0.45 m/s:")
    for amp in (4, 6, 8, 10):
        period = 4 * 0.45 / amp                        # bang-bang +-amp reaching +-0.45 m/s
        f0 = amp / (2 * math.pi * 0.45)                # chirp start: velocity amplitude 0.45 m/s
        print(f"  A {amp:2} m/s^2: bang-bang period {period:.3f} s, x +-{amp * period**2 / 32 * 1e3:.0f} mm; chirp from "
              f"{f0:.2f} Hz, x +-{amp / (2 * math.pi * f0) ** 2 * 1e3:.0f} mm there; a rigid pendulum answers 28 Hz with "
              f"{K * amp / (2 * math.pi * 28) ** 2 / COUNT:.2f} counts")


if __name__ == "__main__":
    main()
