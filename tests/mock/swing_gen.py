"""Drive a synthetic damped pendulum swing onto the gpio-mockup encoder lines (needs root: debugfs).

Writes the quadrature levels of u(t) = c + A e^{-sigma t} sin(w t) (counts), one count at a time,
into /sys/kernel/debug/gpio-mockup/<chip>/<line>. Same decoder convention as the rig (state = 2A+B,
00 -> 01 -> 11 -> 10 is +1). The lines must start at A = B = 0 (count 0).

    sudo python3 tests/mock/swing_gen.py DBG_DIR --amp-deg 5 --duration 10
"""

import argparse
import math
import os
import time

GRAY = [(0, 0), (0, 1), (1, 1), (1, 0)]   # (A, B) for count mod 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dbg")
    ap.add_argument("--line-a", type=int, default=10)
    ap.add_argument("--line-b", type=int, default=19)
    ap.add_argument("--amp-deg", type=float, default=5.0)
    ap.add_argument("--w0", type=float, default=4.931)       # rad/s (plant.w0); w = w0 (1 - A^2/16)
    ap.add_argument("--sigma", type=float, default=0.03)     # 1/s
    ap.add_argument("--centre", type=float, default=0.0)     # counts
    ap.add_argument("--duration", type=float, default=10.0)  # s
    a = ap.parse_args()

    amp = a.amp_deg * 2400 / 360
    w = a.w0 * (1 - math.radians(a.amp_deg) ** 2 / 16)
    fa = os.open(os.path.join(a.dbg, str(a.line_a)), os.O_WRONLY)
    fb = os.open(os.path.join(a.dbg, str(a.line_b)), os.O_WRONLY)
    lv = [0, 0]
    count = 0
    t0 = time.monotonic()
    edges = 0
    while True:
        t = time.monotonic() - t0
        if t > a.duration:
            break
        target = math.floor(a.centre + amp * math.exp(-a.sigma * t) * math.sin(w * t))
        while count != target:
            count += 1 if target > count else -1
            na, nb = GRAY[count % 4]
            if na != lv[0]:
                os.write(fa, b"1" if na else b"0")
                lv[0] = na
            if nb != lv[1]:
                os.write(fb, b"1" if nb else b"0")
                lv[1] = nb
            edges += 1
        time.sleep(0.0005)
    # back to count 0's levels is not needed; leave the lines where they are
    print(f"swing_gen: {edges} edges, final count {count}")


if __name__ == "__main__":
    main()
