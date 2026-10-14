"""Tap test (motor off, sensors only): does the pendulum ring at a high frequency on top of its slow swing?

    .venv/Scripts/python py/tap_test.py [ENCODER_LOG_RUN]

Background: balance runs on 2026-10-14 showed a growing 27-32 Hz oscillation (docs/stage1_demo.md). A rigid
pendulum cannot move 1 deg at 28 Hz under the cart's 2 m/s^2, so either the pendulum structure (rod bending,
shaft coupling) or the drive (belt + stepper, only stiff while the driver holds) resonates near 28 Hz.
Here the driver is off: a ring at ~28 Hz after a tap means the pendulum side; none means the drive side.

Decodes every edge (kernel timestamps), holds the count between edges (2 kHz grid), removes the slow swing
(zero-phase 6 Hz low-pass) and reports per 0.25 s window: swing amplitude, residual RMS (counts) and its
dominant frequency, plus the direction reversals that come faster than 25 ms (an oscillation above ~20 Hz
of at least one count shows up as rapid back-and-forth edges; the plain swing reverses only at its turning points).
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt

from encoder_report import decode_ts
from pendel import latest_run, load_run

FS = 2000.0


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    run = load_run(sys.argv[1] if len(sys.argv) > 1 else latest_run("logs", "encoder-log"))
    d = run.data
    a0, b0 = int(run.meta["start_level_a"]), int(run.meta["start_level_b"])
    order = np.argsort(d["ts_ns"], kind="stable")
    line, typ, ts = d["line"][order], d["type"][order], d["ts_ns"][order]
    counts, same = decode_ts(line, typ, a0, b0, use_type=True)
    t_e = (ts - ts[0]) * 1e-9
    print(f"{run.path.name}: {len(ts)} edges over {t_e[-1]:.1f} s, same-type (invalid) {same}, "
          f"count range {counts.min()} .. {counts.max()}")
    tg = np.arange(0, t_e[-1], 1 / FS)
    c = counts[np.clip(np.searchsorted(t_e, tg, side="right") - 1, 0, len(counts) - 1)].astype(float)
    b, a = butter(2, 6.0 / (FS / 2))
    slow = filtfilt(b, a, c)
    r = c - slow
    # fast reversals: consecutive steps of opposite sign less than 25 ms apart
    steps = np.diff(counts)
    rev_t = t_e[1:][1:][(steps[1:] * steps[:-1] < 0) & (np.diff(t_e[1:]) < 0.025)]
    win = int(0.25 * FS)
    print("\n| t (s) | swing p2p (counts) | residual RMS (counts) | residual peak freq (Hz) | fast reversals |")
    print("|---|---|---|---|---|")
    rows = []
    for k in range(0, len(tg) - win, win):
        seg = slice(k, k + win)
        rr = r[seg] - r[seg].mean()
        sp = np.abs(np.fft.rfft(rr * np.hanning(win)))
        fr = np.fft.rfftfreq(win, 1 / FS)
        band = (fr >= 10) & (fr <= 200)
        fpk = fr[band][np.argmax(sp[band])]
        nrev = int(np.sum((rev_t >= tg[k]) & (rev_t < tg[k] + 0.25)))
        rows.append((tg[k], np.ptp(slow[seg]), rr.std(), fpk, nrev))
    for t0, sw, rms, fpk, nrev in rows:
        if sw > 2 or rms > 0.35 or nrev > 0:               # only the interesting windows
            print(f"| {t0:5.2f} | {sw:6.1f} | {rms:4.2f} | {fpk:5.1f} | {nrev} |")
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    ax[0].plot(tg, c, lw=0.6)
    ax[0].set_ylabel("counts")
    ax[1].plot(tg, r, lw=0.5)
    ax[1].set_ylabel("residual after 6 Hz LP (counts)")
    ax[2].specgram(r, NFFT=512, Fs=FS, noverlap=384, cmap="viridis")
    ax[2].set_ylim(0, 120)
    ax[2].set_ylabel("residual spectrum (Hz)")
    ax[2].set_xlabel("time (s)")
    out = run.path / "tap_test.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"\nplot: {out}")


if __name__ == "__main__":
    main()
