"""Does the ~28 Hz oscillation follow the cart speed, or stay at one frequency?

    .venv/Scripts/python py/osc_speed.py [RUN_DIR ...] [--out PNG]     (default: every logs/*_balance)

Speed-locked excitations on this drive (GT2 belt, 60T pulley, 1/8 step), per metre of cart travel:
  belt teeth (motor pulley, smooth idler)   1 / 2 mm           = 500 /m   28 Hz at 56 mm/s
  stepper electrical cycle (4 full steps)   usteps_per_m / 32  = 417 /m   28 Hz at 67 mm/s
  stepper full step                         usteps_per_m / 8   = 1667 /m  28 Hz at 17 mm/s
A mode of the cart, belt or encoder bracket stays near one frequency whatever the speed |v| is.

Groups (by supervisor mode and session phase):
  ENGAGED      balancing, loop closed; 0.5 s after engage to 0.3 s before the end
  open-loop    homing, re-centring, finishing: shaped moves, no controller (phases Home, Recentre, Finishing)
  swing-up     any other tick with a non-zero step rate (swing-up energy loop, hand-over)

Windows of 0.25 s (hop 0.05 s) of the encoder counts:
  f_pk    peak of the 15-60 Hz spectrum (Hann, zero-padded), Hz
  band    RMS of the 15-60 Hz content, counts (quantisation alone gives about 0.12)
  |v|     mean |v| after a 5 Hz low-pass, m/s. A window is STEADY when v keeps its sign and varies by < 30 %.
A window is PEAKED when band >= 0.5 counts and its peak is >= 5x the band's median power.

Order spectrum: 1 s windows at constant speed (varies < 5 %, |v| >= 20 mm/s), each spectrum in dB above its own
15 Hz running median, resampled onto cycles per metre of travel (f / |v|), 24-34 Hz left out. A speed-locked
excitation that reaches the encoder shows as a peak at its order; a fixed-frequency mode smears out.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import warnings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np
from scipy.ndimage import median_filter
from scipy.signal import butter, filtfilt
from scipy.stats import spearmanr

from pendel.runs import parse_conf

TS = 0.002                                                  # s, control tick
WIN, HOP = 125, 25                                          # samples: 0.25 s windows, 0.05 s hop
NFFT = 2048                                                 # zero padding for f_pk: 0.24 Hz grid
BAND = (15.0, 60.0)                                         # Hz
MODE_BAND = (24.0, 34.0)                                    # Hz, left out of the order spectrum
PITCH = 0.002                                               # m, GT2 tooth pitch
BAND_MIN, PROM_MIN = 0.5, 5.0                               # counts RMS, peak / median power: PEAKED
SPEED_BINS = np.array([0, 5, 15, 30, 45, 60, 80, 110, 150, 200, 300]) / 1000     # m/s, table rows
AMP_V = np.arange(0, 0.3001, 0.01)                          # m/s, amplitude-plot bins
WIN_O, HOP_O, NFFT_O = 500, 125, 4096                       # samples: 1 s order windows, 0.25 s hop
ORDERS = np.arange(100.0, 2501.0, 5.0)                      # 1/m, order-spectrum grid
LINE_HZ = 15.0                                              # Hz, running-median width the lines are measured against
MIN_ORDER_WINDOWS = 20                                      # order spectra with fewer windows are not reported
SESSION_MOVES = (2, 5, 6)                                   # rt::SessionPhase Home, Recentre, Finishing

HP = butter(2, 10 * 2 * TS, btype="high")
BP = butter(2, [BAND[0] * 2 * TS, BAND[1] * 2 * TS], btype="band")
LP = butter(2, 5 * 2 * TS)
HANN, HANN_O = np.hanning(WIN), np.hanning(WIN_O)
FREQ = np.fft.rfftfreq(NFFT, TS)                            # Hz, f_pk grid
FREQ_O = np.fft.rfftfreq(NFFT_O, TS)                        # Hz, order windows
IN_BAND = (FREQ >= BAND[0]) & (FREQ <= BAND[1])
COLS = ("phase", "mode", "counts", "speed_toggles", "v")
GROUPS = ("ENGAGED", "open-loop", "swing-up")


def load(path: pathlib.Path):
    names = (path / "data.csv").open(encoding="utf-8").readline().strip().split(",")
    d = np.loadtxt(path / "data.csv", delimiter=",", skiprows=1, usecols=[names.index(c) for c in COLS], ndmin=2)
    conf = parse_conf((path / "config_used.conf").read_text(encoding="utf-8"))
    return {c: d[:, i] for i, c in enumerate(COLS)}, conf


def stretches(mask: np.ndarray, trim0: int = 0, trim1: int = 0) -> list[tuple[int, int]]:
    e = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(np.int8), [0]))))
    return [(a + trim0, b - trim1) for a, b in zip(e[::2], e[1::2]) if b - trim1 - a - trim0 >= 2 * WIN]


def windows(counts: np.ndarray, v: np.ndarray) -> dict[str, np.ndarray]:
    """Per 0.25 s window: f_pk, prominence, band RMS, mean |v| and the steady flag."""
    sw = np.lib.stride_tricks.sliding_window_view
    c = sw(filtfilt(*HP, counts), WIN)[::HOP]
    pb = (np.abs(np.fft.rfft((c - c.mean(1, keepdims=True)) * HANN, NFFT)) ** 2)[:, IN_BAND]
    vl = sw(filtfilt(*LP, v), WIN)[::HOP]
    vabs = np.abs(vl).mean(1)
    with np.errstate(all="ignore"):                         # a window with no count change has no spectrum
        prom = pb.max(1) / np.median(pb, 1)
    return {
        "f": FREQ[IN_BAND][pb.argmax(1)],
        "prom": prom,
        "band": sw(filtfilt(*BP, counts), WIN)[::HOP].std(1),
        "v": vabs,
        "steady": (vl.min(1) * vl.max(1) > 0) & (vl.max(1) - vl.min(1) < 0.3 * vabs),
    }


def order_spectra(counts: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Spectra of the 1 s constant-speed windows, dB above their own running median (LINE_HZ wide), on the
    ORDERS grid (NaN outside the usable band). Sharp lines stand out; the slope and broad humps cancel."""
    if len(counts) < WIN_O + 50:
        return np.empty((0, len(ORDERS)))
    sw = np.lib.stride_tricks.sliding_window_view
    c = sw(filtfilt(*HP, counts), WIN_O)[::HOP_O]
    vl = sw(filtfilt(*LP, v), WIN_O)[::HOP_O]
    vabs = np.abs(vl).mean(1)
    ok = (vl.min(1) * vl.max(1) > 0) & (vl.max(1) - vl.min(1) < 0.05 * vabs) & (vabs >= 0.02)
    c = c[ok]
    p = 10 * np.log10(np.abs(np.fft.rfft((c - c.mean(1, keepdims=True)) * HANN_O, NFFT_O)) ** 2 + 1e-12)
    p -= median_filter(p, size=(1, int(LINE_HZ / FREQ_O[1]) | 1), mode="nearest")
    out = np.full((len(p), len(ORDERS)), np.nan)
    for i, (row, va) in enumerate(zip(p, vabs[ok])):
        fo = ORDERS * va
        m = (fo >= 12) & (fo <= 245) & ((fo < MODE_BAND[0]) | (fo > MODE_BAND[1]))
        out[i, m] = np.interp(fo[m], FREQ_O, row)
    return out


def collect(paths: list[pathlib.Path]):
    groups = {g: [] for g in GROUPS}
    orders = {g: [] for g in GROUPS}
    sessions = []
    upm = None
    for i, p in enumerate(paths):
        d, conf = load(p)
        upm = upm or float(conf.get("axis.usteps_per_m", 13333))
        moving, engaged = d["speed_toggles"] != 0, d["mode"] == 2
        mover = np.isin(d["phase"], SESSION_MOVES)
        parts = {"ENGAGED": stretches(engaged, int(0.5 / TS), int(0.3 / TS)),
                 "open-loop": stretches(moving & mover & ~engaged),
                 "swing-up": stretches(moving & ~mover & ~engaged)}
        for g, segs in parts.items():
            for a, b in segs:
                w = windows(d["counts"][a:b], d["v"][a:b])
                w["session"] = np.full(len(w["f"]), i)
                groups[g].append(w)
                orders[g].append(order_spectra(d["counts"][a:b], d["v"][a:b]))
        sessions.append((p.name, conf.get("ctrl.est_fc", "?"),
                         *(sum(b - a for a, b in parts[g]) * TS for g in GROUPS)))
    merged = {g: {k: np.concatenate([w[k] for w in ws]) for k in ws[0]} for g, ws in groups.items() if ws}
    orders = {g: np.concatenate(o) for g, o in orders.items() if o and sum(len(x) for x in o) >= MIN_ORDER_WINDOWS}
    return merged, orders, sessions, upm


def lines(upm: float) -> dict[str, float]:
    """Excitation orders, cycles per metre of cart travel (= Hz per m/s)."""
    return {"belt teeth": 1 / PITCH, "stepper electrical": upm / 32, "stepper full step": upm / 8}


def report(name: str, g: dict[str, np.ndarray], upm: float) -> None:
    f, band, vabs, steady = g["f"], g["band"], g["v"], g["steady"]
    peaked = (band >= BAND_MIN) & (g["prom"] >= PROM_MIN)
    print(f"\n### {name}: {len(f)} windows, {steady.sum()} steady, {peaked.sum()} peaked")
    print("| \\|v\\| (mm/s) | windows | band RMS median / p90 (counts) | peaked | f_pk median [IQR] (Hz) | "
          "teeth 500\\|v\\| here (Hz) |")
    print("|---|---|---|---|---|---|")
    rows = [(f"{lo * 1e3:.0f}-{hi * 1e3:.0f}", steady & (vabs >= lo) & (vabs < hi), f"{(lo + hi) / 2 / PITCH:.0f}")
            for lo, hi in zip(SPEED_BINS[:-1], SPEED_BINS[1:])]
    rows.append(("v reverses / varies", ~steady, "-"))
    for label, m, teeth in rows:
        if not m.any():
            continue
        mp = m & peaked
        fq = f"{np.median(f[mp]):.1f} [{np.percentile(f[mp], 25):.1f}-{np.percentile(f[mp], 75):.1f}]" \
            if mp.sum() >= 3 else "-"
        print(f"| {label} | {m.sum()} | {np.median(band[m]):.2f} / {np.percentile(band[m], 90):.2f} | "
              f"{mp.sum()} | {fq} | {teeth} |")
    m = peaked & steady & (vabs >= 0.015)
    if m.sum() < 5:
        print("\nToo few peaked steady windows above 15 mm/s for a fit.")
        return
    slope, icpt = np.polyfit(vabs[m], f[m], 1)
    rho, pval = spearmanr(vabs[m], f[m])
    print(f"\nPeaked steady windows above 15 mm/s: n = {m.sum()}, f_pk = {icpt:.1f} Hz + {slope:.0f} Hz/(m/s) * |v|, "
          f"Spearman rho = {rho:.2f} (p = {pval:.2g})")
    print(f"  f_pk in 26-33 Hz: {np.mean((f[m] >= 26) & (f[m] <= 33)):.0%}")
    for lname, k in lines(upm).items():
        print(f"  f_pk within 10 % of {lname} ({k:.0f}|v|): {np.mean(np.abs(f[m] / (k * vabs[m]) - 1) < 0.1):.0%}")


def order_report(name: str, o: np.ndarray, upm: float) -> np.ndarray:
    """Mean order spectrum (dB above the local median); prints the level at each excitation order and the top peaks."""
    n = np.sum(~np.isnan(o), 0)
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        db = np.where(n >= 5, np.nanmean(o, 0), np.nan)
    print(f"\n### Order spectrum, {name}: {len(o)} constant-speed 1 s windows (dB above the local median)")
    print("| order (1/m) | what | dB above median (max within +-5 %) | windows |")
    print("|---|---|---|---|")
    named = {**lines(upm), "2x stepper electrical": upm / 16, "2x belt teeth": 2 / PITCH}
    for lname, k in sorted(named.items(), key=lambda x: x[1]):
        m = np.abs(ORDERS / k - 1) <= 0.05
        val = f"{np.nanmax(db[m]):.1f}" if np.any(~np.isnan(db[m])) else "-"
        print(f"| {k:.0f} | {lname} | {val} | {n[m].max()} |")
    top = np.argsort(np.nan_to_num(db, nan=-99))[::-1]
    picked = []
    for i in top:
        if len(picked) == 5 or db[i] < 3:
            break
        if all(abs(ORDERS[i] - ORDERS[j]) > 40 for j in picked):
            picked.append(i)
    print("Strongest orders: " + (", ".join(f"{ORDERS[i]:.0f}/m ({db[i]:.1f} dB, n={n[i]})" for i in picked)
                                 or "none above 3 dB"))
    return db


def plot(merged, order_db, upm: float, out: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, ink2, muted, grid, surf = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
    series = {"ENGAGED": "#2a78d6", "open-loop": "#eb6834", "swing-up": "#1baf7a"}
    dashes = {"belt teeth": "-", "stepper electrical": "--", "stepper full step": ":"}
    plt.rcParams.update({"font.family": ["Segoe UI", "DejaVu Sans"], "font.size": 9, "axes.edgecolor": "#c3c2b7",
                         "axes.labelcolor": ink2, "xtick.color": muted, "ytick.color": muted,
                         "axes.facecolor": surf, "figure.facecolor": surf, "axes.grid": True, "grid.color": grid,
                         "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    vline = np.linspace(0.001, 0.3, 300)

    a = ax[0]
    for g, w in merged.items():
        m = (w["band"] >= BAND_MIN) & (w["prom"] >= PROM_MIN) & w["steady"]
        a.scatter(w["v"][m] * 1e3, w["f"][m], s=10, color=series[g], alpha=0.45, linewidths=0,
                  label=f"{g} ({m.sum()} windows)")
    for lname, k in lines(upm).items():
        a.plot(vline * 1e3, k * vline, color=ink, lw=1.2, ls=dashes[lname], label=f"{lname} {k:.0f}|v|")
    a.axhspan(27, 32, color=muted, alpha=0.15, lw=0)
    a.set(xlim=(0, 300), ylim=BAND, xlabel="cart speed |v| (mm/s)", ylabel="peak frequency f_pk (Hz)",
          title="Peaked windows: frequency vs speed")
    a.legend(loc="upper right", frameon=False, fontsize=8)

    a = ax[1]
    for g in ("ENGAGED", "open-loop"):
        if g not in merged:
            continue
        w = merged[g]
        cen, med, p90 = [], [], []
        for lo, hi in zip(AMP_V[:-1], AMP_V[1:]):
            m = w["steady"] & (w["v"] >= lo) & (w["v"] < hi)
            if m.sum() >= 10:
                cen.append((lo + hi) / 2 * 1e3)
                med.append(np.median(w["band"][m]))
                p90.append(np.percentile(w["band"][m], 90))
        a.plot(cen, med, color=series[g], lw=2, marker="o", ms=4, label=f"{g} median")
        a.plot(cen, p90, color=series[g], lw=1.2, ls="--", label=f"{g} 90th percentile")
    for (lname, k), y in zip(lines(upm).items(), (0.99, 0.91, 0.83)):
        a.axvline(28 / k * 1e3, color=ink, lw=1, ls=dashes[lname])
        a.annotate(f"{lname} = 28 Hz", (28 / k * 1e3, y), xycoords=("data", "axes fraction"), xytext=(4, 0),
                   textcoords="offset points", va="top", fontsize=7, color=ink2)
    a.set(xlim=(0, 300), ylim=(0, None), xlabel="cart speed |v| (mm/s)", ylabel="15-60 Hz RMS (counts)",
          title="Oscillation amplitude vs speed (steady windows)")
    a.legend(loc="center right", frameon=False, fontsize=8)

    a = ax[2]
    for g, db in order_db.items():
        a.plot(ORDERS, db, color=series[g], lw=1.2, label=g)
    for lname, k in lines(upm).items():
        a.axvline(k, color=ink, lw=1, ls=dashes[lname], label=f"{lname} {k:.0f}/m")
    a.axhline(0, color="#c3c2b7", lw=0.8)
    a.set(xlim=(ORDERS[0], ORDERS[-1]), xlabel="cycles per metre of cart travel (f / |v|)",
          ylabel="encoder power, dB above local median", title="Order spectrum at constant speed (24-34 Hz left out)")
    a.legend(loc="upper right", frameon=False, fontsize=8)
    fig.savefig(out, dpi=130)
    print(f"\nfigure: {out}")


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--out", default="logs/osc_speed.png")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    paths = [pathlib.Path(p) for p in args.runs] or sorted(pathlib.Path("logs").glob("*_balance"))

    merged, orders, sessions, upm = collect(paths)
    print("| session | est_fc (Hz) | " + " | ".join(f"{g} (s)" for g in GROUPS) + " |")
    print("|---|---|" + "---|" * len(GROUPS))
    for name, fc, *secs in sessions:
        print(f"| {name} | {fc} | " + " | ".join(f"{s:.0f}" for s in secs) + " |")
    for g, w in merged.items():
        report(g, w, upm)
    order_db = {g: order_report(g, o, upm) for g, o in orders.items()}
    if not args.no_plot:
        plot(merged, order_db, upm, pathlib.Path(args.out))


if __name__ == "__main__":
    main()
