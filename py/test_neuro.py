"""Checks for py/neuro (the ENS3 simulation code). PC only, no hardware.

    .venv/Scripts/python py/test_neuro.py
Prints PASS/FAIL per check; exit code 1 on failure.
"""

from __future__ import annotations

import math
import pathlib
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from neuro import bench, ens3, rig
from neuro.net import Batch, Genome, NetController, logistic, transfer
from neuro.published import TABLE, published
from pendel.control import PidParams, PidState, pid_step
from pendel.plant import ActuatorParams, PlantParams, SensorParams, measure, pendulum_accel, realised_accel

failures = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    failures += 0 if ok else 1


def random_genome(rng, n_in=2, cls="t", hidden=3, conns=12) -> Genome:
    g = Genome(n_in, cls)
    for _ in range(hidden):
        g.hidden.append(g.new_id())
    for i in g.targets:
        g.bias[i] = rng.normal()
    for _ in range(conns):
        g.w[(int(rng.choice(g.targets)), int(rng.choice(g.units)))] = rng.normal(0, 2)
    return g


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    rng = np.random.default_rng(0)

    # 1. the paper's cart-pole, linearised at upright: theta'' = M g/(l (4/3 M - mp)) theta - F/(l (4/3 M - mp))
    den = bench.L * (4 / 3 * bench.M - bench.MP)
    _, k_th = bench.accel(np.array(1e-7), np.array(0.0), np.array(0.0))
    _, k_f = bench.accel(np.array(0.0), np.array(0.0), np.array(1.0))
    check("benchmark: theta'' = 15.79 theta - 1.463 F near upright",
          abs(k_th / 1e-7 - bench.M * bench.G / den) < 1e-4 and abs(k_f + 1 / den) < 1e-9,
          f"({k_th / 1e-7:.3f}, {float(k_f):.4f})")

    # 2. the rig model mirrors pendel.plant (the validated Stage 0.12 model) to rounding
    spec = rig.RigSpec()
    err_acc = err_cap = err_meas = 0.0
    for _ in range(300):
        th, thd, a, v = rng.uniform(-0.5, 0.5), rng.uniform(-2, 2), rng.uniform(-20, 20), rng.uniform(-5, 5)
        w0, bJ, c = rng.uniform(4, 6), rng.uniform(0, 0.1), rng.uniform(0, 0.3)
        ref = rng.uniform(-0.01, 0.01)
        pp = PlantParams(w0=w0, bJ=bJ, coulomb=c)
        ap = ActuatorParams(axis_a=spec.axis_a, axis_v=spec.axis_v)
        sp = SensorParams(cpr=spec.cpr, phase=spec.phase, ref_error=ref)
        err_acc = max(err_acc, abs(float(rig.pend_accel(th, thd, a, w0, bJ, c)) - pendulum_accel(th, thd, a, pp)))
        err_cap = max(err_cap, abs(float(rig.axis_cap(a, v, spec)) - realised_accel(a, v, ap)))
        err_meas = max(err_meas, abs(float(rig.measure(th, spec, ref)) - measure(th, sp)[1]))
    check("rig model = pendel.plant (pendulum_accel, realised_accel, measure)",
          max(err_acc, err_cap, err_meas) < 1e-12, f"max error {max(err_acc, err_cap, err_meas):.1e}")

    # 3. the rig model's command limit = pid_step's saturation and v_max taper
    p = PidParams(kp=1.0, ki=0.0, kd=0.0, a_max=spec.a_max, v_max=spec.v_max, v_taper=spec.v_taper)
    err = 0.0
    for _ in range(300):
        u, v = rng.uniform(-4, 4), rng.uniform(-0.4, 0.4)
        a_pid, _, _ = pid_step(p, PidState(prev_theta=u), u, v, 0.002)
        err = max(err, abs(a_pid - float(rig.limit(u, v, spec))))
    check("rig model limit = pid_step saturation with the v_max taper", err < 1e-12, f"max error {err:.1e}")

    # 3b. the rig model's 4th input is pendel.estimators.BdLp2 exactly: same counts in, same estimate out
    from pendel.estimators import BdLp2
    b0, b1, b2, a1, a2 = rig.bdlp2(spec.ts, spec.est_fc)
    est = BdLp2(spec.ts, spec.est_fc)
    q = 2 * math.pi / spec.cpr
    t = np.arange(400) * spec.ts
    theta = 0.05 * np.sin(2 * math.pi * 1.3 * t) + 0.01 * np.sin(2 * math.pi * 9 * t)
    counts = np.floor(theta / q + spec.phase)
    ref_y, mine = [], []
    xs = [0.0, 0.0]
    ys = [0.0, 0.0]
    for k, c in enumerate(counts):
        ref_y.append(est.update(t[k], int(c), []))
        xin = 0.0 if k == 0 else (c - counts[k - 1]) * q / spec.ts
        y = b0 * xin + b1 * xs[0] + b2 * xs[1] - a1 * ys[0] - a2 * ys[1]
        xs, ys = [xin, xs[0]], [y, ys[0]]
        mine.append(y)
    check("rig model omega input = pendel.estimators.BdLp2 (est_fc 14 Hz)",
          float(np.max(np.abs(np.array(ref_y) - np.array(mine)))) < 1e-12)

    # 4. Batch.step is the paper's eq. (2), computed by hand
    ok = True
    for cls in ("t", "s"):
        g = random_genome(rng, n_in=3, cls=cls)
        a = rng.normal(size=len(g.units))
        inp = rng.normal(size=3)
        order = g.units
        o = {u: (inp[k] if k < 3 else transfer(cls, a[k])) for k, u in enumerate(order)}
        hand = np.array([0.0] * 3 + [g.bias.get(i, 0.0) + sum(v * o[j] for (ii, j), v in g.w.items() if ii == i)
                                     for i in order[3:]])
        b = Batch([g])
        got = b.step(a.reshape(1, 1, -1), inp.reshape(1, 1, -1))[0, 0]
        ok &= bool(np.allclose(got[3:], hand[3:], atol=1e-12))
    check("Batch.step = eq. (2) by hand (t and s class)", ok)

    # 5. eq. (4): tanh(x) = s(2x) - s(-2x), so the s-class twin of w1 (w5 = 2 w1, w6 = -2 w1) gives the same run
    g1 = published("w1")
    gs = Genome(4, "s")
    for (i, j), v in g1.w.items():
        gs.w[(4, j)], gs.w[(5, j)] = 2 * v, -2 * v
    r1 = bench.run(Batch([g1]), [1.0], [0.1], t_max=10.0, trace=(0, 0))
    r2 = bench.run(Batch([gs]), [1.0], [0.1], t_max=10.0, trace=(0, 0))
    dF = float(np.max(np.abs(r1["trace"]["F"] - r2["trace"]["F"])))
    check("eq. (4): w1 and its s-class twin give the same forces", dF < 1e-9, f"max |dF| {dF:.1e} N")

    # 6. mutation keeps the structure valid (no connection into an input), JSON round trip is exact
    prm = ens3.Ens3Params(p_add_neuron=0.3, p_del_neuron=0.1, add_conn=1.5, p_del_conn=0.1)
    g = Genome(3, "s")
    bad = 0
    for _ in range(3000):
        g = ens3.mutate(g, prm, rng)
        try:
            g.check()
        except ValueError:
            bad += 1
        W, _ = g.dense()
        bad += int(np.any(W[:g.n_in] != 0))
    with tempfile.TemporaryDirectory() as d:
        g.save(pathlib.Path(d) / "g.json")
        h = Genome.load(pathlib.Path(d) / "g.json")
    same = h.w == g.w and h.bias == g.bias and h.hidden == sorted(g.hidden) and h.cls == g.cls
    check("3000 mutations: structure valid, inputs never targeted; JSON round trip", bad == 0 and same,
          f"final size {g.n_neurons()} neurons, {g.n_synapses()} synapses")

    # 7. selection keeps the population near its target on average
    totals = [ens3.offspring(rng.normal(size=60), ens3.Ens3Params(pop=50), rng).sum() for _ in range(2000)]
    check("Poisson selection: mean population = target", abs(np.mean(totals) - 50) < 1.0,
          f"mean {np.mean(totals):.2f}")

    # 8. the paper's results: w1 centres from (-2.0 m, -0.18 rad); w4 balances from (-1.9 m, -0.18 rad)
    r = bench.run(Batch([published("w1")]), [-2.0], [-0.18], t_max=30.0, trace=(0, 0))
    x_end = abs(r["trace"]["x"][-1])
    check("w1 balances 30 s from the paper's extreme start and centres the cart", bool(r["alive"][0, 0]) and x_end < 0.01,
          f"|x| at 30 s {x_end * 1000:.1f} mm")
    r = bench.run(Batch([published("w4")]), [-1.9], [-0.18], t_max=30.0)
    check("w4 (x and theta only) balances 30 s from the paper's extreme start", bool(r["alive"][0, 0]))
    r = bench.run(Batch([published("w4")]), [-1.9], [-0.18], t_max=30.0, integrator="explicit")
    check("... and falls with explicit Euler (its damping is that small)", not bool(r["alive"][0, 0]),
          f"fell at {r['t_fail'][0, 0]:.2f} s")

    # 9. NetController (one network, step by step) = Batch, for every published network
    ok = True
    for name in TABLE:
        g = published(name)
        c = NetController(g)
        b = Batch([g])
        a = b.reset(1)
        for _ in range(50):
            inp = rng.normal(size=g.n_in)
            a = b.step(a, inp.reshape(1, 1, -1))
            ok &= abs(c.step(inp) - float(b.command(a)[0, 0])) < 1e-12
    check("NetController = Batch step by step (w1-w9)", ok)

    # 10. logistic written with tanh: exact and overflow-free
    z = np.array([-1000.0, -5.0, 0.0, 3.0, 1000.0])
    ref = np.array([0.0, 1 / (1 + math.exp(5)), 0.5, 1 / (1 + math.exp(-3)), 1.0])
    check("logistic(a) = 1/(1+e^-a), no overflow", bool(np.allclose(logistic(z), ref, atol=1e-15)))

    # 11. rig model: no network = no command, so the pole falls; the run ends at the abort angle
    ics = {"th0": np.array([math.radians(2)]), "thd0": np.zeros(1), "x0": np.zeros(1), "v0": np.zeros(1),
           "ref": np.zeros(1), "w0": np.array([4.97]), "bJ": np.array([0.054]), "coulomb": np.array([0.168]),
           "delay": np.array([4])}
    r = rig.run(Batch([Genome(3, "t")]), ics, spec, t_max=5.0)
    t_f = float(r["t_fail"][0, 0])
    t_free = math.acosh(math.radians(22) / math.radians(2)) / 4.97      # theta0 cosh(w0 t), no friction
    check("rig model: an empty network lets the pole fall in about acosh(22/2)/w0",
          not r["alive"][0, 0] and abs(t_f - t_free) < 0.1, f"{t_f:.2f} s vs {t_free:.2f} s")

    # 12. remote-path late replies hold the applied command; without them it changes every tick
    spec1 = rig.RigSpec(n_in=3, net_every=1)
    g = Genome(3, "t")
    g.w[(3, 0)], g.w[(3, 1)] = 1.0, 3.0              # x and theta: the command moves every tick once the cart does
    base = {"th0": np.array([0.01]), "thd0": np.zeros(1), "x0": np.zeros(1), "v0": np.zeros(1), "ref": np.zeros(1),
            "w0": np.array([4.97]), "bJ": np.array([0.054]), "coulomb": np.array([0.0]), "delay": np.array([4])}
    r0 = rig.run(Batch([g]), base, spec1, t_max=0.4, trace=(0, 0))
    r1 = rig.run(Batch([g]), dict(base, stall_rate=200.0, stall_ticks=(1, 3), stall_seed=5), spec1, t_max=0.4,
                 trace=(0, 0))
    rep0 = float(np.mean(np.diff(r0["trace"]["a_app"][10:]) == 0))
    rep1 = float(np.mean(np.diff(r1["trace"]["a_app"][10:]) == 0))
    check("late replies hold the applied command (stall_rate)", rep0 < 0.05 and rep1 > 0.3,
          f"held ticks {100 * rep0:.0f} % -> {100 * rep1:.0f} %")

    # 13. the networks kept in the repo load and are valid; the benchmark one still balances the paper's extreme start
    nets_dir = pathlib.Path(__file__).resolve().parent / "neuro" / "nets"
    nets = sorted(nets_dir.glob("*.json"))
    ok = {"bench_4t", "rig4_budget_seeded"} <= {f.stem for f in nets}
    for f in nets:
        try:
            Genome.load(f).check()
        except (ValueError, KeyError):
            ok = False
    r = bench.run(Batch([Genome.load(nets_dir / "bench_4t.json")]), [-2.0], [-0.18], t_max=30.0)
    check("py/neuro/nets: every kept network loads; bench_4t balances from (-2.0 m, -0.18 rad)",
          ok and bool(r["alive"][0, 0]), f"{len(nets)} networks")

    print(f"\n{'all passed' if failures == 0 else f'{failures} FAILED'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
