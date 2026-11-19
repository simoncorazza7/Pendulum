"""ENS3 evolution runs (PC only, simulation; nothing here talks to the Pi).

    .venv/Scripts/python py/neuro_evolve.py bench --inputs 4 --cls t --gens 1000 [--pop 50] [--seed 1]
    .venv/Scripts/python py/neuro_evolve.py rig [--gens 600] [--pop 50] [--seed 1] [--delay 4]

bench  Pasemann's benchmark (py/neuro/bench.py). Each generation draws --k start states from the benchmark domain
       (x0 within +-2.2 m, theta0 within +-0.2 rad, velocities 0), runs t_max 20 s, and scores
       f = P - cost_n N_n - cost_s N_s - cost_F I_F (the paper's eq. 3; its cost values are not published).
rig    The rig model (py/neuro/rig.py): 3 inputs (x, theta, v), command a_max u, start states and plants drawn from
       rig.TRAIN (+-3 deg, +-0.3 rad/s, reference error +-1.5 counts, 4-tick delay), t_max 10 s, and
       f = P - cost_n N_n - cost_s N_s - cost_a IA - cost_du DU (IA: mean |a|; DU: command changes, against chatter).
Networks start empty (no hidden units, no connections), as in the paper. Each run writes
logs/<time>_neuro-<env>-<inputs><class>-s<seed>/ (logs/ is not committed): log.csv (one line per generation),
elite_<gen>.json every --save-every generations, best.json (of those elites, the best on a fixed test: bench, the
40 x 40 grid at 60 s; rig, 200 runs of 20 s drawn from rig.TEST) and summary.json.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import pathlib
import sys
import time
from dataclasses import asdict, dataclass

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np

from neuro import bench, ens3, rig
from neuro.net import Batch, Genome

HERE = pathlib.Path(__file__).resolve().parent


@dataclass(frozen=True)
class Costs:
    n: float                 # per neuron (outputs + hidden)
    s: float                 # per synapse
    f: float                 # per unit of mean |command| (bench: N, rig: m/s^2)
    du: float = 0.0          # rig: per unit of command change rate (1/s)


BENCH_COSTS = Costs(n=0.2, s=0.05, f=0.1)
RIG_COSTS = Costs(n=0.1, s=0.02, f=0.5, du=0.02)


def sizes(pop):
    return (np.array([g.n_neurons() for g in pop], float), np.array([g.n_synapses() for g in pop], float))


def bench_evaluator(K: int, t_max: float, costs: Costs, rng):
    def evaluate(pop, gen):
        x0, th0 = rng.uniform(-2.2, 2.2, K), rng.uniform(-0.2, 0.2, K)
        r = bench.run(Batch(pop), x0, th0, t_max=t_max)
        P, IF = r["perf"].mean(1), r["IF"].mean(1)
        Nn, Ns = sizes(pop)
        return P - costs.n * Nn - costs.s * Ns - costs.f * IF, {"P": P, "cmd": IF, "Nn": Nn, "Ns": Ns,
                                                                 "t_fail": r["t_fail"].mean(1)}
    return evaluate


def rig_evaluator(K: int, t_max: float, costs: Costs, rng, spec: rig.RigSpec, draw: rig.Draw):
    def evaluate(pop, gen):
        ics = rig.draw(rng, K, draw, spec)
        r = rig.run(Batch(pop), ics, spec, t_max=t_max)
        P, IA, DU = r["perf"].mean(1), r["IA"].mean(1), r["DU"].mean(1)
        Nn, Ns = sizes(pop)
        f = P - costs.n * Nn - costs.s * Ns - costs.f * IA - costs.du * DU
        return f, {"P": P, "cmd": IA, "du": DU, "Nn": Nn, "Ns": Ns, "t_fail": r["t_fail"].mean(1)}
    return evaluate


def bench_test(g: Genome) -> tuple[float, dict]:
    X0, TH0, _ = bench.grid(40)
    inside = (np.abs(X0) <= bench.X_LIM) & (np.abs(TH0) <= bench.TH_LIM)
    r = bench.run(Batch([g]), X0, TH0, t_max=60.0)
    ok = int((r["alive"][0] & inside).sum())
    return ok - 0.01 * g.n_synapses(), {"grid_balanced": ok, "grid_inside": int(inside.sum())}


def rig_test(g: Genome, spec: rig.RigSpec, runs: int = 200, seed: int = 12345) -> tuple[float, dict]:
    ics = rig.draw(np.random.default_rng(seed), runs, rig.TEST, spec)
    r = rig.run(Batch([g]), ics, spec, t_max=20.0)
    alive = r["alive"][0]
    end = float(np.median(np.abs(r["end_x"][0][alive]))) if alive.any() else float("nan")
    return float(alive.mean()) - 0.001 * g.n_synapses(), {"survived_20s": float(alive.mean()), "median_end_x": end,
                                                          "IA": float(r["IA"][0].mean()), "DU": float(r["DU"][0].mean())}


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("env", choices=["bench", "rig"])
    ap.add_argument("--inputs", type=int, default=None, help="bench: 2 or 4 (rig: always 3)")
    ap.add_argument("--cls", choices=["t", "s"], default="t")
    ap.add_argument("--gens", type=int, default=1000)
    ap.add_argument("--pop", type=int, default=50)
    ap.add_argument("--k", type=int, default=None, help="start states per generation (bench 2, rig 4)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--rig-inputs", type=int, choices=[3, 4], default=3,
                    help="rig: 3 = x, theta, v (velocity from recurrence); 4 = + the Pi's velocity estimate")
    ap.add_argument("--delay", type=int, default=4, help="rig: command delay in ticks (remote path: 4)")
    ap.add_argument("--net-every", type=int, default=5, help="rig: ticks per network update (5: 100 Hz)")
    ap.add_argument("--save-every", type=int, default=25)
    ap.add_argument("--print-every", type=int, default=50)
    ap.add_argument("--out", default="", help="run directory (default logs/<time>_neuro-...)")
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    prm = ens3.Ens3Params(pop=a.pop)
    if a.env == "bench":
        n_in = a.inputs or 4
        K = a.k or 2
        costs = BENCH_COSTS
        evaluate = bench_evaluator(K, 20.0, costs, rng)
        spec = None
    else:
        n_in = a.rig_inputs
        K = a.k or 4
        costs = RIG_COSTS
        spec = rig.RigSpec(n_in=n_in, net_every=a.net_every, substeps=2)
        draw = rig.Draw(delay_ticks=(a.delay, a.delay))
        evaluate = rig_evaluator(K, 10.0, costs, rng, spec, draw)
    stamp = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    out = pathlib.Path(a.out) if a.out else HERE.parent / "logs" / f"{stamp}_neuro-{a.env}-{n_in}{a.cls}-s{a.seed}"
    out.mkdir(parents=True, exist_ok=True)
    meta = {"env": a.env, "inputs": n_in, "cls": a.cls, "gens": a.gens, "seed": a.seed, "k": K,
            "ens3": asdict(prm), "costs": asdict(costs), "rig_spec": asdict(spec) if spec else None,
            "delay": a.delay if a.env == "rig" else None}
    (out / "run.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")

    log = open(out / "log.csv", "w", newline="", encoding="utf-8")
    wr = csv.writer(log)
    wr.writerow(["gen", "n", "f_best", "f_mean", "P_best", "t_fail_best", "cmd_best", "Nn_best", "Ns_best"])
    elites: list[tuple[int, pathlib.Path]] = []
    t0 = time.time()

    def on_generation(gen, pop, fit, info):
        b = int(np.argmax(fit))
        wr.writerow([gen, len(pop), f"{fit[b]:.4f}", f"{fit.mean():.4f}", f"{info['P'][b]:.4f}",
                     f"{info['t_fail'][b]:.3f}", f"{info['cmd'][b]:.4f}", int(info["Nn"][b]), int(info["Ns"][b])])
        if gen % a.save_every == 0 or gen == a.gens - 1:
            g = pop[b].copy()
            g.meta = {"env": a.env, "gen": gen, "seed": a.seed, "fitness": float(fit[b])}
            if a.env == "rig":
                g.meta.update({"net_every": a.net_every, "delay_ticks": a.delay})
            p = out / f"elite_{gen:05d}.json"
            g.save(p)
            elites.append((gen, p))
        if gen % a.print_every == 0 or gen == a.gens - 1:
            log.flush()
            print(f"gen {gen:5d}  n {len(pop):3d}  f_best {fit[b]:8.3f}  f_mean {fit.mean():8.3f}  "
                  f"t_fail {info['t_fail'][b]:6.2f} s  neurons {int(info['Nn'][b])}  synapses {int(info['Ns'][b])}  "
                  f"{time.time() - t0:6.0f} s", flush=True)

    ens3.evolve(evaluate, n_in, a.cls, prm, a.gens, rng, on_generation)
    log.close()

    # final pick: the elites of the last 40 % of the run, scored on a fixed test set
    cands = [(gen, p) for gen, p in elites if gen >= 0.6 * a.gens] or elites[-5:]
    scored = []
    for gen, p in cands:
        g = Genome.load(p)
        score, rep = bench_test(g) if a.env == "bench" else rig_test(g, rig.RigSpec(n_in=n_in, net_every=a.net_every))
        scored.append((score, gen, p, rep))
        print(f"  test elite gen {gen}: {rep}", flush=True)
    score, gen, p, rep = max(scored, key=lambda s: s[0])
    best = Genome.load(p)
    best.meta.update({"picked_from_gen": gen, "test": rep})
    best.save(out / "best.json")
    summary = {"best_gen": gen, "test": rep, "neurons": best.n_neurons(), "synapses": best.n_synapses(),
               "rows": best.rows(), "seconds": round(time.time() - t0, 1)}
    (out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(f"best: elite of gen {gen}, {rep}")
    for line in best.rows():
        print("  " + line)
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
