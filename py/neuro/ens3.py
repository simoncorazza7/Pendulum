"""ENS3, 'evolution of neural systems by stochastic synthesis' (Pasemann 1998, section 2).

One generation is p(t+1) = S E V p(t):
  V  variation of every network by six stochastic operators, applied in the order N-, N+, N*, C-, C+, C*:
     delete hidden units, insert a hidden unit, perturb biases, delete connections, insert connections (self-loops
     and loops allowed, never into an input), perturb weights. No crossover.
  E  evaluation: every network runs from the same start states, drawn afresh each generation.
  S  selection: each network leaves a Poisson-distributed number of copies whose mean is above 1 when its fitness
     is above the population average and below 1 otherwise, so the population size floats around its target.
The paper gives this structure but not its probabilities, costs or selection steepness; ours are in Ens3Params.
One deviation, stated: the best network of each generation is also carried over unvaried (elitism), so an unlucky
Poisson draw cannot lose it. Starting point as in the paper: networks with no hidden units and no connections.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .net import Genome


@dataclass(frozen=True)
class Ens3Params:
    pop: int = 50                    # target (average) population size
    p_del_neuron: float = 0.02       # N-: per hidden unit
    p_add_neuron: float = 0.03       # N+: per network
    p_bias: float = 0.15             # N*: per output or hidden unit
    sd_bias: float = 0.2
    p_del_conn: float = 0.02         # C-: per connection
    add_conn: float = 0.2            # C+: mean number of new connections per network (Poisson)
    p_weight: float = 0.25           # C*: per connection
    sd_weight: float = 0.3           # perturbation: normal, sd = sd_weight + rel_weight |w|
    rel_weight: float = 0.1
    sd_new: float = 2.0              # first weight of a new connection
    w_lim: float = 50.0
    b_lim: float = 20.0
    beta: float = 1.0                # selection steepness on the z-scored fitness
    elite: int = 1                   # best networks carried over unvaried
    max_pop_factor: float = 3.0      # hard cap on the population, x pop


def mutate(g: Genome, prm: Ens3Params, rng: np.random.Generator) -> Genome:
    g = g.copy()
    for h in list(g.hidden):                                            # N-
        if rng.random() < prm.p_del_neuron:
            g.hidden.remove(h)
            g.bias.pop(h, None)
            g.w = {k: v for k, v in g.w.items() if h not in k}
    if rng.random() < prm.p_add_neuron:                                 # N+
        h = g.new_id()
        g.hidden.append(h)
        src = int(rng.choice(g.units))
        dst = int(rng.choice(g.targets))
        g.w[(h, src)] = rng.normal(0.0, prm.sd_new)
        g.w[(dst, h)] = rng.normal(0.0, prm.sd_new)
    for i in g.targets:                                                 # N*
        if rng.random() < prm.p_bias:
            g.bias[i] = float(np.clip(g.bias.get(i, 0.0) + rng.normal(0.0, prm.sd_bias), -prm.b_lim, prm.b_lim))
    for key in list(g.w):                                               # C-
        if rng.random() < prm.p_del_conn:
            del g.w[key]
    for _ in range(rng.poisson(prm.add_conn)):                          # C+
        i, j = int(rng.choice(g.targets)), int(rng.choice(g.units))
        if (i, j) not in g.w:
            g.w[(i, j)] = rng.normal(0.0, prm.sd_new)
    for key, v in list(g.w.items()):                                    # C*
        if rng.random() < prm.p_weight:
            sd = prm.sd_weight + prm.rel_weight * abs(v)
            g.w[key] = float(np.clip(v + rng.normal(0.0, sd), -prm.w_lim, prm.w_lim))
    return g


def offspring(fitness: np.ndarray, prm: Ens3Params, rng: np.random.Generator) -> np.ndarray:
    """Poisson copy numbers: mean pop * w_i / sum(w), w_i = exp(beta z_i), z = z-scored fitness."""
    f = np.asarray(fitness, float)
    sd = f.std()
    z = (f - f.mean()) / sd if sd > 0 else np.zeros_like(f)
    w = np.exp(prm.beta * np.clip(z, -4.0, 4.0))
    counts = rng.poisson(prm.pop * w / w.sum())
    cap = int(prm.max_pop_factor * prm.pop)
    while counts.sum() > cap:                       # trim at random, one copy at a time
        i = rng.choice(np.flatnonzero(counts))
        counts[i] -= 1
    if counts.sum() == 0:
        counts[int(np.argmax(f))] = 1
    return counts


def evolve(evaluate, n_in: int, cls: str, prm: Ens3Params, gens: int, rng: np.random.Generator,
           on_generation=None, start: list | None = None) -> list:
    """Run gens generations. evaluate(networks, gen) -> (fitness array, info dict of arrays).

    on_generation(gen, networks, fitness, info): called after every evaluation. Returns the last population.
    """
    pop = [g.copy() for g in start] if start else [Genome(n_in, cls) for _ in range(prm.pop)]
    keep: set = set()
    for gen in range(gens):
        varied = [g if i in keep else mutate(g, prm, rng) for i, g in enumerate(pop)]
        fit, info = evaluate(varied, gen)
        fit = np.asarray(fit, float)
        if on_generation is not None:
            on_generation(gen, varied, fit, info)
        order = np.argsort(-fit)
        nxt = [varied[i] for i in order[:prm.elite]]
        keep = set(range(len(nxt)))
        for i, c in enumerate(offspring(fit, prm, rng)):
            nxt.extend(varied[i] for _ in range(int(c)))   # shared objects are fine: mutate() copies
        pop = nxt
    return pop
