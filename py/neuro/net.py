"""Pasemann's additive recurrent neurons (Network 9 (1998) 495, eq. 2) and the genome that ENS3 evolves.

Units, 0-based (the paper counts from 1, so its unit k is id k-1 here):
  inputs   0 .. n_in-1           buffers: their output is the scaled sensor value; nothing connects into them
  outputs  n_in .. n_in+n_out-1  t-class: one, u = tanh(a); s-class: two, u = s(a_o1) - s(a_o2), s = logistic
  hidden   n_in+n_out ..         any number and any wiring: self-loops, loops, skips
Dynamics: a_i(t+1) = b_i + sum_j w_ij o_j(t), with o_j = sigma(a_j) for outputs and hidden units and
o_j = in_j(t) for inputs. All units update at once, so every connection delays its signal by one step.
The command is u in [-1, 1] times the actuator range: 10 N on the benchmark, a_max on the rig.
"""

from __future__ import annotations

import json
import pathlib
from dataclasses import dataclass, field

import numpy as np


def logistic(a):
    """1 / (1 + e^-a), written with tanh so that large |a| cannot overflow."""
    return 0.5 * (1.0 + np.tanh(0.5 * a))


def transfer(cls: str, a):
    return np.tanh(a) if cls == "t" else logistic(a)


@dataclass
class Genome:
    n_in: int
    cls: str = "t"                                  # 't': tanh, one output; 's': logistic, two outputs
    hidden: list = field(default_factory=list)      # ids of hidden units
    bias: dict = field(default_factory=dict)        # unit id -> bias (outputs and hidden units only)
    w: dict = field(default_factory=dict)           # (to, from) -> weight
    meta: dict = field(default_factory=dict)

    @property
    def n_out(self) -> int:
        return 1 if self.cls == "t" else 2

    @property
    def outputs(self) -> list:
        return list(range(self.n_in, self.n_in + self.n_out))

    @property
    def targets(self) -> list:
        """Units with a bias that take connections: the outputs, then the hidden units."""
        return self.outputs + sorted(self.hidden)

    @property
    def units(self) -> list:
        return list(range(self.n_in)) + self.targets

    def new_id(self) -> int:
        return max(self.units) + 1

    def n_neurons(self) -> int:
        """What the neuron cost counts: outputs and hidden units (inputs are buffers)."""
        return self.n_out + len(self.hidden)

    def n_synapses(self) -> int:
        return sum(1 for v in self.w.values() if v != 0.0)

    def copy(self) -> "Genome":
        return Genome(self.n_in, self.cls, list(self.hidden), dict(self.bias), dict(self.w), dict(self.meta))

    def check(self) -> None:
        """Raise ValueError if the structure is inconsistent."""
        tg, un = set(self.targets), set(self.units)
        if len(tg) != len(self.targets) or any(h < self.n_in + self.n_out for h in self.hidden):
            raise ValueError("bad hidden unit ids")
        for (i, j) in self.w:
            if i not in tg:
                raise ValueError(f"connection into unit {i}, an input or unknown unit")
            if j not in un:
                raise ValueError(f"connection from unknown unit {j}")
        for i in self.bias:
            if i not in tg:
                raise ValueError(f"bias on unit {i}, an input or unknown unit")

    def dense(self, size: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(W, b) over the units in the order inputs, outputs, hidden (sorted); zero-padded to size."""
        order = self.units
        idx = {u: k for k, u in enumerate(order)}
        n = len(order) if size is None else size
        W, b = np.zeros((n, n)), np.zeros(n)
        for (i, j), v in self.w.items():
            W[idx[i], idx[j]] = v
        for i, v in self.bias.items():
            b[idx[i]] = v
        return W, b

    def rows(self) -> list[str]:
        """The paper's notation: per output or hidden unit k, the vector (bias, w_k1, ..., w_kn), units numbered
        from 1 in the order inputs, outputs, hidden."""
        order = self.units
        out = []
        for i in self.targets:
            vec = [self.bias.get(i, 0.0)] + [self.w.get((i, j), 0.0) for j in order]
            out.append(f"w_{order.index(i) + 1} = (" + " ".join(f"{v:.2f}" for v in vec) + ")")
        return out

    def to_dict(self) -> dict:
        return {"n_in": self.n_in, "cls": self.cls, "hidden": sorted(self.hidden),
                "bias": {str(k): v for k, v in sorted(self.bias.items())},
                "w": [[i, j, v] for (i, j), v in sorted(self.w.items())], "meta": self.meta}

    @staticmethod
    def from_dict(d: dict) -> "Genome":
        return Genome(int(d["n_in"]), d["cls"], [int(h) for h in d["hidden"]],
                      {int(k): float(v) for k, v in d["bias"].items()},
                      {(int(i), int(j)): float(v) for i, j, v in d["w"]}, dict(d.get("meta", {})))

    def save(self, path) -> None:
        pathlib.Path(path).write_text(json.dumps(self.to_dict(), indent=1), encoding="utf-8")

    @staticmethod
    def load(path) -> "Genome":
        return Genome.from_dict(json.loads(pathlib.Path(path).read_text(encoding="utf-8")))


class Batch:
    """P networks of one class and input count, zero-padded to a common size, run in lockstep K times each.

    Activities have shape (P, K, U). Padded units have no bias and no connections, so they stay silent.
    """

    def __init__(self, genomes: list[Genome]):
        g0 = genomes[0]
        self.n_in, self.cls, self.n_out = g0.n_in, g0.cls, g0.n_out
        self.P = len(genomes)
        self.U = max(len(g.units) for g in genomes)
        self.W = np.zeros((self.P, self.U, self.U))
        self.b = np.zeros((self.P, self.U))
        for p, g in enumerate(genomes):
            if g.n_in != self.n_in or g.cls != self.cls:
                raise ValueError("a batch needs one class and one input count")
            self.W[p], self.b[p] = g.dense(self.U)

    def reset(self, K: int) -> np.ndarray:
        """All activities 0: a fresh engagement (stale activity would be a hidden initial condition)."""
        return np.zeros((self.P, K, self.U))

    def step(self, a: np.ndarray, inp: np.ndarray) -> np.ndarray:
        """a(t+1) from a(t) and the scaled inputs in(t), shape (P, K, n_in) or broadcastable to it."""
        o = transfer(self.cls, a)
        o[..., :self.n_in] = inp
        return self.b[:, None, :] + np.einsum("pij,pkj->pki", self.W, o)

    def command(self, a: np.ndarray) -> np.ndarray:
        """u in [-1, 1] from the output units' activities, shape (P, K)."""
        k = self.n_in
        if self.cls == "t":
            return np.tanh(a[..., k])
        return logistic(a[..., k]) - logistic(a[..., k + 1])


class NetController:
    """One network as a step-by-step controller: reset() at every engagement, step(inputs) -> u in [-1, 1].

    timing 'immediate': the network updates on the new inputs and its new output is used at once (what a
    controller answering each sample does). 'paper': the output used now is the one from before this update, so
    inputs reach the command one step late (Pasemann's simultaneous update of network and cart-pole).
    """

    def __init__(self, g: Genome, timing: str = "immediate"):
        self.batch = Batch([g])
        self.timing = timing
        self.reset()

    def reset(self) -> None:
        self.a = self.batch.reset(1)

    def step(self, inp) -> float:
        x = np.asarray(inp, float).reshape(1, 1, -1)
        if self.timing == "paper":
            u = self.batch.command(self.a)
            self.a = self.batch.step(self.a, x)
        else:
            self.a = self.batch.step(self.a, x)
            u = self.batch.command(self.a)
        return float(u[0, 0])
