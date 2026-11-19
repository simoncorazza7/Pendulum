"""The controllers published in Pasemann (1998), eqs (5)-(10) and (12)-(14), as genomes.

Read from the PDF's text layer with its character spacing and kerning honoured: the plain text runs numbers
together ("15 :119 :97"), and only the spacing says that w1 holds 15.1 and 19.97 rather than 15.11 and 9.97.
The paper writes each output or hidden unit k as the vector (bias, w_k1, ..., w_kn) over the units 1..n,
numbered inputs first, then outputs, then hidden units. A zero entry is no connection.
"""

from __future__ import annotations

from .net import Genome

# name: (inputs, class, {paper unit k: [bias, w_k1, ..., w_kn]}, what the paper reports)
TABLE = {
    "w1": (4, "t", {5: [0.0, 10.11, 15.1, 19.97, 2.99]},
           "4 inputs, no hidden unit: centres the cart within 10 s, no oscillation"),
    "w2": (4, "s", {5: [-0.22, 8.85, 11.99, 17.54, 3.02, 0.0, 0.0],
                    6: [-0.22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
           "4 inputs, s-class with w1's wiring; only part of the force range"),
    "w3": (4, "s", {5: [-2.94, 10.26, 38.0, 34.27, 8.42, -3.94, 0.0],
                    6: [-3.35, 0.0, -4.1, 0.0, 0.0, 0.0, 5.58]},
           "4 inputs, s-class with self-connections on both outputs"),
    "w4": (2, "t", {3: [0.0, 0.15, 4.59, -0.95]},
           "x and theta only, no hidden unit, inhibitory self-loop: balances, cart keeps oscillating"),
    "w5": (2, "t", {3: [0.0, -0.1, 7.56, -0.82, -0.37, 0.0],
                    4: [0.0, -17.31, 0.0, 0.0, 0.75, -10.93],
                    5: [0.0, -0.08, 0.0, 0.0, 0.0, 0.95]},
           "x and theta, 2 hidden units (balancing + cart-centring modules): centres in < 60 s, best of all"),
    "w6": (2, "t", {3: [0.0, 0.0, 3.16, 0.32, -0.27],
                    4: [0.0, -9.35, 0.0, 10.81, 3.56]},
           "x and theta, 1 hidden unit: 'erratic' forces near the centre that still balance"),
    "w7": (2, "s", {3: [2.72, 21.3, 49.62, 3.83, 0.0],
                    4: [-2.95, 0.0, -10.55, 10.5, -4.14]},
           "x and theta, s-class: cart oscillation damped to a constant amplitude"),
    "w8": (2, "s", {3: [-11.72, 0.0, 30.11, 0.0, 0.0, 25.84],
                    4: [-4.38, 0.0, -13.82, 16.17, -7.11, 0.0],
                    5: [-3.74, 37.69, 35.11, 0.0, 0.0, 0.0]},
           "x and theta, s-class, 1 hidden unit: cart almost at rest, pole oscillates forever"),
    "w9": (2, "s", {3: [-2.74, 0.0, 23.85, -9.79, 14.69],
                    4: [-2.29, -10.05, -35.6, -10.71, 12.08]},
           "x and theta, s-class, chaotic output module: best s-class benchmark"),
}


def published(name: str) -> Genome:
    n_in, cls, rows, note = TABLE[name]
    g = Genome(n_in, cls, meta={"name": name, "source": "Pasemann 1998, Network 9 495-511", "note": note})
    first_hidden = n_in + g.n_out + 1                  # paper numbering
    for k, vec in rows.items():
        i = k - 1
        if k >= first_hidden:
            g.hidden.append(i)
        if vec[0] != 0.0:
            g.bias[i] = vec[0]
        for j, v in enumerate(vec[1:]):                # paper unit j+1 = id j
            if v != 0.0:
                g.w[(i, j)] = v
    g.hidden.sort()
    g.check()
    return g


def all_published() -> dict[str, Genome]:
    return {name: published(name) for name in TABLE}
