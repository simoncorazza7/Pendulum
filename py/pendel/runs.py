"""Load pendctl run directories (data.csv, meta.json, config_used.conf)."""

from __future__ import annotations

import json
import pathlib
from dataclasses import dataclass

import numpy as np


@dataclass
class Run:
    path: pathlib.Path
    meta: dict          # meta.json: build, host, kernel, command line, run results (all strings)
    config: dict        # config_used.conf: key -> value string (units are in config_source.conf)
    data: np.ndarray    # structured array; field names = CSV header

    def __getitem__(self, column: str) -> np.ndarray:
        return self.data[column]

    @property
    def columns(self) -> tuple[str, ...]:
        return self.data.dtype.names

    def cfg(self, key: str) -> float:
        """Numeric config value used by the run."""
        return float(self.config[key])


def parse_conf(text: str) -> dict[str, str]:
    """Parse 'key = value  # comment  (default)' lines into {key: value}."""
    out = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0]
        for marker in ("(default)", "(override)"):
            line = line.replace(marker, "")
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def load_run(path: str | pathlib.Path) -> Run:
    p = pathlib.Path(path)
    meta = json.loads((p / "meta.json").read_text(encoding="utf-8"))
    config = parse_conf((p / "config_used.conf").read_text(encoding="utf-8"))
    data = np.genfromtxt(p / "data.csv", delimiter=",", names=True, dtype=None, encoding="utf-8")
    return Run(p, meta, config, np.atleast_1d(data))


def latest_run(base: str | pathlib.Path = "logs", mode: str | None = None) -> pathlib.Path:
    """Most recent run directory under base (names start with a sortable timestamp)."""
    runs = sorted(d for d in pathlib.Path(base).iterdir() if d.is_dir() and (d / "meta.json").exists())
    if mode:
        runs = [d for d in runs if d.name.endswith("_" + mode) or ("_" + mode + "_") in d.name]
    if not runs:
        raise FileNotFoundError(f"no runs in {base}" + (f" for mode {mode}" if mode else ""))
    return runs[-1]
