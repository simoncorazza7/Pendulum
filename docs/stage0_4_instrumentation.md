# Stage 0.4: instrumentation (2026-10-08)

## What exists
| Piece | Where | Notes |
|---|---|---|
| Ring buffer | `src/rt/ringlog.h` | Allocated and touched in the constructor; `push()` is O(1), with no allocation, lock or I/O. Full → drop newest (default) or overwrite oldest (`log.overwrite`). Drops/overwrites are counted and saved in `meta.json`. |
| CSV writer | `writeCsv()` in the same header | Runs after the loop has stopped. |
| Run directory | `src/common/runlog.*` | `logs/<local time>_<mode>/`: `data.csv`, `meta.json`, `config_used.conf`, `config_source.conf`. |
| Build identity | Makefile `-DGIT_HASH -DGIT_DIRTY` | The commit the binary was **built** from, plus whether tracked files differed. Rebuild after every pull. |
| Config | `src/common/config.*`, `config/pendctl.conf` | `key = value  # unit`; `--set key=value` overrides; every value read (and every default used) is saved; unused keys can be listed. |
| Runtime | `src/app/pendctl.cpp` | Mode `log-selftest` only (no GPIO code). |
| Tests | `tests/unit/` | `make test`, 9/9 pass on the Pi. |
| Python (PC) | `py/` + `.venv` | `fetch_runs.py` (scp from the Pi), `plot_run.py` (plot.png in the run dir), `pendel.load_run()`. Versions pinned in `py/requirements.txt`. |

## Logging conventions
- Raw integers next to SI values (brief §1.7): e.g. `t_ns` (CLOCK_MONOTONIC, ns) next to `t` (s).
  Later: `counts` next to `theta`, `pos_usteps` next to `x`.
- Times are CLOCK_MONOTONIC. The run directory name uses the Pi's local time.
- `logs/` is not committed. Runs are fetched to the PC's `logs/` for analysis.

## First data point (log-selftest, SCHED_OTHER, no load, 500 Hz × 5 s)
Run `2026-10-08T14-28-14_log-selftest`, commit `06ef30df1d16`.

| Quantity | Value |
|---|---|
| Wake-up lateness | median 56.1 µs, p99 57.6 µs, max 74.3 µs |
| Period | mean 2.0000 ms, min 1.9830 ms, max 2.0074 ms |

The ~56 µs floor is the 50 µs default timer slack of SCHED_OTHER threads plus ~6 µs of wake-up latency.
SCHED_FIFO threads have zero slack, so 0.5 should remove most of it. The same slack affects the step
thread's 8 µs sleeps (issue 9).

## Usage
```
# Pi
make && make test
./build/pendctl --mode log-selftest [--set selftest.duration=10]
# PC (repo root)
.venv/Scripts/python py/fetch_runs.py
.venv/Scripts/python py/plot_run.py
```
