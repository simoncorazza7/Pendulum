#!/usr/bin/env bash
# PC ONLY, SIMULATION ONLY. Upright-offset estimate for the balance controller (drift identity, carried between
# catches). No Pi, no GPIO, no motor: nothing here moves the cart. A separate method with its own script: it changes
# none of demo.sh, remote.sh, swingup.sh or their code. Method and results: docs/offset_estimator.md.
#
#   ./offset_sim.sh test                       unit checks (py/test_offset.py)
#   ./offset_sim.sh replay [RUN ...] [--cache DIR]   the estimator on logged balance sessions (needs logs/)
#   ./offset_sim.sh centres RUN ... [--cache DIR]    hanging equilibrium between swing-up runs (needs logs/)
#   ./offset_sim.sh handovers [...]            closed loop: the 29 rig hand-overs, Stage 3b (py/offset_study.py)
#   ./offset_sim.sh draws [...]                closed loop: the rig.TEST draws of docs/neuro_sim.md
set -euo pipefail

if [ -r /proc/device-tree/model ] && grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
    echo "offset_sim.sh is PC-only simulation; refusing to run on the Pi." >&2
    exit 1
fi

cd "$(dirname "$0")"
PY=.venv/Scripts/python
[ -x "$PY" ] || PY=.venv/bin/python
[ -x "$PY" ] || { echo "no .venv: python -m venv .venv && $PY -m pip install -r py/requirements.txt" >&2; exit 1; }

cmd="${1:-}"
[ $# -gt 0 ] && shift
case "$cmd" in
    test)      exec "$PY" py/test_offset.py ;;
    replay)    exec "$PY" py/offset_check.py replay "$@" ;;
    centres)   exec "$PY" py/offset_check.py centres "$@" ;;
    handovers) exec "$PY" py/offset_study.py handovers "$@" ;;
    draws)     exec "$PY" py/offset_study.py draws "$@" ;;
    *)         sed -n '2,10p' "$0"; exit 2 ;;
esac
