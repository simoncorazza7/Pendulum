#!/usr/bin/env bash
# PC ONLY, SIMULATION ONLY. Evolved recurrent neurocontrollers after Pasemann (1998), the ENS3 algorithm.
# No Pi, no GPIO, no motor: nothing here moves the cart. A separate method with its own script: it changes
# none of demo.sh, remote.sh, swingup.sh or their code. Method and results: docs/neuro_sim.md.
#
#   ./neuro_sim.sh test                   unit checks (py/test_neuro.py)
#   ./neuro_sim.sh check [--png FILE]     the paper's published networks on its benchmark (40 x 40 grid)
#   ./neuro_sim.sh evolve bench|rig ...   an ENS3 run (options: py/neuro_evolve.py --help)
#   ./neuro_sim.sh rig NET.json [...]     a saved network on the rig model (--baseline adds the rig's PID)
set -euo pipefail

if [ -r /proc/device-tree/model ] && grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
    echo "neuro_sim.sh is PC-only simulation; refusing to run on the Pi." >&2
    exit 1
fi

cd "$(dirname "$0")"
PY=.venv/Scripts/python
[ -x "$PY" ] || PY=.venv/bin/python
[ -x "$PY" ] || { echo "no .venv: python -m venv .venv && $PY -m pip install -r py/requirements.txt" >&2; exit 1; }

cmd="${1:-}"
[ $# -gt 0 ] && shift
case "$cmd" in
    test)   exec "$PY" py/test_neuro.py ;;
    check)  exec "$PY" py/neuro_check.py published "$@" ;;
    evolve) exec "$PY" py/neuro_evolve.py "$@" ;;
    rig)    exec "$PY" py/neuro_check.py rig "$@" ;;
    *)      sed -n '2,10p' "$0"; exit 2 ;;
esac
