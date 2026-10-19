#!/bin/bash
# Stage 2b: the balance demo with the controller on a PC (pendctl --mode balance, balance.remote=true).
# HARDWARE ACTION: the cart homes, balances and re-centres exactly as in ./demo.sh; only the PID runs on the PC
# (py/remote_ctrl.py), with the constant command delay remote.delay_ticks. docs/remote.md.
# 1. On the PC first:   python py/remote_ctrl.py --host pendel.local
# 2. On the Pi, from a terminal you can type into:
#        ssh -t pendel@pendel.local      then      cd ~/Pendel-main && ./remote.sh
# Extra pendctl options may follow, e.g. ./remote.sh --set remote.delay_ticks=4
set -e
cd "$(dirname "$0")"

cat <<'TXT'
Remote balance (controller on the PC): HARDWARE ACTION, the cart will move.
  1. Motor power cut (rocker switch) within reach. Ctrl-C = controlled stop at any time.
  2. Cart roughly in the middle of the track, not on a switch (push it by hand: the driver is off).
  3. Pendulum hanging; nothing on the track; hands away from the belt and pulleys.
  4. A spare 1 A fuse for the VMOT line at hand.
  5. py/remote_ctrl.py running on the PC (wired), showing "waiting for the Pi".
Limits as ./demo.sh: a_max 2 m/s^2, v_max 0.3 m/s, soft limits +-150 mm. Arming waits for the PC; a run aborts
(controlled stop, on the Pi) if the PC's commands stop arriving. The delay is set by remote.delay_ticks.
TXT
read -r -p "All five checked? Type yes to start: " ok
[ "$ok" = "yes" ] || { echo "not started"; exit 1; }

make -s build/pendctl                  # set -e: a failed build never runs a stale binary
rc=0
sudo build/pendctl --mode balance --set balance.chip=gpiochip4 --set balance.allow_real_chip=true \
    --set balance.remote=true "$@" || rc=$?
echo "driver enable pin after the run (op dh = driver off):"
pinctrl get 24 || true
exit $rc
