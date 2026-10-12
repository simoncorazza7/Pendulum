#!/bin/bash
# Stage 1 demonstration: closed-loop balance on the rig (pendctl --mode balance). HARDWARE ACTION:
# the cart homes from switch to switch, balances the pendulum and re-centres between runs.
# Run it on the Pi from a terminal you can type into:
#     ssh -t pendel@pendel.local      then      cd ~/Pendel-main && ./demo.sh
# Extra pendctl options may follow, e.g. ./demo.sh --set encoder.trim=0.3
# Procedure, what to expect and what the messages mean: docs/stage1_demo.md.
set -e
cd "$(dirname "$0")"

cat <<'TXT'
Stage 1 balance demo (pendctl --mode balance): HARDWARE ACTION, the cart will move.
  1. Motor power cut (rocker switch) within reach. Ctrl-C = controlled stop at any time.
  2. Cart roughly in the middle of the track, not on a switch (push it by hand: the driver is off).
  3. Pendulum hanging; nothing on the track; hands away from the belt and pulleys.
  4. A spare 1 A fuse for the VMOT line at hand.
Limits: a_max 2 m/s^2, v_max 0.3 m/s, soft limits +-150 mm about the track centre.
TXT
read -r -p "All four checked? Type yes to start: " ok
[ "$ok" = "yes" ] || { echo "not started"; exit 1; }

make -s build/pendctl                  # set -e: a failed build never runs a stale binary
rc=0
sudo build/pendctl --mode balance --set balance.chip=gpiochip4 --set balance.allow_real_chip=true "$@" || rc=$?
echo "driver enable pin after the run (op dh = driver off):"
pinctrl get 24 || true
exit $rc
