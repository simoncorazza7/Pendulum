#!/bin/bash
# Stage 0.9 actuator test (pendctl --mode actuator). HARDWARE ACTION: the cart homes from switch to switch, then
# runs a ladder of HARD moves (up to actuator.a_top m/s^2 and actuator.v_top m/s) with the pendulum hanging, and
# touches a switch after every rep (step-loss audit). One run = one block of docs/stage0_9_actuator.md, e.g.
#     ./actuator.sh --set actuator.speeds=0.3                                   (B1: ladder at 0.3 m/s)
#     ./actuator.sh --set actuator.speeds=0.45 --set actuator.accels=6 --set actuator.reps=5   (B5: confirm a rung)
# Run it on the Pi from a terminal you can type into:
#     ssh -t pendel@pendel.local      then      cd ~/Pendel-main && ./actuator.sh ...
# The program lists every rung of the block before homing, and states each rung again before its Enter.
set -e
cd "$(dirname "$0")"

cat <<'TXT'
Stage 0.9 actuator test (pendctl --mode actuator): HARDWARE ACTION, the cart will move HARD.
  1. Motor-only cut (or the rocker: motor AND Pi) within reach. q + Enter or Ctrl-C = controlled stop.
  2. Padding at both hard stops; nothing on the track; hands away from the belt, pulleys and pendulum.
  3. Belt tension, carriage screws and encoder-bracket screws checked.
  4. Spare slow-blow T1A fuse for F2 (VMOT) at hand.
  5. Pendulum mounted and hanging still; cart roughly in the middle, not on a switch (push it by hand).
TXT
read -r -p "All five checked? Type yes to start: " ok
[ "$ok" = "yes" ] || { echo "not started"; exit 1; }
read -r -p "A4988 Vref in V, as measured (Enter = not measured): " vref
case "$vref" in ''|*[!0-9.]*) vref=0 ;; esac

make -s build/pendctl                  # set -e: a failed build never runs a stale binary
rc=0
sudo build/pendctl --mode actuator --set actuator.chip=gpiochip4 --set actuator.allow_real_chip=true \
     --set actuator.vref="$vref" "$@" || rc=$?
echo "driver enable pin after the run (op dh = driver off):"
pinctrl get 24 || true
exit $rc
