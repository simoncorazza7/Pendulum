#!/bin/bash
# Bring-up test of the new carrier PCB (pendctl --mode pcb-test; docs/pcb_bringup.md). One stage per run:
#     ./pcbtest.sh C      GPIO shorts and solder bridges, supply under load   (nothing plugged in, F2 out)
#     ./pcbtest.sh D      limit switches and encoder                          (J3 + J4 plugged in, F2 out)
#     ./pcbtest.sh E      MOTOR MOVES: hold, direction, step modes, span      (J9 + F2 in)
# Do stages A and B of docs/pcb_bringup.md (multimeter, before the Pi goes on the board) first.
# Run it on the Pi from a terminal you can type into:  ssh -t pendel@pendel.local, then  cd ~/Pendel-main && ./pcbtest.sh C
set -e
cd "$(dirname "$0")"
stage=${1:-}
case "$stage" in
C) cat <<'TXT'
PCB test, stage C: GPIO lines and supply. Every header GPIO is driven for a fraction of a millisecond; nothing moves.
  1. Stages A and B of docs/pcb_bringup.md passed (no shorts; 5 V rail 4.90-5.25 V with the Pi OFF the board).
  2. The Pi is on the board and powered ONLY through it (no USB-C cable).
  3. J3 (switches), J4 (encoder) and J9 (motor) are EMPTY.
  4. Motor fuse F2 is REMOVED (no motor voltage). F1 is in.
  5. A multimeter at hand (5 V / 3.3 V / Vref readings are asked for).
TXT
;;
D) cat <<'TXT'
PCB test, stage D: limit switches and encoder. Nothing moves; you press the switches and turn the pendulum.
  1. Stage C passed.
  2. Power was OFF while you plugged the switches into J3 and the encoder into J4.
  3. Motor fuse F2 still REMOVED, nothing in J9.
TXT
;;
E) cat <<'TXT'
PCB test, stage E: HARDWARE ACTION, the motor drives the cart (at most 20 mm/s).
  1. Stage D passed (the program checks the newest stage D log).
  2. Power was OFF while you plugged the motor into J9 and put F2 back. Never plug or unplug the motor with power on.
  3. Cart roughly in the middle of the track, belt on, nothing on the track, pendulum hanging.
  4. Hands clear of the belt and pulleys; the motor power (rocker) switch within reach. Ctrl-C = stop, driver off.
TXT
;;
*) echo "usage: ./pcbtest.sh C|D|E   (docs/pcb_bringup.md)"; exit 2 ;;
esac
read -r -p "All checked? Type yes to start: " ok
[ "$ok" = "yes" ] || { echo "not started"; exit 1; }

make -s build/pendctl                  # set -e: a failed build never runs a stale binary
rc=0
sudo build/pendctl --mode pcb-test --set pcbtest.chip=gpiochip4 --set pcbtest.allow_real_chip=true \
    --set pcbtest.stage="$stage" "${@:2}" || rc=$?
echo "driver enable pin after the run (op dh = driver off):"
pinctrl get 24 || true
exit $rc
