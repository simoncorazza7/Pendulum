#!/bin/bash
# Swing-up test: energy pumping from hanging, hand-over to the balance PID, energy removal at the end
# (pendctl --mode balance with balance.swingup=true). HARDWARE ACTION. docs/swingup_notes.md.
# Run it on the Pi from a terminal you can type into:
#     ssh -t pendel@pendel.local      then      cd ~/Pendel-main && ./swingup.sh
# Extra pendctl options may follow, e.g. ./swingup.sh --set swingup.a_pump=0.6
set -e
cd "$(dirname "$0")"

cat <<'TXT'
Swing-up test: HARDWARE ACTION. The pendulum will swing all the way up and may go over the top.
  1. Motor power cut (rocker switch) within reach. Ctrl-C = hard stop (the cart stops, the pendulum keeps swinging).
  2. Nobody inside the pendulum's circle (radius ~0.5 m around the pivot, in its plane), and nothing in it.
  3. Cart roughly in the middle, not on a switch; pendulum hanging; nothing on the track.
  4. A spare 1 A fuse for the VMOT line at hand.
Sequence: calibration swing -> Enter homes -> READY -> Enter starts a swing-up (~15-25 s of pumping, cart
swings ~+-60-100 mm) -> hand-over to balancing -> at the end the energy is pumped back out.
During a run: Enter = bring it down (energy out); q + Enter / Ctrl-C = hard stop.
Limits: pumping 0.75 m/s^2, balance a_max 2 m/s^2, v_max 0.3 m/s, soft limits +-150 mm.
TXT
read -r -p "All four checked? Type yes to start: " ok
[ "$ok" = "yes" ] || { echo "not started"; exit 1; }

make -s build/pendctl                  # set -e: a failed build never runs a stale binary
rc=0
sudo build/pendctl --mode balance --set balance.chip=gpiochip4 --set balance.allow_real_chip=true \
    --set balance.swingup=true "$@" || rc=$?
echo "driver enable pin after the run (op dh = driver off):"
pinctrl get 24 || true
exit $rc
