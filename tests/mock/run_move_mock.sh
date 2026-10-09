#!/bin/bash
# pendctl move mode on the gpio-mockup chip (never the real chip). Needs gpio-mockup loaded and sudo.
set -u
cd "$(dirname "$0")/../.."
MOCK=$(gpiodetect | awk '/\[gpio-mockup-A\]/{print $1}')
[ -n "$MOCK" ] || { echo "gpio-mockup not loaded"; exit 1; }
DBG=/sys/kernel/debug/gpio-mockup/$MOCK
setin() { echo "$2" | sudo tee "$DBG/$1" >/dev/null; }
run() {
    sudo ./build/pendctl --mode move --set move.chip="$MOCK" "$@" > build/move_mock.out 2>&1
    local rc=$?
    grep -E "^move: |refusing|ABORT" build/move_mock.out | tail -2
    echo "   exit=$rc"
}
make -s build/pendctl || { echo "build failed"; exit 1; }
setin 2 1; setin 9 1; setin 10 0; setin 19 0          # switches free, encoder lines low

echo "== M1: 3 cm out and back (expect completed, net 0)"
run
echo "== M2: 5 cm out, no return (expect net +667 usteps)"
run --set move.distance=0.05 --set move.return=false
echo "== M3: switch approach, left switch closes 1.5 s into a 20 cm move (expect switch reached, braked)"
(sleep 1.5; setin 9 0) & run --set move.distance=0.2 --set move.stop_at_switch=true --set move.return=false
wait; setin 9 1
echo "== M4: unexpected right switch during a move (expect ABORT, exit 3)"
(sleep 1.5; setin 2 0) & run --set move.distance=0.2 --set move.return=false
wait; setin 2 1
echo "== M5: typo distance 1.5 m (expect refused, exit 2)"
run --set move.distance=1.5
echo "== M6: switch pressed at start (expect refused)"
setin 9 0; run; setin 9 1
echo "== M7: SIGINT mid-move (expect ABORT: signal)"
(sleep 1.5; sudo pkill -INT -x pendctl) & run --set move.distance=0.2 --set move.return=false
wait
sudo chown -R "$(id -un)":"$(id -gn)" logs
