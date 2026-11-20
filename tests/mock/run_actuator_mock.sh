#!/bin/bash
# pendctl actuator mode (Stage 0.9 actuator test) on the gpio-mockup chip (never the real chip). Needs gpio-mockup
# loaded and sudo. Enter / s / q go through a FIFO on stdin; the switches are pressed by this script at the times a
# cart at balance.v_seek would reach them. The mock cart moves nothing, so these check the plumbing (refusals,
# homing, driver enable, stdin, signals, logs), not the physics (tests/unit/test_actuator.cpp).
set -u
cd "$(dirname "$0")/../.."
MOCK=$(gpiodetect | awk '/\[gpio-mockup-A\]/{print $1}')
[ -n "$MOCK" ] || { echo "gpio-mockup not loaded"; exit 1; }
DBG=/sys/kernel/debug/gpio-mockup/$MOCK
TR=/sys/kernel/tracing
BASE=$(sudo cat /sys/kernel/debug/gpio | awk -v c="$MOCK:" '$1==c {split($3,r,"-"); print r[1]}')
EN=24; STEP=27; ENC_A=10; ENC_B=19; LIM_R=2; LIM_L=9
trace_start() {
    echo 0 | sudo tee $TR/tracing_on >/dev/null
    echo "gpio >= $BASE" | sudo tee $TR/events/gpio/gpio_value/filter >/dev/null
    echo 1 | sudo tee $TR/events/gpio/gpio_value/enable >/dev/null
    echo | sudo tee $TR/trace >/dev/null
    echo 1 | sudo tee $TR/tracing_on >/dev/null
}
trace_stop() {
    echo 0 | sudo tee $TR/events/gpio/gpio_value/enable >/dev/null
    echo 0 | sudo tee $TR/events/gpio/gpio_value/filter >/dev/null
}
sets() { # values set on line $1 since trace_start, e.g. "1 1 0 1"
    sudo cat $TR/trace | awk -v g=$((BASE + $1)) '{for(i=1;i<NF;i++) if($i=="gpio_value:" && $(i+1)==g && $(i+2)=="set") printf "%s ", $(i+3)}' | sed 's/ $//'
}
setin() { echo "$2" | sudo tee "$DBG/$1" >/dev/null; }
reset_inputs() { setin $LIM_R 1; setin $LIM_L 1; setin $ENC_A 0; setin $ENC_B 0; }
pass=0; fail=0
check() { # name expected actual
    if [ "$2" == "$3" ]; then echo "  PASS $1 [$3]"; pass=$((pass+1))
    else echo "  FAIL $1: expected [$2], got [$3]"; fail=$((fail+1)); fi
}
OUT=build/actuator_mock.out
FIFO=build/actuator_in
lastdir() { ls -td logs/*_actuator | head -1; }
meta() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])" "$(lastdir)/meta.json" "$1"; }
start() { # overrides...: pendctl in the background, stdin from the FIFO (held open by a sleeper)
    rm -f $FIFO; mkfifo $FIFO
    sleep 300 > $FIFO & HOLD=$!
    sudo timeout -s INT 120 stdbuf -oL ./build/pendctl --mode actuator --set actuator.chip=$MOCK "$@" < $FIFO > $OUT 2>&1 &
    PID=$!
}
finish() { wait $PID; RC=$?; kill $HOLD 2>/dev/null; wait $HOLD 2>/dev/null; rm -f $FIFO; }
say() { echo "$1" > $FIFO; }
waitfor() { # pattern, timeout s: 0 when it appeared in $OUT
    local i
    for ((i = 0; i < $2 * 10; i++)); do grep -q "$1" $OUT 2>/dev/null && return 0; sleep 0.1; done
    echo "    (timeout waiting for '$1')"; return 1
}
home() { # Enter, then the switches at the times a cart at balance.v_seek reaches them (span 6104 usteps)
    waitfor 'Press Enter to HOME' 10
    sleep 0.5; say ""
    waitfor 'seeking the RIGHT' 5
    sleep 2; setin $LIM_R 0
    waitfor 'seeking the LEFT' 5
    sleep 0.2; setin $LIM_R 1
    # from rest: accelerates for T0 = 1.264 s (half of it lost: 0.63 s), then 533 usteps/s: 6104 usteps ~ 12.1 s
    sleep 11.9; setin $LIM_L 0
    waitfor 'step mode OK' 5
    sleep 0.2; setin $LIM_L 1
}

make -s build/pendctl || { echo "build failed"; exit 1; }   # the recipe deletes the binary before linking

echo "== A1 left switch pressed at start: refused before anything is enabled"
reset_inputs; setin $LIM_L 0; trace_start
start
finish
setin $LIM_L 1
check "A1 exit 2" 2 "$RC"
check "A1 refused" 1 "$(grep -c 'limit switch is pressed at start' $OUT)"
check "A1 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"

echo "== A2 the ladder is listed; q before homing ends it, driver never enabled"
reset_inputs; trace_start
start
waitfor 'Press Enter to HOME' 10
say q
finish
check "A2 exit 0" 0 "$RC"
check "A2 seven rungs listed" 7 "$(grep -c '^  rung ' $OUT)"
check "A2 finished" 1 "$(grep -c 'block finished' $OUT)"
check "A2 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"
check "A2 STEP never 1 (only the line request's 0)" 0 "$(sets $STEP | tr ' ' '\n' | grep -c '^1$')"

echo "== A3 refusals before any GPIO: a speed above actuator.v_top, a ladder the Axis cap would cut"
start --set actuator.speeds=0.9
finish
check "A3 v_top exit 1" 1 "$RC"
check "A3 v_top message" 1 "$(grep -c 'actuator.speeds' $OUT)"
start --set axis.max_accel=100000
finish
check "A3 Axis cap exit 1" 1 "$RC"
check "A3 Axis cap message" 1 "$(grep -c 'Axis cap' $OUT)"

echo "== A4 homing, the left switch closes far too early: span check fails, braked, driver off"
reset_inputs; trace_start
start
waitfor 'Press Enter to HOME' 10
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; setin $LIM_R 0
waitfor 'seeking the LEFT' 5
sleep 0.2; setin $LIM_R 1
sleep 2; setin $LIM_L 0
finish
setin $LIM_L 1
check "A4 exit 3" 3 "$RC"
check "A4 span check failed" 1 "$(grep -c '^FAILED: track span' $OUT)"
en=$(sets $EN)
check "A4 EN enabled (0) then last write 1" "1 1" "$(echo " $en " | grep -c ' 0 ') ${en: -1}"
check "A4 STEP toggled" 1 "$([ "$(sets $STEP | wc -w)" -gt 100 ] && echo 1 || echo 0)"

echo "== A5 SIGINT during homing: controlled stop, driver off, logs"
reset_inputs; trace_start
start
waitfor 'Press Enter to HOME' 10
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; sudo pkill -INT -x pendctl
finish
check "A5 exit 0" 0 "$RC"
check "A5 finished" 1 "$(grep -c 'block finished' $OUT)"
en=$(sets $EN); check "A5 last EN write 1" 1 "${en: -1}"
d=$(lastdir)
check "A5 logs written" 1 "$([ -s "$d/data.csv" ] && [ -s "$d/rungs.csv" ] && [ -s "$d/reps.csv" ] && echo 1 || echo 0)"

echo "== A6 full homing (switches timed for 6104 usteps), READY states rung 1, s skips the speed, Enter ends"
reset_inputs; trace_start
start
home
waitfor 'home: centred' 15
waitfor '^NEXT: rung 1 of 7' 2
say s
waitfor 'Ladder done' 5
say ""
finish
check "A6 exit 0" 0 "$RC"
check "A6 span accepted" 1 "$(grep -c 'step mode OK' $OUT)"
check "A6 rung 1 stated" 1 "$(grep -c '^NEXT: rung 1 of 7' $OUT)"
check "A6 no RUN" 0 "$(grep -c ' RUN rung' $OUT)"
check "A6 finished" 1 "$(grep -c 'block finished' $OUT)"
en=$(sets $EN); check "A6 last EN write 1" 1 "${en: -1}"
check "A6 meta span within 5 %" 1 "$(python3 -c "print(int(abs($(meta span_usteps) - 6104) < 305))")"
check "A6 rungs.csv: 7 skipped" 7 "$(grep -c ',skipped$' "$(lastdir)/rungs.csv")"

trace_stop
reset_inputs
sudo chown -R "$(id -un)":"$(id -gn)" logs
echo "actuator mock: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
