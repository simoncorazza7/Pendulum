#!/bin/bash
# pendctl balance mode (the balance session) on the gpio-mockup chip (never the real chip). Needs
# gpio-mockup loaded and sudo. The operator's Enter / q go through a FIFO on stdin; the switches are
# pressed by this script at the times a cart at balance.v_seek would reach them; tests/mock/swing_gen.py
# drives the encoder for the calibration. The mock cart moves nothing, so these check the plumbing
# (phases, driver enable, stdin, signals, logs), not the physics (tests/unit/test_session.cpp).
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
OUT=build/balance_mock.out
FIFO=build/balance_in
lastdir() { ls -td logs/*_balance | head -1; }
meta() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])" "$(lastdir)/meta.json" "$1"; }
swing() { sudo python3 tests/mock/swing_gen.py "$DBG" --amp-deg "${2:-5}" --duration "$1" > build/swing_gen.out & }
start() { # overrides...: pendctl in the background, stdin from the FIFO (held open by a sleeper)
    rm -f $FIFO; mkfifo $FIFO
    sleep 300 > $FIFO & HOLD=$!
    sudo timeout -s INT 120 stdbuf -oL ./build/pendctl --mode balance --set balance.chip=$MOCK "$@" < $FIFO > $OUT 2>&1 &
    PID=$!
}
finish() { wait $PID; RC=$?; kill $HOLD 2>/dev/null; wait $HOLD 2>/dev/null; rm -f $FIFO; }
say() { echo "$1" > $FIFO; }
waitfor() { # pattern, timeout s: 0 when it appeared in $OUT
    local i
    for ((i = 0; i < $2 * 10; i++)); do grep -q "$1" $OUT 2>/dev/null && return 0; sleep 0.1; done
    echo "    (timeout waiting for '$1')"; return 1
}

make -s build/pendctl || { echo "build failed"; exit 1; }   # the recipe deletes the binary before linking

echo "== B1 no swing: calibration times out, driver never enabled"
reset_inputs; trace_start
start --set calib.timeout=3
finish
check "B1 exit 3" 3 "$RC"
check "B1 calibration timeout" 1 "$(grep -c '^FAILED: calibration' $OUT)"
check "B1 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"

echo "== B2 swing, calibrated, q at WAIT_GO: ends, driver never enabled"
reset_inputs; trace_start; swing 12
start
waitfor '^calib: accepted' 20
say q
finish
wait
check "B2 exit 0" 0 "$RC"
check "B2 finished" 1 "$(grep -c 'session finished' $OUT)"
check "B2 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"
check "B2 STEP never 1 (only the line request's 0)" 0 "$(sets $STEP | tr ' ' '\n' | grep -c '^1$')"

echo "== B3 homing, the left switch closes far too early: span check fails, braked, driver off"
reset_inputs; trace_start; swing 12
start
waitfor '^calib: accepted' 20
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; setin $LIM_R 0
waitfor 'seeking the LEFT' 5
sleep 0.2; setin $LIM_R 1
sleep 2; setin $LIM_L 0
finish
wait
setin $LIM_L 1
check "B3 exit 3" 3 "$RC"
check "B3 span check failed" 1 "$(grep -c '^FAILED: track span' $OUT)"
en=$(sets $EN)
check "B3 EN enabled (0) then last write 1" "1 1" "$(echo " $en " | grep -c ' 0 ') ${en: -1}"
check "B3 STEP toggled" 1 "$([ "$(sets $STEP | wc -w)" -gt 100 ] && echo 1 || echo 0)"

echo "== B4 left switch pressed at start: refused before anything is enabled"
reset_inputs; setin $LIM_L 0; trace_start
start
finish
setin $LIM_L 1
check "B4 exit 2" 2 "$RC"
check "B4 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"

echo "== B5 SIGINT during homing: controlled stop, driver off, logs"
reset_inputs; trace_start; swing 12
start
waitfor '^calib: accepted' 20
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; sudo pkill -INT -x pendctl
finish
wait
check "B5 exit 0" 0 "$RC"
check "B5 finished" 1 "$(grep -c 'session finished' $OUT)"
en=$(sets $EN); check "B5 last EN write 1" 1 "${en: -1}"
check "B5 logs written" 1 "$([ -s "$(lastdir)/data.csv" ] && echo 1 || echo 0)"

echo "== B6 full homing with switches timed for a 6104-usteps span, arms, q: stops and ends"
reset_inputs; trace_start; swing 12
start
waitfor '^calib: accepted' 20
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; setin $LIM_R 0
waitfor 'seeking the LEFT' 5
sleep 0.2; setin $LIM_R 1
# from rest: accelerates for T0 = 1.274 s (half of it lost: 0.64 s), then 533 usteps/s: 6104 usteps ~ 12.1 s
sleep 11.9; setin $LIM_L 0
waitfor 'step mode OK' 5
sleep 0.2; setin $LIM_L 1
waitfor 'home: centred' 15
waitfor '^ARMED' 10
say q
finish
wait
check "B6 exit 0" 0 "$RC"
check "B6 span accepted" 1 "$(grep -c 'step mode OK' $OUT)"
check "B6 armed" 1 "$(grep -c '^ARMED' $OUT)"
check "B6 finished" 1 "$(grep -c 'session finished' $OUT)"
en=$(sets $EN); check "B6 last EN write 1" 1 "${en: -1}"
check "B6 meta span within 5 %" 1 "$(python3 -c "print(int(abs($(meta span_usteps) - 6104) < 305))")"
grep -E "home:|ARMED|STOPPING|run 1|session" $OUT | head -8

echo "== B7 swing-up mode: homing as B6, READY, q: ends without starting a run"
reset_inputs; trace_start; swing 12
start --set balance.swingup=true
waitfor '^calib: accepted' 20
sleep 0.5; say ""
waitfor 'seeking the RIGHT' 5
sleep 2; setin $LIM_R 0
waitfor 'seeking the LEFT' 5
sleep 0.2; setin $LIM_R 1
sleep 11.9; setin $LIM_L 0
waitfor 'step mode OK' 5
sleep 0.2; setin $LIM_L 1
waitfor 'home: centred' 15
waitfor '^READY (swing-up)' 10
say q
finish
wait
check "B7 exit 0" 0 "$RC"
check "B7 swing-up banner" 1 "$(grep -c '^SWING-UP MODE' $OUT)"
check "B7 ready" 1 "$(grep -c '^READY (swing-up)' $OUT)"
check "B7 no run started" 0 "$(grep -c 'SWING-UP run' $OUT)"
en=$(sets $EN); check "B7 last EN write 1" 1 "${en: -1}"

trace_stop
reset_inputs
sudo chown -R "$(id -un)":"$(id -gn)" logs
echo "balance mock: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
