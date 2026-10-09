#!/bin/bash
# pendctl shadow and damp modes on the gpio-mockup chip (never the real chip). Needs gpio-mockup
# loaded and sudo. A synthetic swing (tests/mock/swing_gen.py) drives the mock encoder lines; the mock
# cart does not move the mock pendulum, so these check the sequencing, not the damping (that is
# tests/unit/test_damp.cpp on the simulated plant). EN writes are read from the gpio_value trace.
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
OUT=build/stage1_mock.out
run() { # mode, overrides...; sets RC, output in $OUT
    local mode=$1; shift
    sudo timeout -s INT 60 stdbuf -oL ./build/pendctl --mode "$mode" --set "$mode.chip=$MOCK" "$@" > $OUT 2>&1
    RC=$?
}
lastdir() { ls -td logs/*_"$1" | head -1; }
meta() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])" "$(lastdir "$1")/meta.json" "$2"; }
swing() { sudo python3 tests/mock/swing_gen.py "$DBG" --amp-deg "${2:-5}" --duration "$1" > build/swing_gen.out & }

make -s build/pendctl || { echo "build failed"; exit 1; }   # the recipe deletes the binary before linking

echo "== S1 shadow, no swing: calibration times out, motor lines never touched"
reset_inputs; trace_start
run shadow --set calib.timeout=3
check "S1 exit 3" 3 "$RC"
check "S1 calibration timeout" 1 "$(grep -c 'calib: no accepted fit' $OUT)"
check "S1 no EN writes" "" "$(sets $EN)"

echo "== S2 shadow with a 5 deg synthetic swing: calibrates, runtime runs, motor lines never touched"
reset_inputs; trace_start; swing 12
run shadow --set shadow.duration=4
wait
check "S2 exit 0" 0 "$RC"
check "S2 calib accepted" 1 "$(grep -c 'calib: accepted' $OUT)"
cb=$(meta shadow c_bottom_fit_counts)
check "S2 |c_bottom| < 0.3 counts (generator centre 0)" 1 "$(python3 -c "print(int(abs($cb) < 0.3))")"
check "S2 motor commands ignored 0" 0 "$(meta shadow motor_commands_ignored)"
check "S2 no EN writes" "" "$(sets $EN)"
check "S2 no STEP writes" "" "$(sets $STEP)"
grep -E "^calib: accepted|IDLE|ARMED" $OUT | head -4

echo "== D1 damp, no swing: calibration times out, driver never enabled"
reset_inputs; trace_start
run damp --set calib.timeout=3
check "D1 exit 3" 3 "$RC"
check "D1 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"

echo "== D2 damp with a 5 deg synthetic swing: calibrates, driver on, engages, completes, driver off"
reset_inputs; trace_start; swing 14
run damp --set damp.duration=2
wait
check "D2 exit 0" 0 "$RC"
check "D2 result completed" completed "$(meta damp result)"
check "D2 engaged within 1 s" 1 "$(python3 -c "print(int(0 <= $(meta damp engaged_at_s) < 1.0))")"
en=$(sets $EN)
check "D2 EN enabled (0) then last write 1" "1 1" "$(echo "$en" | grep -c ' 0 ' ) ${en: -1}"
check "D2 STEP toggled" 1 "$([ "$(sets $STEP | wc -w)" -gt 10 ] && echo 1 || echo 0)"
grep -E "calib: accepted|ENGAGED|STOPPING|DONE|^damp:" $OUT | head -6

echo "== D3 damp SIGINT while engaged: stop request, brake, driver off"
reset_inputs; trace_start; swing 14
(sleep 6.5; sudo pkill -INT -x pendctl) & run damp --set damp.duration=5
wait
check "D3 exit 3" 3 "$RC"
check "D3 result stop request" "stop request" "$(meta damp result)"
en=$(sets $EN); check "D3 last EN write 1" 1 "${en: -1}"

echo "== D4 damp with the left switch pressed: refused before anything is enabled"
reset_inputs; setin $LIM_L 0; trace_start
run damp
setin $LIM_L 1
check "D4 exit 2" 2 "$RC"
check "D4 EN never 0" 0 "$(sets $EN | tr ' ' '\n' | grep -c '^0$')"

trace_stop
reset_inputs
sudo chown -R "$(id -un)":"$(id -gn)" logs
echo "stage1 mock: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
