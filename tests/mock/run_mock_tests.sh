#!/bin/bash
# Stage 0.3: tests of `main` against the gpio-mockup chip. Never touches the real GPIO chip.
#
# Prerequisite (sudo, not persistent across reboots):
#   sudo /sbin/modprobe gpio-mockup gpio_mockup_ranges=-1,28 gpio_mockup_named_lines=1
# Run from anywhere on the Pi:  tests/mock/run_mock_tests.sh
# Input lines are driven through debugfs (needs sudo); outputs are read back the same way.
set -u
cd "$(dirname "$0")/../.."

MOCK=$(gpiodetect | awk '/\[gpio-mockup-A\]/{print $1}')
if [ -z "$MOCK" ] || [ "$MOCK" = gpiochip0 ] || [ "$MOCK" = gpiochip4 ]; then
    echo "gpio-mockup chip not found; refusing to run"; exit 1
fi
DBG=/sys/kernel/debug/gpio-mockup/$MOCK
BIN=build/main_mocktest
LOG=build/mocktest.log
# line offsets hard-coded in pendel.cpp (BCM numbers on the real chip)
ENC_A=10; ENC_B=19; STEP=27; DIR=22; EN=24; RLIM=2; LLIM=9; MS1=20; MS2=21

# The mock chip resets a line to 0 when the program releases it, so values after exit are read
# from the kernel's gpio_value trace event instead (every gpiolib set on the mock chip).
TR=/sys/kernel/tracing
BASE=$(sudo cat /sys/kernel/debug/gpio | awk -v c="$MOCK:" '$1==c {split($3,r,"-"); print r[1]}')
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
    sudo cat $TR/trace | awk -v g=$((BASE + $1))         '{for(i=1;i<NF;i++) if($i=="gpio_value:" && $(i+1)==g && $(i+2)=="set") printf "%s ", $(i+3)}' | sed 's/ $//'
}
setin() { echo "$2" | sudo tee "$DBG/$1" >/dev/null; }   # drive an input (simulated level)
getl()  { sudo cat "$DBG/$1"; }                           # read a line value
q()     { python3 tests/mock/tcpq.py "$@"; }
pass=0; fail=0
check() { # name expected actual
    if [ "$2" == "$3" ]; then echo "  PASS $1 [$3]"; pass=$((pass+1))
    else echo "  FAIL $1: expected [$2], got [$3]"; fail=$((fail+1)); fi
}
toggles() { # number of value changes of line $1 over $2 samples
    local prev n=0 v; prev=$(getl "$1")
    for _ in $(seq 1 "$2"); do v=$(getl "$1"); [ "$v" != "$prev" ] && n=$((n+1)); prev=$v; done
    echo $n
}
init_inputs() { setin $RLIM 1; setin $LLIM 1; setin $ENC_A 0; setin $ENC_B 0; }  # switches free
PID=""
start_main() {
    : > $LOG
    ./$BIN --chip "$MOCK" "$@" > $LOG 2>&1 &
    PID=$!
    for _ in $(seq 1 50); do grep -q "Server wartet" $LOG && return 0; sleep 0.1; done
    echo "main did not start:"; cat $LOG; exit 1
}
exit_code() { wait "$PID"; echo $?; }
trap '[ -n "$PID" ] && kill -KILL "$PID" 2>/dev/null; trace_stop' EXIT

echo "== build ($BIN, against $MOCK)"
mkdir -p build
rm -f $BIN && g++ -Wall -std=c++17 -o $BIN main.cpp encoder.cpp axis.cpp pendel.cpp safety.cpp \
    -lgpiod -lgpiodcxx -pthread || { echo "build failed"; exit 1; }

echo "== T1 bring-up default: driver off, no stepping"
init_inputs; trace_start; start_main
check "T1 EN = 1 (driver off) at start" 1 "$(getl $EN)"
check "T1 EN only ever set to 1" "1 1" "$(sets $EN)"
check "T1 no STEP toggles while idle" 0 "$(toggles $STEP 20)"
check "T1 MS1 = 1" 1 "$(getl $MS1)"
check "T1 MS2 = 1" 1 "$(getl $MS2)"

echo "== T2 protocol replies unchanged"
check "T2 replies" "10000|10000|0|0|0|ERR unknown" \
    "$(q getPos getEndPos getSpeed getAngle getAngleVelocity bogusCmd | paste -sd'|')"

echo "== T3 encoder through simulated edges (issue 2)"
for _ in 1 2 3; do setin $ENC_A 1; setin $ENC_B 1; setin $ENC_A 0; setin $ENC_B 0; done
check "T3 12 edges -> newest edge held back, count -11" -11 "$(q getAngle)"
setin $ENC_A 1
check "T3 next A edge releases it, count -12" -12 "$(q getAngle)"
setin $ENC_B 1                      # processes the held A edge -> -13, holds this B edge
setin $ENC_B 0; setin $ENC_B 1      # B dithers, A static: true count is -14
check "T3 B dithering, A static: count stale at -13" -13 "$(q getAngle)"

echo "== T4 motion command enables the driver; SIGINT brakes, then EN = 1"
check "T4 setSpeed reply" OK "$(q 'setSpeed -2000')"
check "T4 EN = 0 after motion command" 0 "$(getl $EN)"
n=$(toggles $STEP 20); [ "$n" -gt 0 ] && n=">0"
check "T4 STEP toggling" ">0" "$n"
check "T4 DIR = 0 for negative protocol speed" 0 "$(getl $DIR)"
kill -INT "$PID"
check "T4 exit code 130 (SIGINT)" 130 "$(exit_code)"
check "T4 EN writes: request 1, 1, enable 0, SIGINT 1" "1 1 0 1" "$(sets $EN)"
check "T4 stopped within the ramp" 1 "$(grep -c 'Stopped. Driver off' $LOG)"

echo "== T5 SIGTERM while idle"
init_inputs; trace_start; start_main
kill -TERM "$PID"
check "T5 exit code 143 (SIGTERM)" 143 "$(exit_code)"
check "T5 EN writes end with 1" "1 1 1" "$(sets $EN)"

echo "== T6 fatal signal while driving: EN = 1 at once"
init_inputs; trace_start; start_main
q 'setSpeed -2000' >/dev/null
check "T6 EN = 0 while driving" 0 "$(getl $EN)"
kill -SEGV "$PID"
check "T6 exit code 139 (SIGSEGV)" 139 "$(exit_code)"
check "T6 EN writes: 1, 1, enable 0, crash handler 1" "1 1 0 1" "$(sets $EN)"
check "T6 handler message" 1 "$(grep -c 'Fatal signal' $LOG)"

echo "== T7 --plot without gnuplot no longer dies of SIGPIPE (issue 12)"
init_inputs; start_main --plot
sleep 7
alive=$(kill -0 "$PID" 2>/dev/null && echo alive || echo dead)
check "T7 alive after 7 s (old code died at ~5 s)" alive "$alive"
kill -PIPE "$PID"; sleep 0.2
alive=$(kill -0 "$PID" 2>/dev/null && echo alive || echo dead)
check "T7 survives an explicit SIGPIPE" alive "$alive"
kill -TERM "$PID"; exit_code >/dev/null

echo "== T8 empty line hangs the TCP loop (issue 13); SIGTERM still stops the process"
init_inputs; start_main
check "T8 no reply after an empty line" "" "$(q getPos '' getPos)"
sleep 0.5
cpu=$(ps -o %cpu= -p "$PID" | awk '{print ($1 > 50) ? "spinning" : "idle"}')
check "T8 main thread spinning" spinning "$cpu"
kill -TERM "$PID"
check "T8 exit code 143 despite the stuck thread" 143 "$(exit_code)"

echo "== T9 --home on the mock: homing sequence with simulated switches (issues 4, 13)"
init_inputs; trace_start; start_main --home
check "T9 EN = 0 at start with --home" 0 "$(getl $EN)"
check "T9 reply during homing is duplicated" "ERR CALIBRATING|ERR CALIBRATING" "$(q getPos | paste -sd'|')"
check "T9 DIR = 0 toward the right switch (pos decreasing)" 0 "$(getl $DIR)"
setin $RLIM 0; sleep 0.2; setin $RLIM 1; sleep 0.3      # press and release the right switch
check "T9 DIR = 1 after the right switch (pos increasing)" 1 "$(getl $DIR)"
sleep 0.5; setin $LLIM 0; sleep 0.3                        # press the left switch
endpos=$(q getEndPos)
inrange=$([ "$endpos" -gt 500 ] && [ "$endpos" -lt 3000 ] && echo yes || echo "no ($endpos)")
check "T9 homing done, endPos = left-switch pos + 250 in (500, 3000)" yes "$inrange"
sleep 0.2
check "T9 speed 0 after homing" 0 "$(q getSpeed)"
echo "     (endPos = $endpos)"
kill -INT "$PID"
check "T9 exit code 130" 130 "$(exit_code)"
check "T9 EN writes: request 0, 0, SIGINT 1" "0 0 1" "$(sets $EN)"
PID=""

echo
echo "== $pass passed, $fail failed"
[ "$fail" -eq 0 ]
