#!/bin/bash
# pcb-test against the virtual board (tests/pcb/fake_gpiod.cpp): no hardware, any Linux machine (WSL, the Pi).
# Builds build/pcb_test_fake from the real src/app/pcb_test.cpp, then runs stages C, D and E on a good board and with
# injected faults, and checks that every verdict is the one the fault must produce, and that every run ends with the
# driver off and never stepped the motor while disabled.
#   tests/pcb/run_pcb_fake.sh            (about 3 minutes: stage D and E run in real time)
set -u
cd "$(dirname "$0")/../.."
OUT=build/pcbfake
rm -rf "$OUT"
mkdir -p "$OUT"
rm -f build/pcb_test_fake
g++ -std=c++17 -Wall -Wextra -O2 -Itests/pcb -Isrc -I. -DGIT_HASH=\"fake\" -DGIT_DIRTY=1 -o build/pcb_test_fake \
    tests/pcb/pcb_test_main.cpp tests/pcb/fake_gpiod.cpp src/app/pcb_test.cpp src/core/pcb_check.cpp \
    src/common/config.cpp src/common/runlog.cpp src/rt/rt.cpp src/hw/edge_reader.cpp -pthread || { echo "build failed"; exit 1; }

pass=0; fail=0
check() { if [ "$2" == "$3" ]; then echo "  PASS $1 [$3]"; pass=$((pass+1)); else echo "  FAIL $1: expected [$2], got [$3]"; fail=$((fail+1)); fi; }
has() { grep -q -- "$2" "$OUT/$1.out" && echo 1 || echo 0; }        # name, pattern in the program's output
result() { sed -n 's/^pcb-test stage .: \(.*\)$/\1/p' "$OUT/$1.out" | tail -1; }
fakeend() { grep -o "$2=[-0-9]*" "$OUT/$1.err" | tail -1 | cut -d= -f2; }
# run NAME STAGE ANSWER-SCRIPT [--set ...]: the answer script prints the operator's lines (with sleeps where timed)
run() {
    local name=$1 stage=$2 answers=$3
    shift 3
    bash -c "$answers" | timeout 400 ./build/pcb_test_fake --set log.dir="$OUT/logs" --set pcbtest.stage="$stage" "$@" \
        > "$OUT/$name.out" 2> "$OUT/$name.err"
    echo $? > "$OUT/$name.rc"
    echo "== $name: $(result "$name")"
}
common_end() {   # every run: driver off at the end, no steps while disabled, no output fighting another
    check "$1 ends with ~ENABLE high (driver off)" 1 "$(fakeend "$1" EN)"
    check "$1 no steps while the driver was off" 0 "$(fakeend "$1" steps_disabled)"
    check "$1 no outputs fighting" 0 "$(fakeend "$1" conflicts)"
}

echo "--- stage C"
FAKE_PLUGGED="" FAKE_FAULTS="" run C_good C 'printf "yes\n\n"'
check "C good board: PASS" "PASS" "$(result C_good)"
check "C good: no bridges" 1 "$(has C_good 'C3   PASS bridges')"
common_end C_good

FAKE_FAULTS="bridge:20-21" run C_bridge C 'printf "yes\n\n"'
check "C MS1-MS2 solder bridge: FAIL" "FAIL" "$(result C_bridge)"
check "C bridge named" 1 "$(has C_bridge 'GPIO20 (pin 38) and GPIO21 (pin 40) are connected')"
common_end C_bridge

FAKE_FAULTS="open:10" run C_open C 'printf "yes\n\n"'
check "C missing R1 (encoder A pull-up): FAIL" 1 "$(has C_open 'GPIO10 (pin 19).*pull-up resistor is missing')"

FAKE_FAULTS="gnd:27" run C_gnd C 'printf "yes\n\n"'
check "C STEP shorted to GND: held LOW" 1 "$(has C_gnd 'GPIO27 (pin 13).*held LOW')"
check "C the shorted line is never driven" 0 "$(has C_gnd 'GPIO27 (pin 13) drive')"
common_end C_gnd

FAKE_FAULTS="v33:4" run C_v33 C 'printf "yes\n\n"'
check "C GPIO4 shorted to 3V3: held HIGH" 1 "$(has C_v33 'GPIO4 (pin 7).*held HIGH')"

FAKE_FAULTS="used:14" run C_used C 'printf "yes\n\n"'
check "C a line claimed elsewhere is skipped, not touched" 1 "$(has C_used 'SKIP GPIO14 (pin 8): claimed')"
check "C ... and the rest still passes" "PASS" "$(result C_used)"

FAKE_PLUGGED="switches" FAKE_PRESS="right@0+999" run C_plugged C 'printf "yes\n\n"'
check "C with a switch still plugged in and pressed: GPIO2 held LOW" 1 "$(has C_plugged 'GPIO2 (pin 3).*held LOW')"

run C_refused C 'printf "no\n"'
check "C not confirmed: nothing tested" 0 "$(has C_refused 'C2 ')"

echo "--- stage D (real time, ~30 s each)"
DANS='sleep 0.5; echo yes; sleep 23.5; echo; sleep 6; echo'
FAKE_PLUGGED="switches,enc" FAKE_PRESS="right@2+1,left@5+1" FAKE_TURNS="19:-2400:3,25:2400:3" run D_good D "$DANS"
check "D good: PASS" "PASS" "$(result D_good)"
check "D right switch" 1 "$(has D_good 'PASS RIGHT.*0 while pressed')"
check "D turn toward +x" 1 "$(has D_good 'PASS turn toward +x: net -2400')"
check "D turn back" 1 "$(has D_good 'PASS turn toward -x: net 2400')"
common_end D_good

FAKE_PLUGGED="switches,enc" FAKE_FAULTS="swapsw" FAKE_PRESS="right@2+1,left@5+1" FAKE_TURNS="19:-2400:3,25:2400:3" run D_swapsw D "$DANS"
check "D switches swapped on J3: FAIL" 1 "$(has D_swapsw 'OTHER switch')"

FAKE_PLUGGED="switches,enc" FAKE_FAULTS="swapenc" FAKE_PRESS="right@2+1,left@5+1" FAKE_TURNS="19:-2400:3,25:2400:3" run D_swapenc D "$DANS"
check "D encoder A/B swapped: FAIL, counts the other way" 1 "$(has D_swapenc 'counts the OTHER way')"

echo "--- stage E (real time, ~1 min each)"
# stage E's own log folder holds only the passed D_good log, so its stage D gate passes as on the rig
mkdir -p "$OUT/logs_E"
cp -r "$(sed -n 's/^  -> //p' "$OUT/D_good.out")" "$OUT/logs_E/"
EOPT="--set log.dir=$OUT/logs_E --set pcbtest.hold_seconds=1 --set pcbtest.v_slow=0.02 --set pcbtest.v_span=0.03"
EANS='printf "yes\n\ny\ny\n\ny\n\n\n\n"'
# shellcheck disable=SC2086
FAKE_PLUGGED="switches,motor,f2" run E_good E "$EANS" $EOPT
check "E good: PASS" "PASS" "$(result E_good)"
check "E good: stage D gate passed" 1 "$(has E_good 'E0   PASS stage D')"
check "E five step-mode trips pass" 5 "$(grep -c 'E3   PASS step mode' "$OUT/E_good.out")"
check "E span passes (6104 eighth-steps on the virtual track)" 1 "$(has E_good 'E4   PASS span: 61')"
check "E back in the middle (+- 20 eighth-steps)" 1 "$(c=$(fakeend E_good cart); [ "$c" -gt 3032 ] && [ "$c" -lt 3092 ] && echo 1 || echo 0)"
common_end E_good

# shellcheck disable=SC2086
FAKE_PLUGGED="switches,motor,f2" FAKE_FAULTS="msswap" run E_msswap E "$EANS" $EOPT
check "E MS1/MS2 swapped: 1/4 steps like 1/2" 1 "$(has E_msswap 'FAIL step mode: 1/4.*like 1/2')"
check "E MS1/MS2 swapped: 1/2 steps like 1/4" 1 "$(has E_msswap 'FAIL step mode: 1/2.*like 1/4')"
common_end E_msswap

# shellcheck disable=SC2086
FAKE_PLUGGED="switches,motor,f2" FAKE_FAULTS="dirinv" run E_dirinv E 'printf "yes\n\ny\ny\n\nn\n"' $EOPT
check "E motor coil reversed: FAIL and stop before the switch tests" 1 "$(has E_dirinv 'one motor coil is the other way round')"
check "E ... no step-mode trips run" 0 "$(grep -c 'E3 ' "$OUT/E_dirinv.out")"
common_end E_dirinv

# shellcheck disable=SC2086
FAKE_PLUGGED="switches,motor,f2" FAKE_CART=0 run E_onswitch E "$EANS" $EOPT
check "E starting on a switch: refused before anything moves" 1 "$(has E_onswitch 'a limit switch is pressed')"
check "E ... no steps" 0 "$(fakeend E_onswitch steps)"

FAKE_PLUGGED="switches,motor,f2" run E_needsD E "$EANS" --set log.dir="$OUT/logs_noD"
check "E ... and nothing asked or moved before the refusal" 0 "$(has E_needsD 'THE MOTOR WILL MOVE')"
check "E without a passed stage D log: refused" 1 "$(has E_needsD 'needs a passed stage D')"
check "E ... no steps" 0 "$(fakeend E_needsD steps)"

echo "pcb-test on the virtual board: $pass passed, $fail failed  (outputs: $OUT)"
[ "$fail" -eq 0 ]
