#!/bin/bash
# Stage 2b end to end on one Linux machine (WSL, a Linux PC, or the Pi itself): the simulated pendulum with the
# real UDP link (build/remote_sim_test = pendctl --mode remote-sim without libgpiod) and the real PC program
# (py/remote_ctrl.py) over localhost. No GPIO, nothing moves. Checks: it engaged and balanced for the whole run,
# and the PC's commands equal the Pi's shadow PID on the same samples (the protocol, scaling and controller).
# Timing is not what this checks: without SCHED_FIFO (WSL) the loop can stall for tens of ms, so the overrun and
# missed-command aborts are relaxed here and late / missed counts are printed for information. The aborts
# themselves are unit-tested (tests/unit/test_remote.cpp); the real timing comes from remote-sim on the Pi.
#   tests/remote/run_remote_e2e.sh [seconds] [delay_ticks]
set -u
cd "$(dirname "$0")/../.."
SECS=${1:-12}
DELAY=${2:-3}
OUT=build/e2e
mkdir -p "$OUT"
rm -f build/remote_sim_test
g++ -std=c++17 -Wall -Wextra -O2 -Isrc -I. -DGIT_HASH=\"e2e\" -DGIT_DIRTY=1 -o build/remote_sim_test \
    tests/remote/remote_sim_main.cpp src/app/remote_sim.cpp src/app/remote_link.cpp src/common/config.cpp \
    src/common/runlog.cpp src/rt/rt.cpp src/core/pid.cpp src/core/supervisor.cpp src/core/units.cpp \
    src/core/swingfit.cpp src/core/estimator.cpp src/core/profile.cpp src/hw/sim_hw.cpp src/runtime/balance.cpp \
    src/runtime/swing_calib.cpp src/runtime/damp.cpp src/runtime/params.cpp src/runtime/mover.cpp \
    src/runtime/session.cpp src/runtime/swingup.cpp -pthread || { echo "build failed"; exit 1; }

PORT=9761
timeout 180 python3 py/remote_ctrl.py --host 127.0.0.1 --port $PORT --log-dir "$OUT" > "$OUT/pc.out" 2>&1 &
PC=$!
timeout 180 ./build/remote_sim_test --set log.dir="$OUT" --set remote.port=$PORT --set remote.delay_ticks="$DELAY" \
    --set remote.sim_seconds="$SECS" --set remote.sim_wait_pc=20 --set rt.enable=false --set rt.lock_memory=false \
    --set sup.overrun_streak=1000000 --set remote.max_miss=50 > "$OUT/pi.out" 2>&1
RC=$?
wait $PC
echo "--- Pi (remote-sim)"; cat "$OUT/pi.out"
echo "--- PC (remote_ctrl.py)"; tail -4 "$OUT/pc.out"
DIR=$(ls -td "$OUT"/*_remote-sim | head -1)
meta() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])" "$DIR/meta.json" "$1"; }
pass=0; fail=0
check() { if [ "$2" == "$3" ]; then echo "  PASS $1 [$3]"; pass=$((pass+1)); else echo "  FAIL $1: expected [$2], got [$3]"; fail=$((fail+1)); fi; }
check "result" completed "$(meta result)"
check "PC = local PID (max diff < 1e-9)" 1 "$(python3 -c "print(int(float('$(meta remote_max_abs_diff)') < 1e-9))")"
check "engaged >= seconds" 1 "$(python3 -c "print(int(float('$(meta engaged_s)') >= $SECS))")"
check "engaged replies >= 90 % of the engaged ticks" 1 \
      "$(python3 -c "print(int(int('$(meta remote_engaged_replies)') >= 0.9 * float('$(meta engaged_s)') / 0.002))")"
echo "  info: exit $RC, missed ticks $(meta missed_ticks), late replies $(meta remote_late_replies), overruns $(meta overruns)," \
     "round trip p50 $(meta remote_rtt_ms_p50) / p99.9 $(meta remote_rtt_ms_p999) / max $(meta remote_rtt_ms_max) ms"
echo "e2e: $pass passed, $fail failed  (logs: $DIR)"
[ "$fail" -eq 0 ]
