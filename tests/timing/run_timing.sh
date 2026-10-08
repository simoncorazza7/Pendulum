#!/bin/bash
# Stage 0.5 timing run on the gpio-mockup chip (never the real chip).
#   tests/timing/run_timing.sh PROFILE DURATION_S [none|load]
# PROFILE: accel-stream | speed | idle. Needs: gpio-mockup loaded, `make` done, sudo.
# Load (SCHED_OTHER): one busy loop pinned to each core, SD-card writes, a compile loop.
set -u
cd "$(dirname "$0")/../.."
PROFILE=${1:?profile}; DUR=${2:?duration in s}; LOAD=${3:-none}

MOCK=$(gpiodetect | awk '/\[gpio-mockup-A\]/{print $1}')
[ -n "$MOCK" ] || { echo "gpio-mockup not loaded"; exit 1; }
DBG=/sys/kernel/debug/gpio-mockup/$MOCK
for line in 2 9; do echo 1 | sudo tee "$DBG/$line" >/dev/null; done   # limit switches free
[ -x build/pendctl ] || { echo "build/pendctl missing (make)"; exit 1; }

thr_before=$(vcgencmd get_throttled); temp_before=$(vcgencmd measure_temp)
rtthr_before=$(sudo dmesg | grep -c "RT throttling")

LOADPIDS=()
if [ "$LOAD" = load ]; then
    for c in 0 1 2 3; do taskset -c $c sh -c 'while :; do :; done' & LOADPIDS+=($!); done
    sh -c 'while :; do dd if=/dev/zero of=/tmp/ioload bs=1M count=100 oflag=direct status=none; done' & LOADPIDS+=($!)
    sh -c 'while :; do g++ -O2 -std=c++17 -Isrc -I. -c src/common/config.cpp -o /tmp/loadobj.o; done' & LOADPIDS+=($!)
fi

sudo ./build/pendctl --mode timing-probe --set probe.chip="$MOCK" --set probe.profile="$PROFILE" \
    --set probe.duration="$DUR" > build/timing_last.out 2>&1 &
SUDOPID=$!
sleep $((DUR / 2 + 1))
PCPID=$(pgrep -x pendctl | head -1)
threads=$(ps -L -o tid=,cls=,rtprio=,psr=,pcpu=,comm= -p "$PCPID" 2>/dev/null)
wait $SUDOPID

for p in "${LOADPIDS[@]}"; do kill "$p" 2>/dev/null; done
wait 2>/dev/null
rm -f /tmp/ioload /tmp/loadobj.o

cat build/timing_last.out
DIR=$(awk '/-> /{print $2}' build/timing_last.out)
[ -n "$DIR" ] || { echo "no run directory"; exit 1; }
sudo chown -R "$(id -un)":"$(id -gn)" logs
{
    echo "load = $LOAD"
    echo "throttled_before = $thr_before"
    echo "throttled_after = $(vcgencmd get_throttled)"
    echo "temp_before = $temp_before"
    echo "temp_after = $(vcgencmd measure_temp)"
    echo "dmesg_rt_throttling_lines_before = $rtthr_before"
    echo "dmesg_rt_throttling_lines_after = $(sudo dmesg | grep -c 'RT throttling')"
    echo "# threads at mid-run: tid class rtprio cpu %cpu(lifetime) name"
    echo "$threads" | sed 's/^/# /'
} > "$DIR/env.txt"
cat "$DIR/env.txt"
