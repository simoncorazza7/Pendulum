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
# Per-thread CPU over 2 s from /proc/<pid>/task/<tid>/schedstat (ns on CPU), with policy and core.
PCPID=$(pgrep -x pendctl | head -1)
declare -A run0
for t in /proc/$PCPID/task/*; do run0[${t##*/}]=$(cut -d' ' -f1 "$t/schedstat"); done
sleep 2
threads=""
for t in /proc/$PCPID/task/*; do
    tid=${t##*/}; r1=$(cut -d' ' -f1 "$t/schedstat")
    pol=$(ps -L -o tid=,cls=,rtprio= -p "$PCPID" | awk -v t="$tid" '$1==t{print $2" "$3}')
    cpu=$(awk '{print $39}' "$t/stat")
    use=$(echo "(${r1} - ${run0[$tid]:-$r1}) / 20000000" | bc -l)   # ns over 2 s -> percent
    threads+=$(printf "%s %s core%s %.1f%%" "$tid" "$pol" "$cpu" "$use")$'\n'
done
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
    echo "# threads at mid-run: tid class rtprio core cpu-use(2 s window)"
    echo "$threads" | sed 's/^/# /'
} > "$DIR/env.txt"
cat "$DIR/env.txt"
