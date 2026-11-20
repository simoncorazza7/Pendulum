#!/bin/bash
# Upright-offset estimate on the rig (docs/offset_estimator.md). HARDWARE ACTION: it runs the balance session of
# ./demo.sh (hand catches), ./swingup.sh (--swingup) or ./remote.sh (--remote, hand catches with the PC's controller)
# with offset.mode set. Their checklists, limits and code are unchanged; this script only adds the setting.
#   ./offset.sh watch [--swingup | --remote | --swingup --remote] [pendctl options]
#        estimate and log only (data.csv offset_delta / offset_weight, meta.json): the command is the same as without
#   ./offset.sh apply [--swingup] [pendctl options]
#        each catch / swing-up run starts with theta_ref = -delta_hat, learnt in the earlier balances of this program
#        run (>= offset.w_use s of quiet balance), held for that catch. Not with --remote (the PC computes the command).
# Run it on the Pi from a terminal you can type into:  ssh -t pendel@pendel.local  then  cd ~/Pendel-main && ./offset.sh ...
set -e
cd "$(dirname "$0")"

mode="${1:-}"
[ $# -gt 0 ] && shift
case "$mode" in
    watch|apply) ;;
    *) sed -n '2,10p' "$0"; exit 2 ;;
esac
swing=0
remote=0
while [ $# -gt 0 ]; do
    case "$1" in
        --swingup) swing=1; shift ;;
        --remote) remote=1; shift ;;
        *) break ;;
    esac
done
if [ "$mode" = apply ] && [ $remote = 1 ]; then
    echo "offset.sh: apply needs the balance controller on the Pi; with --remote use watch" >&2
    exit 2
fi

if [ "$mode" = apply ]; then
    cat <<'TXT'
Upright-offset estimate: APPLY. The first catch (or swing-up run) of this program run starts untrimmed; every later
one starts with the reference trimmed by what the earlier balances learnt, once at least 3 s of quiet balance stand
behind it. After each run the session prints the estimate and the trim it used. The checklist of the balance mode
follows.
TXT
else
    cat <<'TXT'
Upright-offset estimate: watch. The controller and its commands are unchanged; the estimate is logged
(data.csv offset_delta / offset_weight, meta.json) and printed after each run. The checklist of the balance mode follows.
TXT
fi

if [ $swing = 1 ]; then
    if [ $remote = 1 ]; then exec ./swingup.sh --remote --set offset.mode="$mode" "$@"; fi
    exec ./swingup.sh --set offset.mode="$mode" "$@"
fi
if [ $remote = 1 ]; then exec ./remote.sh --set offset.mode="$mode" "$@"; fi
exec ./demo.sh --set offset.mode="$mode" "$@"
