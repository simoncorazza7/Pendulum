#!/bin/bash
# Set up a freshly imaged Pi like the rig Pi (2026-10-20, for the helper's Pi "pendel2"). Run it ON that Pi:
#     ssh -t pendel@pendel2.local      then      bash setup_pi.sh
# Expects Raspberry Pi OS bookworm (Debian 12), 32-bit armhf: the code uses the libgpiod v1 API (1.6.x, bookworm);
# newer releases ship libgpiod 2 and the build fails there. What the rig Pi has beyond its plain image, and nothing
# else (docs/stage0_discovery.md, docs/session_log.md):
#   1. packages: g++ make git libgpiod-dev gpiod (g++ 12, libgpiod 1.6.3 on bookworm)
#   2. /boot/firmware/config.txt: gpio=24=op,dh (A4988 EN high = driver OFF from power-on), backup kept
#   3. ~/Pendel-main: an empty git repository that accepts pushes into its working tree (updateInstead), so the PC
#      can `git push pi2 control-dev` exactly as to the rig Pi
# Shows what it will do and asks before changing anything; safe to run again. --check: report only.
# --rig: this Pi REPLACES the rig Pi (hostname pendel, user pendel; the old one died 2026-10-22), so the PC's
# remote `pi` (pendel@pendel.local) and every script keep working.
set -u
CHECK=0
RIG=0
for a in "$@"; do
    case "$a" in
    --check) CHECK=1 ;;
    --rig) RIG=1 ;;
    *) echo "usage: bash setup_pi.sh [--check] [--rig]"; exit 2 ;;
    esac
done

ok() { echo "  ok    $*"; }
todo() { echo "  TODO  $*"; }
bad() { echo "  STOP  $*"; }

echo "== this Pi"
. /etc/os-release
ARCH=$(dpkg --print-architecture)
HOST=$(hostname)
MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)
echo "  $MODEL | $PRETTY_NAME | $ARCH | host $HOST | user $(id -un) | kernel $(uname -r)"
fail=0
[ "${VERSION_CODENAME:-}" = "bookworm" ] && ok "bookworm" || { bad "not bookworm (${VERSION_CODENAME:-?}): libgpiod v1 is needed; re-image with the bookworm release"; fail=1; }
[ "$ARCH" = "armhf" ] && ok "32-bit armhf userland (as the rig Pi)" || { bad "userland is $ARCH, the rig Pi is armhf: re-image with the 32-bit bookworm release"; fail=1; }
if [ "$RIG" = 1 ]; then
    [ "$HOST" = "pendel" ] && [ "$(id -un)" = "pendel" ] && ok "rig replacement: host pendel, user pendel" \
        || { bad "--rig needs hostname pendel and user pendel (got $HOST / $(id -un)): re-image with those"; fail=1; }
else
    [ "$HOST" = "pendel" ] && { bad "hostname 'pendel' is the rig Pi's: two of them on one router can't be told apart. Re-image (or rename) first, or --rig if this Pi replaces the rig Pi"; fail=1; } || ok "hostname $HOST"
fi
id -nG | grep -qw gpio && ok "user in group gpio" || todo "user not in group gpio (Raspberry Pi OS adds the first user; check)"
[ "$fail" = 0 ] || { echo "not changing anything"; exit 1; }

echo "== state"
PKGS="g++ make git libgpiod-dev gpiod"
missing=""
for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 && ok "package $p $(dpkg-query -W -f='${Version}' "$p")" || { todo "package $p"; missing="$missing $p"; }; done
CFG=/boot/firmware/config.txt
grep -q '^gpio=24=op,dh' "$CFG" && ok "$CFG has gpio=24=op,dh" || todo "$CFG: add gpio=24=op,dh"
REPO="$HOME/Pendel-main"
if [ -d "$REPO/.git" ]; then
    [ "$(git -C "$REPO" config receive.denyCurrentBranch)" = "updateInstead" ] && ok "$REPO accepts pushes (updateInstead)" \
        || todo "$REPO: receive.denyCurrentBranch updateInstead"
else
    todo "$REPO: create the git repository"
fi
[ "$CHECK" = 1 ] && exit 0

echo
echo "Will now (sudo for 1 and 2):"
[ -n "$missing" ] && echo "  1. sudo apt-get update && sudo apt-get install -y$missing"
grep -q '^gpio=24=op,dh' "$CFG" || echo "  2. back up $CFG to $CFG.bak-$(date +%F), append gpio=24=op,dh (takes effect at the next boot)"
echo "  3. create/configure $REPO (no sudo)"
read -r -p "Type yes to go ahead: " go
[ "$go" = "yes" ] || { echo "nothing changed"; exit 1; }

set -e
if [ -n "$missing" ]; then
    sudo apt-get update
    sudo apt-get install -y $missing
fi
if ! grep -q '^gpio=24=op,dh' "$CFG"; then
    sudo cp -p "$CFG" "$CFG.bak-$(date +%F)"
    printf '\n# Pendel rig: A4988 EN (GPIO24) high = driver OFF from power-on (setup_pi.sh, as on the rig Pi)\ngpio=24=op,dh\n' \
        | sudo tee -a "$CFG" >/dev/null
    REBOOT=1
fi
if [ ! -d "$REPO/.git" ]; then
    git init -q -b control-dev "$REPO"
fi
git -C "$REPO" config receive.denyCurrentBranch updateInstead
echo
echo "done. libgpiod $(dpkg-query -W -f='${Version}' libgpiod-dev), $(g++ --version | head -1)"
if [ "$RIG" = 1 ]; then
    echo "Next on the PC: git push pi control-dev   (the new host key: check it against ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub here)"
else
    echo "Next on the PC: git remote add pi2 $(id -un)@$(hostname).local:Pendel-main && git push pi2 control-dev"
fi
echo "Then here: cd ~/Pendel-main && make && make test"
[ "${REBOOT:-0}" = 1 ] && echo "Reboot once (sudo reboot) so gpio=24=op,dh holds the driver off from power-on; check: pinctrl get 24"
exit 0
