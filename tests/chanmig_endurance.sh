#!/usr/bin/env bash
# chanmig >=1000-cycle migration endurance (issue #278 acceptance gate).
#
# Two duplex adapters: a drone (video TX + responder) and a ground (primary RX
# + proposer). The python controller drives the ground's operator stdin,
# alternating propose A/B on each confirmed migration, and reports outage
# percentiles + zero persistent split-brain.
#
# Both ends are J1/J3 (duplex + AckResponder GO). TX power is reduced to keep
# the near-field rig link out of front-end saturation so markers/video deliver.
#
# Usage: sudo -v && tests/chanmig_endurance.sh
#        CYCLES=1000 GROUND_PID=0x8812 DRONE_PID=0xc812 tests/chanmig_endurance.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CYCLES="${CYCLES:-1000}"
VID="${VID:-0x0bda}"
GROUND_PID="${GROUND_PID:-0x8812}"   # RTL8812AU (J1) duplex ground
DRONE_PID="${DRONE_PID:-0xc812}"     # RTL8812CU (J3) duplex drone
CHAN_A="${CHAN_A:-36}"
CHAN_B="${CHAN_B:-149}"
KEY="${KEY:-c0ffeef00d}"
TX_PWR="${TX_PWR:-12}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$GROUND_PID" || { echo "SKIP: ground $VID:$GROUND_PID not plugged"; exit 77; }
plugged "$VID" "$DRONE_PID" || { echo "SKIP: drone $VID:$DRONE_PID not plugged"; exit 77; }

# In-tree rtw88/rtw89 auto-probe the dongles; temp-unbind any they hold.
unbind() {
    local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "${pid#0x}" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

echo "== build =="
cmake --build "$ROOT/build" -j --target chanmig >/dev/null || exit 1
unbind "$GROUND_PID"; unbind "$DRONE_PID"

echo "== $CYCLES-cycle endurance: ground $GROUND_PID <-> drone $DRONE_PID, ch$CHAN_A<->ch$CHAN_B =="
uv run --project "$ROOT/tests" python3 "$ROOT/tests/chanmig_endurance.py" \
    --cycles "$CYCLES" --vid "$VID" \
    --ground-pid "$GROUND_PID" --drone-pid "$DRONE_PID" \
    --chan-a "$CHAN_A" --chan-b "$CHAN_B" --key "$KEY" --tx-pwr "$TX_PWR" 2>&1 ||
    { echo "endurance run failed/incomplete"; exit 1; }
