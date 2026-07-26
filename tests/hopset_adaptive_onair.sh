#!/usr/bin/env bash
# Adaptive-hopset on-air validation: two adapters, txdemo as the schedule
# authority (scripted commit drops one base channel), rxdemo as the follower.
#
# Phase A — synchronized activation: the follower is tracking before the
#   scripted commit lands; both sides must emit hopset.commit/hopset.activate
#   for generation 1 and the RX must keep decoding markers after the boundary.
# Phase B — missed-commit recovery: a FRESH follower starts after the
#   transition already happened; it must detect the unknown generation (v2
#   marker fingerprint and/or the status beacon), emit hopset.recover, adopt
#   generation 1, and decode on the reduced set.
#
# Usage: sudo -v && tests/hopset_adaptive_onair.sh
#        TX_PID=0xa81a RX_PID=0xc812 CHANNELS=1,6,11 tests/hopset_adaptive_onair.sh
#
# Cleanup: on any exit (incl. Ctrl-C) both demos are killed by exact comm.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

VID="${VID:-0x0bda}"
# The transmitter must actually reach the receiver on this band, and the
# 8812EU is a 5 GHz-only PA part — on 2.4 GHz it transmits without one, which
# is enough for a follower already tracking but leaves a FRESH follower, that
# has to catch a single status beacon while scanning, missing it more often
# than not. The 8812CU has the front end for this band in both directions.
TX_PID="${TX_PID:-0xc812}"     # RTL8812CU (Jaguar3)
RX_PID="${RX_PID:-0xa81a}"     # RTL8812EU (Jaguar3) — receive only here
CHANNELS="${CHANNELS:-1,6,11}" # base hopset (2.4 GHz — every DUT reaches it)
SLOT_MS="${SLOT_MS:-50}"
SEED="${SEED:-c0ffeef00dc0ffeef00dc0ffeef00d01}"
COMMIT_SLOT="${COMMIT_SLOT:-150}" # scripted commit ~7.5 s in
MASK="${MASK:-0x3}"               # drop base position 2 (channel 11)
PHASE_A_S="${PHASE_A_S:-25}"
PHASE_B_S="${PHASE_B_S:-20}"
OUT="${OUT:-/tmp/devourer-hopset-onair}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$TX_PID" || { echo "SKIP: TX $VID:$TX_PID not plugged"; exit 77; }
plugged "$VID" "$RX_PID" || { echo "SKIP: RX $VID:$RX_PID not plugged"; exit 77; }

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

cleanup() {
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x txdemo 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== build =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1
unbind "$TX_PID"; unbind "$RX_PID"
mkdir -p "$OUT"; rm -f "$OUT"/*.log

# This is a protocol test (commit -> activate -> recover), not a policy test:
# it drives the operator's scripted lever on a 3-channel base, so the
# authority's diversity floor is lowered to the protocol minimum. The
# receiver-driven policy's own max(3, configured) floor is not involved.
COMMON=(DEVOURER_HOP_CHANNELS="$CHANNELS" DEVOURER_HOP_SLOT_MS="$SLOT_MS"
        DEVOURER_HOP_SEED="$SEED" DEVOURER_HOP_ADAPTIVE=1
        DEVOURER_HOP_MIN_ACTIVE=2 DEVOURER_LOG_LEVEL=info)

echo "== phase A: follower tracks the scripted transition =="
sudo env "${COMMON[@]}" DEVOURER_VID="$VID" DEVOURER_PID="$RX_PID" \
    "$ROOT/build/rxdemo" >"$OUT/rxA.log" 2>&1 &
RX_A=$!
sleep 3
sudo env "${COMMON[@]}" DEVOURER_VID="$VID" DEVOURER_PID="$TX_PID" \
    DEVOURER_CHANNEL="${CHANNELS%%,*}" DEVOURER_HOP_FAST=1 \
    DEVOURER_HOP_ADAPTIVE_SCRIPT="${COMMIT_SLOT}:${MASK}" \
    "$ROOT/build/txdemo" >"$OUT/tx.log" 2>&1 &
TX=$!
sleep "$PHASE_A_S"
sudo kill -INT "$RX_A" 2>/dev/null || true
wait "$RX_A" 2>/dev/null || true

echo "== phase B: fresh follower joins after the transition =="
sudo env "${COMMON[@]}" DEVOURER_VID="$VID" DEVOURER_PID="$RX_PID" \
    "$ROOT/build/rxdemo" >"$OUT/rxB.log" 2>&1 &
RX_B=$!
sleep "$PHASE_B_S"
sudo kill -INT "$RX_B" 2>/dev/null || true
wait "$RX_B" 2>/dev/null || true
sudo kill -INT "$TX" 2>/dev/null || true
wait "$TX" 2>/dev/null || true

echo "== verdicts (logs: $OUT) =="
fails=0
require() { # file, needle, label
    if grep -qF "$2" "$1"; then echo "PASS: $3"; else echo "FAIL: $3"; fails=$((fails+1)); fi
}
# TX side: commit issued and activated at gen 1
require "$OUT/tx.log"  '"ev":"hopset.commit"'   "tx committed"
require "$OUT/tx.log"  '"ev":"hopset.activate"' "tx activated"
# Phase A follower: tracked, adopted the commit, activated in lockstep, and
# kept decoding markers afterwards (post-activation slot lock)
require "$OUT/rxA.log" '"ev":"hop.rx","state":"track"' "A: follower tracked"
require "$OUT/rxA.log" '"ev":"hopset.commit"'   "A: follower adopted the commit"
require "$OUT/rxA.log" '"ev":"hopset.activate"' "A: follower activated"
act_slot=$(grep -F '"ev":"hopset.activate"' "$OUT/rxA.log" | head -1 |
           sed -n 's/.*"activate_slot":\([0-9]*\).*/\1/p')
if [ -n "${act_slot:-}" ] &&
   grep -F '"state":"decode"' "$OUT/rxA.log" |
   sed -n 's/.*"slot":\([0-9]*\).*/\1/p' |
   awk -v a="$act_slot" 'BEGIN{ok=0} $1>a{ok=1} END{exit ok?0:1}'; then
    echo "PASS: A: decodes continue past the activation slot ($act_slot)"
else
    echo "FAIL: A: no decode past the activation slot"; fails=$((fails+1))
fi
# Phase B follower: detected the missed transition and recovered
require "$OUT/rxB.log" '"ev":"hopset.recover"'  "B: fresh follower recovered"
require "$OUT/rxB.log" '"ev":"hopset.activate"' "B: fresh follower adopted gen 1"
require "$OUT/rxB.log" '"state":"decode"'       "B: decoding after recovery"

[ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
exit "$fails"
