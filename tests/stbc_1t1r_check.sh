#!/usr/bin/env bash
# Validate the STBC/1T1R safety guard (GetTxCaps + send_packet). STBC needs >=2
# TX chains, so forcing it on a 1T1R part produces a malformed PPDU that never
# decodes — the "LDPC/STBC forced on a card that lacks them => link dies"
# footgun from the adaptive-link recipes. Measured A/B on the 8821AU: forcing
# STBC (unguarded) delivered 0 frames; with the guard (STBC dropped) it
# delivered 7000. This test asserts the guard's behaviour per DUT:
#
#   1T1R (8821AU / 8821CU): inject MCS1/STBC -> the "dropping the STBC flag"
#     warning fires AND the frame still delivers (STBC dropped to clean MCS1).
#   2T2R (8812AU / 8822BU): inject MCS1/STBC -> NO warning (STBC honoured) and
#     the frame delivers.
#
# Ground = a second devourer part counting <devourer-tx-hit> from the canonical SA.
#
# Usage: sudo -v && tests/stbc_1t1r_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${STBC_OUT:-/tmp/devourer-stbc-1t1r}"
CH="${CH:-36}"
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() { pkill -x txdemo 2>/dev/null||true; pkill -x rxdemo 2>/dev/null||true; true; }
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

# DUT table: pid vid n_ss(1|2) ground_pid ground_vid
DUTS=(
    "0x0120 0x2357 1 0x8812 0x0bda"  # 8821AU (1T1R) ground 8812AU
    "0xc811 0x0bda 1 0x8812 0x0bda"  # 8821CU (1T1R) ground 8812AU
    "0x8812 0x0bda 2 0xc812 0x0bda"  # 8812AU (2T2R) ground 8822CU
    "0x012d 0x2357 2 0xc812 0x0bda"  # 8822BU (2T2R) ground 8822CU
)

for dut in "${DUTS[@]}"; do
    read -r PID VID NSS GPID GVID <<<"$dut"
    if ! plugged "$PID" "$VID"; then skip "$PID not plugged"; continue; fi
    if ! plugged "$GPID" "$GVID"; then skip "$PID: ground $GPID not plugged"; continue; fi
    tag="${PID#0x}"
    echo "== DUT $PID (${NSS}T${NSS}R), ground $GPID =="
    : >"$OUT/$tag-g.log"
    sudo -n env DEVOURER_PID="$GPID" DEVOURER_VID="$GVID" DEVOURER_CHANNEL="$CH" \
        stdbuf -oL timeout 45 "$ROOT/build/rxdemo" 2>/dev/null \
        | grep --line-buffered "tx-hit" >"$OUT/$tag-g.log" &
    GJ=$!
    sleep 10
    sudo -n env DEVOURER_PID="$PID" DEVOURER_VID="$VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS1/STBC DEVOURER_TX_GAP_US=2000 \
        timeout 18 "$ROOT/build/txdemo" >"$OUT/$tag-tx.log" 2>&1 || true
    sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null
    sleep 2

    warned=$(grep -c "dropping the STBC flag" "$OUT/$tag-tx.log")
    hits=$(grep -oE "hits=[0-9]+" "$OUT/$tag-g.log" | tail -1 | grep -oE "[0-9]+")
    hits=${hits:-0}
    echo "  warning fired: $warned, delivery hits: $hits"

    if [ "$NSS" = "1" ]; then
        if [ "$warned" -ge 1 ] && [ "$hits" -ge 200 ]; then
            pass "$PID (1T1R): STBC dropped + frame delivered ($hits hits)"
        else
            fail "$PID (1T1R): want warning+delivery, got warned=$warned hits=$hits"
        fi
    else
        if [ "$warned" -eq 0 ] && [ "$hits" -ge 200 ]; then
            pass "$PID (2T2R): STBC honoured, no warning, delivered ($hits hits)"
        else
            fail "$PID (2T2R): want no-warning+delivery, got warned=$warned hits=$hits"
        fi
    fi
done

echo
echo "== stbc-1t1r guard: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
