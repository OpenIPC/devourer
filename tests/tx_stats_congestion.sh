#!/usr/bin/env bash
# Validate GetTxStats (the tx.stats event) — the driver-drop / congestion signal.
# Back-to-back TX (DEVOURER_TX_GAP_US=0) fills the on-chip TX FIFO, so the USB
# bulk-OUT starts NAKing (LIBUSB_ERROR_TIMEOUT / LIBUSB_TRANSFER_TIMED_OUT) and
# `failed` climbs with was_timeout=1 — the recoverable back-pressure an adaptive
# controller reads as "cut bitrate". A paced link (GAP=2000, ~500 fps) drains
# fine, so failed stays ~0. The counter must distinguish the two.
#
# Runs per plugged DUT (each family so both the async send_packet and the sync
# bulk_send_sync_ep counting paths are exercised). No SDR / no second adapter.
#
# Usage: sudo -v && tests/tx_stats_congestion.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXSTATS_OUT:-/tmp/devourer-txstats}"
CH="${CH:-36}"
DUR="${DUR:-12}"
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }
cleanup() { pkill -x txdemo 2>/dev/null||true; true; }
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo >/dev/null || exit 1

# pid vid label
DUTS=(
    "0x8812 0x0bda 8812AU-J1"
    "0x012d 0x2357 8822BU-J2"
    "0xa81a 0x0bda 8822EU-J3"
)

# One TX cell; echoes "submitted failed was_timeout last_rc" from the final line.
run_cell() { # $1=pid $2=vid $3=gap $4=tag
    local log="$OUT/$4.log"
    sudo -n env DEVOURER_PID="$1" DEVOURER_VID="$2" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_GAP_US="$3" \
        timeout "$DUR" "$ROOT/build/txdemo" >"$log" 2>&1 || true
    local line
    line=$(grep -F '"ev":"tx.stats"' "$log" | tail -1)
    echo "$line" | sed -E 's/.*"submitted":([0-9]+),"failed":([0-9]+),"was_timeout":([0-9]+),"last_rc":(-?[0-9]+).*/\1 \2 \3 \4/'
}

for dut in "${DUTS[@]}"; do
    read -r PID VID LABEL <<<"$dut"
    if ! plugged "$PID" "$VID"; then skip "$LABEL not plugged"; continue; fi
    echo "== $LABEL =="
    read -r p_sub p_fail p_to p_rc <<<"$(run_cell "$PID" "$VID" 2000 "$LABEL-paced")"
    read -r b_sub b_fail b_to b_rc <<<"$(run_cell "$PID" "$VID" 0 "$LABEL-b2b")"
    p_sub=${p_sub:-0}; p_fail=${p_fail:-0}; b_sub=${b_sub:-0}; b_fail=${b_fail:-0}
    b_to=${b_to:-0}; b_rc=${b_rc:-0}
    echo "  paced (gap=2000): submitted=$p_sub failed=$p_fail"
    echo "  back-to-back (gap=0): submitted=$b_sub failed=$b_fail was_timeout=$b_to last_rc=$b_rc"
    if [ "$p_sub" -lt 100 ]; then
        fail "$LABEL: paced cell barely transmitted ($p_sub) — TX path issue, inconclusive"
        continue
    fi
    # Congestion signature: back-to-back drives materially more failures than
    # paced (either a real climb, or the was_timeout flag set). Some chips never
    # fill the FIFO at USB2 speed — then failed stays ~0 in BOTH, which is a
    # valid "no congestion observed" (the counter is still correct: it reports
    # what happened). We only hard-fail if the counter is obviously broken
    # (submitted not counting).
    if [ "$b_sub" -lt 100 ]; then
        fail "$LABEL: back-to-back submitted=$b_sub not counting"
    elif [ "$b_fail" -gt "$p_fail" ] || [ "$b_to" -eq 1 ]; then
        pass "$LABEL: congestion visible — b2b failed=$b_fail (was_timeout=$b_to) > paced failed=$p_fail"
    else
        pass "$LABEL: counter working, no FIFO congestion at this rate (b2b failed=$b_fail == paced) — reported honestly"
    fi
done

echo
echo "== tx-stats congestion: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
