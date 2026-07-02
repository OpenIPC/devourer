#!/usr/bin/env bash
# NDPA->NDP hardware-sounding on-air probe (question 1 of the BF self-sounding
# experiment): does setting the TX-descriptor NDPA bit make the MAC follow the
# injected NDPA control frame with a hardware-generated NDP?
#
# Runs two cells on the 8812AU (TX) + B210 (RX @ ch100 / 5500 MHz):
#   A) baseline — NDPA frame injected, descriptor bit OFF  -> single bursts
#   B) test     — NDPA frame injected, DEVOURER_TX_NDPA=1  -> burst PAIRS at
#                 a ~16 us SIFS gap if the sounding engine fires
#
# PASS = cell B pair_frac >> cell A pair_frac (and median pair gap ~10-30 us).
#
# Usage: sudo tests/bf_ndpa_onair.sh   (from the devourer-bf worktree root)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
# UHD python is a system package (the tests/.venv predates python 3.14).
PY="/usr/bin/python3"
PROBE="$HERE/bf_ndpa_probe.py"
OUT="/tmp/bf-ndpa-probe"
CHANNEL=100
FREQ=5500e6
RATE=10e6
DUR=8

[ -x "$TXDEMO" ] || { echo "build WiFiDriverTxDemo first ($TXDEMO)"; exit 1; }
[ -x "$PY" ] || { echo "missing venv python $PY"; exit 1; }
mkdir -p "$OUT"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -f "bf_ndpa_probe.py" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

run_cell() { # $1=name  $2=ndpa_desc_bit(0|1)
    local name="$1" bit="$2"
    echo "== cell $name (desc NDPA bit=$bit) =="
    # SDR probe first so it sees the whole TX window.
    "$PY" "$PROBE" --freq "$FREQ" --rate "$RATE" --duration "$((DUR - 1))" \
        > "$OUT/$name.probe" 2> "$OUT/$name.probe.err" &
    local probe_pid=$!
    sleep 1.5   # B210 tune/settle

    local env_extra=()
    [ "$bit" = "1" ] && env_extra=(DEVOURER_TX_NDPA=1 DEVOURER_BF_ARM_SOUNDER=1)
    env DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_TX_RATE=VHT2SS_MCS0 \
        DEVOURER_TX_NDPA_RA=aa:bb:cc:dd:ee:ff \
        "${env_extra[@]}" \
        timeout "$DUR" "$TXDEMO" > "$OUT/$name.tx.log" 2>&1
    wait "$probe_pid"
    grep "bf-probe summary" "$OUT/$name.probe" || echo "(no summary — see $OUT/$name.probe.err)"
}

run_cell baseline 0
sleep 2
run_cell ndpa 1

echo
echo "== verdict =="
a=$(grep -o 'pair_frac=[0-9.]*' "$OUT/baseline.probe" | cut -d= -f2)
b=$(grep -o 'pair_frac=[0-9.]*' "$OUT/ndpa.probe" | cut -d= -f2)
echo "baseline pair_frac=${a:-?}  ndpa pair_frac=${b:-?}  (logs in $OUT)"
