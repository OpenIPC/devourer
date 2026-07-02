#!/usr/bin/env bash
# BF self-sounding — decode-level confirmation (question 2, gold standard).
# Three adapters on ch100:
#   - 8812AU  : sounder  (TX NDPA + descriptor NDPA bit + arm sounder engine)
#   - 8821AU  : beamformee (armed, unassociated)
#   - 8814AU  : passive monitor sniffer, DEVOURER_BF_DETECT_REPORT=1
# A <devourer-bf-report> line on the sniffer with SA = 8821AU MAC is the
# direct proof the unassociated responder emitted a VHT Compressed Beamforming
# report. Run armed vs unarmed to show the report only appears when armed.
#
# Usage: sudo tests/bf_report_sniff.sh   (from the devourer-bf worktree)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
OUT="/tmp/bf-report-sniff"
CHANNEL=100
DUR=8
BFER_MAC="57:42:75:05:d6:00"   # canonical SA = NDPA TA (8812AU sounder)

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first"; exit 1; }
mkdir -p "$OUT"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -x WiFiDriverDemo 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "== read 8821AU (beamformee) MAC =="
env DEVOURER_VID=0x2357 DEVOURER_PID=0x0120 DEVOURER_CHANNEL=$CHANNEL \
    timeout 25 "$RXDEMO" > "$OUT/mac_scan.log" 2>&1
BFEE_MAC=$(grep -o "REG_MACID programmed from EFUSE: [0-9a-f:]*" "$OUT/mac_scan.log" \
           | awk '{print $NF}')
[ -n "$BFEE_MAC" ] || { echo "no 8821AU MAC — see $OUT/mac_scan.log"; exit 1; }
echo "8821AU MAC = $BFEE_MAC"
sleep 2

run_cell() { # $1=name $2=arm(0|1)
    local name="$1" arm="$2"
    echo "== cell $name (bfee armed=$arm) =="

    # 8814AU sniffer (passive, whole cell).
    env DEVOURER_PID=0x8813 DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_BF_DETECT_REPORT=1 \
        timeout $((DUR + 30)) "$RXDEMO" > "$OUT/$name.sniff.log" 2>&1 &
    local sniff_pid=$!
    for _ in $(seq 50); do
        grep -q "In Monitor Mode" "$OUT/$name.sniff.log" && break; sleep 0.5
    done

    # 8821AU beamformee.
    local bfee_env=(DEVOURER_VID=0x2357 DEVOURER_PID=0x0120 DEVOURER_CHANNEL=$CHANNEL)
    [ "$arm" = "1" ] && bfee_env+=(DEVOURER_BF_ARM_BFEE="$BFER_MAC")
    env "${bfee_env[@]}" timeout $((DUR + 8)) "$RXDEMO" \
        > "$OUT/$name.bfee.log" 2>&1 &
    local bfee_pid=$!
    for _ in $(seq 40); do
        grep -q "In Monitor Mode" "$OUT/$name.bfee.log" && break; sleep 0.5
    done
    sleep 1

    # 8812AU sounder.
    env DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_TX_RATE=VHT2SS_MCS0 \
        DEVOURER_TX_NDPA_RA="$BFEE_MAC" \
        DEVOURER_TX_NDPA=1 DEVOURER_BF_ARM_SOUNDER=1 \
        timeout "$DUR" "$TXDEMO" > "$OUT/$name.tx.log" 2>&1

    kill "$bfee_pid" "$sniff_pid" 2>/dev/null
    wait "$bfee_pid" "$sniff_pid" 2>/dev/null

    local n
    n=$(grep -c "<devourer-bf-report>" "$OUT/$name.sniff.log")
    echo "  bf-report lines: $n"
    grep "<devourer-bf-report>" "$OUT/$name.sniff.log" | head -3
}

run_cell unarmed 0
sleep 2
run_cell armed 1

echo
echo "== verdict =="
echo "unarmed reports: $(grep -c '<devourer-bf-report>' "$OUT/unarmed.sniff.log")"
echo "armed reports:   $(grep -c '<devourer-bf-report>' "$OUT/armed.sniff.log")"
echo "(expect armed reports with sa=$BFEE_MAC; logs in $OUT)"
