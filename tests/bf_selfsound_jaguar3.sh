#!/usr/bin/env bash
# BF self-sounding cross-generation test: does a Jaguar-3 (8822C/8822E)
# beamformee, armed with the shared BeamformingSounder recipe, hardware-reply
# a VHT Compressed Beamforming report to an 8812AU (Jaguar-1) beamformer —
# with no association?
#
#   8812AU  : sounder      (proven Jaguar-1 path)
#   8822C/E : beamformee   (shared recipe, kBfeeJaguar23) — the chip under test
#   8814AU  : passive monitor sniffer, DEVOURER_BF_DETECT_REPORT=1
#
# PASS = <devourer-bf-report> with SA = the Jaguar-3 beamformee MAC
# (00:e0:4c:88:22:ce, programmed at arm time) only when armed.
#
# Usage: sudo tests/bf_selfsound_jaguar3.sh <bfee_pid>   (c812 | a81a)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
OUT="/tmp/bf-jaguar3"
CHANNEL=100
DUR=8
BFER_MAC="57:42:75:05:d6:00"          # 8812AU sounder = NDPA TA
BFEE_MAC="00:e0:4c:88:22:ce"          # programmed onto the Jaguar-3 beamformee
BFEE_PID="${1:-0xc812}"               # c812 = 8822CU, a81a = 8822EU

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first"; exit 1; }
mkdir -p "$OUT"
tag="$(echo "$BFEE_PID" | tr -d 'x')"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -x WiFiDriverDemo 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

run_cell() { # $1=name $2=arm(0|1)
    local name="$1" arm="$2"
    echo "== cell $name (Jaguar-3 bfee armed=$arm, pid=$BFEE_PID) =="

    # Sniffer (whole cell) — 8814AU at ch100 (the 8812AU is the sounder, the
    # 8821AU is free but either works per the README 5GHz benchmarks). It sits
    # on a different USB bus from the Jaguar-3 chip.
    env DEVOURER_PID=0x8813 DEVOURER_CHANNEL=$CHANNEL DEVOURER_BF_DETECT_REPORT=1 \
        timeout $((DUR + 90)) "$RXDEMO" > "$OUT/$name.sniff.log" 2>&1 &
    local sniff_pid=$!
    # RX demo signals monitor-ready with "Listening air..." (the Init path);
    # "In Monitor Mode" is only the TX/InitWrite banner.
    for _ in $(seq 40); do
        grep -q "Listening air" "$OUT/$name.sniff.log" && break; sleep 0.5; done

    # Jaguar-3 beamformee.
    local be=(DEVOURER_PID=$BFEE_PID DEVOURER_CHANNEL=$CHANNEL)
    [ "$arm" = 1 ] && be+=(DEVOURER_BF_ARM_BFEE="$BFER_MAC")
    env "${be[@]}" timeout $((DUR + 12)) "$RXDEMO" > "$OUT/$name.bfee.log" 2>&1 &
    local bfee_pid=$!
    # Jaguar-3 init is long (DLFW + cals); wait for the RX loop banner.
    for _ in $(seq 80); do
        grep -q "entering RX loop" "$OUT/$name.bfee.log" && break; sleep 0.5; done
    grep -q "armed" "$OUT/$name.bfee.log" && echo "  (bfee arm logged)"
    sleep 1

    # 8812AU sounder.
    env DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CHANNEL DEVOURER_TX_RATE=VHT2SS_MCS0 \
        DEVOURER_TX_NDPA_RA="$BFEE_MAC" DEVOURER_TX_NDPA=1 DEVOURER_BF_ARM_SOUNDER=1 \
        timeout "$DUR" "$TXDEMO" > "$OUT/$name.tx.log" 2>&1

    kill "$bfee_pid" "$sniff_pid" 2>/dev/null
    wait "$bfee_pid" "$sniff_pid" 2>/dev/null
    local n
    n=$(grep -c "<devourer-bf-report>" "$OUT/$name.sniff.log")
    echo "  bf-report lines: $n"
    grep "<devourer-bf-report>" "$OUT/$name.sniff.log" | grep "$BFEE_MAC" | head -2
}

run_cell "${tag}_unarmed" 0
sleep 2
run_cell "${tag}_armed" 1

echo
echo "== verdict ($BFEE_PID) =="
echo "unarmed reports from bfee: $(grep '<devourer-bf-report>' "$OUT/${tag}_unarmed.sniff.log" | grep -c "$BFEE_MAC")"
echo "armed   reports from bfee: $(grep '<devourer-bf-report>' "$OUT/${tag}_armed.sniff.log" | grep -c "$BFEE_MAC")"
echo "(logs in $OUT)"
