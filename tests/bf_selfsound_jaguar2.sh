#!/usr/bin/env bash
# BF self-sounding cross-generation test, Jaguar-2 (8822BU) edition — both
# directions plus the single-radio ground station:
#
#   cells A/B (beamformee direction, unarmed/armed):
#     Jaguar-1 sounder (default 8814AU — proven single-radio GS path)
#     8822BU  : beamformee   (shared recipe, kBfeeJaguar23) — chip under test
#     Jaguar-1 sniffer (default 8821AU), DEVOURER_BF_DETECT_REPORT=4
#     PASS = <devourer-bf-report> with SA = the Jaguar-2 beamformee MAC
#     (00:e0:4c:88:22:bb, programmed at arm time) only when armed.
#
#   cell C (sounder direction + single-radio ground station):
#     Jaguar-1 beamformee (default 8821AU — the original gate-2 chip; EFUSE
#     MAC learned from its init log)
#     8822BU  : sounder + self-capture (DEVOURER_TX_WITH_RX=thread) — under test
#     Jaguar-1 sniffer (default 8814AU) cross-check
#     PASS = <devourer-bf-report> lines in the 8822BU's OWN log (self-capture).
#     Sniffer>0 with self-capture==0 isolates an RX-side issue; both 0 = the
#     NDPA descriptor / HW NDP never fired.
#
# Usage: sudo tests/bf_selfsound_jaguar2.sh [bfee_pid] [bfee_vid]
#        defaults: 0x012d 0x2357 (TP-Link Archer T3U); pass "0xb82c 0x0bda"
#        for a plain Realtek-VID 8822BU. RUN_MU=1 adds an MU-report cell.
#        J1 helpers override via J1SND_VID/PID (sounder), J1AUX_VID/PID
#        (sniffer in A/B, beamformee in C).

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
OUT="/tmp/bf-jaguar2"
CHANNEL=100
DUR=8
BFER_MAC="57:42:75:05:d6:00"          # canonical SA = NDPA TA
BFEE_MAC="00:e0:4c:88:22:bb"          # programmed onto the Jaguar-2 beamformee
J2_PID="${1:-0x012d}"                 # Archer T3U default
J2_VID="${2:-0x2357}"
J1SND_VID="${J1SND_VID:-0x0bda}"      # cells A/B sounder + cell C sniffer
J1SND_PID="${J1SND_PID:-0x8813}"      #   (8814AU)
J1AUX_VID="${J1AUX_VID:-0x2357}"      # cells A/B sniffer + cell C beamformee
J1AUX_PID="${J1AUX_PID:-0x0120}"      #   (8821AU, Archer T2U Plus)

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first"; exit 1; }
mkdir -p "$OUT"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -x WiFiDriverDemo 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

start_sniffer() { # $1=name $2=vid $3=pid  -> sets sniff_pid
    env DEVOURER_VID=$2 DEVOURER_PID=$3 DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_BF_DETECT_REPORT=4 \
        timeout $((DUR + 90)) "$RXDEMO" > "$OUT/$1.sniff.log" 2>&1 &
    sniff_pid=$!
    # RX demo signals monitor-ready with "Listening air..." (the Init path).
    for _ in $(seq 40); do
        grep -q "Listening air" "$OUT/$1.sniff.log" && break; sleep 0.5; done
}

run_bfee_cell() { # $1=name $2=arm(0|1) $3=mu(0|1)
    local name="$1" arm="$2" mu="${3:-0}"
    echo "== cell $name (Jaguar-2 bfee armed=$arm mu=$mu, $J2_VID:$J2_PID) =="
    start_sniffer "$name" "$J1AUX_VID" "$J1AUX_PID"

    # Jaguar-2 beamformee (arming happens in Init, after bring_up).
    local be=(DEVOURER_VID=$J2_VID DEVOURER_PID=$J2_PID DEVOURER_CHANNEL=$CHANNEL)
    [ "$arm" = 1 ] && be+=(DEVOURER_BF_ARM_BFEE="$BFER_MAC")
    [ "$mu" = 1 ] && be+=(DEVOURER_BF_ARM_BFEE_MU=1)
    env "${be[@]}" timeout $((DUR + 30)) "$RXDEMO" > "$OUT/$name.bfee.log" 2>&1 &
    local bfee_pid=$!
    # Jaguar-2 init includes DLFW + IQK; wait for the RX loop banner.
    for _ in $(seq 80); do
        grep -q "entering RX loop" "$OUT/$name.bfee.log" && break; sleep 0.5; done
    grep -q "armed" "$OUT/$name.bfee.log" && echo "  (bfee arm logged)"
    sleep 1

    # Jaguar-1 sounder.
    local tx=(DEVOURER_VID=$J1SND_VID DEVOURER_PID=$J1SND_PID
              DEVOURER_CHANNEL=$CHANNEL DEVOURER_TX_RATE=VHT2SS_MCS0
              DEVOURER_TX_NDPA_RA="$BFEE_MAC" DEVOURER_TX_NDPA=1
              DEVOURER_BF_ARM_SOUNDER=1)
    [ "$mu" = 1 ] && tx+=(DEVOURER_TX_NDPA_MU=1)
    env "${tx[@]}" timeout "$DUR" "$TXDEMO" > "$OUT/$name.tx.log" 2>&1

    kill "$bfee_pid" "$sniff_pid" 2>/dev/null
    wait "$bfee_pid" "$sniff_pid" 2>/dev/null
    local n
    n=$(grep -c "<devourer-bf-report>" "$OUT/$name.sniff.log")
    echo "  sniffer bf-report lines: $n"
    grep "<devourer-bf-report>" "$OUT/$name.sniff.log" | grep "$BFEE_MAC" | head -2
}

run_sounder_cell() { # cell C: Jaguar-2 as beamformer + single-radio self-capture
    local name="sounder_gs"
    echo "== cell $name (Jaguar-2 sounder + TX_WITH_RX self-capture) =="

    # Learn the Jaguar-1 beamformee's EFUSE MAC — the NDPA RA in this direction.
    env DEVOURER_VID=$J1AUX_VID DEVOURER_PID=$J1AUX_PID DEVOURER_CHANNEL=$CHANNEL \
        timeout 25 "$RXDEMO" > "$OUT/mac_scan.log" 2>&1
    local j1_mac
    j1_mac=$(grep -o "REG_MACID programmed from EFUSE: [0-9a-f:]*" "$OUT/mac_scan.log" \
             | awk '{print $NF}')
    [ -n "$j1_mac" ] || { echo "  could not read J1 bfee MAC — see $OUT/mac_scan.log"; return 1; }
    echo "  Jaguar-1 bfee MAC = $j1_mac"
    sleep 2

    start_sniffer "$name" "$J1SND_VID" "$J1SND_PID"

    # Jaguar-1 armed beamformee.
    env DEVOURER_VID=$J1AUX_VID DEVOURER_PID=$J1AUX_PID DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_BF_ARM_BFEE="$BFER_MAC" \
        timeout $((DUR + 30)) "$RXDEMO" > "$OUT/$name.bfee.log" 2>&1 &
    local bfee_pid=$!
    for _ in $(seq 40); do
        grep -q "Listening air" "$OUT/$name.bfee.log" && break; sleep 0.5; done
    sleep 1

    # Jaguar-2 ground station: one adapter sounds AND captures its own reports.
    env DEVOURER_VID=$J2_VID DEVOURER_PID=$J2_PID DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_TX_RATE=VHT2SS_MCS0 DEVOURER_TX_WITH_RX=thread \
        DEVOURER_BF_ARM_SOUNDER="$BFER_MAC" DEVOURER_TX_NDPA=1 \
        DEVOURER_TX_NDPA_RA="$j1_mac" DEVOURER_BF_DETECT_REPORT=4 \
        timeout $((DUR + 25)) "$TXDEMO" > "$OUT/$name.gs.log" 2>&1

    kill "$bfee_pid" "$sniff_pid" 2>/dev/null
    wait "$bfee_pid" "$sniff_pid" 2>/dev/null
    local self air
    self=$(grep -c "<devourer-bf-report>" "$OUT/$name.gs.log")
    air=$(grep -c "<devourer-bf-report>" "$OUT/$name.sniff.log")
    echo "  self-captured reports: $self   sniffer cross-check: $air"
    grep "<devourer-bf-report>" "$OUT/$name.gs.log" | head -2
}

run_bfee_cell "unarmed" 0
sleep 2
run_bfee_cell "armed" 1
sleep 2
[ "${RUN_MU:-0}" = 1 ] && { run_bfee_cell "armed_mu" 1 1; sleep 2; }
run_sounder_cell

echo
echo "== verdict ($J2_VID:$J2_PID) =="
echo "bfee direction — unarmed reports: $(grep '<devourer-bf-report>' "$OUT/unarmed.sniff.log" | grep -c "$BFEE_MAC")"
echo "bfee direction — armed   reports: $(grep '<devourer-bf-report>' "$OUT/armed.sniff.log" | grep -c "$BFEE_MAC")"
[ "${RUN_MU:-0}" = 1 ] && \
echo "bfee direction — armed MU reports: $(grep '<devourer-bf-report>' "$OUT/armed_mu.sniff.log" | grep -c "$BFEE_MAC")"
echo "sounder direction — self-captured: $(grep -c '<devourer-bf-report>' "$OUT/sounder_gs.gs.log" 2>/dev/null)"
echo "(logs in $OUT)"
