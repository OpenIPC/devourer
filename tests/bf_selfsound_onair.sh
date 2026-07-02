#!/usr/bin/env bash
# BF self-sounding on-air probe (question 2): does an UNASSOCIATED, armed
# beamformee (8821AU) hardware-reply a VHT Compressed Beamforming report to
# our NDPA+NDP (8812AU)?
#
# Detector: B210 burst-chain analysis. Q1 established NDPA+NDP = len-2 chains
# at SIFS gaps. A responding beamformee turns them into len-3 chains
# (NDPA + NDP + report). Cells:
#   A) bfee NOT armed  -> len2 chains dominate, triple_frac ~ 0
#   B) bfee armed      -> triple_frac >> 0 if the responder fires
#
# Usage: sudo tests/bf_selfsound_onair.sh   (from the devourer-bf worktree)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
PY="/usr/bin/python3"
PROBE="$HERE/bf_ndpa_probe.py"
OUT="/tmp/bf-selfsound"
CHANNEL=100
FREQ=5500e6
RATE=10e6
DUR=8
BFER_MAC="57:42:75:05:d6:00"   # canonical SA = NDPA TA

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first"; exit 1; }
mkdir -p "$OUT"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -x WiFiDriverDemo 2>/dev/null
    pkill -f "bf_ndpa_probe.py" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# --- step 0: learn the 8821AU MAC (logged at init as REG_MACID) ------------
echo "== step 0: read 8821AU MAC =="
env DEVOURER_VID=0x2357 DEVOURER_PID=0x0120 DEVOURER_CHANNEL=$CHANNEL \
    timeout 25 "$RXDEMO" > "$OUT/mac_scan.log" 2>&1
BFEE_MAC=$(grep -o "REG_MACID programmed from EFUSE: [0-9a-f:]*" "$OUT/mac_scan.log" \
           | awk '{print $NF}')
[ -n "$BFEE_MAC" ] || { echo "could not read 8821AU MAC — see $OUT/mac_scan.log"; exit 1; }
echo "8821AU MAC = $BFEE_MAC"
sleep 2

run_cell() { # $1=name  $2=arm_bfee(0|1)
    local name="$1" arm="$2"
    echo "== cell $name (bfee armed=$arm) =="
    # Beamformee RX up first (arming happens in Init).
    local bfee_env=(DEVOURER_VID=0x2357 DEVOURER_PID=0x0120 DEVOURER_CHANNEL=$CHANNEL)
    [ "$arm" = "1" ] && bfee_env+=(DEVOURER_BF_ARM_BFEE="$BFER_MAC")
    env "${bfee_env[@]}" timeout $((DUR + 22)) "$RXDEMO" \
        > "$OUT/$name.bfee.log" 2>&1 &
    local bfee_pid=$!
    # Wait for the bfee to reach monitor mode before sounding.
    for _ in $(seq 40); do
        grep -q "In Monitor Mode" "$OUT/$name.bfee.log" && break
        sleep 0.5
    done
    grep -q "In Monitor Mode" "$OUT/$name.bfee.log" || echo "warn: bfee init slow"

    "$PY" "$PROBE" --freq "$FREQ" --rate "$RATE" --duration $((DUR - 1)) \
        > "$OUT/$name.probe" 2> "$OUT/$name.probe.err" &
    local probe_pid=$!
    sleep 1.5

    env DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CHANNEL \
        DEVOURER_TX_RATE=VHT2SS_MCS0 \
        DEVOURER_TX_NDPA_RA="$BFEE_MAC" \
        DEVOURER_TX_NDPA=1 DEVOURER_BF_ARM_SOUNDER=1 \
        timeout "$DUR" "$TXDEMO" > "$OUT/$name.tx.log" 2>&1
    wait "$probe_pid"
    kill "$bfee_pid" 2>/dev/null
    wait "$bfee_pid" 2>/dev/null
    grep "bf-probe \(summary\|chains\)" "$OUT/$name.probe" || \
        echo "(no summary — see $OUT/$name.probe.err)"
}

run_cell unarmed 0
sleep 2
run_cell armed 1

echo
echo "== verdict =="
a=$(grep -o 'triple_frac=[0-9.]*' "$OUT/unarmed.probe" | cut -d= -f2)
b=$(grep -o 'triple_frac=[0-9.]*' "$OUT/armed.probe" | cut -d= -f2)
echo "unarmed triple_frac=${a:-?}  armed triple_frac=${b:-?}  (logs in $OUT)"
