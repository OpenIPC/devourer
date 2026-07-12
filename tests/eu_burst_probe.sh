#!/usr/bin/env bash
# Burst-length fingerprint of the 8812EU TX at a pinned MCS (issue #238):
# run txdemo at the given rate with a fixed gap and measure the on-air burst
# duration histogram with the B210. One SDR read per invocation.
#   sudo tests/eu_burst_probe.sh MCS4
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RATE="${1:-MCS4}"
CH="${CH:-36}"; FREQ="${FREQ:-5180e6}"
cleanup(){ sudo -n pkill -x txdemo 2>/dev/null || true; }
trap cleanup EXIT INT TERM

sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$RATE" DEVOURER_TX_GAP_US=2000 \
    stdbuf -oL timeout 25 "$ROOT/build/txdemo" \
    >/tmp/eu_burst_tx.log 2>&1 &
sleep 6
echo "--- burst fingerprint during $RATE TX (ch$CH) ---"
sudo -n python3 "$ROOT/tests/sdr_burst_len.py" --freq "$FREQ" --secs 4
cleanup
