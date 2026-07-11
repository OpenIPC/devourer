#!/usr/bin/env bash
# Is the 8812EU actually RADIATING during an MCS7 TX session, or are the
# frames dropped before the antenna? (issue #238 MCS4+ bisect: garbled
# constellation would still show ~normal SDR duty; a MAC-level drop shows ~0.)
# One SDR read per invocation — a second back-to-back B210 read can fail to
# reacquire and report ~0 (session lesson), so run once per rate.
#   sudo tests/eu_mcs7_sdr_probe.sh [MCS7]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RATE="${1:-MCS7}"
CH="${CH:-36}"; FREQ="${FREQ:-5180e6}"
cleanup(){ sudo -n pkill -x txdemo 2>/dev/null || true; }
trap cleanup EXIT INT TERM

sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$RATE" DEVOURER_TX_GAP_US=0 \
    stdbuf -oL timeout 25 "$ROOT/build/txdemo" \
    >/tmp/eu_sdr_probe_tx.log 2>&1 &
sleep 6
echo "--- SDR duty during $RATE TX (ch$CH) ---"
sudo -n python3 "$ROOT/tests/sdr_duty.py" --freq "$FREQ" --rate 25e6 --secs 5 \
    --gain 60 --bw 20 2>&1 | grep -iE "duty=|error" | head -3
cleanup
