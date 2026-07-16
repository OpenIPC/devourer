#!/usr/bin/env bash
# kestrel_streamtx_sdr.sh — validate the actual OpenIPC video path (streamtx:
# PSDU-from-stdin wrapped in a probe-req/mgmt header) airs on the 8852BU, via a
# single B210 SDR duty read (ceiling-free). streamtx is fed a continuous random
# byte stream framed as length-prefixed PSDUs.
#
#   sudo tests/kestrel_streamtx_sdr.sh [channel] [seconds] [rate]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; SECS=${2:-4}; RATE=${3:-MCS2}
if [ "$CH" -le 14 ]; then FREQ=$(( 2407 + CH*5 ))e6; else FREQ=$(( 5000 + CH*5 ))e6; fi
TX_ID="35bc:0108"; TX_HUB="3-2.3"; TX_PORT="3"
TXLOG="/tmp/kestrel_streamtx.log"; FEED="/tmp/kestrel_streamtx_feed"
[ -x build/streamtx ] || { echo "FAIL: build streamtx"; exit 2; }

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d i; d=$(sysdir_for "$1")
  [ -n "$d" ] && for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true
  done
}
TXPID=""; FEEDPID=""
cleanup() {
  [ -n "$FEEDPID" ] && kill "$FEEDPID" 2>/dev/null
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  pkill -9 -x -f "build/streamtx" 2>/dev/null || true
  rm -f "$FEED"
}
trap cleanup EXIT

# length-prefixed PSDU feeder: emit [u16 len][len random bytes] forever.
mkfifo "$FEED" 2>/dev/null || { rm -f "$FEED"; mkfifo "$FEED"; }
# Pre-generate a chunk of many length-prefixed records once (fast), then stream
# it into the FIFO in a loop so streamtx is not feeder-rate-limited.
CHUNK="/tmp/kestrel_streamtx_chunk.bin"
python3 - "$CHUNK" <<'PY'
import os,sys,struct
with open(sys.argv[1],"wb") as f:
    rec=struct.pack("<I",1024)+os.urandom(1024)
    for _ in range(2000):
        f.write(rec)
PY
( for _ in $(seq 1 200); do cat "$CHUNK"; done > "$FEED" ) &
FEEDPID=$!

echo ">> freq ${FREQ} (ch$CH) rate=$RATE"
echo ">> bring up streamtx (OpenIPC mgmt-wrapped PSDU path)"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6
unbind_kernel "$TX_ID"; sleep 1
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_LOG_LEVEL=warn DEVOURER_TX_RATE="$RATE" \
  build/streamtx --max-psdu 2048 <"$FEED" >"$TXLOG" 2>&1 &
TXPID=$!
sleep 4
kill -0 "$TXPID" 2>/dev/null || { echo "FAIL: streamtx died"; tail -6 "$TXLOG"; exit 2; }

echo ">> single SDR duty read while streaming"
LIVE=$(python3 tests/sdr_duty.py --freq "$FREQ" --secs "$SECS" --mcs 2 --bw 20 2>/dev/null \
        | grep -iE "duty" | tail -1)
SENT=$(grep -cE '"ev":"stream.tx"' "$TXLOG" 2>/dev/null || echo 0)
FAILS=$(grep -cE "rc=-7|bulk-OUT.*failed" "$TXLOG" 2>/dev/null || echo 0)

kill "$TXPID" 2>/dev/null; TXPID=""
echo "=================================================================="
echo "streamtx tx events: $SENT   bulk-OUT failures: $FAILS"
echo "live SDR: ${LIVE:-<none>}"
echo "No stall + duty above the ~18% (2.4G) / ~5% (5G) ambient == video path airs."
