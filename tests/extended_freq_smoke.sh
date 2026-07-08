#!/usr/bin/env bash
# Extended-frequency on-air smoke: does devourer actually TX+RX at an
# out-of-band 5 GHz channel the vendor tables don't cover? (Part B lifted the
# ChannelFreq.h 5895 MHz cap and opened the per-generation channel buckets.)
#
# Two devourer adapters: TX injects the canonical beacon (SA 57:42:75:05:d6:00)
# at CHAN; RX counts rx.txhit at the same channel. Kernel drivers can't tune the
# extended band, so devourer-vs-devourer is the authoritative cell. A control
# run at a standard channel (149) confirms the rig itself is sound.
#
#   TX_PID=8812 RX_PID=c812 sudo -E tests/extended_freq_smoke.sh
#   CHAN=180 CTRL=149 SECS=8 TX_PID=8812 RX_PID=c812 sudo -E tests/extended_freq_smoke.sh
#
# CHAN 180 = 5900 MHz (5000 + 5*180), ~75 MHz above the ch165 regulatory edge.
set -u
cd "$(dirname "$0")/.."

TX_PID=${TX_PID:?set TX_PID=<usb pid of the TX adapter, e.g. 8812>}
RX_PID=${RX_PID:?set RX_PID=<usb pid of the RX adapter, e.g. c812>}
TX_VID=${TX_VID:-0x0bda}
RX_VID=${RX_VID:-0x0bda}
CHAN=${CHAN:-180}   # extended: 5900 MHz
CTRL=${CTRL:-149}   # control: in-band 5745 MHz
SECS=${SECS:-8}

cleanup() { sudo pkill -x txdemo 2>/dev/null; sudo pkill -x rxdemo 2>/dev/null; }
trap cleanup EXIT

run_cell() {
  local ch=$1 label=$2
  local rxlog txlog
  rxlog=$(mktemp); txlog=$(mktemp)
  echo "-- $label: ch$ch ($((5000 + 5 * ch)) MHz), ${SECS}s --"
  sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="0x${RX_PID}" \
      DEVOURER_CHANNEL="$ch" DEVOURER_LOG_LEVEL=silent \
      build/rxdemo >"$rxlog" 2>/dev/null &
  sleep 3   # RX bring-up before TX starts
  sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="0x${TX_PID}" \
      DEVOURER_CHANNEL="$ch" DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=silent \
      build/txdemo >"$txlog" 2>/dev/null &
  sleep "$SECS"
  cleanup
  sleep 1
  # rx.txhit carries a cumulative "hits" count; take the last one seen.
  local hits
  hits=$(grep -F '"ev":"rx.txhit"' "$rxlog" | tail -1 | grep -oE '"hits":[0-9]+' | grep -oE '[0-9]+')
  hits=${hits:-0}
  echo "   rx.txhit hits=$hits"
  rm -f "$rxlog" "$txlog"
  echo "$hits"
}

echo "=== extended-freq smoke: TX ${TX_VID}:${TX_PID} -> RX ${RX_VID}:${RX_PID} ==="
ctrl_hits=$(run_cell "$CTRL" "control" | tail -1)
ext_hits=$(run_cell "$CHAN" "extended" | tail -1)

echo "--- control ch$CTRL: $ctrl_hits hits | extended ch$CHAN: $ext_hits hits ---"
if [ "$ctrl_hits" -lt 1 ]; then
  echo "INCONCLUSIVE: control cell saw no frames — rig/adapters suspect, not the extended band"
  exit 2
fi
if [ "$ext_hits" -ge 1 ]; then
  echo "PASS: on-air TX+RX confirmed at extended ch$CHAN ($((5000 + 5 * CHAN)) MHz)"
  exit 0
fi
echo "FAIL: control worked but no frames at extended ch$CHAN — this chip may not lock there"
exit 1
