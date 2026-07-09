#!/usr/bin/env bash
# xtal_cfo_sweep.sh — validate the crystal-cap trim lever (issue #217). Two
# modes:
#
#   sdr   (default): arm a CW tone on the DUT, step DEVOURER_XTAL_CAP live on
#          the warmed carrier, and measure the LO peak-frequency shift per cap
#          with the B210. Proves the trim moves the reference oscillator
#          (~1 ppm across the range) independent of decode. Measure the middle
#          of the range — the crystal pulling curve saturates at the extremes,
#          and near DC the peak-finder can lock onto the tone image.
#
#   decode: sweep the trim on one end of a marginal narrowband pair and count
#          delivered frames (the issue's bimodal->stable test). Note the
#          bimodality is intermittent per thermal state — a session where the
#          pair is already stable can't show the flip; the sdr mode is the
#          deterministic proof.
#
#   sudo tests/xtal_cfo_sweep.sh sdr    [PID=0xc812] [FREQ=5220e6]
#   sudo tests/xtal_cfo_sweep.sh decode [TX=0bda:c812] [RX=0bda:c811]
set -u
cd "$(dirname "$0")/.."
MODE=${1:-sdr}

cleanup() { sudo pkill -INT -x txdemo 2>/dev/null; sudo pkill -INT -x rxdemo 2>/dev/null;
            sleep 1; sudo pkill -KILL -x txdemo rxdemo 2>/dev/null; }
trap cleanup EXIT INT TERM

if [ "$MODE" = sdr ]; then
  PID=${PID:-0xc812}; FREQ=${FREQ:-5220e6}; CH=${CH:-44}
  # Live-step the cap on a warmed CW carrier; measure the LO peak per step.
  sudo env DEVOURER_VID=0x0bda DEVOURER_PID=$PID DEVOURER_CHANNEL=$CH \
      DEVOURER_CW_TONE=1 DEVOURER_XTAL_CAP=0x20 \
      DEVOURER_XTAL_STEP="0x10,0x20,0x30,0x40,0x50" DEVOURER_XTAL_STEP_MS=8000 \
      timeout -s INT -k 5 90 ./build/txdemo >/tmp/xcfo_tx.log 2>&1 &
  echo "warming the crystal 30s before stepping..."; sleep 30
  echo "LO peak offset (Hz) vs live crystal-cap step (one warmed chip):"
  for cap in 0x10 0x20 0x30 0x40 0x50; do
    hz=$(sudo python3 tests/sdr_cw_peak.py "$FREQ" 2>/dev/null | tail -1)
    printf "  cap=%-4s : %s Hz\n" "$cap" "$hz"
    sleep 8
  done
  cleanup
else
  TX=${2:-0bda:c812}; RX=${3:-0bda:c811}; CH=${CH:-44}; N=${N:-3}
  TXV=${TX%:*}; TXP=${TX#*:}; RXV=${RX%:*}; RXP=${RX#*:}
  hits() { grep '"ev":"rx.txhit"' "$1" | tail -1 | grep -oE '"hits":[0-9]+' | cut -d: -f2; }
  cell() { local cap="$1" out=""
    for i in $(seq 1 "$N"); do
      sudo env DEVOURER_VID=0x$TXV DEVOURER_PID=0x$TXP DEVOURER_CHANNEL=$CH DEVOURER_NB_BW=5 \
          ${cap:+DEVOURER_XTAL_CAP=$cap} DEVOURER_TX_GAP_US=500 \
          timeout -s INT -k 5 26 ./build/txdemo >/tmp/xcfo_tx.log 2>&1 &
      sleep 9
      sudo env DEVOURER_VID=0x$RXV DEVOURER_PID=0x$RXP DEVOURER_CHANNEL=$CH DEVOURER_NB_BW=5 \
          timeout -s INT -k 5 10 ./build/rxdemo >/tmp/xcfo_rx.log 2>&1
      out="$out $(hits /tmp/xcfo_rx.log)"
      sudo pkill -INT -x txdemo 2>/dev/null; wait 2>/dev/null; sleep 2
    done
    printf "  cap=%-6s :%s\n" "${cap:-default}" "$out"
  }
  echo "5 MHz decode vs TX crystal-cap ($TX -> $RX, ch$CH, $N bring-ups):"
  cell ""
  for c in 0x00 0x10 0x20 0x30 0x40 0x50 0x60; do cell "$c"; done
fi
