#!/usr/bin/env bash
# Active two-ended sounding sweep (issue #149) — recover a coarse H(f) of
# the actual TX->RX path. The prober hops fixed-rate probe beacons (the
# canonical SA) across the bin list via FastRetune; the sounder sweeps the same
# bins with per-dwell probe-frame RSSI/EVM aggregation (DEVOURER_RX_AGG_SA).
#
# Synchronization is asymmetric-duty, no control channel: the TX cycles all
# bins fast, the RX dwells >= ~2 full TX cycles per bin, so every RX dwell
# overlaps at least one TX visit to its bin. A faded bin shows up as low
# rssi_max / few hits; a dead bin catches nothing.
#
#   sudo tests/sounding_sweep.sh --tx-pid 0x8812 --rx-pid 0xc812 --bins 1,6,11
#   sudo tests/sounding_sweep.sh --tx-pid 0xc812 --rx-pid 0xa81a \
#        --bins 1-13/1 --nb-bw 5              # 5 MHz bins, Jaguar3 both ends
#   sudo tests/sounding_sweep.sh ... --sdr    # + B210 wideband cross-check
#
# Bins use the SweepSpec grammar ("1,6,11", "36-48/4", "5170-5250/5") — one
# spec drives both sides. All bins must be intra-band (FastRetune's fast path;
# a cross-band list still works but pays full retunes at the band edge).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEMO_TX="$ROOT/build/WiFiDriverTxDemo"
DEMO_RX="$ROOT/build/WiFiDriverDemo"

TX_VID=0x0bda TX_PID="" RX_VID=0x0bda RX_PID=""
BINS="1,6,11" ROUNDS=4 RATE=MCS0 DWELL_FRAMES=20 GAP_US=2000 NB_BW=""
SDR=0 SDR_GAIN=45 OUT=/tmp/devourer-sounding

while [ $# -gt 0 ]; do
  case "$1" in
    --tx-vid) TX_VID="$2"; shift 2 ;;
    --tx-pid) TX_PID="$2"; shift 2 ;;
    --rx-vid) RX_VID="$2"; shift 2 ;;
    --rx-pid) RX_PID="$2"; shift 2 ;;
    --bins) BINS="$2"; shift 2 ;;
    --rounds) ROUNDS="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --dwell-frames) DWELL_FRAMES="$2"; shift 2 ;;
    --gap-us) GAP_US="$2"; shift 2 ;;
    --nb-bw) NB_BW="$2"; shift 2 ;;
    --sdr) SDR=1; shift ;;
    --sdr-gain) SDR_GAIN="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$TX_PID" ] && [ -n "$RX_PID" ] || { echo "need --tx-pid and --rx-pid" >&2; exit 2; }

cleanup() { for c in WiFiDriverTxDemo WiFiDriverDemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"; rm -f "$OUT"/sweep.log "$OUT"/sdr.csv
cleanup; sleep 1

# Expand the bin spec (sounding_map.py mirrors src/SweepSpec.h) and size the
# duty cycle: RX dwell >= 2x the TX full-cycle time so every dwell overlaps a
# TX visit; ~12 ms/hop retune margin, ~0.6 ms airtime per probe frame.
CHLIST="$(python3 "$HERE/sounding_map.py" --expand "$BINS")"
NBINS="$(wc -w <<< "$CHLIST")"
[ "$NBINS" -ge 2 ] || { echo "bin spec '$BINS' expands to <2 bins" >&2; exit 2; }
FIRST_CH="$(awk '{print $1}' <<< "$CHLIST")"
CYCLE_MS="$(( NBINS * (DWELL_FRAMES * (GAP_US + 600) / 1000 + 12) ))"
RX_DWELL="$(( 2 * CYCLE_MS ))"
[ "$RX_DWELL" -lt 400 ] && RX_DWELL=400
RX_SECS="$(( (NBINS * ROUNDS * RX_DWELL) / 1000 + 20 ))"
echo "bins: $CHLIST"
echo "duty: TX cycle ~${CYCLE_MS} ms -> RX dwell ${RX_DWELL} ms x $ROUNDS rounds (~${RX_SECS}s)"

echo "== prober: $TX_PID hopping $NBINS bins at $RATE (dwell $DWELL_FRAMES frames) =="
timeout -sINT "$((RX_SECS + 30))" env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
  DEVOURER_CHANNEL="$FIRST_CH" DEVOURER_TX_RATE="$RATE" \
  ${NB_BW:+DEVOURER_NB_BW="$NB_BW"} \
  DEVOURER_HOP_CHANNELS="$BINS" DEVOURER_HOP_DWELL_FRAMES="$DWELL_FRAMES" \
  DEVOURER_HOP_FAST=1 DEVOURER_TX_GAP_US="$GAP_US" \
  "$DEMO_TX" >"$OUT/tx.log" 2>&1 &
for _ in $(seq 60); do grep -q "devourer-hop" "$OUT/tx.log" && break; sleep 0.5; done
grep -q "devourer-hop" "$OUT/tx.log" && echo "  prober hopping" \
  || { echo "  FAIL: prober never hopped (see $OUT/tx.log)"; exit 1; }

if [ "$SDR" = 1 ]; then
  # Wideband ground truth: capture while both ends run, then export per-bin
  # integrated power for the rank-correlation cross-check. Center on the mean
  # bin frequency (python does the math).
  SDR_SECS="$(( RX_SECS < 20 ? RX_SECS : 20 ))"
  CENTER="$(python3 - "$CHLIST" <<'PY'
import sys
chs = [int(c) for c in sys.argv[1].split()]
f = [2484 if c == 14 else (2407 + 5*c if c <= 14 else 5000 + 5*c) for c in chs]
print(f"{(min(f)+max(f))/2*1e6:.0f}")
PY
)"
  echo "== SDR: B210 wideband capture (${SDR_SECS}s, center ${CENTER%??????}.x MHz) =="
  python3 "$HERE/hop_rx_probe.py" --channels "$(tr ' ' ',' <<< "$CHLIST")" \
    --center "$CENTER" --gain "$SDR_GAIN" --duration "$SDR_SECS" \
    --bin-power-csv "$OUT/sdr.csv" >"$OUT/sdr.log" 2>&1 &
  SDR_PID=$!
fi

echo "== sounder: $RX_PID sweeping (SA filter = canonical probe) =="
timeout -sINT "$RX_SECS" env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" \
  DEVOURER_RX_SWEEP="$BINS" DEVOURER_RX_SWEEP_DWELL_MS="$RX_DWELL" \
  DEVOURER_RX_AGG_SA=canon \
  ${NB_BW:+DEVOURER_NB_BW="$NB_BW"} \
  "$DEMO_RX" 2>/dev/null | grep --line-buffered "devourer-energy" \
  | tee "$OUT/sweep.log" || true

[ "$SDR" = 1 ] && wait "${SDR_PID:-}" 2>/dev/null
cleanup

echo; echo "== recovered H(f) map =="
python3 "$HERE/sounding_map.py" --log "$OUT/sweep.log" \
  ${SDR:+$( [ -s "$OUT/sdr.csv" ] && echo --sdr-csv "$OUT/sdr.csv" )}
rc=$?
echo "logs: $OUT/{tx,sweep}.log$( [ "$SDR" = 1 ] && echo " sdr.{log,csv}" )"
exit $rc
