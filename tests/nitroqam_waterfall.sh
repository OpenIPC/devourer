#!/usr/bin/env bash
# 256-QAM VHT on the 2.4 GHz band ("NitroQAM"/"TurboQAM") — delivery-vs-TX-power
# waterfall against the 11n 64-QAM ceiling at the same streams and bandwidth.
#
# VHT is an 802.11ac (5 GHz) PPDU; airing it on 2.4 GHz is a vendor extension,
# not a standard mode. devourer's TX path never reads the band when resolving a
# rate, so DEVOURER_TX_RATE=VHT2SS_MCS9/40 on channel 6 is already selectable —
# this harness measures whether the baseband actually emits it, what a peer
# decodes, and what the 256-QAM constellation costs in SNR over 64-QAM.
#
# Method mirrors ldpc_waterfall.sh: one long-lived ground rxdemo whose log is
# sliced by byte offset per cell, the emitter stepping a flat TXAGC index
# (DEVOURER_TX_PWR -> SetTxPowerIndexOverride, 0.5 dB/step on Jaguar1/2,
# 0.25 dB on Jaguar3). Per cell: delivered = sum of rx.energy frames,
# sent = the final tx.stats submitted.
#
# The extra gate: rx.txhit carries the decoded DESC_RATE, so each cell also
# reports how many sampled frames decoded as the *commanded* modulation. A cell
# that delivers frames which decode as HT is a fallback, not a 256-QAM pass.
#
# Bench notes (docs/bench-testing-near-field.md): sweep the noise-limited low
# index regime. Two adapters ~30 cm apart saturate the RX front end at full
# power — if index 0 already delivers ~100%, add attenuation or distance.
#
#   sudo tests/nitroqam_waterfall.sh --emit-vid 0x2357 --emit-pid 0x012d \
#        --ground-vid 0x0bda --ground-pid 0xb812 --channel 6
#
# Output: $OUT/points.jsonl (one JSON per cell) + per-cell raw logs.
# Analysis: python3 tests/nitroqam_waterfall.py report $OUT/points.jsonl
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

EMIT_VID=0x0bda EMIT_PID="" GROUND_VID=0x0bda GROUND_PID=""
CHANNEL=6 BW=40 ENCS=""
# Power axis. PWR_MODE=offset steps DEVOURER_TX_PWR_OFFSET_QDB — a quarter-dB
# shift of the chip's own efuse-calibrated per-rate table, shape preserved. That
# is the right lever for a waterfall: the alternative, the flat DEVOURER_TX_PWR
# index, forces both paths to one level and zeroes the per-rate diffs, which
# measurably degrades the link (bench: a pair that reaches MCS4 at calibrated
# power tops out at MCS1 under a flat index). PWR_MODE=flat keeps the index for
# comparison against harnesses that use it; PWR_MODE=none pins the calibrated
# default and sweeps nothing.
PWR_MODE=offset
PWR_START=-60 PWR_STOP=0 PWR_STEP=8 SECS=6
# VBUS cold-cycle the emitter before every cell (REGRESS_VBUS_MAP, uhubctl).
# This is not optional hygiene — the chip retains state across a warm re-init,
# and a run of back-to-back devourer sessions progressively degrades high-order
# constellation TX until 64-QAM and up stop decoding entirely, which reads
# exactly like a link too weak to carry them. Measured on an 8812AU: MCS7
# delivered 0% warm and 58% after a cold cycle, same power, same channel, same
# gap. Leave this on unless the adapter's hub cannot switch VBUS.
COLD=${COLD:-1}
OUT=/tmp/devourer-nitroqam-waterfall

BASELINE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --emit-vid) EMIT_VID="$2"; shift 2 ;;
    --emit-pid) EMIT_PID="$2"; shift 2 ;;
    --ground-vid) GROUND_VID="$2"; shift 2 ;;
    --ground-pid) GROUND_PID="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --bw) BW="$2"; shift 2 ;;
    --encs) ENCS="$2"; shift 2 ;;
    --baseline) BASELINE="$2"; shift 2 ;;
    --pwr-mode) PWR_MODE="$2"; shift 2 ;;
    --pwr-start) PWR_START="$2"; shift 2 ;;
    --pwr-stop) PWR_STOP="$2"; shift 2 ;;
    --pwr-step) PWR_STEP="$2"; shift 2 ;;
    --secs) SECS="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$EMIT_PID" ] && [ -n "$GROUND_PID" ] || {
  echo "need --emit-pid and --ground-pid" >&2; exit 2; }
[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build first" >&2; exit 2; }
[ "$CHANNEL" -le 14 ] || echo "warning: ch$CHANNEL is 5 GHz — VHT there is the" \
  "standard mode, not the extension this harness is about" >&2

# Default set: the two 256-QAM points against the 11n 64-QAM ceiling at the same
# 2 streams and bandwidth. MCS15 is the highest standards-only 2SS rate, so the
# horizontal shift between the curves is the price of the denser constellation.
[ -n "$ENCS" ] || ENCS="MCS15/$BW/SGI,VHT2SS_MCS8/$BW,VHT2SS_MCS9/$BW"
[ -n "$BASELINE" ] || BASELINE="MCS15/$BW/SGI"

mkdir -p "$OUT"
RXLOG="$OUT/ground.rx.jsonl"
POINTS="$OUT/points.jsonl"
: > "$POINTS"

# Real VBUS cycle for the emitter, keyed by VID:PID in REGRESS_VBUS_MAP
# ("VID:PID=hub,port;..."). The `authorized` toggle is deliberately NOT used as
# a fallback here: it re-enumerates without removing power, so it does not clear
# the chip state this exists to clear (docs/adapter-doctor.md).
# $1=vid $2=pid. Both ends need this, not just the transmitter: a receiver that
# has been through many warm sessions loses high-order decode the same way a
# transmitter loses high-order modulation.
cold_cycle() {
  [ "$COLD" = 1 ] || return 0
  local map=${REGRESS_VBUS_MAP:-} key entry hubport
  [ -n "$map" ] || return 0
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || true
  [ -n "${entry:-}" ] || return 0
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -x txdemo 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cold_cycle "$GROUND_VID" "$GROUND_PID"

# Ground RX up once for the whole sweep (bring-up is ~6 s; per-cell windows are
# carved out of its log by byte offset). The canonical-SA filter keeps ambient
# 2.4 GHz traffic — of which there is plenty — out of the delivery count.
DEVOURER_VID="$GROUND_VID" DEVOURER_PID="$GROUND_PID" \
DEVOURER_CHANNEL="$CHANNEL" DEVOURER_BW="$BW" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 \
DEVOURER_LINKHEALTH=1 DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 8
kill -0 "$RXPID" 2>/dev/null || {
  echo "ground RX died — see $OUT/ground.rx.stderr" >&2; exit 1; }

IFS=',' read -ra ENC_ARR <<<"$ENCS"
for ENC in "${ENC_ARR[@]}"; do
  IDX="$PWR_START"
  while [ "$IDX" -le "$PWR_STOP" ]; do
    TAG="$(echo "$ENC" | tr '/' '-')_idx$IDX"
    TXLOG="$OUT/tx_$TAG.jsonl"
    echo "[$(date +%H:%M:%S)] cell enc=$ENC idx=$IDX ..."
    cold_cycle "$EMIT_VID" "$EMIT_PID"
    OFF0=$(stat -c%s "$RXLOG")
    PWRENV=()
    case "$PWR_MODE" in
      offset) PWRENV=(DEVOURER_TX_PWR_OFFSET_QDB="$IDX") ;;
      flat)   PWRENV=(DEVOURER_TX_PWR="$IDX") ;;
      none)   ;;
      *) echo "bad --pwr-mode: $PWR_MODE (offset|flat|none)" >&2; exit 2 ;;
    esac
    env DEVOURER_VID="$EMIT_VID" DEVOURER_PID="$EMIT_PID" \
    DEVOURER_CHANNEL="$CHANNEL" DEVOURER_HOP_BW="$BW" \
    DEVOURER_TX_RATE="$ENC" "${PWRENV[@]}" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 12)) "$TXDEMO" \
        > "$TXLOG" 2>"$OUT/tx_$TAG.stderr" &
    TXPID=$!
    # Let bring-up finish, then measure for SECS, then a graceful stop (the
    # final tx.stats needs a clean SIGTERM exit, not SIGKILL).
    sleep $((SECS + 6))
    kill -TERM "$TXPID" 2>/dev/null
    wait "$TXPID" 2>/dev/null
    sleep 1
    OFF1=$(stat -c%s "$RXLOG")
    python3 "$HERE/nitroqam_waterfall.py" point \
        "$TXLOG" "$RXLOG" "$OFF0" "$OFF1" "$ENC" "$IDX" >> "$POINTS"
    tail -1 "$POINTS"
    IDX=$((IDX + PWR_STEP))
  done
done

echo
echo "done — analyze: python3 tests/nitroqam_waterfall.py report $POINTS" \
     "--baseline '$BASELINE'"
python3 "$HERE/nitroqam_waterfall.py" report "$POINTS" --baseline "$BASELINE"
