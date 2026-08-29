#!/usr/bin/env bash
# rtl8733b_rf_characterize.sh — the reproducer for the RF-domain numbers in
# docs/rtl8733b.md "Hardware validation": per-rate radiated level (SDR,
# relative), witness EVM/RSSI per rate, and the bounded TSSI-stability soak.
#
# Witness: an RTL8812CU with DEVOURER_STREAM_OUT=1 (per-frame rx.frame
# evm/rssi). SDR level: sdr_obw.py's mean ON-block dB — relative within one
# gain+geometry only. Per-band SDR gain matters: 2.4 GHz arrives ~16 dB
# hotter at this bench's SDR and CLIPS at the 5 GHz gain (first observed as
# a compressed per-rate table); the defaults below are the validated pair.
# TSSI needs settling, so every cell floods single-rate with a settle pause
# before any reading.
#
#   sudo tests/rtl8733b_rf_characterize.sh              # both bands + soak
#   sudo SOAK_SECS=0 tests/rtl8733b_rf_characterize.sh  # skip the soak
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
TX_VID=${TX_VID:-0x0bda}; TX_PID=${TX_PID:-0xb733}
WIT_VID=${WIT_VID:-0x0bda}; WIT_PID=${WIT_PID:-0xc812}
CH_5G=${CH_5G:-149}; FREQ_5G=${FREQ_5G:-5745000000}; GAIN_5G=${GAIN_5G:-40}
CH_2G=${CH_2G:-13};  FREQ_2G=${FREQ_2G:-2472000000}; GAIN_2G=${GAIN_2G:-20}
RATES_5G=${RATES_5G:-"6M MCS0 MCS4 MCS7"}
RATES_2G=${RATES_2G:-"1M 2M 5.5M 11M 6M MCS0 MCS4 MCS7"}
SOAK_SECS=${SOAK_SECS:-180}
OUT=${OUT:-/tmp/devourer-8733b-rf/$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"
RXPID=""; TXPID=""
cleanup() { [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null; [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null; pkill -x txdemo 2>/dev/null; pkill -x rxdemo 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM
die() { echo "[8733b-rf] FATAL: $*" >&2; exit 2; }
FAILED=0

witness_up() { # ch
  env DEVOURER_VID="$WIT_VID" DEVOURER_PID="$WIT_PID" DEVOURER_CHANNEL="$1" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn \
    "$REPO/build/rxdemo" > "$OUT/wit_ch$1.jsonl" 2>"$OUT/wit_ch$1.stderr" & RXPID=$!
  sleep 8
  kill -0 "$RXPID" 2>/dev/null || die "witness died — $OUT/wit_ch$1.stderr"
}
witness_down() { kill "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; RXPID=""; }

sdr_sig() { # freq gain
  python3 "$HERE/sdr_obw.py" --freq "$1" --rate 25e6 --secs 3 --gain "$2" \
    2>/dev/null | grep sdr-obw | grep -o 'sig=[-0-9.]*dB' || echo "sig=FAIL"
}

cell() { # ch freq gain rate tag
  local ch="$1" freq="$2" gain="$3" rate="$4" tag="$5" M0 M1 L
  M0=$(stat -c%s "$OUT/wit_ch$ch.jsonl")
  env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$ch" \
    DEVOURER_TX_RATE="$rate" DEVOURER_TX_PAYLOAD_BYTES=1500 \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
    "$REPO/build/txdemo" >/dev/null 2>&1 & TXPID=$!
  sleep 6
  L=$(sdr_sig "$freq" "$gain")
  [ "$L" = "sig=FAIL" ] && FAILED=1
  sleep 1
  kill -TERM "$TXPID" 2>/dev/null; wait "$TXPID" 2>/dev/null; TXPID=""
  sleep 1
  M1=$(stat -c%s "$OUT/wit_ch$ch.jsonl")
  python3 "$HERE"/rtl8733b_rf_summarize.py window \
    "$OUT/wit_ch$ch.jsonl" "$M0" "$M1" "$tag" "$L" || FAILED=1
  sleep 1
}

echo "== per-rate level + EVM, ch$CH_5G (gain $GAIN_5G)"
witness_up "$CH_5G"
for R in $RATES_5G; do cell "$CH_5G" "$FREQ_5G" "$GAIN_5G" "$R" "ch$CH_5G $R"; done
witness_down

echo "== per-rate level + EVM, ch$CH_2G (gain $GAIN_2G)"
witness_up "$CH_2G"
for R in $RATES_2G; do cell "$CH_2G" "$FREQ_2G" "$GAIN_2G" "$R" "ch$CH_2G $R"; done
witness_down

if [ "$SOAK_SECS" -gt 0 ]; then
  echo "== TSSI stability: ${SOAK_SECS}s MCS7/20 ch$CH_5G"
  witness_up "$CH_5G"
  env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH_5G" \
    DEVOURER_TX_RATE=MCS7/20 DEVOURER_TX_PAYLOAD_BYTES=1500 \
    DEVOURER_TX_GAP_US=2000 DEVOURER_THERMAL_POLL_MS=10000 \
    DEVOURER_LOG_LEVEL=warn "$REPO/build/txdemo" \
    > "$OUT/soak_tx.jsonl" 2>&1 & TXPID=$!
  sleep 10
  E=$(sdr_sig "$FREQ_5G" "$GAIN_5G"); echo "early: $E"
  sleep "$((SOAK_SECS > 30 ? SOAK_SECS - 30 : 5))"
  L=$(sdr_sig "$FREQ_5G" "$GAIN_5G"); echo "late:  $L"
  { [ "$E" = "sig=FAIL" ] || [ "$L" = "sig=FAIL" ]; } && FAILED=1
  kill -TERM "$TXPID" 2>/dev/null; wait "$TXPID" 2>/dev/null; TXPID=""
  grep -F '"ev":"thermal"' "$OUT/soak_tx.jsonl" | tail -1
  witness_down
  python3 "$HERE"/rtl8733b_rf_summarize.py soak "$OUT/wit_ch$CH_5G.jsonl" || FAILED=1
fi

[ "$FAILED" = 0 ] || die "one or more cells failed — see $OUT"
echo "[8733b-rf] logs: $OUT"
