#!/usr/bin/env bash
# Jaguar1 closed-loop CFO CONVERGENCE demo (#217): a real two-adapter J1 link.
# One J1 die transmits the canonical beacon; a second J1 die receives with the
# CFO loop armed. Unlike the ambient smoke, this measures the genuine inter-
# crystal offset between the pair and, if it exceeds the controller deadband,
# shows the receiver stepping its crystal cap to null it — the same convergence
# behaviour demonstrated on Jaguar2/Jaguar3, now on Jaguar1.
#
# Narrowband (default 10 MHz) is used deliberately: quartering/halving the FFT
# grid makes a given Hz offset land as a proportionally larger cfo_tail code, so
# the loop engages at offsets a 20 MHz link would sit inside the deadband for.
#
# Usage: sudo tests/jaguar1_cfo_convergence.sh [TX_PID] [RX_PID] [CH] [NB] [SEC]
set -euo pipefail

TX_PID="${1:-0x8812}"   # 8812AU transmits the beacon
RX_PID="${2:-0x8813}"   # 8814AU receives + runs the loop
CH="${3:-36}"           # 5 GHz: offset ~2.2x larger than 2.4 GHz for the same ppm
NB="${4:-10}"           # 10 MHz narrowband
DUR="${5:-25}"
BIN="$(cd "$(dirname "$0")/.." && pwd)/build"
TXLOG="$(mktemp /tmp/j1-cfo-tx.XXXXXX.log)"
RXLOG="$(mktemp /tmp/j1-cfo-rx.XXXXXX.log)"

cleanup() { pkill -9 -x txdemo 2>/dev/null || true; pkill -9 -x rxdemo 2>/dev/null || true; }
trap cleanup EXIT
cleanup; sleep 1

echo "== J1 CFO convergence: TX=$TX_PID RX=$RX_PID ch=$CH NB=${NB}MHz ${DUR}s =="

# Transmitter: canonical beacon flood, narrowband, max duty.
DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_NB_BW="$NB" \
DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=info \
  "$BIN/txdemo" >"$TXLOG" 2>&1 &
sleep 3   # let the TX bring-up + narrowband retune settle

# Receiver: same channel/BW, CFO loop armed, debug so every tick prints.
DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_NB_BW="$NB" \
DEVOURER_CFO_TRACK=1 DEVOURER_LOG_LEVEL=debug \
  timeout "${DUR}s" "$BIN/rxdemo" >"$RXLOG" 2>&1 || true

echo "--- TX airing? (beacon send count) ---"
grep -cF '"ev":"tx' "$TXLOG" 2>/dev/null || true
echo "--- RX decoded the beacon? (rx.txhit) ---"
grep -F '"ev":"rx.txhit"' "$RXLOG" | tail -1 || echo "(no txhit — pair did not decode)"
echo "--- CFO loop trajectory ---"
grep "cfo.track" "$RXLOG" || echo "(no tick lines)"
echo
echo "--- verdict ---"
if grep -q "cfo.track.*step" "$RXLOG"; then
  echo "PASS: loop ENGAGED — receiver stepped its crystal cap to reduce inter-crystal CFO"
elif grep -q "cfo.track tick" "$RXLOG"; then
  echo "HOLD: loop ran but inter-crystal offset is within deadband (well-matched pair)"
else
  echo "NO-TICK: no frames decoded to drive the loop (check TX/decode above)"
fi
echo "(logs: TX=$TXLOG RX=$RXLOG)"
