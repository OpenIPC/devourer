#!/usr/bin/env bash
# beacon_update_check.sh — M0 contract 2 bench: dynamic beacon-content delivery
# (UpdateBeaconPayload) per generation. For each TX cell: start the versioned
# grant-map beacon (beacon_update_probe), watch it with a witness on a second
# adapter, swap the content N times, and run beacon_update_analyze.py — one
# bcn.verdict JSONL row per generation (skips/update, update→air latency,
# torn/regression atomicity). Results table: docs/beacon-grant.md.
#
#   bash tests/beacon_update_check.sh
#   CELLS="j2-8812bu:0x2357:0x012d" N_UPDATES=50 bash tests/beacon_update_check.sh
set -uo pipefail
cd "$(dirname "$0")/.."

WIT_VID=${WIT_VID:-0x2357}; WIT_PID=${WIT_PID:-0x0120}   # 8821AU witness
CH=${CH:-36}; N_UPDATES=${N_UPDATES:-30}; K_INTERVALS=${K_INTERVALS:-10}
OUT=${OUT:-/tmp/beacon_update}
# name:vid:pid — one TX cell per generation.
CELLS=${CELLS:-"j1-8812au:0x0bda:0x8812 j2-8812bu:0x2357:0x012d j3-8822cu:0x0bda:0xc812"}

# -f, not -x: the comm name truncates at 15 chars ("beacon_update_p"), so an
# exact-name pkill never matches. Killed TX probes leave the chip beaconing
# (no StopBeacon ran) — witness cells only, TX cells run to completion.
cleanup(){ sudo pkill -9 -f 'build/beacon_update_probe' 2>/dev/null; return 0; }
trap cleanup EXIT
mkdir -p "$OUT"

if [ ! -x build/beacon_update_probe ] || [ tests/beacon_update_probe.cpp -nt build/beacon_update_probe ]; then
  echo "== building beacon_update_probe"
  g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/beacon_update_probe.cpp \
      examples/common/env_config.cpp build/libdevourer.a \
      $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/beacon_update_probe
fi

# TX runtime: bring-up (~5s) + baseline (3s) + N*K intervals + tail (3s).
TX_SECS=$(( 11 + N_UPDATES * K_INTERVALS / 9 ))
VERDICTS="$OUT/verdicts.jsonl"; : >"$VERDICTS"

for cell in $CELLS; do
  name=${cell%%:*}; rest=${cell#*:}; vid=${rest%%:*}; pid=${rest#*:}
  echo
  echo "==== cell $name (TX $vid:$pid, ch$CH, $N_UPDATES updates x $K_INTERVALS intervals) ===="
  cleanup; sleep 1
  sudo env MODE=rx DEVOURER_VID=$WIT_VID DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
      DEVOURER_LOG_LEVEL=warn \
      ./build/beacon_update_probe $((TX_SECS + 10)) >"$OUT/wit_$name.jsonl" 2>"$OUT/wit_$name.err" &
  sleep 6   # witness bring-up
  sudo env DEVOURER_VID=$vid DEVOURER_PID=$pid DEVOURER_CHANNEL=$CH \
      DEVOURER_LOG_LEVEL=warn \
      ./build/beacon_update_probe "$N_UPDATES" "$K_INTERVALS" \
      >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err"
  sleep 2
  cleanup; sleep 1
  echo "  beacons: $(grep -c '"ev":"bcn.rx"' "$OUT/wit_$name.jsonl" || true), updates: $(grep -c '"ev":"bcn.update"' "$OUT/tx_$name.jsonl" || true)"
  python3 tests/beacon_update_analyze.py "$OUT/wit_$name.jsonl" "$OUT/tx_$name.jsonl" \
      --gen "$name" | tee -a "$VERDICTS" || true
done

echo
echo "==== VERDICTS ($VERDICTS) ===="
cat "$VERDICTS"
