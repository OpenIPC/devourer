#!/usr/bin/env bash
# mcast_da_rx_check.sh — gate for devourer#474 (telemetry in addr1): does each
# receiver family's monitor RX deliver a stream frame whose DA is a
# non-broadcast GROUP address — the very bytes FrameTimingExt ships in addr1
# (07:…, src/StreamTelemetry.h) — as readily as the broadcast one? One injector (tests/mcast_da_tx.cpp) alternates the two; each witness
# family in turn runs rxdemo with DEVOURER_STREAM_OUT=1 and the body tag says
# which was which. Verdict per RX family: PASS when the group-DA count is at
# least 90% of the broadcast count (and both are non-trivial), FAIL otherwise,
# NONE when the pair decoded nothing (link problem, not an answer). Exit 77
# when no cell could be measured at all — a skipped matrix is not a pass.
#
#   sudo -v && tests/mcast_da_rx_check.sh
#   RX_CELLS="0x0bda:0xb812 0x2357:0x0120" tests/mcast_da_rx_check.sh
# TX is the 8812CU unless the RX cell IS the 8812CU, then the 8822BU injects.
# The MT7612U cell needs MT7612U_FW_DIR (decompressed mt7662 firmware).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
CH="${CH:-6}"; SECS="${SECS:-15}"
RX_CELLS="${RX_CELLS:-0x0bda:0xb812 0x0bda:0xc812 0x2357:0x0120 0x35bc:0x0101 0x0e8d:0x7612}"
OUT="${OUT:-/tmp/devourer-mcast-da}"
MT7612U_FW_DIR="${MT7612U_FW_DIR:-}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
cleanup() {
    sudo pkill -INT -x mcast_da_tx 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x mcast_da_tx 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== build =="
cmake --build build -j --target rxdemo devourer >/dev/null || exit 1
if [ ! -x build/mcast_da_tx ] || [ tests/mcast_da_tx.cpp -nt build/mcast_da_tx ]; then
    g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/mcast_da_tx.cpp \
        examples/common/env_config.cpp build/libdevourer.a \
        $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/mcast_da_tx || exit 1
fi
mkdir -p "$OUT"; rm -f "${OUT:?}"/*.log "${OUT:?}"/*.jsonl
fails=0; measured=0
for cell in $RX_CELLS; do
    rv=${cell%%:*}; rp=${cell#*:}
    tag=$(printf '%04x_%04x' "$rv" "$rp")
    if ! plugged "$rv" "$rp"; then echo "SKIP: RX $rv:$rp not plugged"; continue; fi
    tv=0x0bda; tp=0xc812
    [ "$rp" = "0xc812" ] && tp=0xb812
    plugged "$tv" "$tp" || { echo "SKIP: TX $tv:$tp not plugged for RX $cell"; continue; }
    extra=()
    if [ "$rv" = "0x0e8d" ]; then
        [ -n "$MT7612U_FW_DIR" ] || { echo "SKIP: MT7612U cell needs MT7612U_FW_DIR"; continue; }
        extra=(DEVOURER_MT7612U_FW_DIR="$MT7612U_FW_DIR")
    fi
    echo "== RX $rv:$rp  <-  TX $tv:$tp on ch$CH =="
    sudo env DEVOURER_VID="$rv" DEVOURER_PID="$rp" DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout "${extra[@]}" \
        timeout $((SECS + 25)) ./build/rxdemo >"$OUT/rx_$tag.jsonl" 2>"$OUT/rx_$tag.log" &
    sleep 8
    sudo env DEVOURER_VID="$tv" DEVOURER_PID="$tp" DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=warn \
        ./build/mcast_da_tx "$SECS" >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.log"
    sleep 2; cleanup
    measured=$((measured + 1))
    python3 - "$OUT/rx_$tag.jsonl" "$OUT/tx_$tag.jsonl" "$cell" <<'PY' || fails=$((fails + 1))
import json, sys
sys.path.insert(0, "tests")
from devourer_events import iter_events
ff = g = 0
for e in iter_events(open(sys.argv[1]), ev="rx.frame"):
    if e.get("sa") != "57427505d600": continue
    body = bytes.fromhex(e["body"])
    if body.startswith(b"DAff"): ff += 1
    elif body.startswith(b"DAgp"): g += 1
tx = list(iter_events(open(sys.argv[2]), ev="mcast.tx"))
sent = tx[-1] if tx else {}
cell = sys.argv[3]
if ff + g < 50:
    print(f"NONE  rx={cell} ff={ff} group={g} (tx sent {sent.get('sent')})"); sys.exit(1)
ratio = g / ff if ff else float("inf")
verdict = "PASS" if ff >= 25 and ratio >= 0.9 else "FAIL"
print(f"{verdict}  rx={cell} broadcast={ff} group={g} group/broadcast={ratio:.2f} "
      f"(tx ok_ff={sent.get('ok_ff')} ok_gp={sent.get('ok_gp')})")
sys.exit(0 if verdict == "PASS" else 1)
PY
done
if [ "$measured" -eq 0 ]; then echo "SKIP: no receiver cell could be measured"; exit 77; fi
echo "== done: $measured cell(s) measured, $fails non-PASS; logs in $OUT =="
exit "$fails"
