#!/usr/bin/env bash
# la_capture_smoke.sh — per-adapter smoke test for LA-mode IQ capture
# (issue #150, src/LaCapture.h). Three cells:
#   1. manual   — immediate register trigger, no RX dependency: la.capture
#                 ok, samples>0, buffer non-constant, chip alive after
#   2. crcok    — MAC CRC-OK trigger fired by a canonical-SA TX partner
#                 (falls back to ambient traffic when no TX_PID given)
#   3. tx-conflict — capture, then TX 100 frames, then capture again:
#                 proves the TXFF window doesn't collide with live TX pages
#
#   sudo tests/la_capture_smoke.sh <DUT_PID> [CH] [TX_PID] [DUT_VID]
# e.g. 8822BU DUT + 8812AU TX partner on ch 6:
#   sudo tests/la_capture_smoke.sh 0xb82c 6 0x8812
set -uo pipefail
cd "$(dirname "$0")/.."

DUT_PID="${1:?usage: $0 <DUT_PID> [ch] [tx_pid] [dut_vid]}"
CH="${2:-6}"
TX_PID="${3:-}"
DUT_VID="${4:-0x0bda}"
OUT=/tmp/la-smoke
mkdir -p "$OUT"

cleanup() { pkill -9 -x rxdemo 2>/dev/null; pkill -9 -x txdemo 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

PASS=0; FAIL=0
verdict() { # verdict <name> <0|1>
  if [ "$2" = 1 ]; then echo "  [PASS] $1"; PASS=$((PASS+1));
  else echo "  [FAIL] $1"; FAIL=$((FAIL+1)); fi
}

# check_capture <events.jsonl> <bin> <cellname>
check_capture() {
  local ev="$1" bin="$2" cell="$3"
  python3 - "$ev" "$bin" "$cell" <<'EOF'
import json, struct, sys
ev_path, bin_path, cell = sys.argv[1], sys.argv[2], sys.argv[3]
cap = None; alive_after = False; wedged = False
t_cap = None
for line in open(ev_path, errors="replace"):
    if not line.startswith('{"ev":"'):
        continue
    try: o = json.loads(line)
    except Exception: continue
    if o["ev"] == "la.capture": cap = o
    elif o["ev"] == "la.wedged": wedged = True
    elif o["ev"] in ("rx.pkt", "rx.frame", "rx.energy") and cap is not None:
        alive_after = True   # chip still delivering after the capture
ok = bool(cap and cap.get("ok") == 1 and cap.get("samples", 0) > 0)
nonconst = False
if ok:
    try:
        with open(bin_path, "rb") as f:
            hdr = f.read(32)
            assert hdr[:4] == b"DVLA", "bad magic"
            n = struct.unpack_from("<I", hdr, 12)[0]
            data = f.read(8 * n)
            words = set(struct.unpack(f"<{n}Q", data))
            nonconst = len(words) > 1
    except Exception as e:
        print(f"  [{cell}] bin check error: {e}")
print(f"  [{cell}] capture={'ok' if ok else 'MISSING/FAIL'}"
      f" samples={cap.get('samples') if cap else 0}"
      f" wrap={cap.get('wrap') if cap else '-'}"
      f" nonconst={nonconst} alive_after={alive_after} wedged={wedged}")
sys.exit(0 if (ok and nonconst and alive_after and not wedged) else 1)
EOF
}

echo "############ 1. MANUAL TRIGGER ############"
sudo timeout 45 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info DEVOURER_RX_ENERGY_MS=1000 \
    DEVOURER_LA_CAPTURE=manual/20M/dma0/port:0x880 DEVOURER_LA_OUT="$OUT/manual.bin" \
    DEVOURER_LA_MAX=4096 \
    build/rxdemo >"$OUT/manual.jsonl" 2>"$OUT/manual.log" &
RXPID=$!
# give bring-up + settle + capture + readback time, then stop
sleep 35; sudo kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null
check_capture "$OUT/manual.jsonl" "$OUT/manual.bin" manual
verdict "manual trigger" $((! $?))
cleanup; sleep 2

echo "############ 2. CRC-OK TRIGGER ############"
if [ -n "$TX_PID" ]; then
  sudo env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
      DEVOURER_LOG_LEVEL=silent build/txdemo >/dev/null 2>&1 &
  sleep 4
fi
sudo timeout 60 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info DEVOURER_RX_ENERGY_MS=1000 \
    DEVOURER_LA_CAPTURE=crcok/20M/dma0/port:0x880/t100 DEVOURER_LA_OUT="$OUT/crcok.bin" \
    DEVOURER_LA_MAX=4096 \
    build/rxdemo >"$OUT/crcok.jsonl" 2>"$OUT/crcok.log" &
RXPID=$!
sleep 50; sudo kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null
check_capture "$OUT/crcok.jsonl" "$OUT/crcok.bin" crcok
verdict "crcok trigger" $((! $?))
cleanup; sleep 2

echo "############ 3. TX-CONFLICT (capture -> TX -> capture) ############"
# Run the DUT as TX for ~5 s (100+ frames through the low TXFF pages),
# then re-capture: the second capture must still succeed and the chip
# must still TX/RX. Uses the DUT itself for both.
sudo timeout 20 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=silent DEVOURER_TX_GAP_US=20000 \
    build/txdemo >"$OUT/tx.jsonl" 2>/dev/null &
TXW=$!
sleep 15; sudo kill -INT "$TXW" 2>/dev/null; wait "$TXW" 2>/dev/null
cleanup; sleep 2
sudo timeout 45 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info DEVOURER_RX_ENERGY_MS=1000 \
    DEVOURER_LA_CAPTURE=manual/20M/dma0/port:0x880 DEVOURER_LA_OUT="$OUT/posttx.bin" \
    DEVOURER_LA_MAX=4096 \
    build/rxdemo >"$OUT/posttx.jsonl" 2>"$OUT/posttx.log" &
RXPID=$!
sleep 35; sudo kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null
check_capture "$OUT/posttx.jsonl" "$OUT/posttx.bin" posttx
verdict "post-TX capture" $((! $?))
cleanup

echo
echo "RESULT: $PASS pass, $FAIL fail (logs: $OUT)"
[ "$FAIL" = 0 ]
