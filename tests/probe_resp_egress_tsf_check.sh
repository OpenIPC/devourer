#!/usr/bin/env bash
# probe_resp_egress_tsf_check.sh — Phase-0 gate for the stream-timing marker.
# Does the MAC stamp a HOST-INJECTED probe response (FC 0x50) / beacon (0x80)
# with its live TSF at egress, or does the frame air with the timestamp bytes
# the host wrote? One TX adapter injects both kinds with a constant timestamp
# (tests/probe_resp_egress_tx.cpp); a witness rxdemo reports rx.frame tx_tsf
# (src/RxPacket.h TxEgressTsf) + its own hardware tsfl. Verdict per kind:
#   STAMPED  — tx_tsf is live (distinct per frame, tsfl−tx_tsf constant to <1 ms)
#   CONSTANT — tx_tsf == the host's constant: the MAC does not touch it
#   NONE     — no such frames decoded (link problem, not an answer)
#
#   sudo -v && tests/probe_resp_egress_tsf_check.sh
#   TX_VID=0x2357 TX_PID=0x0120 tests/probe_resp_egress_tsf_check.sh   # 8821AU (Jaguar1)
#   HWBEACON=1 TX_VID=0x2357 TX_PID=0x0120 tests/probe_resp_egress_tsf_check.sh
#     — the fallback: air the hardware TBTT beacon (timesync master,
#       StartBeacon) instead of injecting; reported under the "beacon 0x80"
#       row (its body carries no SSID tag, so it is classed by FC alone).
#   PR0_REGS=1 ...   the injector also samples both TSF ports (0x560/0x568) and
#                    beacon control every 500 ms as pr0.regs events (test-only
#                    second RtlAdapter on the handle), to tell a stamp apart
#                    from either timer.
# Exit: 0 = verdicts produced, 3 = preflight failed, 77 = adapter missing.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
TX_VID="${TX_VID:-0x0bda}"; TX_PID="${TX_PID:-0xc812}"      # RTL8812CU (Jaguar3)
RX_VID="${RX_VID:-0x0bda}"; RX_PID="${RX_PID:-0xb812}"      # CF-924AC (8822BU) witness
CH="${CH:-6}"; SECS="${SECS:-15}"
OUT="${OUT:-/tmp/devourer-pr0}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: RX $RX_VID:$RX_PID not plugged"; exit 77; }

HWBEACON="${HWBEACON:-}"
cleanup() {
    sudo pkill -INT -x timesync 2>/dev/null || true
    sudo pkill -INT -x probe_resp_egress_tx 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x timesync 2>/dev/null || true
    sudo pkill -x probe_resp_egress_tx 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== build =="
cmake --build build -j --target rxdemo timesync devourer >/dev/null || exit 1
if [ ! -x build/probe_resp_egress_tx ] || [ tests/probe_resp_egress_tx.cpp -nt build/probe_resp_egress_tx ]; then
    g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/probe_resp_egress_tx.cpp \
        examples/common/env_config.cpp build/libdevourer.a \
        $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/probe_resp_egress_tx || exit 1
fi
mkdir -p "$OUT"; rm -f "${OUT:?}"/*.log "${OUT:?}"/*.jsonl

tag="$(printf '%04x' "$TX_PID")"
echo "== witness $RX_VID:$RX_PID on ch$CH =="
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout \
    timeout $((SECS + 20)) ./build/rxdemo >"$OUT/rx_$tag.jsonl" 2>"$OUT/rx_$tag.log" &
sleep 6
# Preflight: the witness must be alive and decoding before the TX starts.
if ! grep -q -F '"ev":"rx.frame"' "$OUT/rx_$tag.jsonl" && ! grep -q -F 'first_rx_frame' "$OUT/rx_$tag.jsonl"; then
    if ! pgrep -x rxdemo >/dev/null; then echo "PREFLIGHT FAILED: witness died ($OUT/rx_$tag.log)"; exit 3; fi
fi
echo "== TX $TX_VID:$TX_PID, ${SECS}s =="
if [ -n "$HWBEACON" ]; then
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TSYNC_ROLE=master DEVOURER_TSYNC_HWBEACON=1 DEVOURER_LOG_LEVEL=warn \
        timeout "$SECS" ./build/timesync >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.log"
    rc=$?; [ $rc -eq 124 ] && rc=0   # timeout ends the master; that is the plan
else
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
        PR0_REGS="${PR0_REGS:-}" DEVOURER_LOG_LEVEL=warn ./build/probe_resp_egress_tx "$SECS" >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.log"
    rc=$?
fi
sleep 2; cleanup
[ $rc -eq 0 ] || { echo "PREFLIGHT FAILED: TX exited $rc ($OUT/tx_$tag.log)"; exit 3; }

python3 - "$OUT/rx_$tag.jsonl" "$OUT/tx_$tag.jsonl" <<'PY'
import json, statistics, sys
sys.path.insert(0, "tests")
from devourer_events import iter_events
CONST = 0x1122334455667788
rx = list(iter_events(open(sys.argv[1]), ev="rx.frame"))
tx_tsf = [e["tsf"] for e in iter_events(open(sys.argv[2]), ev="pr0.tsf") if e.get("tsf")]
by = {"P": [], "B": []}
for e in rx:
    if e.get("sa") != "57427505d600" or "tx_tsf" not in e: continue
    body = bytes.fromhex(e["body"])
    k = "P" if b"devourer-tP" in body else "B" if b"devourer-tB" in body else None
    if k is None and e.get("fc1") is not None and len(body) >= 12 and b"devourer" not in body:
        k = "B"   # HWBEACON mode: the timesync beacon, classed by being the only other stamped frame
    if k: by[k].append((int(e["tx_tsf"]), int(e["tsfl"])))
lo, hi = (min(tx_tsf), max(tx_tsf)) if tx_tsf else (0, 0)
print(f"TX ReadTsf span: {lo}..{hi} us ({len(tx_tsf)} reads)")
for k, name in (("P", "probe response 0x50"), ("B", "beacon 0x80")):
    v = by[k]
    if not v: print(f"{name}: NONE (0 frames decoded)"); continue
    n = len(v); nconst = sum(1 for t, _ in v if t == CONST); distinct = len({t for t, _ in v})
    d = [((l - (t & 0xffffffff)) & 0xffffffff) for t, l in v if t != CONST]
    sd = statistics.pstdev(d) if len(d) > 1 else float("nan")
    inspan = sum(1 for t, _ in v if lo - 2_000_000 <= t <= hi + 2_000_000) if tx_tsf else 0
    if nconst / n > 0.9: verdict = "CONSTANT"
    elif nconst == 0 and distinct > n // 2 and sd < 1000: verdict = "STAMPED"
    else: verdict = "MIXED"
    print(f"{name}: {verdict}  n={n} const={nconst} distinct={distinct} "
          f"sd(tsfl-tx_tsf)={sd:.1f}us in_tx_span={inspan}")
PY
