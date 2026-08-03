#!/usr/bin/env bash
# Per-antenna RSSI/SNR/EVM on-air validation (rx.path + adapter.rxpaths).
#
# Two plugged adapters: TX airs the canonical beacon (default 6M legacy), RX
# runs with DEVOURER_RX_ALLPATHS=1 + DEVOURER_RX_ENERGY_MS so both the
# per-frame `rx.path` event and the windowed `adapter.rxpaths` event fire.
# Passes when, on the RX side:
#   - rx.path frames carry nonzero RSSI *and* SNR on every expected chain,
#   - EVM is nonzero-negative on at least chain 0,
#   - adapter.rxpaths carries the per-chain snr_db array.
#
# Usage (root — the demos claim USB interfaces):
#   sudo tests/rxpath_perantenna_onair.sh \
#     --tx-vid 0x0bda --tx-pid 0xb812 \
#     --rx-vid 0x35bc --rx-pid 0x0101 --chains 2 [--channel 36] [--secs 12]
set -u

BUILD=${BUILD:-"$(dirname "$0")/../build"}
TX_VID=0x0bda TX_PID="" RX_VID=0x0bda RX_PID="" CHANNEL=36 SECS=12 CHAINS=2
while [ $# -gt 0 ]; do
  case "$1" in
    --tx-vid) TX_VID=$2; shift 2;;
    --tx-pid) TX_PID=$2; shift 2;;
    --rx-vid) RX_VID=$2; shift 2;;
    --rx-pid) RX_PID=$2; shift 2;;
    --channel) CHANNEL=$2; shift 2;;
    --secs) SECS=$2; shift 2;;
    --chains) CHAINS=$2; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done
[ -n "$TX_PID" ] && [ -n "$RX_PID" ] || { echo "need --tx-pid and --rx-pid" >&2; exit 2; }
[ -x "$BUILD/rxdemo" ] && [ -x "$BUILD/txdemo" ] || { echo "build demos first" >&2; exit 2; }

LOGDIR=$(mktemp -d /tmp/rxpath-onair.XXXXXX)
RXLOG="$LOGDIR/rx.jsonl"
RX_PID_N="" TX_PID_N=""
cleanup() {
  [ -n "$TX_PID_N" ] && kill "$TX_PID_N" 2>/dev/null
  [ -n "$RX_PID_N" ] && kill "$RX_PID_N" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "logs: $LOGDIR (RX ${RX_VID}:${RX_PID}, TX ${TX_VID}:${TX_PID}, ch $CHANNEL)"

DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CHANNEL \
  DEVOURER_RX_ALLPATHS=1 DEVOURER_RX_ENERGY_MS=2000 DEVOURER_RXQUALITY=1 \
  DEVOURER_LOG_LEVEL=trace \
  "$BUILD/rxdemo" >"$RXLOG" 2>"$LOGDIR/rx.err" &
RX_PID_N=$!

sleep 4  # RX bring-up
kill -0 "$RX_PID_N" 2>/dev/null || { echo "FAIL: rxdemo died during bring-up"; tail -5 "$LOGDIR/rx.err"; exit 1; }

DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CHANNEL \
  timeout "$SECS" "$BUILD/txdemo" >"$LOGDIR/tx.jsonl" 2>"$LOGDIR/tx.err" &
TX_PID_N=$!

# Fail-fast preflight: a hit must arrive within ~8 s or the pair is dead.
for _ in $(seq 1 8); do
  sleep 1
  grep -qF '"ev":"rx.path"' "$RXLOG" && break
done
grep -qF '"ev":"rx.path"' "$RXLOG" || {
  echo "FAIL: no rx.path events after 8 s (link dead or ALLPATHS not emitting)"
  tail -5 "$LOGDIR/rx.err"; exit 1; }

wait "$TX_PID_N" 2>/dev/null; TX_PID_N=""
sleep 3  # let one more adapter.rxpaths window drain
kill "$RX_PID_N" 2>/dev/null; wait "$RX_PID_N" 2>/dev/null; RX_PID_N=""

python3 - "$RXLOG" "$CHAINS" <<'EOF'
import json, sys
log, chains = sys.argv[1], int(sys.argv[2])
paths, winds = [], []
for line in open(log):
    if not line.startswith('{"ev":"'):
        continue
    try:
        e = json.loads(line)
    except json.JSONDecodeError:
        continue
    if e.get("ev") == "rx.path":
        paths.append(e)
    elif e.get("ev") == "adapter.rxpaths":
        winds.append(e)
fail = []
if not paths:
    fail.append("no rx.path events")
else:
    n = len(paths)
    for c in range(chains):
        rssi_ok = sum(1 for e in paths if e["rssi"][c] > 0)
        snr_ok = sum(1 for e in paths if e["snr"][c] != 0)
        print(f"chain {c}: rssi nonzero {rssi_ok}/{n}, snr nonzero {snr_ok}/{n}")
        if rssi_ok < n * 0.5:
            fail.append(f"chain {c} rssi mostly zero")
        if snr_ok < n * 0.5:
            fail.append(f"chain {c} snr mostly zero")
    evm0 = sum(1 for e in paths if e["evm"][0] < 0)
    print(f"chain 0: evm negative {evm0}/{n}")
    if evm0 < n * 0.5:
        fail.append("chain 0 evm not populated")
    ex = paths[len(paths)//2]
    print("sample rx.path:", json.dumps({k: ex[k] for k in ("rssi","snr","evm")}))
snr_w = [e for e in winds if "snr_db" in e]
if not snr_w:
    fail.append("no adapter.rxpaths window carried snr_db")
else:
    print("sample adapter.rxpaths:", json.dumps(snr_w[-1]))
if fail:
    print("FAIL:", "; ".join(fail)); sys.exit(1)
print("PASS")
EOF
rc=$?
exit $rc
