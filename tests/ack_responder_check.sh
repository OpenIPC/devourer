#!/usr/bin/env bash
# Hardware ACK responder closed-loop check (src/AckResponder.h), judged by the
# OTHER radio's hardware: the TX side sends unicast QoS-Data with NORMAL
# ack-policy and per-frame TX reports on (DEVOURER_TX_REPORT — the CCX retry
# counts are the ground truth), the responder side is a devourer monitor with
# DEVOURER_ACK_RESPONDER=<mac>.
#
#   responder ON  -> the TX's tx.report events show retries ~0, state=0
#   responder OFF -> retries pinned at the descriptor limit, state=1
#
# The delta is pure hardware on both ends: SIFS-timed ACKs from the responder,
# autonomous retransmission on the TX — no host in either loop.
#
#   sudo bash tests/ack_responder_check.sh
#   TX_PID=0xc812 RESP_PID=0x012d CH=6 sudo bash tests/ack_responder_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

# TX must be a Jaguar3 (TX-only sessions decode CCX reports off the coex
# drain); the responder can be any generation.
TX_PID=${TX_PID:-0xc812};  TX_VID=${TX_VID:-0x0bda}
RESP_PID=${RESP_PID:-0x012d}; RESP_VID=${RESP_VID:-0x2357}
CH=${CH:-6}; SECS=${SECS:-8}
MAC=${MAC:-02:12:34:56:78:9a}
# The TX's TA must be UNICAST — an ACK's RA is the soliciting frame's addr2,
# and the canonical TX SA (57:42:..) is a group address. Third appearance of
# the I/G footgun (docs/ap-mode.md BSSID, the responder MAC, now the TA).
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}
OUT=${OUT:-/tmp/ack_responder}

KILL(){ pkill -9 -x rxdemo 2>/dev/null; pkill -9 -x txdemo 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

run_cell() { # $1 = cell name, $2 = responder env ("" = off)
  local name="$1" resp_env="$2"
  echo "=== cell $name (responder ${resp_env:-OFF})"
  KILL; sleep 1
  sudo env DEVOURER_VID=$RESP_VID DEVOURER_PID=$RESP_PID DEVOURER_CHANNEL=$CH \
       $resp_env DEVOURER_LOG_LEVEL=info \
       "$BUILD/rxdemo" >"$OUT/resp_$name.jsonl" 2>"$OUT/resp_$name.err" &
  sleep 6 # responder bring-up
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$MAC \
       DEVOURER_TX_SA=$TX_SA \
       DEVOURER_TX_PAYLOAD_BYTES=200 DEVOURER_TX_GAP_US=5000 \
       DEVOURER_TX_REPORT=1 DEVOURER_LOG_LEVEL=warn \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" || true
  sleep 1
  KILL; sleep 1
}

run_cell on  "DEVOURER_ACK_RESPONDER=$MAC"
run_cell off ""

echo
echo "=== RESULTS (ch$CH, unicast RA=$MAC, normal ack-policy, MCS3) ==="
python3 - "$OUT" <<'PYEOF'
import collections, json, os, sys

out = sys.argv[1]
for cell in ("on", "off"):
    reports = []
    for line in open(os.path.join(out, f"tx_{cell}.jsonl"), errors="replace"):
        if not line.startswith('{"ev":"tx.report"'):
            continue
        try:
            reports.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    if not reports:
        print(f"{cell:>4}: NO tx.report events — check the TX cell logs")
        continue
    n = len(reports)
    ok = sum(1 for r in reports if r.get("ok"))
    retries = [r.get("retries", 0) for r in reports]
    dist = collections.Counter(retries)
    top = ", ".join(f"{k}x{v}" for k, v in sorted(dist.items())[:5])
    print(f"{cell:>4}: reports={n} delivered={100*ok//n}% "
          f"mean_retries={sum(retries)/n:.1f} retry_dist=[{top}]")
PYEOF
echo "raw logs: $OUT"
