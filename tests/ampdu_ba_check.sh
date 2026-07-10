#!/usr/bin/env bash
# ACKed A-MPDU / hardware BlockAck responder closed-loop check. Establishes
# that the SAME gate as the auto-ACK responder (MACID + net_type, no ADDBA
# session state) makes the MAC auto-generate an immediate SIFS BlockAck for a
# received A-MPDU — so reliable-unicast A-MPDU works end to end, both ends
# host-free.
#
# The TX sends A-MPDU with NORMAL ack-policy (SetAmpduMode no_ack=false ->
# retry limit 12, so it retries waiting for a BlockAck), per-frame TX reports
# on. The responder is a devourer monitor with DEVOURER_ACK_RESPONDER=<mac>.
#
#   responder ON  -> tx.report delivered ~100%, retries ~0 (the MAC BlockAck'd
#                    every aggregate); throughput jumps (no 12x re-air storm)
#   responder OFF -> 0% delivered, every aggregate pinned at the retry limit
#
#   sudo bash tests/ampdu_ba_check.sh
#   TX_PID=0xc812 RESP_PID=0x012d CH=6 sudo bash tests/ampdu_ba_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

# TX = a Jaguar3 (TX-only sessions decode CCX reports off the coex drain).
TX_PID=${TX_PID:-0xc812};  TX_VID=${TX_VID:-0x0bda}
RESP_PID=${RESP_PID:-0x012d}; RESP_VID=${RESP_VID:-0x2357}
CH=${CH:-6}; SECS=${SECS:-8}
MAC=${MAC:-02:12:34:56:78:9a}
# Unicast TA (the BlockAck's RA is the aggregate's addr2; the canonical TX SA
# is a group address — the I/G footgun, same as tests/ack_responder_check.sh).
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}
OUT=${OUT:-/tmp/ampdu_ba}

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
  sleep 6
  # A-MPDU, NORMAL ack-policy (no NOACK) => the aggregate solicits a BlockAck;
  # no_ack=false in the mode keeps the retry limit at 12 so a missing BA shows
  # as retry-pinned. Deep-fed so real aggregates form.
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$MAC \
       DEVOURER_TX_SA=$TX_SA DEVOURER_TX_AMPDU_MODE=0/16/7/0/20 \
       DEVOURER_TX_BATCH=3 DEVOURER_TX_USB_AGG=3 DEVOURER_TX_THREADS=2 \
       DEVOURER_TX_PAYLOAD_BYTES=200 DEVOURER_TX_GAP_US=2000 \
       DEVOURER_TX_REPORT=1 DEVOURER_LOG_LEVEL=warn \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" || true
  sleep 1
  KILL; sleep 1
}

run_cell on  "DEVOURER_ACK_RESPONDER=$MAC"
run_cell off ""

echo
echo "=== RESULTS (ch$CH, A-MPDU normal ack-policy, unicast RA=$MAC) ==="
python3 - "$OUT" "$SECS" <<'PYEOF'
import collections, json, os, sys

out, secs = sys.argv[1], float(sys.argv[2])
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
    top = ", ".join(f"{k}x{v}" for k, v in sorted(dist.items())[:6])
    print(f"{cell:>4}: reports={n} ({n/secs:.0f}/s) delivered={100*ok//n}% "
          f"mean_retries={sum(retries)/n:.1f} retry_dist=[{top}]")
PYEOF
echo "raw logs: $OUT"
