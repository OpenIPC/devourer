#!/usr/bin/env bash
# Kestrel per-frame retry limit (#365 step 4): on-air semantics + efficacy
# check for the AX WD DATA_TXCNT_LMT field, judged by a WITNESS monitor
# counting on-air copies per frame. Kestrel has no CCX tx.report path, so the
# copy count of an UNACKABLE unicast (RA = nonexistent station) is the only
# per-frame retry ground truth. txdemo stamps a u32 payload counter at MPDU
# offset 26; the witness runs DEVOURER_RX_PCTR + DEVOURER_RX_AGG_SA=<TX SA>
# and its rx.seq ledger yields copies-per-pctr = attempts per frame:
#
#   limit N -> modal copies N+1 => the field counts RETRIES (11ac contract;
#                                  DEVOURER_TX_RETRY_LIMIT goes in verbatim)
#   limit N -> modal copies N   => the field counts ATTEMPTS (fold +1 in
#                                  RtlKestrelDevice and update its comment)
#   limit 0 -> zero witness hits while the control arm hears the TX
#                               => ATTEMPTS with 0 = do-not-send; the default
#                                  mapping MUST become +1 before this ships
#
# The witness restarts per arm, so each ledger belongs to exactly one limit.
#
#   sudo TX_PID=0xc832 bash tests/kestrel_retry_witness.sh
#   (TX_PID: the plugged Kestrel adapter — 8832BU/8832CU/8852BU/8852CU PID)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
OUT=${OUT:-/tmp/kestrel-retry}
CH=${CH:-6}
TX_PID=${TX_PID:?set TX_PID to the Kestrel adapter PID}
TX_VID=${TX_VID:-0x0bda}
WIT_PID=${WIT_PID:-0x8813}; WIT_VID=${WIT_VID:-0x0bda}
RA=${RA:-02:de:ad:be:ef:01}   # unicast, nobody home -> never ACKed
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}
LIMITS=${LIMITS:-0 2 8}
SECS=${SECS:-6}

# Kill only THIS tree's demos (anchored, ERE-escaped path) — a bare -x kill
# would take down any other session's rxdemo/txdemo on the machine.
ESC_BUILD=$(printf '%s' "$BUILD" | sed 's/[][\.*^$/]/\\&/g')
KILL(){ sudo pkill -9 -f "^$ESC_BUILD/rxdemo" 2>/dev/null; sudo pkill -9 -f "^$ESC_BUILD/txdemo" 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

for LIM in $LIMITS; do
  echo "=== limit $LIM"
  KILL; sleep 1
  sudo env DEVOURER_VID=$WIT_VID DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=$TX_SA DEVOURER_LOG_LEVEL=info \
       "$BUILD/rxdemo" >"$OUT/wit_$LIM.jsonl" 2>"$OUT/wit_$LIM.err" &
  # RX-start line differs per generation ("Listening air" J1, "entering RX
  # loop" J3, ...) — the URB-ring submit line is the generation-neutral tell.
  w=0; until grep -qE "async ring of .* URBs submitted|Listening air" "$OUT/wit_$LIM.err"; do
    sleep 1; w=$((w+1))
    [ $w -ge 25 ] && { echo "ABORT: witness never started" >&2; exit 1; }
  done; sleep 2
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$RA \
       DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
       DEVOURER_TX_GAP_US=20000 DEVOURER_TX_RETRY_LIMIT=$LIM \
       DEVOURER_LOG_LEVEL=warn \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$LIM.jsonl" 2>"$OUT/tx_$LIM.err" || true
  sleep 1
done
KILL; sleep 1

python3 - "$OUT" <<'PY'
import collections, json, os, sys
out = sys.argv[1]
import glob
for f in sorted(glob.glob(os.path.join(out, "wit_*.jsonl"))):
    lim = os.path.basename(f)[4:-6]
    per = collections.Counter()
    for line in open(f, errors="replace"):
        if not line.startswith('{"ev":"rx.seq"'):
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if r.get("crc"):
            continue          # a corrupt copy still occupied air but isn't
                              # attributable to a pctr — count clean only
        per[r["pctr"]] += 1
    if not per:
        print(f"limit {lim:>2}: ZERO witness frames — either the TX never "
              f"aired (the limit-0-as-attempts trap) or the witness is deaf; "
              f"check tx_{lim}.err / wit_{lim}.err before concluding")
        continue
    dist = collections.Counter(per.values())
    modal = dist.most_common(1)[0][0]
    print(f"limit {lim:>2}: {len(per)} frames, copies dist "
          f"{dict(sorted(dist.items()))} modal={modal}")
PY
echo "raw: $OUT"
