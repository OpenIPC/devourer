#!/usr/bin/env bash
# On-air retry rate-ladder reconstruction: what rate does each RETRY of a
# frame actually air at? TX sends unACKable unicast (RA = nobody) at a fixed
# MCS with a retry limit; the witness logs every copy's hw rate via the
# rx.seq `rate` field; copies of one stamped pctr ordered by tsfl are the
# attempt sequence. Prints the per-attempt-index rate distribution — the
# fallback ladder as flown, not as documented.
#
#   sudo bash tests/retry_ladder_probe.sh                    # 8812CU TX
#   sudo TX_PID=0xb812 RATE=MCS7 LIMIT=8 bash tests/retry_ladder_probe.sh
#
# RATE_ID / DEVOURER_TX_RATEID sweeps ride the same probe: the descriptor
# RA-group governs the fw's fallback rate-space, so comparing ladders across
# rate_id values is how a modern MCS-native ladder is bench-selected.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
OUT=${OUT:-/tmp/retry-ladder}
CH=${CH:-6}
TX_PID=${TX_PID:-0xc812}; TX_VID=${TX_VID:-0x0bda}
WIT_PID=${WIT_PID:-0xb812}; WIT_VID=${WIT_VID:-0x0bda}
RA=${RA:-02:de:ad:be:ef:01}
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}
RATE=${RATE:-MCS3}
LIMIT=${LIMIT:-8}
SECS=${SECS:-8}
RATEID=${RATEID:-}          # optional DEVOURER_TX_RATEID override (RA group)
FALLBACK=${FALLBACK:-}      # off = pin retries at the descriptor rate

# Kill only THIS tree's demos (anchored, ERE-escaped path) — a bare -x kill
# would take down any other session's rxdemo/txdemo on the machine.
ESC_BUILD=$(printf '%s' "$BUILD" | sed 's/[][\.*^$/]/\\&/g')
KILL(){ sudo pkill -9 -f "^$ESC_BUILD/rxdemo" 2>/dev/null; sudo pkill -9 -f "^$ESC_BUILD/txdemo" 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

KILL; sleep 1
sudo env DEVOURER_VID=$WIT_VID DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=$TX_SA DEVOURER_LOG_LEVEL=info \
     "$BUILD/rxdemo" >"$OUT/wit.jsonl" 2>"$OUT/wit.err" &
w=0; until grep -qE "async ring of .* URBs submitted|Listening air" "$OUT/wit.err"; do
  sleep 1; w=$((w+1))
  [ $w -ge 25 ] && { echo "ABORT: witness never started" >&2; exit 1; }
done; sleep 2

sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_RATE=$RATE DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$RA \
     DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
     DEVOURER_TX_GAP_US=20000 DEVOURER_TX_RETRY_LIMIT=$LIMIT \
     ${RATEID:+DEVOURER_TX_RATEID=$RATEID} \
     ${FALLBACK:+DEVOURER_TX_RETRY_FALLBACK=$FALLBACK} \
     DEVOURER_LOG_LEVEL=info \
     timeout -s INT $SECS "$BUILD/txdemo" >"$OUT/tx.jsonl" 2>"$OUT/tx.err" || true
sleep 1; KILL

python3 - "$OUT/wit.jsonl" <<'PY'
import collections, json, sys

RATE = {0: "1M", 1: "2M", 2: "5.5M", 3: "11M", 4: "6M", 5: "9M", 6: "12M",
        7: "18M", 8: "24M", 9: "36M", 10: "48M", 11: "54M"}
for i in range(8):
    RATE[0x0c + i] = f"MCS{i}"
for i in range(8):
    RATE[0x14 + i] = f"MCS{8+i}"
for n in range(4):
    for i in range(10):
        RATE[0x2c + n * 10 + i] = f"VHT{n+1}SS_M{i}"

frames = collections.defaultdict(list)  # pctr -> [(tsfl, rate)]
for line in open(sys.argv[1], errors="replace"):
    if not line.startswith('{"ev":"rx.seq"'):
        continue
    try:
        r = json.loads(line)
    except json.JSONDecodeError:
        continue
    if r.get("crc"):
        continue
    frames[r["pctr"]].append((r["tsfl"], r.get("rate", -1)))

if not frames:
    print("NO witness frames — check wit.err / tx.err")
    sys.exit(2)

# Per-attempt-index rate distribution. tsfl is a u32 µs clock: sort within a
# frame with wrap tolerance (copies of one frame span < a second).
per_idx = collections.defaultdict(collections.Counter)
ladders = collections.Counter()
for pctr, copies in frames.items():
    base = min(t for t, _ in copies)
    copies.sort(key=lambda c: (c[0] - base) & 0xffffffff)
    names = [RATE.get(rt, f"idx{rt}") for _, rt in copies]
    for i, nm in enumerate(names):
        per_idx[i][nm] += 1
    ladders[" > ".join(names)] += 1

print(f"{len(frames)} frames observed; per-attempt rate distribution:")
for i in sorted(per_idx):
    top = ", ".join(f"{nm} x{n}" for nm, n in per_idx[i].most_common(4))
    print(f"  attempt {i}: {top}")
print("\ntop ladders as flown:")
for lad, n in ladders.most_common(6):
    print(f"  {n:>5} x  {lad}")
PY
echo "raw: $OUT"
