#!/usr/bin/env bash
# chanscout retune stress — >=10,000 dwell retunes in one continuous run.
#
# The scout's whole job is to retune forever; this proves the FastRetune-per-
# dwell duty cycle survives at 10^4 scale without wedging the adapter, without
# retune-latency drift (a USB/firmware degradation tell), and with the record
# stream intact end to end. Fast dwells (50 ms observe / 20 ms settle) put
# ~13 retunes/s on the adapter, so the target lands in ~13-15 minutes.
#
# Afterwards the adapter gets a doctor grade (EFUSE stability / fw boot / RX
# smoke) so a marginal-but-not-dead unit is caught here, not mid-soak.
#
# Usage: sudo -v && tests/chanscout_stress.sh
#        TARGET=2000 tests/chanscout_stress.sh     # quick variant
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANSCOUT_OUT:-/tmp/devourer-chanscout-stress}"
TARGET="${TARGET:-10000}"              # required survey.dwell count
PLAN="${PLAN:-36,44/40:b,149}"
SCOUT_VID="${SCOUT_VID:-0x2357}"
SCOUT_PID="${SCOUT_PID:-0x012d}"
SKIP_DOCTOR="${SKIP_DOCTOR:-0}"
mkdir -p "$OUT"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$SCOUT_VID" "$SCOUT_PID" || { echo "SKIP: scout $SCOUT_VID:$SCOUT_PID not plugged"; exit 77; }

echo "== build =="
cmake --build "$ROOT/build" -j --target chanscout doctor >/dev/null || exit 1

# ~13.5 dwells/s at these cadences; cap generously past the target.
CAP=$(( TARGET / 10 + 300 ))
echo "== stress: target $TARGET dwells, cap ${CAP}s =="
sudo env DEVOURER_VID="$SCOUT_VID" DEVOURER_PID="$SCOUT_PID" \
    DEVOURER_SCOUT_PLAN="$PLAN" DEVOURER_SCOUT_DWELL_MS=50 \
    DEVOURER_SCOUT_SETTLE_MS=20 DEVOURER_SCOUT_BACKUP_MS=500 \
    DEVOURER_SCOUT_BG_MS=2000 DEVOURER_SCOUT_NOTE=stress \
    DEVOURER_LOG_LEVEL=info \
    timeout "$CAP" "$ROOT/build/chanscout" >"$OUT/scout.jsonl" 2>"$OUT/scout.err"
rc=$?
[ $rc -eq 0 ] || [ $rc -eq 124 ] || { echo "FAIL: scout exit rc=$rc"; exit 1; }

echo "== analyze =="
python3 - "$OUT/scout.jsonl" "$TARGET" <<'PYEOF'
import json, sys
path, target = sys.argv[1], int(sys.argv[2])
rows, health = [], []
for line in open(path, errors="replace"):
    if '"ev":"survey.dwell"' in line:
        try: rows.append(json.loads(line))
        except json.JSONDecodeError: pass
    elif '"ev":"scout.health"' in line:
        try: health.append(json.loads(line))
        except json.JSONDecodeError: pass

fails = 0
def check(ok, msg):
    global fails
    print(("  ok  " if ok else "  FAIL") + " " + msg)
    if not ok: fails += 1

n = len(rows)
print(f"{n} dwells captured")
check(n >= target, f"dwell count >= {target}")
retune_fail = sum(1 for r in rows if r["flags"] & 0x2)
check(retune_fail == 0, f"zero retune failures ({retune_fail})")
suspect = sum(1 for r in rows if r["flags"] & 0x8)
check(suspect <= n // 1000, f"counter-suspect rate sane ({suspect})")
seqs = [r["seq"] for r in rows]
gaps = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]))
check(gaps == 0, f"record stream intact (gaps={gaps})")

# Retune-latency drift: p95 of the last 1000 vs the first 1000 — a wedging
# USB stack or degrading firmware shows here long before a hard failure.
ret = [r["retune_us"] for r in rows if not (r["flags"] & 0x20)]
if len(ret) >= 2000:
    head = sorted(ret[:1000]); tail = sorted(ret[-1000:])
    p95h, p95t = head[950], tail[950]
    p50h, p50t = head[500], tail[500]
    print(f"  retune_us p50 {p50h}->{p50t}, p95 {p95h}->{p95t}")
    check(p95t <= 3 * p95h, "no retune-latency drift (p95 tail <= 3x head)")
wedged = [h for h in health if h.get("state") == "wedged"]
check(not wedged, "never wedged")
degraded = {h.get("reason") for h in health if h.get("state") == "degraded"}
print(f"  degraded reasons seen: {sorted(degraded) if degraded else 'none'}")

rate = n / max((rows[-1]["end_ms"] - rows[0]["start_ms"]) / 1000.0, 1)
print(f"  sustained {rate:.1f} dwells/s")
print("PASS" if fails == 0 else f"{fails} FAILURES")
sys.exit(1 if fails else 0)
PYEOF
rc=$?
[ $rc -eq 0 ] || exit $rc

if [ "$SKIP_DOCTOR" != 1 ] && [ -x "$ROOT/build/doctor" ]; then
    echo "== post-stress doctor grade =="
    sudo env DEVOURER_VID="$SCOUT_VID" DEVOURER_PID="$SCOUT_PID" \
        "$ROOT/build/doctor" >"$OUT/doctor.jsonl" 2>"$OUT/doctor.err"
    drc=$?
    echo "doctor exit=$drc (0=HEALTHY)"
    [ $drc -eq 0 ] || { echo "FAIL: adapter graded unhealthy after stress"; exit 1; }
fi
echo "PASS: $TARGET-retune stress complete; logs: $OUT"
