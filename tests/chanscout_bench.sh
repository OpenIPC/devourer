#!/usr/bin/env bash
# Two-adapter chanscout smoke bench — the scout's first on-air contract.
#
# Roles (all distinct VID:PIDs on this rig, overridable via env):
#   TX     — canonical-SA beacon flood (txdemo) parked on ONE candidate
#   SCOUT  — chanscout sweeping the candidate plan
# The primary receiver is deliberately absent here: this bench validates the
# SCOUT side alone (coverage, dwell hygiene, attribution); the primary-impact
# question is tests/chanmig_soak.sh's A/B.
#
# Phases:
#   1. quiet     — no TX: full-plan coverage, sane observe/retune, no wedge
#   2. occupied  — TX floods candidate $BUSY_CH: the scout must (a) count the
#                  airtime under dvr_air_us (recognized own-video, canonical
#                  SA), and (b) rank $BUSY_CH's bin busier than the quiet
#                  reference bin by decoded airtime.
#
# Usage: sudo -v && tests/chanscout_bench.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANSCOUT_OUT:-/tmp/devourer-chanscout-bench}"
DUR="${DUR:-20}"                       # seconds per phase
PLAN="${PLAN:-36,44/40:b,149}"         # bins 36, 44+48, 149
BUSY_CH="${BUSY_CH:-149}"              # the candidate the TX occupies
QUIET_CH="${QUIET_CH:-36}"             # quiet reference bin
SCOUT_VID="${SCOUT_VID:-0x2357}"       # Archer T3U (RTL8822BU, Jaguar2)
SCOUT_PID="${SCOUT_PID:-0x012d}"
TX_VID="${TX_VID:-0x0bda}"             # RTL8812CU (Jaguar3)
TX_PID="${TX_PID:-0xc812}"
mkdir -p "$OUT"

tx_pid=""
cleanup() {
    [ -n "$tx_pid" ] && { sudo kill "$tx_pid" 2>/dev/null; wait "$tx_pid" 2>/dev/null; }
    true
}
trap cleanup EXIT INT TERM

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$SCOUT_VID" "$SCOUT_PID" || { echo "SKIP: scout $SCOUT_VID:$SCOUT_PID not plugged"; exit 77; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: tx $TX_VID:$TX_PID not plugged"; exit 77; }

echo "== build =="
cmake --build "$ROOT/build" -j --target chanscout txdemo >/dev/null || exit 1

scout() { # $1=phase-tag $2=duration
    sudo env DEVOURER_VID="$SCOUT_VID" DEVOURER_PID="$SCOUT_PID" \
        DEVOURER_SCOUT_PLAN="$PLAN" DEVOURER_SCOUT_DWELL_MS=100 \
        DEVOURER_SCOUT_SETTLE_MS=30 DEVOURER_SCOUT_BACKUP_MS=1000 \
        DEVOURER_SCOUT_BG_MS=3000 DEVOURER_SCOUT_NOTE="bench-$1" \
        timeout "$2" "$ROOT/build/chanscout" >"$OUT/scout-$1.jsonl" 2>"$OUT/scout-$1.err"
    local rc=$?
    # timeout(1) returns 124 for a handled SIGTERM at expiry (the normal end
    # of a fixed-window run) and 137 when it had to escalate to SIGKILL — a
    # shutdown-path hang, which is a failure here.
    [ $rc -eq 0 ] || [ $rc -eq 124 ] || { echo "FAIL: scout exit rc=$rc (phase $1)"; return 1; }
    return 0
}

echo "== phase 1: quiet sweep (${DUR}s) =="
scout quiet "$DUR" || exit 1

echo "== phase 2: occupied candidate ch$BUSY_CH (${DUR}s) =="
sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
    DEVOURER_CHANNEL="$BUSY_CH" DEVOURER_TX_RATE=6M DEVOURER_TX_PWR=24 \
    DEVOURER_TX_GAP_US=500 \
    timeout $((DUR + 15)) "$ROOT/build/txdemo" >"$OUT/tx.log" 2>&1 &
tx_pid=$!
sleep 5     # TX bring-up before the scout window opens
scout occupied "$DUR" || exit 1
cleanup; tx_pid=""

echo "== analyze =="
python3 - "$OUT" "$BUSY_CH" "$QUIET_CH" <<'PYEOF'
import json, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
out, busy_ch, quiet_ch = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

def dwells(path):
    rows = []
    for line in open(path, errors="replace"):
        if '"ev":"survey.dwell"' not in line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return rows

fails = 0
def check(ok, msg):
    global fails
    print(("  ok  " if ok else "  FAIL") + " " + msg)
    if not ok:
        fails += 1

for phase in ("quiet", "occupied"):
    rows = dwells(f"{out}/scout-{phase}.jsonl")
    print(f"[{phase}] {len(rows)} dwells")
    check(len(rows) >= 50, f"{phase}: enough dwells")
    bins = {r["chan"] for r in rows}
    check(len(bins) >= 4, f"{phase}: all 4 bins covered ({sorted(bins)})")
    seqs = [r["seq"] for r in rows]
    check(seqs == sorted(seqs) and len(set(seqs)) == len(seqs),
          f"{phase}: seq monotone, no dupes")
    gaps = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]))
    check(gaps == 0, f"{phase}: no lost records (gaps={gaps})")
    bad = [r for r in rows if r["flags"] & 0x2]   # RetuneFailed
    check(len(bad) == 0, f"{phase}: zero retune failures")
    obs = [r["observe_ms"] for r in rows if not (r["flags"] & 0x1)]
    check(obs and min(obs) >= 80, f"{phase}: observe window intact (min={min(obs) if obs else 0})")
    ret = sorted(r["retune_us"] for r in rows)
    print(f"  retune_us p50={ret[len(ret)//2]} p95={ret[int(len(ret)*0.95)]}")
    rounds = max(r["round"] for r in rows)
    check(rounds >= 2, f"{phase}: multiple complete rounds ({rounds})")

# Occupied-phase attribution: the canonical-SA flood must land in dvr_air_us
# on the busy bin, and the busy bin must out-rank the quiet reference.
occ = dwells(f"{out}/scout-occupied.jsonl")
def air(rows, ch, key):
    sel = [r for r in rows if r["chan"] == f"5:{ch}/20" and r["observe_ms"] > 0]
    if not sel:
        return 0.0
    return sum(r[key] / (r["observe_ms"] * 1000.0) for r in sel) / len(sel)
busy_dvr = air(occ, busy_ch, "dvr_air_us")
busy_oth = air(occ, busy_ch, "oth_air_us")
quiet_dvr = air(occ, quiet_ch, "dvr_air_us")
print(f"[attribution] busy ch{busy_ch}: dvr_air={busy_dvr:.3f} oth_air={busy_oth:.3f}; "
      f"quiet ch{quiet_ch}: dvr_air={quiet_dvr:.3f}")
check(busy_dvr > 0.02, "flooded candidate shows recognized devourer airtime")
check(busy_dvr > 10 * max(quiet_dvr, 1e-6), "busy bin out-ranks quiet bin")
check(busy_dvr > busy_oth, "flood attributed to devourer, not foreign")

health = [json.loads(l) for l in open(f"{out}/scout-occupied.jsonl", errors="replace")
          if '"ev":"scout.health"' in l]
wedged = [h for h in health if h.get("state") == "wedged"]
check(len(wedged) == 0, "no wedged health state")

print("PASS" if fails == 0 else f"{fails} FAILURES")
sys.exit(1 if fails else 0)
PYEOF
rc=$?
echo "logs: $OUT"
exit $rc
