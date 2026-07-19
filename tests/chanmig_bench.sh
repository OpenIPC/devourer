#!/usr/bin/env bash
# chanmig first-light + fault-injection bench (issue #278 progression 1-5).
#
# Phase 1  a handful of clean operator-approved migrations (first light):
#          measures the PeerClock residual + per-cycle outage.
# Phase 2  fault injection: deterministic message drops via DEVOURER_MIG_DROP
#          (the demo's send-seam filter) must still converge — never split.
# Phase 3  bad destination: an interferer/illegal target forces the drone to
#          roll back both endpoints to the source within a bounded time.
#
# The failure-injection MECHANISM is a one-choke-point counter filter in the
# demo (DEVOURER_MIG_DROP=type:spec), reviewable and deterministic — it leaves
# the real air path intact for everything not dropped, unlike an on-air MITM.
#
# Usage: sudo -v && tests/chanmig_bench.sh
#        PHASE=2 tests/chanmig_bench.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANMIG_OUT:-/tmp/devourer-chanmig-bench}"
VID="${VID:-0x0bda}"
GROUND_PID="${GROUND_PID:-0x8812}"
DRONE_PID="${DRONE_PID:-0xc812}"
CHAN_A="${CHAN_A:-36}"
CHAN_B="${CHAN_B:-149}"
KEY="${KEY:-c0ffeef00d}"
TX_PWR="${TX_PWR:-12}"
PHASE="${PHASE:-all}"
mkdir -p "$OUT"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$GROUND_PID" || { echo "SKIP: ground not plugged"; exit 77; }
plugged "$VID" "$DRONE_PID" || { echo "SKIP: drone not plugged"; exit 77; }

drone_pid=""
cleanup() { [ -n "$drone_pid" ] && { sudo kill -- -"$drone_pid" 2>/dev/null; wait "$drone_pid" 2>/dev/null; }; true; }
trap cleanup EXIT INT TERM

unbind() {
    local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue; [ "$p" = "${pid#0x}" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

echo "== build =="; cmake --build "$ROOT/build" -j --target chanmig >/dev/null || exit 1
unbind "$GROUND_PID"; unbind "$DRONE_PID"

# One run: drone (background) + ground driven by N alternating proposals.
# $1=tag $2=cycles $3=drop-spec (drone) $4=allowed(ground-target legality)
run_migrations() {
    local tag="$1" cycles="$2" drop="$3" allowed="${4:-$CHAN_A,$CHAN_B}"
    echo "-- $tag: $cycles migrations, drop='$drop' allowed='$allowed' --"
    sudo env DEVOURER_MIG_ROLE=drone DEVOURER_VID="$VID" DEVOURER_PID="$DRONE_PID" \
        DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
        DEVOURER_MIG_ALLOWED="$allowed" DEVOURER_MIG_SYNTH_PPS=200 \
        DEVOURER_MIG_DROP="$drop" DEVOURER_LOG_LEVEL=warn \
        setsid "$ROOT/build/chanmig" --role drone >"$OUT/drone-$tag.jsonl" 2>/dev/null &
    drone_pid=$!
    sleep 4
    # drive the ground: alternate proposals, read confirmations
    {
        cur="$CHAN_B"
        for i in $(seq 1 "$cycles"); do
            echo "propose $cur"
            sleep 2
            [ "$cur" = "$CHAN_B" ] && cur="$CHAN_A" || cur="$CHAN_B"
        done
        sleep 2
    } | sudo env DEVOURER_MIG_ROLE=ground DEVOURER_VID="$VID" DEVOURER_PID="$GROUND_PID" \
        DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
        DEVOURER_MIG_RESCUE="$CHAN_A" DEVOURER_LOG_LEVEL=warn \
        timeout $((cycles * 3 + 20)) "$ROOT/build/chanmig" --role ground \
        >"$OUT/ground-$tag.jsonl" 2>/dev/null
    cleanup; drone_pid=""
    python3 - "$OUT/ground-$tag.jsonl" "$tag" <<'PYEOF'
import json,sys
path,tag=sys.argv[1],sys.argv[2]
done={"conf":0,"rb":0}
states=[]
for l in open(path,errors="replace"):
    if '"ev":"migrate.done"' in l:
        try: c=json.loads(l).get("code"); done["conf" if c==0 else "rb"]+=1
        except: pass
print(f"  [{tag}] confirmed={done['conf']} rolled_back/held={done['rb']}")
PYEOF
}

case "$PHASE" in
  1|all) run_migrations "phase1-clean" 6 "" ;;
esac
case "$PHASE" in
  2|all)
    run_migrations "phase2-drop-commit" 4 "2:every:3"   # drop every 3rd commit
    run_migrations "phase2-drop-status" 4 "3:every:4" ;;  # drop periodic status
esac
case "$PHASE" in
  3|all)
    # bad/illegal destination: the drone's allowed list excludes CHAN_B, so a
    # proposal to it must be rejected and both ends stay on CHAN_A.
    run_migrations "phase3-illegal-dest" 3 "" "$CHAN_A" ;;
esac
echo "logs: $OUT"
