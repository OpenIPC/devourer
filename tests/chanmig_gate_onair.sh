#!/usr/bin/env bash
# chanmig autonomous-gate on-air ladder (issue #279).
#
# The full stack on three adapters + an optional B210 interferer:
#   drone  : video TX + responder                       (RTL8812CU, J3)
#   ground : primary RX + proposer + the automation gate (RTL8812AU, J1),
#            automatic mode, tailing the scout's channel.recommend feed
#   scout  : chanscout advise mode, tailing the ground's primary telemetry,
#            emitting channel.recommend to the ground's feed              (T3U, J2)
#
# Ladder rungs (RUNG env, default all):
#   shadow  : gate in automatic mode with DEVOURER_MIG_SHADOW — decides but
#             never actuates; count migrate.shadow proposals.
#   migrate : real automatic migration under a B210 interferer on the source.
#   hold    : clean spectrum — the gate must NOT propose (false-move rate).
#
# Usage: sudo -v && tests/chanmig_gate_onair.sh
#        RUNG=shadow tests/chanmig_gate_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANMIG_OUT:-/tmp/devourer-chanmig-gate}"
VID="${VID:-0x0bda}"
DRONE_PID="${DRONE_PID:-0xc812}"
GROUND_PID="${GROUND_PID:-0x8812}"
SCOUT_VID="${SCOUT_VID:-0x2357}" SCOUT_PID="${SCOUT_PID:-0x012d}"
CHAN_A="${CHAN_A:-36}"           # source (live) channel
CHAN_B="${CHAN_B:-149}"          # migration target
KEY="${KEY:-c0ffeef00d}"
TX_PWR="${TX_PWR:-12}"
DUR="${DUR:-120}"
RUNG="${RUNG:-all}"
mkdir -p "$OUT"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$DRONE_PID" || { echo "SKIP: drone not plugged"; exit 77; }
plugged "$VID" "$GROUND_PID" || { echo "SKIP: ground not plugged"; exit 77; }
plugged "$SCOUT_VID" "$SCOUT_PID" || { echo "SKIP: scout not plugged"; exit 77; }

pids=()
cleanup() {
    for p in "${pids[@]:-}"; do [ -n "$p" ] && sudo kill -- -"$p" 2>/dev/null; done
    "$ROOT/tests/sdr_interferer.py" --stop 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
unbind() { local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do p=$(cat "$d" 2>/dev/null)||continue
        [ "$p" = "${pid#0x}" ]||continue
        for i in "$(dirname "$d")":*; do [ -e "$i/driver" ]&&sudo sh -c "echo '$(basename "$i")'>'$i/driver/unbind'" 2>/dev/null||true; done; done; }

echo "== build =="; cmake --build "$ROOT/build" -j --target chanmig chanscout >/dev/null || exit 1
unbind "$DRONE_PID"; unbind "$GROUND_PID"

# One ladder rung. $1=tag $2=mode $3=shadow(0/1) $4=interferer_ch("" = none)
run_rung() {
    local tag="$1" mode="$2" shadow="$3" ich="${4:-}"
    echo "-- rung $tag: mode=$mode shadow=$shadow interferer=${ich:-none} --"
    local pf="$OUT/primary-$tag.jsonl" rf="$OUT/recommend-$tag.jsonl"
    : >"$pf"; : >"$rf"
    # drone
    sudo env DEVOURER_MIG_ROLE=drone DEVOURER_VID="$VID" DEVOURER_PID="$DRONE_PID" \
        DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
        DEVOURER_MIG_ALLOWED="$CHAN_A,$CHAN_B" DEVOURER_MIG_SYNTH_PPS=200 \
        DEVOURER_LOG_LEVEL=warn setsid "$ROOT/build/chanmig" --role drone \
        >"$OUT/drone-$tag.jsonl" 2>/dev/null & pids+=($!)
    sleep 4
    # ground: primary RX + gate, feed = scout recommends, primary telemetry -> pf
    local shenv=""; [ "$shadow" = 1 ] && shenv="DEVOURER_MIG_SHADOW=1"
    sudo env DEVOURER_MIG_ROLE=ground DEVOURER_VID="$VID" DEVOURER_PID="$GROUND_PID" \
        DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
        DEVOURER_MIG_RESCUE="$CHAN_A" DEVOURER_MIG_MODE="$mode" $shenv \
        DEVOURER_MIG_RECOMMEND_FEED="$rf" DEVOURER_RX_ENERGY_MS=500 \
        DEVOURER_RXQUALITY=1 DEVOURER_LINKHEALTH=1 DEVOURER_LOG_LEVEL=warn \
        setsid "$ROOT/build/chanmig" --role ground >"$pf" 2>/dev/null & pids+=($!)
    sleep 3
    # scout: advise mode, primary feed = ground pf, recommends -> rf
    sudo env DEVOURER_VID="$SCOUT_VID" DEVOURER_PID="$SCOUT_PID" \
        DEVOURER_SCOUT_PLAN="$CHAN_A,$CHAN_B" DEVOURER_SCOUT_ADVISE=1 \
        DEVOURER_SCOUT_ACTIVE="$CHAN_A" DEVOURER_SCOUT_PRIMARY_FEED="$pf" \
        DEVOURER_SCOUT_DWELL_MS=100 DEVOURER_LOG_LEVEL=warn \
        setsid "$ROOT/build/chanscout" >"$rf" 2>/dev/null & pids+=($!)
    # optional B210 interferer on the source to drive an impairment
    [ -n "$ich" ] && "$ROOT/tests/sdr_interferer.py" --channel "$ich" --gain 60 &
    sleep "$DUR"
    cleanup; pids=()
    python3 - "$pf" "$tag" "$shadow" <<'PYEOF'
import json,sys
pf,tag,shadow=sys.argv[1],sys.argv[2],int(sys.argv[3])
props=shadows=confirmed=rolled=holds=0
for l in open(pf,errors="replace"):
    if '"ev":"migrate.shadow"' in l: shadows+=1
    elif '"ev":"migrate.gate"' in l:
        if '"verdict":1' in l: props+=1        # Propose
    elif '"ev":"migrate.done"' in l:
        try:
            c=json.loads(l).get("code"); confirmed+=(c==0); rolled+=(c==1)
        except: pass
print(f"  [{tag}] gate_proposes={props} shadow_proposes={shadows} "
      f"confirmed={confirmed} rolled_back={rolled}")
PYEOF
}

case "$RUNG" in
  shadow|all) run_rung shadow automatic 1 "$CHAN_A" ;;
esac
case "$RUNG" in
  migrate|all) run_rung migrate automatic 0 "$CHAN_A" ;;
esac
case "$RUNG" in
  hold|all) run_rung hold automatic 0 "" ;;   # clean: must not propose
esac
echo "logs: $OUT"
