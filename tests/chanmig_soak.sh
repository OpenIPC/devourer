#!/usr/bin/env bash
# Three-adapter scout-impact soak — the #276-style acceptance A/B:
#   arm A (scout):   video TX -> primary RX parked on the video channel,
#                    while chanscout sweeps candidates on a second adapter
#   arm B (control): identical, scout absent
# Primary delivery (rx.txhit rate + seq-gap loss) must be statistically
# unchanged between arms — the scout may not cost the video link anything
# (USB/controller contention, RF self-interference).
#
# Defaults run arm A for 1 h and the control for 30 min; tests/
# chanmig_soak_analyze.py windows both and prints the verdict.
#
# Usage: sudo -v && tests/chanmig_soak.sh
#        SOAK_DUR=300 CTRL_DUR=300 tests/chanmig_soak.sh   # quick variant
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANMIG_OUT:-/tmp/devourer-chanmig-soak}"
SOAK_DUR="${SOAK_DUR:-3600}"           # arm A (scout on)
CTRL_DUR="${CTRL_DUR:-1800}"           # arm B (control)
VIDEO_CH="${VIDEO_CH:-60}"             # the "live video" channel
PLAN="${PLAN:-36,44/40:b,149}"         # scout candidates (video ch NOT in it)
TX_VID="${TX_VID:-0x0bda}"  TX_PID="${TX_PID:-0xc812}"      # RTL8812CU (J3)
RX_VID="${RX_VID:-0x0bda}"  RX_PID="${RX_PID:-0x8812}"      # RTL8812AU (J1)
SCOUT_VID="${SCOUT_VID:-0x2357}" SCOUT_PID="${SCOUT_PID:-0x012d}"  # T3U (J2)
mkdir -p "$OUT"

pids=()
cleanup() {
    local p
    for p in "${pids[@]:-}"; do
        [ -n "$p" ] && sudo kill "$p" 2>/dev/null
    done
    for p in "${pids[@]:-}"; do
        [ -n "$p" ] && wait "$p" 2>/dev/null
    done
    pids=()
    true
}
trap cleanup EXIT INT TERM

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
for role in "TX $TX_VID $TX_PID" "RX $RX_VID $RX_PID" "SCOUT $SCOUT_VID $SCOUT_PID"; do
    set -- $role
    plugged "$2" "$3" || { echo "SKIP: $1 adapter $2:$3 not plugged"; exit 77; }
done

echo "== build =="
cmake --build "$ROOT/build" -j --target chanscout txdemo rxdemo >/dev/null || exit 1

run_arm() { # $1=tag $2=scout_on $3=duration
    local tag="$1" scout_on="$2" dur="$3"
    echo "== arm $tag: ${dur}s video ch$VIDEO_CH (scout=$scout_on) =="
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$VIDEO_CH" DEVOURER_TX_RATE=MCS1 DEVOURER_TX_PWR=22 \
        DEVOURER_TX_GAP_US=2000 \
        timeout $((dur + 30)) "$ROOT/build/txdemo" >"$OUT/tx-$tag.log" 2>&1 &
    pids+=($!)
    sleep 5
    sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" \
        DEVOURER_CHANNEL="$VIDEO_CH" DEVOURER_RX_ENERGY_MS=500 \
        DEVOURER_RXQUALITY=1 DEVOURER_LINKHEALTH=1 DEVOURER_LOG_LEVEL=info \
        timeout $((dur + 15)) "$ROOT/build/rxdemo" >"$OUT/primary-$tag.jsonl" 2>"$OUT/primary-$tag.err" &
    pids+=($!)
    if [ "$scout_on" = 1 ]; then
        sudo env DEVOURER_VID="$SCOUT_VID" DEVOURER_PID="$SCOUT_PID" \
            DEVOURER_SCOUT_PLAN="$PLAN" DEVOURER_SCOUT_DWELL_MS=100 \
            DEVOURER_SCOUT_SETTLE_MS=30 DEVOURER_SCOUT_BACKUP_MS=1000 \
            DEVOURER_SCOUT_BG_MS=5000 DEVOURER_SCOUT_NOTE="soak-$tag" \
            DEVOURER_LOG_LEVEL=info \
            timeout $((dur + 10)) "$ROOT/build/chanscout" >"$OUT/scout-$tag.jsonl" 2>"$OUT/scout-$tag.err" &
        pids+=($!)
    fi
    sleep "$dur"
    cleanup
    sleep 3    # let the adapters settle between arms
}

run_arm a 1 "$SOAK_DUR"
run_arm b 0 "$CTRL_DUR"

echo "== analyze =="
python3 "$ROOT/tests/chanmig_soak_analyze.py" \
    --scout-arm "$OUT/primary-a.jsonl" --control-arm "$OUT/primary-b.jsonl" \
    --scout-log "$OUT/scout-a.jsonl"
rc=$?
echo "logs: $OUT"
exit $rc
