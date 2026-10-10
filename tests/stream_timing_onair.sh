#!/usr/bin/env bash
# stream_timing_onair.sh — on-air validation of the per-frame stream timing
# telemetry (src/StreamTelemetry.h): streamtx stamps addr3 + airs the
# DEVOURER_STREAM_TIMING marker; a witness rxdemo fits the transmitter's clock
# from the egress-stamped frames and emits lat_us / c2a_us per frame.
#
# Phases (each a fresh TX run against one long-lived witness):
#   floor   REPS identical runs: the run-to-run sd of median lat_us / c2a_us —
#           measured FIRST, so later differences are judged against it.
#   delay   the producer sleeps DELAY_MS between CAPTURE_TS and every DELAY_Nth
#           record: those frames' capture->send must stand DELAY_MS above the
#           rest, and be ~1/N of the stamped frames (the field tracks the
#           producer, not the pipe).
#   hop     slot-hopping TX (DEVOURER_HOP_CHANNELS): the marker and the clock
#           fit must survive retunes — lat_us still present after the first
#           hops, tq_max in the markers shows the retune cost.
#   corrupt witness with DEVOURER_RX_KEEP_CORRUPTED=1: corrupted frames are
#           decoded for what they are and never feed the clock fit.
#   svctx   the SVC demo as the transmitter (synthetic NALs, per-layer rates):
#           the same addr3 field and marker from a second TX demo.
#   duplex  the duplex demo as the transmitter (TX+RX on one chip): same again,
#           with the capture stamp through its control escape.
#
# PACE_US is the producer's record period (default 2000 = 500 fps). A part that
# cannot sustain that (the 8812EU on 5 GHz stalls 20 ms at a time) fills the
# stdin pipe, and capture->send pins at its 655 ms clip — a truthful reading of
# the backlog, not a steady state: run such a part with PACE_US=15000.
#
#   sudo -v && tests/stream_timing_onair.sh
#   TX_VID=0x2357 TX_PID=0x0120 tests/stream_timing_onair.sh   # Jaguar1: beacon fallback
#   PHASES="floor delay" REPS=5 SECS=30 tests/stream_timing_onair.sh
#   CH=36 PACE_US=15000 TX_PID=0xa81a tests/stream_timing_onair.sh    # 8812EU
# Exit: 0 all checks passed, 1 a check failed, 3 preflight failed,
#       77 an adapter is missing. Logs under $OUT.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
TX_VID="${TX_VID:-0x0bda}"; TX_PID="${TX_PID:-0xc812}"      # RTL8812CU (Jaguar3)
RX_VID="${RX_VID:-0x0bda}"; RX_PID="${RX_PID:-0xb812}"      # CF-924AC (8822BU) witness
CH="${CH:-6}"; SECS="${SECS:-20}"; REPS="${REPS:-3}"
MARKER_EVERY="${MARKER_EVERY:-50}"
PACE_US="${PACE_US:-2000}"
DELAY_N="${DELAY_N:-10}"; DELAY_MS="${DELAY_MS:-20}"
HOP_CHANNELS="${HOP_CHANNELS:-1,6,11}"; HOP_SLOT_MS="${HOP_SLOT_MS:-50}"
PHASES="${PHASES:-floor delay hop corrupt}"
OUT="${OUT:-/tmp/devourer-stream-timing}"
PY="${PY:-python3}"
UVRUN=(uv run --quiet --project tools/precoder python)

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: RX $RX_VID:$RX_PID not plugged"; exit 77; }
[ "$(id -u)" -eq 0 ] || sudo -n true 2>/dev/null || { echo "needs sudo"; exit 3; }

WIT_PID=""
cleanup() {
    sudo pkill -INT -x streamtx 2>/dev/null || true
    sudo pkill -INT -x svctx 2>/dev/null || true
    sudo pkill -INT -x duplex 2>/dev/null || true
    [ -n "$WIT_PID" ] && sudo kill -INT "$WIT_PID" 2>/dev/null || true
    sleep 1
    sudo pkill -x streamtx 2>/dev/null || true
    sudo pkill -x svctx 2>/dev/null || true
    sudo pkill -x duplex 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== build =="
cmake --build build -j --target rxdemo streamtx svctx duplex >/dev/null || exit 1
mkdir -p "$OUT"; rm -f "${OUT:?}"/*.log "${OUT:?}"/*.jsonl "${OUT:?}"/*.bin
head -c 200000 /dev/urandom >"$OUT/src.bin"

fails=0
pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; fails=$((fails + 1)); }

start_witness() { # $1 = tag, $2.. = extra env
    local tag="$1"; shift
    sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout "$@" \
        ./build/rxdemo >"$OUT/rx_$tag.jsonl" 2>"$OUT/rx_$tag.log" &
    WIT_PID=$!
    sleep 6
    if ! kill -0 "$WIT_PID" 2>/dev/null; then
        echo "PREFLIGHT FAILED: witness died ($OUT/rx_$tag.log)"; exit 3
    fi
}
stop_witness() { sudo kill -INT "$WIT_PID" 2>/dev/null || true; wait "$WIT_PID" 2>/dev/null; WIT_PID=""; }

# One TX run: $1 = tag, $2 = producer args (string), rest = extra TX env.
run_tx() {
    local tag="$1" prod="$2"; shift 2
    # The producer paces at ~2 ms per record (500 fps); the clip repeats so a
    # run of SECS seconds never runs out of records.
    "${UVRUN[@]}" tools/precoder/stream_tx.py --input "$OUT/src.bin" --repeat 2000 \
        --pace-us "$PACE_US" $prod 2>"$OUT/prod_$tag.log" |
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_TIMING="$MARKER_EVERY" DEVOURER_LOG_LEVEL=info "$@" \
        timeout "$SECS" ./build/streamtx --interval-ms 0 >"$OUT/tx_$tag.out" 2>"$OUT/tx_$tag.log"
    sudo pkill -INT -x streamtx 2>/dev/null || true
    sleep 1
}
# Slice the witness log written during the last run: byte offset bookkeeping.
mark() { stat -c %s "$OUT/rx_$1.jsonl"; }
slice() { tail -c +"$(( $2 + 1 ))" "$OUT/rx_$1.jsonl" >"$OUT/run_$3.jsonl"; }

# ---- preflight: 15 s of liveness before anything long -------------------------
echo "== preflight (TX $TX_VID:$TX_PID -> RX $RX_VID:$RX_PID, ch$CH) =="
start_witness pre
off=$(mark pre)
SECS_SAVE=$SECS; SECS=15
run_tx pre "--capture-ts"
SECS=$SECS_SAVE
slice pre "$off" pre
ntel=$(grep -c -F '"tel":1' "$OUT/run_pre.jsonl")
nmk=$(grep -c -F '"ev":"rx.timing"' "$OUT/run_pre.jsonl")
nlat=$(grep -c -F '"lat_us"' "$OUT/run_pre.jsonl")
echo "preflight: tel=$ntel markers=$nmk lat=$nlat"
if [ "$ntel" -lt 100 ] || { [ "$MARKER_EVERY" -gt 0 ] && [ "$nmk" -lt 2 ]; }; then
    stop_witness
    echo "PREFLIGHT FAILED: tel=$ntel markers=$nmk (tx log: $OUT/tx_pre.log)"; exit 3
fi
grep -E "stream timing:" "$OUT/tx_pre.log" | head -3

# ---- floor ---------------------------------------------------------------
if [[ " $PHASES " == *" floor "* ]]; then
    echo "== floor: $REPS identical ${SECS}s runs =="
    : >"$OUT/floor.tsv"
    for r in $(seq 1 "$REPS"); do
        off=$(mark pre)
        run_tx "floor$r" "--capture-ts"
        slice pre "$off" "floor$r"
        $PY tests/stream_timing_analyze.py "$OUT/run_floor$r.jsonl" >"$OUT/sum_floor$r.json"; rc=$?
        $PY - "$OUT/sum_floor$r.json" "$r" "$rc" <<'PY' >>"$OUT/floor.tsv"
import json, sys
s = json.load(open(sys.argv[1]))
print(sys.argv[2], sys.argv[3], s["tel"], s["lat_frames"], s["lat_p50_us"], s["lat_p99_us"],
      s["c2a_p50_us"], s["c2s_p50_us"], s["depth_max"], s["rx_resid_us"], sep="\t")
PY
        tail -1 "$OUT/floor.tsv"
    done
    $PY - "$OUT/floor.tsv" <<'PY'
import statistics, sys
rows = [l.split("\t") for l in open(sys.argv[1]) if l.strip()]
ok = all(r[1] == "0" for r in rows)
def col(i):
    v = [float(r[i]) for r in rows if r[i] not in ("None", "")]
    return v
for name, i in (("lat_p50_us", 4), ("c2a_p50_us", 6), ("c2s_p50_us", 7)):
    v = col(i)
    if len(v) >= 2:
        print(f"floor {name}: mean {statistics.mean(v):.0f} sd {statistics.pstdev(v):.0f} "
              f"range {min(v):.0f}..{max(v):.0f} (n={len(v)}) — effects under 2 sd are not detectable")
    elif v:
        print(f"floor {name}: {v[0]:.0f} (one run — no sd)")
sys.exit(0 if ok else 1)
PY
    if [ $? -eq 0 ]; then pass "floor: every run carried absolute latency"; else fail "floor: a run failed its checks ($OUT/sum_floor*.json)"; fi
fi

# ---- delay ---------------------------------------------------------------
if [[ " $PHASES " == *" delay "* ]]; then
    echo "== delay: ${DELAY_MS} ms producer delay on every ${DELAY_N}th record =="
    off=$(mark pre)
    run_tx delay "--capture-ts --capture-delay $DELAY_N:$DELAY_MS"
    slice pre "$off" delay
    if $PY tests/stream_timing_analyze.py "$OUT/run_delay.jsonl" --expect-delay "$DELAY_N:$DELAY_MS" >"$OUT/sum_delay.json"; then
        pass "delay: $(tr -d '\n' <"$OUT/sum_delay.json" | grep -o '"delay_step_ms": [0-9.]*\|"delay_frac": [0-9.]*' | tr '\n' ' ')"
    else
        fail "delay: $(cat "$OUT/sum_delay.json")"
    fi
fi
stop_witness

# ---- hop -----------------------------------------------------------------
if [[ " $PHASES " == *" hop "* ]]; then
    echo "== hop: slot hopping over $HOP_CHANNELS at ${HOP_SLOT_MS} ms =="
    start_witness hop DEVOURER_HOP_CHANNELS="$HOP_CHANNELS" DEVOURER_HOP_SLOT_MS="$HOP_SLOT_MS" DEVOURER_HOP_SEED=c0ffeef00dc0ffeef00dc0ffeef00d01
    off=$(mark hop)
    run_tx hop "--capture-ts" DEVOURER_HOP_CHANNELS="$HOP_CHANNELS" DEVOURER_HOP_SLOT_MS="$HOP_SLOT_MS" DEVOURER_HOP_SEED=c0ffeef00dc0ffeef00dc0ffeef00d01 DEVOURER_HOP_FAST=1
    slice hop "$off" hop
    stop_witness
    # On a part without an egress-stamped injected frame there is no clock while
    # hopping (the beacon cannot follow the hops): durations only.
    noabs=""
    grep -q -F '"presp_stamped":0' "$OUT/run_hop.jsonl" && noabs="--no-abs"
    if $PY tests/stream_timing_analyze.py "$OUT/run_hop.jsonl" $noabs --min-lat-frames 50 >"$OUT/sum_hop.json"; then
        pass "hop: $(grep -o '"lat_frames": [0-9]*\|"tq_max_us": [0-9]*\|"markers": [0-9]*' "$OUT/sum_hop.json" | tr '\n' ' ') ${noabs:+(durations only on this part)}"
    else
        fail "hop: $(cat "$OUT/sum_hop.json")"
    fi
fi

# ---- corrupt -------------------------------------------------------------
if [[ " $PHASES " == *" corrupt "* ]]; then
    echo "== corrupt: witness keeps CRC-failed frames =="
    start_witness corrupt DEVOURER_RX_KEEP_CORRUPTED=1
    off=$(mark corrupt)
    run_tx corrupt "--capture-ts"
    slice corrupt "$off" corrupt
    stop_witness
    ncrc=$(grep -F '"ev":"rx.frame"' "$OUT/run_corrupt.jsonl" | grep -c -F '"crc":1')
    if $PY tests/stream_timing_analyze.py "$OUT/run_corrupt.jsonl" >"$OUT/sum_corrupt.json"; then
        pass "corrupt: $ncrc CRC-failed frames surfaced, clock fit intact ($(grep -o '"rx_resid_us": [-0-9.e]*' "$OUT/sum_corrupt.json"))"
    else
        fail "corrupt: $(cat "$OUT/sum_corrupt.json")"
    fi
fi

# ---- svctx ---------------------------------------------------------------
if [[ " $PHASES " == *" svctx "* ]]; then
    echo "== svctx: synthetic SVC NALs at the per-layer ladder =="
    start_witness svctx
    off=$(mark svctx)
    $PY tests/gen_svc_nals.py 40 2>/dev/null |
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_TIMING="$MARKER_EVERY" DEVOURER_LOG_LEVEL=info \
        timeout "$SECS" ./build/svctx --gap-us "$PACE_US" >"$OUT/tx_svctx.out" 2>"$OUT/tx_svctx.log"
    sudo pkill -INT -x svctx 2>/dev/null || true; sleep 1
    slice svctx "$off" svctx
    stop_witness
    if $PY tests/stream_timing_analyze.py "$OUT/run_svctx.jsonl" >"$OUT/sum_svctx.json"; then
        pass "svctx: $(grep -o '"lat_frames": [0-9]*\|"lat_p50_us": [0-9]*\|"markers": [0-9]*' "$OUT/sum_svctx.json" | tr '\n' ' ')"
    else
        fail "svctx: $(cat "$OUT/sum_svctx.json")"
    fi
fi

# ---- duplex --------------------------------------------------------------
if [[ " $PHASES " == *" duplex "* ]]; then
    echo "== duplex: TX+RX on the transmitter chip, capture stamp via the control escape =="
    start_witness duplex
    off=$(mark duplex)
    # duplex brings the chip up before its TX thread exists and says so with
    # a stream.ready event; records written earlier wait in the pipe. The
    # feeder starts at once (DUPLEX_LEAD_S=0, the default) — that this is
    # safe is part of what the phase proves; a positive lead only delays it.
    DUPLEX_LEAD_S="${DUPLEX_LEAD_S:-0}"
    ( sleep "$DUPLEX_LEAD_S"; "${UVRUN[@]}" tools/precoder/stream_tx.py --input "$OUT/src.bin" --repeat 2000 \
        --pace-us "$PACE_US" --capture-ts 2>"$OUT/prod_duplex.log" ) |
    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_TIMING="$MARKER_EVERY" DEVOURER_LOG_LEVEL=info \
        timeout $((SECS + DUPLEX_LEAD_S + 10)) ./build/duplex --interval-ms 0 >"$OUT/tx_duplex.out" 2>"$OUT/tx_duplex.log"
    grep -q -F '"ev":"stream.ready"' "$OUT/tx_duplex.out" || fail "duplex: no stream.ready event"
    sudo pkill -INT -x duplex 2>/dev/null || true; sleep 1
    slice duplex "$off" duplex
    stop_witness
    if $PY tests/stream_timing_analyze.py "$OUT/run_duplex.jsonl" >"$OUT/sum_duplex.json"; then
        pass "duplex: $(grep -o '"lat_frames": [0-9]*\|"lat_p50_us": [0-9]*\|"c2a_p50_us": [0-9]*\|"markers": [0-9]*' "$OUT/sum_duplex.json" | tr '\n' ' ')"
    else
        fail "duplex: $(cat "$OUT/sum_duplex.json")"
    fi
fi

echo "== done: $fails failure(s); logs in $OUT =="
exit "$fails"
