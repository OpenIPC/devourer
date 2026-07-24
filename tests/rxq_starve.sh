#!/usr/bin/env bash
#
# rxq_starve.sh — one RX-ring starvation experiment run for issue #330.
#
# Floods a TX adapter with counter-stamped QoS-Data frames (optionally bursty to
# emulate video keyframes) and captures the RTL8812AU RX under a chosen RX-ring
# servicing mode + injected inline-consumer cost + host CPU constraint. Emits a
# JSONL capture (rx.ring depth telemetry + rx.seq per-frame counters + tx.stats)
# for tests/rxq_analyze.py.
#
# The point the desktop rig missed before: at realistic consumer cost a fast
# xhci host has too much slack to starve. The faithful self-driven lever is to
# SHRINK THE RING (RX_URBS=1..2) so a slow inline consume collapses the armed
# depth immediately — mimicking the phone's shallow dwc3 controller — combined
# with a CPU clamp. See docs/experiments/issue-330-rx-ring-starvation.md.
#
# All config is env-overridable; run one config per invocation. Example:
#   SINK_SPIN_US=1500 RX_URBS=2 RX_MODE=async BURST_ON_MS=8 BURST_OFF_MS=40 \
#     RX_TASKSET=0 tests/rxq_starve.sh
#
set -euo pipefail

BUILD=${BUILD:-./build}
OUTDIR=${OUTDIR:-/tmp/rxq/$(printf '%(%s)T' -1)}
SUDO=${SUDO:-}                        # set to "sudo" if libusb needs privilege

# --- adapters (VID/PID pick the device; different VIDs need no topology hint) --
TX_VID=${TX_VID:-0x2357}; TX_PID=${TX_PID:-0x0120}   # 8821AU flooder (Jaguar1)
RX_VID=${RX_VID:-0x0bda}; RX_PID=${RX_PID:-0x8812}   # 8812AU DUT (the reporter chip)

# --- RF / link ---
CH=${CH:-6}
TX_RATE=${TX_RATE:-MCS4}
TX_PWR_OFFSET_QDB=${TX_PWR_OFFSET_QDB:--40}          # -10 dB: de-saturate near field
TX_PAYLOAD_BYTES=${TX_PAYLOAD_BYTES:-256}

# --- stimulus ---
# NB: gap 0 (max flood) wedges the Jaguar1 async TX path — keep a modest floor;
# a high finite rate already over-subscribes a slow RX consumer.
TX_GAP_US=${TX_GAP_US:-200}                          # ~5000 fps ceiling per sender
BURST_ON_MS=${BURST_ON_MS:-0}
BURST_OFF_MS=${BURST_OFF_MS:-0}

# --- RX ring / consumer / telemetry ---
RX_MODE=${RX_MODE:-async}
RX_URBS=${RX_URBS:-8}
RX_URB_BYTES=${RX_URB_BYTES:-16384}
SINK_SPIN_US=${SINK_SPIN_US:-0}
RING_MS=${RING_MS:-50}

# --- host constraint on the RX process ---
RX_TASKSET=${RX_TASKSET:-}            # e.g. "0" pins the pump to cpu0
RX_NICE=${RX_NICE:-}                  # e.g. "19"
RX_CHRT=${RX_CHRT:-}                  # e.g. "idle" (SCHED_IDLE) — deepest deprio
BUSY_THREADS=${BUSY_THREADS:-0}       # continuous CPU hogs (permanently-loaded SoC)
# Intermittent RT-priority preemptor (faithful VR-compositor model): hog the RX
# core for PREEMPT_ON_MS every PREEMPT_ON_MS+OFF_MS. This is the TRANSIENT
# starvation the reporter sees on keyframes — the ring recovers between hogs.
PREEMPT_ON_MS=${PREEMPT_ON_MS:-0}
PREEMPT_OFF_MS=${PREEMPT_OFF_MS:-0}
PREEMPT_CORE=${PREEMPT_CORE:-${RX_TASKSET:-0}}

DUR=${DUR:-15}

mkdir -p "$OUTDIR"
RX_JSONL="$OUTDIR/rx.jsonl"; RX_ERR="$OUTDIR/rx.err"
TX_ERR="$OUTDIR/tx.err"; META="$OUTDIR/meta.json"

TX_PIDF=""; RX_PIDF=""; PREEMPT_PIDF=""; BUSY_PIDS=()
cleanup() {
  set +e
  [ -n "$RX_PIDF" ] && kill "$RX_PIDF" 2>/dev/null
  [ -n "$TX_PIDF" ] && kill "$TX_PIDF" 2>/dev/null
  [ -n "$PREEMPT_PIDF" ] && $SUDO kill "$PREEMPT_PIDF" 2>/dev/null
  for p in "${BUSY_PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  # exact-comm backstops (never a broad pkill)
  pkill -x -f "$BUILD/txdemo" 2>/dev/null
  pkill -x rxq-busy 2>/dev/null
  $SUDO pkill -x rxq_preempt 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[rxq] out=$OUTDIR mode=$RX_MODE urbs=$RX_URBS spin=${SINK_SPIN_US}us burst=${BURST_ON_MS}/${BURST_OFF_MS}ms taskset=${RX_TASKSET:-none}" >&2

cat > "$META" <<EOF
{"ev":"rxq.meta","mode":"$RX_MODE","urbs":$RX_URBS,"urb_bytes":$RX_URB_BYTES,
 "sink_spin_us":$SINK_SPIN_US,"ring_ms":$RING_MS,"burst_on_ms":$BURST_ON_MS,
 "burst_off_ms":$BURST_OFF_MS,"tx_gap_us":$TX_GAP_US,"tx_rate":"$TX_RATE","ch":$CH,
 "taskset":"${RX_TASKSET:-}","nice":"${RX_NICE:-}","chrt":"${RX_CHRT:-}",
 "busy_threads":$BUSY_THREADS,"tx_pid":"$TX_PID","rx_pid":"$RX_PID","dur":$DUR}
EOF

# --- background CPU hogs to remove host headroom (Phase C: emulate a busy SoC).
# Pinned to the SAME core as the RX pump (RX_TASKSET) so they actually contend
# for it — the point is to starve the pump thread of CPU, not just load the box.
BUSY_PIN=""
[ -n "$RX_TASKSET" ] && BUSY_PIN="taskset -c $RX_TASKSET"
for ((i=0; i<BUSY_THREADS; i++)); do
  ( exec -a rxq-busy $BUSY_PIN bash -c 'while :; do :; done' ) &
  BUSY_PIDS+=("$!")
done

# --- intermittent RT preemptor (VR-compositor model) --------------------------
if [ "$PREEMPT_ON_MS" -gt 0 ] && [ "$PREEMPT_OFF_MS" -gt 0 ]; then
  if [ ! -x "$BUILD/rxq_preempt" ]; then
    cc -O2 -o "$BUILD/rxq_preempt" tests/rxq_preempt.c -lpthread || {
      echo "[rxq] failed to build rxq_preempt" >&2; exit 1; }
  fi
  $SUDO "$BUILD/rxq_preempt" "$PREEMPT_CORE" "$((PREEMPT_ON_MS*1000))" \
        "$((PREEMPT_OFF_MS*1000))" >/dev/null 2>&1 &
  PREEMPT_PIDF=$!
  echo "[rxq] preemptor: core $PREEMPT_CORE hog ${PREEMPT_ON_MS}ms / idle ${PREEMPT_OFF_MS}ms (FIFO)" >&2
fi

# --- TX flood: counter-stamped QoS-Data, optional keyframe bursts -------------
TXENV=( DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID"
        DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$TX_RATE"
        DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_QOS_NOACK=1
        DEVOURER_TX_PWR_OFFSET_QDB="$TX_PWR_OFFSET_QDB"
        DEVOURER_TX_PAYLOAD_BYTES="$TX_PAYLOAD_BYTES"
        DEVOURER_TX_GAP_US="$TX_GAP_US"
        DEVOURER_TX_BURST_ON_MS="$BURST_ON_MS" DEVOURER_TX_BURST_OFF_MS="$BURST_OFF_MS"
        DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout DEVOURER_EVENT_FLUSH=0 )
TX_JSONL="$OUTDIR/tx.jsonl"
$SUDO env "${TXENV[@]}" "$BUILD/txdemo" >"$TX_JSONL" 2>"$TX_ERR" &
TX_PIDF=$!
sleep 2   # let the flood come up before the RX starts counting

# --- RX capture: ring telemetry + per-frame counters + injected consumer cost -
RXENV=( DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH"
        DEVOURER_RX_MODE="$RX_MODE" DEVOURER_RX_URBS="$RX_URBS"
        DEVOURER_RX_URB_BYTES="$RX_URB_BYTES" DEVOURER_RX_RING_MS="$RING_MS"
        DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=canon
        DEVOURER_RX_SINK_SPIN_US="$SINK_SPIN_US"
        DEVOURER_RX_SINK_STALL_MS="${STALL_MS:-0}"
        DEVOURER_RX_SINK_STALL_EVERY="${STALL_EVERY:-100}"
        DEVOURER_RX_POOL_SPARE="${RX_POOL_SPARE:-0}"
        DEVOURER_RX_ENERGY_MS="${RX_ENERGY_MS:-500}" DEVOURER_LINKHEALTH=1
        DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=info )
PREFIX=()
[ -n "$RX_CHRT" ]    && PREFIX=(chrt --"$RX_CHRT" 0)
[ -n "$RX_NICE" ]    && PREFIX=(nice -n "$RX_NICE" "${PREFIX[@]}")
[ -n "$RX_TASKSET" ] && PREFIX=(taskset -c "$RX_TASKSET" "${PREFIX[@]}")

$SUDO env "${RXENV[@]}" "${PREFIX[@]}" "$BUILD/rxdemo" >"$RX_JSONL" 2>"$RX_ERR" &
RX_PIDF=$!

sleep "$DUR"
kill "$RX_PIDF" 2>/dev/null; RX_PIDF=""
kill "$TX_PIDF" 2>/dev/null; TX_PIDF=""
sleep 0.5

# --- quick summary (full analysis in tests/rxq_analyze.py) --------------------
ring=$(grep -c '"ev":"rx.ring"' "$RX_JSONL" 2>/dev/null || echo 0)
seq=$(grep -c '"ev":"rx.seq"' "$RX_JSONL" 2>/dev/null || echo 0)
hit=$(grep -c '"ev":"rx.seq"' "$RX_JSONL" 2>/dev/null || echo 0)
txsub=$(grep '"ev":"tx.stats"' "$TX_JSONL" 2>/dev/null | tail -1 | grep -oE '"submitted":[0-9]+' | cut -d: -f2 || echo "?")
lh=$(grep '"ev":"link.health"' "$RX_JSONL" 2>/dev/null | tail -1 | grep -oE '"verdict":"[A-Z_]+"' | cut -d'"' -f4 || echo "?")
echo "[rxq] captured: rx.ring=$ring rx.seq=$seq tx.submitted=$txsub link=$lh -> $RX_JSONL" >&2
grep -m1 '"ev":"rx.ring"' "$RX_JSONL" 2>/dev/null >&2 || true
grep '"ev":"link.health"' "$RX_JSONL" 2>/dev/null | tail -1 >&2 || true
echo "$OUTDIR"
