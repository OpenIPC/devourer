#!/usr/bin/env bash
# ack_txreport_matrix.sh — M0 contract 3: the unicast-ACK + TxReport capability
# matrix. For each TX generation, three generic phases against a fixed
# hardware-ACK responder (SetAckResponder on a second adapter):
#   on       — responder armed with MAC1, TX injects unicast QoS-Data to MAC1:
#              expect tx.report ok~1, retries~0 (hardware ACK closes the loop).
#   retarget — responder re-armed with a DIFFERENT unicast MAC2, TX targets
#              MAC2: proves the injected descriptor's RA and the responder MAC
#              are both arbitrary, not baked-in.
#   off      — no responder, TX to MAC1: expect ok~0, retries pinned at the
#              descriptor limit set by DEVOURER_TX_RETRY_LIMIT (this matrix
#              pins 12 so the hardware-ARQ capability stays visible) — the
#              no-ACK outcome is VISIBLE per frame.
# A responder with a measured backend-owned hook adds a fourth, same-process
# `disarmed` phase (currently RTL8733B and reference RTL8812AU USB IDs).
# Every phase also measures report_coverage (reports / frames sent) and, on
# HalMAC (J2/J3), SW_DEFINE tag-echo gaps + the firmware missed counter.
#
# TX sessions run with DEVOURER_TX_WITH_RX=thread: the CCX reports arrive on
# the C2H RX path, so a TX-only session without an RX loop never sees them on
# J1/J2 (J3 alone drains C2H off its coex runtime) — this matrix measures the
# capability with the RX loop up, the shape a scheduled MAC runs in anyway.
#
#   bash tests/ack_txreport_matrix.sh
#   CELLS="j3-8822cu:0x0bda:0xc812" SECS=10 bash tests/ack_txreport_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

# 8814AU responder. NOT the 8821AU: bench-measured, an armed 8821AU never
# closed the loop (TX retries stayed pinned) while the 8814AU ACKs ~100%.
RESP_VID=${RESP_VID:-0x0bda}; RESP_PID=${RESP_PID:-0x8813}
CH=${CH:-36}; SECS=${SECS:-8}; GAP_US=${GAP_US:-5000}
MAC1=${MAC1:-02:12:34:56:78:9a}
MAC2=${MAC2:-02:12:34:56:78:9b}
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}   # unicast TA (the ACK RA I/G footgun)
RETRY_LIMIT=${RETRY_LIMIT:-12}      # descriptor retry pin for the off phase
DISARM_MS=${DISARM_MS:-2000}        # backend-anchored post-bring-up delay
MIN_SENT=${MIN_SENT:-100}           # reject dead/too-short transmitter cells
MIN_REPORT_COVERAGE=${MIN_REPORT_COVERAGE:-0.80} # reject sparse CCX samples
MIN_RETRY_PIN_RATE=${MIN_RETRY_PIN_RATE:-0.90} # OFF must be broadly pinned
READY_TIMEOUT=${READY_TIMEOUT:-25}  # bounded responder log/liveness wait
OUT=${OUT:-/tmp/ack_txreport}
CELLS=${CELLS:-"j1-8812au:0x0bda:0x8812 j2-8812bu:0x2357:0x012d j3-8822cu:0x0bda:0xc812"}

fraction_re='^(0\.[0-9]*[1-9][0-9]*|1(\.0+)?)$'
if ! [[ "$SECS" =~ ^[1-9][0-9]*$ && "$GAP_US" =~ ^[0-9]+$ &&
        "$RETRY_LIMIT" =~ ^[0-9]+$ && "$DISARM_MS" =~ ^[0-9]+$ &&
        "$MIN_SENT" =~ ^[1-9][0-9]*$ &&
        "$READY_TIMEOUT" =~ ^[1-9][0-9]*$ &&
        "$MIN_REPORT_COVERAGE" =~ $fraction_re &&
        "$MIN_RETRY_PIN_RATE" =~ $fraction_re ]] ||
   [ "$RETRY_LIMIT" -gt 63 ]; then
  echo "ABORT: invalid numeric matrix setting (SECS/MIN_SENT/READY_TIMEOUT " \
       "must be positive integers; GAP_US/DISARM_MS non-negative integers; " \
       "RETRY_LIMIT 0..63; coverage/pin-rate fractions in (0,1])" >&2
  exit 2
fi

ACTIVE_RESP_PID=
cleanup() {
  if [ -n "$ACTIVE_RESP_PID" ]; then
    sudo kill -INT "$ACTIVE_RESP_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      if ! sudo kill -0 "$ACTIVE_RESP_PID" 2>/dev/null; then
        ACTIVE_RESP_PID=
        return 0
      fi
      sleep 1
    done
    sudo kill -KILL "$ACTIVE_RESP_PID" 2>/dev/null || true
    ACTIVE_RESP_PID=
  fi
  return 0
}
trap cleanup EXIT
mkdir -p "$OUT"
VERDICTS="$OUT/verdicts.jsonl"; : >"$VERDICTS"

wait_for_log() { # $1 pid $2 log $3 regex $4 label
  local pid="$1" log="$2" pattern="$3" label="$4" waited=0
  until grep -qE "$pattern" "$log"; do
    if ! sudo kill -0 "$pid" 2>/dev/null; then
      echo "ABORT: $label process exited" >&2
      tail -8 "$log" >&2
      exit 1
    fi
    if [ "$waited" -ge "$READY_TIMEOUT" ]; then
      echo "ABORT: timed out waiting for $label" >&2
      tail -8 "$log" >&2
      exit 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
}

stop_responder() { # $1 saved sudo/rxdemo process pid
  local pid="$1"
  sudo kill -INT "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    if ! sudo kill -0 "$pid" 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  sudo kill -KILL "$pid" 2>/dev/null || true
  return 1
}

run_phase() { # $1 cell $2 phase $3 tx vid $4 tx pid $5 RA mac $6 responder mac (""=off) $7 expect $8 disarm-after-ms
  local cell="$1" phase="$2" vid="$3" pid="$4" ra="$5" resp="$6" expect="$7"
  local disarm_ms="${8:-}" resp_pid="" tx_rc=0
  local tag="${cell}_${phase}"
  cleanup; sleep 1
  if [ -n "$resp" ]; then
    # $8 arms, then disarms mid-session from a side thread. Without it no cell
    # here can exercise a DISARM at all: every phase is a fresh process, so the
    # off phase starts from a chip that was never armed, which is a different
    # state (and on the RTL8733B an identical one — see AdapterCaps.h).
    # Redirects intentionally belong to the invoking user, not root.
    # shellcheck disable=SC2024
    sudo env DEVOURER_VID="$RESP_VID" DEVOURER_PID="$RESP_PID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_ACK_RESPONDER="$resp" DEVOURER_LOG_LEVEL=info \
        ${disarm_ms:+DEVOURER_ACK_DISARM_AFTER_MS=$disarm_ms} \
        ./build/rxdemo >"$OUT/resp_$tag.jsonl" 2>"$OUT/resp_$tag.err" &
    resp_pid=$!
    ACTIVE_RESP_PID=$resp_pid
    wait_for_log "$resp_pid" "$OUT/resp_$tag.err" \
      "hardware ACK responder armed for $resp" "$tag responder arm"
    wait_for_log "$resp_pid" "$OUT/resp_$tag.err" \
      'async ring of .* URBs submitted' "$tag responder RX readiness"
    if [ -n "$disarm_ms" ]; then
      wait_for_log "$resp_pid" "$OUT/resp_$tag.err" \
        'hardware ACK responder disarmed \(MACID(/BSSID)? back to' \
        "$tag responder disarm"
    fi
    if grep -qE \
         'ACK responder gate did not latch|net_type did not read NoLink|MACID(/BSSID)? (could not be|was not) restored|rollback was not fully verified|hardware state UNKNOWN' \
         "$OUT/resp_$tag.err"; then
      echo "ABORT: $tag responder state was not verified" >&2
      tail -8 "$OUT/resp_$tag.err" >&2
      exit 1
    fi
  fi
  # shellcheck disable=SC2024
  sudo env DEVOURER_VID="$vid" DEVOURER_PID="$pid" DEVOURER_CHANNEL="$CH" \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$ra" DEVOURER_TX_SA="$TX_SA" \
      DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PAYLOAD_BYTES=200 \
      DEVOURER_TX_GAP_US="$GAP_US" DEVOURER_TX_REPORT=1 \
      DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
      DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=warn \
      timeout -s INT "$SECS" ./build/txdemo \
      >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" || tx_rc=$?
  if [ "$tx_rc" -ne 124 ]; then
    echo "ABORT: transmitter exited early in $tag (status=$tx_rc)" >&2
    tail -8 "$OUT/tx_$tag.err" >&2
    exit 1
  fi
  if [ -n "$resp_pid" ] && ! sudo kill -0 "$resp_pid" 2>/dev/null; then
    echo "ABORT: responder exited during TX in $tag" >&2
    tail -8 "$OUT/resp_$tag.err" >&2
    exit 1
  fi
  if [ -n "$resp_pid" ] && ! stop_responder "$resp_pid"; then
    echo "ABORT: responder did not stop cleanly in $tag" >&2
    exit 1
  fi
  ACTIVE_RESP_PID=
  sleep 1
  # Frames sent = the last tx.stats 'submitted' counter (GetTxStats, emitted
  # every 500 frames) — the per-send stderr lines differ per generation.
  local sent
  sent=$(grep '"ev":"tx.stats"' "$OUT/tx_$tag.jsonl" | tail -1 |
         sed -n 's/.*"submitted":\([0-9]*\).*/\1/p')
  sent=${sent:-0}
  if [ "$sent" -lt "$MIN_SENT" ]; then
    echo "ABORT: transmitter submitted only $sent frames in $tag " \
         "(require $MIN_SENT)" >&2
    tail -8 "$OUT/tx_$tag.err" >&2
    exit 1
  fi
  echo "-- $tag: sent=$sent reports=$(grep -c '"ev":"tx.report"' "$OUT/tx_$tag.jsonl" || true)"
  if ! python3 tests/ack_txreport_analyze.py "$OUT/tx_$tag.jsonl" \
       --sent "$sent" --cell "$tag" --expect "$expect" \
       --expect-retries "$RETRY_LIMIT" \
       --min-coverage "$MIN_REPORT_COVERAGE" \
       --min-retry-pin-rate "$MIN_RETRY_PIN_RATE" | tee -a "$VERDICTS"; then
    echo "ABORT: analyzer rejected $tag" >&2
    exit 1
  fi
}

for cell in $CELLS; do
  name=${cell%%:*}; rest=${cell#*:}; vid=${rest%%:*}; pid=${rest#*:}
  echo
  echo "==== cell $name (TX $vid:$pid, ch$CH) ===="
  run_phase "$name" on       "$vid" "$pid" "$MAC1" "$MAC1" on
  run_phase "$name" retarget "$vid" "$pid" "$MAC2" "$MAC2" on
  run_phase "$name" off      "$vid" "$pid" "$MAC1" ""      off
  # The backend-scoped cell that measures a DISARM rather than a never-armed
  # chip: arm on MAC1, start the timer after verified bring-up, disarm, then
  # solicit MAC1. Each accepted backend owns that sequencing; unmeasured
  # generations are skipped rather than racing a generic timer against Init.
  case "${RESP_VID,,}:${RESP_PID,,}" in
    0x0bda:0xf72b|0x0bda:0xb733|0x0bda:0x8812)
      run_phase "$name" disarmed "$vid" "$pid" "$MAC1" "$MAC1" off "$DISARM_MS"
      ;;
    *)
      echo "-- ${name}_disarmed: SKIP (no measured backend-owned hook)"
      ;;
  esac
done

echo
echo "==== MATRIX VERDICTS ($VERDICTS) ===="
cat "$VERDICTS"
