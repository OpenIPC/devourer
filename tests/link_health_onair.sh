#!/usr/bin/env bash
# Hardware validation for the link-health classifier (<devourer-linkhealth>).
# The classifier's core job is the near-field story from the brainstorm: tell
# "strong but self-jamming, BACK OFF power" apart from a clean link. That is
# the A/B this validates on-air, at a fixed short bench distance:
#
#   SATURATED  8812AU TX at FULL power (index 63): rssi_max pegged at the PWDB
#              ceiling + EVM degraded (~-42) — the near-field overload / self-jam.
#   HEALTHY    same geometry, TX backed off to the EVM knee (~22): same strong
#              RSSI, clean EVM (~-51).
#
# INTERFERENCE and WEAK are real classifier outputs but can't be cleanly
# reproduced at bench range — a strong direct path inches away dominates any
# injected noise (the classifier then correctly reports SATURATED). Those
# verdicts are validated synthetically in tests/link_health_selftest.cpp with
# cases calibrated from the AWGN sweep (tests/j3_dig_penalty_sweep.sh). A third
# regime here injects B210 AWGN as an informational check only (no pass/fail).
#
# Ground = a second devourer part running rxdemo with
# DEVOURER_RX_ENERGY_MS + DEVOURER_LINKHEALTH.
#
# Usage: sudo -v && tests/link_health_onair.sh [tx_pid] [ground_pid]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${LINKHEALTH_OUT:-/tmp/devourer-linkhealth}"
CH="${CH:-36}"
TX_PID="${1:-0x8812}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${2:-0xc812}" GROUND_VID="${GROUND_VID:-0x0bda}"
PY="$ROOT/tests/.venv/bin/python"; { [ -x "$PY" ] && "$PY" -c 'import uhd' 2>/dev/null; } || PY=python3
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x txdemo 2>/dev/null || true
    pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -f sdr_interferer 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
plugged "$TX_PID" "$TX_VID" || { echo "SKIP: TX $TX_PID not plugged"; exit 0; }
plugged "$GROUND_PID" "$GROUND_VID" || { echo "SKIP: ground $GROUND_PID not plugged"; exit 0; }

echo "== building =="
cmake --build "$ROOT/build" -j --target rxdemo txdemo >/dev/null || exit 1

# Run a ground-RX window against a beacon at TX index $1 (+ optional interferer
# gain in $2); echo the MODAL <devourer-linkhealth> verdict over the window.
run_regime() { # $1=tx_idx $2=jam_gain("" = none) $3=tag
    local idx="$1" jam="$2" tag="$3" jampid=""
    if [ -n "$jam" ]; then
        sudo -n "$PY" "$ROOT/tests/sdr_interferer.py" --channel "$CH" \
            --tx-gain "$jam" --rate 20e6 --mode noise --secs 30 \
            >"$OUT/$tag-noise.log" 2>&1 &
        jampid=$!
        sleep 5
    fi
    sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_RX_ENERGY_MS=500 DEVOURER_LINKHEALTH=1 \
        timeout 22 "$ROOT/build/rxdemo" >"$OUT/$tag.log" 2>&1 &
    local gj=$!
    sleep 6
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR="$idx" DEVOURER_TX_GAP_US=1500 \
        timeout 14 "$ROOT/build/txdemo" >/dev/null 2>&1 &
    local tx=$!
    wait "$tx" 2>/dev/null
    sudo -n pkill -x rxdemo 2>/dev/null; wait "$gj" 2>/dev/null
    [ -n "$jampid" ] && { kill "$jampid" 2>/dev/null; sudo -n pkill -f sdr_interferer 2>/dev/null; }
    sleep 2
    # Modal verdict across the window's linkhealth lines (skip the first, it can
    # land during TX bring-up before frames arrive).
    grep -oE "<devourer-linkhealth>verdict=[A-Z_]+" "$OUT/$tag.log" \
        | sed 's/.*verdict=//' | tail -n +2 | sort | uniq -c | sort -rn | head -1 \
        | awk '{print $2" ("$1" windows)"}'
}

echo "== regime 1: SATURATED (TX index 63, near-field, no attenuation) =="
r1="$(run_regime 63 "" sat)"
echo "  modal verdict: $r1"
case "$r1" in SATURATED*) pass "full-power near-field -> SATURATED" ;;
    *) fail "expected SATURATED, got: $r1 (see $OUT/sat.log)" ;; esac

echo "== regime 2: HEALTHY (TX index 22, backed off to the EVM knee) =="
r2="$(run_regime 22 "" healthy)"
echo "  modal verdict: $r2"
case "$r2" in HEALTHY*|MARGINAL*) pass "backed-off near-field -> $r2 (clean)" ;;
    *) fail "expected HEALTHY/MARGINAL, got: $r2 (see $OUT/healthy.log)" ;; esac

# Informational only: at bench range the strong direct path dominates injected
# noise, so this typically reads SATURATED (correct physics) rather than a clean
# INTERFERENCE — the INTERFERENCE verdict is validated synthetically in the
# selftest. Reported, never failed.
if [ "${SKIP_INTERF:-0}" != "1" ]; then
    echo "== regime 3 (informational): B210 AWGN gain 74 + low TX =="
    r3="$(run_regime 14 74 interf)"
    echo "  modal verdict: $r3 (informational — bench range can't isolate interference)"
fi

echo
echo "== link-health on-air: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
