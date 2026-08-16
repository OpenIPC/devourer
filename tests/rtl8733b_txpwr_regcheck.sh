#!/usr/bin/env bash
# Register-level validation of the RTL8733B runtime TX-power offset
# (IRtlDevice::SetTxPowerOffsetQdb / GetTxPowerState), the on-device
# counterpart to tests/rtl8733b_txpwr_selftest.cpp's pure math and to
# tests/txpwr_offset_onair.sh's slope measurement.
#
# Separate from txpwr_offset_regcheck.sh on purpose: every cell there reads the
# representative TXAGC INDICES out of GetTxPowerState, and this family reports
# -1 for all three because it is a dBm-target model (caps.index_max == 0). Its
# power lives in the five packed TSSI target dwords at 0x3a00..0x3a10, which
# the bring-up and actuator log verbatim, so that log line is what these cells
# read. Kestrel — the other dBm-model family — is absent from that script for
# the same reason and has its own (kestrel_txpwr_sweep.sh).
#
# Cells:
#   parity   offset 0 leaves the target table byte-identical to the master
#            build's (the key no-regression invariant, and the control that
#            says the offset path costs nothing when unused). Needs a master
#            worktree build, cached at $MASTER_BUILD; skip with SKIP_PARITY=1.
#   nullctl  the MASTER build with an offset REQUESTED writes the same table —
#            the do-nothing control proving the pre-change binary has no
#            actuator, so an on-air null in this geometry is the code and not
#            the bench.
#   move     -24 qdB shifts every capped rate byte by exactly -24 (0xe8).
#   rails    -200 clamps to -128 and +200 to +127, both with their saturation
#            flag - the int8 per-rate delta field at each end. Where the chip
#            stops RESPONDING (~-96 qdB down, ~+32 up) is measured and
#            documented in docs/rtl8733b.md, not clamped here.
#   sticky   -24 qdB then a full SetMonitorChannel to another 5 GHz group
#            re-folds the offset against the new channel; a following
#            FastRetune leaves it in place (the hop rewrites the target table,
#            so a hop that forgot the offset would silently restore power).
#   confirm  GetTxPowerState reports hw_readback=1 — the offset was verified
#            against the chip, not echoed from the session shadow.
#
# Usage: sudo -v && tests/rtl8733b_txpwr_regcheck.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${RTL8733B_TXPWR_OUT:-/tmp/devourer-8733b-txpwr}"
MASTER_BUILD="${MASTER_BUILD:-/tmp/devourer-master-build}"
PID=0xf72b; VID=0x0bda; CH_A=36; CH_B=149; CH_FAST=153
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }
cleanup() { sudo -n pkill -x txpower 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM

lsusb -d "$(printf '%04x:%04x' "$VID" "$PID")" >/dev/null 2>&1 || {
    echo "SKIP: $PID@$VID not plugged"; exit 0; }

echo "== building =="
cmake --build "$ROOT/build" -j --target txpower txdemo >/dev/null || exit 1

# The five target dwords, as the bring-up / actuator log them.
targets_at_bringup() { grep -o "TSSI tracking enabled:.*rates=[0-9a-f/]*" "$1" | tail -1 | grep -o "rates=.*"; }
targets_last()       { grep -oE "TSSI (tracking enabled|offset):.*rates=[0-9a-f/]*" "$1" | tail -1 | grep -o "rates=.*"; }
state_field() { grep -F '"ev":"txpwr.state"' "$1" | sed -n "$2p" | grep -o "\"$3\":-\?[0-9]*" | cut -d: -f2; }
offset_field() { grep -F '"ev":"txpwr.offset"' "$1" | sed -n "$2p" | grep -o "\"$3\":-\?[0-9]*" | cut -d: -f2; }

run_txpower() { local out="$1"; shift
    sudo -n timeout 90 "$ROOT/build/txpower" --vid "$VID" --pid "$PID" "$@" >"$out" 2>&1 || true; }
run_txdemo() { local out="$1" bin="$2"; shift 2
    sudo -n env DEVOURER_PID="$PID" DEVOURER_VID="$VID" DEVOURER_CHANNEL="$CH_A" \
        DEVOURER_TX_RATE=MCS0 DEVOURER_TX_FRAMES=20 "$@" \
        timeout 60 "$bin" >"$out" 2>&1 || true; }

ensure_master_build() {
    [ -x "$MASTER_BUILD/txdemo" ] && return 0
    local wt="/tmp/devourer-master-worktree"
    git -C "$ROOT" worktree add --force "$wt" origin/master >/dev/null 2>&1 || return 1
    cmake -S "$wt" -B "$MASTER_BUILD" >/dev/null 2>&1 || return 1
    cmake --build "$MASTER_BUILD" -j --target txdemo >/dev/null 2>&1 || return 1
}

echo "== DUT $PID@$VID (RTL8733B) =="

# -- caps --------------------------------------------------------------------
run_txpower "$OUT/base.log" --channel "$CH_A" --offset-start 0 --offset-stop 0 --step-ms 200
caps="$(grep -F '"ev":"txpwr.caps"' "$OUT/base.log" | head -1)"
case "$caps" in
    *'"supported":1'*'"max":0'*'"step_qdb":1'*'"min_qdb":-128'*'"max_qdb":127'*)
        pass "caps: dBm model, delta-field range [-128, +127]" ;;
    "") fail "caps: no txpwr.caps event (bring-up failed? see $OUT/base.log)" ;;
    *)  fail "caps: unexpected $caps" ;;
esac
base_targets="$(targets_at_bringup "$OUT/base.log")"
echo "  bring-up targets ch$CH_A: $base_targets"

# -- parity + null control ---------------------------------------------------
if [ "${SKIP_PARITY:-0}" = "1" ]; then
    skip "parity/nullctl (SKIP_PARITY=1)"
elif ensure_master_build; then
    run_txdemo "$OUT/master-0.log" "$MASTER_BUILD/txdemo" DEVOURER_TX_PWR_OFFSET_QDB=0
    run_txdemo "$OUT/master-48.log" "$MASTER_BUILD/txdemo" DEVOURER_TX_PWR_OFFSET_QDB=-48
    run_txdemo "$OUT/new-0.log" "$ROOT/build/txdemo" DEVOURER_TX_PWR_OFFSET_QDB=0
    m0="$(targets_at_bringup "$OUT/master-0.log")"
    m48="$(targets_at_bringup "$OUT/master-48.log")"
    n0="$(targets_at_bringup "$OUT/new-0.log")"
    if [ -z "$m0" ]; then
        skip "parity (master build produced no TSSI line)"
    elif [ "$m0" = "$n0" ]; then
        pass "parity: offset 0 byte-identical to master ($n0)"
    else
        fail "parity: master=$m0 new=$n0"
    fi
    if [ "$m48" = "$m0" ]; then
        pass "nullctl: master ignores a -48 qdB request (no actuator), $m48"
    else
        fail "nullctl: master moved the targets — the pre-change baseline is wrong"
    fi
else
    skip "parity/nullctl (master build unavailable)"
fi

# -- move --------------------------------------------------------------------
run_txpower "$OUT/move.log" --channel "$CH_A" --offset-start -24 --offset-stop -24 --step-ms 200
mv_t="$(targets_last "$OUT/move.log")"
mv_applied="$(offset_field "$OUT/move.log" 1 applied)"
if [ "$mv_t" = "rates=00000000/e8e8e8e8/e8e8e8e8/e8e8e8e8/e8e8e8e8" ] && [ "$mv_applied" = "-24" ]; then
    pass "move: -24 qdB shifts every capped rate byte to 0xe8, applied=-24"
else
    fail "move: applied=$mv_applied $mv_t"
fi

# -- rails -------------------------------------------------------------------
run_txpower "$OUT/rails.log" --channel "$CH_A" --offset-start -200 --offset-stop 200 --step-qdb 400 --step-ms 200
lo_applied="$(offset_field "$OUT/rails.log" 1 applied)"; lo_sat="$(state_field "$OUT/rails.log" 2 satlo)"
hi_applied="$(offset_field "$OUT/rails.log" 2 applied)"; hi_sat="$(state_field "$OUT/rails.log" 3 sathi)"
if [ "$lo_applied" = "-128" ] && [ "$lo_sat" = "1" ] && [ "$hi_applied" = "127" ] && [ "$hi_sat" = "1" ]; then
    pass "rails: -200 -> -128 satlo=1, +200 -> +127 sathi=1"
else
    fail "rails: low(applied=$lo_applied satlo=$lo_sat) high(applied=$hi_applied sathi=$hi_sat)"
fi

# -- sticky ------------------------------------------------------------------
run_txpower "$OUT/sticky.log" --channel "$CH_A" --offset-start -24 --offset-stop -24 --step-ms 200 \
    --switch-channel "$CH_B" --retune "$CH_FAST"
sw_line="$(grep -c "TSSI tracking enabled: ch=$CH_B .*offset=-24" "$OUT/sticky.log")"
post_switch="$(state_field "$OUT/sticky.log" 3 offset_qdb)"
post_retune="$(state_field "$OUT/sticky.log" 4 offset_qdb)"
post_rb="$(state_field "$OUT/sticky.log" 4 rb)"
if [ "$sw_line" -ge 1 ] && [ "$post_switch" = "-24" ] && [ "$post_retune" = "-24" ]; then
    pass "sticky: offset survives SetMonitorChannel($CH_B) and FastRetune($CH_FAST)"
else
    fail "sticky: refold=$sw_line post_switch=$post_switch post_retune=$post_retune"
fi

# -- confirm -----------------------------------------------------------------
if [ "$post_rb" = "1" ]; then
    pass "confirm: hw_readback=1 after the hop (chip truth, not the shadow)"
else
    fail "confirm: hw_readback=$post_rb after the hop"
fi

echo
echo "== rtl8733b txpwr regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
