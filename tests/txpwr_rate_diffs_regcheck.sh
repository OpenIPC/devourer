#!/usr/bin/env bash
# Regression validation of the 8822E per-rate TX-power diff table
# (IRtlDevice::SetTxPowerRateDiffs / GetTxPowerState.rate_diffs_custom),
# exercised end-to-end through the txpower demo's --rate-diffs flag.
#
# Cells per plugged DUT (skip-if-unplugged, PASS/FAIL/SKIP tally like
# txpwr_offset_regcheck.sh):
#
#   baseline  no --rate-diffs given => txpwr.state reports rate_diffs=0
#             (the default phy_reg_pg per-rate walk, not the caller table).
#   apply     --rate-diffs 0,0,10,0,0,0,0,-8,-12,-16 => rate_diffs=1 and the
#             post-diff readback proves the table landed: mcs7_index ==
#             ofdm_index - 16 (ofdm reflects legacy=0, mcs7 reflects
#             mcs[7]=-16, so the delta is exactly -16 regardless of the
#             shared reference the diffs are folded against).
#   sticky    same diffs + --switch-channel to another channel in the same
#             band => post-switch state still rate_diffs=1 with the same
#             mcs7-ofdm delta (the table survives a full SetMonitorChannel
#             re-fold, not just an offset-only step).
#   override  same diffs, then --flat 40 (SetTxPowerIndexOverride forces a
#             flat index and zeroes the diffs in hardware) followed by
#             --flat -1 (clears the override) in a second invocation =>
#             final state rate_diffs=1 with the delta intact (the override
#             clear's full re-apply re-walks the caller table, per
#             RtlJaguar3Device::apply_tx_power_current's full-apply path).
#
# Only the 8822E (RTL8812EU/RTL8822EU, PID 0xa81a) implements
# SetTxPowerRateDiffs in v1 (RtlJaguar3Device::SetTxPowerRateDiffs returns
# false off that variant) — this script targets that DUT only and SKIPs
# cleanly when it isn't plugged in.
#
# Usage: sudo -v && tests/txpwr_rate_diffs_regcheck.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPWR_RATE_DIFFS_REGCHECK_OUT:-/tmp/devourer-txpwr-rate-diffs-regcheck}"
STEP_DEMO="$ROOT/build/txpower"
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x txpower 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j --target txpower >/dev/null || exit 1

# DUT: only the 8822E implements SetTxPowerRateDiffs in v1. CH_A/CH_B are
# same-band channels in different efuse channel groups (same pair the offset
# regcheck uses for its 8822E sticky cell), so the sticky check proves the
# table survives a full channel-group re-fold, not just an offset step.
PID="0xa81a"; VID="0x0bda"; CH_A="36"; CH_B="149"
DIFFS="0,0,10,0,0,0,0,-8,-12,-16"

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

# --- helpers ---------------------------------------------------------------
# Extract numeric field F from the Nth txpwr.state event line of a log.
state_field() { # $1=log $2=line-index(1-based) $3=field
    grep -F '"ev":"txpwr.state"' "$1" | sed -n "$2p" \
        | grep -o "\"$3\":-\?[0-9]*" | cut -d: -f2 | tr -d '\r'
}
# Extract numeric field F from the LAST txpwr.state event line of a log.
state_field_last() { # $1=log $2=field
    grep -F '"ev":"txpwr.state"' "$1" | tail -1 \
        | grep -o "\"$2\":-\?[0-9]*" | cut -d: -f2 | tr -d '\r'
}
caps_field() { # $1=log $2=field
    grep -F '"ev":"txpwr.caps"' "$1" | head -1 \
        | grep -o "\"$2\":-\?[0-9]*" | cut -d: -f2 | tr -d '\r'
}

run_step_demo() { # $1=outfile, rest = args
    local out="$1"; shift
    sudo -n timeout 90 "$STEP_DEMO" "$@" >"$out" 2>&1
}

if ! plugged "$PID" "$VID"; then
    skip "$PID@$VID (8822E) not plugged"
    echo
    echo "== txpwr-rate-diffs regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
    exit 0
fi

name="$PID@$VID (jaguar3/8822E)"
echo "== DUT $name =="
tag="${PID#0x}"

# -- caps sanity -------------------------------------------------------------
caps_log="$OUT/$tag-caps.log"
run_step_demo "$caps_log" --vid "$VID" --pid "$PID" --channel "$CH_A"
if [ "$(caps_field "$caps_log" supported)" != "1" ]; then
    skip "$name: TX-power API not wired for this family yet"
    echo
    echo "== txpwr-rate-diffs regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
    exit 0
fi

# -- (a) baseline: no --rate-diffs => rate_diffs=0 ---------------------------
base_rd="$(state_field "$caps_log" 1 rate_diffs)"
if [ "$base_rd" = "0" ]; then
    pass "$name baseline: rate_diffs=0 (default phy_reg_pg walk)"
else
    fail "$name baseline: rate_diffs='$base_rd' (want 0)"
fi

# -- (b) apply: --rate-diffs => rate_diffs=1, mcs7 == ofdm - 16 --------------
apply_log="$OUT/$tag-apply.log"
run_step_demo "$apply_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
    --rate-diffs "$DIFFS"
apply_rd="$(state_field_last "$apply_log" rate_diffs)"
apply_ofdm="$(state_field_last "$apply_log" ofdm)"
apply_mcs7="$(state_field_last "$apply_log" mcs7)"
if [ -z "$apply_ofdm" ] || [ "$apply_ofdm" = "-1" ]; then
    fail "$name apply: no post-diff state readback"
else
    want_mcs7=$((apply_ofdm - 16))
    if [ "$apply_rd" = "1" ] && [ "$apply_mcs7" = "$want_mcs7" ]; then
        pass "$name apply: rate_diffs=1, mcs7-ofdm=-16 (ofdm=$apply_ofdm mcs7=$apply_mcs7)"
    else
        fail "$name apply: rate_diffs=$apply_rd ofdm=$apply_ofdm mcs7=$apply_mcs7 (want rate_diffs=1, mcs7=$want_mcs7)"
    fi
fi

# -- (c) sticky: same diffs + --switch-channel => table survives the re-fold -
sticky_log="$OUT/$tag-sticky.log"
run_step_demo "$sticky_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
    --rate-diffs "$DIFFS" --switch-channel "$CH_B"
sticky_rd="$(state_field_last "$sticky_log" rate_diffs)"
sticky_ofdm="$(state_field_last "$sticky_log" ofdm)"
sticky_mcs7="$(state_field_last "$sticky_log" mcs7)"
if [ -z "$sticky_ofdm" ] || [ "$sticky_ofdm" = "-1" ]; then
    fail "$name sticky: no post-switch state readback"
else
    want_mcs7="$((sticky_ofdm - 16))"
    if [ "$sticky_rd" = "1" ] && [ "$sticky_mcs7" = "$want_mcs7" ]; then
        pass "$name sticky: rate_diffs=1 after SetMonitorChannel ch$CH_A->ch$CH_B, mcs7-ofdm=-16 (ofdm=$sticky_ofdm mcs7=$sticky_mcs7)"
    else
        fail "$name sticky: rate_diffs=$sticky_rd ofdm=$sticky_ofdm mcs7=$sticky_mcs7 (want rate_diffs=1, mcs7=$want_mcs7)"
    fi
fi

# -- (d) override set-then-clear: diffs survive the flat-override round trip -
# main.cpp's code order is always --flat's SetTxPowerIndexOverride BEFORE
# --rate-diffs' SetTxPowerRateDiffs, regardless of argv order — and there is
# no flag to re-assert --flat a second time (40, then -1) within one process,
# so the set/clear round trip needs two invocations. Both must re-issue
# --rate-diffs (a fresh device open starts with _rate_diffs unset; state
# does not persist across process exit) so each run's OWN internal sequence
# is what's under test:
#
#   run 1: --flat 40 (applied first) then --rate-diffs D (applied second)
#          SetTxPowerIndexOverride(40) flattens+zeroes the hw diffs, THEN
#          SetTxPowerRateDiffs(D) re-walks the table on top of the flat
#          override -> checks rate_diffs=1, delta intact with the override
#          still active.
#   run 2: --flat -1 (applied first) then --rate-diffs D (applied second)
#          SetTxPowerIndexOverride(-1) clears any override (a no-op on a
#          freshly-opened device that was never overridden in THIS process,
#          but exercises the same idx<0 / full-re-apply branch), THEN
#          SetTxPowerRateDiffs(D) walks the table via the override-clear's
#          full-apply path (RtlJaguar3Device::apply_tx_power_current's
#          idx<0 branch) -> checks rate_diffs=1, delta intact.
#
# Together the two runs bracket the override lever (set / clear) around a
# live diff table and confirm GetTxPowerState reports rate_diffs=1 with the
# same mcs7-ofdm delta on both sides.
for flat_val in 40 -1; do
    lbl="flat$flat_val"; lbl="${lbl/-/neg}"
    ov_log="$OUT/$tag-override-$lbl.log"
    run_step_demo "$ov_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --rate-diffs "$DIFFS" --flat "$flat_val"
    ov_rd="$(state_field_last "$ov_log" rate_diffs)"
    ov_ofdm="$(state_field_last "$ov_log" ofdm)"
    ov_mcs7="$(state_field_last "$ov_log" mcs7)"
    if [ -z "$ov_ofdm" ] || [ "$ov_ofdm" = "-1" ]; then
        fail "$name override(--flat $flat_val): no post-diff state readback"
        continue
    fi
    want_mcs7="$((ov_ofdm - 16))"
    if [ "$ov_rd" = "1" ] && [ "$ov_mcs7" = "$want_mcs7" ]; then
        pass "$name override(--flat $flat_val): rate_diffs=1, mcs7-ofdm=-16 intact (ofdm=$ov_ofdm mcs7=$ov_mcs7)"
    else
        fail "$name override(--flat $flat_val): rate_diffs=$ov_rd ofdm=$ov_ofdm mcs7=$ov_mcs7 (want rate_diffs=1, mcs7=$want_mcs7)"
    fi
done

echo
echo "== txpwr-rate-diffs regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
