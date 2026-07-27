#!/usr/bin/env bash
# Regression validation of the per-rate TX-power diff table
# (IRtlDevice::SetTxPowerRateDiffs / GetTxPowerState.rate_diffs_custom),
# exercised end-to-end through the txpower demo's --rate-diffs flag, on every
# plugged DUT that advertises the knob (txpwr.caps rate_diffs=1 — the device's
# own advertisement, so this table needs no per-chip support list).
#
# Cells per DUT (skip-if-unplugged, PASS/FAIL/SKIP tally like
# txpwr_offset_regcheck.sh). The three index cells all reduce to the same
# primitive: mcs7_index - ofdm_index == the applied MCS7 diff, where the
# APPLIED value is the requested qdB quantized to the family step read from
# caps (1 qdB/step on Jaguar3 and Kestrel, 2 qdB/step = 0.5 dB on
# Jaguar1/Jaguar2 — so -16 qdB lands as -16 index steps on the former and -8
# on the latter).
#
#   baseline  no --rate-diffs given => txpwr.state reports rate_diffs=0
#             (the chip's calibrated per-rate shape, not the caller table).
#   apply     --rate-diffs 0,0,10,0,0,0,0,-8,-12,-16 => rate_diffs=1 and
#             mcs7_index == ofdm_index + applied(-16) (ofdm reflects legacy=0,
#             mcs7 reflects mcs[7], so the delta is exactly the applied diff
#             regardless of the anchor they fold against).
#   sticky    same diffs + --switch-channel to another channel in the same
#             band => post-switch state still rate_diffs=1 with the same delta
#             (the table survives a full SetMonitorChannel re-fold, not just
#             an offset-only step).
#   override  same diffs, then one process sequences a flat pulse via
#             --flat-pulse 40 (SetTxPowerIndexOverride(40), dump state,
#             SetTxPowerIndexOverride(-1), dump state again) => mid-pulse
#             state reports rate_diffs=1 (a table is configured) but
#             mcs7_index == ofdm_index (the honest flat truth — the override
#             flattens the chip's per-rate shape, and GetTxPowerState must not
#             paper over that), while the post-clear state reports
#             rate_diffs=1 with the delta intact (the override clear's full
#             re-apply re-walks the caller table).
#
# WHAT EACH READBACK STORY ACTUALLY PROVES — the `rb` column of txpwr.state,
# and the reason the DUT table carries a readback class:
#
#   hw      (8812AU/8821AU; both Jaguar3 dies) hw_readback=1. The reference
#           indices are read back from the TXAGC registers, so the cell proves
#           the reference landed ON THE CHIP. Note the diff half is still
#           software on Jaguar3: GetTxPowerState adds the configured diff to a
#           register-read reference, it does not read 0x3a00 back.
#   shadow  (8814AU packed port; all of Jaguar2) hw_readback=0. The block is
#           write-only, so the delta is a software-shadow tautology. These
#           cells prove STATE MANAGEMENT — the table is stored, survives a
#           channel switch, and comes back after an override clear — and NOT
#           radiated power. A PASS here is not hardware validation.
#   dbm     (Kestrel) no per-rate index exists at all: the fixed-dBm BB model
#           has no TXAGC table, and the diff folds into the target per frame
#           at send time. There is nothing to read, so that DUT runs its own
#           cell asserting the caps flavour (rate_diffs=1 with
#           rate_diffs_hw=0) and the configured/cleared transition instead.
#
# Radiated power is the on-air harness's job for every generation, this
# script's for none of them.
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

# DUT table: pid vid ch_a ch_b family readback   (same rows as
# txpwr_offset_regcheck.sh plus a readback class — see the header). CH_A/CH_B
# are same-band channels in different efuse channel groups, so the sticky cell
# proves the table survives a full channel-group re-fold, not just an offset
# step. Support is read from the device (caps rate_diffs), never assumed here.
DUTS=(
    "0x8812 0x0bda 6 11 jaguar1 hw"       # RTL8812AU
    "0x0120 0x2357 6 11 jaguar1 hw"       # Archer T2U Plus (RTL8821AU)
    "0x8813 0x0bda 6 11 jaguar1 shadow"   # RTL8814AU (write-only packed port)
    "0x012d 0x2357 6 11 jaguar2 shadow"   # Archer T3U (RTL8822BU)
    "0xc811 0x0bda 6 11 jaguar2 shadow"   # RTL8821CU
    "0xc812 0x0bda 36 149 jaguar3 hw"     # RTL8812CU/8822CU
    "0xa81a 0x0bda 36 149 jaguar3 hw"     # RTL8812EU/8822EU
    "0xb832 0x0bda 36 149 kestrel dbm"    # RTL8852BU/8832BU
    "0xc832 0x0bda 36 149 kestrel dbm"    # RTL8852CU/8832CU
    "0x0108 0x35bc 36 149 kestrel dbm"    # Archer TX20U Nano (RTL8852BU)
    "0x0101 0x35bc 36 149 kestrel dbm"    # Archer TX50UH V1 (RTL8832CU)
)
# cck, legacy, mcs0..mcs7 in qdB. legacy=0 and mcs7=-16 are what the index
# cells assert on; the rest exercise the full 10-field parse.
DIFFS="0,0,10,0,0,0,0,-8,-12,-16"
DIFF_MCS7_QDB=-16

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

# Quantize a qdB diff to this family's index steps — the same round-to-nearest,
# ties-away-from-zero rule devourer::rate_diff_steps applies, so the expected
# delta tracks the family instead of assuming the Jaguar3 1 qdB == 1 step.
applied_steps() { # $1=qdb $2=step_qdb
    local q="$1" s="$2"
    [ "$s" -eq 0 ] 2>/dev/null && { echo 0; return; }
    if [ "$q" -ge 0 ]; then echo $(( (q + s / 2) / s ))
    else echo $(( -(( -q + s / 2) / s) )); fi
}

for dut in "${DUTS[@]}"; do
    read -r PID VID CH_A CH_B FAMILY READBACK <<<"$dut"
    name="$PID@$VID ($FAMILY/$READBACK)"
    # Optional PID filter from argv.
    if [ "$#" -gt 0 ]; then
        want=0
        for p in "$@"; do [ "$p" = "$PID" ] && want=1; done
        [ "$want" = "1" ] || continue
    fi
    if ! plugged "$PID" "$VID"; then
        skip "$name not plugged"
        continue
    fi
    echo "== DUT $name =="
    tag="${PID#0x}"

    # -- caps sanity ---------------------------------------------------------
    caps_log="$OUT/$tag-caps.log"
    run_step_demo "$caps_log" --vid "$VID" --pid "$PID" --channel "$CH_A"
    if [ "$(caps_field "$caps_log" supported)" != "1" ]; then
        skip "$name: TX-power API not wired for this family"
        continue
    fi
    # The device's own advertisement gates the DUT: no per-chip support list
    # to keep in sync as generations land.
    if [ "$(caps_field "$caps_log" rate_diffs)" != "1" ]; then
        skip "$name: per-rate diff table not advertised (caps rate_diffs=0)"
        continue
    fi
    STEP_QDB="$(caps_field "$caps_log" step_qdb)"
    HW_TABLE="$(caps_field "$caps_log" rate_diffs_hw)"
    DELTA="$(applied_steps "$DIFF_MCS7_QDB" "$STEP_QDB")"

    # -- (a) baseline: no --rate-diffs => rate_diffs=0 -----------------------
    base_rd="$(state_field "$caps_log" 1 rate_diffs)"
    if [ "$base_rd" = "0" ]; then
        pass "$name baseline: rate_diffs=0 (chip's calibrated shape)"
    else
        fail "$name baseline: rate_diffs='$base_rd' (want 0)"
    fi

    # -- Kestrel (dbm): no per-rate index exists, so assert the caps flavour
    # and the configured/cleared transition instead of an index delta. The
    # per-frame fold is on-air-only truth; this cell just pins the contract.
    if [ "$READBACK" = "dbm" ]; then
        if [ "$HW_TABLE" = "0" ]; then
            pass "$name caps: rate_diffs=1 with rate_diffs_hw=0 (software send-time fold)"
        else
            fail "$name caps: rate_diffs_hw='$HW_TABLE' (want 0 — no per-rate TXAGC table on this family)"
        fi
        k_log="$OUT/$tag-apply.log"
        run_step_demo "$k_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
            --rate-diffs "$DIFFS"
        if [ "$(state_field_last "$k_log" rate_diffs)" = "1" ]; then
            pass "$name apply: rate_diffs=1 configured"
        else
            fail "$name apply: rate_diffs='$(state_field_last "$k_log" rate_diffs)' (want 1)"
        fi
        kc_log="$OUT/$tag-clear.log"
        run_step_demo "$kc_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
            --rate-diffs clear
        if [ "$(state_field_last "$kc_log" rate_diffs)" = "0" ]; then
            pass "$name clear: rate_diffs=0 restored"
        else
            fail "$name clear: rate_diffs='$(state_field_last "$kc_log" rate_diffs)' (want 0)"
        fi
        continue
    fi

    # -- (b) apply: --rate-diffs => rate_diffs=1, mcs7 == ofdm + DELTA -------
    apply_log="$OUT/$tag-apply.log"
    run_step_demo "$apply_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --rate-diffs "$DIFFS"
    apply_rd="$(state_field_last "$apply_log" rate_diffs)"
    apply_ofdm="$(state_field_last "$apply_log" ofdm)"
    apply_mcs7="$(state_field_last "$apply_log" mcs7)"
    apply_rb="$(state_field_last "$apply_log" rb)"
    want_rb=$([ "$READBACK" = "hw" ] && echo 1 || echo 0)
    if [ "$apply_rb" = "$want_rb" ]; then
        pass "$name apply: hw_readback=$apply_rb as expected for a '$READBACK' DUT"
    else
        fail "$name apply: hw_readback=$apply_rb (want $want_rb for '$READBACK')"
    fi
    if [ -z "$apply_ofdm" ] || [ "$apply_ofdm" = "-1" ]; then
        fail "$name apply: no post-diff state readback"
    else
        want_mcs7=$((apply_ofdm + DELTA))
        if [ "$apply_rd" = "1" ] && [ "$apply_mcs7" = "$want_mcs7" ]; then
            pass "$name apply: rate_diffs=1, mcs7-ofdm=$DELTA steps (${DIFF_MCS7_QDB} qdB at ${STEP_QDB} qdB/step; ofdm=$apply_ofdm mcs7=$apply_mcs7)"
        else
            fail "$name apply: rate_diffs=$apply_rd ofdm=$apply_ofdm mcs7=$apply_mcs7 (want rate_diffs=1, mcs7=$want_mcs7)"
        fi
    fi

    # -- (c) sticky: same diffs + --switch-channel => survives the re-fold ----
    sticky_log="$OUT/$tag-sticky.log"
    run_step_demo "$sticky_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --rate-diffs "$DIFFS" --switch-channel "$CH_B"
    sticky_rd="$(state_field_last "$sticky_log" rate_diffs)"
    sticky_ofdm="$(state_field_last "$sticky_log" ofdm)"
    sticky_mcs7="$(state_field_last "$sticky_log" mcs7)"
    if [ -z "$sticky_ofdm" ] || [ "$sticky_ofdm" = "-1" ]; then
        fail "$name sticky: no post-switch state readback"
    else
        want_mcs7="$((sticky_ofdm + DELTA))"
        if [ "$sticky_rd" = "1" ] && [ "$sticky_mcs7" = "$want_mcs7" ]; then
            pass "$name sticky: rate_diffs=1 after SetMonitorChannel ch$CH_A->ch$CH_B, mcs7-ofdm=$DELTA (ofdm=$sticky_ofdm mcs7=$sticky_mcs7)"
        else
            fail "$name sticky: rate_diffs=$sticky_rd ofdm=$sticky_ofdm mcs7=$sticky_mcs7 (want rate_diffs=1, mcs7=$want_mcs7)"
        fi
    fi

    # -- (d) override set-then-clear: sequenced in ONE process via --flat-pulse
    # main.cpp applies --rate-diffs' SetTxPowerRateDiffs first, then (since
    # --flat-pulse's block sits right after it, before the offset ramp) pulses
    # SetTxPowerIndexOverride(N) and prints state, then clears it back to -1
    # and prints state again. One invocation sequences set -> flat -> clear, so
    # this is a real within-process persistence proof instead of two
    # independent process re-opens (each of which would just re-apply the
    # diffs from scratch).
    #
    # The run emits 4 txpwr.state events (baseline, post-diffs, mid-pulse,
    # post-clear); we read the LAST THREE:
    #
    #   post-diffs: rate_diffs=1 with the delta — the table just landed
    #     (confirms the pulse starts clean).
    #   mid-pulse: rate_diffs=1 (a table is still CONFIGURED) BUT mcs7 == ofdm
    #     == the flat index — the honest flat truth: the override flattens the
    #     chip's per-rate shape, and GetTxPowerState must report that rather
    #     than anchor+diff during the override.
    #   post-clear: rate_diffs=1 with the delta restored — clearing the
    #     override drove the full-apply path, which re-walks the still-
    #     configured caller table. This is the persistence proof (d) is for.
    ov_log="$OUT/$tag-override-pulse.log"
    run_step_demo "$ov_log" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --rate-diffs "$DIFFS" --flat-pulse 40

    n_states="$(grep -cF '"ev":"txpwr.state"' "$ov_log")"
    if [ "$n_states" -lt 3 ]; then
        fail "$name override(flat-pulse): only $n_states txpwr.state events (want >=3)"
    else
        pd_rd="$(state_field "$ov_log" "$((n_states - 2))" rate_diffs)"
        pd_ofdm="$(state_field "$ov_log" "$((n_states - 2))" ofdm)"
        pd_mcs7="$(state_field "$ov_log" "$((n_states - 2))" mcs7)"
        mp_rd="$(state_field "$ov_log" "$((n_states - 1))" rate_diffs)"
        mp_ofdm="$(state_field "$ov_log" "$((n_states - 1))" ofdm)"
        mp_mcs7="$(state_field "$ov_log" "$((n_states - 1))" mcs7)"
        pc_rd="$(state_field "$ov_log" "$n_states" rate_diffs)"
        pc_ofdm="$(state_field "$ov_log" "$n_states" ofdm)"
        pc_mcs7="$(state_field "$ov_log" "$n_states" mcs7)"

        if [ -z "$pd_ofdm" ] || [ -z "$mp_ofdm" ] || [ -z "$pc_ofdm" ]; then
            fail "$name override(flat-pulse): missing state field(s) in last 3 events"
        else
            pd_want_mcs7=$((pd_ofdm + DELTA))
            pc_want_mcs7=$((pc_ofdm + DELTA))
            if [ "$pd_rd" = "1" ] && [ "$pd_mcs7" = "$pd_want_mcs7" ]; then
                pass "$name override(flat-pulse) post-diffs: rate_diffs=1, mcs7-ofdm=$DELTA (ofdm=$pd_ofdm mcs7=$pd_mcs7)"
            else
                fail "$name override(flat-pulse) post-diffs: rate_diffs=$pd_rd ofdm=$pd_ofdm mcs7=$pd_mcs7 (want rate_diffs=1, mcs7=$pd_want_mcs7)"
            fi
            if [ "$mp_rd" = "1" ] && [ "$mp_ofdm" = "40" ] && [ "$mp_mcs7" = "40" ]; then
                pass "$name override(flat-pulse) mid-pulse: rate_diffs=1 (configured) but mcs7=ofdm=40 (honest flat truth)"
            else
                fail "$name override(flat-pulse) mid-pulse: rate_diffs=$mp_rd ofdm=$mp_ofdm mcs7=$mp_mcs7 (want rate_diffs=1, ofdm=40, mcs7=40)"
            fi
            if [ "$pc_rd" = "1" ] && [ "$pc_mcs7" = "$pc_want_mcs7" ]; then
                pass "$name override(flat-pulse) post-clear: rate_diffs=1, mcs7-ofdm=$DELTA restored (ofdm=$pc_ofdm mcs7=$pc_mcs7)"
            else
                fail "$name override(flat-pulse) post-clear: rate_diffs=$pc_rd ofdm=$pc_ofdm mcs7=$pc_mcs7 (want rate_diffs=1, mcs7=$pc_want_mcs7)"
            fi
        fi
    fi
done

echo
echo "== txpwr-rate-diffs regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
