#!/usr/bin/env bash
# Register-level validation of the runtime TX-power API (IRtlDevice::
# SetTxPowerOffsetQdb / SetTxPowerIndexOverride / GetTxPowerState).
#
# Cells per plugged DUT (skip-if-unplugged, PASS/FAIL/SKIP tally like
# tone_mask_regcheck.sh):
#
#   parity   offset untouched => the bring-up TXAGC canary is byte-identical
#            to the master build's (the key no-regression invariant). Needs
#            a master worktree build (cached at $MASTER_BUILD, built on first
#            run). Skipped with SKIP_PARITY=1.
#   move     offset -24 qdB moves every representative per-rate index down by
#            exactly 24/step_qdb steps (12 on Jaguar1/2, 24 on Jaguar3).
#   rails    offset -200/+200 qdB clamps to all-0 / all-max with the
#            saturated_low/high flags set.
#   sticky   offset -24 qdB, then a full SetMonitorChannel to another channel
#            group: the offset re-folds against the NEW channel's table
#            (post-switch indices == that channel's no-offset baseline - steps);
#            then a FastRetune: TXAGC registers untouched (indices unchanged).
#
# Usage: sudo -v && tests/txpwr_offset_regcheck.sh [PID ...]
#   (default: every supported plugged PID)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPWR_REGCHECK_OUT:-/tmp/devourer-txpwr-regcheck}"
MASTER_BUILD="${MASTER_BUILD:-/tmp/devourer-master-build}"
STEP_DEMO="$ROOT/build/txpower"
TX_DEMO="$ROOT/build/txdemo"
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x txpower 2>/dev/null || true
    pkill -x txdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j --target txpower txdemo >/dev/null || exit 1

# DUT table: pid vid ch_a ch_b family
# ch_a/ch_b are same-band channels in different efuse channel groups so the
# sticky cell proves the re-fold uses the new group's base.
DUTS=(
    "0x8812 0x0bda 6 11 jaguar1"   # RTL8812AU
    "0x0120 0x2357 6 11 jaguar1"   # Archer T2U Plus (RTL8821AU)
    "0x8813 0x0bda 6 11 jaguar1"   # RTL8814AU (write-only TXAGC: shadow rb=0)
    "0x012d 0x2357 6 11 jaguar2"   # Archer T3U (RTL8822BU)
    "0xc811 0x0bda 6 11 jaguar2"   # RTL8821CU
    "0xc812 0x0bda 36 149 jaguar3" # RTL8812CU/8822CU
    "0xa81a 0x0bda 36 149 jaguar3" # RTL8812EU/8822EU
)

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

# --- helpers ---------------------------------------------------------------
# Extract numeric field F from the Nth txpwr.state event line of a log.
state_field() { # $1=log $2=line-index(1-based) $3=field
    grep -F '"ev":"txpwr.state"' "$1" | sed -n "$2p" \
        | grep -o "\"$3\":-\?[0-9]*" | cut -d: -f2 | tr -d '\r'
}
caps_field() { # $1=log $2=field
    grep -F '"ev":"txpwr.caps"' "$1" | head -1 \
        | grep -o "\"$2\":-\?[0-9]*" | cut -d: -f2 | tr -d '\r'
}

run_step_demo() { # $1=outfile, rest = args
    local out="$1"; shift
    sudo -n timeout 90 "$STEP_DEMO" "$@" >"$out" 2>&1
}

# The canary dump stays "KIND 0xADDR = 0xVALUE" text, but now arrives as
# stderr diagnostics ("devourer [I] " prefix; master's build still emits the
# old "<devourer>" stdout form). The grep -oE extraction is prefix-agnostic,
# so one extractor serves both builds — run_canary captures 2>&1.
last_canary() { # $1 = log file  (same extractor as hop_parity_check.sh)
    awk '/=== DEVOURER_DUMP_CANARY/{buf=""} {buf=buf $0 "\n"}
         /=== END DEVOURER_DUMP_CANARY/{last=buf} END{printf "%s", last}' "$1" \
      | grep -oE "(BB|MAC|RF\[[AB]\]) 0x[0-9a-fA-F]+ = 0x[0-9A-F]+" \
      | sort -u
}

# Run-variant registers excluded from the parity diff (same list the
# hop-parity canary uses): TSF, IQK/RxIQC measurement jitter, live AGC /
# thermal RF words (RF 0x00 / 0x42 — hop_parity_check.sh's LIVE list), and
# the 8822E RXBB word RF 0x1a — its bit 18 latches prior-session residue
# (the chip retains state across soft re-init; A/B/A/B standalone runs of
# both builds read identically, only the in-script cell ordering flips it).
PARITY_EXCLUDE='MAC 0x560|BB 0xc10|BB 0xc14|BB 0xe10|BB 0xe14|BB 0xc90|BB 0xc94|BB 0xe90|BB 0xe94|RF\[[AB]\] 0x00 |RF\[[AB]\] 0x42 |RF\[[AB]\] 0x1a '

ensure_master_build() {
    [ -x "$MASTER_BUILD/txdemo" ] && return 0
    echo "== building master baseline (one-time, $MASTER_BUILD) =="
    local wt="/tmp/devourer-master-worktree"
    git -C "$ROOT" worktree add --force "$wt" origin/master >/dev/null 2>&1 || return 1
    cmake -S "$wt" -B "$MASTER_BUILD" -DPKG_CONFIG_EXECUTABLE=/usr/bin/pkg-config >/dev/null 2>&1 || return 1
    cmake --build "$MASTER_BUILD" -j --target txdemo >/dev/null 2>&1 || return 1
}

run_canary() { # $1=binary $2=pid $3=vid $4=channel $5=outfile
    sudo -n env DEVOURER_PID="$2" DEVOURER_VID="$3" DEVOURER_CHANNEL="$4" \
        DEVOURER_DUMP_CANARY=1 DEVOURER_TX_GAP_US=50000 \
        timeout 20 "$1" >"$5" 2>&1 || true
}

# --- cells ------------------------------------------------------------------
for dut in "${DUTS[@]}"; do
    read -r PID VID CH_A CH_B FAMILY <<<"$dut"
    name="$PID@$VID ($FAMILY)"
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

    # -- caps sanity + baselines --------------------------------------------
    base_a="$OUT/$tag-base-a.log"
    run_step_demo "$base_a" --vid "$VID" --pid "$PID" --channel "$CH_A"
    if [ "$(caps_field "$base_a" supported)" != "1" ]; then
        skip "$name: TX-power API not wired for this family yet"
        continue
    fi
    step_qdb="$(caps_field "$base_a" step_qdb)"
    idx_max="$(caps_field "$base_a" max)"
    steps24=$((24 / step_qdb))
    cck_a="$(state_field "$base_a" 1 cck)"; ofdm_a="$(state_field "$base_a" 1 ofdm)"
    mcs7_a="$(state_field "$base_a" 1 mcs7)"; rb_a="$(state_field "$base_a" 1 rb)"
    if [ -z "$ofdm_a" ] || [ "$ofdm_a" = "-1" ]; then
        fail "$name: no baseline state readback"
        continue
    fi
    echo "  baseline ch$CH_A: cck=$cck_a ofdm=$ofdm_a mcs7=$mcs7_a rb=$rb_a (step_qdb=$step_qdb max=$idx_max)"

    base_b="$OUT/$tag-base-b.log"
    run_step_demo "$base_b" --vid "$VID" --pid "$PID" --channel "$CH_B"
    cck_b="$(state_field "$base_b" 1 cck)"; ofdm_b="$(state_field "$base_b" 1 ofdm)"
    mcs7_b="$(state_field "$base_b" 1 mcs7)"

    # -- parity: offset untouched == master build (TXAGC canary) -------------
    if [ "${SKIP_PARITY:-0}" = "1" ]; then
        skip "$name parity (SKIP_PARITY=1)"
    elif ensure_master_build; then
        run_canary "$MASTER_BUILD/txdemo" "$PID" "$VID" "$CH_A" "$OUT/$tag-canary-master.log"
        run_canary "$TX_DEMO" "$PID" "$VID" "$CH_A" "$OUT/$tag-canary-new.log"
        last_canary "$OUT/$tag-canary-master.log" | grep -Ev "$PARITY_EXCLUDE" >"$OUT/$tag-canary-master.regs"
        last_canary "$OUT/$tag-canary-new.log"    | grep -Ev "$PARITY_EXCLUDE" >"$OUT/$tag-canary-new.regs"
        # Compare on the INTERSECTION of dumped addresses: this branch adds
        # registers to the canary (e.g. the Jaguar3 TXAGC refs) that master
        # doesn't dump — additions are reported, not failed; every register
        # master dumps must be byte-identical.
        rm -f "$OUT/$tag-common-master.regs" "$OUT/$tag-common-new.regs"
        awk 'NR==FNR { m[$1" "$2]=$0; next }
             ($1" "$2) in m { print m[$1" "$2] >> "'"$OUT/$tag-common-master.regs"'";
                              print $0        >> "'"$OUT/$tag-common-new.regs"'" }' \
            "$OUT/$tag-canary-master.regs" "$OUT/$tag-canary-new.regs"
        added=$(( $(wc -l <"$OUT/$tag-canary-new.regs") - $(wc -l <"$OUT/$tag-common-new.regs" 2>/dev/null || echo 0) ))
        if [ ! -s "$OUT/$tag-canary-master.regs" ]; then
            skip "$name parity (no canary block from master build — family lacks DUMP_CANARY?)"
        elif diff -q "$OUT/$tag-common-master.regs" "$OUT/$tag-common-new.regs" >/dev/null 2>&1; then
            pass "$name parity: canary byte-identical to master ($(wc -l <"$OUT/$tag-common-new.regs") regs common, $added new-only)"
        else
            fail "$name parity: canary diverged from master ($OUT/$tag-common-{master,new}.regs)"
            diff "$OUT/$tag-common-master.regs" "$OUT/$tag-common-new.regs" | head -8 | sed 's/^/    /'
        fi
        # Jaguar2's canary omits the TXAGC block; the bring-up log carries the
        # computed per-section indices — compare those (the fold refactor must
        # be value-identical at offset 0). VHT lines are new-build-only (the
        # approved 8822B VHT extension) and excluded.
        if [ "$FAMILY" = "jaguar2" ]; then
            # 8822B logs "TXAGC path N ..." per path; 8821C logs the vendor-
            # formula line with the per-section bases + limits. VHT lines are
            # new-build-only (the approved 8822B VHT extension) and excluded.
            txagc_log() { grep -ohE "TXAGC path [01] ch[0-9]+ .*|per-rate TXAGC \(vendor formula\) .*" "$1" | grep -v VHT | sort -u; }
            txagc_log "$OUT/$tag-canary-master.log" >"$OUT/$tag-txagc-master.txt"
            txagc_log "$OUT/$tag-canary-new.log"    >"$OUT/$tag-txagc-new.txt"
            if [ ! -s "$OUT/$tag-txagc-master.txt" ]; then
                skip "$name txagc-log parity (no TXAGC log lines from master)"
            elif diff -q "$OUT/$tag-txagc-master.txt" "$OUT/$tag-txagc-new.txt" >/dev/null; then
                pass "$name txagc-log parity: per-section indices identical to master"
            else
                fail "$name txagc-log parity: computed TXAGC diverged from master"
                diff "$OUT/$tag-txagc-master.txt" "$OUT/$tag-txagc-new.txt" | head -6 | sed 's/^/    /'
            fi
        fi
    else
        skip "$name parity (master baseline build failed)"
    fi

    # -- thermal plausibility --------------------------------------------------
    th="$OUT/$tag-thermal.log"
    run_step_demo "$th" --vid "$VID" --pid "$PID" --channel "$CH_A" --thermal
    raw="$(grep -F '"ev":"thermal"' "$th" | head -1 \
        | grep -o '"raw":[0-9-]*' | cut -d: -f2 | tr -d '\r')"
    if [ -n "$raw" ] && [ "$raw" -ge 1 ] 2>/dev/null && [ "$raw" -le 63 ]; then
        pass "$name thermal: raw=$raw plausible (baseline=$(grep -F '"ev":"thermal"' "$th" | head -1 | grep -o '"baseline":[^,}]*' | cut -d: -f2))"
    else
        fail "$name thermal: raw='$raw' implausible/missing"
    fi

    # -- move: -24 qdB shifts every index by exactly steps24 ------------------
    move="$OUT/$tag-move.log"
    run_step_demo "$move" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --offset-start -24 --offset-stop -24
    ofdm_m="$(state_field "$move" 2 ofdm)"; mcs7_m="$(state_field "$move" 2 mcs7)"
    cck_m="$(state_field "$move" 2 cck)"
    want_ofdm=$((ofdm_a - steps24)); want_mcs7=$((mcs7_a - steps24))
    want_cck=$((cck_a - steps24)); [ "$want_cck" -lt 0 ] && want_cck=0
    [ "$want_ofdm" -lt 0 ] && want_ofdm=0
    [ "$want_mcs7" -lt 0 ] && want_mcs7=0
    if [ "$ofdm_m" = "$want_ofdm" ] && [ "$mcs7_m" = "$want_mcs7" ] && [ "$cck_m" = "$want_cck" ]; then
        pass "$name move: -24 qdB -> indices -$steps24 (cck $cck_a->$cck_m ofdm $ofdm_a->$ofdm_m mcs7 $mcs7_a->$mcs7_m)"
    else
        fail "$name move: want cck=$want_cck ofdm=$want_ofdm mcs7=$want_mcs7, got cck=$cck_m ofdm=$ofdm_m mcs7=$mcs7_m"
    fi

    # -- rails ----------------------------------------------------------------
    lo="$OUT/$tag-rail-lo.log"; hi="$OUT/$tag-rail-hi.log"
    run_step_demo "$lo" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --offset-start -200 --offset-stop -200
    run_step_demo "$hi" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --offset-start 200 --offset-stop 200
    if [ "$(state_field "$lo" 2 ofdm)" = "0" ] && [ "$(state_field "$lo" 2 satlo)" = "1" ]; then
        pass "$name rail-low: all-0 + saturated_low"
    else
        fail "$name rail-low: ofdm=$(state_field "$lo" 2 ofdm) satlo=$(state_field "$lo" 2 satlo)"
    fi
    if [ "$(state_field "$hi" 2 ofdm)" = "$idx_max" ] && [ "$(state_field "$hi" 2 sathi)" = "1" ]; then
        pass "$name rail-high: all-$idx_max + saturated_high"
    else
        fail "$name rail-high: ofdm=$(state_field "$hi" 2 ofdm) sathi=$(state_field "$hi" 2 sathi)"
    fi

    # -- sticky: full switch re-folds, fast retune leaves TXAGC alone ---------
    st="$OUT/$tag-sticky.log"
    run_step_demo "$st" --vid "$VID" --pid "$PID" --channel "$CH_A" \
        --offset-start -24 --offset-stop -24 --switch-channel "$CH_B" --retune "$CH_A"
    # line 2 = post-offset @CH_A, line 3 = post-SetMonitorChannel @CH_B,
    # line 4 = post-FastRetune @CH_A (registers must still hold CH_B's values
    # only if retune skipped TXAGC — i.e. line4 == line3).
    off_sw="$(state_field "$st" 3 offset_qdb)"
    ofdm_sw="$(state_field "$st" 3 ofdm)"; mcs7_sw="$(state_field "$st" 3 mcs7)"
    want_ofdm_b=$((ofdm_b - steps24)); [ "$want_ofdm_b" -lt 0 ] && want_ofdm_b=0
    want_mcs7_b=$((mcs7_b - steps24)); [ "$want_mcs7_b" -lt 0 ] && want_mcs7_b=0
    if [ "$off_sw" = "-24" ] && [ "$ofdm_sw" = "$want_ofdm_b" ] && [ "$mcs7_sw" = "$want_mcs7_b" ]; then
        pass "$name sticky-full: offset held across SetMonitorChannel and re-folded (ofdm $ofdm_b-$steps24=$ofdm_sw)"
    else
        fail "$name sticky-full: offset=$off_sw ofdm=$ofdm_sw (want -24 / $want_ofdm_b; ch$CH_B base $ofdm_b)"
    fi
    ofdm_rt="$(state_field "$st" 4 ofdm)"; mcs7_rt="$(state_field "$st" 4 mcs7)"
    rb_rt="$(state_field "$st" 4 rb)"
    if [ "$rb_rt" != "1" ]; then
        skip "$name sticky-fast (no hw readback on this family/chip)"
    elif [ "$ofdm_rt" = "$ofdm_sw" ] && [ "$mcs7_rt" = "$mcs7_sw" ]; then
        pass "$name sticky-fast: FastRetune left TXAGC untouched"
    else
        fail "$name sticky-fast: ofdm $ofdm_sw->$ofdm_rt mcs7 $mcs7_sw->$mcs7_rt across FastRetune"
    fi

    # -- 8822E path-B OFDM ref (0x41e8): applied in BOTH TX-only and TX+RX
    #    modes. (The historical RX-desense quirk that pinned it to the table
    #    default in TX+RX is RESOLVED — it was the DPDT/pin-mux front-end
    #    mis-config; re-tested clean by tests/eu_41e8_desense_recheck.sh.)
    #    Canary #1 dumps at the InitWrite channel-set (pre-power, BB-table
    #    default); canary #2 at --switch-channel still holds the FIRST
    #    channel's applied refs (the re-apply runs after the dump). Both
    #    modes: 0x41e8 AND 0x18e8 refs must move between the dumps.
    if [ "$PID" = "0xa81a" ]; then
        qref() { # $1=log $2=reg $3=which(1=first,2=last) -> 7-bit ref field
            python3 - "$1" "$2" "$3" <<'PYEOF'
import re, sys
log, reg, which = sys.argv[1], sys.argv[2].lower(), int(sys.argv[3])
vals = [int(m.group(1), 16) for m in
        re.finditer(r"BB 0x0*" + reg.replace("0x", "") + r" = 0x([0-9A-Fa-f]+)",
                    open(log).read())]
if not vals:
    print("nan"); sys.exit(0)
v = vals[0] if which == 1 else vals[-1]
print((v >> 10) & 0x7f)
PYEOF
        }
        qt="$OUT/$tag-quirk-txonly.log"; qr="$OUT/$tag-quirk-txrx.log"
        sudo -n env DEVOURER_DUMP_CANARY=1 timeout 90 "$STEP_DEMO" \
            --vid "$VID" --pid "$PID" --channel "$CH_A" \
            --offset-start -24 --offset-stop -24 --switch-channel "$CH_B" \
            >"$qt" 2>&1
        sudo -n env DEVOURER_DUMP_CANARY=1 DEVOURER_TX_WITH_RX=thread \
            timeout 90 "$STEP_DEMO" \
            --vid "$VID" --pid "$PID" --channel "$CH_A" \
            --offset-start -24 --offset-stop -24 --switch-channel "$CH_B" \
            >"$qr" 2>&1
        b1_txonly="$(qref "$qt" 41e8 1)"; b2_txonly="$(qref "$qt" 41e8 2)"
        b1_txrx="$(qref "$qr" 41e8 1)";   b2_txrx="$(qref "$qr" 41e8 2)"
        a1_txrx="$(qref "$qr" 18e8 1)";   a2_txrx="$(qref "$qr" 18e8 2)"
        if [ "$b1_txrx" = "nan" ] || [ "$a1_txrx" = "nan" ]; then
            fail "$name pathb-41e8: no canary refs in TX+RX run"
        elif [ "$b1_txrx" = "$b2_txrx" ]; then
            fail "$name pathb-41e8: TX+RX run did NOT write 0x41e8 ($b1_txrx) — path-B power missing (desense-era skip resurrected?)"
        elif [ "$a1_txrx" = "$a2_txrx" ]; then
            fail "$name pathb-41e8: TX+RX run never applied path-A power (0x18e8 $a1_txrx unchanged)"
        elif [ "$b1_txonly" = "$b2_txonly" ]; then
            fail "$name pathb-41e8: TX-only run did NOT write 0x41e8 ($b1_txonly) — path-B power missing"
        else
            pass "$name pathb-41e8: path-B ref applied in both modes (TX+RX $b1_txrx->$b2_txrx, TX-only $b1_txonly->$b2_txonly; path A $a1_txrx->$a2_txrx)"
        fi
    fi
done

echo
echo "== txpwr-offset regcheck: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
