#!/usr/bin/env bash
# Register-level validation of the carrier-sense gate split
# (IRtlRadio::SetCcaGates / GetCcaGates), and the in-tree reproduction of the
# tables in the PR that added it.
#
# The API-level walk is build/CcaGatesProbe (tests/cca_gates_probe.cpp); this
# script cross-checks what the API says against what the chip holds, by
# peeking registers with examples/chipstate --no-claim while the probe owns
# the interface. The two disagreeing is itself the finding — an API reporting
# a gate state the silicon does not have is the failure mode the not-ported
# defaults exist to prevent.
#
# Cells, each of which must be able to FAIL:
#   api       the four gate states, the pre-bring-up refusal and the two legacy
#             SetCcaMode states, as CcaGatesProbe reports them.
#   regs      0x520[14] primary CCA and 0x520[15] EDCCA track the two arguments
#             independently, read from the chip while the state is applied.
#   cntdown   0x524[11] BIT_EDCCA_MSK_CNTDOWN_EN follows the EDCCA gate ALONE.
#             Keyed on the pair instead, the EDCCA-off/primary-on arm leaves
#             EDCCA masking the backoff countdown — the gate the caller asked
#             to turn off is only half off. Whether a backend drives the bit
#             at all is DISCOVERED rather than tabulated: SetCcaMode moves
#             both gates, so a backend in which the bit has this role must
#             move it between the two legacy states. The cell then fails if
#             the two paths disagree in either direction.
#             KNOWN LIMIT: a backend that stops writing the bit ENTIRELY is
#             reported, not failed — with both paths silent there is nothing
#             left to compare against, and deciding it "should" have moved
#             would mean a per-chip expectation table, which this tree
#             deliberately does not keep. The `track` cell is independent and
#             does cover the functional half of the same arm.
#   legacy    SetCcaMode(d) writes what SetCcaGates(d, d) writes, so the split
#             changed no default. The no-regression cell.
#   track     the phydm EDCCA tracker stops in an EDCCA-off arm. Poked rather
#             than sampled: the tracker recomputes the same th_l2h from a
#             static IGI, so an active tracker rewrites the SAME bytes and is
#             indistinguishable from an idle one by observation. Write a value
#             it would never choose and see if it is restored. The threshold
#             register is PER-FAMILY (Jaguar1 0x8a4 bytes 0/1, Jaguar3
#             0x84c[23:16]), taken from the generation the probe reports —
#             poking the other family's register reports "no tracker" and
#             passes a broken tracker silently. Skipped where no tracker runs
#             in the default arm, or where the generation has no known
#             threshold register.
#   retune    the state survives SetMonitorChannel and FastRetune, within a
#             band and across a band change. Jaguar3 re-asserts by design;
#             Jaguar1 merely is not clobbered (see src/IRtlRadio.h) — so this
#             reports the mechanism and fails only on actual loss. Checks
#             0x524[11] alongside the API readback, because GetCcaGates reads
#             0x520 alone and cannot see the countdown bit go missing.
#
# Usage: sudo -v && tests/cca_gates_regcheck.sh            # every plugged part
#        PIDS=0xc812 sudo -v && tests/cca_gates_regcheck.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CCA_GATES_OUT:-/tmp/devourer-cca-gates}"
# Honour a build directory elsewhere: this script is also run against a
# vendored copy of the tree whose build lives outside it.
BUILD="${BUILD:-$ROOT/build}"
VID=${VID:-0x0bda}
PIDS="${PIDS:-0x8812 0xc812 0xf72b}"   # Jaguar1, Jaguar3, and an unported family
CH="${CH:-36}"; CH_SAME="${CH_SAME:-40}"; CH_BAND="${CH_BAND:-6}"
MARK_L2H=0x11                          # nothing max(igi+8,48) can produce
# The BB EDCCA threshold register is per-generation — Jaguar1 writes L2H/H2L
# as 0x8a4 bytes 0/1 (src/jaguar1/RtlJaguarDevice.cpp apply_cca and
# PhydmWatchdog::TickOnce), Jaguar3 writes th_l2h to 0x84c[23:16]
# (src/jaguar3/PhydmRuntimeJaguar3.cpp). Poking the other family's register
# reports "no tracker running" instead of failing, so the probe names its
# generation (GATES-GEN) and the track cell picks from here.
edcca_th_reg() { case "$1" in jaguar1) echo $((0x8a4));; jaguar3) echo $((0x84c));; *) echo "";; esac; }
edcca_th_shift() { case "$1" in jaguar1) echo 0;; jaguar3) echo 16;; *) echo "";; esac; }
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }
note() { echo "  note: $*"; }
probe_pid=""
# The probe is started through sudo, so its process is root-owned and a plain
# kill from this (unprivileged) shell gets EPERM — silently, leaving `wait` to
# block for the probe's whole hold walk. Every stop goes through sudo.
kill_probe() { [ -n "$probe_pid" ] && sudo -n kill "$probe_pid" 2>/dev/null; true; }
cleanup() { kill_probe; sudo -n pkill -x CcaGatesProbe 2>/dev/null; true; }
trap cleanup EXIT INT TERM

echo "== building =="
# A vendored copy's binaries may be built by an enclosing project, in which
# case $BUILD holds the outputs but no build system of its own. Try to build,
# then require the binaries either way — "cmake could not build here" is only
# a failure if the tools are actually missing.
cmake --build "$BUILD" -j --target CcaGatesProbe chipstate >/dev/null 2>&1 || true
for t in CcaGatesProbe chipstate; do
    [ -x "$BUILD/$t" ] || { echo "missing $BUILD/$t — build it, or set BUILD=<dir>"; exit 1; }
done

# One little-endian dword, read out of band. Prints nothing and returns 1 when
# the read did not produce four bytes — a peek that silently returned 0 would
# make "both gates enabled" the reading for a failed sudo, a busy adapter or a
# dead probe, i.e. the default arm would pass on no evidence.
peek32() { # $1=pid $2=addr
    local bytes n
    bytes=$(sudo -n "$BUILD/chipstate" --pid "$1" --no-claim \
        --peek "$(printf '0x%x-0x%x' "$2" $(( $2 + 3 )))" 2>&1 |
        sed -n 's/^0x[0-9a-fA-F]\{4\}://p' | tr -s ' ' '\n' |
        grep -E '^[0-9a-f]{2}$' | head -4)
    n=$(printf '%s\n' "$bytes" | grep -c .)
    [ "$n" -eq 4 ] || return 1
    printf '%s\n' "$bytes" |
        awk '{b[NR]=strtonum("0x"$1)} END{printf "%u\n", b[1]+b[2]*256+b[3]*65536+b[4]*16777216}'
}
poke32() { sudo -n "$BUILD/chipstate" --pid "$1" --no-claim \
            --poke "$(printf '0x%x=0x%x:4' "$2" "$3")" >/dev/null 2>&1; }
bit() { echo $(( ( $1 >> $2 ) & 1 )); }

# Start a probe that parks in each state for --hold, and wait until it has
# REPORTED the wanted arm (the probe holds after reporting, so the state is
# applied for the whole window the marker opens).
start_hold() { # $1=pid $2=log $3=extra args...
    local pid="$1" log="$2"; shift 2
    sudo -n "$BUILD/CcaGatesProbe" --vid "$VID" --pid "$pid" \
        --channel "$CH" --hold 8 "$@" >"$log" 2>&1 &
    probe_pid=$!
}
wait_marker() { # $1=log $2=marker
    local i
    for i in $(seq 1 90); do
        grep -q "$2" "$1" && return 0
        kill -0 "$probe_pid" 2>/dev/null || return 1
        sleep 1
    done
    return 1
}
stop_hold() {
    [ -n "$probe_pid" ] || return 0
    kill_probe
    # sudo forwards the signal to the child, but reaps it first; give the
    # probe a moment to release the interface before the next claim.
    wait "$probe_pid" 2>/dev/null
    sudo -n pkill -x CcaGatesProbe 2>/dev/null
    probe_pid=""
}

sudo -n true 2>/dev/null || { echo "needs a live sudo credential: sudo -v"; exit 2; }

for pid in $PIDS; do
    echo
    echo "######## $pid ########"
    lsusb -d "$(printf '%04x:%04x' "$VID" "$pid")" >/dev/null 2>&1 || {
        skip "$pid not plugged"; continue; }

    log="$OUT/probe-$pid.log"
    sudo -n "$BUILD/CcaGatesProbe" --vid "$VID" --pid "$pid" \
        --channel "$CH" --retune "$CH_SAME" >"$log" 2>&1
    rc=$?
    if [ $rc -eq 3 ]; then
        # Could not open the adapter at all: busy, absent, or CreateRadio
        # refused. That is a bench fact, not a verdict on the gate split, and
        # reporting it as eight failing cells is how a maintainer running this
        # next to his own tools gets told his feature is broken. Quote what
        # the probe said rather than inventing a diagnosis.
        # Prefer the line that names the cause over the teardown noise that
        # follows it — "libusb_release_interface rc=-5" is what happens after
        # the open failed, not why.
        why=$(grep -iE "is BUSY|already (in use|using)|no adapter|CreateRadio failed" "$log" | head -1)
        [ -n "$why" ] || why=$(grep -iE "error|warn" "$log" | head -1)
        skip "$pid could not be opened — ${why:-see $log}"
        continue
    fi
    if [ $rc -eq 4 ]; then skip "$pid is not a Realtek radio"; continue; fi
    if [ $rc -eq 5 ]; then
        # The not-ported default is a PASS, not an absence of one: the point of
        # the contract is that a backend without the split says so. Anchored on
        # "^PASS " because the probe prints PASS and FAIL through the same
        # formatter, so a substring match would accept the failure too.
        if grep -q "^PASS GetCcaGates refuses before bring-up" "$log" &&
           grep -q "^PASS SetCcaGates refuses before bring-up" "$log" &&
           grep -q "^PASS GetCcaGates leaves out-params alone" "$log"; then
            pass "$pid: gate split not ported, and both calls refuse cleanly"
        else
            fail "$pid: not-ported backend did not refuse cleanly (see $log)"
        fi
        continue
    fi
    [ $rc -eq 0 ] && pass "$pid api: probe walk clean" \
                  || fail "$pid api: probe reported failures (see $log)"

    gen=$(sed -n 's/^GATES-GEN //p' "$log" | head -1)
    note "$pid generation: ${gen:-unknown}"

    # --- regs + cntdown ---------------------------------------------------
    cd_seen=""; uses_cd=0
    for arm in "0 0" "0 1" "1 0" "1 1"; do
        set -- $arm; want_p=$1; want_e=$2
        hold_log="$OUT/hold-$pid-$want_p$want_e.log"
        start_hold "$pid" "$hold_log"
        if ! wait_marker "$hold_log" "^GATES set-primary$want_p-edcca$want_e "; then
            fail "$pid regs: probe never reported primary=$want_p edcca=$want_e"
            stop_hold; continue
        fi
        v520=$(peek32 "$pid" $((0x520))) || { fail "$pid regs: 0x520 peek failed (primary=$want_p edcca=$want_e)"; stop_hold; continue; }
        v524=$(peek32 "$pid" $((0x524))) || { fail "$pid regs: 0x524 peek failed (primary=$want_p edcca=$want_e)"; stop_hold; continue; }
        stop_hold

        got_p=$(bit "$v520" 14); got_e=$(bit "$v520" 15)
        if [ "$got_p" = "$want_p" ] && [ "$got_e" = "$want_e" ]; then
            pass "$pid regs: primary=$want_p edcca=$want_e -> 0x520[14/15]=$got_p/$got_e"
        else
            fail "$pid regs: asked primary=$want_p edcca=$want_e, 0x520 says $got_p/$got_e"
        fi
        cd_seen="$cd_seen $want_p$want_e:$(bit "$v524" 11)"
    done

    # The two legacy states, for the same register. SetCcaMode moves both
    # gates, so these bracket what 0x524[11] does on this backend when the
    # pair moves — the reference the split is judged against below.
    cd_mode_on=""; cd_mode_off=""
    for mode in true false; do
        mlog="$OUT/mode-$pid-$mode.log"
        start_hold "$pid" "$mlog"
        if wait_marker "$mlog" "^GATES setccamode-$mode "; then
            v=$(peek32 "$pid" $((0x524))) && {
                if [ "$mode" = true ]; then cd_mode_on=$(bit "$v" 11)
                else cd_mode_off=$(bit "$v" 11); fi
            }
        fi
        stop_hold
    done

    # cntdown: EDCCA-scoped means the bit is CLEAR exactly when EDCCA is
    # disabled, whatever primary CCA is doing. A backend that never moves it
    # does not use it in this role; one that moves it on primary CCA, or with
    # the pair, fails here.
    if [ "$(echo "$cd_seen" | tr ' ' '\n' | grep -c .)" -ne 4 ]; then
        # Fewer than four arms reported, so there is nothing to compare.
        # Say so: dropping the cell without a verdict moves no counter and
        # reads, in the summary, exactly like a cell that was never meant
        # to run here.
        skip "$pid cntdown: only $(echo "$cd_seen" | tr ' ' '\n' | grep -c .)/4 arms reported (see $OUT)"
    else
        vals=$(echo "$cd_seen" | tr ' ' '\n' | grep . | cut -d: -f2 | sort -u | tr -d '\n')
        # Whether this backend drives 0x524[11] is discovered, not tabulated:
        # SetCcaMode moves both gates, so if the bit is in this role at all it
        # must differ between the two legacy states. That turns "the bit never
        # moved" from an untestable observation into a real expectation — a
        # Jaguar3 that stopped writing it would otherwise pass silently.
        uses_cd=0
        if [ "$cd_mode_on" != "$cd_mode_off" ]; then uses_cd=1; fi
        if [ "$uses_cd" = 0 ]; then
            if [ "$vals" = "01" ]; then
                fail "$pid cntdown: 0x524[11] moves with the split but not with SetCcaMode ($cd_seen)"
            else
                note "$pid cntdown: 0x524[11] constant at $vals — not an EDCCA gate on this backend"
            fi
        elif [ "$vals" != "01" ]; then
            fail "$pid cntdown: SetCcaMode moves 0x524[11] but the split leaves it at $vals ($cd_seen)"
        else
            ok=1
            for e in $cd_seen; do
                want_e=${e%%:*}; want_e=${want_e#?}; got=${e##*:}
                exp=$(( want_e == 1 ? 0 : 1 ))
                [ "$got" = "$exp" ] || ok=0
            done
            [ "$ok" = 1 ] \
                && pass "$pid cntdown: 0x524[11] follows the EDCCA gate alone ($cd_seen)" \
                || fail "$pid cntdown: 0x524[11] moves, but not with EDCCA ($cd_seen)"
        fi
    fi

    grep -q "^PASS SetCcaMode moves both gates together" "$log" \
        && pass "$pid legacy: SetCcaMode(d) == SetCcaGates(d, d)" \
        || fail "$pid legacy: SetCcaMode no longer moves both gates"

    # --- track ------------------------------------------------------------
    # Does an EDCCA tracker overwrite the BB thresholds behind the caller?
    th_reg=$(edcca_th_reg "$gen"); th_shift=$(edcca_th_shift "$gen")
    tracker_in_default=""
    if [ -z "$th_reg" ]; then
        skip "$pid track: no EDCCA threshold register known for generation '${gen:-unknown}'"
    else
    th_name=$(printf '0x%x' "$th_reg")
    for arm in "0 0" "0 1"; do
        set -- $arm; want_p=$1; want_e=$2
        tlog="$OUT/track-$pid-$want_p$want_e.log"
        start_hold "$pid" "$tlog"
        if ! wait_marker "$tlog" "^GATES set-primary$want_p-edcca$want_e "; then
            fail "$pid track: probe never reported primary=$want_p edcca=$want_e"
            stop_hold; continue
        fi
        native=$(peek32 "$pid" "$th_reg") || { fail "$pid track: $th_name peek failed"; stop_hold; continue; }
        poke32 "$pid" "$th_reg" $(( (native & ~(0xff << th_shift)) | (MARK_L2H << th_shift) ))
        sleep 5
        # Restore BEFORE any early exit: leaving the BB threshold at the
        # marker would hand the next arm — and the next run — a chip in a
        # state this script invented.
        after=$(peek32 "$pid" "$th_reg")
        rc2=$?
        poke32 "$pid" "$th_reg" "$native"
        [ $rc2 -eq 0 ] || { fail "$pid track: $th_name re-read failed"; stop_hold; continue; }
        stop_hold
        if [ $(( (after >> th_shift) & 0xff )) -eq $(( MARK_L2H )) ]; then
            restored=0; else restored=1; fi
        if [ "$want_e" = 0 ]; then
            tracker_in_default=$restored
            [ "$restored" = 1 ] \
                && note "$pid track: tracker IS running at $th_name in the default arm (as expected)" \
                || note "$pid track: no EDCCA tracker running at $th_name in the default arm"
        else
            if [ -z "$tracker_in_default" ]; then
                fail "$pid track: default arm never measured, so the EDCCA-off arm proves nothing"
            elif [ "$tracker_in_default" = 0 ]; then
                skip "$pid track: no tracker to stop in this configuration"
            elif [ "$restored" = 0 ]; then
                pass "$pid track: EDCCA tracking stops when EDCCA is the gate turned off"
            else
                fail "$pid track: tracker still rewriting $th_name with EDCCA disabled"
            fi
        fi
    done
    fi

    # --- retune -----------------------------------------------------------
    # Both channel paths, and both a same-band hop and a band change, because
    # Jaguar3's FastRetune fallback does not carry SetMonitorChannel's
    # re-assert and so is a separate question.
    for target in "$CH_SAME" "$CH_BAND"; do
        for path in retune fast-retune; do
            rlog="$OUT/$path-$pid-$target.log"
            marker=$([ "$path" = retune ] && echo after-retune || echo after-fast-retune)
            # Held after the report so 0x524 can be peeked while the state is
            # still applied: GetCcaGates reads 0x520 alone, so the API half of
            # this cell cannot see the countdown bit being lost.
            start_hold "$pid" "$rlog" "--$path" "$target"
            if ! wait_marker "$rlog" "^GATES $marker "; then
                fail "$pid $path ch$CH->ch$target: probe never reported $marker"
                stop_hold; continue
            fi
            v524=$(peek32 "$pid" $((0x524))); rc3=$?
            stop_hold
            line=$(grep "^GATES $marker" "$rlog" | head -1)
            if ! echo "$line" | grep -q "ret=1 primary=1 edcca=0"; then
                fail "$pid $path ch$CH->ch$target: expected primary=1 edcca=0, got '${line:-no line}'"
                continue
            fi
            # The arm is EDCCA ENABLED (edcca=0), so an EDCCA-scoped
            # countdown bit must still be set. Only assert it where the
            # cntdown cell established the backend drives the bit at all.
            if [ "$uses_cd" = 1 ] && [ $rc3 -eq 0 ] && [ "$(bit "$v524" 11)" != 1 ]; then
                fail "$pid $path ch$CH->ch$target: 0x520 survived but 0x524[11] was lost"
            else
                pass "$pid $path ch$CH->ch$target: gate state intact"
            fi
        done
    done
done

echo
echo "== $PASS passed, $FAIL failed, $SKIP skipped =="
[ "$FAIL" -eq 0 ]
