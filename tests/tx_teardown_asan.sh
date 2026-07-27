#!/usr/bin/env bash
# Does a max-duty TX session tear down cleanly when it is killed mid-flight?
#
# Jaguar1 is the only generation whose send path is asynchronous: send_packet
# submits a bulk-OUT URB and returns, so at a high frame rate there are always
# transfers outstanding. Two things have to hold at exit, and neither is
# observable at low duty (where the queue drains between frames — which is why
# a gap-2000 run has always looked fine):
#
#   1. the URB may not reference memory the caller has already freed;
#   2. the device — and with it those URBs — must die BEFORE libusb does.
#
# This drives an ASan build of txdemo at DEVOURER_TX_GAP_US=0, kills it with
# SIGTERM mid-stream, and asserts a clean exit with no sanitizer report. The
# gap-2000 and aggregation variants are run too: the first is the case that
# always worked (a regression check), the second exercises the multi-frame
# aggregation URB, which builds a differently-sized buffer.
#
# Build first (any Jaguar1 adapter; the DUT defaults to the 8821AU):
#   cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
#         -DDEVOURER_SANITIZE=address && cmake --build build-asan -j
# Run:
#   sudo -v && tests/tx_teardown_asan.sh
#   TX_VID=0x0bda TX_PID=0x8813 REPS=3 tests/tx_teardown_asan.sh   # 8814AU
#
# BUILD=<dir> points at a different (e.g. pre-fix) tree, which is how the
# defect is demonstrated rather than merely asserted away.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

BUILD="${BUILD:-$ROOT/build-asan}"
TX_VID="${TX_VID:-0x2357}"
TX_PID="${TX_PID:-0x0120}"   # Archer T2U Plus — RTL8821AU (Jaguar1)
CHANNEL="${CHANNEL:-6}"
TX_RATE="${TX_RATE:-MCS7}"
RUN_SECS="${RUN_SECS:-8}"
REPS="${REPS:-3}"
OUT="${OUT:-/tmp/devourer-tx-teardown}"

TXDEMO="$BUILD/txdemo"
[ -x "$TXDEMO" ] || { echo "no txdemo at $TXDEMO — build it first"; exit 1; }
lsusb -d "$(printf '%04x:%04x' "$TX_VID" "$TX_PID")" >/dev/null 2>&1 || {
    echo "SKIP: TX adapter $TX_VID:$TX_PID not plugged"; exit 77; }

mkdir -p "$OUT"
rm -f "$OUT"/*.log "$OUT"/asan.* 2>/dev/null || true

cleanup() {
    sudo pkill -x txdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# The in-tree rtw88 modules claim every Realtek dongle at enumeration. Unbind
# the interface rather than rmmod: a module removal does not survive the next
# re-enumeration, and it de-inits the chip on the way out.
unbind() {
    local want="${TX_PID#0x}" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "$want" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] || continue
            sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

fail=0
pass=0
SIG="${SIG:-TERM}"

# One kill-during-TX cell. $1 = label, remaining args = extra env assignments.
cell() {
    local label="$1"; shift
    local rep log rc
    for rep in $(seq 1 "$REPS"); do
        log="$OUT/${label}-${rep}.log"
        unbind
        # ASAN_OPTIONS: abort_on_error makes a sanitizer finding a distinct
        # exit status instead of a silently-appended report; the log prefix
        # keeps the report out of the demo's own stderr stream.
        sudo env \
            ASAN_OPTIONS="abort_on_error=1:detect_leaks=0:log_path=$OUT/asan.$label.$rep" \
            DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
            DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE="$TX_RATE" \
            DEVOURER_LOG_LEVEL=info \
            "$@" \
            "$TXDEMO" >"$log" 2>&1 &
        local pid=$!
        sleep "$RUN_SECS"
        # Signal the demo itself, not sudo: its handler is what breaks the TX
        # loop and runs the ordered teardown under test.
        sudo pkill "-$SIG" -x txdemo 2>/dev/null || true
        wait "$pid"; rc=$?
        local frames
        frames=$(grep -c '"ev":"tx' "$log" 2>/dev/null || echo 0)
        local asan_report=""
        asan_report=$(ls "$OUT/asan.$label.$rep".* 2>/dev/null | head -1 || true)
        if [ -n "$asan_report" ]; then
            echo "FAIL $label rep$rep: sanitizer report ($asan_report)"
            head -20 "$asan_report"
            fail=$((fail + 1))
        elif [ "$rc" -ne 0 ]; then
            echo "FAIL $label rep$rep: exit $rc (SIGSEGV=139, SIGABRT=134)"
            tail -5 "$log"
            fail=$((fail + 1))
        else
            echo "ok   $label rep$rep: clean exit, ${frames} tx events"
            pass=$((pass + 1))
        fi
        sleep 2   # let the adapter settle before the next claim
    done
}

echo "== txdemo teardown under ASan =="
echo "   build   $BUILD"
echo "   adapter $TX_VID:$TX_PID  ch$CHANNEL  $TX_RATE"
echo

# The reported crash: queue saturated at the moment of the signal.
cell gap0        DEVOURER_TX_GAP_US=0
# Multi-frame aggregation URB — a different buffer size and lifetime.
cell gap0-agg    DEVOURER_TX_GAP_US=0 DEVOURER_TX_USB_AGG=3
# The case that always exited cleanly: nothing outstanding at the signal.
cell gap2000     DEVOURER_TX_GAP_US=2000

# The wedge path: the adapter vanishes while URBs are outstanding, so the
# cancel-and-drain has to complete against a device that will never answer.
# Needs a hub with per-port power switching (uhubctl `ppps`) — an
# authorized-toggle is not a real unplug. WEDGE_HUB/WEDGE_PORT name the port
# the DUT is on (uhubctl with no args lists them).
if [ -n "${WEDGE_HUB:-}" ] && [ -n "${WEDGE_PORT:-}" ]; then
    log="$OUT/wedge.log"
    unbind
    sudo env \
        ASAN_OPTIONS="abort_on_error=1:detect_leaks=0:log_path=$OUT/asan.wedge" \
        DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE="$TX_RATE" \
        DEVOURER_LOG_LEVEL=info DEVOURER_TX_GAP_US=0 \
        "$TXDEMO" >"$log" 2>&1 &
    wedge_pid=$!
    sleep "$RUN_SECS"
    sudo uhubctl -l "$WEDGE_HUB" -p "$WEDGE_PORT" -a off >/dev/null 2>&1
    # Give the demo a bounded window to notice and exit. A quiesce that cannot
    # drain must still return (it logs tx.quiesce_timeout), so a hang here is
    # itself the failure.
    waited=0
    while kill -0 "$wedge_pid" 2>/dev/null && [ "$waited" -lt 20 ]; do
        sleep 1; waited=$((waited + 1))
    done
    if kill -0 "$wedge_pid" 2>/dev/null; then
        echo "FAIL wedge: still running ${waited}s after the adapter lost power"
        sudo pkill -x txdemo 2>/dev/null || true
        fail=$((fail + 1))
    elif ls "$OUT/asan.wedge".* >/dev/null 2>&1; then
        echo "FAIL wedge: sanitizer report"
        sudo head -20 "$OUT/asan.wedge".*
        fail=$((fail + 1))
    else
        wait "$wedge_pid" 2>/dev/null; rc=$?
        echo "ok   wedge: exited ${waited}s after power cut (rc=$rc)"
        grep -q '"ev":"tx.quiesce_timeout"' "$log" &&
            echo "     note: quiesce hit its deadline (drain never completed)"
        pass=$((pass + 1))
    fi
    sudo uhubctl -l "$WEDGE_HUB" -p "$WEDGE_PORT" -a on >/dev/null 2>&1
    sleep 3
fi

echo
echo "== $pass passed, $fail failed (logs in $OUT) =="
[ "$fail" -eq 0 ]
