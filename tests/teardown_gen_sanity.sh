#!/usr/bin/env bash
# Does every plugged generation still init and tear down cleanly?
#
# The TX-quiesce call the Jaguar1 teardown needs is made by every device's
# destructor. On the generations whose send path is synchronous it should be
# inert — this run is what says "inert", not "assumed inert": each adapter gets
# a short RX session and a short TX session under ASan, killed with SIGTERM,
# and both must exit 0 with no sanitizer report.
#
# Build:  cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
#               -DDEVOURER_SANITIZE=address && cmake --build build-asan -j
# Run:    sudo -v && tests/teardown_gen_sanity.sh
#
# DUTS overrides the adapter list: "label:vid:pid,label:vid:pid".
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build-asan}"
CHANNEL="${CHANNEL:-6}"
RUN_SECS="${RUN_SECS:-6}"
OUT="${OUT:-/tmp/devourer-gen-sanity}"

# One per generation; skipped when not plugged.
DUTS="${DUTS:-J1-8821AU:0x2357:0x0120,\
J2-8822BU:0x2357:0x012d,\
J3-8812CU:0x0bda:0xc812,\
J3-8812EU:0x0bda:0xa81a,\
KESTREL:0x35bc:0x0101}"

mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/asan.* 2>/dev/null || true

cleanup() {
    sudo pkill -x txdemo 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# rtw88/rtw89 claim every Realtek dongle at enumeration; unbind the interface
# (a module removal doesn't survive the next re-enumeration).
unbind() {
    local want="${1#0x}" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "$want" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] || continue
            sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

pass=0; fail=0; skip=0

run_one() {
    local label="$1" vid="$2" pid="$3" demo="$4"
    local log="$OUT/${label}-${demo}.log" rc
    unbind "$pid"
    sudo env \
        ASAN_OPTIONS="abort_on_error=1:detect_leaks=0:log_path=$OUT/asan.$label.$demo" \
        DEVOURER_VID="$vid" DEVOURER_PID="$pid" DEVOURER_CHANNEL="$CHANNEL" \
        DEVOURER_LOG_LEVEL=info \
        "$BUILD/$demo" >"$log" 2>&1 &
    local bg=$!
    sleep "$RUN_SECS"
    sudo pkill -TERM -x "$demo" 2>/dev/null || true
    wait "$bg"; rc=$?
    if ls "$OUT/asan.$label.$demo".* >/dev/null 2>&1; then
        echo "FAIL $label $demo: sanitizer report"
        sudo head -12 "$OUT/asan.$label.$demo".*
        fail=$((fail + 1))
    elif [ "$rc" -ne 0 ]; then
        echo "FAIL $label $demo: exit $rc"
        tail -4 "$log"
        fail=$((fail + 1))
    else
        echo "ok   $label $demo: clean exit"
        pass=$((pass + 1))
    fi
    sleep 2
}

echo "== per-generation init/teardown under ASan (ch$CHANNEL) =="
IFS=',' read -r -a duts <<< "$DUTS"
for entry in "${duts[@]}"; do
    IFS=':' read -r label vid pid <<< "$entry"
    if ! lsusb -d "$(printf '%04x:%04x' "$vid" "$pid")" >/dev/null 2>&1; then
        echo "skip $label ($vid:$pid not plugged)"
        skip=$((skip + 1)); continue
    fi
    run_one "$label" "$vid" "$pid" rxdemo
    run_one "$label" "$vid" "$pid" txdemo
done

echo
echo "== $pass passed, $fail failed, $skip skipped (logs in $OUT) =="
[ "$fail" -eq 0 ]
