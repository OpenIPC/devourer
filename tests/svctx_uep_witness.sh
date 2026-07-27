#!/usr/bin/env bash
# On-air verification of svctx's per-layer rate ladder, witnessed by a devourer
# receiver instead of a kernel monitor.
#
# tests/svc_uep_onair.sh does the same job with an 8814AU in kernel monitor
# mode. This variant uses rxdemo as the witness — rx.frame already carries the
# decoded DESC_RATE, which is the only quantity the histogram needs — so the
# test runs on any two adapters and needs no kernel driver at all.
#
# Synthetic HEVC NALs (tests/gen_svc_nals.py) carry controlled temporal_ids;
# svctx maps each to its ladder rung (default CRIT=MCS0, T0=MCS1, T1=MCS4,
# T2=MCS7) and injects it. Passing means the witness decoded MORE THAN ONE
# distinct rate — i.e. the ladder reached the air rather than every layer
# flying at whatever the radio defaulted to.
#
# It doubles as the scope test for the shared radiotap path: svctx sets its
# rate per frame through build_stream_radiotap, the same call the duplex demo's
# SET_RATE uses. If the ladder flies here, that path works and a demo whose
# rate does not change has a fault of its own.
#
# Usage: sudo -v && tests/svctx_uep_witness.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

TX_VID="${TX_VID:-0x0bda}"; TX_PID="${TX_PID:-0x8812}"   # RTL8812AU injector
RX_VID="${RX_VID:-0x0bda}"; RX_PID="${RX_PID:-0xc812}"   # RTL8812CU witness
CH="${CH:-6}"
GOPS="${GOPS:-40}"
GAP_US="${GAP_US:-4000}"          # svctx inter-fragment gap; spread the run out
SECS="${SECS:-30}"
OUT="${OUT:-/tmp/devourer-svctx-uep}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: RX $RX_VID:$RX_PID not plugged"; exit 77; }

cleanup() {
    sudo pkill -INT -x svctx 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x svctx 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

unbind() {
    local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "${pid#0x}" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

echo "== build =="
cmake --build "$ROOT/build" -j --target svctx rxdemo >/dev/null || exit 1
unbind "$TX_PID"; unbind "$RX_PID"
mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/*.bin

python3 "$ROOT/tests/gen_svc_nals.py" "$GOPS" >"$OUT/nals.bin" 2>"$OUT/gen.log"
[ -s "$OUT/nals.bin" ] || { echo "FAIL: NAL generation produced nothing"; cat "$OUT/gen.log"; exit 1; }
echo "   $(wc -c <"$OUT/nals.bin") bytes of synthetic NALs"

echo "== witness (rxdemo $RX_VID:$RX_PID) on ch$CH =="
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=info "$ROOT/build/rxdemo" \
    >"$OUT/rx.log" 2>&1 &
sleep 12

echo "== svctx ($TX_VID:$TX_PID) on ch$CH, default UEP ladder =="
sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_LOG_LEVEL=info \
    "$ROOT/build/svctx" --gap-us "$GAP_US" <"$OUT/nals.bin" \
    >"$OUT/tx.log" 2>&1 &
sleep "$SECS"
cleanup

echo "== verdicts (logs: $OUT) =="
fails=0
grep -E "SVC UEP policy|-> MCS|NALs," "$OUT/tx.log" 2>/dev/null | head -6 | sed 's/^/   /'

frames=$(grep -c '"ev":"rx.frame"' "$OUT/rx.log" 2>/dev/null)
echo "   witnessed frames: ${frames:-0}"
if [ "${frames:-0}" -lt 50 ]; then
    echo "FAIL: only ${frames:-0} frames witnessed — nothing to judge the ladder on"
    fails=$((fails+1))
fi

rates=$(grep -F '"ev":"rx.frame"' "$OUT/rx.log" 2>/dev/null |
        sed -n 's/.*"rate":\([0-9]*\).*/\1/p' | sort -n | uniq -c |
        awk '{printf "%s(x%s) ", $2, $1}')
echo "   DESC_RATE histogram: ${rates:-none}"
echo "   (DESC_RATE 12+N = HT MCS N; the default ladder spans MCS0/1/4/7)"
n_rates=$(grep -F '"ev":"rx.frame"' "$OUT/rx.log" 2>/dev/null |
          sed -n 's/.*"rate":\([0-9]*\).*/\1/p' | sort -nu | wc -l)
if [ "${n_rates:-0}" -ge 2 ]; then
    echo "PASS: $n_rates distinct rates on air — the per-frame ladder reached the radio"
else
    echo "FAIL: ${n_rates:-0} distinct rate(s) — every layer flew at one rate, so the"
    echo "      per-frame radiotap rate is not reaching the air on this transmitter"
    fails=$((fails+1))
fi

[ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
exit "$fails"
