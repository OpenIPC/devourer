#!/usr/bin/env bash
# Is the chip's frame-free energy telemetry actually LIVE, and does that depend
# on whether the session is receive- or transmit-oriented?
#
# The adaptive-FHSS transmitter senses the channel it is about to use by
# reading the false-alarm / CCA counters and the DIG gain index. Those are
# register reads, so they always succeed — but succeeding is not the same as
# being updated. This probe parks a known interferer on one channel and reads
# the same telemetry two ways:
#
#   RX session — rxdemo with DEVOURER_RX_ENERGY_MS (the path chanscout uses)
#   TX session — txdemo with DEVOURER_TX_SENSE quiet windows
#
# and prints both, jammed channel versus clean. If the numbers move in one
# session and are frozen in the other, the sensor is not simply insensitive —
# it is not running, and no amount of window tuning will help.
#
# Usage: sudo -v && tests/tx_sense_probe.sh
#        JAM_CHANNEL=5 CLEAN_CHANNEL=1 SECS=12 tests/tx_sense_probe.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$ROOT/tests"

VID="${VID:-0x0bda}"
RX_PID="${RX_PID:-0xc812}"
TX_PID="${TX_PID:-0xa81a}"
JAM_CHANNEL="${JAM_CHANNEL:-5}"
CLEAN_CHANNEL="${CLEAN_CHANNEL:-1}"
JAM_GAIN="${JAM_GAIN:-50}"
JAM_RATE="${JAM_RATE:-5e6}"
SECS="${SECS:-12}"
SDR_ARGS="${SDR_ARGS-}"
OUT="${OUT:-/tmp/devourer-sense-probe}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$RX_PID" || { echo "SKIP: RX $VID:$RX_PID not plugged"; exit 77; }
plugged "$VID" "$TX_PID" || { echo "SKIP: TX $VID:$TX_PID not plugged"; exit 77; }
SDR_SEL="$(python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} 2>/dev/null)" || {
    python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} >&2; exit 1; }

kill_jammer() { sudo pkill -f "sdr_interferer.py" 2>/dev/null || true; }
cleanup() {
    kill_jammer
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x rxdemo 2>/dev/null || true
    sudo pkill -x txdemo 2>/dev/null || true
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
cmake --build "$ROOT/build" -j --target rxdemo txdemo >/dev/null || exit 1
unbind "$RX_PID"; unbind "$TX_PID"
mkdir -p "$OUT"; rm -f "$OUT"/*.log

PYV="$HERE/.venv/bin/python"
[ -x "$PYV" ] || PYV="$(command -v python3)"

start_jammer() { # channel
    kill_jammer
    sudo "$PYV" "$HERE/sdr_interferer.py" ${SDR_SEL:+--args "$SDR_SEL"} \
        --channel "$1" --tx-gain "$JAM_GAIN" --rate "$JAM_RATE" \
        >>"$OUT/jammer.log" 2>&1 &
    sleep 3
}

# --- RX session: the path chanscout and the link-health monitor use ---
rx_probe() { # channel, label
    sudo env DEVOURER_VID="$VID" DEVOURER_PID="$RX_PID" \
        DEVOURER_CHANNEL="$1" DEVOURER_RX_ENERGY_MS=500 \
        "$ROOT/build/rxdemo" >"$OUT/rx_$2.log" 2>&1 &
    sleep "$SECS"
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 2
}

# --- TX session: the quiet-window path the adaptive transmitter uses ---
tx_probe() { # channel, label
    sudo env DEVOURER_VID="$VID" DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$1" DEVOURER_HOP_CHANNELS="$1" \
        DEVOURER_HOP_SLOT_MS=200 DEVOURER_HOP_SEED=c0ffee00c0ffee00 \
        DEVOURER_HOP_ADAPTIVE=1 DEVOURER_HOP_MIN_ACTIVE=1 \
        DEVOURER_TX_SENSE=1 DEVOURER_TX_SENSE_EVERY=1 \
        DEVOURER_TX_SENSE_WINDOW_US=20000 DEVOURER_TX_SENSE_NHM=1 \
        DEVOURER_TX_WITH_RX=thread \
        "$ROOT/build/txdemo" >"$OUT/tx_$2.log" 2>&1 &
    sleep "$SECS"
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sleep 2
}

# distinct values seen for a field — one value across a whole run means the
# counter never moved
spread() { # file, field
    grep -oE "\"$2\":[0-9]+" "$1" 2>/dev/null | cut -d: -f2 | sort -n |
        awk 'NR==1{m=$1} {M=$1; n++; v[$1]=1} END {
            d=0; for (k in v) d++;
            printf "min=%s max=%s distinct=%d n=%d", (n?m:"-"), (n?M:"-"), d, n }'
}

report() { # file, label
    echo "   $2"
    for f in fa_ofdm cca_ofdm igi nhm_busy; do
        printf '      %-9s %s\n' "$f" "$(spread "$1" "$f")"
    done
}

echo "== interferer on ch$JAM_CHANNEL (gain $JAM_GAIN, ${JAM_RATE} narrowband) =="
start_jammer "$JAM_CHANNEL"

echo "== RX session (rxdemo, DEVOURER_RX_ENERGY_MS) =="
rx_probe "$JAM_CHANNEL" "jam"
rx_probe "$CLEAN_CHANNEL" "clean"
report "$OUT/rx_jam.log"   "ch$JAM_CHANNEL (jammed)"
report "$OUT/rx_clean.log" "ch$CLEAN_CHANNEL (clean)"

echo "== TX session (txdemo, DEVOURER_TX_SENSE quiet windows) =="
tx_probe "$JAM_CHANNEL" "jam"
tx_probe "$CLEAN_CHANNEL" "clean"
report "$OUT/tx_jam.log"   "ch$JAM_CHANNEL (jammed)"
report "$OUT/tx_clean.log" "ch$CLEAN_CHANNEL (clean)"

kill_jammer
echo
echo "Read it this way: a counter whose distinct-value count is 1 across a"
echo "whole run never moved, so that session is not measuring anything. A"
echo "sensor that separates the two channels will show a clearly different"
echo "range on the jammed one. Logs: $OUT"
