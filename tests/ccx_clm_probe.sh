#!/usr/bin/env bash
# Does CLM + NHM-env see an interferer that a frame sniffer calls a free channel?
#
# devourer already scores a channel on decoded foreign airtime plus a false-alarm
# proxy. Neither notices a non-802.11 emitter: no frames are decoded, and a clean
# narrowband carrier does not necessarily generate false alarms. CLM (hardware
# busy airtime) and NHM-env (histogram mass above the receiver's OWN floor) are
# the vendor's answer, and this probe is here to find out whether they actually
# deliver it on our silicon.
#
# Four arms on ONE sensor adapter and one channel, so nothing but the emitter
# changes between them:
#
#   quiet   nothing transmitting            -- the floor. nhm_env must NOT rail.
#   sdr     non-802.11 narrowband carrier   -- frames ~0. THE cell: nhm_env up.
#   wifi    a devourer 802.11 transmitter   -- frames up, clm up.
#   txsess  the sensor itself transmitting  -- does CLM accumulate where the
#                                              FA/CCA counters are known inert?
#
# Each arm is repeated so the spread is visible before any separation is
# believed; a single probe on this bench is worth several points.
#
# Usage: sudo -v && tests/ccx_clm_probe.sh
#        CHANNEL=36 REPS=3 SECS=12 tests/ccx_clm_probe.sh
#        RX_PID=0xb812 tests/ccx_clm_probe.sh      # Jaguar2 sensor
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$ROOT/tests"

VID="${VID:-0x0bda}"
RX_PID="${RX_PID:-0xc812}"          # sensor (8812AU, Jaguar1, 11AC CCX map)
TX_PID="${TX_PID:-0xa81a}"          # the 802.11 arm's transmitter (8821AU)
CHANNEL="${CHANNEL:-6}"
JAM_GAIN="${JAM_GAIN:-50}"
JAM_RATE="${JAM_RATE:-5e6}"
SECS="${SECS:-12}"
JAM_SETTLE="${JAM_SETTLE:-8}"   # B210 acquisition can take several seconds
ARMS="${ARMS:-quiet sdr wifi txsess txsessjam}"
REPS="${REPS:-3}"
ENERGY_MS="${ENERGY_MS:-500}"
SDR_ARGS="${SDR_ARGS-}"
OUT="${OUT:-/tmp/devourer-ccx-clm}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$RX_PID" || { echo "SKIP: sensor $VID:$RX_PID not plugged"; exit 77; }
plugged "$VID" "$TX_PID" || { echo "SKIP: 802.11 TX $VID:$TX_PID not plugged"; exit 77; }
SDR_SEL="$(python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} 2>/dev/null)" || {
    python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} >&2; exit 1; }

kill_jammer() { sudo pkill -f "sdr_interferer.py" 2>/dev/null || true; }
stop_radios() {
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x rxdemo 2>/dev/null || true
    sudo pkill -x txdemo 2>/dev/null || true
}
cleanup() { kill_jammer; stop_radios; }
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

# The B210 can take several seconds to acquire, and a jammer that comes up late
# leaves an arm measuring an empty channel while the log still says it started.
# Wait for the interferer to announce its centre frequency, then give it a
# settle margin on top -- a silent arm is worse than a slow one.
start_jammer() {
    kill_jammer
    local before
    before=$(grep -c "interferer\] freq" "$OUT/jammer.log" 2>/dev/null || true)
    before=${before:-0}
    sudo "$PYV" "$HERE/sdr_interferer.py" ${SDR_SEL:+--args "$SDR_SEL"} \
        --channel "$CHANNEL" --tx-gain "$JAM_GAIN" --rate "$JAM_RATE" \
        >>"$OUT/jammer.log" 2>&1 &
    local waited=0
    while [ "$waited" -lt 25 ]; do
        local now
        now=$(grep -c "interferer\] freq" "$OUT/jammer.log" 2>/dev/null || true)
        if [ "${now:-0}" -gt "$before" ]; then
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done
    if [ "$waited" -ge 25 ]; then
        echo "   WARNING: jammer never announced a centre frequency -- this arm"
        echo "            is measuring an unjammed channel. Treat it as void."
    fi
    sleep "$JAM_SETTLE"
}

start_wifi_tx() {
    sudo env DEVOURER_VID="$VID" DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE=MCS1 \
        DEVOURER_TX_GAP_US=500 \
        "$ROOT/build/txdemo" >>"$OUT/wifi_tx.log" 2>&1 &
    sleep 3
}

# The sensor, receive-oriented: the path chanscout and the link-health monitor
# take. rx.energy now carries clm/nhm_env alongside the FA/CCA counters.
rx_sense() { # label
    sudo env DEVOURER_VID="$VID" DEVOURER_PID="$RX_PID" \
        DEVOURER_CHANNEL="$CHANNEL" DEVOURER_RX_ENERGY_MS="$ENERGY_MS" \
        "$ROOT/build/rxdemo" >"$OUT/$1.log" 2>&1 &
    sleep "$SECS"
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 2
}

# The sensor, transmit-oriented: quiet windows inside a TX session. This is the
# arm that answers whether CLM survives where FA/CCA are measured inert.
tx_sense() { # label
    sudo env DEVOURER_VID="$VID" DEVOURER_PID="$RX_PID" \
        DEVOURER_TX_GAP_US=2000 \
        DEVOURER_CHANNEL="$CHANNEL" DEVOURER_HOP_CHANNELS="$CHANNEL" \
        DEVOURER_HOP_SLOT_MS=1200 DEVOURER_HOP_SEED=c0ffee00c0ffee00 \
        DEVOURER_HOP_ADAPTIVE=1 DEVOURER_HOP_MIN_ACTIVE=1 \
        DEVOURER_TX_SENSE=1 DEVOURER_TX_SENSE_EVERY=1 \
        DEVOURER_TX_SENSE_WINDOW_US=300000 \
        DEVOURER_TX_SENSE_MAX_FRAC_PCT=60 DEVOURER_TX_SENSE_NHM=1 \
        DEVOURER_TX_WITH_RX=thread \
        "$ROOT/build/txdemo" >"$OUT/$1.log" 2>&1 &
    sleep "$SECS"
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sleep 2
}

# --- preflight: is the new field even being emitted? 15 s, not 10 minutes. ---
echo "== preflight (ch$CHANNEL, sensor $VID:$RX_PID) =="
SECS=8 rx_sense preflight
if ! grep -q '"ev":"rx.energy"' "$OUT/preflight.log"; then
    echo "ABORT: no rx.energy events at all -- the sensor never came up."
    echo "       Look at $OUT/preflight.log (bring-up is on stderr)."
    exit 1
fi
if ! grep -oE '"clm":[0-9]+' "$OUT/preflight.log" | head -1 | grep -q .; then
    echo "ABORT: rx.energy is emitting but clm is null on every read."
    echo "       The CCX window armed and the CLM ready bit never set, or this"
    echo "       generation has no CLM map. Nothing below would mean anything."
    grep -m3 '"ev":"rx.energy"' "$OUT/preflight.log"
    exit 1
fi
echo "   ok -- clm is live"

# --- arms ---
for r in $(seq 1 "$REPS"); do
    echo "== rep $r/$REPS =="

    want() { case " $ARMS " in *" $1 "*) return 0;; *) return 1;; esac; }

    if want quiet; then
        echo "   quiet"
        kill_jammer; stop_radios
        rx_sense "quiet_$r"
    fi

    if want sdr; then
        echo "   sdr (non-802.11 narrowband, gain $JAM_GAIN, $JAM_RATE)"
        start_jammer
        rx_sense "sdr_$r"
        kill_jammer
    fi

    if want wifi; then
        echo "   wifi (devourer 802.11 TX, MCS1)"
        start_wifi_tx
        rx_sense "wifi_$r"
        stop_radios
    fi

    if want txsess; then
        echo "   txsess (sensor transmitting, quiet-window read)"
        tx_sense "txsess_$r"
    fi

    if want txsessjam; then
        echo "   txsess+sdr"
        start_jammer
        tx_sense "txsessjam_$r"
        kill_jammer
    fi
done

cleanup
echo
"$PYV" "$HERE/ccx_clm_analyze.py" "$OUT" || exit 1
echo
echo "Logs: $OUT"
