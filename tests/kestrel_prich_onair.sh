#!/usr/bin/env bash
# Does the Kestrel receive what it should, on the channels where the halbb
# `pri_ch` argument actually changes what the baseband is programmed with?
#
# halbb_ctrl_bw_ch takes the primary CHANNEL NUMBER. From it the vendor derives
# the 0x4978[11:8] primary-sub-band index, the 2.4 GHz CCK SCO thresholds
# (indexed pri_ch-1) and the NBI spur-notch placement. Each cell counts
# canonical-SA frames delivered between the Kestrel and a reference adapter:
#
#   cck6         1M CCK on ch6, RX 20 — the SCO threshold tables, RX side
#   ofdm36       6M on ch36, RX 20     — 5 GHz control (no index involved)
#   ofdm6        6M on ch6, RX 20      — 2.4 GHz OFDM, to separate a CCK-only
#                                        fault from a whole-band one
#   ofdm36-bw40  6M on ch36, RX 40     — the sub-band index: the frame occupies
#   ofdm36-bw80  6M on ch36, RX 80       only the primary 20, so a receiver that
#                                        thinks the primary is elsewhere in the
#                                        block decodes nothing
#
# The 20 MHz OFDM cells are the controls: pri_ch reaches no table there, so they
# must not move. Run against two builds and compare.
#
#   BUILD=/path/to/before tests/kestrel_prich_onair.sh   # Kestrel receives
#   KESTREL_TX=1 RX_PID=0x012d tests/kestrel_prich_onair.sh  # Kestrel transmits
#   REF_ONLY=1   RX_PID=0xc812 tests/kestrel_prich_onair.sh  # bench control
#
# REF_ONLY takes the Kestrel out of the path entirely — worth running before
# reading anything into a zero, since a wide-PPDU cell scores zero for plenty
# of reasons that have nothing to do with the chip under test.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"

RX_VID="${RX_VID:-0x35bc}"; RX_PID="${RX_PID:-0x0101}"   # Kestrel 8852C
TX_VID="${TX_VID:-0x2357}"; TX_PID="${TX_PID:-0x0120}"   # RTL8821AU
TX_PWR="${TX_PWR-}"           # optional flat TX index/dBm; unset = efuse default
SECS="${SECS:-12}"
OUT="${OUT:-/tmp/devourer-kestrel-prich}"

for b in rxdemo txdemo; do
    [ -x "$BUILD/$b" ] || { echo "no $b in $BUILD"; exit 1; }
done
plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: Kestrel $RX_VID:$RX_PID not plugged"; exit 77; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }

mkdir -p "$OUT"

cleanup() {
    sudo pkill -x txdemo 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

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

# KESTREL_TX=1 flips the roles: the Kestrel transmits and the reference adapter
# receives, so the same channel configurations are checked from the TX side —
# where the primary sub-band decides which 20 MHz of the block a 20 MHz PPDU
# actually goes out on.
if [ -n "${KESTREL_TX:-}" ]; then
    _t_vid="$RX_VID"; _t_pid="$RX_PID"
    RX_VID="$TX_VID"; RX_PID="$TX_PID"
    TX_VID="$_t_vid"; TX_PID="$_t_pid"
fi

# $1 label, $2 channel, $3 TX rate spec, $4 RX bandwidth, $5 channel offset,
# $6 TX-side chip bandwidth. NB the rate spec's /40 only fills the descriptor
# field — the transmitter's own channel width comes from DEVOURER_HOP_BW, so a
# cell that wants a real wide PPDU has to set both.
cell() {
    local label="$1" ch="$2" rate="$3" bw="$4" off="${5:-0}" txbw="${6:-}"
    local rxlog="$OUT/$label.rx.log" txlog="$OUT/$label.tx.log"
    unbind "$RX_PID"; unbind "$TX_PID"

    sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" \
        DEVOURER_CHANNEL="$ch" DEVOURER_BW="$bw" DEVOURER_CHOFFSET="$off" \
        DEVOURER_LOG_LEVEL=warn \
        "$BUILD/rxdemo" >"$rxlog" 2>&1 &
    local rx=$!
    sleep 6   # Kestrel bring-up (tables + RFK) before the emitter starts

    sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$ch" ${rate:+DEVOURER_TX_RATE="$rate"} \
        ${txbw:+DEVOURER_HOP_BW="$txbw"} ${txbw:+DEVOURER_HOP_OFFSET="$off"} \
        ${TX_PWR:+DEVOURER_TX_PWR="$TX_PWR"} DEVOURER_TX_GAP_US=2000 \
        DEVOURER_LOG_LEVEL=warn \
        "$BUILD/txdemo" >"$txlog" 2>&1 &
    local tx=$!
    sleep "$SECS"

    sudo pkill -TERM -x txdemo 2>/dev/null || true; wait "$tx" 2>/dev/null || true
    sudo pkill -TERM -x rxdemo 2>/dev/null || true; wait "$rx" 2>/dev/null || true

    local hits sent
    hits=$(grep -c '"ev":"rx.txhit"' "$rxlog" 2>/dev/null || true)
    sent=$(grep -o '"submitted":[0-9]*' "$txlog" | tail -1 | cut -d: -f2)
    printf '%-10s ch%-4s %-12s bw%-3s  hits=%-6s tx=%s\n' \
        "$label" "$ch" "$rate" "$bw" "${hits:-0}" "${sent:-?}"
    sleep 2
}

if [ -n "${REF_ONLY:-}" ]; then
    # Neither side is the Kestrel: the reference pair alone, to establish that
    # a real 40/80 MHz PPDU is decodable on this bench at all before reading
    # anything into a Kestrel cell that scores zero.
    echo "== reference pair only, wide PPDUs (build $BUILD) =="
    cell ref-ht40-36 36 MCS7/40 40 1 40
    cell ref-vht80-36 36 VHT1SS_MCS3/80 80 1 80
elif [ -n "${KESTREL_TX:-}" ]; then
    echo "== Kestrel TX vs pri_ch programming (build $BUILD) =="
    # The width comes from the rate spec, so these put the Kestrel itself into
    # the 40/80 MHz configuration; the reference receiver follows.
    cell tx-ht20-36 36 MCS7     20
    cell tx-ht40-36 36 MCS7/40  40 1 40
    cell tx-vht80-36 36 VHT1SS_MCS3/80 80 1 80
else
    echo "== Kestrel RX vs pri_ch programming (build $BUILD, TX_PWR=${TX_PWR:-default}) =="
    cell cck6    6  1M  20
    cell ofdm6   6  ""  20
    cell ofdm36  36 ""  20
    cell ofdm36-bw40 36 "" 40 1
    cell ofdm36-bw80 36 "" 80 1
fi
