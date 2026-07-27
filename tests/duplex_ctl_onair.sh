#!/usr/bin/env bash
# On-air verification of the duplex demo's stdin control-opcode escape.
#
# duplex reads a <u32_le len><PSDU> stream, except that a length with the top
# bit set escapes to a control TLV — <op:u8><payload> — carrying the adaptive
# link's live knobs (SET_PWR, SET_RATE, SET_CHAN). That escape is the one place
# any stdin demo inspects the length before reading the body, and it had no
# coverage of its own: tests/adaptive_onair.sh exercises it, but adaptive_link.py
# consumes duplex's event stream on a pipe and drops everything it is not
# looking for, so nothing observes whether the TLV was read or acted on.
#
# Here the stream is scripted instead of controller-driven, duplex's events are
# captured, and a second adapter witnesses the air:
#
#   PSDUs at the base rate -> SET_RATE MCS5 -> PSDUs -> SET_PWR -> PSDUs
#
# Passing means all three of: duplex logged a stream.ctl for each TLV (it read
# them), the witness decoded a rate change at the right point (it ACTED on
# one), and the PSDU count came out whole (the record stream stayed in sync
# across the escapes — an off-by-one there would desync every later frame).
#
# Usage: sudo -v && tests/duplex_ctl_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

TX_VID="${TX_VID:-0x0bda}"; TX_PID="${TX_PID:-0x8812}"   # RTL8812AU
RX_VID="${RX_VID:-0x0bda}"; RX_PID="${RX_PID:-0xc812}"   # RTL8812CU witness
CH="${CH:-6}"
# The TLV switches DOWN to a more robust rate, never up. Switching up looks
# identical to not switching at all: the frames fly at the new rate and the
# witness simply cannot decode them, so the histogram shows one rate and the
# frame count quietly drops. Measured here at ~13 dB SNR, an MCS1 -> MCS5
# switch read as "SET_RATE never took effect" while it was working perfectly.
BASE_RATE="${BASE_RATE:-MCS1}"      # before the SET_RATE (HT, DESC_RATE 13)
CTL_RATE="${CTL_RATE:-6M}"          # after it (legacy, DESC_RATE 4)
NPSDU="${NPSDU:-500}"               # per segment; three segments
# duplex paces at --interval-ms (default 2), so an unpaced run drains in a
# couple of seconds and a witness barely overlaps it. Stretch the run instead:
# the point is to see the rate change BETWEEN segments, which needs the
# segments to be far enough apart in time to be told apart at all.
INTERVAL_MS="${INTERVAL_MS:-10}"
PSDU_BYTES="${PSDU_BYTES:-200}"
OUT="${OUT:-/tmp/devourer-duplex-ctl}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: RX $RX_VID:$RX_PID not plugged"; exit 77; }

cleanup() {
    sudo pkill -INT -x duplex 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x duplex 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# In-tree rtw88 auto-probes these dongles; temp-unbind whatever holds them.
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
cmake --build "$ROOT/build" -j --target duplex rxdemo >/dev/null || exit 1
unbind "$TX_PID"; unbind "$RX_PID"
mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/*.bin

# The scripted stream. Written by a generator rather than inline so the framing
# is in one readable place: data records carry a plain length, control records
# carry the same length word with the top bit set.
python3 - "$OUT/stream.bin" "$NPSDU" "$PSDU_BYTES" "$CTL_RATE" <<'PY'
import struct, sys
path, npsdu, nbytes, rate = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
def data(i):
    body = bytes(((i + j) & 0xFF) for j in range(nbytes))
    return struct.pack("<I", len(body)) + body
def ctl(op, payload=b""):
    body = bytes([op]) + payload
    return struct.pack("<I", 0x80000000 | len(body)) + body
SET_PWR, SET_RATE = 1, 2
with open(path, "wb") as f:
    for i in range(npsdu):                       # segment 1: base rate
        f.write(data(i))
    f.write(ctl(SET_RATE, rate.encode()))        # TLV 1
    for i in range(npsdu):                       # segment 2: switched rate
        f.write(data(i))
    f.write(ctl(SET_PWR, bytes([20])))           # TLV 2
    for i in range(npsdu):                       # segment 3
        f.write(data(i))
print(f"stream: {npsdu * 3} PSDUs of {nbytes} B + 2 control TLVs", file=sys.stderr)
PY
[ -s "$OUT/stream.bin" ] || { echo "FAIL: stream generation produced nothing"; exit 1; }

echo "== witness (rxdemo $RX_VID:$RX_PID) on ch$CH =="
# STREAM_OUT gives one rx.frame per stream-SA frame. Without it the only
# per-frame event is rx.txhit, which rxdemo deliberately rate-limits to the
# first ten and every hundredth thereafter — a biased sample that reads as
# "the rate never changed" when it simply was not looked at.
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=info "$ROOT/build/rxdemo" \
    >"$OUT/rx.log" 2>&1 &
sleep 12

echo "== duplex TX ($TX_VID:$TX_PID) on ch$CH, base rate $BASE_RATE =="
# duplex's own events ride stdout; keep them, unlike the controller-driven path.
sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$BASE_RATE" DEVOURER_LOG_LEVEL=info \
    "$ROOT/build/duplex" --interval-ms "$INTERVAL_MS" \
    <"$OUT/stream.bin" >"$OUT/tx.log" 2>&1 &
tx_pid=$!
wait "$tx_pid" 2>/dev/null || true
sleep 3
cleanup

echo "== verdicts (logs: $OUT) =="
fails=0
n_ctl=$(grep -c '"ev":"stream.ctl"' "$OUT/tx.log" 2>/dev/null)
echo "   control TLVs read: ${n_ctl:-0}"
grep -o '"ev":"stream.ctl"[^}]*}' "$OUT/tx.log" 2>/dev/null | sed 's/^/      /'
if [ "${n_ctl:-0}" -eq 2 ]; then
    echo "PASS: duplex read both control TLVs"
else
    echo "FAIL: expected 2 stream.ctl events, saw ${n_ctl:-0}"; fails=$((fails+1))
fi

# The escape must not consume a data record, nor leave one behind: duplex airs
# exactly the PSDUs it was given.
sent=$(grep -o '"ev":"stream.eof"[^}]*"tx_count":[0-9]*' "$OUT/tx.log" 2>/dev/null |
       tail -1 | sed -n 's/.*"tx_count":\([0-9]*\).*/\1/p')
want=$((NPSDU * 3))
echo "   PSDUs aired: ${sent:-0} of $want"
if [ "${sent:-0}" -eq "$want" ]; then
    echo "PASS: every PSDU aired — the record stream stayed in sync across the escapes"
else
    echo "FAIL: aired ${sent:-0} PSDUs, expected $want (a desync would lose the tail)"
    fails=$((fails+1))
fi

# ...and the witness has to see the rate actually change, or the TLV was read
# and dropped on the floor. Both rates must be decodable at the bench's link
# margin for this to mean anything — see CTL_RATE above.
rates=$(grep -F '"ev":"rx.frame"' "$OUT/rx.log" 2>/dev/null |
        sed -n 's/.*"rate":\([0-9]*\).*/\1/p' | sort -n | uniq -c |
        awk '{printf "%s(x%s) ", $2, $1}')
echo "   witnessed DESC_RATE values: ${rates:-none}"
n_rates=$(grep -F '"ev":"rx.frame"' "$OUT/rx.log" 2>/dev/null |
          sed -n 's/.*"rate":\([0-9]*\).*/\1/p' | sort -nu | wc -l)
if [ "${n_rates:-0}" -ge 2 ]; then
    echo "PASS: the witness decoded a rate change — SET_RATE was applied on air"
else
    echo "FAIL: only ${n_rates:-0} distinct rate(s) on air; SET_RATE never took effect"
    fails=$((fails+1))
fi

[ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
exit "$fails"
