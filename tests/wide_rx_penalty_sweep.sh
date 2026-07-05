#!/usr/bin/env bash
# wide_rx_penalty_sweep.sh — calibrate the WIDE-RX PENALTY: the extra SNR a
# receiver tuned at 80 MHz needs to decode a 40 MHz frame on its primary half,
# vs the same receiver retuned to 40 MHz (tests/rx80_narrow_tx_probe.sh proved
# the frames decode at all; this measures the dB cost, the constant the
# adaptive op-table needs for rows narrower than the RX tune).
#
# Method: differential delivery-vs-NOISE curves. The TX waveform is identical
# in both cells (T3U, VHT 40 MHz, fixed flat TXAGC); only the RX tune differs
# (80 vs 40). A B210 AWGN source over the signal band is stepped up in gain
# (dB-linear, ~1 dB granularity — far more usable range than the 24 dB TXAGC
# ladder, which point-blank bench margin swallows whole); the penalty is the
# horizontal shift between the two delivery curves at the 50% crossing,
# directly in dB. Everything else (TX nonlinearity, absolute path loss, B210
# calibration) is common to both curves and cancels.
#
# The RX is started FIRST and the TX window opens only after the RX logs its
# listening marker — per-bandwidth init time varies by seconds and otherwise
# aliases into the hit counts.
#
# Usage: sudo tests/wide_rx_penalty_sweep.sh [GAINS] [TXAGC] [DUR] [TX_VID:PID] [RX_VID:PID]
#   GAINS: quoted noise-gain ladder, default "66 70 74 78 82 86"

set -u
cd "$(dirname "$0")/.."

GAINS=${1:-"66 70 74 78 82 86"}
TXAGC=${2:-32}
DUR=${3:-12}
TX_SPEC=${4:-2357:012d}
RX_SPEC=${5:-0bda:8813}

TX_VID=${TX_SPEC%:*}; TX_PID=${TX_SPEC#*:}
RX_VID=${RX_SPEC%:*}; RX_PID=${RX_SPEC#*:}

RATE=VHT1SS_MCS7/40
NOISE_FREQ=5190e6           # center of the primary 40 (5170-5210)
NOISE_RATE=20e6

LOGDIR=/tmp/devourer-wide-rx-penalty
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

JAM_PID=""
cleanup() {
  [ -n "$JAM_PID" ] && kill -INT "$JAM_PID" 2>/dev/null
  pkill -INT -x WiFiDriverTxDem 2>/dev/null
  pkill -INT -x WiFiDriverDemo 2>/dev/null
  pkill -INT -f sdr_interferer 2>/dev/null
  sleep 1
  pkill -KILL -x WiFiDriverTxDem 2>/dev/null
  pkill -KILL -x WiFiDriverDemo 2>/dev/null
  pkill -KILL -f sdr_interferer 2>/dev/null
}
trap cleanup EXIT INT TERM

noise_start() { # <gain>
  python3 tests/sdr_interferer.py --freq "$NOISE_FREQ" --rate "$NOISE_RATE" \
    --tx-gain "$1" >"$LOGDIR/noise-$1.log" 2>&1 &
  JAM_PID=$!
  sleep 3
  kill -0 "$JAM_PID" 2>/dev/null || {
    echo "FATAL: noise source died — $LOGDIR/noise-$1.log"; exit 2; }
}

noise_stop() {
  [ -n "$JAM_PID" ] && { kill -INT "$JAM_PID" 2>/dev/null; wait "$JAM_PID" 2>/dev/null; }
  JAM_PID=""
}

# run_cell <gain> <rx_bw>: RX up first, TX window of exactly DUR, then stop.
run_cell() {
  local gain=$1 rx_bw=$2
  local tag="g$gain-rx$rx_bw"
  local rxlog="$LOGDIR/rx-$tag.log" txlog="$LOGDIR/tx-$tag.log"

  env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL=36 \
      DEVOURER_BW="$rx_bw" \
      timeout -s INT -k 5 $((DUR + 60)) ./build/WiFiDriverDemo >"$rxlog" 2>&1 &
  local rxpid=$!
  for _ in $(seq 60); do
    grep -q "Listening air" "$rxlog" && break
    sleep 0.5
  done
  grep -q "Listening air" "$rxlog" || {
    echo "  gain=$gain rx@$rx_bw : RX never came up — see $rxlog"
    kill -INT "$rxpid" 2>/dev/null; wait "$rxpid" 2>/dev/null
    echo 0 >"$LOGDIR/$tag.count"; sleep 5; return
  }

  # A back-to-back re-open of the T3U can fail DLFW while its firmware is
  # still winding down (std::runtime_error -> abort); retry the TX once.
  local attempt
  for attempt in 1 2; do
    env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL=36 \
        DEVOURER_HOP_BW=40 DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PWR="$TXAGC" \
        timeout -s INT -k 5 $((DUR + 12)) ./build/WiFiDriverTxDemo >"$txlog" 2>&1 \
      && break
    grep -q "DLFW failed" "$txlog" || break
    echo "    (TX DLFW warm-failure — retrying cell TX)"
    sleep 8
  done
  kill -INT "$rxpid" 2>/dev/null
  wait "$rxpid" 2>/dev/null
  sleep 5  # warm-FW settle before the same dongles re-open

  local hits
  hits=$(grep -c "devourer-tx-hit" "$rxlog")
  echo "$hits" >"$LOGDIR/$tag.count"
  printf "  noise=%-3s rx@%-2s : %s hits\n" "$gain" "$rx_bw" "$hits"
}

echo "wide-RX penalty sweep: TX $TX_SPEC ($RATE, txagc=$TXAGC) -> RX $RX_SPEC, ch36"
echo "noise ladder (dB): $GAINS at ${NOISE_FREQ}/${NOISE_RATE}"
echo

for g in $GAINS; do
  noise_start "$g"
  run_cell "$g" 40
  run_cell "$g" 80
  noise_stop
done

echo
echo "== delivery vs noise gain =="
printf "%8s %10s %10s\n" "noise" "rx@40" "rx@80"
for g in $GAINS; do
  printf "%8s %10s %10s\n" "$g" \
    "$(cat "$LOGDIR/g$g-rx40.count")" "$(cat "$LOGDIR/g$g-rx80.count")"
done

# 50%-crossing per curve (linear interpolation in noise-dB), penalty =
# crossing(rx@40) - crossing(rx@80): the narrower-tuned RX should survive MORE
# noise, so a positive value is the wide-RX cost, already in dB.
python3 - "$LOGDIR" $GAINS <<'EOF'
import sys
logdir, ladder = sys.argv[1], [int(x) for x in sys.argv[2:]]
def curve(bw):
    return [(g, int(open(f"{logdir}/g{g}-rx{bw}.count").read())) for g in ladder]
c40, c80 = curve(40), curve(80)
ref = max(h for _, h in c40 + c80)
if ref < 20:
    print(f"\nVERDICT: bench broken (max {ref} hits)"); sys.exit(1)
half = ref / 2.0
def crossing(c):
    # walk low->high noise; find the first segment straddling half delivery
    for (g1, h1), (g2, h2) in zip(c, c[1:]):
        if h1 >= half > h2:
            return g1 + (h1 - half) * (g2 - g1) / (h1 - h2)
    return None
x40, x80 = crossing(c40), crossing(c80)
print()
if x40 is None or x80 is None:
    print("VERDICT: cliff not bracketed "
          f"(50% crossing rx@40={x40}, rx@80={x80}) — adjust the GAINS ladder")
    sys.exit(1)
print(f"50% crossing: rx@40 at noise {x40:.1f} dB, rx@80 at noise {x80:.1f} dB")
print(f"WIDE-RX PENALTY = {x40 - x80:.1f} dB")
EOF
