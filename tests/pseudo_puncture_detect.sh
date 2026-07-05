#!/usr/bin/env bash
# pseudo_puncture_detect.sh — the DETECTION half of the pseudo-puncturing
# study (docs/pseudo-preamble-puncturing.md): locate the jammed 20 MHz slice
# of the ch36 80 MHz block. B210 AWGN jams the ch48 slice (5230-5250).
#
# Two detector modes (MODE arg):
#
#   sound — per-tone SNR via MU self-sounding (docs/beamforming-self-sounding
#           .md): sound each 20 MHz slice in turn, decode the beamformee's MU
#           report. Roles: 8822EU sounder (0bda:a81a), 8812CU MU beamformee
#           (0bda:c812), 8814AU sniffer (0bda:8813 — RX-only role, works even
#           when its TX FW-boot fails). MEASURED CAVEAT: this senses the
#           channel AT THE BEAMFORMEE; on a bench where the sounding pair sits
#           point-blank (NDP SNR ~50 dB) the jam barely dents the report and
#           the detector does NOT discriminate the slice.
#
#   probe — per-slice link probing with the victim pair itself: TX (8822BU
#           T3U) + RX (8814AU) hop 20 MHz across ch36/40/44/48 and compare
#           canonical-SA hit rates. The jammed slice is the delivery minimum.
#           This senses the victim's own channel — the quantity that matters.
#
# Usage: sudo tests/pseudo_puncture_detect.sh [sound|probe|both] [JAM_GAIN] [DUR]

set -u
cd "$(dirname "$0")/.."

MODE=${1:-probe}
JAM_GAIN=${2:-70}
DUR=${3:-12}

SND_PID=0xa81a   # 8822EU sounder
BFEE_PID=0xc812  # 8812CU MU beamformee
SNF_PID=0x8813   # 8814AU sniffer
PROBE_TX_VID=2357; PROBE_TX_PID=0x012d  # 8822BU T3U
PROBE_RX_VID=0bda; PROBE_RX_PID=0x8813  # 8814AU

BFER_MAC="57:42:75:05:d6:00"   # canonical SA = NDPA TA
BFEE_MAC="00:e0:4c:88:22:ce"   # 8812CU EFUSE MAC (see tests/bf_waterfall.sh)

JAM_FREQ=5240e6  # center of the ch48 slice (5230-5250)
JAM_RATE=20e6

LOGDIR=/tmp/devourer-pseudo-puncture-detect
mkdir -p "$LOGDIR"

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

jam_start() {
  python3 tests/sdr_interferer.py --freq "$JAM_FREQ" --rate "$JAM_RATE" \
    --tx-gain "$JAM_GAIN" >"$LOGDIR/jammer.log" 2>&1 &
  JAM_PID=$!
  sleep 3
  kill -0 "$JAM_PID" 2>/dev/null || {
    echo "FATAL: jammer died — $LOGDIR/jammer.log"; exit 2; }
}

jam_stop() {
  [ -n "$JAM_PID" ] && { kill -INT "$JAM_PID" 2>/dev/null; wait "$JAM_PID" 2>/dev/null; }
  JAM_PID=""
  sleep 1
}

# ---- sound mode -----------------------------------------------------------

sound_slice() { # <label> <channel>
  local label=$1 ch=$2
  local bfee_log="$LOGDIR/$label.bfee.log" snf_log="$LOGDIR/$label.snf.log"
  local tx_log="$LOGDIR/$label.tx.log"

  env DEVOURER_PID=$BFEE_PID DEVOURER_CHANNEL="$ch" \
      DEVOURER_BF_ARM_BFEE="$BFER_MAC" DEVOURER_BF_ARM_BFEE_MU=1 \
      timeout -s INT -k 5 $((DUR + 55)) ./build/WiFiDriverDemo >"$bfee_log" 2>&1 &
  local bfee_pid=$!
  for _ in $(seq 80); do grep -q "entering RX loop\|Listening air" "$bfee_log" && break; sleep 0.5; done

  env DEVOURER_PID=$SNF_PID DEVOURER_CHANNEL="$ch" DEVOURER_BF_DETECT_REPORT=4 \
      timeout -s INT -k 5 $((DUR + 35)) ./build/WiFiDriverDemo >"$snf_log" 2>&1 &
  local snf_pid=$!
  for _ in $(seq 60); do grep -q "Listening air\|entering RX loop" "$snf_log" && break; sleep 0.5; done

  env DEVOURER_PID=$SND_PID DEVOURER_CHANNEL="$ch" DEVOURER_TX_RATE=VHT2SS_MCS0 \
      DEVOURER_TX_NDPA_RA="$BFEE_MAC" DEVOURER_TX_NDPA=1 DEVOURER_TX_NDPA_MU=1 \
      DEVOURER_BF_ARM_SOUNDER=1 \
      timeout -s INT -k 5 "$DUR" ./build/WiFiDriverTxDemo >"$tx_log" 2>&1

  kill -INT "$bfee_pid" "$snf_pid" 2>/dev/null
  wait "$bfee_pid" "$snf_pid" 2>/dev/null
  sleep 5  # warm-FW settle before re-opening the same dongles

  local n
  n=$(grep -c "devourer-bf-report-raw" "$snf_log")
  echo "$n" >"$LOGDIR/$label.count"
  if [ "$n" -gt 0 ]; then
    grep "devourer-bf-report-raw" "$snf_log" \
      | python3 tools/bf_report_decode.py --max-frames 50 \
      >"$LOGDIR/$label.decode" 2>&1 || true
  fi
  echo "  slice ch$ch [$label]: $n reports"
}

run_sound() {
  echo "== sound detector: jam OFF (reference) =="
  for ch in 36 40 44 48; do sound_slice "off-ch$ch" "$ch"; done
  echo "== sound detector: jam ON =="
  jam_start
  for ch in 36 40 44 48; do sound_slice "on-ch$ch" "$ch"; done
  jam_stop
  echo
  echo "== report counts (off -> on); per-tone SNR in $LOGDIR/*.decode =="
  for ch in 36 40 44 48; do
    printf "  ch%-3s %5s -> %s\n" "$ch" \
      "$(cat "$LOGDIR/off-ch$ch.count")" "$(cat "$LOGDIR/on-ch$ch.count")"
  done
}

# ---- probe mode -----------------------------------------------------------

probe_slice() { # <label> <channel>
  local label=$1 ch=$2
  local rxlog="$LOGDIR/$label.rx.log" txlog="$LOGDIR/$label.tx.log"

  env DEVOURER_VID="0x$PROBE_TX_VID" DEVOURER_PID="$PROBE_TX_PID" \
      DEVOURER_CHANNEL="$ch" \
      timeout -s INT -k 5 $((DUR + 10)) ./build/WiFiDriverTxDemo >"$txlog" 2>&1 &
  local txpid=$!
  sleep 5

  env DEVOURER_VID="0x$PROBE_RX_VID" DEVOURER_PID="$PROBE_RX_PID" \
      DEVOURER_CHANNEL="$ch" \
      timeout -s INT -k 5 "$DUR" ./build/WiFiDriverDemo >"$rxlog" 2>&1
  wait "$txpid" 2>/dev/null
  sleep 5

  local hits
  hits=$(grep -c "devourer-tx-hit" "$rxlog")
  echo "$hits" >"$LOGDIR/$label.count"
  echo "  slice ch$ch [$label]: $hits hits"
}

run_probe() {
  echo "== probe detector: jam ON, scanning slices =="
  jam_start
  for ch in 36 40 44 48; do probe_slice "probe-ch$ch" "$ch"; done
  jam_stop
  echo
  local min_ch="" min_n=999999 n
  echo "== per-slice delivery =="
  for ch in 36 40 44 48; do
    n=$(cat "$LOGDIR/probe-ch$ch.count")
    printf "  ch%-3s %s hits\n" "$ch" "$n"
    if [ "$n" -lt "$min_n" ]; then min_n=$n; min_ch=$ch; fi
  done
  echo
  if [ "$min_ch" = 48 ]; then
    echo "VERDICT: probe detector PICKS the jammed slice (ch48 minimum, $min_n hits)"
  else
    echo "VERDICT: probe detector MISSED — minimum is ch$min_ch, expected ch48"
    return 1
  fi
}

echo "pseudo-puncture detect: mode=$MODE, jam ch48-slice @ ${JAM_GAIN} dB, ${DUR}s cells"
echo
rc=0
case "$MODE" in
  sound) run_sound ;;
  probe) run_probe || rc=1 ;;
  both)  run_sound; run_probe || rc=1 ;;
  *) echo "usage: $0 [sound|probe|both] [JAM_GAIN] [DUR]"; exit 2 ;;
esac
exit $rc
