#!/bin/sh
# mt7612u_busy_bringup.sh - does an MT7612U come up in about a second when the
# channel is already busy?
#
# The stall this guards against (src/mt7612u/CLAUDE.md, "MCU replies and
# EP 4"): frames on the channel reach EP 4 during the RXDCOC calibration of a
# channel set, before any RX ring reads EP 4, and while they sit there the
# MCU's EP 5 replies are held back too. Every command from RXDCOC on then ran
# out its 3 s reply budget, so a bring-up that takes ~1 s on a quiet channel
# took 27-34 s - on some bring-ups, not all, which is why one run proves little.
#
# How often depends on the peer's load (docs/mt7612u.md, "Bring-up on a busy
# channel"): a light peer stalls a bring-up rarely or never, a peer at zero gap
# stalled every one measured. The defaults are that shape; with a lighter peer
# a passing run means little.
#
# A Realtek adapter under devourer's txdemo floods the channel with the
# canonical test frame first. The DUT is then brought up REPS times under it
# with rxdemo, each timed from launch to its RX ring starting (the last step of
# bring-up); a canonical frame arriving afterwards (rx.txhit) shows the peer
# was audible. One more bring-up before the peer starts is the control.
#
# Needs a build with -DDEVOURER_MT7612U=ON (rxdemo must carry the backend).
#
#   sudo tests/mt7612u_busy_bringup.sh
#   sudo env DUT_SYSFS=8-1 PEER_SYSFS=1-1 PEER_PID=0xa81a CH=100 \
#        tests/mt7612u_busy_bringup.sh
#   sudo env BW=40 PEER_RATE=MCS3/40/LDPC tests/mt7612u_busy_bringup.sh
#
# Exit status: 0 every loaded bring-up reached its RX ring within SLOW_MS; 1
# at least one did not; 2 INCONCLUSIVE (the rig was refused, the control did
# not come up, a bring-up exited early or would not exit, or no bring-up heard
# the peer); 3 interrupted (INT/TERM).
#
# Env: DUT_SYSFS, PEER_VID, PEER_PID, PEER_SYSFS, CH, BW, PEER_RATE, REPS, PEER_GAP_US,
#      PEER_PWR_QDB, SLOW_MS, FW_DIR, BUILD, OUT.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
cd "$ROOT" || exit 1
DUT_SYSFS="${DUT_SYSFS:-7-1}"
PEER_VID="${PEER_VID:-0x0bda}"
PEER_PID="${PEER_PID:-0xc812}"
PEER_SYSFS="${PEER_SYSFS:-5-1}"
CH="${CH:-36}"
# Width of the DUT's tune and of the peer's frames; 40 = HT40+ on CH.
BW="${BW:-20}"
# Peer TX rate (txdemo DEVOURER_TX_RATE grammar); empty = txdemo's 6M default.
PEER_RATE="${PEER_RATE-MCS3/$BW/LDPC}"
REPS="${REPS:-10}"
# Zero gap: as many frames as the peer can air (see the load note above).
PEER_GAP_US="${PEER_GAP_US:-0}"
# Near-field rigs: keep the peer well below its calibrated power.
PEER_PWR_QDB="${PEER_PWR_QDB:--40}"
# Healthy bring-ups measured ~0.9-1.3 s, stalled ones 27-34 s.
SLOW_MS="${SLOW_MS:-10000}"
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"
OUT="${OUT:-}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
for b in rxdemo txdemo; do
  [ -x "$BUILD/$b" ] || { echo "missing $BUILD/$b - build first"; exit 2; }
done
# The option defaults OFF, and an rxdemo without the backend cannot open the
# DUT - which would read as "never came up", not as a missing build.
grep -q '^DEVOURER_MT7612U:BOOL=ON' "$BUILD/CMakeCache.txt" 2>/dev/null || {
  echo "INCONCLUSIVE: $BUILD was not configured with -DDEVOURER_MT7612U=ON"; exit 2; }
# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init dut peer
sta_peer_record || { sta_lock_release; exit 2; }

CLEANED=no
# shellcheck disable=SC2317,SC2329  # reached through the traps below
cleanup() {
  trap '' INT TERM
  [ "$CLEANED" = yes ] && return 0
  CLEANED=yes
  sta_pid_kill_hard dut; dut_gone=$?
  sta_pid_kill_hard peer INT; peer_gone=$?
  if [ "$dut_gone" = 0 ]; then sta_dut_handback
  else echo "DUT still running - not re-enumerating DUT_SYSFS=$DUT_SYSFS"; fi
  if [ "$peer_gone" = 0 ]; then sta_peer_handback
  else echo "peer still running - not re-enumerating PEER_SYSFS=$PEER_SYSFS"; fi
  sta_lock_release
}
trap cleanup EXIT
trap 'cleanup; exit 3' INT TERM

sta_dut_take || exit 2

# A sysfs path is <bus>-<port path>; the demos select by bus number + port path.
DUT_BUS="${DUT_SYSFS%%-*}"; DUT_PORT="${DUT_SYSFS#*-}"
PEER_BUS="${PEER_SYSFS%%-*}"; PEER_PORT="${PEER_SYSFS#*-}"

# Milliseconds since boot from /proc/uptime (10 ms grain; no GNU date %N).
now_ms() { awk '{ printf "%d\n", $1 * 1000 }' /proc/uptime; }

# One bring-up. $1 = tag. Timed from launch to the RX ring starting (the last
# step of bring-up; the same end point loaded or not), then up to 5 s more for
# a canonical frame to show the peer is audible. Prints three words:
#   <ms to the ring | NONE (not within 60 s) | DIED (exited first) | STUCK>
#   <1 if a canonical frame arrived, else 0>  <MCU reply timeouts logged>
# STUCK = the DUT would not die; the run must stop rather than start another
# on a held adapter. Everything else this function says goes to stderr, so it
# cannot land in the caller's $(...).
bringup() {
  tag="$1"
  : > "$OUT/dut_$tag.jsonl"
  start=$(now_ms)
  env DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 DEVOURER_USB_BUS="$DUT_BUS" \
      DEVOURER_USB_PORT="$DUT_PORT" DEVOURER_CHANNEL="$CH" DEVOURER_BW="$BW" \
      DEVOURER_MT7612U_FW_DIR="$FW_DIR" DEVOURER_LOG_LEVEL=info \
      "$BUILD/rxdemo" >"$OUT/dut_$tag.jsonl" 2>"$OUT/dut_$tag.log" &
  sta_pid_record dut $!
  ms=NONE; heard=0; t=0
  while [ "$t" -lt 600 ]; do
    if grep -q 'RX transfers in flight' "$OUT/dut_$tag.log" 2>/dev/null; then
      ms=$(( $(now_ms) - start )); break
    fi
    sta_pid_live dut || { ms=DIED; break; }
    sleep 0.1; t=$((t + 1))
  done
  if [ "$ms" != NONE ] && [ "$ms" != DIED ]; then
    t=0
    while [ "$t" -lt 50 ]; do
      grep -q '"ev":"rx.txhit"' "$OUT/dut_$tag.jsonl" 2>/dev/null && { heard=1; break; }
      sleep 0.1; t=$((t + 1))
    done
  fi
  sta_pid_kill_hard dut >&2
  sta_pid_live dut && ms=STUCK
  echo "$ms $heard $(grep -c 'mcu command timed out' "$OUT/dut_$tag.log")"
}

echo "DUT  MT7612U at $DUT_SYSFS; peer $PEER_VID:$PEER_PID at $PEER_SYSFS;" \
     "ch$CH ${BW} MHz, peer ${PEER_RATE:-6M} gap ${PEER_GAP_US} us;" \
     "$REPS loaded bring-ups; slow above $SLOW_MS ms; logs in $OUT"

# Control: the same bring-up before the peer starts.
# shellcheck disable=SC2046  # three words by construction
set -- $(bringup control)
case "$1" in
  STUCK) echo "INCONCLUSIVE: the control DUT would not exit"; exit 2 ;;
  NONE|DIED) echo "INCONCLUSIVE: the DUT did not come up on a quiet channel ($1)"; exit 2 ;;
esac
echo "  control (no peer yet):   $1 ms to the RX ring, $3 MCU timeouts"

# Recorded before the launch: an INT between the two must still hand it back.
sta_peer_opened
env DEVOURER_VID="$PEER_VID" DEVOURER_PID="$PEER_PID" DEVOURER_USB_BUS="$PEER_BUS" \
    DEVOURER_USB_PORT="$PEER_PORT" DEVOURER_CHANNEL="$CH" DEVOURER_HOP_BW="$BW" \
    ${PEER_RATE:+DEVOURER_TX_RATE="$PEER_RATE"} \
    DEVOURER_TX_GAP_US="$PEER_GAP_US" DEVOURER_TX_PAYLOAD_BYTES=1400 \
    DEVOURER_TX_PWR_OFFSET_QDB="$PEER_PWR_QDB" DEVOURER_LOG_LEVEL=warn \
    "$BUILD/txdemo" >/dev/null 2>"$OUT/peer.log" &
sta_pid_record peer $!
sleep 5
sta_pid_live peer || { echo "INCONCLUSIVE: the peer exited (see $OUT/peer.log)"; exit 2; }

slow=0; heard=0; died=0
i=1
while [ "$i" -le "$REPS" ]; do
  # shellcheck disable=SC2046  # three words by construction
  set -- $(bringup "load$i")
  case "$1" in
    STUCK) echo "INCONCLUSIVE: bring-up $i would not exit"; exit 2 ;;
    DIED)  died=$((died + 1)); r="exited before its RX ring" ;;
    NONE)  slow=$((slow + 1)); r="no RX ring within 60 s" ;;
    *)     [ "$1" -gt "$SLOW_MS" ] && slow=$((slow + 1)); r="$1 ms to the RX ring" ;;
  esac
  heard=$((heard + $2))
  printf '  bring-up %2d: %s, %s MCU timeouts, peer heard: %s\n' "$i" "$r" "$3" "$2"
  sleep 3
  i=$((i + 1))
done

if [ "$died" -gt 0 ]; then
  echo "INCONCLUSIVE: $died bring-up(s) exited early (see $OUT/dut_load*.log)"
  exit 2
fi
if [ "$heard" = 0 ]; then
  echo "INCONCLUSIVE: no bring-up heard the peer - the channel was not loaded"
  exit 2
fi
if [ "$slow" -gt 0 ]; then
  echo "FAIL: $slow of $REPS bring-ups under load took over $SLOW_MS ms"
  exit 1
fi
echo "PASS: all $REPS bring-ups under load reached the RX ring within $SLOW_MS ms"
exit 0
