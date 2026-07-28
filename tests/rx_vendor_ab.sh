#!/usr/bin/env bash
# Is a receiver that cannot decode high-order rates broken in OUR code, or is the
# adapter itself degraded? Same dongle, devourer vs the vendor kernel driver.
#
# Context: an 8822BU ground station decodes MCS1 at 98% and MCS7 at 0.4% from a
# transmitter that an 8814AU ground decodes at 83% and an AR9271 sniffer at 62%.
# So the transmitter is fine and this receiver is the broken end. That still
# leaves two very different answers — a devourer RX bug, or a dying adapter —
# and only the vendor driver on the same hardware separates them.
#
# The transmitter is held constant (devourer, fixed rate, fixed channel) and only
# the RECEIVER's driver changes. Frames are counted by source address so both
# sides count the same thing.
#
#   sudo REGRESS_VBUS_MAP="2357:012d=10,2;0bda:8812=3-2.3.4,3" tests/rx_vendor_ab.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

TX_VID=${TX_VID:-0x0bda} TX_PID=${TX_PID:-0x8812}
RX_VID=${RX_VID:-2357}   RX_PID=${RX_PID:-012d}     # receiver under test, bare hex
RX_REF=${RX_REF:-rtl88x2bu}
RX_KO=${RX_KO:-88x2bu_ohd.ko}
RX_MOD=${RX_MOD:-88x2bu_ohd}
INTREE=${INTREE:-rtw88_8822bu}
CH=${CH:-6}
RATES=${RATES:-"MCS7/20 MCS1/20"}
SECS=${SECS:-6}
CANON=57:42:75:05:d6:00
OUT=${OUT:-/tmp/devourer-rx-vendor-ab}
mkdir -p "$OUT"

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  sudo pkill -x txdemo 2>/dev/null
  sudo pkill -f "tcpdump -i" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cold_cycle() { # $1=vid $2=pid, bare hex
  local map=${REGRESS_VBUS_MAP:-} entry hubport
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$1:$2=" | head -1) || return 1
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

rx_usb_node() {
  for d in /sys/bus/usb/devices/*/; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$RX_VID" ] || continue
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "$RX_PID" ] || continue
    basename "$d"; return 0
  done
  return 1
}

# Transmit $1 for SECS while the caller is already listening. Echoes frames sent.
transmit() {
  local rate="$1"
  local log="$OUT/tx_${rate//\//-}.jsonl"
  # Explicit sleep-then-SIGTERM rather than letting `timeout` do the killing:
  # the final tx.stats (the frames-sent denominator) is emitted on a clean
  # SIGTERM, and a bring-up that runs long enough for `timeout` to fire first
  # loses it — which silently turns a measured cell into a divide-by-zero.
  sudo env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$rate" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_TEARDOWN_POWER_DOWN=0 \
    DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 25)) "$TXDEMO" > "$log" 2>/dev/null &
  local p=$!
  sleep $((SECS + 8))
  kill -TERM "$p" 2>/dev/null
  wait "$p" 2>/dev/null
  grep -o '"submitted":[0-9]*' "$log" | tail -1 | cut -d: -f2
}

# ---- leg A: receiver under devourer -----------------------------------------
devourer_leg() {
  local rate="$1" off0 off1 sent d
  DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
  "$RXDEMO" > "$OUT/dev_rx.jsonl" 2>/dev/null &
  RXPID=$!
  sleep 9
  off0=$(stat -c%s "$OUT/dev_rx.jsonl")
  sent=$(transmit "$rate")
  sleep 1
  off1=$(stat -c%s "$OUT/dev_rx.jsonl")
  kill "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; RXPID=""
  d=$(python3 - "$OUT/dev_rx.jsonl" "$off0" "$off1" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event
f = open(sys.argv[1], "rb"); f.seek(int(sys.argv[2]))
blob = f.read(int(sys.argv[3]) - int(sys.argv[2])).decode(errors="replace")
print(sum(int(ev.get("frames") or 0)
          for ev in (parse_event(l, "rx.energy") for l in blob.splitlines()) if ev))
EOF
)
  printf '%s %s' "${d:-0}" "${sent:-0}"
}

# ---- leg B: same dongle under the vendor kernel driver ----------------------
build_vendor() {
  local dir="$ROOT/reference/$RX_REF"
  [ -f "$dir/$RX_KO" ] && return 0
  echo "  building $RX_REF (first run, a few minutes) ..."
  make -C "$dir" -j"$(nproc)" > "$OUT/vendor_build.log" 2>&1 || {
    echo "  !! vendor build failed — see $OUT/vendor_build.log" >&2
    tail -12 "$OUT/vendor_build.log" >&2; return 1; }
  [ -f "$dir/$RX_KO" ]
}

vendor_iface() {
  local u prev="" cur=""
  u=$(rx_usb_node) || return 1
  for _ in $(seq 30); do
    cur=$(basename "$(ls -d /sys/bus/usb/devices/$u/*/net/* 2>/dev/null | head -1)" 2>/dev/null)
    [ -n "$cur" ] && [ "$cur" = "$prev" ] && [ -e "/sys/class/net/$cur" ] && {
      printf '%s' "$cur"; return 0; }
    prev="$cur"; sleep 1
  done
  return 1
}

vendor_leg() {
  local rate="$1"
  local iface sent n
  local cap="$OUT/vendor_${rate//\//-}.txt"
  iface=$(vendor_iface) || { echo "0 0"; return; }
  sudo ip link set "$iface" down
  sudo iw dev "$iface" set type monitor
  sudo ip link set "$iface" up
  sudo iw dev "$iface" set channel "$CH"
  sleep 1
  sudo timeout $((SECS + 14)) tcpdump -i "$iface" -nn -e "wlan addr2 $CANON" \
      > "$cap" 2>/dev/null &
  sleep 1
  sent=$(transmit "$rate")
  sleep 2
  n=$(grep -c "$CANON" "$cap" 2>/dev/null || echo 0)
  printf '%s %s' "${n:-0}" "${sent:-0}"
}

echo "receiver A/B — TX held constant (devourer 8812AU), only the RX driver changes"
echo

build_vendor || exit 1
printf '  %-10s %-22s %-22s\n' "rate" "devourer RX" "vendor RX ($RX_MOD)"
for RATE in $RATES; do
  # devourer leg: kernel drivers off the dongle, chip cold.
  sudo rmmod "$RX_MOD" 2>/dev/null
  sudo modprobe -r "$INTREE" 2>/dev/null
  cold_cycle "$RX_VID" "$RX_PID"
  sudo modprobe -r "$INTREE" 2>/dev/null
  read -r d_rx d_sent <<<"$(devourer_leg "$RATE")"

  # vendor leg: same dongle, cold again so neither leg inherits the other.
  sudo modprobe -r "$INTREE" 2>/dev/null
  cold_cycle "$RX_VID" "$RX_PID"
  sudo modprobe -r "$INTREE" 2>/dev/null
  sudo insmod "$ROOT/reference/$RX_REF/$RX_KO" 2>/dev/null || true
  read -r v_rx v_sent <<<"$(vendor_leg "$RATE")"
  sudo rmmod "$RX_MOD" 2>/dev/null

  d_pct=$(python3 -c "print('%.1f%%' % (100*$d_rx/$d_sent) if $d_sent else 'n/a')")
  v_pct=$(python3 -c "print('%.1f%%' % (100*$v_rx/$v_sent) if $v_sent else 'n/a')")
  printf '  %-10s %-22s %-22s\n' "$RATE" \
      "$d_rx/$d_sent = $d_pct" "$v_rx/$v_sent = $v_pct"
done

echo
echo "vendor decodes the high rate and devourer does not => devourer RX bug."
echo "neither decodes it                                 => the adapter is degraded."
