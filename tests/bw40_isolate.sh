#!/usr/bin/env bash
# 40 MHz delivers nothing. Which side is broken — our transmitter, or our
# receiver?
#
# A devourer->devourer 40 MHz link delivers zero even on a receiver qualified
# flat to MCS7 at 20 MHz, and even at the most robust 40 MHz rate. That says the
# link is broken; it does not say which end. Both legs below hold one end at the
# VENDOR kernel driver, which is known-good, so each leg tests exactly one
# devourer path:
#
#   TX leg: devourer transmits 40 MHz  -> vendor driver receives (HT40 monitor)
#   RX leg: vendor driver transmits 40 MHz -> devourer receives (DEVOURER_BW=40)
#
# A 20 MHz control cell runs in each leg with the same instruments, so "the leg
# is set up correctly" is proven rather than assumed — an HT40 monitor that is
# deaf to everything would otherwise read as a devourer failure.
#
# Note a 20 MHz receiver genuinely cannot decode an HT40 data field (only the
# duplicated legacy preamble spans both halves), so the monitor really must be
# in HT40 with a matching secondary-channel offset. Getting that wrong is the
# easiest way to fake this bug.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;0bda:b812=4-2.3,3" \
#        tests/bw40_isolate.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

# devourer side (8812AU) and the vendor-driver side (CF-924AC 8822BU, external
# antennas — the qualified ground station).
DEV_VID=${DEV_VID:-0x0bda} DEV_PID=${DEV_PID:-0x8812}
VEN_VID=${VEN_VID:-0bda}   VEN_PID=${VEN_PID:-b812}
VEN_REF=${VEN_REF:-rtl88x2bu}
VEN_KO=${VEN_KO:-88x2bu_ohd.ko}
VEN_MOD=${VEN_MOD:-88x2bu_ohd}
INTREE=${INTREE:-rtw88_8822bu}
CH=${CH:-6}
HT=${HT:-HT40+}          # secondary above primary; devourer offset 1
DEV_OFFSET=${DEV_OFFSET:-1}
SECS=${SECS:-6}
PAYLOAD=${PAYLOAD:-1400}
CANON=57:42:75:05:d6:00
OUT=${OUT:-/tmp/devourer-bw40-isolate}
mkdir -p "$OUT"

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  sudo pkill -x txdemo 2>/dev/null
  sudo pkill -f "tcpdump -i" 2>/dev/null
  sudo pkill -f kernel_tx_inject 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cold_cycle() { # $1=vid $2=pid (bare hex)
  local map=${REGRESS_VBUS_MAP:-} entry hubport
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$1:$2=" | head -1) || return 0
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

build_vendor() {
  local dir="$ROOT/reference/$VEN_REF"
  [ -f "$dir/$VEN_KO" ] && return 0
  echo "  building $VEN_REF ..."
  make -C "$dir" -j"$(nproc)" > "$OUT/vendor_build.log" 2>&1 || {
    tail -12 "$OUT/vendor_build.log" >&2; return 1; }
  [ -f "$dir/$VEN_KO" ]
}

vendor_up() { # load the vendor driver on the vendor-side adapter, monitor mode
  local width="$1"        # 20 | 40
  sudo modprobe -r "$INTREE" 2>/dev/null
  cold_cycle "$VEN_VID" "$VEN_PID"
  sudo modprobe -r "$INTREE" 2>/dev/null
  sudo insmod "$ROOT/reference/$VEN_REF/$VEN_KO" 2>/dev/null || true
  local u prev="" cur="" iface=""
  for d in /sys/bus/usb/devices/*/; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$VEN_VID" ] || continue
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "$VEN_PID" ] || continue
    u=$(basename "$d"); break
  done
  [ -n "${u:-}" ] || return 1
  for _ in $(seq 30); do
    cur=$(basename "$(ls -d /sys/bus/usb/devices/$u/*/net/* 2>/dev/null | head -1)" 2>/dev/null)
    [ -n "$cur" ] && [ "$cur" = "$prev" ] && [ -e "/sys/class/net/$cur" ] && { iface="$cur"; break; }
    prev="$cur"; sleep 1
  done
  [ -n "$iface" ] || return 1
  sudo ip link set "$iface" down
  sudo iw dev "$iface" set type monitor
  sudo ip link set "$iface" up
  if [ "$width" = 40 ]; then
    sudo iw dev "$iface" set channel "$CH" "$HT" 2>/dev/null || return 1
  else
    sudo iw dev "$iface" set channel "$CH" 2>/dev/null || return 1
  fi
  sleep 1
  printf '%s' "$iface"
}

vendor_down() { sudo rmmod "$VEN_MOD" 2>/dev/null; sleep 1; }

devourer_tx() { # $1=bw $2=rate -> frames submitted
  local bw="$1" rate="$2" e=()
  [ "$bw" = 40 ] && e=(DEVOURER_HOP_BW=40 DEVOURER_HOP_OFFSET="$DEV_OFFSET")
  local log="$OUT/tx_${bw}_${rate//\//-}.jsonl"
  sudo env DEVOURER_VID="$DEV_VID" DEVOURER_PID="$DEV_PID" \
    DEVOURER_CHANNEL="$CH" "${e[@]}" DEVOURER_TX_RATE="$rate" \
    DEVOURER_TX_PAYLOAD_BYTES="$PAYLOAD" DEVOURER_TX_GAP_US=2000 \
    DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 25)) "$TXDEMO" > "$log" 2>/dev/null &
  local p=$!
  sleep $((SECS + 8))
  kill -TERM "$p" 2>/dev/null; wait "$p" 2>/dev/null
  grep -o '"submitted":[0-9]*' "$log" | tail -1 | cut -d: -f2
}

echo "40 MHz isolation — one end is always the vendor kernel driver"
echo "  devourer side $DEV_VID:$DEV_PID   vendor side $VEN_VID:$VEN_PID   ch$CH $HT"
echo
build_vendor || exit 1

# ---- TX leg: devourer transmits, vendor receives ---------------------------
echo "== TX leg: devourer transmits -> vendor driver receives"
# Cells are "<monitor width>:<devourer TX rate>". The third is the control that
# matters: an HT40 monitor hearing a 20 MHz frame on its primary. Without it a
# monitor whose HT40 mode is simply deaf reads as a devourer TX failure — and
# the 20-monitor/20-TX cell does not test the HT40 monitor at all.
for cell in "20:MCS0/20" "40:MCS0/40" "40:MCS0/20"; do
  IFS=':' read -r bw rate <<<"$cell"
  IFACE=$(vendor_up "$bw") || { echo "  mon$bw: vendor monitor setup FAILED — leg invalid" >&2; continue; }
  tx_bw=$([ "${rate#*/}" = "40" ] && echo 40 || echo 20)
  cap="$OUT/txleg_mon${bw}_tx${tx_bw}.txt"
  sudo timeout $((SECS + 20)) tcpdump -i "$IFACE" -nn -e "wlan addr2 $CANON" > "$cap" 2>/dev/null &
  sleep 1
  sent=$(devourer_tx "$tx_bw" "$rate")
  sleep 2
  n=$(grep -c "$CANON" "$cap" 2>/dev/null || echo 0)
  printf '  monitor=%-4s TX=%-9s sent=%-7s vendor decoded=%-7s\n' \
      "${bw}MHz" "$rate" "${sent:-0}" "$n"
  vendor_down
done

# ---- RX leg: vendor transmits, devourer receives ---------------------------
echo
echo "== RX leg: vendor driver transmits -> devourer receives"
rx_ctrl=""
for bw in 20 40; do
  IFACE=$(vendor_up "$bw") || { echo "  bw$bw: vendor monitor setup FAILED — leg invalid" >&2; continue; }
  # The devourer receiver gets a cold start per cell. Without it a receiver left
  # over from the previous cell can read 0 and void the leg — which happened,
  # and briefly looked like a second bug.
  cold_cycle "${DEV_VID#0x}" "${DEV_PID#0x}"
  e=(); [ "$bw" = 40 ] && e=(DEVOURER_BW=40 DEVOURER_CHOFFSET="$DEV_OFFSET")
  log="$OUT/rxleg_$bw.jsonl"
  sudo env DEVOURER_VID="$DEV_VID" DEVOURER_PID="$DEV_PID" DEVOURER_CHANNEL="$CH" \
    "${e[@]}" DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 \
    DEVOURER_LOG_LEVEL=warn "$RXDEMO" > "$log" 2>/dev/null &
  RXPID=$!
  sleep 9
  sent=$(sudo timeout $((SECS + 8)) python3 "$HERE/kernel_tx_inject.py" \
         "$IFACE" 0 "$PAYLOAD" "$SECS" 2>/dev/null |
         grep -oE "injected [0-9]+" | awk '{print $2}')
  sleep 1
  kill "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; RXPID=""
  got=$(grep -o '"frames":[0-9]*' "$log" | awk -F: '{s+=$2} END{print s+0}')
  amb=$(grep -c '"ev":"rx.pkt"' "$log" 2>/dev/null || echo 0)
  printf '  %-8s %-9s injected=%-7s devourer decoded=%-7s (ambient rx.pkt=%s)\n' \
      "bw$bw" "MCS0" "${sent:-0}" "$got" "$amb"
  [ "$bw" = 20 ] && rx_ctrl="$got"
  vendor_down
done
if [ "${rx_ctrl:-0}" -lt 100 ] 2>/dev/null; then
  echo "  !! RX leg VOID: its own 20 MHz control decoded ${rx_ctrl:-0}. The"
  echo "     receiver was not working at all, so the 40 MHz cell says nothing."
fi

echo
echo "read the TX leg's third cell FIRST: monitor=40MHz TX=MCS0/20."
echo "  decoded ~0 there  -> the HT40 MONITOR is deaf; the whole TX leg is void."
echo "  decoded fine there -> the monitor works, so a 0 at TX=MCS0/40 is a real"
echo "                        devourer 40 MHz TX failure."
echo "RX leg: the injector sends a 20 MHz frame on the primary channel, so a 0"
echo "  at DEVOURER_BW=40 means our 40 MHz RX cannot hear even a 20 MHz frame."
echo "logs: $OUT"
