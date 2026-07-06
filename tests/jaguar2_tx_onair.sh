#!/usr/bin/env bash
# jaguar2_tx_onair.sh — validate RTL8822BU (Archer T3U) on-air TX by sniffing
# devourer's canonical beacon (SA 57:42:75:05:d6:00) on a separate monitor
# adapter (host 8812au, a DIFFERENT chip — not the port target).
set -uo pipefail

CH=${1:-9}
RATE=${2:-MCS1}
# NB: use an 8822cu (wlp4s0u2u4) NOT an 8812au sniffer for the T3U — the 8812au
# monitor front-end cannot decode the near-field (~-8dBm) T3U signal and reads 0
# frames from ANY driver (devourer, kernel rtw88, vendor rtl88x2bu), a harness
# false-negative. The 8822cu decodes ~95% of frames.
SNIFF_IF=${SNIFF_IF:-wlp4s0u2u4}  # host 8822cu in monitor mode
CANON_SA=57:42:75:05:d6:00
DUR=${DUR:-10}

cleanup() {
  sudo pkill -f "tcpdump -i $SNIFF_IF" 2>/dev/null
  sudo pkill -x txdemo 2>/dev/null
}
trap cleanup EXIT

echo "[tx] recovering T3U (uhubctl power-cycle) FIRST — so the sniffer isn't left"
echo "     active on the near-field strong signal for ~15s before TX (an 8822cu/"
echo "     8812au monitor front-end degrades on a close strong TX over time)."
sudo modprobe -r rtw88_8822bu 2>/dev/null
sudo /usr/local/bin/uhubctl -l 4-2.3 -p 3 -a cycle >/dev/null 2>&1
sleep 6
sudo modprobe -r rtw88_8822bu 2>/dev/null
sleep 1

echo "[tx] freshly resetting sniffer $SNIFF_IF -> monitor ch$CH (right before TX)"
# A full driver reload clears any accumulated monitor front-end state.
SNIFF_DRV=$(readlink -f /sys/class/net/$SNIFF_IF/device/driver 2>/dev/null | xargs basename 2>/dev/null)
[ -n "$SNIFF_DRV" ] && { sudo modprobe -r "$SNIFF_DRV" 2>/dev/null; sleep 2; sudo modprobe "$SNIFF_DRV" 2>/dev/null; sleep 3; }
sudo ip link set "$SNIFF_IF" down 2>/dev/null
sudo iw dev "$SNIFF_IF" set type monitor 2>/dev/null
sudo ip link set "$SNIFF_IF" up 2>/dev/null
sudo iw dev "$SNIFF_IF" set channel "$CH" 2>/dev/null
sleep 1

echo "[tx] starting tcpdump (filter SA $CANON_SA)"
sudo timeout $((DUR + 3)) tcpdump -i "$SNIFF_IF" -nn -e -c 20 \
  "wlan addr2 $CANON_SA" >/tmp/j2_tx_sniff.txt 2>/dev/null &
SNIFF_PID=$!
sleep 1

echo "[tx] starting txdemo (ch$CH rate $RATE)"
timeout "$DUR" sudo stdbuf -oL env \
  DEVOURER_VID=0x2357 DEVOURER_PID=0x012d DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_RATE=$RATE ${DEVOURER_TX_PWR:+DEVOURER_TX_PWR=$DEVOURER_TX_PWR} \
  ${DEVOURER_TX_DRAIN:+DEVOURER_TX_DRAIN=$DEVOURER_TX_DRAIN} \
  ${DEVOURER_TX_COEX_LOOP:+DEVOURER_TX_COEX_LOOP=$DEVOURER_TX_COEX_LOOP} \
  ${DEVOURER_RFE:+DEVOURER_RFE=$DEVOURER_RFE} \
  ${DEVOURER_TX_PWR_OVERRIDE:+DEVOURER_TX_PWR_OVERRIDE=$DEVOURER_TX_PWR_OVERRIDE} \
  ./build/txdemo >/tmp/j2_tx_demo.txt 2>&1 || true

wait $SNIFF_PID 2>/dev/null
echo "=== TX demo tail ==="; grep -iE 'bulk|TX|ready for TX|error|send' /tmp/j2_tx_demo.txt | tail -5
echo "=== sniffer caught (canonical SA beacons) ==="
n=$(grep -c "$CANON_SA" /tmp/j2_tx_sniff.txt 2>/dev/null || echo 0)
echo "frames from $CANON_SA: $n"
head -4 /tmp/j2_tx_sniff.txt
[ "$n" -gt 0 ] && echo "[tx] ON-AIR TX CONFIRMED" || echo "[tx] no frames sniffed"
