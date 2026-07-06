#!/usr/bin/env bash
# jaguar2_tx_bw40.sh — validate RTL8822BU 40 MHz on-air TX. Sniffer in HT40 mode
# on the same primary channel; T3U transmits a 40 MHz frame (DEVOURER_HOP_BW=40 +
# a /40 radiotap rate). Uses the harness-safe ordering (recover T3U first, then
# fresh-reset the near-field sniffer right before TX).
set -uo pipefail
CH=${1:-9}
RATE=${2:-MCS1/40}
HT=${3:-HT40-}          # HT40-: secondary below (primary upper) -> our offset 2
OFFSET=${4:-2}
SNIFF_IF=${SNIFF_IF:-wlp4s0u2u4}   # 8822cu (supports 5G + HT40)
CANON=57:42:75:05:d6:00
DUR=${DUR:-16}

cleanup() { sudo pkill -f "tcpdump -i $SNIFF_IF" 2>/dev/null; sudo pkill -x txdemo 2>/dev/null; }
trap cleanup EXIT

echo "[bw40] recovering T3U first"
sudo modprobe -r rtw88_8822bu 2>/dev/null
sudo /usr/local/bin/uhubctl -l 4-2.3 -p 3 -a cycle >/dev/null 2>&1; sleep 6
sudo modprobe -r rtw88_8822bu 2>/dev/null; sleep 1

echo "[bw40] fresh sniffer $SNIFF_IF -> monitor ch$CH $HT"
# Full modprobe reset clears the near-field-degraded monitor front-end. The c812
# sometimes needs a second cycle to re-enumerate its netdev.
SNIFF_DRV=$(readlink -f /sys/class/net/$SNIFF_IF/device/driver 2>/dev/null | xargs basename 2>/dev/null)
SNIFF_DRV=${SNIFF_DRV:-rtw88_8822cu}
sudo modprobe -r "$SNIFF_DRV" 2>/dev/null; sleep 2
sudo modprobe "$SNIFF_DRV" 2>/dev/null; sleep 3
if [ ! -e "/sys/class/net/$SNIFF_IF" ]; then
  sudo modprobe -r "$SNIFF_DRV" 2>/dev/null; sleep 2; sudo modprobe "$SNIFF_DRV" 2>/dev/null; sleep 4
fi
sudo ip link set "$SNIFF_IF" down 2>/dev/null
sudo iw dev "$SNIFF_IF" set type monitor 2>/dev/null
sudo ip link set "$SNIFF_IF" up 2>/dev/null
sudo iw dev "$SNIFF_IF" set channel "$CH" "$HT" 2>/dev/null || sudo iw dev "$SNIFF_IF" set channel "$CH" 2>/dev/null
sleep 1
echo "[bw40] sniffer ambient sanity: $(sudo timeout 3 tcpdump -i "$SNIFF_IF" -nn 2>/dev/null | wc -l) frames/3s"

sudo timeout $((DUR + 3)) tcpdump -i "$SNIFF_IF" -nn -e "wlan addr2 $CANON" >/tmp/j2_bw40_sniff.txt 2>/dev/null &
sleep 1
HOPBW=${HOPBW:-40}   # devourer TX bandwidth (40 or 80)
echo "[bw40] TX ch$CH bw$HOPBW offset$OFFSET rate $RATE"
timeout "$DUR" sudo env DEVOURER_VID=0x2357 DEVOURER_PID=0x012d DEVOURER_CHANNEL=$CH \
  DEVOURER_HOP_BW=$HOPBW DEVOURER_HOP_OFFSET=$OFFSET DEVOURER_TX_RATE=$RATE \
  ./build/txdemo >/tmp/j2_bw40_demo.txt 2>&1 || true
sleep 2
echo "=== rf18/bw: $(grep -oE 'channel set ch=[0-9]+ bw=[0-9]+ .rf18=0x[0-9a-f]+' /tmp/j2_bw40_demo.txt | head -1) ==="
echo "=== sent: $(grep -c bulk_send /tmp/j2_bw40_demo.txt) ==="
echo "=== frames decoded from $CANON ==="
grep -c "$CANON" /tmp/j2_bw40_sniff.txt
grep -oE "MCS [0-9]+|40 MHz|80 MHz|HT40|VHT|[0-9]+\.[0-9]+ Mb/s" /tmp/j2_bw40_sniff.txt | sort | uniq -c | head
