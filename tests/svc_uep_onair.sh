#!/usr/bin/env bash
# On-air verification of SvcTxDemo's TID -> TxMode UEP mapping.
#
# Synthetic HEVC NALs (tests/gen_svc_nals.py, ratio 1:4:8:16 = IDR:T0:T1:T2) are
# injected by the 8812; the 8814 kernel monitor witness decodes the per-frame
# MCS. The decoded histogram should track the default_policy ladder
# (critical=MCS0, T0=MCS1, T1=MCS4, T2=MCS7) in the same 1:4:8:16 proportion.
#
# ch6 (2.4 GHz, low current) avoids the USB Vbus-sag gotcha; a fresh power-cycle
# is taken first. Run: sudo bash tests/svc_uep_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SA=57:42:75:05:d6:00
TX=9-2; WIT=4-2.3.2
KILL(){ sudo pkill -9 SvcTxDemo 2>/dev/null; sudo pkill -9 -f gen_svc_nals 2>/dev/null; }
trap KILL EXIT

echo "=== fresh rail (power-cycle hub tree) ==="
sudo modprobe -r rtw88_8812au rtw88_8814au rtw88_8821au 2>/dev/null
for hp in "9 2" "4-2.3 2" "9-1 4"; do sudo uhubctl -a off -l ${hp% *} -p ${hp#* } >/dev/null 2>&1; done; sleep 18
for hp in "9 2" "4-2.3 2" "9-1 4"; do sudo uhubctl -a on -l ${hp% *} -p ${hp#* } >/dev/null 2>&1; done; sleep 9
sudo modprobe rtw88_8812au rtw88_8814au rtw88_8821au 2>/dev/null

# 8814 kernel monitor witness @ ch6
for i in /sys/bus/usb/devices/$WIT/$WIT:*; do printf '%s' "$(basename "$i")"|sudo tee /sys/bus/usb/drivers_probe>/dev/null 2>&1; done; sleep 4
W=$(basename "$(readlink -f /sys/bus/usb/devices/$WIT/*:1.0/net/* 2>/dev/null)")
sudo ip link set "$W" down 2>/dev/null; sudo iw dev "$W" set monitor none 2>/dev/null
sudo ip link set "$W" up; sudo iw dev "$W" set channel 6 2>/dev/null
echo "witness=$W $(iw dev "$W" info 2>/dev/null|grep -oE 'channel [0-9]+')"

# free the 8812 for devourer, inject the SVC stream
for i in /sys/bus/usb/devices/$TX/$TX:*; do ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null); [ -n "$drv" ]&&echo "$ifc"|sudo tee "$drv/unbind">/dev/null 2>&1; done; sleep 1
# Witness-friendly ladder: all 20 MHz, distinct MCS, no LDPC/STBC — so a plain
# 20 MHz monitor decodes every rung. (The realistic default_policy stacks
# 40 MHz/LDPC/STBC on the rungs, which a 20 MHz sniffer can't fully decode;
# that ladder is for real links, not this histogram check.)
python3 "$ROOT/tests/gen_svc_nals.py" 12 | \
  sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 \
       DEVOURER_SVC_LADDER="CRIT=MCS0;T0=MCS1;T1=MCS4;T2=MCS7" \
       "$ROOT/build/SvcTxDemo" --gap-us 600 >/tmp/svc.log 2>&1 &
sleep 9
echo "=== SvcTxDemo policy + per-TID counters ==="
grep -E "SVC UEP policy|  T[0-9] ->|<svc>" /tmp/svc.log | tail -6
echo "=== witness decoded MCS histogram (expect MCS0:MCS1:MCS4:MCS7 ~ 1:4:8:16) ==="
sudo timeout 6 tcpdump -i "$W" -e -nn "ether src $SA" 2>/dev/null | grep -oE "MCS [0-9]+" | sort -V | uniq -c
KILL
