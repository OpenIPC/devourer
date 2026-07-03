#!/usr/bin/env bash
# jaguar2_tx_wseq.sh — capture the kernel rtl88x2bu COMPLETE register-write
# sequence from a cold driver (re)bind -> monitor-up -> inject, via a full-payload
# usbmon pcap. Realtek USB reg writes are vendor control-OUT (bmRequestType 0x40,
# bRequest 0x05, wValue=addr, data=value). Decodes the ordered (addr,len,val)
# list so it can be diffed against devourer's bring-up — the decisive test for
# the BB-never-keys-TX gate (a write-triggered action static state can't show).
set -uo pipefail

VMNAME=${VMNAME:-devourer-testrig}
VMSSH=${VMSSH:-josephnef@10.216.129.144}
VID=${VID:-2357}; PID=${PID:-012d}
CH=${CH:-9}
OUT=${OUT:-/tmp/j2_tx_wseq}
SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 $VMSSH"
mkdir -p "$OUT"

attached=0
cleanup() {
  $SSH 'sudo pkill -f inj.py; sudo pkill -f tcpdump' 2>/dev/null
  if [ "$attached" = "1" ]; then
    sudo virsh detach-device "$VMNAME" /tmp/_j2w_dut.xml --live 2>/dev/null
  fi
}
trap cleanup EXIT

cat >/tmp/_j2w_dut.xml <<XML
<hostdev mode='subsystem' type='usb' managed='yes'>
  <source><vendor id='0x${VID}'/><product id='0x${PID}'/></source>
</hostdev>
XML

echo "[wseq] attaching T3U to VM..."
sudo virsh attach-device "$VMNAME" /tmp/_j2w_dut.xml --live || { echo "attach failed"; exit 1; }
attached=1
sleep 5

IFACE=$($SSH 'for i in $(seq 1 15); do w=$(iw dev 2>/dev/null | awk "/Interface/{print \$2}" | grep -vE "wlp13|wlp4" | head -1); [ -n "$w" ] && { echo "$w"; break; }; sleep 1; done')
[ -z "$IFACE" ] && { echo "no iface"; exit 1; }
BUS=$($SSH "lsusb -d ${VID}:${PID} 2>/dev/null | sed -E 's/Bus 0*([0-9]+).*/\1/' | head -1")
DEVPATH=$($SSH "readlink -f /sys/bus/usb/devices/*/ 2>/dev/null | true")
echo "[wseq] iface=$IFACE bus=$BUS"

# scapy injector (guaranteed TX)
$SSH "cat >/tmp/inj.py" <<'PY'
import sys, time
from scapy.all import RadioTap, Dot11, sendp
f = RadioTap()/Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff",
                     addr2="57:42:75:05:d6:00", addr3="57:42:75:05:d6:00")
for _ in range(3000):
    sendp(f, iface=sys.argv[1], verbose=0); time.sleep(0.003)
PY

# find the driver + usb sysfs id for unbind/rebind (forces a full cold re-init)
USBID=$($SSH "for d in /sys/bus/usb/drivers/rtl88x2bu*/; do ls \$d 2>/dev/null | grep -E '^[0-9]+-' | head -1; done | head -1")
DRV=$($SSH "ls -d /sys/bus/usb/drivers/rtl88x2bu* 2>/dev/null | head -1")
echo "[wseq] driver=$DRV usbid=$USBID"

echo "[wseq] starting full-payload usbmon pcap on bus $BUS..."
$SSH "sudo bash -c 'nohup tcpdump -i usbmon${BUS} -w /tmp/wseq.pcap -U >/dev/null 2>&1 &'"
sleep 1

echo "[wseq] cold rebind (unbind->bind) to capture the full init write sequence..."
$SSH "echo '$USBID' | sudo tee ${DRV}/unbind >/dev/null 2>&1; sleep 2; echo '$USBID' | sudo tee ${DRV}/bind >/dev/null 2>&1; sleep 4"
# re-resolve iface after rebind
IFACE=$($SSH 'for i in $(seq 1 12); do w=$(iw dev 2>/dev/null | awk "/Interface/{print \$2}" | grep -vE "wlp13|wlp4" | head -1); [ -n "$w" ] && { echo "$w"; break; }; sleep 1; done')
echo "[wseq] post-rebind iface=$IFACE; monitor + inject..."
$SSH "sudo ip link set $IFACE down; sudo iw dev $IFACE set type monitor 2>/dev/null; sudo ip link set $IFACE up; sleep 1; sudo iw dev $IFACE set channel $CH 2>/dev/null; sleep 1"
$SSH "sudo bash -c 'nohup python3 /tmp/inj.py $IFACE >/tmp/inj.log 2>&1 &'"
sleep 3
$SSH 'sudo pkill -f tcpdump; sleep 1'

echo "[wseq] pulling pcap + decoding vendor reg-writes..."
$SSH "sudo chmod a+r /tmp/wseq.pcap 2>/dev/null; wc -c /tmp/wseq.pcap"
scp -q -o StrictHostKeyChecking=no "$VMSSH:/tmp/wseq.pcap" "$OUT/wseq.pcap" 2>/dev/null
echo "[wseq] pcap -> $OUT/wseq.pcap ($(wc -c <"$OUT/wseq.pcap" 2>/dev/null || echo 0) bytes)"
echo "[wseq] decode with: tests/decode_wseq.py $OUT/wseq.pcap"
