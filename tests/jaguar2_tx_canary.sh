#!/usr/bin/env bash
# jaguar2_tx_canary.sh — capture the kernel rtl88x2bu register + USB state while
# it is ACTIVELY INJECTING (aireplay-ng), so the TX path is genuinely enabled.
# A passive monitor interface never keys TX, so its register/usbmon state has no
# TX-path activation to diff against — this is the fix for that.
#
# Flow: attach T3U to VM -> monitor mode ch9 -> aireplay-ng injection loop ->
# while injecting: dump mac/bb/rf regs + short usbmon capture -> pull back.
set -uo pipefail

VMNAME=${VMNAME:-devourer-testrig}
VMSSH=${VMSSH:-josephnef@10.216.129.144}
VID=${VID:-2357}; PID=${PID:-012d}
CH=${CH:-9}
OUT=${OUT:-/tmp/j2_tx_canary}
SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 $VMSSH"
mkdir -p "$OUT"

attached=0
cleanup() {
  $SSH 'sudo pkill -f aireplay-ng; sudo pkill -f "cat /sys/kernel/debug/usb/usbmon" 2>/dev/null' 2>/dev/null
  if [ "$attached" = "1" ]; then
    echo "[canary] detaching T3U from VM..."
    sudo virsh detach-device "$VMNAME" /tmp/_j2t_dut.xml --live 2>/dev/null
  fi
}
trap cleanup EXIT

# libvirt USB host-device XML for the T3U (by vendor:product)
cat >/tmp/_j2t_dut.xml <<XML
<hostdev mode='subsystem' type='usb' managed='yes'>
  <source><vendor id='0x${VID}'/><product id='0x${PID}'/></source>
</hostdev>
XML

echo "[canary] attaching T3U ${VID}:${PID} to VM ${VMNAME}..."
sudo virsh attach-device "$VMNAME" /tmp/_j2t_dut.xml --live || { echo "attach failed"; exit 1; }
attached=1
sleep 4

IFACE=$($SSH 'for i in $(seq 1 15); do w=$(iw dev 2>/dev/null | awk "/Interface/{print \$2}" | grep -vE "wlp13|wlp4" | head -1); [ -n "$w" ] && { echo "$w"; break; }; sleep 1; done')
[ -z "$IFACE" ] && { echo "no iface"; exit 1; }
echo "[canary] iface = $IFACE"

echo "[canary] monitor mode ch${CH}..."
$SSH "sudo ip link set $IFACE down; sudo iw dev $IFACE set type monitor 2>/dev/null; sudo ip link set $IFACE up; sleep 1; sudo iw dev $IFACE set channel $CH 2>/dev/null; sudo iwconfig $IFACE channel $CH 2>/dev/null; sleep 1; iw dev $IFACE info" | tee "$OUT/iface_info.txt"

PROC=$($SSH 'ls -d /proc/net/rtl88x2bu_ohd/*/ 2>/dev/null | grep -vE "chplan|country|drv_cfg|global|halmac" | head -1')
echo "[canary] proc dir = '$PROC'"

# find the usbmon bus for the T3U in the VM
BUS=$($SSH "lsusb -d ${VID}:${PID} 2>/dev/null | sed -E 's/Bus 0*([0-9]+).*/\1/' | head -1")
echo "[canary] T3U on VM usb bus = ${BUS:-?}"

# scapy injector: sends a Dot11 frame with the canonical devourer SA repeatedly
# on the monitor iface — guaranteed on-air TX so the kernel keys its TX path.
$SSH "cat >/tmp/inj.py" <<'PY'
import sys, time
from scapy.all import RadioTap, Dot11, sendp
iface = sys.argv[1]
f = RadioTap()/Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff",
                     addr2="57:42:75:05:d6:00", addr3="57:42:75:05:d6:00")
for _ in range(4000):
    sendp(f, iface=iface, verbose=0)
    time.sleep(0.002)
PY

echo "[canary] starting scapy injection + usbmon capture..."
$SSH "sudo bash -c 'nohup python3 /tmp/inj.py $IFACE >/tmp/inj.log 2>&1 &'"
sleep 1
# usbmon text capture: bulk-OUT URBs (' Bo') show the first 32 payload bytes =
# TX descriptor dwords 0-7 (MACID/QSEL/RATE_ID/rate/bw). Grab a window.
$SSH "sudo timeout 5 cat /sys/kernel/debug/usb/usbmon/${BUS}u 2>/dev/null | grep ' Bo:' | head -80" > "$OUT/usbmon_tx.txt" 2>/dev/null
echo "  usbmon bulk-OUT lines: $(wc -l < "$OUT/usbmon_tx.txt" 2>/dev/null || echo 0)"
# full-payload capture via tcpdump on the usbmon device (text truncates at 32B;
# the TX descriptor is 48B, so grab dwords 8-11 too). -X hexdumps the payload.
$SSH "sudo timeout 4 tcpdump -i usbmon${BUS} -c 8 -X 'greater 60' 2>/dev/null" > "$OUT/usbmon_full.txt" 2>/dev/null
echo "  usbmon full-payload lines: $(wc -l < "$OUT/usbmon_full.txt" 2>/dev/null || echo 0)"
sleep 1

echo "[canary] dumping registers WHILE injecting..."
for f in mac_reg_dump bb_reg_dump rf_reg_dump; do
  $SSH "cat ${PROC}${f} 2>/dev/null" > "$OUT/${f}.txt" 2>/dev/null
  echo "  ${f}: $(wc -l < "$OUT/${f}.txt" 2>/dev/null || echo 0) lines"
done
$SSH 'tail -3 /tmp/inj.log 2>/dev/null' | tee "$OUT/inj_tail.txt"

echo "[canary] done. Dumps (kernel injecting) in $OUT/"
