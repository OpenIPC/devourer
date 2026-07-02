#!/usr/bin/env bash
# jaguar2_rx_canary.sh — capture the WORKING RX register state of the RTL8822BU
# (Archer T3U, 2357:012d) from the pinned-kernel VM's vendor 88x2bu_ohd driver
# (== reference/rtl88x2bu), for diffing against devourer's jaguar2 RX bring-up.
#
# Flow: attach T3U to the VM -> vendor driver binds -> monitor mode on ch9 ->
# dump MAC/BB/RF registers via the rtw_proc interface -> detach back to host.
# The T3U is returned to the host on exit (trap) so devourer can use it again.
set -uo pipefail

VMNAME=devourer-testrig
VMSSH="josephnef@10.216.129.144"
VID=2357
PID=012d
CH=${1:-9}
OUT=${OUT:-/tmp/jaguar2_canary}
SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no $VMSSH"

mkdir -p "$OUT"

XML="<hostdev mode='subsystem' type='usb' managed='yes'><source><vendor id='0x${VID}'/><product id='0x${PID}'/></source></hostdev>"

attached=0
cleanup() {
  if [ "$attached" = "1" ]; then
    echo "[canary] detaching T3U back to host..."
    echo "$XML" > /tmp/_j2_dut.xml
    sudo virsh detach-device "$VMNAME" /tmp/_j2_dut.xml --live 2>/dev/null
    rm -f /tmp/_j2_dut.xml
    sleep 2
  fi
}
trap cleanup EXIT

echo "[canary] ensuring T3U detached from host kernel..."
# devourer claims at runtime only; nothing to unbind normally.

echo "[canary] attaching T3U ${VID}:${PID} to VM ${VMNAME}..."
echo "$XML" > /tmp/_j2_dut.xml
sudo virsh attach-device "$VMNAME" /tmp/_j2_dut.xml --live || { echo "attach failed"; exit 1; }
rm -f /tmp/_j2_dut.xml
attached=1
sleep 6

echo "[canary] VM: locating wlan iface..."
IFACE=$($SSH 'for i in $(seq 1 15); do w=$(iw dev 2>/dev/null | awk "/Interface/{print \$2}" | grep -vE "wlp13|wlp4" | head -1); [ -n "$w" ] && { echo "$w"; break; }; sleep 1; done')
# fall back: any new interface
[ -z "$IFACE" ] && IFACE=$($SSH 'iw dev 2>/dev/null | awk "/Interface/{print \$2}" | head -1')
echo "[canary] VM iface = '${IFACE}'"
[ -z "$IFACE" ] && { echo "no iface appeared"; $SSH 'dmesg | tail -20'; exit 1; }

echo "[canary] VM: monitor mode on ch${CH}..."
$SSH "sudo ip link set $IFACE down; sudo iw dev $IFACE set type monitor 2>/dev/null; sudo ip link set $IFACE up; sudo iw dev $IFACE set channel $CH 2>/dev/null; sleep 1; iw dev $IFACE info" | tee "$OUT/iface_info.txt"

echo "[canary] VM: confirm RX works (short capture)..."
$SSH "sudo timeout 6 tcpdump -i $IFACE -c 20 -nn 2>/dev/null | head -25" | tee "$OUT/rx_sample.txt"

echo "[canary] VM: proc interface layout..."
PROC=$($SSH 'ls -d /proc/net/rtl88x2bu_ohd/*/ 2>/dev/null | head -1')
echo "[canary] proc dir = '$PROC'"
$SSH "ls -la ${PROC} 2>/dev/null" | tee "$OUT/proc_ls.txt"

echo "[canary] VM: dump MAC/BB/RF registers via rtw_proc..."
for f in mac_reg_dump bb_reg_dump rf_reg_dump dump_reg read_reg rf_reg; do
  $SSH "cat ${PROC}${f} 2>/dev/null" > "$OUT/${f}.txt" 2>/dev/null
  sz=$(wc -l < "$OUT/${f}.txt" 2>/dev/null || echo 0)
  echo "  ${f}: ${sz} lines"
done

echo "[canary] done. Register dumps in $OUT/"
