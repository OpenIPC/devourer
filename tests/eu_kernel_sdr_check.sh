#!/usr/bin/env bash
# Is the vendor rtl88x2eu_ohd actually RADIATING during monitor injection on
# this unit? (issue #238 ground-truth harness debugging: the CU ground caught
# 0 of 33k kernel-injected MCS0 frames — silent-drop vs deaf-ground bisect.)
# One SDR read per invocation.
#   sudo tests/eu_kernel_sdr_check.sh [mcs] [payload]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MCS="${1:-0}"; PAYLOAD="${2:-148}"
CH="${CH:-36}"; FREQ="${FREQ:-5180e6}"
KMOD=rtl88x2eu_ohd
cleanup(){ sudo -n pkill -9 -f kernel_tx_inject 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# Bind the vendor driver if not already up (VBUS cold first — kernel FW-DL
# fails on a chip warm from devourer).
IF=""
for d in /sys/class/net/*; do
    drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
    [ "$drv" = "$KMOD" ] && { IF=$(basename "$d"); break; }
done
if [ -z "$IF" ]; then
    SYS=""
    for d in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
    done
    [ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
    sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || true
    sleep 4
    lsmod | grep -q "^$KMOD" || sudo -n insmod "$ROOT/reference/rtl88x2eu/${KMOD}.ko"
    sleep 6
    for d in /sys/class/net/*; do
        drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
        [ "$drv" = "$KMOD" ] && { IF=$(basename "$d"); break; }
    done
fi
[ -n "$IF" ] || { echo "ERROR: no $KMOD netdev"; exit 1; }
echo "EU kernel netdev = $IF"
sudo -n ip link set "$IF" down
sudo -n iw dev "$IF" set monitor none
sudo -n ip link set "$IF" up
sudo -n iw dev "$IF" set channel "$CH"
sleep 1

sudo -n python3 "$ROOT/tests/kernel_tx_inject.py" "$IF" "$MCS" "$PAYLOAD" 20 \
    >/tmp/eu_ktx_sdr.log 2>&1 &
sleep 4
echo "--- SDR duty during kernel MCS$MCS inject (ch$CH) ---"
sudo -n python3 "$ROOT/tests/sdr_duty.py" --freq "$FREQ" --rate 25e6 --secs 5 \
    --gain 60 --bw 20 2>&1 | grep -iE "duty=|error" | head -3
cleanup
tail -1 /tmp/eu_ktx_sdr.log
