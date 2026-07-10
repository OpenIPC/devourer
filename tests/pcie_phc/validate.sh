#!/usr/bin/env bash
# validate.sh — build, load, and exercise the RTL8821CE TSF PHC prototype.
# Run ON the radxa-x4 (the 8821CE host). rtw88 stays bound throughout; this
# module is a passive BAR2 reader.
#
# The MAC clock domain (which holds the TSF at 0x560) is power-gated by rtw88
# when the card is idle/unassociated — registers there read 0xEAEAEAEA and the
# TSF is frozen. Putting the card in MONITOR mode keeps the MAC RX continuously
# clocked, so the TSF free-runs cleanly (associated mode works too, and also
# syncs it to the AP). We use monitor mode here because it needs no credentials.
#
#   sudo tests/pcie_phc/validate.sh
set -uo pipefail
cd "$(dirname "$0")"
MOD=rtl8821ce_phc
IF=wlp1s0
PTP=""

cleanup() {
  echo "[phc] cleanup"
  sudo rmmod "$MOD" 2>/dev/null
  # restore the interface to managed + re-enable networking management
  sudo ip link set "$IF" down 2>/dev/null
  sudo iw dev "$IF" set type managed 2>/dev/null
  sudo ip link set "$IF" up 2>/dev/null
  command -v nmcli >/dev/null && sudo nmcli dev set "$IF" managed yes 2>/dev/null
}
trap cleanup EXIT

# --- put the card in monitor mode so the MAC (and TSF) stays clocked ---
echo "[phc] setting $IF to monitor mode (keeps the MAC/TSF clocked)"
sudo pkill -x wpa_supplicant 2>/dev/null
command -v nmcli >/dev/null && sudo nmcli dev set "$IF" managed no 2>/dev/null
sleep 1
sudo ip link set "$IF" down
sudo iw dev "$IF" set type monitor 2>&1 | head -1
sudo ip link set "$IF" up
sudo iw dev "$IF" set channel 6 2>&1 | head -1
sleep 1

# --- build ---
echo "[phc] building module + tools"
make clean >/dev/null 2>&1
make 2>&1 | grep -iE "error|warning|\.ko" | head || true
[ -f "$MOD.ko" ] || { echo "[phc] build failed"; exit 1; }
cc -O2 -o pcie_phc_lat pcie_phc_lat.c || { echo "[phc] lat build failed"; exit 1; }

# --- load ---
sudo rmmod "$MOD" 2>/dev/null
sudo dmesg -C
echo "[phc] insmod $MOD.ko"
sudo insmod "./$MOD.ko" || { echo "[phc] insmod failed"; dmesg | tail -5; exit 1; }
sleep 1
echo "[phc] === dmesg ==="
sudo dmesg | grep rtl8821ce_phc | tail -4
PTP=$(sudo dmesg | grep -oE '/dev/ptp[0-9]+' | tail -1)
[ -z "$PTP" ] && { echo "[phc] no /dev/ptp registered"; exit 1; }
echo "[phc] PHC device: $PTP"

# --- ethtool -T style capability via the PHC index; confirm it advances ---
if command -v phc_ctl >/dev/null; then
  echo "[phc] === phc_ctl: clock advances in real time? (two reads ~1 s apart) ==="
  sudo phc_ctl "$PTP" get 2>&1 | sed 's/^/  /'
  sleep 1.0
  sudo phc_ctl "$PTP" get 2>&1 | sed 's/^/  /'   # ~1.0 s later
  echo "[phc] === write ops: set 0 / adj +5 s / freq +1000 ppm ==="
  sudo phc_ctl "$PTP" set 0.0 get 2>&1 | sed 's/^/  /'
  sudo phc_ctl "$PTP" adj 5.0 get 2>&1 | sed 's/^/  /'
  sudo phc_ctl "$PTP" freq 1000000 freq 2>&1 | sed 's/^/  /'
fi

# --- the money metric: PHC gettime read window (the PCIe MMIO floor) ---
echo "[phc] === gettime read-window latency (PTP_SYS_OFFSET_EXTENDED) ==="
sudo ./pcie_phc_lat "$PTP" 25

# --- discipline the PHC from CLOCK_REALTIME as a real consumer (PHC is slave,
#     so the system clock is untouched); the offset should settle ---
if command -v phc2sys >/dev/null; then
  echo "[phc] === phc2sys disciplining the PHC from CLOCK_REALTIME ~5 s ==="
  sudo timeout 6 phc2sys -s CLOCK_REALTIME -c "$PTP" -O 0 -m 2>&1 | grep -iE "offset" | head -5 || true
fi
echo "[phc] done"
