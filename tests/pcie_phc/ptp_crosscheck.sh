#!/usr/bin/env bash
# ptp_crosscheck.sh — on the radxa-x4, (1) show the Intel I226 Ethernet doing
# real IEEE-1588 hardware PTP, and (2) cross-compare the 8821CE Wi-Fi TSF PHC
# against that real PTP clock on the SAME machine — quantifying the Wi-Fi TSF's
# frequency error and short-term stability versus genuine PTP hardware, with no
# second machine and no network dependency.
#
#   sudo tests/pcie_phc/ptp_crosscheck.sh
set -uo pipefail
cd "$(dirname "$0")"
ETH=enp2s0
WIF=wlp1s0
MOD=rtl8821ce_phc

cleanup() {
  echo "[xchk] cleanup"
  sudo pkill -x ptp4l phc2sys 2>/dev/null
  sudo rmmod "$MOD" 2>/dev/null
  sudo ip link set "$WIF" down 2>/dev/null
  sudo iw dev "$WIF" set type managed 2>/dev/null
  sudo ip link set "$WIF" up 2>/dev/null
  command -v nmcli >/dev/null && sudo nmcli dev set "$WIF" managed yes 2>/dev/null
}
trap cleanup EXIT

# --- find the igc (Ethernet) PHC ---
IGC_PTP=""
for d in /sys/class/ptp/ptp*; do
  n=$(cat "$d/clock_name" 2>/dev/null)
  case "$n" in
    rtl8821ce_tsf) : ;;                        # skip ours if already loaded
    *) [ -e "/dev/$(basename "$d")" ] && IGC_PTP="/dev/$(basename "$d")" ;;
  esac
done
echo "[xchk] Ethernet (I226) PHC = ${IGC_PTP:-<none>}"

# --- 1. prove the I226 does hardware-timestamped PTP ---
echo "[xchk] === ptp4l on $ETH (I226): does it use HARDWARE timestamping? ==="
sudo timeout 8 ptp4l -i "$ETH" -m 2>&1 | grep -iE "clock|hardware|selected|MASTER|assuming|state" | head -6 || true

# --- put the Wi-Fi MAC in monitor mode so the TSF ticks, then load our PHC ---
echo "[xchk] arming Wi-Fi TSF PHC"
sudo pkill -x wpa_supplicant 2>/dev/null
command -v nmcli >/dev/null && sudo nmcli dev set "$WIF" managed no 2>/dev/null
sleep 1
sudo ip link set "$WIF" down; sudo iw dev "$WIF" set type monitor 2>/dev/null; sudo ip link set "$WIF" up
sudo iw dev "$WIF" set channel 6 2>/dev/null
sleep 1
make -s >/dev/null 2>&1
sudo rmmod "$MOD" 2>/dev/null
sudo insmod "./$MOD.ko" || { echo "[xchk] PHC insmod failed"; exit 1; }
sleep 1
WIF_PTP=""
for d in /sys/class/ptp/ptp*; do
  [ "$(cat "$d/clock_name" 2>/dev/null)" = "rtl8821ce_tsf" ] && WIF_PTP="/dev/$(basename "$d")"
done
echo "[xchk] Wi-Fi TSF PHC = ${WIF_PTP:-<none>}"
[ -z "$WIF_PTP" ] && { echo "[xchk] no Wi-Fi PHC"; exit 1; }
[ -z "$IGC_PTP" ] && { echo "[xchk] no Ethernet PHC to compare against"; exit 1; }

# --- 2. cross-compare: discipline the Wi-Fi PHC to the I226 hardware clock.
# The Wi-Fi PHC is the slave (system + I226 untouched). Steady-state:
#   'freq' ~= the Wi-Fi crystal's frequency error vs the I226 (ppb)
#   'offset' jitter ~= the Wi-Fi TSF short-term stability vs real PTP hardware ---
echo "[xchk] === phc2sys: Wi-Fi TSF ($WIF_PTP) vs real PTP hardware ($IGC_PTP), ~25 s ==="
sudo timeout 27 phc2sys -s "$IGC_PTP" -c "$WIF_PTP" -O 0 -m 2>&1 | grep -iE "offset" | tail -20 || true
echo "[xchk] done"
