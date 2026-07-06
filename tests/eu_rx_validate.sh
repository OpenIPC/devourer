#!/usr/bin/env bash
# Two-adapter RX validation for the RTL8812EU (0bda:a81a): the RTL8812AU
# (Jaguar1, 9-2) floods canonical-SA beacons (57:42:75:05:d6:00) on a channel
# while the EU runs the RX demo. Success = the EU prints <devourer-tx-hit>
# (it decoded the injected SA), proving the 8822e RX datapath delivers frames.
#
#   sudo tests/eu_rx_validate.sh [channel] [seconds]
set -u
CH=${1:-1}
SECS=${2:-30}
# TX source is configurable; default 8812AU. The CU (c812 / rtw88_8822cu) is a
# good alternative when the 8812AU's bulk-OUT is browning out on the hub chain.
TXPID=${TXPID:-8812}
TXDRV=${TXDRV:-rtw88_8812au}
# RX DUT is configurable too (default the 8812EU). RXDRV may be empty when the
# receiver has no kernel driver bound (the bare EU module).
RXPID=${RXPID:-a81a}
RXDRV=${RXDRV:-}

# --- locate adapters by PID ---
find_sys() { for d in /sys/bus/usb/devices/*/idProduct; do
  [ "$(cat "$d" 2>/dev/null)" = "$1" ] && { basename "$(dirname "$d")"; return; }; done; }
TXS=$(find_sys "$TXPID")   # TX source
RXS=$(find_sys "$RXPID")   # receiver DUT
[ -z "$TXS" ] && { echo "ERROR: TX PID=$TXPID not found"; exit 1; }
[ -z "$RXS" ] && { echo "ERROR: RX PID=$RXPID not found"; exit 1; }
echo "[harness] TX $TXPID=$TXS  RX $RXPID=$RXS  ch=$CH  ${SECS}s"
TXHUB=${TXS%.*}; TXPORT=${TXS##*.}

cleanup() {
  sudo pkill -9 -x txdemo 2>/dev/null
  sudo pkill -9 -x rxdemo 2>/dev/null
  echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/$TXDRV/bind >/dev/null 2>&1
  [ -n "$RXDRV" ] && echo "$RXS:1.0" | sudo tee /sys/bus/usb/drivers/$RXDRV/bind >/dev/null 2>&1
  sleep 2
}
trap cleanup EXIT
[ -n "$RXDRV" ] && { echo "$RXS:1.0" | sudo tee /sys/bus/usb/drivers/$RXDRV/unbind >/dev/null 2>&1; sleep 1; }

# --- release 8812AU from kernel, start beacon flood ---
echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/$TXDRV/unbind >/dev/null 2>&1; sleep 1
echo "=== starting $TXPID beacon flood on ch$CH ==="
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x$TXPID DEVOURER_CHANNEL=$CH \
     stdbuf -oL timeout -k 5 "$SECS" "${TXBIN:-build/txdemo}" >/tmp/eu_rxval_tx.log 2>&1 &
sleep 6   # let TX come up before RX starts listening

# --- run RX DUT ---
echo "=== running RX DUT ($RXPID) on ch$CH ==="
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x$RXPID DEVOURER_CHANNEL=$CH \
     stdbuf -oL -eL timeout -k 5 $((SECS-7)) "${RXBIN:-build/rxdemo}" 2>&1 \
     | tee /tmp/eu_rxval_rx.log | grep -iE "IQK done|RX pkt|tx-hit|entering RX" | sed 's/^/[rx] /'

echo "==================== RESULT ===================="
echo "EU RX pkt lines : $(grep -c 'RX pkt' /tmp/eu_rxval_rx.log)"
echo "EU canonical hit: $(grep -c 'tx-hit' /tmp/eu_rxval_rx.log)"
echo "TX bulk sends   : $(grep -c 'bulk_send' /tmp/eu_rxval_tx.log)"
grep -m1 "tx-hit" /tmp/eu_rxval_rx.log && echo ">>> EU RX DATAPATH CONFIRMED <<<" || echo ">>> no canonical-SA hit (see /tmp/eu_rxval_{tx,rx}.log) <<<"
