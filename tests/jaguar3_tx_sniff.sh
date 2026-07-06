#!/usr/bin/env bash
# M5 on-air TX proof for the Jaguar-3 port, by DECODE (not just power).
#
# DUT:     RTL8812CU (0bda:c812, 9-1.3) runs devourer's Jaguar-3 TX demo,
#          injecting the canonical beacon SA 57:42:75:05:d6:00 on ch36.
# Witness: RTL8812AU (0bda:8812, 9-2) runs devourer's Jaguar-1 RX demo in
#          monitor on ch36 — examples/rx/main.cpp already prints <devourer-tx-hit>
#          when it decodes that SA. Dogfoods devourer on both ends and avoids
#          the RTL8814AU host-capture path entirely.
# Bonus:   USRP B210 in-band power at 5180 MHz.
#
#   sudo tests/jaguar3_tx_sniff.sh
set -u
cd "$(dirname "$0")/.."

SNIFF_LOG=/tmp/j3_sniff.log
DUT_LOG=/tmp/j3_dut_tx.log
rm -f "$SNIFF_LOG" "$DUT_LOG"

SNIFF_COMM=$(basename build/rxdemo | cut -c1-15)
cleanup() {
  sudo pkill -9 -x "$SNIFF_COMM" 2>/dev/null
}
trap cleanup EXIT

echo "=== start RX sniffer: devourer on RTL8812AU (ch36 monitor) ==="
sudo env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL=36 \
     stdbuf -oL -eL build/rxdemo > "$SNIFF_LOG" 2>&1 &
SNIFF=$!
# Wait until the sniffer is actually in its RX loop before keying the DUT.
for i in $(seq 1 20); do
  grep -qiE "rx loop|monitor|first RX|RX frame|channel" "$SNIFF_LOG" && break
  sleep 0.5
done
sleep 2
echo "    sniffer up ($(wc -l < "$SNIFF_LOG") log lines)"

echo "=== key DUT: devourer Jaguar-3 TX on RTL8812CU (ch36) ==="
sudo SECS=22 tests/jaguar3_devourer_run.sh build/txdemo 22 \
    > "$DUT_LOG" 2>&1 &
HARNESS=$!

# Capture SDR power mid-burst (DUT keys ~8-10 s in, after DLFW+replay).
sleep 11
python3 tests/sdr_tx_probe.py --freq 5180e6 --label tx 2>&1 | grep '\[sdr' || true

wait "$HARNESS" 2>/dev/null
sleep 1
cleanup

echo "=== verdict ==="
HITS=$(grep -c "devourer-tx-hit" "$SNIFF_LOG" 2>/dev/null || echo 0)
echo "DUT TX submit lines: $(grep -cE 'bulk_send EP|TX #' "$DUT_LOG")"
echo "sniffer total RX frames: $(grep -cE 'devourer.*hit|RX frame|<devourer>' "$SNIFF_LOG")"
echo "<devourer-tx-hit> count: $HITS"
grep "devourer-tx-hit" "$SNIFF_LOG" | head -3
if [ "$HITS" -gt 0 ]; then
  echo "ON-AIR TX CONFIRMED BY DECODE (canonical SA received over the air)"
else
  echo "NO DECODE — sniffer did not hear the canonical SA"
  echo "--- sniffer tail ---"; tail -5 "$SNIFF_LOG"
  echo "--- DUT tail ---"; tail -3 "$DUT_LOG"
fi
