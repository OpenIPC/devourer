#!/usr/bin/env bash
# M5 on-air TX validation for the Jaguar-3 port (RTL8812CU, 0bda:c812).
#
# Confirms devourer's 8822C TX path actually radiates on ch36 (5180 MHz, 20 MHz)
# by comparing USRP B210 in-band power with the chip silent vs transmitting the
# canonical beacon continuously. The c812 (bus 009) and B210 (bus 010) are on
# separate host controllers, so there's no USB contention during the capture.
#
#   sudo tests/jaguar3_tx_onair.sh
#
# Witness is the SDR, per the Jaguar-3 plan (radiotap can't prove on-air TX).
set -u
cd "$(dirname "$0")/.."

FREQ=5180e6
JSON=/tmp/j3_tx_onair.json
rm -f "$JSON"

probe() { python3 tests/sdr_tx_probe.py --freq "$FREQ" --label "$1" --json "$JSON"; }

echo "=== [1/3] baseline: c812 silent (kernel-bound), ambient only ==="
probe baseline
probe baseline   # second sample to gauge ambient variance

echo "=== [2/3] TX: devourer transmits continuously while SDR captures ==="
# Run the TX demo in the background via the standard run harness (it unbinds the
# kernel driver, forces the Jaguar-3 family, and handles power-cycle recovery on
# exit). Give it time to power-on + DLFW + replay before the chip starts TXing.
sudo SECS=25 tests/jaguar3_devourer_run.sh build/txdemo 25 \
    > /tmp/j3_tx_onair_dev.log 2>&1 &
HARNESS=$!
sleep 10   # power-on -> DLFW -> kernel replay -> first bulk-OUT
probe tx
probe tx
echo "    (waiting for TX harness to finish + rebind)"
wait "$HARNESS" 2>/dev/null

echo "=== [3/3] verdict ==="
python3 - "$JSON" <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
base = [r["mean_power_db"] for r in rows if r["label"] == "baseline"]
tx   = [r["mean_power_db"] for r in rows if r["label"] == "tx"]
b, t = sum(base)/len(base), sum(tx)/len(tx)
print(f"baseline mean power: {b:.2f} dB  {['%.2f'%x for x in base]}")
print(f"TX       mean power: {t:.2f} dB  {['%.2f'%x for x in tx]}")
print(f"delta: {t-b:+.2f} dB")
print(f"TX occupied BW: {[r['occupied_bw_hz']/1e6 for r in rows if r['label']=='tx']} MHz")
# A real on-air TX next to the B210 lifts in-band power well clear of bursty
# ambient AP traffic. Require >=6 dB rise to call it.
print("ON-AIR TX CONFIRMED" if (t - b) >= 6.0 else "INCONCLUSIVE — TX not clearly above ambient")
PY
grep -cE "bulk_send EP|TX #" /tmp/j3_tx_onair_dev.log | xargs echo "devourer TX submit lines:"
tail -3 /tmp/j3_tx_onair_dev.log | sed -E 's/^/[dev] /'
