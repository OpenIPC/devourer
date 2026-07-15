#!/usr/bin/env bash
# ue_rx_attribution_check.sh — on-air validation of the per-UE RX attribution
# seed (src/cell/UeRxAttribution.h, M0 contract 4 of the 5G-NR RAN epic).
#
# Two transmitters with DISTINCT unicast SAs air frames at different cadences
# against one ue_rx_probe receiver. Pass = the probe's ue.rx windows attribute
# both TAs separately, with per-TA frame counts that reflect the cadence ratio
# — the per-transmitter view the device-wide GetRxQuality cannot give.
#
#   sudo bash tests/ue_rx_attribution_check.sh
#   TX1_PID=0x8812 TX2_PID=0xc812 RX_PID=0x012d CH=6 sudo bash tests/ue_rx_attribution_check.sh
set -uo pipefail
cd "$(dirname "$0")/.."

TX1_VID=${TX1_VID:-0x0bda}; TX1_PID=${TX1_PID:-0x8812}   # 8812AU (Jaguar1)
TX2_VID=${TX2_VID:-0x0bda}; TX2_PID=${TX2_PID:-0xc812}   # 8822CU (Jaguar3)
RX_VID=${RX_VID:-0x2357};  RX_PID=${RX_PID:-0x012d}      # 8812BU witness
CH=${CH:-6}; SECS=${SECS:-12}
# Distinct unicast TAs (I/G clear) — the attribution keys under test.
TA1=${TA1:-02:aa:bb:cc:dd:01}
TA2=${TA2:-02:aa:bb:cc:dd:02}
OUT=${OUT:-/tmp/ue_rx_attribution}

cleanup(){ sudo pkill -9 -x txdemo 2>/dev/null; sudo pkill -9 -x ue_rx_probe 2>/dev/null; return 0; }
trap cleanup EXIT
mkdir -p "$OUT"

if [ ! -x build/ue_rx_probe ] || [ tests/ue_rx_probe.cpp -nt build/ue_rx_probe ]; then
  echo "== building ue_rx_probe"
  g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ue_rx_probe.cpp \
      examples/common/env_config.cpp build/libdevourer.a \
      $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/ue_rx_probe
fi

cleanup; sleep 1

# QoS-Data frames (DEVOURER_TX_QOS_DATA — the SA override rides that shape),
# broadcast RA so neither TX solicits ACKs from the other.
echo "== TX1 ($TX1_VID:$TX1_PID SA=$TA1, 2 ms gap)"
sudo env DEVOURER_VID=$TX1_VID DEVOURER_PID=$TX1_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_SA=$TA1 \
     DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
     ./build/txdemo >"$OUT/tx1.jsonl" 2>"$OUT/tx1.err" &
echo "== TX2 ($TX2_VID:$TX2_PID SA=$TA2, 8 ms gap)"
sudo env DEVOURER_VID=$TX2_VID DEVOURER_PID=$TX2_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_SA=$TA2 \
     DEVOURER_TX_GAP_US=8000 DEVOURER_LOG_LEVEL=warn \
     ./build/txdemo >"$OUT/tx2.jsonl" 2>"$OUT/tx2.err" &
sleep 6   # TX bring-up

echo "== probe RX ($RX_VID:$RX_PID, ${SECS}s)"
sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_LOG_LEVEL=warn \
     ./build/ue_rx_probe "$SECS" >"$OUT/probe.jsonl" 2>"$OUT/probe.err"
cleanup; sleep 1

echo
python3 - "$OUT/probe.jsonl" "$TA1" "$TA2" <<'PYEOF'
import sys, json, collections
path, ta1, ta2 = sys.argv[1], sys.argv[2].lower(), sys.argv[3].lower()
frames = collections.Counter(); windows = collections.Counter()
rssi = collections.defaultdict(list)
for line in open(path):
    try: ev = json.loads(line)
    except ValueError: continue
    if ev.get("ev") != "ue.rx": continue
    ta = ev["ta"].lower()
    frames[ta] += ev["frames"]; windows[ta] += 1
    rssi[ta].append(ev["rssi_dbm"])
print(f"{'TA':<20}{'frames':>8}{'windows':>9}{'rssi_dbm':>10}")
for ta, n in frames.most_common():
    mean = sum(rssi[ta]) / len(rssi[ta])
    tag = " <- TX1" if ta == ta1 else (" <- TX2" if ta == ta2 else "")
    print(f"{ta:<20}{n:>8}{windows[ta]:>9}{mean:>10.1f}{tag}")
f1, f2 = frames.get(ta1, 0), frames.get(ta2, 0)
ok = True
if f1 < 100 or f2 < 25:
    print(f"FAIL: expected both TAs attributed (TX1={f1}, TX2={f2})"); ok = False
elif not (1.5 <= f1 / f2 <= 12.0):
    # 2 ms vs 8 ms gaps -> ~4x ratio; wide tolerance for airtime contention.
    print(f"FAIL: cadence ratio implausible (TX1/TX2 = {f1/f2:.1f}, expected ~4)"); ok = False
else:
    print(f"PASS: two UEs attributed separately (TX1={f1}, TX2={f2}, ratio {f1/f2:.1f})")
sys.exit(0 if ok else 1)
PYEOF
