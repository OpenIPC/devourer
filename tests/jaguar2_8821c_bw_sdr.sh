#!/usr/bin/env bash
# jaguar2_8821c_bw_sdr.sh — validate RTL8811CU (8821C) channel bandwidth on-air
# with the B210 SDR (sdr_tx_probe.py 99%-power occupied bandwidth). Directly
# exercises the 0x8ac ADC/DAC clock word: a 20 MHz TX should occupy ~20 MHz,
# 40 MHz ~40 MHz, 80 MHz the full ~46 MS/s window (wider than 40). Before the
# 0x8ac fix the 80 MHz word set the wrong clock bits, so this is the on-air
# proof of that fix.
#
# 5 GHz ch149 (UNII-3, clear). The 8821C worldwide-min txpwr_lmt clamps UNII-3
# to 0, so DEVOURER_TX_PWR forces a flat TXAGC to guarantee radiation (occupied
# bandwidth is independent of power level). Central per devourer offset math:
# 20M=149(5745), 40M off1=151(5755), 80M off1=155(5775).
set -uo pipefail
RATE=46.08e6
GAIN=45
NS=4e6
PWR=0x2d
OUT=/tmp/j2_8821c_bw_sdr
rm -rf "$OUT"; mkdir -p "$OUT"

cleanup() { sudo pkill -x WiFiDriverTxDemo 2>/dev/null; sudo modprobe rtw88_8821cu 2>/dev/null; }
trap cleanup EXIT

probe() { # label freq
  sudo python3 tests/sdr_tx_probe.py --freq "$2" --rate "$RATE" --gain "$GAIN" \
      --nsamps "$NS" --label "$1" 2>&1 | grep -oE "\[sdr:[^]]*\][^(]*" | head -1
}

run_bw() { # bw offset ch freq label
  local BW=$1 OFF=$2 CH=$3 FREQ=$4 LBL=$5
  echo "[bw-sdr] === $LBL: 8821C TX bw${BW:-20} ch${CH} off${OFF:-0} @ ${FREQ} ==="
  # DLFW is flaky; retry the TX bring-up until it floods (up to 4 tries).
  local try
  for try in 1 2 3 4; do
    sudo modprobe -r rtw88_8821cu 2>/dev/null; sleep 1
    timeout 24 sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xc811 DEVOURER_CHANNEL=$CH \
      ${BW:+DEVOURER_HOP_BW=$BW} ${OFF:+DEVOURER_HOP_OFFSET=$OFF} \
      DEVOURER_TX_RATE=MCS1${BW:+/$BW} DEVOURER_TX_GAP_US=0 DEVOURER_TX_PWR=$PWR \
      ./build/WiFiDriverTxDemo >"$OUT/dev_${LBL}.log" 2>&1 &
    sleep 10   # power-on -> DLFW -> init -> TX flooding
    if grep -q 'ready for TX' "$OUT/dev_${LBL}.log"; then break; fi
    echo "  (try $try: bring-up failed, retrying)"; sudo pkill -x WiFiDriverTxDemo 2>/dev/null; wait 2>/dev/null
  done
  probe "$LBL" "$FREQ"
  sudo pkill -x WiFiDriverTxDemo 2>/dev/null; wait 2>/dev/null; sleep 1
}

echo "[bw-sdr] baseline (no TX) @ 5755MHz"
sudo modprobe -r rtw88_8821cu 2>/dev/null; sleep 1
probe baseline 5755e6

run_bw ""  ""  149 5745e6 bw20   # 20 MHz
run_bw 40  1   149 5755e6 bw40   # 40 MHz, central 151
run_bw 80  1   149 5775e6 bw80   # 80 MHz, central 155

echo ""
echo "=== SUMMARY: occ_bw should be ~20 / ~40 / >40 (clipped) MHz ==="
echo "(baseline noise ~ full window; a real signal narrows occ_bw to its width)"
