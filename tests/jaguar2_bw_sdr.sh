#!/usr/bin/env bash
# jaguar2_bw_sdr.sh — validate RTL8822BU channel bandwidth on-air with the B210
# SDR (the method the jaguar3 chips used — sdr_tx_probe.py 99%-power occupied
# bandwidth), NOT a chip sniffer. A 20 MHz TX should occupy ~20 MHz, 40 MHz
# ~40 MHz, 80 MHz the full window (B210 ~46 MS/s can't span a whole 80 MHz, so
# 80 MHz reads as "clipped to the window" = clearly wider than 40 MHz).
#
# 5 GHz, ch149 primary (UNII-3, TX-allowed, usually clear). Central channel per
# devourer's offset math: 20M=149(5745), 40M off1=151(5755), 80M off1=155(5775).
set -uo pipefail
RATE=46.08e6
GAIN=45
NS=4e6
OUT=/tmp/j2_bw_sdr
rm -rf "$OUT"; mkdir -p "$OUT"

cleanup() { sudo pkill -x WiFiDriverTxDem 2>/dev/null; }
trap cleanup EXIT

recover() {
  sudo modprobe -r rtw88_8822bu 2>/dev/null
  sudo /usr/local/bin/uhubctl -l 4-2.3 -p 3 -a cycle >/dev/null 2>&1; sleep 6
  sudo modprobe -r rtw88_8822bu 2>/dev/null; sleep 1
}
probe() { # label freq
  sudo python3 tests/sdr_tx_probe.py --freq "$2" --rate "$RATE" --gain "$GAIN" \
      --nsamps "$NS" --label "$1" 2>&1 | grep -oE "\[sdr:[^]]*\][^(]*" | head -1
}

echo "[bw-sdr] baseline (T3U idle) @ 5755MHz"
recover
probe baseline 5755e6

# label  hopbw  offset  primary_ch  center_freq
run_bw() { # bw offset ch freq label
  local BW=$1 OFF=$2 CH=$3 FREQ=$4 LBL=$5
  echo "[bw-sdr] === $LBL: devourer TX bw${BW} ch${CH} off${OFF} @ ${FREQ} ==="
  recover
  timeout 26 sudo env DEVOURER_VID=0x2357 DEVOURER_PID=0x012d DEVOURER_CHANNEL=$CH \
    ${BW:+DEVOURER_HOP_BW=$BW} ${OFF:+DEVOURER_HOP_OFFSET=$OFF} \
    DEVOURER_TX_RATE=MCS1${BW:+/$BW} DEVOURER_TX_GAP_US=0 \
    ./build/WiFiDriverTxDemo >"$OUT/dev_${LBL}.log" 2>&1 &
  sleep 12   # power-on -> DLFW -> init -> TX flooding
  probe "$LBL" "$FREQ"
  sudo pkill -x WiFiDriverTxDem 2>/dev/null; wait 2>/dev/null; sleep 1
}

run_bw ""  ""  149 5745e6 bw20   # 20 MHz (no HOP_BW)
run_bw 40  1   149 5755e6 bw40   # 40 MHz, central 151
run_bw 80  1   149 5775e6 bw80   # 80 MHz, central 155

echo ""
echo "=== SUMMARY (occupied bandwidth per mode) ==="
echo "baseline (noise) ~ full window; a real signal narrows occ_bw to its width."
