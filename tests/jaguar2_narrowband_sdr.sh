#!/usr/bin/env bash
# jaguar2_narrowband_sdr.sh — 5/10 MHz narrowband validation for Jaguar2
# (RTL8811CU/8821C default; CHIP=8822b selects the RTL8822BU). The Jaguar2
# twin of
# tests/jaguar3_narrowband_sdr.sh: radiotap stays 20 MHz in narrowband mode, so
# the ONLY witness is an SDR. Measures the OCCUPIED BANDWIDTH of devourer's
# continuous TX at 20 / 10 / 5 MHz with the USRP B210 and confirms it halves /
# quarters — the on-air proof of the 0x8ac ADC/DAC re-clock (small-BW [7:6] +
# 0x8c4[30]/0x8c8[31]).
#
# Ambient isolation: DIFFERENTIAL PSD (TX PSD - silent-baseline PSD); the
# -10 dB-about-peak width of that difference spectrum is the discriminator.
#
#   sudo tests/jaguar2_narrowband_sdr.sh            # 8811CU (0bda:c811)
#   CHIP=8822b sudo tests/jaguar2_narrowband_sdr.sh # 8822BU (T3U 2357:012d)
set -u
cd "$(dirname "$0")/.."

CHIP=${CHIP:-8821c}
if [ "$CHIP" = 8821c ]; then
  VID=0x0bda; PID=0xc811; DRV=rtw88_8821cu
  # 8821C worldwide-min txpwr_lmt clamps UNII-3 to 0 — force a flat TXAGC
  # (occupied bandwidth is independent of power level).
  TX_PWR=${TX_PWR:-0x2d}
else
  VID=0x2357; PID=0x012d; DRV=rtw88_8822bu
  TX_PWR=${TX_PWR:-}
fi

# CHANNEL/FREQ must agree (ch44=5220e6). ch44 default — ch36 frequently has a
# strong ambient AP that swamps the differential PSD.
CHANNEL=${CHANNEL:-44}
FREQ=${FREQ:-5220e6}
RATE=46.08e6   # wide enough to see the full 20 MHz channel + skirts
OUT=/tmp/j2_nb_${CHIP}
rm -rf "$OUT"; mkdir -p "$OUT"

cleanup() {
  sudo pkill -x txdemo 2>/dev/null
  sudo modprobe "$DRV" 2>/dev/null
}
trap cleanup EXIT

probe() { # label  psd_out
  sudo python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 6e6 --label "$1" --psd-out "$2" 2>&1 | grep '\[sdr' || true
}

echo "=== baseline: $CHIP silent (ambient PSD) ==="
sudo modprobe -r "$DRV" 2>/dev/null; sleep 1
probe baseline "$OUT/baseline.npy"

for BW in 20 10 5; do
  echo "=== TX at ${BW} MHz: devourer continuous, capture PSD ==="
  # DLFW is flaky on the 8821C; retry the TX bring-up until it floods.
  for try in 1 2 3 4; do
    sudo modprobe -r "$DRV" 2>/dev/null; sleep 1
    timeout 26 sudo env DEVOURER_VID=$VID DEVOURER_PID=$PID \
      DEVOURER_CHANNEL="$CHANNEL" DEVOURER_NB_BW="$BW" \
      DEVOURER_TX_PWR="$TX_PWR" DEVOURER_TX_GAP_US=0 \
      ./build/txdemo >"$OUT/dev_${BW}.log" 2>&1 &
    sleep 11   # power-on -> DLFW -> init -> TX flooding
    if grep -q 'ready for TX' "$OUT/dev_${BW}.log"; then break; fi
    echo "  (try $try: bring-up failed, retrying)"
    sudo pkill -x txdemo 2>/dev/null; wait 2>/dev/null
  done
  probe "tx${BW}" "$OUT/tx_${BW}.npy"
  sudo pkill -x txdemo 2>/dev/null; wait 2>/dev/null; sleep 2
done

echo "=== differential-PSD occupied bandwidth ==="
python3 - "$OUT" "$RATE" <<'PY'
import sys, numpy as np
out, rate = sys.argv[1], float(sys.argv[2])
base = np.load(f"{out}/baseline.npy"); bf, bp = base[0], base[1]

def occ_bw(label):
    a = np.load(f"{out}/{label}.npy"); f, p = a[0], a[1]
    diff = p - bp                       # isolate devourer's emission
    diff = np.clip(diff, 0, None)
    if diff.max() <= 0:
        return None, 0.0
    k = 33
    sm = np.convolve(diff, np.ones(k)/k, mode='same')
    tot = sm.sum()
    csum = np.cumsum(sm)
    lo = np.searchsorted(csum, 0.005*tot)
    hi = np.searchsorted(csum, 0.995*tot)
    bw = f[min(hi, len(f)-1)] - f[max(lo,0)]
    # -10 dB width about the peak — CONTIGUOUS run containing the peak (a
    # first/last-above-threshold scan bridges the DUT lobe with ambient
    # bursts elsewhere in the span and inflates the width).
    pk = sm.max(); thr = pk/10.0
    ipk = int(np.argmax(sm))
    lo_i = ipk
    while lo_i > 0 and sm[lo_i-1] >= thr:
        lo_i -= 1
    hi_i = ipk
    while hi_i < len(sm)-1 and sm[hi_i+1] >= thr:
        hi_i += 1
    bw10 = f[hi_i] - f[lo_i]
    return bw, bw10

print(f"{'bw':>6} {'-10dB(MHz)':>12} {'occ99ref(MHz)':>14}")
res = {}
for bw in (20, 10, 5):
    o99, o10 = occ_bw(f"tx_{bw}")
    res[bw] = o10
    s99 = f"{o99/1e6:.2f}" if o99 else "n/a"
    print(f"{bw:>5}M {o10/1e6:>12.2f} {s99:>14}")
if res.get(20) and res.get(10):
    print(f"\nratio 20M/10M = {res[20]/res[10]:.2f} (expect ~2.0)")
if res.get(20) and res.get(5):
    print(f"ratio 20M/5M  = {res[20]/res[5]:.2f} (expect ~4.0)")
ok = (res.get(20) and res.get(10) and res.get(5)
      and 1.6 <= res[20]/res[10] <= 2.5
      and 3.0 <= res[20]/res[5] <= 5.0)
print("\nNARROWBAND CONFIRMED (10 MHz ~ half, 5 MHz ~ quarter of 20 MHz "
      "occupied BW)" if ok
      else "\nINCONCLUSIVE — bandwidth ratio not as expected (see PSDs in "+out+")")
PY
