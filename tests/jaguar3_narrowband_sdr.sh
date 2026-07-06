#!/usr/bin/env bash
# M6 narrowband validation for the Jaguar-3 port (RTL8812CU, 0bda:c812, the
# default; PID=a81a selects the RTL8812EU).
#
# The original goal of the whole Jaguar-3 port: 5/10 MHz channels via the
# baseband clock divider (0x9b0/0x9b4), which Jaguar-1 cannot do. Radiotap stays
# 20 MHz, so the ONLY witness is an SDR. We measure the OCCUPIED BANDWIDTH of
# devourer's continuous TX at 20 / 10 / 5 MHz with the USRP B210 and confirm it
# halves / quarters.
#
# Because the on-air signal is weak vs ambient 5 GHz APs on ch36, we isolate
# devourer's footprint with a DIFFERENTIAL PSD: (TX PSD - silent-baseline PSD).
# The width of that difference spectrum is the real emitted bandwidth.
#
#   sudo tests/jaguar3_narrowband_sdr.sh
set -u
cd "$(dirname "$0")/.."

# CHANNEL/FREQ must agree (ch36=5180e6, ch44=5220e6). Default ch44 — ch36
# frequently has a strong ambient AP that swamps the differential PSD.
CHANNEL=${CHANNEL:-44}
FREQ=${FREQ:-5220e6}
RATE=46.08e6   # wide enough to see the full 20 MHz channel + skirts
OUT=/tmp/j3_nb
rm -rf "$OUT"; mkdir -p "$OUT"

probe() { # label  psd_out  [extra sdr args]
  python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 6e6 --label "$1" --psd-out "$2" 2>&1 | grep '\[sdr' || true
}

echo "=== baseline: c812 silent (ambient PSD) ==="
probe baseline "$OUT/baseline.npy"

for BW in 20 10 5; do
  echo "=== TX at ${BW} MHz: devourer continuous, capture PSD ==="
  # TX_PWR: optional flat-TXAGC boost (e.g. TX_PWR=0x3f) — needed on the EU,
  # whose efuse-level TX can sit below the ambient floor at bench distance.
  sudo PID="${PID:-c812}" DEVOURER_CHANNEL="$CHANNEL" DEVOURER_NB_BW="$BW" \
      DEVOURER_TX_PWR="${TX_PWR:-}" \
      SECS=24 tests/jaguar3_devourer_run.sh \
      build/txdemo 24 > "$OUT/dev_${BW}.log" 2>&1 &
  H=$!
  sleep 11   # power-on -> DLFW -> replay -> narrowband re-clock -> TX
  probe "tx${BW}" "$OUT/tx_${BW}.npy"
  grep -m1 "re-clock" "$OUT/dev_${BW}.log" | sed 's/^/    /' || \
    echo "    (no re-clock log for ${BW} — check dev log)"
  wait "$H" 2>/dev/null
  sleep 2
done

echo "=== differential-PSD occupied bandwidth ==="
python3 - "$OUT" "$RATE" <<'PY'
import sys, glob, numpy as np
out, rate = sys.argv[1], float(sys.argv[2])
base = np.load(f"{out}/baseline.npy"); bf, bp = base[0], base[1]

def occ_bw(label):
    a = np.load(f"{out}/{label}.npy"); f, p = a[0], a[1]
    diff = p - bp                       # isolate devourer's emission
    diff = np.clip(diff, 0, None)
    if diff.max() <= 0:
        return None, 0.0
    # Smooth, then take the band carrying 99% of the *difference* power.
    k = 33
    sm = np.convolve(diff, np.ones(k)/k, mode='same')
    tot = sm.sum()
    csum = np.cumsum(sm)
    lo = np.searchsorted(csum, 0.005*tot)
    hi = np.searchsorted(csum, 0.995*tot)
    bw = f[min(hi, len(f)-1)] - f[max(lo,0)]
    # also -10 dB width about the peak for a second estimate. CONTIGUOUS run
    # containing the peak — first/last-above-threshold bridges the DUT lobe
    # with any ambient burst elsewhere in the span and inflates the width.
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

# The -10 dB-about-peak width is the clean discriminator for an OFDM signal with
# skirts on a noisy band; occ99 of the difference is contaminated by ambient AP
# fluctuation, so it's reported only for reference.
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
