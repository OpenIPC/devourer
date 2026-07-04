#!/usr/bin/env bash
# 5 MHz spectral A/B on the RTL8812EU (0bda:a81a, chip 8822e).
#
# Compares the SHIPPED 5 MHz DAC-divider code (0x9b4[10:8]=0x4, the 8822c
# value libc0607's rtl88x2eu 5mhz_bw branch adopted) against the 8822e vendor
# phydm's own table value (0x2), which is dead on real silicon (TX keys up
# and frames drain but nothing airs). Quantifies the worst out-of-lobe spur
# relative to the main lobe in a differential PSD (TX minus silent baseline):
#   shipped : DEVOURER_NB_BW=5                      (dac=0x4, this driver)
#   vendtbl : DEVOURER_NB_BW=5 DEVOURER_NB_DAC=0x2  (8822e vendor table)
# plus a 20 MHz reference run to confirm the wide channel is unaffected.
#
#   sudo tests/jaguar3_eu_5mhz_mirror_ab.sh
set -u
cd "$(dirname "$0")/.."

# CHANNEL/FREQ must agree (ch44=5220e6, ch140=5700e6). Default ch140 — a
# DFS channel that is quiet indoors; ch36-48 carry ambient APs whose bursts
# poison the differential PSD.
CHANNEL=${CHANNEL:-140}
FREQ=${FREQ:-5700e6}
RATE=46.08e6   # ±23 MHz of visibility around the 5 MHz lobe
OUT=/tmp/j3_eu_mirror
rm -rf "$OUT"; mkdir -p "$OUT"

probe() { # label  psd_out
  # Mean PSD on a quiet DFS channel (median suppresses the DUT's own bursty
  # frames; back-to-back TX overruns the 4x-slower 5 MHz PHY drain and the
  # demo aborts on NAKs — so paced TX + mean it is).
  python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 12e6 --label "$1" --psd-out "$2" 2>&1 \
      | grep '\[sdr' || true
}

run_case() { # name  nb_bw  nb_dac('' = default)
  local name=$1 bw=$2 dacovr=$3
  echo "=== case $name: NB_BW=$bw NB_DAC=${dacovr:-<native>} ==="
  # TX_PWR: flat TXAGC boost for SDR visibility (same for every case, so the
  # A/B stays fair). The EU's efuse-level TX is too weak for the differential
  # PSD at bench distance.
  sudo PID=a81a DEVOURER_CHANNEL="$CHANNEL" DEVOURER_NB_BW="$bw" \
      DEVOURER_NB_DAC="$dacovr" DEVOURER_TX_PWR="${TX_PWR:-0x3f}" SECS=24 \
      tests/jaguar3_devourer_run.sh build/WiFiDriverTxDemo 24 \
      > "$OUT/dev_${name}.log" 2>&1 &
  local H=$!
  sleep 11   # power-on -> DLFW -> init -> narrowband re-clock -> TX
  probe "$name" "$OUT/${name}.npy"
  sleep 5
  probe "${name}_b" "$OUT/${name}_b.npy"   # second look: catches mid-run death
  grep -m1 -E "re-clock|NB_DAC" "$OUT/dev_${name}.log" | sed 's/^/    /' || \
    echo "    (no re-clock log — check $OUT/dev_${name}.log)"
  wait "$H" 2>/dev/null
  grep -m1 "TX aborting" "$OUT/dev_${name}.log" | sed 's/^/    !! /' || true
  echo "    bulks: $(grep -c 'bulk_send EP 5 OK' "$OUT/dev_${name}.log")"
  sleep 2
}

echo "=== baseline: a81a silent (ambient PSD) ==="
probe baseline "$OUT/baseline.npy"

run_case shipped 5  ""
run_case vendtbl 5  "0x2"
run_case ref20   20 ""

echo "=== differential-PSD mirror analysis ==="
python3 - "$OUT" <<'PY'
import sys, numpy as np
out = sys.argv[1]
base = np.load(f"{out}/baseline.npy"); bf, bp = base[0], base[1]

def analyse(label, half_lobe_hz):
    a = np.load(f"{out}/{label}.npy"); f, p = a[0], a[1]
    diff = np.clip(p - bp, 0, None)
    k = 33
    sm = np.convolve(diff, np.ones(k)/k, mode='same')
    if sm.max() <= 0:
        return None
    ipk = int(np.argmax(sm)); fpk = f[ipk]; pk = sm[ipk]
    # -10 dB width about the peak (the M6 discriminator)
    above = np.where(sm >= pk/10.0)[0]
    bw10 = (f[above[-1]] - f[above[0]]) if len(above) else 0.0
    # worst spur OUTSIDE the main lobe (exclude ±half_lobe around the peak,
    # and the outer 10% of the span where the B210's own filter skirts live)
    span = f[-1] - f[0]
    guard = (np.abs(f - fpk) > half_lobe_hz) & \
            (f > f[0] + 0.05*span) & (f < f[-1] - 0.05*span)
    spur_db = None; fspur = None
    if guard.any() and sm[guard].max() > 0:
        j = np.flatnonzero(guard)[int(np.argmax(sm[guard]))]
        spur_db = 10*np.log10(sm[j]/pk); fspur = f[j]
    return dict(fpk=fpk, bw10=bw10, spur_db=spur_db, fspur=fspur)

print(f"{'case':>8} {'-10dB BW(MHz)':>14} {'worst spur(dB rel lobe)':>24} {'@offset(MHz)':>13}")
res = {}
for label, half in (("shipped", 4e6), ("vendtbl", 4e6), ("ref20", 12e6)):
    r = analyse(label, half)
    res[label] = r
    if r is None:
        print(f"{label:>8} {'n/a':>14}")
        continue
    sp = f"{r['spur_db']:.1f}" if r['spur_db'] is not None else "none"
    off = f"{(r['fspur']-r['fpk'])/1e6:+.2f}" if r['fspur'] is not None else "-"
    print(f"{label:>8} {r['bw10']/1e6:>14.2f} {sp:>24} {off:>13}")

n, o = res.get("shipped"), res.get("vendtbl")
if n and o and n["spur_db"] is not None and o["spur_db"] is not None:
    print(f"\nspur delta (vendtbl - shipped): {o['spur_db']-n['spur_db']:+.1f} dB")
PY
echo "PSDs kept in $OUT for offline inspection"
