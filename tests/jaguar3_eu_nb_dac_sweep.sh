#!/usr/bin/env bash
# Empirically map the 8822e DAC-divider field (0x9b4[10:8]) in narrowband mode
# on the RTL8812EU: sweep DEVOURER_NB_DAC codes at a fixed NB_BW and measure
# what actually reaches the air (differential PSD vs silent baseline).
#
# Motivation: the field is undocumented ("no even definitions in .h files" —
# OpenHD), the 8822c and 8822e vendor drivers use different codes for the same
# nominal clock, and the vendor-native 5 MHz code must be verified on real
# silicon before trusting it.
#
#   sudo tests/jaguar3_eu_nb_dac_sweep.sh            # 5 MHz, codes 1-6
#   sudo NB_BW=10 CODES="3 4 5" tests/jaguar3_eu_nb_dac_sweep.sh
set -u
cd "$(dirname "$0")/.."

CHANNEL=${CHANNEL:-44}
FREQ=${FREQ:-5220e6}
RATE=46.08e6
NB_BW=${NB_BW:-5}
CODES=${CODES:-"1 2 3 4 5 6"}
OUT=/tmp/j3_eu_dac_sweep
rm -rf "$OUT"; mkdir -p "$OUT"

probe() { # label  psd_out
  python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 6e6 --label "$1" --psd-out "$2" 2>&1 | grep '\[sdr' || true
}

echo "=== baseline: a81a silent (ambient PSD) ==="
probe baseline "$OUT/baseline.npy"

for CODE in $CODES; do
  echo "=== NB_BW=${NB_BW} DAC code ${CODE} ==="
  sudo PID=a81a DEVOURER_CHANNEL="$CHANNEL" DEVOURER_NB_BW="$NB_BW" \
      DEVOURER_NB_DAC="$CODE" DEVOURER_TX_PWR="${TX_PWR:-0x3f}" SECS=24 \
      tests/jaguar3_devourer_run.sh build/txdemo 24 \
      > "$OUT/dev_${CODE}.log" 2>&1 &
  H=$!
  sleep 11
  probe "dac${CODE}" "$OUT/dac_${CODE}.npy"
  grep -m1 -E "NB_DAC override" "$OUT/dev_${CODE}.log" | sed 's/^/    /' || \
    echo "    (override log missing — check $OUT/dev_${CODE}.log)"
  wait "$H" 2>/dev/null
  grep -m1 "TX aborting" "$OUT/dev_${CODE}.log" | sed 's/^/    !! /' || true
  echo "    bulks: $(grep -c 'bulk_send EP 5 OK' "$OUT/dev_${CODE}.log")"
  sleep 2
done

echo "=== differential-PSD per DAC code ==="
python3 - "$OUT" "$CODES" <<'PY'
import sys, numpy as np
out, codes = sys.argv[1], sys.argv[2].split()
base = np.load(f"{out}/baseline.npy"); bf, bp = base[0], base[1]

print(f"{'code':>5} {'lobe?':>6} {'-10dB BW(MHz)':>14} {'peak vs noise(dB)':>18} {'peak@(MHz off ctr)':>19}")
for c in codes:
    a = np.load(f"{out}/dac_{c}.npy"); f, p = a[0], a[1]
    diff = np.clip(p - bp, 0, None)
    k = 33
    sm = np.convolve(diff, np.ones(k)/k, mode='same')
    if sm.max() <= 0:
        print(f"{c:>5} {'no':>6}")
        continue
    ipk = int(np.argmax(sm)); pk = sm[ipk]
    med = np.median(sm[sm > 0]) if (sm > 0).any() else 1e-30
    snr_db = 10*np.log10(pk/med) if med > 0 else 0.0
    above = np.where(sm >= pk/10.0)[0]
    bw10 = (f[above[-1]] - f[above[0]]) if len(above) else 0.0
    span = f[-1] - f[0]
    # a real lobe: narrow (not the whole span) and well above the diff floor
    lobe = "YES" if (bw10 < 0.6*span and snr_db > 8.0) else "no"
    fctr = (f[0] + f[-1]) / 2
    print(f"{c:>5} {lobe:>6} {bw10/1e6:>14.2f} {snr_db:>18.1f} {(f[ipk]-fctr)/1e6:>+19.2f}")
PY
echo "PSDs kept in $OUT"
