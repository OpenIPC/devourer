#!/usr/bin/env bash
# sdr_mask.sh — stitched spectral-mask measurement around a DUT flood.
#
# The B210's usable span cannot hold a 20 MHz signal and its +/-30 MHz skirts
# in one capture with enough dynamic range, so this stitches three tunes:
# one centered (the in-band reference), one high and one low (the skirts),
# each captured twice — at the base gain and at base+20 dB — with the actual
# gain delta measured on the +/-8 MHz skirt overlap. A transfer within
# ~1 dB of +20 proves the front end stayed linear at the higher gain; a
# shortfall is compression and the run is not trustworthy. Offsets are
# reported in dBr against the in-band peak of the centered capture.
#
# The dynamic-range floor still bounds what is provable: on the RTL8733B
# qualification run the +/-20 and +/-25 MHz points resolved (-33/-35 dBr)
# and the 802.11n -40 dBr far point at +/-30 MHz remained out of reach.
#
#   sudo TX_PID=0xb733 CH=149 FREQ_MHZ=5745 tests/sdr_mask.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
TX_VID=${TX_VID:-0x0bda}
TX_PID=${TX_PID:-0xb733}
CH=${CH:-149}
FREQ_MHZ=${FREQ_MHZ:-5745}
RATE=${RATE:-MCS7/20}
GAIN=${GAIN:-40}
DEEP_GAIN=${DEEP_GAIN:-60}
TUNE_MHZ=${TUNE_MHZ:-15}
OUT=${OUT:-/tmp/devourer-sdr-mask/$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"
TXPID=""
cleanup() { [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null; pkill -x txdemo 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PAYLOAD_BYTES=1500 DEVOURER_TX_GAP_US=0 \
  DEVOURER_LOG_LEVEL=warn "$REPO/build/txdemo" >/dev/null 2>&1 & TXPID=$!
sleep 6

cap() { # name freq_mhz gain
  python3 "$HERE/sdr_obw.py" --freq "$(($2 * 1000000))" --rate 25e6 --secs 3 \
    --gain "$3" --dump-psd "$OUT/$1.csv" 2>/dev/null | grep sdr-obw \
    | sed "s/^/$1: /"
}
cap c        "$FREQ_MHZ"                  "$GAIN"
cap hi       "$((FREQ_MHZ + TUNE_MHZ))"   "$GAIN"
cap lo       "$((FREQ_MHZ - TUNE_MHZ))"   "$GAIN"
cap hi_deep  "$((FREQ_MHZ + TUNE_MHZ))"   "$DEEP_GAIN"
cap lo_deep  "$((FREQ_MHZ - TUNE_MHZ))"   "$DEEP_GAIN"
kill -TERM "$TXPID" 2>/dev/null; wait "$TXPID" 2>/dev/null; TXPID=""

python3 - "$OUT" "$TUNE_MHZ" <<'EOF'
import csv, sys
out, tune = sys.argv[1], float(sys.argv[2]) * 1e6

def load(name, center_off):
    d = {}
    for row in csv.DictReader(open(f"{out}/{name}.csv")):
        d[float(row["offset_hz"]) + center_off] = float(row["psd_db"])
    return d

def level(d, off, width=500e3):
    vals = [v for f, v in d.items() if abs(f - off) < width]
    return sum(vals) / len(vals) if vals else None

c = load("c", 0.0)
hi, lo = load("hi", tune), load("lo", -tune)
hid, lod = load("hi_deep", tune), load("lo_deep", -tune)
ref = max(v for f, v in c.items() if abs(f) < 8e6)
s1 = level(c, 5e6) - level(hi, 5e6)
s2 = level(c, 8e6) - level(hi, 8e6)
print(f"stitch check (center vs hi at +5/+8 MHz): {s1:+.1f} / {s2:+.1f} dB")
g_hi = level(hid, 8e6) - level(hi, 8e6)
g_lo = level(lod, -8e6) - level(lo, -8e6)
print(f"gain transfer (base -> deep): hi {g_hi:+.1f} dB, lo {g_lo:+.1f} dB "
      f"(a shortfall vs the gain step = compression, distrust the run)")
for off in (11e6, -11e6, 20e6, -20e6, 25e6, -25e6):
    src, g = (c, 0.0) if abs(off) <= 12e6 else \
             ((hid, g_hi) if off > 0 else (lod, g_lo))
    v = level(src, off)
    print(f"offset {off/1e6:+5.0f} MHz: {v - g - ref:6.1f} dBr"
          if v is not None else f"offset {off/1e6:+5.0f} MHz: n/a")
EOF
echo "PSD dumps: $OUT"
