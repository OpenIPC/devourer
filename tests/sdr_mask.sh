#!/usr/bin/env bash
# sdr_mask.sh — stitched spectral-mask measurement around a DUT flood.
#
# The B210's usable span cannot hold a signal and its far skirts in one
# capture with enough dynamic range, so this stitches three tunes: one
# centered (the in-band reference), one high and one low (the skirts), the
# skirt tunes captured twice — at the base gain and at base+20 dB — with the
# actual gain delta measured on the skirt overlap. Controls are ENFORCED, not
# advisory: the run fails (nonzero exit, no mask table) when the
# center-vs-skirt stitch disagrees by more than STITCH_TOL_DB, when a gain
# transfer lands more than GAIN_TOL_DB away from the configured step
# (compression), or when any capture/analysis stage fails. The in-band
# reference is taken from a 5-bin median-smoothed PSD (same spur rejection
# sdr_obw.py applies to its own dBr reference).
#
# WIDTH=20 (default): 25 Msps captures, tunes at +/-15 MHz, mask points
#   +/-11/20/25 MHz. WIDTH=40: 50 Msps captures, tunes at +/-24 MHz, mask
#   points +/-21/30/35/40 MHz.
# The dynamic-range floor still bounds what is provable: on the RTL8733B
# qualification runs the 802.11n -40 dBr far points stayed out of reach.
#
#   sudo TX_PID=0xb733 CH=149 FREQ_MHZ=5745 tests/sdr_mask.sh
#   sudo TX_PID=0xb733 CH=149 FREQ_MHZ=5755 WIDTH=40 TX_EXTRA="DEVOURER_HOP_BW=40" tests/sdr_mask.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
TX_VID=${TX_VID:-0x0bda}
TX_PID=${TX_PID:-0xb733}
CH=${CH:-149}
FREQ_MHZ=${FREQ_MHZ:-5745}
WIDTH=${WIDTH:-20}
RATE=${RATE:-MCS7/$WIDTH}
TX_EXTRA=${TX_EXTRA:-}
GAIN=${GAIN:-40}
STITCH_TOL_DB=${STITCH_TOL_DB:-2}
GAIN_TOL_DB=${GAIN_TOL_DB:-1.5}
OUT=${OUT:-/tmp/devourer-sdr-mask/$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"
if [ "$WIDTH" = 40 ]; then
  # Tunes at +/-24 MHz: the skirt capture must still contain several MHz of
  # in-band signal or the ON gate has nothing to key on (at +/-30 MHz tunes a
  # 40 MHz signal leaves ~1.5 MHz of edge in span and every block reads OFF).
  # All captures run at the same 50 Msps (mixing rates shifts PSD-per-bin by
  # the bin-width ratio and breaks the stitch), and the deep step is +10 dB —
  # the skirt tunes hold half the signal, and +20 dB compresses the front end
  # (both were caught by the enforced controls, not guessed).
  CSPAN=50e6; SSPAN=50e6; TUNE_MHZ=${TUNE_MHZ:-24}
  DEEP_GAIN=${DEEP_GAIN:-50}
  OVL="18e6 20e6"; PTS="21e6 30e6 35e6 40e6"; REFW=16e6
else
  CSPAN=25e6; SSPAN=25e6; TUNE_MHZ=${TUNE_MHZ:-15}
  DEEP_GAIN=${DEEP_GAIN:-60}
  OVL="5e6 8e6";  PTS="11e6 20e6 25e6"; REFW=8e6
fi
TXPID=""
cleanup() { [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null; pkill -x txdemo 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM
die() { echo "[sdr-mask] FATAL: $*" >&2; exit 2; }

env DEVOURER_VID="$TX_VID" DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
  $TX_EXTRA DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PAYLOAD_BYTES=1500 \
  DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn \
  "$REPO/build/txdemo" >/dev/null 2>"$OUT/tx.stderr" & TXPID=$!
sleep 6
kill -0 "$TXPID" 2>/dev/null || die "flood died at bring-up — $OUT/tx.stderr"

cap() { # name freq_mhz gain span
  local line
  line=$(python3 "$HERE/sdr_obw.py" --freq "$(($2 * 1000000))" --rate "$4" \
    --secs 3 --gain "$3" --dump-psd "$OUT/$1.csv" 2>"$OUT/$1.stderr") \
    || die "capture $1 failed: $line — $OUT/$1.stderr"
  echo "$1: $line"
}
cap c        "$FREQ_MHZ"                  "$GAIN"      "$CSPAN"
cap hi       "$((FREQ_MHZ + TUNE_MHZ))"   "$GAIN"      "$SSPAN"
cap lo       "$((FREQ_MHZ - TUNE_MHZ))"   "$GAIN"      "$SSPAN"
cap hi_deep  "$((FREQ_MHZ + TUNE_MHZ))"   "$DEEP_GAIN" "$SSPAN"
cap lo_deep  "$((FREQ_MHZ - TUNE_MHZ))"   "$DEEP_GAIN" "$SSPAN"
kill -TERM "$TXPID" 2>/dev/null; wait "$TXPID" 2>/dev/null; TXPID=""

# shellcheck disable=SC2086 — OVL/PTS are deliberate multi-value lists
python3 - "$OUT" "$TUNE_MHZ" "$((DEEP_GAIN - GAIN))" "$STITCH_TOL_DB" \
    "$GAIN_TOL_DB" "$REFW" $OVL $PTS <<'EOF' || die "stitch controls failed"
import csv, statistics, sys
out, tune = sys.argv[1], float(sys.argv[2]) * 1e6
gstep, stol, gtol, refw = (float(sys.argv[3]), float(sys.argv[4]),
                           float(sys.argv[5]), float(sys.argv[6]))
ovl = [float(x) for x in sys.argv[7:9]]
pts = [float(x) for x in sys.argv[9:]]

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

ok = True
# Control 1: center-vs-skirt stitch on the overlap points.
for o in ovl:
    for d, sgn, tag in ((hi, 1, "hi"), (lo, -1, "lo")):
        a, b = level(c, sgn * o), level(d, sgn * o)
        if a is None or b is None or abs(a - b) > stol:
            print(f"CONTROL FAIL: stitch c-vs-{tag} at {sgn*o/1e6:+.0f} MHz: "
                  f"{'n/a' if a is None or b is None else f'{a-b:+.1f} dB'} "
                  f"(tol {stol})")
            ok = False
# Control 2: gain transfer within tolerance of the configured step.
g = {}
for base, deep, sgn, tag in ((hi, hid, 1, "hi"), (lo, lod, -1, "lo")):
    t = level(deep, sgn * ovl[1]) - level(base, sgn * ovl[1])
    g[tag] = t
    if abs(t - gstep) > gtol:
        print(f"CONTROL FAIL: gain transfer {tag} {t:+.1f} dB vs step "
              f"{gstep:+.0f} (tol {gtol}) — compression, distrust the run")
        ok = False
if not ok:
    sys.exit(1)
print(f"controls: stitch within {stol} dB, gain transfer hi {g['hi']:+.1f} / "
      f"lo {g['lo']:+.1f} dB")

# Spur-safe in-band reference: 5-point median smoothing, then max (the same
# rejection sdr_obw.py applies to its own dBr reference).
inband = sorted((f, v) for f, v in c.items() if abs(f) < refw)
vals = [v for _, v in inband]
sm = [statistics.median(vals[max(0, i - 2):i + 3]) for i in range(len(vals))]
ref = max(sm)

for off in [p * s for p in pts for s in (1, -1)]:
    src, gg = (c, 0.0) if abs(off) < tune - 12e6 or abs(off) <= refw + 4e6 \
        else ((hid, g["hi"]) if off > 0 else (lod, g["lo"]))
    v = level(src, off)
    print(f"offset {off/1e6:+5.0f} MHz: {v - gg - ref:6.1f} dBr"
          if v is not None else
          f"offset {off/1e6:+5.0f} MHz: outside capture span")
EOF
echo "[sdr-mask] PSD dumps: $OUT"
