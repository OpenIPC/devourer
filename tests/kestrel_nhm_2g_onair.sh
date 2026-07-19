#!/usr/bin/env bash
# Validate the band-gated Kestrel active NHM floor (DEVOURER_RX_NOISE_FLOOR ->
# RxEnergy.abs_noise_floor_dbm, emitted in rx.quality as abs_noise_floor_dbm):
#   - C8852C: NHM emitted on 2.4 GHz (validated correct), NULL on 5 GHz (the AGC
#     gain-lock reads ~24 dB high there, so it falls back to the passive floor).
#   - C8852B: NHM emitted on both bands (unchanged).
#
# Usage: sudo -v && tests/kestrel_nhm_2g_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${NHM2G_OUT:-/tmp/devourer-nhm-2g}"; mkdir -p "$OUT"
CH_24="${CH_24:-6}" CH_5="${CH_5:-36}" DUR="${DUR:-8}"
VID=0x35bc; C_PID=0x0101; B_PID=0x0108

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
unbind() { local pid="$1" d p i
  for d in /sys/bus/usb/devices/*/idProduct; do p=$(cat "$d" 2>/dev/null) || continue
    [ "$p" = "${pid#0x}" ] || continue
    for i in "$(dirname "$d")":*; do
      [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
    done; done; }

cat >"$OUT/med.py" <<'PYEOF'
import json,sys,statistics as st
v=[]
for l in sys.stdin:
    try: x=json.loads(l).get("abs_noise_floor_dbm")
    except: continue
    if isinstance(x,(int,float)): v.append(x)
print(f"{int(st.median(v))}" if v else "null")
PYEOF

nhm() { # $1=pid $2=ch -> median abs_noise_floor_dbm or "null"
  unbind "$1"
  sudo env DEVOURER_VID="$VID" DEVOURER_PID="$1" DEVOURER_CHANNEL="$2" \
    DEVOURER_RXQUALITY=1 DEVOURER_RX_NOISE_FLOOR=1 DEVOURER_RX_ENERGY_MS=500 \
    timeout "$DUR" "$ROOT/build/rxdemo" 2>/dev/null | grep '"ev":"rx.quality"' \
    | python3 "$OUT/med.py"; }

echo "== building rxdemo =="; cmake --build "$ROOT/build" -j --target rxdemo >/dev/null || exit 1
printf "\n%-8s %-16s %-16s\n" DIE "2.4G(ch$CH_24)" "5G(ch$CH_5)"
c24="skip"; c5="skip"; b24="skip"; b5="skip"
if plugged "$VID" "$C_PID"; then c24=$(nhm "$C_PID" "$CH_24"); c5=$(nhm "$C_PID" "$CH_5"); fi
if plugged "$VID" "$B_PID"; then b24=$(nhm "$B_PID" "$CH_24"); b5=$(nhm "$B_PID" "$CH_5"); fi
printf "%-8s %-16s %-16s\n" C8852C "$c24" "$c5"
printf "%-8s %-16s %-16s\n" C8852B "$b24" "$b5"

echo
python3 - "$c24" "$c5" "$b24" "$b5" <<'PYEOF'
import sys
c24,c5,b24,b5=sys.argv[1:5]
def num(x):
    try: return int(x)
    except: return None
ok=True
n=num(c24)
if n is not None and -100<=n<=-80: print(f"  C8852C 2.4G NHM {c24} dBm -> PASS (emitted, plausible)")
else: print(f"  C8852C 2.4G NHM {c24} -> FAIL (expected a plausible -80..-100)"); ok=False
if c5=="null": print(f"  C8852C 5G NHM null -> PASS (gated off, uses passive floor)")
else: print(f"  C8852C 5G NHM {c5} -> FAIL (expected null; the 5G gain-lock)"); ok=False
if b24!="skip":
    print(f"  C8852B 2.4G {b24} / 5G {b5} (reference; expect both emitted)")
print("  =>", "ALL PASS" if ok else "CHECK ABOVE")
PYEOF
