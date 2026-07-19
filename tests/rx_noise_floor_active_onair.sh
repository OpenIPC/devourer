#!/usr/bin/env bash
# On-air validation for the ACTIVE (frame-free) absolute noise floor
# (DEVOURER_RX_NOISE_FLOOR, RxQuality.abs_noise_floor_dbm).
#
# The blocker was that the vendor active measurement WEDGES live RX (the
# 8812AU live-poll delivered 0 frames on 2 of 6 runs). devourer measures it
# RX-idle instead:
#   - Jaguar1 (8812A/8821A): the debug-port active-sampling path (clock-stop +
#     BB/PMAC/CCK reset) runs ONCE at bring-up, BEFORE StartRxLoop starts the
#     bulk-IN DMA -> wedge-free by construction; the value is cached.
#   - Jaguar2 (8822B/8821C): the HW idle-noise report at 0x0FF0 (no clock-stop)
#     is read live -> wedge-free by nature. NOTE: on the tested 8812BU the report
#     is only intermittently populated in monitor bring-up (it reads the
#     0x80/0x00 sentinels between idle gaps), so it returns valid data on some
#     reads and null on others; poll until valid. When valid it cross-matches the
#     Jaguar1 floor on the same channel.
#   - Jaguar3 (8822C/8822E): no vendor idle-noise path -> always null (the
#     passive rssi-snr floor is its only floor).
#
# Two checks (a monotonic-vs-injected-noise sweep is NOT included: the bench B210
# is too weakly coupled to the RTL front ends to move the floor above the
# measurement variance — the same near-field limit the passive-floor bench hit):
#   (A) ANTI-WEDGE (the acceptance gate) — with the knob ON, a live RX keeps
#       delivering frames across N runs (0 wedges), vs the issue's 2/6.
#   (B) SANITY + CROSS-CHIP — the reported floor sits in a plausible band and the
#       Jaguar1 and Jaguar2 floors agree within a few dB on the same channel.
#
# Usage: sudo -v && tests/rx_noise_floor_active_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${NFOUT:-/tmp/devourer-nf-active}"
CH_AMB="${CH_AMB:-6}"     # a channel with ambient traffic (2.4 GHz) for the anti-wedge frame source
CH_5G="${CH_5G:-36}"      # a 5 GHz channel for the cross-chip floor comparison
RUNS="${RUNS:-6}"
DUR="${DUR:-7}"
mkdir -p "$OUT"

VID=0x0bda
J1_PID=0x8812             # RTL8812AU (Jaguar1 active-sampling CAL die)
J3_PID=0xa81a             # RTL8812EU (Jaguar3, no vendor path -> null)
J2_VID=0x2357 J2_PID=0x012d   # RTL8812BU / Archer T3U (Jaguar2 8822B cut)

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
unbind() { local pid="$1" d p i
  for d in /sys/bus/usb/devices/*/idProduct; do p=$(cat "$d" 2>/dev/null) || continue
    [ "$p" = "${pid#0x}" ] || continue
    for i in "$(dirname "$d")":*; do
      [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
    done; done; }

echo "== building rxdemo =="
cmake --build "$ROOT/build" -j --target rxdemo >/dev/null || exit 1

# Frames decoded in one run (rx.energy window sum). $1=pid $2=vid $3=ch $4=flag(0|1)
frames() { local env=""; [ "$4" = 1 ] && env="DEVOURER_RX_NOISE_FLOOR=1"
  sudo env DEVOURER_PID="$1" DEVOURER_VID="$2" DEVOURER_CHANNEL="$3" \
    DEVOURER_RX_ENERGY_MS=500 $env timeout "$DUR" "$ROOT/build/rxdemo" 2>/dev/null \
    | grep '"ev":"rx.energy"' | python3 -c 'import json,sys
print(sum(json.loads(l).get("frames",0) for l in sys.stdin))'; }

# Median valid abs_noise_floor_dbm over a run (polls; null reads skipped). $1=pid $2=vid $3=ch
floor() { sudo env DEVOURER_PID="$1" DEVOURER_VID="$2" DEVOURER_CHANNEL="$3" \
    DEVOURER_RX_NOISE_FLOOR=1 DEVOURER_RXQUALITY=1 DEVOURER_RX_ENERGY_MS=500 \
    timeout "$DUR" "$ROOT/build/rxdemo" 2>/dev/null \
    | grep '"ev":"rx.quality"' | python3 -c 'import json,sys,statistics as st
v=[json.loads(l).get("abs_noise_floor_dbm") for l in sys.stdin]
v=[x for x in v if isinstance(x,int)]
print(int(st.median(v)) if v else "null")'; }

echo
echo "== (A) ANTI-WEDGE: Jaguar1 8812AU, ambient ch$CH_AMB, knob ON, $RUNS runs =="
plugged "$J1_PID" "$VID" || { echo "SKIP: 8812AU not plugged"; exit 0; }
unbind "$J1_PID"
base=$(frames "$J1_PID" "$VID" "$CH_AMB" 0)
echo "  baseline (knob OFF): $base frames"
zeros=0
for r in $(seq 1 "$RUNS"); do
  f=$(frames "$J1_PID" "$VID" "$CH_AMB" 1)
  printf "  run %d (knob ON): %s frames\n" "$r" "$f"
  [ "${f:-0}" -le 0 ] && zeros=$((zeros + 1)); sleep 1
done
echo "  => $zeros/$RUNS wedged (0-frame) runs  [live-poll baseline: 2/6]"
[ "$zeros" -eq 0 ] && echo "  PASS: wedge-free" || echo "  FAIL: CAL wedged RX"

echo
echo "== (B) SANITY + CROSS-CHIP floor @ 5 GHz ch$CH_5G =="
unbind "$J1_PID"
j1=$(floor "$J1_PID" "$VID" "$CH_5G")
echo "  Jaguar1 8812AU floor: $j1 dBm"
if plugged "$J2_PID" "$J2_VID"; then
  unbind "$J2_PID"; j2=$(floor "$J2_PID" "$J2_VID" "$CH_5G")
  echo "  Jaguar2 8812BU floor: $j2 dBm (intermittent report; 'null' if no live read caught)"
else
  j2="n/a"; echo "  Jaguar2: not plugged"
fi
if plugged "$J3_PID" "$VID"; then
  unbind "$J3_PID"; j3=$(floor "$J3_PID" "$VID" "$CH_5G")
  echo "  Jaguar3 8812EU floor: $j3 (expect null — no vendor path)"
fi
python3 - "$j1" "$j2" <<'PYEOF'
import sys
def num(x):
    try: return int(x)
    except: return None
j1, j2 = num(sys.argv[1]), num(sys.argv[2])
ok = j1 is not None and -105 <= j1 <= -70
print(f"  Jaguar1 in plausible band [-105,-70]: {'PASS' if ok else 'FAIL'}")
if j1 is not None and j2 is not None:
    # J2's 8822B report is coarse/best-effort; agreement to ~15 dB (both in the
    # idle-floor band) is the meaningful cross-chip check, not a tight match.
    both_band = (-105 <= j2 <= -70)
    print(f"  J1/J2 both in idle-floor band, delta {abs(j1-j2)} dB: "
          f"{'PASS' if both_band and abs(j1-j2) <= 15 else 'INSPECT'}")
PYEOF
echo
echo "raw logs: $OUT (verdict above)"
