#!/usr/bin/env bash
# On-air dB-per-step slope of the TXAGC index, per family — the over-air half
# of the runtime TX-power API validation. tests/txpwr_offset_regcheck.sh
# proves an offset moves the registers by exactly N index steps; this measures
# what one index step is WORTH on air, classifying each family's step against
# the nominal 0.5 dB (Jaguar1/2, 6-bit) vs 0.25 dB (Jaguar3, 7-bit ref).
# Offset truth follows by composition: -24 qdB = the measured slope x the
# register-verified step count.
#
# SENSOR: a second devourer adapter (ground station) reporting per-frame RSSI
# (dB-linear on Realtek phystatus) for the DUT's canonical-SA beacons — NOT
# the B210. Measured on this bench: with the antennas inches apart the B210's
# front end hard-limits on the DUT's frames at ANY RX gain (idx 8 and idx 56
# both read -56.9 dBFS at gain 0), so wideband SDR power cannot see the TXAGC
# slope at all; a WiFi chip's AGC is built for exactly this input range.
#
# Method: one FIXED-INDEX TxDemo process per point (DEVOURER_TX_PWR — the flat
# override riding the runtime API), 6 points spanning the family's range, the
# ground's median per-frame RSSI per cell, least-squares slope. Default cell
# channel is 5 GHz ch36, per-DUT overrides in the table (canonical-SA
# filtering keeps the ground blind to ambient frames either way).
#
# Usage: sudo -v && tests/txpwr_offset_onair.sh [PID ...] (default: plugged set)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPWR_ONAIR_OUT:-/tmp/devourer-txpwr-onair}"
CH=36
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x WiFiDriverTxDem 2>/dev/null || true
    pkill -x WiFiDriverDemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j --target WiFiDriverTxDemo WiFiDriverDemo >/dev/null || exit 1

# DUT table: pid vid ramp_start ramp_stop nominal_db_per_step channel
#  - 8821AU runs at ch6: its 5 GHz chain IGNORES BB TXAGC (measured flat at
#    ch36 across two grounds while registers move; 0.50 dB/idx exactly at
#    2.4 GHz) — the power lever is 2.4 GHz-only on that part.
#  - 8822EU nominal "tssi": its TSSI/kfree trims reshape the ref->power
#    transfer (measured 0.3->0.9 dB/idx across the range, ~0.55 avg), so the
#    cell asserts a working monotone lever (slope 0.15..1.0) instead of a
#    fixed step classification.
DUTS=(
    "0x8812 0x0bda 8 56 0.5 36"    # RTL8812AU  (Jaguar1)
    "0x0120 0x2357 8 56 0.5 6"     # RTL8821AU  (Jaguar1, 2.4G-only lever)
    "0x8813 0x0bda 8 56 0.5 36"    # RTL8814AU  (Jaguar1)
    "0x012d 0x2357 8 56 0.5 36"    # RTL8822BU  (Jaguar2)
    "0xc811 0x0bda 8 56 0.5 36"    # RTL8821CU  (Jaguar2)
    "0xc812 0x0bda 24 104 0.25 36" # RTL8822CU  (Jaguar3)
    "0xa81a 0x0bda 24 104 tssi 36" # RTL8822EU  (Jaguar3, TSSI-reshaped)
)
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

# Ground-station preference: 8822CU (RX-proven J3), else 8812AU — first
# plugged one that is not the DUT.
pick_ground() { # $1=dut_pid -> "pid vid" or ""
    for g in "0xc812 0x0bda" "0x8812 0x0bda" "0x012d 0x2357"; do
        read -r gp gv <<<"$g"
        [ "$gp" = "$1" ] && continue
        plugged "$gp" "$gv" && { echo "$g"; return; }
    done
}

for dut in "${DUTS[@]}"; do
    read -r PID VID START STOP NOMINAL DCH <<<"$dut"
    if [ "$#" -gt 0 ]; then
        want=0; for p in "$@"; do [ "$p" = "$PID" ] && want=1; done
        [ "$want" = "1" ] || continue
    fi
    if ! plugged "$PID" "$VID"; then
        skip "$PID@$VID not plugged"
        continue
    fi
    ground="$(pick_ground "$PID")"
    if [ -z "$ground" ]; then
        skip "$PID: no ground-station adapter available"
        continue
    fi
    read -r GPID GVID <<<"$ground"
    tag="${PID#0x}"
    CH="$DCH"
    echo "== DUT $PID@$VID (ground $GPID): ch$CH cells $START..$STOP (nominal $NOMINAL dB/idx) =="

    # Ground RX for the whole DUT session, stream lines epoch-stamped.
    : >"$OUT/$tag-ground.log"
    sudo -n env DEVOURER_PID="$GPID" DEVOURER_VID="$GVID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 \
        stdbuf -oL timeout 300 "$ROOT/build/WiFiDriverDemo" 2>"$OUT/$tag-ground.err" \
        | while IFS= read -r line; do
            printf '%s %s\n' "$(date +%s.%N)" "$line"
        done >>"$OUT/$tag-ground.log" &
    GROUND_JOB=$!
    sleep 12 # ground bring-up

    span=$((STOP - START))
    idxs=""
    for k in 0 1 2 3 4 5; do idxs="$idxs $((START + span * k / 5))"; done
    : >"$OUT/$tag-cells.txt"
    for idx in $idxs; do
        t0="$(date +%s.%N)"
        sudo -n env DEVOURER_PID="$PID" DEVOURER_VID="$VID" \
            DEVOURER_CHANNEL="$CH" DEVOURER_TX_PWR="$idx" \
            DEVOURER_TX_GAP_US=2000 \
            timeout 14 "$ROOT/build/WiFiDriverTxDemo" >"$OUT/$tag-cell$idx.log" 2>&1 || true
        t1="$(date +%s.%N)"
        echo "$idx $t0 $t1" >>"$OUT/$tag-cells.txt"
        sleep 2
    done
    sudo -n pkill -x WiFiDriverDemo 2>/dev/null
    wait "$GROUND_JOB" 2>/dev/null

    # Slope fit: per cell, median ground RSSI (chain A) of the canonical-SA
    # stream lines in [t0+6, t1-1] (bring-up transmits nothing at first).
    python3 - "$OUT/$tag-ground.log" "$OUT/$tag-cells.txt" "$NOMINAL" >"$OUT/$tag-fit.txt" 2>&1 <<'PYEOF'
import re, statistics, sys
ground_log, cells_txt = sys.argv[1], sys.argv[2]
tssi = sys.argv[3] == "tssi"
nominal = None if tssi else float(sys.argv[3])
frames = []
rx = re.compile(r"^([0-9.]+) .*<devourer-stream>.*\brssi=(-?\d+),(-?\d+)")
for line in open(ground_log, errors="replace"):
    m = rx.match(line)
    if m:
        frames.append((float(m.group(1)), int(m.group(2))))
pts = []
for line in open(cells_txt):
    idx, t0, t1 = line.split()
    idx, t0, t1 = int(idx), float(t0), float(t1)
    vals = [r for (t, r) in frames if t0 + 6 <= t <= t1 - 1]
    if len(vals) >= 20:
        pts.append((idx, statistics.median(vals)))
if len(pts) < 5:
    print(f"RESULT skip pts={len(pts)} (ground caught too few beacons)")
    sys.exit(0)
n = len(pts)
sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
slope = (n * sxy - sx * sy) / (n * sxx - sx * sx)
resid = [y - (sy / n + slope * (x - sx / n)) for x, y in pts]
rms = (sum(r * r for r in resid) / n) ** 0.5
span_db = slope * (pts[-1][0] - pts[0][0])
detail = " ".join(f"{x}:{y:.1f}" for x, y in pts)
if tssi:
    ok = 0.15 <= slope <= 1.0
    print(f"RESULT slope={slope:.3f} dB/idx (TSSI-reshaped lever; monotone "
          f"0.15..1.0) rms_resid={rms:.2f} dB span={span_db:.1f} dB pts={n} [{detail}]")
    sys.exit(0 if ok else 1)
klass = 0.5 if abs(slope - 0.5) < abs(slope - 0.25) else 0.25
print(f"RESULT slope={slope:.3f} dB/idx (nominal {nominal}, classified {klass}) "
      f"rms_resid={rms:.2f} dB span={span_db:.1f} dB pts={n} [{detail}]")
sys.exit(0 if klass == nominal and rms < 2.5 else 1)
PYEOF
    rc=$?
    res="$(grep RESULT "$OUT/$tag-fit.txt" | tail -1)"
    echo "  $res"
    case "$res" in
    *skip*) skip "$PID slope ($res)" ;;
    *) if [ $rc -eq 0 ]; then
           pass "$PID $res"
       else
           fail "$PID $res"
       fi ;;
    esac
done

echo
echo "== txpwr-offset on-air: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
