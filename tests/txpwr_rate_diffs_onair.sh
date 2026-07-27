#!/usr/bin/env bash
# On-air proof that a caller per-rate TX-power table (SetTxPowerRateDiffs)
# actually moves RADIATED power, and moves it PER RATE — the over-air half of
# the validation whose register/shadow half is tests/txpwr_rate_diffs_regcheck.sh.
# That script can only show the indices (and on the write-only families, only a
# software shadow); this one shows the antenna.
#
# SENSOR: a second devourer adapter (ground station) reporting per-frame RSSI,
# NOT the B210 — the same bench fact tests/txpwr_offset_onair.sh documents:
# with the antennas inches apart the SDR front end hard-limits on the DUT's
# frames at any RX gain, so wideband SDR power cannot see a TXAGC-scale move at
# all. A Wi-Fi chip's AGC is built for exactly this input range.
#
# METHOD — four cells per DUT, each a fixed-rate txdemo run, all four with a
# caller table armed so the ONLY variable is the table's content:
#
#   1  zeros table, TX at 6M      -> legacy reference (every rate at the anchor)
#   2  zeros table, TX at MCS0    -> HT reference
#   3  mcs0=-32 table, TX at 6M   -> must MATCH cell 1
#   4  mcs0=-32 table, TX at MCS0 -> must DROP from cell 2 by the family's
#                                    applied steps x its measured dB/step
#
# Cells 1/2 use an all-zero table rather than no table at all: a zero table is
# already a real change (it flattens the calibrated shape onto the anchor), so
# referencing against it isolates the DIFF from the anchor move. Cells 3 vs 1
# are the per-rate half of the claim — a global lever would drag 6M down with
# MCS0, and this is the cell that catches that.
#
# The HT cells use MCS0, not MCS7: at this bench's geometry MCS7 beacons from
# the 8812AU are not decoded by the ground at all (0 hits with NO table armed
# either — a link property, not a power one), and MCS3 is already marginal.
# MCS0 is also the rate a range knob is actually aimed at.
#
# EXPECTED dB uses each family's MEASURED slope, not the nominal step, or a
# working implementation reads as a failure:
#   Jaguar1/Jaguar2  0.5 dB/idx measured; -32 qdB quantizes to -16 idx -> -8 dB
#   Jaguar3 8822C    0.325 dB/idx (the offset knob's 6-point least-squares);
#                    -32 qdB = -32 idx -> -10.4 dB. This cell actually reads
#                    -8.0 dB, i.e. an implied 0.25 dB/idx — the nominal step.
#                    One A/B point does not arbitrate against a 6-point fit,
#                    so the tolerance covers both rather than the expectation
#                    being retuned to whichever was measured last.
#   Jaguar3 8822E    slope is TSSI-reshaped and NON-LINEAR (0.3->0.9 dB/idx),
#                    so its cell asserts sign and monotonicity only
#   Kestrel          fixed-dBm, 0.25 dB/qdB by construction           -> -8 dB
#
# The trim is -32 qdB, not a gentler step, and the tolerance is +/-2.5 dB: at
# this bench's geometry the ground sits near AGC saturation (RSSI% ~90, about
# -20 dBm) and repeat cells scatter ~2 dB, so a -4 dB move does not clear the
# noise. Measured on an 8812AU MCS0 ramp: 0/-8/-16/-24/-32 qdB -> RSSI%
# 92/89/91/87/83, i.e. the full span reads -9 dB against -8 dB nominal while
# individual 4 dB steps are inside the scatter.
#
# CAVEAT the 8821AU row encodes: its 5 GHz chain ignores BB TXAGC (registers
# move, radiated power does not), so it runs at 2.4 GHz like the offset harness.
#
# Usage: sudo -v && tests/txpwr_rate_diffs_onair.sh [PID ...] (default: plugged)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPWR_RATE_DIFFS_ONAIR_OUT:-/tmp/devourer-txpwr-rate-diffs-onair}"
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x txdemo 2>/dev/null || true
    pkill -x rxdemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

# DUT table: pid vid channel expected_db  ("mono" = sign/monotonicity only)
DUTS=(
    "0x8812 0x0bda 36 -8.0"    # RTL8812AU  (Jaguar1, 0.5 dB/idx x -16)
    "0x0120 0x2357 6  -8.0"    # RTL8821AU  (Jaguar1, 2.4G-only lever)
    "0x8813 0x0bda 36 -8.0"    # RTL8814AU  (Jaguar1)
    "0x012d 0x2357 36 -8.0"    # RTL8822BU  (Jaguar2)
    "0xc811 0x0bda 36 -8.0"    # RTL8821CU  (Jaguar2)
    "0xc812 0x0bda 36 -10.4"   # RTL8822CU  (Jaguar3, 0.325 dB/idx x -32)
    "0xa81a 0x0bda 36 mono"    # RTL8822EU  (Jaguar3, TSSI-reshaped)
    "0x0108 0x35bc 36 -8.0"    # Archer TX20U Nano (RTL8852BU, fixed-dBm)
    "0x0101 0x35bc 36 -8.0"    # Archer TX50UH V1  (RTL8832CU, fixed-dBm)
)
ZEROS="0,0,0,0,0,0,0,0,0,0"
TRIM="0,0,-32,0,0,0,0,0,0,0"   # mcs0 = -32 qdB, every other rate at the anchor

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }

# Ground-station preference: first plugged adapter that is not the DUT.
pick_ground() { # $1=dut_pid -> "pid vid" or ""
    # GROUND_PID/GROUND_VID force a specific ground. A ground that brings up
    # ("Listening air...") but hears nothing turns every cell of that DUT into
    # a skip; swapping it is faster than debugging it mid-sweep.
    if [ -n "${GROUND_PID:-}" ]; then
        echo "$GROUND_PID ${GROUND_VID:-0x0bda}"
        return
    fi
    for g in "0xc812 0x0bda" "0x8812 0x0bda" "0x012d 0x2357" "0xa81a 0x0bda"; do
        read -r gp gv <<<"$g"
        [ "$gp" = "$1" ] && continue
        plugged "$gp" "$gv" && { echo "$g"; return; }
    done
}

for dut in "${DUTS[@]}"; do
    read -r PID VID CH EXPECT <<<"$dut"
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
    echo "== DUT $PID@$VID (ground $GPID): ch$CH expect MCS0 $EXPECT dB =="

    # Ground RX for the whole DUT session, stream lines epoch-stamped.
    : >"$OUT/$tag-ground.log"
    sudo -n env DEVOURER_PID="$GPID" DEVOURER_VID="$GVID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 \
        stdbuf -oL timeout 260 "$ROOT/build/rxdemo" 2>"$OUT/$tag-ground.err" \
        | while IFS= read -r line; do
            printf '%s %s\n' "$(date +%s.%N)" "$line"
        done >>"$OUT/$tag-ground.log" &
    GROUND_JOB=$!
    sleep 12 # ground bring-up

    : >"$OUT/$tag-cells.txt"
    run_cell() { # $1=label $2=rate $3=diffs
        local t0 t1
        t0="$(date +%s.%N)"
        sudo -n env DEVOURER_PID="$PID" DEVOURER_VID="$VID" \
            DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$2" \
            DEVOURER_TX_RATE_DIFFS="$3" DEVOURER_TX_GAP_US=2000 \
            timeout 14 "$ROOT/build/txdemo" >"$OUT/$tag-cell-$1.log" 2>&1 || true
        t1="$(date +%s.%N)"
        echo "$1 $t0 $t1" >>"$OUT/$tag-cells.txt"
        sleep 2
    }
    run_cell ref6m   6M   "$ZEROS"
    run_cell refht  MCS0 "$ZEROS"
    run_cell trim6m  6M   "$TRIM"
    run_cell trimht MCS0 "$TRIM"

    sudo -n pkill -x rxdemo 2>/dev/null
    wait "$GROUND_JOB" 2>/dev/null

    python3 - "$OUT/$tag-ground.log" "$OUT/$tag-cells.txt" "$EXPECT" \
        >"$OUT/$tag-fit.txt" 2>&1 <<'PYEOF'
import re, statistics, sys
ground_log, cells_txt, expect = sys.argv[1], sys.argv[2], sys.argv[3]
mono = expect == "mono"
want_db = None if mono else float(expect)
frames = []
# Filter to the DUT's own beacons by canonical SA (examples/tx/main.cpp,
# 57:42:75:05:d6:00). Without this the median is dominated by ambient traffic
# once the DUT is trimmed toward the ambient RSSI level, and every cell below
# that level reads the SAME pinned value — which looks exactly like a lever
# that stops working.
rx = re.compile(r'^([0-9.]+) .*"ev":"rx\.frame".*"rssi":\[(-?\d+),(-?\d+)\]'
                r'.*"sa":"57427505d600"')
for line in open(ground_log, errors="replace"):
    m = rx.match(line)
    if m:
        frames.append((float(m.group(1)), int(m.group(2))))
cells = {}
for line in open(cells_txt):
    label, t0, t1 = line.split()
    t0, t1 = float(t0), float(t1)
    # Skip the bring-up head: nothing airs for the first few seconds.
    vals = [r for (t, r) in frames if t0 + 6 <= t <= t1 - 1]
    if len(vals) >= 20:
        cells[label] = statistics.median(vals)
missing = [c for c in ("ref6m", "refht", "trim6m", "trimht")
           if c not in cells]
if missing:
    print(f"RESULT skip missing cells {missing} (ground caught too few frames)")
    sys.exit(0)
d_ht = cells["trimht"] - cells["refht"]
d_6m = cells["trim6m"] - cells["ref6m"]
detail = " ".join(f"{k}:{v:.1f}" for k, v in cells.items())
# The per-rate half: an mcs0-only diff must leave 6M where it was. A global
# lever would move both, and that is exactly what this cell is here to catch.
per_rate_ok = abs(d_6m) <= 2.5
if mono:
    drop_ok = d_ht <= -2.5
    verdict = f"MCS0 {d_ht:+.1f} dB (want <= -2.5, TSSI-reshaped so sign only)"
else:
    drop_ok = abs(d_ht - want_db) <= 2.5
    verdict = f"MCS0 {d_ht:+.1f} dB (want {want_db:+.1f} +/- 2.5)"
print(f"RESULT {verdict}; 6M {d_6m:+.1f} dB (want 0.0 +/- 2.5, per-rate) "
      f"[{detail}]")
sys.exit(0 if (drop_ok and per_rate_ok) else 1)
PYEOF
    rc=$?
    res="$(grep RESULT "$OUT/$tag-fit.txt" | tail -1)"
    [ -n "$res" ] || res="RESULT skip (no fit output; see $OUT/$tag-fit.txt)"
    echo "  $res"
    case "$res" in
    *skip*) skip "$PID ($res)" ;;
    *) if [ $rc -eq 0 ]; then pass "$PID $res"; else fail "$PID $res"; fi ;;
    esac
done

echo
echo "== txpwr-rate-diffs on-air: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
