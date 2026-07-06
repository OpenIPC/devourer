#!/usr/bin/env bash
# Hardware validation for the runtime RX-chain mask (SetRxPathMask, issue #189).
# The 8814AU is the only plugged part with >2 chains to trade, so it is the RX;
# an 8812AU beacon on the same channel is the source. DEVOURER_RX_ALLPATHS emits
# per-chain RSSI (A,B,C,D) on <devourer-rxpath> lines — masking a chain drops
# its RSSI to the noise floor, which is the functional truth the check keys on.
#
# Two things proven:
#   functional  mask 0x33 (A+B) collapses chains C,D vs mask 0xFF (all up) —
#               the write reaches the right chains (0x808 byte 0).
#   sticky      the mask is applied via DEVOURER_RX_PATHS (which now routes
#               through SetRxPathMask) and the RX runs under DEVOURER_RX_SWEEP_FULL
#               (a full SetMonitorChannel per dwell — IQK saves/restores 0x808).
#               C,D staying collapsed across those channel sets proves the
#               re-apply; without it the first SetMonitorChannel reverts to
#               all-paths and C,D pop back up.
#
# Usage: sudo -v && tests/rx_path_mask_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${RXPATH_CHECK_OUT:-/tmp/devourer-rxpath-check}"
RX_PID=0x8813 RX_VID=0x0bda      # RTL8814AU (4T4R) as the receiver
TX_PID=0x8812 TX_VID=0x0bda      # RTL8812AU beacon source
CH=36
mkdir -p "$OUT"

PASS=0; FAIL=0; SKIP=0
pass() { echo "  PASS: $*"; PASS=$((PASS+1)); }
fail() { echo "  FAIL: $*"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP: $*"; SKIP=$((SKIP+1)); }

cleanup() {
    pkill -x WiFiDriverDemo 2>/dev/null || true
    pkill -x WiFiDriverTxDem 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
if ! plugged "$RX_PID" "$RX_VID"; then echo "SKIP: 8814AU (RX) not plugged"; exit 0; fi
if ! plugged "$TX_PID" "$TX_VID"; then echo "SKIP: 8812AU (TX) not plugged"; exit 0; fi

echo "== building =="
cmake --build "$ROOT/build" -j --target WiFiDriverDemo WiFiDriverTxDemo >/dev/null || exit 1

# Median per-chain RSSI (chains A B C D) over a run's <devourer-rxpath> lines,
# restricted to a channel by cross-referencing the sweep's dwell markers is
# overkill here: the beacon is on ONE channel, so only that channel's dwells
# catch frames — every <devourer-rxpath> line already belongs to it.
medians() { # $1=log -> "A B C D"
    python3 - "$1" <<'PYEOF'
import re, statistics, sys
ch = {0: [], 1: [], 2: [], 3: []}
rx = re.compile(r"<devourer-rxpath>.*\brssi=(-?\d+),(-?\d+),(-?\d+),(-?\d+)")
for line in open(sys.argv[1], errors="replace"):
    m = rx.search(line)
    if m:
        for i in range(4):
            ch[i].append(int(m.group(i + 1)))
out = []
for i in range(4):
    out.append(str(int(statistics.median(ch[i]))) if ch[i] else "nan")
print(" ".join(out), len(ch[0]))  # A B C D count
PYEOF
}

# --- beacon source: 8812AU on CH, canonical SA, for the whole session --------
run_rx() { # $1=mask $2=outfile $3=extra_env
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_GAP_US=1500 \
        timeout 40 "$ROOT/build/WiFiDriverTxDemo" >/dev/null 2>&1 &
    local txpid=$!
    sleep 4
    sudo -n env DEVOURER_PID="$RX_PID" DEVOURER_VID="$RX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_RX_ALLPATHS=1 DEVOURER_RX_PATHS="$1" $3 \
        timeout 28 "$ROOT/build/WiFiDriverDemo" >"$2" 2>&1 || true
    kill "$txpid" 2>/dev/null; wait "$txpid" 2>/dev/null
    sleep 2
}

echo "== run 1: mask 0xFF (all paths), static channel =="
run_rx 0xFF "$OUT/all.log" ""
read -r a0 b0 c0 d0 n0 <<<"$(medians "$OUT/all.log")"
echo "  medians A=$a0 B=$b0 C=$c0 D=$d0 ($n0)"

echo "== run 2: mask 0x33 (A+B only), static channel =="
run_rx 0x33 "$OUT/ab.log" ""
read -r a1 b1 c1 d1 n1 <<<"$(medians "$OUT/ab.log")"
echo "  medians A=$a1 B=$b1 C=$c1 D=$d1 ($n1)"

echo "== run 3: mask 0x33 under DEVOURER_RX_SWEEP_FULL (SetMonitorChannel/dwell) =="
run_rx 0x33 "$OUT/ab-sweep.log" "DEVOURER_RX_SWEEP=$CH,$((CH+4)) DEVOURER_RX_SWEEP_FULL=1 DEVOURER_RX_SWEEP_DWELL_MS=500"
read -r a2 b2 c2 d2 n2 <<<"$(medians "$OUT/ab-sweep.log")"
echo "  medians A=$a2 B=$b2 C=$c2 D=$d2 ($n2)"

# --- assertions --------------------------------------------------------------
lt() { [ "$1" != nan ] && [ "$2" != nan ] && [ "$1" -lt "$2" ] 2>/dev/null; }

# functional: at 0xFF all four chains report a real RSSI (Realtek RSSI is a
# positive dB-ish index, floor is single digits / 0); at 0x33 chains C,D drop
# well below their 0xFF level AND below the enabled A,B.
if [ "$n0" -lt 20 ] || [ "$n1" -lt 20 ]; then
    fail "too few frames caught (n_all=$n0 n_ab=$n1) — check the beacon/link"
else
    if lt "$c1" "$c0" && lt "$d1" "$d0" && lt "$c1" "$a1" && lt "$d1" "$b1"; then
        pass "functional: mask 0x33 collapsed C,D (0xFF C=$c0 D=$d0 -> 0x33 C=$c1 D=$d1; A,B stay up A=$a1 B=$b1)"
    else
        fail "functional: C,D did not collapse under 0x33 (0xFF C=$c0 D=$d0 / 0x33 C=$c1 D=$d1 A=$a1 B=$b1)"
    fi
fi

# sticky: under sweep-full (a full SetMonitorChannel per dwell), C,D must STAY
# collapsed — without the re-apply the channel set reverts 0x808 to all-paths
# and C,D would rise back to ~their 0xFF level.
if [ "$n2" -lt 20 ]; then
    fail "sweep run caught too few frames (n=$n2)"
elif lt "$c2" "$a2" && lt "$d2" "$b2" && lt "$c2" "$c0"; then
    pass "sticky: mask held across SetMonitorChannel dwells (C=$c2 D=$d2 vs enabled A=$a2 B=$b2, 0xFF C was $c0)"
else
    fail "sticky: C,D rose after a full channel set — re-apply not working (sweep C=$c2 D=$d2 A=$a2 B=$b2; 0xFF C=$c0)"
fi

echo
echo "== rx-path-mask check: PASS=$PASS FAIL=$FAIL SKIP=$SKIP =="
[ "$FAIL" -eq 0 ]
