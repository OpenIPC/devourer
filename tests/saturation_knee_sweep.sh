#!/usr/bin/env bash
# Near-field saturation-knee sweep — the measurement behind the link-health
# classifier and the bench-testing guidance. Walks the TX-power flat index over
# its full range (the runtime knob DEVOURER_TX_PWR) while a ground adapter
# reports per-frame RSSI / SNR / EVM (<devourer-stream>). On a healthy link EVM
# improves monotonically as RSSI rises; at the front-end saturation knee it
# STOPS improving and reverses — more signal made it worse. This finds that
# knee and, as a side effect, calibrates the RSSI scale (does raw RSSI climb
# toward a ceiling as power rises?) and the EVM sign (which direction is good?).
#
# Two adapters, no SDR: TX = 8812AU, ground = a second devourer part. Keep them
# at a FIXED short bench distance (this is the near-field regime the classifier
# has to recognise).
#
# Output per index: <sat-knee> idx=N rssi=med snr=med evm=med frames=K
# plus a fitted summary: the EVM-optimal index (the linear operating point) and
# whether an EVM turnover (saturation) was observed.
#
# Usage: sudo -v && tests/saturation_knee_sweep.sh [tx_pid] [ground_pid]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${SATKNEE_OUT:-/tmp/devourer-sat-knee}"
CH="${CH:-36}"
TX_PID="${1:-0x8812}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${2:-0xc812}" GROUND_VID="${GROUND_VID:-0x0bda}"
# Full 6-bit index ladder (J1/J2 range). Jaguar3's ref is 7-bit but the demo's
# DEVOURER_TX_PWR clamps to the family; the ladder still spans low->high power.
IDXS="${IDXS:-4 10 16 22 28 34 40 46 52 58 63}"
mkdir -p "$OUT"

cleanup() {
    pkill -x WiFiDriverTxDem 2>/dev/null || true
    pkill -x WiFiDriverDemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
plugged "$TX_PID" "$TX_VID" || { echo "SKIP: TX $TX_PID not plugged"; exit 0; }
plugged "$GROUND_PID" "$GROUND_VID" || { echo "SKIP: ground $GROUND_PID not plugged"; exit 0; }

echo "== building =="
cmake --build "$ROOT/build" -j --target WiFiDriverTxDemo WiFiDriverDemo >/dev/null || exit 1

echo "== ground RX ($GROUND_PID) up on ch$CH =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 400 "$ROOT/build/WiFiDriverDemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

: >"$OUT/cells.txt"
for idx in $IDXS; do
    t0="$(date +%s.%N)"
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR="$idx" DEVOURER_TX_GAP_US=1500 \
        timeout 12 "$ROOT/build/WiFiDriverTxDemo" >/dev/null 2>&1 || true
    t1="$(date +%s.%N)"
    echo "$idx $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done
sudo -n pkill -x WiFiDriverDemo 2>/dev/null
wait "$GJ" 2>/dev/null

python3 - "$OUT/ground.log" "$OUT/cells.txt" <<'PYEOF'
import re, statistics, sys
frames = []  # (t, rssi0, snr0, evm0)
rx = re.compile(r"^([0-9.]+) .*<devourer-stream>.*\brssi=(-?\d+),-?\d+ "
                r"evm=(-?\d+),-?\d+ snr=(-?\d+),-?\d+")
for line in open(sys.argv[1], errors="replace"):
    m = rx.match(line)
    if m:
        frames.append((float(m.group(1)), int(m.group(2)),
                       int(m.group(4)), int(m.group(3))))  # t, rssi, snr, evm
pts = []
for line in open(sys.argv[2]):
    idx, t0, t1 = line.split(); idx, t0, t1 = int(idx), float(t0), float(t1)
    win = [(r, s, e) for (t, r, s, e) in frames if t0 + 6 <= t <= t1 - 1]
    if len(win) < 15:
        continue
    rssi = int(statistics.median(x[0] for x in win))
    snr = int(statistics.median(x[1] for x in win))
    evm = int(statistics.median(x[2] for x in win))
    pts.append((idx, rssi, snr, evm, len(win)))
    print(f"<sat-knee> idx={idx} rssi={rssi} snr={snr} evm={evm} frames={len(win)}")
if len(pts) < 4:
    print("RESULT insufficient points (ground caught too few frames)"); sys.exit(0)

# EVM sign is unknown a priori: pick the "best" as the extreme that co-occurs
# with the HIGHEST SNR (SNR direction is unambiguous — higher = better). Then
# the EVM-optimal index is where EVM is best; a turnover = EVM getting worse at
# the top of the power ladder while RSSI still climbs.
best_snr_pt = max(pts, key=lambda p: p[2])
lo_evm = min(p[3] for p in pts); hi_evm = max(p[3] for p in pts)
evm_lower_is_better = abs(best_snr_pt[3] - lo_evm) <= abs(best_snr_pt[3] - hi_evm)

rssi_lo, rssi_hi = pts[0][1], pts[-1][1]
rssi_rises = rssi_hi > rssi_lo
# best EVM index
if evm_lower_is_better:
    knee = min(pts, key=lambda p: p[3])
else:
    knee = max(pts, key=lambda p: p[3])
# turnover: at the top-power cell, is EVM worse than at the knee AND SNR worse?
top = pts[-1]
worse_evm = (top[3] > knee[3]) if evm_lower_is_better else (top[3] < knee[3])
turnover = worse_evm and top[0] > knee[0] and top[2] <= knee[2]

print(f"RESULT rssi span {rssi_lo}->{rssi_hi} ({'rises' if rssi_rises else 'flat/falls'} "
      f"with power); EVM {'lower' if evm_lower_is_better else 'higher'}=better; "
      f"EVM-optimal at idx={knee[0]} (rssi={knee[1]} snr={knee[2]} evm={knee[3]}); "
      f"saturation turnover={'YES' if turnover else 'no'} "
      f"(top idx={top[0]} rssi={top[1]} snr={top[2]} evm={top[3]})")
PYEOF
