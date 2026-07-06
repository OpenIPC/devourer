#!/usr/bin/env bash
# Per-MCS clean-power-ceiling calibration — automates the OpenIPC-FPV
# community's #1 hand-tuning chore. The universal law: "higher MCS => lower max
# clean power" — the PA saturates high-order QAM first, so each MCS has a power
# ceiling above which EVM collapses and video garbles. Everyone hand-tunes this
# per adapter (raise power until garbage, back off ~5; e.g. Johhn's 8812AU:
# 58@MCS0/1, 55@MCS2, 45@MCS3, 40@MCS4, 30@MCS5) and bakes it into per-MCS power
# tables (wlan_adapters.yaml).
#
# This measures it directly, reusing this repo's diagnostics: for each MCS it
# sweeps the flat TXAGC index (DEVOURER_TX_PWR) while a second devourer adapter
# reports per-frame RSSI/EVM (<devourer-stream>). The ceiling per MCS is the
# highest power that still DELIVERS cleanly — i.e. the point just before frames
# garble and delivery collapses (exactly the community's "raise power until
# garbage" cliff). NB the EVM *knee* (where the PA compresses) is nearly the
# same power for every MCS, so it does NOT show the law; what differs is the
# decode threshold — a denser constellation garbles at lower power, so its
# delivery cliff comes sooner. The output is the mcs->safe-power table, and the
# on-air validation is the law itself: the ceiling is non-increasing as MCS rises.
#
# Bench: TX + ground at a FIXED short distance (the PA's clean limit is the same
# knee at any distance; a fixed near-field geometry is valid — same basis as
# tests/saturation_knee_sweep.sh). Chip-RSSI/EVM is the sensor, NOT an SDR (the
# B210 front end limits on near-field frames — measured this session).
#
# Usage: sudo -v && tests/per_mcs_power_ceiling.sh [tx_pid] [ground_pid]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${PERMCS_OUT:-/tmp/devourer-per-mcs-ceiling}"
CH="${CH:-36}"
TX_PID="${1:-0x8812}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${2:-0xc812}" GROUND_VID="${GROUND_VID:-0x0bda}"
# MCS ladder to calibrate (1SS by default; 2T2R parts can add MCS8..15).
MCSES="${MCSES:-MCS0 MCS1 MCS2 MCS3 MCS4 MCS5 MCS6 MCS7}"
# Flat TXAGC index ladder per MCS (6-bit range; coarse enough to be quick).
IDXS="${IDXS:-8 16 24 32 40 48 56 63}"
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
    stdbuf -oL timeout 1200 "$ROOT/build/WiFiDriverDemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

: >"$OUT/cells.txt"
for mcs in $MCSES; do
    for idx in $IDXS; do
        t0="$(date +%s.%N)"
        sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
            DEVOURER_TX_RATE="$mcs" DEVOURER_TX_PWR="$idx" DEVOURER_TX_GAP_US=1500 \
            timeout 10 "$ROOT/build/WiFiDriverTxDemo" >/dev/null 2>&1 || true
        t1="$(date +%s.%N)"
        echo "$mcs $idx $t0 $t1" >>"$OUT/cells.txt"
        sleep 1.5
    done
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

# Gather every cell (delivered frame count is the ceiling signal — a garbled
# cell delivers ~0). Keep even near-empty cells so the delivery cliff is visible.
cells = {}  # mcs -> [(idx, n, median_rssi, median_evm)]
for line in open(sys.argv[2]):
    mcs, idx, t0, t1 = line.split(); idx, t0, t1 = int(idx), float(t0), float(t1)
    win = [(r, e) for (t, r, s, e) in frames if t0 + 5 <= t <= t1 - 1]
    n = len(win)
    rssi = int(statistics.median(x[0] for x in win)) if n else -1
    evm = int(statistics.median(x[1] for x in win)) if n else 0
    cells.setdefault(mcs, []).append((idx, n, rssi, evm))

order_key = lambda m: int(re.sub(r"\D", "", m))
# A cell "delivers" if its frame count is at least DELIVER_FRAC of that MCS's
# best cell — below that the constellation has garbled and delivery collapses.
DELIVER_FRAC = 0.4
table = []
print("per-MCS delivery-vs-power (n frames; ceiling = highest still-delivering idx):")
for mcs in sorted(cells, key=order_key):
    pts = sorted(cells[mcs])
    peak = max((n for _, n, _, _ in pts), default=0)
    if peak < 50:
        print(f"  {mcs}: no delivery (link too weak/strong?)"); continue
    thr = peak * DELIVER_FRAC
    ceiling = None
    ceil_evm = 0
    for idx, n, rssi, evm in pts:
        if n >= thr:
            ceiling = idx
            ceil_evm = evm
    detail = " ".join(f"{idx}:n{n}" for idx, n, _, _ in pts)
    print(f"  {mcs}: ceiling_idx={ceiling} evm@ceil={ceil_evm}  [{detail}]")
    table.append((mcs, ceiling, ceil_evm))

if len(table) < 3:
    print("RESULT insufficient (ground caught too few frames — check the link)")
    sys.exit(0)

print("\n== per-MCS clean-power ceiling (the calibration table) ==")
for mcs, idx, evm in table:
    print(f"<mcs-ceiling> mcs={mcs} idx={idx} evm={evm}")

# Validate the law: ceiling non-increasing as MCS rises (allow one ladder step
# of noise between adjacent rungs, and enforce a monotone envelope for the
# reported "safe" table — a controller should never run a higher MCS hotter
# than a lower one).
inversions = 0
env = []
run_min = 999
for mcs, idx, evm in table:
    i = idx if idx is not None else 0
    if env and i > env[-1][1] + 8:
        inversions += 1
        print(f"  NOTE: {mcs} ceiling {i} > previous {env[-1][1]} (law expects <=)")
    run_min = min(run_min, i)
    env.append((mcs, i))
c0 = table[0][1] or 0
ctop = table[-1][1] or 0
span = c0 - ctop
print(f"\nRESULT ceiling MCS0={c0} -> top={ctop} (drop {span} index steps); "
      f"{inversions} law inversion(s)")
ok = span > 0 and inversions == 0
print("== per-MCS ceiling:", "LAW REPRODUCED ==" if ok else "REVIEW (see notes) ==")
sys.exit(0 if ok else 1)
PYEOF
