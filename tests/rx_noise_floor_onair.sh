#!/usr/bin/env bash
# Validate the passive noise-floor + the GetRxQuality() feed (src/RxQuality.h,
# rx.quality events) ON-AIR. The passive noise floor is nf_dbm = rssi_dbm -
# snr_db per decoded OFDM/HT frame — the effective noise+interference floor the
# receiver sees, and the Fluke self-jamming signal ("decrease txpower until NF
# returns to normal"): a raised floor drops SNR while RSSI holds, so nf rises.
#
# WHAT THIS ASSERTS (and why not more): the metric's CORRECTNESS on real frames
#   1. the feed emits valid windows (frames>0, nf_valid) across a TX-power sweep;
#   2. it is SELF-CONSISTENT on-air: noise_floor_dbm ~= rssi_mean_dbm -
#      snr_mean_db (within ~2 dB — CCK/rounding), i.e. the per-frame formula is
#      wired correctly through the aggregate, not just in the unit test;
#   3. the LinkHealth verdict rides the same feed (reported).
#
# It deliberately does NOT assert a large monotonic NF excursion vs TX power:
# at bench range the two adapters are inches apart, the wanted beacon is ~-50
# dBm, and neither cranking TX power nor a co-located B210 AWGN interferer at 78
# dB moves the RX's SNR enough to swing the floor (the 8822EU AGC absorbs it) —
# measured, the same near-field/front-end limit the SDR-power and dis_cca work
# hit. A large controllable NF swing needs a real far-field link. The unit test
# (RxQualitySelftest) covers the NF math against a synthesized saturation tuple.
#
# TX 8812AU beacon, RX 8822EU. No SDR needed.
# Usage: sudo -v && tests/rx_noise_floor_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${NF_OUT:-/tmp/devourer-rx-nf}"
CH="${CH:-36}"
DUR="${DUR:-9}"
INDICES="${INDICES:-8 20 34 48 63}"   # flat TXAGC index ladder (low -> high)
mkdir -p "$OUT"

TX_PID=0x8812 TX_VID=0x0bda
RX_PID=0xa81a RX_VID=0x0bda

cleanup() {
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x txdemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
for d in "$TX_PID $TX_VID beacon-8812AU" "$RX_PID $RX_VID RX-8822EU"; do
    read -r p v name <<<"$d"
    plugged "$p" "$v" || { echo "SKIP: $name ($p:$v) not plugged"; exit 0; }
done

echo "== building =="
cmake --build "$ROOT/build" -j --target rxdemo txdemo >/dev/null || exit 1

# One cell at flat TX index $1. Echoes "frames nf rssi snr verdict" (medians).
run_cell() {
    local idx="$1" log="$OUT/idx$idx.log"
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS5 DEVOURER_TX_PWR="$idx" DEVOURER_TX_GAP_US=1200 \
        timeout $((DUR + 6)) "$ROOT/build/txdemo" >/dev/null 2>&1 &
    local tx=$!
    sleep 3
    sudo -n env DEVOURER_PID="$RX_PID" DEVOURER_VID="$RX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_RX_ENERGY_MS=1000 DEVOURER_RXQUALITY=1 \
        timeout "$DUR" "$ROOT/build/rxdemo" 2>/dev/null \
        | grep -F '"ev":"rx.quality"' >"$log" || true
    kill "$tx" 2>/dev/null; wait "$tx" 2>/dev/null
    sleep 2
    python3 - "$log" <<'PYEOF'
import json, statistics, sys
nf, rssi, snr, verdicts, frames = [], [], [], [], []
for line in open(sys.argv[1], errors="replace"):
    try:
        ev = json.loads(line)
    except ValueError:
        continue
    if ev.get("ev") != "rx.quality" or not ev.get("frames"):
        continue
    frames.append(int(ev["frames"]))
    nfv, rv, sv = ev.get("noise_floor_dbm"), ev.get("rssi_mean_dbm"), ev.get("snr_mean_db")
    v = ev.get("verdict")
    if nfv is not None: nf.append(float(nfv))
    if rv is not None: rssi.append(float(rv))
    if sv is not None: snr.append(float(sv))
    if v: verdicts.append(v)
def med(a): return statistics.median(a) if a else float("nan")
tail = verdicts[len(verdicts)//2:] or verdicts
vv = max(set(tail), key=tail.count) if tail else "NONE"
print(f"{sum(frames)} {med(nf):.1f} {med(rssi):.1f} {med(snr):.1f} {vv}")
PYEOF
}

printf "%-6s | %-7s | %-9s | %-9s | %-9s | %s\n" \
    "txidx" "frames" "nf_dbm" "rssi_dbm" "snr_db" "verdict"
printf -- "-------+---------+-----------+-----------+-----------+--------\n"
: >"$OUT/summary.tsv"
for idx in $INDICES; do
    read -r frames nf rssi snr verdict <<<"$(run_cell "$idx")"
    printf "%-6s | %-7s | %-9s | %-9s | %-9s | %s\n" \
        "$idx" "$frames" "$nf" "$rssi" "$snr" "$verdict"
    printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$idx" "$frames" "$nf" "$rssi" "$snr" \
        "$verdict" >>"$OUT/summary.tsv"
done

echo
python3 - "$OUT/summary.tsv" <<'PYEOF'
import sys, math
rows = []
for line in open(sys.argv[1]):
    idx, fr, nf, rssi, snr, v = line.split()
    rows.append((int(idx), int(fr), float(nf), float(rssi), float(snr), v))
valid = [r for r in rows if r[1] > 0 and not math.isnan(r[2])]
if len(valid) < 2:
    print("FAIL: too few valid cells"); sys.exit(1)
# 1. every swept cell produced a valid feed.
all_valid = len(valid) == len(rows)
# 2. on-air self-consistency: nf ~= rssi_mean - snr_mean (within 2 dB).
worst = max(abs(r[2] - (r[3] - r[4])) for r in valid)
consistent = worst <= 2.0
sats = [r[0] for r in rows if r[5] == "SATURATED"]
print(f"cells valid: {len(valid)}/{len(rows)}")
print(f"self-consistency worst |nf-(rssi-snr)|: {worst:.2f} dB (<=2 required)")
print(f"NF across sweep: {[r[2] for r in rows]}")
print(f"SNR across sweep: {[r[4] for r in rows]}")
print(f"SATURATED verdict at tx-idx: {sats or 'none (bench geometry)'}")
ok = all_valid and consistent
print("PASS: GetRxQuality feed valid + noise_floor self-consistent on-air"
      if ok else "FAIL: feed invalid or noise-floor not self-consistent")
sys.exit(0 if ok else 1)
PYEOF
