#!/usr/bin/env bash
# TX beamforming APPLY on Jaguar3 (8822E/C): does steering injected TX toward a
# sounded peer lift the peer's RX signal? Closed loop:
#   A (beamformer, 8822EU): TX+RX, arms the sounder, injects VHT1SS data + a
#     periodic VHT2SS NDPA (DEVOURER_TX_NDPA=64), and — when DEVOURER_BF_TXBF is
#     set — configures the beamformer entry and enables the V-matrix apply toggle
#     from its RX loop the moment it ingests the peer's Compressed Beamforming
#     Report (the CBR gate; a blind apply with no V degrades the link).
#   B (beamformee + sensor, 8822CU): responds to the NDPA with a CBR and measures
#     A's frames via GetRxQuality (<devourer-rxquality>).
# A/B: apply OFF (no DEVOURER_BF_TXBF) vs ON. PASS = ON lifts B's snr/rssi and
# A logs "TXBF apply ENABLED". Low TX power dodges the near-field RSSI ceiling.
#
# Usage: sudo -v && tests/txbf_apply_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXBF_OUT:-/tmp/devourer-txbf}"
CH="${CH:-100}"
DUR="${DUR:-13}"
REPS="${REPS:-2}"
BFER=57:42:75:05:d6:00          # A's self-MAC / NDPA TA
BFEE=00:e0:4c:88:22:ce          # B's beamformee MAC (programmed by arm_bfee)
# Beamformer must have a healthy RX while injecting (to ingest the CBR): the
# 8822CU does; the 8822EU's TX+RX desenses its own RX (0x41e8 quirk), so it can't
# ingest — hence 8822CU=beamformer, 8822EU=beamformee (RX-only, no desense).
A_PID=0xc812 A_VID=0x0bda        # 8822CU beamformer (TX+RX)
B_PID=0xa81a B_VID=0x0bda        # 8822EU beamformee + sensor
TXPWR="${TXPWR:-16}"
mkdir -p "$OUT"

cleanup() {
    pkill -x txdemo 2>/dev/null || true
    pkill -x rxdemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
for d in "$A_PID $A_VID beamformer-8822EU" "$B_PID $B_VID beamformee-8822CU"; do
    read -r p v name <<<"$d"
    plugged "$p" "$v" || { echo "SKIP: $name ($p:$v) not plugged"; exit 0; }
done

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

# One cell. $1 = apply(0|1)  $2 = tag. Echoes "rssi snr applied".
run_cell() {
    local apply="$1" tag="$2"
    : >"$OUT/$tag-b.log"; : >"$OUT/$tag-a.log"
    sudo -n env DEVOURER_PID="$B_PID" DEVOURER_VID="$B_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_BF_ARM_BFEE="$BFER" DEVOURER_RXQUALITY=1 DEVOURER_RX_ENERGY_MS=1000 \
        timeout $((DUR + 20)) "$ROOT/build/rxdemo" 2>&1 \
        | tee "$OUT/$tag-b.full.log" | grep --line-buffered devourer-rxquality \
        >"$OUT/$tag-b.log" &
    local bpid=$!
    # Jaguar3 beamformee init is long (DLFW + cals ~10-15s) — wait for the RX
    # loop banner + the beamformee arm before sounding, else no CBRs.
    for _ in $(seq 50); do
        grep -q "entering RX loop" "$OUT/$tag-b.full.log" 2>/dev/null && break
        sleep 0.5
    done
    sleep 2
    local extra=()
    [ "$apply" = 1 ] && extra=(DEVOURER_BF_TXBF="$BFEE")
    sudo -n env DEVOURER_PID="$A_PID" DEVOURER_VID="$A_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_WITH_RX=thread DEVOURER_BF_ARM_SOUNDER="$BFER" \
        DEVOURER_TX_NDPA_RA="$BFEE" DEVOURER_TX_NDPA=64 DEVOURER_TX_RATE=VHT1SS_MCS0 \
        DEVOURER_TX_PWR="$TXPWR" DEVOURER_TX_GAP_US=1000 "${extra[@]}" \
        timeout "$DUR" "$ROOT/build/txdemo" >"$OUT/$tag-a.log" 2>&1 || true
    sudo -n pkill -x rxdemo 2>/dev/null || true
    wait "$bpid" 2>/dev/null || true
    sleep 2
    # median rssi_mean / snr_mean over settled windows (frames>0)
    python3 - "$OUT/$tag-b.log" <<'PYEOF'
import re, statistics, sys
r, s = [], []
for line in open(sys.argv[1], errors="replace"):
    fr = re.search(r"frames=(\d+)", line)
    if not fr or int(fr.group(1)) == 0: continue
    m = re.search(r"rssi_mean_dbm=(-?\d+)", line)
    n = re.search(r"snr_mean_db=(-?\d+\.?\d*)", line)
    if m: r.append(int(m.group(1)))
    if n: s.append(float(n.group(1)))
def med(a): return statistics.median(a) if a else float("nan")
print(f"{med(r):.1f} {med(s):.1f}")
PYEOF
}

applied_any=0
declare -a OFF_R OFF_S ON_R ON_S
for rep in $(seq 1 "$REPS"); do
    read -r r s <<<"$(run_cell 0 "off$rep")"; OFF_R+=("$r"); OFF_S+=("$s")
    read -r r s <<<"$(run_cell 1 "on$rep")";  ON_R+=("$r"); ON_S+=("$s")
    grep -q "apply ENABLED" "$OUT/on$rep-a.log" && applied_any=1
    printf "rep%s  OFF rssi=%s snr=%s | ON rssi=%s snr=%s | apply=%s cbr_gate=%s\n" \
        "$rep" "${OFF_R[-1]}" "${OFF_S[-1]}" "${ON_R[-1]}" "${ON_S[-1]}" \
        "$(grep -c 'apply ENABLED' "$OUT/on$rep-a.log")" \
        "$(grep -c 'entry configured' "$OUT/on$rep-a.log")"
done

echo
python3 - "$applied_any" "${OFF_S[@]}" "__" "${ON_S[@]}" <<'PYEOF'
import sys, statistics
applied = sys.argv[1] == "1"
args = sys.argv[2:]
sep = args.index("__")
off = [float(x) for x in args[:sep]]
on  = [float(x) for x in args[sep+1:]]
d = statistics.median(on) - statistics.median(off)
print(f"OFF snr median: {statistics.median(off):.1f} dB")
print(f"ON  snr median: {statistics.median(on):.1f} dB")
print(f"delta (ON - OFF): {d:+.1f} dB   apply-gate fired: {applied}")
if applied and d >= 1.5:
    print("PASS: CBR-gated TXBF apply lifts the peer's SNR")
elif applied and d <= -1.0:
    print("FAIL: apply fired but DEGRADED the peer (V/steering wrong)")
elif not applied:
    print("FAIL: apply gate never fired (A did not ingest a CBR)")
else:
    print("INCONCLUSIVE: apply fired, delta within noise")
PYEOF
