#!/usr/bin/env bash
# Rendezvous building block — re-find a peer when the link has dropped. The
# mains-powered "ground station" parks a modulated continuous beacon
# (DEVOURER_CONT_TX) on a known channel; the battery-powered "drone" scans the
# candidate channels with its frame-free energy sensor (DEVOURER_RX_SWEEP) and
# locks onto the channel carrying the beacon. This is the asymmetric-duty
# rendezvous the adaptive-link design describes — an expensive always-on beaconer,
# a cheap low-duty listener — built from the continuous-TX stimulus + the RX
# energy sensor. Two adapters, no SDR; the beacon works on all three generations.
#
#   sudo tests/rendezvous.sh --beacon-pid 0x8812 --scanner-pid 0xc812 \
#        --beacon-channel 6 --channels 1,6,11
#
# The scanner should report its peak (or, on a saturating 1T1R part, dip) at the
# beacon's channel — "peer found". The scanner does not know --beacon-channel; it
# discovers it from the sweep.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEMO_TX="$ROOT/build/WiFiDriverTxDemo"
DEMO_RX="$ROOT/build/WiFiDriverDemo"

BEACON_VID=0x0bda BEACON_PID="" BEACON_CHANNEL=6 RATE=MCS0
SCANNER_VID=0x0bda SCANNER_PID="" CHANNELS="1,6,11" DWELL_MS=350 ROUNDS=4
OUT=/tmp/devourer-rendezvous

while [ $# -gt 0 ]; do
  case "$1" in
    --beacon-vid) BEACON_VID="$2"; shift 2 ;;
    --beacon-pid) BEACON_PID="$2"; shift 2 ;;
    --beacon-channel) BEACON_CHANNEL="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --scanner-vid) SCANNER_VID="$2"; shift 2 ;;
    --scanner-pid) SCANNER_PID="$2"; shift 2 ;;
    --channels) CHANNELS="$2"; shift 2 ;;
    --dwell-ms) DWELL_MS="$2"; shift 2 ;;
    --rounds) ROUNDS="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$BEACON_PID" ] && [ -n "$SCANNER_PID" ] || {
  echo "need --beacon-pid and --scanner-pid" >&2; exit 2; }

cleanup() { for c in WiFiDriverTxDem WiFiDriverDemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"; rm -f "$OUT"/sweep.log
cleanup; sleep 1

echo "== ground beacon: $BEACON_PID modulated continuous carrier on ch$BEACON_CHANNEL =="
timeout -sINT 600 env DEVOURER_VID="$BEACON_VID" DEVOURER_PID="$BEACON_PID" \
  DEVOURER_CHANNEL="$BEACON_CHANNEL" DEVOURER_TX_RATE="$RATE" DEVOURER_CONT_TX=1 \
  "$DEMO_TX" >"$OUT/beacon.log" 2>&1 &
for _ in $(seq 40); do grep -q "continuous TX armed\|carrier up" "$OUT/beacon.log" && break; sleep 0.5; done
grep -q "continuous TX armed\|carrier up" "$OUT/beacon.log" && echo "  beacon up" \
  || echo "  WARN: beacon not up (see $OUT/beacon.log)"

nbins="$(awk -F',' '{print NF}' <<< "$CHANNELS")"
secs="$(( (nbins * ROUNDS * DWELL_MS) / 1000 + 8 ))"
echo "== drone scan: sweep $CHANNELS x $ROUNDS rounds (~${secs}s), locking onto the beacon =="
timeout -sINT "$secs" env DEVOURER_VID="$SCANNER_VID" DEVOURER_PID="$SCANNER_PID" \
  DEVOURER_RX_SWEEP="$CHANNELS" DEVOURER_RX_SWEEP_DWELL_MS="$DWELL_MS" \
  "$DEMO_RX" 2>/dev/null | grep --line-buffered "devourer-energy" \
  | tee "$OUT/sweep.log" || true
cleanup

echo "== rendezvous result =="
# Pick the channel with the most total in-band energy (OFDM + CCK CCA) — the
# modulated beacon can register on either detector, so sum both rather than key
# on cca_ofdm alone (the CW-tone sweep's metric).
python3 - "$OUT/sweep.log" <<'PY'
import re, sys, statistics
per = {}
for line in open(sys.argv[1]):
    kv = dict(re.findall(r"(\w+)=(-?\d+)", line))
    if "ch" not in kv:
        continue
    ch = int(kv["ch"])
    e = int(kv.get("cca_ofdm", 0)) + int(kv.get("cca_cck", 0))
    per.setdefault(ch, []).append(e)
if not per:
    print("no sweep samples — beacon or scan failed"); sys.exit(1)
energy = {ch: statistics.median(v) for ch, v in per.items()}
width = 46
peak = max(energy.values()) or 1
for ch in sorted(energy):
    bar = "#" * round(width * energy[ch] / peak)
    print(f"  ch {ch:>3} | {bar:<{width}} {int(energy[ch]):>6}")
best = max(energy, key=energy.__getitem__)
floor = statistics.median(list(energy.values()))
if energy[best] > 3 * max(floor, 1):
    print(f"\nRENDEZVOUS: peer found on channel {best} "
          f"(energy {int(energy[best])} vs floor {int(floor)}).")
else:
    print(f"\nRENDEZVOUS: no clear peer — strongest ch {best} "
          f"(energy {int(energy[best])}, floor {int(floor)}); beacon too weak or "
          f"off the swept list.")
PY
