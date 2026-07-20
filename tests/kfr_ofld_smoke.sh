#!/usr/bin/env bash
# kfr_ofld_smoke.sh — de-risk the Kestrel FastRetune firmware IO-offload.
#
# Runs the dwelltx dwell-1 hopper on a Kestrel (8852BU) DUT over a same-sub-band
# 5 GHz pair (offload path is the non-relock hop), first with the direct
# write-only path (DEVOURER_KFR_OFLD=0) then with the fw IO-offload
# (DEVOURER_KFR_OFLD=1). No oracle — this only answers: does the offloaded H2C
# get accepted (no fw error, no crash), and does per-hop switch_us drop? The
# oracle-attributed delivery/wrong-channel A/B is dwell1_ab.py.
set -u

# DUT must be an 8852B (die-id 0x51) — the offload path is 8852B-only; an
# 8852C (die 0x52) falls through to the vendored tune. VID:PID overridable.
DUT_VID="${DUT_VID:-0x35bc}"
DUT_PID="${DUT_PID:-0x0108}"   # TP-Link Archer TX20U Nano (RTL8852BU)
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DWELLTX="$REPO/build/dwelltx"
OUT="/tmp/kfr-ofld-smoke"
CHANS="36,40"       # both 5G-L (gain bucket 1) -> same-sub-band, offload path
SLOTS=300
SLOT_MS=30
TXPWR=12            # near-field: back off so the chip isn't railed
# The offload collapses the host hop cost but not the ~1.5 ms RF synth settle;
# the oracle-attributed correctness A/B (delivery + wrong-channel) is
# dwell1_ab.py --kfr-ofld with --settle-us >= 1500.

mkdir -p "$OUT"
declare -a PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

run_one() {
  local ofld="$1" tag="$2"
  echo "=== KFR_OFLD=$ofld ($tag) ==="
  DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=info \
  DEVOURER_KFR_OFLD="$ofld" \
  DEVOURER_DWELL_CHANNELS="$CHANS" \
  DEVOURER_DWELL_SLOT_MS="$SLOT_MS" \
  DEVOURER_DWELL_SLOTS="$SLOTS" \
  DEVOURER_HOP_FAST=1 \
  DEVOURER_TX_PWR="$TXPWR" \
    "$DWELLTX" >"$OUT/dwelltx.$tag.jsonl" 2>"$OUT/dwelltx.$tag.stderr"
  local rc=$?
  echo "exit=$rc"
  echo "-- fw/error lines (stderr) --"
  grep -iE 'error|fail|ofld|offload|timeout' "$OUT/dwelltx.$tag.stderr" | head -12
  echo "-- switch_us (median / p90 / max over same-sub-band hops) --"
  python3 - "$OUT/dwelltx.$tag.jsonl" <<'PY'
import json,sys,statistics
sw=[]
for l in open(sys.argv[1]):
    l=l.strip()
    if '"dwell.slot"' not in l: continue
    try: e=json.loads(l)
    except: continue
    if 'switch_us' in e: sw.append(e['switch_us'])
sw=sw[3:]  # drop warmup
if not sw:
    print("  no dwell.slot switch_us samples"); sys.exit()
sw_s=sorted(sw)
p90=sw_s[int(len(sw_s)*0.9)]
print(f"  n={len(sw)} median={statistics.median(sw)/1000:.2f}ms "
      f"p90={p90/1000:.2f}ms max={max(sw)/1000:.2f}ms")
PY
  echo
}

run_one 0 direct
sleep 2
run_one 1 offload
