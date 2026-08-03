#!/usr/bin/env bash
#
# arq_retry_sweep.sh — hardware-ARQ retry-limit vs delivery/airtime curve.
#
# Hardware retries are backoff-spaced, so a small DEVOURER_TX_RETRY_LIMIT can
# burn entirely inside one ground-station feedback burst (2-3 ms) and drop the
# frame, while a larger limit straddles the burst tail and delivers late.
# This sweep runs the arq_e2e bench (collision regime: default async DUT, no
# consumer stalls — the residual is pure burst-collision loss) once per retry
# limit and tabulates:
#   delivered%   — reports ok=1 / reports
#   mean_retries — airtime-cost proxy (each retry re-airs the whole frame)
#   drops        — reports ok=0 (retry budget exhausted)
# and hands each run to arq_fec_dimension.py for the post-ARQ residual the
# FEC floor must cover — the (retry_limit, K/N) pairing dataset.
#
#   sudo bash tests/arq_retry_sweep.sh
#   LIMITS="3 8" CYCLES=3 sudo bash tests/arq_retry_sweep.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LIMITS=${LIMITS:-"3 8 16 32"}
OUT=${OUT:-/tmp/arq-retry-sweep/$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"
[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 3; }

declare -A RUNDIR
for L in $LIMITS; do
  echo "=== retry_limit=$L"
  RETRY_LIMIT="$L" bash "$ROOT/tests/arq_e2e_delivery.sh" \
      >"$OUT/limit_$L.log" 2>&1 || {
    echo "run failed (see $OUT/limit_$L.log)"; exit 1; }
  RUNDIR[$L]=$(ls -td /tmp/arq-e2e/*/ | head -1)
  echo "    -> ${RUNDIR[$L]}"
done

echo
printf "%8s %10s %12s %8s %10s\n" limit reports "delivered%" drops mean_rtry
for L in $LIMITS; do
  D=${RUNDIR[$L]}
  python3 - "$D/drone.jsonl" "$L" <<'PYEOF'
import json, sys
n = ok = drops = 0
rsum = 0
for line in open(sys.argv[1], errors="replace"):
    if not line.startswith('{"ev":"tx.report"'):
        continue
    try:
        ev = json.loads(line)
    except Exception:
        continue
    n += 1
    rsum += ev.get("retries", 0)
    if ev.get("ok"):
        ok += 1
    else:
        drops += 1
print(f"{sys.argv[2]:>8} {n:>10} {100.0*ok/max(1,n):>11.2f} "
      f"{drops:>8} {rsum/max(1,n):>10.3f}")
PYEOF
done | tee "$OUT/summary.txt"

echo
python3 "$ROOT/tests/arq_fec_dimension.py" \
    $(for L in $LIMITS; do echo "${RUNDIR[$L]}"; done) | tee -a "$OUT/summary.txt"
echo "[sweep] logs: $OUT"
