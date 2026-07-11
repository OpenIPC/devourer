#!/usr/bin/env bash
# Issue #238 follow-up validation: 8822E (8812EU) MCS4+/64-QAM TX garbling.
#
# Root cause (bench-proven by snokvist, ordering confirmed in source): the FW
# power-mode / coex H2Cs at the tail of InitWrite reprogram the OFDM TXAGC
# references (0x18e8/0x41e8) wholesale, clobbering the pre-coex TXAGC apply —
# so ALL construction-time TX-power state (DEVOURER_TX_PWR et al.) was inert
# on the EU and the chip always transmitted at the FW's efuse-derived refs
# (~idx 72/83 on this board), driving the PA into compression and shredding
# 16/64-QAM while BPSK/QPSK ride through. Fix = re-apply TXAGC as the LAST
# bring-up register step (RtlJaguar3Device::InitWrite).
#
# This script proves the fix end-to-end on-air:
#   TX = 8812EU (0bda:a81a), RX ground = 8822CU (0bda:c812), ch36 / 20 MHz.
#   Cells: {MCS0, MCS7} x {default power, TX_PWR=39, TX_PWR=28}.
# Expected:
#   - MCS0 delivers in every cell (control).
#   - MCS7 @ default: ~0 clean frames (PA compression — unchanged physics).
#   - MCS7 @ TX_PWR=39/28: substantial clean delivery (rate=19) with sane EVM.
#     Pre-fix these cells read ~0 because the construction-time TX_PWR never
#     reached the hardware.
#   - txdemo stderr shows "TXAGC refs re-applied post-coex (FW had rewritten
#     them)" — direct evidence of the FW clobber.
#
# Usage: sudo -v && tests/eu_mcs7_txagc_fix.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${EU_MCS7_OUT:-/tmp/devourer-eu-mcs7-fix}"
CH="${CH:-36}"
TX_PID="${TX_PID:-0xa81a}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${GROUND_PID:-0xc812}" GROUND_VID="${GROUND_VID:-0x0bda}"
CELL_SECS="${CELL_SECS:-10}"
mkdir -p "$OUT"

cleanup() {
    sudo -n pkill -x txdemo 2>/dev/null || true
    sudo -n pkill -x rxdemo 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
plugged "$TX_PID" "$TX_VID" || { echo "SKIP: TX $TX_PID not plugged"; exit 0; }
plugged "$GROUND_PID" "$GROUND_VID" || { echo "SKIP: ground $GROUND_PID not plugged"; exit 0; }

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

echo "== ground RX ($GROUND_PID) up on ch$CH =="
# The 8822CU ground is intermittently deaf at 2.4 GHz across bring-ups
# (issue #238 quirk-#2-adjacent): gate every session on ambient traffic —
# a ground that hears nothing in GROUND_PROBE_SECS on a busy channel is
# restarted (up to GROUND_TRIES bring-ups) before any TX cell is judged.
GROUND_TRIES="${GROUND_TRIES:-4}"
GROUND_PROBE_SECS="${GROUND_PROBE_SECS:-14}"
GJ=""
start_ground() {
    : >"$OUT/ground.log"
    sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 ${GROUND_EXTRA_ENV:-} \
        stdbuf -oL timeout 600 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
        | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
        >>"$OUT/ground.log" &
    GJ=$!
}
try=1
while :; do
    start_ground
    sleep "$GROUND_PROBE_SECS"
    n=$(grep -c '"ev":"rx.frame"' "$OUT/ground.log" 2>/dev/null || true)
    n=${n:-0}
    [ "$n" -gt 0 ] && { echo "ground alive (ambient n=$n, try $try)"; break; }
    echo "ground deaf (try $try/$GROUND_TRIES) — restarting rxdemo"
    sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null; sleep 3
    try=$((try+1))
    if [ "$try" -gt "$GROUND_TRIES" ]; then
        # Ambient absence is NOT proof of a deaf ground (quiet channels /
        # hours give false negatives — bench-observed on ch6): by default
        # warn and continue; GROUND_GATE=strict makes it fatal. Verify a
        # suspicious ground with a known-good TX (e.g. the 8812AU control).
        if [ "${GROUND_GATE:-warn}" = "strict" ]; then
            echo "RESULT SKIP: ground never heard ambient traffic on ch$CH — cells unjudgeable"
            exit 2
        fi
        echo "WARN: no ambient traffic on ch$CH after $GROUND_TRIES ground bring-ups — continuing (channel may just be quiet; verify with a control TX)"
        start_ground
        sleep 3
        break
    fi
done

# cells: "<label> <mcs> <tx_pwr-or-'-'> [extra-env,comma,separated]"
# tx_pwr = flat TXAGC index (DEVOURER_TX_PWR, zeroes per-rate diffs + per-path
# trim); extra-env is where the shape-preserving offset goes, e.g.
# DEVOURER_TX_PWR_OFFSET_QDB=-44.
CELLS="${CELLS:-mcs0-def MCS0 -
mcs7-def MCS7 -
mcs0-p39 MCS0 39
mcs7-p39 MCS7 39
mcs7-p28 MCS7 28}"

: >"$OUT/cells.txt"
while read -r label mcs pwr extra; do
    [ -n "$label" ] || continue
    envs=(DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH"
          DEVOURER_TX_RATE="$mcs" DEVOURER_TX_GAP_US=1500)
    # TX_EXTRA_ENV="A=1 B=2" — extra env for the TX side (e.g.
    # DEVOURER_LOG_LEVEL=debug for the coex-tick TXAGC-ref drift watch).
    [ -n "${TX_EXTRA_ENV:-}" ] && envs+=($TX_EXTRA_ENV)
    [ -n "$extra" ] && envs+=(${extra//,/ })
    [ "$pwr" != "-" ] && envs+=(DEVOURER_TX_PWR="$pwr")
    echo "== cell $label ($mcs pwr=$pwr) =="
    t0="$(date +%s.%N)"
    sudo -n env "${envs[@]}" timeout "$CELL_SECS" "$ROOT/build/txdemo" \
        >"$OUT/tx-$label.log" 2>"$OUT/tx-$label.err" || true
    t1="$(date +%s.%N)"
    echo "$label $mcs $pwr $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done <<<"$CELLS"
sudo -n pkill -x rxdemo 2>/dev/null
wait "$GJ" 2>/dev/null

echo
echo "== FW-clobber evidence (txdemo stderr) =="
grep -h "re-applied post-coex" "$OUT"/tx-*.err | sort -u || echo "  (no re-apply log line — FW did not rewrite the refs?)"

python3 - "$OUT/ground.log" "$OUT/cells.txt" <<'PYEOF'
import json, statistics, sys

RATE = {"MCS0": 12, "MCS1": 13, "MCS2": 14, "MCS3": 15,
        "MCS4": 16, "MCS5": 17, "MCS6": 18, "MCS7": 19,
        "6M": 4, "9M": 5, "12M": 6, "18M": 7,
        "24M": 8, "36M": 9, "48M": 10, "54M": 11,
        "1M": 0, "2M": 1, "5.5M": 2, "11M": 3}

frames = []  # (t, rate, evm0)
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line:
        continue
    ts, _, js = line.partition(" ")
    try:
        ev = json.loads(js)
    except ValueError:
        continue
    evm = ev.get("evm")
    frames.append((float(ts), ev.get("rate", -1),
                   evm[0] if isinstance(evm, list) and evm else 0))

fails = []
print("\ncell            rate-matched  other-rate  medEVM(dB)")
for line in open(sys.argv[2]):
    label, mcs, pwr, t0, t1 = line.split()
    t0, t1 = float(t0), float(t1)
    want = RATE[mcs]
    win = [(r, e) for (t, r, e) in frames if t0 + 3 <= t <= t1 - 0.5]
    hit = [e for (r, e) in win if r == want]
    other = sum(1 for (r, _) in win if r != want)
    med = int(statistics.median(hit)) if hit else 0
    print(f"{label:<15} {len(hit):>12}  {other:>10}  {med:>10}")
    if label == "mcs0-def" and len(hit) < 200:
        fails.append("control mcs0-def under-delivered — link/bench problem")
    if label in ("mcs7-p39", "mcs7-p28") and len(hit) < 200:
        fails.append(f"{label}: MCS7 at reduced power did not deliver "
                     f"(n={len(hit)}) — fix NOT effective")

print()
if fails:
    print("RESULT FAIL:")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("RESULT PASS: construction-time TX power reaches the hardware; "
      "MCS7 clean below the PA-compression knee")
PYEOF
