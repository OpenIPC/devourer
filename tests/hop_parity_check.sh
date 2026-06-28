#!/usr/bin/env bash
# Register-parity validation for the 40/80 MHz FastRetune path.
#
# The 5 GHz bench power sags, so a clean on-air 40 MHz hop test isn't possible
# here. Instead we prove the fast 40/80 retune leaves the chip in the SAME
# channel/BW register state as the known-good full SetMonitorChannel: drive the
# chip to the same target channel two ways (full path, then fast path) with
# DEVOURER_DUMP_CANARY=1, and diff the post-channel-set register dumps.
#
# Channel/BW registers (RF 0x18 = center channel + BW + MOD_AG, BB 0x860 fc_area,
# 0x8ac rRFMOD/SubChnl, 0x8c4 ADC clk, ...) MUST match. TX-power / thermal
# registers (0xc1c/0xe1c, 0xc20..0xc40, 0xe20..0xe40) are EXPECTED to differ —
# the fast path deliberately skips the per-rate TX-power loop and the pwrtrk
# tick (same as the 20 MHz fast path), so those keep the prior channel's values.
#
#   ./tests/hop_parity_check.sh                 # 5 GHz 40 MHz, ch36->ch44
#   INIT=36 TARGET=44 BW=40 OFFSET=1 ./tests/hop_parity_check.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

TX_PID="${TX_PID:-0x8812}"
INIT="${INIT:-36}"
TARGET="${TARGET:-44}"
BW="${BW:-40}"
OFFSET="${OFFSET:-1}"          # 1 = HT40+/LOWER primary
OUT="${OUT:-/tmp/devourer-hop-parity}"
mkdir -p "$OUT"

cleanup() { pkill -x WiFiDriverTxDemo 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "== building WiFiDriverTxDemo =="
cmake --build "$ROOT/build" -j --target WiFiDriverTxDemo >/dev/null

# Run the demo to: InitWrite(INIT@BW) [full], then hop to TARGET@BW via the
# selected path. One full round of [INIT,TARGET] reaches TARGET last; the final
# canary block is TARGET's. DUMP_CANARY logs to the demo's stdout/err.
run_path() {  # $1 = HOP_FAST value, $2 = output file
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$INIT" \
        DEVOURER_DUMP_CANARY=1 \
        DEVOURER_HOP_CHANNELS="$INIT,$TARGET" DEVOURER_HOP_BW="$BW" \
        DEVOURER_HOP_OFFSET="$OFFSET" \
        DEVOURER_HOP_DWELL_FRAMES=3 DEVOURER_HOP_ROUNDS=1 \
        DEVOURER_HOP_FAST="$1" \
        timeout 40 "$ROOT/build/WiFiDriverTxDemo" >"$2" 2>&1 || true
}

# Extract the LAST canary block (the TARGET channel) as "REG ADDR = VALUE" lines.
last_canary() {  # $1 = log file
    awk '/=== DEVOURER_DUMP_CANARY/{buf=""} {buf=buf $0 "\n"}
         /=== END DEVOURER_DUMP_CANARY/{last=buf} END{printf "%s", last}' "$1" \
      | grep -oE "(BB|MAC|RF\[[AB]\]) 0x[0-9a-fA-F]+ = 0x[0-9A-F]+" \
      | sort -u
}

# Two full runs (control) isolate registers that vary run-to-run on their own —
# the TSF timer (MAC 0x560) and IQK/RxIQC measurement jitter (0xc10/0xe10/...) —
# so they aren't mistaken for a fast-path break.
echo "== full path (run 1) =="; run_path 0 "$OUT/full.log"
echo "== full path (run 2, control) =="; run_path 0 "$OUT/full2.log"
echo "== fast path =="; run_path 1 "$OUT/fast.log"

last_canary "$OUT/full.log"  >"$OUT/full.regs"
last_canary "$OUT/full2.log" >"$OUT/full2.regs"
last_canary "$OUT/fast.log"  >"$OUT/fast.regs"

for f in full full2 fast; do
    [ -s "$OUT/$f.regs" ] || { echo "PARITY: FAIL — no canary in $f (see $OUT/$f.log)"; exit 1; }
done

# Addresses that differ between the two full runs = inherently run-variant.
addrs() { grep -E '^[<>]' | grep -oE "0x[0-9a-fA-F]+ =" | grep -oE "0x[0-9a-fA-F]+" | sort -u; }
VARIANT="$(diff "$OUT/full.regs" "$OUT/full2.regs" | addrs || true)"
# Registers the fast path intentionally does NOT reprogram (per-rate TX-power
# loop + tx-power training + thermal swing — same stages the 20 MHz path skips).
TXPWR='0x8b0 0xc1c 0xe1c 0xc20 0xc24 0xc28 0xc2c 0xc30 0xc34 0xc38 0xc3c 0xc40 0xe20 0xe24 0xe28 0xe2c 0xe30 0xe34 0xe38 0xe3c 0xe40'
# IQK / RxIQC measurement-output registers (set by the init IQK, not by the
# channel set). They carry run-to-run measurement jitter — the full-vs-full
# control catches them probabilistically, so list them too for robustness.
IQK='0xc10 0xc14 0xe10 0xe14 0xc90 0xc94 0xe90 0xe94'

echo; echo "== fast-vs-full diff (< full  > fast) =="
diff "$OUT/full.regs" "$OUT/fast.regs" || true
FAST_DIFF="$(diff "$OUT/full.regs" "$OUT/fast.regs" | addrs || true)"

echo; echo "run-variant (control): ${VARIANT:-none}"
echo "tx-power (fast skips):  $TXPWR"

# A real channel/BW break = a fast-vs-full diff address that's neither
# run-variant nor an intentionally-skipped tx-power register.
EXCLUDE="$(printf '%s\n%s\n%s\n' "$VARIANT" "$(echo $TXPWR | tr ' ' '\n')" \
          "$(echo $IQK | tr ' ' '\n')" | sort -u)"
BREAKS=""
for a in $FAST_DIFF; do
    echo "$EXCLUDE" | grep -qx "$a" || BREAKS="$BREAKS $a"
done

echo
if [ -z "$BREAKS" ]; then
    echo "PARITY: PASS — every channel/BW register matches the full path; the"
    echo "         only fast-vs-full diffs are run-variant (timer/IQK) or the"
    echo "         tx-power registers the fast path deliberately skips."
else
    echo "PARITY: FAIL — channel/BW register break(s):$BREAKS"
    exit 1
fi
echo "Regs: $OUT/full.regs  $OUT/fast.regs"
