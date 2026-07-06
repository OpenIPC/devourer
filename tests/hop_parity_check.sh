#!/usr/bin/env bash
# Register-parity validation for the FastRetune path, any generation.
#
# Prove the fast retune leaves the chip in the SAME channel/BW register state
# as the known-good full SetMonitorChannel: drive the chip to the same target
# channel two ways (full path, then fast path) with DEVOURER_DUMP_CANARY=1,
# and diff the post-channel-set register dumps. All three generations emit the
# same canary format (Jaguar1 RadioManagementModule::DumpCanary, Jaguar2
# HalJaguar2::DumpCanary, Jaguar3 RadioManagementJaguar3::DumpCanary); the
# family — selected from TX_PID, override with FAMILY= — only picks the
# expected-to-differ exclusion lists:
#   jaguar1: TX-power / thermal registers (the fast path deliberately skips
#            the per-rate TX-power loop + pwrtrk) and IQK measurement outputs.
#   jaguar2/jaguar3: none — their channel set has no TX-power stage and the
#            canary excludes live counters, so every dumped register must
#            match up to the full-vs-full control.
#
#   ./tests/hop_parity_check.sh                 # 8812AU, 5 GHz 40 MHz, ch36->44
#   INIT=36 TARGET=44 BW=40 OFFSET=1 ./tests/hop_parity_check.sh
#   TX_PID=0xc812 INIT=1 TARGET=6 BW=20 ./tests/hop_parity_check.sh   # 8822CU
#   TX_PID=0xa81a NB_BW=5 INIT=1 TARGET=6 BW=20 ./tests/hop_parity_check.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

TX_PID="${TX_PID:-0x8812}"
INIT="${INIT:-36}"
TARGET="${TARGET:-44}"
BW="${BW:-40}"
OFFSET="${OFFSET:-1}"          # 1 = HT40+/LOWER primary
NB_BW="${NB_BW:-}"             # 5|10 = Jaguar3 narrowband re-clock
OUT="${OUT:-/tmp/devourer-hop-parity}"
mkdir -p "$OUT"

# Chip family from the PID (override with FAMILY=jaguar1|jaguar2|jaguar3).
if [ -z "${FAMILY:-}" ]; then
    case "$TX_PID" in
        0xc812|0xc82c|0xc82e|0xa81a|0x881a|0x881b|0x881c|0xe822|0xa82a)
            FAMILY=jaguar3 ;;
        0xb82c|0x012d|0x012e|0xc811) FAMILY=jaguar2 ;;
        *) FAMILY=jaguar1 ;;
    esac
fi
echo "family: $FAMILY (TX_PID=$TX_PID)"

cleanup() { pkill -x txdemo 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "== building txdemo =="
cmake --build "$ROOT/build" -j --target txdemo >/dev/null

# Run the demo to: InitWrite(INIT@BW) [full], then hop to TARGET@BW via the
# selected path. One full round of [INIT,TARGET] reaches TARGET last; the final
# canary block is TARGET's. DUMP_CANARY logs to the demo's stdout/err.
run_path() {  # $1 = HOP_FAST value, $2 = output file
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$INIT" \
        ${TX_VID:+DEVOURER_VID="$TX_VID"} \
        DEVOURER_DUMP_CANARY=1 \
        ${NB_BW:+DEVOURER_NB_BW="$NB_BW"} \
        DEVOURER_HOP_CHANNELS="$INIT,$TARGET" DEVOURER_HOP_BW="$BW" \
        DEVOURER_HOP_OFFSET="$OFFSET" \
        DEVOURER_HOP_DWELL_FRAMES=3 DEVOURER_HOP_ROUNDS=1 \
        DEVOURER_HOP_FAST="$1" \
        timeout 60 "$ROOT/build/txdemo" >"$2" 2>&1 || true
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
# Family exclusion lists. Jaguar1: registers the fast path intentionally does
# NOT reprogram (per-rate TX-power loop + tx-power training + thermal swing)
# plus the IQK / RxIQC measurement outputs (init-IQK jitter the full-vs-full
# control only catches probabilistically). Jaguar2/Jaguar3 channel sets have
# no TX-power stage and their canaries exclude live counters — nothing is
# expected to differ beyond the control.
if [ "$FAMILY" = jaguar1 ]; then
    TXPWR='0x8b0 0xc1c 0xe1c 0xc20 0xc24 0xc28 0xc2c 0xc30 0xc34 0xc38 0xc3c 0xc40 0xe20 0xe24 0xe28 0xe2c 0xe30 0xe34 0xe38 0xe3c 0xe40'
    IQK='0xc10 0xc14 0xe10 0xe14 0xc90 0xc94 0xe90 0xe94'
    # Live chip state in the J1 canary, not channel/BW config: RF 0x00's low
    # bits carry the AGC-driven mode/gain word (samples differently depending
    # on the instant of the read) and RF 0x42 is the thermal meter. Like the
    # IQK outputs, the full-vs-full control catches them only probabilistically
    # — observed flipping between runs — so exclude them statically.
    LIVE='0x00 0x42'
else
    TXPWR=''
    IQK=''
    LIVE=''
    # 8822C only: RF 0x1a is not written by any channel path on the C (it is
    # the 8822E's RXBB register) and its bit17 floats — observed flipping
    # between two full-path runs. Excluded statically since the full-vs-full
    # control catches it only probabilistically. On the E it is REAL RXBB
    # config and stays checked.
    case "$TX_PID" in
        0xc812|0xc82c|0xc82e) LIVE='0x1a' ;;
    esac
fi

echo; echo "== fast-vs-full diff (< full  > fast) =="
diff "$OUT/full.regs" "$OUT/fast.regs" || true
FAST_DIFF="$(diff "$OUT/full.regs" "$OUT/fast.regs" | addrs || true)"

echo; echo "run-variant (control): ${VARIANT:-none}"
echo "tx-power (fast skips):  $TXPWR"

# A real channel/BW break = a fast-vs-full diff address that's neither
# run-variant, an intentionally-skipped tx-power register, nor live chip state.
EXCLUDE="$(printf '%s\n%s\n%s\n%s\n' "$VARIANT" "$(echo $TXPWR | tr ' ' '\n')" \
          "$(echo $IQK | tr ' ' '\n')" "$(echo $LIVE | tr ' ' '\n')" | sort -u)"
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
