#!/usr/bin/env bash
# la_sdr_crosscheck.sh — issue #150: validate the chip's LA-derived
# per-tone H(k) against SDR ground truth via the NOTCH PROTOCOL.
#
# The B210 transmits an L-LTF train with a chosen asymmetric set of tones
# ZEROED — a deep deterministic spectral feature multipath cannot mimic.
# The DUT takes a CCA-triggered LA capture of that burst and its H(k)
# must show the notch at exactly those tones (tests/la_sdr_compare.py
# check-notch, local-neighbour depth).
#
# Why not |H(k)| correlation chip-vs-SDR (the original idea): measured
# ~0 on this bench — the two receivers' antennas see INDEPENDENT
# frequency-selective fading, while each side is strongly self-consistent.
# The `compare` subcommand survives as that diagnostic.
#
# Expected depth is chip-dependent: the 8822BU resolves >20 dB on every
# notched tone; the 8822CU (Jaguar3) has an ~8 dB adjacent-tone leakage
# floor (both RX paths, common LO) so single-tone edges bound at ~5-8 dB
# while notch centres reach 15-23 dB — pass it with MIN_NOTCH=5.
#
#   sudo tests/la_sdr_crosscheck.sh <DUT_PID> [CH] [DUT_VID] [MIN_NOTCH]
#   sudo tests/la_sdr_crosscheck.sh 0x012d 44 0x2357 10   # 8822BU
#   sudo tests/la_sdr_crosscheck.sh 0xc812 44 0x0bda 5    # 8822CU (J3)
set -uo pipefail
cd "$(dirname "$0")/.."

DUT_PID="${1:?usage: $0 <DUT_PID> [ch] [dut_vid] [min_notch]}"
CH="${2:-44}"
DUT_VID="${3:-0x0bda}"
MIN_NOTCH="${4:-10}"
NOTCH="-21,-20,-19,7,8,9"   # asymmetric: catches tone-mapping/inversion bugs
OUT=/tmp/la-sdr
mkdir -p "$OUT"

cleanup() { pkill -9 -x rxdemo 2>/dev/null; pkill -9 -f la_sdr_compare 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

echo "== B210 notched-LTF train on ch $CH (notch $NOTCH) =="
sudo python3 tests/la_sdr_compare.py txltf --channel "$CH" \
    --notch="$NOTCH" --secs 60 --gain 76 >"$OUT/txltf.log" 2>&1 &
sleep 10

echo "== DUT LA capture (CCA trigger) =="
sudo timeout 45 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
    DEVOURER_LA_CAPTURE=cca/20M/dma0/port:0x880/t200 \
    DEVOURER_LA_OUT="$OUT/chip.bin" DEVOURER_LA_MAX=8192 \
    build/rxdemo >"$OUT/chip.jsonl" 2>"$OUT/chip.log" &
RXPID=$!
sleep 35; sudo kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null
cleanup

grep -F '"ev":"la.capture"' "$OUT/chip.jsonl" || { echo "NO CHIP CAPTURE"; exit 1; }
echo
echo "== notch check (min depth ${MIN_NOTCH} dB) =="
python3 tests/la_sdr_compare.py check-notch --chip "$OUT/chip.bin" \
    --notch="$NOTCH" --min-notch "$MIN_NOTCH"
