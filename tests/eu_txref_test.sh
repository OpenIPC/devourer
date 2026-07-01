#!/usr/bin/env bash
# Does raising the TXAGC reference raise on-air power? Measure 2 GHz ch6 (well
# above SDR threshold) at default vs forced ref 0x4b, and dump the per-rate diff
# table (0x3a00) so we know the effective index = ref + diff.
#   sudo tests/eu_txref_test.sh
set -u
PY=${PY:-python3}; EU_VID=0x0bda; EU_PID=0xa81a; CH=6; FREQ=2437e6; SECS=5
cleanup(){ sudo pkill -9 -x WiFiDriverTxDe 2>/dev/null; }
trap cleanup EXIT
sudo rmmod rtl88x2eu_ohd 2>/dev/null

cell() { # $1=label $2=extra-env
  cleanup; sleep 2
  sudo env DEVOURER_VID=$EU_VID DEVOURER_PID=$EU_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 DEVOURER_RFDUMP=1 $2 \
       stdbuf -oL timeout -k 5 $((SECS+5)) build/WiFiDriverTxDemo \
       >/tmp/txref_$1.log 2>&1 &
  sleep 5
  echo "--- $1 ($2) ---"
  sudo $PY tests/sdr_duty.py --freq $FREQ --rate 25e6 --secs $SECS --gain 60 \
       --mcs 7 --bw 20 2>&1 | grep -iE "duty=" | head -1
  cleanup; sleep 1
}
cell default ""
cell ref4b   "DEVOURER_TX_PWR=0x4b"
echo "=== default per-rate refs/diffs ==="
grep -E "BBDUMP 0x(18e8|41e8)" /tmp/txref_default.log
echo "=== ref4b refs (confirm write stuck) ==="
grep -E "BBDUMP 0x(18e8|41e8)" /tmp/txref_ref4b.log
