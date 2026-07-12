#!/usr/bin/env bash
# Issue #238 quirk #1 re-test on the FIXED driver (post DPDT-0x4c / pinmux /
# 1SS-path fixes): does a nonzero path-B OFDM TXAGC ref write (0x41e8) still
# desense the 8812EU's RX to near-deaf?
#
# Cells (EU = RX under test on ch36, 8812AU = MCS0 traffic source):
#   base    EU rxdemo, 0x41e8 untouched (structural skip active)
#   poke    EU rxdemo + DEVOURER_REPLAY_WSEQ writes 0x41e8 = 0x00009e00
#           (the kernel's own working value) at end of bring-up
# Desense gone  => poke count ~ base count (quirk was a front-end artifact;
#                  skip_path_b_ofdm_ref can be retired, recovering path-B
#                  OFDM TX power in TX+RX mode).
# Desense stays => quirk #1 is real independent of the front-end fixes.
#   sudo tests/eu_41e8_desense_recheck.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/eu-41e8-recheck
CH="${CH:-36}"
SECS="${SECS:-14}"
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

printf '41e8 4 9e00\n' > /tmp/eu_41e8_poke.wseq

cell() { # $1=label $2=extra-env
    : >"$OUT/rx-$1.log"
    sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_OUT=1 $2 \
        stdbuf -oL timeout $((SECS+16)) "$ROOT/build/rxdemo" \
        >"$OUT/rx-$1.log" 2>"$OUT/rx-$1.err" &
    RJ=$!
    sleep 12   # EU bring-up
    sudo -n env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS0 DEVOURER_TX_GAP_US=1500 \
        timeout "$SECS" "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    sleep 1
    sudo -n pkill -x rxdemo 2>/dev/null; wait "$RJ" 2>/dev/null
    n=$(grep -c '"ev":"rx.frame"' "$OUT/rx-$1.log" || true)
    echo "$1: rx.frame n=${n:-0}"
    sleep 2
}

echo "== EU RX ch$CH, AU MCS0 source =="
cell base ""
cell poke "DEVOURER_REPLAY_WSEQ=/tmp/eu_41e8_poke.wseq"
grep -m1 "replay_wseq: DONE" "$OUT/rx-poke.err" || echo "WARN: poke replay did not run"
