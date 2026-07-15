#!/usr/bin/env bash
# 6 GHz 160 MHz synth-lock ground-truth probe (8832CU, C8852C).
#
# Question: does the 6 GHz 160 MHz RF synth actually lock, or is the earlier
# "synthLock=0 / RX=0" a downstream (BB/test) artifact? The BB 0xc5[15]
# "synthLock" bit is unreliable (5G-20 decodes with it 0). The RF LCK-done bit
# RF 0xb7[8] (0 = locked) is the real signal, now logged by
# halrf8852c_ctl_band_ch_bw. This tunes three configs and prints that line:
#   5G-160  ch36  (known good, on-air validated 66.6 Mbps)  center 50
#   6G-20   ch37  (known good, decodes ambient)             center 37
#   6G-160  ch1   (suspect)                                 center 15
# A short rxdemo run per config is enough — the lock read happens at tune.
set -u
cd "$(dirname "$0")/.."
BUS=9   # standalone 35bc:0101 on hub 9-1
LOG=/tmp/kestrel_6g160_lck
rm -f "$LOG".*

run() {  # $1=label $2=channel $3=band(''|6) $4=bw
  local label=$1 ch=$2 band=$3 bw=$4
  echo "=== $label: ch$ch band=${band:-def} bw$bw ==="
  timeout -s KILL 12 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 \
    DEVOURER_USB_BUS=$BUS DEVOURER_CHANNEL=$ch DEVOURER_BW=$bw \
    ${band:+DEVOURER_BAND=$band} DEVOURER_LOG_LEVEL=info \
    ./build/rxdemo >/dev/null 2>"$LOG.$label" || true
  grep -E "band[0-9] ch[0-9]+ bw" "$LOG.$label" | tail -3
  grep -Eo "synthLock=[01]" "$LOG.$label" | tail -1
  echo
}

run 5g160 36 ""  160
run 6g20  37 6   20
run 6g160 1  6   160
echo "logs: $LOG.*"
