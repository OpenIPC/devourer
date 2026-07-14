#!/usr/bin/env bash
# kestrel_bw_rx.sh — on-air RX regression across the RTL8852BU bandwidth set
# (20 / 40 / 80 MHz on 5 GHz). Each cell VBUS-cold-cycles the adapter first: the
# chip retains state across a soft re-init, and a prior wide-BW bring-up can
# leave the next run's RX deaf otherwise (learned the hard way — a stale-state
# zero once masqueraded as a 40 MHz bug). Also confirms the RF DC calibration
# (halrf DACK/ADDCK) completes without an MSBK/DADCK/ADDCK timeout.
#
# Cells default to ch36 (the lab's 36/40/44/48 AP block covers 20/40/80). A cell
# PASSES when the 11ax parser decodes >0 ambient frames.
#
#   sudo tests/kestrel_bw_rx.sh [channel]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
VID=35bc PID=0108
CH="${1:-36}"
TX_HUB="3-2.3"; TX_PORT="3"   # uhubctl VBUS map for the 8852BU
[ -x build/rxdemo ] || { echo "FAIL: build/rxdemo missing"; exit 2; }

sysdir() { local d; for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
    && { echo "$d"; return; }; done; }
unbind() { local d; d=$(sysdir); [ -n "$d" ] && for i in "$d":*; do
  [ -L "$i/driver" ] && echo "$(basename "$i")" \
    > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true; done; }
cleanup() { pkill -9 -f "build/rxdemo" 2>/dev/null || true; }
trap cleanup EXIT

fails=0
cell() { # $1=label $2=DEVOURER_BW(20|40|80) $3=CHOFFSET
  local label="$1" bw="$2" off="$3" out=/tmp/kestrel_bwrx.jsonl log=/tmp/kestrel_bwrx.log
  echo ">> $label (ch$CH)"
  uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6
  sysdir >/dev/null || { echo "   FAIL: $VID:$PID not on bus"; fails=$((fails+1)); return; }
  unbind; sleep 1
  local env_bw=""
  [ "$bw" != 20 ] && env_bw="DEVOURER_BW=$bw"
  # -k 3: SIGKILL 3 s after SIGTERM — rxdemo's USB RX thread can ignore a plain
  # TERM, and a lingering instance collides with the next cell on the same claim.
  env DEVOURER_VID=0x$VID DEVOURER_PID=0x$PID DEVOURER_CHANNEL=$CH \
      DEVOURER_CHOFFSET=$off ${env_bw} \
      timeout -k 3 8 build/rxdemo >"$out" 2>"$log" || true
  pkill -9 -f "build/rxdemo" 2>/dev/null || true; sleep 1
  local n tmo
  n=$(grep -cE '"ev":"rx\.' "$out" 2>/dev/null); n=${n:-0}
  tmo=$(grep -cE 'MSBK timeout|DADCK timeout|ADDCK: S. timeout' "$log" 2>/dev/null); tmo=${tmo:-0}
  echo "   tuned: $(grep -oE 'tuned to ch[0-9]+ \(center [0-9]+\) bw[0-9]+' "$log" | tail -1)"
  echo "   rx frames: $n ; cal timeouts: $tmo"
  { [ "$n" -gt 0 ] && [ "$tmo" -eq 0 ]; } || { echo "   FAIL"; fails=$((fails+1)); }
}

cell "20 MHz" 20 0
cell "40 MHz" 40 1
cell "80 MHz" 80 1
echo "=================================================================="
[ "$fails" -eq 0 ] && echo "RESULT: PASS — 20/40/80 MHz RX decode + clean DACK" \
  || { echo "RESULT: FAIL — $fails cell(s)"; exit 1; }
