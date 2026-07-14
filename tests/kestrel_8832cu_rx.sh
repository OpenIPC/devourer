#!/usr/bin/env bash
# kestrel_8832cu_rx.sh — RTL8832CU (C8852C, RISC-V die) monitor-RX regression.
# Guards the 8852C bring-up chain: RISC-V FWDL (u2_nic on the CBV die) + the
# _V1 register banks (USB/HFC/RXAGG) + the 8852C halbb/halrf PHY tables. A cell
# PASSES when the 11ax parser decodes >0 ambient frames AND firmware boots with
# no FWDL/DACK timeout. Each cell VBUS-cold-cycles the adapter first (the chip
# retains state across a soft re-init).
#
#   sudo tests/kestrel_8832cu_rx.sh [channel...]   (default: 36 6)
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
VID=35bc PID=0101
TX_HUB="3-2.3"; TX_PORT="1"   # uhubctl VBUS map for the 8832CU
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
cell() { # $1 = channel
  local ch="$1" out=/tmp/kestrel_8832rx.jsonl log=/tmp/kestrel_8832rx.log
  echo ">> ch$ch"
  uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6
  sysdir >/dev/null || { echo "   FAIL: $VID:$PID not on bus"; fails=$((fails+1)); return; }
  unbind; sleep 1
  # -k 3: rxdemo's USB RX thread can ignore a plain TERM; SIGKILL 3 s later.
  env DEVOURER_VID=0x$VID DEVOURER_PID=0x$PID DEVOURER_CHANNEL=$ch \
      timeout -k 3 10 build/rxdemo >"$out" 2>"$log" || true
  pkill -9 -f "build/rxdemo" 2>/dev/null || true; sleep 1
  local n boot img tmo
  n=$(grep -cE '"ev":"rx\.pkt"' "$out" 2>/dev/null); n=${n:-0}
  boot=$(grep -c "firmware booted" "$log" 2>/dev/null); boot=${boot:-0}
  img=$(grep -oE "image u[0-9]_nic" "$log" | tail -1)
  tmo=$(grep -cE "H2C_PATH_RDY timeout|DACK timeout|fw-ready poll timeout" "$log" 2>/dev/null); tmo=${tmo:-0}
  echo "   fw: $img booted=$boot ; rx frames: $n ; timeouts: $tmo"
  { [ "$n" -gt 0 ] && [ "$boot" -gt 0 ] && [ "$tmo" -eq 0 ]; } \
    || { echo "   FAIL"; fails=$((fails+1)); }
}

for ch in "${@:-36 6}"; do cell "$ch"; done
echo "=================================================================="
[ "$fails" -eq 0 ] && echo "RESULT: PASS — 8832CU FWDL + monitor RX decode" \
  || { echo "RESULT: FAIL — $fails cell(s)"; exit 1; }
