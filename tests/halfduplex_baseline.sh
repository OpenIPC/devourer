#!/usr/bin/env bash
# Diagnose the BASELINE (no-interferer) frame loss in the adaptive link.
#
# The fade-SLA sweep showed delivery never reaches ~0.99 even at low interference
# (~0.7-0.84). This localizes that loss by sweeping the VRX RCF feedback period
# with NO interferer running, reporting per rate:
#   A  = VTX frames TXed on-air        (VTX duplex "tx #N")
#   B  = VRX frames RXed on-air        (VRX duplex "rx hits")  -> B/A = on-air delivery
#   FB = VRX feedback frames TXed      (VRX duplex "tx #N")    -> half-duplex RX-blind windows
#   deliv = 1 - seq_gap_loss           (<adaptive-vrx>)
#
# Hypothesis: the VRX is half-duplex, so each RCF TX blinds its own RX and drops
# video. If delivery (and B/A) RISE as the feedback period grows, the half-duplex
# feedback TX is the baseline-loss cause. If B/A is already ~1 while deliv is low,
# the loss is in the seq-gap metric / software pipe instead.
#
#   sudo bash tests/halfduplex_baseline.sh
#   SECS=30 FB_LIST="100 500 2000" sudo bash tests/halfduplex_baseline.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREC="$ROOT/tools/precoder"

VTX_PID=${VTX_PID:-0x8812}; VTX_VID=${VTX_VID:-0x0bda}; VTX=${VTX_SYSFS:-9-2}
VRX_PID=${VRX_PID:-0x0120}; VRX_VID=${VRX_VID:-0x2357}; VRX=${VRX_SYSFS:-9-1.4}
CH=${CH:-6}; SECS=${SECS:-20}; VTX_ID=${VTX_ID:-0xABCD}
FB_LIST=${FB_LIST:-"100 250 1000"}

VIDEO=/tmp/hdx_video.bin
VRX_LOG=/tmp/hdx_vrx.log; VTX_LOG=/tmp/hdx_vtx.log

KILL(){ sudo pkill -9 -f adaptive_link 2>/dev/null
        sudo pkill -9 duplex 2>/dev/null; }
trap KILL EXIT

free_adapters(){
  for D in "$VTX" "$VRX"; do
    for i in /sys/bus/usb/devices/$D/$D:*; do
      ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
      [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
    done
  done; sleep 2
}

head -c 4000000 /dev/urandom > "$VIDEO"

run_once(){                # $1 = feedback period ms
  local fb="$1"
  KILL; free_adapters
  sudo env DEVOURER_VID=$VRX_VID DEVOURER_PID=$VRX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" --role vrx --pid $VRX_PID --channel $CH \
       --vtx-id $VTX_ID --feedback-ms "$fb" \
       --duplex "$ROOT/build/duplex" >"$VRX_LOG" 2>&1 &
  sleep 4
  sudo env DEVOURER_VID=$VTX_VID DEVOURER_PID=$VTX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" --role vtx --pid $VTX_PID --channel $CH \
       --vtx-id $VTX_ID --video "$VIDEO" \
       --duplex "$ROOT/build/duplex" >"$VTX_LOG" 2>&1 &
  for _ in $(seq 1 20); do grep -q 'state=SESSION' "$VRX_LOG" && break; sleep 1; done
  sleep "$SECS"; KILL; sleep 2
}

deliv_mean(){ grep -E '<adaptive-vrx>state=SESSION' "$VRX_LOG" | grep -oP 'deliv=\K[0-9.]+' \
  | python3 -c 'import sys;v=[float(x) for x in sys.stdin if x.strip()];print(f"{sum(v)/len(v):.3f}" if v else "n/a")'; }

echo "=== half-duplex baseline diagnosis (NO interferer) ==="
printf "%-8s | %-7s | %-7s | %-6s | %-6s | %s\n" "fb_ms" "A(tx)" "B(rx)" "B/A" "FB(tx)" "deliv"
for fb in $FB_LIST; do
  run_once "$fb"
  A=$(grep -oP 'tx #\K\d+' "$VTX_LOG" | tail -1); A=${A:-0}
  B=$(grep -oP 'rx hits=\K\d+' "$VRX_LOG" | tail -1); B=${B:-0}
  FB=$(grep -oP 'tx #\K\d+' "$VRX_LOG" | tail -1); FB=${FB:-0}
  BA=$(python3 -c "print(f'{$B/$A:.3f}' if $A else 'n/a')")
  printf "%-8s | %-7s | %-7s | %-6s | %-6s | %s\n" "$fb" "$A" "$B" "$BA" "$FB" "$(deliv_mean)"
done
echo
echo "If deliv & B/A rise as fb_ms grows -> half-duplex RCF TX blinds the VRX RX"
echo "(baseline loss is the feedback's fault). If B/A ~1 but deliv low -> seq-gap"
echo "metric / pipe, not the air."
