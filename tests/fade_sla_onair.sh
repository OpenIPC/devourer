#!/usr/bin/env bash
# Fade-SLA validation — does devourer's adaptive controller hold the delivery SLA
# under time-correlated fading?
#
# Runs the closed loop (8812 VTX <-> 8821 VRX, both StreamDuplexDemo driven by
# adaptive_link.py) TWICE under a B210 interferer at the SAME gain: once STATIC,
# once FADING (--fade-coherence). It then compares the VRX's post-seq delivery
# (the `deliv=` field of <adaptive-vrx>). The hypothesis, from the linklab sim, is
# that the controller's fixed margin can't cover deep fades, so the FADING run's
# worst-window delivery drops below the SLA while the STATIC run holds it.
#
# This ONLY MEASURES — no controller code is changed. It is the precondition for
# deciding whether a variance-aware fade margin is worth adding to the controller.
#
#   sudo bash tests/fade_sla_onair.sh                  # default ch6, 30 s/phase
#   IGAIN=80 FADE_DEPTH=14 SECS=45 sudo bash tests/fade_sla_onair.sh
# SKIP_RAIL=1 after a clean boot (skips the control-adapter rail re-check).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREC="$ROOT/tools/precoder"

VTX_PID=${VTX_PID:-0x8812}; VTX_VID=${VTX_VID:-0x0bda}; VTX=${VTX_SYSFS:-9-2}
VRX_PID=${VRX_PID:-0x0120}; VRX_VID=${VRX_VID:-0x2357}; VRX=${VRX_SYSFS:-9-1.4}
CH=${CH:-6}; SECS=${SECS:-30}; VTX_ID=${VTX_ID:-0xABCD}
IGAIN=${IGAIN:-78}; FADE_COH=${FADE_COH:-0.3}; FADE_DEPTH=${FADE_DEPTH:-12}
SLA=${SLA:-0.99}

VIDEO=/tmp/fade_sla_video.bin
declare -A VRX_LOG=( [static]=/tmp/fade_sla_vrx_static.log [fading]=/tmp/fade_sla_vrx_fading.log )
declare -A VTX_LOG=( [static]=/tmp/fade_sla_vtx_static.log [fading]=/tmp/fade_sla_vtx_fading.log )
INTF_LOG=/tmp/fade_sla_intf.log

KILL(){ sudo pkill -9 -f adaptive_link 2>/dev/null
        sudo pkill -9 StreamDuplexD 2>/dev/null
        sudo pkill -9 -f sdr_interferer 2>/dev/null; }
trap KILL EXIT

# free both Wi-Fi adapters (B210 is UHD-accessed)
for D in "$VTX" "$VRX"; do
  for i in /sys/bus/usb/devices/$D/$D:*; do
    ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
    [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
  done
done; sleep 1

head -c 4000000 /dev/urandom > "$VIDEO"   # synthetic video bytes

run_phase(){               # $1=label  $2..=extra sdr_interferer args
  local label="$1"; shift
  echo "=== PHASE $label: interferer ch$CH gain=$IGAIN $* ==="
  sudo python3 "$ROOT/tests/sdr_interferer.py" --channel "$CH" --tx-gain "$IGAIN" \
       --rate 20e6 --mode noise --secs $((SECS + 12)) "$@" >"$INTF_LOG" 2>&1 &
  sleep 8
  sudo env DEVOURER_VID=$VRX_VID DEVOURER_PID=$VRX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" --role vrx --pid $VRX_PID --channel $CH \
       --vtx-id $VTX_ID --duplex "$ROOT/build/StreamDuplexDemo" >"${VRX_LOG[$label]}" 2>&1 &
  sleep 5
  sudo env DEVOURER_VID=$VTX_VID DEVOURER_PID=$VTX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" --role vtx --pid $VTX_PID --channel $CH \
       --vtx-id $VTX_ID --video "$VIDEO" --duplex "$ROOT/build/StreamDuplexDemo" \
       >"${VTX_LOG[$label]}" 2>&1 &
  sleep "$SECS"; KILL; sleep 2
}

# delivery stats from a VRX log's <adaptive-vrx> deliv= field
stats(){ grep -oP '<adaptive-vrx>.*?deliv=\K[0-9.]+' "$1" | python3 -c '
import sys
v=[float(x) for x in sys.stdin if x.strip()]
if not v: print("n=0 (no samples)"); sys.exit()
v.sort(); n=len(v)
mean=sum(v)/n; mn=v[0]; p10=v[max(0,int(0.10*n))]
print(f"n={n} mean={mean:.3f} p10={p10:.3f} min={mn:.3f}")'; }

run_phase static
run_phase fading --fade-coherence "$FADE_COH" --fade-depth-db "$FADE_DEPTH"

echo
echo "=== RESULT (SLA target deliv >= $SLA) ==="
echo "[static] $(stats "${VRX_LOG[static]}")"
echo "[fading] $(stats "${VRX_LOG[fading]}")"
echo "interferer fade envelope samples: $(grep -c '<interferer-fade>' "$INTF_LOG")"
echo "trajectories:"
echo "  static: $(grep -c '<adaptive-vrx>' "${VRX_LOG[static]}") samples -> ${VRX_LOG[static]}"
echo "  fading: $(grep -c '<adaptive-vrx>' "${VRX_LOG[fading]}") samples -> ${VRX_LOG[fading]}"
echo
echo "Interpretation: if [fading] min/p10 deliv drops below $SLA while [static]"
echo "holds it (at the same interferer gain), the fixed-margin controller is"
echo "confirmed to under-deliver under fading — the precondition for adding a"
echo "variance-aware fade margin."
