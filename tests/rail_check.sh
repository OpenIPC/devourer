#!/usr/bin/env bash
# Rail-sag guard — CLAUDE.md defence #1: re-check the known-good control adapter's
# 5 GHz on-air duty EACH SESSION before trusting any 5 GHz TX measurement.
#
# 5 GHz TX draws far more PA current than 2.4 GHz; fed through a bus-powered hub
# chain the rail browns out the PA, collapsing on-air power *intermittently* and
# worst on the most deeply-nested adapter — while frames still submit fine (0
# send-fails) and 2.4 GHz keeps working. The trap is to mis-diagnose this as a
# per-chip dead PA / a 5 GHz code gate / an EFUSE TX-power bug. This guard catches
# it: it floods the control adapter at 5 GHz and 2.4 GHz via devourer and compares
# the SDR duty. A healthy rail gives comparable 5 GHz and 2.4 GHz duty; a sagging
# rail gives 5 GHz << 2.4 GHz (and 5 GHz below the absolute floor).
#
#   sudo tests/rail_check.sh                  # default control = CU c812
#   sudo CTRL_PID=0xc812 RAIL_MIN_DUTY=50 RAIL_MIN_RATIO=0.6 tests/rail_check.sh
#
# Exit 0 = rail OK (5 GHz measurements trustworthy); exit 1 = rail sag (do NOT
# trust 5 GHz numbers — power-cycle the hub / move the control to a powered hub or
# direct root port and re-run).
set -u
PY=${PY:-python3}
CTRL_VID=${CTRL_VID:-0x0bda}
CTRL_PID=${CTRL_PID:-0xc812}          # CU (RTL8812CU) — the validated 5 GHz control
SECS=${SECS:-5}
RAIL_MIN_DUTY=${RAIL_MIN_DUTY:-50}    # absolute 5 GHz duty floor (%), tune per SDR setup
RAIL_MIN_RATIO=${RAIL_MIN_RATIO:-0.6} # min 5GHz/2.4GHz duty ratio on a healthy rail
TXBIN=${TXBIN:-build/txdemo}

cleanup(){ sudo pkill -9 -x txdemo 2>/dev/null; }
trap cleanup EXIT

# free the control from any bound kernel driver
free_ctrl() {
  for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "${CTRL_PID#0x}" ] || continue
    local dev; dev=$(dirname "$d")
    for intf in "$dev":*; do
      local drv="$intf/driver"
      [ -e "$drv" ] && echo "$(basename "$intf")" | \
        sudo tee "$drv/unbind" >/dev/null 2>&1
    done
  done
}

measure() { # $1=channel $2=freq -> echoes duty%
  cleanup; sleep 2
  sudo env DEVOURER_VID=$CTRL_VID DEVOURER_PID=$CTRL_PID DEVOURER_CHANNEL=$1 \
       DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 \
       stdbuf -oL timeout -k 5 $((SECS+5)) "$TXBIN" >/tmp/railchk_$1.log 2>&1 &
  sleep 5
  local out
  out=$(sudo $PY tests/sdr_duty.py --freq "$2" --rate 25e6 --secs "$SECS" \
        --gain 60 --mcs 7 --bw 20 2>&1 | grep -oiE "duty=[0-9.]+%" | head -1)
  cleanup; sleep 1
  echo "${out#duty=}" | tr -d '%'
}

echo "# rail_check: control $CTRL_VID:$CTRL_PID"
sudo rmmod rtl88x2eu_ohd 2>/dev/null
free_ctrl
D24=$(measure 6   2437e6)
D5=$(measure  36  5180e6)
D24=${D24:-0}; D5=${D5:-0}
echo "  2.4 GHz duty = ${D24}%   5 GHz duty = ${D5}%"

# ratio in integer math (avoid bc dependency): 100*D5/D24 vs 100*RATIO
ratio_ok=1
if awk "BEGIN{exit !($D24 > 0 && $D5/$D24 < $RAIL_MIN_RATIO)}"; then ratio_ok=0; fi
floor_ok=1
if awk "BEGIN{exit !($D5 < $RAIL_MIN_DUTY)}"; then floor_ok=0; fi

if [ "$floor_ok" = 0 ] || [ "$ratio_ok" = 0 ]; then
  echo "  RESULT: RAIL SAG — 5 GHz on-air power is collapsing on the control."
  echo "          5 GHz measurements are NOT trustworthy. Do not chase a 5 GHz"
  echo "          'code gate' / PA-bias / EFUSE bug until this is fixed."
  echo "          Fix: power-cycle the hub (uhubctl -l <hub> -a cycle), or move"
  echo "          the adapter to a powered USB hub / direct root port, then re-run."
  exit 1
fi
echo "  RESULT: rail OK — 5 GHz measurements are trustworthy."
exit 0
