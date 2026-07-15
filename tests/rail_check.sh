#!/usr/bin/env bash
# Rail-sag guard — re-check the known-good control adapter's 5 GHz on-air duty
# EACH SESSION before trusting any 5 GHz TX measurement.
#
# 5 GHz TX draws far more PA current than 2.4 GHz; fed through a bus-powered hub
# chain the rail browns out the PA, collapsing on-air power *intermittently* and
# worst on the most deeply-nested adapter — while frames still submit fine (0
# send-fails) and 2.4 GHz keeps working. The trap is to mis-diagnose this as a
# per-chip dead PA / a 5 GHz code gate / an EFUSE TX-power bug.
#
# The guard itself has a trap: the B210 has a degraded-read mode on back-to-back
# sessions (reads ~1.5% duty with a clean-looking noise floor while the air is
# in fact saturated), which used to false-trigger this check as "RAIL SAG". The
# check therefore now (a) takes the 5 GHz reading in the trustworthy
# first-read-of-session slot with the noise floor pinned from an idle
# measurement, (b) on a low reading: cools down, usbreset's the SDR and retries
# once, and (c) arbitrates a persistent low reading against an independent
# devourer GROUND adapter — the ground hearing the control at strong RSSI means
# the SDR is degraded, not the rail.
#
#   sudo tests/rail_check.sh
#   sudo CTRL_PID=0xc812 GROUND_PID=0xa81a RAIL_MIN_DUTY=50 tests/rail_check.sh
#
# Exit 0 = rail OK (5 GHz measurements trustworthy)
# Exit 1 = RAIL SAG (5 GHz on-air genuinely collapsing — fix the power feed)
# Exit 2 = SDR DEGRADED (rail is fine per the ground, but the B210 cannot be
#          trusted this session — reset/reseat it; bench numbers unreliable)
set -u
PY=${PY:-python3}
CTRL_VID=${CTRL_VID:-0x0bda}
CTRL_PID=${CTRL_PID:-0xc812}          # CU (RTL8812CU) — the validated 5 GHz control
GROUND_VID=${GROUND_VID:-0x0bda}
GROUND_PID=${GROUND_PID:-0xa81a}      # EU as the independent decode sensor
SECS=${SECS:-5}
RAIL_MIN_DUTY=${RAIL_MIN_DUTY:-50}    # absolute 5 GHz duty floor (%), tune per SDR setup
RAIL_MIN_RATIO=${RAIL_MIN_RATIO:-0.6} # 5GHz/2.4GHz duty ratio: warning-only
SDR_USB_ID=${SDR_USB_ID:-2500:0020}   # B210, for the usbreset recovery attempt
TXBIN=${TXBIN:-build/txdemo}
RXBIN=${RXBIN:-build/rxdemo}

cleanup(){ sudo pkill -9 -x txdemo 2>/dev/null; sudo pkill -x rxdemo 2>/dev/null; }
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

idle_floor() { # $1=freq -> echoes noise floor dB (idle, no TX)
  sudo $PY tests/sdr_duty.py --freq "$1" --rate 25e6 --secs 2 --gain 60 \
      --mcs 7 --bw 20 2>&1 | grep -oiE "noise=-?[0-9.]+" | head -1 | cut -d= -f2
}

duty_read() { # $1=freq $2=floor -> echoes duty%
  local args=(--freq "$1" --rate 25e6 --secs "$SECS" --gain 60 --mcs 7 --bw 20)
  [ -n "${2:-}" ] && args+=(--noise-db "$2")
  sudo $PY tests/sdr_duty.py "${args[@]}" 2>&1 | \
      grep -oiE "duty=[0-9.]+%" | head -1 | sed 's/duty=//;s/%//'
}

flood() { # $1=channel — start the control flooding (backgrounded)
  cleanup; sleep 2
  sudo env DEVOURER_VID=$CTRL_VID DEVOURER_PID=$CTRL_PID DEVOURER_CHANNEL=$1 \
       DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 \
       stdbuf -oL timeout -k 5 60 "$TXBIN" >/tmp/railchk_$1.log 2>&1 &
  sleep 5
}

echo "# rail_check: control $CTRL_VID:$CTRL_PID"
sudo rmmod rtl88x2eu_ohd 2>/dev/null
free_ctrl

# Idle noise floors first (also serves as the SDR warm-up read).
F5=$(idle_floor 5180e6); sleep 5
F24=$(idle_floor 2437e6); sleep 8
echo "  idle floors: 5 GHz ${F5:-?} dB, 2.4 GHz ${F24:-?} dB"

# 5 GHz reading in the fresh slot, floor pinned.
flood 36
D5=$(duty_read 5180e6 "$F5"); D5=${D5:-0}

if awk "BEGIN{exit !($D5 < $RAIL_MIN_DUTY)}"; then
  echo "  5 GHz duty low (${D5}%) — cooling down, resetting the SDR, retrying once"
  cleanup; sleep 15
  command -v usbreset >/dev/null && sudo usbreset "$SDR_USB_ID" >/dev/null 2>&1
  sleep 5
  flood 36
  D5=$(duty_read 5180e6 "$F5"); D5=${D5:-0}
fi

if awk "BEGIN{exit !($D5 < $RAIL_MIN_DUTY)}"; then
  # Persistent low reading: arbitrate against an independent devourer ground.
  if lsusb -d "${GROUND_VID#0x}:${GROUND_PID#0x}" >/dev/null 2>&1 && \
     [ "$GROUND_PID" != "$CTRL_PID" ]; then
    echo "  arbitrating with ground $GROUND_VID:$GROUND_PID ..."
    sudo env DEVOURER_VID=$GROUND_VID DEVOURER_PID=$GROUND_PID \
         DEVOURER_CHANNEL=36 DEVOURER_STREAM_OUT=1 \
         timeout 25 "$RXBIN" >/tmp/railchk_ground.log 2>/dev/null &
    sleep 22
    G=$(python3 - /tmp/railchk_ground.log <<'PYEOF'
import json, statistics, sys
n=0; rssi=[]
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    try: ev=json.loads(line)
    except ValueError: continue
    if ev.get("rate")==19:
        n+=1; rssi.append(ev["rssi"][0])
print(f"{n} {int(statistics.median(rssi)) if rssi else 0}")
PYEOF
)
    GN=${G% *}; GR=${G#* }
    echo "  ground heard n=$GN medRSSI=$GR"
    if [ "${GN:-0}" -gt 500 ] && [ "${GR:-0}" -gt 50 ]; then
      echo "  RESULT: SDR DEGRADED — the ground hears the control at strong RSSI"
      echo "          while the B210 reads ${D5}%. The rail is fine; the SDR is"
      echo "          not trustworthy this session. Reset/reseat the B210 (or"
      echo "          power-cycle its port) and re-run."
      exit 2
    fi
  fi
  echo "  RESULT: RAIL SAG — 5 GHz on-air power is collapsing on the control."
  echo "          5 GHz measurements are NOT trustworthy. Do not chase a 5 GHz"
  echo "          'code gate' / PA-bias / EFUSE bug until this is fixed."
  echo "          Fix: power-cycle the hub (uhubctl -l <hub> -a cycle), or move"
  echo "          the adapter to a powered USB hub / direct root port, then re-run."
  exit 1
fi

# 2.4 GHz reading (floor pinned — an unpinned floor on a busy 2.4 GHz channel
# under-reads duty and used to poison the ratio heuristic). Ratio is
# warning-only: the absolute 5 GHz floor above is the real guard.
cleanup; sleep 10
flood 6
D24=$(duty_read 2437e6 "$F24"); D24=${D24:-0}
cleanup
echo "  2.4 GHz duty = ${D24}%   5 GHz duty = ${D5}%"
if awk "BEGIN{exit !($D24 > 0 && $D5/$D24 < $RAIL_MIN_RATIO)}"; then
  echo "  WARN: 5GHz/2.4GHz duty ratio below $RAIL_MIN_RATIO — watch for partial sag."
fi
echo "  RESULT: rail OK — 5 GHz measurements are trustworthy."
exit 0
