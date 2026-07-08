#!/usr/bin/env bash
# Adapter-capabilities smoke: open each plugged devourer-supported adapter in
# turn, run rxdemo just long enough to emit the `adapter.caps` JSONL event
# (examples/common/caps_event.h), and print it. Validates the Part-A API on real
# silicon across generations — chip name, chain counts, bandwidth set, tunable /
# characterized frequency spans, feature flags.
#
#   sudo tests/adapter_caps_probe.sh                 # probe a default PID list
#   PIDS="8812 8813 012d" sudo tests/adapter_caps_probe.sh   # explicit set
#
# Each PID is a USB product id (DEVOURER_PID); VID defaults to Realtek 0x0bda,
# override per-PID as VID:PID (e.g. 2357:012d for the TP-Link 8812BU).
set -u
cd "$(dirname "$0")/.."

# Default set covers what's typically on the bench; missing ones are skipped.
PIDS=${PIDS:-"8812 8813 c812 c811 a81a 2357:012d"}

cleanup() { sudo pkill -x rxdemo 2>/dev/null; }
trap cleanup EXIT

have_jq=0; command -v jq >/dev/null 2>&1 && have_jq=1

fail=0
for spec in $PIDS; do
  if [[ "$spec" == *:* ]]; then vid=${spec%%:*}; pid=${spec##*:}; else vid=0bda; pid=$spec; fi
  # Skip PIDs that aren't actually plugged in.
  if ! lsusb -d "${vid}:${pid}" >/dev/null 2>&1; then
    echo "-- ${vid}:${pid}: not present, skipping"
    continue
  fi
  echo "== probing ${vid}:${pid} =="
  log=$(mktemp)
  # rxdemo emits adapter.caps right after CreateRtlDevice, well before the RX
  # loop; 6 s covers bring-up on the slowest chip. 2>/dev/null = pure events.
  sudo timeout 6 env DEVOURER_VID="0x${vid}" DEVOURER_PID="0x${pid}" \
      DEVOURER_LOG_LEVEL=silent build/rxdemo >"$log" 2>/dev/null || true
  cleanup
  caps=$(grep -F '"ev":"adapter.caps"' "$log" | head -1)
  if [ -z "$caps" ]; then
    echo "   FAIL: no adapter.caps event emitted"
    fail=$((fail + 1))
  elif [ "$have_jq" = 1 ]; then
    echo "$caps" | jq -c '{chip,names,gen,variant,transport,tx_chains,rx_chains,bw,tune_5g,char_5g,per_pkt_txpwr,narrowband,per_chain_rssi}'
  else
    echo "   $caps"
  fi
  rm -f "$log"
done

echo "=== adapter_caps_probe: $fail adapter(s) failed to report caps ==="
[ "$fail" -eq 0 ]
