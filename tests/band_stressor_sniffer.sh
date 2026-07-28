#!/usr/bin/env bash
# Does band switching actually degrade the TRANSMITTER? Judged by an independent
# receiver, so the answer cannot be an artifact of the devourer link.
#
# The band-switch arm of the severe-form hunt showed MCS7 collapsing to 0% while
# MCS1 held at 97%, measured over a devourer->devourer link. That looked like the
# #348 severe form. It was not safe to believe: minutes later the same
# transmitter delivered MCS7 at 62% to an AR9271 sniffer, and a fresh
# power-cycled devourer receiver still read ~0%. Delivery is a property of the
# whole link, and the original #348 report was measured the same way.
#
# So: measure the transmitter alone. The AR9271 (ath9k_htc) shares no silicon,
# firmware or driver code with the DUT, and it is not being stressed. Anything it
# still decodes was transmitted correctly.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3" tests/band_stressor_sniffer.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"

DUT_VID=${DUT_VID:-0bda} DUT_PID=${DUT_PID:-8812}
SNIFF=${SNIFF:-wlp13s0u1u3}
CH=${CH:-6}
RATES=${RATES:-"7 4 1"}
SESSIONS=${SESSIONS:-8}
SECS=${SECS:-6}
PAYLOAD=${PAYLOAD:-1400}
CANON=57:42:75:05:d6:00
OUT=${OUT:-/tmp/devourer-band-sniffer}
mkdir -p "$OUT"

cleanup() { sudo pkill -f "tcpdump -i $SNIFF" 2>/dev/null; sudo pkill -x txdemo 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

[ -e "/sys/class/net/$SNIFF" ] || { echo "no sniffer $SNIFF" >&2; exit 2; }
sudo ip link set "$SNIFF" down; sudo iw dev "$SNIFF" set type monitor
sudo ip link set "$SNIFF" up;   sudo iw dev "$SNIFF" set channel "$CH"
sleep 1

cold_cycle() {
  local map=${REGRESS_VBUS_MAP:-} entry hubport
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$DUT_VID:$DUT_PID=" | head -1) || return 1
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

# Transmit at MCS $1 and count what the sniffer decodes, and at which rate.
probe() {
  local mcs="$1" tag="$2"
  local cap="$OUT/${tag}.txt"
  sudo timeout $((SECS + 6)) tcpdump -i "$SNIFF" -nn -e "wlan addr2 $CANON" \
      > "$cap" 2>/dev/null &
  sleep 1
  local sent
  sent=$(sudo env DEVOURER_VID="0x$DUT_VID" DEVOURER_PID="0x$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="MCS$mcs/20" \
    DEVOURER_TX_PAYLOAD_BYTES="$PAYLOAD" DEVOURER_TX_GAP_US=2000 \
    DEVOURER_TEARDOWN_POWER_DOWN=0 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 10)) "$TXDEMO" 2>/dev/null |
    grep -o '"submitted":[0-9]*' | tail -1 | cut -d: -f2)
  sleep 2
  local n rates
  n=$(grep -c "$CANON" "$cap" 2>/dev/null || echo 0)
  rates=$(grep -oE "MCS [0-9]+" "$cap" 2>/dev/null | sort | uniq -c | tr -s ' ' | tr '\n' ' ')
  printf '%s %s %s' "${sent:-0}" "$n" "${rates:-none}"
}

row() { # $1=label
  local label="$1" m out sent rx rates
  printf '  %-22s' "$label"
  for m in $RATES; do
    read -r sent rx rates <<<"$(probe "$m" "${label// /_}_mcs$m")"
    printf ' MCS%-2s rx=%-6s' "$m" "$rx"
  done
  printf '\n'
}

echo "band-switch stressor, judged by $SNIFF (independent receiver)"
echo "  sniffer ambient: $(sudo timeout 3 tcpdump -i "$SNIFF" -nn 2>/dev/null | wc -l) frames/3s"
echo
cold_cycle || { echo "cold cycle failed" >&2; exit 1; }
row "before (cold)"

echo "  (applying $SESSIONS band-switching sessions 2.4 <-> 5 GHz ...)"
i=0
while [ "$i" -lt "$SESSIONS" ]; do
  c=$([ $((i % 2)) -eq 0 ] && echo 36 || echo "$CH")
  sudo env DEVOURER_VID="0x$DUT_VID" DEVOURER_PID="0x$DUT_PID" \
    DEVOURER_CHANNEL="$c" DEVOURER_TX_RATE="MCS7/20" DEVOURER_TX_GAP_US=0 \
    DEVOURER_TEARDOWN_POWER_DOWN=0 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM 14 "$TXDEMO" >/dev/null 2>&1
  i=$((i + 1))
done
row "after band sessions"

echo
echo "a real transmitter defect shows the sniffer's high-MCS count collapsing."
echo "unchanged counts mean the transmitter is fine and the earlier collapse was"
echo "a property of the devourer receive link, not of TX."
