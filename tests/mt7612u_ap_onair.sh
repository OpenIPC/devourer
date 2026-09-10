#!/usr/bin/env bash
# mt7612u_ap_onair.sh — the whole MT7612U AP claim, end to end, on hardware.
#
# Every number in docs/mt7612u-ap-mode.md's "Verified through IRadio" table
# comes from this script. It exists because those numbers were hand-run first,
# and three of the readings were wrong in ways that a script would not have
# repeated:
#
#   - "the beacon was gone after the process exited" measured nothing. Both AP
#     harnesses used to end in _exit(0), so StopBeacon never ran; the beacon
#     was still airing and one scan happened to miss it.
#   - `iw scan` without `flush` reports a stopped beacon as present for ~30 s
#     out of its BSS cache.
#   - a bring-up that fails leaves no beacon either, so "no beacon" read as a
#     pass when the AP had in fact never started. Every phase here checks the
#     AP came up BEFORE it believes an absence.
#
# Three cells, each with its own witness:
#
#   open   ap_responder     beacon -> scan, associate, ARP/ICMP ping
#   wpa2   ap_wpa2          beacon -> scan, 4-way handshake, encrypted ping
#   stop   beacon_stop_check armed -> stopped -> re-armed, by scan
#
# Bench: two MT7612U. One is the AP (devourer claims it); the other stays on
# the kernel mt76x2u driver and is the station. They are told apart by sysfs
# id, not by PID - they share one.
#
#   sudo tests/mt7612u_ap_onair.sh
#   sudo AP_SYSFS=5-1 STA_SYSFS=2-1 CH=36 tests/mt7612u_ap_onair.sh open
#
# Env: AP_SYSFS, STA_SYSFS, CH, PSK, FW_DIR, SECS. Cells: open|wpa2|stop|all.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
CELLS="${1:-all}"

AP_SYSFS="${AP_SYSFS:-5-1}"
STA_SYSFS="${STA_SYSFS:-2-1}"
CH="${CH:-36}"
FREQ=$(( CH < 15 ? 2407 + CH * 5 : 5000 + CH * 5 ))
PSK="${PSK:-devourer123}"
SECS="${SECS:-40}"
FW_DIR="${FW_DIR:-}"
APIP=192.168.99.1
STAIP=192.168.99.2
OUT="${OUT:-/tmp/mt7612u-ap-onair}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
mkdir -p "$OUT"
pass=0; fail=0
say()  { printf '%s\n' "$*"; }
ok()   { pass=$((pass+1)); printf '  PASS  %s\n' "$*"; }
bad()  { fail=$((fail+1)); printf '  FAIL  %s\n' "$*"; }

# PIDs this script started, so cleanup kills those and nothing else. `pkill -x
# wpa_supplicant` would drop every wireless client on the host, and a name kill
# would reach a concurrent run of this same test.
KIDS=""
reap() {
  local pid
  for pid in $KIDS; do kill "$pid" 2>/dev/null; done
  KIDS=""
}

cleanup() {
  reap
  [ -n "${STA_IF:-}" ] && { ip addr flush dev "$STA_IF" 2>/dev/null
                            iw dev "$STA_IF" disconnect 2>/dev/null; }
  # The MAC beacons autonomously. If a cell died before its teardown, only a
  # port power-cycle is certain to silence it - and leaving one airing poisons
  # the next run of this very script.
  #
  # Confirmed against the VID:PID first. This runs as root and writes a
  # deauthorize to a path the caller supplied; a stale or mistyped AP_SYSFS
  # would otherwise yank whatever else is plugged there - someone's keyboard,
  # a disk mid-write.
  if [ "$(cat "/sys/bus/usb/devices/$AP_SYSFS/idVendor" 2>/dev/null)" = "0e8d" ] &&
     [ "$(cat "/sys/bus/usb/devices/$AP_SYSFS/idProduct" 2>/dev/null)" = "7612" ]; then
    echo 0 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
    sleep 2
    echo 1 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
    sleep 3
  fi
}
trap cleanup EXIT INT TERM

# --- the station -----------------------------------------------------------
STA_IF=$(ls "/sys/bus/usb/devices/$STA_SYSFS:1.0/net/" 2>/dev/null | head -1)
if [ -z "$STA_IF" ]; then
  echo "$STA_SYSFS:1.0" > /sys/bus/usb/drivers_probe 2>/dev/null
  sleep 3
  STA_IF=$(ls "/sys/bus/usb/devices/$STA_SYSFS:1.0/net/" 2>/dev/null | head -1)
fi
[ -n "$STA_IF" ] || { echo "no station iface at $STA_SYSFS (is mt76x2u bound?)"; exit 2; }
ip link set "$STA_IF" up 2>/dev/null
say "AP $AP_SYSFS   station $STA_SYSFS ($STA_IF)   ch$CH ($FREQ MHz)"

# `flush` is not optional: without it the BSS cache reports a beacon that
# stopped up to ~30 s ago as still present, which is how a broken StopBeacon
# reads as working.
# Scan up to three times and take the HIGHEST count. A scan can come back empty
# for its own reasons - colliding with another scan, a busy card, a dwell that
# misses a 100 TU beacon - and one empty result is not evidence of absence.
# Taking the max is the conservative reading in BOTH directions: it cannot turn
# a live beacon into a pass for "gone", and it stops a missed scan reporting a
# live beacon as absent. Observed: a "beacon not scannable" FAIL in a run where
# the station then associated, pinged, and got an auth at retry=0.
seen() {   # $1 = SSID, $2 = BSSID
  local i n best=0
  for i in 1 2 3; do
    # Matched on BSSID *and* SSID: a neighbour running "devourerAP" would
    # otherwise pass an arm check, fail a stop check, or break the exact-count
    # comparison. awk keeps the pairing - grep -c on two patterns would count
    # them independently.
    n=$(iw dev "$STA_IF" scan flush freq "$FREQ" 2>/dev/null |
        awk -v b="$2" -v ss="SSID: $1" '
          /^BSS /   { cur = tolower($2); sub(/\(.*/, "", cur) }
          index($0, ss) { if (cur == tolower(b)) c++ }
          END { print c + 0 }')
    n=${n:-0}
    [ "$n" -gt "$best" ] && best=$n
    [ "$best" -gt 0 ] && break
    sleep 2
  done
  printf '%s' "$best"
}

apenv() {
  set -- DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 DEVOURER_CHANNEL="$CH" \
         DEVOURER_BCN_TU=100 DEVOURER_TX_WITH_RX=thread "$@"
  [ -n "$FW_DIR" ] && set -- DEVOURER_MT7612U_FW_DIR="$FW_DIR" "$@"
  printf '%s\n' "$@"
}

build() { # $1 = source stem, $2 = output name, $3.. = extra libs
  local src="$1" out="$2"; shift 2
  g++ -std=c++20 -O2 -I"$ROOT/src" -I"$ROOT/examples/common" \
      "$ROOT/tests/$src.cpp" "$ROOT/examples/common/env_config.cpp" \
      "$BUILD/libdevourer.a" $(pkg-config --cflags --libs libusb-1.0) \
      "$@" -lpthread -o "/tmp/$out" || return 1
}

# A cell must prove the AP CAME UP before it may believe any absence. A failed
# bring-up beacons nothing, which otherwise reads as a pass.
came_up() { grep -q "beaconing every" "$1"; }

# --- cell: open network ----------------------------------------------------
cell_open() {
  say "== open network (ap_responder) =="
  build ap_responder apr_onair || { bad "open: build"; return; }
  env $(apenv) timeout $((SECS + 20)) /tmp/apr_onair "$SECS" \
      >"$OUT/open.jsonl" 2>"$OUT/open.log" &
  local ap=$!; KIDS="$KIDS $ap"
  sleep 12
  came_up "$OUT/open.log" || { bad "open: AP did not come up (see $OUT/open.log)"; kill $ap 2>/dev/null; return; }
  ok "open: beacon armed"

  [ "$(seen devourerAP 02:42:75:05:d6:00)" = 1 ] && ok "open: beacon on air" || bad "open: beacon not scannable"

  ip addr flush dev "$STA_IF" 2>/dev/null
  if timeout 30 iw dev "$STA_IF" connect -w devourerAP >/dev/null 2>&1; then
    ok "open: station associated"
  else
    bad "open: station did not associate"; kill $ap 2>/dev/null; return
  fi

  ip addr add "$STAIP/24" dev "$STA_IF" 2>/dev/null
  ping -c 1 -W 2 -I "$STA_IF" "$APIP" >/dev/null 2>&1   # warm ARP
  if ping -c 6 -W 1 -I "$STA_IF" "$APIP" 2>&1 | tee "$OUT/open.ping" | grep -q " 0% packet loss"; then
    ok "open: data plane ($(grep -oE 'rtt [^ ]+ = [0-9./]+' "$OUT/open.ping" | head -1))"
  else
    bad "open: ping lost packets ($(grep -oE '[0-9]+% packet loss' "$OUT/open.ping" | head -1))"
  fi
  # retry=0 on auth IS the hardware ACK: an un-ACKed frame comes back with FC
  # Retry set. This is the only evidence that the APC slot and port identity
  # are both right.
  grep -q "AUTH req .* retry=0" "$OUT/open.log" \
    && ok "open: hardware auto-ACK (auth at retry=0)" \
    || bad "open: no auth at retry=0 - the MAC did not ACK"

  iw dev "$STA_IF" disconnect 2>/dev/null; ip addr flush dev "$STA_IF" 2>/dev/null
  wait $ap 2>/dev/null
  sleep 3
  [ "$(seen devourerAP 02:42:75:05:d6:00)" = 0 ] \
    && ok "open: nothing left airing after exit" \
    || bad "open: beacon STILL AIRING after exit"
}

# --- cell: WPA2-PSK --------------------------------------------------------
cell_wpa2() {
  say "== WPA2-PSK (ap_wpa2) =="
  build ap_wpa2 apw_onair -lcrypto || { bad "wpa2: build"; return; }
  env $(apenv) DEVOURER_WPA2_PSK="$PSK" timeout $((SECS + 20)) /tmp/apw_onair "$SECS" \
      >"$OUT/wpa2.jsonl" 2>"$OUT/wpa2.log" &
  local ap=$!; KIDS="$KIDS $ap"
  sleep 12
  came_up "$OUT/wpa2.log" || { bad "wpa2: AP did not come up (see $OUT/wpa2.log)"; kill $ap 2>/dev/null; return; }
  ok "wpa2: beacon armed"

  local wpa="$OUT/wpa.conf"
  printf 'network={\n\tssid="devourerAP"\n\tpsk="%s"\n\tkey_mgmt=WPA-PSK\n\tproto=RSN\n\tpairwise=CCMP\n\tgroup=CCMP\n\tscan_ssid=1\n}\n' "$PSK" > "$wpa"
  ip addr flush dev "$STA_IF" 2>/dev/null
  wpa_supplicant -i "$STA_IF" -c "$wpa" -P "$OUT/wpa.pid" -B >/dev/null 2>&1
  KIDS="$KIDS $(cat "$OUT/wpa.pid" 2>/dev/null)"
  local i
  for i in $(seq 1 20); do
    grep -q "4-WAY HANDSHAKE COMPLETE" "$OUT/wpa2.log" && break
    sleep 1
  done
  if grep -q "4-WAY HANDSHAKE COMPLETE" "$OUT/wpa2.log"; then
    ok "wpa2: 4-way complete (MIC verified, station keyed)"
  else
    bad "wpa2: 4-way did not complete"
    kill "$(cat "$OUT/wpa.pid" 2>/dev/null)" 2>/dev/null
    kill $ap 2>/dev/null; return
  fi

  ip addr add "$STAIP/24" dev "$STA_IF" 2>/dev/null
  ping -c 1 -W 2 -I "$STA_IF" "$APIP" >/dev/null 2>&1
  if ping -c 6 -W 1 -I "$STA_IF" "$APIP" 2>&1 | tee "$OUT/wpa2.ping" | grep -q " 0% packet loss"; then
    ok "wpa2: encrypted data plane ($(grep -oE 'rtt [^ ]+ = [0-9./]+' "$OUT/wpa2.ping" | head -1))"
  else
    bad "wpa2: encrypted ping lost packets"
  fi

  kill "$(cat "$OUT/wpa.pid" 2>/dev/null)" 2>/dev/null
  ip addr flush dev "$STA_IF" 2>/dev/null
  wait $ap 2>/dev/null
  sleep 3
  [ "$(seen devourerAP 02:42:75:05:d6:00)" = 0 ] \
    && ok "wpa2: nothing left airing after exit" \
    || bad "wpa2: beacon STILL AIRING after exit"
}

# --- cell: the beacon lifecycle -------------------------------------------
cell_stop() {
  say "== beacon lifecycle (StartBeacon / StopBeacon / re-arm) =="
  build mt7612u_beacon_stop_check bstop_onair || { bad "stop: build"; return; }
  local phase=24
  env $(apenv) timeout $((phase * 3 + 40)) /tmp/bstop_onair "$phase" \
      >"$OUT/stop.log" 2>&1 &
  local ap=$!; KIDS="$KIDS $ap"

  # Wait for the ARM ITSELF, not for the phase banner. The banner prints
  # before StartBeacon, and the arm is not instant - it copies a 1600-byte
  # page over EP0 and reads the identity back. Sleeping a guessed interval
  # after the banner is how phase 1 of this very cell reported "armed but not
  # scannable" while phase 3, which happened to sleep longer, passed.
  # grep -c PRINTS 0 and EXITS 1 when it matches nothing, so `|| echo 0`
  # appends a second line and every later [ -gt ] dies on "0\n0".
  armed() {
    local n
    n=$(grep -c "beaconing every" "$OUT/stop.log" 2>/dev/null)
    printf '%s' "${n:-0}"
  }
  wait_arm() { # $1 = the count to exceed, $2 = seconds to wait
    local i
    for i in $(seq 1 "$2"); do [ "$(armed)" -gt "$1" ] && return 0; sleep 1; done
    return 1
  }
  wait_gone() { # the log line that says StopBeacon ran, then a settle
    local i
    for i in $(seq 1 "$1"); do grep -q "PHASE 3" "$OUT/stop.log" && return 0; sleep 1; done
    return 1
  }

  wait_arm 0 30 || { bad "stop: never armed"; kill $ap 2>/dev/null; return; }
  sleep 4
  [ "$(seen mtStopCheck 02:4d:54:53:54:50)" = 1 ] && ok "stop: armed - beacon on air" || bad "stop: armed but not scannable"

  local n_arms; n_arms=$(armed)
  local i
  for i in $(seq 1 60); do grep -q "PHASE 2" "$OUT/stop.log" && break; sleep 1; done
  sleep 6
  [ "$(seen mtStopCheck 02:4d:54:53:54:50)" = 0 ] && ok "stop: stopped - beacon gone" || bad "stop: STILL AIRING after StopBeacon"

  # The re-arm is the same non-instant operation: wait for the second
  # "beaconing every", not for the banner that precedes it.
  wait_arm "$n_arms" 60 || { bad "stop: re-arm never reported"; kill $ap 2>/dev/null; return; }
  sleep 4
  [ "$(seen mtStopCheck 02:4d:54:53:54:50)" = 1 ] && ok "stop: re-armed - beacon back" || bad "stop: re-arm did not air"

  wait $ap 2>/dev/null
  grep -q "0 failure(s)" "$OUT/stop.log" \
    && ok "stop: local contract checks (2nd stop false, update-with-no-beacon false)" \
    || bad "stop: local contract checks failed"
}

case "$CELLS" in
  open) cell_open ;;
  wpa2) cell_wpa2 ;;
  stop) cell_stop ;;
  all)  cell_open; cleanup; cell_wpa2; cleanup; cell_stop ;;
  *)    echo "usage: $0 [open|wpa2|stop|all]"; exit 2 ;;
esac

say ""
say "=== $pass passed, $fail failed   (logs: $OUT) ==="
exit $(( fail > 0 ))
