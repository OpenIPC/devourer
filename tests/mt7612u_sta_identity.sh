#!/usr/bin/env bash
# mt7612u_sta_identity.sh - the two measurements behind the MT7612U half of
# SetStationIdentity, end to end:
#
#   BSSID     what does programming the BSSID do for a MANAGED STATION?
#   auto-ACK  does this MAC auto-ACK unicast to its own address with nothing
#             armed, and what happens to that if MT_MAC_ADDR is moved?
#
# The auto-ACK verdict itself comes from tests/mt7612u_sta_autoack.sh (it asks
# the transmitter); the `staack` gate run here is kept for the register state
# it prints and for its MT_MAC_ADDR-moved arm.
#
# Results and how to read them: docs/mt7612u-station-identity.md.
#
# Rig: a second adapter with AP-mode support runs hostapd and is the AP; the
# MT7612U is the device under test and is driven by build/mt7612uprobe. The
# BSSID does not choose the APC slot: mt76 keys a station's slot on the
# station's OWN address (sta_station_slot in the bring-up tool), which is slot
# 0 for a factory MAC - so on such a DUT the "slot 0" and "station slot" arms
# (C and D) write the same slot. The BSSID here is locally administered only
# so it cannot collide with a real device's address.
#
#   sudo tests/mt7612u_sta_identity.sh
#   sudo AP_SYSFS=1-1 DUT_SYSFS=7-1 CH=6 tests/mt7612u_sta_identity.sh
#
# RIG REQUIREMENT: both adapters at high speed (480 Mbit/s) or better, and no
# hub between either of them and the host that negotiates FULL speed (check
# with `lsusb -t`). That is necessary, not sufficient: an rtw88 AP (RTL8812BU,
# `rtw88_8822bu: failed to get tx report from firmware`) has stalled its
# transmit path under the 300 frames/s stimulus on a full-speed hub AND on a
# SuperSpeed root port, its beacons dropping with it
# (docs/mt7612u-station-identity.md). The harness catches that - the injector
# rate check refuses a table fed at under half the asked rate - but it cannot
# prevent it; a stalled run is re-run, not read.
#
# PORTABILITY:
#   - The AP is judged up by `iw dev <if> info` reporting `type AP`, not by
#     hostapd's log: some hostapd builds (Arch 2.11) accept -f but never write
#     the file. The -f log is still kept for diagnostics where it is written.
#   - An AP adapter can re-enumerate to a DIFFERENT sysfs path when its driver
#     first binds (seen: bus 9 -> 10, 3-2.3.3 -> 4-2.3.3). Read AP_SYSFS from
#     `lsusb -t` after the driver has loaded; a stale one is refused.
#
# Exit status: 0 every gate passed; 1 a gate failed; 2 INCONCLUSIVE (a gate
# could not measure, or the rig was refused); 3 interrupted (a gate, or the
# run itself by INT/TERM).
#
# Env: AP_SYSFS, DUT_SYSFS, CH, BSSID, SECS, OUT, FW_DIR.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
# The bring-up tool resolves its firmware directory RELATIVE TO THE WORKING
# DIRECTORY ("firmware/mt7662_rom_patch.bin"), and the symlink below is created
# at $ROOT. Running this script from anywhere else therefore fails the DUT's
# firmware load, which surfaces as "could not read the DUT's MAC" - a message
# that names neither the cause nor the cure. Pin the directory instead.
cd "$ROOT" || exit 1
AP_SYSFS="${AP_SYSFS:-1-1}"
DUT_SYSFS="${DUT_SYSFS:-7-1}"
CH="${CH:-6}"
BSSID="${BSSID:-02:42:75:05:d6:aa}"
SECS="${SECS:-20}"
# Unset: a fresh private directory (sta_out_prepare in the lib).
OUT="${OUT:-}"
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
command -v hostapd >/dev/null || { echo "hostapd is required"; exit 2; }
# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init hostapd inject gate

# mt7612uprobe takes no firmware-directory argument and looks for ./firmware,
# so give it one rather than requiring the caller to cd somewhere specific.
# Only a link THIS run created is removed afterwards - anything already at
# $ROOT/firmware, a dangling symlink included, is the operator's.
sta_fw_link || { sta_fw_unlink; sta_lock_release; exit 2; }

AP_IF=""
# The accepted AP's idVendor:idProduct:serial, recorded once the guard has
# passed; cleanup re-enumerates AP_SYSFS only while it still names this device.
AP_ID=""
# Set only once AP_SYSFS has passed the AP guard below, and before anything
# touches the interface: before that, the trap has no business
# re-enumerating anything (a wrong or default AP_SYSFS naming a hub would
# power-cycle every device under it).
AP_REENUM=no
CLEANED=no
# shellcheck disable=SC2317  # reached through the traps below
cleanup() {
  # Ignored, not deferred: a second INT/TERM during the hand-back would
  # otherwise end it half done (CLEANED is already set, so it cannot rerun).
  trap '' INT TERM
  [ "$CLEANED" = yes ] && return 0
  CLEANED=yes
  sta_fw_unlink
  sta_pid_kill inject
  local gate_gone=0
  sta_pid_kill gate || gate_gone=1
  # Never re-enumerate the DUT while its gate is still in de-init.
  if [ "$gate_gone" = 0 ]; then sta_dut_handback
  else echo "DUT gate still running - not re-enumerating DUT_SYSFS=$DUT_SYSFS"; fi
  # hostapd -B daemonizes; its PID is the one it wrote to -P for this run.
  # Unconditional: nothing is recorded unless hostapd started.
  sta_pid_kill hostapd
  [ "$AP_REENUM" = yes ] || { sta_lock_release; return 0; }
  sleep 1
  iw dev staid_mon del 2>/dev/null
  # RE-ENUMERATE the AP adapter, do not just bounce the link.
  #
  # hostapd's `bssid=` leaves the interface carrying that address after it
  # exits, and the adapter does not recover from `ip link down/up` - it comes
  # back still holding the BSSID, DOWN, and scanning nothing. Left that way it
  # silently breaks the next harness that expects this adapter to be a
  # station (tests/mt7612u_ap_onair.sh, for one).
  if [ -n "$AP_ID" ] && [ "$(sta_usb_id "$AP_SYSFS")" = "$AP_ID" ]; then
    echo 0 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
    sleep 3
    echo 1 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
    sleep 8
  else
    echo "AP_SYSFS=$AP_SYSFS no longer names the accepted AP ($AP_ID) -" \
         "not re-enumerating it"
  fi
  AP_IF=$(sta_first_netdev "$AP_SYSFS")
  [ -n "$AP_IF" ] && {
    rfkill unblock wlan 2>/dev/null
    ip link set "$AP_IF" up 2>/dev/null
    nmcli device set "$AP_IF" managed yes >/dev/null 2>&1
  }
  sta_lock_release
}
trap cleanup EXIT
# AND IT MUST STOP: with INT/TERM on the EXIT trap the shell runs cleanup
# and then CARRIES ON into the next arm. cleanup is idempotent, so the EXIT
# pass after it is harmless.
trap 'cleanup; exit 3' INT TERM

# --- the AP ----------------------------------------------------------------
# THE AP GUARD. Cleanup re-enumerates AP_SYSFS as root, so it is accepted only
# when ALL of these hold, checked before anything is written to it:
#   1. it is a USB device (idVendor readable) and not a hub (class 09);
#   2. it is not the DUT's path;
#   3. its interface 0 carries a WIRELESS netdev (a phy80211 link);
#   4. that netdev carries no default route, IPv4 or IPv6 - an adapter the
#      host is using for its uplink is never an AP here;
#   5. its phy advertises AP mode.
# Anything else is refused with the reason.
ap_refuse() {
  echo "refusing AP_SYSFS=$AP_SYSFS: $* - cleanup would re-enumerate it."
  exit 2
}
ap_cls=$(cat "/sys/bus/usb/devices/$AP_SYSFS/bDeviceClass" 2>/dev/null)
ap_vid=$(cat "/sys/bus/usb/devices/$AP_SYSFS/idVendor" 2>/dev/null)
[ -n "$ap_vid" ] || ap_refuse "not a USB device - if its driver just loaded, it may have moved; re-read lsusb -t"
[ "$ap_cls" != "09" ] || ap_refuse "a hub"
[ "$AP_SYSFS" != "$DUT_SYSFS" ] || ap_refuse "the DUT's own path"
AP_IF=$(sta_first_netdev "$AP_SYSFS")
if [ -z "$AP_IF" ]; then
  echo "$AP_SYSFS:1.0" > /sys/bus/usb/drivers_probe 2>/dev/null
  sleep 3
  AP_IF=$(sta_first_netdev "$AP_SYSFS")
fi
[ -n "$AP_IF" ] || ap_refuse "no network interface on it"
[ -e "/sys/class/net/$AP_IF/phy80211" ] || ap_refuse "$AP_IF is not wireless"
for fam in -4 -6; do
  ip "$fam" route show default 2>/dev/null |
    grep -qw "dev $AP_IF" && ap_refuse "$AP_IF carries a default route"
done
PHY=$(basename "$(readlink -f "/sys/class/net/$AP_IF/phy80211")")
iw phy "$PHY" info 2>/dev/null | grep -q '\* AP$' ||
  ap_refuse "$AP_IF ($PHY) does not support AP mode"
AP_ID=$(sta_usb_id "$AP_SYSFS")
# The BSSID gate needs a monitor vif on the AP's phy. Probed here, before
# the gates spend their minute, and again (unchanged) where it is used.
iw dev staid_mon del 2>/dev/null
if ! iw phy "$PHY" interface add staid_mon type monitor 2>/dev/null; then
  echo "no monitor vif on $PHY: the BSSID gate would measure broadcast"
  echo "reception only - refusing this AP."
  exit 2
fi
iw dev staid_mon del 2>/dev/null
# From here on the trap restores the AP: everything below changes it.
AP_REENUM=yes

echo "AP  $AP_IF ($PHY) bssid $BSSID ch$CH"
echo "DUT $DUT_SYSFS (MT7612U)"

# hostapd and NetworkManager fight over the interface; and a previous run can
# leave the vif in AP type, which makes hostapd fail with "Match already
# configured" rather than anything that names the real problem.
nmcli device set "$AP_IF" managed no >/dev/null 2>&1
sleep 1
ip link set "$AP_IF" down 2>/dev/null
iw dev "$AP_IF" set type managed 2>/dev/null
ip link set "$AP_IF" up 2>/dev/null

cat > "$OUT/hostapd.conf" <<EOF
interface=$AP_IF
driver=nl80211
ssid=staidentity
bssid=$BSSID
hw_mode=g
channel=$CH
auth_algs=1
wmm_enabled=0
EOF
hostapd -B -P "$OUT/.pid_hostapd" -f "$OUT/hostapd.log" "$OUT/hostapd.conf" \
    >/dev/null 2>&1
# Up means the interface is in AP mode, read from the kernel (host-
# independent); the log is diagnostics only.
ap_up=no
for _ in 1 2 3 4 5 6 7 8 9 10; do
  if iw dev "$AP_IF" info 2>/dev/null | grep -q 'type AP'; then
    ap_up=yes; break
  fi
  sleep 1
done
if [ "$ap_up" != yes ]; then
  echo "hostapd did not bring $AP_IF up in AP mode:"
  tail -12 "$OUT/hostapd.log" 2>/dev/null || echo "(no hostapd log written)"
  exit 2   # the rig, not the DUT
fi

# --- free the DUT ----------------------------------------------------------
sta_dut_take || exit 2

# The contract gate needs no AP, prints the DUT's own address, and is the
# cheapest thing that fails loudly if the DUT is not usable - so it runs
# first and doubles as this harness's source for the MAC. (`regs` does not
# work for this: it never calls mt_eeprom_init, so it prints no MAC.)
echo
echo "########## the SetStationIdentity contract (no AP needed) ##########"
# PIPESTATUS[0], captured straight after each pipeline: `$?` is tee's
# status, so a failing probe would score as a pass. PIPESTATUS is a bash-ism,
# which is why this script is #!/usr/bin/env bash and not #!/bin/sh.
"$BUILD/mt7612uprobe" staid 2>&1 | tee "$OUT/staid.txt"
staid=${PIPESTATUS[0]}
DUT_MAC=$(sed -n 's/^own \([0-9a-f:]\{17\}\).*/\1/p' "$OUT/staid.txt" | head -1)
[ -n "$DUT_MAC" ] || { echo "could not read the DUT's MAC from the staid gate"; exit 1; }
echo "DUT MAC $DUT_MAC"

# --- probe-response gate: no monitor vif needed ----------------------------
# Its auto-ACK VERDICT is not evidence and reads INCONCLUSIVE (rc 2) against
# hostapd by construction - the control cannot move (mt7612uprobe's gate_staack
# comment; the auto-ACK answer is tests/mt7612u_sta_autoack.sh). What this
# harness takes from it is arm C: MT_MAC_ADDR moved under the managed filter
# receives nothing. rc 1 is an error; rc 0 or 2 is a run, judged on arm C.
echo
echo "########## probe-response gate (arm C: MT_MAC_ADDR moved) ##########"
"$BUILD/mt7612uprobe" staack "$CH" "$SECS" "$BSSID" 2>&1 | tee "$OUT/staack.txt"
r_ack=${PIPESTATUS[0]}
if [ "$r_ack" = 0 ] || [ "$r_ack" = 2 ]; then
  if grep -q '^C (MT_MAC_ADDR moved)   : received NOTHING' "$OUT/staack.txt"; then
    r_ack=0
    echo "arm C: received nothing with MT_MAC_ADDR moved - as recorded"
  elif grep -q '^C (MT_MAC_ADDR moved)   : *[0-9]' "$OUT/staack.txt"; then
    echo "arm C RECEIVED with MT_MAC_ADDR moved - contradicts the record"
    r_ack=1
  else
    # Arm A or B got no probe response and the gate stopped before arm C.
    echo "the gate stopped before reporting arm C - no measurement"
    r_ack=2
  fi
fi

# --- BSSID: needs unicast aimed at the DUT for the whole run ----------------
echo
echo "########## BSSID: what does programming the BSSID change? ##########"
iw dev staid_mon del 2>/dev/null
if ! { iw phy "$PHY" interface add staid_mon type monitor 2>/dev/null &&
       ip link set staid_mon up 2>/dev/null; }; then
  echo "no monitor vif on $PHY: the BSSID gate would measure broadcast"
  echo "reception only, which is not the question - refusing to run it."
  exit 2   # the rig, not the DUT
fi
# ORDER MATTERS. The gate's bring-up runs the MT7612U's calibrations, whose
# MCU replies arrive late under a strong transmitter nearby (mcu.cpp); the
# unicast stimulus is a 300 pps flood from 20 cm. So the gate starts first,
# the injector only once the gate prints "bring-up done", and the gate then
# pauses 3 s before arm A so the stimulus covers every arm.
: > "$OUT/bssid.txt"
# BOUNDED: six arms of SECS, a 3 s pause, and 3 min for bring-up and slack.
# INT lets the gate restore the registers (exit 3); KILL 10 s later if not.
gate_bound=$(( SECS * 6 + 183 ))
timeout -s INT -k 10 "$gate_bound" \
    "$BUILD/mt7612uprobe" sta "$CH" "$SECS" "$BSSID" > "$OUT/bssid.txt" 2>&1 &
sta_gate=$!
sta_pid_record gate "$sta_gate"
waited=0
until grep -q '^GATE STA: bring-up done' "$OUT/bssid.txt" 2>/dev/null; do
  if ! kill -0 "$sta_gate" 2>/dev/null || [ "$waited" -ge 120 ]; then
    break
  fi
  sleep 1; waited=$((waited + 1))
done
inj_t0=$(date +%s)
if grep -q '^GATE STA: bring-up done' "$OUT/bssid.txt" 2>/dev/null; then
  # Without this every arm reads to_us=0: hostapd sends an unassociated
  # station no unicast, so the gate would measure broadcast reception only
  # and could not answer the question it exists for.
  python3 "$ROOT/tests/sta_unicast_inject.py" staid_mon "$DUT_MAC" "$BSSID" \
      $(( SECS * 6 + 40 )) 300 > "$OUT/inject.log" 2>&1 &
  sta_pid_record inject $!
  echo "(unicast injector running on staid_mon)"
fi
wait "$sta_gate"
r_bss=$?
rm -f "$OUT/.pid_gate"
gate_overran=no
if [ "$r_bss" = 124 ] || [ "$r_bss" = 137 ]; then
  echo "the BSSID gate overran its ${gate_bound}s bound - no measurement"
  r_bss=2; gate_overran=yes
fi
inj_secs=$(( $(date +%s) - inj_t0 ))
cat "$OUT/bssid.txt"
# Stop the injector (it prints its count on SIGTERM) and require that it
# actually injected - and at a plausible rate: an AP whose transmit path has
# stalled (rtw88 logs "failed to get tx report from firmware") sends a
# fraction of what was asked while its beacons stop too, and the gate then
# sees an empty channel that is the AP's fault, not the DUT's.
sta_pid_kill inject
injected=$(sed -n 's/^injected \([0-9][0-9]*\) unicast frames.*/\1/p' "$OUT/inject.log" | tail -1)
if [ "${injected:-0}" -gt 0 ] 2>/dev/null; then
  echo "injector: $injected unicast frames at $DUT_MAC in ${inj_secs}s (asked 300/s)"
  if [ "$inj_secs" -gt 0 ] && [ $((injected / inj_secs)) -lt 150 ]; then
    echo "the injector achieved under half its rate: the AP's transmit path"
    echo "stalled (check the kernel log for its driver), so this BSSID table"
    echo "is not a measurement of the DUT."
    [ "$r_bss" = 0 ] && r_bss=2
  fi
else
  echo "the unicast injector injected NOTHING (see $OUT/inject.log) - the BSSID"
  echo "table measured broadcast reception only."
  # An interrupted or overrun gate stays "no verdict": a bring-up that
  # wedged before the injector started is not a failed measurement.
  [ "$r_bss" = 3 ] || [ "$gate_overran" = yes ] || r_bss=1
fi

echo
echo "=== logs: $OUT ==="
# A gate's rc 2 is INCONCLUSIVE and rc 3 INTERRUPTED: neither is a pass, and
# neither is reported as a failure.
[ "${staid:-0}" = 0 ] || echo "the contract gate FAILED - see $OUT/staid.txt"
case "$r_ack" in
  0) ;;
  2) echo "the probe-response gate stopped before arm C - INCONCLUSIVE, see $OUT/staack.txt" ;;
  3) echo "the probe-response gate was INTERRUPTED - no verdict" ;;
  *) echo "the probe-response gate's arm C did not hold - see $OUT/staack.txt" ;;
esac
case "$r_bss" in
  0) ;;
  2) echo "the BSSID gate could not measure - INCONCLUSIVE, see $OUT/bssid.txt" ;;
  3) echo "the BSSID gate was INTERRUPTED - no verdict" ;;
  *) echo "the BSSID gate did not pass - see $OUT/bssid.txt" ;;
esac
# 1 if any gate failed, else 3 if any was interrupted, else 2 if any could
# not measure, else 0.
rc=0
for r in "$r_bss" "$r_ack" "${staid:-0}"; do
  case "$r" in
    0) ;;
    3) [ "$rc" = 1 ] || rc=3 ;;
    2) [ "$rc" = 0 ] && rc=2 ;;
    *) rc=1 ;;
  esac
done
exit "$rc"
