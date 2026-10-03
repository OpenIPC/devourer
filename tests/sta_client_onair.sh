#!/usr/bin/env bash
# sta_client_onair.sh - tests/sta_client.cpp joining a real hostapd AP, end
# to end, with the station identity armed through IRadio::SetStationIdentity.
#
# The DUT (DUT_SYSFS) runs sta_client: scan, authenticate, associate, the
# WPA2-PSK four-way and CCMP over src/sta/, a TAP device for the host. It is
# an MT7612U (0e8d:7612) or a Realtek die whose AdapterCaps::station_mode_ok
# is true - the 8822C (RTL8812CU, 0bda:c812) and the 8822B (RTL8812BU,
# 0bda:b812). A kernel-driven adapter (AP_SYSFS) runs hostapd - an
# independent implementation on independent silicon, which is what makes its
# log a witness: "EAPOL-4WAY-HS-COMPLETED" means the AUTHENTICATOR verified
# our message 4's MIC. The AP need not be a Realtek.
#
# THE AP LIVES IN A NETWORK NAMESPACE. Both radios are on one host; with both
# addresses in the root namespace the kernel routes the ping locally and it
# never touches the air. The phy moves with `iw phy <phy> set netns`, and
# every data-plane check first asserts that `ip route get` leaves through the
# station's TAP.
#
# THE ARM DIFFERS BY DIE. On MT7612U the arm writes no identity register (it
# checks MT_MAC_ADDR and the auto-responder) but installs the managed receive
# filter 0x00015f97 in place of the RX loop's monitor filter, so an unarmed
# station still gets in and acknowledges, and what DEVOURER_STA_ARM=0 changes
# is the filter: `noarm` is the filter's control there (the unicast injection
# below). On a Realtek die the arm WRITES the port registers
# (docs/realtek-station-arm.md) and unarmed the MAC does not acknowledge
# own-addressed unicast, so the AP's authentication response is never ACKed
# and hostapd never lets the station in: `noarm` is the arm's control. On
# both, the clear restores what the arm wrote and its verification is scored.
#
# THE MANAGED-FILTER STIMULUS (MT7612U only; on a Realtek DUT it is skipped,
# said as INFO). Once the wpa2 / noarm four-way is in, a monitor vif on the
# AP's phy injects two plaintext unicast data streams from the AP's BSSID
# for INJECT_S at INJECT_PPS each (tests/sta_unicast_inject.py): one to
# FOREIGN (an address nobody holds), one to the station's own address. The
# own stream is the positive witness that the injection reaches the DUT - the
# station refuses it on a WPA2 link and counts it (`plaintext refused`), and
# it must reach half of what was injected, else the filter check is
# INCONCLUSIVE. A phy that cannot add a monitor vif makes the check
# INCONCLUSIVE, not the cell.
#
# Cells (each scored against its own witness):
#   open     hostapd open, with a ping running from the start: the AP
#            associates OUR address (hostapd's AP-STA-CONNECTED - its own
#            record) within 30 s, which covers recovering a first
#            association the AP did not hold (sta_client nudges the AP with
#            a probe request on associating, and re-joins when its ARP gets
#            no unicast reply, kConfirmMs); ping 0% loss over
#            the TAP; the ledger shows plaintext and no decryption; the arm
#            line; the clear ran on exit and verified.
#   wpa2     hostapd WPA2-PSK with group and pairwise rekeys: four-way, both
#            rekeys completed at the AP, ping 0% loss before and after them,
#            ONE association throughout, no four-way MIC failure, and
#            data-plane MIC failures <= PTK installs (a pairwise rekey has a
#            one-frame switchover window in which the AP still sends under
#            the old key - see rx_frame() in sta_client.cpp), the arm line, the clear (as for open), and
#            NO tx.retry_limit=0 warning (the station default is nonzero).
#            MT7612U also: the managed filter - the own stream arrives and
#            `not-for-us` stays under 1% of the foreign one; that PASS is
#            held until the noarm control of the same run has seen the
#            foreign stream arrive, else INCONCLUSIVE.
#   noarm    the control: wpa2 with DEVOURER_STA_ARM=0 - nothing else
#            changes. Scored everywhere: no SetStationIdentity and no clear
#            ran. On Realtek also scored: the station tried (beacons seen,
#            authentication sent) and the AP did NOT complete the four-way
#            for it within 30 s - a completed four-way is a FAIL, because then
#            the arm is not what makes the wpa2 cell work. Its positive
#            control is the wpa2 cell of the SAME run: without an armed
#            four-way against the same hostapd configuration, a silent AP
#            proves nothing and noarm is INCONCLUSIVE. On MT7612U also
#            scored: under the monitor filter BOTH injected streams arrive
#            (each at least half); the link is reported over a PING_S ping
#            window, not scored.
#   retry0   wpa2 with DEVOURER_TX_RETRY_LIMIT=0. Scored: the library's
#            arm-time warning about tx.retry_limit=0 (logged inside a
#            successful SetStationIdentity, so just before sta_client's
#            "armed" line), and the clear. Reported, not scored: the link
#            over a PING_S ping window with a single-shot uplink.
#   reconnect   hostapd WPA2-PSK, stopped for DOWN_S and restarted with the
#            same configuration. Scored: the station reports the lost link;
#            the AP completes a second four-way for it within REJOIN_S of
#            hostapd being started again; ping 0% loss over a PING_S window after the
#            re-join; the ledger counts 2 associations and 1 reconnect;
#            exactly ONE arm (the arm is per BSSID and stays in place across
#            a re-join to the same BSSID - on Realtek the second association
#            is itself the proof that it still holds); the clear on exit.
#   noreconnect   reconnect with DEVOURER_STA_RECONNECT=0: the station reports
#            the lost link, the AP sees NO second four-way within REJOIN_S,
#            and the ledger ends Failed with 1 association.
#
# Liveness: a data-plane check runs only while sta_client is alive, and again
# checks it afterwards - a ping that straddles the station's exit reports
# loss on a working link. A station that exits, or is not up within
# READY_TIMEOUT, before printing `sta_client up:` is a rig / bring-up problem
# (INCONCLUSIVE, whatever its status). After `up:`, an exit with status 0
# ran out of SECS (INCONCLUSIVE); 3 is a FAULT the station caught (an
# exception, a failed TAP or an unverified clear; `fault=1` in its ledger)
# and is a FAIL with the cause named, wherever in the cell it happens; any
# other status is a FAIL.
#
# Exit status: 0 every scored check passed; 1 a check failed; 2 INCONCLUSIVE
# (the rig was refused, the station did not come up, the AP did not come up,
# the route did not leave through the TAP, or a cell was cut short);
# 3 interrupted.
#
#   sudo DUT_SYSFS=1-1 AP_SYSFS=8-1 tests/sta_client_onair.sh
#   sudo DUT_SYSFS=5-1 AP_SYSFS=1-1 CH=6 tests/sta_client_onair.sh wpa2 noarm
#
# Rig: DUT_SYSFS an MT7612U, an RTL8812CU or an RTL8812BU (another die: set
# DUT_VID / DUT_PID; sta_client refuses it unless its station_mode_ok is
# true). Its kernel driver (mt76x2u, rtw88, or an out-of-tree rtl88x2*) is
# unbound here and the device re-enumerated at the end, never while
# sta_client is alive. AP_SYSFS an adapter whose kernel driver supports AP
# mode AND lets its phy change network namespace (`iw phy` lists
# set_wiphy_netns): an in-kernel cfg80211 driver such as mt76 or rtw88.
# Out-of-tree drivers such as rtl88x2cu / 88x2bu cannot, and are refused.
# Read both from `lsusb -t` after the drivers have loaded (they can move).
# FW_DIR (an MT7612U DUT only) must hold the DECOMPRESSED MT7612U blobs
# (mt7662*.bin); a host that ships only mt7662*.bin.zst gets INCONCLUSIVE
# (rig/bring-up).
# Build first: cmake --build build --target StaClientSelftest (build/sta_client).
#
# HOSTAPD_DEBUG=1: hostapd runs with -dd, its debug output in the same
# per-cell log (hostapd_<cell>.log), for the AP's view of an association
# (sta_add, TX status). Meant for the open cell: the extra lines can repeat
# the text the wpa2 cell counts (rekey completions), so read its rekey
# verdicts with that in mind. Whatever the setting, a cell that FAILs saves
# the kernel log's tail (dmesg_<cell>.txt) and prints its mt76 / rtw88 /
# cfg80211 lines - read-only, best effort.
#
# AP_OFDM_ONLY=1 (2.4 GHz): hostapd advertises and uses OFDM rates only
# (no 1/2/5.5/11 Mb/s), so its management frames - authentication and
# association responses - go out at 6 Mb/s OFDM instead of 1 Mb/s CCK. A
# diagnostic: whether a station's association depends on CCK.
#
# Env: DUT_SYSFS, AP_SYSFS, DUT_VID, DUT_PID, CH, SSID, PSK, SECS, REKEY_S,
#      PTK_REKEY_S, PING_S, DOWN_S, REJOIN_S, AP_OFDM_ONLY, HOSTAPD_DEBUG,
#      INJECT_S, INJECT_PPS, FOREIGN, FW_DIR, NS, TAP, READY_TIMEOUT, OUT,
#      BUILD.
# Cells: open | wpa2 | noarm | retry0 | reconnect | noreconnect | all
# (default: all six).

# The cells are reached as "cell_$c" and cleanup through the traps.
# shellcheck disable=SC2317
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
DUT_SYSFS="${DUT_SYSFS:-}"
AP_SYSFS="${AP_SYSFS:-}"
DUT_VID="${DUT_VID:-}"
DUT_PID="${DUT_PID:-}"
CH="${CH:-6}"
SSID="${SSID:-devourerSTA}"
PSK="${PSK:-devourer123}"
# Budget from process start, not from association: firmware load, the TAP and
# the association all come before the measurement.
SECS="${SECS:-90}"
# hostapd's group and pairwise rekey intervals for the wpa2 cell. Both must
# fire inside the run: the rekeys travel inside the cipher, a path the
# four-way alone never exercises.
REKEY_S="${REKEY_S:-20}"
PTK_REKEY_S="${PTK_REKEY_S:-25}"
# The ping window behind every reported link line and the reconnect cell's
# post-re-join check: PING_S seconds at 2 pings a second.
PING_S="${PING_S:-30}"
# reconnect: how long the AP is away, and the bound on the re-join once it is
# back (loss noticed, re-join backoff, authentication, association, four-way).
DOWN_S="${DOWN_S:-8}"
REJOIN_S="${REJOIN_S:-30}"
AP_OFDM_ONLY="${AP_OFDM_ONLY:-0}"
HOSTAPD_DEBUG="${HOSTAPD_DEBUG:-0}"
# The managed-filter stimulus (wpa2, noarm; MT7612U DUT). FOREIGN is locally
# administered and held by nobody on the rig.
INJECT_S="${INJECT_S:-10}"
INJECT_PPS="${INJECT_PPS:-100}"
FOREIGN="${FOREIGN:-02:00:00:de:ad:01}"
MON=staon_mon
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"
NS="${NS:-staonair}"
TAP="${TAP:-dvsta0}"
READY_TIMEOUT="${READY_TIMEOUT:-30}"
OUT="${OUT:-}"
APIP=192.168.98.1
STAIP=192.168.98.2

CELLS="${*:-all}"
[ "$CELLS" = all ] && CELLS="open wpa2 noarm retry0 reconnect noreconnect"
for c in $CELLS; do
  case "$c" in open|wpa2|noarm|retry0|reconnect|noreconnect) ;;
    *) echo "unknown cell '$c'"; exit 2 ;; esac
done

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
for v in CH SECS REKEY_S PTK_REKEY_S PING_S DOWN_S REJOIN_S AP_OFDM_ONLY HOSTAPD_DEBUG READY_TIMEOUT INJECT_S INJECT_PPS; do
  case "${!v}" in ''|*[!0-9]*) echo "$v must be a non-negative integer"; exit 2 ;; esac
done
[ "$PING_S" -ge 1 ] || { echo "PING_S must be at least 1"; exit 2; }
if [ "$INJECT_S" -lt 1 ] || [ "$INJECT_PPS" -lt 1 ] || [ "$INJECT_PPS" -gt 2000 ]; then
  echo "INJECT_S must be at least 1, INJECT_PPS 1..2000 (the injector's cap)"; exit 2
fi
[ -n "$DUT_SYSFS" ] || { echo "DUT_SYSFS is required (lsusb -t)"; exit 2; }
[ -n "$AP_SYSFS" ] || { echo "AP_SYSFS is required (lsusb -t)"; exit 2; }
[ "$DUT_SYSFS" != "$AP_SYSFS" ] || { echo "DUT_SYSFS and AP_SYSFS must differ"; exit 2; }
command -v hostapd >/dev/null || { echo "hostapd is required"; exit 2; }
[ -x "$BUILD/sta_client" ] || {
  echo "$BUILD/sta_client is not built (cmake --build build --target StaClientSelftest)"; exit 2; }
if [ -e "/sys/class/net/$TAP" ]; then
  echo "TAP=$TAP already exists - refusing to use or delete it (set TAP=)"; exit 2
fi
ns_exists() { ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$NS"; }
if ns_exists; then
  echo "netns $NS already exists - recover or remove it first (set NS= to use another name)"
  exit 2
fi

# --- which die the DUT is -----------------------------------------------------
# DUT_KIND decides the take / hand-back and what the arm scores.
dut_have="$(cat "/sys/bus/usb/devices/$DUT_SYSFS/idVendor" 2>/dev/null):$(cat "/sys/bus/usb/devices/$DUT_SYSFS/idProduct" 2>/dev/null)"
if [ -n "$DUT_VID" ] || [ -n "$DUT_PID" ]; then
  dut_want=$(printf '%04x:%04x' "$((DUT_VID))" "$((DUT_PID))" 2>/dev/null)
  [ "$dut_have" = "$dut_want" ] || {
    echo "refusing DUT_SYSFS=$DUT_SYSFS - it reports '$dut_have', not DUT_VID:DUT_PID $dut_want"; exit 2; }
fi
case "$dut_have" in
  0e8d:7612) DUT_KIND=mt7612u ;;
  0bda:c812|0bda:b812) DUT_KIND=realtek ;;
  *:*)
    if [ -n "$DUT_VID" ] && [ "$dut_have" != ":" ]; then DUT_KIND=realtek
    else echo "refusing DUT_SYSFS=$DUT_SYSFS ('$dut_have') - not an MT7612U, RTL8812CU or RTL8812BU (set DUT_VID / DUT_PID to name another die)"; exit 2
    fi ;;
esac
DUT_VID="0x${dut_have%%:*}"
DUT_PID="0x${dut_have#*:}"

# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init sta hostapd probe inject inject_own

# --- the AP adapter: refused unless it is plainly a spare wireless adapter --
ap_refuse() { echo "refusing AP_SYSFS=$AP_SYSFS: $*"; sta_lock_release; exit 2; }
[ -n "$(cat "/sys/bus/usb/devices/$AP_SYSFS/idVendor" 2>/dev/null)" ] ||
  ap_refuse "not a USB device - if its driver just loaded it may have moved; re-read lsusb -t"
[ "$(cat "/sys/bus/usb/devices/$AP_SYSFS/bDeviceClass" 2>/dev/null)" != "09" ] || ap_refuse "a hub"
AP_IF=$(sta_first_netdev "$AP_SYSFS")
[ -n "$AP_IF" ] || ap_refuse "no network interface on it"
[ -e "/sys/class/net/$AP_IF/phy80211" ] || ap_refuse "$AP_IF is not wireless"
for fam in -4 -6; do
  ip "$fam" route show default 2>/dev/null | grep -qw "dev $AP_IF" &&
    ap_refuse "$AP_IF carries a default route"
done
AP_PHY=$(basename "$(readlink -f "/sys/class/net/$AP_IF/phy80211")")
iw phy "$AP_PHY" info 2>/dev/null | grep -q '\* AP$' || ap_refuse "$AP_IF ($AP_PHY) does not support AP mode"
iw phy "$AP_PHY" info 2>/dev/null | grep -q set_wiphy_netns ||
  ap_refuse "the AP phy cannot change network namespace (in-kernel cfg80211 driver needed, e.g. rtw88/mt76; out-of-tree rtl88x2cu/88x2bu cannot)"
# Restored after the phy comes back: the move takes the interface down.
AP_WAS_UP=no
ip link show "$AP_IF" 2>/dev/null | grep -q '[<,]UP[,>]' && AP_WAS_UP=yes

# Flags and traps BEFORE anything is taken, so every exit from here on hands
# back what was.
NM_AP=no; NS_OURS=no; CLEANED=no; STA_HUNG=no; STA_PID=""; CELL=""
cleanup() {
  [ "$CLEANED" = yes ] && return 0
  CLEANED=yes
  local sta_gone=0
  # INT, then KILL after its window (sta_stop) - never a bare blocking wait.
  [ -n "$STA_PID" ] && sta_stop
  [ "$STA_HUNG" = yes ] && sta_gone=1
  sta_pid_kill probe
  sta_pid_kill inject
  sta_pid_kill inject_own
  sta_pid_kill hostapd
  ns_exists && ip netns exec "$NS" iw dev "$MON" del 2>/dev/null
  # THE PHY COMES BACK BEFORE THE NAMESPACE GOES: `ip netns del` on a
  # namespace still holding a phy destroys the phy (only a re-enumeration
  # brings it back). So the delete is conditional on the move having worked.
  if [ "$NS_OURS" = yes ] && ns_exists; then
    ip netns exec "$NS" iw phy "$AP_PHY" set netns 1 2>/dev/null
    sleep 1
    if ip netns exec "$NS" ls /sys/class/ieee80211/ 2>/dev/null | grep -q .; then
      echo "WARNING: a phy is still in netns $NS - NOT deleting it. Recover with:"
      echo "  sudo ip netns exec $NS iw phy $AP_PHY set netns 1; sudo ip netns del $NS"
    else
      ip netns del "$NS" 2>/dev/null
      [ "$AP_WAS_UP" = yes ] && ip link set "$AP_IF" up 2>/dev/null
    fi
  fi
  [ "$NM_AP" = yes ] && nmcli device set "$AP_IF" managed yes >/dev/null 2>&1
  # The DUT is re-enumerated only once sta_client has really exited: a
  # re-enumeration inside its teardown is what the hand-back must not do.
  if [ "$sta_gone" = 0 ] && [ "$STA_HUNG" = no ]; then
    if [ "$DUT_KIND" = mt7612u ]; then sta_dut_handback
    else sta_dev_handback dut "$DUT_SYSFS"; fi
  else
    echo "sta_client still running - not re-enumerating $DUT_SYSFS"
  fi
  sta_lock_release
}
trap cleanup EXIT
trap 'cleanup; exit 3' INT TERM

if [ "$DUT_KIND" = mt7612u ]; then
  sta_dut_take || exit 2
else
  # Marked opened BEFORE the unbind, so the hand-back re-binds its driver
  # whatever happens after this point.
  sta_dev_record dut "$DUT_SYSFS" "$DUT_VID" "$DUT_PID" || exit 2
  sta_dev_opened dut
  sta_dev_unbind_wifi "$DUT_SYSFS" || exit 2
fi

if command -v nmcli >/dev/null 2>&1; then
  case "$(nmcli -t -f DEVICE,STATE device 2>/dev/null | grep "^$AP_IF:")" in
    "$AP_IF:unmanaged"|"") : ;;
    *) nmcli device set "$AP_IF" managed no >/dev/null 2>&1 && NM_AP=yes ;;
  esac
fi
rfkill unblock wlan 2>/dev/null
NS_OURS=yes
ip netns add "$NS" || { echo "could not create netns $NS"; exit 2; }
iw phy "$AP_PHY" set netns name "$NS" || { echo "could not move $AP_PHY into $NS"; exit 2; }
sleep 2
ip netns exec "$NS" ip link set "$AP_IF" up 2>/dev/null

echo "DUT  $DUT_KIND $dut_have at $DUT_SYSFS ($(sta_usb_id "$DUT_SYSFS"))"
echo "AP   $AP_IF ($AP_PHY) at $AP_SYSFS, in netns $NS"
echo "ch$CH  ssid '$SSID'  tap $TAP  cells: $CELLS"
echo "logs: $OUT"

pass=0; fail=0; inconclusive=0
ok()   { pass=$((pass+1)); printf '  PASS  %s\n' "$*"; }
bad()  { fail=$((fail+1)); printf '  FAIL  %s\n' "$*"; }
inc()  { inconclusive=$((inconclusive+1)); printf '  INCONCLUSIVE  %s\n' "$*"; }
info() { printf '  INFO  %s\n' "$*"; }

# Is PID running? kill -0 also succeeds on an unreaped zombie.
proc_running() {
  local st
  st=$(sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -d' ' -f1)
  [ -n "$st" ] && [ "$st" != Z ] && [ "$st" != X ]
}

# Wait for an extended regex in a file. 0 found, 1 timed out.
wait_for() { # $1 file, $2 regex, $3 seconds
  local t=0
  until grep -qE "$2" "$1" 2>/dev/null; do
    [ "$t" -ge "$3" ] && return 1
    sleep 1; t=$((t + 1))
  done
}

# --- the AP -----------------------------------------------------------------
# hostapd runs in the foreground (backgrounded here) with its event stream on
# stdout: AP-STA-CONNECTED, EAPOL-4WAY-HS-COMPLETED and the rekey lines are
# read from that file. AP up is judged by the interface type, not the log.
ap_up() { # $1 open | wpa2 | wpa2norekey, $2 log tag
  {
    printf 'interface=%s\ndriver=nl80211\nssid=%s\n' "$AP_IF" "$SSID"
    if [ "$CH" -le 14 ]; then printf 'hw_mode=g\n'; else printf 'hw_mode=a\n'; fi
    printf 'channel=%s\nieee80211n=1\nauth_algs=1\nwmm_enabled=1\n' "$CH"
    if [ "$1" != open ]; then
      printf 'wpa=2\nwpa_passphrase=%s\nwpa_key_mgmt=WPA-PSK\nrsn_pairwise=CCMP\n' "$PSK"
    fi
    if [ "$1" = wpa2 ]; then
      printf 'wpa_group_rekey=%s\nwpa_ptk_rekey=%s\n' "$REKEY_S" "$PTK_REKEY_S"
    fi
    if [ "$AP_OFDM_ONLY" = 1 ] && [ "$CH" -le 14 ]; then
      printf 'supported_rates=60 90 120 180 240 360 480 540\nbasic_rates=60 120 240\n'
    fi
  } > "$OUT/hostapd_$2.conf"
  # The previous cell's hostapd exiting is not its interface being back: a
  # launch 30 ms after AP-DISABLED found the netdev gone ("Could not read
  # interface <if> flags: No such device" / "nl80211 driver initialization
  # failed"). Wait, bounded, until the netdev is present, and FORCE it to a
  # station once it is: a driver may leave the vif in AP type after hostapd
  # exits, and hostapd then fails with "Match already configured" rather
  # than anything that names the problem (tests/mt7612u_sta_identity.sh).
  local t=0 info
  while :; do
    info=$(ip netns exec "$NS" iw dev "$AP_IF" info 2>/dev/null)
    case "$info" in
      *'type managed'*) break ;;
      '') ;;   # not back yet
      *) ip netns exec "$NS" ip link set "$AP_IF" down 2>/dev/null
         ip netns exec "$NS" iw dev "$AP_IF" set type managed 2>/dev/null ;;
    esac
    if [ "$t" -ge 100 ]; then
      echo "rig: $AP_IF not back as a managed netdev in netns $NS within 10 s" \
           "- hostapd not started" | tee "$OUT/hostapd_$2.log"
      # The loop may just have taken it down; leave it up (best effort).
      ip netns exec "$NS" ip link set "$AP_IF" up 2>/dev/null
      return 1
    fi
    sleep 0.1; t=$((t + 1))
  done
  ip netns exec "$NS" ip link set "$AP_IF" up 2>/dev/null
  AP_START_MS=$(date +%s%3N)   # reconnect measures its re-join from here
  local dbg=""
  [ "$HOSTAPD_DEBUG" = 1 ] && dbg=-dd
  # shellcheck disable=SC2086  # $dbg is empty or one flag
  ip netns exec "$NS" hostapd $dbg -t "$OUT/hostapd_$2.conf" > "$OUT/hostapd_$2.log" 2>&1 &
  sta_pid_record hostapd $!
  t=0
  until ip netns exec "$NS" iw dev "$AP_IF" info 2>/dev/null | grep -q 'type AP'; do
    [ "$t" -ge 15 ] && return 1
    sleep 1; t=$((t + 1))
  done
  ip netns exec "$NS" ip addr flush dev "$AP_IF" 2>/dev/null
  ip netns exec "$NS" ip addr add "$APIP/24" dev "$AP_IF"
}

# --- the station --------------------------------------------------------------
# 0 up; 1 exited before `sta_client up:`; 2 still not up after READY_TIMEOUT.
sta_up() { # $1 cell, $2 seconds, $3.. extra env
  local cell="$1" secs="$2"; shift 2
  env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
      DEVOURER_USB_BUS="${DUT_SYSFS%%-*}" DEVOURER_USB_PORT="${DUT_SYSFS#*-}" \
      DEVOURER_MT7612U_FW_DIR="$FW_DIR" DEVOURER_LOG_LEVEL=info \
      DEVOURER_CHANNEL="$CH" DEVOURER_STA_SSID="$SSID" DEVOURER_STA_TAP="$TAP" \
      "$@" "$BUILD/sta_client" "$secs" > "$OUT/sta_$cell.log" 2>&1 &
  STA_PID=$!
  sta_pid_record sta "$STA_PID"
  local t=0
  until grep -q 'sta_client up:' "$OUT/sta_$cell.log" 2>/dev/null; do
    proc_running "$STA_PID" || return 1
    [ "$t" -ge "$READY_TIMEOUT" ] && return 2
    sleep 1; t=$((t + 1))
  done
}

# The station never printed `sta_client up:` - a rig / bring-up problem, not
# a verdict on the station, whatever the exit status. $1 cell, $2 sta_up's rc.
station_not_up() {
  if [ "$2" = 2 ]; then
    inc "$1: sta_client not up within READY_TIMEOUT=${READY_TIMEOUT}s (rig/bring-up) - see $OUT/sta_$1.log"
    return
  fi
  sta_stop
  if [ "$STA_RC" = 2 ]; then
    inc "$1: sta_client REFUSED the adapter (station_mode_ok false): $(grep -m1 REFUSED "$OUT/sta_$1.log")"
  else
    inc "$1: sta_client exited before 'up' (status $STA_RC; rig/bring-up): $(tail -1 "$OUT/sta_$1.log" 2>/dev/null)"
  fi
}

# Stop the station (INT: it leaves the BSS, clears the identity and prints its
# ledger) and record its exit status in STA_RC.
STA_RC=""
sta_stop() {
  STA_RC=""
  [ -n "$STA_PID" ] || return 0
  if proc_running "$STA_PID"; then kill -INT "$STA_PID" 2>/dev/null; fi
  local t=0
  while proc_running "$STA_PID" && [ "$t" -lt 15 ]; do sleep 1; t=$((t + 1)); done
  if proc_running "$STA_PID"; then
    # KILL, not TERM: TERM is handled exactly like INT. Still alive after it
    # means the DUT must not be re-enumerated under it.
    sta_pid_kill sta KILL || STA_HUNG=yes
    STA_RC=killed
  else
    wait "$STA_PID" 2>/dev/null; STA_RC=$?
    rm -f "$OUT/.pid_sta"
  fi
  STA_PID=""
  # Exit 3 is a fault the station caught and tore down cleanly: a FAIL
  # wherever it happens, with the cause named.
  if [ "$STA_RC" = 3 ] && [ -n "$CELL" ]; then
    bad "$CELL: sta_client FAULT (exit 3): $(fault_cause "$CELL")"
  fi
}

# The TAP up, and the route to the AP proven to leave through it.
tap_up() {
  local t=0
  until [ -d "/sys/class/net/$TAP" ]; do
    [ "$t" -ge 20 ] && return 1
    sleep 1; t=$((t + 1))
  done
  command -v nmcli >/dev/null 2>&1 && nmcli device set "$TAP" managed no >/dev/null 2>&1
  ip link set "$TAP" up 2>/dev/null
  ip addr flush dev "$TAP" 2>/dev/null
  ip addr add "$STAIP/24" dev "$TAP" 2>/dev/null
  sleep 1
  case "$(ip route get "$APIP" 2>/dev/null)" in *"dev $TAP"*) return 0 ;; esac
  return 1
}

own_of() { sed -n 's/^sta_client up: own \([0-9a-f:]\{17\}\) .*/\1/p' "$OUT/sta_$1.log" | head -1; }
# One numeric field from the station's exit ledger.
led() { sed -n "s/.*$2=\\([0-9][0-9]*\\).*/\\1/p" "$OUT/sta_$1.log" | tail -1; }

# Ping the AP over the air. 0 = 0% loss, 1 = loss, 2 = the station was not
# alive for the whole measurement (no verdict on the link).
ping_ap() { # $1 tag
  proc_running "$STA_PID" || return 2
  ping -c 1 -W 3 -I "$TAP" "$APIP" >/dev/null 2>&1            # warm ARP
  ping -c 6 -W 1 -I "$TAP" "$APIP" > "$OUT/ping_$1.txt" 2>&1
  proc_running "$STA_PID" || return 2
  grep -q ' 0% packet loss' "$OUT/ping_$1.txt"
}
# The same over a real window: PING_S seconds, two pings a second.
ping_window() { # $1 tag
  proc_running "$STA_PID" || return 2
  ping -c 1 -W 3 -I "$TAP" "$APIP" >/dev/null 2>&1            # warm ARP
  ping -c $(( PING_S * 2 )) -i 0.5 -W 1 -I "$TAP" "$APIP" > "$OUT/ping_$1.txt" 2>&1
  proc_running "$STA_PID" || return 2
  grep -q ' 0% packet loss' "$OUT/ping_$1.txt"
}
loss() { grep -oE '[0-9]+ packets transmitted, [0-9]+ received.*packet loss' "$OUT/ping_$1.txt" 2>/dev/null | head -1; }

# The station exited after `sta_client up:` but before its measurement:
# status 0 ran out of SECS (INCONCLUSIVE); anything else is a FAIL.
station_gone() { # $1 cell
  sta_stop
  case "$STA_RC" in
    0) inc "$1: sta_client ran out of time before the measurement - raise SECS (now $SECS)" ;;
    3) ;;   # a FAULT: reported by sta_stop
    *) bad "$1: sta_client exited early (status $STA_RC) - see $OUT/sta_$1.log" ;;
  esac
}

# The cause of a sta_client FAULT (exit 3, `fault=1` in the ledger).
fault_cause() {
  grep -m1 'FAULT\|threw' "$OUT/sta_$1.log" 2>/dev/null | sed 's/^ *//'
}

# The clear ran on exit and verified: on a Realtek die it restores the port
# registers, on MT7612U the pre-arm (monitor) receive filter, and either reads
# back. (An unverified clear is also a station FAULT, exit 3, scored by
# sta_stop.)
check_cleared() { # $1 cell
  local line
  line=$(grep -m1 'station identity clear:' "$OUT/sta_$1.log" | sed 's/^ *//')
  case "$line" in
    *"restored (verified)"*) ok "$1: ClearStationIdentity ran and verified on exit" ;;
    '') bad "$1: ClearStationIdentity did not run on exit" ;;
    *) bad "$1: ClearStationIdentity did not verify: $line" ;;
  esac
}

# The managed-filter stimulus (header): the two streams off a monitor vif on
# the AP's own phy, while the station is associated. They share a transmitter
# address, so they get disjoint sequence ranges (0 and 2048): the managed
# filter's hardware DUP drop must not take one for a retransmission of the
# other. The injector counts frames it SUBMITTED, not frames that aired -
# hence the own stream as the positive witness. Each PID is recorded on the
# statement after its launch, and every injector is bounded by `timeout -k`
# whatever happens to the harness. Sets INJ_FOREIGN / INJ_OWN (empty when it
# could not run).
INJ_FOREIGN=""; INJ_OWN=""
inject_count() { sed -n 's/^injected \([0-9][0-9]*\) unicast frames.*/\1/p' "$1" 2>/dev/null | tail -1; }
inject_unicast() { # $1 cell
  INJ_FOREIGN=""; INJ_OWN=""
  local bssid own pf po
  bssid=$(ip netns exec "$NS" cat "/sys/class/net/$AP_IF/address" 2>/dev/null)
  own=$(own_of "$1")
  [ -n "$bssid" ] && [ -n "$own" ] || return 1
  ip netns exec "$NS" iw dev "$MON" del 2>/dev/null
  if ! { ip netns exec "$NS" iw phy "$AP_PHY" interface add "$MON" type monitor 2>/dev/null &&
         ip netns exec "$NS" ip link set "$MON" up 2>/dev/null; }; then
    ip netns exec "$NS" iw dev "$MON" del 2>/dev/null
    return 1
  fi
  ip netns exec "$NS" timeout -k 5 $(( INJECT_S + 10 )) \
    python3 "$ROOT/tests/sta_unicast_inject.py" "$MON" "$FOREIGN" "$bssid" \
      "$INJECT_S" "$INJECT_PPS" > "$OUT/inject_$1.log" 2>&1 &
  pf=$!; sta_pid_record inject "$pf"
  ip netns exec "$NS" timeout -k 5 $(( INJECT_S + 10 )) \
    python3 "$ROOT/tests/sta_unicast_inject.py" "$MON" "$own" "$bssid" \
      "$INJECT_S" "$INJECT_PPS" 2048 > "$OUT/inject_own_$1.log" 2>&1 &
  po=$!; sta_pid_record inject_own "$po"
  wait "$pf" 2>/dev/null; wait "$po" 2>/dev/null
  rm -f "$OUT/.pid_inject" "$OUT/.pid_inject_own"
  ip netns exec "$NS" iw dev "$MON" del 2>/dev/null
  INJ_FOREIGN=$(inject_count "$OUT/inject_$1.log")
  INJ_OWN=$(inject_count "$OUT/inject_own_$1.log")
  [ -n "$INJ_FOREIGN" ] && [ -n "$INJ_OWN" ]
}

# Score the stimulus against the station's ledger. $1 cell, $2 managed |
# monitor (what the filter should be). Both need the OWN stream to have
# arrived (>= half) before the FOREIGN count means anything. The own stream
# shows the injection path airs, not that the FOREIGN injector did: an armed
# PASS ("almost none arrived") is therefore held until a control in the same
# run has seen the foreign stream arrive (noarm, FOREIGN_SEEN), and scored
# after the last cell (score_held_filter) - INCONCLUSIVE without one. A FAIL
# needs no such witness: the frames arrived.
FOREIGN_SEEN=no
HELD_FILTER_PASS=""
check_filter() {
  local nfu ref
  nfu=$(led "$1" 'not-for-us'); ref=$(led "$1" 'plaintext refused')
  if [ "${INJ_FOREIGN:-0}" = 0 ] || [ "${INJ_OWN:-0}" = 0 ]; then
    inc "$1: the unicast injectors did not run (no monitor vif on $AP_PHY?) - see $OUT/inject_$1.log, $OUT/inject_own_$1.log"
    return
  fi
  if [ -z "$nfu" ] || [ -z "$ref" ]; then
    inc "$1: no not-for-us / plaintext refused count in the ledger - see $OUT/sta_$1.log"
    return
  fi
  if [ $(( ref * 2 )) -lt "$INJ_OWN" ]; then
    inc "$1: only $ref of $INJ_OWN frames injected at the station's own address arrived - the injection is not reaching the DUT, so the filter check is not evidence"
    return
  fi
  if [ "$2" = managed ]; then
    if [ $(( nfu * 100 )) -lt "$INJ_FOREIGN" ]; then
      HELD_FILTER_PASS="$1: managed filter on: own-addressed $ref of $INJ_OWN arrived, foreign not-for-us=$nfu of $INJ_FOREIGN"
      info "$1: managed-filter result held until the noarm control has seen the foreign stream"
    else
      bad "$1: armed station still receives others' unicast: not-for-us=$nfu of $INJ_FOREIGN (own-addressed $ref of $INJ_OWN arrived)"
    fi
  elif [ $(( nfu * 2 )) -ge "$INJ_FOREIGN" ]; then
    FOREIGN_SEEN=yes
    ok "$1: control: unarmed, both streams arrive: own-addressed $ref of $INJ_OWN, foreign not-for-us=$nfu of $INJ_FOREIGN"
  else
    bad "$1: unarmed (monitor filter) yet the foreign stream did not arrive: not-for-us=$nfu of $INJ_FOREIGN while own-addressed $ref of $INJ_OWN did"
  fi
}

# Arm and clear lines: the identity was armed for the AP's BSSID, and the
# clear ran on the way out.
check_armed() { # $1 cell
  if grep -q '^  station identity armed for BSSID' "$OUT/sta_$1.log"; then
    ok "$1: SetStationIdentity armed ($(grep -m1 '^  station identity armed' "$OUT/sta_$1.log" | sed 's/^ *//'))"
  else
    bad "$1: the identity was never armed ($(grep -m1 'station identity' "$OUT/sta_$1.log" || echo 'no arm line'))"
  fi
  check_cleared "$1"
}

# A cell that FAILED keeps the kernel's view (the AP driver may log a
# station-add or TX-status error): the tail of dmesg into its own file, and
# the AP-relevant lines echoed. Read-only; skipped silently if unreadable.
CELL_FAIL0=0
dmesg_on_fail() {
  [ "$fail" -gt "$CELL_FAIL0" ] || return 0
  dmesg 2>/dev/null | tail -80 > "$OUT/dmesg_${CELL:-cell}.txt" || return 0
  grep -iE 'mt76|rtw|rtl8|cfg80211|ieee80211' "$OUT/dmesg_${CELL:-cell}.txt" |
    tail -10 | sed 's/^/  dmesg  /'
  return 0
}
cell_end() { sta_pid_kill probe; sta_stop; sta_pid_kill hostapd; dmesg_on_fail; }

# --- open ---------------------------------------------------------------------
cell_open() {
  CELL=open
  echo; echo "== open: hostapd open network =="
  ap_up open open || { inc "open: hostapd did not bring $AP_IF up in AP mode - see $OUT/hostapd_open.log"; cell_end; return; }
  local up=0
  sta_up open "$SECS" DEVOURER_STA_PSK= || up=$?
  [ "$up" = 0 ] || { station_not_up open "$up"; cell_end; return; }
  local own; own=$(own_of open)
  tap_up || { inc "open: no TAP, or the route to $APIP does not leave through $TAP"; cell_end; return; }
  # Traffic from the start, as a user's host would send: an open association
  # the AP does not hold is only visible to the station as questions (ARP)
  # that get no reply, and it re-joins on that (kConfirmMs in sta_client.cpp).
  # The AP's own record - AP-STA-CONNECTED for our address - is the witness,
  # and the 30 s bound covers a first association that has to be recovered.
  ping -I "$TAP" -i 1 "$APIP" >/dev/null 2>&1 &
  sta_pid_record probe $!
  if ! wait_for "$OUT/hostapd_open.log" "AP-STA-CONNECTED $own" 30; then
    if proc_running "$STA_PID"; then bad "open: the AP never associated $own within 30 s"; cell_end
    else sta_pid_kill probe; station_gone open; sta_pid_kill hostapd; fi
    return
  fi
  sta_pid_kill probe
  ok "open: the AP associated $own"
  ping_ap open; case $? in
    0) ok "open: ping over the air, $(loss open)" ;;
    1) bad "open: ping $(loss open)" ;;
    *) station_gone open; sta_pid_kill hostapd; return ;;
  esac
  cell_end
  local unconf assoc
  unconf=$(led open 'unconfirmed'); assoc=$(led open 'associations')
  if [ "${unconf:-0}" -gt 0 ] 2>/dev/null; then
    info "open: recovered: ${unconf} association(s) the AP did not hold were found unconfirmed and re-joined (${assoc:-?} associations, assoc_repeat=$(led open 'assoc_repeat'))"
  fi
  local plain enc
  plain=$(led open 'plaintext rx'); enc=$(led open 'encrypted rx')
  if [ "${plain:-0}" -gt 0 ] && [ "${enc:-x}" = 0 ]; then
    ok "open: ledger plaintext rx=$plain, encrypted rx=0"
  else
    bad "open: ledger plaintext rx=${plain:-?} encrypted rx=${enc:-?} (expected >0 and 0)"
  fi
  check_armed open
}

# --- wpa2 and its two variants --------------------------------------------------
# $1 cell (wpa2 | noarm | retry0), $2.. extra station env. Returns after the
# station has stopped; the caller scores the arm-specific lines. WPA2_LINK is
# "no four-way" or "four-way completed, ping ...", ending in OK on 0% loss.
WPA2_LINK=""
# Set once the ARMED wpa2 cell has completed a four-way in this run: the
# positive control the Realtek noarm control needs.
ARMED_FOURWAY=no
run_wpa2() {
  local cell="$1"; shift
  CELL="$cell"
  WPA2_LINK=""
  ap_up wpa2 "$cell" || { inc "$cell: hostapd did not bring $AP_IF up in AP mode - see $OUT/hostapd_$cell.log"; cell_end; return 1; }
  local secs=$(( SECS + 2 * REKEY_S + PTK_REKEY_S ))
  local up=0
  sta_up "$cell" "$secs" DEVOURER_STA_PSK="$PSK" "$@" || up=$?
  [ "$up" = 0 ] || { station_not_up "$cell" "$up"; cell_end; return 1; }
  local own; own=$(own_of "$cell")
  tap_up || { inc "$cell: no TAP, or the route to $APIP does not leave through $TAP"; cell_end; return 1; }
  if ! wait_for "$OUT/hostapd_$cell.log" "EAPOL-4WAY-HS-COMPLETED $own" 30; then
    WPA2_LINK="no four-way"
    if ! proc_running "$STA_PID"; then station_gone "$cell"; sta_pid_kill hostapd; return 1; fi
    cell_end
    return 0
  fi
  local p=0
  if [ "$cell" = wpa2 ]; then ping_ap "$cell" || p=$?
  else ping_window "$cell" || p=$?; fi
  if [ "$p" = 2 ]; then station_gone "$cell"; sta_pid_kill hostapd; return 1; fi
  WPA2_LINK="four-way completed, ping $(loss "$cell")"
  [ "$p" = 0 ] && WPA2_LINK="$WPA2_LINK OK"
  INJ_FOREIGN=""; INJ_OWN=""
  if [ "$DUT_KIND" = mt7612u ] && { [ "$cell" = wpa2 ] || [ "$cell" = noarm ]; }; then
    inject_unicast "$cell"
    proc_running "$STA_PID" || { station_gone "$cell"; sta_pid_kill hostapd; return 1; }
  fi
  [ "$cell" = wpa2 ] || { cell_end; return 0; }

  # The rekeys: waited for while the station is alive. "pairwise key
  # handshake completed" is logged for the initial four-way too, so a PTK
  # rekey is the SECOND such line.
  local t=0 gk=0 pk=0 lim=$(( REKEY_S + PTK_REKEY_S + 25 ))
  while [ "$t" -lt "$lim" ] && proc_running "$STA_PID"; do
    gk=$(grep -c 'group key handshake completed' "$OUT/hostapd_$cell.log" 2>/dev/null)
    pk=$(grep -c 'pairwise key handshake completed' "$OUT/hostapd_$cell.log" 2>/dev/null)
    [ "${gk:-0}" -ge 1 ] && [ "${pk:-0}" -ge 2 ] && break
    sleep 1; t=$((t + 1))
  done
  if ! proc_running "$STA_PID" && { [ "${gk:-0}" -lt 1 ] || [ "${pk:-0}" -lt 2 ]; }; then
    station_gone "$cell"; sta_pid_kill hostapd; return 1
  fi
  if [ "${gk:-0}" -ge 1 ]; then ok "$cell: the AP completed a group rekey"
  else bad "$cell: no group rekey completed in ${lim}s"; fi
  if [ "${pk:-0}" -ge 2 ]; then ok "$cell: the AP completed a pairwise rekey ($pk pairwise handshakes)"
  else bad "$cell: no pairwise rekey in ${lim}s (${pk:-0} pairwise handshake(s))"; fi
  # Still carrying traffic after both rekeys.
  ping_ap "${cell}_after"; case $? in
    0) ok "$cell: ping after the rekeys, $(loss "${cell}_after")" ;;
    1) bad "$cell: ping after the rekeys $(loss "${cell}_after")" ;;
    *) station_gone "$cell"; sta_pid_kill hostapd; return 1 ;;
  esac
  cell_end
  return 0
}

cell_wpa2() {
  echo; echo "== wpa2: hostapd WPA2-PSK, group rekey ${REKEY_S}s, pairwise rekey ${PTK_REKEY_S}s =="
  run_wpa2 wpa2 || return
  case "$WPA2_LINK" in
    *OK) ok "wpa2: $WPA2_LINK" ;;
    *) bad "wpa2: ${WPA2_LINK:-no result} - see $OUT/sta_wpa2.log and $OUT/hostapd_wpa2.log" ;;
  esac
  [ "$WPA2_LINK" = "no four-way" ] && { check_armed wpa2; return; }
  ARMED_FOURWAY=yes
  # Two different MIC counters: the four-way's (mic_failures=, the
  # supplicant's EAPOL-Key MIC check) must be 0; the data plane's (MIC
  # failures=, CCMP on received data) may reach one per pairwise rekey.
  local assoc mic fwmic ptk ans
  assoc=$(led wpa2 'associations'); mic=$(led wpa2 'MIC failures')
  fwmic=$(led wpa2 'mic_failures')
  ptk=$(led wpa2 'PTK'); ans=$(led wpa2 'answered')
  if [ "${assoc:-0}" = 1 ] && [ "${ans:-0}" -gt 0 ] && [ "${ptk:-0}" -ge 2 ] &&
     [ "${fwmic:-1}" = 0 ] && [ "${mic:-999}" -le "${ptk:-0}" ]; then
    ok "wpa2: ledger associations=1, rekeys answered=$ans, PTK installs=$ptk, four-way MIC failures=0, data-plane MIC failures=$mic (<= PTK installs)"
  else
    bad "wpa2: ledger associations=${assoc:-?} answered=${ans:-?} PTK=${ptk:-?} four-way MIC failures=${fwmic:-?} data-plane MIC failures=${mic:-?} (expected 1, >0, >=2, 0, <= PTK)"
  fi
  check_armed wpa2
  if [ "$DUT_KIND" = mt7612u ]; then check_filter wpa2 managed
  else info "wpa2: the managed-filter check is MT7612U-only (skipped on $DUT_KIND)"; fi
  # The station default retry limit is nonzero, so the arm must NOT warn.
  if grep -q 'station identity armed with tx.retry_limit=0' "$OUT/sta_wpa2.log"; then
    bad "wpa2: the tx.retry_limit=0 warning fired with the station default limit"
  else
    ok "wpa2: no tx.retry_limit=0 warning ($(grep -m1 'tx.retry_limit' "$OUT/sta_wpa2.log" | sed 's/^ *//'))"
  fi
}

cell_noarm() {
  echo; echo "== noarm (control): wpa2 with DEVOURER_STA_ARM=0 =="
  run_wpa2 noarm DEVOURER_STA_ARM=0 || return
  if grep -q 'sta_client up:.* arm=0' "$OUT/sta_noarm.log" &&
     ! grep -q 'station identity' "$OUT/sta_noarm.log"; then
    ok "noarm: no SetStationIdentity and no ClearStationIdentity ran"
  else
    bad "noarm: an arm or clear ran with DEVOURER_STA_ARM=0 ($(grep -m1 'station identity' "$OUT/sta_noarm.log"))"
  fi
  if [ "$DUT_KIND" != realtek ]; then
    [ "$WPA2_LINK" = "no four-way" ] || check_filter noarm monitor
    info "noarm: link unarmed: ${WPA2_LINK:-no result} (unarmed = the monitor filter; a link difference from wpa2 here is worth a look)"
    return
  fi
  # Realtek: unarmed, the MAC does not ACK own-addressed unicast, so hostapd
  # never sees its authentication response acknowledged and never lets the
  # station in. Meaningful only if the station tried.
  local beacons auth noack
  beacons=$(led noarm 'beacons observed'); auth=$(led noarm 'auth_tx')
  noack=$(grep -c 'did not acknowledge' "$OUT/hostapd_noarm.log" 2>/dev/null)
  if [ "$ARMED_FOURWAY" != yes ]; then
    inc "noarm: no ARMED four-way against this hostapd configuration in this run (run the wpa2 cell first, and it must get in) - a silent AP proves nothing"
  elif [ "${beacons:-0}" = 0 ] || [ "${auth:-0}" = 0 ]; then
    inc "noarm: the unarmed station never tried (beacons observed=${beacons:-?}, auth_tx=${auth:-?}) - not a control"
  elif [ "$WPA2_LINK" = "no four-way" ]; then
    ok "noarm: unarmed, the AP never completed the four-way (auth_tx=$auth) - the arm is what makes the wpa2 link"
  else
    bad "noarm: the UNARMED station got in (${WPA2_LINK}) - the arm is not what makes the wpa2 link, or this die answers unarmed"
  fi
  info "noarm: hostapd 'did not acknowledge' lines: ${noack:-0}"
}

cell_retry0() {
  echo; echo "== retry0: wpa2 with DEVOURER_TX_RETRY_LIMIT=0 =="
  run_wpa2 retry0 DEVOURER_TX_RETRY_LIMIT=0 || return
  local armed warn
  armed=$(grep -n -m1 '^  station identity armed for BSSID' "$OUT/sta_retry0.log" | cut -d: -f1)
  warn=$(grep -n -m1 'station identity armed with tx.retry_limit=0' "$OUT/sta_retry0.log" | cut -d: -f1)
  if [ -z "$armed" ]; then
    inc "retry0: the identity was never armed, so the arm-time warning could not fire - see $OUT/sta_retry0.log"
  elif [ -n "$warn" ] && [ "$warn" -lt "$armed" ]; then
    ok "retry0: the library warned at arm time: $(sed -n "${warn}p" "$OUT/sta_retry0.log" | cut -c1-100)..."
  else
    bad "retry0: armed with tx.retry_limit=0 and no arm-time warning"
  fi
  check_cleared retry0
  info "retry0: single-shot uplink: ${WPA2_LINK:-no result}"
}

# --- reconnect and its no-re-join variant ---------------------------------------
# $1 cell, $2 DEVOURER_STA_RECONNECT (1 | 0).
run_reconnect() {
  local cell="$1" rc="$2"
  CELL="$cell"
  # No rekeys: this cell measures the re-join, and a rekey in the window
  # would be a second thing happening.
  ap_up wpa2norekey "$cell" || { inc "$cell: hostapd did not come up - see $OUT/hostapd_$cell.log"; cell_end; return; }
  local secs=$(( SECS + DOWN_S + REJOIN_S + PING_S + 30 ))
  local up=0
  sta_up "$cell" "$secs" DEVOURER_STA_PSK="$PSK" DEVOURER_STA_RECONNECT="$rc" || up=$?
  [ "$up" = 0 ] || { station_not_up "$cell" "$up"; cell_end; return; }
  local own; own=$(own_of "$cell")
  tap_up || { inc "$cell: no TAP, or the route to $APIP does not leave through $TAP"; cell_end; return; }
  if ! wait_for "$OUT/hostapd_$cell.log" "EAPOL-4WAY-HS-COMPLETED $own" 30; then
    if proc_running "$STA_PID"; then
      inc "$cell: the first association never completed - nothing to reconnect"; cell_end
    else station_gone "$cell"; sta_pid_kill hostapd; fi
    return
  fi
  ping_ap "$cell"; case $? in
    0) ;;
    1) inc "$cell: ping $(loss "$cell") before the loss - nothing to compare against"; cell_end; return ;;
    *) station_gone "$cell"; sta_pid_kill hostapd; return ;;
  esac

  # THE AP GOES AWAY (hostapd deauthenticates its stations on the way out,
  # and its beacons stop), then comes back on the same BSSID.
  echo "  stopping hostapd for ${DOWN_S}s"
  sta_pid_kill hostapd
  sleep "$DOWN_S"
  if ! grep -q '^  station link lost:' "$OUT/sta_$cell.log"; then
    proc_running "$STA_PID" || { station_gone "$cell"; return; }
  fi
  ap_up wpa2norekey "${cell}2" || { inc "$cell: hostapd did not come back - see $OUT/hostapd_${cell}2.log"; cell_end; return; }
  # The bound runs from hostapd being started again, not from ap_up
  # returning (which waits for the AP type first).
  local back=$AP_START_MS deadline=$(( AP_START_MS + REJOIN_S * 1000 )) rejoined=no
  while [ "$(date +%s%3N)" -lt "$deadline" ]; do
    if grep -q "EAPOL-4WAY-HS-COMPLETED $own" "$OUT/hostapd_${cell}2.log" 2>/dev/null; then
      rejoined=yes; break
    fi
    sleep 0.2
  done

  if [ "$rc" = 1 ]; then
    if [ "$rejoined" = yes ]; then
      local ms=$(( $(date +%s%3N) - back ))
      ok "$cell: re-joined and re-keyed $(( ms / 1000 )).$(( ms % 1000 / 100 ))s after hostapd was started again (bound ${REJOIN_S}s)"
    else
      if proc_running "$STA_PID"; then
        bad "$cell: no second four-way within ${REJOIN_S}s of the AP coming back"; cell_end
      else station_gone "$cell"; sta_pid_kill hostapd; fi
      return
    fi
    ping_window "${cell}_after"; case $? in
      0) ok "$cell: ping after the re-join, $(loss "${cell}_after")" ;;
      1) bad "$cell: ping after the re-join, $(loss "${cell}_after")" ;;
      *) station_gone "$cell"; sta_pid_kill hostapd; return ;;
    esac
  else
    if [ "$rejoined" = yes ]; then
      bad "$cell: re-joined with DEVOURER_STA_RECONNECT=0"
    else
      proc_running "$STA_PID" || { station_gone "$cell"; sta_pid_kill hostapd; return; }
      ok "$cell: no re-join within ${REJOIN_S}s with DEVOURER_STA_RECONNECT=0"
    fi
  fi
  cell_end

  if grep -q '^  station link lost:' "$OUT/sta_$cell.log"; then
    ok "$cell: the station reported the lost link ($(grep -m1 '^  station link lost:' "$OUT/sta_$cell.log" | sed 's/^ *station link lost: //'))"
  else
    bad "$cell: the station never reported losing the link"
  fi
  local assoc reconn
  assoc=$(led "$cell" 'associations'); reconn=$(led "$cell" 'reconnects')
  if [ "$rc" = 1 ]; then
    if [ "${assoc:-0}" = 2 ] && [ "${reconn:-0}" = 1 ]; then
      ok "$cell: ledger associations=2, reconnects=1"
    else
      bad "$cell: ledger associations=${assoc:-?}, reconnects=${reconn:-?} (expected 2 and 1)"
    fi
  else
    if [ "${assoc:-0}" = 1 ] && grep -q '^fault=0 state=Failed' "$OUT/sta_$cell.log"; then
      ok "$cell: ledger ends Failed after 1 association"
    else
      bad "$cell: ledger associations=${assoc:-?}, final state $(grep -m1 '^fault=' "$OUT/sta_$cell.log" | cut -d' ' -f2) (expected 1 and Failed)"
    fi
  fi
  # ONE arm for the run: the arm is per BSSID, and a re-join to the same
  # BSSID keeps it (nothing between the two associations touches it).
  local arms; arms=$(grep -c '^  station identity armed for BSSID' "$OUT/sta_$cell.log")
  if [ "${arms:-0}" = 1 ]; then
    ok "$cell: armed once for the BSSID, across the re-join"
  else
    bad "$cell: ${arms:-0} arm lines (expected exactly 1)"
  fi
  check_cleared "$cell"
}

cell_reconnect() {
  echo; echo "== reconnect: hostapd away for ${DOWN_S}s, re-join within ${REJOIN_S}s =="
  run_reconnect reconnect 1
}

cell_noreconnect() {
  echo; echo "== noreconnect: as reconnect, with DEVOURER_STA_RECONNECT=0 =="
  run_reconnect noreconnect 0
}

for c in $CELLS; do CELL_FAIL0=$fail; "cell_$c"; done
score_held_filter() {
  [ -n "$HELD_FILTER_PASS" ] || return 0
  echo; echo "== the held managed-filter result =="
  if [ "$FOREIGN_SEEN" = yes ]; then
    ok "$HELD_FILTER_PASS (the noarm control saw the foreign stream arrive)"
  else
    inc "${HELD_FILTER_PASS%%:*}: managed filter not scored - no control in this run saw the foreign stream arrive (run noarm with wpa2)"
  fi
}
score_held_filter

echo
echo "=== $pass passed, $fail failed, $inconclusive inconclusive  (logs: $OUT) ==="
[ "$fail" -gt 0 ] && exit 1
[ "$inconclusive" -gt 0 ] && exit 2
exit 0
