#!/usr/bin/env bash
# mt7612u_sta_onair.sh - tests/sta_client.cpp joining a real hostapd AP, end
# to end, with the station identity armed through IRadio::SetStationIdentity.
#
# The MT7612U (DUT_SYSFS) runs sta_client: scan, authenticate, associate, the
# WPA2-PSK four-way and CCMP over src/sta/, a TAP device for the host. A
# kernel-driven adapter (AP_SYSFS) runs hostapd - an independent
# implementation on independent silicon, which is what makes its log a
# witness: "EAPOL-4WAY-HS-COMPLETED" means the AUTHENTICATOR verified our
# message 4's MIC.
#
# THE AP LIVES IN A NETWORK NAMESPACE. Both radios are on one host; with both
# addresses in the root namespace the kernel routes the ping locally and it
# never touches the air. The phy moves with `iw phy <phy> set netns`, and
# every data-plane check first asserts that `ip route get` leaves through the
# station's TAP.
#
# Cells (each scored against its own witness):
#   open     hostapd open: the AP associates OUR address; ping 0% loss over
#            the TAP; the ledger shows plaintext and no decryption; the arm
#            line, and the clear ran on exit.
#   wpa2     hostapd WPA2-PSK with group and pairwise rekeys: four-way, both
#            rekeys completed at the AP, ping 0% loss, ONE association
#            throughout, MIC failures <= PTK installs (a pairwise rekey has a
#            one-frame switchover window, see rx_frame() in sta_client.cpp),
#            the arm line, the clear ran on exit, and NO tx.retry_limit=0
#            warning (the station default is nonzero).
#   noarm    the control: wpa2 with DEVOURER_STA_ARM=0 - nothing else
#            changes. Scored: no SetStationIdentity and no clear ran.
#            Reported, not scored: whether it associated and carried the
#            ping (the MT7612U arm writes no register - docs/station-client.md).
#   retry0   wpa2 with DEVOURER_TX_RETRY_LIMIT=0. Scored: the library's
#            arm-time warning about tx.retry_limit=0 (logged inside a
#            successful SetStationIdentity, so just before sta_client's
#            "armed" line), and the clear ran on exit.
#            Reported, not scored: the link outcome with a single-shot uplink.
#
# The clear is scored as having RUN, not by its result: on MT7612U
# ClearStationIdentity is trivially true (the arm wrote nothing), so the
# result is printed as information.
#
# Liveness: a data-plane check runs only while sta_client is alive, and again
# checks it afterwards - a ping that straddles the station's exit reports
# loss on a working link. A station that exits, or is not up within
# READY_TIMEOUT, before printing `sta_client up:` is a rig / bring-up problem
# (INCONCLUSIVE, whatever its status). After `up:`, an exit with status 0
# ran out of SECS (INCONCLUSIVE); 3 is a FAULT the station caught (an
# exception or a failed TAP; `fault=1` in its ledger) and is a FAIL with the
# cause named, wherever in the cell it happens; any other status is a FAIL.
#
# Exit status: 0 every scored check passed; 1 a check failed; 2 INCONCLUSIVE
# (the rig was refused, the station did not come up, the AP did not come up,
# the route did not leave through the TAP, or a cell was cut short);
# 3 interrupted.
#
#   sudo DUT_SYSFS=1-1 AP_SYSFS=5-1 tests/mt7612u_sta_onair.sh
#   sudo DUT_SYSFS=1-1 AP_SYSFS=5-1 CH=6 tests/mt7612u_sta_onair.sh wpa2 retry0
#
# Rig: DUT_SYSFS an MT7612U (0e8d:7612), unbound from mt76x2u here and
# re-enumerated at the end; AP_SYSFS an adapter whose kernel driver supports
# AP mode AND lets its phy change network namespace (`iw phy` lists
# set_wiphy_netns): an in-kernel cfg80211 driver such as rtw88 or mt76.
# Out-of-tree drivers such as rtl88x2cu / 88x2bu cannot, and are refused.
# Read AP_SYSFS from `lsusb -t` after its driver has loaded (it can move).
# FW_DIR must hold the DECOMPRESSED MT7612U blobs (mt7662*.bin); a host that
# ships only mt7662*.bin.zst gets INCONCLUSIVE (rig/bring-up).
# Build first: cmake --build build --target StaClientSelftest (build/sta_client).
#
# Env: DUT_SYSFS, AP_SYSFS, CH, SSID, PSK, SECS, REKEY_S, PTK_REKEY_S, FW_DIR,
#      NS, TAP, READY_TIMEOUT, OUT, BUILD.
# Cells: open | wpa2 | noarm | retry0 | all (default: all four).

# The cells are reached as "cell_$c" and cleanup through the traps.
# shellcheck disable=SC2317
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
DUT_SYSFS="${DUT_SYSFS:-}"
AP_SYSFS="${AP_SYSFS:-}"
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
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"
NS="${NS:-staonair}"
TAP="${TAP:-dvsta0}"
READY_TIMEOUT="${READY_TIMEOUT:-30}"
OUT="${OUT:-}"
APIP=192.168.98.1
STAIP=192.168.98.2

CELLS="${*:-all}"
[ "$CELLS" = all ] && CELLS="open wpa2 noarm retry0"
for c in $CELLS; do
  case "$c" in open|wpa2|noarm|retry0) ;; *) echo "unknown cell '$c'"; exit 2 ;; esac
done

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
for v in CH SECS REKEY_S PTK_REKEY_S READY_TIMEOUT; do
  case "${!v}" in ''|*[!0-9]*) echo "$v must be a non-negative integer"; exit 2 ;; esac
done
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

# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init sta hostapd

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
  sta_pid_kill hostapd
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
  if [ "$sta_gone" = 0 ] && [ "$STA_HUNG" = no ]; then sta_dut_handback
  else echo "sta_client still running - not re-enumerating $DUT_SYSFS"; fi
  sta_lock_release
}
trap cleanup EXIT
trap 'cleanup; exit 3' INT TERM

sta_dut_take || exit 2

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

echo "DUT  MT7612U at $DUT_SYSFS ($(sta_usb_id "$DUT_SYSFS"))"
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
ap_up() { # $1 open | wpa2, $2 cell
  {
    printf 'interface=%s\ndriver=nl80211\nssid=%s\n' "$AP_IF" "$SSID"
    if [ "$CH" -le 14 ]; then printf 'hw_mode=g\n'; else printf 'hw_mode=a\n'; fi
    printf 'channel=%s\nieee80211n=1\nauth_algs=1\nwmm_enabled=1\n' "$CH"
    if [ "$1" = wpa2 ]; then
      printf 'wpa=2\nwpa_passphrase=%s\nwpa_key_mgmt=WPA-PSK\nrsn_pairwise=CCMP\n' "$PSK"
      printf 'wpa_group_rekey=%s\nwpa_ptk_rekey=%s\n' "$REKEY_S" "$PTK_REKEY_S"
    fi
  } > "$OUT/hostapd_$2.conf"
  ip netns exec "$NS" hostapd -t "$OUT/hostapd_$2.conf" > "$OUT/hostapd_$2.log" 2>&1 &
  sta_pid_record hostapd $!
  local t=0
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
  env DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 \
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
ping_ap() { # $1 cell
  proc_running "$STA_PID" || return 2
  ping -c 1 -W 3 -I "$TAP" "$APIP" >/dev/null 2>&1            # warm ARP
  ping -c 6 -W 1 -I "$TAP" "$APIP" > "$OUT/ping_$1.txt" 2>&1
  proc_running "$STA_PID" || return 2
  grep -q ' 0% packet loss' "$OUT/ping_$1.txt"
}
loss() { grep -oE '[0-9.]+% packet loss' "$OUT/ping_$1.txt" 2>/dev/null | head -1; }

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

# The clear ran on exit (scored); its result, which is trivially true on
# MT7612U, is information.
check_cleared() { # $1 cell
  local line
  line=$(grep -m1 'station identity clear:' "$OUT/sta_$1.log" | sed 's/^ *//')
  if [ -n "$line" ]; then
    ok "$1: ClearStationIdentity ran on exit"
    info "$1: $line (trivially true on MT7612U: the arm wrote nothing)"
  else
    bad "$1: ClearStationIdentity did not run on exit"
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

cell_end() { sta_stop; sta_pid_kill hostapd; }

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
  if ! wait_for "$OUT/hostapd_open.log" "AP-STA-CONNECTED $own" 30; then
    if proc_running "$STA_PID"; then bad "open: the AP never associated $own"; cell_end
    else station_gone open; sta_pid_kill hostapd; fi
    return
  fi
  ok "open: the AP associated $own"
  ping_ap open; case $? in
    0) ok "open: ping over the air, $(loss open)" ;;
    1) bad "open: ping $(loss open)" ;;
    *) station_gone open; sta_pid_kill hostapd; return ;;
  esac
  cell_end
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
# station has stopped; the caller scores the arm-specific lines.
WPA2_LINK=""
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
  ping_ap "$cell" || p=$?
  if [ "$p" = 2 ]; then station_gone "$cell"; sta_pid_kill hostapd; return 1; fi
  WPA2_LINK="four-way completed, ping $(loss "$cell")"
  [ "$p" = 0 ] && WPA2_LINK="$WPA2_LINK OK"
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
  local assoc mic ptk ans
  assoc=$(led wpa2 'associations'); mic=$(led wpa2 'MIC failures')
  ptk=$(led wpa2 'PTK'); ans=$(led wpa2 'answered')
  if [ "${assoc:-0}" = 1 ] && [ "${ans:-0}" -gt 0 ] && [ "${ptk:-0}" -ge 2 ] &&
     [ "${mic:-999}" -le "${ptk:-0}" ]; then
    ok "wpa2: ledger associations=1, rekeys answered=$ans, PTK installs=$ptk, MIC failures=$mic (<= PTK installs)"
  else
    bad "wpa2: ledger associations=${assoc:-?} answered=${ans:-?} PTK=${ptk:-?} MIC failures=${mic:-?} (expected 1, >0, >=2, MIC <= PTK)"
  fi
  check_armed wpa2
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
  info "noarm: link unarmed: ${WPA2_LINK:-no result} (the MT7612U arm writes no register; a difference from wpa2 here is worth a look)"
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

for c in $CELLS; do "cell_$c"; done

echo
echo "=== $pass passed, $fail failed, $inconclusive inconclusive  (logs: $OUT) ==="
[ "$fail" -gt 0 ] && exit 1
[ "$inconclusive" -gt 0 ] && exit 2
exit 0
