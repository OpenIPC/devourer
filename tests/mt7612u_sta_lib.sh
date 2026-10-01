# shellcheck shell=sh
# mt7612u_sta_lib.sh - shared plumbing for the station harnesses
# (tests/mt7612u_sta_identity.sh, _autoack.sh, _uplink.sh; the generic
# helpers also serve tests/realtek_station_onair.sh). Sourced, not run.
#
# Four rules these scripts run as root under:
#
#   - A private OUT. sta_out_prepare() gives an unset OUT a fresh
#     `mktemp -d` directory (mode 0700). A given OUT must not be a symlink,
#     must be a directory owned by root or by the invoking user (SUDO_UID
#     when run through sudo), and is created 0700 when missing - so a local
#     user cannot point this root run's writes somewhere else.
#   - One run per OUT. sta_lock_take() claims $OUT/.lock (mkdir is atomic) and
#     refuses while the run that holds it is alive, so two concurrent runs
#     cannot share - and kill each other through - one set of PID files. A
#     lock whose holder is gone is reclaimed. (The adapters are exclusive
#     anyway: mt7612uprobe and the Realtek demos take a per-adapter lock.)
#   - Kill only what this run started, by recorded PID. No pattern kills, and
#     no PID read from a file an earlier run left behind: sta_pid_init()
#     removes stale PID files before anything is started.
#   - Hand every adapter back: the Realtek peer through sta_peer_handback()
#     (below), the AP in tests/mt7612u_sta_identity.sh's cleanup, and the DUT
#     here. The harnesses unbind the MT7612U from mt76x2u so
#     mt7612uprobe can claim it; sta_dut_handback() re-enumerates it with an
#     `authorized` 0/1 toggle so the kernel driver binds again, exactly as
#     tests/mt7612u_ap_onair.sh does - and only after confirming the path
#     still names an MT7612U (idVendor 0e8d, idProduct 7612), because a stale
#     DUT_SYSFS would otherwise re-enumerate whatever else is plugged there.
#     The toggle is not a power cycle: chip state survives it.
#
# Needs: DUT_SYSFS, OUT (and PEER_SYSFS for the sta_peer_* helpers).

sta_out_prepare() {
  if [ -z "${OUT:-}" ]; then
    OUT=$(mktemp -d "${TMPDIR:-/tmp}/mt7612u-sta.XXXXXX") || {
      echo "could not create a private OUT directory"; return 1; }
    return 0
  fi
  if [ -L "$OUT" ]; then
    echo "refusing OUT=$OUT - it is a symlink"; return 1
  fi
  if [ ! -e "$OUT" ]; then
    # One level only, so the mode applies to the directory this run owns.
    mkdir -m 0700 "$OUT" || {
      echo "could not create OUT=$OUT (its parent must exist)"; return 1; }
  fi
  if [ ! -d "$OUT" ]; then
    echo "refusing OUT=$OUT - not a directory"; return 1
  fi
  _sta_owner=$(stat -c %u "$OUT" 2>/dev/null)
  if [ "$_sta_owner" != "$(id -u)" ] && [ "$_sta_owner" != "${SUDO_UID:-x}" ]; then
    echo "refusing OUT=$OUT - owned by uid $_sta_owner, not by root or the" \
         "invoking user"
    return 1
  fi
  return 0
}

# A process's start time in clock ticks since boot (/proc/PID/stat field 22),
# or nothing. Field 2 is the command name in parentheses and may hold spaces,
# so the fields are counted from after its closing parenthesis.
sta_proc_start() {
  sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -d' ' -f20
}

STA_LOCKED=no
# The lock records "PID starttime". A holder counts as live only when that
# PID exists AND started at the recorded time - a recycled PID of an
# unrelated process is a stale lock, not a live run.
sta_lock_take() {
  if ! mkdir "$OUT/.lock" 2>/dev/null; then
    read -r _sta_holder _sta_hstart < "$OUT/.lock/pid" 2>/dev/null
    case "${_sta_holder:-}" in
      ''|*[!0-9]*) ;;
      *) if [ -n "${_sta_hstart:-}" ] &&
            [ "$(sta_proc_start "$_sta_holder")" = "$_sta_hstart" ]; then
           echo "OUT=$OUT is in use by run $_sta_holder - refusing; give this" \
                "run its own OUT"
           return 1
         fi ;;
    esac
    rm -rf "$OUT/.lock"
    mkdir "$OUT/.lock" 2>/dev/null || { echo "could not lock OUT=$OUT"; return 1; }
  fi
  echo "$$ $(sta_proc_start "$$")" > "$OUT/.lock/pid"
  STA_LOCKED=yes
  return 0
}

sta_lock_release() {
  [ "$STA_LOCKED" = yes ] || return 0
  STA_LOCKED=no
  rm -rf "$OUT/.lock"
}

sta_is_mt7612u() {
  [ "$(cat "/sys/bus/usb/devices/$1/idVendor" 2>/dev/null)" = "0e8d" ] &&
  [ "$(cat "/sys/bus/usb/devices/$1/idProduct" 2>/dev/null)" = "7612" ]
}

STA_DUT_TAKEN=no
STA_DUT_ID=""
# Refuse a DUT_SYSFS that is not an MT7612U, then unbind it from mt76x2u and
# require that interface 0 really has no driver afterwards - a failed unbind
# leaves the kernel driver owning the chip under the probe. Only a DUT that was
# taken is handed back, and only while DUT_SYSFS still reports the
# idVendor:idProduct:serial recorded here.
sta_dut_take() {
  if ! sta_is_mt7612u "$DUT_SYSFS"; then
    echo "refusing DUT_SYSFS=$DUT_SYSFS - not an MT7612U (0e8d:7612)"
    return 1
  fi
  if [ -e "/sys/bus/usb/devices/$DUT_SYSFS:1.0/driver" ]; then
    echo "$DUT_SYSFS:1.0" > /sys/bus/usb/drivers/mt76x2u/unbind 2>/dev/null
    sleep 2
  fi
  if [ -e "/sys/bus/usb/devices/$DUT_SYSFS:1.0/driver" ]; then
    echo "could not free DUT_SYSFS=$DUT_SYSFS: interface 0 is still bound to" \
         "$(basename "$(readlink -f "/sys/bus/usb/devices/$DUT_SYSFS:1.0/driver")")"
    return 1
  fi
  STA_DUT_ID=$(sta_usb_id "$DUT_SYSFS")
  STA_DUT_TAKEN=yes
  return 0
}

sta_dut_handback() {
  [ "$STA_DUT_TAKEN" = yes ] || return 0
  STA_DUT_TAKEN=no
  if ! sta_is_mt7612u "$DUT_SYSFS" ||
     [ "$(sta_usb_id "$DUT_SYSFS")" != "$STA_DUT_ID" ]; then
    echo "DUT_SYSFS=$DUT_SYSFS no longer names the DUT taken ($STA_DUT_ID)" \
         "- not re-enumerating it"
    return 0
  fi
  echo 0 > "/sys/bus/usb/devices/$DUT_SYSFS/authorized" 2>/dev/null
  sleep 2
  echo 1 > "/sys/bus/usb/devices/$DUT_SYSFS/authorized" 2>/dev/null
}

# PID files live in $OUT as .pid_<name>. Every name a script uses is listed
# once in sta_pid_init so a previous run's file is gone before this run
# starts anything.
sta_pid_init() {
  for _sta_n in "$@"; do rm -f "$OUT/.pid_$_sta_n"; done
}

sta_pid_record() { echo "$2" > "$OUT/.pid_$1"; }

# Signal ($2, default TERM) the process recorded under $1, reap it if it is
# our child, and forget it. A process started inside a command substitution
# is not this shell's child, so `wait` returns at once for it: poll `kill -0`
# for up to 10 s so the caller knows it has really exited (a demo's chip
# de-init runs after the signal). Returns 1, and says so, if it is still
# alive then; 0 otherwise, and silently when nothing is recorded.
sta_pid_kill() {
  [ -f "$OUT/.pid_$1" ] || return 0
  _sta_pid=$(cat "$OUT/.pid_$1" 2>/dev/null)
  rm -f "$OUT/.pid_$1"
  case "$_sta_pid" in ''|*[!0-9]*) return 0 ;; esac
  kill "-${2:-TERM}" "$_sta_pid" 2>/dev/null
  wait "$_sta_pid" 2>/dev/null
  _sta_t=0
  while kill -0 "$_sta_pid" 2>/dev/null; do
    if [ "$_sta_t" -ge 100 ]; then
      echo "$1 (pid $_sta_pid) is still running 10 s after SIG${2:-TERM}"
      return 1
    fi
    sleep 0.1; _sta_t=$((_sta_t + 1))
  done
  return 0
}

# A USB device's identity as idVendor:idProduct:serial (serial empty when the
# device has none), or nothing when the path names no device. Recorded when a
# device is accepted and compared before anything destructive is done to the
# same path later: a different device can enumerate there in between.
sta_usb_id() {
  [ -e "/sys/bus/usb/devices/$1/idVendor" ] || return 0
  printf '%s:%s:%s\n' \
    "$(cat "/sys/bus/usb/devices/$1/idVendor" 2>/dev/null)" \
    "$(cat "/sys/bus/usb/devices/$1/idProduct" 2>/dev/null)" \
    "$(cat "/sys/bus/usb/devices/$1/serial" 2>/dev/null)"
}

# The Realtek peer (PEER_SYSFS) is opened by txdemo / rxdemo, whose libusb
# open detaches its kernel driver and never re-attaches it. sta_peer_record()
# checks and notes the peer's identity before the run; sta_peer_opened() marks
# it touched (a file in OUT, because the peer is started inside a command
# substitution whose variables the parent never sees) just before a peer
# process starts; sta_peer_handback() re-enumerates it with an `authorized`
# 0/1 toggle so its driver binds again - only when this run did open it, and
# only while PEER_SYSFS still reports the recorded identity, so a device that
# replaced it at the same path is left alone.
STA_PEER_ID=""
# The peer must be the adapter the run was told about: PEER_VID:PEER_PID at
# PEER_SYSFS, not a hub, not the DUT's path. Checked before anything runs.
sta_peer_record() {
  _sta_pd="/sys/bus/usb/devices/$PEER_SYSFS"
  if [ "$PEER_SYSFS" = "$DUT_SYSFS" ]; then
    echo "refusing PEER_SYSFS=$PEER_SYSFS - it is the DUT's path"; return 1
  fi
  if [ "$(cat "$_sta_pd/bDeviceClass" 2>/dev/null)" = "09" ]; then
    echo "refusing PEER_SYSFS=$PEER_SYSFS - a hub"; return 1
  fi
  _sta_want=$(printf '%04x:%04x' "$((PEER_VID))" "$((PEER_PID))" 2>/dev/null)
  _sta_have="$(cat "$_sta_pd/idVendor" 2>/dev/null):$(cat "$_sta_pd/idProduct" 2>/dev/null)"
  if [ "$_sta_have" != "$_sta_want" ]; then
    echo "refusing PEER_SYSFS=$PEER_SYSFS - it reports $_sta_have, not" \
         "PEER_VID:PEER_PID $_sta_want"
    return 1
  fi
  STA_PEER_ID=$(sta_usb_id "$PEER_SYSFS")
  rm -f "$OUT/.peer_opened"
  return 0
}

sta_peer_opened() { : > "$OUT/.peer_opened"; }

sta_peer_handback() {
  [ -n "$STA_PEER_ID" ] || return 0
  _sta_peer_id=$STA_PEER_ID
  STA_PEER_ID=""
  if [ ! -e "$OUT/.peer_opened" ]; then
    return 0
  fi
  rm -f "$OUT/.peer_opened"
  if [ "$(sta_usb_id "$PEER_SYSFS")" != "$_sta_peer_id" ]; then
    echo "PEER_SYSFS=$PEER_SYSFS no longer names the recorded peer" \
         "($_sta_peer_id) - not re-enumerating it"
    return 0
  fi
  echo 0 > "/sys/bus/usb/devices/$PEER_SYSFS/authorized" 2>/dev/null
  sleep 2
  echo 1 > "/sys/bus/usb/devices/$PEER_SYSFS/authorized" 2>/dev/null
}

# mt7612uprobe loads its firmware from ./firmware. sta_fw_link() creates
# $ROOT/firmware -> FW_DIR only when nothing is there - not even a dangling
# symlink, which `-e` alone would miss - and sta_fw_unlink() removes it only
# if this run created it and it still points where this run pointed it.
STA_FW_LINK_OURS=no
sta_fw_link() {
  if [ ! -e "$ROOT/firmware" ] && [ ! -L "$ROOT/firmware" ] &&
     ln -sn "$FW_DIR" "$ROOT/firmware" 2>/dev/null; then
    STA_FW_LINK_OURS=yes
  fi
}

sta_fw_unlink() {
  [ "$STA_FW_LINK_OURS" = yes ] || return 0
  STA_FW_LINK_OURS=no
  [ -L "$ROOT/firmware" ] &&
    [ "$(readlink "$ROOT/firmware")" = "$FW_DIR" ] && rm -f "$ROOT/firmware"
  return 0
}

# The first netdev on USB interface <sysfs>:1.0, or nothing.
sta_first_netdev() {
  for _sta_d in "/sys/bus/usb/devices/$1:1.0/net/"*; do
    [ -e "$_sta_d" ] && { basename "$_sta_d"; return 0; }
  done
  return 0
}
