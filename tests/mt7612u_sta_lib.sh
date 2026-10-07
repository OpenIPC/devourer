# shellcheck shell=sh
# mt7612u_sta_lib.sh - shared plumbing for the station harnesses
# (tests/mt7612u_sta_identity.sh, _autoack.sh, _uplink.sh; the generic helpers
# also serve tests/sta_client_onair.sh and tests/realtek_station_onair.sh).
# Sourced, not run.
#
# Four rules these scripts run as root under:
#
#   - A private OUT. sta_out_prepare() gives an unset OUT a fresh
#     `mktemp -d` directory (mode 0700). A given OUT must not be a symlink,
#     must be a directory owned by root or by the invoking user (SUDO_UID
#     when run through sudo), and is created 0700 when missing - so a local
#     user cannot point this root run's writes somewhere else.
#   - One run per OUT. sta_lock_take() takes an flock(1) on the OUT directory
#     itself and refuses while another run holds it, so two concurrent runs
#     cannot share - and kill each other through - one set of PID files. The
#     kernel drops the lock when the last holder exits, so there is no owner
#     record to race and no stale lock to reclaim. Two runs with different
#     OUTs are NOT kept apart by this lock. sta_dut_take() refuses an
#     MT7612U with a LIVE holder - an interface bound to a driver other than
#     mt76x2u (usbfs: a process has claimed it), or a process with its
#     /dev/bus/usb node open - so a run never toggles `authorized` under a
#     devourer process. An unbound interface nothing holds is taken as it is:
#     a devourer demo detaches mt76x2u and never reattaches it, and a host
#     may blacklist mt76x2u (docs/mt7612u.md). What this cannot see is
#     another harness between two of its gates, when nothing holds the
#     adapter: give concurrent runs different adapters.
#   - Kill only what this run started, by recorded PID. No pattern kills, and
#     no PID read from a file an earlier run left behind: sta_pid_init()
#     removes stale PID files before anything is started.
#   - Hand every adapter back: a devourer-opened adapter through
#     sta_dev_handback() (below; sta_peer_handback() is its PEER_SYSFS
#     form), the AP in tests/mt7612u_sta_identity.sh's cleanup, and the
#     MT7612U DUT here. The harnesses unbind the MT7612U from mt76x2u so
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

STA_LOCKED=no
# The lock lives on fd 9, opened on the OUT directory (nothing is written,
# so nothing can be redirected through a planted file). Taking and holding it
# is one atomic flock. The children this run starts inherit fd 9, so a
# process that outlives the harness (a hung sta_client) keeps OUT locked
# until it exits - which is what it should do.
sta_lock_take() {
  command -v flock >/dev/null 2>&1 || { echo "flock(1) is required"; return 1; }
  exec 9<"$OUT" || { echo "could not open OUT=$OUT"; return 1; }
  if ! flock -n 9; then
    exec 9<&-
    echo "OUT=$OUT is in use by another run - refusing; give this run its" \
         "own OUT"
    return 1
  fi
  STA_LOCKED=yes
  # A reused OUT starts with no device records: a marker an earlier run left
  # would make this run hand back a device it never recorded or opened.
  rm -f "$OUT"/.id_* "$OUT"/.opened_*
  return 0
}

sta_lock_release() {
  [ "$STA_LOCKED" = yes ] || return 0
  STA_LOCKED=no
  exec 9<&-
}

sta_is_mt7612u() {
  [ "$(cat "/sys/bus/usb/devices/$1/idVendor" 2>/dev/null)" = "0e8d" ] &&
  [ "$(cat "/sys/bus/usb/devices/$1/idProduct" 2>/dev/null)" = "7612" ]
}

STA_DUT_TAKEN=no
STA_DUT_ID=""
# The PID of a process that has the USB device at sysfs path $1 open
# (/dev/bus/usb/BBB/DDD), or nothing. Root sees every process's fds.
sta_usb_holder() {
  _sta_b=$(cat "/sys/bus/usb/devices/$1/busnum" 2>/dev/null)
  _sta_d=$(cat "/sys/bus/usb/devices/$1/devnum" 2>/dev/null)
  [ -n "$_sta_b" ] && [ -n "$_sta_d" ] || return 0
  _sta_h=$(find /proc/[0-9]*/fd -maxdepth 1 \
             -lname "$(printf '/dev/bus/usb/%03d/%03d' "$_sta_b" "$_sta_d")" \
             2>/dev/null | head -1)
  [ -n "$_sta_h" ] || return 0
  _sta_h=${_sta_h#/proc/}
  echo "${_sta_h%%/*}"
}

# Refuse a DUT_SYSFS that is not an MT7612U, or that something live holds:
# interface 0 bound to a driver other than mt76x2u (usbfs: a process has
# claimed it), or a process with its device node open. Then unbind it from
# mt76x2u if that is bound, and require that interface 0 really has no
# driver afterwards - a failed unbind leaves the kernel driver owning the
# chip under the probe. An unbound interface nothing holds is taken as it is
# (an earlier devourer session left it so, or mt76x2u is not loaded). Only a
# DUT that was taken is handed back, and only while DUT_SYSFS still reports
# the idVendor:idProduct:serial recorded here.
sta_dut_take() {
  if ! sta_is_mt7612u "$DUT_SYSFS"; then
    echo "refusing DUT_SYSFS=$DUT_SYSFS - not an MT7612U (0e8d:7612)"
    return 1
  fi
  _sta_drv="/sys/bus/usb/devices/$DUT_SYSFS:1.0/driver"
  if [ -e "$_sta_drv" ]; then
    _sta_drv=$(basename "$(readlink -f "$_sta_drv")")
    if [ "$_sta_drv" != mt76x2u ]; then
      echo "refusing DUT_SYSFS=$DUT_SYSFS - interface 0 is held by" \
           "$_sta_drv (usbfs: a process has claimed it)"
      return 1
    fi
  fi
  _sta_pid=$(sta_usb_holder "$DUT_SYSFS")
  if [ -n "$_sta_pid" ]; then
    echo "refusing DUT_SYSFS=$DUT_SYSFS - PID $_sta_pid" \
         "($(cat "/proc/$_sta_pid/comm" 2>/dev/null)) has its USB device open"
    return 1
  fi
  if [ "$_sta_drv" = mt76x2u ]; then
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
  # Back to bound before the next run starts: wait, up to 5 s, for mt76x2u
  # to probe it again - when mt76x2u is loaded at all.
  [ -d /sys/bus/usb/drivers/mt76x2u ] || return 0
  _sta_t=0
  until [ -e "/sys/bus/usb/devices/$DUT_SYSFS:1.0/driver" ]; do
    if [ "$_sta_t" -ge 50 ]; then
      echo "DUT_SYSFS=$DUT_SYSFS: mt76x2u did not bind again within 5 s"
      return 0
    fi
    sleep 0.1; _sta_t=$((_sta_t + 1))
  done
}

# PID files live in $OUT as .pid_<name>. Every name a script uses is listed
# once in sta_pid_init so a previous run's file is gone before this run
# starts anything.
sta_pid_init() {
  for _sta_n in "$@"; do rm -f "$OUT/.pid_$_sta_n"; done
}

sta_pid_record() { echo "$2" > "$OUT/.pid_$1"; }

# Is PID running? `kill -0` alone also succeeds on an exited but unreaped
# child (a zombie, state Z in /proc/PID/stat after the command name).
sta_pid_alive() {
  # An empty or non-numeric PID is not live: /proc//stat is /proc/stat.
  case "$1" in ''|*[!0-9]*) return 1 ;; esac
  _sta_st=$(sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -d' ' -f1)
  [ -n "$_sta_st" ] && [ "$_sta_st" != Z ] && [ "$_sta_st" != X ]
}

# Signal ($2, default TERM) the process recorded under $1, wait up to 10 s
# for it to exit (a demo's chip de-init runs after the signal), reap it if it
# is our child, and forget it. POLLED, never a bare `wait` first: a child
# that ignores the signal - or a background job started with SIGINT ignored,
# as a non-interactive shell starts them - would block that `wait` for good.
# Returns 1, and says so, if it is still alive then - unreaped, and STILL
# RECORDED, so a later call (or sta_pid_live) still finds it and nothing is
# started or re-enumerated under it; 0 otherwise, and silently when nothing
# is recorded.
sta_pid_kill() {
  [ -f "$OUT/.pid_$1" ] || return 0
  _sta_pid=$(cat "$OUT/.pid_$1" 2>/dev/null)
  case "$_sta_pid" in ''|*[!0-9]*) rm -f "$OUT/.pid_$1"; return 0 ;; esac
  kill "-${2:-TERM}" "$_sta_pid" 2>/dev/null
  _sta_t=0
  while sta_pid_alive "$_sta_pid"; do
    if [ "$_sta_t" -ge 100 ]; then
      echo "$1 (pid $_sta_pid) is still running 10 s after SIG${2:-TERM}"
      return 1
    fi
    sleep 0.1; _sta_t=$((_sta_t + 1))
  done
  rm -f "$OUT/.pid_$1"
  wait "$_sta_pid" 2>/dev/null   # exited: reaps our child, no-op otherwise
  return 0
}

# sta_pid_kill, escalated to KILL when the first signal did not end it.
# 1 when the process outlived both; its record is kept.
sta_pid_kill_hard() {
  sta_pid_kill "$1" "${2:-TERM}" || sta_pid_kill "$1" KILL
}

# 0 when a process recorded under $1 is still running.
sta_pid_live() {
  [ -f "$OUT/.pid_$1" ] && sta_pid_alive "$(cat "$OUT/.pid_$1" 2>/dev/null)"
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

# An adapter devourer opens over libusb (a Realtek DUT or peer): the libusb
# open detaches its kernel driver and nothing re-attaches it. Each is kept
# under a NAME, in files in OUT (a process started inside a command
# substitution sets them too, and its variables never reach the parent):
#   sta_dev_record NAME SYSFS VID PID - before the run: refuse a hub and any
#     device that is not VID:PID, and note its idVendor:idProduct:serial;
#   sta_dev_opened NAME - just before a process opens it;
#   sta_dev_handback NAME SYSFS - re-enumerate it with an `authorized` 0/1
#     toggle so its kernel driver binds again - only when this run opened it,
#     and only while SYSFS still reports the recorded identity, so a device
#     that replaced it at the same path is left alone. Idempotent.
sta_dev_record() {
  _sta_dd="/sys/bus/usb/devices/$2"
  if [ "$(cat "$_sta_dd/bDeviceClass" 2>/dev/null)" = "09" ]; then
    echo "refusing $1 at $2 - a hub"; return 1
  fi
  _sta_want=$(printf '%04x:%04x' "$(($3))" "$(($4))" 2>/dev/null)
  _sta_have="$(cat "$_sta_dd/idVendor" 2>/dev/null):$(cat "$_sta_dd/idProduct" 2>/dev/null)"
  if [ "$_sta_have" != "$_sta_want" ]; then
    echo "refusing $1 at $2 - it reports $_sta_have, not $_sta_want"; return 1
  fi
  sta_usb_id "$2" > "$OUT/.id_$1"
  rm -f "$OUT/.opened_$1"
  return 0
}

sta_dev_opened() { : > "$OUT/.opened_$1"; }

sta_dev_handback() {
  [ -f "$OUT/.id_$1" ] || return 0
  _sta_id=$(cat "$OUT/.id_$1" 2>/dev/null)
  rm -f "$OUT/.id_$1"
  [ -e "$OUT/.opened_$1" ] || return 0
  rm -f "$OUT/.opened_$1"
  if [ "$(sta_usb_id "$2")" != "$_sta_id" ]; then
    echo "$1 path $2 no longer names the recorded device ($_sta_id) -" \
         "not re-enumerating it"
    return 0
  fi
  echo 0 > "/sys/bus/usb/devices/$2/authorized" 2>/dev/null
  sleep 2
  echo 1 > "/sys/bus/usb/devices/$2/authorized" 2>/dev/null
}

# Unbind the kernel driver from every interface of the USB device at $1 that
# carries a wireless netdev (rtw88, an out-of-tree rtl88x2*, mt76x2u - and
# not a composite adapter's Bluetooth interface), then require that none is
# left: a driver still bound would own the chip under devourer. Nothing
# bound is fine. Hand the device back with sta_dev_handback.
sta_dev_unbind_wifi() {
  for _sta_if in "/sys/bus/usb/devices/$1:"*; do
    [ -e "$_sta_if/driver" ] || continue
    for _sta_n in "$_sta_if/net/"*; do
      [ -e "$_sta_n/phy80211" ] || continue
      basename "$_sta_if" > "$_sta_if/driver/unbind" 2>/dev/null
      break
    done
  done
  sleep 2
  for _sta_if in "/sys/bus/usb/devices/$1:"*; do
    for _sta_n in "$_sta_if/net/"*; do
      if [ -e "$_sta_n/phy80211" ]; then
        echo "could not free $1: $(basename "$_sta_if") still carries" \
             "$(basename "$_sta_n") ($(basename "$(readlink -f "$_sta_if/driver")"))"
        return 1
      fi
    done
  done
  return 0
}

# The Realtek peer of the MT7612U harnesses: the sta_dev_* helpers on
# PEER_SYSFS / PEER_VID:PEER_PID, which must not be the DUT's path.
sta_peer_record() {
  if [ "$PEER_SYSFS" = "$DUT_SYSFS" ]; then
    echo "refusing PEER_SYSFS=$PEER_SYSFS - it is the DUT's path"; return 1
  fi
  sta_dev_record peer "$PEER_SYSFS" "$PEER_VID" "$PEER_PID"
}
sta_peer_opened() { sta_dev_opened peer; }
sta_peer_handback() { sta_dev_handback peer "$PEER_SYSFS"; }

# mt7612uprobe loads its firmware from ./firmware. sta_fw_link() creates
# $ROOT/firmware -> FW_DIR only when nothing is there - not even a dangling
# symlink, which `-e` alone would miss - and sta_fw_unlink() removes it only
# if this run created it and it still points where this run pointed it.
# Either way it then checks the blobs are readable THROUGH the link (below),
# and returns 1 when they are not.
STA_FW_LINK_OURS=no
sta_fw_link() {
  if [ ! -e "$ROOT/firmware" ] && [ ! -L "$ROOT/firmware" ] &&
     ln -sn "$FW_DIR" "$ROOT/firmware" 2>/dev/null; then
    STA_FW_LINK_OURS=yes
  fi
  sta_fw_readable "$ROOT/firmware"
}

# 0 when directory $1 holds both MT7612U blobs, readable and non-empty. A
# host whose firmware is compressed (/lib/firmware/mediatek/*.bin.zst only,
# as most distributions ship it) links fine, and the DUT then fails its
# bring-up with "cannot open firmware/mt7662_rom_patch.bin", which a gate
# scores as an empty ABORTED or a missing MAC. This check makes that dead rig
# a refusal before anything runs.
sta_fw_readable() {
  for _sta_fw in mt7662_rom_patch.bin mt7662.bin; do
    _sta_p="$1/$_sta_fw"
    if [ ! -e "$_sta_p" ]; then
      _sta_z=""
      for _sta_x in zst xz gz; do
        [ -e "$_sta_p.$_sta_x" ] && _sta_z="$_sta_z $_sta_fw.$_sta_x"
      done
      if [ -n "$_sta_z" ]; then
        echo "refusing: $_sta_p is missing; only compressed firmware is there" \
             "(${_sta_z# }). The DUT loads the blobs uncompressed: decompress" \
             "both into a directory and pass it as FW_DIR"
      else
        echo "refusing: $_sta_p is missing (FW_DIR=$FW_DIR)"
      fi
      return 1
    fi
    if [ ! -f "$_sta_p" ] || [ ! -r "$_sta_p" ] || [ ! -s "$_sta_p" ]; then
      echo "refusing: $_sta_p is not a readable, non-empty file"
      return 1
    fi
  done
  return 0
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
