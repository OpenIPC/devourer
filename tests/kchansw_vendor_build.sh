#!/usr/bin/env bash
# kchansw_vendor_build.sh — build reference/rtl88x2bu with a firmware-offload
# path compiled IN, insmod it on the host for the kchansw phase-b
# measurements, and restore the in-tree rtw88 binding afterwards.
#
# The two offload paths are MUTUALLY EXCLUSIVE builds — include/drv_conf.h
# hard-errors on TDLS+MCC together, and MCC additionally requires
# CONCURRENT_MODE on and BT_COEXIST off:
#
#   VARIANT=tdls (default)  CONFIG_TDLS=y → TDLS ch-sw offload H2C 0x1C
#                           (autoconf.h auto-enables CONFIG_TDLS_CH_SW,
#                           runtime-gated on rtw_wifi_spec=1)
#   VARIANT=mcc             CONFIG_MCC_MODE=y CONFIG_BT_COEXIST=n
#                           + -DCONFIG_CONCURRENT_MODE → MCC/FCS H2C family
#                           0x10/0x11/0x18/0x19 + two-vif support
#
# The module is never installed — insmod from the submodule build dir only.
#
# Usage (root):
#   [VARIANT=tdls|mcc] tests/kchansw_vendor_build.sh build   # unattended-safe
#   tests/kchansw_vendor_build.sh load       # unbind rtw88, insmod, show iface
#   tests/kchansw_vendor_build.sh restore    # rmmod, rebind rtw88
#   tests/kchansw_vendor_build.sh status
#
# Env:
#   DUT_VIDPID=2357:012d   the adapter the vendor module should own

set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-build}"
DUT_VIDPID="${DUT_VIDPID:-2357:012d}"
VID="${DUT_VIDPID%%:*}"
PID="${DUT_VIDPID##*:}"
DRVDIR="reference/rtl88x2bu"
KO="$DRVDIR/88x2bu.ko"
INTREE_MOD="rtw88_8822bu"

need_root() { [ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }; }

find_devdir() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$VID" ] && \
    [ "$(cat "$d/idProduct")" = "$PID" ] && { echo "$d"; return 0; }
  done
  return 1
}

iface_for_devdir() {
  local n
  for n in /sys/class/net/*; do
    [ -e "$n/device" ] || continue
    case "$(readlink -f "$n/device")" in
      "$(readlink -f "$1")"*) basename "$n"; return 0 ;;
    esac
  done
  return 1
}

do_build() {
  [ -f "$DRVDIR/Makefile" ] || {
    echo "FAIL: $DRVDIR missing — git submodule update --init"; exit 1; }
  local variant="${VARIANT:-tdls}" flags=() want_sym
  case "$variant" in
    tdls)
      flags=(CONFIG_TDLS=y)
      want_sym=rtw_hal_ch_sw_oper_offload ;;
    mcc)
      flags=(CONFIG_MCC_MODE=y CONFIG_BT_COEXIST=n
             "USER_EXTRA_CFLAGS=-DCONFIG_CONCURRENT_MODE")
      want_sym=rtw_hal_mcc_change_scan_flag ;;
    *) echo "FAIL: VARIANT must be tdls or mcc"; exit 2 ;;
  esac
  echo "== building VARIANT=$variant: ${flags[*]} =="
  make -C "$DRVDIR" clean >/dev/null 2>&1 || true
  if ! make -C "$DRVDIR" "${flags[@]}" -j"$(nproc)" 2>"$DRVDIR/build.err"; then
    echo "FAIL: vendor build broke — errors:"
    grep -E 'error:' "$DRVDIR/build.err" | sort -u | head -15 | sed 's/^/    /'
    tail -8 "$DRVDIR/build.err" | sed 's/^/    /'
    exit 1
  fi
  echo "PASS: built $KO ($(du -h "$KO" | cut -f1)) for $(uname -r), VARIANT=$variant"
  # Prove the offload symbol actually made it in.
  if nm "$KO" 2>/dev/null | grep -q "$want_sym"; then
    echo "  offload symbol present: $want_sym"
  else
    echo "  WARNING: $want_sym not in $KO symbol table"
  fi
}

do_load() {
  need_root
  [ -f "$KO" ] || { echo "FAIL: $KO not built — run '$0 build' first"; exit 1; }
  local devdir iface drv
  devdir="$(find_devdir)" || { echo "FAIL: $DUT_VIDPID not on the bus"; exit 1; }
  # Unbind + unload the in-tree driver so a mid-run re-enumeration cannot
  # race the vendor module for the device (CLAUDE.md auto-probe trap).
  for ifdir in "$devdir":*; do
    [ -L "$ifdir/driver" ] || continue
    drv="$(readlink -f "$ifdir/driver")"
    echo "  unbinding $(basename "$ifdir") from $(basename "$drv")"
    echo "$(basename "$ifdir")" > "$drv/unbind" || true
  done
  modprobe -r "$INTREE_MOD" 2>/dev/null || true
  insmod "$KO" rtw_wifi_spec=1 || { echo "FAIL: insmod (dmesg tail:)";
    dmesg | tail -12 | sed 's/^/    /'; exit 1; }
  local wlan="" i
  for i in $(seq 1 20); do
    wlan="$(iface_for_devdir "$devdir" || true)"
    [ -n "$wlan" ] && break
    sleep 0.5
  done
  [ -n "$wlan" ] || { echo "FAIL: no netdev appeared (dmesg tail:)";
    dmesg | tail -12 | sed 's/^/    /'; exit 1; }
  echo "PASS: 88x2bu loaded (rtw_wifi_spec=1), netdev $wlan"
  echo "next: sudo tests/kchansw_bench.py phase-b"
}

do_restore() {
  need_root
  rmmod 88x2bu 2>/dev/null && echo "  rmmod 88x2bu" || true
  modprobe "$INTREE_MOD" 2>/dev/null || true
  local devdir
  if devdir="$(find_devdir)"; then
    echo "$(basename "$devdir"):1.0" > /sys/bus/usb/drivers_probe 2>/dev/null \
      || true
    echo "  reprobed $(basename "$devdir") → in-tree driver"
  fi
  echo "restore done; dmesg tail:"
  dmesg | tail -5 | sed 's/^/    /'
}

do_status() {
  lsmod | grep -E '^88x2bu|^rtw88_8822bu' || echo "  neither module loaded"
  local devdir
  if devdir="$(find_devdir)"; then
    local drv="none"
    [ -L "$devdir:1.0/driver" ] && drv="$(basename "$(readlink -f "$devdir:1.0/driver")")"
    echo "  $DUT_VIDPID at $(basename "$devdir"), driver: $drv"
  else
    echo "  $DUT_VIDPID not on the bus"
  fi
}

case "$MODE" in
  build)   do_build ;;
  load)    do_load ;;
  restore) do_restore ;;
  status)  do_status ;;
  *) echo "usage: $0 {build|load|restore|status}"; exit 2 ;;
esac
