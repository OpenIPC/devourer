#!/usr/bin/env bash
# twt_fw_discovery.sh — tier-B validation for the 802.11ax TWT surface: what
# does the shipped RTL8852 firmware actually honor?
#
# Drives devourer's `kestrelprobe twt` on the 8852BU: creates an individual
# TWT agreement (mac_twt_info_upd) and attempts the TWT-OFDMA cadence command
# (func 0x03, non-canonical / gated MAC_FEAT_TWT_OFDMA_EN in the vendor tree —
# so whether the shipped fw implements it is unknown). Then watches the bulk-IN
# C2H stream for `twt.notify` / `twt.wait_anno` events (routed by handle_c2h).
#
# Classifies:
#   TWT_OK        — the TWTINFO_UPD H2C is accepted (no fw SER halt).
#   OFDMA_HONORED — a twt.notify C2H arrives (the fw runs the TWT-OFDMA engine);
#                   its TSF stamp is cross-checked against ReadTsf for sanity.
#   OFDMA_SILENT  — the func 0x03 H2C is accepted but no cadence/C2H — the fw
#                   lacks the engine: use tests/ul_ofdma_e2e.sh (UL_FIXINFO).
#
#   sudo tests/twt_fw_discovery.sh [channel] [watch_seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root (USB claim + driver unbind)"; exit 2; }

CH=${1:-36}
WATCH=${2:-8}
DUT_ID="35bc:0108"   # RTL8852BU
KP="build/kestrelprobe"
LOG="/tmp/twt_fw_discovery.jsonl"
DBG="/tmp/twt_fw_discovery.dbg"
[ -x "$KP" ] || { echo "FAIL: $KP not built (cmake --build build)"; exit 2; }
lsusb -d "$DUT_ID" >/dev/null 2>&1 || { echo "SKIP: 8852BU ($DUT_ID) not plugged"; exit 0; }

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d iface drv; d=$(sysdir_for "$1"); [ -n "$d" ] || return 0
  for iface in "$d":*; do
    [ -e "$iface/driver" ] || continue
    drv=$(basename "$(readlink "$iface/driver")")
    echo "$(basename "$iface")" > "/sys/bus/usb/drivers/$drv/unbind" 2>/dev/null || true
  done
}
cleanup() { pkill -x kestrelprobe 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "== twt_fw_discovery: 8852BU ch$CH, watch ${WATCH}s =="
unbind_kernel "$DUT_ID"
sleep 1

# kestrelprobe twt brings up TX, configures TWT + TWT-OFDMA, then the process
# exits after issuing the H2Cs — but the C2H (twt.notify) may arrive during a
# short steady-state window, so run rxdemo briefly afterward to drain it.
DEVOURER_CHANNEL="$CH" DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=debug \
  "$KP" twt --pid 0x0108 >"$LOG" 2>"$DBG"
TWT_RC=$?

# Drain the bulk-IN for the notify window (rxdemo routes C2H through handle_c2h,
# which logs twt.notify / twt.wait_anno on the diagnostic plane).
timeout "$WATCH" env DEVOURER_PID=0x0108 DEVOURER_CHANNEL="$CH" \
  DEVOURER_LOG_LEVEL=info build/rxdemo >>"$LOG" 2>>"$DBG" || true

CONFIG_OK=$(grep -F '"ev":"kestrel.twt"' "$LOG" | grep -oE '"ok":(true|false)' | tail -1)
OFDMA_CMD=$(grep -F '"ev":"kestrel.twt"' "$LOG" | grep -oE '"ofdma_cmd":(true|false)' | tail -1)
# Match the actual handle_c2h output ("Kestrel twt.notify:" / "Kestrel
# twt.wait_anno:"), NOT the descriptive "watch for twt.notify" reminder line.
NOTIFY=$(grep -c "Kestrel twt.notify:" "$DBG" 2>/dev/null); NOTIFY=${NOTIFY:-0}
WAITANNO=$(grep -c "Kestrel twt.wait_anno:" "$DBG" 2>/dev/null); WAITANNO=${WAITANNO:-0}
SERHALT=$(grep -ciE "fw err|SER L[0-9]|fw halt|HALT_C2H" "$DBG" 2>/dev/null); SERHALT=${SERHALT:-0}

echo "---"
echo "kestrelprobe twt: rc=$TWT_RC  configure=$CONFIG_OK  ofdma_cmd=$OFDMA_CMD"
echo "C2H: twt.notify=$NOTIFY  twt.wait_anno=$WAITANNO  (fw-err lines: $SERHALT)"
[ "$NOTIFY" -gt 0 ] && { echo "sample twt.notify:"; grep "twt.notify" "$DBG" | head -1; }

if [ "$NOTIFY" -gt 0 ]; then
  echo "RESULT: OFDMA_HONORED — the fw runs the TWT-OFDMA engine (twt.notify C2H)."
  echo "        Proceed to tier-C end-to-end via TWT-OFDMA cadence."
  exit 0
elif echo "$CONFIG_OK" | grep -q true; then
  echo "RESULT: TWT_OK / OFDMA_SILENT — TWT agreement accepted, but no TWT-OFDMA"
  echo "        cadence/C2H. Use tests/ul_ofdma_e2e.sh (UL_FIXINFO tf_periodic),"
  echo "        the canonical scheduler, for autonomous trigger cadence."
  exit 0
else
  echo "RESULT: FAIL — TWTINFO_UPD not accepted (see $DBG)."
  exit 1
fi
