#!/usr/bin/env bash
# bench_8812au_row.sh — measure the missing RTL8812AU row of
# docs/startup-time.md (issue #205) on the KNOWN-GOOD unit at 3-2.2.
#
# Table semantics: true cold per rep (real VBUS cycle via REGRESS_VBUS_MAP —
# per-CELL, so devourer rx, devourer tx and the kernel cell each start from
# first-plug state), 2.4 GHz ch 6, median of 2, kernel cell from the
# reference/ 88XXau_ohd.ko on the host, beacon flood from the 8814AU.
#
# Rig specifics handled here:
#   - TWO 8812AU units are plugged; bench_init selects DUTs by PID only.
#     The degraded old unit (3-2.3.3, hub 3-2.3 port 3) is powered OFF for
#     the whole session so discovery finds only the good one.
#   - in-tree rtw88_8812au auto-probes on every re-enumeration (udev
#     modalias) and would touch the chip before the timed window — temp
#     blacklist + rmmod for the duration (same pattern as
#     zz-temp-blacklist-8852au.conf).
#
# Usage: sudo bash tests/bench_8812au_row.sh [reps]
#        WARM=1 sudo bash tests/bench_8812au_row.sh [reps]
#            — authorized-toggle between reps instead of VBUS (the doc's
#              "warm" table), still rtw88-free and old-unit-free.

set -u

REPS="${1:-2}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=/tmp/devourer-bench-8812au-row.md
BLACKLIST=/etc/modprobe.d/zz-temp-blacklist-rtw88-8812au.conf

NEW_SYSFS="3-2.2"; NEW_HUB="3-2"; NEW_PORT=2
OLD_HUB="3-2.3";   OLD_PORT=3

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
command -v uhubctl >/dev/null || { echo "uhubctl not installed"; exit 2; }
[ -f "$REPO/reference/rtl8812au/88XXau_ohd.ko" ] || { echo "88XXau_ohd.ko not built"; exit 2; }

cleanup() {
  trap - EXIT INT TERM
  pkill -x txdemo 2>/dev/null
  pkill -x rxdemo 2>/dev/null
  rmmod 88XXau_ohd 2>/dev/null
  rm -f "$BLACKLIST"
  modprobe rtw88_8812au 2>/dev/null
  uhubctl -l "$OLD_HUB" -p "$OLD_PORT" -a on >/dev/null 2>&1
  wait 2>/dev/null
  echo "[cleanup] old-unit port re-powered, rtw88_8812au restored"
}
trap cleanup EXIT INT TERM

echo "blacklist rtw88_8812au" > "$BLACKLIST"
modprobe -r rtw88_8812au 2>/dev/null
uhubctl -l "$OLD_HUB" -p "$OLD_PORT" -a off >/dev/null || exit 1
sleep 2   # let the old unit disappear before DUT discovery

VBUS_MAP="$NEW_SYSFS=$NEW_HUB:$NEW_PORT"
[ "${WARM:-0}" = 1 ] && { VBUS_MAP=""; OUT=/tmp/devourer-bench-8812au-row-warm.md; }
REGRESS_VBUS_MAP="$VBUS_MAP" \
  python3 "$REPO/tests/bench_init.py" \
    --pids 8812 --reps "$REPS" --kernel-host --traffic-from 8813 \
    --variants quiet ${BENCH_EXTRA:-} --out "$OUT"
rc=$?

echo; echo "=== report ($OUT) ==="; cat "$OUT" 2>/dev/null
exit $rc
