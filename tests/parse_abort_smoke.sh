#!/usr/bin/env bash
#
# parse_abort_smoke.sh — per-generation ambient-RX smoke for rx.parse_abort.
#
# The rx.parse_abort event (src/RxParseAbort.h) fires when a generation's RX
# descriptor walk abandons a bulk-IN buffer on a malformed descriptor. Two
# properties need hardware on every family: frames still flow (the walk is
# untouched on the success path), and the all-zero-padding exclusion holds for
# that family's aggregate format (no spurious event flood on ambient traffic).
#
#   sudo bash tests/parse_abort_smoke.sh          # every known plugged DUT
#   DUTS="0x8813 0xb812" sudo bash tests/parse_abort_smoke.sh
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD=${BUILD:-$ROOT/build}
CH=${CH:-6}          # 2.4 GHz: ambient beacons guarantee RX traffic
DWELL_S=${DWELL_S:-20}
# J1 8814AU, J2 8822BU, J3 8812CU, RTL8733B, Kestrel. The J3 default is the
# 8812CU, not the 8822EU: the 8822E's DPDT front end decodes no ambient 2.4 GHz
# on this bench (green init, DIG sees energy, zero frames) while its 5 GHz RX
# is proven — the walk under test is identical on both dies.
DUTS=${DUTS:-"0x8813 0xb812 0xc812 0xf72b 0x0101"}
OUT=${OUT:-/tmp/parse-abort-smoke}

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 3; }
[ -x "$BUILD/rxdemo" ] || { echo "build rxdemo first"; exit 3; }
mkdir -p "$OUT"

MODS="rtw88_8812au rtw88_8821au rtw88_8822bu rtw88_8814au rtw88_8822cu rtw88_8822eu rtw89_8852bu rtw89_8852cu"
BLACKLIST=/run/modprobe.d/zz-temp-blacklist-pabort.conf
cleanup() {
  trap - EXIT INT TERM
  esc_build=$(printf '%s' "$BUILD" | sed 's/[][\\.^$*+?(){}|]/\\&/g')
  pkill -f "^$esc_build/rxdemo" 2>/dev/null
  rm -f "$BLACKLIST"
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM
mkdir -p "$(dirname "$BLACKLIST")"
: > "$BLACKLIST"
for m in $MODS; do echo "blacklist $m" >> "$BLACKLIST"; modprobe -r "$m" 2>/dev/null; done

rc=0
for pid in $DUTS; do
  vid=0x0bda
  [ "$pid" = "0x0101" ] && vid=0x35bc
  log="$OUT/rx-${vid#0x}${pid#0x}.jsonl"
  echo "[pabort] DUT $vid:$pid — ${DWELL_S}s ambient RX on ch$CH"
  env DEVOURER_VID="$vid" DEVOURER_PID="$pid" DEVOURER_CHANNEL="$CH" \
      DEVOURER_RX_AGG_SA=any \
      DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout \
      timeout -s INT "$DWELL_S" "$BUILD/rxdemo" >"$log" 2>"$OUT/rx-${vid#0x}${pid#0x}.err"
  pkts=$(grep -cF '"ev":"rx.pkt"' "$log" || true)
  frames=$(grep -cF '"ev":"rx.frame"' "$log" || true)
  aborts=$(grep -cF '"ev":"rx.parse_abort"' "$log" || true)
  verdict=OK
  # rx.pkt samples (first 10 + every 100th) — >=2 proves the walk delivers.
  [ "${pkts:-0}" -ge 2 ] || { verdict="FAIL(no-rx)"; rc=1; }
  [ "${aborts:-0}" -eq 0 ] || { verdict="FAIL(aborts=$aborts)"; rc=1; }
  echo "[pabort] $vid:$pid rx.pkt=$pkts rx.frame=$frames parse_aborts=$aborts -> $verdict"
done
echo "[pabort] logs: $OUT"
exit "$rc"
