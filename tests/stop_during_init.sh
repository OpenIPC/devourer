#!/usr/bin/env bash
# Does a StopRxLoop() issued while Init() is still bringing the chip up end
# Init, and does a stale stop leave the next loop alone? Six scenarios on one
# adapter through build/StopDuringInitProbe (the contract is on StopRxLoop in
# src/IRadio.h):
#
#   initstop at 100 / 400 / 1000 / 2500 ms  -- the first two or three land
#              inside bring-up on every generation (Kestrel's is ~3.4 s, so
#              all four do there); the rest hit a running loop. Init must
#              still be running when the stop lands, must return after it,
#              and the chip it left behind must then receive on a standalone
#              loop -- so a bring-up that failed and returned on its own
#              cannot pass as an honoured stop.
#   restart    stop + join, then a standalone StartRxLoop on a new thread --
#              must run and see frames.
#   stale      StopRxLoop before Init -- Init's own loop must still run.
#
# The frame checks need a channel that carries traffic (ambient beacons do).
#
#   sudo tests/stop_during_init.sh                                 # 8822BU, ch6
#   sudo env PID=0xa81a CHANNEL=36 tests/stop_during_init.sh       # 8812EU
#   sudo env VID=0x35bc PID=0x0101 CHANNEL=36 NO_RESET=1 tests/stop_during_init.sh
#   sudo env VID=0x0e8d PID=0x7612 FW_DIR=reference/mt76/firmware tests/stop_during_init.sh
#
# FW_DIR names the decompressed mt7662.bin + mt7662_rom_patch.bin for an
# MT7612U (not embedded, tests/README.md "For MediaTek (MT7612U) DUTs").
#
# (`sudo env ...` rather than `VAR=... sudo`: sudo scrubs the caller's
# environment, and the script reads its knobs from its own.)
#
# A DEADLOCK line means the Init thread never came back: the probe drops the
# USB lock and _Exits, so the adapter may need a re-plug before the next run.
# Exit 0 = every scenario OK, 1 = at least one failed or produced no result.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PROBE="$ROOT/build/StopDuringInitProbe"
VID="${VID:-0x0bda}"
PID="${PID:-0xb812}"
CHANNEL="${CHANNEL:-6}"
NO_RESET="${NO_RESET:-0}"
TIMEOUT_S="${TIMEOUT_S:-20}"
FW_DIR="${FW_DIR:-}"

[ -x "$PROBE" ] || { echo "missing $PROBE (cmake --build build --target StopDuringInitProbe)" >&2; exit 1; }
# A fresh private directory per run: the script is documented as a sudo run,
# and a predictable path under /tmp would let any local user plant a symlink
# for root's stderr redirection to follow.
LOG="$(mktemp -d "${TMPDIR:-/tmp}/devourer-stop-during-init.XXXXXX")"
child=""
cleanup() {
  # The probe's comm is truncated to 15 characters, so a name match cannot
  # find it; kill the child this script started, by pid.
  [ -n "$child" ] && kill "$child" 2>/dev/null
  [ -n "$child" ] && wait "$child" 2>/dev/null
  :
}
trap cleanup EXIT INT TERM

extra=()
[ "$NO_RESET" = 1 ] && extra+=(--no-reset)
[ -n "$FW_DIR" ] && extra+=(--fw-dir "$FW_DIR")
fail=0
run() { # name args...
  local name=$1; shift
  local out
  timeout $((TIMEOUT_S * 3)) "$PROBE" --vid "$VID" --pid "$PID" --channel "$CHANNEL" \
      --timeout-s "$TIMEOUT_S" "${extra[@]}" "$@" >"$LOG/$name.out" 2>"$LOG/$name.err" &
  child=$!
  wait "$child"
  child=""
  out=$(grep RESULT "$LOG/$name.out")
  echo "[$VID:$PID ch$CHANNEL] $name: ${out:-NO-RESULT}"
  case "$out" in *outcome=OK*) ;; *) fail=1 ;; esac
  sleep 1
}
for ms in 100 400 1000 2500; do run "initstop_$ms" --mode initstop --stop-at-ms "$ms"; done
run restart --mode restart --stop-at-ms 6000 --rx-ms 4000
run stale   --mode stale --rx-ms 6000
if [ $fail = 0 ]; then echo "VERDICT: PASS"; rm -rf "$LOG"; else echo "VERDICT: FAIL (logs in $LOG)"; fi
exit $fail
