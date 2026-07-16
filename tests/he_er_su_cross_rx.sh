#!/usr/bin/env bash
# he_er_su_cross_rx.sh — does the HE ER SU (extended range) PPDU actually AIR
# and DECODE between the two Kestrel dies? The witness is the RX descriptor's
# PPDU-format nibble surfaced in rx.txhit (`ppdu_type`: 7=HE_SU, 8=HE_ERSU) —
# the payload rate alone cannot distinguish ER SU MCS0 from plain SU MCS0, and
# a monitor sniffer's radiotap is blind to the format.
#
# Cells: a plain HE SU MCS0 control (must classify 7), then ER 242-tone,
# ER 106-tone, and ER+DCM (all must classify 8 and still deliver frames).
#
# Defaults pair the 8832CU (35bc:0101, 8852C die) TX with the 8852BU TX20U
# Nano (35bc:0108) RX; pass specs to reverse. When both DUTs share a VID:PID
# (a same-die pair), append the USB topology (@BUS:dotted-port-path, i.e.
# DEVOURER_USB_BUS / DEVOURER_USB_PORT) to disambiguate:
#
#   sudo tests/he_er_su_cross_rx.sh                       # 8852C -> 8852B
#   sudo tests/he_er_su_cross_rx.sh 35bc:0108 35bc:0101   # 8852B -> 8852C
#   sudo tests/he_er_su_cross_rx.sh \
#       35bc:0101@3:2.3.2 35bc:0101@3:2.3.1               # 8852C -> 8852C
#
# Kestrel RX bring-up is bimodal on this bench: a repeatedly soft-re-inited
# DUT comes up deaf (0 frames incl. ambient) until a real VBUS cold — a
# pre-existing bring-up trait, not an ER SU property. Zero-hit cells are
# therefore retried once after a VBUS cold-cycle of the RX DUT when
# REGRESS_VBUS_MAP maps it (format "VID:PID=hub,port;...", uhubctl hubs).
#
# Usage: sudo tests/he_er_su_cross_rx.sh [TX_VID:PID] [RX_VID:PID] [DUR] [CH]
set -u
cd "$(dirname "$0")/.."

TX_SPEC=${1:-35bc:0101}
RX_SPEC=${2:-35bc:0108}
DUR=${3:-15}
CH=${4:-6}

# Split "VID:PID[@BUS:PORTPATH]" -> id part + optional topology part.
TX_ID=${TX_SPEC%%@*}; TX_TOPO=${TX_SPEC#"$TX_ID"}; TX_TOPO=${TX_TOPO#@}
RX_ID=${RX_SPEC%%@*}; RX_TOPO=${RX_SPEC#"$RX_ID"}; RX_TOPO=${RX_TOPO#@}
TX_VID=${TX_ID%:*}; TX_PID=${TX_ID#*:}
RX_VID=${RX_ID%:*}; RX_PID=${RX_ID#*:}
TX_BUS=${TX_TOPO%%:*}; TX_PORT=${TX_TOPO#*:}   # empty when no @topology
RX_BUS=${RX_TOPO%%:*}; RX_PORT=${RX_TOPO#*:}

LOGDIR=/tmp/devourer-he-er-su-cross-rx
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

cleanup() {
  pkill -INT -x txdemo 2>/dev/null
  pkill -INT -x rxdemo 2>/dev/null
  sleep 1
  pkill -KILL -x txdemo 2>/dev/null
  pkill -KILL -x rxdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

# Keep the in-tree rtw89 auto-probe off both DUTs (best effort — modprobe -r
# does NOT survive a re-enumeration; harness rigs blacklist these already).
for m in rtw89_8852bu rtw89_8852cu rtw89_8852bue rtw89_usb; do
  sudo modprobe -r "$m" 2>/dev/null
done

# VBUS cold-cycle the RX DUT when REGRESS_VBUS_MAP maps it (best effort).
vbus_cycle_rx() {
  local map=${REGRESS_VBUS_MAP:-}
  [ -z "$map" ] && return 1
  local entry
  # Full-spec keys (VID:PID@BUS:PORT=hub,port) take precedence — a same-die
  # pair shares its VID:PID, so the id alone is ambiguous there.
  entry=$(tr ';' '\n' <<<"$map" | grep -iF "$RX_SPEC=" | head -1)
  [ -z "$entry" ] && { entry=$(tr ';' '\n' <<<"$map" |
                              grep -i "^$RX_VID:$RX_PID=" | head -1) || return 1; }
  [ -z "$entry" ] && return 1
  local hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  # Post-cycle the dongle first enumerates as its ZeroCD ROM id and only then
  # self-switches to the WLAN id — give it time to complete both stages.
  sleep 12
}

# run_cell_once <name> <tx_rate_spec> <expected ppdu_type>
run_cell_once() {
  local name=$1 rate=$2 want_ppdu=$3
  local rxlog="$LOGDIR/rx-$name.log" txlog="$LOGDIR/tx-$name.log"

  # From real cold the first open of a Kestrel DUT dies at libusb_reset_device
  # ("handle is stale"): the reset reloads firmware and re-enumerates the
  # device. The next open finds it settled — so retry the TX launch too, not
  # just the RX side.
  local txpid try
  for try in 1 2 3; do
    env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL=$CH \
        ${TX_BUS:+DEVOURER_USB_BUS="$TX_BUS"} ${TX_PORT:+DEVOURER_USB_PORT="$TX_PORT"} \
        DEVOURER_TX_RATE="$rate" \
        timeout -s INT -k 5 $((DUR * 3 + 20)) ./build/txdemo >"$txlog" 2>&1 &
    txpid=$!
    sleep 7   # TX bring-up (FWDL + cals) before the RX window opens
    if grep -qE "handle is stale|matched no device" "$txlog" &&
       ! kill -0 "$txpid" 2>/dev/null; then
      wait "$txpid" 2>/dev/null
      echo "  ($name: TX open hit cold re-enumeration, retry $try)"
      sleep 5
      continue
    fi
    break
  done
  for try in 1 2 3; do
    env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL=$CH \
        ${RX_BUS:+DEVOURER_USB_BUS="$RX_BUS"} ${RX_PORT:+DEVOURER_USB_PORT="$RX_PORT"} \
        timeout -s INT -k 5 "$DUR" ./build/rxdemo >"$rxlog" 2>&1
    grep -qE "entering RX loop|starting RX loop" "$rxlog" && break
    echo "  ($name: RX bring-up failed, retry $try)"
  done
  wait "$txpid" 2>/dev/null
  sleep 3

  # rx.txhit events are throttled (first 10, then every 100th) — the true
  # count is the `hits` field of the LAST event. ppdu_type must be uniform
  # across the cell's txhit events, not just present once.
  local hits ptypes
  hits=$(grep '"ev":"rx.txhit"' "$rxlog" | tail -1 |
         grep -oE '"hits":[0-9]+' | cut -d: -f2)
  hits=${hits:-0}
  ptypes=$(grep '"ev":"rx.txhit"' "$rxlog" |
           grep -oE '"ppdu_type":[0-9]+' | cut -d: -f2 | sort -u |
           tr '\n' ',' | sed 's/,$//')
  echo "$hits" >"$LOGDIR/$name.count"
  echo "${ptypes:-none}" >"$LOGDIR/$name.ptype"
  printf "  %-8s (%-18s): %5s hits, ppdu_type={%s} (want %s)\n" \
         "$name" "$rate" "$hits" "${ptypes:-none}" "$want_ppdu"
}

# A zero-hit cell is retried once after a VBUS cold of the RX DUT — the
# deaf-bring-up trait above, not a per-encoding verdict.
run_cell() {
  run_cell_once "$@"
  if [ "$(cat "$LOGDIR/$1.count")" = 0 ] && vbus_cycle_rx; then
    echo "  ($1: zero hits — VBUS-cycled RX DUT, retrying cell)"
    run_cell_once "$@"
  fi
}

echo "HE ER SU cross-RX: TX $TX_SPEC -> RX $RX_SPEC, ch$CH, ${DUR}s cells"
run_cell su    "HE1SS_MCS0"           7
run_cell er    "HE1SS_MCS0/ER"        8
run_cell er106 "HE1SS_MCS0/ER106"     8
run_cell erdcm "HE1SS_MCS0/ER/DCM"    8

verdict() { # <name> <want_ppdu> — OK when delivering and uniformly classified
  local hits ptype
  hits=$(cat "$LOGDIR/$1.count"); ptype=$(cat "$LOGDIR/$1.ptype")
  [ "$hits" -ge 10 ] && [ "$ptype" = "$2" ] && echo OK || echo FAIL
}

C_SU=$(cat "$LOGDIR/su.count")
echo
if [ "$C_SU" -lt 20 ] || [ "$(cat "$LOGDIR/su.ptype")" != 7 ]; then
  echo "VERDICT: bench broken (HE SU control: $C_SU hits, ppdu_type=$(cat "$LOGDIR/su.ptype")) — fix before judging ER"
  exit 1
fi
V_ER=$(verdict er 8); V_106=$(verdict er106 8); V_DCM=$(verdict erdcm 8)
echo "VERDICT: SU=$C_SU/pt7 (control), ER=$(cat "$LOGDIR/er.count")/pt$(cat "$LOGDIR/er.ptype") [$V_ER]," \
     "ER106=$(cat "$LOGDIR/er106.count")/pt$(cat "$LOGDIR/er106.ptype") [$V_106]," \
     "ER+DCM=$(cat "$LOGDIR/erdcm.count")/pt$(cat "$LOGDIR/erdcm.ptype") [$V_DCM]"
echo "logs: $LOGDIR"
[ "$V_ER" = OK ] && [ "$V_106" = OK ] && [ "$V_DCM" = OK ]
