#!/usr/bin/env bash
# On-air cells for the RTL8822CE (Jaguar3 HAL over the 88xx ring plane) on
# vfio-PCIe, witnessed by a USB Realtek adapter on the same host (the lab
# box z890p carries the 8822CE at 0000:85:00.0 and an RTL8812AU). Every
# cell counts `rx.txhit` events (frames carrying the canonical injection SA,
# see examples/tx/main.cpp) against the frames the transmitter submitted, so
# a number here is a delivery figure for ONE direction of ONE pair; the
# reverse direction is always run beside it.
#
#   sudo tests/pcie_8822ce_onair.sh                       # defaults below
#   sudo tests/pcie_8822ce_onair.sh --channel 6 --frames 1000
#   sudo tests/pcie_8822ce_onair.sh --usb 0bda:c811       # another witness
#
# Cells:  usb->pcie   pcie->usb   then the hop cell: the PCIe card injects
# while FastRetune-hopping over $HOP_CHANNELS and the witness sits on each of
# those channels in turn — every channel must see hits (a hop that lands but
# never returns shows as one channel at 0).
#
# Preflight per leg: the receiver must log its RX loop start within
# $LIVENESS_S seconds or the cell aborts with the receiver's last lines — a
# multi-minute run that silently measures a dead receiver is the trap this
# avoids. Cleanup signals only the demo PIDs this run started. The host's
# AER correctable-error count for the PCIe card is bracketed around every
# cell and reported in the summary (the 8822CE logs replay-timer timeouts in
# 5 s bursts whenever it is powered, under rtw88 and under devourer alike;
# tests/pcie_host_prep.sh prints the same counter).
set -u

BDFS="0000:85:00.0"
USB_VID=0x0bda
USB_PID=0x8812
CHANNEL=36
BAND=0
FRAMES=2000
GAP_US=5000
HOP_CHANNELS="36,40,44"
HOP_DWELL_FRAMES=50
BUILD="$(cd "$(dirname "$0")/.." && pwd)/build"
LIVENESS_S=40
RX_SETTLE_S=3
OUT=/tmp/pcie-8822ce-onair
RATE="${DEVOURER_TX_RATE:-6M}"
while [ $# -gt 0 ]; do
  case "$1" in
    --bdfs) BDFS="$2"; shift 2 ;;
    --usb) USB_VID="0x${2%%:*}"; USB_PID="0x${2##*:}"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --band) BAND="$2"; shift 2 ;;
    --hop) HOP_CHANNELS="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --gap-us) GAP_US="$2"; shift 2 ;;
    --build) BUILD="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

[ "$(id -u)" = 0 ] || { echo "ERROR: run as root (vfio + libusb claim)" >&2; exit 2; }
[ -x "$BUILD/rxdemo" ] && [ -x "$BUILD/txdemo" ] || {
  echo "ERROR: $BUILD/rxdemo or txdemo missing (build with -DDEVOURER_PCIE=ON)" >&2; exit 2; }
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"

# Only the demos this run started are ours to stop: the receiver runs in the
# background (RXPID), the transmitter in the foreground (TXPID); another
# user's radio process on the same host is never touched.
RXPID=""
TXPID=""
cleanup() {
  for pid in $RXPID $TXPID; do kill -INT "$pid" 2>/dev/null; done
  sleep 1
  for pid in $RXPID $TXPID; do kill -KILL "$pid" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

IFS=, read -r -a BDF_LIST <<<"$BDFS"

echo "== preflight =="
# The in-tree rtw88 modules auto-probe both the PCIe card (rtw88_8822ce) and
# the USB witness (rtw88_8812au on kernels that ship it); pcie_host_prep.sh
# temp-blacklists + unloads them and binds the card to vfio-pci (idempotent).
for bdf in "${BDF_LIST[@]}"; do
  bash "$HERE/pcie_host_prep.sh" "$bdf" --usb-driver rtw88_8812au >/dev/null \
    || { echo "FAIL: host prep / vfio bind $bdf" >&2; exit 1; }
  echo "  $bdf bound to vfio-pci"
done
aer_count() { journalctl -k -b 0 --no-pager 2>/dev/null | grep -c "${BDF_LIST[0]}: PCIe Bus Error" || true; }
lsusb -d "${USB_VID#0x}:${USB_PID#0x}" >/dev/null || { echo "FAIL: USB DUT ${USB_VID}:${USB_PID} not present" >&2; exit 1; }
echo "  USB DUT ${USB_VID}:${USB_PID} present"

# run_cell NAME RX_KIND RX_SEL TX_KIND TX_SEL [RX_CHANNEL] [TX_HOP_SPEC]
#   KIND = pcie|usb; SEL = bdf for pcie, ignored for usb. RX_CHANNEL
#   overrides $CHANNEL for the receiver; TX_HOP_SPEC makes the transmitter
#   FastRetune-hop (DEVOURER_HOP_CHANNELS) with $HOP_DWELL_FRAMES per dwell.
run_cell() {
  local name="$1" rxk="$2" rxsel="$3" txk="$4" txsel="$5"
  local rxch="${6:-$CHANNEL}" hop="${7:-}"
  local rxlog="$OUT/$name.rx.jsonl" rxerr="$OUT/$name.rx.err"
  local txlog="$OUT/$name.tx.jsonl" txerr="$OUT/$name.tx.err"
  echo "== cell $name: $txk($txsel) -> $rxk($rxsel) band$BAND rxch$rxch${hop:+ hop=$hop} $RATE x$FRAMES =="
  local aer0; aer0=$(aer_count)
  local rx_env=(DEVOURER_CHANNEL="$rxch" DEVOURER_BAND="$BAND" DEVOURER_LOG_LEVEL=info)
  local tx_env=(DEVOURER_CHANNEL="$CHANNEL" DEVOURER_BAND="$BAND" DEVOURER_LOG_LEVEL=info
                DEVOURER_TX_FRAMES="$FRAMES" DEVOURER_TX_GAP_US="$GAP_US"
                DEVOURER_TX_RATE="$RATE")
  if [ -n "$hop" ]; then
    tx_env+=(DEVOURER_HOP_CHANNELS="$hop" DEVOURER_HOP_DWELL_FRAMES="$HOP_DWELL_FRAMES"
             DEVOURER_HOP_FAST=1)
  fi
  if [ "$rxk" = pcie ]; then rx_env+=(DEVOURER_PCIE_BDF="$rxsel");
  else rx_env+=(DEVOURER_VID="$USB_VID" DEVOURER_PID="$USB_PID"); fi
  if [ "$txk" = pcie ]; then tx_env+=(DEVOURER_PCIE_BDF="$txsel");
  else tx_env+=(DEVOURER_VID="$USB_VID" DEVOURER_PID="$USB_PID"); fi

  env "${rx_env[@]}" timeout -s INT $((LIVENESS_S + RX_SETTLE_S + FRAMES * GAP_US / 1000000 + 40)) \
      "$BUILD/rxdemo" >"$rxlog" 2>"$rxerr" &
  local rxpid=$!
  RXPID=$rxpid
  local t=0
  while ! grep -q "RX loop started\|starting RX loop\|async ring of" "$rxerr" 2>/dev/null; do
    sleep 1; t=$((t + 1))
    if ! kill -0 "$rxpid" 2>/dev/null || [ "$t" -ge "$LIVENESS_S" ]; then
      echo "FAIL: receiver not live after ${t}s:"; tail -8 "$rxerr" | sed 's/^/    /'
      kill -INT "$rxpid" 2>/dev/null; wait "$rxpid" 2>/dev/null
      RXPID=""
      echo "{\"ev\":\"cell\",\"name\":\"$name\",\"ok\":false,\"why\":\"rx-not-live\"}" | tee -a "$OUT/summary.jsonl"
      return 1
    fi
  done
  sleep "$RX_SETTLE_S"
  env "${tx_env[@]}" timeout $((FRAMES * GAP_US / 1000000 + 60)) "$BUILD/txdemo" >"$txlog" 2>"$txerr" &
  TXPID=$!
  wait "$TXPID"
  local txrc=$?
  TXPID=""
  sleep 2
  kill -INT "$rxpid" 2>/dev/null; wait "$rxpid" 2>/dev/null
  RXPID=""
  local hits failed
  # rxdemo's final rx.txhit (emitted at exit) carries the exact hit count; the
  # in-loop ones only fire every 100 hits.
  hits=$(grep '"ev":"rx.txhit"' "$rxlog" | grep '"final":1' | tail -1 | grep -o '"hits":[0-9]*' | grep -o '[0-9]*$')
  local approx=""
  if [ -z "$hits" ]; then
    # No exit summary (receiver killed hard): the last in-loop event is a lower
    # bound, quantized to 100 — say so rather than report 0.
    hits=$(grep '"ev":"rx.txhit"' "$rxlog" | tail -1 | grep -o '"hits":[0-9]*' | grep -o '[0-9]*$'); hits=${hits:-0}
    approx=" (lower bound: no final summary)"
  fi
  # txdemo submitted exactly FRAMES frames (DEVOURER_TX_FRAMES bound); the
  # tx.stats "submitted" counter also counts FWDL/H2C submissions, so the
  # denominator is the request and "failed" is the transport's refusal count.
  failed=$(grep '"ev":"tx.stats"' "$txlog" | tail -1 | grep -o '"failed":[0-9]*' | grep -o '[0-9]*$'); failed=${failed:-0}
  local submitted=$FRAMES
  local pct=$(( hits * 1000 / submitted ))
  local aer1; aer1=$(aer_count)
  printf '  tx rc=%s frames=%s tx_failed=%s hits=%s delivery=%d.%d%%%s aer_delta=%s\n' "$txrc" "$submitted" "$failed" "$hits" $((pct / 10)) $((pct % 10)) "$approx" "$((aer1 - aer0))"
  grep -E "\[E\]" "$txerr" "$rxerr" | head -5 | sed 's/^/    /'
  echo "{\"ev\":\"cell\",\"name\":\"$name\",\"ok\":$([ "$hits" -gt 0 ] && echo true || echo false),\"tx\":\"$txk:$txsel\",\"rx\":\"$rxk:$rxsel\",\"band\":$BAND,\"rx_channel\":$rxch,\"hop\":\"$hop\",\"rate\":\"$RATE\",\"frames\":$submitted,\"tx_failed\":$failed,\"hits\":$hits,\"tx_rc\":$txrc,\"aer_delta\":$((aer1 - aer0))}" | tee -a "$OUT/summary.jsonl"
  [ "$hits" -gt 0 ]
}

: >"$OUT/summary.jsonl"
fail=0
for bdf in "${BDF_LIST[@]}"; do
  tag="${bdf%:*}"; tag="${tag##*:}"
  run_cell "usb-to-pcie$tag" pcie "$bdf" usb - || fail=1
  run_cell "pcie$tag-to-usb" usb - pcie "$bdf" || fail=1
  # Hop cell: one TX run per witness channel; the transmitter cycles
  # $HOP_CHANNELS and the witness must see hits on each of them.
  IFS=, read -r -a HOP_LIST <<<"$HOP_CHANNELS"
  for ch in "${HOP_LIST[@]}"; do
    run_cell "pcie$tag-hop-rx$ch" usb - pcie "$bdf" "$ch" "$HOP_CHANNELS" || fail=1
  done
done
echo "== summary ($OUT/summary.jsonl) =="
cat "$OUT/summary.jsonl"
exit $fail
