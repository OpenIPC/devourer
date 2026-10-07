#!/usr/bin/env bash
# On-air cells for the RTL8852CE over vfio-PCIe, witnessed by a USB RTL8852C
# adapter on the same host (the lab box carries two 8852CE cards and a TP-Link
# TX50UH / RTL8832CU). Every cell counts `rx.txhit` events (frames carrying the
# canonical injection SA, see examples/tx/main.cpp) against the frames the
# transmitter submitted, so a number here is a delivery figure for ONE
# direction of ONE pair; the reverse direction is always run beside it.
#
#   sudo tests/pcie_8852ce_onair.sh                     # defaults below
#   sudo tests/pcie_8852ce_onair.sh --channel 6 --frames 1000
#
# Cells, per PCIe card:   usb->pcie   pcie->usb
# then once:              pcie->pcie  (first card TX, second card RX)
#
# Preflight per leg: the receiver must log its RX loop start within
# $LIVENESS_S seconds or the cell aborts with the receiver's last lines — a
# multi-minute run that silently measures a dead receiver is the trap this
# avoids. Cleanup kills only our own rxdemo/txdemo by exact comm name.
set -u

BDFS="0000:05:00.0,0000:09:00.0"
USB_VID=0x35bc
USB_PID=0x0101
CHANNEL=36
FRAMES=2000
GAP_US=5000
BUILD="$(cd "$(dirname "$0")/.." && pwd)/build"
LIVENESS_S=25
RX_SETTLE_S=3
OUT=/tmp/pcie-8852ce-onair
RATE="${DEVOURER_TX_RATE:-6M}"

while [ $# -gt 0 ]; do
  case "$1" in
    --bdfs) BDFS="$2"; shift 2 ;;
    --usb) USB_VID="0x${2%%:*}"; USB_PID="0x${2##*:}"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
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

cleanup() {
  pkill -INT -x rxdemo 2>/dev/null
  pkill -INT -x txdemo 2>/dev/null
  sleep 1
  pkill -KILL -x rxdemo 2>/dev/null
  pkill -KILL -x txdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

IFS=, read -r -a BDF_LIST <<<"$BDFS"

echo "== preflight =="
# rtw89 pre-initializes every Realtek device it binds; the USB DUT must be
# unbound for libusb to claim it, and a loaded rtw89_pci would race the vfio
# bind. Unload what is loaded (idempotent).
for m in rtw89_8852cu rtw89_8852ce rtw89_usb rtw89_pci rtw89_8852c rtw89_core; do
  lsmod | grep -q "^$m " && { modprobe -r "$m" 2>/dev/null && echo "  unloaded $m"; }
done
for bdf in "${BDF_LIST[@]}"; do
  bash "$HERE/pcie_vfio_bind.sh" "$bdf" >/dev/null || { echo "FAIL: vfio bind $bdf" >&2; exit 1; }
  echo "  $bdf bound to vfio-pci"
done
lsusb -d "${USB_VID#0x}:${USB_PID#0x}" >/dev/null || { echo "FAIL: USB DUT ${USB_VID}:${USB_PID} not present" >&2; exit 1; }
echo "  USB DUT ${USB_VID}:${USB_PID} present"

# run_cell NAME RX_KIND RX_SEL TX_KIND TX_SEL
#   KIND = pcie|usb; SEL = bdf for pcie, ignored for usb.
run_cell() {
  local name="$1" rxk="$2" rxsel="$3" txk="$4" txsel="$5"
  local rxlog="$OUT/$name.rx.jsonl" rxerr="$OUT/$name.rx.err"
  local txlog="$OUT/$name.tx.jsonl" txerr="$OUT/$name.tx.err"
  echo "== cell $name: $txk($txsel) -> $rxk($rxsel) ch$CHANNEL $RATE x$FRAMES =="
  local rx_env=(DEVOURER_CHANNEL="$CHANNEL" DEVOURER_LOG_LEVEL=info)
  local tx_env=(DEVOURER_CHANNEL="$CHANNEL" DEVOURER_LOG_LEVEL=info
                DEVOURER_TX_FRAMES="$FRAMES" DEVOURER_TX_GAP_US="$GAP_US"
                DEVOURER_TX_RATE="$RATE")
  if [ "$rxk" = pcie ]; then rx_env+=(DEVOURER_PCIE_BDF="$rxsel");
  else rx_env+=(DEVOURER_VID="$USB_VID" DEVOURER_PID="$USB_PID"); fi
  if [ "$txk" = pcie ]; then tx_env+=(DEVOURER_PCIE_BDF="$txsel");
  else tx_env+=(DEVOURER_VID="$USB_VID" DEVOURER_PID="$USB_PID"); fi

  env "${rx_env[@]}" timeout -s INT $((LIVENESS_S + RX_SETTLE_S + FRAMES * GAP_US / 1000000 + 40)) \
      "$BUILD/rxdemo" >"$rxlog" 2>"$rxerr" &
  local rxpid=$!
  local t=0
  while ! grep -q "RX loop started\|starting RX loop\|async ring of" "$rxerr" 2>/dev/null; do
    sleep 1; t=$((t + 1))
    if ! kill -0 "$rxpid" 2>/dev/null || [ "$t" -ge "$LIVENESS_S" ]; then
      echo "FAIL: receiver not live after ${t}s:"; tail -8 "$rxerr" | sed 's/^/    /'
      kill -INT "$rxpid" 2>/dev/null; wait "$rxpid" 2>/dev/null
      echo "{\"ev\":\"cell\",\"name\":\"$name\",\"ok\":false,\"why\":\"rx-not-live\"}" | tee -a "$OUT/summary.jsonl"
      return 1
    fi
  done
  sleep "$RX_SETTLE_S"
  env "${tx_env[@]}" timeout $((FRAMES * GAP_US / 1000000 + 60)) "$BUILD/txdemo" >"$txlog" 2>"$txerr"
  local txrc=$?
  sleep 2
  kill -INT "$rxpid" 2>/dev/null; wait "$rxpid" 2>/dev/null
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
  printf '  tx rc=%s frames=%s tx_failed=%s hits=%s delivery=%d.%d%%%s\n' "$txrc" "$submitted" "$failed" "$hits" $((pct / 10)) $((pct % 10)) "$approx"
  grep -E "\[E\]" "$txerr" "$rxerr" | grep -v "LTE interface not ready" | head -5 | sed 's/^/    /'
  echo "{\"ev\":\"cell\",\"name\":\"$name\",\"ok\":$([ "$hits" -gt 0 ] && echo true || echo false),\"tx\":\"$txk:$txsel\",\"rx\":\"$rxk:$rxsel\",\"channel\":$CHANNEL,\"rate\":\"$RATE\",\"frames\":$submitted,\"tx_failed\":$failed,\"hits\":$hits,\"tx_rc\":$txrc}" | tee -a "$OUT/summary.jsonl"
  [ "$hits" -gt 0 ]
}

: >"$OUT/summary.jsonl"
fail=0
for bdf in "${BDF_LIST[@]}"; do
  tag="${bdf##*:}"; tag="${bdf%:*}"; tag="${tag##*:}"
  run_cell "usb-to-pcie$tag" pcie "$bdf" usb - || fail=1
  run_cell "pcie$tag-to-usb" usb - pcie "$bdf" || fail=1
done
if [ "${#BDF_LIST[@]}" -ge 2 ]; then
  a="${BDF_LIST[0]}"; b="${BDF_LIST[1]}"
  run_cell "pcie-to-pcie" pcie "$b" pcie "$a" || fail=1
fi
echo "== summary ($OUT/summary.jsonl) =="
cat "$OUT/summary.jsonl"
exit $fail
