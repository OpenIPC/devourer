#!/usr/bin/env bash
# devourer vs the vendor kernel driver on the SAME adapter, same channel, same
# rates, judged by a THIRD-PARTY receiver — the control that decides whether a
# high-MCS delivery wall is a property of the rig or a devourer TX defect.
#
# Delivery alone cannot tell those apart: a weak link and a transmitter that
# mis-drives high-order constellations look identical from the receive side.
# Running the vendor driver on the same dongle at the same MCS into the same
# sniffer separates them. If the vendor lands MCS7 where devourer does not, it
# is our bug.
#
# The sniffer is an AR9271 (ath9k_htc): a different vendor's 802.11n radio, so
# it shares no silicon, firmware or driver code with either side under test.
# 1x1 HT, so HT MCS0-7 only — deliberately, since the question is about the
# constellation (16-QAM vs 64-QAM), not the stream count.
#
# Three methodology rules this harness enforces, each learned by getting a wrong
# answer without it:
#
#  1. CONTROL DRIVER. The in-tree rtw88 accepts frames on a monitor netdev and
#     reports them injected while airing nothing a sniffer decodes. It reads as
#     a kernel-side failure at every rate and makes the comparison worthless.
#     Use the vendor driver from reference/ (raw-injection capable), which is
#     what devourer treats as ground truth anyway.
#  2. COLD CYCLE BETWEEN DRIVERS. Swapping libusb <-> kernel driver on a warm
#     chip leaves it wedged: an observed run had the vendor side accept 144k
#     frames and deliver zero at every rate, right after a run where the same
#     rate delivered fine. VBUS-cycle between halves (REGRESS_VBUS_MAP).
#  3. INTERLEAVE AND SANITY-GATE. Ambient 2.4 GHz drifts enough to move one
#     side's delivery 2x between runs, so all-A-then-all-B compares two
#     different channels. Interleave per rate, and bracket every cell with a
#     robust-rate sanity cell — a half whose sanity cell delivers nothing is
#     reported INVALID, never as a zero result.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=4-2.3,2" tests/nitroqam_kernel_ab.sh 6
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

CH=${CH:-${1:-6}}
DUT_VID=${DUT_VID:-0bda} DUT_PID=${DUT_PID:-8812}
DUT_REF=${DUT_REF:-rtl8812au}
DUT_KO=${DUT_KO:-88XXau_ohd.ko}
DUT_MOD=${DUT_MOD:-88XXau_ohd}
INTREE_DRV=${INTREE_DRV:-rtw88_8812au}
SNIFF=${SNIFF:-wlp13s0u1u3}          # AR9271
RATES=${RATES:-"4 5 6 7"}
SANITY_RATE=${SANITY_RATE:-0}        # robust rate that must always deliver
SECS=${SECS:-6}
# Kernel-side body size. The monitor netdev's MTU caps radiotap + 802.11 header
# + body at 1500, so the body must leave room for both. devourer is handed the
# matching PSDU (body + the 24-byte 802.11 header) so both sides put the same
# number of bits on air per frame.
# Inter-frame gap for the devourer side. Exposed so the cold-cycle effect can
# be separated from the feed rate: the original zero-delivery runs used 2000.
TX_GAP_US=${TX_GAP_US:-0}
PAYLOAD=${PAYLOAD:-1400}
DOT11_HDR=24
CANON=57:42:75:05:d6:00
OUT=${OUT:-/tmp/devourer-nq-kernel-ab}

mkdir -p "$OUT"
RESULTS="$OUT/cells.tsv"
: > "$RESULTS"
command -v tcpdump >/dev/null || { echo "need tcpdump" >&2; exit 2; }
[ -x "$ROOT/build/txdemo" ] || { echo "build first" >&2; exit 2; }

DUT_IF=""
cleanup() {
  sudo pkill -f "tcpdump -i $SNIFF" 2>/dev/null
  sudo pkill -x txdemo 2>/dev/null
  sudo pkill -f kernel_tx_inject 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# --- sniffer ---------------------------------------------------------------
[ -e "/sys/class/net/$SNIFF" ] || { echo "no sniffer iface $SNIFF" >&2; exit 2; }
sudo ip link set "$SNIFF" down
sudo iw dev "$SNIFF" set type monitor
sudo ip link set "$SNIFF" up
sudo iw dev "$SNIFF" set channel "$CH"
sleep 1
echo "sniffer $SNIFF ambient: $(sudo timeout 3 tcpdump -i "$SNIFF" -nn 2>/dev/null | wc -l) frames/3s on ch$CH"

dut_usb() {
  for d in /sys/bus/usb/devices/*/; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$DUT_VID" ] || continue
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "$DUT_PID" ] || continue
    basename "$d"; return 0
  done
  return 1
}

# Real VBUS cycle where the hub supports it; the authorized toggle is a weaker
# fallback that does NOT clear chip state (docs/adapter-doctor.md).
cold_cycle() {
  local map=${REGRESS_VBUS_MAP:-} entry hubport
  if [ -n "$map" ]; then
    entry=$(tr ';' '\n' <<<"$map" | grep -i "^$DUT_VID:$DUT_PID=" | head -1) || true
    if [ -n "${entry:-}" ]; then
      hubport=${entry#*=}
      sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
      sleep 12
      return 0
    fi
  fi
  local u; u=$(dut_usb) || return 1
  echo 0 | sudo tee "/sys/bus/usb/devices/$u/authorized" >/dev/null
  sleep 2
  echo 1 | sudo tee "/sys/bus/usb/devices/$u/authorized" >/dev/null
  sleep 6
  return 0
}

build_vendor() {
  local dir="$ROOT/reference/$DUT_REF"
  [ -d "$dir/core" ] || {
    echo "!! $dir not populated — git submodule update --init reference/$DUT_REF" >&2
    return 1; }
  [ -f "$dir/$DUT_KO" ] && return 0
  echo "building $DUT_REF (first run, a few minutes) ..."
  make -C "$dir" -j"$(nproc)" > "$OUT/vendor_build.log" 2>&1 || {
    echo "!! vendor build failed — see $OUT/vendor_build.log" >&2; return 1; }
  [ -f "$dir/$DUT_KO" ]
}

use_devourer() {
  sudo rmmod "$DUT_MOD" 2>/dev/null
  sudo modprobe -r "$INTREE_DRV" 2>/dev/null
  cold_cycle
  # rtw88 auto-probes on every enumeration; drop it again after the cycle.
  sudo modprobe -r "$INTREE_DRV" 2>/dev/null
}

use_vendor() {
  sudo modprobe -r "$INTREE_DRV" 2>/dev/null
  cold_cycle
  sudo modprobe -r "$INTREE_DRV" 2>/dev/null
  sudo insmod "$ROOT/reference/$DUT_REF/$DUT_KO" 2>/dev/null || true
  local u prev="" cur=""
  u=$(dut_usb) || return 1
  for _ in $(seq 30); do
    cur=$(basename "$(ls -d /sys/bus/usb/devices/$u/*/net/* 2>/dev/null | head -1)" 2>/dev/null)
    if [ -n "$cur" ] && [ "$cur" = "$prev" ] && [ -e "/sys/class/net/$cur" ]; then
      DUT_IF="$cur"
      sudo ip link set "$DUT_IF" down
      sudo iw dev "$DUT_IF" set type monitor
      sudo ip link set "$DUT_IF" up
      sudo iw dev "$DUT_IF" set channel "$CH"
      sleep 1
      return 0
    fi
    prev="$cur"; sleep 1
  done
  return 1
}

# $1 = side (devourer|vendor), $2 = mcs. Echoes "sent<TAB>rx<TAB>rates".
run_cell() {
  # Declare before use: within a single `local a=$1 b="${a}"`, bash marks every
  # name local first, so `${a}` there reads the (unset) variable, not $1.
  local side="$1" mcs="$2"
  local cap="$OUT/${side}_mcs${mcs}.txt" sent=0
  sudo timeout $((SECS + 6)) tcpdump -i "$SNIFF" -nn -e \
      "wlan addr2 $CANON" > "$cap" 2>/dev/null &
  sleep 1
  if [ "$side" = devourer ]; then
    sudo env DEVOURER_VID="0x$DUT_VID" DEVOURER_PID="0x$DUT_PID" \
      DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="MCS$mcs/20" \
      DEVOURER_TX_PAYLOAD_BYTES="$((PAYLOAD + DOT11_HDR))" \
      DEVOURER_TX_GAP_US="$TX_GAP_US" DEVOURER_LOG_LEVEL=warn \
      timeout --signal=TERM $((SECS + 10)) "$ROOT/build/txdemo" \
      > "$OUT/${side}_tx_mcs$mcs.jsonl" 2>"$OUT/${side}_tx_mcs$mcs.err"
    sent=$(grep -o '"submitted":[0-9]*' "$OUT/${side}_tx_mcs$mcs.jsonl" |
           tail -1 | cut -d: -f2)
  else
    sent=$(sudo timeout $((SECS + 6)) python3 "$HERE/kernel_tx_inject.py" \
           "$DUT_IF" "$mcs" "$PAYLOAD" "$SECS" 2>/dev/null |
           grep -oE "injected [0-9]+" | awk '{print $2}')
  fi
  sleep 2
  local n rates
  n=$(grep -c "$CANON" "$cap" 2>/dev/null || echo 0)
  rates=$(grep -oE "MCS [0-9]+" "$cap" 2>/dev/null | sort | uniq -c |
          tr -s ' ' | tr '\n' ' ')
  printf '%s\t%s\t%s\t%s\t%s\n' "$side" "$mcs" "${sent:-0}" "$n" "${rates:-none}"
}

emit() {
  local row=$1
  # A cell that produced no row at all is a harness failure, not a zero result —
  # say so rather than letting `read` leave the fields unset under `set -u`.
  if [ -z "$row" ]; then
    echo "  !! cell produced no result row (TX never started?) — INVALID" >&2
    return 1
  fi
  local side mcs sent rx rates
  IFS=$'\t' read -r side mcs sent rx rates <<<"$row"
  local pct="n/a"
  [ "${sent:-0}" -gt 0 ] 2>/dev/null && \
    pct=$(awk -v r="$rx" -v s="$sent" 'BEGIN{printf "%.1f%%", 100*r/s}')
  printf '  %-8s MCS%-2s sent=%-7s rx=%-7s %-8s %s\n' \
      "$side" "$mcs" "$sent" "$rx" "$pct" "$rates"
  printf '%s\n' "$row" >> "$RESULTS"
}

build_vendor || exit 1

echo
echo "== interleaved A/B, ch$CH, PSDU $((PAYLOAD + DOT11_HDR)) B, ${SECS}s/cell"
for m in $SANITY_RATE $RATES; do
  use_devourer  || { echo "  devourer half: DUT unavailable" >&2; exit 1; }
  emit "$(run_cell devourer "$m")"
  use_vendor    || { echo "  vendor half: no netdev — INVALID" >&2; exit 1; }
  emit "$(run_cell vendor "$m")"
done

echo
python3 - "$RESULTS" "$SANITY_RATE" <<'EOF'
import sys
rows = [l.rstrip("\n").split("\t") for l in open(sys.argv[1]) if l.strip()]
sanity = sys.argv[2]
by = {(s, m): (int(sent or 0), int(rx or 0)) for s, m, sent, rx, _ in rows}

# A side whose robust sanity rate delivered nothing was not transmitting; its
# zeros at every other rate mean nothing and must not be read as a result.
bad = [s for s in ("devourer", "vendor")
       if by.get((s, sanity), (0, 0))[1] == 0]
if bad:
    print("!! %s delivered nothing at the MCS%s sanity rate — that half was not "
          "airing. Every zero it reported is INVALID, not a measurement. "
          "Cold-cycle and rerun." % (" and ".join(bad), sanity))
    raise SystemExit(1)

print("%-6s %12s %12s   %s" % ("rate", "devourer", "vendor", "verdict"))
for _, m, _, _, _ in rows:
    pass
seen = []
for s, m, sent, rx, _ in rows:
    if m not in seen:
        seen.append(m)
for m in seen:
    d_sent, d_rx = by.get(("devourer", m), (0, 0))
    v_sent, v_rx = by.get(("vendor", m), (0, 0))
    d = 100.0 * d_rx / d_sent if d_sent else 0.0
    v = 100.0 * v_rx / v_sent if v_sent else 0.0
    if d_rx == 0 and v_rx == 0:
        verdict = "both dead — rig limit, not a devourer defect"
    elif d_rx == 0 and v_rx > 0:
        verdict = "*** DEVOURER DEFECT: vendor delivers, devourer does not ***"
    elif v_rx == 0 and d_rx > 0:
        verdict = "devourer delivers, vendor does not"
    else:
        verdict = "both deliver"
    print("MCS%-3s %11.1f%% %11.1f%%   %s" % (m, d, v, verdict))
EOF
echo
echo "logs: $OUT"
