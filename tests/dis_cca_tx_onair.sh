#!/usr/bin/env bash
# TX-SIDE test for the dis_cca / carrier-sense-disable knob (SetCcaMode,
# DEVOURER_DIS_CCA) — the TX-side experiment. The sibling
# tests/dis_cca_onair.sh measured a NULL, but it ran the DUT as an RX (decoding a
# beacon through B210 AWGN) and never exercised the TX side. Carrier sense gates TX
# *deferral*, not RX decode, so the real test drives the DUT as a TX injector and
# asks: does it defer to a busy channel, and does dis_cca let it punch through?
#
# METHOD — co-channel RTL flooder (not a B210). A second RTL adapter (default the
# 8812AU) floods the DUT's channel with real 802.11 frames; the DUT's primary CCA
# (carrier-sense of the decodable preamble) then defers its injection. We measure
# the DUT's tx.stats `submitted` rate over a fixed window:
#   alone            -> full rate (baseline)
#   co-chan + dis0   -> deferred (CCA holds the sync send back; lower submit rate)
#   co-chan + dis1   -> recovered if dis_cca disables the gate
#   FAR-chan + dis0  -> USB-hub-contention control (same USB load, no co-channel RF;
#                       both adapters share a bus, so this isolates the ~5% USB cost)
# The B210 AWGN path (tests/sdr_interferer.py) is NOT used here: on this rig it is
# too weakly coupled to the RTL front ends to trip a TX-deferring gate even at max
# gain (the #198 "near-field B210 couldn't bury the signal" limit). A co-channel
# RTL flooder couples strongly and is the reliable, rig-independent CCA trigger.
#
# FINDING (on-air, 8822EU): injection defers ~41-45% to a co-channel flooder (real
# RF, not USB — the far-channel control keeps the rate). The knob must disable
# BIT_DIS_CCA 0x520[14] (primary carrier-sense) to recover it (~1.5x, back to ~90%
# of the alone rate); BIT_DIS_EDCCA 0x520[15] alone (energy detect) is null against
# a decodable preamble. SetCcaMode disables both.
#
# Usage: sudo -v && tests/dis_cca_tx_onair.sh            # all configs
#        ONLY=eu5g REPS=3 sudo -v && tests/dis_cca_tx_onair.sh
#        ONLY=b7335g FLOOD_PID=0xb812 sudo -v && tests/dis_cca_tx_onair.sh   # RTL8733B, 8822BU flooder
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${DISCCATX_OUT:-/tmp/devourer-dis-cca-tx}"
REPS="${REPS:-2}"                 # repeats per arm (reproducibility)
DUR="${DUR:-10}"                  # injector TX window (s) per cell
INJ_PWR="${INJ_PWR:-20}"          # DUT TXAGC index
FLOOD_PWR="${FLOOD_PWR:-30}"      # flooder TXAGC index (occupy the channel)
mkdir -p "$OUT"

VID=0x0bda
EU_PID=0xa81a                     # RTL8812EU/8822EU — Jaguar3, the community chip
CU_PID=0xc812                     # RTL8812CU — Jaguar3 rtl8822c sibling
B733_PID=0xb733                   # RTL8733BU — HALMAC 87xx, same 0x520 gate map
FLOOD_PID="${FLOOD_PID:-0x8812}"  # RTL8812AU — universal co-channel flooder (dual-band); any dual-band TX works

flood_pid=""
cleanup() { [ -n "$flood_pid" ] && { kill "$flood_pid" 2>/dev/null; wait "$flood_pid" 2>/dev/null; }; true; }
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$VID" "$1")" >/dev/null 2>&1; }

# The in-tree rtw88 auto-probes the dongles on enumeration; unbind any it holds.
unbind_rtw88() {
    local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "${pid#0x}" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

# One DUT injection cell; echoes `submitted`. $1=dut_pid $2=ch $3=discca $4=tag
inject() {
    local dis=""; [ "$3" = 1 ] && dis="DEVOURER_DIS_CCA=1"
    sudo env DEVOURER_PID="$1" DEVOURER_VID="$VID" DEVOURER_CHANNEL="$2" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR="$INJ_PWR" \
        DEVOURER_TX_GAP_US=0 DEVOURER_TX_USB_AGG=0 $dis \
        timeout "$DUR" "$ROOT/build/txdemo" >"$OUT/$4.log" 2>/dev/null || true
    grep '"ev":"tx.stats"' "$OUT/$4.log" 2>/dev/null | tail -1 \
        | python3 -c 'import json,sys
try: print(json.loads(sys.stdin.read()).get("submitted",0))
except: print(0)'
}
flood_start() { # $1=ch  — start the co-channel/far flooder, set $flood_pid
    sudo env DEVOURER_PID="$FLOOD_PID" DEVOURER_VID="$VID" DEVOURER_CHANNEL="$1" \
        DEVOURER_TX_RATE=6M DEVOURER_TX_PWR="$FLOOD_PWR" DEVOURER_TX_GAP_US=0 \
        timeout $((DUR * REPS * 2 + 60)) "$ROOT/build/txdemo" >"$OUT/flood-$1.log" 2>/dev/null &
    flood_pid=$!
}
flood_stop() { [ -n "$flood_pid" ] && { kill "$flood_pid" 2>/dev/null; wait "$flood_pid" 2>/dev/null; }; flood_pid=""; }
mean() { python3 -c 'import sys; v=[int(x) for x in sys.argv[1:] if x]; print(round(sum(v)/len(v)) if v else 0)' "$@"; }

run_config() { # $1=dut_pid $2=ch $3=farch $4=label
    local dut="$1" ch="$2" farch="$3" label="$4" r
    echo
    echo "######## CONFIG $label  DUT=$dut ch=$ch (flood=$FLOOD_PID, far=$farch) ########"
    plugged "$dut" || { echo "SKIP $label: DUT $dut not plugged"; return; }
    plugged "$FLOOD_PID" || { echo "SKIP $label: flooder $FLOOD_PID not plugged"; return; }
    unbind_rtw88 "$dut"; unbind_rtw88 "$FLOOD_PID"

    # far-chan/dis1 is the USB-contention control that holds on a BUSY
    # channel too: with the gate off the injector ignores ambient, so the
    # only thing a far-channel flooder can still cost it is the host/USB
    # path. far-chan/dis0 folds ambient deferral into that number and is
    # kept for the idle-channel reading it was designed for.
    local A=() D0=() D1=() F0=() F1=()
    for r in $(seq 1 "$REPS"); do
        A+=("$(inject "$dut" "$ch" 0 "$label-alone-$r")")
        flood_start "$ch"; sleep 4
        D0+=("$(inject "$dut" "$ch" 0 "$label-co0-$r")")
        D1+=("$(inject "$dut" "$ch" 1 "$label-co1-$r")")
        flood_stop; sleep 1
        flood_start "$farch"; sleep 4
        F0+=("$(inject "$dut" "$ch" 0 "$label-far0-$r")")
        F1+=("$(inject "$dut" "$ch" 1 "$label-far1-$r")")
        flood_stop; sleep 1
    done
    local a d0 d1 f0 f1; a=$(mean "${A[@]}"); d0=$(mean "${D0[@]}"); d1=$(mean "${D1[@]}"); f0=$(mean "${F0[@]}"); f1=$(mean "${F1[@]}")
    printf "  alone=%s  co-chan/dis0=%s  co-chan/dis1=%s  far-chan/dis0=%s  far-chan/dis1=%s\n" "$a" "$d0" "$d1" "$f0" "$f1"
    printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$label" "$a" "$d0" "$d1" "$f0" "$f1" >>"$OUT/summary.tsv"
    python3 - "$a" "$d0" "$d1" "$f0" "$f1" "$label" <<'PYEOF'
import sys
a,d0,d1,f0,f1,label=int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]),int(sys.argv[5]),sys.argv[6]
if a==0: print("  (no baseline)"); sys.exit()
# "alone" is only a ceiling on an idle channel. On a channel with a resident
# network the gated injector defers to ambient and alone reads BELOW the
# gates-off co-channel cell; the gates-off rate is then the ceiling, and the
# USB term comes from the gates-off far-channel cell, which ambient cannot
# touch (measured: RTL8733B on a busy ch36, alone 3.0k vs dis1 7.2k fps,
# far/dis1 7.1k).
ceiling=a; usb_ref=f0
if d1 > a*1.1:
    ceiling=d1; usb_ref=f1
    print(f"  note: alone={a} < co-chan/dis1={d1} — 'alone' is not an idle baseline here (resident traffic); ceiling=dis1, USB term from far-chan/dis1={f1}")
defr=100*(1-d0/ceiling); usb=100*(1-usb_ref/ceiling); rec=(d1/d0) if d0 else 0
print(f"  co-chan deferral(dis0)={defr:+.0f}%  USB-only(far)={usb:+.0f}%  recovery dis1/dis0={rec:.2f}x  dis1 vs ceiling={100*(1-d1/ceiling):+.0f}%")
real = defr - usb   # deferral net of USB contention
if real >= 15 and d1 >= ceiling*0.85:
    print(f"  => {label}: dis_cca HELPS — real co-channel CCA deferral {real:.0f}% removed, TX back to ~the ceiling")
elif real >= 15 and rec >= 1.25:
    print(f"  => {label}: dis_cca HELPS (partial) — {real:.0f}% deferral, {rec:.2f}x recovery")
elif real < 8:
    print(f"  => {label}: no real co-channel deferral (mostly USB) — inconclusive on this pairing")
else:
    print(f"  => {label}: deferral present but dis_cca does NOT recover it")
PYEOF
}

echo "== building rxdemo + txdemo =="
cmake --build "$ROOT/build" -j --target rxdemo txdemo >/dev/null || exit 1
: >"$OUT/summary.tsv"

ONLY="${ONLY:-eu5g cu5g eu24g}"
for cfg in $ONLY; do
    case "$cfg" in
        eu5g)  run_config "$EU_PID" 36 149 "EU5G"  ;;
        cu5g)  run_config "$CU_PID" 36 149 "CU5G"  ;;
        eu24g) run_config "$EU_PID" 6  149 "EU24G" ;;   # 2.4 GHz DUT, 5 GHz far-control
        b7335g) run_config "$B733_PID" 36 149 "B733-5G" ;; # RTL8733B; INJ_PWR is inert there (flat index refused)
        *) echo "unknown config: $cfg" ;;
    esac
done
echo; echo "raw: $OUT/summary.tsv"
