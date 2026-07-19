#!/usr/bin/env bash
# On-air A/B for the adaptive link's adjacent-MCS probes: VTX (8812) <-> VRX
# (8821), both running duplex driven by adaptive_link.py, same rig as
# tests/adaptive_onair.sh. Four paired cells on one channel:
#
#   A  probe=off  unbiased    — baseline; discovers the settled MCS M
#   B  probe=off  bias (M+1)  — the model-only controller believes M+1 works
#                               at 12 dB less SNR than it does and promotes
#                               into it (the miscalibration trap, on-air)
#   C  probe=on   bias (M+1)  — probe evidence gates/blocks the same lie
#   D  probe=on   unbiased    — probe cost vs A (duty, operating point)
#
# The mismatch needs the upper rates to GENUINELY fail on this channel, which
# near-field bench SNR does not give — so the B210 interferer runs by default
# (USE_INTERFERER=0 disables; without it the harness still validates the
# plumbing end-to-end: probes fly, stats accrue, no baseline regression, but
# the B-vs-C delivery contrast may not appear). The interferer's effective
# bite sits on a ~2 dB cliff and drifts between cells/runs, so judge within
# one run and lean on the SAME-BIAS pair: B vs C isolates the probes as the
# only difference. frames/win is the health signal; deliv= is seq-based and
# lies under crc floods.
#
#   sudo bash tests/mcs_probe_onair.sh
#   USE_INTERFERER=0 sudo bash tests/mcs_probe_onair.sh   # plumbing only
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREC="$ROOT/tools/precoder"

VTX_PID=${VTX_PID:-0x8812}; VTX_VID=${VTX_VID:-0x0bda}; VTX=${VTX_SYSFS:-9-2}
VRX_PID=${VRX_PID:-0x0120}; VRX_VID=${VRX_VID:-0x2357}; VRX=${VRX_SYSFS:-9-1.4}
CH=${CH:-6}; SECS=${SECS:-40}; VTX_ID=${VTX_ID:-0xABCD}
USE_INTERFERER=${USE_INTERFERER:-1}; IGAIN=${IGAIN:-75}; IMODE=${IMODE:-noise}
# Keep the host otherwise idle: B210 TX underruns (host-load-starved) turn the
# interferer into chaotic bursts and the cells stop being comparable.
IRATE=${IRATE:-20e6}
# Mild optimism: a few dB is the realistic miscalibration AND keeps the biased
# row out of the (deliberately ungated) cold pick — the trap the probes exist
# for is the PROMOTION into the lie, not a wrong first guess.
BIAS_DB=${BIAS_DB:--4}
LOGDIR=${LOGDIR:-/tmp/mcs-probe-onair}
VIDEO=$LOGDIR/video.bin

KILL(){ sudo pkill -9 -f adaptive_link 2>/dev/null; sudo pkill -9 duplex 2>/dev/null
        sudo pkill -9 -f sdr_interferer 2>/dev/null; }
trap KILL EXIT

mkdir -p "$LOGDIR"
head -c 4000000 /dev/urandom > "$VIDEO"

# free both Wi-Fi adapters (B210 is uhd-accessed)
for D in "$VTX" "$VRX"; do
  for i in /sys/bus/usb/devices/$D/$D:*; do
    ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
    [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
  done
done; sleep 1

# ---- one cell: [interferer] + VRX + VTX for SECS, logs under $LOGDIR/<name>
run_cell(){ # name probe_flag bias_spec
  local name=$1 probe=$2 bias=$3
  local vrx_log=$LOGDIR/${name}_vrx.log vtx_log=$LOGDIR/${name}_vtx.log
  local vrx_args=(--role vrx --pid $VRX_PID --channel $CH --vtx-id $VTX_ID
                  --duplex "$ROOT/build/duplex")
  local vtx_args=(--role vtx --pid $VTX_PID --channel $CH --vtx-id $VTX_ID
                  --video "$VIDEO" --duplex "$ROOT/build/duplex")
  [ "$probe" = 1 ] && { vrx_args+=(--mcs-probe); vtx_args+=(--mcs-probe); }
  [ -n "$bias" ] && vrx_args+=(--mcs-bias "$bias")

  KILL; sleep 1
  if [ "$USE_INTERFERER" = 1 ]; then
    sudo python3 "$ROOT/tests/sdr_interferer.py" --channel $CH --tx-gain "$IGAIN" \
         --rate "$IRATE" --mode "$IMODE" --secs $((SECS + 30)) \
         >"$LOGDIR/${name}_intf.log" 2>&1 &
    sleep 11
  fi
  echo "=== cell $name: probe=$probe bias='${bias:-none}' ==="
  sudo env DEVOURER_VID=$VRX_VID DEVOURER_PID=$VRX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" "${vrx_args[@]}" >"$vrx_log" 2>&1 &
  sleep 6
  sudo env DEVOURER_VID=$VTX_VID DEVOURER_PID=$VTX_PID PYTHONPATH="$PREC" \
       python3 "$PREC/adaptive_link.py" "${vtx_args[@]}" >"$vtx_log" 2>&1 &
  sleep "$SECS"
  KILL; sleep 1
  grep '<adaptive-vrx>' "$vrx_log" | tail -1
}

# deliv= is only meaningful on populated windows: near-empty windows read
# 1.000 (no gaps to see) and crc-flooded ones read ~0 (corrupt bodies carry
# garbage seqs) — so gate on frames>=20 and report the window fill too.
cell_deliv(){ grep -oP '<adaptive-vrx>.*deliv=\K[0-9.]+(?=.*frames=(2[0-9]|[3-9][0-9]|[0-9]{3,}))' \
              "$LOGDIR/${1}_vrx.log" | tail -20 | sort -n \
              | awk '{a[NR]=$1} END{print (NR ? a[int(NR/2)+1] : "dead")}'; }
cell_frames(){ grep -oP '<adaptive-vrx>.*frames=\K\d+' "$LOGDIR/${1}_vrx.log" \
              | tail -20 | sort -n | awk '{a[NR]=$1} END{print (NR ? a[int(NR/2)+1] : 0)}'; }
cell_mcs(){ grep -oP '<adaptive-vrx>.*-> MCS\K\d+' "$LOGDIR/${1}_vrx.log" | tail -1; }
flew_mcs(){ grep -c "ladder=CRIT=MCS${2}/" "$LOGDIR/${1}_vtx.log" 2>/dev/null || true; }

run_cell A 0 ""
M=$(cell_mcs A)
if [ -z "$M" ]; then echo "FATAL: cell A produced no controller trajectory"; exit 1; fi
T=$((M + 1))
if [ "$T" -gt 7 ]; then
  echo "WARN: settled at MCS$M (top of the set) — no row above to bias;"
  echo "      raise IGAIN to pull the operating point down and re-run."
  T=7
fi
BIAS_SPEC="$T:$BIAS_DB"
echo "settled MCS$M -> biasing MCS$T by ${BIAS_DB} dB (model-optimistic)"

run_cell B 0 "$BIAS_SPEC"
run_cell C 1 "$BIAS_SPEC"
run_cell D 1 ""

echo
echo "=== RESULT (settled MCS$M, biased candidate MCS$T) ==="
printf "%-4s %-8s %-10s %-14s %-10s %-14s %s\n" cell probe bias "median-deliv" "frames/win" "flew-MCS$T(s)" "probe-evidence"
for c in A B C D; do
  case $c in A) p=off b=none;; B) p=off b=yes;; C) p=on b=yes;; D) p=on b=none;; esac
  pe=$(grep -c 'probe\[MCS' "$LOGDIR/${c}_vrx.log" 2>/dev/null || true)
  printf "%-4s %-8s %-10s %-14s %-10s %-14s %s\n" \
    "$c" "$p" "$b" "$(cell_deliv $c)" "$(cell_frames $c)" "$(flew_mcs $c $T)" "${pe} lines"
done
echo
echo "verdict guide:"
echo "  A vs B — the trap on-air: B flies MCS$T and its frames/win collapse."
echo "  C — the measured ESCAPE: on this static bench a bias attractive"
echo "      enough to matter is usually captured by the (model-trusted) cold"
echo "      pick, so C may fly MCS$T briefly — but its measured active-row"
echo "      delivery must condemn it within seconds (flew-MCS$T well below"
echo "      B's), delivery must recover toward A/D, and probe[..] evidence"
echo "      lines must be present (zero probe[MCS lines = dead rate feed)."
echo "  D — the promotion gate with real evidence: D must not fly a"
echo "      candidate whose measured lcb is low."
echo "  (The promotion gate + escape are also pinned deterministically by"
echo "   mcs_probe_ab_sim.py, incl. the no-acquisition 'ambush' scenario.)"
echo "logs: $LOGDIR"
