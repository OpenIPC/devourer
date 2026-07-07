#!/usr/bin/env bash
# On-air chip↔chip fused-FEC gain on real Realtek silicon.
#
# devourer TX (8812) → devourer RX (8821, DEVOURER_RX_KEEP_CORRUPTED=1). The TX
# flies at a FRAGILE high MCS (64-QAM, no LDPC): the robust BPSK preamble/SIG
# keep each frame DETECTED and surfaced while the 64-QAM data fails the FCS, so
# a fraction of frames arrive "received-but-corrupt" — exactly the salvage
# regime. At MCS7, inches apart, ~10-15% of frames are corrupt-but-received with
# NO interferer at all; the RX salvages the CRC-valid sub-blocks and the SBI
# decoder recovers RS blocks the drop-whole-frame baseline throws away.
#
# IMPORTANT recipe notes (learned on the bench):
#  * Use NORMAL TX power. Low power (DEVOURER_TX_POWER small) doesn't make frames
#    "partially corrupt" — it kills DETECTION (preamble too weak), so the RX gets
#    ZERO frames. Power controls reception; MCS (and the optional interferer)
#    control data corruption.
#  * Capture-then-offline-analyse (not a live pipe): rxdemo → raw log,
#    then fused_fec_rx reads the log. Robust + re-analysable with different FEC
#    params.
#  * Optional USRP B210 interferer (USE_INTERFERER=1, IGAIN dB) raises the noise
#    floor to BOOST and make the corrupt rate reproducible. It needs ~8-10 s of
#    FPGA-load warmup before it actually radiates.
#
# ch6 (2.4 GHz) avoids the USB Vbus-sag gotcha. SKIP_RAIL=1 after a fresh boot.
# Run: sudo bash tests/fused_fec_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FEC="$ROOT/tools/precoder"
PY="${PY:-python3}"

TX_PID=${TX_PID:-0x8812}; TX_VID=${TX_VID:-0x0bda}; TX=${TX_SYSFS:-9-2}
RX_PID=${RX_PID:-0x0120}; RX_VID=${RX_VID:-0x2357}; RX=${RX_SYSFS:-9-1.4}
CH=${CH:-6}
HUBS=${HUBS:-"9 2;9-1 4"}                 # ';'-sep "hub port" for the rail cycle
MODULES=${MODULES:-"rtw88_8812au rtw88_8821au"}
TX_RATE=${TX_RATE:-MCS7}                  # fragile 64-QAM → natural corruption
TX_POWER=${TX_POWER:-}                    # EMPTY = chip default (normal). Do NOT lower.
TX_PWR_OVERRIDE=${TX_PWR_OVERRIDE:-}      # absolute TXAGC 0..63 (rarely needed)
# One RS block per frame (N=10): a single FCS failure is a whole-block loss for
# the baseline but recoverable by SBI when ≤2 of its 10 sub-blocks are bad.
# Raise --overhead for more salvage headroom (recovers more corrupt frames).
FEC_ARGS=${FEC_ARGS:-"--symbol-size 32 --overhead 0.25 --blocks-per-body 10 --k 8"}
SECS=${SECS:-15}                          # TX window
USE_INTERFERER=${USE_INTERFERER:-}        # set to 1 to add the B210 noise booster
IGAIN=${IGAIN:-80}                        # B210 TX gain dB when USE_INTERFERER=1
IMODE=${IMODE:-noise}                     # noise | cw

RAW=/tmp/fused_fec_rx_raw.log

KILL(){ sudo pkill -9 streamtx 2>/dev/null; sudo pkill -9 rxdemo 2>/dev/null
        sudo pkill -9 -f fused_fec 2>/dev/null; sudo pkill -9 -f sdr_interferer 2>/dev/null; }
trap KILL EXIT

# Fresh rail. SKIP_RAIL=1 bypasses the uhubctl power-cycle — REQUIRED on adapters
# on a root-hub port (e.g. 8812 on root hub 9), where `uhubctl off` drops the
# device but power-restore does NOT re-enumerate it (wedges until a host reboot).
if [ -z "${SKIP_RAIL:-}" ]; then
  echo "=== fresh rail (power-cycle Wi-Fi hub tree; B210 port untouched) ==="
  echo "    WARNING: on a root-hub port this can wedge the adapter; use SKIP_RAIL=1 post-reboot"
  sudo modprobe -r $MODULES 2>/dev/null
  IFS=';' read -ra _hubs <<< "$HUBS"
  for hp in "${_hubs[@]}"; do sudo uhubctl -a off -l ${hp% *} -p ${hp#* } >/dev/null 2>&1; done; sleep 16
  for hp in "${_hubs[@]}"; do sudo uhubctl -a on  -l ${hp% *} -p ${hp#* } >/dev/null 2>&1; done; sleep 9
  sudo modprobe $MODULES 2>/dev/null; sleep 2
else
  echo "=== SKIP_RAIL set — using current rail as-is (assumes a recent clean boot) ==="
fi

# free both Wi-Fi adapters from the kernel driver (the B210 is uhd-accessed)
for D in "$TX" "$RX"; do
  for i in /sys/bus/usb/devices/$D/$D:*; do
    ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
    [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
  done
done; sleep 1

head -c 400000 /dev/urandom > /tmp/fused_fec_src.bin

# Optional interferer — start it FIRST and let the FPGA load before TX.
if [ -n "$USE_INTERFERER" ]; then
  echo "=== B210 interferer: ch$CH gain=${IGAIN}dB mode=$IMODE (warming up FPGA) ==="
  sudo $PY "$ROOT/tests/sdr_interferer.py" --channel $CH --tx-gain "$IGAIN" \
       --rate 20e6 --mode "$IMODE" --secs $((SECS + 40)) >/tmp/intf.log 2>&1 &
  sleep 11
fi

# RX: capture the 8821's surfaced frames (background, raw — robust vs a live pipe).
: > "$RAW"
sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_STREAM_OUT=1 DEVOURER_RX_KEEP_CORRUPTED=1 \
     timeout $((SECS + 30)) "$ROOT/build/rxdemo" >"$RAW" 2>/dev/null &
RXBG=$!
sleep 8   # RX init (open + reset + monitor up)

# TX: 8812 at the fragile MCS, NORMAL power, for the window.
echo "=== TX (8812 $TX_RATE${TX_POWER:+ pwr=$TX_POWER}) for ${SECS}s ==="
pwr_env=""
[ -n "$TX_POWER" ]        && pwr_env="$pwr_env DEVOURER_TX_POWER=$TX_POWER"
[ -n "$TX_PWR_OVERRIDE" ] && pwr_env="$pwr_env DEVOURER_TX_PWR_OVERRIDE=$TX_PWR_OVERRIDE"
$PY "$FEC/fused_fec_tx.py" --input /tmp/fused_fec_src.bin --repeat 4 $FEC_ARGS 2>/dev/null \
  | sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
         DEVOURER_TX_RATE=$TX_RATE $pwr_env \
         timeout "$SECS" "$ROOT/build/streamtx" >/dev/null 2>&1

sleep 3
sudo pkill -9 -f sdr_interferer 2>/dev/null
sudo pkill -9 rxdemo 2>/dev/null
wait "$RXBG" 2>/dev/null

# Offline analysis: same FEC params, baseline vs SBI.
echo "=== fused-FEC result (offline analysis of the on-air capture) ==="
grep -F '"ev":"rx.frame"' "$RAW" | $PY "$FEC/fused_fec_rx.py" $FEC_ARGS >/dev/null 2>/tmp/fused_gain.log
grep "fused_fec_rx" /tmp/fused_gain.log || echo "  no frames captured — check TX rate/power and that both chips are unbound"
echo "=== raw capture at $RAW (re-analyse with other FEC params if desired) ==="
