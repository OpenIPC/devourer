#!/usr/bin/env bash
# Fair head-to-head: chip RX vs SDR RX on the SAME on-air TX session.
#
# One devourer 8812 transmits fused-FEC bodies (HT MCS7, marginal power) once;
# two receivers decode that SAME radiation simultaneously:
#   * chip RX  — devourer 8821 (DEVOURER_RX_KEEP_CORRUPTED), live <devourer-stream>
#   * SDR RX   — USRP B210, IQ recorded to disk (no live-decode overflow), then
#                replayed offline through the gr-ieee802-11 fork HARD and SOFT.
# Both feed the identical FusedFecReceiver (same FEC params) so "RS blocks /
# packets recovered" is directly comparable, and each receiver's own per-frame
# SNR is reported alongside.
#
# CAVEAT (no RF splitter): the chip and the B210 use separate antennas, so they
# do NOT see a bit-identical channel — only the same transmitter at the same
# instant. Compare the recovered payload AT EACH RECEIVER'S OWN measured SNR, not
# blindly head-to-head. A splitter feeding both RX from one antenna would remove
# this; lacking one, the per-receiver SNR columns are how you read the result.
#
# ch6 (2.4 GHz). 8812 on root hub 9 wedges on a uhubctl cycle, so SKIP_RAIL
# defaults on (assumes a recent clean boot). Run: sudo bash tests/fused_fec_headtohead.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FEC="$ROOT/tools/precoder"
# Resolve the INVOKING user's home — this script runs under `sudo bash`, so $HOME
# is /root and would mis-resolve the sister-repo / GNU Radio paths.
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
SDR="${SDR_DIR:-$REAL_HOME/git/sdr2wifi}"
PY="${PY:-python3}"
SKIP_SDR=${SKIP_SDR:-}                      # set to 1 for a fast chip-only SNR sweep

TX_PID=${TX_PID:-0x8812}; TX_VID=${TX_VID:-0x0bda}; TX=${TX_SYSFS:-9-2}
RX_PID=${RX_PID:-0x0120}; RX_VID=${RX_VID:-0x2357}; RX=${RX_SYSFS:-9-1.4}
CH=${CH:-6}; FREQ=${FREQ:-2437e6}
TX_RATE=${TX_RATE:-MCS7}
TX_PWR_OVERRIDE=${TX_PWR_OVERRIDE:-24}    # marginal 64-QAM waterfall (per earlier sweep)
RXGAIN=${RXGAIN:-20}                       # B210 RX gain dB
SECS=${SECS:-12}                           # TX window
RECSECS=${RECSECS:-7}                      # B210 capture (inside the TX window)
FEC_ARGS=${FEC_ARGS:-"--symbol-size 32 --overhead 0.25 --blocks-per-body 10 --k 8"}

RAW=/tmp/h2h_chip_raw.log
CAP=/tmp/h2h_sdr.cf32
BODIES=/tmp/h2h_bodies.bin

KILL(){ sudo pkill -9 StreamTxDem 2>/dev/null; sudo pkill -9 WiFiDriverDem 2>/dev/null
        sudo pkill -9 -f tools_record_iq 2>/dev/null; sudo pkill -9 -f fused_fec 2>/dev/null; }
trap KILL EXIT

# gr-ieee802-11 env for the offline SDR replay
export PREFIX="$REAL_HOME/grwifi-install"
OOT=$(find "$PREFIX" -name ieee802_11 -type d -path '*packages*' 2>/dev/null | head -1); OOT="${OOT%/ieee802_11}"
export PYTHONPATH="$OOT:$SDR"
GRL="$PREFIX/lib:$PREFIX/lib64"; for d in "$PREFIX"/lib/*-linux-gnu; do [ -d "$d" ] && GRL="$d:$GRL"; done
export LD_LIBRARY_PATH="$GRL:${LD_LIBRARY_PATH:-}"; ulimit -n 8192

if [ -z "${SKIP_RAIL:-}" ]; then
  echo "WARNING: rail power-cycle can wedge the 8812 on root hub 9; prefer SKIP_RAIL=1 post-reboot"
fi

# free both Wi-Fi adapters (B210 is uhd-accessed, leave it)
for D in "$TX" "$RX"; do
  for i in /sys/bus/usb/devices/$D/$D:*; do
    ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
    [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
  done
done; sleep 1

# Fresh bodies (root-owned /tmp leftovers from prior sudo runs break the redirect)
sudo rm -f "$BODIES" "$CAP" "$RAW" 2>/dev/null
head -c 400000 /dev/urandom > /tmp/h2h_src.bin
$PY "$FEC/fused_fec_tx.py" --input /tmp/h2h_src.bin --repeat 200 $FEC_ARGS \
    > "$BODIES" 2>/dev/null
echo "[h2h] TX bodies: $(wc -c <"$BODIES") bytes  rate=$TX_RATE pwr=$TX_PWR_OVERRIDE"

# --- chip RX (8821): capture <devourer-stream> for the whole window ---
: > "$RAW"
sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_STREAM_OUT=1 DEVOURER_RX_KEEP_CORRUPTED=1 \
     timeout $((SECS + 25)) "$ROOT/build/WiFiDriverDemo" >"$RAW" 2>/dev/null &
sleep 8   # chip init (open + reset + monitor up)

# --- TX (8812): marginal MCS7, background, for the window ---
echo "[h2h] TX + simultaneous chip-RX & B210-record ..."
sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_RATE=$TX_RATE DEVOURER_TX_PWR_OVERRIDE=$TX_PWR_OVERRIDE \
     timeout "$SECS" "$ROOT/build/StreamTxDemo" < "$BODIES" >/tmp/h2h_tx.log 2>&1 &
sleep 2   # let TX ramp

# --- SDR RX (B210): record IQ mid-window (foreground; both others are live) ---
if [ -z "$SKIP_SDR" ]; then
  sudo -E $PY "$SDR/tools_record_iq.py" --freq "$FREQ" --bw 20e6 --rx-gain "$RXGAIN" \
       --secs "$RECSECS" --out "$CAP" 2>&1 | tail -1
  sudo chown "$USER" "$CAP" 2>/dev/null
else
  sleep "$RECSECS"
fi

# let TX finish, then stop the chip RX
sleep $((SECS)); sudo pkill -9 StreamTxDem 2>/dev/null; sudo pkill -9 WiFiDriverDem 2>/dev/null
sleep 1

# ===================== offline analysis (same FEC params) =====================
echo
echo "=== CHIP RX (8821) ==="
grep "<devourer-stream>" "$RAW" | $PY "$FEC/fused_fec_rx.py" $FEC_ARGS \
    >/dev/null 2>/tmp/h2h_chip.out
grep "fused_fec_rx" /tmp/h2h_chip.out || echo "  (no chip frames — check rate/power/unbind)"
# chip per-frame SNR split by FCS pass/fail
$PY - "$RAW" <<'PYEOF'
import re,sys,statistics
clean=[];corr=[]
rx=re.compile(r"<devourer-stream>.*?crc_err=(\d+).*?snr=(-?\d+),")
for line in open(sys.argv[1],errors="ignore"):
    m=rx.search(line)
    if not m: continue
    (corr if int(m.group(1)) else clean).append(int(m.group(2)))
def med(x): return f"{statistics.median(x):.0f}" if x else "-"
print(f"  chip SNR(dB) median: clean={med(clean)} (n={len(clean)})  "
      f"corrupt={med(corr)} (n={len(corr)})")
PYEOF

if [ -z "$SKIP_SDR" ]; then
  echo
  echo "=== SDR RX (B210) — HARD Viterbi ==="
  timeout 200 $PY "$SDR/iq_fec_diag.py" "$CAP" 20e6 2>/dev/null | grep -E "fec-diag|RESULT"
  echo
  echo "=== SDR RX (B210) — SOFT Viterbi (GR_SOFT_VITERBI) ==="
  GR_SOFT_VITERBI=1 timeout 200 $PY "$SDR/iq_fec_diag.py" "$CAP" 20e6 2>/dev/null | grep -E "fec-diag|RESULT"
fi

echo
echo "=== head-to-head (recovered RS blocks / packets; read at each RX's own SNR) ==="
echo "  chip  : $(grep -h 'baseline\|sbi ' /tmp/h2h_chip.out | tr '\n' ' ')"
echo "  raw chip log: $RAW   SDR capture: $CAP"
