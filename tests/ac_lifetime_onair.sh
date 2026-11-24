#!/usr/bin/env bash
# ac_lifetime_onair.sh — does DeviceConfig::Tx::ac_lifetime_us bound how old a
# retrying DATA frame can get on air? (AdapterCaps::tx_ac_lifetime_ok)
#
# The DUT sends unicast QoS-Data to an address nobody owns: every frame is
# heard and never ACKed, so each one runs its whole retry chain. A witness on
# the SAME host logs every copy it decodes (rx.seq, per-frame counter). A
# frame's age is the witness arrival of its LAST copy minus its submit time,
# both on the host monotonic clock, so no clock is shared with the air.
#
# Arms (ARMS, in order):
#   off          knobs off: the control
#   <us>         data_on_be + be_fast_retry + ac_lifetime_us=<us>, InitWrite()
#   init:<us>    the same, brought up through Init() (DEVOURER_TX_VIA_INIT)
#   init:off     knobs off, brought up through Init()
#
# The control is what makes a bounded arm mean anything. If the knobs-off arm
# is not clearly older than the lifetime, the peer is ACKing or the retry
# chain is short, and a "bounded" arm proves nothing: that is an ABORT (exit
# 2), not a pass. A lifetime arm passes when its median age sits at the
# lifetime and its p99 stays within TOL_MS of it. The verdict uses p99, not
# the max: one late copy in 5000 is the witness host's scheduling, and the
# max is printed beside it.
#
# What it does not measure: throughput. The lifetime drops frames, so a
# saturated link can deliver less with it on (8-12 % at MCS2 on the Jaguar2,
# Jaguar3 and RTL8733B parts measured); that is a separate bench
# (tests/bench_onair.py).
#
# The init: arms are for Jaguar1/2/3: the RTL8733B and Kestrel cannot send
# after Init(), so leave them out of ARMS there.
# BE_FAST_RETRY=0 for Jaguar1, which ignores that knob and says so.
#
#   sudo bash tests/ac_lifetime_onair.sh
#   DUT_PID=0x8812 WIT_PID=0xc85a BE_FAST_RETRY=0 sudo -E bash tests/ac_lifetime_onair.sh
#   ARMS="off 68000 off" FRAMES=2000 sudo -E bash tests/ac_lifetime_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD=${BUILD:-$ROOT/build}

DUT_VID=${DUT_VID:-0x0bda}; DUT_PID=${DUT_PID:-0xc812}   # RTL8812CU
WIT_VID=${WIT_VID:-0x2357}; WIT_PID=${WIT_PID:-0x012d}   # Archer T3U (8812BU)
CH=${CH:-149}
ARMS=${ARMS:-"off 68000 20000 init:68000 init:off off"}
FRAMES=${FRAMES:-5000}; GAP_US=${GAP_US:-2000}
RATE=${RATE:-MCS4}; PAYLOAD=${PAYLOAD:-1400}; RETRY=${RETRY:-16}
PWR_QDB=${PWR_QDB:--40}       # near field: keep the witness out of saturation
BE_FAST_RETRY=${BE_FAST_RETRY:-1}
INIT_WAIT_MS=${INIT_WAIT_MS:-8000}
ARM_TIMEOUT=${ARM_TIMEOUT:-60} # an unbounded control never finishes its frames
TOL_MS=${TOL_MS:-15}
RA=${RA:-02:12:34:56:78:9a}   # nobody owns it
if [ -z "${TX_SA:-}" ]; then  # run-unique, so ambient traffic is excluded
  printf -v TX_SA '02:61:63:%02x:%02x:%02x' \
    $((($$ >> 16) & 255)) $((($$ >> 8) & 255)) $(($$ & 255))
fi
OUT=${OUT:-/tmp/ac_lifetime}
BLACKLIST=/run/modprobe.d/zz-temp-blacklist-ac-lifetime-$$.conf
MODS="rtw88_8812au rtw88_8821au rtw88_8814au rtw88_8822bu rtw88_8821cu
      rtw88_8822cu rtw88_8733bu rtw89_8852bu rtw89_8852cu"

for b in txdemo rxdemo; do
  [ -x "$BUILD/$b" ] || { echo "[aclife] no $BUILD/$b - build first"; exit 2; }
done
rm -rf "${OUT:?}"; mkdir -p "$OUT"

# WIT_P / TX_P are the demos themselves (their output goes to the stamper
# through a process substitution, not a pipeline), so cleanup signals exactly
# what this run started and wait returns the transmitter's own status.
WIT_P=""; TX_P=""; BL_MINE=0
cleanup() {
  trap - EXIT INT TERM
  for p in "$TX_P" "$WIT_P"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  wait 2>/dev/null
  [ "$BL_MINE" = 1 ] && rm -f "$BLACKLIST"
}
trap cleanup EXIT INT TERM

# The in-tree drivers re-probe a dongle at every enumeration; modprobe -r alone
# does not survive that.
mkdir -p "$(dirname "$BLACKLIST")"
[ -e "$BLACKLIST" ] && { echo "[aclife] $BLACKLIST exists - not mine"; exit 2; }
: >"$BLACKLIST"; BL_MINE=1
for m in $MODS; do
  echo "blacklist $m" >>"$BLACKLIST"; modprobe -r "$m" 2>/dev/null
done

now_ns() { python3 -c 'import time; print(time.monotonic_ns())'; }
# Prefix each line with the host monotonic time it arrived at.
stamp() { python3 -u -c '
import sys, time
for l in sys.stdin:
    sys.stdout.write("%d %s" % (time.monotonic_ns(), l))
'; }

env DEVOURER_VID="$WIT_VID" DEVOURER_PID="$WIT_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA="$TX_SA" \
    DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout \
    "$BUILD/rxdemo" 2>"$OUT/wit.err" > >(stamp >"$OUT/wit.jsonl") &
WIT_P=$!
sleep 8
kill -0 "$WIT_P" 2>/dev/null || {
  echo "[aclife] PREFLIGHT FAIL: the witness did not come up"
  tail -5 "$OUT/wit.err"; exit 2; }

n=0
for arm in $ARMS; do
  n=$((n + 1)); knobs=(); life=${arm#init:}
  case $arm in init:*) knobs+=(DEVOURER_TX_VIA_INIT="$INIT_WAIT_MS") ;; esac
  case $life in
    off) ;;
    ''|*[!0-9]*) echo "[aclife] bad arm '$arm'"; exit 2 ;;
    *) knobs+=(DEVOURER_TX_DATA_BE=1 DEVOURER_TX_AC_LIFETIME_US="$life")
       [ "$BE_FAST_RETRY" = 1 ] && knobs+=(DEVOURER_TX_BE_FAST_RETRY=1) ;;
  esac
  f="$OUT/tx-$n"
  echo "$n $arm $(now_ns)" >>"$OUT/marks"
  # The four knobs under test are unset first: a caller's environment
  # (sudo -E) must not turn a control into a knob arm or pick the bring-up.
  env -u DEVOURER_TX_DATA_BE -u DEVOURER_TX_BE_FAST_RETRY \
      -u DEVOURER_TX_AC_LIFETIME_US -u DEVOURER_TX_VIA_INIT \
      DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" DEVOURER_CHANNEL="$CH" \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$RA" DEVOURER_TX_SA="$TX_SA" \
      DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PAYLOAD_BYTES="$PAYLOAD" \
      DEVOURER_TX_GAP_US="$GAP_US" DEVOURER_TX_FRAMES="$FRAMES" \
      DEVOURER_TX_RETRY_LIMIT="$RETRY" DEVOURER_TX_PWR_OFFSET_QDB="$PWR_QDB" \
      DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout "${knobs[@]}" \
      timeout -s INT "$ARM_TIMEOUT" "$BUILD/txdemo" 2>"$f.err" \
      > >(stamp >"$f.jsonl") &
  TX_P=$!
  if [ "$n" = 1 ]; then
    # Fail in seconds, not after every arm: is the witness hearing the DUT?
    sleep 20
    hits=$(grep -c '"ev":"rx.seq"' "$OUT/wit.jsonl")
    [ "$hits" -gt 50 ] || {
      echo "[aclife] PREFLIGHT FAIL: the witness decoded $hits frames in 20 s"
      tail -5 "$f.err" "$OUT/wit.err"; exit 2; }
    echo "[aclife] preflight ok: $hits witness frames"
  fi
  wait "$TX_P"; echo $? >"$f.rc"; TX_P=""   # 124 = ARM_TIMEOUT cut it short
  grep -hE "not applied|does not reach|VIA_INIT: Init" "$f.err" | head -4
  sleep 3   # let the queue's tail air and the witness flush
done
echo "end end $(now_ns)" >>"$OUT/marks"
kill -INT "$WIT_P" 2>/dev/null; wait "$WIT_P" 2>/dev/null; WIT_P=""
sleep 1   # the stampers drain after their demo exits
python3 "$ROOT/tests/ac_lifetime_analyze.py" "$OUT" --tol-ms "$TOL_MS" \
    --frames "$FRAMES"
