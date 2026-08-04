#!/usr/bin/env bash
#
# arq_e2e_delivery.sh — does a hardware-ACKed frame actually reach the RX app?
#
# Field report (PixelPilot, 8812AU drone -> 8812EU ground): with hardware ARQ
# (DEVOURER_TX_RETRY_LIMIT + DEVOURER_ACK_RESPONDER) the drone's tx.report says
# a packet was delivered after a retry, yet the ground app never receives it —
# "ACKed-but-undelivered". The existing ACK harnesses judge the loop from the
# TX side's CCX reports only, so this failure was structurally invisible.
#
# This bench closes the loop with three per-frame ledgers on one channel:
#   DUT     8812EU (J3), examples/duplex — the PixelPilot one-handle topology:
#           RX loop + stdin-fed uplink bursts, ACK responder armed, host
#           delivery ledger = rx.seq (pctr), ring telemetry = rx.ring.
#   DRONE   8812CU (J3), txdemo — unicast QoS-Data at the responder MAC,
#           normal ack-policy, retrying descriptor, per-frame tx.report.
#           The halmac report tag echoes the frame counter (mod 256), which
#           IS the payload pctr, so reports join to payloads exactly.
#   WITNESS 8814AU (J1), rxdemo — passive monitor with the same rx.seq ledger:
#           what a quiet, healthy receiver decodes on this channel. A pctr the
#           witness has and the DUT lacks is a loss INSIDE the DUT chain.
#
# The DUT's uplink alternates phases inside ONE session (idle control /
# PixelPilot burst shape / 3x burst), marked by stream.ctl events in file
# order. The drone-sim defaults to DEVOURER_DIS_CCA=1: in the field the drone
# is effectively a hidden node to the ground's low-power feedback, so it
# transmits INTO the bursts; near-field CSMA would defer instead and suppress
# the collision population under test.
#
#   sudo bash tests/arq_e2e_delivery.sh
#   RETRY_LIMIT=8 CYCLES=3 CH=36 sudo bash tests/arq_e2e_delivery.sh
#   DUT_RX_MODE=spsc-fat sudo bash tests/arq_e2e_delivery.sh   # mitigation arm
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD=${BUILD:-$ROOT/build}

DUT_VID=${DUT_VID:-0x0bda};   DUT_PID=${DUT_PID:-0xa81a}    # RTL8812EU (J3)
DRONE_VID=${DRONE_VID:-0x0bda}; DRONE_PID=${DRONE_PID:-0xc812} # RTL8812CU (J3)
WIT_VID=${WIT_VID:-0x0bda};   WIT_PID=${WIT_PID:-0x8813}    # RTL8814AU (J1)

CH=${CH:-36}
MAC1=${MAC1:-02:12:34:56:78:9a}        # DUT responder identity = drone RA
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}      # drone TA (unicast — the I/G footgun)
RETRY_LIMIT=${RETRY_LIMIT:-3}          # field report used 3
DRONE_REPORT_N=${DRONE_REPORT_N:-1}    # CCX sampling divisor (1 = every frame)
DRONE_FALLBACK=${DRONE_FALLBACK:-}     # retry rate fallback: off | "" (=fw ladder)
DRONE_AMPDU=${DRONE_AMPDU:-}           # A-MPDU mode spec "tid/max[...]" (off="")
DRONE_BATCH=${DRONE_BATCH:-}           # send_packets batch depth (queue feed —
                                       # aggregation only forms with frames
                                       # queued back-to-back; rate = batch/gap)
RECEIPT_MS=${RECEIPT_MS:-}             # windowed RX receipts cadence (off="")
DRONE_RATE=${DRONE_RATE:-MCS3}
DRONE_PAYLOAD=${DRONE_PAYLOAD:-512}    # >= 30 so the pctr stamp fits
DRONE_GAP_US=${DRONE_GAP_US:-1000}     # ~1k fps video-sim
DRONE_DIS_CCA=${DRONE_DIS_CCA:-1}
PWR_QDB=${PWR_QDB:--40}                # de-saturate the near field (-10 dB)

CYCLES=${CYCLES:-5}
PHASE_S=${PHASE_S:-8}
PHASES=${PHASES:-"6M:10,idle,6M:30,idle"}  # burst first: preflight sees TX
PERIOD_MS=${PERIOD_MS:-100}            # PixelPilot adaptive-link period
PSDU=${PSDU:-100}                      # PixelPilot uplink PSDU
WARMUP_S=${WARMUP_S:-12}               # feeder idle lead-in (duplex bring-up)
PREFLIGHT_S=${PREFLIGHT_S:-14}         # liveness check delay after drone start
DUT_RX_MODE=${DUT_RX_MODE:-}           # empty = default async ring
DUT_POOL_SPARE=${DUT_POOL_SPARE:-16}   # only read by the pool modes
DUT_POOL_EXHAUST=${DUT_POOL_EXHAUST:-} # spsc-fat: backpressure (default) | drop
DUT_SPIN_US=${DUT_SPIN_US:-0}          # per-frame inline consumer cost model
DUT_STALL_MS=${DUT_STALL_MS:-0}        # periodic consumer hiccup (GC pause)
DUT_STALL_EVERY=${DUT_STALL_EVERY:-1500}

MODS=${MODS:-"rtw88_8812au rtw88_8821au rtw88_8822bu rtw88_8814au rtw88_8822cu rtw88_8822eu"}
# /run/modprobe.d (tmpfs): modprobe reads it like /etc/modprobe.d, but the
# blacklist self-cleans on reboot even if the trap never runs (SIGKILL, power
# loss) — a temp blacklist must never outlive the bench across boots.
BLACKLIST=/run/modprobe.d/zz-temp-blacklist-arqe2e.conf
OUT=${OUT:-/tmp/arq-e2e/$(date +%Y%m%d-%H%M%S)}
mkdir -p "$OUT"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 3; }
for b in duplex txdemo rxdemo; do
  [ -x "$BUILD/$b" ] || { echo "build $b first"; exit 3; }
done

# Total span the drone must cover: feeder warmup + all phases. Float-safe:
# WARMUP_S/PHASE_S may be fractional and truncating them would shorten the
# drone's timeout below the feeder's real runtime.
NPHASES=$(awk -F, '{print NF}' <<<"$PHASES")
SPAN=$(awk -v w="$WARMUP_S" -v c="$CYCLES" -v n="$NPHASES" -v p="$PHASE_S" \
        'BEGIN{v = w + c*n*p + 11; printf "%d", (v == int(v)) ? v : int(v) + 1}')

WIT_PIDF=""; DUT_PIDF=""; FEED_PIDF=""; DRONE_PIDF=""
cleanup() {
  trap - EXIT INT TERM
  for p in "$FEED_PIDF" "$DRONE_PIDF" "$DUT_PIDF" "$WIT_PIDF"; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  # Backstop scoped to THIS checkout's binaries (full-path match) — a bare
  # `pkill -x duplex` from root would kill unrelated instances host-wide.
  # $BUILD is caller-controlled and pkill -f takes an ERE: escape the
  # metacharacters so a path like ".../build+asan" cannot broaden the match.
  esc_build=$(printf '%s' "$BUILD" | sed 's/[][\\.^$*+?(){}|]/\\&/g')
  for b in duplex txdemo rxdemo; do pkill -f "^$esc_build/$b" 2>/dev/null; done
  rm -f "$BLACKLIST" "$OUT/fifo"
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# Temp-blacklist (modprobe -r does NOT survive the re-enumeration that
# claim_interface_then_reset triggers) + unbind by exact VID:PID. Never touch
# device class 09: the dongles sit behind Realtek-branded hubs and a
# vendor-only match would unbind the hub driver and drop the whole tree.
mkdir -p "$(dirname "$BLACKLIST")"   # /run/modprobe.d may not exist yet
: > "$BLACKLIST"
for m in $MODS; do echo "blacklist $m" >> "$BLACKLIST"; modprobe -r "$m" 2>/dev/null; done
ADAPTERS="${DUT_VID#0x}${DUT_PID#0x} ${DRONE_VID#0x}${DRONE_PID#0x} ${WIT_VID#0x}${WIT_PID#0x}"
for d in /sys/bus/usb/devices/*/; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/bDeviceClass" 2>/dev/null)" = "09" ] && continue
  id="$(cat "$d/idVendor")$(cat "$d/idProduct")"
  case " $ADAPTERS " in *" $id "*) ;; *) continue ;; esac
  for i in "$d"*:*; do
    [ -d "$i" ] && [ -e "$i/driver" ] || continue
    basename "$i" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null &&
      echo "[arq-e2e] unbound $(basename "$i")"
  done
done
sleep 1

echo "[arq-e2e] ch=$CH retry_limit=$RETRY_LIMIT drone=$DRONE_RATE/${DRONE_GAP_US}us/${DRONE_PAYLOAD}B" \
     "dis_cca=$DRONE_DIS_CCA phases=$PHASES cycles=$CYCLES span=${SPAN}s" \
     "dut_rx_mode=${DUT_RX_MODE:-async} out=$OUT"

# --- witness: quiet monitor, third ledger --------------------------------
env DEVOURER_VID="$WIT_VID" DEVOURER_PID="$WIT_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA="$TX_SA" \
    DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout \
    "$BUILD/rxdemo" >"$OUT/wit.jsonl" 2>"$OUT/wit.err" &
WIT_PIDF=$!
sleep 8

# --- DUT: PixelPilot topology — duplex RX + burst uplink, responder armed ---
mkfifo "$OUT/fifo"
env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_ACK_RESPONDER="$MAC1" \
    DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA="$TX_SA" DEVOURER_RX_RING_MS=200 \
    ${RECEIPT_MS:+DEVOURER_RX_RECEIPT_MS="$RECEIPT_MS"} \
    ${DUT_RX_MODE:+DEVOURER_RX_MODE="$DUT_RX_MODE"} \
    ${DUT_RX_MODE:+DEVOURER_RX_POOL_SPARE="$DUT_POOL_SPARE"} \
    ${DUT_POOL_EXHAUST:+DEVOURER_RX_POOL_EXHAUST="$DUT_POOL_EXHAUST"} \
    DEVOURER_RX_SINK_SPIN_US="$DUT_SPIN_US" \
    DEVOURER_RX_SINK_STALL_MS="$DUT_STALL_MS" \
    DEVOURER_RX_SINK_STALL_EVERY="$DUT_STALL_EVERY" \
    DEVOURER_TX_PWR_OFFSET_QDB="$PWR_QDB" \
    DEVOURER_LOG_LEVEL=info DEVOURER_EVENTS=stdout \
    "$BUILD/duplex" --interval-ms 0 --max-psdu 4096 \
    <"$OUT/fifo" >"$OUT/dut.jsonl" 2>"$OUT/dut.err" &
DUT_PIDF=$!

python3 "$ROOT/tests/pp109_uplink_feeder.py" \
    --phases "$PHASES" --cycles "$CYCLES" --phase-s "$PHASE_S" \
    --period-ms "$PERIOD_MS" --psdu "$PSDU" --warmup-s "$WARMUP_S" \
    >"$OUT/fifo" 2>"$OUT/feeder.log" &
FEED_PIDF=$!

# --- drone-sim: retrying unicast flood, per-frame reports -------------------
# Started immediately: its ~6-8 s bring-up overlaps the feeder's idle warmup,
# so the flood is airing before phase 1's first burst.
env DEVOURER_VID="$DRONE_VID" DEVOURER_PID="$DRONE_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$MAC1" DEVOURER_TX_SA="$TX_SA" \
    DEVOURER_TX_RATE="$DRONE_RATE" DEVOURER_TX_PAYLOAD_BYTES="$DRONE_PAYLOAD" \
    DEVOURER_TX_GAP_US="$DRONE_GAP_US" \
    DEVOURER_TX_REPORT="$DRONE_REPORT_N" DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
    ${DRONE_FALLBACK:+DEVOURER_TX_RETRY_FALLBACK="$DRONE_FALLBACK"} \
    ${DRONE_AMPDU:+DEVOURER_TX_AMPDU_MODE="$DRONE_AMPDU"} \
    ${DRONE_BATCH:+DEVOURER_TX_BATCH="$DRONE_BATCH"} \
    ${RECEIPT_MS:+DEVOURER_TX_RECEIPTS=1 DEVOURER_TX_WITH_RX=thread} \
    DEVOURER_DIS_CCA="$DRONE_DIS_CCA" \
    DEVOURER_TX_PWR_OFFSET_QDB="$PWR_QDB" \
    DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=stdout \
    timeout -s INT "$SPAN" "$BUILD/txdemo" \
    >"$OUT/drone.jsonl" 2>"$OUT/drone.err" &
DRONE_PIDF=$!

# --- preflight: fail fast with a diagnosis, not a zero-filled matrix --------
# Judged after warmup + one burst phase: every ledger must be alive by then.
sleep "$(awk -v w="$WARMUP_S" -v p="$PREFLIGHT_S" 'BEGIN{print w + p}')"
fail=""
n_dut=$(grep -c '"ev":"rx.seq"' "$OUT/dut.jsonl" 2>/dev/null || true)
n_wit=$(grep -c '"ev":"rx.seq"' "$OUT/wit.jsonl" 2>/dev/null || true)
n_rpt=$(grep -c '"ev":"tx.report"' "$OUT/drone.jsonl" 2>/dev/null || true)
n_ok=$(grep '"ev":"tx.report"' "$OUT/drone.jsonl" 2>/dev/null | grep -c '"ok":true' || true)
hits=$(grep '"ev":"rx.txhit"' "$OUT/wit.jsonl" 2>/dev/null | tail -1 |
       sed -n 's/.*"hits":\([0-9]*\).*/\1/p'); hits=${hits:-0}
[ "${n_dut:-0}" -ge 50 ]  || fail="$fail DUT-rx.seq=$n_dut(<50:not-receiving-flood)"
[ "${n_wit:-0}" -ge 50 ]  || fail="$fail WIT-rx.seq=$n_wit(<50:witness-deaf)"
[ "${n_rpt:-0}" -ge 20 ]  || fail="$fail tx.report=$n_rpt(<20:reports-missing)"
if [ "${n_rpt:-0}" -ge 20 ] && [ $((n_ok * 2)) -lt "$n_rpt" ]; then
  fail="$fail ok=$n_ok/$n_rpt(responder-not-ACKing)"
fi
[ "$hits" -ge 5 ] || fail="$fail wit-txhit=$hits(<5:DUT-uplink-not-airing)"
if [ -n "$fail" ]; then
  echo "[arq-e2e] PREFLIGHT FAILED:$fail"
  echo "[arq-e2e] logs: $OUT (dut.err / drone.err / wit.err)"
  exit 3
fi
echo "[arq-e2e] preflight OK: dut_seq=$n_dut wit_seq=$n_wit reports=$n_rpt" \
     "ok=$n_ok wit_txhit=$hits — running matrix (${SPAN}s total)"

wait "$FEED_PIDF"; FEED_PIDF=""
sleep 2
kill "$DRONE_PIDF" 2>/dev/null; wait "$DRONE_PIDF" 2>/dev/null; DRONE_PIDF=""
sleep 1
kill "$DUT_PIDF" 2>/dev/null; DUT_PIDF=""
kill "$WIT_PIDF" 2>/dev/null; WIT_PIDF=""
sleep 1

python3 "$ROOT/tests/arq_e2e_analyze.py" \
    --dut "$OUT/dut.jsonl" --drone "$OUT/drone.jsonl" --wit "$OUT/wit.jsonl" \
    --phases "$PHASES" --cycles "$CYCLES" | tee "$OUT/report.txt"
ANALYZE_RC=${PIPESTATUS[0]}   # tee would otherwise eat the analyzer verdict
echo "[arq-e2e] logs: $OUT"
exit "$ANALYZE_RC"
