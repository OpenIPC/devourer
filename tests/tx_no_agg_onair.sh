#!/usr/bin/env bash
# tx_no_agg_onair.sh — does TxMode::no_agg keep a frame out of an A-MPDU, so
# it airs at its OWN rate? (AdapterCaps::tx_no_agg_ok, RadiotapTxFlags.h)
#
# Under SetAmpduMode the MAC folds consecutive co-queued frames into one PPDU
# aired at the FIRST MPDU's rate, so a frame's own rate is silently replaced
# whenever it lands behind a frame of another rate. The DUT floods QoS-Data
# with an A-MPDU session on, alternating two HT rates by frame counter:
#
#   even counter  BASE rate (DEVOURER_TX_RATE), aggregatable
#   odd counter   ALT rate  (DEVOURER_TX_ALT_RATE), with /NOAGG in the
#                 "noagg" arm and without it in the "control" arm
#
# The "basenoagg" arm moves the flag to the BASE side: the even frames are
# rate-less, so /NOAGG reaches them only through the SetTxMode default, not
# their own radiotap. Dropping that default leaves the arm a copy of the
# control (odd frames fold), so it guards the rate-less path.
#
# A passive witness decodes each copy's rate (rx.seq: pctr + hw rate index),
# so every received frame is scored against the rate it was submitted with.
#
# The CONTROL arm is what makes the flagged result mean anything: if the
# control does not show odd frames folding to the BASE rate, no mixed
# aggregates formed (feed too shallow, session off, wrong queue) and a clean
# flagged arm proves nothing. That is an ABORT, not a pass.
#
# Witness: local by default (WIT_VID/WIT_PID). WIT_SSH=user@host runs a
# prebuilt rxdemo (WIT_BIN) on another machine instead — the adapter there
# must be free of any other owner. Ambient traffic is excluded by a
# run-unique SA.
#
#   sudo bash tests/tx_no_agg_onair.sh
#   CH=36 BASE=MCS5 ALT=MCS0 SECS=15 sudo bash tests/tx_no_agg_onair.sh
#   WIT_SSH=root@10.18.0.1 WIT_BIN=/tmp/rxdemo sudo -E bash tests/tx_no_agg_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD=${BUILD:-$ROOT/build}

DUT_VID=${DUT_VID:-0x0bda}; DUT_PID=${DUT_PID:-0xa81a}   # RTL8812EU
WIT_VID=${WIT_VID:-0x0bda}; WIT_PID=${WIT_PID:-0xa81a}
WIT_SSH=${WIT_SSH:-}; WIT_BIN=${WIT_BIN:-$BUILD/rxdemo}
CH=${CH:-36}
BASE=${BASE:-MCS5}; ALT=${ALT:-MCS0}
ARMS=${ARMS:-"noagg control basenoagg noagg control basenoagg"}
SECS=${SECS:-12}
AMPDU=${AMPDU:-0/6}          # DEVOURER_TX_AMPDU_MODE: TID0, 6 MPDUs
THREADS=${THREADS:-4}        # deep feed: aggregation needs co-queued frames
PAYLOAD=${PAYLOAD:-1000}
if [ -z "${TX_SA:-}" ]; then
  run_id=$$
  printf -v TX_SA '02:6e:61:%02x:%02x:%02x' \
    $(((run_id >> 16) & 255)) $(((run_id >> 8) & 255)) $((run_id & 255))
fi
OUT=${OUT:-/tmp/tx_no_agg}
SUDO=${SUDO-sudo}            # SUDO= when the adapters' USB nodes are user-writable

# HT rates only: the witness reports the raw descriptor rate, MCSn = 0x0c + n.
rate_code() {
  if [[ "$1" =~ ^MCS([0-9]|[12][0-9]|3[01])$ ]]; then
    echo $((12 + BASH_REMATCH[1]))
  else
    echo "ABORT: '$1' is not an HT MCS0..MCS31 rate" >&2; exit 2
  fi
}
BASE_CODE=$(rate_code "$BASE") || exit 2
ALT_CODE=$(rate_code "$ALT") || exit 2
[ "$BASE_CODE" -ne "$ALT_CODE" ] || { echo "ABORT: BASE and ALT must differ" >&2; exit 2; }
case " $ARMS " in *" noagg "*|*" basenoagg "*) ;; *) echo "ABORT: ARMS needs a noagg or basenoagg arm" >&2; exit 2;; esac
case " $ARMS " in *" control "*) ;; *) echo "ABORT: ARMS needs a control arm" >&2; exit 2;; esac

# Kill only this tree's demos (ERE-escaped build prefix), and the remote
# witness if there is one.
ESC_BUILD=$(printf '%s' "$BUILD" | sed 's#[][\\.^$*+?(){}|/]#\\&#g')
KILL() {
  $SUDO pkill -9 -f "^$ESC_BUILD/rxdemo" 2>/dev/null
  $SUDO pkill -9 -f "^$ESC_BUILD/txdemo" 2>/dev/null
  [ -n "$WIT_SSH" ] && ssh "$WIT_SSH" \
    "pkill -9 -f '^$WIT_BIN' || killall -9 $(basename "$WIT_BIN")" 2>/dev/null
  return 0
}
trap KILL EXIT
mkdir -p "$OUT"; RESULTS="$OUT/results.jsonl"; : >"$RESULTS"

WIT_ENV="DEVOURER_CHANNEL=$CH DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=$TX_SA DEVOURER_LOG_LEVEL=info"
idx=0
for arm in $ARMS; do
  idx=$((idx+1))
  tag="$(printf '%02d_%s' "$idx" "$arm")"
  case "$arm" in
    noagg)     base_spec="$BASE";       alt_spec="$ALT/NOAGG" ;;
    control)   base_spec="$BASE";       alt_spec="$ALT" ;;
    basenoagg) base_spec="$BASE/NOAGG"; alt_spec="$ALT" ;;
    *) echo "ABORT: unknown arm '$arm' (noagg|control|basenoagg)" >&2; exit 2 ;;
  esac
  KILL; sleep 3   # USB release after a -9 is not instantaneous
  # shellcheck disable=SC2024
  if [ -n "$WIT_SSH" ]; then
    ssh "$WIT_SSH" "env DEVOURER_VID=$WIT_VID DEVOURER_PID=$WIT_PID $WIT_ENV $WIT_BIN" \
      >"$OUT/wit_$tag.jsonl" 2>"$OUT/wit_$tag.err" &
  else
    $SUDO env DEVOURER_VID="$WIT_VID" DEVOURER_PID="$WIT_PID" $WIT_ENV \
      "$WIT_BIN" >"$OUT/wit_$tag.jsonl" 2>"$OUT/wit_$tag.err" &
  fi
  waited=0
  until grep -qE "async ring of .* URBs submitted|Listening air" "$OUT/wit_$tag.err"; do
    sleep 1; waited=$((waited+1))
    if [ "$waited" -ge 30 ]; then
      echo "ABORT: witness never reached RX for arm $arm (#$idx)" >&2
      tail -5 "$OUT/wit_$tag.err" >&2; exit 1
    fi
  done
  sleep 2
  # shellcheck disable=SC2024
  $SUDO env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
       DEVOURER_CHANNEL="$CH" DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_QOS_NOACK=1 \
       DEVOURER_TX_SA="$TX_SA" DEVOURER_TX_RATE="$base_spec" \
       DEVOURER_TX_ALT_RATE="$alt_spec" DEVOURER_TX_AMPDU_MODE="$AMPDU" \
       DEVOURER_TX_THREADS="$THREADS" DEVOURER_TX_GAP_US=0 \
       DEVOURER_TX_PAYLOAD_BYTES="$PAYLOAD" DEVOURER_LOG_LEVEL=warn \
       timeout -s INT "$((SECS + 8))" "$BUILD/txdemo" \
       >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" || true
  sleep 2
  KILL; sleep 1
  sent=$(grep '"ev":"tx.stats"' "$OUT/tx_$tag.jsonl" | tail -1 |
         sed -n 's/.*"submitted":\([0-9]*\).*/\1/p'); sent=${sent:-0}
  python3 - "$OUT/wit_$tag.jsonl" "$arm" "$idx" "$sent" "$BASE_CODE" "$ALT_CODE" \
      >>"$RESULTS" <<'PY' || exit 1
import json, sys
path, arm, idx, sent = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
base, alt = int(sys.argv[5]), int(sys.argv[6])
n = {0: 0, 1: 0}; own = {0: 0, 1: 0}; t0 = t1 = None
for line in open(path, errors="replace"):
    if not line.startswith('{"ev":"rx.seq"'):
        continue
    try:
        e = json.loads(line)
    except json.JSONDecodeError:
        continue
    if e.get("crc"):
        continue
    par = e["pctr"] & 1
    n[par] += 1
    own[par] += e["rate"] == (alt if par else base)
    t = e.get("t")
    if t is not None:
        t0 = t if t0 is None else t0
        t1 = t
# A cell that did not run is not a measurement: no submissions or no odd
# (ALT) frames heard is a harness failure, never a 0 % or 100 % result.
if sent == 0 or n[0] < 500 or n[1] < 500:
    sys.stderr.write(f"ABORT: arm {arm} (#{idx}) did not run - submitted={sent} "
                     f"heard even={n[0]} odd={n[1]}\n")
    sys.exit(1)
secs = (t1 - t0) / 1000.0 if t0 is not None and t1 > t0 else 0.0
print(json.dumps({"ev": "noagg.arm", "arm": arm, "idx": idx, "submitted": sent,
                  "odd": n[1], "even": n[0],
                  "odd_own_pct": round(100.0 * own[1] / n[1], 1),
                  "even_own_pct": round(100.0 * own[0] / n[0], 1),
                  "heard_fps": round((n[0] + n[1]) / secs) if secs else None}))
PY
  tail -1 "$RESULTS"
done

echo "==== VERDICT ===="
python3 - "$RESULTS" <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
ctl = [r for r in rows if r["arm"] == "control"]
flg = [r for r in rows if r["arm"] in ("noagg", "basenoagg")]
for r in rows:
    print(f"  #{r['idx']} {r['arm']:<8} odd@own {r['odd_own_pct']:>5}%  "
          f"even@own {r['even_own_pct']:>5}%  heard {r['heard_fps']} fps")
# The control must show the fold, or nothing aggregated and the flagged arm
# is not a test of no_agg.
folded = all(r["odd_own_pct"] <= 80.0 for r in ctl)
if not folded:
    print("ABORT: control arm shows no fold (odd frames kept their own rate "
          "without the flag) - no mixed A-MPDUs formed; not a measurement")
    sys.exit(2)
ok = all(r["odd_own_pct"] >= 99.0 and r["even_own_pct"] >= 99.0 for r in flg)
print(json.dumps({"ev": "noagg.verdict", "tx_no_agg_ok": bool(ok),
                  "arms": len(rows)}))
sys.exit(0 if ok else 1)
PY
