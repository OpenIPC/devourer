#!/usr/bin/env bash
#
# quest_rxq_run.sh — issue #330 confirmatory run on a Meta Quest 3 (constrained
# XR2 host) driven fully headlessly. The Quest hosts the RTL8812AU RX via the
# org.openipc.rxq auto-grant APK (fork()+exec of the devourer rxdemo with the
# USB fd); this desktop side floods a counter-stamped 6M QoS-Data stream from
# the 8812EU and, for each RX-ring servicing MODE, launches the Quest capture,
# pulls the JSONL, and computes seq_num delivery + ring telemetry.
#
# The link only closes with: TX=8812EU (0bda:a81a, not the 8821AU nano), 6M
# OFDM (not 1M CCK), on 2.4GHz ch6 (the Quest's adb-WiFi lives on 5GHz ch36, so
# 5GHz self-desenses). See kaeru issue330-quest-link-WORKS-6M-adb-band-collision.
set -euo pipefail

MODES=${MODES:-"async reorder-pool spsc-fat"}
SINK_SPIN_US=${SINK_SPIN_US:-0}
DUR=${DUR:-20}
CH=${CH:-6}
TX_RATE=${TX_RATE:-6M}
TX_GAP_US=${TX_GAP_US:-800}
TX_PAYLOAD=${TX_PAYLOAD:-200}
BURST_ON_MS=${BURST_ON_MS:-0}     # keyframe-burst model: flat-out for ON ms ...
BURST_OFF_MS=${BURST_OFF_MS:-0}   # ... then idle for OFF ms
URBS=${URBS:-4}
POOL_SPARE=${POOL_SPARE:-8}
TX_VID=${TX_VID:-0x0bda}; TX_PID=${TX_PID:-0xa81a}   # 8812EU flooder
BUILD=${BUILD:-./build}
PKG=org.openipc.rxq
FDIR=/storage/emulated/0/Android/data/$PKG/files
OUT=${OUT:-/tmp/quest-rxq/$(printf '%(%s)T' -1)}
mkdir -p "$OUT"

TXLOG="$OUT/tx.log"
cleanup() {
  sudo -n pkill -x -f "$BUILD/txdemo" 2>/dev/null || true
  adb shell am start -n $PKG/.MainActivity --es action stop >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[quest-rxq] modes=[$MODES] spin=${SINK_SPIN_US}us dur=${DUR}s ch=$CH rate=$TX_RATE gap=${TX_GAP_US}us out=$OUT" >&2

# keep the headset awake (off-head standby drops adb-WiFi)
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null 2>&1 || true
adb shell svc power stayon true >/dev/null 2>&1 || true

# start the steady TX flood (counter-stamped QoS-Data)
sudo -n env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_RATE=$TX_RATE DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_QOS_NOACK=1 \
  DEVOURER_TX_PAYLOAD_BYTES=$TX_PAYLOAD DEVOURER_TX_GAP_US=$TX_GAP_US \
  DEVOURER_TX_BURST_ON_MS=$BURST_ON_MS DEVOURER_TX_BURST_OFF_MS=$BURST_OFF_MS \
  DEVOURER_EVENTS=off DEVOURER_LOG_LEVEL=warn "$BUILD/txdemo" >"$TXLOG" 2>&1 &
sleep 3

for mode in $MODES; do
  tag="m_${mode//-/_}"
  adb shell am start -n $PKG/.MainActivity --es action stop >/dev/null 2>&1 || true
  sleep 3
  adb shell am start -n $PKG/.MainActivity \
    --es tag "$tag" --es rxmode "$mode" --es channel "$CH" \
    --es urbs "$URBS" --es poolspare "$POOL_SPARE" \
    --es sinkspin "$SINK_SPIN_US" >/dev/null 2>&1
  echo "[quest-rxq] mode=$mode capturing ${DUR}s ..." >&2
  sleep "$DUR"
  adb shell am start -n $PKG/.MainActivity --es action stop >/dev/null 2>&1 || true
  sleep 3
  adb pull "$FDIR/rxq_${tag}.jsonl" "$OUT/rxq_${tag}.jsonl" >/dev/null 2>&1 || \
    echo "[quest-rxq] WARN pull failed for $tag" >&2
done

sudo -n pkill -x -f "$BUILD/txdemo" 2>/dev/null || true
trap - EXIT INT TERM

echo "[quest-rxq] analysis:" >&2
python3 tests/quest_rxq_analyze.py "$OUT"/rxq_*.jsonl