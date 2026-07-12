#!/usr/bin/env bash
# dl_departure_matrix.sh — M0 contract 1: the per-transport send_packet→air
# guard-time matrix. One fixed witness receiver; the TX role sweeps the
# transports (Jaguar1 async-USB, Jaguar2 sync-USB, Jaguar3 sync-USB, and the
# 8821CE PCIe cell on a remote rig). Every TX frame is a TD v2 tag carrying
# tx_tsf (ReadTsf) + host_ns (steady_clock immediately before send_packet); the
# witness records them with its hardware rx_tsfl, and txegress_analyze.py's
# host-clock fit turns the residual tail into the per-transport guard contract
# (one txeg.verdict JSONL line per cell). Results table: docs/dl-departure.md.
#
#   sudo bash tests/dl_departure_matrix.sh
#   SECS=60 SLOT_US=1000 sudo bash tests/dl_departure_matrix.sh
#   SKIP_PCIE=1 sudo bash tests/dl_departure_matrix.sh   # USB cells only
set -uo pipefail
cd "$(dirname "$0")/.."

WIT_VID=${WIT_VID:-0x2357}; WIT_PID=${WIT_PID:-0x0120}   # 8821AU witness
CH=${CH:-6}; SECS=${SECS:-40}; N=${N:-2000}; GAP_US=${GAP_US:-15000}
SLOT_US=${SLOT_US:-1000}   # go/no-go bound for "fine DL slots"
OUT=${OUT:-/tmp/dl_departure}
# name:vid:pid — the USB TX cells (transport is per generation: J1 async USB2,
# J2/J3 sync USB3).
CELLS=${CELLS:-"j1-8812au-usb:0x0bda:0x8812 j2-8812bu-usb:0x2357:0x012d j3-8822cu-usb:0x0bda:0xc812"}
# PCIe cell (remote rig; SKIP_PCIE=1 to skip).
PCIE_HOST=${PCIE_HOST:-radxa-x4}
PCIE_DIR=${PCIE_DIR:-devourer}                # remote tree (rsync'd, not git)
PCIE_BDF=${PCIE_BDF:-0000:01:00.0}

cleanup(){
  sudo pkill -9 -x dl_departure_tx 2>/dev/null
  sudo pkill -9 -x txegress_witness 2>/dev/null
  return 0
}
trap cleanup EXIT
mkdir -p "$OUT"

build_local() { # $1 = src basename (no .cpp), extra include dirs baked in
  echo "== building $1"
  g++ -std=c++20 -O2 -Isrc -Iexamples/common -Iexamples/tdma "tests/$1.cpp" \
      examples/common/env_config.cpp build/libdevourer.a \
      $(pkg-config --cflags --libs libusb-1.0) -lpthread -o "build/$1"
}
[ -x build/dl_departure_tx ] && [ ! tests/dl_departure_tx.cpp -nt build/dl_departure_tx ] || build_local dl_departure_tx
[ -x build/txegress_witness ] && [ ! tests/txegress_witness.cpp -nt build/txegress_witness ] \
    && [ ! examples/tdma/tdma.h -nt build/txegress_witness ] || build_local txegress_witness

run_witness() { # $1 = out file — witness capped at N frames or SECS+15 s
  sudo env DEVOURER_VID=$WIT_VID DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
      timeout $((SECS + 15)) ./build/txegress_witness "$N" 2>"$1.err" >"$1"
}

VERDICTS="$OUT/verdicts.jsonl"; : >"$VERDICTS"

for cell in $CELLS; do
  name=${cell%%:*}; rest=${cell#*:}; vid=${rest%%:*}; pid=${rest#*:}
  echo
  echo "==== cell $name (TX $vid:$pid, ch$CH, ${SECS}s) ===="
  cleanup; sleep 1
  sudo env DEVOURER_VID=$vid DEVOURER_PID=$pid DEVOURER_CHANNEL=$CH \
      GAP_US=$GAP_US DEVOURER_LOG_LEVEL=warn \
      ./build/dl_departure_tx "$SECS" >"$OUT/tx_$name.log" 2>&1 &
  sleep 6   # TX bring-up
  run_witness "$OUT/wit_$name.jsonl"
  cleanup; sleep 1
  echo "  frames: $(grep -c '"ev":"txeg"' "$OUT/wit_$name.jsonl" || true)"
  python3 tests/txegress_analyze.py "$OUT/wit_$name.jsonl" \
      --transport "$name" --slot-us "$SLOT_US" | tee "$OUT/an_$name.txt"
  grep '"ev": *"txeg.verdict"' "$OUT/an_$name.txt" >>"$VERDICTS" || true
done

if [ "${SKIP_PCIE:-0}" != "1" ]; then
  name=pcie-8821ce
  echo
  echo "==== cell $name ($PCIE_HOST $PCIE_BDF, ch$CH, ${SECS}s) ===="
  cleanup; sleep 1
  # Ship the TD-v2 sources, build the probe remotely against the remote
  # libdevourer.a, vfio-bind, run, restore the kernel driver.
  scp -q examples/tdma/tdma.h "$PCIE_HOST:$PCIE_DIR/examples/tdma/tdma.h"
  scp -q tests/pcie_txegress_tx.cpp "$PCIE_HOST:$PCIE_DIR/tests/pcie_txegress_tx.cpp"
  if ssh "$PCIE_HOST" "cd $PCIE_DIR && \
      g++ -std=c++20 -O2 -DDEVOURER_HAVE_PCIE=1 -Isrc -Iexamples/tdma tests/pcie_txegress_tx.cpp \
        build-pcie/libdevourer.a \$(pkg-config --cflags --libs libusb-1.0) \
        -lpthread -o build-pcie/pcie_txegress_tx && \
      sudo bash tests/pcie_vfio_bind.sh $PCIE_BDF"; then
    ssh "$PCIE_HOST" "cd $PCIE_DIR && \
        sudo DEVOURER_PCIE_BDF=$PCIE_BDF DEVOURER_CHANNEL=$CH GAP_US=$GAP_US \
        build-pcie/pcie_txegress_tx $SECS" >"$OUT/tx_$name.log" 2>&1 &
    PCIE_SSH=$!
    sleep 8   # remote bring-up
    run_witness "$OUT/wit_$name.jsonl"
    wait $PCIE_SSH 2>/dev/null
    ssh "$PCIE_HOST" "cd $PCIE_DIR && sudo bash tests/pcie_vfio_bind.sh $PCIE_BDF --restore" \
        >/dev/null 2>&1 || true
    echo "  frames: $(grep -c '"ev":"txeg"' "$OUT/wit_$name.jsonl" || true)"
    python3 tests/txegress_analyze.py "$OUT/wit_$name.jsonl" \
        --transport "$name" --slot-us "$SLOT_US" | tee "$OUT/an_$name.txt"
    grep '"ev": *"txeg.verdict"' "$OUT/an_$name.txt" >>"$VERDICTS" || true
  else
    echo "  PCIe cell SKIPPED (remote build/bind failed)"
  fi
fi

echo
echo "==== MATRIX VERDICTS ($VERDICTS) ===="
cat "$VERDICTS"
