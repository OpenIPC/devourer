#!/usr/bin/env bash
# Does a PCIe DUT init and tear down cleanly under the sanitizers?
#
# The PCIe sibling of tests/teardown_gen_sanity.sh: one short RX session and
# one short TX session on the vfio-held card, built with
# DEVOURER_SANITIZE=address+undefined, each stopped with SIGINT. Both must
# exit 0 with no sanitizer report, and the host kernel log must show no
# DMAR/IOMMU fault — the lifetime bug a PCIe teardown can have that USB
# cannot: DMA still running into a slab the transport already unmapped.
#
# Build:  cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
#               -DDEVOURER_PCIE=ON -DDEVOURER_SANITIZE=address+undefined \
#         && cmake --build build-asan -j
# Run:    sudo tests/pcie_teardown_asan.sh <bdf>      (card bound to vfio-pci)
set -u
BDF="${1:?usage: $0 <bdf>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build-asan}"
CHANNEL="${CHANNEL:-36}"
RUN_SECS="${RUN_SECS:-8}"
OUT="${OUT:-/tmp/devourer-pcie-teardown}"
[ "$(id -u)" = 0 ] || { echo "ERROR: run as root (vfio)" >&2; exit 2; }
[ -x "$BUILD/rxdemo" ] && [ -x "$BUILD/txdemo" ] || {
  echo "ERROR: $BUILD/rxdemo or txdemo missing (sanitizer PCIe build)" >&2; exit 2; }
mkdir -p "$OUT"
PID=""
cleanup() { [ -n "$PID" ] && kill -KILL "$PID" 2>/dev/null; true; }
trap cleanup EXIT INT TERM

dmar_faults() { journalctl -k -b 0 --no-pager 2>/dev/null | grep -c -E "DMAR.*(fault|PTE)" || true; }

fail=0
run_session() {  # NAME DEMO [extra env...]
  local name="$1" demo="$2"; shift 2
  local log="$OUT/$name.log" ev="$OUT/$name.jsonl"
  local d0; d0=$(dmar_faults)
  env DEVOURER_PCIE_BDF="$BDF" DEVOURER_CHANNEL="$CHANNEL" DEVOURER_LOG_LEVEL=info \
      ASAN_OPTIONS=detect_leaks=0 "$@" "$BUILD/$demo" >"$ev" 2>"$log" &
  PID=$!
  sleep "$RUN_SECS"
  kill -INT "$PID" 2>/dev/null
  local t=0
  while kill -0 "$PID" 2>/dev/null && [ "$t" -lt 20 ]; do sleep 1; t=$((t + 1)); done
  local rc=124
  if kill -0 "$PID" 2>/dev/null; then kill -KILL "$PID"; else wait "$PID"; rc=$?; fi
  PID=""
  local san; san=$(grep -c -E "AddressSanitizer|UndefinedBehaviorSanitizer|runtime error:" "$log" || true)
  local d1; d1=$(dmar_faults)
  local ok=true
  [ "$rc" = 0 ] && [ "$san" = 0 ] && [ "$d1" = "$d0" ] || ok=false
  echo "$name: exit=$rc sanitizer_reports=$san dmar_faults_delta=$((d1 - d0)) -> $([ $ok = true ] && echo PASS || echo FAIL)"
  grep -E "\[E\]|Sanitizer|runtime error" "$log" | head -5 | sed 's/^/    /'
  echo "{\"ev\":\"teardown\",\"name\":\"$name\",\"ok\":$ok,\"exit\":$rc,\"sanitizer\":$san,\"dmar_delta\":$((d1 - d0))}"
  [ $ok = true ] || fail=1
}

run_session rx rxdemo
run_session tx txdemo DEVOURER_TX_GAP_US=2000
exit $fail
