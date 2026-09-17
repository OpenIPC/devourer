#!/usr/bin/env bash
# Does the Jaguar3 RF direct window keep anything above bit 19?
#
# The RF radio tables are applied as plain 20-bit writes to BB[0x3c00/0x4c00
# + (rf_addr & 0xff) * 4] instead of the vendor's MASK20BITS read-modify-
# write. That is bit-identical to the RMW only if bits [31:20] of every
# window word read back 0 — which this measures, per die, after a full
# bring-up: it dumps both path windows through chipstate --init --peek and
# histograms the high 12 bits of each 32-bit word.
#
#   sudo tests/j3_rf_window_readback.sh 0xc812      # 8812CU
#   sudo tests/j3_rf_window_readback.sh 0xa81a      # 8812EU
set -euo pipefail
PID=${1:?usage: $0 <pid-hex>}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-/tmp/j3_rf_window_readback}; mkdir -p "$OUT"
dump=$OUT/pid${PID}.peek
"$ROOT/build/chipstate" --pid "$PID" --init --peek 0x3c00-0x3fff:4 --peek 0x4c00-0x4fff:4 \
  >"$dump" 2>"$dump.err" || { echo "FAIL: chipstate exited non-zero"; tail -5 "$dump.err"; exit 1; }
python3 - "$dump" "$PID" <<'PY'
import re, sys, collections
words = []
for line in open(sys.argv[1]):
    m = re.match(r'^0x[0-9a-fA-F]{4}:((?:\s+[0-9a-fA-F]{8}){1,4})\s*$', line)
    if not m: continue
    words += [int(w, 16) for w in m.group(1).split()]
if not words:
    print("FAIL: no register rows parsed from", sys.argv[1]); sys.exit(1)
hi = collections.Counter(w >> 20 for w in words)
print(f"pid={sys.argv[2]} words={len(words)} hi12_histogram={dict(sorted(hi.items()))}")
print("VERDICT:", "write-only is bit-identical (all [31:20] == 0)" if hi.keys() == {0} else "NOT ZERO — keep the RMW on this die")
PY
