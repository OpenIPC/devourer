#!/usr/bin/env bash
# Does the Jaguar3 RF direct window hold anything above bit 19?
#
# The RF radio tables are applied as plain 20-bit writes to BB[0x3c00/0x4c00
# + (rf_addr & 0xff) * 4] instead of the vendor's MASK20BITS read-modify-
# write. That is bit-identical to the RMW only if bits [31:20] of every
# window word are not storage the RMW could have preserved. Two legs, per
# die, both through `chipstate --init` (ops run on the configured chip):
#
#   1. histogram: dump both path windows after a bring-up and histogram the
#      high 12 bits of every word (expected all 0). On its own this proves
#      little, because the write-only load itself clears those bits.
#   2. write-back: for every word in each window (SAMPLE=N narrows it), poke the word with
#      its high 12 bits SET (low 20 bits unchanged, so the RF register keeps
#      its value), read it back, then restore. If the bits are storage they
#      read back set and the RMW was preserving real state; if they read back
#      0 the RMW could never have preserved anything. This leg is what
#      decides, and it does not depend on which bring-up ran first.
#
# Exit 0 only when both legs say the bits are not storage: a non-zero
# histogram fails leg 1 outright (the write-back leg is not run), and any
# poked word reading back with its high bits set fails leg 2.
#
#   sudo tests/j3_rf_window_readback.sh 0xc812      # 8812CU
#   sudo tests/j3_rf_window_readback.sh 0xa81a      # 8812EU
set -euo pipefail
PID=${1:?usage: $0 <pid-hex>}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CS=$ROOT/build/chipstate
OUT=${OUT:-/tmp/j3_rf_window_readback}; mkdir -p "$OUT"
SAMPLE=${SAMPLE:-256} # words per path window for the write-back leg (256 = the whole window)
case $SAMPLE in ''|*[!0-9]*) echo "FAIL: SAMPLE must be an integer 1..256 (got '$SAMPLE')"; exit 2;; esac
if [ "$SAMPLE" -lt 1 ] || [ "$SAMPLE" -gt 256 ]; then echo "FAIL: SAMPLE must be 1..256 (got $SAMPLE) — a probe of zero words proves nothing"; exit 2; fi
dump=$OUT/pid${PID}.peek
"$CS" --pid "$PID" --init --peek 0x3c00-0x3fff:4 --peek 0x4c00-0x4fff:4 \
  >"$dump" 2>"$dump.err" || { echo "FAIL: chipstate exited non-zero (leg 1)"; tail -5 "$dump.err"; exit 1; }
# leg 1 + the op list for leg 2 (poke set / peek / poke restore per word)
ops=$(python3 - "$dump" "$PID" "$SAMPLE" <<'PY'
import re, sys, collections
words = {}
for line in open(sys.argv[1]):
    m = re.match(r'^0x([0-9a-fA-F]{4}):((?:\s+[0-9a-fA-F]{8}){1,4})\s*$', line)
    if not m: continue
    base = int(m.group(1), 16)
    for i, w in enumerate(m.group(2).split()):
        words[base + 4 * i] = int(w, 16)
if not words:
    print("FAIL: no register rows parsed", file=sys.stderr); sys.exit(1)
hi = collections.Counter(w >> 20 for w in words.values())
print(f"leg1 pid={sys.argv[2]} words={len(words)} hi12_histogram={dict(sorted(hi.items()))}", file=sys.stderr)
if set(hi) != {0}:
    print("FAIL leg1: some window words hold bits above 19 after the bring-up — "
          "contradicts the no-storage premise; the write-back leg is not run", file=sys.stderr)
    sys.exit(1)
n = int(sys.argv[3]); ops = []; restore = []
for base in (0x3c00, 0x4c00):
    for a in range(base, base + 4 * n, 4):
        v = words[a]
        ops += [f"--poke 0x{a:04x}=0x{(v | 0xFFF00000):08x}:4", f"--peek 0x{a:04x}-0x{a+3:04x}:4", f"--poke 0x{a:04x}=0x{v:08x}:4"]
        restore.append(f"--poke 0x{a:04x}=0x{v:08x}:4")
print(" ".join(ops)); print(" ".join(restore))
PY
) || exit 1
restore_ops=$(tail -n1 <<<"$ops"); ops=$(head -n1 <<<"$ops")
# Whatever happens once the first high-bit poke is out, put every sampled
# word back to its dump value. The chip stays configured after chipstate
# exits (its device teardown does not de-init), so a plain attach suffices.
finish() {
  local rc=$?
  # shellcheck disable=SC2086
  if ! "$CS" --pid "$PID" $restore_ops >"$OUT/pid${PID}.restore" 2>&1; then
    echo "FAIL: restore pass failed — window words may be left modified (see $OUT/pid${PID}.restore)" >&2
    [ "$rc" -ne 0 ] || rc=1   # an unrestored radio is not a passing run
  fi
  exit "$rc"
}
trap finish EXIT
wb=$OUT/pid${PID}.writeback
# shellcheck disable=SC2086
"$CS" --pid "$PID" --init $ops >"$wb" 2>"$wb.err" || { echo "FAIL: chipstate exited non-zero (leg 2)"; tail -5 "$wb.err"; exit 1; }
python3 - "$wb" "$PID" "$SAMPLE" <<'PY'
import re, sys
# Each poke is followed by its own one-word peek, printed as a 16-byte row
# with the other three columns blank; a row may still carry 1-4 words, so
# every word on it is checked. The count must equal the pokes issued.
vals = []
for l in open(sys.argv[1]):
    m = re.match(r'^0x([0-9a-fA-F]{4}):((?:\s+[0-9a-fA-F]{8}){1,4})\s*$', l)
    if m:
        vals += [int(w, 16) for w in m.group(2).split()]
n = int(sys.argv[3])
# The peeks were issued in this exact order (path A window, then path B),
# one word each, so the values pair with these addresses — not with the
# printed row base, which is the 16-byte row the word sits in.
addrs = [a for base in (0x3c00, 0x4c00) for a in range(base, base + 4 * n, 4)]
if len(vals) != len(addrs):
    print(f"FAIL: parsed {len(vals)} read-back words, expected {len(addrs)} (one per poke)"); sys.exit(1)
rows = list(zip(addrs, vals))
bad = [(a, v) for a, v in rows if v >> 20]
print(f"leg2 pid={sys.argv[2]} words_poked_with_hi_bits_set={len(rows)} read_back_nonzero_hi={len(bad)}")
for a, v in bad: print(f"  0x{a:04x} -> 0x{v:08x}")
if bad:
    print("VERDICT: bits [31:20] ARE storage on this die — keep the MASK20BITS RMW"); sys.exit(1)
print("VERDICT: bits [31:20] are not storage (written 1s read back 0) — the write-only load is bit-identical to the RMW")
PY
