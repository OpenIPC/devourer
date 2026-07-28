#!/usr/bin/env bash
# Why is 40 MHz broken? Diff devourer's 40 MHz register state against the vendor
# driver's, on the same adapter.
#
# Reading the code is not enough and was actively misleading here: devourer's
# `phy_PostSetBwMode8812` 40 MHz branch is register-for-register identical to the
# vendor's `phy_PostSetBwMode8812`, yet 40 MHz fails in both directions while
# 20 MHz is perfect. So whatever diverges is set somewhere else in bring-up, and
# only the chip can say where.
#
# Method: put the SAME adapter into 40 MHz under each driver and dump the same
# canary register set, then diff. The 20 MHz pair is captured too — registers
# that differ at 20 MHz as well are pre-existing drift and not what we are
# hunting; the interesting set is what differs at 40 and matches at 20.
#
#   sudo tests/bw40_regdiff.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

DUT_VID=${DUT_VID:-0bda} DUT_PID=${DUT_PID:-8812}
DUT_REF=${DUT_REF:-rtl8812au}
DUT_KO=${DUT_KO:-88XXau_ohd.ko}
DUT_MOD=${DUT_MOD:-88XXau_ohd}
INTREE=${INTREE:-rtw88_8812au}
CHIP=${CHIP:-8812}
CH=${CH:-6}
HT=${HT:-HT40+}
DEV_OFFSET=${DEV_OFFSET:-1}
OUT=${OUT:-/tmp/devourer-bw40-regdiff}
mkdir -p "$OUT"

cleanup() { sudo pkill -x rxdemo 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

cold_cycle() {
  local map=${REGRESS_VBUS_MAP:-} entry hubport
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$DUT_VID:$DUT_PID=" | head -1) || return 0
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

# --- devourer side: bring up at $1 MHz, dump the canary ----------------------
devourer_dump() {
  local bw="$1" out="$2" e=()
  sudo rmmod "$DUT_MOD" 2>/dev/null
  sudo modprobe -r "$INTREE" 2>/dev/null
  cold_cycle
  sudo modprobe -r "$INTREE" 2>/dev/null
  [ "$bw" = 40 ] && e=(DEVOURER_BW=40 DEVOURER_CHOFFSET="$DEV_OFFSET")
  sudo env DEVOURER_VID="0x$DUT_VID" DEVOURER_PID="0x$DUT_PID" \
    DEVOURER_CHANNEL="$CH" "${e[@]}" DEVOURER_DUMP_CANARY=1 \
    DEVOURER_LOG_LEVEL=info \
    timeout --signal=TERM 22 "$ROOT/build/rxdemo" >/dev/null 2>"$OUT/dev_$bw.raw"
  # Last canary block only — the first is pre-retune on some paths. Match the
  # OPENING marker specifically: "END DEVOURER_DUMP_CANARY" also contains the
  # bare name, so a loose start pattern resets the block on the very line that
  # should close it and yields an empty capture.
  awk '/DEVOURER_DUMP_CANARY \(post/{blk=""} {blk=blk $0 ORS}
       /END DEVOURER_DUMP_CANARY/{last=blk} END{printf "%s", last}' \
      "$OUT/dev_$bw.raw" |
    sed -E 's/^devourer \[[A-Z]\] //' |
    grep -E '^(BB|BBC1|MAC|RF)' > "$out"
  wc -l < "$out"
}

# --- vendor side: same adapter, same width, same register set ----------------
vendor_dump() {
  local bw="$1" out="$2"
  sudo modprobe -r "$INTREE" 2>/dev/null
  cold_cycle
  sudo modprobe -r "$INTREE" 2>/dev/null
  sudo insmod "$ROOT/reference/$DUT_REF/$DUT_KO" 2>/dev/null || true
  local u prev="" cur="" iface=""
  for d in /sys/bus/usb/devices/*/; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$DUT_VID" ] || continue
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "$DUT_PID" ] || continue
    u=$(basename "$d"); break
  done
  [ -n "${u:-}" ] || return 1
  for _ in $(seq 30); do
    cur=$(basename "$(ls -d /sys/bus/usb/devices/$u/*/net/* 2>/dev/null | head -1)" 2>/dev/null)
    [ -n "$cur" ] && [ "$cur" = "$prev" ] && [ -e "/sys/class/net/$cur" ] && { iface="$cur"; break; }
    prev="$cur"; sleep 1
  done
  [ -n "$iface" ] || return 1
  sudo ip link set "$iface" down
  sudo iw dev "$iface" set type monitor
  sudo ip link set "$iface" up
  if [ "$bw" = 40 ]; then
    sudo iw dev "$iface" set channel "$CH" "$HT" || return 1
  else
    sudo iw dev "$iface" set channel "$CH" || return 1
  fi
  sleep 2
  # Pass the HT mode through: the dump tool sets the channel itself, and
  # without this it does so at 20 MHz — silently capturing a 20 MHz chip and
  # labelling it 40 MHz.
  local htarg=""; [ "$bw" = 40 ] && htarg="$HT"
  sudo "$ROOT/tools/canary_kernel_dump.sh" "$iface" "$CH" "$CHIP" $htarg \
      > "$out" 2>"$OUT/ven_$bw.err"
  sudo rmmod "$DUT_MOD" 2>/dev/null
  wc -l < "$out"
}

echo "40 MHz register diff — same adapter ($DUT_VID:$DUT_PID) under each driver"
echo
for bw in 20 40; do
  echo "  capturing bw$bw ..."
  d=$(devourer_dump "$bw" "$OUT/devourer_$bw.canary") || true
  v=$(vendor_dump  "$bw" "$OUT/vendor_$bw.canary") || true
  echo "    devourer $d lines, vendor $v lines"
done

echo
python3 - "$OUT" <<'EOF'
import re, sys, os
out = sys.argv[1]

def load(p):
    d = {}
    if not os.path.exists(p):
        return d
    for line in open(p, errors="replace"):
        m = re.match(r'^(\w+)\s+(0x[0-9a-fA-F]+)\s*=\s*(0x[0-9a-fA-F]+)', line.strip())
        if m:
            d[(m.group(1), m.group(2).lower())] = int(m.group(3), 16)
    return d

sets = {(s, b): load(os.path.join(out, "%s_%s.canary" % (s, b)))
        for s in ("devourer", "vendor") for b in ("20", "40")}
for k, v in sets.items():
    if not v:
        print("  !! %s_%s.canary is empty — that half did not capture, diff is void" % k)
        raise SystemExit(2)

d20, v20 = sets[("devourer", "20")], sets[("vendor", "20")]
d40, v40 = sets[("devourer", "40")], sets[("vendor", "40")]

keys = sorted(set(d40) & set(v40))
diff40 = [k for k in keys if d40[k] != v40[k]]
same20 = [k for k in diff40 if k in d20 and k in v20 and d20[k] == v20[k]]

print("  registers compared at 40 MHz: %d" % len(keys))
print("  differ devourer-vs-vendor at 40 MHz: %d" % len(diff40))
print()
print("  === PRIME SUSPECTS: differ at 40 MHz, MATCH at 20 MHz ===")
if not same20:
    print("  (none — the 40 MHz divergence is not specific to 40 MHz)")
for k in same20:
    print("    %-5s %-8s devourer=0x%08X vendor=0x%08X   (both 0x%08X at 20 MHz)"
          % (k[0], k[1], d40[k], v40[k], d20[k]))

rest = [k for k in diff40 if k not in same20]
if rest:
    print()
    print("  === also differ at 20 MHz (pre-existing drift, not this bug) ===")
    for k in rest[:20]:
        print("    %-5s %-8s devourer=0x%08X vendor=0x%08X" % (k[0], k[1], d40[k], v40[k]))
    if len(rest) > 20:
        print("    ... and %d more" % (len(rest) - 20))
EOF
echo
echo "captures: $OUT"
