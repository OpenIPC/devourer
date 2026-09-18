#!/usr/bin/env bash
# mt7612u_tsf_wrap.sh — the MT7612U TSF read across the low-word wrap.
#
# Every number in the "two TSF halves are not latched" table in
# docs/mt7612u.md comes from `bringup tsfwrap`, which this wraps. The gate
# itself is the measurement; this exists so the invocation, the runtime and
# the re-run rule are not folk knowledge:
#
#   - The wrap is 71.6 min after bring-up, which restarts the counter, so ONE
#     run is ~72 min and covers ONE gap of the read. Both gaps therefore cost
#     ~2.4 h on a single adapter, or ~72 min on two in parallel.
#   - The gate can end with no verdict (rc 3): the wrap landed in the other
#     gap, or someone interrupted it. That is a re-run, not a defect. This
#     script re-runs a gap once for that, and never for rc 1.
#   - One adapter is enough to verify the claim. Two only buys the second
#     unit and the wall-clock.
#
# Nothing here power-cycles a port, unbinds a driver, or touches an adapter
# other than the one named: the gate is register reads on the device devourer
# has claimed.
#
#   tests/mt7612u_tsf_wrap.sh                    # first MT7612U, both gaps
#   MT7612U_DEV=6-1 tests/mt7612u_tsf_wrap.sh    # that adapter, both gaps
#   DEVS="6-1 7-1" tests/mt7612u_tsf_wrap.sh     # one gap each, in parallel
#   GAPS=1 tests/mt7612u_tsf_wrap.sh             # just gap 1
#   SMOKE=1 tests/mt7612u_tsf_wrap.sh            # ~2.5 min, no wrap verdict
#
# Exit: 0 every run PASSed (or SMOKEd), 1 any run FAILed, 3 a run ended
# without a verdict twice, 2 bad invocation.
set -uo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bringup=${BRINGUP:-$here/../src/mt7612u/bringup}
# The C library resolves its firmware relative to the working directory, so
# every run happens in the tool's own directory - the same place `make -C
# src/mt7612u bringup` builds it and the hardware notes invoke it from.
bringup_dir=$(cd -- "$(dirname -- "$bringup")" && pwd 2>/dev/null) || bringup_dir=''
bringup_exe=./$(basename -- "$bringup")
devs=${DEVS:-${MT7612U_DEV:-}}
gaps=${GAPS:-"1 2"}
smoke=${SMOKE:-0}
bits=32
max_min=${MAX_MIN:-80}

if [ ! -x "$bringup" ]; then
	echo "no bringup at $bringup — build it with: make -C src/mt7612u bringup" >&2
	exit 2
fi
if [ ! -r "$bringup_dir/firmware/mt7662.bin" ] ||
   [ ! -r "$bringup_dir/firmware/mt7662_rom_patch.bin" ]; then
	echo "no firmware in $bringup_dir/firmware (mt7662.bin + mt7662_rom_patch.bin, from linux-firmware)" >&2
	exit 2
fi
if [ "$smoke" != 0 ]; then
	# The carry out of bit 24 comes every 16.7 s. It checks the schedule, the
	# host-clock model and the plumbing; it cannot tear a read, and the gate
	# says SMOKE rather than PASS for exactly that reason.
	bits=24
	max_min=5
fi

# One run. Re-runs once on "no verdict", which is the gate asking for it.
run_gap() {
	local dev=$1 gap=$2 attempt rc
	local label="gap $gap${dev:+ on $dev}"

	for attempt in 1 2; do
		echo "== $label (attempt $attempt, up to $max_min min)"
		if [ -n "$dev" ]; then
			( cd "$bringup_dir" && MT7612U_DEV=$dev "$bringup_exe" tsfwrap "$gap" "$bits" "$max_min" )
		else
			( cd "$bringup_dir" && "$bringup_exe" tsfwrap "$gap" "$bits" "$max_min" )
		fi
		rc=$?
		case $rc in
		0) return 0 ;;
		3) echo "== $label: no verdict (rc 3); the gate asks for a re-run" ;;
		*) return "$rc" ;;
		esac
	done
	echo "== $label: no verdict twice — the wrap keeps missing the gap" >&2
	return 3
}

rc_all=0
if [ "$(echo "$devs" | wc -w)" -gt 1 ]; then
	# One gap per adapter, in parallel: the wrap is per-adapter, so this is
	# the only way to cover both gaps in one wrap's worth of wall-clock.
	i=0
	pids=()
	for dev in $devs; do
		i=$((i + 1))
		gap=$(echo "$gaps" | cut -d' ' -f$(( (i - 1) % $(echo "$gaps" | wc -w) + 1 )))
		run_gap "$dev" "$gap" &
		pids+=($!)
	done
	for pid in "${pids[@]}"; do
		wait "$pid" || rc_all=$?
	done
else
	for gap in $gaps; do
		run_gap "$devs" "$gap" || rc_all=$?
	done
fi

if [ "$rc_all" = 0 ]; then
	echo "TSF-WRAP: every run returned a verdict and it was PASS"
else
	echo "TSF-WRAP: rc $rc_all — see the per-run verdicts above" >&2
fi
exit "$rc_all"
