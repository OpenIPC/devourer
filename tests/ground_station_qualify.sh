#!/usr/bin/env bash
# Is this ground station fit to measure the rate you are about to measure?
#
# Run this BEFORE trusting any delivery number, and before believing any story
# about a transmitter. A receiver whose modulation cliff sits at or below the
# test rate does not measure the transmitter — it measures itself, and it does so
# with enormous swings, because a few dB of ambient moves a cliff-edge rate
# between "fine" and "zero" while leaving the robust rates untouched.
#
# That failure mode looks exactly like a transmitter that degrades and recovers,
# and it cost this project an entire investigation. Measured here on the same
# transmitter, same channel, minutes apart:
#
#   TP-Link Archer T3U (8822BU, INTERNAL antennas)
#     MCS3 97.8  MCS4 98.3  MCS5 79.1  MCS6 49.0  MCS7 2.9   <- cliff inside 64-QAM
#   RTL8814AU (two EXTERNAL antennas)
#     MCS3 80.6  MCS4 80.2  MCS5 80.8  MCS6 80.6  MCS7 80.4  <- flat, margin to spare
#
# Both were healthy. The T3U simply has less antenna gain, so its cliff lands in
# the middle of the rates under test. Earlier in the same session, with lower
# ambient, that same T3U delivered MCS7 at 68% — the cliff had not moved, the
# operating point had.
#
# A qualified ground station is FLAT from the test rate downwards. If delivery is
# already rolling off at the test rate, raise the ground's margin (external
# antennas, distance, orientation, a quieter channel) or measure a lower rate.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;0bda:8813=4-2.3,2" \
#        GND_VID=0x0bda GND_PID=0x8813 tests/ground_station_qualify.sh MCS7/20
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TEST_RATE=${1:-MCS7/20}
DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:-0x8812}
GND_VID=${GND_VID:-0x0bda} GND_PID=${GND_PID:-0x8813}
CH=${CH:-6}
LADDER=${LADDER:-"MCS1/20 MCS3/20 MCS4/20 MCS5/20 MCS6/20 MCS7/20"}
SECS=${SECS:-5}
OUT=${OUT:-/tmp/devourer-ground-qualify}
mkdir -p "$OUT"

# The ladder must include the test rate, or the verdict is about nothing.
case " $LADDER " in
  *" $TEST_RATE "*) ;;
  *) LADDER="$LADDER $TEST_RATE" ;;
esac

"$HERE/nitroqam_waterfall.sh" \
    --emit-vid "$DUT_VID" --emit-pid "$DUT_PID" \
    --ground-vid "$GND_VID" --ground-pid "$GND_PID" \
    --channel "$CH" --bw 20 \
    --encs "$(echo "$LADDER" | tr ' ' ',')" \
    --baseline "${LADDER%% *}" \
    --pwr-mode none --pwr-start 0 --pwr-stop 0 --secs "$SECS" \
    --outdir "$OUT" 2>/dev/null | grep -E '^\{' > "$OUT/points.jsonl"

python3 - "$OUT/points.jsonl" "$TEST_RATE" <<'EOF'
import json, sys

pts = {}
for line in open(sys.argv[1]):
    line = line.strip()
    if not line:
        continue
    p = json.loads(line)
    if p["sent"] > 0:
        pts[p["enc"]] = 100.0 * p["delivered"] / p["sent"]
test = sys.argv[2]

if not pts:
    print("no usable points — the link delivered nothing at any rate")
    raise SystemExit(2)

print("  ground-station rate ladder")
for enc, v in pts.items():
    print("    %-12s %5.1f%%  %s" % (enc, v, "#" * int(v / 4)))

if test not in pts:
    print("\n  test rate %s not measured — cannot qualify" % test)
    raise SystemExit(2)

# Flat means the test rate keeps up with the best robust rate below it. A cliff
# shows as the test rate falling well short of what the same link does lower down.
best = max(pts.values())
at = pts[test]
ratio = at / best if best else 0.0
print()
if ratio >= 0.85:
    print("  QUALIFIED for %s: %.1f%% vs %.1f%% best on the ladder — flat, the "
          "receiver has margin and delivery reflects the TRANSMITTER."
          % (test, at, best))
    raise SystemExit(0)
print("  NOT QUALIFIED for %s: %.1f%% vs %.1f%% best on the ladder. This "
      "receiver is on its cliff at the test rate, so delivery there measures "
      "the RECEIVER, not the transmitter, and will swing hugely with ambient. "
      "Use a ground station with more margin (external antennas, distance, "
      "orientation, quieter channel) or measure a lower rate."
      % (test, at, best))
raise SystemExit(1)
EOF
