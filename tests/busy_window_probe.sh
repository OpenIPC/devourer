#!/usr/bin/env bash
# Does an ARMED busy window measure the caller's dwell, where the sampled read
# only samples it — and does a spoiled window refuse instead of lying?
#
# Two adapters on one channel: a devourer flooder and a sensor. The flooder is
# the reference, so the same load can be put to both silicon families and the
# answers compared; with DUTY_ON/DUTY_OFF it becomes a bursty interferer, which
# is where the sampled path falls apart (one ~2 ms read per dwell measured ZERO
# in 55 of 71 windows on an RTL8822BU while 300-400 frames per window were
# decoded).
#
# Arms, in order:
#   quiet      sensor alone       -- the floor. Must read ~0, not "no reading".
#   window     armed, under load  -- the feature.
#   sampled    unarmed, same load -- the spread comparison.
#   interrupt  NHM read mid-window   -- must be INVALID, spoil=interrupted.
#   retune     retune mid-window     -- must be INVALID, spoil=retuned.
#   early      read before elapsed   -- must be INVALID, spoil=not-elapsed.
#   txsess     sensor transmits      -- must stay VALID and be flagged own_tx.
#
#   sudo tests/busy_window_probe.sh
#   SENSOR_PID=0xc812 FLOOD_PID=0x8812 CHANNEL=100 sudo tests/busy_window_probe.sh
#   DUTY_ON=50 DUTY_OFF=450 sudo tests/busy_window_probe.sh    # bursty arm
#
# A MediaTek sensor needs SENSOR_RX=1 (its MAC only counts busy with the
# receiver running) and must be brought up BEFORE the flooder: its bring-up
# does not reliably complete on an already-saturated channel.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PROBE="$ROOT/build/BusyWindowProbe"
TXDEMO="$ROOT/build/txdemo"

SENSOR_VID="${SENSOR_VID:-0x0bda}"
SENSOR_PID="${SENSOR_PID:-0x8812}"   # 8812AU (Jaguar1)
FLOOD_VID="${FLOOD_VID:-0x0e8d}"
FLOOD_PID="${FLOOD_PID:-0x7612}"     # MT7612U
CHANNEL="${CHANNEL:-165}"
OTHER="${OTHER:-100}"
REPS="${REPS:-5}"
WINDOW_MS="${WINDOW_MS:-240}"
SENSOR_RX="${SENSOR_RX:-0}"
DUTY_ON="${DUTY_ON:-0}"
DUTY_OFF="${DUTY_OFF:-0}"
OUT="${OUT:-/tmp/devourer-busy-window}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$SENSOR_VID" "$SENSOR_PID" || {
  echo "SKIP: sensor $SENSOR_VID:$SENSOR_PID not plugged"; exit 77; }
plugged "$FLOOD_VID" "$FLOOD_PID" || {
  echo "SKIP: flooder $FLOOD_VID:$FLOOD_PID not plugged"; exit 77; }

mkdir -p "$OUT"
rx_flag=""; [ "$SENSOR_RX" = "1" ] && rx_flag="--rx"
fails=0
skips=0

stop_flood() { pkill -x txdemo 2>/dev/null; sleep 1; }
trap 'stop_flood' EXIT INT TERM

start_flood() {
  stop_flood
  local burst=()
  if [ "$DUTY_ON" != "0" ]; then
    burst=(DEVOURER_TX_BURST_ON_MS="$DUTY_ON" DEVOURER_TX_BURST_OFF_MS="$DUTY_OFF")
  fi
  env DEVOURER_VID="$FLOOD_VID" DEVOURER_PID="$FLOOD_PID" \
      DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE=MCS1 DEVOURER_TX_GAP_US=0 \
      "${burst[@]}" "$TXDEMO" > "$OUT/flood.log" 2>&1 &
  sleep 4
  grep -qE "tx\.(frame|stats)|bulk_send" "$OUT/flood.log" || {
    echo "WARN: flooder produced no frames — every 'under load' arm below is"
    echo "      really a quiet-channel run and proves nothing."
    fails=$((fails + 1))
  }
}

flood_frames() { # how many frames the flooder has emitted so far
  local n
  n=$(grep -cE "tx\.(frame|stats)|bulk_send" "$OUT/flood.log" 2>/dev/null)
  echo "${n:-0}"   # see expect_all: grep -c exits 1 on no match
}

# A per-arm liveness check. The one-shot grep at start_flood time cannot tell
# that the flooder DIED before the arm that needed it, and a dead flooder turns
# every "under load" assertion into a quiet-channel run that passes by
# accident.
assert_flood_alive() { # label
  local before="$1" label="$2" after
  after="$(flood_frames)"
  if [ "$after" -le "$before" ]; then
    echo "FAIL $label: flooder emitted nothing during this arm"
    fails=$((fails + 1))
  fi
}

busy_values() { # mode -> one busy percentage per valid sample
  grep -E "^BUSY mode" "$OUT/$1.log" 2>/dev/null | grep "valid=1" \
    | sed -E 's/.*busy=([0-9]+)%.*/\1/'
}

stat_of() { # mode, mean|min|max|spread
  busy_values "$1" | awk -v want="$2" '
    { n++; s += $1; if (n == 1 || $1 < lo) lo = $1; if (n == 1 || $1 > hi) hi = $1 }
    END { if (!n) { print "nan"; exit }
          if (want == "mean") printf "%.0f\n", s / n;
          else if (want == "min") print lo;
          else if (want == "max") print hi;
          else print hi - lo }'
}

assert_le() { # label, value, bound
  [ "$2" = "nan" ] && { echo "FAIL $1: no samples"; fails=$((fails + 1)); return; }
  if [ "$2" -le "$3" ]; then echo "  ok: $1 ($2 <= $3)";
  else echo "FAIL $1: $2 > $3"; fails=$((fails + 1)); fi
}

assert_ge() { # label, value, bound
  [ "$2" = "nan" ] && { echo "FAIL $1: no samples"; fails=$((fails + 1)); return; }
  if [ "$2" -ge "$3" ]; then echo "  ok: $1 ($2 >= $3)";
  else echo "FAIL $1: $2 < $3"; fails=$((fails + 1)); fi
}

run_arm() { # mode, label
  local mode="$1" label="$2"
  echo "== $label"
  timeout 120 "$PROBE" --vid "$SENSOR_VID" --pid "$SENSOR_PID" \
      --channel "$CHANNEL" --other "$OTHER" --reps "$REPS" \
      --window-ms "$WINDOW_MS" --mode "$mode" $rx_flag 2>/dev/null \
      | tee "$OUT/$mode.log" | grep -E "^BUSY"
}

# A spoiled arm must produce NO valid readings; a working arm must produce
# only valid ones. Both directions matter: a probe that passes whatever the
# hardware does is not a test.
expect_all() { # mode, valid(0|1), [spoil]
  local mode="$1" want_valid="$2" want_spoil="${3:-}"
  local n bad
  # `grep -c` prints 0 and EXITS 1 when it matches nothing, so a `|| echo 0`
  # here appended a second line and made this guard dead: an empty log then
  # reported "ok: 0 samples". The whole point of the guard is the empty case.
  n=$(grep -cE "^BUSY mode" "$OUT/$mode.log" 2>/dev/null); n=${n:-0}
  if [ "$n" -eq 0 ]; then
    echo "FAIL $mode: no samples"; fails=$((fails + 1)); return
  fi
  bad=$(grep -E "^BUSY mode" "$OUT/$mode.log" | grep -vc "valid=$want_valid")
  if [ "$bad" -ne 0 ]; then
    echo "FAIL $mode: $bad/$n samples not valid=$want_valid"
    fails=$((fails + 1))
  fi
  if [ -n "$want_spoil" ]; then
    bad=$(grep -E "^BUSY mode" "$OUT/$mode.log" | grep -vc "spoil=$want_spoil")
    [ "$bad" -eq 0 ] || {
      echo "FAIL $mode: $bad/$n samples not spoil=$want_spoil"
      fails=$((fails + 1)); }
  fi
  echo "  ok: $n samples valid=$want_valid ${want_spoil:+spoil=$want_spoil}"
}

echo "== sensor $SENSOR_VID:$SENSOR_PID, flooder $FLOOD_VID:$FLOOD_PID, ch$CHANNEL"

# The Realtek-only arms drive IRtlRadio facilities (the NHM read, the CCX
# result latch). A MediaTek sensor runs the rest.
realtek_sensor=1
[ "$SENSOR_VID" = "0x0e8d" ] && realtek_sensor=0

stop_flood
run_arm window "quiet floor (no flooder)"
mv -f "$OUT/window.log" "$OUT/quiet.log"
expect_all quiet 1   # a quiet channel is a READING of ~0, never "no reading"
quiet_mean="$(stat_of quiet mean)"
# The floor must BE a floor. Without this the whole comparison below passes
# with the sensor parked on the wrong channel.
assert_le "quiet floor is quiet" "$quiet_mean" 5

# A MediaTek sensor cannot be brought up on an already-saturated channel: its
# firmware MCU times out and the arm never becomes available (measured). The
# loaded arms therefore need the flooder started AFTER the sensor is up, which
# is one process order this script cannot express — each arm launches its own
# probe. Skipped explicitly rather than run into a failure that says nothing
# about the code, and never counted as a pass.
if [ "$realtek_sensor" != "1" ]; then
  echo "== SKIPPING the loaded arms: a MediaTek sensor must be brought up"
  echo "   BEFORE the interferer. Run those arms by hand:"
  echo "     build/BusyWindowProbe --vid $SENSOR_VID --pid $SENSOR_PID \\"
  echo "       --channel $CHANNEL --mode window --reps 45 --rx &"
  echo "     # then start the flooder once it prints its first sample"
  skips=$((skips + 1))
  echo
  [ "$fails" -eq 0 ] && echo "busy_window_probe: PASS (quiet arms only, $skips skipped)" \
                     || echo "busy_window_probe: $fails FAILURE(S)"
  exit $((fails ? 1 : 0))
fi

start_flood
f0="$(flood_frames)"
run_arm window "armed window under load"
expect_all window 1
assert_flood_alive "$f0" "window"
window_mean="$(stat_of window mean)"
# The measurement must MOVE with the load, by much more than the floor's own
# variation. An implementation returning a plausible constant fails here.
#
# The bar depends on the arm, because the load does: a saturating flooder puts
# the channel above 60%, while DUTY_ON=50/DUTY_OFF=450 is ~9% occupancy by
# construction and a 15-point bar would be asserting something untrue.
sep_min=15
[ "$DUTY_ON" != "0" ] && sep_min=5
assert_ge "load separates from floor" "$((window_mean - quiet_mean))" "$sep_min"

f0="$(flood_frames)"
run_arm sampled "shipped sampled read, same load"
expect_all sampled 1
assert_flood_alive "$f0" "sampled"

f0="$(flood_frames)"
if [ "$realtek_sensor" = "1" ]; then
  run_arm interrupt "NHM read mid-window"
  expect_all interrupt 0 interrupted
  run_arm quality "GetRxQuality() mid-window — the trap a consumer springs"
  expect_all quality 0 interrupted
  # The locking, on air: a second thread hammering GetRxQuality for the whole
  # window must never yield a SHORT window wearing a valid flag. Without the
  # CCX lock the note can land before a concurrent arm while the re-arm lands
  # after it, and a destroyed window reads back as data.
  run_arm race "GetRxQuality() hammered from another thread"
  expect_all race 0 interrupted
else
  echo "== NHM arms skipped (non-Realtek sensor has no NHM engine)"
fi

run_arm retune "retune mid-window"
expect_all retune 0 retuned

if [ "$realtek_sensor" = "1" ]; then
  # Realtek-only: the not-elapsed refusal exists because the CCX result
  # register latches the previous window. The MediaTek timers have no such
  # concept — a short arm there is simply a short, and valid, window.
  run_arm early "read before the window elapsed"
  expect_all early 0 not-elapsed
  run_arm stale "re-arm, then read before the new window elapsed"
  # The first read of each pair must be a real measurement and the second must
  # refuse. If the trigger did NOT clear the ready bit, the second read would
  # return the first window's latched value and look perfectly valid.
  grep -E "^BUSY mode=stale-1st" "$OUT/stale.log" | grep -q "valid=1" || {
    echo "FAIL stale: the completed window did not read back"; fails=$((fails + 1)); }
  bad=$(grep -E "^BUSY mode=stale " "$OUT/stale.log" | grep -vc "valid=0")
  if [ "${bad:-1}" -eq 0 ]; then
    echo "  ok: a re-armed window never returns the previous latched result"
  else
    echo "FAIL stale: $bad early reads returned a stale latched value"
    fails=$((fails + 1))
  fi
fi

f0="$(flood_frames)"
run_arm txsess "sensor transmitting inside its own window"
expect_all txsess 1
assert_flood_alive "$f0" "txsess"
grep -E "^BUSY mode" "$OUT/txsess.log" | grep -q "own_tx=1" || {
  echo "FAIL txsess: own transmission not flagged"; fails=$((fails + 1)); }
stop_flood

echo
echo "== spread: the point of the window"
for m in window sampled; do
  printf "  %-8s n=%s mean=%s%% min=%s%% max=%s%% spread=%s pts\n" \
    "$m" "$(busy_values $m | wc -l)" "$(stat_of $m mean)" "$(stat_of $m min)" \
    "$(stat_of $m max)" "$(stat_of $m spread)"
done
# The claim this feature exists for, gated rather than printed. Under a STEADY
# load both are tight, so this only means something with DUTY_ON set — hence
# the guard rather than an unconditional assert.
if [ "$DUTY_ON" != "0" ]; then
  w_spread="$(stat_of window spread)"; s_spread="$(stat_of sampled spread)"
  if [ "$w_spread" != "nan" ] && [ "$s_spread" != "nan" ]; then
    if [ "$w_spread" -lt "$s_spread" ]; then
      echo "  ok: armed spread $w_spread < sampled spread $s_spread (bursty load)"
    else
      echo "FAIL bursty: armed spread $w_spread not below sampled $s_spread"
      fails=$((fails + 1))
    fi
  fi
fi

echo
[ "$fails" -eq 0 ] && echo "busy_window_probe: PASS" || echo "busy_window_probe: $fails FAILURE(S)"
exit $((fails ? 1 : 0))
