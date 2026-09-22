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
# Arms (listed by role, not run order):
#   quiet      sensor alone       -- the floor. Must read ~0, not "no reading".
#   window     armed, under load  -- the feature.
#   sampled    unarmed, same load -- the spread comparison.
#   interrupt  NHM read mid-window   -- must be INVALID, spoil=interrupted.
#   quality    GetRxQuality() mid-window -- the same trap by the route a
#              consumer actually takes. INVALID, spoil=interrupted.
#   race       GetRxQuality() hammered from a second thread -- must never
#              yield a SHORT window wearing a valid flag.
#   retune     retune mid-window     -- must be INVALID, spoil=retuned.
#   early      read before elapsed   -- must be INVALID, spoil=not-elapsed
#              (both families: Realtek from the CCX ready bit, MediaTek from
#              the window length recorded at arm).
#   stale      re-arm, then read early -- must not return the latched result
#              of the PREVIOUS window.
#   revive     arm, Stop(), retune, read -- must carry spoil=NONE: no reason
#              may survive from the session that ended. Whether the reading is
#              VALID is backend-dependent and deliberately not asserted (a
#              Jaguar2 Stop leaves the chip live, so its sampled path answers
#              with a 2 ms window; the others tear down and report nothing).
#              Realtek only -- see the arm.
#   txsess     sensor transmits      -- must stay VALID and be flagged own_tx.
#
# Eleven arms. The skip counts below are stated against this list, so if you
# add one, add it here too or the verdict's "N arm(s) skipped" stops being
# auditable.
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
checks_skipped=0

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
  # Wait for the first frame OUT, then for the flooder to be airing at level.
  # A Jaguar2 (8822BU) txdemo does not radiate at level for ~4 s after its
  # first submit (measured with a Jaguar3 sensor armed first: 50 ms windows
  # at 0% for ~60 windows after the first bulk_send, then 61-65%). A fixed
  # 4 s sleep put the Jaguar3's first arm exactly on that edge and it read a
  # VALID 0% — which the assertions below cannot tell from a quiet channel.
  for _ in $(seq 1 100); do
    grep -qE "tx\.(frame|stats)|bulk_send" "$OUT/flood.log" && break
    sleep 0.2
  done
  sleep "${FLOOD_SETTLE_S:-6}"
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
  python3 -c "
import json,sys
for line in open(sys.argv[1]):
    line = line.strip()
    if not line.startswith('{'): continue
    try: r = json.loads(line)
    except ValueError: continue
    if r.get('ev') == 'busy.window' and r.get('valid'): print(r['busy_pct'])
" "$OUT/$1.log" 2>/dev/null
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

# Assert only the spoil REASON, ignoring validity. Some arms have a verdict
# that is the same on every backend while the reading's validity is not: a
# post-Stop read is invalid where Stop kills the chip (RTL8733B, Jaguar1/3)
# and a valid 2 ms sampled reading where it does not (Jaguar2 Stop only joins
# its runtime threads — measured). Asserting valid there would encode one
# family's teardown depth as a contract.
expect_spoil() { # mode, spoil
  local mode="$1" want="$2" n bad before="$fails"
  n=$(jq_field "$OUT/$mode.log" spoil | wc -l); n=${n:-0}
  if [ "$n" -eq 0 ]; then
    echo "FAIL $mode: no samples"; fails=$((fails + 1)); return 0
  fi
  if [ "$n" -ne "$REPS" ]; then
    echo "FAIL $mode: $n/$REPS records — the probe stopped early"
    fails=$((fails + 1)); return 0
  fi
  bad=$(jq_field "$OUT/$mode.log" spoil | grep -vc "^$want$"); bad=${bad:-ERR}
  [ "$bad" = "ERR" ] && { echo "FAIL $mode: could not read spoil reasons"
                          fails=$((fails + 1)); return 0; }
  [ "$bad" -ne 0 ] && { echo "FAIL $mode: $bad/$n samples not spoil=$want"
                        fails=$((fails + 1)); }
  [ "$fails" -eq "$before" ] && echo "  ok: $n samples spoil=$want"
  return 0
}

# The probe emits JSON Lines (ev=busy.window / busy.caps / busy.skip), so the
# assertions below read fields out of the JSON rather than scraping a bespoke
# text format.
jq_field() { # file, field -> one value per busy.window record
  python3 -c "
import json,sys
for line in open(sys.argv[1]):
    line = line.strip()
    if not line.startswith('{'): continue
    try: r = json.loads(line)
    except ValueError: continue
    if r.get('ev') == 'busy.window': print(r.get(sys.argv[2]))
" "$1" "$2"
}

run_arm() { # mode, label
  local mode="$1" label="$2"
  echo "== $label"
  timeout 120 "$PROBE" --vid "$SENSOR_VID" --pid "$SENSOR_PID" \
      --channel "$CHANNEL" --other "$OTHER" --reps "$REPS" \
      --window-ms "$WINDOW_MS" --mode "$mode" $rx_flag 2>/dev/null \
      | tee "$OUT/$mode.log" | grep -E '"ev":"busy\.'
}

# How many arms a skip branch bypasses, stated at the branch rather than
# assumed to be one. The verdict line reports this, so an undercount is a
# claim that more of the suite ran than did.
skip_arms() { skips=$((skips + $1)); }

caps_field() { # file, field -> that field from the probe's busy.caps record
  python3 -c "
import json,sys
for line in open(sys.argv[1]):
    line = line.strip()
    if not line.startswith('{'): continue
    try: r = json.loads(line)
    except ValueError: continue
    if r.get('ev') == 'busy.caps': print(r.get(sys.argv[2])); break
" "$1" "$2"
}

# A spoiled arm must produce NO valid readings; a working arm must produce
# only valid ones. Both directions matter: a probe that passes whatever the
# hardware does is not a test.
expect_all() { # mode, valid(0|1), [spoil]
  local mode="$1" want_valid="$2" want_spoil="${3:-}"
  local n bad before="$fails"
  # `grep -c` prints 0 and EXITS 1 when it matches nothing, so a `|| echo 0`
  # here appended a second line and made this guard dead: an empty log then
  # reported "ok: 0 samples". The whole point of the guard is the empty case.
  n=$(jq_field "$OUT/$mode.log" valid | wc -l); n=${n:-0}
  if [ "$n" -eq 0 ]; then
    echo "FAIL $mode: no samples"; fails=$((fails + 1)); return
  fi
  # A probe that died partway leaves fewer records than reps. Without this a
  # single surviving sample passes the whole arm — the same hole expect_spoil
  # closes, and there is no reason for the two to disagree.
  if [ "$n" -ne "$REPS" ]; then
    echo "FAIL $mode: $n/$REPS records — the probe stopped early"
    fails=$((fails + 1)); return
  fi
  local want_json="False"; [ "$want_valid" = "1" ] && want_json="True"
  # `grep -vc` on an empty stream prints 0 but a FAILED extractor prints
  # nothing at all, and `[ "" -ne 0 ]` is a bash error, not false — which
  # skipped the whole if-body and printed "ok". Default to a sentinel so a
  # broken extractor fails loudly instead of passing silently.
  bad=$(jq_field "$OUT/$mode.log" valid | grep -vc "^$want_json$"); bad=${bad:-ERR}
  if [ "$bad" = "ERR" ]; then
    echo "FAIL $mode: could not read the probe's records"
    fails=$((fails + 1)); return
  fi
  if [ "$bad" -ne 0 ]; then
    echo "FAIL $mode: $bad/$n samples not valid=$want_valid"
    fails=$((fails + 1))
  fi
  if [ -n "$want_spoil" ]; then
    bad=$(jq_field "$OUT/$mode.log" spoil | grep -vc "^$want_spoil$"); bad=${bad:-ERR}
    [ "$bad" = "ERR" ] && { echo "FAIL $mode: could not read spoil reasons"
                            fails=$((fails + 1)); return; }
    [ "$bad" -eq 0 ] || {
      echo "FAIL $mode: $bad/$n samples not spoil=$want_spoil"
      fails=$((fails + 1)); }
  fi
  # Only when nothing above failed. This line used to print unconditionally,
  # so a failing arm reported its FAILs and then said "ok" underneath them.
  [ "$fails" -eq "$before" ] &&
    echo "  ok: $n samples valid=$want_valid ${want_spoil:+spoil=$want_spoil}"
  return 0
}

echo "== sensor $SENSOR_VID:$SENSOR_PID, flooder $FLOOD_VID:$FLOOD_PID, ch$CHANNEL"

stop_flood
run_arm window "quiet floor (no flooder)"
mv -f "$OUT/window.log" "$OUT/quiet.log"

# EVERY per-backend decision below is read from the probe's own caps record,
# not from the sensor's USB VID. The VID cannot answer any of these questions:
# Kestrel ships under 0x0bda/0x0586/0x0b05, MediaTek ships under nine OEM
# VIDs beyond 0x0e8d, and 0x0b05, 0x2c4e and 0x7392 appear in BOTH tables —
# so no VID test can even separate those two. A "not MediaTek" gate therefore called a Kestrel
# Realtek and ran arms it cannot pass, and called an OEM-VID MediaTek Realtek
# and ran the loaded arms its bring-up cannot survive.
sensor_gen="$(caps_field "$OUT/quiet.log" generation)"
busy_cap="$(caps_field "$OUT/quiet.log" busy_airtime_ok)"

# busy_airtime_ok says the backend HAS an engine, not that an arm will
# succeed right now — a MediaTek with RX off and a Realtek before bring-up
# both report true and refuse. That is fine here: the flag answers "is this
# harness in scope", and the quiet arm's own assertion catches an engine that
# exists but cannot arm.
#
# No busy-airtime engine at all (Kestrel) means this harness does not apply.
# Say so once and leave, rather than emitting a failure per arm for hardware
# that was never in scope.
if [ "$busy_cap" = "False" ]; then
  echo "SKIP: $sensor_gen has no busy-airtime engine (busy_airtime_ok=false),"
  echo "      so there is no armed window for this harness to measure."
  exit 77
fi
# "Realtek" here means "not the backend whose Stop() closes the device and
# whose bring-up cannot survive a saturated channel" — a generation fact, and
# the caps record carries the generation by name.
realtek_sensor=1
[ "$sensor_gen" = "mt7612u" ] && realtek_sensor=0

if [ "$busy_cap" != "True" ]; then
  echo "FAIL caps: busy_airtime_ok not readable from the probe (got '$busy_cap')"
  echo "     — refusing to decide which arms apply from a caps record this"
  echo "       harness cannot parse."
  fails=$((fails + 1))
fi

expect_all quiet 1   # a quiet channel is a READING of ~0, never "no reading"
quiet_mean="$(stat_of quiet mean)"
# The floor must BE a floor. Without this the whole comparison below passes
# with the sensor parked on the wrong channel.
assert_le "quiet floor is quiet" "$quiet_mean" 5

# Which of the arms below apply is a CAPABILITY question, not a chip-identity
# one. The NHM arms spoil a window by driving GetRxQuality -> GetRxEnergy, and
# the sampled arm reads the path GetRxEnergy feeds. A backend that implements
# no GetRxEnergy has nothing to interrupt the window WITH, and its sampled
# path reports no reading by design — the RTL8733B is exactly that: Realtek,
# CCX CLM working, no phydm FA/CCA block. A VID test calls it Realtek and runs
# arms it cannot pass, which is the same wrong discriminator AdapterCaps.h
# documents ("a successful dynamic_cast<IRtlRadio*> is not a correct
# discriminator and never was"). Ask the caps instead.
nhm_sensor=0
rx_energy_cap="$(caps_field "$OUT/quiet.log" rx_energy_ok)"
case "$rx_energy_cap" in
  True)  nhm_sensor=1 ;;
  False) nhm_sensor=0 ;;
  *)
    # Anything else means no caps record, or the field was renamed. Quietly
    # treating that as "no GetRxEnergy" would skip the sampled arm, all three
    # NHM arms AND the bursty comparison, and the run would still print PASS
    # — a gate that switches its own tests off the moment it stops working.
    # The arms it guards are the ones that catch a spoiled window being
    # reported as data, so failing loudly is the only safe default.
    echo "FAIL caps: rx_energy_ok not readable from the probe (got '$rx_energy_cap')"
    echo "     — refusing to decide which arms apply from a caps record this"
    echo "       harness cannot parse."
    fails=$((fails + 1))
    ;;
esac

# A MediaTek sensor cannot be brought up on an already-saturated channel: its
# firmware MCU times out and the arm never becomes available (measured). The
# loaded arms therefore need the flooder started AFTER the sensor is up, which
# is one process order this script cannot express — each arm launches its own
# probe. Skipped explicitly rather than run into a failure that says nothing
# about the code, and never counted as a pass.
# These need no interferer, so they run on every family — and on the MediaTek
# they are the only automated on-air cover for the armed-window rules this
# change adds there.
run_arm retune "retune mid-window (no load needed)"
expect_all retune 0 retuned
run_arm early "read before the window elapsed (no load needed)"
expect_all early 0 not-elapsed
# The window must not outlive its own hardware session. Stop() ends the
# session, but a retune re-runs bring-up, so a backend that forgets to reset
# the window there revives it and answers with a spoil reason earned by a
# session that no longer exists. spoil=none is the assertion that matters:
# valid=0 alone passes either way.
# MediaTek is excluded: its Stop() closes the device and nulls the handle, so
# the arm cannot retune afterwards — measured, the probe wedges rather than
# reporting. This is a Realtek lifecycle rule and is checked where it applies.
# Gated on the reported GENERATION, not on a capability: there is no cap for
# "Stop() is survivable", so this one exclusion stays a lifecycle fact. The
# two cases: MediaTek's Stop() nulls the device handle, leaving no retune
# path (measured: the probe wedges), and with SENSOR_RX=1 the Realtek Init
# runs on a detached thread that Stop() would be torn down underneath.
if [ "$realtek_sensor" = "1" ] && [ "$SENSOR_RX" != "1" ]; then
  run_arm revive "arm, Stop(), retune, read (no load needed)"
  # SPOIL only. Whether the post-Stop read is valid depends on how deeply that
  # backend's Stop tears the chip down; what must hold everywhere is that no
  # spoil reason survives from the session that ended.
  expect_spoil revive none
else
  echo "== revive arm skipped: needs a Realtek sensor with the RX loop OFF —"
  echo "   MediaTek's Stop() closes the device, and under SENSOR_RX=1 the"
  echo "   Init thread would be torn down underneath."
  skip_arms 1
fi

if [ "$realtek_sensor" != "1" ]; then
  echo "== SKIPPING the loaded arms: a MediaTek sensor must be brought up"
  echo "   BEFORE the interferer. Run those arms by hand:"
  echo "     build/BusyWindowProbe --vid $SENSOR_VID --pid $SENSOR_PID \\"
  echo "       --channel $CHANNEL --mode window --reps 45 --rx &"
  echo "     # then start the flooder once it prints its first sample"
  # window, sampled, interrupt, quality, race, stale, txsess — everything
  # below this exit. Four of them would be skipped by their own branches on
  # this backend anyway, but "did not run" is what the verdict claims, so
  # they are counted here.
  skip_arms 7
  echo
  [ "$fails" -eq 0 ] && echo "busy_window_probe: PASS (quiet arms only, $skips arm(s) skipped)" \
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
if [ "$window_mean" = "nan" ] || [ "$quiet_mean" = "nan" ]; then
  echo "FAIL load separates from floor: an arm produced no valid samples"
  fails=$((fails + 1))
else
  assert_ge "load separates from floor" "$((window_mean - quiet_mean))" "$sep_min"
fi

if [ "$nhm_sensor" = "1" ]; then
  f0="$(flood_frames)"
  run_arm sampled "shipped sampled read, same load"
  expect_all sampled 1
  assert_flood_alive "$f0" "sampled"
else
  echo "== sampled arm skipped: this backend implements no GetRxEnergy, so"
  echo "   the sampled path reports NO READING by design — the armed window"
  echo "   is the only way a number comes out of it."
  skip_arms 1
fi

f0="$(flood_frames)"
if [ "$nhm_sensor" = "1" ]; then
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
  echo "== NHM arms skipped: no GetRxEnergy on this backend, so there is no"
  echo "   NHM read that could re-arm the shared engine mid-window."
  # interrupt, quality, race — three arms, not one.
  skip_arms 3
fi

# No guard: the MediaTek early exit above is unconditional, so every path
# that reaches here is one that can arm. This arm's skip is already counted
# in that exit's total.
run_arm stale "re-arm, then read before the new window elapsed"
  # The first read of each pair must be a real measurement and the second must
  # refuse. If the trigger did NOT clear the ready bit, the second read would
  # return the first window's latched value and look perfectly valid.
  python3 -c "
import json,sys
ok = any(json.loads(l).get('mode') == 'stale-1st' and json.loads(l).get('valid')
         for l in open(sys.argv[1]) if l.strip().startswith('{'))
sys.exit(0 if ok else 1)
" "$OUT/stale.log" || {
    echo "FAIL stale: the completed window did not read back"; fails=$((fails + 1)); }
  # Count the second reads too: if the re-arm was refused the probe emits
  # busy.skip and exits, leaving ZERO stale records — and "no bad records"
  # would otherwise read as a pass.
  read -r n_stale bad <<EOF
$(python3 -c "
import json,sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip().startswith('{')]
stale = [r for r in rows if r.get('mode') == 'stale']
print(len(stale), sum(1 for r in stale if r.get('valid')))
" "$OUT/stale.log")
EOF
  if [ "${n_stale:-0}" -eq 0 ]; then
    echo "FAIL stale: no second-read records (was the re-arm refused?)"
    fails=$((fails + 1))
  elif [ "${bad:-1}" -eq 0 ]; then
    echo "  ok: a re-armed window never returns the previous latched result"
  else
    echo "FAIL stale: $bad early reads returned a stale latched value"
    fails=$((fails + 1))
  fi

f0="$(flood_frames)"
run_arm txsess "sensor transmitting inside its own window"
expect_all txsess 1
assert_flood_alive "$f0" "txsess"
jq_field "$OUT/txsess.log" own_tx | grep -q "^True$" || {
  echo "FAIL txsess: own transmission not flagged"; fails=$((fails + 1)); }
stop_flood

echo
echo "== spread: the point of the window"
arms="window"; [ "$nhm_sensor" = "1" ] && arms="window sampled"
for m in $arms; do
  printf "  %-8s n=%s mean=%s%% min=%s%% max=%s%% spread=%s pts\n" \
    "$m" "$(busy_values $m | wc -l)" "$(stat_of $m mean)" "$(stat_of $m min)" \
    "$(stat_of $m max)" "$(stat_of $m spread)"
done
# The claim this feature exists for, gated rather than printed. Under a STEADY
# load both are tight, so this only means something with DUTY_ON set — hence
# the guard rather than an unconditional assert. It also needs a sampled arm
# to compare against, which a backend without GetRxEnergy does not have.
if [ "$DUTY_ON" != "0" ] && [ "$nhm_sensor" != "1" ]; then
  echo "  (armed-vs-sampled comparison needs a sampled path; this backend has"
  echo "   none, so the bursty claim is not testable on it)"
  # A check, not an arm: counted separately so "arm(s) skipped" stays true.
  checks_skipped=$((checks_skipped + 1))
elif [ "$DUTY_ON" != "0" ]; then
  # Comparing spreads alone is fragile at small REPS: on a ~10% duty channel a
  # handful of 2 ms reads can ALL miss the burst, which looks like a spread of
  # zero while being entirely wrong. The claim is that the sampled estimator is
  # BIMODAL — it either misses the load or catches a burst and reports it as
  # the channel — so test that: the armed window must see the load, and the
  # sampled arm must give itself away either by reporting an empty channel at
  # least once or by spreading wider than the armed one.
  w_mean="$(stat_of window mean)"; w_spread="$(stat_of window spread)"
  s_spread="$(stat_of sampled spread)"
  s_zeros="$(busy_values sampled | grep -c '^0$')"; s_zeros=${s_zeros:-0}
  if [ "$w_mean" = "nan" ] || [ "$s_spread" = "nan" ]; then
    echo "FAIL bursty: an arm produced no valid samples"
    fails=$((fails + 1))
  elif [ "$w_mean" -le 0 ]; then
    echo "FAIL bursty: the armed window did not see the load (mean $w_mean%)"
    fails=$((fails + 1))
  elif [ "$s_zeros" -ge 1 ] || [ "$s_spread" -gt "$w_spread" ]; then
    echo "  ok: armed mean $w_mean% tracks the load; sampled missed it"\
         "$s_zeros/$(busy_values sampled | wc -l) times, spread $s_spread vs $w_spread"
  else
    echo "FAIL bursty: the sampled read neither missed the load nor spread"\
         "wider than the armed window — the premise does not hold here"
    fails=$((fails + 1))
  fi
fi

echo
# The skip count belongs in the verdict line, not only in the per-arm chatter
# above: a backend without GetRxEnergy legitimately skips four arms, and a
# reader scraping the last line for PASS would otherwise see the same word for
# "every arm ran" and "two thirds of them did not".
if [ "$fails" -ne 0 ]; then
  echo "busy_window_probe: $fails FAILURE(S)"
elif [ "$skips" -ne 0 ] || [ "$checks_skipped" -ne 0 ]; then
  echo "busy_window_probe: PASS ($skips arm(s), $checks_skipped check(s) skipped"\
       "— see above for why)"
else
  echo "busy_window_probe: PASS"
fi
exit $((fails ? 1 : 0))
