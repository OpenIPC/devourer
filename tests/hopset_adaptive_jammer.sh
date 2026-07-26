#!/usr/bin/env bash
# Receiver-driven adaptive-exclusion validation against a real interferer.
#
# Three radios: a TX (schedule authority), an RX (lockstep follower running
# the exclusion policy), and a B210 parked on ONE base-hopset channel
# (tests/sdr_interferer.py). The receiver must observe the jammed channel,
# propose its exclusion, the authority must commit it, both must activate in
# lockstep, and delivery on the remaining set must improve. Killing the
# jammer must then let the keyed recovery probes restore the channel.
#
#   MODE=sense    parked, with transmitter sensing armed under the default
#                 fusion mode: the receiver is right and the transmitter must
#                 not contradict it (the #264 result must be reproduced)
#   MODE=failsafe the receiver's uplink is muted, so nothing adapts under its
#                 authority; the transmitter must carry the link on its own
#                 evidence and the receiver must follow without split-brain
#   MODE=prepost  a REACTIVE follower jammer with both sense windows armed in
#                 one run: it keys up only after it hears us, so it should be
#                 invisible before a burst and visible after one
#   MODE=overhead what the quiet windows cost, sensing off versus on
#   MODE=parked   (default) exclude -> improve -> restore after the jam ends
#   MODE=herding  the jammer moves onto a newly-active channel after every
#                 exclusion; the diversity floor must hold and the link must
#                 never oscillate
#
# Usage: sudo -v && tests/hopset_adaptive_jammer.sh
#        MODE=herding CHANNELS=1,6,11,3,8 tests/hopset_adaptive_jammer.sh
#
# Cleanup: on any exit (incl. Ctrl-C) both demos and the SDR are killed by
# exact comm / exact arg — never a broad pattern.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$ROOT/tests"

MODE="${MODE:-parked}"
VID="${VID:-0x0bda}"
# The two endpoints need not be the same vendor id: an OEM-rebadged part (the
# 8812BU ships as TP-Link 2357:012d) enumerates outside Realtek's own space.
TX_VID="${TX_VID:-$VID}"
# Both endpoints must be able to work this band, and on 2.4 GHz that rules the
# 8812EU out of BOTH roles: it is a 5 GHz-only PA part, so it cannot get a
# proposal out as the receiver (its proposals simply time out) and its 2.4 GHz
# front end shows no jammed-versus-clean separation as the sensing transmitter.
# Measured here against a narrowband interferer that destroys delivery on one
# channel, the 8812CU separates by roughly five to one whether receiving or
# transmitting. None of that is a statement about the 8812EU die — on 5 GHz it
# is the part with the front end; it is the wrong part for THIS band.
TX_PID="${TX_PID:-0xc812}"      # RTL8812CU (Jaguar3) — senses and transmits
RX_VID="${RX_VID:-0x2357}"      # TP-Link OEM vendor id
RX_PID="${RX_PID:-0x012d}"      # RTL8812BU (Jaguar2) — 2.4 GHz capable both ways
# Four ADJACENT BUT NON-OVERLAPPING 2.4 GHz channels: 1/5/9/13 sit on 2412,
# 2432, 2452, 2472 — 20 MHz apart, so their passbands abut without overlapping.
# The interferer has to degrade ONE hopset member and leave the rest healthy;
# the usual 1/6/11 set has only three members (no room above the max(3, ...)
# diversity floor), and any closer spacing spills into the neighbours, which
# the policy correctly reads as broad degradation and refuses to act on.
#
# 2.4 GHz and not 5 GHz because this bench's SDR antenna is 2.4-tuned: measured
# here, a 5 GHz interferer at max TX gain (85 dB) leaves a near-field link at
# 100% delivery on every channel, while 2.4 GHz bites hard. Keep the jammer
# NARROWBAND (JAM_RATE) so it stays inside the one channel it is aimed at.
CHANNELS="${CHANNELS:-1,5,9,13}"
JAM_CHANNEL="${JAM_CHANNEL:-5}"   # must be a member of CHANNELS
JAM_RATE="${JAM_RATE:-5e6}"
NO_JAM="${NO_JAM:-0}"             # 1 = rig sanity run, no interferer at all
SLOT_MS="${SLOT_MS:-50}"
SEED="${SEED:-a5a5c3c3f00d1234deadbeef00c0ffee}"
PROBE_ROUNDS="${PROBE_ROUNDS:-4}"
# Bench setpoint, measured with the 5 MHz narrowband interferer and the roles
# above (8812CU transmitting, 8812BU receiving): 88 dB puts the jammed channel
# at 0.22 delivery — under the 0.30 exclusion floor — and leaves every other
# hopset member at 1.00. The sweep that found it: 50 dB gave 0.885, 65 gave
# 0.708, 78 gave 0.312.
#
# The setpoint is specific to the geometry, not just the gain. With the roles
# reversed the same profile appeared at 50 dB, and at 65 dB it splattered
# across the whole set (0.09 / 0.24 / 0.61), which is broad degradation rather
# than a channel fault and the policy correctly refuses to act on it. Swap the
# adapters or move an antenna and this needs re-deriving: NO_JAM=1 for a
# sanity run, then a short sweep reading the per-channel table.
JAM_GAIN="${JAM_GAIN:-88}"
# UHD device selection. Empty = let tests/uhd_select.py resolve it (env, then
# the untracked tests/.uhd_args, then the only device present — and a hard
# error rather than a coin flip when a bench has more than one radio).
SDR_ARGS="${SDR_ARGS-}"
TX_PWR="${TX_PWR-}"              # flat TXAGC index; empty = efuse default
SENSE="${SENSE:-0}"              # DEVOURER_TX_SENSE
FUSION="${FUSION:-veto}"         # rx | veto | either | failsafe
SENSE_WINDOW_US="${SENSE_WINDOW_US:-4000}"
SENSE_POSTBURST_US="${SENSE_POSTBURST_US:-0}"
SENSE_EVERY="${SENSE_EVERY:-4}"
# The false-alarm/CCA counters count PACKET DETECTIONS — they fire on 802.11
# preambles, not on band-limited noise. Measured here: with a 50 dB AWGN
# interferer parked on a channel the receiver cannot decode at all, the
# transmitter's CCA and FA read exactly zero on that channel and IGI barely
# leaves its floor. NHM is a power histogram rather than an event counter, so
# it is the instrument that actually sees this interferer — at the cost of a
# ~3 ms armed window per read.
SENSE_NHM="${SENSE_NHM:-0}"
SENSE_FLOOR="${SENSE_FLOOR:-0.80}"   # absolute delivery floor for MODE=sense
BASELINE_S="${BASELINE_S:-25}"
ADAPT_S="${ADAPT_S:-70}"
RESTORE_S="${RESTORE_S:-70}"
OVH_S="${OVH_S:-30}"
OUT="${OUT:-/tmp/devourer-hopset-jammer}"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$TX_VID" "$TX_PID" || { echo "SKIP: TX $TX_VID:$TX_PID not plugged"; exit 77; }
plugged "$RX_VID" "$RX_PID" || { echo "SKIP: RX $RX_VID:$RX_PID not plugged"; exit 77; }
SDR_SEL="$(python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} 2>/dev/null)" || {
    python3 "$HERE/uhd_select.py" ${SDR_ARGS:+"$SDR_ARGS"} >&2
    exit 1; }
uhd_find_devices ${SDR_SEL:+--args "$SDR_SEL"} >/dev/null 2>&1 || {
    echo "SKIP: no UHD device matching '${SDR_SEL:-any}'"; exit 77; }

# In-tree rtw88/rtw89 auto-probe the dongles; temp-unbind any they hold.
unbind() {
    local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do
        p=$(cat "$d" 2>/dev/null) || continue
        [ "$p" = "${pid#0x}" ] || continue
        for i in "$(dirname "$d")":*; do
            [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
        done
    done
}

fails=0
require() { # file, needle, label
    if grep -qF "$2" "$1"; then echo "PASS: $3"; else echo "FAIL: $3"; fails=$((fails+1)); fi
}

kill_jammer() {
    sudo pkill -f "sdr_interferer.py" 2>/dev/null || true
    sudo pkill -f "sdr_follower_jammer.py" 2>/dev/null || true
}
cleanup() {
    kill_jammer
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    sleep 1
    sudo pkill -x txdemo 2>/dev/null || true
    sudo pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== build =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1
unbind "$TX_PID"; unbind "$RX_PID"
mkdir -p "$OUT"; rm -f "$OUT"/*.log "$OUT"/*.jsonl

# uv venv for the SDR script (system site-packages for the uhd bindings)
if [ ! -d "$HERE/.venv" ]; then uv venv --system-site-packages "$HERE/.venv" >/dev/null 2>&1 || true; fi
PY="$HERE/.venv/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"
"$PY" -c "import uhd" 2>/dev/null || PY="$(command -v python3)"

start_jammer() { # channel
    [ "$NO_JAM" = "1" ] && return 0
    kill_jammer
    sudo "$PY" "$HERE/sdr_interferer.py" ${SDR_SEL:+--args "$SDR_SEL"} \
        --channel "$1" --tx-gain "$JAM_GAIN" --rate "$JAM_RATE" \
        >>"$OUT/jammer.log" 2>&1 &
    sleep 2
}

COMMON=(DEVOURER_HOP_CHANNELS="$CHANNELS" DEVOURER_HOP_SLOT_MS="$SLOT_MS"
        DEVOURER_HOP_SEED="$SEED" DEVOURER_HOP_ADAPTIVE=1
        DEVOURER_HOP_PROBE_ROUNDS="$PROBE_ROUNDS" DEVOURER_LOG_LEVEL=info)

start_tx() { # logfile
    # TX_PWR="" leaves the efuse-calibrated default alone. A flat TXAGC index
    # is per-band on these dies — an index that merely backs 5 GHz off can put
    # 2.4 GHz below the receiver's sensitivity and look like a dead rig.
    local pwr=() sense=()
    [ -n "$TX_PWR" ] && pwr=(DEVOURER_TX_PWR="$TX_PWR")
    [ "$SENSE" = "1" ] && sense=(DEVOURER_TX_SENSE=1
        DEVOURER_TX_SENSE_WINDOW_US="$SENSE_WINDOW_US"
        DEVOURER_TX_SENSE_POSTBURST_US="$SENSE_POSTBURST_US"
        DEVOURER_TX_SENSE_EVERY="$SENSE_EVERY"
        DEVOURER_TX_SENSE_NHM="$SENSE_NHM"
        DEVOURER_HOP_FUSION="$FUSION")
    sudo env "${COMMON[@]}" "${pwr[@]}" "${sense[@]}" DEVOURER_VID="$TX_VID" \
        DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="${CHANNELS%%,*}" \
        DEVOURER_HOP_FAST=1 DEVOURER_TX_WITH_RX=thread \
        "$ROOT/build/txdemo" >"$1" 2>&1 &
}
start_rx() { # logfile, policy(0|1)
    local pol=()
    [ "$2" = "1" ] && pol=(DEVOURER_HOP_POLICY=1 DEVOURER_HOP_POLICY_EVENTS=2)
    [ "$MODE" = "failsafe" ] && pol+=(DEVOURER_HOP_MUTE=1)
    sudo env "${COMMON[@]}" "${pol[@]}" DEVOURER_VID="$RX_VID" \
        DEVOURER_PID="$RX_PID" "$ROOT/build/rxdemo" >"$1" 2>&1 &
}

# Delivery proxy, straight from the hop.rx event stream: a dwell counts as
# delivered when a marker decoded before the next retune. This is exactly what
# the policy scores, so the two must agree.
delivery() { # logfile
    awk -F'"' '/"ev":"hop.rx"/ {
        for (i = 1; i < NF; i++) if ($i == "state") s = $(i+2);
        if (s == "retune") r++; if (s == "decode") d++;
    } END { if (r) printf "%.3f", d / r; else print "0" }' "$1"
}

# Per-channel delivery — the diagnostic that tells "one channel is jammed"
# apart from "the whole band is desensed" (a too-hot interferer near-fields
# the receiver on every channel and the policy correctly refuses to act).
per_channel() { # logfile
    awk -F'"' '
        /"ev":"hop.rx"/ {
            s = ""; ch = "";
            for (i = 1; i < NF; i++) {
                if ($i == "state") s = $(i+2);
                if ($i == "channel") { v = $(i+1); gsub(/[:,}]/, "", v); ch = v+0 }
            }
            if (s == "retune") { cur = ch; tot[ch]++; pend = 1 }
            else if (s == "decode" && pend) { dec[cur]++; pend = 0 }
        }
        END {
            n = asorti(tot, idx);
            for (j = 1; j <= n; j++) {
                c = idx[j];
                printf "      ch%-4s %5d dwells  delivery %.3f\n",
                       c, tot[c], (tot[c] ? dec[c] / tot[c] : 0);
            }
        }' "$1" 2>/dev/null || true
}

# --- modes that do not use the parked interferer at all ---
if [ "$MODE" = "prepost" ]; then
    # Both windows armed in ONE run, not two runs of one window each: a
    # following jammer's behaviour depends on its own detection of each burst,
    # so across two runs its variance would be indistinguishable from the
    # effect being measured. In one run the two phases sample the same dwells.
    echo "== reactive follower jammer (it chases whatever it hears) =="
    sudo "$PY" "$HERE/sdr_follower_jammer.py" ${SDR_SEL:+--args "$SDR_SEL"} \
        --channels "$CHANNELS" --tx-gain "$JAM_GAIN" \
        --secs "$((ADAPT_S + 10))" --log "$OUT/follow.jsonl" \
        >>"$OUT/jammer.log" 2>&1 &
    sleep 4
    start_rx "$OUT/rx_prepost.log" 0
    sleep 2
    SENSE=1 SENSE_EVERY=1 start_tx "$OUT/tx_prepost.log"
    sleep "$ADAPT_S"
    sudo pkill -INT -x txdemo 2>/dev/null || true
    sudo pkill -INT -x rxdemo 2>/dev/null || true
    kill_jammer
    sleep 2
    echo "== verdicts (logs: $OUT) =="
    mean_occ() { grep -F "\"phase\":\"$2\"" "$1" 2>/dev/null |
        grep -oE '"occ":[0-9.]+' | cut -d: -f2 |
        awk '{n++; s+=$1} END { printf "%.4f", (n ? s/n : 0) }'; }
    count_ph() { grep -cF "\"phase\":\"$2\"" "$1" 2>/dev/null || echo 0; }
    pre_m=$(mean_occ "$OUT/tx_prepost.log" pre)
    post_m=$(mean_occ "$OUT/tx_prepost.log" post)
    retunes=$(grep -c '"ev": "follow.jam"' "$OUT/follow.jsonl" 2>/dev/null || echo 0)
    echo "   follower retunes seen: $retunes"
    echo "   pre-burst  n=$(count_ph "$OUT/tx_prepost.log" pre)  mean occupancy $pre_m"
    echo "   post-burst n=$(count_ph "$OUT/tx_prepost.log" post)  mean occupancy $post_m"
    require "$OUT/tx_prepost.log" '"phase":"pre"'  "pre-burst samples taken"
    require "$OUT/tx_prepost.log" '"phase":"post"' "post-burst samples taken"
    [ "${retunes:-0}" -gt 0 ] && echo "PASS: the follower was actually chasing" ||
        { echo "FAIL: the follower never retuned — nothing was measured"
          fails=$((fails+1)); }
    # Not a pass/fail: whether post exceeds pre depends on whether this jammer
    # LOCKS ON after detecting us or merely sweeps. A sweeper is on our channel
    # the same fraction of the time in both windows, so the two read alike, and
    # that is a fact about the adversary rather than about the sensor.
    awk -v a="$post_m" -v b="$pre_m" 'BEGIN { exit !(a > b * 1.2) }' &&
        echo "NOTE: post-burst clearly exceeds pre-burst ($pre_m -> $post_m) — this jammer locks on" ||
        echo "NOTE: post-burst does not exceed pre-burst ($pre_m -> $post_m) — this jammer sweeps rather than locking on, so both windows see it equally"
    [ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
    exit "$fails"
fi

if [ "$MODE" = "overhead" ]; then
    # A clean channel on purpose: an interferer adds carrier-sense deferral
    # variance that has nothing to do with what the quiet windows cost.
    NO_JAM=1
    run_ovh() { # label, sense
        start_rx "$OUT/rx_ovh_$1.log" 0
        sleep 2
        SENSE="$2" start_tx "$OUT/tx_ovh_$1.log"
        sleep "$OVH_S"
        sudo pkill -INT -x txdemo 2>/dev/null || true
        sudo pkill -INT -x rxdemo 2>/dev/null || true
        sleep 2
    }
    fps() { grep -F '"ev":"tx.stats"' "$1" 2>/dev/null | tail -1 |
            sed -n 's/.*"submitted":\([0-9]*\).*/\1/p' |
            awk -v t="$OVH_S" '{printf "%.1f", $1/t}'; }
    echo "== overhead: sensing off =="; run_ovh off 0
    echo "== overhead: sensing on  =="; run_ovh on 1
    echo "== verdicts (logs: $OUT) =="
    f_off=$(fps "$OUT/tx_ovh_off.log"); f_on=$(fps "$OUT/tx_ovh_on.log")
    # What the windows themselves claim to have cost, from their own stamps.
    acct=$(grep -F '"ev":"hopset.sense"' "$OUT/tx_ovh_on.log" 2>/dev/null |
        grep -oE '"(window_us|settle_us|read_us)":[0-9]+' | cut -d: -f2 |
        awk -v t="$OVH_S" '{s+=$1} END{printf "%.1f", 100*s/(t*1e6)}')
    meas=$(awk -v a="$f_off" -v b="$f_on" 'BEGIN{printf "%.1f", (a>0)?100*(1-b/a):0}')
    echo "   frames/s   off=$f_off  on=$f_on"
    echo "   overhead   measured ${meas}%   accounted ${acct}%"
    awk -v m="$meas" -v b="${OVH_BOUND:-25}" 'BEGIN { exit !(m <= b) }' &&
        echo "PASS: measured overhead ${meas}% within the ${OVH_BOUND:-25}% bound" ||
        { echo "FAIL: overhead ${meas}% exceeds ${OVH_BOUND:-25}%"; fails=$((fails+1)); }
    # A large disagreement means something is being paid that nothing recorded.
    awk -v m="$meas" -v a="$acct" 'BEGIN { d=m-a; if (d<0) d=-d; exit !(d <= 8) }' &&
        echo "PASS: measured and accounted agree within 8 points" ||
        echo "NOTE: measured ${meas}% vs accounted ${acct}% — unrecorded cost worth chasing"
    [ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
    exit "$fails"
fi

echo "== jammer parked on ch$JAM_CHANNEL (gain $JAM_GAIN) =="
start_jammer "$JAM_CHANNEL"

echo "== phase A: fixed keyed hopset (policy off), ${BASELINE_S}s =="
start_rx "$OUT/rx_baseline.log" 0
sleep 3
start_tx "$OUT/tx_baseline.log"
sleep "$BASELINE_S"
sudo pkill -INT -x rxdemo 2>/dev/null || true
sudo pkill -INT -x txdemo 2>/dev/null || true
sleep 2
BASE_DELIV=$(delivery "$OUT/rx_baseline.log")

echo "== phase B: receiver-driven exclusion (policy on), ${ADAPT_S}s =="
start_rx "$OUT/rx_adapt.log" 1
sleep 3
start_tx "$OUT/tx_adapt.log"

if [ "$MODE" = "herding" ]; then
    # Follow every exclusion: park the jammer on the lowest channel that is
    # still active according to the receiver's own committed mask.
    IFS=',' read -r -a CHLIST <<<"$CHANNELS"
    end=$((SECONDS + ADAPT_S))
    last_mask=""
    while [ "$SECONDS" -lt "$end" ]; do
        sleep 5
        mask=$(grep -F '"ev":"hopset.activate"' "$OUT/rx_adapt.log" 2>/dev/null |
               tail -1 | sed -n 's/.*"mask":"0x\([0-9a-f]*\)".*/\1/p')
        [ -n "$mask" ] && [ "$mask" != "$last_mask" ] || continue
        last_mask="$mask"
        dec=$((16#$mask))
        for i in "${!CHLIST[@]}"; do
            if (( (dec >> i) & 1 )); then
                echo "   herding: mask 0x$mask -> jam ch${CHLIST[$i]}"
                start_jammer "${CHLIST[$i]}"
                break
            fi
        done
    done
else
    sleep "$ADAPT_S"
fi
ADAPT_DELIV=$(delivery "$OUT/rx_adapt.log")

if [ "$MODE" = "parked" ] || [ "$MODE" = "sense" ] || [ "$MODE" = "failsafe" ]; then
    # The failsafe needs this phase as much as the receiver-driven modes do:
    # an autonomous exclusion that can never be undone is a link that shrinks
    # permanently the first time something transient sits on a channel. With
    # the receiver still mute, the only evidence that can bring the channel
    # back is the transmitter's own keyed probes.
    echo "== phase C: jammer off — probes must restore the channel, ${RESTORE_S}s =="
    kill_jammer
    : > "$OUT/rx_restore_mark"
    sleep "$RESTORE_S"
fi
sudo pkill -INT -x rxdemo 2>/dev/null || true
sudo pkill -INT -x txdemo 2>/dev/null || true
sleep 2

echo "== verdicts (logs: $OUT) =="
jam_idx=$(awk -v c="$JAM_CHANNEL" -F, '{for(i=1;i<=NF;i++) if($i==c) print i-1}' <<<"$CHANNELS")

echo "   baseline delivery=$BASE_DELIV   adaptive delivery=$ADAPT_DELIV"
echo "   per-channel, baseline (policy off):"
per_channel "$OUT/rx_baseline.log"
echo "   per-channel, adaptive (policy on):"
per_channel "$OUT/rx_adapt.log"
if [ "$NO_JAM" = "1" ]; then
    echo "   NO_JAM=1 — rig sanity run only, no exclusion expected"
    [ "$(awk -v d="$BASE_DELIV" 'BEGIN{print (d > 0.5) ? 1 : 0}')" = "1" ] &&
        echo "== RIG OK (baseline delivery $BASE_DELIV) ==" ||
        echo "== RIG BAD: link does not deliver without an interferer =="
    exit 0
fi
if [ "$MODE" = "failsafe" ]; then
    echo "NOTE: receiver is muted in this mode — it decides nothing by design"
else
    require "$OUT/rx_adapt.log" '"ev":"hopset.decision","t"' "policy emitted decisions"
    if grep -F '"ev":"hopset.decision"' "$OUT/rx_adapt.log" | grep -qF '"kind":"exclude"'; then
        echo "PASS: receiver proposed an exclusion"
    else
        echo "FAIL: receiver proposed an exclusion"; fails=$((fails+1))
    fi
fi
require "$OUT/tx_adapt.log" '"ev":"hopset.commit"' "authority committed it"
require "$OUT/rx_adapt.log" '"ev":"hopset.activate"' "receiver activated it"
require "$OUT/rx_adapt.log" '"ev":"hop.rx","state":"track"' "lockstep held"

# The excluded channel must be the jammed one. Only meaningful while the
# interferer stays put — in herding mode it has already moved by the time the
# first exclusion lands, and targeting wherever it went is the correct answer.
target=$(grep -F '"kind":"exclude"' "$OUT/rx_adapt.log" | head -1 |
         sed -n 's/.*"target":\([0-9]*\).*/\1/p')
if [ "$MODE" = "parked" ] || [ "$MODE" = "sense" ]; then
    if [ "${target:-x}" = "${jam_idx:-y}" ]; then
        echo "PASS: excluded the jammed channel (base index $jam_idx)"
    else
        echo "FAIL: excluded base index '${target:-none}', jammed is '$jam_idx'"
        fails=$((fails+1))
    fi
else
    echo "NOTE: first exclusion targeted base index ${target:-none} (jammer moves)"
fi

# the diversity floor must hold in every committed mask
minactive=$(grep -F '"ev":"hopset.activate"' "$OUT/rx_adapt.log" |
    sed -n 's/.*"mask":"0x\([0-9a-f]*\)".*/\1/p' |
    awk '{ n = 0; v = strtonum("0x" $0); while (v) { n += v % 2; v = int(v/2) }
           if (m == "" || n < m) m = n } END { print (m == "" ? 99 : m) }')
if [ "${minactive:-0}" -ge 3 ]; then
    echo "PASS: diversity floor held (min active = $minactive)"
else
    echo "FAIL: active set fell to $minactive"; fails=$((fails+1))
fi

if [ "$MODE" = "sense" ]; then
    # Sensing must actually have run: a silent no-op would pass the
    # non-regression trivially and prove nothing.
    require "$OUT/tx_adapt.log" '"ev":"hopset.sense"' "TX sensing emitted samples"
    n_pre=$(grep -c '"phase":"pre"' "$OUT/tx_adapt.log" 2>/dev/null || echo 0)
    [ "${n_pre:-0}" -ge 20 ] &&
        echo "PASS: $n_pre pre-burst samples taken" ||
        { echo "FAIL: only ${n_pre:-0} pre-burst samples"; fails=$((fails+1)); }
    # THE acceptance criterion. The interferer is real and on a hopset member,
    # so the receiver is right; the transmitter must not contradict it.
    if grep -F '"ev":"hopset.decision"' "$OUT/tx_adapt.log" 2>/dev/null |
       grep -qE '"kind":"(delay|hold)".*"veto":"tx_'; then
        echo "FAIL: the transmitter vetoed the receiver's correct exclusion"
        grep -F '"veto":"tx_' "$OUT/tx_adapt.log" | head -2
        fails=$((fails+1))
    else
        echo "PASS: no veto of the receiver-driven decision"
    fi
    # An absolute floor, not the loose a>b: sensing overhead that ate the gain
    # would still satisfy "improved".
    awk -v a="$ADAPT_DELIV" -v f="$SENSE_FLOOR" 'BEGIN { exit !(a >= f) }' &&
        echo "PASS: delivery $ADAPT_DELIV clears the regression floor $SENSE_FLOOR" ||
        { echo "FAIL: sensing regressed delivery ($ADAPT_DELIV < $SENSE_FLOOR)"
          fails=$((fails+1)); }
    require "$OUT/rx_adapt.log" '"ev":"hopset.probe"' "keyed probes aired"
elif [ "$MODE" = "failsafe" ]; then
    # The receiver hears everything and answers nothing, so its authority is
    # unusable and the transmitter must carry the link on its own evidence.
    require "$OUT/tx_adapt.log" '"origin":"failsafe"' "TX originated under outage"
    require "$OUT/tx_adapt.log" '"ev":"hopset.commit"' "authority committed it"
    t=$(grep -F '"origin":"failsafe"' "$OUT/tx_adapt.log" 2>/dev/null |
        grep -F '"kind":"exclude"' | head -1 |
        sed -n 's/.*"target":\([0-9]*\).*/\1/p')
    [ "${t:-x}" = "${jam_idx:-y}" ] &&
        echo "PASS: failsafe excluded the jammed channel (base index $jam_idx)" ||
        { echo "FAIL: failsafe excluded '${t:-none}', jammed is '$jam_idx'"
          fails=$((fails+1)); }
    # The definitive split-brain check: a split brain IS two endpoints holding
    # different (generation, mask), so compare the sets directly.
    # Generation 0 is where both sides START, so only the receiver ever logs an
    # "activation" for it — adopting the authority's status beacon is a no-op
    # state change. Comparing that against the transmitter, which never
    # activates into its own initial state, is noise rather than split-brain.
    gm() { grep -F '"ev":"hopset.activate"' "$1" 2>/dev/null |
           sed -n 's/.*"gen":\([0-9]*\).*"mask":"\([0-9a-fx]*\)".*/\1 \2/p' |
           awk '$1 > 0' | sort -u; }
    if [ -n "$(gm "$OUT/tx_adapt.log")" ] &&
       diff <(gm "$OUT/tx_adapt.log") <(gm "$OUT/rx_adapt.log") >/dev/null; then
        echo "PASS: receiver activated exactly the transmitter's generations"
    else
        echo "FAIL: generation/mask sets differ between the endpoints"
        echo "   tx: $(gm "$OUT/tx_adapt.log" | tr '\n' ' ')"
        echo "   rx: $(gm "$OUT/rx_adapt.log" | tr '\n' ' ')"
        fails=$((fails+1))
    fi
    # An autonomous exclusion must be undoable by the transmitter's own
    # evidence, or the link ratchets down permanently. With the receiver still
    # mute through phase C, a restore here can only have come from the keyed
    # probes the transmitter aired on the excluded channel.
    require "$OUT/tx_adapt.log" '"ev":"hopset.probe"' "TX aired keyed probes"
    if grep -F '"origin":"failsafe"' "$OUT/tx_adapt.log" 2>/dev/null |
       grep -qF '"kind":"restore"'; then
        echo "PASS: the transmitter restored the channel from its own probes"
    else
        echo "FAIL: no autonomous restore after the jammer stopped"
        fails=$((fails+1))
    fi
    # ...and the receiver has to have followed that too.
    last_rx=$(grep -F '"ev":"hopset.activate"' "$OUT/rx_adapt.log" 2>/dev/null |
              tail -1 | sed -n 's/.*"mask":"0x\([0-9a-f]*\)".*/\1/p')
    nch=$(( $(tr -cd ',' <<<"$CHANNELS" | wc -c) + 1 ))
    full=$(printf '%x' $(( (1 << nch) - 1 )))
    [ "${last_rx:-x}" = "$full" ] &&
        echo "PASS: receiver ended on the full hopset again (0x$full)" ||
        { echo "FAIL: receiver ended on 0x${last_rx:-none}, expected 0x$full"
          fails=$((fails+1)); }

    # Every committed change moved exactly one channel: the autonomous path is
    # bound by the same structural limits a proposal is.
    if gm "$OUT/tx_adapt.log" | awk '{print $2}' |
       awk 'NR>1{d=xor(strtonum(p),strtonum($0)); n=0; while(d){n+=d%2;d=int(d/2)}
                 if(n!=1){bad=1}} {p=$0} END{exit bad?1:0}'; then
        echo "PASS: one channel per autonomous update"
    else
        echo "FAIL: an autonomous update moved more than one channel"
        fails=$((fails+1))
    fi
elif [ "$MODE" = "parked" ]; then
    awk -v b="$BASE_DELIV" -v a="$ADAPT_DELIV" 'BEGIN { exit !(a > b) }' &&
        echo "PASS: delivery improved ($BASE_DELIV -> $ADAPT_DELIV)" ||
        { echo "FAIL: delivery did not improve ($BASE_DELIV -> $ADAPT_DELIV)"
          fails=$((fails+1)); }
    require "$OUT/rx_adapt.log" '"ev":"hopset.probe"' "keyed probes aired"
    if grep -F '"ev":"hopset.probe"' "$OUT/rx_adapt.log" | grep -qF '"delivered":true'; then
        echo "PASS: probes delivered after the jammer left"
    else
        echo "FAIL: no probe ever delivered"; fails=$((fails+1))
    fi
    if grep -F '"ev":"hopset.decision"' "$OUT/rx_adapt.log" | grep -qF '"kind":"restore"'; then
        echo "PASS: receiver proposed restoration"
    else
        echo "FAIL: receiver never proposed restoration"; fails=$((fails+1))
    fi
else
    # Against a jammer that moves after every exclusion the link SHOULD keep
    # deciding: exclude whichever channel is jammed now, restore whichever one
    # the probes prove recovered. Repeated decisions are the system tracking a
    # moving interferer, not damage. What must never happen is the set being
    # walked down — so the invariant is on the NET exclusion depth, not on the
    # number of decisions.
    seq=$(grep -F '"ev":"hopset.decision"' "$OUT/rx_adapt.log" |
          sed -n 's/.*"kind":"\([a-z]*\)".*"target":\([0-9]*\).*/\1:\2/p')
    echo "   decision sequence: $(echo "$seq" | tr '\n' ' ')"
    nch=$(( $(tr -cd ',' <<<"$CHANNELS" | wc -c) + 1 ))
    budget=$(( nch - 3 ))
    # Measured on the COMMITTED masks, not on proposals: a proposal the
    # authority refuses (its own cooldown / diversity limits) never moved the
    # link, and counting those would score the defences as damage.
    depth=$(grep -F '"ev":"hopset.activate"' "$OUT/rx_adapt.log" |
        sed -n 's/.*"mask":"0x\([0-9a-f]*\)".*/\1/p' |
        awk -v n="$nch" '{ c = 0; v = strtonum("0x" $0);
                           while (v) { c += v % 2; v = int(v/2) }
                           d = n - c; if (d > max) max = d }
                         END { print max+0 }')
    if [ "${depth:-0}" -le "$budget" ]; then
        echo "PASS: committed exclusion depth $depth within the floor budget $budget"
    else
        echo "FAIL: committed exclusion depth $depth exceeds budget $budget"
        fails=$((fails+1))
    fi
    # A herding jammer cannot be out-run — with the set floored at 3 it can
    # always sit on an active channel, so delivery is not expected to improve.
    # What must not happen is a collapse: the adaptive link has to stay in the
    # same league as the fixed one while keeping its diversity.
    awk -v b="$BASE_DELIV" -v a="$ADAPT_DELIV" 'BEGIN { exit !(a >= 0.8 * b) }' &&
        echo "PASS: no collapse under a moving jammer ($BASE_DELIV -> $ADAPT_DELIV)" ||
        { echo "FAIL: delivery collapsed ($BASE_DELIV -> $ADAPT_DELIV)"
          fails=$((fails+1)); }
fi

[ "$fails" -eq 0 ] && echo "== ALL PASS ==" || echo "== $fails FAILURES =="
exit "$fails"
