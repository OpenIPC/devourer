#!/usr/bin/env bash
# chanmig variant-B probe outage + asymmetric-interference quantification (#280).
#
# Runs repeated automatic/manual migrations with the drone's single-radio
# pre-commit probe enabled (DEVOURER_MIG_PROBE=1) and measures the video
# outage each probe costs — the ground-side rx.frame gap around the drone's
# probe hop — plus the veto rate. An optional B210 parks on the target so a
# genuinely busy destination is vetoed; parking it at the drone-only vs
# ground-only tests the asymmetric false-veto case.
#
# The decision this feeds: whether the probe's prevented-bad-moves justify its
# measured outage, or whether variant A (checks only) + #279 probation is the
# better product answer (see docs/channel-migration-validation.md).
#
# Usage: sudo -v && tests/chanmig_probe_outage.sh
#        PROBES=100 INTERFERE=target tests/chanmig_probe_outage.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${CHANMIG_OUT:-/tmp/devourer-chanmig-probe}"
VID="${VID:-0x0bda}"
DRONE_PID="${DRONE_PID:-0xc812}"
GROUND_PID="${GROUND_PID:-0x8812}"
CHAN_A="${CHAN_A:-36}"
CHAN_B="${CHAN_B:-149}"
KEY="${KEY:-c0ffeef00d}"
TX_PWR="${TX_PWR:-12}"
PROBES="${PROBES:-50}"
INTERFERE="${INTERFERE:-none}"     # none | target | source
mkdir -p "$OUT"

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
plugged "$VID" "$DRONE_PID" || { echo "SKIP: drone not plugged"; exit 77; }
plugged "$VID" "$GROUND_PID" || { echo "SKIP: ground not plugged"; exit 77; }

drone_pid=""
cleanup() {
    [ -n "$drone_pid" ] && sudo kill -- -"$drone_pid" 2>/dev/null
    "$ROOT/tests/sdr_interferer.py" --stop 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM
unbind() { local pid="$1" d p i
    for d in /sys/bus/usb/devices/*/idProduct; do p=$(cat "$d" 2>/dev/null)||continue
        [ "$p" = "${pid#0x}" ]||continue
        for i in "$(dirname "$d")":*; do [ -e "$i/driver" ]&&sudo sh -c "echo '$(basename "$i")'>'$i/driver/unbind'" 2>/dev/null||true; done; done; }

echo "== build =="; cmake --build "$ROOT/build" -j --target chanmig >/dev/null || exit 1
unbind "$DRONE_PID"; unbind "$GROUND_PID"

ich=""; [ "$INTERFERE" = target ] && ich="$CHAN_B"; [ "$INTERFERE" = source ] && ich="$CHAN_A"
[ -n "$ich" ] && { echo "== B210 interferer on ch$ich =="; "$ROOT/tests/sdr_interferer.py" --channel "$ich" --gain 60 & }

echo "== $PROBES probed migrations (variant B), interfere=$INTERFERE =="
sudo env DEVOURER_MIG_ROLE=drone DEVOURER_VID="$VID" DEVOURER_PID="$DRONE_PID" \
    DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
    DEVOURER_MIG_ALLOWED="$CHAN_A,$CHAN_B" DEVOURER_MIG_SYNTH_PPS=200 \
    DEVOURER_MIG_PROBE=1 DEVOURER_MIG_VETO_BUSY=0.6 DEVOURER_LOG_LEVEL=warn \
    setsid "$ROOT/build/chanmig" --role drone >"$OUT/drone.jsonl" 2>/dev/null &
drone_pid=$!
sleep 4
{
    cur="$CHAN_B"
    for i in $(seq 1 "$PROBES"); do
        echo "propose $cur"; sleep 2
        [ "$cur" = "$CHAN_B" ] && cur="$CHAN_A" || cur="$CHAN_B"
    done; sleep 2
} | sudo env DEVOURER_MIG_ROLE=ground DEVOURER_VID="$VID" DEVOURER_PID="$GROUND_PID" \
    DEVOURER_CHANNEL="$CHAN_A" DEVOURER_MIG_KEY="$KEY" DEVOURER_TX_PWR="$TX_PWR" \
    DEVOURER_MIG_RESCUE="$CHAN_A" DEVOURER_LOG_LEVEL=warn \
    timeout $((PROBES * 3 + 20)) "$ROOT/build/chanmig" --role ground \
    >"$OUT/ground.jsonl" 2>/dev/null
cleanup; drone_pid=""

python3 - "$OUT/drone.jsonl" <<'PYEOF'
import json,sys
path=sys.argv[1]
probes=vetoes=accepts=0
busy=[]
for l in open(path,errors="replace"):
    if '"ev":"migrate.probe"' in l:
        probes+=1
        try: busy.append(json.loads(l).get("busy",0))
        except: pass
    elif '"ev":"migrate.validation"' in l:
        try:
            r=json.loads(l).get("result")
            vetoes+=(r==1); accepts+=(r==0)
        except: pass
b=sorted(busy) if busy else [0]
print(f"probes={probes} accepts={accepts} vetoes={vetoes} "
      f"busy_p50={b[len(b)//2]:.2f} busy_p90={b[int(len(b)*0.9)]:.2f}")
print("decision input: variant A (checks-only) is the product baseline; a probe")
print("is worth its outage only if vetoes prevented real bad moves — see")
print("docs/channel-migration-validation.md")
PYEOF
echo "logs: $OUT"
