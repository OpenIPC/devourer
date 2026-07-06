#!/usr/bin/env bash
# Measurement for issue #190: does Jaguar3's STATIC initial gain (no DIG) cost
# RX delivery under a rising interference floor, vs the FA-driven DIG the older
# families have? Measure BEFORE deciding to port DIG to Jaguar3.
#
# A/B under one swept B210 AWGN interferer:
#   DUT      8822EU (Jaguar3) — IGI static after bring-up (the gap under test)
#   control  8822BU (Jaguar2) — dig_step raises IGI as false alarms climb
# Same marginal TX beacon (8812AU, TX power dialled low via the runtime knob so
# the noise sweep actually reaches a delivery cliff), same noise ladder.
#
# Per (chip, noise-gain) cell: delivery = the final <devourer-tx-hit>hits=N
# (canonical-SA beacons the RX decoded), plus the median IGI + false-alarm rate
# from <devourer-energy> (DEVOURER_RX_ENERGY_MS). The story is in two numbers:
#   - does the J3's IGI stay pinned while its FA climbs (confirming no DIG)?
#   - is the J3's 50%-delivery noise-gain WORSE than the J2's (the penalty DIG
#     would recover)?
#
# Decision: a >= ~3 dB gap at the 50% crossing (with the J2's IGI visibly
# adapting) justifies building J3 DIG; a <= ~1 dB gap says the newer BB already
# compensates and DIG is not worth the runtime cost.
#
# Usage: sudo -v && tests/j3_dig_penalty_sweep.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${J3DIG_OUT:-/tmp/devourer-j3-dig-sweep}"
CH=36
BEACON_PWR="${BEACON_PWR:-10}"          # 8812AU TXAGC index — marginal link
GAINS="${GAINS:-40 46 52 58 64 70 76}"  # B210 AWGN gain ladder (dB)
DUR="${DUR:-14}"                         # RX cell seconds per (chip, gain)
PY="$ROOT/tests/.venv/bin/python"
{ [ -x "$PY" ] && "$PY" -c 'import uhd' 2>/dev/null; } || PY=python3
mkdir -p "$OUT"

TX_PID=0x8812 TX_VID=0x0bda             # 8812AU beacon source
DUT_PID=0xa81a DUT_VID=0x0bda           # 8822EU — J3 static-IGI DUT
CTL_PID=0x012d CTL_VID=0x2357           # 8822BU (T3U) — J2 adaptive-IGI control

cleanup() {
    sudo -n pkill -x WiFiDriverDemo 2>/dev/null || true
    sudo -n pkill -x WiFiDriverTxDem 2>/dev/null || true
    sudo -n pkill -f sdr_interferer 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
for d in "$TX_PID $TX_VID beacon-8812AU" "$DUT_PID $DUT_VID DUT-8822EU" \
         "$CTL_PID $CTL_VID control-8822BU"; do
    read -r p v name <<<"$d"
    plugged "$p" "$v" || { echo "SKIP: $name ($p:$v) not plugged"; exit 0; }
done

echo "== building =="
cmake --build "$ROOT/build" -j --target WiFiDriverDemo WiFiDriverTxDemo >/dev/null || exit 1

# One RX cell against a running interferer + beacon. Echoes "hits igi_med fa_med".
run_cell() { # $1=rx_pid $2=rx_vid $3=gain $4=tag
    local rxpid="$1" rxvid="$2" gain="$3" tag="$4"
    # B210 AWGN at this gain for the whole cell + margins.
    sudo -n "$PY" "$ROOT/tests/sdr_interferer.py" --channel "$CH" \
        --tx-gain "$gain" --rate 20e6 --mode noise --secs $((DUR + 12)) \
        >"$OUT/$tag-noise.log" 2>&1 &
    local jam=$!
    sleep 5
    # Marginal beacon: 8812AU, MCS3, TX power dialled low (runtime flat index).
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR="$BEACON_PWR" \
        DEVOURER_TX_GAP_US=1500 \
        timeout $((DUR + 8)) "$ROOT/build/WiFiDriverTxDemo" >/dev/null 2>&1 &
    local tx=$!
    sleep 3
    # RX under test.
    sudo -n env DEVOURER_PID="$rxpid" DEVOURER_VID="$rxvid" \
        DEVOURER_CHANNEL="$CH" DEVOURER_RX_ENERGY_MS=500 \
        timeout "$DUR" "$ROOT/build/WiFiDriverDemo" >"$OUT/$tag.log" 2>&1 || true
    kill "$tx" 2>/dev/null; wait "$tx" 2>/dev/null
    kill "$jam" 2>/dev/null; wait "$jam" 2>/dev/null
    sudo -n pkill -f sdr_interferer 2>/dev/null
    sleep 3
    python3 - "$OUT/$tag.log" <<'PYEOF'
import re, statistics, sys
log = open(sys.argv[1], errors="replace").read()
hits = 0
for m in re.finditer(r"<devourer-tx-hit>.*hits=(\d+)", log):
    hits = max(hits, int(m.group(1)))
igis, fas = [], []
for m in re.finditer(r"<devourer-energy>.*", log):
    s = m.group(0)
    gi = re.search(r"\bigi=(-?\d+)", s)
    fo = re.search(r"\bfa_ofdm=(-?\d+|-)", s)
    if gi and gi.group(1) != "-":
        igis.append(int(gi.group(1)))
    if fo and fo.group(1) not in ("-",):
        fas.append(int(fo.group(1)))
im = int(statistics.median(igis)) if igis else -1
fm = int(statistics.median(fas)) if fas else -1
# IGI spread: does it move at all across the cell? (max-min)
isp = (max(igis) - min(igis)) if igis else 0
print(f"{hits} {im} {fm} {isp}")
PYEOF
}

printf "%-8s | %-28s | %-28s\n" "gain_dB" "8822EU J3 (hits/igi/fa/isp)" "8822BU J2 (hits/igi/fa/isp)"
printf -- "---------+------------------------------+------------------------------\n"
: >"$OUT/summary.tsv"
for gain in $GAINS; do
    read -r d_hits d_igi d_fa d_isp <<<"$(run_cell "$DUT_PID" "$DUT_VID" "$gain" "dut-g$gain")"
    read -r c_hits c_igi c_fa c_isp <<<"$(run_cell "$CTL_PID" "$CTL_VID" "$gain" "ctl-g$gain")"
    printf "%-8s | %-28s | %-28s\n" "$gain" \
        "$d_hits/$d_igi/$d_fa/$d_isp" "$c_hits/$c_igi/$c_fa/$c_isp"
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
        "$gain" "$d_hits" "$d_igi" "$d_fa" "$d_isp" \
        "$c_hits" "$c_igi" "$c_fa" "$c_isp" >>"$OUT/summary.tsv"
done

echo
python3 - "$OUT/summary.tsv" <<'PYEOF'
import sys
rows = []
for line in open(sys.argv[1]):
    g, dh, di, dfa, disp, ch, ci, cfa, cisp = line.split()
    rows.append((float(g), int(dh), int(di), int(disp), int(ch), int(ci), int(cisp)))
if len(rows) < 3:
    print("insufficient points"); sys.exit(0)
d_max = max(r[1] for r in rows) or 1
c_max = max(r[4] for r in rows) or 1
# 50%-delivery crossing (linear interp on the descending curve).
def crossing(pts, mx):
    half = mx / 2.0
    for (g0, h0), (g1, h1) in zip(pts, pts[1:]):
        if h0 >= half >= h1 and h0 != h1:
            return g0 + (g1 - g0) * (h0 - half) / (h0 - h1)
    return None
d_pts = [(r[0], r[1]) for r in rows]
c_pts = [(r[0], r[4]) for r in rows]
dc, cc = crossing(d_pts, d_max), crossing(c_pts, c_max)
# Did the J3 IGI move at all across the whole sweep? (adaptive control should.)
d_isp_max = max(r[3] for r in rows)
c_isp_max = max(r[6] for r in rows)
print(f"J3 (8822EU) delivery peak={d_max}, 50% crossing at "
      f"{dc:.1f} dB" if dc else f"J3 delivery peak={d_max}, no 50% crossing in range")
print(f"J2 (8822BU) delivery peak={c_max}, 50% crossing at "
      f"{cc:.1f} dB" if cc else f"J2 delivery peak={c_max}, no 50% crossing in range")
print(f"IGI adaptivity across sweep: J3 max-spread={d_isp_max} (expect ~0 = static), "
      f"J2 max-spread={c_isp_max} (expect >0 = DIG moving)")
if dc and cc:
    gap = cc - dc
    verdict = ("JUSTIFIES DIG (>=3 dB)" if gap >= 3 else
               "MARGINAL (1-3 dB)" if gap >= 1 else "NOT WORTH IT (<=1 dB)")
    print(f"\n>>> Static-IGI penalty (J2 crossing - J3 crossing) = {gap:+.1f} dB -> {verdict}")
else:
    print("\n>>> One curve did not cross 50% in the swept range — widen GAINS and re-run")
PYEOF
