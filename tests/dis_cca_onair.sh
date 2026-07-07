#!/usr/bin/env bash
# MEASURE-FIRST gate for the dis_cca / EDCCA-disable knob (SetCcaMode,
# DEVOURER_DIS_CCA). The OpenIPC-FPV community reports dis_cca "doubles range /
# removes stuttering" on the rtl88x2eu. devourer injects in monitor mode, which
# already bypasses the MAC CSMA/CCA backoff on the *TX* side — so any real
# benefit must be RX-side: EDCCA muting the receiver (or forcing it to hold off)
# when in-band energy crosses the energy-detect threshold. dis_cca removes that
# gate, so the RX should keep demodulating the wanted frames through interference
# that would otherwise pin CCA busy.
#
# The test: ONE 8822EU DUT, ONE swept B210 AWGN interferer, ONE marginal beacon.
# A/B is DEVOURER_DIS_CCA off vs on at each noise gain. delivery = the final
# rx.txhit event's hits (canonical-SA beacons decoded); we also log median IGI +
# OFDM false-alarm from the rx.energy events.
#
# MEASURED RESULT (8822EU, 5-point AWGN sweep 46..76 dB): NULL. The MAC
# EDCCA-disable (DEVOURER_DIS_CCA=1) leaves delivery within 1% at every noise
# gain (33700 vs 33400 hits, ratio 0.99; IGI pinned, FA identical). The full
# vendor recipe (incl. the BB 0x1d58 OFDM-CCA-off write) instead DEAFENS the RX
# (~6800 -> ~10 hits), so it is not implemented. Mechanism: EDCCA gates TX
# deferral, not RX decode, and devourer injects in monitor mode — which already
# bypasses the CSMA/EDCCA backoff the community's "range" benefit comes from.
# So SetCcaMode is NOT promoted to IRtlDevice; the J3 knob + this harness are
# kept for anyone re-measuring on a real (non-monitor) link. This bench can't
# bury the wanted beacon under B210 AWGN (near-field front-end limit), so the
# null is "no effect in the testable regime" + the mechanism above.
#
# Usage: sudo -v && tests/dis_cca_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${DISCCA_OUT:-/tmp/devourer-dis-cca}"
CH="${CH:-36}"
BEACON_PWR="${BEACON_PWR:-10}"          # 8812AU TXAGC index — marginal link
GAINS="${GAINS:-40 46 52 58 64 70 76}"  # B210 AWGN gain ladder (dB)
DUR="${DUR:-14}"                         # RX cell seconds per (arm, gain)
PY="$ROOT/tests/.venv/bin/python"
{ [ -x "$PY" ] && "$PY" -c 'import uhd' 2>/dev/null; } || PY=python3
mkdir -p "$OUT"

TX_PID=0x8812 TX_VID=0x0bda             # 8812AU beacon source
DUT_PID=0xa81a DUT_VID=0x0bda           # 8822EU — Jaguar3, the community chip

cleanup() {
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x txdemo 2>/dev/null || true
    sudo -n pkill -f sdr_interferer 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
for d in "$TX_PID $TX_VID beacon-8812AU" "$DUT_PID $DUT_VID DUT-8822EU"; do
    read -r p v name <<<"$d"
    plugged "$p" "$v" || { echo "SKIP: $name ($p:$v) not plugged"; exit 0; }
done

echo "== building =="
cmake --build "$ROOT/build" -j --target rxdemo txdemo >/dev/null || exit 1

# One RX cell against a running interferer + beacon. $5 = "1" arms dis_cca.
# Echoes "hits igi_med fa_med".
run_cell() { # $1=gain $2=tag $3=discca(0|1)
    local gain="$1" tag="$2" discca="$3"
    sudo -n "$PY" "$ROOT/tests/sdr_interferer.py" --channel "$CH" \
        --tx-gain "$gain" --rate 20e6 --mode noise --secs $((DUR + 12)) \
        >"$OUT/$tag-noise.log" 2>&1 &
    local jam=$!
    sleep 5
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR="$BEACON_PWR" \
        DEVOURER_TX_GAP_US=1500 \
        timeout $((DUR + 8)) "$ROOT/build/txdemo" >/dev/null 2>&1 &
    local tx=$!
    sleep 3
    local discca_env=""
    [ "$discca" = "1" ] && discca_env="DEVOURER_DIS_CCA=1"
    sudo -n env DEVOURER_PID="$DUT_PID" DEVOURER_VID="$DUT_VID" \
        DEVOURER_CHANNEL="$CH" DEVOURER_RX_ENERGY_MS=500 $discca_env \
        timeout "$DUR" "$ROOT/build/rxdemo" >"$OUT/$tag.log" 2>&1 || true
    kill "$tx" 2>/dev/null; wait "$tx" 2>/dev/null
    kill "$jam" 2>/dev/null; wait "$jam" 2>/dev/null
    sudo -n pkill -f sdr_interferer 2>/dev/null
    sleep 3
    python3 - "$OUT/$tag.log" <<'PYEOF'
import json, statistics, sys
hits = 0
igis, fas = [], []
for line in open(sys.argv[1], errors="replace"):
    try:
        ev = json.loads(line)
    except ValueError:
        continue
    name = ev.get("ev")
    if name == "rx.txhit":
        hits = max(hits, int(ev.get("hits", 0)))
    elif name == "rx.energy":
        if ev.get("igi") is not None:
            igis.append(int(ev["igi"]))
        if ev.get("fa_ofdm") is not None:
            fas.append(int(ev["fa_ofdm"]))
im = int(statistics.median(igis)) if igis else -1
fm = int(statistics.median(fas)) if fas else -1
print(f"{hits} {im} {fm}")
PYEOF
}

# Confirm the knob actually lands its registers once (no interferer) before the
# sweep. The "MAC EDCCA DISABLED" line is a human diagnostic on stderr.
echo "== register-landing check (no interferer) =="
sudo -n env DEVOURER_PID="$DUT_PID" DEVOURER_VID="$DUT_VID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_DIS_CCA=1 timeout 6 "$ROOT/build/rxdemo" 2>&1 >/dev/null \
    | grep -m1 "MAC EDCCA" || echo "  WARN: no MAC EDCCA diagnostic seen"

printf "%-8s | %-22s | %-22s\n" "gain_dB" "dis_cca=0 (hits/igi/fa)" "dis_cca=1 (hits/igi/fa)"
printf -- "---------+------------------------+------------------------\n"
: >"$OUT/summary.tsv"
for gain in $GAINS; do
    read -r a_hits a_igi a_fa <<<"$(run_cell "$gain" "off-g$gain" 0)"
    read -r b_hits b_igi b_fa <<<"$(run_cell "$gain" "on-g$gain" 1)"
    printf "%-8s | %-22s | %-22s\n" "$gain" \
        "$a_hits/$a_igi/$a_fa" "$b_hits/$b_igi/$b_fa"
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
        "$gain" "$a_hits" "$a_igi" "$a_fa" "$b_hits" "$b_igi" "$b_fa" >>"$OUT/summary.tsv"
done

echo
python3 - "$OUT/summary.tsv" <<'PYEOF'
import sys
rows = []
for line in open(sys.argv[1]):
    g, ah, ai, afa, bh, bi, bfa = line.split()
    rows.append((float(g), int(ah), int(bh)))
if len(rows) < 3:
    print("insufficient points"); sys.exit(0)
a_max = max(r[1] for r in rows) or 1
b_max = max(r[2] for r in rows) or 1
def crossing(pts, mx):
    half = mx / 2.0
    for (g0, h0), (g1, h1) in zip(pts, pts[1:]):
        if h0 >= half >= h1 and h0 != h1:
            return g0 + (g1 - g0) * (h0 - half) / (h0 - h1)
    return None
a_pts = [(r[0], r[1]) for r in rows]
b_pts = [(r[0], r[2]) for r in rows]
ac, bc = crossing(a_pts, a_max), crossing(b_pts, b_max)
tot_a = sum(r[1] for r in rows); tot_b = sum(r[2] for r in rows)
print(f"dis_cca=0 50%-delivery gain: {ac if ac else 'n/a'}")
print(f"dis_cca=1 50%-delivery gain: {bc if bc else 'n/a'}")
print(f"total hits: dis_cca=0 {tot_a}  dis_cca=1 {tot_b}  "
      f"(ratio {tot_b/max(tot_a,1):.2f})")
if ac and bc:
    d = bc - ac
    print(f"crossing shift (dis_cca ON - OFF): {d:+.1f} dB")
    if d >= 3:
        print("VERDICT: dis_cca HELPS (>=3 dB) — SHIP the knob")
    elif d <= 1 and tot_b <= tot_a * 1.15:
        print("VERDICT: NULL (<=1 dB, no delivery gain) — DO NOT promote; document")
    else:
        print("VERDICT: marginal — inspect the curve before deciding")
else:
    r = tot_b / max(tot_a, 1)
    if r >= 1.3:
        print("VERDICT: dis_cca HELPS (>=30% more delivery) — SHIP the knob")
    elif r <= 1.15:
        print("VERDICT: NULL — DO NOT promote; document")
    else:
        print("VERDICT: marginal — inspect the curve before deciding")
PYEOF
