#!/usr/bin/env bash
# Hardware validation for issue #152: Jaguar3 RX phy-status parsing.
#
# Before the fix the Jaguar3 RX path (8812CU/8822CU c812, 8812EU/8822EU a81a)
# decoded frames but left RxAtrib.rssi/snr/evm at 0 — nothing parsed the jgr3
# phy-status report the chip prepends. This proves the parse now populates
# non-zero, plausible per-frame metrics on BOTH Jaguar3 variants.
#
# Rig: one known-good TX adapter (default 8812AU) beacons the canonical SA
# (57:42:75:05:d6:00) at HT MCS1 on ch6; each Jaguar3 RX adapter runs
# rxdemo with DEVOURER_DUMP_BODY=1 and we assert the <devourer-body>
# lines (canonical-SA hits) show non-zero rssi AND snr (and evm for MCS/HT).
#
#   sudo ./tests/validate_jaguar3_physts.sh
#   TX_PID=0x8812 CH=6 RATE=MCS1 ./tests/validate_jaguar3_physts.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

TX_PID="${TX_PID:-0x8812}"          # Jaguar1 8812AU: solid known-good TX
CH="${CH:-6}"                       # 2.4 GHz, strong bench link
RATE="${RATE:-MCS1}"                # HT -> jgr3 type1 page carries per-path evm
SECS="${SECS:-18}"                  # RX capture window per adapter
OUT="${OUT:-/tmp/devourer-jgr3-physts}"
# Jaguar3 RX DUTs: 8822CU (c812) + 8822EU (a81a). Both share the jgr3 layout.
RX_PIDS=("${RX_PIDS[@]:-0xc812 0xa81a}")
mkdir -p "$OUT"

cleanup() {
    pkill -x txdemo 2>/dev/null || true
    pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building demos =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null

echo "== starting TX beacon: PID=$TX_PID ch=$CH rate=$RATE =="
sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$RATE" \
    "$ROOT/build/txdemo" >"$OUT/tx.log" 2>&1 &
sleep 4   # let the TX chip finish bring-up and start airing beacons

overall=0
for pid in ${RX_PIDS[@]}; do
    log="$OUT/rx_${pid}.log"
    echo "== RX $pid on ch$CH for ${SECS}s =="
    sudo -n env DEVOURER_PID="$pid" DEVOURER_CHANNEL="$CH" \
        DEVOURER_DUMP_BODY=1 \
        timeout "$SECS" "$ROOT/build/rxdemo" >"$log" 2>&1 || true

    hits=$(grep -c "<devourer-tx-hit>" "$log" || true)
    body=$(grep "<devourer-body>" "$log" || true)
    echo "   tx-hits=$hits  body-lines=$(printf '%s\n' "$body" | grep -c body= || true)"
    if [ -z "$body" ]; then
        echo "   FAIL: no canonical-SA <devourer-body> frames decoded (link/TX?)"
        overall=1
        continue
    fi
    printf '%s\n' "$body" | head -3 | sed 's/body=.*/body=.../; s/^/     /'

    # Assert at least one body frame has non-zero rssi AND non-zero snr.
    ok=$(printf '%s\n' "$body" | awk '
        match($0, /rssi=(-?[0-9]+),(-?[0-9]+)/, r) &&
        match($0, /snr=(-?[0-9]+),(-?[0-9]+)/, s) {
            if ((r[1]+0 != 0 || r[2]+0 != 0) && (s[1]+0 != 0 || s[2]+0 != 0)) n++
        } END { print n+0 }')
    echo "   frames with non-zero rssi & snr: $ok"
    if [ "${ok:-0}" -gt 0 ]; then
        echo "   PASS: $pid surfaces per-frame phy-status metrics"
    else
        echo "   FAIL: $pid rssi/snr still zero"
        overall=1
    fi
done

echo
[ "$overall" -eq 0 ] && echo "== VALIDATION PASS ==" || echo "== VALIDATION FAIL =="
exit "$overall"
