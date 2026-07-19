#!/usr/bin/env bash
# On-air validation for the Kestrel passive noise floor (RtlKestrelDevice PPDU
# handler -> rssi-snr -> RxQualityAccumulator) on BOTH dies.
#
# The gap this closes: the C8852C emitted EMPTY physts reports
# (is_valid=0, all-zero content) so it had no per-frame RSSI/SNR and a NULL
# passive floor. Root cause was the 8852C physts MEASUREMENT bring-up, not the
# parse: (a) halbb_ic_hw_setting_init_8852c sets the BB evm/measurement
# report-enable (0xa10[0]); (b) the physts CR map + IE bitmap must be programmed
# (halbb_cr_cfg_physts_init + halbb_physts_parsing_init); (c) R_AX_PPDU_STAT
# (0xCE40) must NOT set the APP_MAC_INFO/RX_CNT/PLCP_HDR prepend bits (1/2/3),
# which pushed the physts_hdr_info out of place. With those, the 8852C physts is
# bit-identical to the 8852B (byte0 is_valid, byte3 rssi_avg_td, IE_01 avg_snr),
# so the existing hand-parse works for both dies.
#
# CHECK: with DEVOURER_RXQUALITY on an ambient channel, the passive
# noise_floor_dbm populates (non-null, plausible -80..-100 band) on BOTH dies,
# and the C8852B is unchanged from before.
#
# Usage: sudo -v && tests/kestrel_physts_floor_onair.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${KPFOUT:-/tmp/devourer-kestrel-physts}"
CH="${CH:-6}"        # ambient-traffic channel (2.4 GHz; both dies do monitor RX)
DUR="${DUR:-8}"
mkdir -p "$OUT"

VID=0x35bc
C_PID=0x0101         # RTL8832CU / C8852C  (the die the fix targets)
B_PID=0x0108         # RTL8852BU / C8852B  (regression control)

plugged() { lsusb -d "$(printf '%04x:%04x' "$1" "$2")" >/dev/null 2>&1; }
unbind() { local pid="$1" d p i
  for d in /sys/bus/usb/devices/*/idProduct; do p=$(cat "$d" 2>/dev/null) || continue
    [ "$p" = "${pid#0x}" ] || continue
    for i in "$(dirname "$d")":*; do
      [ -e "$i/driver" ] && sudo sh -c "echo '$(basename "$i")' > '$i/driver/unbind'" 2>/dev/null || true
    done; done; }

run_die() { # $1=pid $2=tag -> "n snr passiveNF activeNF rssiMax"
  sudo env DEVOURER_VID="$VID" DEVOURER_PID="$1" DEVOURER_CHANNEL="$CH" \
    DEVOURER_RXQUALITY=1 DEVOURER_RX_NOISE_FLOOR=1 DEVOURER_RX_ENERGY_MS=500 \
    timeout "$DUR" "$ROOT/build/rxdemo" >"$OUT/$2.log" 2>/dev/null || true
  grep '"ev":"rx.quality"' "$OUT/$2.log" 2>/dev/null | python3 "$OUT/med.py"
}

cat >"$OUT/med.py" <<'PYEOF'
import json,sys,statistics as st
rows=[json.loads(l) for l in sys.stdin]
def med(k):
    v=[x[k] for x in rows if isinstance(x.get(k),(int,float))]
    return round(st.median(v),1) if v else None
def s(x): return "null" if x is None else str(x)
print("\t".join([str(len(rows)), s(med("snr_mean_db")), s(med("noise_floor_dbm")),
                 s(med("abs_noise_floor_dbm")), s(med("rssi_max_dbm"))]))
PYEOF

echo "== building rxdemo =="; cmake --build "$ROOT/build" -j --target rxdemo >/dev/null || exit 1
printf "\n%-8s %-7s %-8s %-11s %-10s %-9s\n" DIE qual_ev snr_dB passiveNF activeNF rssiMax
: >"$OUT/summary.tsv"
for spec in "C8852C:$C_PID" "C8852B:$B_PID"; do
  name="${spec%%:*}"; pid="${spec##*:}"
  plugged "$VID" "$pid" || { printf "%-8s SKIP (not plugged)\n" "$name"; continue; }
  unbind "$pid"
  read -r n snr nf anf rmax < <(run_die "$pid" "$name")
  printf "%-8s %-7s %-8s %-11s %-10s %-9s\n" "$name" "${n:-0}" "${snr:-null}" "${nf:-null}" "${anf:-null}" "${rmax:-null}"
  printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$name" "${n:-0}" "${snr:-null}" "${nf:-null}" "${anf:-null}" "${rmax:-null}" >>"$OUT/summary.tsv"
done

echo
python3 - "$OUT/summary.tsv" <<'PYEOF'
import sys
def num(x):
    try: return float(x)
    except: return None
rows={}
for l in open(sys.argv[1]):
    f=l.rstrip("\n").split("\t"); rows[f[0]]=f
for name in ("C8852C","C8852B"):
    r=rows.get(name)
    if not r: print(f"  {name}: no data"); continue
    nf=num(r[3])
    if nf is None: print(f"  {name}: passive floor NULL (FAIL)")
    elif -100<=nf<=-80: print(f"  {name}: passive floor {nf:.0f} dBm, snr {r[2]} dB -> PASS")
    else: print(f"  {name}: passive floor {nf} dBm (snr {r[2]}) -> INSPECT")
c,b=rows.get("C8852C"),rows.get("C8852B")
if c and b and num(c[3]) is not None and num(b[3]) is not None:
    print(f"  cross-die delta {abs(num(c[3])-num(b[3])):.0f} dB")
print(f"\n  raw: {sys.argv[1]}")
PYEOF
