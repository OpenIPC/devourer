#!/usr/bin/env bash
# SDR validation for the Jaguar-1 CW single-tone (DEVOURER_CW_TONE, issue #165).
#
# For each (DUT, band) cell it runs the issue's checklist against a UHD SDR
# (B210): carrier presence + center-frequency accuracy, spectral purity
# (carrier-to-spur), power monotonicity vs the gain index, a clean stop (carrier
# vanishes + normal beacon TX resumes), and a 60 s frequency-drift hold.
#
# Each measurement launches its OWN WiFiDriverTxDemo session: the tone is armed
# in InitWrite, so a fresh gain index means a fresh bring-up (no live-gain API).
#
# Requires: two USB Wi-Fi adapters (8812AU + 8821AU) plugged in, a UHD SDR near
# them, and sudo (USB claim). The SDR's usable instantaneous bandwidth must
# cover --rate around each tone center.
#
#   sudo ./tests/cw_tone_sdr.sh                 # full 3-cell matrix
#   sudo ./tests/cw_tone_sdr.sh --cell 8821au_ch6
#   sudo ./tests/cw_tone_sdr.sh --gains 0,8,16,24,31 --drift-secs 60
#
# Cleanup: exact-comm kills of the demo + probe on any exit (belt-and-braces on
# top of the demo's own signal handlers).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEMO="$ROOT/build/WiFiDriverTxDemo"

# --- matrix cells: name|VID|PID|channel|sdr_center_mhz -------------------------
# VID/PID are the adapter's USB IDs (`lsusb`); override the whole cell list with
# --cells or pick one with --cell. The RTL8821AU here is the TP-Link Archer T2U
# Plus, which enumerates rebadged as 2357:0120 (VID override needed).
CELLS=(
    "8812au_ch6|0x0bda|0x8812|6|2437"
    "8812au_ch36|0x0bda|0x8812|36|5180"
    "8821au_ch6|0x2357|0x0120|6|2437"
    "8814au_ch6|0x0bda|0x8813|6|2437"
    "8822bu_ch6|0x2357|0x012d|6|2437"
    "8821cu_ch6|0x0bda|0xc811|6|2437"
    "8822cu_ch6|0x0bda|0xc812|6|2437"
    "8822eu_ch6|0x0bda|0xa81a|6|2437"
)
GAINS="0,4,8,16,24,31"
RATE="10e6"
RX_GAIN="40"
UHD_ARGS="type=b200"
NFFT="8192"
DRIFT_SECS="60"
BRINGUP_S="6"       # seconds to let InitWrite arm the tone before capturing
                    # (8814AU's firmware-download bring-up needs ~12 — use
                    #  --bringup-s 14 for that cell)
OUTDIR="/tmp/devourer-cw-tone"
ONLY_CELL=""

while [ $# -gt 0 ]; do
    case "$1" in
        --cell) ONLY_CELL="$2"; shift 2 ;;
        --cells) IFS=';' read -r -a CELLS <<< "$2"; shift 2 ;;
        --gains) GAINS="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --rx-gain) RX_GAIN="$2"; shift 2 ;;
        --uhd-args) UHD_ARGS="$2"; shift 2 ;;
        --nfft) NFFT="$2"; shift 2 ;;
        --bringup-s) BRINGUP_S="$2"; shift 2 ;;
        --drift-secs) DRIFT_SECS="$2"; shift 2 ;;
        --outdir) OUTDIR="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

DEMO_PID=""
cleanup() {
    [ -n "$DEMO_PID" ] && kill "$DEMO_PID" 2>/dev/null || true
    pkill -x WiFiDriverTxDemo 2>/dev/null || true
    pkill -f "tests/cw_tone_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- build + venv --------------------------------------------------------------
echo "== building devourer =="
cmake --build "$ROOT/build" -j >/dev/null
[ -x "$DEMO" ] || { echo "missing $DEMO" >&2; exit 1; }

echo "== preparing uv venv (system site-packages for uhd) =="
if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found — install it or run cw_tone_probe.py with system python3." >&2
    exit 1
fi
[ -d "$HERE/.venv" ] || uv venv --system-site-packages "$HERE/.venv" >/dev/null
# cw_tone_probe.py only needs numpy + the system `uhd` module; the editable
# install of the whole tests package is unnecessary (and trips setuptools
# flat-layout discovery), so it's best-effort. numpy is pulled in directly.
uv pip install --python "$HERE/.venv/bin/python" -q numpy >/dev/null 2>&1 || true
PY="$HERE/.venv/bin/python"
"$PY" -c "import uhd, numpy" 2>/dev/null || PY="$(command -v python3)"
"$PY" -c "import uhd, numpy" 2>/dev/null || {
    echo "ERROR: neither venv nor system python3 can import uhd + numpy." >&2
    exit 1
}

mkdir -p "$OUTDIR"

# Launch a CW-tone demo session in the background. $1=vid $2=pid $3=channel $4=gain(''=none)
start_tone() {
    local vid="$1" pid="$2" ch="$3" gain="$4"
    local logf="$OUTDIR/demo_${pid}_ch${ch}_g${gain}.log"
    # `env` (not a bare prefix) so the optional gain assignment survives the
    # ${gain:+...} expansion — a prefix expanded from a parameter is treated as
    # a command word, not an assignment.
    env DEVOURER_VID="$vid" DEVOURER_PID="$pid" DEVOURER_CHANNEL="$ch" \
        DEVOURER_CW_TONE=1 ${gain:+DEVOURER_CW_TONE_GAIN="$gain"} \
        "$DEMO" >"$logf" 2>&1 &
    DEMO_PID=$!
    sleep "$BRINGUP_S"
    if ! kill -0 "$DEMO_PID" 2>/dev/null; then
        echo "  !! demo exited during bring-up — see $logf" >&2
        DEMO_PID=""
        return 1
    fi
}

stop_tone() {
    [ -n "$DEMO_PID" ] || return 0
    kill -INT "$DEMO_PID" 2>/dev/null || true
    wait "$DEMO_PID" 2>/dev/null || true
    DEMO_PID=""
}

probe() {   # $1=center_mhz $2=label extra... -> appends JSON to $JSONL
    local mhz="$1" label="$2"; shift 2
    "$PY" "$HERE/cw_tone_probe.py" --freq "${mhz}e6" --rate "$RATE" \
        --gain "$RX_GAIN" --args "$UHD_ARGS" --nfft "$NFFT" \
        --label "$label" --json "$JSONL" "$@"
}

run_cell() {
    local name="$1" vid="$2" pid="$3" ch="$4" mhz="$5"
    JSONL="$OUTDIR/${name}.jsonl"
    : > "$JSONL"
    echo
    echo "########## cell $name  ($vid:$pid, ch$ch, SDR $mhz MHz) ##########"

    # 1) presence + purity + 2) power monotonicity: one launch per gain index.
    echo "-- gain sweep ($GAINS) --"
    IFS=',' read -r -a garr <<< "$GAINS"
    for g in "${garr[@]}"; do
        if start_tone "$vid" "$pid" "$ch" "$g"; then
            probe "$mhz" "${name}_g${g}" --secs 0.2 || true
            stop_tone
        fi
    done

    # 3) clean stop: arm mid-gain, confirm carrier present, SIGINT, confirm gone.
    echo "-- clean stop --"
    if start_tone "$vid" "$pid" "$ch" 16; then
        probe "$mhz" "${name}_stop_before" --secs 0.2 || true
        stop_tone
        sleep 1
        probe "$mhz" "${name}_stop_after" --secs 0.2 || true
    fi

    # 3b) normal beacon TX resumes (no CW tone) — proves the chip restored.
    echo "-- normal TX resumes --"
    local rlog="$OUTDIR/${name}_normal_tx.log"
    DEVOURER_VID="$vid" DEVOURER_PID="$pid" DEVOURER_CHANNEL="$ch" \
        "$DEMO" >"$rlog" 2>&1 &
    DEMO_PID=$!
    sleep "$BRINGUP_S"
    if grep -q "first_tx_submit" "$rlog"; then
        echo "   OK: normal beacon TX submitted (first_tx_submit seen)"
    else
        echo "   WARN: no first_tx_submit in $rlog — check TX resume"
    fi
    stop_tone

    # 4) drift hold.
    echo "-- drift hold (${DRIFT_SECS}s) --"
    if start_tone "$vid" "$pid" "$ch" 16; then
        probe "$mhz" "${name}_drift" --drift-secs "$DRIFT_SECS" || true
        stop_tone
    fi

    echo "-- summary for $name (JSON: $JSONL) --"
    "$PY" - "$JSONL" <<'PYEOF'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
sweep = [(r["label"], r) for r in rows if r.get("mode") == "psd" and "_g" in r["label"] and "stop" not in r["label"]]
if sweep:
    print("   gain-sweep peak_dbfs / c2s_db / peak_offset_khz:")
    prev = None; mono = True
    for lab, r in sweep:
        g = lab.rsplit("_g", 1)[-1]
        flag = ""
        if prev is not None and r["peak_dbfs"] < prev - 0.5:
            mono = False; flag = "  <-- drop"
        prev = r["peak_dbfs"]
        print(f"     g={g:>2}  peak={r['peak_dbfs']:7.2f}  c2s={r['c2s_db']:6.2f}  off={r['peak_offset_khz']:8.2f}{flag}")
    print(f"   power monotonic vs gain: {'YES' if mono else 'NO'}")
before = next((r for _, r in [(r['label'], r) for r in rows] if r['label'].endswith('_stop_before')), None)
after  = next((r for _, r in [(r['label'], r) for r in rows] if r['label'].endswith('_stop_after')), None)
if before and after:
    gone = after["peak_to_floor_db"] < before["peak_to_floor_db"] - 10
    print(f"   clean stop: before peak-to-floor={before['peak_to_floor_db']:.1f}dB, "
          f"after={after['peak_to_floor_db']:.1f}dB -> carrier {'GONE' if gone else 'STILL PRESENT'}")
drift = next((r for r in rows if r.get("mode") == "drift"), None)
if drift:
    print(f"   drift: span={drift['peak_offset_khz_span']}kHz std={drift['peak_offset_khz_std']}kHz "
          f"over {drift['drift_secs']}s ({drift['n_captures']} captures)")
PYEOF
}

for cell in "${CELLS[@]}"; do
    IFS='|' read -r name vid pid ch mhz <<< "$cell"
    [ -n "$ONLY_CELL" ] && [ "$ONLY_CELL" != "$name" ] && continue
    run_cell "$name" "$vid" "$pid" "$ch" "$mhz"
done

echo
echo "== done. per-cell JSONL + demo logs under $OUTDIR =="
