#!/usr/bin/env bash
# rtl8733b_lifecycle_soak.sh — bounded warm lifecycle/health repetition.
#
# This is deliberately a warm/reset test. It does not claim a VBUS-cold boot;
# use adapter_doctor_cold.sh with a switchable hub for that gate. Each cycle:
#   - selects one exact RTL8733B USB topology,
#   - runs doctor with a hard wall-clock timeout,
#   - requires stable EFUSE, firmware-ready, clean RX parsing, bounded thermal,
#     TSSI rollback, and the RTL8733B card-disabled register readback,
#   - verifies that the same USB device remains enumerated.
#
# Usage:
#   sudo BUILD=build-pr REPS=20 CHANNELS="6 36" \
#     bash tests/rtl8733b_lifecycle_soak.sh
#
# Knobs:
#   BUILD                 build directory (default: <repo>/build)
#   VID/PID               exact DUT identity (default: 0x0bda/0xf72b)
#   BUS/PORT/SYSFS        topology (default: 1 / 1.2 / BUS-PORT)
#   REPS                  number of cycles (default: 10)
#   CHANNELS              whitespace-separated channel rotation (default: 6)
#   READS                 physical EFUSE reads per cycle (default: 4)
#   LISTEN_SECS           ambient RX duration per cycle (default: 3)
#   TIMEOUT_SECS          doctor wall-clock bound (default: 30)
#   COOLDOWN_SECS         delay between cycles (default: 1)
#   MAX_THERMAL_DELTA     maximum raw delta from EFUSE baseline (default: 20)
#   MAX_THERMAL_SPREAD    maximum raw spread over all cycles (default: 12)
#   OUT                   output directory (default: timestamp under /tmp)
#
# A SUSPECT doctor verdict is accepted only for reason 0x40 (no clean ambient
# traffic without --expect-traffic). All hardware/firmware/EFUSE invariants
# must still pass. Exit 0 means every requested lifecycle passed.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
DOCTOR="$BUILD/doctor"
PROBE="$BUILD/rtl8733bprobe"
VID="${VID:-0x0bda}"
PID="${PID:-0xf72b}"
BUS="${BUS:-1}"
PORT="${PORT:-1.2}"
SYSFS="${SYSFS:-$BUS-$PORT}"
REPS="${REPS:-10}"
CHANNELS="${CHANNELS:-6}"
READS="${READS:-4}"
LISTEN_SECS="${LISTEN_SECS:-3}"
TIMEOUT_SECS="${TIMEOUT_SECS:-30}"
COOLDOWN_SECS="${COOLDOWN_SECS:-1}"
MAX_THERMAL_DELTA="${MAX_THERMAL_DELTA:-20}"
MAX_THERMAL_SPREAD="${MAX_THERMAL_SPREAD:-12}"
OUT="${OUT:-/tmp/devourer-rtl8733b-soak/$(date +%Y%m%d-%H%M%S)}"

die() { echo "[rtl8733b-soak] FATAL: $*" >&2; exit 3; }
is_uint() { [[ "$1" =~ ^[0-9]+$ ]]; }

# Canonicalise the identity knobs to lowercase 0xNNNN. sysfs reports lowercase
# hex without the prefix, so a VID=0x0BDA (or a decimal PID) must not fail
# verification of a present, correct device.
for name in VID PID; do
  [[ "${!name}" =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]] || die "$name is not numeric: ${!name}"
  printf -v "$name" '0x%04x' "$((${!name}))"
done

[[ "$(id -u)" -eq 0 ]] || die "must run as root"
[[ -x "$DOCTOR" ]] || die "$DOCTOR missing — build doctor first"
[[ -x "$PROBE" ]] || die "$PROBE missing — build rtl8733bprobe first"
for value in "$BUS" "$REPS" "$READS" "$LISTEN_SECS" "$TIMEOUT_SECS" \
             "$COOLDOWN_SECS" "$MAX_THERMAL_DELTA" "$MAX_THERMAL_SPREAD"; do
  is_uint "$value" || die "numeric knob has invalid value: $value"
done
[[ "$REPS" -gt 0 ]] || die "REPS must be positive"
[[ "$TIMEOUT_SECS" -ge 10 ]] || die "TIMEOUT_SECS must be at least 10"

read -r -a channel_list <<< "$CHANNELS" || true
[[ "${#channel_list[@]}" -gt 0 ]] || die "CHANNELS is empty"
for channel in "${channel_list[@]}"; do
  is_uint "$channel" || die "invalid channel: $channel"
done

SYSFS_DIR="/sys/bus/usb/devices/$SYSFS"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'rep\tchannel\trc\tverdict\treasons\trx_ok\trx_crc\tthermal\tbaseline\tdelta\telapsed_s\tsha256\n' > "$SUMMARY"

probe_off() {
  local address
  [[ -r "$SYSFS_DIR/devnum" ]] || return 1
  address="$(sed -n '1p' "$SYSFS_DIR/devnum")"
  timeout -k 3 -s INT 10 "$PROBE" off --vid "$VID" --pid "$PID" \
    --bus "$BUS" --address "$address"
}

verify_usb() {
  local vid pid
  [[ -r "$SYSFS_DIR/idVendor" && -r "$SYSFS_DIR/idProduct" ]] || return 1
  vid="$(sed -n '1p' "$SYSFS_DIR/idVendor")"
  pid="$(sed -n '1p' "$SYSFS_DIR/idProduct")"
  [[ "0x${vid,,}" = "$VID" ]] || return 1
  [[ "0x${pid,,}" = "$PID" ]] || return 1
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ "$rc" -ne 0 ]] && verify_usb; then
    echo "[rtl8733b-soak] failure cleanup: requesting explicit card-disable" >&2
    probe_off >> "$OUT/failure-cleanup.log" 2>&1 || true
  fi
  echo "[rtl8733b-soak] logs: $OUT"
  exit "$rc"
}
trap cleanup EXIT INT TERM

verify_usb || die "$VID:$PID is not present at $SYSFS"

min_thermal=999
max_thermal=-1
for ((rep = 1; rep <= REPS; ++rep)); do
  channel="${channel_list[$(((rep - 1) % ${#channel_list[@]}))]}"
  log="$OUT/rep$(printf '%03d' "$rep")-ch$channel.log"
  start=$SECONDS
  echo "[rtl8733b-soak] rep $rep/$REPS ch$channel"

  # A non-zero doctor exit is data, not a script error — capture it explicitly
  # so `set -e` does not abort before the per-cycle gates below can run.
  doctor_rc=0
  timeout -k 5 -s INT "$TIMEOUT_SECS" env \
    DEVOURER_LOG_LEVEL=info DEVOURER_EVENTS=stdout \
    "$DOCTOR" --vid "$VID" --pid "$PID" --bus "$BUS" --port "$PORT" \
    --channel "$channel" --reads "$READS" --listen-secs "$LISTEN_SECS" \
    > "$log" 2>&1 || doctor_rc=$?
  elapsed=$((SECONDS - start))

  # `|| true`: a missing verdict line is a graded failure below, not a reason
  # to abort under `set -o pipefail`.
  verdict_line="$(grep -F '"ev":"doctor.verdict"' "$log" | tail -1 || true)"
  verdict="$(sed -n 's/.*"verdict":"\([^"]*\)".*/\1/p' <<< "$verdict_line")"
  reasons="$(sed -n 's/.*"reasons":"\([^"]*\)".*/\1/p' <<< "$verdict_line")"
  rx_ok="$(sed -n 's/.*"rx_ok":\([0-9]*\).*/\1/p' <<< "$verdict_line")"
  rx_crc="$(sed -n 's/.*"rx_crc":\([0-9]*\).*/\1/p' <<< "$verdict_line")"
  thermal_pair="$(sed -n 's/.*ready for bounded monitor injection:.*thermal=\([0-9][0-9]*\)\/\([0-9][0-9]*\).*/\1 \2/p' "$log" | tail -1)"
  read -r thermal baseline <<< "$thermal_pair" || true
  thermal="${thermal:-}"
  baseline="${baseline:-}"
  digest="$(sha256sum "$log" | cut -d' ' -f1)"

  failed=0
  [[ "$doctor_rc" -eq 0 || ("$doctor_rc" -eq 1 && "$reasons" = "0x40") ]] || failed=1
  [[ -n "$verdict_line" ]] || failed=1
  # Anchor the JSON value delimiter: an unanchored '"efuse_reads":4' also
  # matches '"efuse_reads":40'.
  grep -qE '"efuse_reads":'"$READS"'[,}]' "$log" || failed=1
  grep -qE '"efuse_mismatch":0[,}]' "$log" || failed=1
  grep -qE '"efuse_bad_id":0[,}]' "$log" || failed=1
  grep -qF '"efuse_id":"0x8129"' "$log" || failed=1
  grep -qE '"fw_ready":1[,}]' "$log" || failed=1
  grep -qE '"init":1[,}]' "$log" || failed=1
  grep -qF 'RTL8733B TSSI tracking disabled: rollback=1' "$log" || failed=1
  grep -qF 'RTL8733B power-off: ready=1 CR=0xea' "$log" || failed=1
  grep -qF 'RF_CTRL=0x00' "$log" || failed=1
  if [[ "$LISTEN_SECS" -gt 0 ]]; then
    grep -Eq 'RTL8733B RX: stopped .*malformed=0 agg_mismatch=0' "$log" || failed=1
    # Negated match: `! grep || failed=1`, not `grep && failed=1` — the latter
    # exits non-zero on the (expected) no-match case, aborting under `set -e`.
    ! grep -qF '"ev":"rx.parse_abort"' "$log" || failed=1
  fi
  verify_usb || failed=1

  if ! is_uint "$thermal" || ! is_uint "$baseline"; then
    failed=1
    delta=-1
  else
    delta=$((thermal - baseline))
    [[ "$delta" -ge 0 ]] || delta=$((-delta))
    [[ "$delta" -le "$MAX_THERMAL_DELTA" ]] || failed=1
    [[ "$thermal" -ge "$min_thermal" ]] || min_thermal=$thermal
    [[ "$thermal" -le "$max_thermal" ]] || max_thermal=$thermal
    [[ $((max_thermal - min_thermal)) -le "$MAX_THERMAL_SPREAD" ]] || failed=1
  fi

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$rep" "$channel" "$doctor_rc" "${verdict:-missing}" \
    "${reasons:-missing}" "${rx_ok:-missing}" "${rx_crc:-missing}" \
    "${thermal:-missing}" "${baseline:-missing}" "$delta" "$elapsed" \
    "$digest" >> "$SUMMARY"

  if [[ "$failed" -ne 0 ]]; then
    echo "[rtl8733b-soak] FAIL rep=$rep rc=$doctor_rc verdict=${verdict:-missing} reasons=${reasons:-missing} thermal=${thermal:-missing}/${baseline:-missing}" >&2
    tail -n 30 "$log" >&2
    exit 1
  fi

  echo "[rtl8733b-soak] PASS rep=$rep verdict=$verdict rx=${rx_ok:-0}/${rx_crc:-0} thermal=$thermal/$baseline elapsed=${elapsed}s"
  [[ "$rep" -eq "$REPS" ]] || sleep "$COOLDOWN_SECS"
done

echo "[rtl8733b-soak] PASS $REPS/$REPS cycles; thermal range $min_thermal..$max_thermal"
