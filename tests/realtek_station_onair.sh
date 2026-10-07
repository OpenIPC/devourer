#!/usr/bin/env bash
# realtek_station_onair.sh - both halves of AdapterCaps::station_mode_ok's bar
# for a Realtek station armed through IRadio::SetStationIdentity:
#
#   DOWN  unicast addressed to the station is received AND acknowledged;
#   UP    the station's own unicast is acknowledged.
#
# The transmitter is the only party that knows whether its frame was
# acknowledged, so each half asks the transmitter: a devourer Realtek adapter
# reading its own per-frame CCX reports (tx.report), retries ~0 when the
# frame was answered and pinned at RETRY_LIMIT when it was not - the
# instrument tests/ack_txreport_matrix.sh and tests/mt7612u_sta_autoack.sh
# use. Unlike the MT7612U cells, the DUT here is armed through the library
# seam itself (rxdemo / txdemo DEVOURER_STA_IDENTITY, which emits `sta.arm`).
#
# DOWN - the DUT runs rxdemo, armed as own=self, BSSID; the PEER (a second
# Realtek adapter) injects ACK-requesting QoS-Data with TA = BSSID, as the AP
# would:
#   A  DUT armed,     RA = DUT's own address  -> the claim: ACKed, and the
#                                                DUT's rx.seq shows it arrived
#   B  DUT armed,     RA = NOBODY             -> control: the instrument fails
#   C  DUT absent,    RA = DUT's own address  -> control: the DUT is the ACKer
#                                                (the previous arm's Stop()
#                                                cleared its station arm)
#   D  DUT unarmed,   RA = DUT's own address  -> the arm is what answers
#   E  DUT armed then cleared (DEVOURER_STA_CLEAR_AFTER_MS), RA = own
#                                             -> Clear silences the port
# The bar is A against B and C, plus A's reception. D and E are verdicts on
# the arm itself and count toward the exit status; on a die whose bring-up
# already answers for its own address (the Jaguar1 8812 programs the EFUSE
# MAC into MACID) D is expected to fail - set EXPECT_UNARMED_SILENT=0 to
# report it without scoring it.
#
# UP - hostapd on AP_SYSFS holds BSSID; the DUT runs txdemo with
# DEVOURER_TX_WITH_RX=thread (the CCX reports ride the RX path), armed as
# own, BSSID, sending ACK-requesting QoS-Data with TA = own and a nonzero
# retry limit, and reads its own reports:
#   F  RA = BSSID   -> the claim: the AP acknowledges the station's unicast
#   G  RA = NOBODY  -> control: same transmitter, nobody answers
#   H  RA = BSSID, the DUT NOT armed -> reported, never scored: whether the
#                    TX side's ACK matching needs the arm at all (Jaguar2/3
#                    bring-up never programs MACID)
# hostapd's MAC acknowledges by address, so F holds for an unassociated
# station (the AP then deauths the "class 3" sender - expected, harmless).
#
# An arm is scored only when its transmitter kept airing through the window
# (no silence over MAX_GAP_MS before its first report, between its
# reports, or after the last one - see summarize) and carried MIN_REPORTS reports and its submission floor
# (MIN_SUBMITTED, scaled to the span it aired). Reception in arm A is
# judged against the peer's REPORTED frames, which aired; submitted frames
# left unreported at window close are printed separately.
#
# Exit status: 0 every verdict passed; 1 a verdict failed; 2 INCONCLUSIVE (an
# arm aborted, produced no reports, or the rig was refused); 3 interrupted.
#
#   sudo DUT_PID=0xc812 DUT_SYSFS=5-1 PEER_PID=0xb812 PEER_SYSFS=6-1 \
#        AP_SYSFS=1-1 tests/realtek_station_onair.sh
#
# Rig: DUT and PEER are devourer Realtek adapters (Jaguar1/2/3); AP_SYSFS is
# any adapter whose kernel driver supports AP mode. Temp-blacklist rtw88
# (CLAUDE.md, "Hardware testing"): it auto-probes every Realtek dongle at
# each enumeration, and the hand-back below re-enumerates both. Read
# AP_SYSFS from `lsusb -t` after its driver has loaded - it can move.
#
# Env: DUT_VID, DUT_PID, DUT_SYSFS, PEER_VID, PEER_PID, PEER_SYSFS, AP_SYSFS,
#      CH, BSSID, NOBODY, SECS, RETRY_LIMIT, RATE, GAP_US, HALF (both|down|up),
#      MIN_REPORTS, MIN_RX_PCT, MAX_GAP_MS, MIN_SUBMITTED,
#      EXPECT_UNARMED_SILENT, READY_TIMEOUT, OUT.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
cd "$ROOT" || exit 2
DUT_VID="${DUT_VID:-0x0bda}"
DUT_PID="${DUT_PID:-0xc812}"
DUT_SYSFS="${DUT_SYSFS:-}"
PEER_VID="${PEER_VID:-0x0bda}"
PEER_PID="${PEER_PID:-0xb812}"
PEER_SYSFS="${PEER_SYSFS:-}"
AP_SYSFS="${AP_SYSFS:-}"
CH="${CH:-6}"
BSSID="${BSSID:-02:42:75:05:d6:ab}"
NOBODY="${NOBODY:-02:00:00:de:ad:07}"
SECS="${SECS:-10}"
RETRY_LIMIT="${RETRY_LIMIT:-12}"
RATE="${RATE:-MCS3}"
GAP_US="${GAP_US:-5000}"
HALF="${HALF:-both}"
MIN_REPORTS="${MIN_REPORTS:-50}"
MIN_RX_PCT="${MIN_RX_PCT:-80}"
# Transmitter liveness: the longest silence allowed from an arm's first
# submit to its first CCX report, between two reports, and from its last
# report to its final tx.stats.
MAX_GAP_MS="${MAX_GAP_MS:-2000}"
# Per-arm floor on frames the transmitter submitted. Default: a quarter of
# the nominal GAP_US rate over the span the arm actually aired - its first
# submit to its final tx.stats (see summarize) - not over SECS, which also
# holds the transmitter's bring-up: an 8812BU peer has spent 6-9 s of a 10 s
# window in InitWrite and been scored a stall at 283-327 healthy frames.
# The slowest arm on record, an unacknowledged one at 12 retries, submitted
# ~900 in 10 s against the 500 a full window asks. A set MIN_SUBMITTED is a
# fixed floor instead.
MIN_SUBMITTED="${MIN_SUBMITTED:-}"
EXPECT_UNARMED_SILENT="${EXPECT_UNARMED_SILENT:-1}"
READY_TIMEOUT="${READY_TIMEOUT:-30}"
CLEAR_AFTER_MS="${CLEAR_AFTER_MS:-2000}"
# Unset: a fresh private directory (sta_out_prepare in the lib).
OUT="${OUT:-}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
case "$HALF" in both|down|up) ;; *) echo "HALF must be both, down or up"; exit 2 ;; esac
for v in SECS RETRY_LIMIT GAP_US MIN_REPORTS MIN_RX_PCT MAX_GAP_MS READY_TIMEOUT CLEAR_AFTER_MS CH; do
  case "${!v}" in ''|*[!0-9]*) echo "$v must be a non-negative integer"; exit 2 ;; esac
done
case "$MIN_SUBMITTED" in *[!0-9]*) echo "MIN_SUBMITTED must be a non-negative integer"; exit 2 ;; esac
# RETRY_LIMIT 0 would make every control indistinguishable from a one-shot
# send: the retry count is the instrument.
if [ "$RETRY_LIMIT" -lt 1 ] || [ "$RETRY_LIMIT" -gt 63 ]; then
  echo "RETRY_LIMIT must be 1..63"; exit 2
fi
[ -n "$DUT_SYSFS" ] || { echo "DUT_SYSFS is required (lsusb -t)"; exit 2; }
if [ "$HALF" != up ] && [ -z "$PEER_SYSFS" ]; then
  echo "PEER_SYSFS is required for the DOWN half"; exit 2
fi
if [ "$HALF" != down ]; then
  [ -n "$AP_SYSFS" ] || { echo "AP_SYSFS is required for the UP half"; exit 2; }
  command -v hostapd >/dev/null || { echo "hostapd is required for the UP half"; exit 2; }
fi
for b in rxdemo txdemo; do
  [ -x "$BUILD/$b" ] || { echo "$BUILD/$b is not built"; exit 2; }
done

# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"

# The AP guard (tests/mt7612u_sta_identity.sh's): cleanup re-enumerates
# AP_SYSFS as root, so it must be a USB device that is not a hub, carrying a
# wireless netdev with no default route on a phy that supports AP mode. Sets
# AP_IF and PHY. Run in the preflight, before the DOWN half spends its
# minute, and again at the start of UP - the adapter can move in between.
ap_refuse() { echo "refusing AP_SYSFS=$AP_SYSFS: $* - cleanup would re-enumerate it."; exit 2; }
ap_guard() {
  local fam
  [ -n "$(cat "/sys/bus/usb/devices/$AP_SYSFS/idVendor" 2>/dev/null)" ] ||
    ap_refuse "not a USB device - if its driver just loaded it may have moved; re-read lsusb -t"
  [ "$(cat "/sys/bus/usb/devices/$AP_SYSFS/bDeviceClass" 2>/dev/null)" != "09" ] || ap_refuse "a hub"
  AP_IF=$(sta_first_netdev "$AP_SYSFS")
  [ -n "$AP_IF" ] || ap_refuse "no network interface on it"
  [ -e "/sys/class/net/$AP_IF/phy80211" ] || ap_refuse "$AP_IF is not wireless"
  for fam in -4 -6; do
    ip "$fam" route show default 2>/dev/null | grep -qw "dev $AP_IF" &&
      ap_refuse "$AP_IF carries a default route"
  done
  PHY=$(basename "$(readlink -f "/sys/class/net/$AP_IF/phy80211")")
  iw phy "$PHY" info 2>/dev/null | grep -q '\* AP$' || ap_refuse "$AP_IF ($PHY) does not support AP mode"
}
[ "$HALF" = down ] || ap_guard
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init dut peer hostapd

# --- adapter identity and hand-back -----------------------------------------
# The DUT and the PEER are opened by rxdemo / txdemo, whose libusb open
# detaches the kernel driver and never re-attaches it: each goes through the
# lib's sta_dev_record / sta_dev_opened / sta_dev_handback.

if [ "$DUT_SYSFS" = "${PEER_SYSFS:-x}" ] || [ "$DUT_SYSFS" = "${AP_SYSFS:-x}" ] ||
   { [ -n "$PEER_SYSFS" ] && [ "$PEER_SYSFS" = "${AP_SYSFS:-x}" ]; }; then
  echo "DUT_SYSFS, PEER_SYSFS and AP_SYSFS must be three different adapters"
  sta_lock_release; exit 2
fi
sta_dev_record dut "$DUT_SYSFS" "$DUT_VID" "$DUT_PID" || { sta_lock_release; exit 2; }
if [ "$HALF" != up ]; then
  sta_dev_record peer "$PEER_SYSFS" "$PEER_VID" "$PEER_PID" || { sta_lock_release; exit 2; }
fi

AP_IF=""
AP_ID=""
AP_REENUM=no
CLEANED=no
# shellcheck disable=SC2317  # reached through the traps below
cleanup() {
  [ "$CLEANED" = yes ] && return 0
  CLEANED=yes
  local dut_gone=0 peer_gone=0
  sta_pid_kill peer INT || peer_gone=1
  sta_pid_kill dut INT || dut_gone=1
  sta_pid_kill hostapd
  # Only once a process has really exited: re-enumerating an adapter still
  # inside its de-init is what the hand-back must not do.
  if [ "$dut_gone" = 0 ]; then sta_dev_handback dut "$DUT_SYSFS"
  else echo "DUT still running - not re-enumerating $DUT_SYSFS"; fi
  if [ "$peer_gone" = 0 ]; then sta_dev_handback peer "${PEER_SYSFS:-}"
  else echo "peer still running - not re-enumerating $PEER_SYSFS"; fi
  if [ "$AP_REENUM" = yes ]; then
    # hostapd's `bssid=` leaves the interface carrying that address after it
    # exits; re-enumerate rather than bounce the link
    # (tests/mt7612u_sta_identity.sh has the history).
    if [ -n "$AP_ID" ] && [ "$(sta_usb_id "$AP_SYSFS")" = "$AP_ID" ]; then
      echo 0 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
      sleep 3
      echo 1 > "/sys/bus/usb/devices/$AP_SYSFS/authorized" 2>/dev/null
      sleep 8
      AP_IF=$(sta_first_netdev "$AP_SYSFS")
      [ -n "$AP_IF" ] && {
        rfkill unblock wlan 2>/dev/null
        ip link set "$AP_IF" up 2>/dev/null
        nmcli device set "$AP_IF" managed yes >/dev/null 2>&1
      }
    else
      echo "AP_SYSFS=$AP_SYSFS no longer names the accepted AP ($AP_ID) - not re-enumerating it"
    fi
  fi
  sta_lock_release
}
trap cleanup EXIT
# AND IT MUST STOP: with INT/TERM on the EXIT trap the shell would run
# cleanup and then carry on into the next arm.
trap 'cleanup; exit 3' INT TERM

sel_env() { # $1 sysfs -> DEVOURER_USB_BUS / _PORT assignments
  printf 'DEVOURER_USB_BUS=%s DEVOURER_USB_PORT=%s' "${1%%-*}" "${1#*-}"
}

# Is PID running? `kill -0` is not enough: a background child that has exited
# but is not yet reaped is a zombie, and kill -0 still succeeds on it. The
# state field of /proc/PID/stat (after the parenthesised command name) is Z
# for a zombie.
proc_running() { # $1 pid
  local st
  st=$(sed 's/^.*) //' "/proc/$1/stat" 2>/dev/null | cut -d' ' -f1)
  [ -n "$st" ] && [ "$st" != Z ] && [ "$st" != X ]
}

# Wait for a regex in a file while the process lives. 0 found, 1 not.
wait_for() { # $1 pid, $2 file, $3 regex, $4 timeout s
  local t=0
  until grep -qE "$3" "$2" 2>/dev/null; do
    proc_running "$1" || return 1
    [ "$t" -ge "$4" ] && return 1
    sleep 1; t=$((t + 1))
  done
  return 0
}

# One summary line from a transmitter's JSONL (and, optionally, the DUT's
# rx.seq stream for frames from TA): reports, submitted, unreported,
# ok_pct, retries_mean, lead_ms, max_gap_ms, tail_ms, live, aired_ms,
# min_submitted, rx_distinct.
#
# One clock: tx.report, the first tx.frame (txdemo's first submit) and the
# final tx.stats all carry t in the host-monotonic tx.report timebase.
#
# LIVENESS. A report is a frame that aired, so the reports' own timestamps
# show whether the transmitter kept airing through the window: lead_ms is
# the silence from the first submit to the first report, max_gap_ms the
# longest silence between two reports, tail_ms the silence from the last
# report to the final tx.stats (clamped at 0: that t can precede the last
# report's by a few ms). live=0 when any of them exceeds MAX_GAP_MS,
# or a timestamp it needs is missing - an arm that aired a burst and
# stalled, or that started, stalled and burst at the end, which MIN_REPORTS
# alone would accept.
#
# FLOOR. aired_ms runs from the first submit to the final tx.stats - not
# over SECS, which also holds the transmitter's bring-up, and not from the
# first report, which a late burst would move to the end. min_submitted
# is a quarter of the GAP_US rate over it (0 at GAP_US=0, which has no
# nominal rate), or MIN_SUBMITTED when set.
summarize() { # $1 tx jsonl, $2 tag, $3 dut jsonl or ""
  python3 - "$1" "$2" "${3:-}" "$MAX_GAP_MS" "$GAP_US" "$MIN_SUBMITTED" <<'PYEOF'
import json, sys
tx, tag, rx, max_gap = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
gap_us, fixed_floor = int(sys.argv[5]), sys.argv[6]
n = okc = retries = 0
submitted = 0
ts = []
final_t = None
first_submit_t = None
for line in open(tx, errors='replace'):
    if not line.startswith('{'):
        continue
    try:
        e = json.loads(line)
    except ValueError:
        continue
    if e.get('ev') == 'tx.report':
        n += 1
        okc += 1 if e.get('ok') else 0
        retries += int(e.get('retries', 0) or 0)
        if 't' in e:
            ts.append(int(e['t']))
    elif e.get('ev') == 'tx.frame':
        if first_submit_t is None and 't' in e:
            first_submit_t = int(e['t'])
    elif e.get('ev') == 'tx.stats':
        submitted = int(e.get('submitted', submitted) or submitted)
        if e.get('final') and 't' in e:
            final_t = int(e['t'])
gap = max((b - a for a, b in zip(ts, ts[1:])), default=0)
tail = max(final_t - ts[-1], 0) if (final_t is not None and ts) else None
lead = (ts[0] - first_submit_t) if (first_submit_t is not None and ts) else None
live = int(bool(ts) and tail is not None and lead is not None
           and lead <= max_gap and gap <= max_gap and tail <= max_gap)
out = (f"{tag} reports={n} submitted={submitted} "
       f"unreported={max(submitted - n, 0)}")
if n:
    out += f" ok_pct={100.0*okc/n:.1f} retries_mean={retries/n:.2f}"
out += (f" lead_ms={'none' if lead is None else lead} max_gap_ms={gap}"
        f" tail_ms={'none' if tail is None else tail} live={live}")
aired = (final_t - first_submit_t) \
    if (final_t is not None and first_submit_t is not None) else 0
if fixed_floor:
    floor = int(fixed_floor)
else:
    floor = aired * 1000 // gap_us // 4 if gap_us > 0 else 0
out += f" aired_ms={aired} min_submitted={floor}"
if rx:
    seen = set()
    try:
        for line in open(rx, errors='replace'):
            if '"ev":"rx.seq"' not in line:
                continue
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if not e.get('crc'):
                seen.add(e.get('pctr'))
    except OSError:
        pass
    out += f" rx_distinct={len(seen)}"
print(out)
PYEOF
}

# --- DOWN: one arm ----------------------------------------------------------
# $1 tag, $2 RA, $3 DUT mode: armed | unarmed | cleared | absent.
# Writes the summary line (or "<tag> ABORTED <why>") to $OUT/res_<tag>.
down_arm() {
  local tag="$1" ra="$2" mode="$3" dut="" peer rc=0 t0 t1
  local res="$OUT/res_$tag"
  : > "$res"
  if [ "$mode" != absent ]; then
    local sta_env=""
    case "$mode" in
      armed)   sta_env="DEVOURER_STA_IDENTITY=self,$BSSID" ;;
      cleared) sta_env="DEVOURER_STA_IDENTITY=self,$BSSID DEVOURER_STA_CLEAR_AFTER_MS=$CLEAR_AFTER_MS" ;;
    esac
    sta_dev_opened dut
    # shellcheck disable=SC2046,SC2086  # word-split assignments on purpose
    env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" $(sel_env "$DUT_SYSFS") \
        DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
        DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA="$BSSID" $sta_env \
        "$BUILD/rxdemo" >"$OUT/dut_$tag.jsonl" 2>"$OUT/dut_$tag.err" &
    dut=$!
    sta_pid_record dut "$dut"
    local ready='ring of .* URBs submitted' file="$OUT/dut_$tag.err"
    case "$mode" in
      armed)   ready='"ev":"sta.arm"'; file="$OUT/dut_$tag.jsonl" ;;
      cleared) ready='"ev":"sta.clear"'; file="$OUT/dut_$tag.jsonl" ;;
    esac
    if ! wait_for "$dut" "$file" "$ready" "$READY_TIMEOUT"; then
      echo "$tag ABORTED the DUT never reached '$ready': $(tail -1 "$OUT/dut_$tag.err" 2>/dev/null)" > "$res"
      sta_pid_kill dut INT; return 0
    fi
    case "$mode" in
      armed|cleared)
        if ! grep -q '"ev":"sta.arm","ok":1' "$OUT/dut_$tag.jsonl"; then
          echo "$tag ABORTED SetStationIdentity was refused: $(grep -m1 'station identity' "$OUT/dut_$tag.err")" > "$res"
          sta_pid_kill dut INT; return 0
        fi ;;
    esac
    if [ "$mode" = cleared ] &&
       ! grep -q '"ev":"sta.clear","ok":1' "$OUT/dut_$tag.jsonl"; then
      echo "$tag FAILCLEAR ClearStationIdentity did not verify its rollback" > "$res"
      sta_pid_kill dut INT; return 0
    fi
    # The arm sits after the first RX frame; give the ring a moment either way.
    sleep 2
  fi

  t0=$(date +%s)
  sta_dev_opened peer
  # shellcheck disable=SC2046  # word-split assignments on purpose
  env DEVOURER_VID="$PEER_VID" DEVOURER_PID="$PEER_PID" $(sel_env "$PEER_SYSFS") \
      DEVOURER_CHANNEL="$CH" \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$ra" DEVOURER_TX_SA="$BSSID" \
      DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PAYLOAD_BYTES=200 \
      DEVOURER_TX_GAP_US="$GAP_US" DEVOURER_TX_REPORT=1 \
      DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
      DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=warn \
      timeout -s INT -k 3 "$SECS" "$BUILD/txdemo" \
      >"$OUT/peer_$tag.jsonl" 2>"$OUT/peer_$tag.err" &
  peer=$!
  sta_pid_record peer "$peer"
  wait "$peer"; rc=$?
  rm -f "$OUT/.pid_peer"
  t1=$(date +%s)
  # timeout(1) returns 124 when it delivered the INT: the normal end.
  if [ "$rc" -ne 124 ]; then
    echo "$tag ABORTED the peer exited early (status $rc): $(tail -1 "$OUT/peer_$tag.err" 2>/dev/null)" > "$res"
    [ -n "$dut" ] && sta_pid_kill dut INT
    return 0
  fi
  if [ $((t1 - t0)) -gt $((SECS + 5)) ]; then
    echo "$tag ABORTED the peer overran its ${SECS}s window" > "$res"
    [ -n "$dut" ] && sta_pid_kill dut INT
    return 0
  fi
  # LIVENESS AFTER THE WINDOW: a DUT that died mid-window reads ~0% ACKed,
  # which is a control's PASSING value.
  if [ -n "$dut" ] && ! proc_running "$dut"; then
    echo "$tag ABORTED the DUT died during the window: $(tail -1 "$OUT/dut_$tag.err" 2>/dev/null)" > "$res"
    rm -f "$OUT/.pid_dut"; return 0
  fi
  if [ -n "$dut" ]; then
    sta_pid_kill dut INT || {
      echo "$tag ABORTED the DUT did not stop" > "$res"; return 0; }
    summarize "$OUT/peer_$tag.jsonl" "$tag" "$OUT/dut_$tag.jsonl" > "$res"
  else
    summarize "$OUT/peer_$tag.jsonl" "$tag" "" > "$res"
  fi
  return 0
}

# --- UP: one arm ------------------------------------------------------------
up_arm() { # $1 tag, $2 RA, $3 armed | unarmed (default armed)
  local tag="$1" ra="$2" mode="${3:-armed}" dut rc=0 t0 t1 sta_env=""
  local res="$OUT/res_$tag"
  : > "$res"
  [ "$mode" = armed ] && sta_env="DEVOURER_STA_IDENTITY=$OWN,$BSSID"
  t0=$(date +%s)
  sta_dev_opened dut
  # shellcheck disable=SC2046,SC2086  # word-split assignments on purpose
  env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" $(sel_env "$DUT_SYSFS") \
      DEVOURER_CHANNEL="$CH" $sta_env \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$ra" DEVOURER_TX_SA="$OWN" \
      DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PAYLOAD_BYTES=200 \
      DEVOURER_TX_GAP_US="$GAP_US" DEVOURER_TX_REPORT=1 \
      DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
      DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=info \
      timeout -s INT -k 3 "$((SECS + 8))" "$BUILD/txdemo" \
      >"$OUT/dut_$tag.jsonl" 2>"$OUT/dut_$tag.err" &
  dut=$!
  sta_pid_record dut "$dut"
  wait "$dut"; rc=$?
  rm -f "$OUT/.pid_dut"
  t1=$(date +%s)
  if [ "$mode" = armed ] &&
     ! grep -q '"ev":"sta.arm","ok":1' "$OUT/dut_$tag.jsonl"; then
    echo "$tag ABORTED the DUT's SetStationIdentity was refused or never ran: $(grep -m1 -i 'station identity\|error' "$OUT/dut_$tag.err")" > "$res"
    return 0
  fi
  if [ "$mode" = unarmed ] && grep -q '"ev":"sta.arm"' "$OUT/dut_$tag.jsonl"; then
    echo "$tag ABORTED the unarmed DUT ran an arm" > "$res"
    return 0
  fi
  if [ "$rc" -ne 124 ]; then
    echo "$tag ABORTED the DUT exited early (status $rc): $(tail -1 "$OUT/dut_$tag.err" 2>/dev/null)" > "$res"
    return 0
  fi
  if [ $((t1 - t0)) -gt $((SECS + 13)) ]; then
    echo "$tag ABORTED the DUT overran its window" > "$res"; return 0
  fi
  # The AP must still be up at the end, or G's "nobody answered" and F's
  # failure would both be the AP's absence.
  if ! iw dev "$AP_IF" info 2>/dev/null | grep -q 'type AP'; then
    echo "$tag ABORTED the AP left AP mode during the window" > "$res"; return 0
  fi
  summarize "$OUT/dut_$tag.jsonl" "$tag" "" > "$res"
  return 0
}

pass=0; fail=0; inconclusive=0
ok()  { pass=$((pass+1)); printf '  PASS  %s\n' "$*"; }
bad() { fail=$((fail+1)); printf '  FAIL  %s\n' "$*"; }
inc() { inconclusive=$((inconclusive+1)); printf '  INCONCLUSIVE  %s\n' "$*"; }
field() { sed -n "s/.* $2=\\([0-9.]*\\).*/\\1/p" "$OUT/res_$1"; }
# A usable arm: not aborted, carrying at least MIN_REPORTS reports and its
# min_submitted submissions, and with a transmitter that kept airing through
# the window (live=1, see summarize). Anything else leaves its verdicts
# INCONCLUSIVE.
usable() {
  local r n sub floor
  r=$(cat "$OUT/res_$1" 2>/dev/null)
  case "$r" in
    *ABORTED*|*FAILCLEAR*|'') return 1 ;;
  esac
  case "$r" in *" live=1"*) ;; *) return 1 ;; esac
  n=$(field "$1" reports)
  sub=$(field "$1" submitted)
  floor=$(field "$1" min_submitted)
  [ -n "$floor" ] || return 1
  [ "${n:-0}" -ge "$MIN_REPORTS" ] && [ "${sub:-0}" -ge "$floor" ]
}
show() { echo "  $(cat "$OUT/res_$1" 2>/dev/null)"; }

OWN="${DUT_MAC:-}"
# The DUT's own address: the one `self` resolves to (GetPermanentMacAddress),
# learned from a short armed rxdemo run's sta.arm event - which also proves,
# before any arm is scored, that the seam arms on this DUT at all.
if [ -z "$OWN" ]; then
  sta_dev_opened dut
  # shellcheck disable=SC2046  # word-split assignments on purpose
  env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" $(sel_env "$DUT_SYSFS") \
      DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
      DEVOURER_STA_IDENTITY="self,$BSSID" \
      "$BUILD/rxdemo" >"$OUT/dut_own.jsonl" 2>"$OUT/dut_own.err" &
  own_pid=$!
  sta_pid_record dut "$own_pid"
  wait_for "$own_pid" "$OUT/dut_own.jsonl" '"ev":"sta.arm"' "$READY_TIMEOUT"
  sta_pid_kill dut INT
  OWN=$(sed -n 's/.*"ev":"sta.arm","ok":1,"own":"\([0-9a-f:]\{17\}\)".*/\1/p' \
        "$OUT/dut_own.jsonl" | head -1)
  if [ -z "$OWN" ]; then
    echo "the DUT did not arm (or reported no own address) - see $OUT/dut_own.err"
    grep -m3 -i 'station identity' "$OUT/dut_own.err"
    echo "VERDICT: INCONCLUSIVE"; exit 2
  fi
fi
case "$OWN" in
  [0-9a-f][02468ace]:*) ;;
  *) echo "DUT_MAC=$OWN is not a lowercase unicast address"; exit 2 ;;
esac
echo "DUT  $DUT_VID:$DUT_PID at $DUT_SYSFS   BSSID $BSSID   ch$CH   rate $RATE   retry limit $RETRY_LIMIT"
echo "DUT own address $OWN"
[ "$HALF" != up ] && echo "PEER $PEER_VID:$PEER_PID at $PEER_SYSFS"
echo "logs: $OUT"

# ============================== DOWN ========================================
if [ "$HALF" != up ]; then
  echo
  echo "== DOWN A: DUT armed, peer -> its own address $OWN =="
  down_arm A "$OWN" armed;    show A
  echo "== DOWN B: DUT armed, peer -> NOBODY ($NOBODY) (control) =="
  down_arm B "$NOBODY" armed; show B
  echo "== DOWN C: DUT absent, peer -> $OWN (control) =="
  down_arm C "$OWN" absent;   show C
  echo "== DOWN D: DUT running UNARMED, peer -> $OWN =="
  down_arm D "$OWN" unarmed;  show D
  echo "== DOWN E: DUT armed then CLEARED, peer -> $OWN =="
  down_arm E "$OWN" cleared;  show E
  echo
  if usable A && usable B && usable C; then
    a=$(field A ok_pct); b=$(field B ok_pct); c=$(field C ok_pct)
    if awk -v a="$a" -v b="$b" -v c="$c" 'BEGIN{exit !(a > b + 40 && a > c + 40)}'; then
      ok "DOWN ack: A ${a}% ACKed against B ${b}% (nobody) and C ${c}% (DUT absent)"
    else
      bad "DOWN ack: A ${a}% is not clearly above B ${b}% and C ${c}%"
    fi
    # The denominator is the peer's REPORT count: a reported frame aired,
    # while a submitted one may still have been queued when the window
    # closed (reported separately as unreported).
    rep=$(field A reports); rxd=$(field A rx_distinct); unr=$(field A unreported)
    if [ "${rep:-0}" -gt 0 ] &&
       awk -v r="${rxd:-0}" -v s="$rep" -v m="$MIN_RX_PCT" 'BEGIN{exit !(100*r/s >= m)}'; then
      ok "DOWN receive: the DUT delivered ${rxd} distinct frames of the peer's ${rep} reported (>= ${MIN_RX_PCT}%; ${unr:-0} submitted unreported)"
    else
      bad "DOWN receive: the DUT delivered ${rxd:-0} distinct frames of the peer's ${rep:-0} reported (< ${MIN_RX_PCT}%; ${unr:-0} submitted unreported)"
    fi
  else
    inc "DOWN: arm A, B or C aborted, carried under $MIN_REPORTS reports or its min_submitted floor, or its transmitter stalled (live=0) - not a measurement"
  fi
  if usable A && usable D; then
    a=$(field A ok_pct); d=$(field D ok_pct)
    if awk -v a="$a" -v d="$d" 'BEGIN{exit !(d < a - 40)}'; then
      ok "ARM is the cause: unarmed D ${d}% against armed A ${a}%"
    elif [ "$EXPECT_UNARMED_SILENT" = 0 ]; then
      echo "  NOTE  unarmed D ${d}% against armed A ${a}% - this die answers unarmed (not scored)"
    else
      bad "ARM is the cause: unarmed D ${d}% is not clearly below armed A ${a}%"
    fi
  else
    inc "ARM is the cause: arm D aborted or carried under $MIN_REPORTS reports"
  fi
  if grep -q FAILCLEAR "$OUT/res_E" 2>/dev/null; then
    bad "CLEAR: ClearStationIdentity returned false (rollback not verified)"
  elif usable A && usable E && usable D; then
    a=$(field A ok_pct); e=$(field E ok_pct); d=$(field D ok_pct)
    # Clear returns to the pre-arm port, so E must read like D (unarmed),
    # not like A.
    if awk -v a="$a" -v e="$e" -v d="$d" 'BEGIN{exit !(e < a - 40 && e < d + 20)}'; then
      ok "CLEAR silences: cleared E ${e}% against armed A ${a}% and unarmed D ${d}%"
    elif [ "$EXPECT_UNARMED_SILENT" = 0 ]; then
      echo "  NOTE  cleared E ${e}% (A ${a}%, D ${d}%) - this die answers unarmed (not scored)"
    else
      bad "CLEAR silences: cleared E ${e}% (A ${a}%, D ${d}%)"
    fi
  else
    inc "CLEAR: arm E (or D) aborted or carried under $MIN_REPORTS reports"
  fi
  # The DOWN half no longer needs the peer: hand it back now.
  sta_dev_handback peer "$PEER_SYSFS"
fi

# =============================== UP =========================================
if [ "$HALF" != down ]; then
  # --- the AP: the full guard again (it can have moved during DOWN) ---
  ap_guard
  AP_ID=$(sta_usb_id "$AP_SYSFS")

  nmcli device set "$AP_IF" managed no >/dev/null 2>&1
  sleep 1
  ip link set "$AP_IF" down 2>/dev/null
  iw dev "$AP_IF" set type managed 2>/dev/null
  ip link set "$AP_IF" up 2>/dev/null
  if [ "$CH" -le 14 ]; then hw_mode=g; else hw_mode=a; fi
  cat > "$OUT/hostapd.conf" <<EOF
interface=$AP_IF
driver=nl80211
ssid=rtlstation
bssid=$BSSID
hw_mode=$hw_mode
channel=$CH
auth_algs=1
wmm_enabled=0
EOF
  AP_REENUM=yes   # hostapd may run and leave its bssid behind
  hostapd -B -P "$OUT/.pid_hostapd" -f "$OUT/hostapd.log" "$OUT/hostapd.conf" >/dev/null 2>&1
  ap_up=no
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if iw dev "$AP_IF" info 2>/dev/null | grep -q 'type AP'; then ap_up=yes; break; fi
    sleep 1
  done
  if [ "$ap_up" != yes ]; then
    echo "hostapd did not bring $AP_IF up in AP mode (a 5 GHz channel may be no-IR here):"
    tail -12 "$OUT/hostapd.log" 2>/dev/null || echo "(no hostapd log written)"
    echo "VERDICT UP: INCONCLUSIVE"; exit 2
  fi
  echo
  echo "AP   $AP_IF ($PHY) at $AP_SYSFS holds $BSSID; station $OWN"
  echo "== UP F: station -> BSSID (the AP) =="
  up_arm F "$BSSID";  show F
  echo "== UP G: station -> NOBODY ($NOBODY) (control) =="
  up_arm G "$NOBODY"; show G
  echo "== UP H: station UNARMED -> BSSID (reported, not scored) =="
  up_arm H "$BSSID" unarmed; show H
  echo
  if usable F && usable G; then
    f=$(field F ok_pct); g=$(field G ok_pct)
    fr=$(field F retries_mean); gr=$(field G retries_mean)
    if awk -v f="$f" -v g="$g" 'BEGIN{exit !(f > g + 40)}'; then
      ok "UP ack: F ${f}% ACKed (retries ${fr}) against G ${g}% (retries ${gr}, limit $RETRY_LIMIT)"
    else
      bad "UP ack: F ${f}% is not clearly above the control G ${g}%"
    fi
  else
    inc "UP: arm F or G aborted, carried under $MIN_REPORTS reports or its min_submitted floor, or its transmitter stalled (live=0) - not a measurement"
  fi
  # H is information, not a verdict: whether TX ACK matching needs the arm
  # at all on this die. Never counted into pass/fail/inconclusive.
  if usable H; then
    echo "  INFO  unarmed uplink H $(field H ok_pct)% ACKed (retries $(field H retries_mean)) - against armed F $(field F ok_pct)%"
  else
    echo "  INFO  unarmed uplink H: no usable result ($(cat "$OUT/res_H" 2>/dev/null))"
  fi
fi

echo
echo "=== $pass passed, $fail failed, $inconclusive inconclusive  (logs: $OUT) ==="
[ "$fail" -gt 0 ] && exit 1
[ "$inconclusive" -gt 0 ] && exit 2
exit 0
