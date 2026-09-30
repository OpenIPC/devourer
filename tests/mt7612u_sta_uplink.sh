#!/bin/sh
# mt7612u_sta_uplink.sh - is an MT7612U station's OWN traffic acknowledged?
#
# The other half of AdapterCaps::station_mode_ok's bar.
# tests/mt7612u_sta_autoack.sh measures frames sent TO the DUT; this measures
# frames sent BY it. A station whose uplink is never acknowledged retransmits
# everything to the retry limit and gives up, which looks like a link problem
# and is not.
#
# Instrument: the DUT's own MT_TX_STAT_FIFO, via `mt7612uprobe txs`, which
# reports the MAC's per-MPDU retry count. The peer is a Realtek adapter running
# rxdemo with DEVOURER_ACK_RESPONDER armed on a chosen address - the same
# responder tests/ack_txreport_matrix.sh uses. Results:
# docs/mt7612u-station-identity.md, "The uplink".
#
# Two arms, and the second is what makes the first mean anything:
#
#   A  peer armed on the address we transmit to   -> expect retries ~0
#   B  peer armed on a DIFFERENT address          -> expect retries pinned
#
# B is the control. It holds the peer present, on channel, and transmitting
# nothing different - only the address it answers for changes. Without it,
# "few retries" might be what this channel always gives.
#
# The DUT's retry limit is set EXPLICITLY (RETRY_LIMIT, default 15, passed to
# the gate as DEVOURER_TX_RETRY_LIMIT) rather than left to whatever the chip
# holds: the uplink question is only answerable when an unacknowledged frame
# is retried, and the gate's frames request an ACK. 15 is the initvals' short
# limit, so the default reproduces docs/mt7612u-station-identity.md's table.
# It is NOT what a library session airs by default - there tx.retry_limit
# defaults to 0 and the stream radiotap to NOACK (IRadio::SetStationIdentity).
# RETRY_LIMIT=0 is refused here: it would make arm B indistinguishable from a
# one-shot send.
#
# RUNTIME. Each harness arm runs the whole `txs` gate - eight gate arms, each
# twice (MAC receiver off, then on), FRAMES frames apiece - and most gate arms
# settle one frame at a time at about 6 frames/s. Measured at FRAMES=200: about
# 9 minutes for arm A; arm B, where nothing is acknowledged, is slower. Budget
# 25 minutes at FRAMES=200 and about a third of that at the default 60 - an
# outer timeout shorter than that cuts arm B off and reads as INCONCLUSIVE.
#
#   sudo tests/mt7612u_sta_uplink.sh
#
# Env: PEER_VID, PEER_PID, PEER_SYSFS, DUT_SYSFS, CH, FRAMES, RETRY_LIMIT, OUT.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
# The bring-up tool resolves its firmware directory RELATIVE TO THE WORKING
# DIRECTORY ("firmware/mt7662_rom_patch.bin"), and the symlink below is created
# at $ROOT. Running this script from anywhere else therefore fails the DUT's
# firmware load, which surfaces as "could not read the DUT's MAC" - a message
# that names neither the cause nor the cure. Pin the directory instead.
cd "$ROOT" || exit 1
PEER_VID="${PEER_VID:-0x0bda}"
PEER_PID="${PEER_PID:-0xc812}"
PEER_SYSFS="${PEER_SYSFS:-5-1}"
DUT_SYSFS="${DUT_SYSFS:-7-1}"
CH="${CH:-6}"
FRAMES="${FRAMES:-60}"
RETRY_LIMIT="${RETRY_LIMIT:-15}"
# Unset: a fresh private directory (sta_out_prepare in the lib).
OUT="${OUT:-}"
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"
# The address the DUT transmits to. The peer answers for this in arm A and for
# OTHER in arm B.
TARGET="${TARGET:-02:aa:bb:cc:dd:11}"
OTHER="${OTHER:-02:aa:bb:cc:dd:12}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
case "$RETRY_LIMIT" in
  ''|*[!0-9]*|0) echo "RETRY_LIMIT must be a positive integer (got '$RETRY_LIMIT')"; exit 2 ;;
esac
# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init resp dut
sta_peer_record || { sta_lock_release; exit 2; }
# Only a link THIS run created is removed afterwards - anything already at
# $ROOT/firmware, a dangling symlink included, is the operator's.
sta_fw_link

pass=0; fail=0
ok()  { pass=$((pass+1)); printf '  PASS  %s\n' "$*"; }
bad() { fail=$((fail+1)); printf '  FAIL  %s\n' "$*"; }

RESP=""
# shellcheck disable=SC2317  # reached through the traps below
cleanup() {
  # arm() runs in a command substitution, so its PIDs are recorded in $OUT
  # (tests/mt7612u_sta_lib.sh) for this trap to find.
  sta_pid_kill dut
  sta_pid_kill resp; peer_gone=$?
  RESP=""
  sta_dut_handback
  # Only once the peer process has really exited: re-enumerating an adapter
  # still inside its de-init is what the hand-back must not do.
  if [ "$peer_gone" = 0 ]; then sta_peer_handback
  else echo "peer still running - not re-enumerating PEER_SYSFS=$PEER_SYSFS"; fi
  sta_fw_unlink
  sta_lock_release
}
trap cleanup EXIT
# AND IT MUST STOP: with INT/TERM on the EXIT trap the shell runs cleanup
# and then CARRIES ON into the next arm. cleanup is idempotent, so the EXIT
# pass after it is harmless.
trap 'cleanup; exit 130' INT TERM

sta_dut_take || exit 2
echo "DUT  MT7612U at $DUT_SYSFS transmitting to $TARGET"
echo "peer $PEER_VID:$PEER_PID at $PEER_SYSFS, ch$CH"
echo

# $1 = tag, $2 = the address the peer answers for
arm() {
  tag="$1"; resp="$2"
  sta_peer_opened
  env DEVOURER_VID="$PEER_VID" DEVOURER_PID="$PEER_PID" \
      DEVOURER_USB_BUS="${PEER_SYSFS%%-*}" DEVOURER_USB_PORT="${PEER_SYSFS#*-}" \
      DEVOURER_CHANNEL="$CH" DEVOURER_ACK_RESPONDER="$resp" \
      DEVOURER_LOG_LEVEL=info \
      "$BUILD/rxdemo" >"$OUT/resp_$tag.jsonl" 2>"$OUT/resp_$tag.err" &
  RESP=$!
  # Same subshell trap as the autoack harness: record the pid where the
  # parent's cleanup can reach it.
  sta_pid_record resp "$RESP"
  sleep 10
  if ! kill -0 "$RESP" 2>/dev/null; then
    printf '%s ABORTED the peer exited: %s' "$tag" "$(tail -1 "$OUT/resp_$tag.err")"
    RESP=""; rm -f "$OUT/.pid_resp"; return 1
  fi
  # The arm must be VERIFIED, not assumed: an unarmed responder and a
  # responder armed on the wrong address look identical from here, and that is
  # exactly what arm B is supposed to be.
  # The SUCCESS line only: "ack responder" alone also matches the
  # backends' refusal and write-failure lines, which would pass as armed.
  if ! grep -q "ACK responder armed for" "$OUT/resp_$tag.err"; then
    printf '%s ABORTED the peer never reported arming a responder' "$tag"
    sta_pid_kill resp; RESP=""; return 1
  fi

  DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
      "$BUILD/mt7612uprobe" txs "$CH" "$FRAMES" "$TARGET" \
      >"$OUT/dut_$tag.txt" 2>&1 &
  dut=$!
  sta_pid_record dut "$dut"
  wait "$dut"
  rm -f "$OUT/.pid_dut"
  # The limit must have LANDED, not merely been asked for: the gate prints
  # this line only after mt7612u_set_retry_limit() read it back.
  if ! grep -q "^retry limit set to $RETRY_LIMIT " "$OUT/dut_$tag.txt"; then
    printf '%s ABORTED the DUT did not confirm retry limit %s: %s' \
           "$tag" "$RETRY_LIMIT" "$(grep -m1 -i 'retry limit' "$OUT/dut_$tag.txt")"
    sta_pid_kill resp; RESP=""; return 1
  fi

  # LIVENESS AFTER THE WINDOW. The 10 s probe proves the peer started; if it
  # dies mid-dwell the DUT's frames go unanswered, arm B reads 0/200 at the
  # retry limit, and that is the CONTROL's passing value - so a dead peer
  # would be scored as a working control.
  if ! kill -0 "$RESP" 2>/dev/null; then
    printf '%s ABORTED the peer died DURING the measurement window: %s' \
           "$tag" "$(tail -1 "$OUT/resp_$tag.err" 2>/dev/null)"
    RESP=""; rm -f "$OUT/.pid_resp"; return 1
  fi
  sta_pid_kill resp; RESP=""

  # `mt7612uprobe txs` prints the arm table TWICE - once with the MAC receiver
  # OFF and once with it ON. With the receiver off the MAC cannot hear an ACK,
  # so every unicast arm runs its retry ladder to exhaustion regardless of what
  # the peer does (docs/mt7612u-tx-retry.md).
  #
  # A station runs with its receiver on, so the "MAC receiver ON" table is the
  # only one that answers this question. The first arm-d row in the output is
  # the receiver-OFF one, which reads UNSETTLED in both arms whatever the peer
  # does - so the parser keys on the section header, not on the first match.
  #
  # The row is arm `d`, "ucast peer ownSA Normal": transmitted from our OWN
  # address to the peer with normal ack policy, which is what a station's
  # uplink is. Columns: fps, entries/sent, success, mean retries, max retries,
  # plus an UNSETTLED marker when too few frames were sent for the 16-slot
  # status ring to attribute cleanly.
  python3 - "$OUT/dut_$tag.txt" "$tag" <<'PYEOF'
import re, sys
path, tag = sys.argv[1], sys.argv[2]

section = None
row = None
for line in open(path, errors='replace'):
    if 'MAC receiver' in line:
        section = 'ON' if 'ON' in line else 'OFF'
        continue
    if section == 'ON' and re.match(r'\s*d\s+ucast peer ownSA Normal', line):
        row = line
        break

if row is None:
    print(f"{tag} NOPARSE no receiver-ON arm-d row in {path}")
    sys.exit()
m = re.search(r'(\d+)\s*/\s*(\d+)\s+(\d+)\s+([\d.]+)\s+(\d+)', row)
if not m:
    print(f"{tag} NOPARSE arm-d row unreadable: {row.strip()}")
    sys.exit()
entries, sent, success, mean_rtry, max_rtry = m.groups()
if 'UNSETTLED' in row and int(success) > 0:
    # UNSETTLED means fewer status entries landed than frames were sent, so
    # the gate cannot guarantee each entry belongs to the arm it is printed
    # under. That matters when an arm CLAIMS SUCCESS - refuse it.
    #
    # It does not matter for an arm that succeeded at nothing, and refusing
    # those would make this measurement impossible: the failing control is
    # UNSETTLED BY CONSTRUCTION. When no peer acknowledges, every frame runs
    # the full RETRY_LIMIT ladder, the MAC is roughly two orders of magnitude
    # slower per frame, and the 16-slot status ring can never keep up with
    # submission. A control that always reads UNSETTLED is not a control.
    #
    # The direction of the risk also points the safe way. Misattributed
    # entries would come from the NEIGHBOURING arms, which in this table run
    # at 200/200 and zero retries - so contamination can only make a failing
    # arm look BETTER. An arm reading 0 success at max retries is therefore a
    # floor, and a floor is all a control needs to be.
    print(f"{tag} UNSETTLED entries={entries}/{sent} success={success} "
          f"retries={mean_rtry} - success claimed on unreliable attribution")
    sys.exit()
sent_i = int(sent) or 1
note = "  [entries<sent: a floor, see the note in this script]" \
       if 'UNSETTLED' in row else ""
print(f"{tag} acked={success}/{sent} ok_pct={100.0*int(success)/sent_i:.1f} "
      f"retries={mean_rtry} max={max_rtry}{note}")
PYEOF
}

echo "== A: peer answers for $TARGET (the address we transmit to) =="
a=$(arm A "$TARGET"); echo "  $a"
echo "== B: peer answers for $OTHER instead (control) =="
b=$(arm B "$OTHER"); echo "  $b"
echo

for pair in "A:$a" "B:$b"; do
  t=${pair%%:*}; v=${pair#*:}
  case "$v" in
    *ABORTED*) echo "ARM $t ABORTED: ${v#* ABORTED }"; echo "GATE UPLINK: INCONCLUSIVE"; exit 2 ;;
    *NOPARSE*) echo "ARM $t produced nothing parseable - not a measurement."
               echo "GATE UPLINK: INCONCLUSIVE"; exit 2 ;;
    *UNSETTLED*) echo "ARM $t: $v"
               echo "The gate flagged its own attribution as unreliable, so"
               echo "these numbers are not a measurement. Raise FRAMES."
               echo "GATE UPLINK: INCONCLUSIVE"; exit 2 ;;
  esac
done

a_ok=$(printf '%s' "$a" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
b_ok=$(printf '%s' "$b" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
if awk -v a="${a_ok:-0}" -v b="${b_ok:-0}" 'BEGIN{
  printf "A (peer answers for us) acked=%.1f%%\nB (peer answers elsewhere) acked=%.1f%%\n", a, b
  exit !(a > b + 40)
}'; then ok "the station's own uplink is acknowledged, and the control shows the gate can fail"
else bad "A is not clearly above the control - the uplink is not shown to be acknowledged"
fi

echo
echo "=== $pass passed, $fail failed  (logs: $OUT) ==="
exit $(( fail > 0 ))
