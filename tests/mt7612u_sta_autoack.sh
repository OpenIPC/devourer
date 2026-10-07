#!/bin/sh
# mt7612u_sta_autoack.sh - does an MT7612U station acknowledge unicast sent to
# its own address, with NOTHING armed?
#
# Results and limits (notably: a ONE-PEER result): docs/mt7612u-station-
# identity.md.
#
# The transmitter is the only party that knows whether its frame was
# acknowledged, and on Realtek that knowledge is a per-frame CCX report. Two
# methods that ask anyone else cannot answer, and are recorded in that doc so
# they are not retried: capturing the DUT's ACKs on a monitor vif on the
# peer's own phy (a radio cannot hear an ACK to its own transmission), and
# counting retried probe responses from hostapd (it does not retransmit an
# unacknowledged probe response, so the control cannot move).
#
# So: a Realtek adapter under devourer injects unicast QoS-Data at the DUT and
# reads its own tx.report events. retries~0 means the DUT answered; retries
# pinned at the descriptor limit means it did not. That is the same instrument
# tests/ack_txreport_matrix.sh uses, pointed the other way round - there the
# Realtek part is the responder, here it is the witness.
#
# Jaguar3 (8812CU/8822CU) is the preferred peer because it drains C2H off its
# coex runtime, so the reports arrive without further arrangement. Any
# generation with ack_responder_ok works if it runs DEVOURER_TX_WITH_RX=thread.
#
#   sudo tests/mt7612u_sta_autoack.sh
#   sudo PEER_PID=0xc812 DUT_SYSFS=7-1 CH=6 tests/mt7612u_sta_autoack.sh
#
# Exit status: 0 every check passed; 1 a check failed; 2 INCONCLUSIVE (the rig
# was refused, or the gate could not measure); 3 interrupted (INT/TERM).
#
# Env: PEER_VID, PEER_PID, PEER_SYSFS, DUT_SYSFS, CH, SECS, RETRY_LIMIT, OUT.

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
SECS="${SECS:-10}"
RETRY_LIMIT="${RETRY_LIMIT:-12}"
# Unset: a fresh private directory (sta_out_prepare in the lib).
OUT="${OUT:-}"
FW_DIR="${FW_DIR:-/lib/firmware/mediatek}"
# An address nobody holds. The control arm targets this: same transmitter,
# same rate, same channel, only the destination changes.
NOBODY="${NOBODY:-02:00:00:de:ad:07}"
TX_SA="${TX_SA:-02:aa:bb:cc:dd:07}"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 2; }
# shellcheck source=tests/mt7612u_sta_lib.sh
. "$ROOT/tests/mt7612u_sta_lib.sh"
sta_out_prepare || exit 2
sta_lock_take || exit 2
sta_pid_init dut peer
sta_peer_record || { sta_lock_release; exit 2; }
# Only a link THIS run created is removed afterwards - anything already at
# $ROOT/firmware, a dangling symlink included, is the operator's.
sta_fw_link || { sta_fw_unlink; sta_lock_release; exit 2; }

pass=0; fail=0
ok()  { pass=$((pass+1)); printf '  PASS  %s\n' "$*"; }
bad() { fail=$((fail+1)); printf '  FAIL  %s\n' "$*"; }

DUT_PID=""
CLEANED=no
# shellcheck disable=SC2317  # reached through the traps below
cleanup() {
  # Ignored, not deferred: a second INT/TERM during the hand-back would
  # otherwise end it half done (CLEANED is already set, so it cannot rerun).
  trap '' INT TERM
  [ "$CLEANED" = yes ] && return 0
  CLEANED=yes
  # arm() runs in a command substitution, so the PIDs it starts are recorded
  # in $OUT (tests/mt7612u_sta_lib.sh) for this trap to find. The peer first:
  # an orphan txdemo keeps its USB lock and fails the NEXT run's peer open
  # with "adapter already in use", which yields zero reports - and zero is a
  # control's passing value. INT, as timeout(1) forwards it to txdemo.
  sta_pid_kill peer INT; peer_gone=$?
  sta_pid_kill dut; dut_gone=$?
  DUT_PID=""
  # The same rule for the DUT: never re-enumerate it mid-de-init.
  if [ "$dut_gone" = 0 ]; then sta_dut_handback
  else echo "DUT still running - not re-enumerating DUT_SYSFS=$DUT_SYSFS"; fi
  # Only once the peer process has really exited: re-enumerating an adapter
  # still inside its de-init is what the hand-back must not do.
  if [ "$peer_gone" = 0 ]; then sta_peer_handback
  else echo "peer still running - not re-enumerating PEER_SYSFS=$PEER_SYSFS"; fi
  sta_fw_unlink
  sta_lock_release
}
trap cleanup EXIT
# AND IT MUST STOP: with INT/TERM on the EXIT trap the shell runs cleanup
# and then CARRIES ON into the next arm. CLEANED makes the EXIT pass after it
# a no-op: sta_pid_kill forgets a PID on the first pass, so a second pass
# would hand back an adapter the first refused to.
trap 'cleanup; exit 3' INT TERM

sta_dut_take || exit 2

DUT_MAC=$("$BUILD/mt7612uprobe" staid 2>&1 | sed -n 's/^own \([0-9a-f:]\{17\}\).*/\1/p' | head -1)
[ -n "$DUT_MAC" ] || { echo "could not read the DUT's MAC"; exit 1; }
echo "DUT  MT7612U at $DUT_SYSFS, own $DUT_MAC"
echo "peer $PEER_VID:$PEER_PID at $PEER_SYSFS, ch$CH, retry limit $RETRY_LIMIT"
echo

# $1 = tag, $2 = destination, $3 = 1 if the DUT should be receiving
arm() {
  tag="$1"; ra="$2"; dut_up="$3"
  DUT_PID=""
  if [ "$dut_up" != 0 ]; then
    # 1 = receiver on, MANAGED filter, nothing armed - the claim.
    # 2 = the same code path with MT_AUTO_RSP_EN cleared - the control.
    # 3 = managed, a WRONG BSSID in the station's APC slot (mt76's station
    #     rule), ENABLED - closes the BSSID question
    #     (docs/mt7612u-station-identity.md).
    #
    # Arms 1 and 2 both run `norsp`, which takes the bit as an argument, so
    # they share one code path and one (managed) receive filter and differ by
    # exactly that bit. `bringup arx` would NOT do for arm 1: it installs the
    # MONITOR filter at the top of gate_arx.
    if [ "$dut_up" = 3 ]; then
      "$BUILD/mt7612uprobe" bssen "$CH" $((SECS + 14)) \
          >"$OUT/dut_$tag.log" 2>&1 &
    elif [ "$dut_up" = 2 ]; then
      "$BUILD/mt7612uprobe" norsp "$CH" $((SECS + 14)) 1 \
          >"$OUT/dut_$tag.log" 2>&1 &
    else
      "$BUILD/mt7612uprobe" norsp "$CH" $((SECS + 14)) 0 \
          >"$OUT/dut_$tag.log" 2>&1 &
    fi
    DUT_PID=$!
    # arm() runs inside a command substitution, so this assignment is invisible
    # to the parent's EXIT trap. Record it where cleanup can find it, or a
    # Ctrl-C mid-arm leaves a bringup holding the adapter.
    sta_pid_record dut "$DUT_PID"
    sleep 8
    # `arm` runs inside a command substitution, so `exit` here would only kill
    # the subshell and the caller would carry on with an EMPTY result - which
    # parses as 0% and reads as a passing control. Emit a marker instead and
    # let the verdicts refuse it.
    kill -0 "$DUT_PID" 2>/dev/null || {
      printf '%s ABORTED the DUT exited before this arm: %s' \
             "$tag" "$(tail -1 "$OUT/dut_$tag.log" 2>/dev/null)"
      rm -f "$OUT/.pid_dut"; return 1; }
  fi

  tx_start=$(date +%s)
  sta_peer_opened
  env DEVOURER_VID="$PEER_VID" DEVOURER_PID="$PEER_PID" \
      DEVOURER_USB_BUS="${PEER_SYSFS%%-*}" DEVOURER_USB_PORT="${PEER_SYSFS#*-}" \
      DEVOURER_CHANNEL="$CH" \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA="$ra" DEVOURER_TX_SA="$TX_SA" \
      DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PAYLOAD_BYTES=200 \
      DEVOURER_TX_GAP_US=5000 DEVOURER_TX_REPORT=1 \
      DEVOURER_TX_RETRY_LIMIT="$RETRY_LIMIT" \
      DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=warn \
      timeout -s INT -k 3 "$SECS" "$BUILD/txdemo" \
      >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" &
  peer=$!
  sta_pid_record peer "$peer"
  wait "$peer"
  rm -f "$OUT/.pid_peer"
  tx_end=$(date +%s)

  # -k 3 IS LOAD-BEARING. Without it a peer that does not act on SIGINT blocks
  # this arm indefinitely, still holding its USB lock - and that orphan then
  # fails the NEXT run's peer open with "adapter already in use", which yields
  # zero reports, and zero is a control's passing value.

  # Did the PEER stay inside its own window? Ask this BEFORE asking anything
  # about the DUT, because a peer that overran is the one explanation under
  # which the DUT's apparent death is not a death at all.
  #
  # The DUT gets SECS+14 s and the peer SECS, so the DUT outlives any peer that
  # behaves. If the peer blocks, the DUT reaches the end of its OWN window and
  # exits NORMALLY - and the liveness check below then reports "the DUT died
  # during the measurement window" while quoting the DUT's own success line as
  # the evidence ("ABORTED the DUT died DURING ... GATE NORSP: done
  # (restored)"). A clean completion is not a death; the fault is the peer's,
  # and this names it as the peer's.
  # SECS+5, not SECS+8: with -k 3 a well-behaved peer is done by SECS+3, and
  # the DUT's own window expires SECS+16 s after ITS launch, i.e. SECS+8 s
  # after the peer started. A threshold of SECS+8 would sit exactly on that
  # boundary and let the misfire back in on a tie.
  if [ $((tx_end - tx_start)) -gt $((SECS + 5)) ]; then
    printf '%s ABORTED the PEER overran its %ss window (took %ss) - nothing about the DUT can be read from this arm' \
           "$tag" "$SECS" "$((tx_end - tx_start))"
    [ -n "$DUT_PID" ] && { kill "$DUT_PID" 2>/dev/null; wait "$DUT_PID" 2>/dev/null; }
    DUT_PID=""; rm -f "$OUT/.pid_dut"; return 1
  fi

  # LIVENESS AFTER THE WINDOW, not only before it.
  #
  # The 8 s probe above only proves the arm STARTED. If the DUT wedges at
  # second 9 - a documented failure mode on this part - the peer keeps
  # injecting at an address nobody answers, ok_pct reads ~0, and for a CONTROL
  # arm that is the PASSING value. Arm D would then print "MT_AUTO_RSP_EN is
  # the gate" on the strength of a dead device. Arms A and E fail closed, so
  # this matters for the controls specifically, which is the worse direction.
  if [ -n "$DUT_PID" ] && ! kill -0 "$DUT_PID" 2>/dev/null; then
    printf '%s ABORTED the DUT died DURING the measurement window: %s' \
           "$tag" "$(tail -1 "$OUT/dut_$tag.log" 2>/dev/null)"
    DUT_PID=""; rm -f "$OUT/.pid_dut"; return 1
  fi
  [ -n "$DUT_PID" ] && { kill "$DUT_PID" 2>/dev/null; wait "$DUT_PID" 2>/dev/null; }
  DUT_PID=""; rm -f "$OUT/.pid_dut"

  python3 - "$OUT/tx_$tag.jsonl" "$tag" <<'PYEOF'
import json, sys
path, tag = sys.argv[1], sys.argv[2]
n = okc = 0
retries = 0
for line in open(path, errors='replace'):
    if '"ev":"tx.report"' not in line:
        continue
    try:
        e = json.loads(line)
    except ValueError:
        continue
    n += 1
    if e.get('ok'):
        okc += 1
    retries += int(e.get('retries', 0) or 0)
if not n:
    print(f"{tag} reports=0")
else:
    print(f"{tag} reports={n} ok={okc} ok_pct={100.0*okc/n:.1f} "
          f"retries_mean={retries/n:.2f}")
PYEOF
}

echo "== A: destination is the DUT, DUT receiving, nothing armed =="
a=$(arm A "$DUT_MAC" 1); echo "  $a"
echo "== B: destination is an address NOBODY holds (control) =="
b=$(arm B "$NOBODY" 1); echo "  $b"
echo "== C: destination is the DUT, DUT NOT running (control) =="
c=$(arm C "$DUT_MAC" 0); echo "  $c"
echo "== D: destination is the DUT, DUT receiving, MT_AUTO_RSP_EN CLEARED =="
echo "     (single-variable: which mechanism answers?)"
d=$(arm D "$DUT_MAC" 2); echo "  $d"
echo "== E: destination is the DUT, DUT receiving, WRONG BSSID in the station APC slot, ENABLED =="
echo "     (the BSSID question's last caveat: does an enabled slot gate a station?)"
e=$(arm E "$DUT_MAC" 3); echo "  $e"
echo

sta_fw_unlink

# Verdicts. A must differ from BOTH controls, or the instrument is not
# measuring the DUT.
a_ok=$(printf '%s' "$a" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
b_ok=$(printf '%s' "$b" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
c_ok=$(printf '%s' "$c" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
d_ok=$(printf '%s' "$d" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')
e_ok=$(printf '%s' "$e" | sed -n 's/.*ok_pct=\([0-9.]*\).*/\1/p')

# EVERY arm must have produced reports. An arm that aborted, or one the peer
# never reported on, yields an empty ok_pct - and an empty value compared
# numerically reads as 0, which is the PASSING value for a control - an
# aborted arm D would read as "MT_AUTO_RSP_EN is the gate".
for pair in "A:$a" "B:$b" "C:$c" "D:$d" "E:$e"; do
  t=${pair%%:*}; v=${pair#*:}
  case "$v" in
    *ABORTED*)   echo "ARM $t ABORTED: ${v#* ABORTED }"
                 echo "GATE AUTOACK: INCONCLUSIVE"; exit 2 ;;
  esac
  n=$(printf '%s' "$v" | sed -n 's/.*reports=\([0-9]*\).*/\1/p')
  if [ -z "${n:-}" ] || [ "${n:-0}" -eq 0 ]; then
    echo "ARM $t produced NO tx.report events, so it is not a measurement and"
    echo "must not be compared. Check the peer runs DEVOURER_TX_WITH_RX=thread"
    echo "and that its generation has a CCX path."
    echo "GATE AUTOACK: INCONCLUSIVE"; exit 2
  fi
done

if awk -v a="${a_ok:-0}" -v b="${b_ok:-0}" -v c="${c_ok:-0}" 'BEGIN{
  printf "A (DUT present)        ok=%.1f%%\nB (nobody)             ok=%.1f%%\nC (DUT absent)         ok=%.1f%%\n", a, b, c
  exit !(a > b + 40 && a > c + 40)
}'; then ok "the DUT acknowledges unicast to its own address with nothing armed"
else bad "A is not clearly above both controls - the DUT is not shown to acknowledge"
fi

# D decides whether SetStationIdentity's MT_AUTO_RSP_EN refusal is justified.
# It is a separate verdict: the claim above stands either way.
if awk -v a="${a_ok:-0}" -v d="${d_ok:-0}" 'BEGIN{
  printf "D (AUTO_RSP_EN off)    ok=%.1f%%\n", d
  exit !(d < a - 40)
}'; then ok "MT_AUTO_RSP_EN is the gate - SetStationIdentity is right to refuse when it is clear"
else bad "MT_AUTO_RSP_EN is NOT the gate here - the seam refuses on a bit that does not control this"
fi

# E closes the BSSID question's last caveat: `mt7612uprobe sta` leaves mt76's
# per-slot enable clear, so "a wrong BSSID changes nothing" could mean
# "nothing was reading the BSSID". Here the slot is wrong AND enabled.
if awk -v a="${a_ok:-0}" -v e="${e_ok:-0}" 'BEGIN{
  printf "E (wrong BSSID, slot ENABLED) ok=%.1f%%\n", e
  exit !(e > a - 20)
}'; then ok "a wrong BSSID in the ENABLED station APC slot does not gate the station"
else bad "an enabled APC slot DOES gate the station - the BSSID gate's null result was an artefact of the enable bit being clear"
fi

echo
echo "=== $pass passed, $fail failed  (logs: $OUT) ==="
exit $(( fail > 0 ))
