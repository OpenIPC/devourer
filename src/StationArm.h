#pragma once

/* StationArm - the per-device state behind IRadio::SetStationIdentity on the
 * Realtek generations that share the port-0 register map (Jaguar1/2/3).
 *
 * The register recipe lives in AckResponder.h (arm_station / station_is /
 * restore_station); this holds the one piece of state the seam's contract
 * needs on top of it - the exact pre-arm port snapshot - and the refusal
 * rules, so the three backends carry a call rather than three copies of the
 * logic. The caller supplies the lock (each backend serializes register
 * access its own way) and its own port-0 ownership checks.
 *
 * CONFIGURE, NOT CHECK. The MT7612U arm writes nothing: its bring-up already
 * leaves the port identity on the station's own address, so its seam only
 * verifies that. These dies cannot take that shape. Jaguar2/3 bring-up never
 * programs MACID, and net_type comes up NoLink on all three, so a port left
 * as bring-up made it does not answer for `own`. The arm therefore writes
 * MACID = own, BSSID = the AP, net_type = Infra, and reads all three back.
 * (On the Jaguar1 8812 die bring-up does program the EFUSE MAC into MACID;
 * the arm writes it anyway, so every die ends in the same verified state.)
 *
 * REFUSALS, all logged, none throwing:
 *   - the seam's argument rule (both unicast, different), plus neither
 *     address all-zero (MACID = 0 is not a safe identity). Like the MT7612U,
 *     this refusal writes nothing and leaves an existing arm in place - a
 *     false return is not proof of a passive port (IRadio);
 *   - port 0 already has a net_type set by someone else on the FIRST arm:
 *     a beacon or an ACK responder owns it, and arming over it would take
 *     the port away from them. A re-arm (a new BSSID while armed) is not
 *     refused - it replaces our own arm and keeps the original snapshot, so
 *     Clear still returns to the state before the FIRST arm;
 *   - any readback that does not match. A FAILED arm - first or re-arm -
 *     rolls back to the pre-FIRST-arm snapshot and, once that verifies, is
 *     no longer armed: a re-arm that fails tears down the arm before it
 *     rather than guessing that the old identity still holds, and returns
 *     false so the caller knows the port is passive. When the rollback does
 *     not verify either, the arm stays recorded so ClearStationIdentity can
 *     retry it.
 *
 * LATER PORT-0 CLAIMANTS are REFUSED by the backends while a station is
 * armed (IRadio leaves the rule backend-specific): SetAckResponder and
 * StartBeacon return false, and ClearAckResponder leaves the port alone on
 * Jaguar2/3, where its gate-only clear would otherwise close the station's
 * net_type. Unlike the MT7612U, a station arm is never dropped from under
 * the caller.
 *
 * ORDERING (IRadio's clause): on Jaguar1/2/3 the port-0 writers are
 * bring-up, the tail of Init/InitWrite (the BF beamformee identity into
 * MACID, the configured ACK responder), the beacon and the ACK responder -
 * no RX-loop start writes 0x0102/0x0610/0x0618. So the backends refuse the
 * arm until Init/InitWrite has made its last port-0 write (each backend's
 * _station_ready, committed at the end of a bring-up that did not throw) and
 * need nothing else; calling before or after StartRxLoop is the same.
 *
 * LIFETIME: the arm ends with the session. Stop() and the destructor clear
 * it (best effort, logged), because not every teardown powers the chip down
 * (Jaguar2's does not; Jaguar1's is optional) and a port left on MACID = own
 * / Infra goes on acknowledging for a station whose process has gone. A
 * (re-)bring-up clears a held arm first (retire()); a clear that does not
 * verify keeps the record for ClearStationIdentity to retry. Stop() also
 * clears _station_ready, so an arm after Stop() is refused until the next
 * bring-up. */

#include <algorithm>
#include <cstdint>
#include <optional>

#include "AckResponder.h"
#include "IRadio.h"
#include "logger.h"

namespace devourer {

class StationArm {
public:
  bool armed() const { return _restore.has_value(); }

  /* The (re-)bring-up entry: a re-Init of an object that still holds an arm
   * clears it first. A clear that does not verify KEEPS the record, so a
   * later ClearStationIdentity retries it and armed() keeps the other port-0
   * claimants refused: bring-up does not reprogram MACID or net_type on
   * Jaguar2/3, so the port can still be answering for the station, and
   * dropping the record would make the next Clear a trivially-true no-op.
   * Keeping it is safe on every die - the snapshot is the station-free port
   * state an earlier bring-up left (MACID as bring-up programs it, or not;
   * net_type NoLink), which is what restoring it writes back. Caller holds
   * the backend's station lock. */
  void retire(RtlAdapter &dev, const Logger_t &log, const char *tag) {
    if (!armed())
      return;
    bool cleared = false;
    try {
      cleared = clear(dev, log, tag);
    } catch (...) {
    }
    if (!cleared) {
      try {
        log->error("{}: station identity kept across the bring-up: its clear "
                   "did not verify, so ClearStationIdentity will retry it",
                   tag);
      } catch (...) {
      }
    }
  }

  /* `retry_limit` is the session's DeviceConfig::tx.retry_limit as this die
   * applies it, or nullopt where the die ignores the knob and airs every
   * unicast once (the Jaguar1 8814A carve-out). It is not part of the arm;
   * it decides the WARN a successful arm prints when the station's own
   * unicast would never be retransmitted. */
  bool arm(RtlAdapter &dev, const MacAddr &own, const MacAddr &bssid,
           std::optional<int> retry_limit, const Logger_t &log,
           const char *tag) {
    if (!ack::station_args_ok(own.data(), bssid.data())) {
      log->error("{}: station identity refused: own and BSSID must both be "
                 "unicast, non-zero and different",
                 tag);
      return false;
    }
    if (!_restore) {
      ack::StationRestore snap;
      if (!ack::snapshot_station_restore(dev, snap)) {
        log->error("{}: station identity refused: the port-0 state could "
                   "not be read, so no rollback target exists",
                   tag);
        return false;
      }
      if (snap.net_type != 0) {
        log->error("{}: station identity refused: port 0 already has "
                   "net_type {} (a beacon or an ACK responder owns it, or "
                   "an earlier session left it set)",
                   tag, snap.net_type);
        return false;
      }
      _restore = snap;
    }
    /* The transfer status is not the verdict - a write can report failure
     * and land, or report success and not. station_is() decides. */
    (void)ack::arm_station(dev, own.data(), bssid.data());
    if (ack::station_is(dev, own.data(), bssid.data())) {
      log->info("{}: station identity armed: own "
                "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} BSSID "
                "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} (net_type=Infra)",
                tag, own.bytes[0], own.bytes[1], own.bytes[2], own.bytes[3],
                own.bytes[4], own.bytes[5], bssid.bytes[0], bssid.bytes[1],
                bssid.bytes[2], bssid.bytes[3], bssid.bytes[4],
                bssid.bytes[5]);
      warn_if_unicast_never_retried(retry_limit, log, tag);
      return true;
    }
    if (ack::restore_station(dev, *_restore)) {
      log->error("{}: station identity arm did not read back; pre-arm port "
                 "state restored and verified",
                 tag);
      _restore.reset();
    } else {
      log->error("{}: station identity arm did not read back AND the "
                 "rollback did not verify; port-0 state is unknown (Clear "
                 "will retry it)",
                 tag);
    }
    return false;
  }

  /* IRadio::ClearStationIdentity: true when nothing was armed (nothing to
   * undo) or the pre-arm state was restored AND read back. */
  bool clear(RtlAdapter &dev, const Logger_t &log, const char *tag) {
    if (!_restore)
      return true;
    if (!ack::restore_station(dev, *_restore)) {
      log->error("{}: station identity clear did not verify; the port may "
                 "still answer for the station address",
                 tag);
      return false;
    }
    _restore.reset();
    log->info("{}: station identity cleared (pre-arm MACID/BSSID/net_type "
              "restored)",
              tag);
    return true;
  }

  /* The arm covers RECEIVE and auto-ACK only. What the station transmits is
   * the caller's: its unicast must request an ACK
   * (build_stream_radiotap(mode, false)), and the descriptor retries an
   * unacknowledged frame only tx.retry_limit times - default 0. Not refused
   * (an RX-only or test session may want exactly that), but said, because
   * nothing else would say it. Same condition and wording as the MT7612U
   * arm. */
  static void warn_if_unicast_never_retried(std::optional<int> retry_limit,
                                            const Logger_t &log,
                                            const char *tag) {
    if (!retry_limit) {
      log->warn("{}: station identity armed on a die that ignores "
                "tx.retry_limit and sends every unicast frame once - the MAC "
                "will not retransmit this station's unacknowledged unicast",
                tag);
      return;
    }
    if (std::clamp(*retry_limit, 0, 63) == 0)
      log->warn("{}: station identity armed with tx.retry_limit=0 - the MAC "
                "will not retransmit this station's unacknowledged unicast. "
                "Set DEVOURER_TX_RETRY_LIMIT / tx.retry_limit (nonzero) and "
                "send unicast with an ACK-requesting radiotap",
                tag);
  }

private:
  std::optional<ack::StationRestore> _restore;
};

} /* namespace devourer */
