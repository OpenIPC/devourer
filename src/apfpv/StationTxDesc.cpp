// ============================================================================
//  StationTxDesc.cpp — unicast station TX descriptor (impl)
//  KEY FIX vs devourer monitor path: BMC=0 (unicast, enables ACK handshake).
//  Reuses devourer's working HWSEQ/retry/rate/checksum machinery.
//  Declarations in StationTxDesc.h.
// ============================================================================
#include "basic_types.h"
#include "jaguar1/FrameParser.h"
#include "StationTxDesc.h"
#include <cstdlib>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace apfpv {

// Firmware-RA uplink toggle. ON by default. Disable with DEVOURER_NO_FW_RA (host) or, on Android
// where env isn't settable, `setprop debug.pixelpilot.fwra 0`. Disabling reverts DATA frames to a
// fixed low uplink rate → the AP stops aggregating (declines-BA clean path) → no A-MPDU PN-replay
// loss (smoother for low-bitrate video; loses the VHT throughput headroom).
static bool fwRaEnabled() {
    if (std::getenv("DEVOURER_NO_FW_RA") != nullptr) return false;
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get("debug.pixelpilot.fwra", v) > 0 && v[0] == '0') return false;
#endif
    return true;
}

void FillStationTxDesc(uint8_t* txdesc, uint16_t payloadLen, uint8_t descOffset,
                       uint8_t macId, StationFrameKind kind, uint8_t rateId, uint8_t txRate) {
    SET_TX_DESC_PKT_SIZE_8812(txdesc, payloadLen);
    SET_TX_DESC_OFFSET_8812(txdesc, descOffset);
    SET_TX_DESC_FIRST_SEG_8812(txdesc, 1);
    SET_TX_DESC_LAST_SEG_8812(txdesc, 1);
    SET_TX_DESC_OWN_8812(txdesc, 1);
    // Broadcast mgmt (beacon) must set BMC=1 so the chip does NOT wait for an ACK
    // and retry — otherwise the frame never radiates. Unicast station frames keep
    // BMC=0 (the ACK-handshake fix).
    // Beacon + broadcast mgmt are BMC (no ACK wait / no retry). Beacon additionally goes to the
    // BEACON queue (QSEL 0x10) so that in AP/master mode (MSR=AP + ENSWBCN) the MAC actually
    // radiates it at TBTT — the normal mgmt queue (0x12) is NOT transmitted once MSR=AP.
    bool bmc = (kind == StationFrameKind::BroadcastMgmt || kind == StationFrameKind::Beacon);
    SET_TX_DESC_BMC_8812(txdesc, bmc ? 1 : 0);
    SET_TX_DESC_MACID_8812(txdesc, macId);
    SET_TX_DESC_QUEUE_SEL_8812(txdesc,
        (kind == StationFrameKind::Beacon) ? 0x10 :
        (kind == StationFrameKind::Mgmt || kind == StationFrameKind::BroadcastMgmt) ? 0x12 : 0x00);
    SET_TX_DESC_SEC_TYPE_8812(txdesc, 0);            // SW-CCMP / cleartext mgmt
    SET_TX_DESC_HWSEQ_EN_8812(txdesc, 1);
    SET_TX_DESC_RATE_ID_8812(txdesc, rateId);
    // ⭐ Uplink rate control — THE VHT lever (validated on the VM kernel-diff rig 2026-07-14).
    // mgmt/EAPOL/handshake use fixed USE_RATE=1 (reliable — they must not ride an un-converged
    // firmware RA and fail to associate). DATA frames use USE_RATE=0 = firmware rate-adaptation
    // (per-macid RA, ramps with link quality), matching the kernel which NEVER pins a fixed
    // uplink rate. Root cause of the old ~22Mbps ceiling: the RTL8812 AP mirrors our uplink
    // rate onto its downlink RA, so a fixed-low (6Mbps) uplink pinned us at HT-1SS-MCS2 with NO
    // aggregation. Firmware-RA uplink → AP ramps to VHT-2SS/80MHz AND auto-emits the compressed
    // BlockAck (A-MPDU 99.8%): 22 → 48-53 Mbps (2.2-2.4x), the wall 10+ prior sessions couldn't
    // pass. DEFAULT ON for data; DEVOURER_NO_FW_RA reverts to fixed-rate for A/B / if an AP
    // dislikes it. (HW-decrypt + BA-accept are NOT required — SW decrypt handles the A-MPDU.)
    bool fwRa = (kind == StationFrameKind::CcmpData) && fwRaEnabled();
    SET_TX_DESC_USE_RATE_8812(txdesc, fwRa ? 0 : 1);
    if (!fwRa) SET_TX_DESC_TX_RATE_8812(txdesc, txRate);
    SET_TX_DESC_RETRY_LIMIT_ENABLE_8812(txdesc, bmc ? 0 : 1);   // no retry for broadcast
    SET_TX_DESC_DATA_RETRY_LIMIT_8812(txdesc, bmc ? 0 : 12);
    rtl8812a_cal_txdesc_chksum(txdesc);
}

} // namespace apfpv
