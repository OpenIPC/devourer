/* BeamformingSounder — Realtek 802.11ac explicit-sounding helpers for the
 * two-adapter "self-sounding" CSI path, shared across chip generations.
 *
 * With one adapter as beamformer and another as beamformee, an NDPA +
 * hardware-NDP exchange makes the beamformee emit a VHT Compressed Beamforming
 * report over the air, which a monitor RX captures — giving per-subcarrier
 * channel state the chip otherwise terminates in hardware. See
 * docs/beamforming-self-sounding.md.
 *
 * WHY THIS IS GENERATION-NEUTRAL: the MAC sounding registers are byte-identical
 * across the whole Realtek AC family — kSndPtclCtrl (0x718),
 * kBfmer0Info (0x6E4), REG_CSI_RPT_PARAM_BW20 (0x6F4), kBfmeeSel
 * (0x714), kTxbfCtrl (0x42C) are the same on 8192E/8703B/8723D/8812A/
 * 8814A/8822B/8822C/8822E (verified against every vendor include tree). The
 * beamformer arm (arm_sounder) is therefore fully shared. Only the beamformee
 * PHY details differ per generation, and those are captured in BfeeConfig:
 *   - the sounding-protocol control byte (0xCB Jaguar-1 / 0xDB Jaguar-2/3);
 *   - the BB CSI-content register 0x9B4 is CSI content on Jaguar-1 but the
 *     NARROWBAND CLOCK DIVIDER on Jaguar-3 (RadioManagementJaguar3.h) — it
 *     MUST NOT be written there, so csi_content_reg is 0 on Jaguar-2/3;
 *   - Jaguar-2/3 additionally gate the responder behind RX-filter bits
 *     (accept NDPA + action-no-ack) and an own-AID register (0x1680).
 *
 * Register values transcribed from the vendor beamformee-entry functions:
 *   Jaguar-1   hal_txbf_jaguar_enter()  (haltxbfjaguar.c)
 *   Jaguar-2/3 hal_txbf_8822b_enter()   (haltxbf8822b.c, shared 8822B/C/E)
 * Entry 0, P_AID = 0 (unassociated sounding — no CAM/macid entry).
 */
#ifndef BEAMFORMING_SOUNDER_H
#define BEAMFORMING_SOUNDER_H

#include <cstdint>

#include "RtlUsbAdapter.h"

namespace devourer {
namespace bf {

/* Shared MAC sounding registers (identical across the Realtek AC family).
 * k-prefixed to avoid colliding with the REG_* preprocessor macros in the
 * vendor hal_com_reg.h (macros ignore namespaces). */
enum : uint16_t {
  kTxbfCtrl        = 0x042C,
  kRxFltMap0       = 0x06A0, /* +1 bit6 = accept action-no-ack           */
  kRxFltMap1       = 0x06A2, /* [5:4] = accept NDPA + BF report poll      */
  kBfmer0Info      = 0x06E4,
  kCsiRptParam20   = 0x06F4,
  kCsiRptParam40   = 0x06F8,
  kCsiRptParam80   = 0x06FC,
  kBfmeeSel        = 0x0714,
  kSndPtclCtrl     = 0x0718,
  kSndNdpStandby   = 0x071B,
  kBbCsiContent    = 0x09B4, /* Jaguar-1 only (clock divider on Jaguar-3) */
  kOurAid          = 0x1680, /* Jaguar-2/3 only: our AID [11:0]           */
};

enum : uint8_t {
  kTxbfNdpaEnables = 0x10 | 0x40 | 0x80, /* 0x42F BIT4|6|7: NDPA TX enable */
  kRxFlt0ActNoAck  = 0x40,               /* 0x6A1 bit6                     */
  kRxFlt1NdpaPoll  = 0x30,               /* 0x6A2 [5:4]                    */
};

/* Per-generation beamformee knobs. */
struct BfeeConfig {
  uint8_t  snd_ptcl_ctrl;      /* 0xCB Jaguar-1 / 0xDB Jaguar-2/3            */
  uint8_t  ndp_standby;        /* 0x50                                       */
  uint32_t csi_rpt_param;      /* report matrix dims (1T1R VHT SU = 0x0109)  */
  bool     csi_rpt_param_16;   /* 16-bit @0x6F4 (Jg2/3) vs 32-bit x3 (Jg1)   */
  uint16_t csi_content_reg;    /* 0x9B4 on Jaguar-1; 0 = skip on Jaguar-2/3  */
  uint32_t csi_content_val;
  bool     set_our_aid;        /* write 0 to 0x1680 (Jaguar-2/3)             */
  bool     set_rxflt;          /* enable NDPA/action-no-ack RX (Jaguar-2/3)  */
};

/* Jaguar-1 (8812/8814/8821): BB CSI-content at 0x9B4, 32-bit params, no
 * RX-filter poke (monitor mode already accepts the frames). */
constexpr BfeeConfig kBfeeJaguar1{
    /*snd_ptcl_ctrl*/ 0xCB, /*ndp_standby*/ 0x50,
    /*csi_rpt_param*/ 0x01080108, /*csi_rpt_param_16*/ false,
    /*csi_content_reg*/ kBbCsiContent, /*csi_content_val*/ 0x01081008,
    /*set_our_aid*/ false, /*set_rxflt*/ false};

/* Jaguar-2/3 (8822B/C/E): 0xDB, 16-bit param at 0x6F4, own-AID + RX-filter
 * gates, and NO 0x9B4 write (that address is the narrowband clock divider). */
constexpr BfeeConfig kBfeeJaguar23{
    /*snd_ptcl_ctrl*/ 0xDB, /*ndp_standby*/ 0x50,
    /*csi_rpt_param*/ 0x0109, /*csi_rpt_param_16*/ true,
    /*csi_content_reg*/ 0, /*csi_content_val*/ 0,
    /*set_our_aid*/ true, /*set_rxflt*/ true};

/* Beamformer side (fully shared): arm the MAC sounding engine so a
 * TX-descriptor-marked NDPA is followed by a hardware-generated NDP.
 * Beamformee entry 0, P_AID = 0 (unassociated). */
inline void arm_sounder(RtlUsbAdapter& dev) {
  dev.rtw_write8(kSndPtclCtrl, 0xCB);
  dev.rtw_write8(kSndNdpStandby, 0x50);
  dev.rtw_write16(kTxbfCtrl, 0x0000);                 /* P_AID = 0 */
  uint8_t v = dev.rtw_read8(kTxbfCtrl + 3);
  dev.rtw_write8(kTxbfCtrl + 3, v | kTxbfNdpaEnables);
  uint8_t s = dev.rtw_read8(kBfmeeSel + 3);
  dev.rtw_write8(kBfmeeSel + 3, (s & 0x03) | 0x60);
  dev.rtw_write16(kBfmeeSel, 0x0200);                 /* entry-0 select */
}

/* Beamformee side: arm the hardware CSI responder to reply to NDPA+NDP from
 * `beamformer_mac` (matched against the NDPA TA), no association required.
 * `cfg` selects the per-generation register recipe. */
inline void arm_beamformee(RtlUsbAdapter& dev, const uint8_t beamformer_mac[6],
                           const BfeeConfig& cfg) {
  dev.rtw_write8(kSndPtclCtrl, cfg.snd_ptcl_ctrl);
  dev.rtw_write8(kSndNdpStandby, cfg.ndp_standby);
  for (uint16_t i = 0; i < 6; ++i)
    dev.rtw_write8(kBfmer0Info + i, beamformer_mac[i]);
  if (cfg.set_our_aid)
    dev.rtw_write16(kOurAid, 0x0000);                 /* our AID = 0 */
  if (cfg.set_rxflt) {
    uint8_t r0 = dev.rtw_read8(kRxFltMap0 + 1);
    dev.rtw_write8(kRxFltMap0 + 1, r0 | kRxFlt0ActNoAck);
    uint8_t r1 = dev.rtw_read8(kRxFltMap1);
    dev.rtw_write8(kRxFltMap1, r1 | kRxFlt1NdpaPoll);
  }
  if (cfg.csi_rpt_param_16) {
    dev.rtw_write16(kCsiRptParam20,
                    static_cast<uint16_t>(cfg.csi_rpt_param));
  } else {
    dev.rtw_write32(kCsiRptParam20, cfg.csi_rpt_param);
    dev.rtw_write32(kCsiRptParam40, cfg.csi_rpt_param);
    dev.rtw_write32(kCsiRptParam80, cfg.csi_rpt_param);
  }
  if (cfg.csi_content_reg)
    dev.rtw_write32(cfg.csi_content_reg, cfg.csi_content_val);
}

}  // namespace bf
}  // namespace devourer

#endif  // BEAMFORMING_SOUNDER_H
