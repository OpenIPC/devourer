/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT76x0U register definitions. Phase 1: a copy of src/mt7612u/regs.h, which
 * is trimmed from openwrt/mt76 @ be5ce79 (mt76x02_regs.h, mt76x2/mcu.h,
 * mt76x02_mcu.h, mt76x02_eeprom.h). The mt76x02_* parts are shared by both
 * MACs; mt76x2-only entries (the CFG-space power-up block, the MT7612U
 * calibration and CR-load commands) are still here and get replaced by their
 * mt76x0 counterparts (mt76x0/mcu.h, mt76x0/eeprom.h, mt76x0/phy.h) as the
 * bring-up phases port the code that uses them.
 * Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo Bianconi / Stanislaw Gruszka.
 */
#ifndef MT76X0U_REGS_H
#define MT76X0U_REGS_H

#include <stdint.h>

#define BIT(n)              (1u << (n))
#define GENMASK(h, l)       (((~0u) - (1u << (l)) + 1) & (~0u >> (31 - (h))))

/*
 * Position of the lowest set bit of a mask, for FIELD_PREP/FIELD_GET.
 *
 * Not __builtin_ctz: MSVC has no such builtin, and its _BitScanForward is a
 * function taking an out-parameter, so it cannot appear in a constant
 * expression - which these must be, because FIELD_PREP initialises static
 * tables (see ext_cca_chan in phy.c). The isolate-lowest-bit plus binary
 * search below is a constant expression on every compiler and folds to a
 * single instruction under optimisation.
 *
 * `m` is evaluated several times; every mask here is a compile-time constant,
 * so that is a documentation point rather than a hazard.
 *
 * The old name for this was `_SHIFT`, which is reserved to the implementation
 * in every scope - leading underscore followed by a capital.
 */
#define MT_LOWBIT(m)        ((uint32_t)(m) & (~(uint32_t)(m) + 1u))
#define MT_CTZ(m) ( \
	((MT_LOWBIT(m) & 0xffff0000u) ? 16u : 0u) | \
	((MT_LOWBIT(m) & 0xff00ff00u) ?  8u : 0u) | \
	((MT_LOWBIT(m) & 0xf0f0f0f0u) ?  4u : 0u) | \
	((MT_LOWBIT(m) & 0xccccccccu) ?  2u : 0u) | \
	((MT_LOWBIT(m) & 0xaaaaaaaau) ?  1u : 0u))
#define FIELD_PREP(m, v)    (((uint32_t)(v) << MT_CTZ(m)) & (m))
#define FIELD_GET(m, v)     (((uint32_t)(v) & (m)) >> MT_CTZ(m))

/* Address-space selectors. Stripped before the transfer; they pick bRequest. */
#define MT_VEND_TYPE_EEPROM BIT(31)
#define MT_VEND_TYPE_CFG    BIT(30)
#define MT_VEND_TYPE_MASK   (MT_VEND_TYPE_EEPROM | MT_VEND_TYPE_CFG)
#define CFG_ADDR(n)         (MT_VEND_TYPE_CFG | (n))
#define EEP_ADDR(n)         (MT_VEND_TYPE_EEPROM | (n))

/* EP0 vendor requests (mt76.h enum mt0_vendor_req) */
#define MT_VEND_DEV_MODE     0x01
#define MT_VEND_WRITE        0x02
#define MT_VEND_POWER_ON     0x04
#define MT_VEND_MULTI_WRITE  0x06
#define MT_VEND_MULTI_READ   0x07
#define MT_VEND_READ_EEPROM  0x09
#define MT_VEND_WRITE_FCE    0x42
#define MT_VEND_WRITE_CFG    0x46
#define MT_VEND_READ_CFG     0x47
#define MT_VEND_READ_EXT     0x63
#define MT_VEND_WRITE_EXT    0x66

/* Bulk endpoints, in mt76u_set_endpoints() descriptor order. Verified live. */
#define MT_EP_IN_PKT_RX      0x84
#define MT_EP_IN_CMD_RESP    0x85
#define MT_EP_OUT_INBAND_CMD 0x08
#define MT_EP_OUT_AC_BE      0x04
#define MT_EP_OUT_AC_BK      0x05
#define MT_EP_OUT_AC_VI      0x06
#define MT_EP_OUT_AC_VO      0x07
#define MT_EP_OUT_HCCA       0x09

/* ---- identity / power ---- */
#define MT_ASIC_VERSION      0x0000
#define MT_COEXCFG0          0x0040
#define MT_COEXCFG0_COEX_EN  BIT(0)
#define MT_WLAN_FUN_CTRL     0x0080
#define MT_WLAN_FUN_CTRL_WLAN_EN        BIT(0)
#define MT_WLAN_FUN_CTRL_WLAN_CLK_EN    BIT(1)
#define MT_WLAN_FUN_CTRL_WLAN_RESET_RF  BIT(2)
#define MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL BIT(5)

#define MT_XO_CTRL5          0x0114
#define MT_XO_CTRL5_C2_VAL   GENMASK(14, 8)
#define MT_XO_CTRL6          0x0118
#define MT_XO_CTRL6_C2_CTRL  GENMASK(14, 8)
#define MT_XO_CTRL7          0x011c

/* CFG-space power-up block (mt76x2u_power_on) */
#define MT_CFG_MTC_CTRL      0x148   /* via CFG_ADDR() */
#define MT_WLAN_MTC_CTRL_MTCMOS_PWR_UP BIT(0)
#define MT_WLAN_MTC_CTRL_PWR_ACK       BIT(12)
#define MT_WLAN_MTC_CTRL_PWR_ACK_S     BIT(13)
#define MT_WLAN_MTC_CTRL_STATE_UP      BIT(28)

#define MT_USB_U3DMA_CFG     0x9018  /* via CFG_ADDR() */
#define MT_USB_DMA_CFG_RX_BULK_AGG_TOUT GENMASK(7, 0)
#define MT_USB_DMA_CFG_RX_DROP_OR_PAD   BIT(18)
#define MT_USB_DMA_CFG_TX_CLR           BIT(19)  /* declared by mt76, written by nothing */
#define MT_USB_DMA_CFG_RX_BULK_AGG_EN   BIT(21)
#define MT_USB_DMA_CFG_RX_BULK_EN       BIT(22)
#define MT_USB_DMA_CFG_TX_BULK_EN       BIT(23)
#define MT_USB_DMA_CFG_RX_BUSY          BIT(30)
#define MT_USB_DMA_CFG_TX_BUSY          BIT(31)

/* ---- MCU ---- */
#define MT_MCU_CPU_CTL       0x0704
#define MT_MCU_CLOCK_CTL     0x0708
#define MT_MCU_RESET_CTL     0x070c
#define MT_MCU_INT_LEVEL     0x0718
#define MT_MCU_COM_REG0      0x0730
#define MT_MCU_COM_REG1      0x0734
#define MT_MCU_SEMAPHORE_03  0x07bc

#define MT_FCE_PSE_CTRL              0x0800
#define MT_FCE_L2_STUFF              0x080c
#define MT_FCE_L2_STUFF_WR_MPDU_LEN_EN BIT(4)
#define MT_FCE_DMA_ADDR              0x0230
#define MT_FCE_DMA_LEN               0x0234
#define MT_TX_CPU_FROM_FCE_BASE_PTR  0x09a0
#define MT_TX_CPU_FROM_FCE_MAX_COUNT 0x09a4
#define MT_TX_CPU_FROM_FCE_CPU_DESC_IDX 0x09a8
#define MT_FCE_PDMA_GLOBAL_CONF      0x09c4
#define MT_FCE_SKIP_FS               0x0a6c

/* MCU in-band message header (mt76x02_dma.h) */
#define MT_MCU_MSG_LEN       GENMASK(15, 0)
#define MT_MCU_MSG_CMD_SEQ   GENMASK(19, 16)
#define MT_MCU_MSG_CMD_TYPE  GENMASK(26, 20)
#define MT_MCU_MSG_PORT      GENMASK(29, 27)
#define MT_MCU_MSG_TYPE_CMD  BIT(30)

#define MT_RX_FCE_INFO_LEN      GENMASK(13, 0)
#define MT_RX_FCE_INFO_CMD_SEQ  GENMASK(19, 16)
#define MT_RX_FCE_INFO_EVT_TYPE GENMASK(23, 20)
#define MT_EVT_CMD_DONE         0

enum mt0_dma_msg_port { WLAN_PORT, CPU_RX_PORT, CPU_TX_PORT, HOST_PORT };

enum mt0_mcu_cmd {
	CMD_FUN_SET_OP = 1, CMD_LOAD_CR = 2, CMD_INIT_GAIN_OP = 3,
	CMD_RANDOM_READ = 10, CMD_RANDOM_WRITE = 12,
	CMD_POWER_SAVING_OP = 20, CMD_SWITCH_CHANNEL_OP = 30,
	CMD_CALIBRATION_OP = 31,
};
enum mt0_mcu_function { Q_SELECT = 1, BW_SETTING = 2, GET_FW_VERSION = 5 };
enum mt0_mcu_power_mode { RADIO_OFF = 0x30, RADIO_ON = 0x31 };
/* mt76x0/mcu.h - NOT mt76x2's numbering (the MT7612U copy had that: there
 * RXDCOC is 3 and there is no VCO). The firmware takes these ids. */
enum mt0_mcu_calibration {
	MCU_CAL_R = 1, MCU_CAL_RXDCOC, MCU_CAL_LC, MCU_CAL_LOFT, MCU_CAL_TXIQ,
	MCU_CAL_BW, MCU_CAL_DPD, MCU_CAL_RXIQ, MCU_CAL_TXDCOC,
	MCU_CAL_RX_GROUP_DELAY, MCU_CAL_TX_GROUP_DELAY, MCU_CAL_VCO,
	MCU_CAL_NO_SIGNAL = 0xfe, MCU_CAL_FULL = 0xff,
};
enum mt0_mcu_cr_mode { MT_RF_CR, MT_BBP_CR, MT_RF_BBP_CR, MT_HL_TEMP_CR_UPDATE };

/* ---- MAC ---- */
#define MT_MAC_CSR0          0x1000
#define MT_MAC_SYS_CTRL      0x1004
#define MT_MAC_SYS_CTRL_RESET_CSR BIT(0)
#define MT_MAC_SYS_CTRL_RESET_BBP BIT(1)
#define MT_MAC_SYS_CTRL_ENABLE_TX BIT(2)
#define MT_MAC_SYS_CTRL_ENABLE_RX BIT(3)
#define MT_MAC_ADDR_DW0      0x1008
#define MT_MAC_ADDR_DW1      0x100c
#define MT_MAC_ADDR_DW1_U2ME_MASK GENMASK(23, 16)
#define MT_MAC_BSSID_DW0     0x1010
#define MT_MAC_BSSID_DW1     0x1014
/*
 * These three were transcribed two bits high (MBSS_MODE at 19:18, MBEACON_N at
 * 22:20, LOCAL_BIT at 23) - the whole group shifted left by 2 versus mt76's
 * mt76x02_regs.h. It was harmless while this HAL only injected: mac_setaddr()
 * is the only user, and MBSS_MODE / MBEACON_N steer beacon generation, which
 * an injector never exercises. The AP path beacons, so the true positions
 * matter - the shifted masks programmed MBSS_MODE=4 (invalid) and MBEACON_N=15.
 */
#define MT_MAC_BSSID_DW1_ADDR           GENMASK(15, 0)
#define MT_MAC_BSSID_DW1_MBSS_MODE      GENMASK(17, 16)
#define MT_MAC_BSSID_DW1_MBEACON_N      GENMASK(20, 18)
#define MT_MAC_BSSID_DW1_MBSS_LOCAL_BIT BIT(21)
#define MT_MAC_BSSID_DW1_MBSS_MODE_B2   BIT(22)
#define MT_MAC_BSSID_DW1_MBEACON_N_B3   BIT(23)
#define MT_MAX_LEN_CFG       0x1018
#define MT_XIFS_TIME_CFG     0x1100
#define MT_XIFS_TIME_CFG_OFDM_SIFS GENMASK(15, 8)
#define MT_BKOFF_SLOT_CFG    0x1104
#define MT_BKOFF_SLOT_CFG_CC_DELAY GENMASK(11, 8)
#define MT_BEACON_TIME_CFG   0x1114
#define MT_BEACON_TIME_CFG_INTVAL   GENMASK(15, 0)   /* in 1/16 TU units */
#define MT_BEACON_TIME_CFG_TIMER_EN BIT(16)
#define MT_BEACON_TIME_CFG_SYNC_MODE GENMASK(18, 17)
#define MT_BEACON_TIME_CFG_TBTT_EN  BIT(19)
#define MT_BEACON_TIME_CFG_BEACON_TX BIT(20)
/* Reserved-page beacon (mt76x02_beacon.c). BCN_OFFSET packs four slot offsets
 * (each offset/64) per register. BCN_BYPASS_MASK is per-slot and INVERTED: a
 * set bit SUPPRESSES that slot (0xffff = all suppressed, used during an
 * update); a slot only airs once its bit is CLEARED. mt76 enables N written
 * beacons with 0xff00 | ~(0xff00 >> N). */
#define MT_BCN_OFFSET_BASE   0x041c
#define MT_BCN_OFFSET(_n)    (MT_BCN_OFFSET_BASE + ((_n) << 2))
#define MT_BCN_BYPASS_MASK   0x108c
#define MT_BEACON_BASE       0xc000
#define MT_TSF_TIMER_DW0     0x111c
#define MT_TSF_TIMER_DW1     0x1120
#define MT_MAC_STATUS        0x1200
#define MT_MAC_STATUS_TX     BIT(1)
#define MT_MAC_STATUS_RX     BIT(0)

#define MT_WPDMA_GLO_CFG     0x0208
#define MT_WPDMA_GLO_CFG_TX_DMA_BUSY BIT(1)
#define MT_WPDMA_GLO_CFG_RX_DMA_BUSY BIT(3)
#define MT_PBF_TX_MAX_PCNT   0x0408
#define MT_PBF_RX_MAX_PCNT   0x040c
#define MT_WMM_AIFSN         0x0214
#define MT_WMM_CWMIN         0x0218
#define MT_WMM_CWMAX         0x021c
#define MT_US_CYC_CFG        0x02a4
#define MT_US_CYC_CNT        GENMASK(7, 0)

#define MT_TXOP_CTRL_CFG     0x1340
#define MT_TXOP_ED_CCA_EN    BIT(20)
#define MT_TX_RTS_CFG        0x1344
#define MT_TX_RTS_CFG_RETRY_LIMIT GENMASK(7, 0)
#define MT_TX_RETRY_CFG      0x134c
/* rt2800 TX_RTY_CFG layout (initvals 0x47f01f0f = short 15, long 31, long
 * threshold 2032 bytes): a frame longer than the threshold uses the long
 * limit, anything else the short one. */
#define MT_TX_RETRY_CFG_SHORT GENMASK(7, 0)
#define MT_TX_RETRY_CFG_LONG  GENMASK(15, 8)
/* The word mt76x0u_set_retry_limit() writes: both limits set to `limit`
 * (0..255), every other field (the long threshold at bits 16+) preserved.
 * Pure, so tests/field_macros pins it without a device. */
static inline uint32_t mt0_retry_cfg_with_limit(uint32_t cur, uint32_t limit)
{
	cur &= ~(MT_TX_RETRY_CFG_SHORT | MT_TX_RETRY_CFG_LONG);
	return cur | FIELD_PREP(MT_TX_RETRY_CFG_SHORT, limit) |
	       FIELD_PREP(MT_TX_RETRY_CFG_LONG, limit);
}
#define MT_TX_LINK_CFG       0x1350
#define MT_TX_CFACK_EN       BIT(12)
#define MT_TX_PWR_CFG_0      0x1314
#define MT_TX_PWR_CFG_1      0x1318
#define MT_TX_PWR_CFG_2      0x131c
#define MT_TX_PWR_CFG_3      0x1320
#define MT_TX_PWR_CFG_4      0x1324
#define MT_TX_PWR_CFG_7      0x13d4
#define MT_TX_PWR_CFG_8      0x13d8
#define MT_TX_PWR_CFG_9      0x13dc
#define MT_TX_ALC_CFG_0      0x13b0
#define MT_TX_ALC_CFG_1      0x13b4
#define MT_TX_ALC_CFG_2      0x13a8
#define MT_TX_ALC_CFG_3      0x13ac
#define MT_TX_ALC_CFG_4      0x13c0
#define MT_RX_FILTR_CFG      0x1400
#define MT_RX_FILTR_CFG_CRC_ERR   BIT(0)
#define MT_RX_FILTR_CFG_PHY_ERR   BIT(1)
#define MT_RX_FILTR_CFG_PROMISC   BIT(2)
#define MT_RX_FILTR_CFG_OTHER_BSS BIT(3)
#define MT_RX_FILTR_CFG_VER_ERR   BIT(4)
#define MT_RX_FILTR_CFG_MCAST     BIT(5)
#define MT_RX_FILTR_CFG_BCAST     BIT(6)
#define MT_RX_FILTR_CFG_DUP       BIT(7)
#define MT_RX_FILTR_CFG_CFACK     BIT(8)
#define MT_RX_FILTR_CFG_CFEND     BIT(9)
#define MT_RX_FILTR_CFG_ACK       BIT(10)
#define MT_RX_FILTR_CFG_CTS       BIT(11)
#define MT_RX_FILTR_CFG_RTS       BIT(12)
#define MT_RX_FILTR_CFG_PSPOLL    BIT(13)
#define MT_RX_FILTR_CFG_BA        BIT(14)
#define MT_RX_FILTR_CFG_BAR       BIT(15)
#define MT_RX_FILTR_CFG_CTRL_RSV  BIT(16)
/* Every bit is a DROP bit. The managed-station filter: the initvals value
 * mt0_mac_start() programs, and what every station cell in
 * docs/mt7612u-station-identity.md measured. Drops FCS and PLCP failures,
 * unicast whose addr1 is not MT_MAC_ADDR (PROMISC - mt76x2u_config() sets it
 * whenever the phy is not in monitor mode), bad protocol versions, hardware-detected
 * duplicates and the control frames a station has no use for. KEEPS
 * broadcast, multicast, other-BSS frames (so beacons and group traffic from
 * every BSS, for a re-scan), PS-Poll and BAR. */
#define MT_RX_FILTR_CFG_MANAGED                                            \
	(MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR |                  \
	 MT_RX_FILTR_CFG_PROMISC | MT_RX_FILTR_CFG_VER_ERR |                  \
	 MT_RX_FILTR_CFG_DUP | MT_RX_FILTR_CFG_CFACK | MT_RX_FILTR_CFG_CFEND | \
	 MT_RX_FILTR_CFG_ACK | MT_RX_FILTR_CFG_CTS | MT_RX_FILTR_CFG_RTS |     \
	 MT_RX_FILTR_CFG_BA | MT_RX_FILTR_CFG_CTRL_RSV)
#define MT_AUTO_RSP_CFG      0x1404
#define MT_AUTO_RSP_EN       BIT(0)
#define MT_AUTO_RSP_PREAMB_SHORT BIT(4)
#define MT_EXT_CCA_CFG       0x141c
#define MT_EXT_CCA_CFG_CCA0     GENMASK(1, 0)
#define MT_EXT_CCA_CFG_CCA1     GENMASK(3, 2)
#define MT_EXT_CCA_CFG_CCA2     GENMASK(5, 4)
#define MT_EXT_CCA_CFG_CCA3     GENMASK(7, 6)
#define MT_EXT_CCA_CFG_CCA_MASK GENMASK(11, 8)
#define MT_TXOP_HLDR_ET      0x1608
#define MT_TXOP_HLDR_TX40M_BLK_EN BIT(1)
#define MT_PROT_AUTO_TX_CFG  0x1648

#define MT_WCID_ATTR_BASE    0xa800
#define MT_WCID_ATTR(_n)     (MT_WCID_ATTR_BASE + ((_n) & 0xff) * 4)
#define MT_WCID_ADDR_BASE    0x1800
#define MT_WCID_ADDR(_n)     (MT_WCID_ADDR_BASE + (_n) * 8)
#define MT_SKEY_BASE_0       0xac00
#define MT_SKEY_BASE_1       0xb400
#define MT_SKEY_0(_b, _i)    (MT_SKEY_BASE_0 + (4 * (_b) + (_i)) * 32)
#define MT_SKEY_1(_b, _i)    (MT_SKEY_BASE_1 + (4 * ((_b) & 7) + (_i)) * 32)
#define MT_SKEY(_b, _i)      (((_b) & 8) ? MT_SKEY_1(_b, _i) : MT_SKEY_0(_b, _i))
#define MT_SKEY_MODE_BASE_0  0xb000
#define MT_SKEY_MODE_BASE_1  0xb3f0
#define MT_SKEY_MODE_0(_b)   (MT_SKEY_MODE_BASE_0 + (((_b) / 2) << 2))
#define MT_SKEY_MODE_1(_b)   (MT_SKEY_MODE_BASE_1 + ((((_b) & 7) / 2) << 2))
#define MT_SKEY_MODE(_b)     (((_b) & 8) ? MT_SKEY_MODE_1(_b) : MT_SKEY_MODE_0(_b))
#define MT_SKEY_MODE_MASK    GENMASK(3, 0)
#define MT_SKEY_MODE_SHIFT(_b, _i) (4 * ((_i) + 4 * ((_b) & 1)))
#define MT_PBF_SYS_CTRL      0x0400
#define MT_PBF_SYS_CTRL_DMA_RESET BIT(1)   /* mt76x02_regs.h */
#define MT_PBF_SYS_CTRL_PBF_RESET BIT(3)
#define MT_WCID_TX_RATE_BASE 0x1c00
#define MT_WCID_TX_RATE(_i)  (MT_WCID_TX_RATE_BASE + ((_i) << 3))
#define MT_WCID_TX_INFO_RATE      GENMASK(15, 0)
#define MT_WCID_TX_INFO_NSS       GENMASK(17, 16)
#define MT_WCID_TX_INFO_TXPWR_ADJ GENMASK(25, 18)
#define MT_WCID_TX_INFO_SET       BIT(31)

/* BBP indirect. NOTE: each group base is an ABSOLUTE address, not an offset
 * from a common base - getting this wrong silently writes into a different
 * BBP group (AGC 0x2300 vs RXC 0x2500 differ by exactly that mistake). */
#define MT_BBP_CORE_BASE     0x2000
#define MT_BBP_IBI_BASE      0x2100
#define MT_BBP_AGC_BASE      0x2300
#define MT_BBP_TXC_BASE      0x2400
#define MT_BBP_RXC_BASE      0x2500
#define MT_BBP_TXO_BASE      0x2600
#define MT_BBP_TXBE_BASE     0x2700
#define MT_BBP_RXFE_BASE     0x2800
#define MT_BBP_RXO_BASE      0x2900
#define MT_BBP_DFS_BASE      0x2a00
#define MT_BBP_TR_BASE       0x2b00
#define MT_BBP_CAL_BASE      0x2c00
#define MT_BBP_DSC_BASE      0x2e00
#define MT_BBP_PFMU_BASE     0x2f00
#define MT_BBP(_type, _n)    (MT_BBP_##_type##_BASE + ((_n) << 2))

#define MT_MAC_APC_BSSID_BASE   0x1090
#define MT_MAC_APC_BSSID_L(_n)  (MT_MAC_APC_BSSID_BASE + ((_n) * 8))
#define MT_MAC_APC_BSSID_H(_n)  (MT_MAC_APC_BSSID_BASE + ((_n) * 8 + 4))
#define MT_MAC_APC_BSSID_H_ADDR GENMASK(15, 0)

/* ---- PHY / TX power ---- */
#define MT_BB_PA_MODE_CFG0   0x1214
#define MT_BB_PA_MODE_CFG1   0x1218
#define MT_RF_PA_MODE_CFG0   0x121c
#define MT_RF_PA_MODE_CFG1   0x1220
#define MT_RF_PA_MODE_ADJ0   0x1228
#define MT_RF_PA_MODE_ADJ1   0x122c
#define MT_TX_BAND_CFG       0x132c
#define MT_TX_BAND_CFG_UPPER_40M BIT(0)
#define MT_TX_BAND_CFG_5G    BIT(1)
#define MT_TX_BAND_CFG_2G    BIT(2)
#define MT_TX_SW_CFG0        0x1330
#define MT_TX_SW_CFG1        0x1334
#define MT_TX0_RF_GAIN_CORR  0x13a0
#define MT_TX1_RF_GAIN_CORR  0x13a4
#define MT_TX_ALC_CFG_0_CH_INIT_0 GENMASK(5, 0)
#define MT_TX_ALC_CFG_0_CH_INIT_1 GENMASK(13, 8)
#define MT_TX_ALC_CFG_1_TEMP_COMP GENMASK(5, 0)
#define MT_TX_ALC_CFG_2_TEMP_COMP GENMASK(5, 0)
/* --- MIB counters. All read-and-clear: mt76x02_mac_reset_counters() zeroes
 * them by reading, and mt76x02_mac_cc_reset() says so of the channel timers.
 * A caller must therefore difference nothing - each read IS the interval. --- */
#define MT_CH_TIME_CFG       0x110c
#define MT_CH_TIME_CFG_TIMER_EN      BIT(0)
#define MT_CH_TIME_CFG_TX_AS_BUSY    BIT(1)
#define MT_CH_TIME_CFG_RX_AS_BUSY    BIT(2)
#define MT_CH_TIME_CFG_NAV_AS_BUSY   BIT(3)
#define MT_CH_TIME_CFG_EIFS_AS_BUSY  BIT(4)
#define MT_CH_TIME_CFG_MDRDY_CNT_EN  BIT(5)
#define MT_CH_CCA_RC_EN              BIT(6)
#define MT_CH_TIME_CFG_CH_TIMER_CLR  GENMASK(9, 8)
#define MT_CH_IDLE           0x1130
#define MT_CH_BUSY           0x1134

#define MT_RX_STAT_0         0x1700
#define MT_RX_STAT_0_CRC_ERRORS GENMASK(15, 0)
#define MT_RX_STAT_0_PHY_ERRORS GENMASK(31, 16)
#define MT_RX_STAT_1         0x1704
#define MT_RX_STAT_1_CCA_ERRORS GENMASK(15, 0)
#define MT_RX_STAT_1_PLCP_ERRORS GENMASK(31, 16)
#define MT_RX_STAT_2         0x1708
#define MT_RX_STAT_2_DUP_ERRORS      GENMASK(15, 0)
#define MT_RX_STAT_2_OVERFLOW_ERRORS GENMASK(31, 16)
#define MT_TX_STA_0          0x170c
#define MT_TX_STA_1          0x1710
#define MT_TX_STA_2          0x1714
/*
 * Per-MPDU transmit status. mt76x02 pops one entry per read of
 * MT_TX_STAT_FIFO while VALID is set, and the matching retry count and pktid
 * live in the separate EXT register, which mt76 reads FIRST - the main read
 * pops the entry (mt76x02_mac_load_tx_status).
 *
 * The MAC only files an entry for a frame whose txwi pktid is non-zero, which
 * is why MT_TXOPT_TXS exists: the library's normal send path leaves pktid 0
 * and therefore generates no status traffic at all. The bring-up tool's `txs`
 * gate is the one reader (docs/mt7612u-tx-retry.md).
 */
#define MT_TX_STAT_FIFO      0x1718
#define MT_TX_STAT_FIFO_VALID     BIT(0)
#define MT_TX_STAT_FIFO_SUCCESS   BIT(5)
#define MT_TX_STAT_FIFO_AGGR      BIT(6)
#define MT_TX_STAT_FIFO_ACKREQ    BIT(7)
#define MT_TX_STAT_FIFO_WCID      GENMASK(15, 8)
#define MT_TX_STAT_FIFO_RATE      GENMASK(31, 16)
#define MT_TX_STAT_FIFO_EXT  0x1798
#define MT_TX_STAT_FIFO_EXT_RETRY GENMASK(7, 0)
#define MT_TX_STAT_FIFO_EXT_PKTID GENMASK(15, 8)
/* 16 registers, two 16-bit buckets each: the A-MPDU length histogram. */
#define MT_TX_AGG_CNT_BASE0  0x1720
#define MT_TX_AGG_CNT_BASE1  0x174c
#define MT_TX_AGG_CNT(_id)   ((_id) < 8 ? MT_TX_AGG_CNT_BASE0 + ((_id) << 2) \
                                        : MT_TX_AGG_CNT_BASE1 + (((_id) - 8) << 2))
#define MT_TEMP_SENSOR       0x1d000
#define MT_TEMP_SENSOR_VAL   GENMASK(6, 0)
#define MT_BBP_CORE_R1_BW    GENMASK(4, 3)
#define MT_BBP_AGC_R0_BW     GENMASK(14, 12)
#define MT_BBP_AGC_R0_CTRL_CHAN GENMASK(9, 8)
#define MT_BBP_TXBE_R0_CTRL_CHAN GENMASK(1, 0)
#define MT_BBP_AGC_GAIN      GENMASK(14, 8)
#define MT_BBP_AGC_LNA_HIGH_GAIN GENMASK(21, 16)

/* ---- EEPROM ----
 * The MT7612U copy's EEPROM map (mt76x2 TX-power layout, XTAL trim) was
 * removed here; the mt76x0 fields are at the end of this file. */

/* ---- descriptors ---- */
#define MT_TXD_INFO_LEN      GENMASK(15, 0)
#define MT_TXD_INFO_NEXT_VLD BIT(16)
#define MT_TXD_INFO_TX_BURST BIT(17)
#define MT_TXD_INFO_80211    BIT(19)
#define MT_TXD_INFO_WIV      BIT(24)
#define MT_TXD_INFO_QSEL     GENMASK(26, 25)
#define MT_TXD_INFO_DPORT    GENMASK(29, 27)
#define MT_QSEL_MGMT         0
#define MT_QSEL_EDCA         1

#define MT_TXWI_LEN          20
#define MT_RXWI_LEN          32
#define MT_DMA_HDR_LEN       4

#define MT_TXWI_FLAGS_TS           BIT(3)  /* MAC inserts the TSF timestamp (beacon/probe-resp) */
#define MT_TXWI_FLAGS_AMPDU        BIT(4)
#define MT_TXWI_FLAGS_MPDU_DENSITY GENMASK(7, 5)
#define MT_TXWI_ACK_CTL_BA_WINDOW  GENMASK(7, 2)
#define MT_TXWI_FLAGS_TX_RATE_LUT BIT(15)
#define MT_TXWI_ACK_CTL_REQ  BIT(0)
#define MT_TXWI_ACK_CTL_NSEQ BIT(1)
#define MT_TX_PWR_ADJ        GENMASK(3, 0)

/* The 16-bit per-packet rate word, shared by TXWI and RXWI. */
#define MT_RATE_INDEX        GENMASK(5, 0)
#define MT_RATE_LDPC         BIT(6)
#define MT_RATE_BW           GENMASK(8, 7)
#define MT_RATE_SGI          BIT(9)
#define MT_RATE_STBC         BIT(10)
#define MT_RATE_PHY          GENMASK(15, 13)
#define MT_RATE_VHT_IDX      GENMASK(3, 0)
#define MT_RATE_VHT_NSS      GENMASK(5, 4)

#define MT_RXWI_CTL_WCID     GENMASK(7, 0)
#define MT_RXWI_CTL_MPDU_LEN GENMASK(29, 16)
#define MT_RXINFO_CRCERR     BIT(8)
#define MT_RXINFO_RSSI       BIT(13)
#define MT_RXINFO_L2PAD      BIT(14)
#define MT_RXINFO_AMPDU      BIT(15)
#define MT_RXINFO_PN_LEN     GENMASK(21, 19)

/* --- mt76x0-specific (reference/mt76 @ be5ce79) ----------------------------
 * Added by the MT76x0U port rather than inherited from the MT7612U copy. */

/* mt76x02_regs.h:161. On mt76x0 the USB DMA config is this MAC-space
 * register; mt76x2 programs the CFG-space MT_USB_U3DMA_CFG (0x9018) instead.
 * The MT_USB_DMA_CFG_* bit names above apply to both. */
#define MT_USB_DMA_CFG                  0x0238
#define MT_USB_DMA_CFG_UDMA_TX_WL_DROP  BIT(16)

/* mt76x0/mcu.h */
#define MT_MCU_IVB_SIZE        0x40
#define MT_MCU_DLM_OFFSET      0x80000
#define MT_MCU_MEMMAP_RF       0x80000000u
/* mt76x02_mcu.h */
#define MT_MCU_MEMMAP_WLAN     0x410000u
#define MT_INBAND_PACKET_MAX_LEN 192
/* One CMD_RANDOM_* command carries at most this many (address, value)
 * pairs: MT_INBAND_PACKET_MAX_LEN / 8 (mt76x02u_mcu_wr_rp). */
#define MT0_RP_MAX_PER_CMD     (MT_INBAND_PACKET_MAX_LEN / 8)

/* mt76x0/usb_mcu.c */
#define MT0_FW_URB_MAX_PAYLOAD 0x38f8
#define MT0_FW_HDR_LEN         32     /* sizeof(struct mt76x02_fw_header) */
/* mt76x0/mt76x0.h. The kernel requests "mediatek/<name>"; this port looks
 * the bare name up in a firmware directory (DeviceConfig mt76x0u). */
#define MT7610E_FIRMWARE_NAME  "mt7610e.bin"
#define MT7610U_FIRMWARE_NAME  "mt7610u.bin"

/* mt76x0/phy.h: an RF register as the MCU addresses it. */
#define MT_RF(bank, reg)       ((uint32_t)(bank) << 16 | (uint32_t)(reg))

/* mt76x02_regs.h:14-59, 158, 415-416 - the mt76x0 bring-up's registers. */
#define MT_CMB_CTRL                     0x0020
#define MT_CMB_CTRL_XTAL_RDY            BIT(22)
#define MT_CMB_CTRL_PLL_LD              BIT(23)
#define MT_EFUSE_CTRL                   0x0024
#define MT_EFUSE_CTRL_AOUT              GENMASK(5, 0)
#define MT_EFUSE_CTRL_MODE              GENMASK(7, 6)
#define MT_EFUSE_CTRL_AIN               GENMASK(25, 16)
#define MT_EFUSE_CTRL_KICK              BIT(30)
#define MT_EFUSE_CTRL_SEL               BIT(31)
#define MT_EFUSE_DATA(_n)               (0x0028 + ((_n) << 2))
#define MT_COEXCFG3                     0x004c
#define MT_WLAN_FUN_CTRL_WLAN_RESET     BIT(3)          /* MT76x0 */
#define MT_WLAN_FUN_CTRL_GPIO_OUT_EN    GENMASK(31, 24) /* MT76x0 */
#define MT_CSR_EE_CFG1                  0x0104
#define MT_WMM_CTRL                     0x0230          /* MT76x0 */
#define MT_TXOP_TRUN_EN                 GENMASK(5, 0)
#define MT_TXOP_EXT_CCA_DLY             GENMASK(15, 8)
/* Channel set (mt76x02_regs.h:209, 485, 504; mt76x02_dfs.h:23) */
#define MT_RF_MISC                      0x0518
#define MT_RXQ_STA                      0x0430   /* mt76x02_regs.h:196 */
#define MT_TX0_RF_GAIN_ATTEN            0x13a8   /* MT76x0 */
#define MT_TX_ALC_VGA3                  0x13c8
#define MT_DFS_CH_EN                    0xf

/* mt76x02_eeprom.h - the fields the mt76x0 bring-up reads. Word offsets
 * into the 512-byte EFUSE image, read as little-endian u16. */
#define MT76X0_EEPROM_SIZE              512
#define MT76X0U_EE_MAX_VER              0x0c
enum mt0_ee_field {
	MT_EE_VERSION           = 0x002,
	MT_EE_MAC_ADDR          = 0x004,
	MT_EE_ANTENNA           = 0x022,
	MT_EE_CFG1_INIT         = 0x024,
	MT_EE_NIC_CONF_0        = 0x034,
	MT_EE_NIC_CONF_1        = 0x036,
	MT_EE_FREQ_OFFSET       = 0x03a,
	MT_EE_NIC_CONF_2        = 0x042,
	MT_EE_LNA_GAIN          = 0x044,
	MT_EE_RSSI_OFFSET_2G_0  = 0x046,
	MT_EE_RSSI_OFFSET_2G_1  = 0x048,
	MT_EE_RSSI_OFFSET_5G_0  = 0x04a,
	MT_EE_RSSI_OFFSET_5G_1  = 0x04c,
	MT_EE_2G_TARGET_POWER   = 0x0d0,
	MT_EE_TSSI_BOUND4       = 0x0da,
	MT_EE_USAGE_MAP_START   = 0x1e0,
	MT_EE_USAGE_MAP_END     = 0x1fc,
};
#define MT_EFUSE_USAGE_MAP_SIZE         (MT_EE_USAGE_MAP_END - MT_EE_USAGE_MAP_START + 1)
#define MT_EE_ANTENNA_DUAL              BIT(15)
#define MT_EE_NIC_CONF_0_RX_PATH        GENMASK(3, 0)
#define MT_EE_NIC_CONF_0_TX_PATH        GENMASK(7, 4)
#define MT_EE_NIC_CONF_0_PA_INT_2G      BIT(8)
#define MT_EE_NIC_CONF_0_PA_INT_5G      BIT(9)
#define MT_EE_NIC_CONF_0_BOARD_TYPE     GENMASK(13, 12)
#define MT_EE_NIC_CONF_1_HW_RF_CTRL     BIT(0)
#define MT_EE_NIC_CONF_2_ANT_OPT        BIT(3)
#define MT_EE_NIC_CONF_2_ANT_DIV        BIT(4)
enum mt0_ee_mode { MT_EE_READ = 0, MT_EE_PHYSICAL_READ = 1 };
enum mt0_board_type { BOARD_TYPE_2GHZ = 1, BOARD_TYPE_5GHZ = 2 };

#endif /* MT76X0U_REGS_H */
