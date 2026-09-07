/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT7612U register definitions, trimmed from openwrt/mt76 @ be5ce79
 * (mt76x02_regs.h, mt76x2/mcu.h, mt76x02_mcu.h, mt76x02_eeprom.h).
 * Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo Bianconi / Stanislaw Gruszka.
 *
 * Only the registers this HAL actually touches are here. See
 * ../../INVESTIGATION.md for what each init block is for.
 */
#ifndef MT7612U_REGS_H
#define MT7612U_REGS_H

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

/* EP0 vendor requests (mt76.h enum mt_vendor_req) */
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

enum mt_dma_msg_port { WLAN_PORT, CPU_RX_PORT, CPU_TX_PORT, HOST_PORT };

enum mt_mcu_cmd {
	CMD_FUN_SET_OP = 1, CMD_LOAD_CR = 2, CMD_INIT_GAIN_OP = 3,
	CMD_RANDOM_READ = 10, CMD_RANDOM_WRITE = 12,
	CMD_POWER_SAVING_OP = 20, CMD_SWITCH_CHANNEL_OP = 30,
	CMD_CALIBRATION_OP = 31,
};
enum mt_mcu_function { Q_SELECT = 1, BW_SETTING = 2, GET_FW_VERSION = 5 };
enum mt_mcu_power_mode { RADIO_OFF = 0x30, RADIO_ON = 0x31 };
enum mt_mcu_calibration {
	MCU_CAL_R = 1, MCU_CAL_TEMP_SENSOR, MCU_CAL_RXDCOC, MCU_CAL_RC,
	MCU_CAL_SX_LOGEN, MCU_CAL_LC, MCU_CAL_TX_LOFT, MCU_CAL_TXIQ,
	MCU_CAL_TSSI, MCU_CAL_TSSI_COMP, MCU_CAL_DPD, MCU_CAL_RXIQC_FI,
	MCU_CAL_RXIQC_FD, MCU_CAL_PWRON, MCU_CAL_TX_SHAPING,
};
enum mt_mcu_cr_mode { MT_RF_CR, MT_BBP_CR, MT_RF_BBP_CR, MT_HL_TEMP_CR_UPDATE };

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
#define MT_MAC_BSSID_DW1_MBSS_MODE      GENMASK(19, 18)
#define MT_MAC_BSSID_DW1_MBEACON_N      GENMASK(22, 20)
#define MT_MAC_BSSID_DW1_MBSS_LOCAL_BIT BIT(23)
#define MT_MAX_LEN_CFG       0x1018
#define MT_XIFS_TIME_CFG     0x1100
#define MT_XIFS_TIME_CFG_OFDM_SIFS GENMASK(15, 8)
#define MT_BKOFF_SLOT_CFG    0x1104
#define MT_BKOFF_SLOT_CFG_CC_DELAY GENMASK(11, 8)
#define MT_BEACON_TIME_CFG   0x1114
#define MT_BEACON_TIME_CFG_INTVAL   GENMASK(15, 0)
#define MT_BEACON_TIME_CFG_TIMER_EN BIT(16)
#define MT_BEACON_TIME_CFG_TBTT_EN  BIT(19)
#define MT_BEACON_TIME_CFG_BEACON_TX BIT(20)
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
#define MT_RX_FILTR_CFG_DUP       BIT(7)
#define MT_RX_FILTR_CFG_CTRL_RSV  BIT(16)
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
#define MT_EE_NIC_CONF_0_PA_INT_2G BIT(8)
#define MT_EE_NIC_CONF_0_PA_INT_5G BIT(9)

/* ---- EEPROM ---- */
#define MT7612U_EEPROM_SIZE  512
enum mt_ee_field {
	MT_EE_CHIP_ID       = 0x000,
	MT_EE_MAC_ADDR      = 0x004,
	MT_EE_NIC_CONF_0    = 0x034,
	MT_EE_NIC_CONF_1    = 0x036,
	MT_EE_NIC_CONF_2    = 0x042,
	MT_EE_XTAL_TRIM_1   = 0x03a,
	MT_EE_XTAL_TRIM_2   = 0x09e,
	MT_EE_BT_RCAL_RESULT = 0x138,
	/* TX power */
	MT_EE_TX_POWER_DELTA_BW40  = 0x050,
	MT_EE_TX_POWER_DELTA_BW80  = 0x052,
	MT_EE_TX_POWER_0_START_2G  = 0x056,
	MT_EE_TX_POWER_1_START_2G  = 0x05c,
	MT_EE_TX_POWER_0_START_5G  = 0x062,
	MT_EE_TX_POWER_1_START_5G  = 0x080,
	MT_EE_TX_POWER_CCK         = 0x0a0,
	MT_EE_TX_POWER_OFDM_2G_6M  = 0x0a2,
	MT_EE_TX_POWER_OFDM_2G_24M = 0x0a4,
	MT_EE_TX_POWER_HT_MCS0     = 0x0a6,
	MT_EE_TX_POWER_HT_MCS4     = 0x0a8,
	MT_EE_TX_POWER_HT_MCS8     = 0x0aa,
	MT_EE_TX_POWER_HT_MCS12    = 0x0ac,
	MT_EE_TX_POWER_OFDM_5G_6M  = 0x0b2,
	MT_EE_TX_POWER_OFDM_5G_24M = 0x0b4,
	MT_EE_TX_POWER_VHT_MCS8    = 0x0be,
	MT_EE_RF_2G_TSSI_OFF_TXPOWER = 0x0f6,
	MT_EE_RF_2G_RX_HIGH_GAIN     = 0x0f8,
	/* RX gain / RSSI correction */
	MT_EE_LNA_GAIN               = 0x044,
	MT_EE_RSSI_OFFSET_2G_0       = 0x046,
	MT_EE_RSSI_OFFSET_2G_1       = 0x048,
	MT_EE_RSSI_OFFSET_5G_0       = 0x04a,
	MT_EE_RSSI_OFFSET_5G_1       = 0x04c,
	MT_EE_RF_5G_GRP0_1_RX_HIGH_GAIN = 0x0fa,
	MT_EE_RF_5G_GRP2_3_RX_HIGH_GAIN = 0x0fc,
	MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN = 0x0fe,
};
#define MT_TX_POWER_GROUP_SIZE_5G 5
#define MT_EE_NIC_CONF_0_RX_PATH   GENMASK(3, 0)
#define MT_EE_NIC_CONF_0_TX_PATH   GENMASK(7, 4)
#define MT_EE_NIC_CONF_1_TEMP_TX_ALC BIT(1)
#define MT_EE_NIC_CONF_1_LNA_EXT_2G  BIT(2)
#define MT_EE_NIC_CONF_1_LNA_EXT_5G  BIT(3)
#define MT_EE_NIC_CONF_1_TX_ALC_EN   BIT(13)
#define MT_EE_NIC_CONF_2_XTAL_OPTION GENMASK(9, 8)

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

#endif /* MT7612U_REGS_H */
