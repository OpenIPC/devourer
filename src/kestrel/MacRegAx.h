#ifndef KESTREL_MAC_REG_AX_H
#define KESTREL_MAC_REG_AX_H

#include <cstdint>

/* Register + bitfield constants for the Kestrel (Wi-Fi 6 / G6 "phl") MAC,
 * transcribed from the vendor tree reference/rtl8852bu
 * (phl/hal_g6/mac/mac_reg_ax.h). Only the subset the Kestrel bring-up touches
 * is mirrored here; extend as new features need more. Values are verbatim
 * from the vendor headers (file:line noted at each block).
 *
 * NB: this is the AX register map — a different address space from the 11ac
 * hal_com_reg.h the Jaguar HALs use. 0x00FC, for example, is R_AX_SYS_CHIPINFO
 * here (die id), not the 11ac SYS_CFG2 dispatch byte. */

namespace kestrel::reg {

/* ---- identity (mac_ax.c get_chip_info) ---- */
constexpr uint16_t R_AX_SYS_CFG1 = 0x00F0;     /* [15:12] = cut version */
constexpr uint16_t R_AX_SYS_CHIPINFO = 0x00FC; /* D-die id */

/* ---- system power control (pwr_seq_func_8852b.c mac_pwr_on_usb_8852b) ---- */
constexpr uint16_t R_AX_SYS_ISO_CTRL = 0x0000;
constexpr uint32_t B_AX_ISO_EB2CORE = 1u << 8;
constexpr uint8_t B_AX_PWC_EV2EF_SH = 14; /* efuse power-cut bits [15:14] */

constexpr uint16_t R_AX_SYS_PW_CTRL = 0x0004;
constexpr uint32_t B_AX_RDY_SYSPWR = 1u << 17;
constexpr uint32_t B_AX_EN_WLON = 1u << 16;
constexpr uint32_t B_AX_APDM_HPDN = 1u << 15;
constexpr uint32_t B_AX_AFSM_PCIE_SUS_EN = 1u << 12;
constexpr uint32_t B_AX_AFSM_WLSUS_EN = 1u << 11;
constexpr uint32_t B_AX_APFM_SWLPS = 1u << 10;
constexpr uint32_t B_AX_APFM_OFFMAC = 1u << 9;
constexpr uint32_t B_AX_APFN_ONMAC = 1u << 8;
constexpr uint32_t B_AX_DIS_WLBT_PDNSUSEN_SOPC = 1u << 18;
/* mac_pwr_switch on-entry (pwr.c:314): the WLMAC power-FSM state. A USB chip
 * from real cold reads non-OFF here (its AFSM auto-powers to enumerate), and
 * the vendor forces the MAC off (EN_WLON + APFM_OFFMAC poll) before running
 * the power-on sequence — without it the WCPU bootrom comes up FWDL_RDY but
 * never raises H2C_PATH_RDY. */
constexpr uint16_t R_AX_IC_PWR_STATE = 0x03F0;
constexpr uint8_t B_AX_WLMAC_PWR_STE_SH = 8;
constexpr uint32_t B_AX_WLMAC_PWR_STE_MSK = 0x3;
constexpr uint32_t MAC_AX_MAC_OFF = 0;
constexpr uint32_t B_AX_CMAC_CLK_SEL = 1u << 21; /* R_AX_AFE_CTRL1 */
/* mac_pwr_switch BOOT_MODE-exit preamble (pwr.c:300): the RISC-V 8852C
 * enumerates in a special boot mode; if GPIO_MUXCFG[19] is set, the driver must
 * clear it (+ AUTO_WLPON, APFN_ONMAC, R_DIS_PRST) before the power sequence, or
 * the WCPU bootrom auto-boots its ROM fw instead of entering the FWDL H2C-wait
 * (it never raises H2C_PATH_RDY). */
constexpr uint16_t R_AX_RSV_CTRL = 0x001C;
constexpr uint32_t B_AX_R_DIS_PRST = 1u << 6;
constexpr uint16_t R_AX_GPIO_MUXCFG = 0x0040;
constexpr uint32_t B_AX_BOOT_MODE = 1u << 19;
constexpr uint16_t R_AX_SYS_STATUS1 = 0x00F4;
constexpr uint32_t B_AX_AUTO_WLPON = 1u << 10;

constexpr uint16_t R_AX_SYS_ADIE_PAD_PWR_CTRL = 0x0018;
constexpr uint32_t B_AX_SYM_PADPDN_WL_PTA_1P3 = 1u << 6;
constexpr uint32_t B_AX_SYM_PADPDN_WL_RFC_1P3 = 1u << 5;

constexpr uint16_t R_AX_AFE_LDO_CTRL = 0x0020;
constexpr uint32_t B_AX_AON_OFF_PC_EN = 1u << 23;

constexpr uint16_t R_AX_EFUSE_CTRL = 0x0030;
constexpr uint32_t B_AX_EF_RDY = 1u << 29;
constexpr uint8_t B_AX_EF_ADDR_SH = 16;
constexpr uint32_t B_AX_EF_ADDR_MSK = 0x7ff;

constexpr uint16_t R_AX_PLATFORM_ENABLE = 0x0088;
constexpr uint8_t B_AX_PLATFORM_EN = 1u << 0;

constexpr uint16_t R_AX_WLLPS_CTRL = 0x0090;
constexpr uint32_t B_AX_DIS_WLBT_LPSEN_LOPC = 1u << 1;

constexpr uint16_t R_AX_PMC_DBG_CTRL2 = 0x00CC;
constexpr uint32_t B_AX_SYSON_DIS_PMCR_AX_WRMSK = 1u << 2;

/* SPS digital voltage rails (SET_CLR_WORD fields). */
constexpr uint16_t R_AX_SPS_DIG_ON_CTRL0 = 0x0200;
constexpr uint8_t B_AX_VOL_L1_SH = 0;
constexpr uint32_t B_AX_VOL_L1_MSK = 0xf;
constexpr uint8_t B_AX_VREFPFM_L_SH = 22;
constexpr uint32_t B_AX_VREFPFM_L_MSK = 0xf;

/* --- RTL8852C power-on deltas (mac_pwr_on_usb_8852c, pwr_seq_func_8852c.c) --- */
constexpr uint16_t R_AX_LDO_AON_CTRL0 = 0x0218;
constexpr uint32_t B_AX_PD_REGU_L = 1u << 16;
constexpr uint16_t R_AX_SYS_ISO_CTRL_EXTEND = 0x0080;
constexpr uint32_t B_AX_CMAC1_FEN = 1u << 30;
constexpr uint32_t B_AX_R_SYM_ISO_CMAC12PP = 1u << 5;
constexpr uint16_t R_AX_AFE_CTRL1 = 0x0024;
constexpr uint32_t B_AX_WLCMAC1_PC_EN_ALL = 0x1f; /* [4:0] P4..P1 + base */
constexpr uint16_t R_AX_LED1_FUNC_SEL = 0x02DC;
constexpr uint8_t B_AX_PINMUX_EESK_FUNC_SEL_V1_SH = 24;
constexpr uint32_t B_AX_PINMUX_EESK_FUNC_SEL_V1_MSK = 0xf;
constexpr uint8_t XTAL_SI_XTAL_XMD_2_8852C = 0x10; /* [6:4]=001 (vs 0 on 8852B) */

constexpr uint16_t R_AX_SPS_DIG_OFF_CTRL0 = 0x0400;
constexpr uint8_t B_AX_C1_L1_SH = 0;
constexpr uint32_t B_AX_C1_L1_MSK = 0x3;
constexpr uint8_t B_AX_C3_L1_SH = 4;
constexpr uint32_t B_AX_C3_L1_MSK = 0x3;

constexpr uint16_t R_AX_EECS_EESK_FUNC_SEL = 0x02D8;
constexpr uint8_t B_AX_PINMUX_EESK_FUNC_SEL_SH = 4;
constexpr uint32_t B_AX_PINMUX_EESK_FUNC_SEL_MSK = 0xf;

/* ---- XTAL_SI indirect interface (hw.c mac_write_xtal_si) ---- */
constexpr uint16_t R_AX_WLAN_XTAL_SI_CTRL = 0x0270;
constexpr uint32_t B_AX_WL_XTAL_SI_CMD_POLL = 1u << 31;
constexpr uint8_t B_AX_WL_XTAL_SI_MODE_SH = 24;
constexpr uint32_t B_AX_WL_XTAL_SI_MODE_MSK = 0x3;
constexpr uint8_t B_AX_WL_XTAL_SI_BITMASK_SH = 16;
constexpr uint32_t B_AX_WL_XTAL_SI_BITMASK_MSK = 0xff;
constexpr uint8_t B_AX_WL_XTAL_SI_DATA_SH = 8;
constexpr uint32_t B_AX_WL_XTAL_SI_DATA_MSK = 0xff;
constexpr uint8_t B_AX_WL_XTAL_SI_ADDR_SH = 0;
constexpr uint32_t B_AX_WL_XTAL_SI_ADDR_MSK = 0xff;
constexpr uint8_t XTAL_SI_NORMAL_WRITE = 0x00;
constexpr uint8_t XTAL_SI_NORMAL_READ = 0x01;
/* XTAL_SI sub-addresses (pwr_seq_func_8852b.c comments). */
constexpr uint8_t XTAL_SI_ANAPAR_WL = 0x90;
constexpr uint8_t XTAL_SI_SRAM_CTRL = 0xA1;
constexpr uint8_t XTAL_SI_XTAL_XMD_2 = 0x24;
constexpr uint8_t XTAL_SI_XTAL_XMD_4 = 0x26;

/* ---- USB interface pre-init (_usb_8852b.c usb_pre_init_8852b; addresses
 * from hci_reg_ax.h, verified against source) ---- */
constexpr uint16_t R_AX_USB_HOST_REQUEST_2 = 0x1078;
constexpr uint32_t B_AX_R_USBIO_MODE = 1u << 4;
constexpr uint16_t R_AX_USB_WLAN0_1 = 0x1174;
constexpr uint32_t B_AX_USBRX_RST = 1u << 9;
constexpr uint32_t B_AX_USBTX_RST = 1u << 8;
/* 8852C USB (_V1 register space, hci_reg_ax.h): usb_pre_init_8852c writes these
 * instead of the 8852B addresses above. Same bit positions, different offsets.
 * Without them the 8852C USB TRX path stays in reset and the RISC-V bootrom
 * never raises H2C_PATH_RDY (it self-boots its ROM fw). B_AX_R_USBIO_MODE /
 * USBRX_RST / USBTX_RST bit numbers are shared with the 8852B (BIT4/9/8). */
constexpr uint16_t R_AX_USB_HOST_REQUEST_2_V1 = 0x5078;
constexpr uint16_t R_AX_USB_WLAN0_1_V1 = 0x5174;
constexpr uint16_t R_AX_HCI_FUNC_EN = 0x8380;    /* 8852B/MIPS */
constexpr uint16_t R_AX_HCI_FUNC_EN_V1 = 0x7880; /* 8852C/RISC-V (init.c else) */

/* --- 8852C HAXI (Host AXI DMA) USB transport (dmac_func_pre_en_8852c) — the
 * 8852C routes USB DMA through the HAXI block; without it the H2C/fw transport
 * (and thus the WCPU boot handshake) never comes up. Absent on the 8852B. --- */
constexpr uint32_t B_AX_H_AXIDMA_EN = 1u << 14; /* DMAC_FUNC_EN (0x8400) */
constexpr uint32_t B_AX_AXIDMA_EN = 1u << 3;    /* PLATFORM_ENABLE (0x88) */
constexpr uint16_t R_AX_HAXI_INIT_CFG1 = 0x1000;
constexpr uint8_t B_AX_DMA_MODE_SH = 18;
constexpr uint32_t B_AX_DMA_MODE_MSK = 0x3;
constexpr uint32_t DMA_MOD_USB = 0x2;
constexpr uint32_t B_AX_STOP_AXI_MST = 1u << 17;
constexpr uint32_t B_AX_TXHCI_EN_V1 = 1u << 7;
constexpr uint32_t B_AX_RXHCI_EN_V1 = 1u << 15;
constexpr uint16_t R_AX_HAXI_DMA_STOP1 = 0x1010;
/* ACH0..ACH7 = BIT8..15, CH8=16, CH9=17, CH12=18. */
constexpr uint32_t HAXI_DMA_STOP1_CHANS = 0x7FF00u;
constexpr uint16_t R_AX_HAXI_DMA_STOP2 = 0x11C0;
constexpr uint32_t HAXI_DMA_STOP2_CHANS = 0x3u; /* CH10=0, CH11=1 */
constexpr uint32_t B_AX_HCI_RXDMA_EN = 1u << 1;
constexpr uint32_t B_AX_HCI_TXDMA_EN = 1u << 0;

/* ---- poll / efuse timing (pwr.h, hw.h, efuse.h) ---- */
constexpr uint32_t PWR_POLL_CNT = 2000;
constexpr uint32_t PWR_POLL_DLY_US = 1000;
constexpr uint32_t XTAL_SI_POLLING_CNT = 1000;
constexpr uint32_t XTAL_SI_POLLING_DLY_US = 50;
constexpr uint32_t EFUSE_WAIT_CNT_PLUS = 30000; /* 8852B extended */
constexpr uint8_t UNLOCK_CODE = 0x69;

/* ---- efuse geometry (hw_info_ax.h, efuse.c offset_usb_8852b) ---- */
constexpr uint32_t WL_EFUSE_PHYS_SIZE_8852B = 1536; /* physical dump bytes */
constexpr uint32_t WL_EFUSE_LOG_MAP_SIZE_8852B = 2048; /* logical map bytes */
constexpr uint32_t WL_SEC_CTRL_EFUSE_SIZE_8852B = 4;   /* header skip */
constexpr uint16_t EFUSE_USB_MAC_ADDR_8852B = 0x488;   /* logical offset */
/* RF calibration bases in the logical map (halrf_efuse_8852b.h). */
constexpr uint16_t EFUSE_RF_XTAL_8852B = 0x2B9;
constexpr uint16_t EFUSE_RF_RFE_8852B = 0x2CA;
constexpr uint16_t EFUSE_RF_THERMAL_A_8852B = 0x2D0;
constexpr uint16_t EFUSE_RF_THERMAL_B_8852B = 0x2D1;
/* Power-calibration-done marker read during power-on (physical offset). */
constexpr uint32_t PWR_K_CHK_OFFSET = 0x5E9;
constexpr uint8_t PWR_K_CHK_VALUE = 0xAA;

/* ---- DMAC / DLE (dle.c, dle_8852b.c, init_8852b.c) ---- */
constexpr uint16_t R_AX_DMAC_FUNC_EN = 0x8400;
constexpr uint32_t B_AX_MAC_FUNC_EN = 1u << 30;
constexpr uint32_t B_AX_DMAC_FUNC_EN = 1u << 29;
constexpr uint32_t B_AX_DLE_WDE_EN = 1u << 26;
constexpr uint32_t B_AX_DLE_PLE_EN = 1u << 23;
constexpr uint32_t B_AX_PKT_BUF_EN = 1u << 22;
constexpr uint32_t B_AX_DISPATCHER_EN = 1u << 18;

constexpr uint16_t R_AX_WDE_PKTBUF_CFG = 0x8C08;
constexpr uint16_t R_AX_PLE_PKTBUF_CFG = 0x9008;
constexpr uint8_t B_AX_WDE_PAGE_SEL_SH = 0;
constexpr uint32_t B_AX_WDE_PAGE_SEL_MSK = 0x3;
constexpr uint8_t B_AX_WDE_START_BOUND_SH = 8;
constexpr uint32_t B_AX_WDE_START_BOUND_MSK = 0x3f;
constexpr uint8_t B_AX_WDE_FREE_PAGE_NUM_SH = 16;
constexpr uint32_t B_AX_WDE_FREE_PAGE_NUM_MSK = 0x1fff;
constexpr uint8_t B_AX_PLE_PAGE_SEL_SH = 0;
constexpr uint32_t B_AX_PLE_PAGE_SEL_MSK = 0x3;
constexpr uint8_t B_AX_PLE_START_BOUND_SH = 8;
constexpr uint32_t B_AX_PLE_START_BOUND_MSK = 0x3f;
constexpr uint8_t B_AX_PLE_FREE_PAGE_NUM_SH = 16;
constexpr uint32_t B_AX_PLE_FREE_PAGE_NUM_MSK = 0x1fff;
constexpr uint8_t S_AX_WDE_PAGE_SEL_64 = 0;
constexpr uint8_t S_AX_PLE_PAGE_SEL_128 = 1;
constexpr uint32_t DLE_BOUND_UNIT = 8 * 1024;

/* QTA cfg registers: min in [11:0], max in [23:12]... actually MAX_SIZE_SH=16
 * (min [11:0], max [27:16], each mask 0xfff). */
constexpr uint8_t QTA_MIN_SH = 0;
constexpr uint8_t QTA_MAX_SH = 16;
constexpr uint32_t QTA_SIZE_MSK = 0xfff;
constexpr uint16_t R_AX_WDE_QTA0_CFG = 0x8C40; /* hif */
constexpr uint16_t R_AX_WDE_QTA1_CFG = 0x8C44; /* wcpu */
constexpr uint16_t R_AX_WDE_QTA3_CFG = 0x8C4C; /* pkt_in */
constexpr uint16_t R_AX_WDE_QTA4_CFG = 0x8C50; /* cpu_io */
constexpr uint16_t R_AX_PLE_QTA0_CFG = 0x9040;
constexpr uint16_t R_AX_PLE_QTA2_CFG = 0x9048; /* c2h */
constexpr uint16_t R_AX_PLE_QTA3_CFG = 0x904C; /* h2c */
constexpr uint16_t R_AX_PLE_QTA10_CFG = 0x9068; /* highest PLE QTA (8852B) */
constexpr uint16_t R_AX_PLE_QTA11_CFG = 0x906C; /* 8852C writes through QTA11 */

constexpr uint16_t R_AX_WDE_INI_STATUS = 0x8D00;
constexpr uint16_t R_AX_PLE_INI_STATUS = 0x9100;
constexpr uint32_t B_AX_WDE_Q_MGN_INI_RDY = 1u << 1;
constexpr uint32_t B_AX_WDE_BUF_MGN_INI_RDY = 1u << 0;
constexpr uint32_t B_AX_PLE_Q_MGN_INI_RDY = 1u << 1;
constexpr uint32_t B_AX_PLE_BUF_MGN_INI_RDY = 1u << 0;
constexpr uint32_t DLE_WAIT_CNT = 2000;
constexpr uint32_t DLE_WAIT_US = 1;

/* WD-release / TX-report (set_host_rpr, hw.c:1710) — how the fw releases the
 * WD/PLE pages after a frame is transmitted. WITHOUT this, RLSRPT0 sits at its
 * reset default and the fw batches page releases badly, so the mgmt bulk-OUT
 * pool fills (~103 frames at 210 turbo pages / 2) and NAKs. The STF (USB) cfg:
 * WDRLS_MODE=STF, clear the RLSRPT0 filter map, AGGNUM=121 + TO=255 (the
 * timeout is what actually releases pages before the pool caps). */
constexpr uint16_t R_AX_WDRLS_CFG = 0x9408;
constexpr uint8_t B_AX_WDRLS_MODE_SH = 0;
constexpr uint32_t B_AX_WDRLS_MODE_MSK = 0x3;
constexpr uint16_t R_AX_RLSRPT0_CFG0 = 0x9410;
constexpr uint32_t B_WDRLS_FLTR_TXOK = 1u << 24;
constexpr uint32_t B_WDRLS_FLTR_RTYLMT = 1u << 25;
constexpr uint32_t B_WDRLS_FLTR_LIFTIM = 1u << 26;
constexpr uint32_t B_WDRLS_FLTR_MACID = 1u << 27;
constexpr uint16_t R_AX_RLSRPT0_CFG1 = 0x9414;
constexpr uint8_t B_AX_RLSRPT0_TO_SH = 16;
constexpr uint32_t B_AX_RLSRPT0_TO_MSK = 0xff;
constexpr uint8_t B_AX_RLSRPT0_AGGNUM_SH = 0;
constexpr uint32_t B_AX_RLSRPT0_AGGNUM_MSK = 0xff;
constexpr uint8_t MAC_AX_RPR_MODE_STF = 1; /* rpr_cfg_stf */
constexpr uint8_t RPR_STF_AGG = 121;       /* rpr_cfg_stf.agg */
constexpr uint8_t RPR_STF_TMR = 255;       /* rpr_cfg_stf.tmr */

/* CMAC port config (mport.c mac_port_init) — the BSS/PTCL TX context. Without
 * an ENABLED port the CMAC accepts frames into the queue but never airs them
 * (the mgmt bulk-OUT stalls at ~103). Band-0 port-0 register + bits
 * (mac_reg_ax.h R_AX_PORT_CFG_P0 block); the port-enable bit is what
 * mac_port_init (mport.c) sets to gate the transmit engine. */
constexpr uint16_t R_AX_PORT_CFG_P0 = 0xC400;
/* Per-port free-running MAC TSF timer (mac_get_tsf, twt.c). Band-0 port-0. */
constexpr uint16_t R_AX_TSFTR_LOW_P0 = 0xC438;
constexpr uint16_t R_AX_TSFTR_HIGH_P0 = 0xC43C;
constexpr uint32_t B_AX_BCNTX_EN_P0 = 1u << 12;   /* TX_SW (beacon TX) */
constexpr uint8_t B_AX_NET_TYPE_P0_SH = 10;       /* [11:10] */
constexpr uint32_t B_AX_NET_TYPE_P0_MSK = 0x3;
constexpr uint32_t B_AX_RX_BSSID_FIT_EN_P0 = 1u << 4; /* RX_SW */
constexpr uint32_t B_AX_TSF_UDT_EN_P0 = 1u << 3;      /* RX_SYNC */
constexpr uint32_t B_AX_PORT_FUNC_EN_P0 = 1u << 2;    /* b_en_l[0] — port enable */
constexpr uint32_t B_AX_TXBCN_RPT_EN_P0 = 1u << 1;
constexpr uint32_t B_AX_RXBCN_RPT_EN_P0 = 1u << 0;
/* mac_ax_net_type. */
constexpr uint8_t MAC_AX_NET_TYPE_NO_LINK = 0;
constexpr uint8_t MAC_AX_NET_TYPE_ADHOC = 1;
constexpr uint8_t MAC_AX_NET_TYPE_INFRA = 2;
constexpr uint8_t MAC_AX_NET_TYPE_AP = 3;

/* --- AP beacon-engine port timing (mac_port_init AP branch, mport.c). All
 * band-0 port-0, verified against mac_reg_ax.h. Enables a HW-timed beacon at
 * TBTT (bcn_interval TU). --- */
constexpr uint32_t B_AX_TBTT_PROHIB_EN_P0 = 1u << 13; /* PORT_CFG_P0 BCN_PRCT */
constexpr uint32_t B_AX_BRK_SETUP_P0 = 1u << 16;      /* PORT_CFG_P0 BCN_PRCT */
constexpr uint16_t R_AX_TBTT_PROHIB_P0 = 0xC404;
constexpr uint8_t B_AX_TBTT_SETUP_P0_SH = 0; /* BCN_SETUP */
constexpr uint32_t B_AX_TBTT_SETUP_P0_MSK = 0xff;
constexpr uint8_t B_AX_TBTT_HOLD_P0_SH = 16; /* BCN_HOLD_TIME */
constexpr uint32_t B_AX_TBTT_HOLD_P0_MSK = 0xfff;
constexpr uint16_t R_AX_BCN_AREA_P0 = 0xC408;
constexpr uint8_t B_AX_BCN_MSK_AREA_P0_SH = 16;
constexpr uint32_t B_AX_BCN_MSK_AREA_P0_MSK = 0xfff;
constexpr uint16_t R_AX_BCNERLYINT_CFG_P0 = 0xC40C;
constexpr uint8_t B_AX_BCNERLY_P0_SH = 0;
constexpr uint32_t B_AX_BCNERLY_P0_MSK = 0xfff;
constexpr uint16_t R_AX_TBTTERLYINT_CFG_P0 = 0xC40E;
constexpr uint8_t B_AX_TBTTERLY_P0_SH = 0;
constexpr uint32_t B_AX_TBTTERLY_P0_MSK = 0xfff;
constexpr uint16_t R_AX_TBTT_AGG_P0 = 0xC412;
constexpr uint8_t B_AX_TBTT_AGG_NUM_P0_SH = 8;
constexpr uint32_t B_AX_TBTT_AGG_NUM_P0_MSK = 0xff;
constexpr uint16_t R_AX_BCN_SPACE_CFG_P0 = 0xC414;
constexpr uint8_t B_AX_BCN_SPACE_P0_SH = 0;
constexpr uint32_t B_AX_BCN_SPACE_P0_MSK = 0xffff;
constexpr uint16_t R_AX_PTCL_BSS_COLOR_0 = 0xC6A0;
constexpr uint8_t B_AX_BSS_COLOR_P0_SH = 0;
constexpr uint32_t B_AX_BSS_COLOR_P0_MSK = 0x3f;
/* mport.h beacon-timing defaults. */
constexpr uint32_t BCN_ERLY_DEF = 160;
constexpr uint32_t BCN_SETUP_DEF = 4;
constexpr uint32_t BCN_HOLD_DEF = 400;
constexpr uint32_t BCN_MASK_DEF = 0;
constexpr uint32_t TBTT_ERLY_DEF = 5;
constexpr uint32_t TBTT_AGG_DEF = 1;

/* BCN_UPD_V1 H2C (mac_send_bcn_h2c, beacon.c): CL_FR_EXCHG(5)/FUNC 0xf. Beacon
 * body follows a 9-dword (36-byte) header. */
constexpr uint8_t FWCMD_H2C_FUNC_BCN_UPD_V1 = 0xf;
constexpr uint32_t BCN_UPD_V1_HDR_LEN = 36;
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_PORT_SH = 0;      /* dw0 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_PORT_MSK = 0xff;
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_BAND_SH = 16;     /* dw0 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_BAND_MSK = 0xff;
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_MACID_SH = 0;     /* dw1 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_MACID_MSK = 0xff;
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_RATE_SH = 12;     /* dw1 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_RATE_MSK = 0x1ff;
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_ECSA_SUPPORT = 1u << 24; /* dw1 */
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_NTX_PATH_EN_SH = 1; /* dw2 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_NTX_PATH_EN_MSK = 0xf;
constexpr uint8_t FWCMD_H2C_BCN_UPD_V1_PATH_MAP_A_SH = 5;  /* dw2 */
constexpr uint32_t FWCMD_H2C_BCN_UPD_V1_PATH_MAP_A_MSK = 0x3;

/* DLFW quota values (dle_mem_usb3_8852b DLFW entry; SCC wcpu override = 6). */
constexpr uint16_t DLFW_WDE_FREE_PAGE = 0;   /* wde_size9 lnk_pge_num */
constexpr uint16_t DLFW_WDE_UNLNK_PAGE = 1024; /* wde_size9 unlnk_pge_num */
constexpr uint16_t DLFW_PLE_FREE_PAGE = 64;  /* ple_size8 lnk_pge_num */
constexpr uint16_t DLFW_WDE_WCPU_MIN = 6;    /* wde_qt25.wcpu (SCC override) */
constexpr uint16_t DLFW_PLE_C2H = 16;        /* ple_qt13.c2h */
constexpr uint16_t DLFW_PLE_H2C = 48;        /* ple_qt13.h2c */

/* 8852C DLFW quota (dle_mem_usb*_8852c DLFW: wde_size18/ple_size18/wde_qt17/
 * ple_qt44 min/ple_qt45 max). The RISC-V bootrom will not raise H2C_PATH_RDY
 * with the 8852B quotas — its H2C PLE reserve is far larger (256 vs 48) and its
 * WDE holds no wcpu pages. USB DLFW HFC H2C pre-cost stays 32 (hfc_preccfg_usb_
 * 8852c), so only these DLE values diverge. */
constexpr uint16_t DLFW_WDE_UNLNK_PAGE_8852C = 2048; /* wde_size18 unlnk */
constexpr uint16_t DLFW_PLE_FREE_PAGE_8852C = 2544;  /* ple_size18 lnk */
constexpr uint16_t DLFW_WDE_WCPU_MIN_8852C = 0;      /* wde_qt17.wcpu */
constexpr uint16_t DLFW_PLE_C2H_MIN_8852C = 16;      /* ple_qt44.c2h */
constexpr uint16_t DLFW_PLE_C2H_MAX_8852C = 32;      /* ple_qt45.c2h */
constexpr uint16_t DLFW_PLE_H2C_8852C = 256;         /* ple_qt44/45.h2c */

/* ---- HFC (hci_fc.c, hci_fc_8852b.c) ---- */
constexpr uint16_t R_AX_HCI_FC_CTRL = 0x8A00;
constexpr uint32_t B_AX_HCI_FC_EN = 1u << 0;
constexpr uint32_t B_AX_HCI_FC_CH12_EN = 1u << 3;
constexpr uint8_t B_AX_HCI_FC_CH12_FULL_COND_SH = 10;
constexpr uint32_t B_AX_HCI_FC_CH12_FULL_COND_MSK = 0x3;
constexpr uint16_t R_AX_CH_PAGE_CTRL = 0x8A04;
constexpr uint8_t B_AX_PREC_PAGE_CH12_SH = 16;
constexpr uint32_t B_AX_PREC_PAGE_CH12_MSK = 0x1ff;
constexpr uint16_t HFC_USB_H2C_PREC_8852B = 32;   /* hfc_preccfg_usb_8852b */
constexpr uint8_t HFC_FULL_COND_X2 = 1;
/* 8852C HFC flow-control lives in the _V1 register bank (same bit layout as the
 * 8852B FC_CTRL/CH_PAGE_CTRL). hfc_init_dlfw must target these on the 8852C —
 * configuring the H2C (CH12) flow control on the 8852B addresses leaves the
 * 8852C's H2C path unconfigured so the RISC-V bootrom never raises
 * H2C_PATH_RDY — the 8852C hfc ops target the _V1 bank at 0x1700/0x1704,
 * not 0x8A00/0x8A04 (hci_fc_8852c.c). */
constexpr uint16_t R_AX_HCI_FC_CTRL_V1 = 0x1700;
constexpr uint16_t R_AX_CH_PAGE_CTRL_V1 = 0x1704;

/* ---- NIC-mode HFC (hfc_init(rst,en,h2c_en); usb_scc_8852b quotas) ----
 * Runtime page/credit programming: without it the FW only has the tiny DLFW
 * h2c credit and CH12 IO-offload stalls after one 512B page. */
constexpr uint16_t R_AX_ACH0_PAGE_CTRL = 0x8A10;
constexpr uint16_t R_AX_ACH1_PAGE_CTRL = 0x8A14;
constexpr uint16_t R_AX_ACH2_PAGE_CTRL = 0x8A18;
constexpr uint16_t R_AX_ACH3_PAGE_CTRL = 0x8A1C;
constexpr uint16_t R_AX_CH8_PAGE_CTRL = 0x8A30; /* B0MGQ */
constexpr uint16_t R_AX_CH9_PAGE_CTRL = 0x8A34; /* B0HIQ */
constexpr uint8_t B_AX_ACH_MIN_PG_SH = 0;
constexpr uint32_t B_AX_ACH_MIN_PG_MSK = 0x1fff;
constexpr uint8_t B_AX_ACH_MAX_PG_SH = 16;
constexpr uint32_t B_AX_ACH_MAX_PG_MSK = 0x1fff;
constexpr uint32_t B_AX_ACH_GRP = 1u << 31;
constexpr uint16_t R_AX_PUB_PAGE_CTRL1 = 0x8A90;
constexpr uint8_t B_AX_PUBPG_G0_SH = 0;
constexpr uint32_t B_AX_PUBPG_G0_MSK = 0x1fff;
constexpr uint8_t B_AX_PUBPG_G1_SH = 16;
constexpr uint32_t B_AX_PUBPG_G1_MSK = 0x1fff;
constexpr uint16_t R_AX_PUB_PAGE_CTRL2 = 0x8A94;
constexpr uint8_t B_AX_PUBPG_ALL_SH = 0;
constexpr uint32_t B_AX_PUBPG_ALL_MSK = 0x1fff;
constexpr uint16_t R_AX_WP_PAGE_CTRL1 = 0x8AA0;
constexpr uint8_t B_AX_PREC_PAGE_WP_CH07_SH = 0;
constexpr uint32_t B_AX_PREC_PAGE_WP_CH07_MSK = 0x1ff;
constexpr uint8_t B_AX_PREC_PAGE_WP_CH811_SH = 16;
constexpr uint32_t B_AX_PREC_PAGE_WP_CH811_MSK = 0x1ff;
constexpr uint16_t R_AX_WP_PAGE_CTRL2 = 0x8AA4;
constexpr uint8_t B_AX_WP_THRD_SH = 0;
constexpr uint32_t B_AX_WP_THRD_MSK = 0x1fff;
constexpr uint8_t B_AX_PREC_PAGE_CH011_SH = 0;
constexpr uint32_t B_AX_PREC_PAGE_CH011_MSK = 0x1ff;
constexpr uint8_t B_AX_HCI_FC_MODE_SH = 1;
constexpr uint32_t B_AX_HCI_FC_MODE_MSK = 0x3;
constexpr uint8_t B_AX_HCI_FC_WD_FULL_COND_SH = 4;
constexpr uint32_t B_AX_HCI_FC_WD_FULL_COND_MSK = 0x3;
constexpr uint8_t B_AX_HCI_FC_WP_CH07_FULL_COND_SH = 6;
constexpr uint32_t B_AX_HCI_FC_WP_CH07_FULL_COND_MSK = 0x3;
constexpr uint8_t B_AX_HCI_FC_WP_CH811_FULL_COND_SH = 8;
constexpr uint32_t B_AX_HCI_FC_WP_CH811_FULL_COND_MSK = 0x3;
/* hfc_chcfg_usb_scc_turbo_8852b {min,max,grp0}: ACH0-3 + B0MG + B0HI
 * (hci_fc.c:1073 — the TURBO family, matching the SCC_TURBO DLE quota). */
constexpr uint16_t HFC_NIC_CH_MIN = 18;
constexpr uint16_t HFC_NIC_CH_MAX = 210;
/* hfc_pubcfg_usb_scc_turbo_8852b (hci_fc.c:1199). */
constexpr uint16_t HFC_NIC_PUB_G0 = 210;
constexpr uint16_t HFC_NIC_PUB_G1 = 0;
constexpr uint16_t HFC_NIC_PUB_MAX = 210;
constexpr uint16_t HFC_NIC_WP_THRD = 0;
/* hfc_preccfg_usb_8852b. */
constexpr uint16_t HFC_NIC_CH011_PREC = 9;
constexpr uint16_t HFC_NIC_WP_CH07_PREC = 64;
constexpr uint16_t HFC_NIC_WP_CH811_PREC = 24;

/* 8852C runtime (NIC) HFC lives in the _V1 register bank (mirrors the 8852B
 * 0x8A.. block at 0x17..) with the same field layout. The runtime CH12 credit
 * programming must target it or the 8852C's H2C (CH12) bulk-out gets no credits
 * and every post-FWDL cmd_ofld/IO H2C times out (rc=-7). */
constexpr uint16_t R_AX_HCI_FC_CTRL_V1_NIC = 0x1700; /* == R_AX_HCI_FC_CTRL_V1 */
constexpr uint16_t R_AX_ACH0_PAGE_CTRL_V1 = 0x1710;
constexpr uint16_t R_AX_ACH1_PAGE_CTRL_V1 = 0x1714;
constexpr uint16_t R_AX_ACH2_PAGE_CTRL_V1 = 0x1718;
constexpr uint16_t R_AX_ACH3_PAGE_CTRL_V1 = 0x171C;
constexpr uint16_t R_AX_CH8_PAGE_CTRL_V1 = 0x1730;
constexpr uint16_t R_AX_CH9_PAGE_CTRL_V1 = 0x1734;
constexpr uint16_t R_AX_PUB_PAGE_CTRL1_V1 = 0x1790;
constexpr uint16_t R_AX_PUB_PAGE_CTRL2_V1 = 0x1794;
constexpr uint16_t R_AX_WP_PAGE_CTRL1_V1 = 0x17A0;
constexpr uint16_t R_AX_WP_PAGE_CTRL2_V1 = 0x17A4;
/* 8852C USB SCC HFC quotas: hfc_chcfg_usb_scc_8852c {24,412}, hfc_pubcfg_usb_
 * scc_8852c {412,0,412,0}, hfc_preccfg_usb_8852c (CH011=9,H2C=32,WP07/811=148). */
constexpr uint16_t HFC_NIC_CH_MIN_8852C = 24;
constexpr uint16_t HFC_NIC_CH_MAX_8852C = 412;
constexpr uint16_t HFC_NIC_PUB_G0_8852C = 412;
constexpr uint16_t HFC_NIC_PUB_MAX_8852C = 412;
constexpr uint16_t HFC_NIC_WP_CH07_PREC_8852C = 148;
constexpr uint16_t HFC_NIC_WP_CH811_PREC_8852C = 148;

/* ---- FWDL (fwdl.c, mac_reg_ax.h) ---- */
constexpr uint16_t R_AX_SYS_CLK_CTRL = 0x0008;
constexpr uint32_t B_AX_CPU_CLK_EN = 1u << 14;
constexpr uint32_t B_AX_APB_WRAP_EN = 1u << 2; /* R_AX_PLATFORM_ENABLE */
constexpr uint32_t B_AX_WCPU_EN = 1u << 1;     /* R_AX_PLATFORM_ENABLE */
constexpr uint16_t R_AX_HALT_H2C_CTRL = 0x0160;
constexpr uint16_t R_AX_HALT_C2H_CTRL = 0x0164;
constexpr uint16_t R_AX_HALT_H2C = 0x0168;
constexpr uint16_t R_AX_HALT_C2H = 0x016C;
constexpr uint16_t R_AX_HISR0 = 0x01A4;
constexpr uint16_t R_AX_WCPU_FW_CTRL = 0x01E0;
/* RISC-V (8852C) bootrom debug: [31:16]=boot_status (fwdl-step code),
 * [15:0]=secureboot_status. The MIPS R_AX_BOOT_DBG is 0x83F0. */
constexpr uint16_t R_AX_BOOT_DBG_V1 = 0x78F0;
constexpr uint8_t B_AX_WCPU_FWDL_STS_SH = 5;
constexpr uint32_t B_AX_WCPU_FWDL_STS_MSK = 0x7;
constexpr uint32_t B_AX_FWDL_PATH_RDY = 1u << 2;
constexpr uint32_t B_AX_H2C_PATH_RDY = 1u << 1;
constexpr uint32_t B_AX_WCPU_FWDL_EN = 1u << 0;
/* mac_set_dut_env_mode (init.c:26): WCPU_FW_CTRL[29:28] = env (0 = normal). */
constexpr uint8_t B_AX_FW_ENV_SH = 28;
constexpr uint32_t B_AX_FW_ENV_MSK = 0x3;
/* WDT block right before mac_enable_cpu (mac_hal_init): clear the WDT-wake
 * enables, clear WDT_PLT_RST_EN (fwdl_info->wdt_plt_rst_en == 0). */
constexpr uint32_t B_AX_WDT_PLT_RST_EN = 1u << 16; /* WCPU_FW_CTRL */
constexpr uint16_t R_AX_SYS_CFG5 = 0x0170;
constexpr uint32_t B_AX_WDT_WAKE_PCIE_EN = 1u << 10;
constexpr uint32_t B_AX_WDT_WAKE_USB_EN = 1u << 9;
constexpr uint16_t R_AX_BOOT_REASON = 0x01E6;
constexpr uint8_t B_AX_BOOT_REASON_SH = 0;
constexpr uint32_t B_AX_BOOT_REASON_MSK = 0x7;
constexpr uint16_t R_AX_LDM = 0x01E8;
constexpr uint16_t R_AX_UDM0 = 0x01F0;
constexpr uint16_t R_AX_UDM1 = 0x01F4;
constexpr uint16_t R_AX_BOOT_DBG = 0x83F0; /* MIPS boot-status debug */
constexpr uint8_t B_AX_BOOT_STATUS_SH = 16;
/* fw_dl_status enum (fwdl.h). */
constexpr uint8_t FWDL_INITIAL_STATE = 0;
constexpr uint8_t FWDL_WCPU_FW_INIT_RDY = 7;
constexpr uint8_t FWDL_CHECKSUM_FAIL = 2;
constexpr uint8_t FWDL_SECURITY_FAIL = 3;
constexpr uint8_t FWDL_CUT_NOT_MATCH = 4;
constexpr uint32_t FWDL_WAIT_CNT = 400000; /* 1us cadence */
constexpr uint32_t FWDL_SECTION_PER_PKT_LEN = 2020;
constexpr uint32_t FWDL_SECTION_CHKSUM_LEN = 8;
constexpr uint8_t FWDL_SECURITY_SECTION_TYPE = 9;
constexpr uint32_t FWDL_SECURITY_SIGLEN = 512; /* per-key MSS signature bytes */
constexpr uint8_t FWDL_TRY_CNT_MIPS = 3;
constexpr uint8_t AX_BOOT_REASON_PWR_ON = 0;

/* Firmware header/section layout (fwdl.h struct fwhdr_hdr_t / fwhdr_section_t). */
constexpr uint32_t FWHDR_HDR_LEN = 32;     /* sizeof fwhdr_hdr_t (8 dwords) */
constexpr uint32_t FWHDR_SECTION_LEN = 16; /* sizeof fwhdr_section_t (4 dw) */
/* dword6: sec_num [15:8]; dword3: fwhdr_sz [23:16]; dword7: fw_dyn_hdr [16]. */
constexpr uint8_t FWHDR_SEC_NUM_SH = 8;
constexpr uint32_t FWHDR_SEC_NUM_MSK = 0xff;
constexpr uint8_t FWHDR_FWHDR_SZ_SH = 16;
constexpr uint32_t FWHDR_FWHDR_SZ_MSK = 0xff;
constexpr uint8_t FWHDR_FW_DYN_HDR_SH = 16;
constexpr uint32_t FWHDR_FW_DYN_HDR_MSK = 0x1;
/* section dword1: sec_size [23:0], sectiontype [27:24], checksum [28], redl [29]. */
constexpr uint8_t SECTION_INFO_SEC_SIZE_SH = 0;
constexpr uint32_t SECTION_INFO_SEC_SIZE_MSK = 0xffffff;
constexpr uint8_t SECTION_INFO_SECTIONTYPE_SH = 24;
constexpr uint32_t SECTION_INFO_SECTIONTYPE_MSK = 0xf;
constexpr uint32_t SECTION_INFO_CHECKSUM = 1u << 28;
constexpr uint32_t SECTION_INFO_REDL = 1u << 29;

/* H2C/fwcmd envelope (fwcmd.h, fwcmd_intf.h, type.h, txdesc.h). */
constexpr uint32_t FWCMD_HDR_LEN = 8;
constexpr uint32_t WD_BODY_LEN = 24; /* sizeof wd_body_t (6 dwords), 8852B */
/* 8852C FWDL/H2C descriptor: a 16-byte rxd_short_t (mac_txdesc_len_8852c returns
 * RXD_SHORT_LEN for H2C/FWDL), NOT the 24-byte WD body. dword0 packs the packet
 * length + rpkt_type; dwords 1-3 are zero (txdes_proc_h2c_fwdl_8852c). */
constexpr uint32_t RXD_SHORT_LEN = 16;
constexpr uint8_t AX_RXD_RPKT_LEN_SH = 0;
constexpr uint32_t AX_RXD_RPKT_LEN_MSK = 0x3fff;
constexpr uint8_t AX_RXD_RPKT_TYPE_SH = 24;
constexpr uint32_t AX_RXD_RPKT_TYPE_MSK = 0xf;
constexpr uint8_t RXD_S_RPKT_TYPE_H2C = 13;  /* FWDL header (fwcmd) */
constexpr uint8_t RXD_S_RPKT_TYPE_FWDL = 14; /* FWDL section data */
constexpr uint8_t FWCMD_TYPE_H2C = 0;
constexpr uint8_t FWCMD_H2C_CAT_MAC = 0x1;
constexpr uint8_t FWCMD_H2C_CL_FWDL = 0x3;
constexpr uint8_t FWCMD_H2C_FUNC_FWHDR_DL = 0x0;
/* H2C hdr0 fields (fwcmd_intf.h). */
constexpr uint8_t H2C_HDR_CAT_SH = 0;
constexpr uint8_t H2C_HDR_CLASS_SH = 2;
constexpr uint8_t H2C_HDR_FUNC_SH = 8;
constexpr uint8_t H2C_HDR_DEL_TYPE_SH = 16;
constexpr uint8_t H2C_HDR_H2C_SEQ_SH = 24;
/* H2C hdr1 fields. */
constexpr uint8_t H2C_HDR_TOTAL_LEN_SH = 0;
/* TX WD (txdesc.h): dword0 CH_DMA [19:16], FWDL_EN [20]; dword2 TXPKTSIZE [15:0]. */
constexpr uint8_t AX_TXD_CH_DMA_SH = 16;
constexpr uint32_t AX_TXD_CH_DMA_MSK = 0xf;
constexpr uint32_t AX_TXD_FWDL_EN = 1u << 20;
constexpr uint8_t AX_TXD_TXPKTSIZE_SH = 0;
constexpr uint32_t AX_TXD_TXPKTSIZE_MSK = 0xffff;
constexpr uint8_t MAC_AX_DMA_H2C = 12; /* CH12 */
constexpr uint8_t BULKOUTID_H2C = 2;   /* get_bulkout_id_8852b: H2C -> id 2 */

/* ---- FWDL IDMEM/CPU-clock patch (fwdl.c idmem_share_mode_check +
 * fwdl_patch_fw_delay). On a NON-secure IC the vendor (a) forces the SEC_CTRL
 * IDMEM-share field to the AX-MIPS default (0x1) and (b) writes the fw CPU-clk
 * setting into IDMEM via the indirect-access window so the fw's secure-checksum
 * delay is accurate — WITHOUT this the running fw mis-times and faults on the
 * HCI bus (HALT_C2H = L2_ERR_AH_HCI). is_sec_ic from OTP key cell 0x5ED[7]. ---*/
constexpr uint16_t R_AX_SEC_CTRL = 0x0C00;
constexpr uint8_t B_SEC_IDMEM_SIZE_CONFIG_SH = 16; /* [17:16] */
constexpr uint32_t B_SEC_IDMEM_SIZE_CONFIG_MSK = 0x3;
constexpr uint8_t FWDL_IDMEM_SHARE_DEFAULT_MODE = 0x1;       /* AX MIPS (8852B) */
constexpr uint8_t FWDL_IDMEM_SHARE_DEFAULT_MODE_8852C = 0x0; /* AX RISC-V */
constexpr uint16_t R_AX_FILTER_MODEL_ADDR = 0x0C04;
constexpr uint32_t R_AX_INDIR_ACCESS_ENTRY = 0x40000; /* wide: wIndex=4 */
constexpr uint32_t FW_CPU_CLK_ADDR_8852B = 0x18E0C3D8;
constexpr uint32_t FW_FAKE_CPU_CLK_8852B = 0x0000000A;
/* 8852C fw CPU-clk patch IDMEM address/value (fwdl_patch_fw_delay). Patching
 * the 8852B address leaves the 8852C fw's clock wrong so it faults during its
 * post-download HW init (hangs at "INIT HW POSTDL", STS=6). */
constexpr uint32_t FW_CPU_CLK_ADDR_8852C = 0x2020FD40;
constexpr uint32_t FW_FAKE_CPU_CLK_8852C = 0x0000003C;
constexpr uint16_t OTP_KEY_INFO_CELL_02_ADDR = 0x5ED; /* [7] = security-record */

/* ---- Firmware IO-offload (fwofld.c, fwcmd_intf.h) — how BB/RF register
 * tables are actually programmed on USB: batched into H2C commands the FW
 * replays on-chip (the vendor does ZERO direct BB/RF-window writes). ---- */
constexpr uint8_t FWCMD_H2C_CAT_OUTSRC = 0x2;
constexpr uint8_t FWCMD_H2C_CL_FW_OFLD = 0x9;
/* FW log config (mac_fw_log_cfg): cat=MAC, class=FW_INFO(0x0), func=LOG_CFG(0x0).
 * content dword0 = level[7:0] | output-path[15:8], dword1 = comp bitmap,
 * dword2 = comp_ext. output=MAC_AX_FL_LV_C2H(BIT1) routes the fw log to
 * packet-C2H (rpkt_type=10, C2H class FW_INFO(0x0) func C2H_LOG(0x2)) — the
 * decisive probe of whether async packet-C2H reaches the host at all. */
constexpr uint8_t FWCMD_H2C_CL_FW_INFO = 0x0;
constexpr uint8_t FWCMD_H2C_FUNC_LOG_CFG = 0x0;
constexpr uint8_t MAC_AX_FL_LV_TR = 5;   /* verbose (trace) level */
constexpr uint8_t MAC_AX_FL_LV_C2H = 0x2; /* BIT(1): route log to C2H packets */
constexpr uint8_t FWCMD_C2H_CL_FW_INFO = 0x0;
constexpr uint8_t FWCMD_C2H_FUNC_C2H_LOG = 0x02;
constexpr uint8_t FWCMD_H2C_FUNC_CMD_OFLD_REG = 0x11;
constexpr uint8_t FWCMD_H2C_FUNC_CMD_OFLD_PKT = 0x13;
/* USR_TX_RPT (mac_cfg_usr_tx_rpt): enable the per-user TX report — content
 * dword0 = MODE[2:0] | RTP_START(bit3), dword1 = MACID[7:0] | BAND(bit10) |
 * PORT[13:11]. The report returns as C2H USR_TX_RPT_INFO (func 0x7). */
constexpr uint8_t FWCMD_H2C_FUNC_USR_TX_RPT = 0x12;
constexpr uint32_t B_H2C_USR_TX_RPT_RTP_START = 1u << 3;
constexpr uint32_t B_H2C_USR_TX_RPT_BAND = 1u << 10;
constexpr uint8_t H2C_USR_TX_RPT_PORT_SH = 11;
/* NB: the C2H class enum is a SEPARATE numbering from the H2C class enum —
 * FW_OFLD is 0x9 for H2C (above) but 0x1 for C2H (fwcmd_intf.h
 * FWCMD_C2H_CL_FW_OFLD). A received USR_TX_RPT_INFO carries C2H class 0x1;
 * matching it against the H2C 0x9 silently drops the report. */
constexpr uint8_t FWCMD_C2H_CL_FW_OFLD = 0x1;
constexpr uint8_t FWCMD_C2H_FUNC_USR_TX_RPT_INFO = 0x7;
constexpr uint8_t USR_TX_RPT_MODE_PERIOD = 1;
constexpr uint8_t USR_TX_RPT_MODE_LAST_PKT = 2;
/* FWROLE_MAINTAIN (mac_fw_role_maintain, role.c): the H2C that makes the fw
 * create/track a MACID role — the linchpin that lets per-MACID features (data
 * TX, power-by-rate, USR_TX_RPT frame-stat) engage. cat=MAC, class=MEDIA_RPT,
 * func=FWROLE_MAINTAIN. dword0 = MACID[7:0] | SELF_ROLE[9:8] | UPD_MODE[12:10]
 * | WIFI_ROLE[16:13] | BAND[18:17] | PORT[21:19]. */
constexpr uint8_t FWCMD_H2C_CL_MEDIA_RPT = 0x8;
constexpr uint8_t FWCMD_H2C_FUNC_FWROLE_MAINTAIN = 0x4;
constexpr uint8_t H2C_FWROLE_SELF_ROLE_SH = 8;  /* mask 0x3 */
constexpr uint8_t H2C_FWROLE_UPD_MODE_SH = 10;  /* mask 0x7 */
constexpr uint8_t H2C_FWROLE_WIFI_ROLE_SH = 13; /* mask 0xf */
constexpr uint8_t H2C_FWROLE_BAND_SH = 17;      /* mask 0x3 */
constexpr uint8_t H2C_FWROLE_PORT_SH = 19;      /* mask 0x7 */
/* mac_ax_self_role. */
constexpr uint8_t MAC_AX_SELF_ROLE_CLIENT = 0;
constexpr uint8_t MAC_AX_SELF_ROLE_AP = 1;
/* mac_ax_wifi_role. */
constexpr uint8_t MAC_AX_WIFI_ROLE_NONE = 0;
constexpr uint8_t MAC_AX_WIFI_ROLE_STATION = 1;
constexpr uint8_t MAC_AX_WIFI_ROLE_AP = 2;
/* mac_ax_upd_mode. */
constexpr uint8_t MAC_AX_ROLE_CREATE = 0;
constexpr uint8_t MAC_AX_ROLE_REMOVE = 1;
/* Scheduler contention TX-enable (R_AX_CTN_TXEN, cmac_tx.c set_hw_sch_tx_en).
 * A per-queue gate: a queue whose bit is 0 never contends for the medium, so
 * its frames sit in PLE forever (the ~103 mgmt-TX stall). The vendor enables
 * these via mac_set_hw_value(SCH_TXEN_CFG) when the vif comes up; devourer
 * never did. Enable the queues our TX uses: BE0..VO0 (data ACs), MGQ + CPUMGQ
 * (host + cpu mgmt), HGQ (high-priority). */
constexpr uint16_t R_AX_CTN_TXEN = 0xC348;
/* Driver-side contention TX enable (band-0). CTN_TXEN gates the HW queues;
 * CTN_DRV_TXEN gates driver-injected frames into contention. The 8852C powers
 * up with it cleared, so injected frames never win a TX opportunity, never
 * complete, and the PLE pool fills (~103-frame stall, no rpkt_type=7 release).
 * Vendor set_hw_sch_tx_en sets it to 0x0003ffff at ifup. */
constexpr uint16_t R_AX_CTN_DRV_TXEN = 0xC398;
constexpr uint32_t B_AX_CTN_DRV_TXEN_ALL = 0x0003ffff;
constexpr uint16_t B_AX_CTN_TXEN_BE_0 = 1u << 0;
constexpr uint16_t B_AX_CTN_TXEN_BK_0 = 1u << 1;
constexpr uint16_t B_AX_CTN_TXEN_VI_0 = 1u << 2;
constexpr uint16_t B_AX_CTN_TXEN_VO_0 = 1u << 3;
constexpr uint16_t B_AX_CTN_TXEN_MGQ = 1u << 8;
constexpr uint16_t B_AX_CTN_TXEN_CPUMGQ = 1u << 10;
constexpr uint16_t B_AX_CTN_TXEN_HGQ = 1u << 11;

/* --- Self-STA registration H2Cs (the "rest of _add_role" after fw_role_maintain):
 * ADDR_CAM update + CMAC control-table (cctl). Without these the CMAC has no
 * per-MACID BSS/rate/antenna context, so injected frames queue but never air —
 * the mgmt bulk-OUT stalls at ~103 (the PLE page pool fills). Ported verbatim
 * from mac_ax addr_cam.c / tblupd.c (fwcmd_intf.h field layouts). */

/* ADDR_CAM: CAT_MAC, CL_ADDR_CAM_UPDATE(6), FUNC_ADDRCAM_INFO(0). 15-dword body. */
constexpr uint8_t FWCMD_H2C_CL_ADDR_CAM_UPDATE = 0x6;
constexpr uint8_t FWCMD_H2C_FUNC_ADDRCAM_INFO = 0x0;
/* CCTL/DCTL: CAT_MAC, CL_FR_EXCHG(5), FUNC CCTLINFO_UD(2)/DCTLINFO_UD(1). */
constexpr uint8_t FWCMD_H2C_CL_FR_EXCHG = 0x5;
constexpr uint8_t FWCMD_H2C_FUNC_CCTLINFO_UD = 0x2;
constexpr uint8_t FWCMD_H2C_FUNC_DCTLINFO_UD = 0x1;

/* addr_cam entry geometry (addr_cam.h): 8852B uses the LONG entry (0x40). */
constexpr uint8_t ADDR_CAM_ENT_LONG_SIZE = 0x40;
constexpr uint8_t BSSID_CAM_ENT_SIZE = 0x08;
/* mac_ax_addr_msk_sel (mac_def.h). */
constexpr uint8_t MAC_AX_NO_MSK = 0;
/* mac_ax_data_rate (type.h) — the CCTL default rate. */
constexpr uint16_t MAC_AX_OFDM6 = 0x4;

/* fwcmd_cctlinfo_ud dword0 (tblupd.c). */
constexpr uint8_t FWCMD_H2C_CCTLINFO_UD_MACID_MSK = 0x7f;
constexpr uint32_t FWCMD_H2C_CCTLINFO_UD_OP = 1u << 7;
/* CCTL field shifts/masks used for a basic mgmt-TX entry (fwcmd_intf.h). */
constexpr uint8_t FWCMD_H2C_CCTRL_DATARATE_SH = 0; /* dword1 */
constexpr uint32_t FWCMD_H2C_CCTRL_DATARATE_MSK = 0x1ff;
constexpr uint32_t FWCMD_H2C_CCTRL_DISRTSFB = 1u << 25;  /* dword1 */
constexpr uint32_t FWCMD_H2C_CCTRL_DISDATAFB = 1u << 26; /* dword1 */
/* Per-queue TX-report enables (dword1). Without the queue's *_rpt_en bit the fw
 * emits no USR_TX_RPT C2H for frames on that queue — the host-visible TX-egress
 * timestamp (#236) never fires. mgmt/beacon rides MGQ, data rides ACQ. */
constexpr uint32_t FWCMD_H2C_CCTRL_ACQ_RPT_EN = 1u << 20; /* dword1 */
constexpr uint32_t FWCMD_H2C_CCTRL_MGQ_RPT_EN = 1u << 21; /* dword1 */
constexpr uint32_t FWCMD_H2C_CCTRL_BMC = 1u << 3;        /* dword5 */
constexpr uint8_t FWCMD_H2C_CCTRL_NTX_PATH_EN_SH = 16;   /* dword6 */
constexpr uint32_t FWCMD_H2C_CCTRL_NTX_PATH_EN_MSK = 0xf;
constexpr uint8_t FWCMD_H2C_CCTRL_PATH_MAP_A_SH = 20; /* dword6 */
constexpr uint32_t FWCMD_H2C_CCTRL_PATH_MAP_A_MSK = 0x3;
constexpr uint8_t FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_SH = 0; /* dword7 */
constexpr uint32_t FWCMD_H2C_CCTRL_ADDR_CAM_INDEX_MSK = 0xff;

/* rtw_mac_src_cmd_ofld (mac_outsrc_def.h). */
constexpr uint8_t OFLD_SRC_BB = 0;
constexpr uint8_t OFLD_SRC_RF = 1;
constexpr uint8_t OFLD_SRC_MAC = 2;
constexpr uint8_t OFLD_SRC_RF_DDIE = 3; /* sent as SRC_RF with dword1 offset=1 */
/* rtw_mac_cmd_type_ofld. */
constexpr uint8_t OFLD_TYPE_WRITE = 0;
constexpr uint8_t OFLD_TYPE_COMPARE = 1;
constexpr uint8_t OFLD_TYPE_DELAY = 2;
/* fwcmd_cmd_ofld 16-byte command dword0/1 field shifts (fwcmd_intf.h). */
constexpr uint8_t CMD_OFLD_SRC_SH = 0;
constexpr uint8_t CMD_OFLD_TYPE_SH = 2;
constexpr uint32_t CMD_OFLD_LC = 1u << 4;
constexpr uint8_t CMD_OFLD_PATH_SH = 5;
constexpr uint8_t CMD_OFLD_CMD_NUM_SH = 8;
constexpr uint8_t CMD_OFLD_OFFSET_SH = 16;
constexpr uint32_t CMD_OFLD_OFFSET_MSK = 0xffff;
constexpr uint32_t CMD_OFLD_SIZE = 16;     /* sizeof(fwcmd_cmd_ofld) */
constexpr uint32_t CMD_OFLD_MAX_LEN = 2000; /* buffer / max content per H2C */
/* halbb flushes every fwofld batch by appending a dummy LC-flagged BB write to
 * 0x1a24 with mask 0xff, value 0 (halbb_fwofld.c:93 halbb_fwofld_bitmap_en) —
 * the LC command is the trailing sentinel, never a real table entry. */
constexpr uint16_t BB_OFLD_FLUSH_ADDR = 0x1a24;
constexpr uint32_t BB_OFLD_FLUSH_MASK = 0xff;
constexpr uint32_t MASKDWORD = 0xffffffffu;
constexpr uint32_t MASKRF = 0x000fffffu; /* 20-bit RF register width */
/* radio-to-fw (halrf) OUTSRC H2C classes: 8 = radio A, 9 = radio B, func = page. */
constexpr uint8_t OUTSRC_CL_RADIO_A = 8;
constexpr uint8_t OUTSRC_CL_RADIO_B = 9;
constexpr uint32_t RADIO_TO_FW_DATA_SIZE = 500; /* entries per page */
constexpr uint8_t H2CB_TYPE_LONG_DATA = 2;

/* C2H-register mailbox (fwcmd.c __recv_c2hreg) — the flow-control handshake
 * after each cmd_ofld batch: wait for TRIGGER, read data, CLEAR trigger (ack).
 * The ack releases the fw to return the H2C page; without it the H2C queue
 * fills after a few batches and the next bulk-OUT jams. */
constexpr uint16_t R_AX_C2HREG_DATA0 = 0x8150;
constexpr uint16_t R_AX_C2HREG_DATA1 = 0x8154;
constexpr uint16_t R_AX_C2HREG_DATA2 = 0x8158;
constexpr uint16_t R_AX_C2HREG_DATA3 = 0x815C;
constexpr uint16_t R_AX_C2HREG_CTRL = 0x8164;
constexpr uint8_t B_AX_C2HREG_TRIGGER = 1u << 0;
constexpr uint32_t CMD_OFLD_POLL_CNT = 4000;
constexpr uint32_t CMD_OFLD_POLL_US = 50;
constexpr uint8_t FWCMD_C2HREG_FUNC_IO_OFLD_RESULT = 0x9;

/* ---- USB runtime init (usb_init_8852b, _usb.c/_usb_8852b.c): endpoint NUMP
 * (burst) config. Without it a bulk-OUT endpoint accepts one packet then
 * halts — the "512 then stall" the H2C IO-offload hits. ---- */
constexpr uint16_t R_AX_USB_STATUS = 0x11F0;
constexpr uint32_t B_AX_R_USB2_SEL = 1u << 1;
constexpr uint32_t B_AX_MODE_HS = 1u << 0;
constexpr uint16_t R_AX_RXDMA_SETTING = 0x8908;
constexpr uint8_t USB3_BULKSIZE = 0x0;
constexpr uint8_t USB2_BULKSIZE = 0x1;
constexpr uint8_t USB11_BULKSIZE = 0x2;
constexpr uint16_t R_AX_USB_ENDPOINT_0 = 0x1060;
constexpr uint32_t B_AX_EP_IDX_MSK = 0xf;
constexpr uint8_t B_AX_EP_IDX_SH = 0;
constexpr uint16_t R_AX_USB_ENDPOINT_2 = 0x1068; /* NUMP at +1 = 0x1069 */
constexpr uint16_t R_AX_USB3_MAC_NPI_CONFIG_INTF_0 = 0x1114;
constexpr uint32_t B_AX_SSPHY_LFPS_FILTER = 1u << 31;
/* 8852C runtime usb_init (usb_init_8852c) targets the _V1 USB register bank —
 * endpoint index/NUMP, mode status, LFPS filter. Same bit layout. Programming
 * NUMP on the 8852B endpoint registers leaves the 8852C's per-EP burst count
 * unset so its H2C (EP7) bulk-out stalls after FWDL. */
constexpr uint16_t R_AX_USB_STATUS_V1 = 0x51F0;
constexpr uint16_t R_AX_USB_ENDPOINT_0_V1 = 0x5060;
constexpr uint16_t R_AX_USB_ENDPOINT_2_V1 = 0x5068; /* NUMP at +1 = 0x5069 */
constexpr uint16_t R_AX_USB3_MAC_NPI_CONFIG_INTF_0_V1 = 0x5114;
constexpr uint8_t USB_NUMP = 0x1;
/* Endpoints configured with NUMP (EP5,6,7,9,10,11,12). */
constexpr uint8_t USB_EP_LIST[7] = {0x5, 0x6, 0x7, 0x9, 0xA, 0xB, 0xC};
constexpr uint32_t MAC_AX_USB3 = 3;
constexpr uint32_t MAC_AX_USB2 = 2;
constexpr uint32_t MAC_AX_USB11 = 1;

/* ---- MAC TRX init (M2a; trxcfg.c, mac_reg_ax.h) ---- */
constexpr uint32_t TRXCFG_WAIT_CNT = 2000;
constexpr uint32_t TRXCFG_WAIT_US = 1;
/* sta_sch_init (station scheduler). */
constexpr uint16_t R_AX_SS_CTRL = 0x9E10;
constexpr uint32_t B_AX_SS_EN = 1u << 0;
constexpr uint32_t B_AX_SS_INIT_DONE_1 = 1u << 31;
constexpr uint32_t B_AX_SS_WARM_INIT_FLG = 1u << 29;
constexpr uint32_t B_AX_SS_NONEMPTY_SS2FINFO_EN = 1u << 28;
/* mpdu_proc_init. */
constexpr uint16_t R_AX_MPDU_PROC = 0x9C00;
constexpr uint32_t B_AX_A_ICV_ERR = 1u << 1;
constexpr uint32_t B_AX_APPEND_FCS = 1u << 0;
constexpr uint16_t R_AX_ACTION_FWD0 = 0x9C04;
constexpr uint16_t R_AX_TF_FWD = 0x9C14;
constexpr uint16_t R_AX_CUT_AMSDU_CTRL = 0x9C40;
constexpr uint32_t TRXCFG_MPDU_PROC_ACT_FRWD = 0x02A95A95;
constexpr uint32_t TRXCFG_MPDU_PROC_TF_FRWD = 0x0000AA55;
constexpr uint32_t TRXCFG_MPDU_PROC_CUT_CTRL = 0x010E05F0;
/* sec_eng_init. */
constexpr uint16_t R_AX_SEC_MPDU_PROC = 0x9D04; /* |= APPEND_ICV|APPEND_MIC */
constexpr uint32_t B_AX_APPEND_ICV = 1u << 1;
constexpr uint32_t B_AX_APPEND_MIC = 1u << 0;
constexpr uint16_t R_AX_SEC_ENG_CTRL = 0x9D00;
constexpr uint32_t B_AX_TX_PARTIAL_MODE = 1u << 11;
constexpr uint32_t B_AX_CLK_EN_CGCMP = 1u << 10;
constexpr uint32_t B_AX_CLK_EN_WAPI = 1u << 9;
constexpr uint32_t B_AX_CLK_EN_WEP_TKIP = 1u << 8;
constexpr uint32_t B_AX_BMC_MGNT_DEC = 1u << 5;
constexpr uint32_t B_AX_UC_MGNT_DEC = 1u << 4;
constexpr uint32_t B_AX_MC_DEC = 1u << 3;
constexpr uint32_t B_AX_BC_DEC = 1u << 2;
constexpr uint32_t B_AX_SEC_RX_DEC = 1u << 1;
constexpr uint32_t B_AX_SEC_TX_ENC = 1u << 0;

/* NIC-mode DLE quota: the 8852BU USB hal runs qta_mode = MAC_AX_QTA_SCC_TURBO
 * (rtl8852bu_halinit.c:33, quota_turbo default) and this rig enumerates USB2
 * HS, so the live table is the dle_mem_usb2_8852b SCC_TURBO row:
 * wde_size30 / ple_size31 / wde_qt30 / ple_qt27(min) / ple_qt28(max),
 * constants below sourced row-by-row from dle.c. */
constexpr uint16_t SCC_WDE_LNK_PAGE = 224;  /* wde_size30 lnk (dle.c:221) */
constexpr uint16_t SCC_WDE_UNLNK_PAGE = 32; /* wde_size30 unlnk */
constexpr uint16_t SCC_PLE_LNK_PAGE = 1392; /* ple_size31 lnk (dle.c:475) */
/* wde_qt30: hif/wcpu/pkt_in/cpu_io (min==max) (dle.c:838). */
constexpr uint16_t SCC_WDE_QT_HIF = 210;
constexpr uint16_t SCC_WDE_QT_WCPU = 6;
constexpr uint16_t SCC_WDE_QT_CPU_IO = 8;
/* ple_qt27 (min, dle.c:1352) then ple_qt28 (max, dle.c:1369), 11 fields
 * Q0..Q10 (cmac0_tx, cmac1_tx, c2h, h2c, wcpu, mpdu_proc, cmac0_dma,
 * cmac1_dma, bb_rpt, wd_rel, cpu_io). */
constexpr uint16_t SCC_PLE_MIN[11] = {1040, 0, 16, 48, 13, 13, 178, 0, 32, 14, 8};
constexpr uint16_t SCC_PLE_MAX[11] = {1040, 0, 32, 48, 43, 13, 208, 0, 62, 14, 24};

/* 8852C USB3 SCC (NIC-mode) DLE quota — dle_mem_usb3_8852c: wde_size17/
 * ple_size17/wde_qt16/ple_qt42(min)/ple_qt43(max). Differs substantially from
 * the 8852B and — critically — carries a tx_rpt (PLE Q11) quota (16/41) that
 * the 8852B set omits. Without a tx_rpt PLE reserve the fw cannot store a TX
 * completion report, so an injected frame never releases its WD/PLE pages
 * (no rpkt_type=4/7) and the mgmt bulk-OUT ~103/203-stalls. 12 fields
 * Q0..Q11 (…cpu_io, tx_rpt). */
constexpr uint16_t SCC_WDE_LNK_PAGE_8852C = 512;
constexpr uint16_t SCC_WDE_UNLNK_PAGE_8852C = 0;
constexpr uint16_t SCC_PLE_LNK_PAGE_8852C = 3312;
constexpr uint16_t SCC_WDE_QT_HIF_8852C = 412;
constexpr uint16_t SCC_WDE_QT_WCPU_8852C = 60;
constexpr uint16_t SCC_WDE_QT_CPU_IO_8852C = 40;
/* SCC PLE quota — USB3 (dle_mem_usb3_8852c: ple_qt42 min / ple_qt43 max). */
constexpr uint16_t SCC_PLE_MIN_8852C[12] = {1068, 0, 16, 48, 120, 13,
                                            178,  0, 16, 38, 8,   16};
constexpr uint16_t SCC_PLE_MAX_8852C[12] = {2859, 0, 32, 48, 145, 13,
                                            178,  0, 16, 38, 8,   41};
/* SCC PLE quota — USB2 (dle_mem_usb2_8852c: ple_qt78 min / ple_qt79 max). The
 * 8852C has a DEDICATED USB2 DLE config (unlike the 8852B, which only has
 * USB3): on the slower USB2 drain the CMAC0-DMA RX queue (Q6) needs a far
 * larger ceiling — 390/1646 vs the USB3 178/178 — plus a bigger BB-RPT/physts
 * (Q8) 32/1288 vs 16/16. Selected by the live USB mode in dle_init_nic; using
 * the USB3 quota on a USB2 8832CU caps the RX-DMA pool and wedges RX after the
 * init buffer fills (burst-then-idle). WDE quota + page sizes are identical. */
constexpr uint16_t SCC_PLE_MIN_8852C_USB2[12] = {1482, 0, 16, 48, 13, 13,
                                                 390,  0, 32, 38, 8,  16};
constexpr uint16_t SCC_PLE_MAX_8852C_USB2[12] = {1482, 0,    32,   48, 1269, 13,
                                                 1646, 0, 1288, 38,  120, 1272};

/* ---- BB/RF enable (hw.c set_enable_bb_rf) — releases the BB from reset
 * so the halbb/halrf register windows actually accept writes. All MAC/system
 * space (wIndex=0). MUST run before applying the BB/RF tables. ---- */
constexpr uint16_t R_AX_SYS_FUNC_EN = 0x0002;
constexpr uint32_t B_AX_FEN_BB_GLB_RSTN = 1u << 1;
constexpr uint32_t B_AX_FEN_BBRSTB = 1u << 0;
constexpr uint16_t R_AX_WLRF_CTRL = 0x02F0;
constexpr uint32_t B_AX_AFC_AFEDIG = 1u << 17;
constexpr uint8_t B_AX_REG_ZCDC_H_SH = 17;
constexpr uint32_t B_AX_REG_ZCDC_H_MSK = 0x3;
constexpr uint16_t R_AX_PHYREG_SET = 0x8040;
constexpr uint8_t PHYREG_SET_XYN_CYCLE = 0xE;
constexpr uint8_t XTAL_SI_WL_RFC_S0 = 0x80;
constexpr uint8_t XTAL_SI_WL_RFC_S1 = 0x81;
/* 8852C set_enable_bb_rf (hw.c) ADC/AFE + xtal block: XTAL0=0x7, ANAPAR=0x6c,
 * XTAL3=0xd; AFE LDO for ADC power. */
constexpr uint8_t XTAL_SI_XTAL0 = 0x0;
constexpr uint8_t XTAL_SI_XTAL3 = 0x3;
constexpr uint16_t R_AX_AFE_OFF_CTRL1 = 0x0444;
constexpr uint32_t LDO2PW_LDO_VSEL = 0x3600000;
constexpr uint8_t B_AX_S0_LDO_VSEL_F_SH = 21; /* [22:21] */
constexpr uint8_t B_AX_S1_LDO_VSEL_F_SH = 24; /* [25:24] */

/* ---- CMAC RX init (M2a cont.; trxcfg.c / rx_filter.c / mac_reg_ax.h) ---- */
constexpr uint16_t R_AX_RXDMA_CTRL_0 = 0xC804;
constexpr uint32_t RX_FULL_MODE = /* RU0/1/2/3 + CSI + RXSTS ptr-full-mode */
    (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5);
constexpr uint16_t R_AX_TX_SUB_CARRIER_VALUE = 0xC088;
constexpr uint8_t B_AX_TXSC_20M_SH = 0;
constexpr uint8_t B_AX_TXSC_40M_SH = 4;
constexpr uint8_t B_AX_TXSC_80M_SH = 8;
constexpr uint32_t B_AX_TXSC_MSK = 0xf;
constexpr uint16_t R_AX_PTCL_RRSR1 = 0xC090;
constexpr uint8_t B_AX_RRSR_RATE_EN_SH = 8;
constexpr uint32_t B_AX_RRSR_RATE_EN_MSK = 0xf;
constexpr uint8_t RRSR_OFDM_CCK_EN = 3;
constexpr uint16_t R_AX_CCA_CONTROL = 0xC390;
/* cca_ctrl_init: bits to set vs clear (verified mac_reg_ax.h). */
constexpr uint32_t CCA_CTRL_SET =
    (1u << 30) | (1u << 29) | (1u << 28) | (1u << 24) | /* TB basic_nav/btcca/edcca/p20 */
    (1u << 21) | (1u << 16) |                           /* SIFS btcca/p20 */
    (1u << 7) | (1u << 6) | (1u << 5) | (1u << 4) |     /* CTN intra/basic/btcca/edcca */
    (1u << 3) | (1u << 2) | (1u << 1) | (1u << 0);      /* CTN cca s80/s40/s20/p20 */
constexpr uint32_t CCA_CTRL_CLR =
    (1u << 31) | (1u << 27) | (1u << 26) | (1u << 25) | /* TB tx_nav/cca s80/s40/s20 */
    (1u << 19) | (1u << 18) | (1u << 17) |              /* SIFS cca s80/s40/s20 */
    (1u << 8) | (1u << 20);                             /* CTN txnav, SIFS edcca */
constexpr uint16_t R_AX_RCR = 0xCE00;
constexpr uint8_t B_AX_CH_EN_SH = 0;
constexpr uint32_t B_AX_CH_EN_MSK = 0xf; /* 8852B RCR channel-enable [3:0] */
/* 8852C rmac_init uses B_AX_CH_EN_V1 ([7:0], mask 0xff) — a WIDER field than the
 * 8852B [3:0]. The single-CMAC 8852C enables only channel 0 (=0x1, the vendor's
 * live monitor value); writing 0xF into [3:0] enables channels 1-3 that don't
 * exist and wedges RX. */
constexpr uint8_t B_AX_CH_EN_V1_SH = 0;
constexpr uint32_t B_AX_CH_EN_V1_MSK = 0xff;
/* R_AX_PLCP_HDR_FLTR (0xCE04): rmac_init clears VHT_SU_SIGB_CRC_CHK (not all
 * vendors compute VHT SIG-B CRC). */
constexpr uint16_t R_AX_PLCP_HDR_FLTR = 0xCE04;
constexpr uint32_t B_AX_VHT_SU_SIGB_CRC_CHK = 1u << 4;
/* R_AX_RCR drv-info size (8852C sec_eng_init: DRVINFO_PATCH_SIZE=0x5). */
constexpr uint8_t B_AX_DRV_INFO_SIZE_SH = 8;
constexpr uint32_t B_AX_DRV_INFO_SIZE_MSK = 0x7;
constexpr uint32_t DRVINFO_PATCH_SIZE = 0x5;
/* 8852C sec_eng_init: SEC_DEBUG1 tx-timeout select. */
constexpr uint16_t R_AX_SEC_DEBUG1 = 0x9D1C;
constexpr uint8_t B_AX_TX_TIMEOUT_SEL_SH = 30;
constexpr uint32_t B_AX_TX_TIMEOUT_SEL_MSK = 0x3;
constexpr uint32_t B_AX_TX_TO = 0x2;
/* 8852C scheduler/trxptcl timing constants. */
constexpr uint32_t SIFS_MACTXEN_T1_V2 = 0x3E;      /* vs V0=0x47 */
constexpr uint32_t WMAC_SPEC_SIFS_OFDM_52C = 0x11; /* vs _52B */
constexpr uint8_t B_AX_R_SIFS_AGGR_TIME_V1_SH = 24;
constexpr uint32_t B_AX_R_SIFS_AGGR_TIME_V1_MSK = 0xff; /* vs 0x7f */
constexpr uint16_t R_AX_DLK_PROTECT_CTL = 0xCE02;
constexpr uint8_t B_AX_RX_DLK_DATA_TIME_SH = 4;
constexpr uint32_t B_AX_RX_DLK_DATA_TIME_MSK = 0xf;
constexpr uint8_t B_AX_RX_DLK_CCA_TIME_SH = 8;
constexpr uint32_t B_AX_RX_DLK_CCA_TIME_MSK = 0xff;
constexpr uint16_t B_AX_RX_DLK_RST_EN = 1u << 1;
constexpr uint16_t R_AX_RX_FLTR_OPT = 0xCE20;
constexpr uint32_t B_AX_SNIFFER_MODE = 1u << 0;
constexpr uint32_t B_AX_A_A1_MATCH = 1u << 1;
constexpr uint32_t B_AX_A_BC = 1u << 2;
constexpr uint32_t B_AX_A_MC = 1u << 3;
constexpr uint32_t B_AX_A_UC_CAM_MATCH = 1u << 4;
constexpr uint32_t B_AX_A_BC_CAM_MATCH = 1u << 5;
constexpr uint32_t B_AX_A_CRC32_ERR = 1u << 11;
constexpr uint8_t B_AX_RX_MPDU_MAX_LEN_SH = 16;
constexpr uint32_t B_AX_RX_MPDU_MAX_LEN_MSK = 0x3f;
constexpr uint16_t R_AX_CTRL_FLTR = 0xCE24;
constexpr uint16_t R_AX_MGNT_FLTR = 0xCE28;
constexpr uint16_t R_AX_DATA_FLTR = 0xCE2C;
constexpr uint32_t RX_FLTR_ALL_TO_HOST = 0x55555555; /* 16 subtypes x 2b, FWD_TO_HOST=1 */
constexpr uint16_t R_AX_RESPBA_CAM_CTRL = 0xCE3C;
constexpr uint32_t B_AX_SSN_SEL = 1u << 2;
constexpr uint16_t R_AX_RX_TIME_MON = 0xCEEC;
constexpr uint8_t B_AX_INTF_TIMEOUT_THR_SH = 16;
constexpr uint32_t B_AX_INTF_TIMEOUT_THR_MSK = 0x3f;
constexpr uint16_t TRXCFG_RMAC_DATA_TO = 15;
constexpr uint16_t TRXCFG_RMAC_CCA_TO = 32;
/* rx_max_len for band0 = min(c0_rx_qta[=cma0_dma 178], PLD_RLS_MAX_PG[127])
 * * ple_pg_size[128] / RX_MAX_LEN_UNIT[512] = 127*128/512 = 31. */
constexpr uint16_t RMAC_RX_MPDU_MAX_LEN = 31;
/* USB RX aggregation (rx_agg_cfg_usb_8852b, _usb_8852b.c). The RXAGG engine
 * is the mechanism that DMAs RX frames to the USB bulk-IN — for USB mode the
 * vendor ENABLES it (B_AX_RXAGG_EN) with DMA_STORE off and size/timeout
 * thresholds. NB: the enable bit is BIT(31), not bit 0 (bit 0 is LEN_TH[0]). */
constexpr uint16_t R_AX_RXAGG_0 = 0x8900;
/* 8852C USB RXAGG engine is the _V1 register (usb_rx_agg_cfg_8852c); same bit
 * layout. Enabling RXAGG on the 8852B address leaves the 8852C's engine off, so
 * WiFi frames never aggregate onto the bulk-IN (only C2H, via a separate path)
 * and monitor RX is deaf on both bands. */
constexpr uint16_t R_AX_RXAGG_0_V1 = 0x6000;
constexpr uint32_t B_AX_RXAGG_EN = 1u << 31;
constexpr uint32_t B_AX_RXAGG_DMA_STORE = 1u << 30;
constexpr uint32_t B_AX_RXAGG_SW_EN = 1u << 29;
constexpr uint8_t B_AX_RXAGG_PKTNUM_TH_SH = 16;
constexpr uint32_t B_AX_RXAGG_PKTNUM_TH_MSK = 0xff;
constexpr uint8_t B_AX_RXAGG_TIMEOUT_TH_SH = 8;
constexpr uint8_t B_AX_RXAGG_LEN_TH_SH = 0;
/* _usb.h: RXAGGSIZE=0x5, RXAGGTO=0x20 (timeout), pkt_num=0. */
constexpr uint8_t RXAGGSIZE = 0x5;
constexpr uint8_t RXAGGTO = 0x20;
/* usb_rx_agg_cfg_8852c (_usb_8852c.c:345): LEN_TH = RXAGGSIZE*COMPAT_RX_AGG_UNIT
 * (=4), plus it writes R_AX_RXAGG_1_V1 to neutralise the small-packet
 * aggregation (SML_PKTNUM_TH=max, SML_PKT_SIZE=0, LEN_TH_HIGH=0). devourer
 * omitted both, so small RX frames are held and WIFI RX stalls after the
 * initial burst. */
constexpr uint8_t COMPAT_RX_AGG_UNIT = 4;
constexpr uint16_t R_AX_RXAGG_1_V1 = 0x6004;
constexpr uint32_t RXAGG_1_V1_DISABLE_SML = 0x1f; /* SML_PKTNUM_TH_MSK[4:0] */

/* ---- Full CMAC init (trxcfg.c cmac_init: the remaining band-0 sub-inits that
 * were previously skipped — scheduler / tmac / trxptcl / ptcl / nav /
 * spatial-reuse / addr-cam). All band-0, 8852B, ASIC path. ---- */
/* scheduler_init */
constexpr uint16_t R_AX_PREBKF_CFG_1 = 0xC33C;
constexpr uint8_t B_AX_SIFS_MACTXEN_T1_SH = 0;
constexpr uint32_t B_AX_SIFS_MACTXEN_T1_MSK = 0x7f;
constexpr uint32_t SIFS_MACTXEN_T1_V0 = 0x47;
constexpr uint16_t R_AX_SCH_EXT_CTRL = 0xC3FC;
constexpr uint32_t B_AX_PORT_RST_TSF_ADV = 1u << 1;
constexpr uint16_t R_AX_CCA_CFG_0 = 0xC340;
constexpr uint32_t B_AX_BTCCA_EN = 1u << 5;
/* CCA medium-busy enables (R_AX_CCA_CFG_0 bits 0-4). Cleared for injection TX:
 * without the RX-DCK/DACK BB calibration the carrier/energy detectors assert a
 * perpetual busy that freezes the CSMA backoff, so the scheduler never grants a
 * TX opportunity and injected frames stall in the CMAC MBH queue (~103-frame
 * mgmt-TX stall). Clearing CCA_EN + the secondary-channel + EDCCA gates lets the
 * frames air — the intended TX/monitor-link mode; carrier-sense returns with
 * the energy-detector calibration. On-air validated: the 8852BU radiates. */
constexpr uint32_t B_AX_CCA_EN = 1u << 0;
constexpr uint32_t B_AX_SEC20_EN = 1u << 1;
constexpr uint32_t B_AX_SEC40_EN = 1u << 2;
constexpr uint32_t B_AX_SEC80_EN = 1u << 3;
constexpr uint32_t B_AX_EDCCA_EN = 1u << 4;
constexpr uint32_t B_AX_CCA_ALL_EN = B_AX_CCA_EN | B_AX_SEC20_EN |
                                     B_AX_SEC40_EN | B_AX_SEC80_EN |
                                     B_AX_EDCCA_EN;
constexpr uint8_t B_AX_R_SIFS_AGGR_TIME_SH = 24;
constexpr uint32_t B_AX_R_SIFS_AGGR_TIME_MSK = 0x7f;
constexpr uint16_t R_AX_PREBKF_CFG_0 = 0xC338;
constexpr uint8_t B_AX_PREBKF_TIME_SH = 0;
constexpr uint32_t B_AX_PREBKF_TIME_MSK = 0x1f;
constexpr uint32_t SCH_PREBKF_16US = 0x10;
constexpr uint16_t R_AX_EDCA_BCNQ_PARAM = 0xC324;
constexpr uint8_t B_AX_BE_0_CW_SH = 8;
constexpr uint32_t B_AX_BE_0_CW_MSK = 0xff;
constexpr uint8_t B_AX_BE_0_AIFS_SH = 0;
constexpr uint32_t B_AX_BE_0_AIFS_MSK = 0xff;
constexpr uint16_t BCN_IFS_25US = 0x19;
/* tmac_init */
constexpr uint16_t R_AX_MAC_LOOPBACK = 0xCC20;
constexpr uint32_t B_AX_MACLBK_EN = 1u << 0;
constexpr uint16_t R_AX_TCR0 = 0xCA00;
constexpr uint8_t B_AX_TCR_UDF_THSD_SH = 16;
constexpr uint32_t B_AX_TCR_UDF_THSD_MSK = 0x7f;
constexpr uint32_t TCR_UDF_THSD = 0x6;
constexpr uint16_t R_AX_TXD_FIFO_CTRL = 0xCA1C;
constexpr uint8_t B_AX_TXDFIFO_HIGH_MCS_THRE_SH = 12;
constexpr uint32_t B_AX_TXDFIFO_HIGH_MCS_THRE_MSK = 0xf;
constexpr uint8_t B_AX_TXDFIFO_LOW_MCS_THRE_SH = 8;
constexpr uint32_t B_AX_TXDFIFO_LOW_MCS_THRE_MSK = 0xf;
constexpr uint32_t TXDFIFO_HIGH_MCS_THRE = 0x7;
constexpr uint32_t TXDFIFO_LOW_MCS_THRE = 0x7;
constexpr uint16_t R_AX_TB_PPDU_CTRL = 0xC60C;
constexpr uint8_t B_AX_SW_PREFER_AC_SH = 0;
constexpr uint32_t B_AX_SW_PREFER_AC_MSK = 0x3;
constexpr uint32_t MAC_AX_CMAC_AC_SEL_BK = 1;
/* trxptcl_init */
constexpr uint16_t R_AX_TRXPTCL_RESP_0 = 0xCC04;
constexpr uint8_t B_AX_WMAC_SPEC_SIFS_CCK_SH = 0;
constexpr uint32_t B_AX_WMAC_SPEC_SIFS_CCK_MSK = 0xff;
constexpr uint32_t WMAC_SPEC_SIFS_CCK = 0xA;
constexpr uint8_t B_AX_WMAC_SPEC_SIFS_OFDM_SH = 8;
constexpr uint32_t B_AX_WMAC_SPEC_SIFS_OFDM_MSK = 0xff;
constexpr uint32_t WMAC_SPEC_SIFS_OFDM_52B = 0x11;
constexpr uint16_t R_AX_RXTRIG_TEST_USER_2 = 0xCCB0;
constexpr uint32_t B_AX_RXTRIG_FCSCHK_EN = 1u << 20;
constexpr uint16_t R_AX_TRXPTCL_RRSR_CTL_0 = 0xCC08;
constexpr uint16_t R_AX_TRXPTCL_RRSR_CTL_1 = 0xCC0C;
constexpr uint8_t B_AX_WMAC_RESP_RATE_EN_SH = 12;
constexpr uint32_t B_AX_WMAC_RESP_RATE_EN_MSK = 0xf;
constexpr uint8_t B_AX_WMAC_RRSB_AX_CCK_SH = 16;
constexpr uint32_t B_AX_WMAC_RRSB_AX_CCK_MSK = 0xf;
constexpr uint32_t B_AX_WMAC_RESP_REF_RATE_SEL = 1u << 9;
constexpr uint32_t WMAC_CCK_EN_1M = 0x1;
constexpr uint32_t WMAC_RRSR_RATE_LEGACY_EN = 0x1;
constexpr uint32_t REF2RXRATEANDCCTBL = 0;
/* ptcl_init */
constexpr uint16_t R_AX_PTCL_COMMON_SETTING_0 = 0xC600;
constexpr uint32_t B_AX_CMAC_TX_MODE_0 = 1u << 0;
constexpr uint32_t B_AX_CMAC_TX_MODE_1 = 1u << 1;
constexpr uint32_t B_AX_PTCL_TRIGGER_SS_EN_0 = 1u << 2;
constexpr uint32_t B_AX_PTCL_TRIGGER_SS_EN_1 = 1u << 3;
constexpr uint32_t B_AX_PTCL_TRIGGER_SS_EN_UL = 1u << 4;
constexpr uint16_t R_AX_PTCLRPT_FULL_HDL = 0xC660;
constexpr uint8_t B_AX_SPE_RPT_PATH_SH = 4;
constexpr uint32_t B_AX_SPE_RPT_PATH_MSK = 0x3;
constexpr uint32_t FWD_TO_WLCPU = 1;
constexpr uint16_t R_AX_AGG_BK_0 = 0xC604;
constexpr uint32_t B_AX_WDBK_CFG = 1u << 2;
constexpr uint32_t B_AX_EN_RTY_BK = 1u << 1;
constexpr uint32_t B_AX_EN_RTY_BK_COD = 1u << 0;
constexpr uint16_t R_AX_AMPDU_AGG_LIMIT = 0xC610;
constexpr uint8_t B_AX_MAX_AGG_NUM_SH = 0;
constexpr uint32_t B_AX_MAX_AGG_NUM_MSK = 0xff;
constexpr uint8_t B_AX_AMPDU_MAX_TIME_SH = 24;
constexpr uint32_t B_AX_AMPDU_MAX_TIME_MSK = 0xff;
constexpr uint16_t PTCL_MAX_AGG_NUM = 0x40;
constexpr uint8_t PTCL_AMPDU_MAX_TIME_8852B = 0x3e; /* hw_info max_agg_txtime_reg */
/* nav_ctrl_init (mac_two_nav_cfg) */
constexpr uint16_t R_AX_WMAC_NAV_CTL = 0xCC80;
constexpr uint32_t B_AX_WMAC_PLCP_UP_NAV_EN = 1u << 17;
constexpr uint32_t B_AX_WMAC_TF_UP_NAV_EN = 1u << 16;
constexpr uint8_t B_AX_WMAC_NAV_UPPER_SH = 8;
constexpr uint32_t B_AX_WMAC_NAV_UPPER_MSK = 0xff;
constexpr uint32_t B_AX_WMAC_NAV_UPPER_EN = 1u << 26;
constexpr uint32_t NAV_25MS = 0xC4;
constexpr uint32_t NAV_UPPER_DEFAULT = 0;
constexpr uint16_t R_AX_SPECIAL_TX_SETTING = 0xC620;
constexpr uint32_t B_AX_BMC_NAV_PROTECT = 1u << 26;
/* spatial_reuse_init */
constexpr uint16_t R_AX_RX_SR_CTRL = 0xCE4A;
constexpr uint32_t B_AX_SR_EN = 1u << 0;
constexpr uint32_t B_AX_SR_CTRL_PLCP_EN = 1u << 1;
constexpr uint16_t R_AX_BSSID_SRC_CTRL = 0xCE4B;
constexpr uint32_t B_AX_PLCP_SRC_EN = 1u << 0;
/* addr_cam_init */
constexpr uint16_t R_AX_ADDR_CAM_CTRL = 0xCE34;
constexpr uint8_t B_AX_ADDR_CAM_RANGE_SH = 16;
constexpr uint32_t B_AX_ADDR_CAM_RANGE_MSK = 0xff;
constexpr uint32_t ADDR_CAM_SERCH_RANGE = 0x7f;
constexpr uint32_t B_AX_ADDR_CAM_EN = 1u << 0;
constexpr uint32_t B_AX_ADDR_CAM_CLR = 1u << 8;

/* ---- coex_mac_init (coex_8852b.c) — LTE/BT coexistence bring-up on the
 * WiFi+BT combo die: disable LTE-coex, set the SDIO-ctrl coex bit. ---- */
constexpr uint16_t R_AX_LTE_CTRL = 0xDAF0;
constexpr uint16_t R_AX_LTE_WDATA = 0xDAF4;
constexpr uint32_t R_AX_LTECOEX_CTRL = 0x38;   /* LTE-space indirect offset */
constexpr uint32_t R_AX_LTECOEX_CTRL_2 = 0x3C; /* LTE-space indirect offset */
constexpr uint16_t R_AX_SYS_SDIO_CTRL = 0x0070;
/* BT/LTE-coex block enable — mac_coex_init_8852b (coex_8852b.c:154). Arms the
 * LTE indirect interface (write_lte's ready bit) from real cold. */
constexpr uint32_t B_AX_ENBT = 1u << 5; /* R_AX_GPIO_MUXCFG */
/* LTE-space GNT arbitration (mac_cfg_gnt_8852b; offsets are LTE-space, ridden
 * through write_lte). SW_CTRL pins the mux, SW_VAL is the pinned level. */
constexpr uint32_t R_AX_LTE_SW_CFG_1 = 0x0038; /* == R_AX_LTECOEX_CTRL */
constexpr uint32_t B_AX_GNT_BT_RFC_S1_SW_VAL = 1u << 31;
constexpr uint32_t B_AX_GNT_BT_RFC_S1_SW_CTRL = 1u << 30;
constexpr uint32_t B_AX_GNT_WL_RFC_S1_SW_VAL = 1u << 29;
constexpr uint32_t B_AX_GNT_WL_RFC_S1_SW_CTRL = 1u << 28;
constexpr uint32_t B_AX_GNT_BT_BB_S1_SW_VAL = 1u << 27;
constexpr uint32_t B_AX_GNT_BT_BB_S1_SW_CTRL = 1u << 26;
constexpr uint32_t B_AX_GNT_WL_BB_S1_SW_VAL = 1u << 25;
constexpr uint32_t B_AX_GNT_WL_BB_S1_SW_CTRL = 1u << 24;
constexpr uint32_t B_AX_GNT_BT_RFC_S0_SW_VAL = 1u << 15;
constexpr uint32_t B_AX_GNT_BT_RFC_S0_SW_CTRL = 1u << 14;
constexpr uint32_t B_AX_GNT_WL_RFC_S0_SW_VAL = 1u << 13;
constexpr uint32_t B_AX_GNT_WL_RFC_S0_SW_CTRL = 1u << 12;
constexpr uint32_t B_AX_GNT_BT_BB_S0_SW_VAL = 1u << 11;
constexpr uint32_t B_AX_GNT_BT_BB_S0_SW_CTRL = 1u << 10;
constexpr uint32_t B_AX_GNT_WL_BB_S0_SW_VAL = 1u << 9;
constexpr uint32_t B_AX_GNT_WL_BB_S0_SW_CTRL = 1u << 8;
constexpr uint32_t R_AX_LTE_SW_CFG_2 = 0x003C; /* == R_AX_LTECOEX_CTRL_2 */
constexpr uint32_t B_AX_WL_RX_CTRL = 1u << 8;
constexpr uint32_t B_AX_GNT_WL_RX_SW_VAL = 1u << 7;
constexpr uint32_t B_AX_GNT_WL_RX_SW_CTRL = 1u << 6;
constexpr uint32_t B_AX_GNT_WL_TX_SW_VAL = 1u << 5;
constexpr uint32_t B_AX_GNT_WL_TX_SW_CTRL = 1u << 4;
constexpr uint32_t B_AX_GNT_BT_RX_SW_VAL = 1u << 3;
constexpr uint32_t B_AX_GNT_BT_RX_SW_CTRL = 1u << 2;
constexpr uint32_t B_AX_GNT_BT_TX_SW_VAL = 1u << 1;
constexpr uint32_t B_AX_GNT_BT_TX_SW_CTRL = 1u << 0;
constexpr uint32_t LTE_WRITE_CMD = 0xC00F0000; /* | offset */
constexpr uint32_t B_AX_LTE_MUX_CTRL_PATH = 1u << 26; /* R_AX_LTE_CTRL+3 bit5 rdy */

/* chip_func_en OCP patch (init.c mac_sys_init). */
constexpr uint8_t B_AX_OCP_L1_SH = 13;
constexpr uint32_t B_AX_OCP_L1_MSK = 0x7;

/* mac_sys_init cmac_func_en (init.c:219): CK_EN (0xC004) gets the clock
 * enables FIRST, then CMAC_FUNC_EN (0xC000) the block enables — both RMW-OR,
 * band 0. Bits from mac_reg_ax.h:8161-8181. */
constexpr uint16_t R_AX_CK_EN = 0xC004;
constexpr uint32_t CMAC_CK_EN_BITS = /* CMAC|PHYINTF|CMAC_DMA|PTCLTOP|SCH|TMAC|RMAC CKEN */
    (1u << 30) | (1u << 5) | (1u << 4) | (1u << 3) | (1u << 2) | (1u << 1) |
    (1u << 0);
constexpr uint16_t R_AX_CMAC_FUNC_EN = 0xC000;
constexpr uint32_t CMAC_FUNC_EN_BITS = /* CRPRT|CMAC_EN|TXEN|RXEN|PHYINTF|DMA|PTCLTOP|SCH|TMAC|RMAC */
    (1u << 31) | (1u << 30) | (1u << 29) | (1u << 28) | (1u << 5) |
    (1u << 4) | (1u << 3) | (1u << 2) | (1u << 1) | (1u << 0);

/* mac_enable_imr — SER interrupt masks (ser_imr_config + err_imr_ctrl). */
constexpr uint16_t R_AX_PHYINFO_ERR_IMR = 0xCCFC;
constexpr uint8_t B_AX_PHYINTF_TIMEOUT_THR_SH = 0;
constexpr uint32_t B_AX_PHYINTF_TIMEOUT_THR_MSK = 0x3f;
constexpr uint16_t R_AX_DMAC_ERR_IMR = 0x8520;
constexpr uint16_t R_AX_CMAC_ERR_IMR = 0xC160;

/* SET_CLR_WORD(orig, val, FIELD) — replace FIELD bits with val. */
inline uint32_t set_clr_word(uint32_t orig, uint32_t val, uint32_t msk,
                             uint8_t sh) {
  return (orig & ~(msk << sh)) | ((val & msk) << sh);
}

} /* namespace kestrel::reg */

#endif /* KESTREL_MAC_REG_AX_H */
