#ifndef KESTREL_MAC_REG_AX_H
#define KESTREL_MAC_REG_AX_H

#include <cstdint>

/* Register + bitfield constants for the Kestrel (Wi-Fi 6 / G6 "phl") MAC,
 * transcribed from the vendor tree reference/rtl8852bu
 * (phl/hal_g6/mac/mac_reg_ax.h). Only the subset the Kestrel bring-up touches
 * is mirrored here; extend as later milestones need more. Values are verbatim
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
constexpr uint32_t B_AX_APFN_ONMAC = 1u << 8;
constexpr uint32_t B_AX_DIS_WLBT_PDNSUSEN_SOPC = 1u << 18;

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
constexpr uint16_t R_AX_HCI_FUNC_EN = 0x8380; /* mac_reg_ax.h */
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
constexpr uint16_t R_AX_PLE_QTA10_CFG = 0x9068; /* highest PLE QTA */

constexpr uint16_t R_AX_WDE_INI_STATUS = 0x8D00;
constexpr uint16_t R_AX_PLE_INI_STATUS = 0x9100;
constexpr uint32_t B_AX_WDE_Q_MGN_INI_RDY = 1u << 1;
constexpr uint32_t B_AX_WDE_BUF_MGN_INI_RDY = 1u << 0;
constexpr uint32_t B_AX_PLE_Q_MGN_INI_RDY = 1u << 1;
constexpr uint32_t B_AX_PLE_BUF_MGN_INI_RDY = 1u << 0;
constexpr uint32_t DLE_WAIT_CNT = 2000;
constexpr uint32_t DLE_WAIT_US = 1;

/* DLFW quota values (dle_mem_usb3_8852b DLFW entry; SCC wcpu override = 6). */
constexpr uint16_t DLFW_WDE_FREE_PAGE = 0;   /* wde_size9 lnk_pge_num */
constexpr uint16_t DLFW_WDE_UNLNK_PAGE = 1024; /* wde_size9 unlnk_pge_num */
constexpr uint16_t DLFW_PLE_FREE_PAGE = 64;  /* ple_size8 lnk_pge_num */
constexpr uint16_t DLFW_WDE_WCPU_MIN = 6;    /* wde_qt25.wcpu (SCC override) */
constexpr uint16_t DLFW_PLE_C2H = 16;        /* ple_qt13.c2h */
constexpr uint16_t DLFW_PLE_H2C = 48;        /* ple_qt13.h2c */

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
constexpr uint8_t B_AX_WCPU_FWDL_STS_SH = 5;
constexpr uint32_t B_AX_WCPU_FWDL_STS_MSK = 0x7;
constexpr uint32_t B_AX_FWDL_PATH_RDY = 1u << 2;
constexpr uint32_t B_AX_H2C_PATH_RDY = 1u << 1;
constexpr uint32_t B_AX_WCPU_FWDL_EN = 1u << 0;
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
constexpr uint32_t WD_BODY_LEN = 24; /* sizeof wd_body_t (6 dwords) */
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

/* NIC-mode SCC DLE quota (dle_mem_usb3_8852b SCC entry; contrast the DLFW
 * quota above). wde_size25 / ple_size33 / wde_qt25 / ple_qt74(min) /
 * ple_qt75(max). */
constexpr uint16_t SCC_WDE_LNK_PAGE = 166;
constexpr uint16_t SCC_WDE_UNLNK_PAGE = 90;
constexpr uint16_t SCC_PLE_LNK_PAGE = 624;
/* wde_qt25: hif/wcpu/pkt_in/cpu_io (min==max). */
constexpr uint16_t SCC_WDE_QT_HIF = 152;
constexpr uint16_t SCC_WDE_QT_WCPU = 6;
constexpr uint16_t SCC_WDE_QT_CPU_IO = 8;
/* ple_qt74 (min) then ple_qt75 (max), 11 fields Q0..Q10. */
constexpr uint16_t SCC_PLE_MIN[11] = {286, 0, 16, 48, 13, 13, 178, 0, 32, 14, 8};
constexpr uint16_t SCC_PLE_MAX[11] = {286, 0, 32, 48, 29, 13, 194, 0, 48, 14, 24};

/* SET_CLR_WORD(orig, val, FIELD) — replace FIELD bits with val. */
inline uint32_t set_clr_word(uint32_t orig, uint32_t val, uint32_t msk,
                             uint8_t sh) {
  return (orig & ~(msk << sh)) | ((val & msk) << sh);
}

} /* namespace kestrel::reg */

#endif /* KESTREL_MAC_REG_AX_H */
