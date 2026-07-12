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

/* ---- USB interface pre-init (_usb_8852b.c usb_pre_init_8852b) ---- */
constexpr uint16_t R_AX_USB_HOST_REQUEST_2 = 0x01A0;
constexpr uint32_t B_AX_R_USBIO_MODE = 1u << 3;
constexpr uint16_t R_AX_USB_WLAN0_1 = 0x01A2; /* R_AX_HCI... region */
constexpr uint16_t R_AX_HCI_FUNC_EN = 0x0098;
constexpr uint32_t B_AX_HCI_RXDMA_EN = 1u << 0;
constexpr uint32_t B_AX_HCI_TXDMA_EN = 1u << 1;

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

/* SET_CLR_WORD(orig, val, FIELD) — replace FIELD bits with val. */
inline uint32_t set_clr_word(uint32_t orig, uint32_t val, uint32_t msk,
                             uint8_t sh) {
  return (orig & ~(msk << sh)) | ((val & msk) << sh);
}

} /* namespace kestrel::reg */

#endif /* KESTREL_MAC_REG_AX_H */
