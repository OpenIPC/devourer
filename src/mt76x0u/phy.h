/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Port of mt76/mt76x0/phy.h (reference/mt76 @ be5ce79): the band/width flags
 * and the item types src/mt76x0u/initvals.h (generated) is written in.
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 * Copyright (C) 2018 Stanislaw Gruszka <stf_xl@wp.pl>
 */
#ifndef MT76X0U_PHY_H
#define MT76X0U_PHY_H

#include <stdint.h>

#define RF_G_BAND      0x0100
#define RF_A_BAND      0x0200
#define RF_A_BAND_LB   0x0400
#define RF_A_BAND_MB   0x0800
#define RF_A_BAND_HB   0x1000
#define RF_A_BAND_11J  0x2000

#define RF_BW_20       1
#define RF_BW_40       2
#define RF_BW_10       4
#define RF_BW_80       8

#define MT_RF_BANK(offset)  ((offset) >> 16)
#define MT_RF_REG(offset)   ((offset) & 0xff)

#define MT_RF_PLL_DEN_MASK        GENMASK(4, 0)
#define MT_RF_PLL_K_MASK          GENMASK(4, 0)
#define MT_RF_SDM_RESET_MASK      BIT(7)
#define MT_RF_SDM_MASH_PRBS_MASK  GENMASK(6, 2)
#define MT_RF_SDM_BP_MASK         BIT(1)
#define MT_RF_ISI_ISO_MASK        GENMASK(7, 6)
#define MT_RF_PFD_DLY_MASK        GENMASK(5, 4)
#define MT_RF_CLK_SEL_MASK        GENMASK(3, 2)
#define MT_RF_XO_DIV_MASK         GENMASK(1, 0)

struct mt0_bbp_switch_item {
	uint16_t bw_band;
	struct mt0_reg_pair reg_pair;
};

struct mt0_rf_switch_item {
	uint32_t rf_bank_reg;
	uint16_t bw_band;
	uint8_t value;
};

/* mt76x0_freq_item, field for field (used by the channel set, phase 4). */
struct mt0_freq_item {
	uint8_t channel;
	uint32_t band;
	uint8_t pllR37;
	uint8_t pllR36;
	uint8_t pllR35;
	uint8_t pllR34;
	uint8_t pllR33;
	uint8_t pllR32_b7b5;
	uint8_t pllR32_b4b0;  /* PLL_DEN (Denomina - 8) */
	uint8_t pllR31_b7b5;
	uint8_t pllR31_b4b0;  /* PLL_K (Nominator *) */
	uint8_t pllR30_b7;    /* sdm_reset_n */
	uint8_t pllR30_b6b2;  /* sdmmash_prbs,sin */
	uint8_t pllR30_b1;    /* sdm_bp */
	uint16_t pll_n;       /* R30<0>, R29<7:0> (hex) */
	uint8_t pllR28_b7b6;  /* isi,iso */
	uint8_t pllR28_b5b4;  /* pfd_dly */
	uint8_t pllR28_b3b2;  /* clksel option */
	uint32_t pll_sdm_k;   /* R28<1:0>, R27<7:0>, R26<7:0> (hex) SDM_k */
	uint8_t pllR24_b1b0;  /* xo_div */
};

#endif /* MT76X0U_PHY_H */
