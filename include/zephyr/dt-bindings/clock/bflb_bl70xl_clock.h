/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BFLB_BL70XL_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BFLB_BL70XL_CLOCK_H_

#include "bflb_clock_common.h"

#define BL70XL_CLKID_CLK_ROOT    BFLB_CLKID_CLK_ROOT
#define BL70XL_CLKID_CLK_RC32M   BFLB_CLKID_CLK_RC32M
#define BL70XL_CLKID_CLK_CRYSTAL BFLB_CLKID_CLK_CRYSTAL
#define BL70XL_CLKID_CLK_BCLK    BFLB_CLKID_CLK_BCLK
#define BL70XL_CLKID_CLK_DLL     4

/* DLL output frequencies: 128 MHz base / integer dividers
 * PLL_SEL values for GLB_REG_PLL_SEL field in GLB_CLK_CFG0
 */
#define BL70XL_DLL_25P6MHZ	0	/* 128 / 5 = 25.6 MHz */
#define BL70XL_DLL_42P67MHZ	1	/* 128 / 3 = 42.67 MHz */
#define BL70XL_DLL_64MHZ	2	/* 128 / 2 = 64 MHz */
#define BL70XL_DLL_128MHZ	3	/* 128 / 1 = 128 MHz */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_BFLB_BL70XL_CLOCK_H_ */
