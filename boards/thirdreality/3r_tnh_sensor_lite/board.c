/*
 * Copyright (c) 2025-2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>

#include "bflb_soc.h"
#include "hbn_reg.h"

static int board_jtag_init(void)
{
	/* Enable JTAG on broken out pins */
	/* Clear HBN pin control */
	*(uint32_t*)(HBN_BASE + HBN_PAD_CTRL_0_OFFSET) &= HBN_REG_EN_AON_CTRL_GPIO_UMSK;

	/* Reset Pin 0,1,2 and 7 to GPIO */
	*(uint32_t*)(0x40000100) = (*(uint32_t*)(0x40000100) & ~0x1F00) | 0xB << 8;
	*(uint32_t*)(0x40000100) = (*(uint32_t*)(0x40000100) & ~0x1F000000) | 0xB << 24;
	*(uint32_t*)(0x40000104) = (*(uint32_t*)(0x40000104) & ~0x1F00) | 0xB << 8;
	*(uint32_t*)(0x4000010c) = (*(uint32_t*)(0x4000010c) & ~0x1F000000) | 0xB << 24;

	/* Set Pins 17,18,19 and 20 to JTAG and configure
	 * GLB_BASE + GLB_GPIO_CFGCTL*_OFFSET...
	 */
	*(uint32_t*)(0x40000120) = (*(uint32_t*)(0x40000120) & ~0x1F000000) | 0xE << 24;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x1F00) | 0xE << 8;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x1F000000) | 0xE << 24;
	*(uint32_t*)(0x40000128) = (*(uint32_t*)(0x40000128) & ~0x1F00) | 0xE << 8;

	*(uint32_t*)(0x40000120) = (*(uint32_t*)(0x40000120) & ~0x100000) | 1 << 20;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x10) | 1 << 4;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x100000) | 1 << 20;
	*(uint32_t*)(0x40000128) = (*(uint32_t*)(0x40000128) & ~0x10) | 1 << 4;

	*(uint32_t*)(0x40000120) = (*(uint32_t*)(0x40000120) & ~0x20000) | 1 << 17;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x2) | 1 << 1;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x20000) | 1 << 17;
	*(uint32_t*)(0x40000128) = (*(uint32_t*)(0x40000128) & ~0x2) | 1 << 1;

	*(uint32_t*)(0x40000120) = (*(uint32_t*)(0x40000120) & ~0x10000) | 1 << 16;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x1) | 1;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x10000) | 1 << 16;
	*(uint32_t*)(0x40000128) = (*(uint32_t*)(0x40000128) & ~0x1) | 1;

	*(uint32_t*)(0x40000120) = (*(uint32_t*)(0x40000120) & ~0x80000000) | 1 << 31;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x8000) | 1 << 15;
	*(uint32_t*)(0x40000124) = (*(uint32_t*)(0x40000124) & ~0x80000000) | 1 << 31;
	*(uint32_t*)(0x40000128) = (*(uint32_t*)(0x40000128) & ~0x8000) | 1 << 15;

	return 0;
}


SYS_INIT(board_jtag_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
