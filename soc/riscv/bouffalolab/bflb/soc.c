/*
 * Copyright (c) 2021, ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <board.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

static int bl_riscv_init(void)
{
	board_init();
	return 0;
}

SYS_INIT(bl_riscv_init, PRE_KERNEL_1, 0);
