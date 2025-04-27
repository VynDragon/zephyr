/*
 * Copyright (c) 2025 tinyVision.ai
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/rpi_pico_rp2350_clock.h>
#include <zephyr/init.h>

const struct device *const clocks = DEVICE_DT_GET(DT_NODELABEL(clocks));

static int board_init(void)
{
	/* enable FPGA clocks */
	return clock_control_on(clocks, RPI_PICO_CLKID_CLK_GPOUT0);
}

SYS_INIT(board_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
