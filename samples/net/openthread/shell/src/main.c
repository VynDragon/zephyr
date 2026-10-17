/*
 * Copyright (c) 2024 A Labs GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

int main(void)
{
	/* Nothing to do here. The shell is automatically started in the background. */
/*
	while (true) {
		k_msleep(10000);
		for (uint32_t i = 0x2c000004; i < 0x2c0007fc; i +=4) {
			uint32_t tmp = sys_read32(i);
			printf("%x: %x\n", i, tmp);
		}
	}*/

	// while (true) {
	// 	for (uint32_t i = 0x2c000004; i < 0x2c0007fc; i +=4) {
	// 		uint32_t tmp = sys_read32(i);
	// 		if (tmp & BIT(11)) {
	// 		printf("%x: %x\n", i, tmp);
	// 		}
	// 	}
 //
	// 	k_msleep(500);
	// }

	return 0;
}

static int doomp_handler(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (uint32_t i = 0x2c000004; i < 0x2c0007fc; i +=4) {
		uint32_t tmp = sys_read32(i);
		shell_print(sh, "%x: %x", i, tmp);
	}

	// shell_print(sh, "0x20002824: %x", sys_read32(0x20002824U));
	// shell_print(sh, "0x24920028: %x", sys_read32(0x24920028U));
	// shell_print(sh, "0x24920028: %x", sys_read32(0x24920028U));

	return 0;
}

SHELL_CMD_ARG_REGISTER(doomp, NULL, "Show kernel version", doomp_handler, 1, 0);
