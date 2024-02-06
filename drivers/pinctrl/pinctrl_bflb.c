/*
 * Copyright (c) 2021-2024 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <bl_soc_pinctrl.h>
#include <bl_soc_glb.h>
#include <bl_soc_gpio.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	GLB_GPIO_Cfg_Type pincfg[2];
	uint8_t i;

	ARG_UNUSED(reg);

	for (i = 0U; i < pin_cnt; i++) {
		pincfg[i].gpioFun  = BFLB_PINMUX_GET_FUN(pins[i]);
		pincfg[i].gpioMode = BFLB_PINMUX_GET_MODE(pins[i]);
		pincfg[i].gpioPin  = BFLB_PINMUX_GET_PIN(pins[i]);
		pincfg[i].pullType = BFLB_PINMUX_GET_PULL_MODES(pins[i]);
		pincfg[i].smtCtrl  = BFLB_PINMUX_GET_SMT(pins[i]);
		pincfg[i].drive    = BFLB_PINMUX_GET_DRIVER_STRENGTH(pins[i]);

		if (pincfg[i].gpioFun == BFLB_PINMUX_FUN_INST_uart0) {
			GLB_UART_Fun_Sel(pincfg[i].gpioPin % 8,
					 (BFLB_PINMUX_GET_INST(pins[i]))
					  * 0x4U /* rts, cts, rx, tx */
					  + BFLB_PINMUX_GET_SIGNAL(pins[i])
					 );
		}

		GLB_GPIO_Init(&pincfg[i]);
	}

	return 0;
}
