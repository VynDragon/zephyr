/*
 * Copyright (c) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bflb-pinctrl.h>
#include <bflb_gpio.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint8_t i;
	struct bflb_device_s *gpio = bflb_device_get_by_name("gpio");

	ARG_UNUSED(reg);

	for (i = 0U; i < pin_cnt; i++) {
		uint32_t cfg = pins[i].fun & 0x1F;
		cfg |= (pins[i].cfg & 0xF) << 5U;
		cfg |= (pins[i].cfg & 0x30) << 5U;
		cfg |= (pins[i].cfg & 0x40) << 5U;
		cfg |= (pins[i].cfg & 0x3 >> 16U) << 5U << 12U;

		bflb_gpio_init(gpio, pins[i].pin, cfg);
	}

	return 0;
}
