/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 * Based on works by Gerson Fernando Budke <nandojve@gmail.com>
 * And the Bouffalolab SDK
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bflb-common-pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bl60x-pinctrl.h>
#include <glb_reg.h>

#if defined(CONFIG_SOC_SERIES_BL60X) || defined(CONFIG_SOC_SERIES_BL70X)

void pinctrl_configure_uart(uint8_t pin, uint8_t func)
{
	/* uart func for BL602 and BL702 Only*/
	uint32_t regval;
	uint8_t sig;
	uint8_t sig_pos;

	regval = sys_read32(GLB_BASE + GLB_UART_SIG_SEL_0_OFFSET);

	sig = pin % 8;
	sig_pos = sig << 2;

	regval &= (~(0x0f << sig_pos));
	regval |= (func << sig_pos);

	for (uint8_t i = 0; i < 8; i++) {
		/* reset other sigs which are the same with uart_func */
		sig_pos = i << 2;
		if (((regval & (0x0f << sig_pos)) == (func << sig_pos)) && (i != sig) && (func !=
0x0f)) {
			regval &= (~(0x0f << sig_pos));
			regval |= (0x0f << sig_pos);
		}
	}

	sys_write32(regval, GLB_BASE + GLB_UART_SIG_SEL_0_OFFSET);
}

void pinctrl_init_pin(pinctrl_soc_pin_t pin)
{
	uint8_t drive;
	uint8_t function;
	uint16_t mode;
	uint32_t regval;
	uint8_t real_pin;
	uint8_t is_odd = 0;
	uint32_t cfg = 0;
	uint32_t cfg_address;

	real_pin = BFLB_PINMUX_GET_PIN(pin);
	function = BFLB_PINMUX_GET_FUN(pin);
	mode = BFLB_PINMUX_GET_MODE(pin);
	drive = BFLB_PINMUX_GET_DRIVER_STRENGTH(pin);

	/* Disable output anyway */
	regval = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));
	regval &= ~(1 << (pin & 0x1f));
	sys_write32(regval, GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	is_odd = real_pin & 1;

	cfg_address = GLB_BASE + GLB_GPIO_CFGCTL0_OFFSET + (real_pin / 2 * 4);
	cfg = sys_read32(cfg_address);
	cfg &= ~(0xffff << (16 * is_odd));

	regval = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	if (mode == BFLB_PINMUX_MODE_analog) {
		regval &= ~(1 << (real_pin & 0x1f));
		function = 10;
	} else if (mode == BFLB_PINMUX_MODE_periph) {
		cfg |= (1 << (is_odd * 16 + 0));
		regval &= ~(1 << (real_pin & 0x1f));
	} else {
		function = 11;

		if (mode == BFLB_PINMUX_MODE_input) {
			cfg |= (1 << (is_odd * 16 + 0));
		}

		if (mode == BFLB_PINMUX_MODE_output) {
			regval |= (1 << (real_pin & 0x1f));
		}
	}

	sys_write32(regval, GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	uint8_t pull_up = BFLB_PINMUX_GET_PULL_UP(pin);
	uint8_t pull_down = BFLB_PINMUX_GET_PULL_DOWN(pin);

	if (pull_up) {
		cfg |= (1 << (is_odd * 16 + 4));
	} else if (pull_down) {
		cfg |= (1 << (is_odd * 16 + 5));
	} else {
	}

	if (BFLB_PINMUX_GET_SMT(pin)) {
		cfg |= (1 << (is_odd * 16 + 1));
	}

	cfg |= (drive << (is_odd * 16 + 2));
	cfg |= (function << (is_odd * 16 + 8));
	sys_write32(cfg, cfg_address);
}

#elif defined(CONFIG_SOC_SERIES_BL61X)
void pinctrl_configure_uart(uint8_t pin, uint8_t func)
{
	uint32_t	regval = 0, regval2 = 0;
	uint8_t		sig = 0, sig_pos = 0;

	/* gpio pad check goes here */

	sig = pin % 12;

	if (sig < 8) {
		sig_pos = sig << 2;

		regval = sys_read32(GLB_BASE + GLB_UART_CFG1_OFFSET);
		regval &= (~(0x0f << sig_pos));
		regval |= (func << sig_pos);

		for (uint8_t i = 0; i < 8; i++) {
			/* reset other sigs which are the same as func */
			sig_pos = i << 2;
			if (((regval & (0x0f << sig_pos)) == (func << sig_pos)) && (i != sig) && (func != 0x0f)) {
			regval &= (~(0x0f << sig_pos));
			regval |= (0x0f << sig_pos);
			}
		}
		regval2 = sys_read32(GLB_BASE + GLB_UART_CFG2_OFFSET);

		for (uint8_t i = 8; i < 12; i++) {
			/* reset other sigs which are the same as func */
			sig_pos = (i - 8) << 2;
			if (((regval2 & (0x0f << sig_pos)) == (func << sig_pos)) && (i != sig) && (func != 0x0f)) {
			regval2 &= (~(0x0f << sig_pos));
			regval2 |= (0x0f << sig_pos);
			}
		}
		sys_write32(regval, GLB_BASE + GLB_UART_CFG1_OFFSET);
		sys_write32(regval2, GLB_BASE + GLB_UART_CFG2_OFFSET);
	} else {
		sig_pos = (sig - 8) << 2;

		regval = sys_read32(GLB_BASE + GLB_UART_CFG2_OFFSET);
		regval &= (~(0x0f << sig_pos));
		regval |= (func << sig_pos);

		for (uint8_t i = 8; i < 12; i++) {
			/* reset other sigs which are the same as func */
			sig_pos = (i - 8) << 2;
			if (((regval & (0x0f << sig_pos)) == (func << sig_pos)) && (i != sig) && (func != 0x0f)) {
			regval &= (~(0x0f << sig_pos));
			regval |= (0x0f << sig_pos);
			}
		}
		regval2 = sys_read32(GLB_BASE + GLB_UART_CFG1_OFFSET);

		for (uint8_t i = 0; i < 8; i++) {
			/* reset other sigs which are the same as func */
			sig_pos = i << 2;
			if (((regval2 & (0x0f << sig_pos)) == (func << sig_pos)) && (i != sig) && (func != 0x0f)) {
			regval2 &= (~(0x0f << sig_pos));
			regval2 |= (0x0f << sig_pos);
			}
		}
		sys_write32(regval, GLB_BASE + GLB_UART_CFG2_OFFSET);
		sys_write32(regval2, GLB_BASE + GLB_UART_CFG1_OFFSET);
	}
}

void pinctrl_init_pin(pinctrl_soc_pin_t pin)
{
	uint8_t drive;
	uint8_t function;
	uint16_t mode;
	uint32_t regval;
	uint8_t real_pin;
	uint32_t cfg = 0;

	real_pin = BFLB_PINMUX_GET_PIN(pin);
	function = BFLB_PINMUX_GET_FUN(pin);
	mode = BFLB_PINMUX_GET_MODE(pin);
	drive = BFLB_PINMUX_GET_DRIVER_STRENGTH(pin);

	/* gpio pad check goes here */

	/* disable RC32K muxing */
	if (real_pin == 16) {
		*(volatile uint32_t *)(0x2000f000 + 0x38) &= ~(1 << 20);
	} else if (real_pin == 17) {
		*(volatile uint32_t *)(0x2000f000 + 0x38) &= ~(1 << 21);
	}

	/* mask interrupt */
	cfg = GLB_REG_GPIO_0_INT_MASK_MSK;

	if (mode == BFLB_PINMUX_MODE_analog) {
		function = 10;
	} else if (mode == BFLB_PINMUX_MODE_periph) {
		cfg |= GLB_REG_GPIO_0_IE_MSK;
	} else {
		function = 11;

		if (mode == BFLB_PINMUX_MODE_input) {
			cfg |= GLB_REG_GPIO_0_IE_MSK;
		}

		if (mode == BFLB_PINMUX_MODE_output) {
			cfg |= GLB_REG_GPIO_0_OE_MSK;
		}
	}

	uint8_t pull_up = BFLB_PINMUX_GET_PULL_UP(pin);
	uint8_t pull_down = BFLB_PINMUX_GET_PULL_DOWN(pin);

	if (pull_up) {
		cfg |= GLB_REG_GPIO_0_PU_MSK;
	} else if (pull_down) {
		cfg |= GLB_REG_GPIO_0_PD_MSK;
	} else {
	}

	if (BFLB_PINMUX_GET_SMT(pin)) {
		cfg |= GLB_REG_GPIO_0_SMT_MSK;
	}

	cfg |= (drive << GLB_REG_GPIO_0_DRV_POS);
	cfg |= (function << GLB_REG_GPIO_0_FUNC_SEL_POS);

	/* output is controlled by _set and _clr and not value of _o*/
	cfg |= 0x1 << GLB_REG_GPIO_0_MODE_POS;

	sys_write32(cfg, GLB_BASE + GLB_GPIO_CFG0_OFFSET + (real_pin << 2));
}
#else
#error "Unsupported Platform"
#endif


int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint8_t i;

	ARG_UNUSED(reg);

	for (i = 0U; i < pin_cnt; i++) {

		if (BFLB_PINMUX_GET_FUN(pins[i]) == BFLB_PINMUX_FUN_INST_uart0) {
			pinctrl_configure_uart(BFLB_PINMUX_GET_PIN(pins[i]),
			BFLB_PINMUX_GET_SIGNAL(pins[i]) + 4 * BFLB_PINMUX_GET_INST(pins[i]));
		}

		/* gpio init*/
		pinctrl_init_pin(pins[i]);
	}

	return 0;
}
